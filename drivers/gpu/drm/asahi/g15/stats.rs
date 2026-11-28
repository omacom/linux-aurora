// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Sysfs qualified statistics reader for the G15 (M3 Pro/Max: T6030/T6031/T6034) backend.
//!
//! Owns the shared session handle and DVFS table-A frequencies. Reads acquire
//! `Shared::state`, drain the statistics ring without waking the GPU or servicing
//! submissions, and return a qualified `Readout`.

use core::sync::atomic::Ordering;
use kernel::{prelude::*, sync::Arc};

use super::{
    channel::{Channels, Ring},
    engine,
    fw::channels::{STATE_READ, STATE_WRITE},
    power::pstate::{self, StatsSeen},
    regs::{mb, rmb, wmb},
    Shared,
};
use crate::{
    g17::telemetry::Readout,
    stats_m3::{self, FreqTable, HwPstateSnapshot, Telemetry},
};

impl Channels {
    fn note_stats_telemetry(
        ring: &Ring,
        seen: &mut StatsSeen,
        telemetry: &mut Telemetry,
        now_ns: u64,
    ) -> Result<Option<u32>> {
        let producer = ring.state.r32(STATE_WRITE)?;
        let read = ring.state.r32(STATE_READ)?;
        let Some(slots) = pstate::pending(read, producer) else {
            return Ok(None);
        };
        rmb();
        for slot in slots {
            let at = slot as usize * pstate::STATS_RECORD;
            let tag = ring.ring.r32(at + pstate::STATS_TAG)?;
            let pstate_word = ring.ring.r32(at + pstate::POWER_STATE_INDEX)?;
            seen.note(tag, pstate_word);
            let words = match tag {
                stats_m3::TAG_UTILIZATION | stats_m3::TAG_POWER_STATE => [
                    tag,
                    ring.ring.r32(at + 0x04)?,
                    ring.ring.r32(at + 0x08)?,
                    ring.ring.r32(at + 0x0c)?,
                    ring.ring.r32(at + 0x10)?,
                    ring.ring.r32(at + 0x14)?,
                    ring.ring.r32(at + 0x18)?,
                    ring.ring.r32(at + 0x1c)?,
                    pstate_word,
                ],
                _ => [tag, 0, 0, 0, 0, 0, 0, 0, pstate_word],
            };
            telemetry.record_words(&words, now_ns, (pstate::STATES - 1) as u32);
        }
        Ok(Some(producer))
    }

    /// Drains only the statistics ring, updating both `stats_seen` and `telemetry`,
    /// without touching event, log or trace rings and without waking the GPU.
    pub(crate) fn drain_stats_only(&mut self, now_ns: u64) -> Result {
        let mut seen = self.stats_seen;
        let mut telemetry = self.telemetry.clone();
        if let Some(producer) =
            Self::note_stats_telemetry(&self.stats, &mut seen, &mut telemetry, now_ns)?
        {
            self.stats_seen = seen;
            self.telemetry = telemetry;
            if self.stats.read()? != producer {
                mb();
                self.stats.state.w32(STATE_READ, producer)?;
                wmb();
            }
            return Ok(());
        }
        self.telemetry.invalidate();
        let producer = self.stats.state.r32(STATE_WRITE)?;
        if self.stats.state.r32(STATE_READ)? != producer {
            self.stats.state.w32(STATE_READ, producer)?;
            wmb();
        }
        Ok(())
    }
}

pub(crate) struct Reader(pub(super) Arc<Shared>, pub(super) FreqTable);

impl Reader {
    pub(crate) fn read(&self) -> Readout {
        let mut state = self.0.state.lock();
        let Some(firmware) = state.as_deref_mut() else {
            return Readout {
                util_source: stats_m3::UTIL_SOURCE_TAG_04,
                ..Readout::default()
            };
        };
        let now_ns = engine::now_ns();
        let unhealthy = self.0.crashed.load(Ordering::Acquire)
            || self.0.orphaned.load(Ordering::Acquire)
            || firmware.recovery.failed().is_some();
        if unhealthy {
            firmware.channels.telemetry.invalidate();
            return firmware.channels.telemetry.read(now_ns, None, &self.1);
        }
        if firmware.channels.drain_stats_only(now_ns).is_err() {
            firmware.channels.telemetry.invalidate();
            return firmware.channels.telemetry.read(now_ns, None, &self.1);
        }
        let hw = firmware.pstate_words().ok().map(|w| HwPstateSnapshot {
            actual: w.actual,
            current: w.current,
            freq_mhz_bits: w.freq_mhz,
            sgx_pstate: None,
        });
        firmware.channels.telemetry.read(now_ns, hw, &self.1)
    }

    pub(crate) fn suspend(&self, suspended: bool) {
        if let Some(firmware) = self.0.state.lock().as_deref_mut() {
            firmware.channels.telemetry.suspend(suspended);
        }
    }

    pub(crate) fn frequency(&self) -> Option<(Option<u64>, u64)> {
        let view = self.read();
        stats_m3::frequency_pair(&view, &self.1)
    }
}
