// SPDX-License-Identifier: GPL-2.0-only OR MIT

#![cfg_attr(
    not(any(CONFIG_DRM_ASAHI_G15, CONFIG_DRM_ASAHI_M3_T8122)),
    allow(dead_code)
)]

//! Qualified statistics decoder for M3 backends (`m3/` T8122 and `g15/` T6030/T6031/T6034).
//!
//! RTKit 2419 (`m3/` legacy profile and `g15/`) emits 0x40-byte records on its
//! 256-slot statistics ring using the classic AGX `StatsMsg` tags (`0x02` PowerOn,
//! `0x03` PowerOff, `0x04` Utilization, `0x0b` PowerState), while `HardwareDataA`
//! carries the firmware's latest performance state and frequency words. The extended
//! M3 profile (`m3/` macOS 26 firmware) exposes hardware pstate and its frequency
//! table, with utilization/busy time marked unsupported until its ring tags are
//! qualified.
//!
//! Core-only (no `kernel` imports in the state machine) so host `rustc --test` and
//! KUnit can both exercise every transition.

#[cfg(not(test))]
pub(crate) use crate::g17::telemetry::{Readout, STALE, UNAVAILABLE, UNSUPPORTED, VALID};

#[cfg(test)]
pub(crate) const UNAVAILABLE: u32 = 0;
#[cfg(test)]
pub(crate) const VALID: u32 = 1;
#[cfg(test)]
pub(crate) const STALE: u32 = 2;
#[cfg(test)]
pub(crate) const UNSUPPORTED: u32 = 3;

#[cfg(test)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(C)]
pub(crate) struct Readout {
    pub(crate) util_status: u32,
    pub(crate) pstate_status: u32,
    pub(crate) tiler: u32,
    pub(crate) renderer: u32,
    pub(crate) device: u32,
    pub(crate) pstate: u32,
    pub(crate) util_host_ns: u64,
    pub(crate) pstate_host_ns: u64,
    pub(crate) util_samples: u64,
    pub(crate) pstate_timestamp: u64,
    pub(crate) busy_ns: u64,
    pub(crate) achieved_frequency_hz: u64,
    pub(crate) busy_status: u32,
    pub(crate) util_source: u32,
}

/// Statistics ring wire layout on RTKit 2419 (matches `RawStatsMsgG13V13_5`).
pub(crate) const STATS_RECORD_BYTES: usize = 0x40;
pub(crate) const TAG_POWER_ON: u32 = 0x02;
pub(crate) const TAG_POWER_OFF: u32 = 0x03;
pub(crate) const TAG_UTILIZATION: u32 = 0x04;
pub(crate) const TAG_POWER_STATE: u32 = 0x0b;

/// `Readout::util_source` values matching `sysfs.c::asahi_stats_util_source`.
pub(crate) const UTIL_SOURCE_TAG_04: u32 = 1;
pub(crate) const UTIL_SOURCE_UNSUPPORTED: u32 = 2;

/// Firmware timestamp clock rate (24 MHz on every Apple Silicon SoC).
const TIMER_HZ: u64 = 24_000_000;
/// Maximum host age of a `0x04` utilization sample before it turns stale (2.5 s).
/// Classic AGX firmware emits `Utilization` at ~1 Hz while active.
const MAX_UTIL_AGE_NS: u64 = 2_500_000_000;
/// Maximum single `Utilization` window credited to `busy_ns` (2 s).
const MAX_BUSY_WINDOW_TICKS: u64 = 2 * TIMER_HZ;

/// Frequency table of the firmware's performance states in MHz; state 0 is off.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) struct FreqTable {
    mhz: [u32; Self::MAX],
    max_state: u32,
}

impl FreqTable {
    pub(crate) const MAX: usize = 16;

    pub(crate) fn from_slice(mhz: &[u32], max_state: u32) -> Self {
        let clamped_max = (max_state as usize).min(Self::MAX - 1);
        let mut table = [0u32; Self::MAX];
        for (slot, &val) in table.iter_mut().zip(mhz.iter()).take(clamped_max + 1) {
            *slot = val;
        }
        Self {
            mhz: table,
            max_state: clamped_max as u32,
        }
    }

    pub(crate) fn max_state(&self) -> u32 {
        self.max_state
    }

    pub(crate) fn hz(&self, pstate: u32) -> Option<u64> {
        if pstate == 0 {
            return Some(0);
        }
        if pstate > self.max_state {
            return None;
        }
        let mhz = self.mhz[pstate as usize];
        (mhz != 0).then_some(u64::from(mhz) * 1_000_000)
    }

    pub(crate) fn max_mhz(&self) -> u32 {
        self.mhz[..=self.max_state as usize]
            .iter()
            .copied()
            .max()
            .unwrap_or(0)
    }

    pub(crate) fn max_hz(&self) -> Option<u64> {
        let mhz = self.max_mhz();
        (mhz != 0).then_some(u64::from(mhz) * 1_000_000)
    }
}

/// Snapshot of firmware shared-RAM `HardwareDataA` and optional SGX pstate register.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) struct HwPstateSnapshot {
    pub(crate) actual: u32,
    pub(crate) current: u32,
    /// IEEE-754 single-precision MHz from `HardwareDataA + 0x58`, or 0 when unavailable.
    pub(crate) freq_mhz_bits: u32,
    /// Low nibble of `SGX + 0xe01000` when readable without waking the GPU.
    pub(crate) sgx_pstate: Option<u32>,
}

/// Convert IEEE-754 single-precision MHz bits to rounded kHz without floating-point ops.
pub(crate) fn f32_mhz_to_khz(bits: u32) -> Option<u32> {
    let exp = (bits >> 23) & 0xff;
    let frac = bits & 0x7f_ffff;
    if bits & 0x7fff_ffff == 0 {
        return Some(0);
    }
    if bits >> 31 != 0 || exp == 0xff {
        return None;
    }
    let (m, e) = if exp == 0 {
        (u64::from(frac), 1i32)
    } else {
        (u64::from(frac | 0x80_0000), exp as i32)
    };
    let scaled = m * 1000;
    let shift = e - 150;
    let khz = if shift >= 0 {
        if shift > 31 {
            return None;
        }
        scaled << shift
    } else {
        let s = (-shift) as u32;
        if s >= 64 {
            0
        } else {
            let q = scaled >> s;
            let rem = scaled & ((1u64 << s) - 1);
            let half = 1u64 << (s - 1);
            if rem > half || (rem == half && q & 1 == 1) {
                q + 1
            } else {
                q
            }
        }
    };
    if khz > u64::from(u32::MAX) {
        None
    } else {
        Some(khz as u32)
    }
}

/// Stateful qualified telemetry accumulator for M3 (`m3/` and `g15/`).
///
/// Protected by the backend's device mutex (`g15::Shared::state` or `m3::m3_drm::Shared`).
#[derive(Clone, Debug)]
pub(crate) struct Telemetry {
    supports_ring_util: bool,
    util_status: u32,
    util_tiler: u32,
    util_renderer: u32,
    util_device: u32,
    util_host_ns: u64,
    util_samples: u64,
    busy_status: u32,
    busy_ts: Option<u64>,
    busy_ns: u64,
    ring_pstate: Option<u32>,
    ring_pstate_ts: u64,
    ring_pstate_host_ns: u64,
    had_pstate: bool,
    powered_off: bool,
    suspended: bool,
}

impl Telemetry {
    pub(crate) const fn new(supports_ring_util: bool) -> Self {
        Self {
            supports_ring_util,
            util_status: if supports_ring_util {
                UNAVAILABLE
            } else {
                UNSUPPORTED
            },
            util_tiler: 0,
            util_renderer: 0,
            util_device: 0,
            util_host_ns: 0,
            util_samples: 0,
            busy_status: if supports_ring_util {
                UNAVAILABLE
            } else {
                UNSUPPORTED
            },
            busy_ts: None,
            busy_ns: 0,
            ring_pstate: None,
            ring_pstate_ts: 0,
            ring_pstate_host_ns: 0,
            had_pstate: false,
            powered_off: false,
            suspended: false,
        }
    }

    /// End the current telemetry epoch (fault, crash, suspend, resume, or teardown).
    /// Busy time remains cumulative, while its window baseline is dropped so gaps
    /// are never credited.
    pub(crate) fn invalidate(&mut self) {
        if self.supports_ring_util && self.util_status == VALID {
            self.util_status = STALE;
        }
        self.busy_ts = None;
        self.ring_pstate = None;
        self.powered_off = false;
    }

    /// PM suspend/resume boundary callback.
    pub(crate) fn suspend(&mut self, suspended: bool) {
        self.suspended = suspended;
        self.invalidate();
    }

    /// Decode the first 9 little-endian u32 words (`0x00..0x24`) of a 0x40-byte
    /// RTKit 2419 statistics ring record.
    pub(crate) fn record_words(&mut self, words: &[u32; 9], host_ns: u64, max_pstate: u32) {
        if self.suspended {
            return;
        }
        match words[0] {
            TAG_POWER_ON => {
                self.busy_ts = None;
                self.powered_off = false;
                if self.supports_ring_util && self.util_status == VALID {
                    self.util_status = STALE;
                }
            }
            TAG_POWER_OFF => {
                self.busy_ts = None;
                self.powered_off = true;
            }
            TAG_UTILIZATION if self.supports_ring_util => {
                let ts = u64::from(words[1]) | (u64::from(words[2]) << 32);
                let u1 = words[3];
                let u2 = words[4];
                let u3 = words[5];
                let u4 = words[6];
                if u1 > 100 || u2 > 100 || u3 > 100 || u4 > 100 {
                    self.util_status = UNAVAILABLE;
                    self.busy_ts = None;
                    return;
                }
                let device = u1.max(u2).max(u3).max(u4);
                self.integrate_busy(ts, device);
                self.util_tiler = u1;
                self.util_renderer = u2;
                self.util_device = device;
                self.util_host_ns = host_ns;
                self.util_samples = self.util_samples.saturating_add(1);
                self.util_status = VALID;
                self.busy_status = VALID;
                self.powered_off = false;
            }
            TAG_POWER_STATE => {
                let ts = u64::from(words[1]) | (u64::from(words[2]) << 32);
                let poweroff = words[6];
                let pstate = words[8];
                if pstate <= max_pstate {
                    self.ring_pstate = Some(pstate);
                    self.ring_pstate_ts = ts;
                    self.ring_pstate_host_ns = host_ns;
                    self.had_pstate = true;
                }
                if poweroff != 0 || pstate == 0 {
                    self.powered_off = true;
                    self.busy_ts = None;
                } else {
                    self.powered_off = false;
                }
            }
            _ => {}
        }
    }

    /// Decode a raw 0x40-byte statistics ring record.
    pub(crate) fn record_bytes(&mut self, raw: &[u8; STATS_RECORD_BYTES], host_ns: u64, max_pstate: u32) {
        let mut words = [0u32; 9];
        for (i, slot) in words.iter_mut().enumerate() {
            let off = i * 4;
            *slot = u32::from_le_bytes([raw[off], raw[off + 1], raw[off + 2], raw[off + 3]]);
        }
        self.record_words(&words, host_ns, max_pstate);
    }

    fn integrate_busy(&mut self, ts: u64, device_pct: u32) {
        if let Some(last) = self.busy_ts {
            if ts > last {
                let delta = ts - last;
                if delta <= MAX_BUSY_WINDOW_TICKS {
                    let util = u64::from(device_pct.min(100));
                    let whole_s = delta / TIMER_HZ;
                    let rem = delta % TIMER_HZ;
                    let ns = whole_s
                        .saturating_mul(util)
                        .saturating_mul(10_000_000)
                        .saturating_add(rem * util * 1_000_000_000 / (100 * TIMER_HZ));
                    self.busy_ns = self.busy_ns.saturating_add(ns);
                }
            }
        }
        self.busy_ts = Some(ts);
    }

    /// Build the C-compatible `Readout` without modifying firmware state.
    pub(crate) fn read(
        &self,
        now_ns: u64,
        hw: Option<HwPstateSnapshot>,
        table: &FreqTable,
    ) -> Readout {
        let mut out = Readout {
            util_source: if self.supports_ring_util {
                UTIL_SOURCE_TAG_04
            } else {
                UTIL_SOURCE_UNSUPPORTED
            },
            ..Readout::default()
        };

        let gated = self.powered_off || hw.is_some_and(|h| h.sgx_pstate == Some(0));

        // Utilization & busy_ns capability.
        if !self.supports_ring_util {
            out.util_status = UNSUPPORTED;
            out.busy_status = UNSUPPORTED;
        } else {
            let expired =
                now_ns < self.util_host_ns || now_ns.saturating_sub(self.util_host_ns) >= MAX_UTIL_AGE_NS;
            if self.suspended {
                out.util_status = if self.util_samples > 0 { STALE } else { UNAVAILABLE };
            } else if gated {
                // Power-gated GPU cores execute no work: 0% idle is a real observation
                // immediately on gating, matching pstate 0 / 0 Hz without waiting for expiry.
                out.util_status = VALID;
                out.tiler = 0;
                out.renderer = 0;
                out.device = 0;
            } else if self.util_status == VALID && expired {
                out.util_status = STALE;
            } else if self.util_status == VALID {
                out.util_status = VALID;
                out.tiler = self.util_tiler;
                out.renderer = self.util_renderer;
                out.device = self.util_device;
            } else {
                out.util_status = self.util_status;
            }
            out.util_samples = self.util_samples;
            out.util_host_ns = self.util_host_ns;
            out.busy_status = self.busy_status;
            if self.busy_status == VALID {
                out.busy_ns = self.busy_ns;
            }
        }

        // Performance state & achieved frequency capability.
        if self.suspended {
            out.pstate_status = if self.had_pstate || hw.is_some() {
                STALE
            } else {
                UNAVAILABLE
            };
            return out;
        }

        let resolved_pstate = if gated {
            Some(0)
        } else if let Some(ps) = self.ring_pstate.filter(|&ps| ps <= table.max_state()) {
            Some(ps)
        } else if let Some(h) = hw {
            if h.current > 0 && h.current <= table.max_state() {
                Some(h.current)
            } else if h.actual > 0 && h.actual <= table.max_state() {
                Some(h.actual)
            } else if let Some(reg) = h.sgx_pstate.filter(|&s| s <= table.max_state()) {
                Some(reg)
            } else if h.current == 0 && h.actual == 0 {
                Some(0)
            } else {
                None
            }
        } else {
            None
        };

        if let Some(pstate) = resolved_pstate {
            out.pstate_status = VALID;
            out.pstate = pstate;
            out.pstate_host_ns = if self.ring_pstate.is_some() {
                self.ring_pstate_host_ns
            } else {
                now_ns
            };
            out.pstate_timestamp = self.ring_pstate_ts;
            out.achieved_frequency_hz = if pstate == 0 {
                0
            } else {
                let hw_hz = hw
                    .filter(|h| {
                        h.current == pstate || h.actual == pstate || table.hz(pstate).is_none()
                    })
                    .and_then(|h| {
                        let khz = f32_mhz_to_khz(h.freq_mhz_bits)?;
                        let max_khz = table.max_mhz().saturating_mul(1000);
                        (khz > 0 && (max_khz == 0 || khz <= max_khz))
                            .then_some(u64::from(khz) * 1000)
                    });
                hw_hz.or_else(|| table.hz(pstate)).unwrap_or(0)
            };
        } else {
            out.pstate_status = if self.had_pstate { STALE } else { UNAVAILABLE };
        }

        out
    }
}

/// Extract `(current_hz, max_hz)` for DRM fdinfo from a qualified readout and frequency table.
pub(crate) fn frequency_pair(view: &Readout, table: &FreqTable) -> Option<(Option<u64>, u64)> {
    let max_hz = table.max_hz()?;
    let current = (view.pstate_status == VALID
        && (view.achieved_frequency_hz != 0 || view.pstate == 0))
        .then_some(view.achieved_frequency_hz);
    Some((current, max_hz))
}

#[cfg_attr(not(test), kernel::macros::kunit_tests(asahi_m3_stats))]
#[cfg(any(test, CONFIG_KUNIT))]
mod tests {
    use super::*;

    fn m3_table() -> FreqTable {
        FreqTable::from_slice(&[0, 338, 618, 796, 954, 1056, 1188, 1296, 1380], 8)
    }

    fn util_record(ts: u64, u1: u32, u2: u32, u3: u32, u4: u32) -> [u32; 9] {
        [
            TAG_UTILIZATION,
            ts as u32,
            (ts >> 32) as u32,
            u1,
            u2,
            u3,
            u4,
            0,
            0,
        ]
    }

    fn pstate_record(ts: u64, poweroff: u32, pstate: u32) -> [u32; 9] {
        [
            TAG_POWER_STATE,
            ts as u32,
            (ts >> 32) as u32,
            0,
            0,
            0,
            poweroff,
            0,
            pstate,
        ]
    }

    #[test]
    fn initial_state_reports_unavailable_never_fabricated_zero() {
        let t = Telemetry::new(true);
        let table = m3_table();
        let hw = HwPstateSnapshot {
            actual: 2,
            current: 2,
            freq_mhz_bits: 618.0f32.to_bits(),
            sgx_pstate: Some(2),
        };
        let view = t.read(1_000, Some(hw), &table);
        assert_eq!(view.util_status, UNAVAILABLE);
        assert_eq!(view.busy_status, UNAVAILABLE);
        assert_eq!(view.util_source, UTIL_SOURCE_TAG_04);
        assert_eq!(view.pstate_status, VALID);
        assert_eq!(view.pstate, 2);
        assert_eq!(view.achieved_frequency_hz, 618_000_000);
    }

    #[test]
    fn extended_profile_marks_utilization_unsupported_honestly() {
        let t = Telemetry::new(false);
        let table = m3_table();
        let hw = HwPstateSnapshot {
            actual: 5,
            current: 5,
            freq_mhz_bits: 0,
            sgx_pstate: Some(5),
        };
        let view = t.read(1_000, Some(hw), &table);
        assert_eq!(view.util_status, UNSUPPORTED);
        assert_eq!(view.busy_status, UNSUPPORTED);
        assert_eq!(view.util_source, UTIL_SOURCE_UNSUPPORTED);
        assert_eq!(view.pstate_status, VALID);
        assert_eq!(view.pstate, 5);
        assert_eq!(view.achieved_frequency_hz, 1_056_000_000);
    }

    #[test]
    fn utilization_records_integrate_busy_ns_and_reject_out_of_range() {
        let mut t = Telemetry::new(true);
        let table = m3_table();
        let mut t0 = 24_000_000u64;
        t.record_words(&util_record(t0, 40, 60, 20, 60), 100_000_000, 8);
        let v0 = t.read(100_000_000, None, &table);
        assert_eq!(v0.util_status, VALID);
        assert_eq!((v0.tiler, v0.renderer, v0.device), (40, 60, 60));
        assert_eq!((v0.busy_status, v0.busy_ns), (VALID, 0));

        // 500 ms window at 80% busy -> 400 ms added.
        t0 += 12_000_000;
        t.record_words(&util_record(t0, 50, 80, 10, 80), 600_000_000, 8);
        let v1 = t.read(700_000_000, None, &table);
        assert_eq!(v1.util_status, VALID);
        assert_eq!(v1.device, 80);
        assert_eq!(v1.busy_ns, 400_000_000);

        // Out-of-range percentage invalidates utilization instead of clamping.
        t0 += 12_000_000;
        t.record_words(&util_record(t0, 101, 50, 50, 50), 1_100_000_000, 8);
        assert_eq!(t.read(1_100_000_000, None, &table).util_status, UNAVAILABLE);
        assert_eq!(t.read(1_100_000_000, None, &table).busy_ns, 400_000_000);
    }

    #[test]
    fn power_off_and_suspend_drop_busy_baseline_and_mark_stale() {
        let mut t = Telemetry::new(true);
        let table = m3_table();
        t.record_words(&util_record(24_000_000, 100, 100, 0, 100), 1_000_000_000, 8);
        t.record_words(&util_record(36_000_000, 100, 100, 0, 100), 1_500_000_000, 8);
        assert_eq!(t.read(1_500_000_000, None, &table).busy_ns, 500_000_000);

        // PowerOff reads valid 0% and pstate 0 (0 Hz) immediately while gated,
        // without waiting for the 2.5 s staleness window.
        t.record_words(&[TAG_POWER_OFF, 0, 0, 0, 0, 0, 0, 0, 0], 1_600_000_000, 8);
        let immediate_idle = t.read(1_610_000_000, None, &table);
        assert_eq!((immediate_idle.util_status, immediate_idle.device), (VALID, 0));
        assert_eq!(
            (
                immediate_idle.pstate_status,
                immediate_idle.pstate,
                immediate_idle.achieved_frequency_hz
            ),
            (VALID, 0, 0)
        );
        let idle = t.read(5_000_000_000, None, &table);
        assert_eq!((idle.util_status, idle.device), (VALID, 0));
        assert_eq!((idle.pstate_status, idle.pstate, idle.achieved_frequency_hz), (VALID, 0, 0));

        // PowerOn + first sample after gap only rebases; busy_ns does not jump.
        t.record_words(&[TAG_POWER_ON, 0, 0, 0, 0, 0, 0, 0, 0], 6_000_000_000, 8);
        t.record_words(&util_record(120_000_000, 100, 100, 0, 100), 6_100_000_000, 8);
        assert_eq!(t.read(6_100_000_000, None, &table).busy_ns, 500_000_000);

        // Suspend marks stale and ignores records until resumed.
        t.record_words(&pstate_record(120_000_000, 0, 8), 6_200_000_000, 8);
        assert_eq!(t.read(6_200_000_000, None, &table).achieved_frequency_hz, 1_380_000_000);
        t.suspend(true);
        let susp = t.read(6_300_000_000, None, &table);
        assert_eq!(susp.util_status, STALE);
        assert_eq!(susp.pstate_status, STALE);
        t.record_words(&util_record(132_000_000, 100, 100, 0, 100), 6_400_000_000, 8);
        assert_eq!(t.read(6_400_000_000, None, &table).busy_ns, 500_000_000);
        t.suspend(false);
        assert_eq!(t.read(6_500_000_000, None, &table).util_status, STALE);
        assert_eq!(t.read(6_500_000_000, None, &table).pstate_status, STALE);
    }

    #[test]
    fn pstate_record_overrides_hwdata_and_exposes_fdinfo_frequency() {
        let mut t = Telemetry::new(true);
        let table = m3_table();
        let hw = HwPstateSnapshot {
            actual: 2,
            current: 2,
            freq_mhz_bits: 0,
            sgx_pstate: Some(2),
        };
        t.record_words(&pstate_record(48_000_000, 0, 7), 2_000_000_000, 8);
        let view = t.read(2_000_000_000, Some(hw), &table);
        assert_eq!((view.pstate_status, view.pstate), (VALID, 7));
        assert_eq!(view.achieved_frequency_hz, 1_296_000_000);
        assert_eq!(
            frequency_pair(&view, &table),
            Some((Some(1_296_000_000), 1_380_000_000))
        );
    }

    #[test]
    fn record_bytes_and_stale_hw_freq_use_matching_table_frequency() {
        let mut t = Telemetry::new(true);
        let table = m3_table();
        assert_eq!(table.max_state(), 8);
        let mut raw = [0u8; STATS_RECORD_BYTES];
        let words = pstate_record(72_000_000, 0, 7);
        for (i, w) in words.iter().enumerate() {
            raw[i * 4..i * 4 + 4].copy_from_slice(&w.to_le_bytes());
        }
        t.record_bytes(&raw, 3_000_000_000, table.max_state());
        let stale_hw = HwPstateSnapshot {
            actual: 2,
            current: 2,
            freq_mhz_bits: 618.0f32.to_bits(),
            sgx_pstate: Some(2),
        };
        let view = t.read(3_000_000_000, Some(stale_hw), &table);
        assert_eq!((view.pstate_status, view.pstate), (VALID, 7));
        assert_eq!(view.achieved_frequency_hz, 1_296_000_000);
    }
}
