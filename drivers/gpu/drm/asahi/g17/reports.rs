// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Preserve the firmware's statistics without assigning unqualified units or
//! treating another firmware's pstate report as an achieved GPU frequency.
//! Writers and readers hold the device mutex; reads never power the GPU up.

use super::{fw::channels::{EVENT_WORDS, STATISTICS_TYPES}, Role};
use kernel::prelude::*;

#[derive(Clone, Copy, Default)]
struct Sample {
    count: u64,
    host_ns: u64,
    words: [u32; EVENT_WORDS],
}

pub(super) struct Reports {
    samples: KVec<Sample>,
}

impl Reports {
    pub(super) fn new() -> Result<Self> {
        // Allocate before firmware starts; receive and debugfs paths allocate nothing.
        let mut samples = KVec::with_capacity(64, GFP_KERNEL)?;
        for _ in 0..64 {
            samples.push(Sample::default(), GFP_KERNEL)?;
        }
        Ok(Self { samples })
    }

    pub(super) fn record(&mut self, role: Role, words: [u32; EVENT_WORDS], host_ns: u64) {
        let tag = words[0];
        if tag >= u32::BITS || STATISTICS_TYPES & (1 << tag) == 0 {
            return;
        }
        let sample = &mut self.samples[role as usize * 32 + tag as usize];
        sample.count = sample.count.saturating_add(1);
        sample.host_ns = host_ns;
        sample.words = words;
    }

    pub(super) fn write(&self, f: &mut kernel::fmt::Formatter<'_>) -> kernel::fmt::Result {
        writeln!(f, "format=1 units=raw host_time=monotonic_ns achieved_frequency=unqualified")?;
        for (index, sample) in self.samples.iter().enumerate().filter(|(_, s)| s.count != 0) {
            writeln!(f, "role={} tag={:#x} count={} host_ns={} words={:08x?}",
                index / 32, index % 32, sample.count, sample.host_ns, sample.words)?;
        }
        Ok(())
    }
}

/// The view owns the session, not a pointer into a detachable Firmware.
pub(super) struct View(pub(super) kernel::sync::Arc<super::Shared>);

impl kernel::debugfs::Writer for View {
    fn write(&self, f: &mut kernel::fmt::Formatter<'_>) -> kernel::fmt::Result {
        let state = self.0.state.lock();
        let Some(firmware) = state.as_deref() else { return writeln!(f, "online=0"); };
        writeln!(f, "online=1 graph={:?} sampled_ns={}", self.0.variant.graph, super::now_ns())?;
        write_power(firmware, f)?;
        firmware.rings.write_reports(f)
    }
}

fn write_power(firmware: &super::Firmware, f: &mut kernel::fmt::Formatter<'_>) -> kernel::fmt::Result {
    let core_power = firmware.bridge.regs.g17_core_power();
    writeln!(f, "power_management=firmware sampled_ns={} core_power={:#x} cores_powered={} host_hold={} idle_hold_ms={}",
        super::now_ns(), core_power, core_power & 0xf != 0, firmware.hold.active(),
        super::power::IDLE_HOLD_MS.load(core::sync::atomic::Ordering::Relaxed))?;
    firmware.port.write_power(f)
}

/// Actual firmware-managed core gating, separate from generic Linux runtime PM.
pub(super) struct PowerView(pub(super) kernel::sync::Arc<super::Shared>);

impl kernel::debugfs::Writer for PowerView {
    fn write(&self, f: &mut kernel::fmt::Formatter<'_>) -> kernel::fmt::Result {
        let state = self.0.state.lock();
        let Some(firmware) = state.as_deref() else { return writeln!(f, "online=0"); };
        write_power(firmware, f)
    }
}

#[kernel::macros::kunit_tests(asahi_g17_reports)]
mod tests {
    use super::*;

    #[test]
    fn retains_all_words_and_separates_role_and_tag() {
        let mut reports = Reports::new().unwrap();
        let mut words = [0; EVENT_WORDS];
        words[0] = 16;
        words[17] = 0xdeadbeef;
        reports.record(Role::Primary, words, 100);
        reports.record(Role::Primary, words, 101);
        reports.record(Role::Secondary, words, 102);
        assert_eq!(reports.samples[16].count, 2);
        assert_eq!(reports.samples[16].host_ns, 101);
        assert_eq!(reports.samples[16].words, words);
        assert_eq!(reports.samples[48].count, 1);
        words[0] = 17;
        reports.record(Role::Primary, words, 103);
        assert_eq!(reports.samples[17].count, 1);
        assert_eq!(reports.samples[16].count, 2);
    }

    #[test]
    fn rejects_invalid_tags_before_indexing() {
        let mut reports = Reports::new().unwrap();
        for tag in [0, 15, 29, 31, 32, u32::MAX] {
            let mut words = [0; EVENT_WORDS];
            words[0] = tag;
            reports.record(Role::Primary, words, 1);
        }
        assert!(reports.samples.iter().all(|s| s.count == 0));
    }
}
