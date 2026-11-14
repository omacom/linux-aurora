// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Qualified V25G83/V26A428 statistics, decoded from complete copied records.
//! No record measures busy time, power, temperature or achieved frequency.
//! A/B table selection is absent, so neither nominal candidate is exported as
//! a requested or achieved frequency. All state is protected by the device mutex.

pub(super) const UNAVAILABLE: u32 = 0;
pub(super) const VALID: u32 = 1;
pub(super) const STALE: u32 = 2;

/// C read callback result; independent capabilities never imply one another.
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
}

const _: () = {
    assert!(core::mem::offset_of!(Readout, util_host_ns) == 24);
    assert!(core::mem::size_of::<Readout>() == 56);
};

#[derive(Default)]
pub(super) struct Telemetry {
    view: Readout,
    sums: [u64; 3],
    window_ns: u64,
    timestamp: Option<u64>,
    /// Diagnostic interval only, never used as busy time.
    pub(super) delta_ticks: Option<u64>,
}

impl Telemetry {
    /// End an epoch. Preserve the reason that formerly valid data is stale,
    /// but never carry an average or a timestamp baseline into a new epoch.
    pub(super) fn invalidate(&mut self) {
        if self.view.util_status != UNAVAILABLE {
            self.view.util_status = STALE;
        }
        if self.view.pstate_status != UNAVAILABLE {
            self.view.pstate_status = STALE;
        }
        self.view.util_samples = 0;
        self.timestamp = None;
        self.delta_ticks = None;
    }

    /// `max_age_ns` is a host policy (two nominal periods), not a packet duration.
    /// Every role contributes one equally weighted sample to the same bounded
    /// collection window. Reads do not consume samples or change the mean.
    pub(super) fn record(&mut self, words: &[u32; 18], host_ns: u64, max_age_ns: u64) {
        match words[0] {
            0x13 => {
                let values = [words[3], words[4], words[6]];
                if values.iter().any(|&value| value > 100) {
                    self.view.util_status = UNAVAILABLE;
                    self.view.util_samples = 0;
                    return;
                }
                if self.view.util_samples == 0
                    || host_ns.saturating_sub(self.window_ns) >= max_age_ns
                    || host_ns < self.window_ns
                    || self.view.util_samples >= u64::MAX / 100
                {
                    self.sums = [0; 3];
                    self.view.util_samples = 0;
                    self.window_ns = host_ns;
                }
                for (sum, value) in self.sums.iter_mut().zip(values) {
                    *sum += u64::from(value);
                }
                self.view.util_samples += 1;
                self.view.util_host_ns = host_ns;
                self.view.util_status = VALID;
                self.view.tiler = (self.sums[0] / self.view.util_samples) as u32;
                self.view.renderer = (self.sums[1] / self.view.util_samples) as u32;
                self.view.device = (self.sums[2] / self.view.util_samples) as u32;
            }
            0x10 => {
                let timestamp = u64::from(words[1]) | (u64::from(words[2]) << 32);
                if self.timestamp.is_some_and(|last| timestamp < last) {
                    self.invalidate();
                    self.view.pstate_status = UNAVAILABLE;
                    return;
                }
                self.delta_ticks = self.timestamp.map(|last| timestamp - last);
                self.timestamp = Some(timestamp);
                self.view.pstate_host_ns = host_ns;
                self.view.pstate_timestamp = timestamp;
                self.view.pstate_status = if words[6] <= 10 { VALID } else { UNAVAILABLE };
                self.view.pstate = words[6];
            }
            _ => {}
        }
    }

    pub(super) fn read(&self, now_ns: u64, max_age_ns: u64) -> Readout {
        let mut view = self.view;
        let expired = |received| now_ns < received || now_ns - received >= max_age_ns;
        // The oldest contributing sample bounds the whole averaging window.
        if view.util_status == VALID && expired(self.window_ns) {
            view.util_status = STALE;
        }
        // A pstate report describes a state transition, with no qualified
        // cadence. Its validity ends on replacement or epoch invalidation,
        // rather than an invented periodic-report deadline.
        view
    }
}

#[cfg_attr(not(test), kernel::macros::kunit_tests(asahi_g17_telemetry))]
#[cfg(any(test, CONFIG_KUNIT))]
mod tests {
    use super::*;

    const AGE: u64 = 16_000_000;
    const U1: [u32; 18] = [
        0x13, 0, 0, 25, 75, 0xdeadbeef, 50, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    ];
    const U2: [u32; 18] = [
        0x13, 0, 0, 35, 85, 0x12345678, 65, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    ];
    const U3: [u32; 18] = [0x13, 0, 0, 101, 50, 50, 50, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    const P1: [u32; 18] = [0x10, 0x10, 1, 0, 0, 0, 6, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    const P2A: [u32; 18] = [0x10, 0x1000, 0, 0, 0, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    const P2B: [u32; 18] = [0x10, 0x2500, 0, 0, 0, 0, 6, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    const P3: [u32; 18] = [
        0x10, 0x3000, 0, 0, 0, 0, 11, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    ];

    #[test]
    fn spec_u1_u2_u3_both_firmware_policies() {
        for age in [AGE, 2 * AGE] {
            let mut stats = Telemetry::default();
            assert_eq!(stats.read(0, age), Readout::default());
            stats.record(&U1, 100, age);
            let view = stats.read(100, age);
            assert_eq!((view.tiler, view.renderer, view.device), (25, 75, 50));
            assert_eq!(view.util_status, VALID);
            stats.record(&U2, 101, age); // May be the other role; no deduplication.
            let view = stats.read(101, age);
            assert_eq!((view.tiler, view.renderer, view.device), (30, 80, 57));
            assert_eq!(view.util_samples, 2);
            stats.record(&U3, 102, age);
            assert_eq!(stats.read(102, age).util_status, UNAVAILABLE);
            stats.record(&U2, 103, age);
            assert_eq!(stats.read(103, age).device, 65);
        }
    }

    #[test]
    fn spec_p1_p2_p3_and_regression() {
        let mut stats = Telemetry::default();
        stats.record(&P1, 100, AGE);
        let view = stats.read(100, AGE);
        assert_eq!(view.pstate, 6);
        assert_eq!(view.pstate_status, VALID);
        assert_eq!(view.pstate_timestamp, 0x1_0000_0010);
        assert_eq!(stats.delta_ticks, None); // First sample is only a baseline.
        stats.invalidate();
        stats.record(&P2A, 101, AGE);
        stats.record(&P2B, 102, AGE);
        assert_eq!(stats.delta_ticks, Some(5376));
        assert_eq!(
            stats.delta_ticks.unwrap() * 1_000_000_000 / 24_000_000,
            224_000
        );
        stats.record(&P3, 103, AGE);
        assert_eq!(stats.read(103, AGE).pstate_status, UNAVAILABLE);
        stats.record(&U1, 104, AGE);
        stats.record(&P2A, 105, AGE); // Backwards, including across role drain order.
        assert_eq!(stats.read(105, AGE).pstate_status, UNAVAILABLE);
        assert_eq!(stats.read(105, AGE).util_status, STALE);
        assert_eq!(stats.delta_ticks, None);
        stats.record(&P2A, 106, AGE); // New epoch establishes a new baseline.
        assert_eq!(stats.read(106, AGE).pstate_status, VALID);
        assert_eq!(stats.delta_ticks, None);
        stats.record(&P2A, 107, AGE); // Equal timestamps are non-regressing.
        assert_eq!(stats.delta_ticks, Some(0));
    }

    #[test]
    fn age_epoch_and_zero_are_independent() {
        let mut stats = Telemetry::default();
        stats.record(&U1, 100, AGE);
        stats.record(&P1, 200, AGE);
        assert_eq!(stats.read(99, AGE).util_status, STALE);
        assert_eq!(stats.read(100 + AGE - 1, AGE).util_status, VALID);
        let view = stats.read(100 + AGE, AGE);
        assert_eq!(view.util_status, STALE);
        assert_eq!(view.pstate_status, VALID);
        assert_eq!(stats.read(200 + AGE, AGE).pstate_status, VALID);
        assert_eq!(stats.read(u64::MAX, AGE).pstate_status, VALID);
        stats.record(&U2, 100 + AGE, AGE);
        assert_eq!(stats.read(100 + AGE, AGE).device, 65); // Old window was discarded.
        stats.invalidate(); // Suspend, restart, quarantine, gate or teardown.
        let view = stats.read(100 + AGE, AGE);
        assert_eq!(view.util_status, STALE);
        assert_eq!(view.pstate_status, STALE);
        let mut zero = [0; 18];
        zero[0] = 0x13;
        stats.record(&zero, 101 + AGE, AGE);
        let view = stats.read(101 + AGE, AGE);
        assert_eq!(view.util_status, VALID);
        assert_eq!((view.tiler, view.renderer, view.device), (0, 0, 0));
        zero[0] = 0x10;
        stats.record(&zero, 102 + AGE, AGE);
        assert_eq!(stats.read(102 + AGE, AGE).pstate, 0);
        assert_eq!(stats.read(102 + AGE, AGE).pstate_status, VALID);
    }

    #[test]
    fn invalid_components_and_selectors_never_clamp() {
        for word in [3, 4, 6] {
            let mut stats = Telemetry::default();
            let mut words = U1;
            words[word] = u32::MAX;
            stats.record(&words, 1, AGE);
            assert_eq!(stats.read(1, AGE).util_status, UNAVAILABLE);
        }
        for selector in [11, 15, 16, u32::MAX] {
            let mut stats = Telemetry::default();
            let mut words = P1;
            words[6] = selector;
            stats.record(&words, 1, AGE);
            assert_eq!(stats.read(1, AGE).pstate_status, UNAVAILABLE);
        }
        let mut stats = Telemetry::default();
        for tag in [0x11, 0x12, 0x18, 0x1d, u32::MAX] {
            let mut words = U1;
            words[0] = tag;
            stats.record(&words, 1, AGE);
        }
        assert_eq!(stats.read(1, AGE), Readout::default());
    }
}
