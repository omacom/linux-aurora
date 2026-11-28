// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Qualified V25G83/V26A428 statistics, decoded from complete copied records.
//! No record of these firmwares measures power or temperature.
//!
//! The 26A428 firmware (the A18 Pro's, and the M5's) reports the performance state the
//! GPU runs at (tags 0x19 and 0x1b), and its utilisation records carry their own
//! timestamps, from which busy time is integrated ([`Protocol`]). 25G83 sends neither.
//! All state is protected by the device mutex.

pub(crate) const UNAVAILABLE: u32 = 0;
pub(crate) const VALID: u32 = 1;
pub(crate) const STALE: u32 = 2;
/// The firmware never emits the record, so the capability cannot become
/// valid. Chosen from the firmware match data, never from missing packets.
pub(crate) const UNSUPPORTED: u32 = 3;

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
    /// Utilisation-weighted busy time since the device was created, while `busy_status` is
    /// [`VALID`] ([`Protocol::util_period_ticks`]).
    pub(crate) busy_ns: u64,
    /// Frequency of `pstate` while `pstate_status` is [`VALID`], from the frequency of each
    /// state ([`super::stats::Reader`]); zero for state 0 or where it is unknown.
    pub(crate) achieved_frequency_hz: u64,
    pub(crate) busy_status: u32,
    /// Utilisation source selector for `sysfs.c`: 0 = tag 0x13 (G16S/G17P),
    /// 1 = tag 0x04 (G15/M3 RTKit 2419), 2 = unsupported.
    pub(crate) util_source: u32,
}

const _: () = {
    assert!(core::mem::offset_of!(Readout, util_host_ns) == 24);
    assert!(core::mem::offset_of!(Readout, busy_ns) == 56);
    assert!(core::mem::size_of::<Readout>() == 80);
};

const TIMER_HZ: u64 = 24_000_000;

/// What a firmware reports beyond utilisation.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) struct Protocol {
    /// Highest performance state its state records (tags 0x19 and 0x1b) name; zero where it
    /// sends none. State 0 is off.
    pub(crate) max_pstate: u32,
    /// Interval of its utilisation record (tag 0x13), in timer ticks, whose timestamps busy
    /// time is integrated from; zero where it is not integrated.
    pub(crate) util_period_ticks: u64,
}

impl Protocol {
    /// 25G83: utilisation only.
    pub(crate) const NONE: Self = Self {
        max_pstate: 0,
        util_period_ticks: 0,
    };
    /// M5: the device tree's operating points 1 to 13, utilisation every 8 ms.
    pub(crate) const M5: Self = Self {
        max_pstate: 13,
        util_period_ticks: 192_000,
    };
    /// A18 Pro: states 1 to 10 of its eleven-state performance tables, utilisation every
    /// 16 ms.
    pub(crate) const A18_PRO: Self = Self {
        max_pstate: 10,
        util_period_ticks: 384_000,
    };

    pub(crate) const fn reports_pstate(self) -> bool {
        self.max_pstate != 0
    }

    fn integrates_busy(self) -> bool {
        self.util_period_ticks != 0
    }

    /// A utilisation record more than two intervals after the previous one follows a gap the
    /// firmware did not report (gated or lost records): it only sets the baseline.
    fn max_window_ticks(self) -> u64 {
        2 * self.util_period_ticks
    }
}

#[derive(Default)]
pub(super) struct Telemetry {
    view: Readout,
    sums: [u64; 3],
    window_ns: u64,
    timestamp: Option<u64>,
    /// Diagnostic interval only, never used as busy time.
    pub(super) delta_ticks: Option<u64>,
    /// Host receive time of the newest utilisation delivery in this epoch.
    delivery_ns: Option<u64>,
    /// The last two intervals between utilisation deliveries in this epoch.
    pub(super) delivery_intervals: [Option<u64>; 2],
    /// The GPU cores were power-gated at the last observation. The firmware
    /// gates them a few milliseconds after the last work completes and emits no
    /// utilisation record while gated, so a gated GPU is idle, not unknown.
    gated: bool,
    /// The records decoded beyond utilisation.
    protocol: Protocol,
    /// Timestamp of the latest utilisation record in this epoch: the start of the window the
    /// next one closes.
    busy_ts: Option<u64>,
    busy_ns: u64,
    /// End of the latest performance state interval (tag 0x19), in timer ticks.
    pstate_interval_end: Option<u64>,
}

impl Telemetry {
    /// Decoder for a firmware that also sends `protocol`'s records.
    pub(super) fn new(protocol: Protocol) -> Self {
        Self {
            protocol,
            ..Self::default()
        }
    }

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
        self.delivery_ns = None;
        self.delivery_intervals = [None; 2];
        self.gated = false;
        // Busy time keeps counting across epochs; only the window start is lost.
        self.busy_ts = None;
        self.pstate_interval_end = None;
    }

    /// The cores were observed power-gated. End the averaging epoch exactly as
    /// any other boundary does, but keep the last published utilisation until
    /// it ages out: it describes the work that ran just before the gate. After
    /// that the GPU reads as idle (0%) for as long as it stays gated.
    pub(super) fn gate(&mut self) {
        let view = self.view;
        self.invalidate();
        self.view.util_status = view.util_status;
        self.gated = true;
    }

    /// The firmware may publish several records per producer update: V25G83
    /// produces a utilisation record every 8 ms but the host receives them
    /// three at a time, every 24 ms. A sample is only late once the next
    /// delivery is overdue, so the age limit runs on top of the delivery
    /// interval. The shorter of the last two intervals is used, so one long
    /// gap cannot extend the validity of the sample that follows it.
    fn delivery_slack(&self) -> u64 {
        match self.delivery_intervals {
            [Some(a), Some(b)] => a.min(b),
            _ => 0,
        }
    }

    fn note_delivery(&mut self, host_ns: u64) {
        match self.delivery_ns {
            Some(last) if host_ns == last => return,
            Some(last) if host_ns > last => {
                self.delivery_intervals = [self.delivery_intervals[1], Some(host_ns - last)];
            }
            _ => self.delivery_intervals = [None; 2],
        }
        self.delivery_ns = Some(host_ns);
    }

    /// `max_age_ns` is a host policy (two nominal periods), not a packet duration.
    /// Every role contributes one equally weighted sample to the same bounded
    /// collection window. Reads do not consume samples or change the mean.
    pub(super) fn record(&mut self, words: &[u32; 18], host_ns: u64, max_age_ns: u64) {
        match words[0] {
            0x13 => {
                let values = [words[3], words[4], words[6]];
                self.note_delivery(host_ns);
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
                if self.protocol.integrates_busy() {
                    self.integrate_busy(
                        u64::from(words[1]) | (u64::from(words[2]) << 32),
                        words[6],
                    );
                }
                self.view.util_samples += 1;
                self.view.util_host_ns = host_ns;
                self.view.util_status = VALID;
                self.view.tiler = (self.sums[0] / self.view.util_samples) as u32;
                self.view.renderer = (self.sums[1] / self.view.util_samples) as u32;
                self.view.device = (self.sums[2] / self.view.util_samples) as u32;
            }
            // The logical pstate record, which no current firmware emits. A firmware that
            // reports its state in tags 0x19 and 0x1b never has it decoded.
            0x10 if !self.protocol.reports_pstate() => {
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
            // 26A428: the interval [words 3-4, words 1-2] that just ended. Its end
            // timestamps the state report that follows it.
            0x19 if self.protocol.reports_pstate() => {
                self.pstate_interval_end = Some(u64::from(words[1]) | (u64::from(words[2]) << 32));
            }
            // 26A428: the performance state now in effect, in words 1 and 3. A report whose
            // two words differ is not used.
            0x1b if self.protocol.reports_pstate() => {
                let pstate = words[1];
                if pstate != words[3] || pstate == 0 || pstate > self.protocol.max_pstate {
                    self.view.pstate_status = UNAVAILABLE;
                    return;
                }
                self.view.pstate = pstate;
                self.view.pstate_status = VALID;
                self.view.pstate_host_ns = host_ns;
                self.view.pstate_timestamp = self.pstate_interval_end.unwrap_or(0);
            }
            _ => {}
        }
    }

    /// Credits the window a utilisation record closes, at its device utilisation. The window
    /// starts at the previous record of the epoch; a record after a longer gap than
    /// [`Protocol::max_window_ticks`] (the GPU was gated or records were lost) or one whose
    /// timestamp went back only sets the baseline, so an idle gap is never credited.
    fn integrate_busy(&mut self, timestamp: u64, device: u32) {
        if let Some(last) = self.busy_ts {
            let window = timestamp.wrapping_sub(last);
            if timestamp > last && window <= self.protocol.max_window_ticks() {
                // At most two 16 ms periods (768000 ticks) x 100 x 10^9: no overflow.
                let ns = window * u64::from(device) * 1_000_000_000 / (100 * TIMER_HZ);
                self.busy_ns = self.busy_ns.saturating_add(ns);
            }
        }
        self.busy_ts = Some(timestamp);
    }

    pub(super) fn read(&self, now_ns: u64, max_age_ns: u64) -> Readout {
        let mut view = self.view;
        let limit = max_age_ns.saturating_add(self.delivery_slack());
        let expired = |received| now_ns < received || now_ns - received >= limit;
        // The oldest contributing sample bounds the whole averaging window,
        // measured against the next expected delivery plus the host policy.
        if self.gated && (view.util_status != VALID || expired(self.window_ns)) {
            // No work can run on gated cores: idle is a measurement here.
            view.util_status = VALID;
            view.tiler = 0;
            view.renderer = 0;
            view.device = 0;
            view.util_samples = 0;
        } else if view.util_status == VALID && expired(self.window_ns) {
            view.util_status = STALE;
        }
        // A pstate report describes a state transition, with no qualified
        // cadence. Its validity ends on replacement or epoch invalidation,
        // rather than an invented periodic-report deadline. Gated cores run
        // at no state: off (0) is a measurement here, as idle is above.
        if self.gated && self.protocol.reports_pstate() {
            view.pstate_status = VALID;
            view.pstate = 0;
        }
        if self.protocol.integrates_busy() {
            view.busy_status = VALID;
            view.busy_ns = self.busy_ns;
        } else {
            view.busy_status = UNSUPPORTED;
        }
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
            assert_eq!(stats.read(0, age), NOTHING);
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
    fn a_gated_gpu_reads_idle_after_its_last_sample_ages_out() {
        let mut stats = Telemetry::default();
        // Gated before any record (idle since boot): idle, not unknown.
        stats.gate();
        let view = stats.read(0, AGE);
        assert_eq!(view.util_status, VALID);
        assert_eq!((view.tiler, view.renderer, view.device), (0, 0, 0));
        // Busy, then gated: the last sample stays until it ages out.
        stats.invalidate();
        stats.record(&U1, 100, AGE);
        stats.gate();
        let view = stats.read(100 + AGE - 1, AGE);
        assert_eq!((view.util_status, view.device), (VALID, 50));
        let view = stats.read(100 + AGE, AGE);
        assert_eq!((view.util_status, view.tiler, view.renderer, view.device), (VALID, 0, 0, 0));
        // The gate ends the averaging epoch: the next record starts a new window.
        stats.record(&U2, 200 + AGE, AGE);
        let view = stats.read(200 + AGE, AGE);
        assert_eq!((view.device, view.util_samples), (65, 1));
        // A non-power boundary (suspend, crash, quarantine) is stale, never idle.
        stats.gate();
        stats.invalidate();
        assert_eq!(stats.read(u64::MAX, AGE).util_status, STALE);
    }

    #[test]
    fn batched_deliveries_stay_valid_until_the_next_one_is_late() {
        const BATCH: u64 = 24_000_000;
        let mut stats = Telemetry::default();
        // Three 8 ms records per producer update, received together.
        for n in 0..3u64 {
            for _ in 0..3 {
                stats.record(&U1, 1_000 + n * BATCH, AGE);
            }
        }
        let last = 1_000 + 2 * BATCH;
        assert_eq!(stats.delivery_intervals, [Some(BATCH), Some(BATCH)]);
        assert_eq!(stats.read(last, AGE).util_samples, 3);
        // Between two on-time deliveries the sample is valid.
        assert_eq!(stats.read(last + BATCH - 1, AGE).util_status, VALID);
        assert_eq!(stats.read(last + BATCH + AGE - 1, AGE).util_status, VALID);
        // Once the next delivery is late by the host policy, it is stale.
        assert_eq!(stats.read(last + BATCH + AGE, AGE).util_status, STALE);
        // One long gap does not extend the next sample's validity.
        stats.record(&U2, last + 10 * BATCH, AGE);
        assert_eq!(stats.delivery_intervals, [Some(BATCH), Some(10 * BATCH)]);
        assert_eq!(
            stats.read(last + 10 * BATCH + BATCH + AGE, AGE).util_status,
            STALE
        );
        // An epoch boundary forgets the cadence.
        stats.invalidate();
        stats.record(&U1, last + 20 * BATCH, AGE);
        assert_eq!(stats.delivery_intervals, [None, None]);
        assert_eq!(stats.read(last + 20 * BATCH + AGE, AGE).util_status, STALE);
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
        assert_eq!(stats.read(1, AGE), NOTHING);
    }

    /// M5 firmware records: the utilisation of three 8 ms windows of a full-GPU compute
    /// job, a performance state interval and the state report after it.
    const M5_U: [[u32; 18]; 3] = [
        [
            0x13, 0x433a6047, 0x5c, 0, 0, 0x64, 0x64, 0x36, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0,
        ],
        [
            0x13, 0x433d4e4f, 0x5c, 0, 0, 0x64, 0x64, 0x36, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0,
        ],
        [
            0x13, 0x43403c9b, 0x5c, 0, 0, 0x64, 0x64, 0x36, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0,
        ],
    ];
    const M5_INTERVAL: [u32; 18] = [
        0x19, 0x3c042f8f, 0x59, 0x3c040ecf, 0x59, 1, 0, 9, 0xd, 0xd, 9, 0x514, 0, 6, 0, 0, 0, 0,
    ];
    const M5_STATE: [u32; 18] = [
        0x1b, 0xd, 0x7f, 0xd, 0x7f, 0, 1, 9, 0, 0, 9, 0x514, 0, 6, 0, 0, 0, 0,
    ];
    const NOTHING: Readout = Readout {
        util_status: UNAVAILABLE,
        pstate_status: UNAVAILABLE,
        tiler: 0,
        renderer: 0,
        device: 0,
        pstate: 0,
        util_host_ns: 0,
        pstate_host_ns: 0,
        util_samples: 0,
        pstate_timestamp: 0,
        busy_ns: 0,
        achieved_frequency_hz: 0,
        busy_status: UNSUPPORTED,
        util_source: 0,
    };

    fn ticks_ns(ticks: u64) -> u64 {
        ticks * 1_000_000_000 / 24_000_000
    }

    #[test]
    fn m5_state_reports_give_the_pstate() {
        let mut stats = Telemetry::new(Protocol::M5);
        stats.record(&M5_INTERVAL, 100, AGE);
        assert_eq!(stats.read(100, AGE).pstate_status, UNAVAILABLE);
        stats.record(&M5_STATE, 101, AGE);
        let view = stats.read(101, AGE);
        assert_eq!((view.pstate_status, view.pstate), (VALID, 13));
        assert_eq!(view.pstate_timestamp, 0x59_3c04_2f8f);
        assert_eq!(view.pstate_host_ns, 101);
        let mut low = M5_STATE;
        (low[1], low[3]) = (1, 1);
        stats.record(&low, 102, AGE);
        assert_eq!(stats.read(102, AGE).pstate, 1);
        // Two different states in one report, off, or beyond the table: not a state.
        for (a, b) in [(1, 13), (0, 0), (14, 14)] {
            let mut words = M5_STATE;
            (words[1], words[3]) = (a, b);
            stats.record(&words, 103, AGE);
            assert_eq!(stats.read(103, AGE).pstate_status, UNAVAILABLE);
        }
        // Gated cores are off: state 0. Power-up ends the epoch: the state before it is
        // no longer current.
        stats.record(&M5_STATE, 104, AGE);
        stats.gate();
        let view = stats.read(105, AGE);
        assert_eq!((view.pstate_status, view.pstate), (VALID, 0));
        stats.invalidate();
        assert_eq!(stats.read(106, AGE).pstate_status, STALE);
    }

    #[test]
    fn m5_busy_time_integrates_utilisation_windows() {
        let mut stats = Telemetry::new(Protocol::M5);
        assert_eq!(
            (stats.read(0, AGE).busy_status, stats.read(0, AGE).busy_ns),
            (VALID, 0)
        );
        // The first record only sets the baseline; the next two close one window each.
        for (n, words) in M5_U.iter().enumerate() {
            stats.record(words, 100 + n as u64, AGE);
        }
        // Each window is rounded down to whole nanoseconds on its own.
        let windows = ticks_ns(u64::from(M5_U[1][1] - M5_U[0][1]))
            + ticks_ns(u64::from(M5_U[2][1] - M5_U[1][1]));
        assert_eq!(stats.read(102, AGE).busy_ns, windows);
        assert!((15_900_000..16_100_000).contains(&stats.read(102, AGE).busy_ns));
        // A window is credited at its device utilisation.
        let mut half = M5_U[2];
        half[1] += 192_000;
        half[6] = 50;
        stats.record(&half, 103, AGE);
        assert_eq!(stats.read(103, AGE).busy_ns, windows + 4_000_000);
        // A gate, a long gap or a timestamp going back only sets a new baseline.
        let before = stats.read(103, AGE).busy_ns;
        stats.gate();
        let mut after = half;
        after[1] += 192_000;
        stats.record(&after, 104, AGE);
        assert_eq!(stats.read(104, AGE).busy_ns, before);
        after[1] += 3 * 192_000;
        stats.record(&after, 105, AGE);
        assert_eq!(stats.read(105, AGE).busy_ns, before);
        after[1] -= 1000;
        stats.record(&after, 106, AGE);
        assert_eq!(stats.read(106, AGE).busy_ns, before);
        // An epoch boundary keeps the total.
        stats.invalidate();
        assert_eq!(stats.read(107, AGE).busy_ns, before);
    }

    #[test]
    fn a18_pro_state_reports_give_the_pstate() {
        let mut stats = Telemetry::new(Protocol::A18_PRO);
        let mut interval = M5_INTERVAL;
        let mut state = M5_STATE;
        (state[1], state[3]) = (10, 10);
        stats.record(&interval, 100, AGE);
        stats.record(&state, 101, AGE);
        let view = stats.read(101, AGE);
        assert_eq!((view.pstate_status, view.pstate), (VALID, 10));
        assert_eq!(view.pstate_timestamp, 0x59_3c04_2f8f);
        // Its table ends at state 10.
        (state[1], state[3]) = (11, 11);
        stats.record(&state, 102, AGE);
        assert_eq!(stats.read(102, AGE).pstate_status, UNAVAILABLE);
        // Tag 0x10 is not this firmware's state report.
        interval[1] += 1;
        stats.record(&interval, 103, AGE);
        stats.record(&P1, 103, AGE);
        assert_eq!(stats.read(103, AGE).pstate_status, UNAVAILABLE);
        (state[1], state[3]) = (1, 1);
        stats.record(&state, 104, AGE);
        assert_eq!(stats.read(104, AGE).pstate, 1);
        // Gated cores are off.
        stats.gate();
        let view = stats.read(105, AGE);
        assert_eq!((view.pstate_status, view.pstate), (VALID, 0));
    }

    #[test]
    fn a18_pro_busy_time_integrates_16_ms_windows() {
        let mut stats = Telemetry::new(Protocol::A18_PRO);
        let mut words = M5_U[0];
        stats.record(&words, 100, AGE);
        // One full 16 ms window at 100%, then one at 25%.
        words[1] += 384_000;
        stats.record(&words, 101, AGE);
        assert_eq!(stats.read(101, AGE).busy_ns, 16_000_000);
        words[1] += 384_000;
        words[6] = 25;
        stats.record(&words, 102, AGE);
        assert_eq!(stats.read(102, AGE).busy_ns, 20_000_000);
        // Up to two periods is one window; longer is a gap the GPU spent gated.
        words[1] += 768_000;
        stats.record(&words, 103, AGE);
        assert_eq!(stats.read(103, AGE).busy_ns, 28_000_000);
        words[1] += 768_001;
        stats.record(&words, 104, AGE);
        assert_eq!(stats.read(104, AGE).busy_ns, 28_000_000);
    }

    #[test]
    fn other_firmwares_ignore_the_m5_records() {
        let mut stats = Telemetry::default();
        for words in [M5_INTERVAL, M5_STATE, M5_U[0], M5_U[1]] {
            stats.record(&words, 1, AGE);
        }
        let view = stats.read(1, AGE);
        assert_eq!(
            (view.pstate_status, view.busy_status, view.busy_ns),
            (UNAVAILABLE, UNSUPPORTED, 0)
        );
    }
}
