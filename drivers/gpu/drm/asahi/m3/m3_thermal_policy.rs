// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Decisions of the M3 thermal limit (see `m3_thermal`), kept free of kernel dependencies so
//! that host tests drive exactly this code (`tools/asahi/m3-runtime-tests.rs`).
//!
//! Times are monotonic nanoseconds from any fixed origin, temperatures millidegrees Celsius, and
//! states indices of the published performance-state table.

/// Highest fixed cap for non-T8122 device-tree InitData, whose firmware die-temperature
/// controller is disabled. T8122 retains its lower default cap without temperature feedback.
const ADT_MAX_PSTATE_LIMIT: u32 = 5;
/// T8122's unmonitored cap, independent of any experiment ceiling or future default change.
const T8122_SAFE_PSTATE: u32 = 2;

/// How the runtime backend limits the GPU performance state by temperature.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub(crate) enum ThermalMode {
    /// A fixed ceiling no higher than the safe cap, without temperature feedback.
    Off,
    /// Publish the requested ceiling, but hold the runtime cap at the safe cap.
    Hold,
    /// Allow the runtime cap to follow the die temperature up to the requested ceiling.
    On,
}

impl ThermalMode {
    /// Return (firmware ceiling, boot/failure cap) for a table-clamped request. A T8122
    /// experiment ceiling permits higher states only with working temperature feedback;
    /// it must not also raise the cap used before a reading or after a sensor failure.
    pub(crate) fn pstate_caps(self, requested: u32, t8122_ceiling: Option<u32>) -> (u32, u32) {
        let (max, limit) = match t8122_ceiling {
            Some(ceiling) => (requested.min(ceiling), T8122_SAFE_PSTATE),
            None => (requested, ADT_MAX_PSTATE_LIMIT),
        };
        let safe = max.min(limit);
        (if self == Self::Off { safe } else { max }, safe)
    }
}

/// Required thermal-zone bits: SMC die (bit 0), PMP hotspot (bit 1).
/// A missing or malformed compatible list requires both zones.
pub(crate) fn required_zones(compatible: &[u8]) -> u32 {
    if compatible.is_empty() || compatible.last() != Some(&0) {
        return 3;
    }
    let has = |name: &[u8]| compatible.split(|b| *b == 0).any(|s| s == name);
    if has(b"apple,j514s") || has(b"apple,j516s") {
        3
    } else if has(b"apple,t8122") || has(b"apple,j613") || has(b"apple,j615") {
        1
    } else {
        0
    }
}

/// Nanoseconds per millisecond and per second.
pub(crate) const MS: i64 = 1_000_000;
pub(crate) const S: i64 = 1_000 * MS;

/// Readings older than this are not used.
pub(crate) const STALE_NS: i64 = 3 * S;
/// A reading that has not changed at all for this long is not used.
pub(crate) const STUCK_NS: i64 = 60 * S;
/// Plausible SoC die temperatures.
pub(crate) const PLAUSIBLE_MIN_MC: i32 = 10_000;
pub(crate) const PLAUSIBLE_MAX_MC: i32 = 125_000;
/// Minimum time between two one-state steps down and up.
pub(crate) const DOWN_PACE_NS: i64 = 500 * MS;
pub(crate) const UP_PACE_NS: i64 = 2 * S;
/// How long the previous cap is tolerated after a lowering.
pub(crate) const GRACE_NS: i64 = S;
/// A state above the runtime cap in this many consecutive checks spanning this long means the
/// firmware does not follow the runtime cap.
pub(crate) const IGNORED_CHECKS: u32 = 20;
pub(crate) const IGNORED_NS: i64 = 3 * S;
/// Distance of `cool` below and `critical` above `hot`.
pub(crate) const COOL_BELOW_MC: i32 = 10_000;
pub(crate) const CRITICAL_ABOVE_MC: i32 = 8_000;

/// The latest sensor reading.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) enum Reading {
    /// Not read yet.
    None,
    /// The hottest temperature of the zones read (bit `i` of `zones` for zone `i`), taken at `at`.
    Temp { mc: i32, at: i64, zones: u32 },
    /// The read failed with this error number at `at`.
    Failed { errno: i32, at: i64 },
}

/// Whether a reading is in use (`Ok`, with its zones), or why not.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) enum Source {
    Unknown,
    Ok(u32),
    NoReading,
    Failed(i32),
    Stale,
    Implausible,
    Stuck,
}

/// Why the runtime cap changed.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) enum Why {
    /// At or above `critical`: the safe cap at once.
    Critical,
    /// At or above `hot`: one state lower.
    Hot,
    /// At or below `cool`: one state higher.
    Cool,
    /// No usable reading: the safe cap at once.
    NoTemperature(Source),
}

/// A change of the runtime cap.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) struct Change {
    pub(crate) from: u32,
    pub(crate) to: u32,
    pub(crate) why: Why,
}

/// The runtime cap and the rules that move it.
pub(crate) struct Policy {
    /// Whether the cap follows the temperature (`on`); otherwise it stays at `safe` (`hold`).
    adjust: bool,
    safe: u32,
    ceiling: u32,
    hot: i32,
    cool: i32,
    critical: i32,
    cap: u32,
    /// The last usable temperature and since when it has not changed.
    same: Option<(i32, i64)>,
    /// When the last reading acted upon was taken.
    used: Option<i64>,
    last_change: Option<i64>,
    /// When the cap was last lowered, and the highest cap of the grace period before it.
    lowered: Option<(i64, u32)>,
    /// Since when, and in how many consecutive checks, the firmware reported a state above the
    /// allowed cap.
    over: Option<(i64, u32)>,
    raised: u32,
    lowered_count: u32,
}

impl Policy {
    /// A policy with the runtime cap at `safe`, never above `ceiling` (at least `safe`), and
    /// `hot` in millidegrees Celsius.
    pub(crate) fn new(adjust: bool, safe: u32, ceiling: u32, hot: i32) -> Self {
        Policy {
            adjust,
            safe,
            ceiling: ceiling.max(safe),
            hot,
            cool: hot - COOL_BELOW_MC,
            critical: hot + CRITICAL_ABOVE_MC,
            cap: safe,
            same: None,
            used: None,
            last_change: None,
            lowered: None,
            over: None,
            raised: 0,
            lowered_count: 0,
        }
    }

    pub(crate) fn cap(&self) -> u32 {
        self.cap
    }
    pub(crate) fn safe(&self) -> u32 {
        self.safe
    }
    pub(crate) fn ceiling(&self) -> u32 {
        self.ceiling
    }
    pub(crate) fn thresholds(&self) -> (i32, i32, i32) {
        (self.cool, self.hot, self.critical)
    }
    pub(crate) fn counts(&self) -> (u32, u32) {
        (self.raised, self.lowered_count)
    }

    /// The temperature of `reading` if it may be used at `now` (with when it was taken and its
    /// zones), or why not.
    pub(crate) fn usable(&mut self, now: i64, reading: Reading) -> Result<(i32, i64, u32), Source> {
        let (mc, at, zones) = match reading {
            Reading::Temp { mc, at, zones } if now - at <= STALE_NS => (mc, at, zones),
            Reading::Failed { errno, at } if now - at <= STALE_NS => {
                return Err(Source::Failed(errno))
            }
            Reading::Temp { .. } | Reading::Failed { .. } => return Err(Source::Stale),
            Reading::None => return Err(Source::NoReading),
        };
        if !(PLAUSIBLE_MIN_MC..=PLAUSIBLE_MAX_MC).contains(&mc) {
            return Err(Source::Implausible);
        }
        match self.same {
            Some((value, since)) if value == mc => {
                if now - since >= STUCK_NS {
                    return Err(Source::Stuck);
                }
            }
            _ => self.same = Some((mc, now)),
        }
        Ok((mc, at, zones))
    }

    /// Recompute the runtime cap after a job at `now` from `reading`. Returns the usable
    /// temperature (or why there is none) and the change of the cap, if any.
    pub(crate) fn update(
        &mut self,
        now: i64,
        reading: Reading,
    ) -> (Result<(i32, i64, u32), Source>, Option<Change>) {
        let usable = self.usable(now, reading);
        if !self.adjust {
            return (usable, None);
        }
        let last_change = self.last_change;
        let paced = |pace: i64| last_change.is_none_or(|t| now - t >= pace);
        let (to, why) = match usable {
            Err(source) => (self.safe, Why::NoTemperature(source)),
            Ok((mc, at, _)) => {
                let fresh = self.used.is_none_or(|used| at > used);
                if mc >= self.critical {
                    (self.safe, Why::Critical)
                } else if mc >= self.hot && fresh && paced(DOWN_PACE_NS) {
                    (self.cap.saturating_sub(1).max(self.safe), Why::Hot)
                } else if mc <= self.cool && fresh && paced(UP_PACE_NS) {
                    (self.cap.saturating_add(1).min(self.ceiling), Why::Cool)
                } else {
                    (self.cap, Why::Cool)
                }
            }
        };
        if to == self.cap {
            return (usable, None);
        }
        if let Ok((_, at, _)) = usable {
            self.used = Some(at);
        }
        if to < self.cap {
            let previous = match self.lowered {
                Some((t, old)) if now - t < GRACE_NS => old.max(self.cap),
                _ => self.cap,
            };
            self.lowered = Some((now, previous));
            self.lowered_count = self.lowered_count.saturating_add(1);
        } else {
            self.raised = self.raised.saturating_add(1);
        }
        let change = Change {
            from: self.cap,
            to,
            why,
        };
        self.cap = to;
        self.last_change = Some(now);
        (usable, Some(change))
    }

    /// The highest state the firmware may report at `now`: the runtime cap, or the highest cap
    /// of the grace period after a lowering.
    pub(crate) fn allowed(&self, now: i64) -> u32 {
        match self.lowered {
            Some((t, old)) if now - t < GRACE_NS => old.max(self.cap),
            _ => self.cap,
        }
    }

    /// Account one check of the reported state at `now`; `over` says whether it stayed above
    /// [`Self::allowed`]. Returns the number of consecutive checks and their span when they
    /// show that the firmware does not follow the runtime cap.
    pub(crate) fn note_check(&mut self, now: i64, over: bool) -> Option<(u32, i64)> {
        if !over {
            self.over = None;
            return None;
        }
        let (since, count) = match self.over {
            Some((since, count)) => (since, count.saturating_add(1)),
            None => (now, 1),
        };
        self.over = Some((since, count));
        (count >= IGNORED_CHECKS && now - since >= IGNORED_NS).then_some((count, now - since))
    }
}

#[cfg(test)]
mod cap_tests {
    use super::*;

    fn temp(mc: i32, at: i64) -> Reading {
        Reading::Temp { mc, at, zones: 1 }
    }

    fn boosted_air() -> Policy {
        let (ceiling, safe) = ThermalMode::On.pstate_caps(8, Some(5));
        let mut policy = Policy::new(true, safe, ceiling, 80_000);
        assert_eq!(policy.cap(), 2);
        for (at, mc, cap) in [(0, 60_000, 3), (2 * S, 60_001, 4), (4 * S, 60_002, 5)] {
            policy.update(at, temp(mc, at)).0.unwrap();
            assert_eq!(policy.cap(), cap);
        }
        policy
    }

    #[test]
    fn air_ceiling_never_raises_the_unmonitored_cap() {
        assert_eq!(ThermalMode::On.pstate_caps(8, Some(5)), (5, 2));
        assert_eq!(ThermalMode::Hold.pstate_caps(8, Some(5)), (5, 2));
        assert_eq!(ThermalMode::Off.pstate_caps(8, Some(5)), (2, 2));
        for mode in [ThermalMode::On, ThermalMode::Hold, ThermalMode::Off] {
            assert_eq!(mode.pstate_caps(1, Some(5)), (1, 1));
            assert_eq!(mode.pstate_caps(8, Some(1)), (1, 1));
        }
    }

    #[test]
    fn non_air_caps_keep_the_existing_safe_limit() {
        assert_eq!(ThermalMode::On.pstate_caps(8, None), (8, 5));
        assert_eq!(ThermalMode::Hold.pstate_caps(8, None), (8, 5));
        assert_eq!(ThermalMode::Off.pstate_caps(8, None), (5, 5));
    }

    #[test]
    fn failed_missing_stale_and_implausible_readings_drop_air_to_two() {
        for (reading, source) in [
            (Reading::None, Source::NoReading),
            (Reading::Failed { errno: -5, at: 5 * S }, Source::Failed(-5)),
            (temp(60_000, S), Source::Stale),
            (temp(9_999, 5 * S), Source::Implausible),
            (temp(125_001, 5 * S), Source::Implausible),
        ] {
            let mut policy = boosted_air();
            let (usable, change) = policy.update(5 * S, reading);
            assert_eq!(usable, Err(source));
            assert_eq!(change, Some(Change { from: 5, to: 2, why: Why::NoTemperature(source) }));
            policy.update(7 * S, temp(60_003, 7 * S)).0.unwrap();
            assert_eq!(policy.cap(), 3);
        }
    }

    #[test]
    fn air_boost_requires_fresh_paced_readings_and_respects_the_ceiling() {
        let (ceiling, safe) = ThermalMode::On.pstate_caps(8, Some(5));
        let mut policy = Policy::new(true, safe, ceiling, 80_000);
        policy.update(0, temp(60_000, 0)).0.unwrap();
        assert_eq!(policy.cap(), 3);
        policy.update(S, temp(60_001, S)).0.unwrap();
        assert_eq!(policy.cap(), 3);
        policy.update(2 * S, temp(60_000, 0)).0.unwrap();
        assert_eq!(policy.cap(), 3);
        policy.update(2 * S, temp(60_002, 2 * S)).0.unwrap();
        assert_eq!(policy.cap(), 4);
        policy.update(4 * S, temp(60_003, 4 * S)).0.unwrap();
        policy.update(6 * S, temp(60_004, 6 * S)).0.unwrap();
        assert_eq!(policy.cap(), 5);
    }

    #[test]
    fn air_hot_and_critical_readings_reduce_the_cap() {
        let mut policy = boosted_air();
        policy.update(4 * S + 500 * MS, temp(80_000, 4 * S + 500 * MS)).0.unwrap();
        assert_eq!(policy.cap(), 4);
        policy.update(5 * S, temp(80_000, 4 * S + 500 * MS)).0.unwrap();
        assert_eq!(policy.cap(), 4);
        policy.update(5 * S, temp(80_001, 5 * S)).0.unwrap();
        assert_eq!(policy.cap(), 3);
        policy.update(5 * S + MS, temp(88_000, 5 * S + MS)).0.unwrap();
        assert_eq!(policy.cap(), 2);
        assert_eq!(policy.allowed(5 * S + 2 * MS), 5);
        assert_eq!(policy.allowed(6 * S + MS), 2);
    }

    #[test]
    fn unchanged_air_temperature_cannot_sustain_boost_forever() {
        let (ceiling, safe) = ThermalMode::On.pstate_caps(8, Some(5));
        let mut policy = Policy::new(true, safe, ceiling, 80_000);
        for at in [0, 2 * S, 4 * S] {
            policy.update(at, temp(60_000, at)).0.unwrap();
        }
        assert_eq!(policy.cap(), 5);
        let (usable, _) = policy.update(STUCK_NS, temp(60_000, STUCK_NS));
        assert_eq!(usable, Err(Source::Stuck));
        assert_eq!(policy.cap(), 2);
    }

    #[test]
    fn hold_mode_never_boosts_the_air() {
        let (ceiling, safe) = ThermalMode::Hold.pstate_caps(8, Some(5));
        let mut policy = Policy::new(false, safe, ceiling, 80_000);
        for at in [0, 2 * S, 4 * S] {
            policy.update(at, temp(60_000, at)).0.unwrap();
        }
        assert_eq!(policy.cap(), 2);
    }
}

#[cfg(test)]
mod zone_tests {
    use super::required_zones;

    #[test]
    fn every_t8122_board_requires_the_die_zone() {
        for board in ["j613", "j615", "j504", "j433", "j434"] {
            let compatible = format!("apple,{board}\0apple,t8122\0apple,arm-platform\0");
            assert_eq!(required_zones(compatible.as_bytes()), 1);
        }
        assert_eq!(required_zones(b"apple,j613\0"), 1);
        assert_eq!(required_zones(b"apple,j615\0"), 1);
        assert_eq!(required_zones(b"apple,j504\0apple,t8122\0"), 1);
    }

    #[test]
    fn pro_and_malformed_lists_keep_both_zone_requirement() {
        assert_eq!(required_zones(b"apple,j514s\0apple,t6030\0"), 3);
        assert_eq!(required_zones(b"apple,j516s\0apple,t6030\0"), 3);
        assert_eq!(required_zones(b""), 3);
        assert_eq!(required_zones(b"apple,t8122"), 3);
        assert_eq!(required_zones(b"apple,j274\0apple,t8103\0"), 0);
        assert_eq!(required_zones(b"apple,t81220\0"), 0);
    }
}

#[cfg(test)]
mod range_tests {
    use super::*;

    /// `hot` of the tests: the default of `asahi.m3_thermal_hot`. Cool is 70 C, critical 88 C.
    const HOT: i32 = 80_000;

    fn temp(mc: i32, at: i64) -> Reading {
        Reading::Temp { mc, at, zones: 1 }
    }

    /// A policy that has climbed from `safe` to `ceiling` on cool readings, the last at
    /// `2 * (ceiling - safe - 1)` s; returns it with the time of that last reading.
    fn raised(safe: u32, ceiling: u32) -> (Policy, i64) {
        let mut p = Policy::new(true, safe, ceiling, HOT);
        let mut now = 0;
        for step in 0..ceiling - safe {
            now = i64::from(step) * UP_PACE_NS;
            let (usable, change) = p.update(now, temp(50_000 + step as i32, now));
            assert!(usable.is_ok());
            assert_eq!(change.map(|c| (c.from, c.to, c.why)), Some((safe + step, safe + step + 1, Why::Cool)));
        }
        assert_eq!(p.cap(), ceiling);
        (p, now)
    }

    #[test]
    fn the_cap_starts_at_the_safe_cap_and_climbs_one_state_per_two_seconds_to_the_ceiling() {
        for ceiling in 6..=8 {
            let mut p = Policy::new(true, 5, ceiling, HOT);
            assert_eq!((p.cap(), p.safe(), p.ceiling()), (5, 5, ceiling));
            assert_eq!(p.thresholds(), (70_000, 80_000, 88_000));
            // No reading yet: the safe cap.
            assert_eq!(p.update(0, Reading::None), (Err(Source::NoReading), None));
            assert_eq!(p.cap(), 5);
            // A cool reading every 500 ms: one state every 2 s, never above the ceiling.
            let mut caps = [0u32; 21];
            for (i, slot) in caps.iter_mut().enumerate() {
                let now = i as i64 * 500 * MS;
                let _ = p.update(now, temp(60_000 + i as i32, now));
                *slot = p.cap();
                assert!(p.cap() <= ceiling && p.cap() >= 5);
            }
            // t = 0, 2, 4 s: 6, 7, 8 (as far as the ceiling allows).
            assert_eq!(caps[0], 6);
            assert_eq!(caps[3], 6);
            assert_eq!(caps[4], 7.min(ceiling));
            assert_eq!(caps[7], 7.min(ceiling));
            assert_eq!(caps[8], 8.min(ceiling));
            assert_eq!(caps[20], ceiling);
            assert_eq!(p.counts(), (ceiling - 5, 0));
            // Exactly cool (70.0 C) still raises; anything above it does not.
            let (mut p, t) = raised(5, ceiling);
            let _ = p.update(t + 500 * MS, temp(HOT, t + 500 * MS));
            assert_eq!(p.cap(), ceiling - 1);
            let _ = p.update(t + 3 * S, temp(70_001, t + 3 * S));
            assert_eq!(p.cap(), ceiling - 1);
            let _ = p.update(t + 4 * S, temp(70_000, t + 4 * S));
            assert_eq!(p.cap(), ceiling);
        }
    }

    #[test]
    fn between_cool_and_hot_the_cap_holds() {
        for ceiling in 6..=8 {
            let (mut p, t) = raised(5, ceiling);
            for (i, mc) in [70_001, 75_000, 79_999].into_iter().enumerate() {
                let now = t + (i as i64 + 1) * 3 * S;
                assert_eq!(p.update(now, temp(mc, now)).1, None);
                assert_eq!(p.cap(), ceiling);
            }
        }
    }

    #[test]
    fn hot_lowers_one_state_per_fresh_reading_every_500_ms_and_stops_at_the_safe_cap() {
        for ceiling in 6..=8 {
            let (mut p, t) = raised(5, ceiling);
            // Less than 500 ms after the last change: not yet.
            let now = t + 499 * MS;
            assert_eq!(p.update(now, temp(HOT, now)).1, None);
            let mut now = t + 500 * MS;
            for expected in (5..ceiling).rev() {
                let change = p.update(now, temp(HOT + (now / MS) as i32 % 7, now)).1.unwrap();
                assert_eq!((change.to, change.why), (expected, Why::Hot));
                // The same reading again, however much later, is not acted on twice.
                assert_eq!(p.update(now + 400 * MS, temp(HOT, now)).1, None);
                now += 500 * MS;
            }
            assert_eq!(p.cap(), 5);
            // Hot at the safe cap: nothing lower to go to.
            for i in 0..4 {
                let at = now + i * 500 * MS;
                assert_eq!(p.update(at, temp(HOT + 1_000 + i as i32, at)).1, None);
                assert_eq!(p.cap(), 5);
            }
            assert_eq!(p.counts(), (ceiling - 5, ceiling - 5));
        }
    }

    #[test]
    fn critical_and_unusable_readings_restore_the_safe_cap_at_once() {
        for ceiling in 6..=8 {
            let (_, t) = raised(5, ceiling);
            // Right after the last raise, so no pacing would allow a step yet.
            let now = t + MS;
            let cases = [
                (temp(88_000, now), Why::Critical),
                (temp(110_000, now), Why::Critical),
                (Reading::None, Why::NoTemperature(Source::NoReading)),
                (Reading::Failed { errno: -19, at: now }, Why::NoTemperature(Source::Failed(-19))),
                (Reading::Failed { errno: -5, at: now }, Why::NoTemperature(Source::Failed(-5))),
                // Older than 3 s, as a temperature and as a failure.
                (temp(50_000, now - STALE_NS - 1), Why::NoTemperature(Source::Stale)),
                (Reading::Failed { errno: -5, at: now - STALE_NS - 1 }, Why::NoTemperature(Source::Stale)),
                // Outside 10..=125 C. The SMC's reading of a powered-down sensor is one of them.
                (temp(9_999, now), Why::NoTemperature(Source::Implausible)),
                (temp(-2_849, now), Why::NoTemperature(Source::Implausible)),
                (temp(125_001, now), Why::NoTemperature(Source::Implausible)),
            ];
            for (reading, why) in cases {
                let (mut p, _) = raised(5, ceiling);
                let change = p.update(now, reading).1.unwrap();
                assert_eq!((change.from, change.to, change.why), (ceiling, 5, why), "{reading:?}");
                // The previous cap is tolerated for the grace period only.
                assert_eq!(p.allowed(now + GRACE_NS - 1), ceiling);
                assert_eq!(p.allowed(now + GRACE_NS), 5);
            }
            // A reading at exactly 3 s is still used.
            let (mut p, _) = raised(5, ceiling);
            assert_eq!(p.update(now, temp(60_000, now - STALE_NS)).1, None);
            assert_eq!(p.cap(), ceiling);
            // A reading that has not changed for 60 s is stuck, even while fresh.
            let (mut p, t) = raised(5, ceiling);
            let since = t + S;
            assert_eq!(p.update(since, temp(61_234, since)).1, None);
            assert_eq!(p.update(since + STUCK_NS - 1, temp(61_234, since + STUCK_NS - 1)).1, None);
            assert_eq!(p.cap(), ceiling);
            let change = p.update(since + STUCK_NS, temp(61_234, since + STUCK_NS)).1.unwrap();
            assert_eq!((change.to, change.why), (5, Why::NoTemperature(Source::Stuck)));
        }
    }

    #[test]
    fn after_a_fallback_the_cap_climbs_again_one_state_per_two_seconds() {
        for ceiling in 6..=8 {
            let (mut p, t) = raised(5, ceiling);
            let _ = p.update(t + S, Reading::Failed { errno: -19, at: t + S });
            assert_eq!(p.cap(), 5);
            // A usable reading right away does not raise before 2 s have passed since the fall.
            let _ = p.update(t + 2 * S, temp(55_000, t + 2 * S));
            assert_eq!(p.cap(), 5);
            for step in 0..ceiling - 5 {
                let now = t + 3 * S + i64::from(step) * UP_PACE_NS;
                let _ = p.update(now, temp(55_001 + step as i32, now));
                assert_eq!(p.cap(), 6 + step);
            }
            assert_eq!(p.cap(), ceiling);
        }
    }

    #[test]
    fn hold_never_leaves_the_safe_cap() {
        for ceiling in 6..=8 {
            let mut p = Policy::new(false, 5, ceiling, HOT);
            for i in 0..40 {
                let now = i * S;
                let reading = match i % 4 {
                    0 => temp(40_000 + i as i32, now),
                    1 => temp(85_000 + i as i32, now),
                    2 => temp(95_000 + i as i32, now),
                    _ => Reading::Failed { errno: -19, at: now },
                };
                assert_eq!(p.update(now, reading).1, None);
                assert_eq!((p.cap(), p.allowed(now)), (5, 5));
            }
            assert_eq!(p.ceiling(), ceiling);
            assert_eq!(p.counts(), (0, 0));
        }
    }

    #[test]
    fn a_ceiling_at_the_safe_cap_never_moves() {
        // Today's boots: asahi.t8122_pstate_cap of 1 to 5 gives safe == ceiling.
        for cap in 1..=5 {
            let mut p = Policy::new(true, cap, cap, HOT);
            for i in 0..40 {
                let now = i * S;
                let reading = match i % 5 {
                    0 => temp(40_000 + i as i32, now),
                    1 => temp(81_000 + i as i32, now),
                    2 => temp(99_000 + i as i32, now),
                    3 => Reading::None,
                    _ => Reading::Failed { errno: -19, at: now },
                };
                assert_eq!(p.update(now, reading).1, None);
                assert_eq!((p.cap(), p.allowed(now)), (cap, cap));
            }
            assert_eq!(p.counts(), (0, 0));
        }
        // A ceiling below the safe cap is raised to it, not the other way round.
        let p = Policy::new(true, 5, 3, HOT);
        assert_eq!((p.safe(), p.ceiling(), p.cap()), (5, 5, 5));
    }

    #[test]
    fn a_firmware_that_ignores_a_lowered_cap_is_found_after_twenty_checks_over_three_seconds() {
        let (mut p, t) = raised(5, 8);
        let fell = t + S;
        let _ = p.update(fell, Reading::None);
        assert_eq!((p.cap(), p.allowed(fell)), (5, 8));
        // Within the grace period a state of 8 is not "over".
        assert!(p.note_check(fell + 500 * MS, false).is_none());
        // Afterwards: 20 checks in less than 3 s are not enough; the next one 3 s after the
        // first is.
        let start = fell + GRACE_NS;
        for i in 0..19 {
            assert!(p.note_check(start + i * 100 * MS, true).is_none());
        }
        assert!(p.note_check(start + IGNORED_NS - 1, true).is_none());
        assert_eq!(p.note_check(start + IGNORED_NS, true), Some((21, IGNORED_NS)));
        // One check within the cap starts the count again.
        let again = start + IGNORED_NS + MS;
        assert!(p.note_check(again, false).is_none());
        // 19 checks are not enough however long they span; the 20th then is.
        for i in 0..19 {
            assert!(p.note_check(again + (i + 1) * S, true).is_none());
        }
        assert_eq!(p.note_check(again + 20 * S, true), Some((20, 19 * S)));
    }
}
