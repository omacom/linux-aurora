// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Decisions of the M3 thermal limit (see `m3_thermal`), kept free of kernel dependencies so
//! that host tests drive exactly this code (`tools/asahi/m3-thermal-policy-tests.rs`).
//!
//! Times are monotonic nanoseconds from any fixed origin, temperatures millidegrees Celsius, and
//! states indices of the published performance-state table.

/// Thermal fallback is independent of the experimental T8122 ceiling.
/// Keep other SoCs' existing fallback and never exceed a lower requested cap.
pub(crate) fn safe_cap(max: u32, t8122: bool, legacy_limit: u32) -> u32 {
    max.min(if t8122 {
        crate::t8122_knobs::PSTATE_CAP
    } else {
        legacy_limit
    })
}

/// Required thermal-zone bits: SMC die (bit 0), PMP hotspot (bit 1).
/// A missing or malformed compatible list requires both zones. The M3 Max MacBook Pros (J514C,
/// J516C) require the SMC die zone: their PMP does not run, so it has no hotspot zone.
pub(crate) fn required_zones(compatible: &[u8]) -> u32 {
    if compatible.is_empty() || compatible.last() != Some(&0) {
        return 3;
    }
    let has = |name: &[u8]| compatible.split(|b| *b == 0).any(|s| s == name);
    if has(b"apple,j514s") || has(b"apple,j516s") {
        3
    } else if has(b"apple,t8122") || has(b"apple,j613") || has(b"apple,j615") {
        1
    } else if has(b"apple,j514c") || has(b"apple,j516c") {
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
    }

    #[test]
    fn m3_max_boards_require_the_die_zone() {
        for board in ["j514c", "j516c"] {
            let compatible = format!("apple,{board}\0apple,t6031\0apple,arm-platform\0");
            assert_eq!(required_zones(compatible.as_bytes()), 1);
        }
        // The 14-core M3 Max (T6034) boards have no entry yet.
        assert_eq!(required_zones(b"apple,j516m\0apple,t6034\0apple,arm-platform\0"), 0);
        assert_eq!(required_zones(b"apple,j516c"), 3);
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
mod fallback_tests {
    use super::*;

    fn temp(mc: i32, at: i64) -> Reading {
        Reading::Temp { mc, at, zones: 1 }
    }

    #[test]
    fn t8122_higher_ceilings_keep_state_two_fallback() {
        for ceiling in 1..=5 {
            assert_eq!(safe_cap(ceiling, true, 5), ceiling.min(2));
            assert_eq!(safe_cap(ceiling, false, 5), ceiling);
        }
        assert_eq!(safe_cap(8, false, 5), 5);
    }

    #[test]
    fn next_state_returns_to_two_when_hot_or_reading_fails() {
        for reading in [
            temp(80_000, 3 * S),
            temp(88_000, 3 * S),
            Reading::None,
            Reading::Failed { errno: -19, at: 3 * S },
        ] {
            let mut p = Policy::new(true, safe_cap(3, true, 5), 3, 80_000);
            assert_eq!(p.cap(), 2);
            let _ = p.update(0, temp(50_000, 0));
            assert_eq!(p.cap(), 3);
            let _ = p.update(3 * S, reading);
            assert_eq!(p.cap(), 2);
        }
    }

    #[test]
    fn stale_or_stuck_sensor_returns_to_two() {
        let mut p = Policy::new(true, safe_cap(3, true, 5), 3, 80_000);
        let _ = p.update(0, temp(50_000, 0));
        assert_eq!(p.cap(), 3);
        let _ = p.update(4 * S, temp(50_000, 0));
        assert_eq!(p.cap(), 2);
        let _ = p.update(6 * S, temp(51_000, 6 * S));
        assert_eq!(p.cap(), 3);
        let _ = p.update(67 * S, temp(51_000, 67 * S));
        assert_eq!(p.cap(), 2);
    }

    #[test]
    fn hold_never_raises_t8122_above_two() {
        let mut p = Policy::new(false, safe_cap(3, true, 5), 3, 80_000);
        let _ = p.update(0, temp(50_000, 0));
        let _ = p.update(3 * S, temp(50_000, 3 * S));
        assert_eq!(p.cap(), 2);
    }
}
