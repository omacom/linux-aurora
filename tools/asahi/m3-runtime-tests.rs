// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Host tests for the production M3 admission, IO ownership, thermal policy, the T8122 start
//! parameters and the T8122 HwDataA +0x418c block.
//! Run: rustc --edition=2021 --test tools/asahi/m3-runtime-tests.rs -o /tmp/m3-runtime-tests
#![allow(dead_code)]

#[path = "../../drivers/gpu/drm/asahi/m3/t8122_admission.rs"]
mod t8122_admission;
#[path = "../../drivers/gpu/drm/asahi/m3/m3_init_layout.rs"]
mod m3_init_layout;
#[path = "../../drivers/gpu/drm/asahi/m3/m3_init_storage.rs"]
mod m3_init_storage;
#[path = "../../drivers/gpu/drm/asahi/m3/m3_thermal_policy.rs"]
mod m3_thermal_policy;
#[path = "../../drivers/gpu/drm/asahi/m3/t8122_hwdata.rs"]
mod t8122_hwdata;

// `t8122_knobs` takes its default InitData version from `m3_firmware`, which needs the kernel.
// Only that constant is stood in for (`defaults_are_the_plan_baseline` checks its value); the
// parameter parsing and validation are the production module.
mod m3 {
    pub mod m3_firmware {
        pub const G15_V14_8_3_INITDATA: u64 = 0x0c08_e21e_8380_0490;
    }
}
#[path = "../../drivers/gpu/drm/asahi/m3/t8122_knobs.rs"]
mod t8122_knobs;

/// `asahi.t8122_pstate_cap` through the thermal limit: what the parameter resolves to, what
/// `pstate_cap_use` makes of it with each `asahi.m3_thermal` mode, and what the governor's
/// decisions then let the firmware use.
#[cfg(test)]
mod t8122_pstate_cap {
    use super::m3_thermal_policy::{Policy, Reading, ThermalMode, S};
    use super::t8122_knobs::{self as knobs, CapUse, Feedback, Raw, Refusal};

    const MODES: [Feedback; 3] = [Feedback::Off, Feedback::Hold, Feedback::On];

    fn cap_use(cap: u64, feedback: Feedback) -> Result<CapUse, Refusal> {
        let raw = Raw { start: 1, pstate_cap: cap, ..Raw::NONE };
        Ok(knobs::pstate_cap_use(knobs::resolve(&raw)?.pstate_cap, feedback))
    }

    /// The governor of `m3_thermal` for `u` (`Governor::new`): it follows the temperature
    /// only with `On`.
    fn policy(u: CapUse, feedback: Feedback) -> Policy {
        Policy::new(feedback == Feedback::On, u.fallback, u.ceiling, 80_000)
    }

    /// The highest runtime cap in 30 s of cool readings from `from` on, one every 500 ms.
    fn highest_when_cool(p: &mut Policy, from: i64) -> u32 {
        let mut highest = p.cap();
        for i in 0..60 {
            let now = from + i * S / 2;
            let _ = p.update(now, Reading::Temp { mc: 45_000 + i as i32, at: now, zones: 1 });
            highest = highest.max(p.cap());
            assert!(p.allowed(now) <= p.ceiling());
        }
        highest
    }

    #[test]
    fn states_above_five_are_reached_only_with_the_thermal_limit_on_and_a_usable_reading() {
        assert_eq!(knobs::PSTATE_CAP_MAX, 8);
        for cap in 1..=8u64 {
            for feedback in MODES {
                let u = cap_use(cap, feedback).unwrap();
                // Without the thermal limit on the firmware never gets a state above 5.
                if feedback != Feedback::On {
                    assert!(u.ceiling <= knobs::PSTATE_CAP_NO_FEEDBACK_MAX, "cap {cap}");
                }
                let mut p = policy(u, feedback);
                // Before any reading the runtime cap is the fallback, at most 2.
                assert_eq!(p.cap(), u.fallback);
                assert!(u.fallback <= knobs::PSTATE_FALLBACK);
                assert_eq!(p.update(0, Reading::None).1, None);
                assert_eq!(p.cap(), u.fallback);
                // Without a usable reading (here: no zone) the cap never rises.
                for i in 1..20 {
                    let now = i * S / 2;
                    let _ = p.update(now, Reading::Failed { errno: -19, at: now });
                    assert_eq!(p.cap(), u.fallback);
                }
                let highest = highest_when_cool(&mut p, 10 * S);
                let expected = if feedback == Feedback::On { cap as u32 } else { (cap as u32).min(2) };
                assert_eq!(highest, expected, "cap {cap} {feedback:?}");
                assert_eq!(u.ranges, highest > u.fallback, "cap {cap} {feedback:?}");
            }
        }
    }

    #[test]
    fn caps_six_to_eight_without_the_thermal_limit_on_still_start_the_gpu() {
        for cap in 6..=8u64 {
            // A cap left on the command line from a ranging boot must not cost the GPU when the
            // thermal limit is switched off or held: it is lowered to 5, and the boot starts at 2.
            let u = cap_use(cap, Feedback::Off).unwrap();
            assert_eq!(u, CapUse { ceiling: 2, fallback: 2, ranges: false });
            let mut p = policy(u, Feedback::Off);
            assert!(!p.above_safe());
            assert_eq!(highest_when_cool(&mut p, 0), 2);
            let u = cap_use(cap, Feedback::Hold).unwrap();
            assert_eq!(u, CapUse { ceiling: 5, fallback: 2, ranges: false });
            let mut p = policy(u, Feedback::Hold);
            assert!(p.above_safe());
            assert_eq!(highest_when_cool(&mut p, 0), 2);
            // With the limit on, the firmware is told about the ceiling and starts at 2.
            let u = cap_use(cap, Feedback::On).unwrap();
            assert_eq!(u, CapUse { ceiling: cap as u32, fallback: 2, ranges: true });
        }
        for cap in [0u64, 9, 10, 16, 255] {
            for feedback in MODES {
                assert_eq!(cap_use(cap, feedback).unwrap_err().name, "t8122_pstate_cap");
            }
        }
    }

    #[test]
    fn every_loss_of_feedback_returns_a_raised_cap_to_two() {
        for cap in 3..=8u64 {
            let u = cap_use(cap, Feedback::On).unwrap();
            let losses = [
                Reading::None,
                Reading::Failed { errno: -19, at: 30 * S },
                Reading::Failed { errno: -61, at: 30 * S },
                Reading::Temp { mc: 46_000, at: 26 * S, zones: 1 },
                Reading::Temp { mc: -2_849, at: 30 * S, zones: 1 },
                Reading::Temp { mc: 88_000, at: 30 * S, zones: 1 },
            ];
            for reading in losses {
                let mut p = policy(u, Feedback::On);
                assert_eq!(highest_when_cool(&mut p, 0), cap as u32);
                let _ = p.update(30 * S, reading);
                assert_eq!(p.cap(), 2, "cap {cap} {reading:?}");
            }
        }
    }

    #[test]
    fn caps_one_and_two_never_move_whatever_the_thermal_limit() {
        for cap in 1..=2u64 {
            for feedback in MODES {
                let u = cap_use(cap, feedback).unwrap();
                assert_eq!(u, CapUse { ceiling: cap as u32, fallback: cap as u32, ranges: false });
                let mut p = policy(u, feedback);
                assert!(!p.above_safe());
                assert_eq!(highest_when_cool(&mut p, 0), cap as u32);
                let _ = p.update(30 * S, Reading::Temp { mc: 95_000, at: 30 * S, zones: 1 });
                assert_eq!(p.cap(), cap as u32);
                assert_eq!(p.counts(), (0, 0));
            }
        }
    }

    /// `ThermalMode` as `t8122_start::feedback` maps it.
    fn mode(feedback: Feedback) -> ThermalMode {
        match feedback {
            Feedback::Off => ThermalMode::Off,
            Feedback::Hold => ThermalMode::Hold,
            Feedback::On => ThermalMode::On,
        }
    }

    #[test]
    fn pstate_cap_use_is_what_pstate_policy_computes() {
        // `PstatePolicy::new` with `asahi.m3_max_pstate` unset: the whole table (8) with the
        // limit on or holding, the default of 5 with it off, then `pstate_caps` on the admitted
        // cap. `Contents::select` refuses the start unless the result is within `pstate_cap_use`;
        // without `asahi.m3_max_pstate` the two must be equal.
        for cap in 1..=8u32 {
            for feedback in MODES {
                let requested = if feedback == Feedback::Off { 5 } else { 8 };
                let admitted = knobs::admitted_cap(cap, feedback);
                let (max, safe) = mode(feedback).pstate_caps(requested, Some(admitted));
                let u = knobs::pstate_cap_use(cap, feedback);
                assert_eq!((max, safe), (u.ceiling, u.fallback), "cap {cap} {feedback:?}");
                assert_eq!(u.ranges, feedback == Feedback::On && max > safe);
                // A lower asahi.m3_max_pstate stays within it as well.
                for lower in 1..requested {
                    let (max, safe) = mode(feedback).pstate_caps(lower, Some(admitted));
                    assert!(max <= u.ceiling && safe <= u.fallback, "cap {cap} {feedback:?} {lower}");
                }
            }
        }
    }
}
