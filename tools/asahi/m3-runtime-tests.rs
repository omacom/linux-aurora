// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Host tests for the production M3 admission, IO ownership, thermal policy and the T8122
//! HwDataA +0x418c block.
//! Run: rustc --edition=2021 --test tools/asahi/m3-runtime-tests.rs -o /tmp/m3-runtime-tests
#![allow(dead_code)]

#[path = "../../drivers/gpu/drm/asahi/m3/m3_init_layout.rs"]
mod m3_init_layout;
#[path = "../../drivers/gpu/drm/asahi/m3/m3_init_storage.rs"]
mod m3_init_storage;
#[path = "../../drivers/gpu/drm/asahi/m3/m3_thermal_policy.rs"]
mod m3_thermal_policy;
#[path = "../../drivers/gpu/drm/asahi/m3/t8122_admission.rs"]
mod t8122_admission;
#[path = "../../drivers/gpu/drm/asahi/m3/t8122_hwdata.rs"]
mod t8122_hwdata;

// Only the unrelated firmware-version default needs a kernel-free fixture. All parameter
// parsing, cap selection and thermal decisions below use the production modules directly.
mod m3 {
    pub mod m3_firmware {
        pub const G15_V14_8_3_INITDATA: u64 = 0x0c08_e21e_8380_0490;
    }
}
#[path = "../../drivers/gpu/drm/asahi/m3/t8122_knobs.rs"]
mod t8122_knobs;

#[cfg(test)]
mod t8122_performance {
    use super::m3_thermal_policy::{Policy, Reading, Source, Why, S};
    use super::t8122_knobs::{self as knobs, Raw};

    fn policy(raw: Raw, adjust: bool) -> Policy {
        let limits = knobs::resolve(&raw).unwrap().pstate_limits(true);
        Policy::new(adjust, limits.safe, limits.ceiling.unwrap_or(8), 80_000)
    }

    fn cool_to_ceiling(policy: &mut Policy) {
        for i in 0..8 {
            let now = i * 2 * S;
            let (reading, _) = policy.update(
                now,
                Reading::Temp {
                    mc: 45_000 + i as i32,
                    at: now,
                    zones: 1,
                },
            );
            assert!(reading.is_ok());
        }
    }

    #[test]
    fn default_reaches_full_table_and_sensor_failure_restores_state_two() {
        let mut p = policy(Raw::NONE, true);
        assert_eq!(p.cap(), 2);
        assert_eq!(p.ceiling(), 8);
        let _ = p.update(0, Reading::None);
        assert_eq!(p.cap(), 2);
        cool_to_ceiling(&mut p);
        assert_eq!(p.cap(), 8);
        let (_, change) = p.update(
            16 * S,
            Reading::Failed {
                errno: -19,
                at: 16 * S,
            },
        );
        let change = change.unwrap();
        assert_eq!((change.from, change.to), (8, 2));
        assert_eq!(change.why, Why::NoTemperature(Source::Failed(-19)));
    }

    #[test]
    fn full_cap_steps_down_on_heat_and_keeps_the_upward_pacing() {
        let mut p = policy(Raw::NONE, true);
        cool_to_ceiling(&mut p);
        let (_, change) = p.update(
            16 * S,
            Reading::Temp {
                mc: 80_000,
                at: 16 * S,
                zones: 1,
            },
        );
        assert_eq!(change.unwrap().why, Why::Hot);
        assert_eq!(p.cap(), 7);
        let _ = p.update(
            17 * S,
            Reading::Temp {
                mc: 45_000,
                at: 17 * S,
                zones: 1,
            },
        );
        assert_eq!(p.cap(), 7);
        let _ = p.update(
            18 * S,
            Reading::Temp {
                mc: 45_001,
                at: 18 * S,
                zones: 1,
            },
        );
        assert_eq!(p.cap(), 8);
    }

    #[test]
    fn missing_stale_stuck_and_critical_readings_cannot_keep_the_full_cap() {
        let cases = [
            Reading::None,
            Reading::Temp {
                mc: 45_000,
                at: 12 * S,
                zones: 1,
            },
            Reading::Temp {
                mc: 5_000,
                at: 16 * S,
                zones: 1,
            },
            Reading::Temp {
                mc: 88_000,
                at: 16 * S,
                zones: 1,
            },
        ];
        for reading in cases {
            let mut p = policy(Raw::NONE, true);
            cool_to_ceiling(&mut p);
            let _ = p.update(16 * S, reading);
            assert_eq!(p.cap(), 2, "{reading:?}");
        }
        let mut p = policy(Raw::NONE, true);
        cool_to_ceiling(&mut p);
        let _ = p.update(
            16 * S,
            Reading::Temp {
                mc: 45_007,
                at: 16 * S,
                zones: 1,
            },
        );
        assert_eq!(p.cap(), 8);
        let _ = p.update(
            76 * S,
            Reading::Temp {
                mc: 45_007,
                at: 76 * S,
                zones: 1,
            },
        );
        assert_eq!(p.cap(), 2);
    }

    #[test]
    fn hold_and_explicit_caps_do_not_gain_higher_runtime_states() {
        let mut held = policy(Raw::NONE, false);
        cool_to_ceiling(&mut held);
        assert_eq!(held.cap(), 2);
        assert_eq!(held.ceiling(), 8);
        for cap in 1..=5 {
            let mut limited = policy(
                Raw {
                    pstate_cap: cap,
                    ..Raw::NONE
                },
                true,
            );
            cool_to_ceiling(&mut limited);
            assert_eq!(limited.cap(), cap as u32);
            assert_eq!(limited.ceiling(), cap as u32);
        }
    }

    #[test]
    fn failure_to_obey_a_lowered_cap_is_still_detected() {
        let mut p = policy(Raw::NONE, true);
        cool_to_ceiling(&mut p);
        let _ = p.update(16 * S, Reading::None);
        assert_eq!(p.allowed(16 * S), 8);
        assert_eq!(p.allowed(17 * S), 2);
        for i in 0..19 {
            assert!(p.note_check(17 * S + i * S / 5, true).is_none());
        }
        assert!(p.note_check(21 * S, true).is_some());
        p.note_check(22 * S, false);
        assert!(p.note_check(23 * S, true).is_none());
    }
}
