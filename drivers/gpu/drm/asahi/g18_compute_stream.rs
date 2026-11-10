// SPDX-License-Identifier: GPL-2.0-only OR MIT
//! G18 compute control-stream admission: the one place the byte limit lives.
//!
//! The profile imported for the G18 adapter admits one dispatch, 48 bytes
//! (`m5_drm.rs:16-20`). Mesa's streams for the M6 are longer: the smoke and leg-1
//! runs of 2026-10-30 were refused at 92 (five times), 136 and 180 bytes
//! (`G18 compute admission rejected ... stream_bytes=N`). Those sizes are
//! 44 * k + 4 for k = 2, 3, 4, which reads as k dispatch records of 44 bytes plus a
//! 4 byte terminator, and 48 as k = 1. That reading is an INFERENCE from the sizes,
//! not a decoded layout, so the limit is a byte count and nothing here counts
//! dispatches.
//!
//! `g18_compute_stream_bytes` (module parameter, default 48) raises the limit.
//! The default keeps today's behaviour exactly. This module only decides whether
//! a length is admitted; what the firmware does with a longer stream is proven
//! on hardware, not here.
//!
//! Pure on purpose: no kernel types, so the host test builds it alone.
#![allow(dead_code)]

/// One direct dispatch plus terminator: the M5 profile and the default limit.
pub(crate) const BASE_STREAM_BYTES: u64 = 48;
/// Hard ceiling for the module parameter. A larger value is refused at probe.
/// Raised to k24's 64 KiB rule (w7Y cell 329a9ebe, Main directive 2026-11-10).
pub(crate) const MAX_STREAM_BYTES: u64 = 65536;

/// Validate the module parameter. `None` means refuse to bind: below the
/// base profile would silently tighten what is admitted today, above the ceiling
/// is not a reviewed size.
pub(crate) fn limit_from_param(param: u32) -> Option<u64> {
    let limit = u64::from(param);
    if (BASE_STREAM_BYTES..=MAX_STREAM_BYTES).contains(&limit) { Some(limit) } else { None }
}

/// The limit a configured adapter enforces. A zero field (`Default` config) means
/// the base profile; a configured value never goes below it.
pub(crate) fn effective_limit(configured: u64) -> u64 {
    if configured < BASE_STREAM_BYTES { BASE_STREAM_BYTES } else { configured.min(MAX_STREAM_BYTES) }
}

/// Whether a control stream of `stream_bytes` is admitted under `limit`.
pub(crate) fn admits(stream_bytes: u64, limit: u64) -> bool {
    stream_bytes <= limit
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_keeps_the_existing_profile() {
        let limit = effective_limit(0);
        assert_eq!(limit, 48);
        assert!(admits(48, limit), "one dispatch plus terminator is admitted");
        assert!(admits(4, limit) && admits(0, limit), "shorter streams are admitted as before");
        for refused in [49, 92, 136, 180, 4096] {
            assert!(!admits(refused, limit), "{refused} bytes stays refused by default");
        }
    }

    #[test]
    fn the_sizes_seen_on_the_m6_are_the_steps() {
        for (bytes, step_limit) in [(92u64, 92u64), (136, 136), (180, 180)] {
            assert!(!admits(bytes, effective_limit(0)), "{bytes} refused at the default");
            assert!(!admits(bytes, effective_limit(step_limit - 1)), "{bytes} refused one byte short");
            assert!(admits(bytes, effective_limit(step_limit)), "{bytes} admitted at its own step");
        }
        // The steps nest: a wider limit admits everything a narrower one did.
        for (narrow, wide) in [(48u64, 92u64), (92, 136), (136, 180)] {
            for bytes in 0..=wide {
                if admits(bytes, narrow) { assert!(admits(bytes, wide)); }
            }
        }
    }

    #[test]
    fn the_parameter_range_is_checked_before_anything_binds() {
        assert_eq!(limit_from_param(48), Some(48));
        assert_eq!(limit_from_param(92), Some(92));
        assert_eq!(limit_from_param(4096), Some(4096));
        for refused in [0u32, 1, 47, 4097, u32::MAX] {
            assert_eq!(limit_from_param(refused), None, "{refused} must refuse to bind");
        }
    }

    #[test]
    fn a_configured_value_never_tightens_and_never_exceeds_the_ceiling() {
        assert_eq!(effective_limit(1), 48);
        assert_eq!(effective_limit(47), 48);
        assert_eq!(effective_limit(48), 48);
        assert_eq!(effective_limit(5000), 4096);
        assert_eq!(effective_limit(u64::MAX), 4096);
        assert!(!admits(4097, effective_limit(u64::MAX)));
    }

    #[test]
    fn a_length_that_overflows_arithmetic_is_refused_not_wrapped() {
        assert!(!admits(u64::MAX, effective_limit(4096)));
        assert!(!admits(u64::MAX, effective_limit(0)));
    }
}
