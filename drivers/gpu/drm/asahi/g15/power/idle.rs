// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! The firmware idle-policy words.
//!
//! Globals inputs of the firmware's idle decision:
//!
//! - +0x7a8 standby time, us (board template 700): the GPU stays up about this long after its
//!   last work; tuned 2500 (with 700 us the GPU powers down between works ~100 us apart and
//!   pays ~300 us at the next pickup).
//! - +0x7c4 minimum confidence of the predictive model, f32 (template 0x3f333333 = 0.7); tuned
//!   0.5.
//! - +0x30 predictive branch (template 1); 0 = the ordinary finite delay: the GPU powers down
//!   once idle for the +0x9a0 delay (template 2, about 2 ms). The predictive branch makes 2-3 %
//!   of works under sustained low-duty submission pay a ~300 us power-up.
//! - +0x2c enables the idle decision (template 1, unchanged).
//!
//! Core-only (no `kernel` imports), so that it can be tested on the host.

/// Globals offsets (mirrors of `fw::initdata::globals`).
pub(crate) const ENABLE: usize = 0x2c;
pub(crate) const PREDICTIVE: usize = 0x30;
pub(crate) const STANDBY_US: usize = 0x7a8;
pub(crate) const MIN_CONF: usize = 0x7c4;
pub(crate) const DELAY: usize = 0x9a0;

/// IEEE-754 single bits of `permille / 1000`, `permille` clamped to 1..=1000 (0 cannot be
/// expressed: it reads 0.001).
pub(crate) const fn permille_f32(permille: u32) -> u32 {
    let p = if permille < 1 {
        1
    } else if permille > 1000 {
        1000
    } else {
        permille
    };
    // p/1000 * 2^k in [0.5, 1).
    let mut k = 0u32;
    while p * (1u32 << k) < 500 {
        k += 1;
    }
    if p == 1000 {
        return 0x3f800000;
    }
    // p/1000 = m/2^24 * 2^(1-k-1), m in [2^23, 2^24).
    ((126 - k) << 23) | ((((p as u64) << (24 + k)) / 1000) as u32 - (1u32 << 23))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn f32_bits(x: f32) -> u32 {
        x.to_bits()
    }

    #[test]
    fn permille() {
        assert_eq!(permille_f32(500), 0x3f000000);
        assert_eq!(permille_f32(700), 0x3f333333);
        assert_eq!(permille_f32(1000), 0x3f800000);
        assert_eq!(permille_f32(2000), 0x3f800000);
        assert_eq!(permille_f32(0), permille_f32(1));
        assert_eq!(permille_f32(250), 0x3e800000);
        // Within one ulp (truncation) of the nearest float everywhere.
        for p in 1..=1000u32 {
            let want = f32_bits(p as f32 / 1000.0);
            let got = permille_f32(p);
            assert!(want.abs_diff(got) <= 1, "{p}: {got:#x} vs {want:#x}");
        }
    }
}
