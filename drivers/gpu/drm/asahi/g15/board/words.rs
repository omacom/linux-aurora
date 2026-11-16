// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! The J516C Globals and Power images from the device tree.
//!
//! The 2419 firmware's Globals (15,868 bytes) and Power (2,292 bytes) are, on the J516C, the G15S
//! templates ([`super::data`]) with these words of its own:
//!
//! - The GPU node's power words ([`PROPS`]): `apple,idleoff-standby-timer` (Globals +0x7a8);
//!   in Power the average-power controller (+0x10 filter periods = `apple,avg-power-filter-tc-ms`
//!   / `apple,power-sample-period`, +0x14 `apple,avg-power-ki-only` x period, +0x18
//!   `apple,avg-power-kp`, +0x1c `apple,avg-power-min-duty-cycle`, +0x20
//!   `apple,avg-power-target-filter-tc`), the power zones (+0xc count, +0x24 up to five
//!   {target, target - offset, filter} rows of `apple,power-zones`), the fast-die controller
//!   (+0xb8 `apple,fast-die0-proportional-gain`, +0xbc `apple,fast-die0-integral-gain` x period)
//!   and the PPM controller (+0xd4 `apple,ppm-kp`, +0xd8 `apple,ppm-ki` x period). A gain x
//!   period is the IEEE single product of the gain and `period_ms / 1000`, each rounded to
//!   single ([`f32_mul`], [`f32_div`]).
//! - The maximum power: the hardware record's HardwareDataA +0x6a4 word (Globals +0x78, Power
//!   +0xd0). The G15S template carries its own machine's there too.
//! - The performance-state cap: 100 x the top firmware DVFS state, 8 (Globals +0x7c, +0x80,
//!   +0x8c).
//! - Words the device tree does not carry: command submission enabled at initdata (Globals
//!   +0x70 = 1; the G15S template has 0), the start path's +0x980 = 100 (written again at
//!   start on every SoC), the soft-fault word (u32 3 at the unaligned +0xdf9 of the G15S image)
//!   at +0x1201, where the G15C Globals has it, and the Power +0x244 flag = 1.
//!
//! Core-only (no `kernel` imports): this file is compiled by the host test harness as well.

use core::ffi::CStr;

pub(crate) const GLOBALS_SIZE: usize = 15868;
pub(crate) const POWER_SIZE: usize = 2292;

/// HardwareDataA in the hardware record (= `fw::initdata::hwdata::HWDATA_A`) and its maximum
/// power word (= `fw::initdata::hwdata::MAX_POWER_A`).
pub(crate) const HWDATA_A: usize = 0x4580;
pub(crate) const HWDATA_A_MAX_POWER: usize = 0x6a4;

/// The top firmware DVFS state (= `board::DVFS_STATES - 1`).
pub(crate) const TOP_STATE: u32 = 8;

/// Globals words.
mod globals {
    pub(super) const COMMAND_SUBMISSION: usize = 0x70;
    pub(super) const MAX_POWER: usize = 0x78;
    /// The power-interface targets 1 and 2 and the performance-state cap.
    pub(super) const STATE_CAP: [usize; 3] = [0x7c, 0x80, 0x8c];
    pub(super) const STANDBY_TIMER: usize = 0x7a8;
    pub(super) const UNK_980: usize = 0x980;
    /// The soft-fault word: unaligned, in the G15S and the G15C image.
    pub(super) const SOFT_FAULT_G15S: usize = 0xdf9;
    pub(super) const SOFT_FAULT_G15C: usize = 0x1201;
    pub(super) const SOFT_FAULT: u32 = 3;
}

/// Power words.
mod power {
    pub(super) const ZONE_COUNT: usize = 0x0c;
    pub(super) const AVG_FILTER_PERIODS: usize = 0x10;
    pub(super) const AVG_KI_DT: usize = 0x14;
    pub(super) const AVG_KP: usize = 0x18;
    pub(super) const AVG_MIN_DUTY_CYCLE: usize = 0x1c;
    pub(super) const AVG_TARGET_FILTER_TC: usize = 0x20;
    pub(super) const ZONES: usize = 0x24;
    pub(super) const ZONE_STRIDE: usize = 12;
    pub(super) const FAST_DIE_KP: usize = 0xb8;
    pub(super) const FAST_DIE_KI_DT: usize = 0xbc;
    pub(super) const MAX_POWER: usize = 0xd0;
    pub(super) const PPM_KP: usize = 0xd4;
    pub(super) const PPM_KI_DT: usize = 0xd8;
    pub(super) const FLAG_244: usize = 0x244;
}

/// The GPU node's power properties, each one u32 (gains are IEEE single bits), in the order of
/// [`PowerWords::values`].
pub(crate) const PROPS: [&CStr; 11] = [
    c"apple,power-sample-period",
    c"apple,idleoff-standby-timer",
    c"apple,avg-power-filter-tc-ms",
    c"apple,avg-power-ki-only",
    c"apple,avg-power-kp",
    c"apple,avg-power-min-duty-cycle",
    c"apple,avg-power-target-filter-tc",
    c"apple,fast-die0-proportional-gain",
    c"apple,fast-die0-integral-gain",
    c"apple,ppm-kp",
    c"apple,ppm-ki",
];
pub(crate) const PERIOD_MS: usize = 0;
pub(crate) const STANDBY_TIMER: usize = 1;
pub(crate) const AVG_POWER_FILTER_TC_MS: usize = 2;
pub(crate) const AVG_POWER_KI_ONLY: usize = 3;
pub(crate) const AVG_POWER_KP: usize = 4;
pub(crate) const AVG_POWER_MIN_DUTY_CYCLE: usize = 5;
pub(crate) const AVG_POWER_TARGET_FILTER_TC: usize = 6;
pub(crate) const FAST_DIE0_PROPORTIONAL_GAIN: usize = 7;
pub(crate) const FAST_DIE0_INTEGRAL_GAIN: usize = 8;
pub(crate) const PPM_KP: usize = 9;
pub(crate) const PPM_KI: usize = 10;

/// The power zones, u32 triples {target, offset, filter time constant}; at most five.
pub(crate) const ZONES: &CStr = c"apple,power-zones";
pub(crate) const MAX_ZONES: usize = 5;

/// The GPU node's power words.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) struct PowerWords {
    pub(crate) values: [u32; PROPS.len()],
    pub(crate) zones: [[u32; 3]; MAX_ZONES],
    pub(crate) zone_count: u32,
}

impl PowerWords {
    /// `values` in [`PROPS`] order and the `apple,power-zones` cells.
    pub(crate) fn new(values: [u32; PROPS.len()], zones: &[u32]) -> Result<Self, &'static str> {
        if values[PERIOD_MS] == 0 {
            return Err("the power sample period is 0");
        }
        if zones.len() % 3 != 0 || zones.len() > 3 * MAX_ZONES {
            return Err("the power zones are not up to five u32 triples");
        }
        let mut out = Self {
            values,
            zones: [[0; 3]; MAX_ZONES],
            zone_count: (zones.len() / 3) as u32,
        };
        for (row, cells) in out.zones.iter_mut().zip(zones.chunks_exact(3)) {
            row.copy_from_slice(cells);
        }
        Ok(out)
    }

    /// The sample period in seconds, IEEE single bits.
    fn period_s(&self) -> Result<u32, &'static str> {
        f32_div(f32_from_u32(self.values[PERIOD_MS]), f32_from_u32(1000))
            .ok_or("the power sample period has no single-precision quotient")
    }

    /// A gain x the sample period, IEEE single bits.
    fn per_period(&self, gain: usize) -> Result<u32, &'static str> {
        f32_mul(self.values[gain], self.period_s()?)
            .ok_or("a gain is not a finite normal single, or its product with the period is not")
    }
}

fn get32(b: &[u8], offset: usize) -> Result<u32, &'static str> {
    let w = b.get(offset..offset + 4).ok_or("image too short")?;
    Ok(u32::from_le_bytes([w[0], w[1], w[2], w[3]]))
}

fn put32(b: &mut [u8], offset: usize, value: u32) -> Result<(), &'static str> {
    b.get_mut(offset..offset + 4)
        .ok_or("image too short")?
        .copy_from_slice(&value.to_le_bytes());
    Ok(())
}

/// Writes the J516C words into `image`, the expanded G15S Globals template.
pub(crate) fn globals(
    image: &mut [u8],
    max_power_mw: u32,
    w: &PowerWords,
) -> Result<(), &'static str> {
    if image.len() != GLOBALS_SIZE {
        return Err("Globals image of another size");
    }
    put32(image, globals::COMMAND_SUBMISSION, 1)?;
    put32(image, globals::MAX_POWER, max_power_mw)?;
    for at in globals::STATE_CAP {
        put32(image, at, 100 * TOP_STATE)?;
    }
    put32(image, globals::STANDBY_TIMER, w.values[STANDBY_TIMER])?;
    put32(image, globals::UNK_980, 100)?;
    if get32(image, globals::SOFT_FAULT_G15S)? != globals::SOFT_FAULT {
        return Err("Globals template without the soft-fault word at its G15S place");
    }
    put32(image, globals::SOFT_FAULT_G15S, 0)?;
    put32(image, globals::SOFT_FAULT_G15C, globals::SOFT_FAULT)?;
    Ok(())
}

/// Writes the J516C words into `image`, the expanded G15S Power template.
pub(crate) fn power(
    image: &mut [u8],
    max_power_mw: u32,
    w: &PowerWords,
) -> Result<(), &'static str> {
    if image.len() != POWER_SIZE {
        return Err("Power image of another size");
    }
    let periods = w.values[AVG_POWER_FILTER_TC_MS] / w.values[PERIOD_MS];
    if periods == 0 {
        return Err("the average-power filter time constant is shorter than the sample period");
    }
    put32(image, power::ZONE_COUNT, w.zone_count)?;
    put32(image, power::AVG_FILTER_PERIODS, periods)?;
    put32(image, power::AVG_KI_DT, w.per_period(AVG_POWER_KI_ONLY)?)?;
    put32(image, power::AVG_KP, w.values[AVG_POWER_KP])?;
    put32(
        image,
        power::AVG_MIN_DUTY_CYCLE,
        w.values[AVG_POWER_MIN_DUTY_CYCLE],
    )?;
    put32(
        image,
        power::AVG_TARGET_FILTER_TC,
        w.values[AVG_POWER_TARGET_FILTER_TC],
    )?;
    for (i, zone) in w.zones.iter().enumerate() {
        let at = power::ZONES + power::ZONE_STRIDE * i;
        let (target, offset, filter) = (zone[0], zone[1], zone[2]);
        let target_off = target
            .checked_sub(offset)
            .ok_or("a power zone's offset is larger than its target")?;
        put32(image, at, target)?;
        put32(image, at + 4, target_off)?;
        put32(image, at + 8, filter)?;
    }
    put32(
        image,
        power::FAST_DIE_KP,
        w.values[FAST_DIE0_PROPORTIONAL_GAIN],
    )?;
    put32(
        image,
        power::FAST_DIE_KI_DT,
        w.per_period(FAST_DIE0_INTEGRAL_GAIN)?,
    )?;
    put32(image, power::MAX_POWER, max_power_mw)?;
    put32(image, power::PPM_KP, w.values[PPM_KP])?;
    put32(image, power::PPM_KI_DT, w.per_period(PPM_KI)?)?;
    put32(image, power::FLAG_244, 1)?;
    Ok(())
}

/// The first word (4-byte aligned offset) at which `a` and `b` differ, with both values; a
/// missing word reads 0. `None` when they are equal.
#[cfg(test)]
pub(crate) fn first_difference(a: &[u8], b: &[u8]) -> Option<(usize, u32, u32)> {
    let word = |s: &[u8], at: usize| -> u32 {
        let mut w = [0u8; 4];
        for (i, out) in w.iter_mut().enumerate() {
            *out = s.get(at + i).copied().unwrap_or(0);
        }
        u32::from_le_bytes(w)
    };
    let len = a.len().max(b.len());
    (0..len)
        .step_by(4)
        .map(|at| (at, word(a, at), word(b, at)))
        .find(|&(_, x, y)| x != y)
}

// --- IEEE binary32 arithmetic on bit patterns (the kernel has no floating point) ---
//
// Operands and results are finite normal numbers or zero; anything else is `None`.
// Rounding is to nearest, ties to even.

/// Sign, unbiased exponent of the leading bit and mantissa with its implicit bit; zero is
/// `(sign, 0, 0)`.
fn unpack(bits: u32) -> Option<(bool, i32, u32)> {
    let sign = bits >> 31 != 0;
    let exponent = ((bits >> 23) & 0xff) as i32;
    let mantissa = bits & 0x7f_ffff;
    match exponent {
        0 if mantissa == 0 => Some((sign, 0, 0)),
        0 | 0xff => None,
        _ => Some((sign, exponent - 127, mantissa | 0x80_0000)),
    }
}

/// The single nearest to `sign * m * 2^exp`, `m` with its leading bit anywhere; a sticky bit in
/// the lowest bit of `m` may stand for a nonzero remainder below it.
fn pack(sign: bool, exp: i32, m: u64) -> Option<u32> {
    if m == 0 {
        return Some((sign as u32) << 31);
    }
    let shift = m.leading_zeros();
    let m = m << shift;
    let exp = exp - shift as i32 + 63;
    let mut mantissa = (m >> 40) as u32;
    let rest = m & ((1u64 << 40) - 1);
    let half = 1u64 << 39;
    if rest > half || (rest == half && mantissa & 1 == 1) {
        mantissa += 1;
    }
    let mut exp = exp;
    if mantissa == 1 << 24 {
        mantissa >>= 1;
        exp += 1;
    }
    let biased = exp + 127;
    if !(1..=254).contains(&biased) {
        return None;
    }
    Some((sign as u32) << 31 | (biased as u32) << 23 | (mantissa & 0x7f_ffff))
}

/// `n` as a single.
pub(crate) fn f32_from_u32(n: u32) -> u32 {
    // Every u32 rounds to a normal single or zero.
    pack(false, 0, n as u64).unwrap_or(0)
}

/// `a * b`.
pub(crate) fn f32_mul(a: u32, b: u32) -> Option<u32> {
    let (sa, ea, ma) = unpack(a)?;
    let (sb, eb, mb) = unpack(b)?;
    if ma == 0 || mb == 0 {
        return Some(((sa ^ sb) as u32) << 31);
    }
    pack(sa ^ sb, ea + eb - 46, ma as u64 * mb as u64)
}

/// `a / b`.
pub(crate) fn f32_div(a: u32, b: u32) -> Option<u32> {
    let (sa, ea, ma) = unpack(a)?;
    let (sb, eb, mb) = unpack(b)?;
    if mb == 0 {
        return None;
    }
    if ma == 0 {
        return Some(((sa ^ sb) as u32) << 31);
    }
    // ma < 2^24, so ma << 40 fits; the quotient has at least 40 bits, 16 and more below the
    // rounding position, so a sticky lowest bit stands in for a nonzero remainder.
    let n = (ma as u64) << 40;
    let mut q = n / mb as u64;
    if n % mb as u64 != 0 {
        q |= 1;
    }
    pack(sa ^ sb, ea - eb - 40, q)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn get32(b: &[u8], o: usize) -> u32 {
        u32::from_le_bytes(b[o..o + 4].try_into().unwrap())
    }

    /// The J516C GPU node's power words, as the saved device tree of that machine carries them.
    fn j516c_words() -> PowerWords {
        PowerWords::new(
            [
                8,          // apple,power-sample-period
                1500,       // apple,idleoff-standby-timer
                32,         // apple,avg-power-filter-tc-ms
                0x41480000, // apple,avg-power-ki-only (12.5)
                0x3e99999a, // apple,avg-power-kp (0.3)
                40,         // apple,avg-power-min-duty-cycle
                1,          // apple,avg-power-target-filter-tc
                0x42080000, // apple,fast-die0-proportional-gain (34.0)
                0x443b8000, // apple,fast-die0-integral-gain (750.0)
                0x3e99999a, // apple,ppm-kp (0.3)
                0x4198cccd, // apple,ppm-ki (19.1)
            ],
            &[],
        )
        .unwrap()
    }

    #[test]
    fn single_arithmetic() {
        // The products the power words need, against the host's IEEE singles.
        let dt = f32_div(f32_from_u32(8), f32_from_u32(1000)).unwrap();
        assert_eq!(dt, (8.0f32 / 1000.0).to_bits());
        for ki in [0x443b8000u32, 0x41480000, 0x4198cccd] {
            assert_eq!(
                f32_mul(ki, dt).unwrap(),
                (f32::from_bits(ki) * f32::from_bits(dt)).to_bits()
            );
        }
        // A sweep of normal operands, both signs.
        let mut x = 0x2545f491u32;
        for _ in 0..20000 {
            x ^= x << 13;
            x ^= x >> 17;
            x ^= x << 5;
            let a = f32::from_bits(x);
            let b = f32::from_bits(x.rotate_left(11) ^ 0x5a5a5a5a);
            if !a.is_normal() || !b.is_normal() {
                continue;
            }
            let p = a * b;
            if p.is_normal() {
                assert_eq!(
                    f32_mul(a.to_bits(), b.to_bits()),
                    Some(p.to_bits()),
                    "{a} * {b}"
                );
            }
            let q = a / b;
            if q.is_normal() {
                assert_eq!(
                    f32_div(a.to_bits(), b.to_bits()),
                    Some(q.to_bits()),
                    "{a} / {b}"
                );
            }
        }
        for n in [
            0u32,
            1,
            8,
            1000,
            1500,
            0xffffff,
            0x1000000,
            0x1000001,
            u32::MAX,
        ] {
            assert_eq!(f32_from_u32(n), (n as f32).to_bits());
        }
        assert_eq!(f32_mul(0, 0x3f800000), Some(0));
        assert_eq!(f32_mul(0x3f800000, 0x80000000), Some(0x80000000));
        assert_eq!(f32_div(0x3f800000, 0), None);
        assert_eq!(f32_div(0, 0x3f800000), Some(0));
        // Infinities, NaNs and subnormals are not operands.
        assert_eq!(f32_mul(0x7f800000, 0x3f800000), None);
        assert_eq!(f32_mul(0x00000001, 0x3f800000), None);
        // Overflow and underflow of the result are refused, not rounded.
        assert_eq!(f32_mul(0x7f000000, 0x7f000000), None);
        assert_eq!(f32_mul(0x00800000, 0x00800000), None);
    }

    #[test]
    fn words_shape() {
        let w = j516c_words();
        assert_eq!(w.zone_count, 0);
        assert_eq!(PowerWords::new(w.values, &[1, 2, 3]).unwrap().zone_count, 1);
        assert_eq!(
            PowerWords::new(w.values, &[1, 2, 3]).unwrap().zones[0],
            [1, 2, 3]
        );
        assert!(PowerWords::new(w.values, &[1, 2]).is_err());
        assert!(PowerWords::new(w.values, &[0; 18]).is_err());
        let mut v = w.values;
        v[PERIOD_MS] = 0;
        assert!(PowerWords::new(v, &[]).is_err());
        assert_eq!(PROPS.len(), w.values.len());
        assert_eq!(PROPS[PERIOD_MS], c"apple,power-sample-period");
        assert_eq!(PROPS[STANDBY_TIMER], c"apple,idleoff-standby-timer");
    }

    #[test]
    fn globals_words() {
        let w = j516c_words();
        let mut img = vec![0u8; GLOBALS_SIZE];
        // A G15S-shaped image: the soft-fault word at its unaligned place.
        img[0xdf9] = 3;
        globals(&mut img, 54321, &w).unwrap();
        assert_eq!(get32(&img, 0x70), 1);
        assert_eq!(get32(&img, 0x78), 54321);
        assert_eq!(get32(&img, 0x7c), 800);
        assert_eq!(get32(&img, 0x80), 800);
        assert_eq!(get32(&img, 0x8c), 800);
        assert_eq!(get32(&img, 0x7a8), 1500);
        assert_eq!(get32(&img, 0x980), 100);
        assert_eq!(get32(&img, 0xdf8), 0);
        assert_eq!(get32(&img, 0x1200), 0x300);
        // Everything else untouched.
        let touched = [0x70, 0x78, 0x7c, 0x80, 0x8c, 0x7a8, 0x980, 0xdf8, 0x1200];
        for o in (0..GLOBALS_SIZE).step_by(4) {
            if !touched.contains(&o) {
                assert_eq!(get32(&img, o), 0, "+{o:#x}");
            }
        }
        // Without the soft-fault word where the G15S image has it, the image is not the one
        // this rule is written for.
        let mut bad = vec![0u8; GLOBALS_SIZE];
        assert!(globals(&mut bad, 1, &w).is_err());
        let mut short = vec![0u8; GLOBALS_SIZE - 1];
        assert!(globals(&mut short, 1, &w).is_err());
    }

    #[test]
    fn power_words() {
        let w = j516c_words();
        let mut img = vec![0u8; POWER_SIZE];
        power(&mut img, 54321, &w).unwrap();
        assert_eq!(get32(&img, 0x0c), 0);
        assert_eq!(get32(&img, 0x10), 4);
        assert_eq!(get32(&img, 0x14), 0x3dcccccd);
        assert_eq!(get32(&img, 0x18), 0x3e99999a);
        assert_eq!(get32(&img, 0x1c), 40);
        assert_eq!(get32(&img, 0x20), 1);
        assert_eq!(get32(&img, 0xb8), 0x42080000);
        assert_eq!(get32(&img, 0xbc), 0x40c00001);
        assert_eq!(get32(&img, 0xd0), 54321);
        assert_eq!(get32(&img, 0xd4), 0x3e99999a);
        assert_eq!(get32(&img, 0xd8), 0x3e1c779b);
        assert_eq!(get32(&img, 0x244), 1);
        let touched = [
            0x10, 0x14, 0x18, 0x1c, 0x20, 0xb8, 0xbc, 0xd0, 0xd4, 0xd8, 0x244,
        ];
        for o in (0..POWER_SIZE).step_by(4) {
            if !touched.contains(&o) {
                assert_eq!(get32(&img, o), 0, "+{o:#x}");
            }
        }
        // Power zones: count and {target, target - offset, filter} rows.
        let z = PowerWords::new(w.values, &[5000, 200, 7, 9000, 1000, 3]).unwrap();
        let mut img = vec![0u8; POWER_SIZE];
        power(&mut img, 1, &z).unwrap();
        assert_eq!(get32(&img, 0x0c), 2);
        assert_eq!(get32(&img, 0x24), 5000);
        assert_eq!(get32(&img, 0x28), 4800);
        assert_eq!(get32(&img, 0x2c), 7);
        assert_eq!(get32(&img, 0x30), 9000);
        assert_eq!(get32(&img, 0x34), 8000);
        assert_eq!(get32(&img, 0x38), 3);
        assert_eq!(get32(&img, 0x3c), 0);
        // An offset larger than its target has no target - offset word.
        let under = PowerWords::new(w.values, &[100, 101, 1]).unwrap();
        let mut img = vec![0u8; POWER_SIZE];
        assert!(power(&mut img, 1, &under).is_err());
        // A filter time constant shorter than the sample period has no period count.
        let mut v = w.values;
        v[AVG_POWER_FILTER_TC_MS] = 4;
        let mut img = vec![0u8; POWER_SIZE];
        assert!(power(&mut img, 1, &PowerWords::new(v, &[]).unwrap()).is_err());
        let mut short = vec![0u8; POWER_SIZE - 1];
        assert!(power(&mut short, 1, &w).is_err());
    }

    #[test]
    fn differences() {
        let a = [1u8, 0, 0, 0, 2, 0, 0, 0, 3, 0, 0, 0];
        let mut b = a;
        assert_eq!(first_difference(&a, &b), None);
        b[9] = 1;
        assert_eq!(first_difference(&a, &b), Some((8, 3, 0x103)));
        assert_eq!(first_difference(&a, &b[..8]), Some((8, 3, 0)));
        assert_eq!(first_difference(&a[..8], &b), Some((8, 0, 0x103)));
    }

    // --- The fixture: a saved copy of a J516C's device tree and its private board file. ---

    fn be32(b: &[u8], o: usize) -> u32 {
        u32::from_be_bytes(b[o..o + 4].try_into().unwrap())
    }

    /// Properties of the flattened device tree's node whose `compatible` names `compatible`.
    fn fdt_node_props(dtb: &[u8], compatible: &[u8]) -> Option<Vec<(String, Vec<u8>)>> {
        assert_eq!(be32(dtb, 0), 0xd00dfeed);
        let strings = be32(dtb, 12) as usize;
        let mut at = be32(dtb, 8) as usize;
        let mut stack: Vec<Vec<(String, Vec<u8>)>> = Vec::new();
        loop {
            let token = be32(dtb, at);
            at += 4;
            match token {
                1 => {
                    let end = dtb[at..].iter().position(|&b| b == 0).unwrap();
                    at = (at + end + 1).next_multiple_of(4);
                    stack.push(Vec::new());
                }
                2 => {
                    let props = stack.pop().unwrap();
                    let hit = props.iter().any(|(n, v)| {
                        n == "compatible" && v.split(|&b| b == 0).any(|s| s == compatible)
                    });
                    if hit {
                        return Some(props);
                    }
                }
                3 => {
                    let len = be32(dtb, at) as usize;
                    let name_off = strings + be32(dtb, at + 4) as usize;
                    let name_end = dtb[name_off..].iter().position(|&b| b == 0).unwrap();
                    let name =
                        String::from_utf8(dtb[name_off..name_off + name_end].to_vec()).unwrap();
                    let value = dtb[at + 8..at + 8 + len].to_vec();
                    at = (at + 8 + len).next_multiple_of(4);
                    stack.last_mut().unwrap().push((name, value));
                }
                4 => {}
                9 => return None,
                _ => panic!("fdt token {token}"),
            }
        }
    }

    /// Builder on the saved device tree == the private file's Globals and Power, byte for byte.
    /// Needs `G15_J516C_FIXTURES=<dir>` with `j516c.dtb` (`/sys/firmware/fdt` of the machine)
    /// and `t6031-j516c-board.bin` (its own images); both stay off the tree, so the test is
    /// ignored by default (`--ignored` runs it) and fails, rather than passes, without them.
    #[test]
    #[ignore = "needs the off-tree J516C fixtures (G15_J516C_FIXTURES)"]
    fn j516c_fixture() {
        let dir = std::env::var_os("G15_J516C_FIXTURES")
            .expect("G15_J516C_FIXTURES names the directory with j516c.dtb and the board file");
        let dir = std::path::Path::new(&dir);
        let dtb = std::fs::read(dir.join("j516c.dtb")).unwrap();
        let blob = std::fs::read(dir.join("t6031-j516c-board.bin")).unwrap();

        // The machine's images: magic, version, count, 3 x (kind, size), then the images.
        assert_eq!(&blob[..8], b"T6031BRD");
        assert_eq!(get32(&blob, 8), 1);
        assert_eq!(get32(&blob, 12), 3);
        let mut at = 16 + 3 * 8;
        let mut images = Vec::new();
        for i in 0..3 {
            assert_eq!(get32(&blob, 16 + i * 8) as usize, i);
            let size = get32(&blob, 16 + i * 8 + 4) as usize;
            images.push(blob[at..at + size].to_vec());
            at += size;
        }
        assert_eq!(at, blob.len());
        let (hardware, file_globals, file_power) = (&images[0], &images[1], &images[2]);
        assert_eq!(hardware.len(), 0xb480);
        assert_eq!(file_globals.len(), GLOBALS_SIZE);
        assert_eq!(file_power.len(), POWER_SIZE);

        // The GPU node's words.
        let props = fdt_node_props(&dtb, b"apple,agx-t6031").expect("a T6031 GPU node");
        let prop = |name: &core::ffi::CStr| -> Option<&[u8]> {
            let name = name.to_str().unwrap();
            props
                .iter()
                .find(|(n, _)| n == name)
                .map(|(_, v)| v.as_slice())
        };
        let mut values = [0u32; PROPS.len()];
        for (v, name) in values.iter_mut().zip(PROPS.iter()) {
            let bytes = prop(name).unwrap_or_else(|| panic!("GPU node has no {name:?}"));
            assert_eq!(bytes.len(), 4, "{name:?}");
            *v = be32(bytes, 0);
        }
        let zones: Vec<u32> = prop(ZONES)
            .map(|b| b.chunks(4).map(|c| be32(c, 0)).collect())
            .unwrap_or_default();
        let words = PowerWords::new(values, &zones).unwrap();

        // The images: the G15S templates with the J516C words, the hardware record's maximum
        // power and the top state.
        let max_power = get32(hardware, HWDATA_A + HWDATA_A_MAX_POWER);
        let mut built_globals = vec![0u8; GLOBALS_SIZE];
        super::super::template::fill(&super::super::data::GLOBALS, &mut built_globals).unwrap();
        globals(&mut built_globals, max_power, &words).unwrap();
        let mut built_power = vec![0u8; POWER_SIZE];
        super::super::template::fill(&super::super::data::POWER, &mut built_power).unwrap();
        power(&mut built_power, max_power, &words).unwrap();

        assert_eq!(
            first_difference(&built_globals, file_globals),
            None,
            "Globals"
        );
        assert_eq!(first_difference(&built_power, file_power), None, "Power");
    }
}
