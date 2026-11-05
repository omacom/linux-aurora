// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! The J516C hardware record (0xb480 bytes) from the device tree and the ADT.
//!
//! The record is HardwareDataB (identity, IOMapping records, the DVFS tables) at 0, HardwareDataA
//! (the power, performance and thermal controllers) at [`HWDATA_A`], and the G15C tail (sensor
//! keys). Every word is one of:
//!
//! - a device-tree value or a function of one ([`PROPS`], the operating-point tables, the
//!   reserved `ttbs` region): the controller gains, time constants and their derived filter
//!   words (`(tc - 1) / tc`, `1 / tc`, gain x period, periods x 4, periods x period, clocks),
//!   the two voltage-sorted DVFS tables with per-cluster voltages, the CS and AFR tables, the
//!   state cap and base state;
//! - a layout constant of the 2419 firmware's G15C record: flags, the IOMapping flag words,
//!   the identity words, fixed factors (65536, 95, 100, 1.02, 0.8/0.2, 625, 11000, 125, 5);
//! - the tail's six sensor rows: entries of the ADT's `aon-ptd` sensor table ([`sensor_rows`]);
//! - a word neither carries ([`REFERENCE_WORDS`]): the value of the J516C reference machine
//!   ([`Reference`]), as the T6030 templates carry their reference machine's: the maximum
//!   power (HardwareDataA +0x6a4; every other maximum-power word derives from it), the MTR
//!   sensor masks and the relative-power table (the device tree's operating-point powers are a
//!   boot-loader stand-in);
//! - a word the reference machine's record carries and the firmware was shown not to need
//!   ([`UNNEEDED`]): zero.
//!
//! HardwareDataA has the G15S shape of the T6030 templates with insertions ([`a`]): 8 bytes
//! before the fast-die controller (+0x8bc), 16-wide instead of 8-wide coefficient rows (+0x200
//! twice), 16-byte instead of 8-byte sensor masks (+8 twice), 0x600 and 0x2000 bytes of zero
//! blocks further on. The driver still writes the TTBAT, the IOMapping VAs, the timestamp arena
//! and the initdata timestamp as it does for the templates.
//!
//! Core-only (no `kernel` imports): this file is compiled by the host test harness as well.

use core::ffi::CStr;

use super::words::{f32_div, f32_from_u32, f32_mul};

pub(crate) const SIZE: usize = 0xb480;
pub(crate) const HWDATA_A: usize = 0x4580;
/// IOMapping records: 32 bytes per slot.
pub(crate) const IOMAPS: usize = 0x640;
pub(crate) const IOMAP_SLOTS: usize = 31;
/// DVFS table entries.
pub(crate) const STATES: usize = 16;
/// Entries of the CS and AFR tables.
const CS_STATES: usize = 16;
const AFR_STATES: usize = 8;
/// Operating points of the layout and voltage lanes per state.
pub(crate) const PERF_STATES: usize = 14;
pub(crate) const CLUSTERS: usize = 4;
const LANES: usize = 8;
/// The AIC timer base clock, kHz.
const BASE_CLOCK_KHZ: u32 = 24000;

// IEEE single bit patterns of the record's fixed factors.
const F_1: u32 = 0x3f800000;
const F_0_2: u32 = 0x3e4ccccd;
const F_0_8: u32 = 0x3f4ccccd;
const F_0_95: u32 = 0x3f733333;
const F_1_02: u32 = 0x3f828f5c;
const F_5: u32 = 0x40a00000;
const F_95: u32 = 0x42be0000;
const F_100: u32 = 0x42c80000;
const F_8000: u32 = 0x45fa0000;
const F_65536: u32 = 0x47800000;

/// The GPU node's properties the record takes, each one u32 (gains are IEEE single bits), with
/// the value a node without the property gets (`None`: required).
pub(crate) const PROPS: [(&CStr, Option<u32>); 43] = [
    (c"apple,power-sample-period", None),
    (c"apple,perf-base-pstate", None),
    (c"apple,min-sram-microvolt", None),
    (c"apple,csafr-min-sram-microvolt", None),
    (c"apple,avg-power-filter-tc-ms", None),
    (c"apple,avg-power-ki-only", None),
    (c"apple,avg-power-kp", None),
    (c"apple,avg-power-min-duty-cycle", None),
    (c"apple,avg-power-target-filter-tc", None),
    (c"apple,fast-die0-integral-gain", None),
    (c"apple,fast-die0-proportional-gain", None),
    (c"apple,fast-die0-prop-tgt-delta", Some(0)),
    (c"apple,perf-boost-ce-step", None),
    (c"apple,perf-boost-min-util", None),
    (c"apple,perf-filter-drop-threshold", None),
    (c"apple,perf-filter-time-constant", None),
    (c"apple,perf-filter-time-constant2", None),
    (c"apple,perf-integral-gain", None),
    (c"apple,perf-integral-gain2", None),
    (c"apple,perf-integral-min-clamp", None),
    (c"apple,perf-proportional-gain", None),
    (c"apple,perf-proportional-gain2", None),
    (c"apple,perf-reset-iters", Some(6)),
    (c"apple,perf-tgt-utilization", None),
    (c"apple,ppm-filter-time-constant-ms", None),
    (c"apple,ppm-ki", None),
    (c"apple,ppm-kp", None),
    (c"apple,pwr-filter-time-constant", None),
    (c"apple,pwr-integral-gain", None),
    (c"apple,pwr-integral-min-clamp", None),
    (c"apple,pwr-min-duty-cycle", None),
    (c"apple,pwr-proportional-gain", None),
    (c"apple,pwr-sample-period-aic-clks", None),
    (c"apple,se-engagement-criteria", None),
    (c"apple,se-filter-time-constant", None),
    (c"apple,se-filter-time-constant-1", None),
    (c"apple,se-inactive-threshold", None),
    (c"apple,se-ki", None),
    (c"apple,se-ki-1", None),
    (c"apple,se-kp", None),
    (c"apple,se-kp-1", None),
    (c"apple,se-reset-criteria", None),
    (c"apple,se-target", None),
];
const PERIOD_MS: usize = 0;
const BASE_PSTATE: usize = 1;
const MIN_SRAM_UV: usize = 2;
const CSAFR_MIN_SRAM_UV: usize = 3;
const AVG_TC_MS: usize = 4;
const AVG_KI: usize = 5;
const AVG_KP: usize = 6;
const AVG_MIN_DUTY: usize = 7;
const AVG_TGT_TC: usize = 8;
const FD_KI: usize = 9;
const FD_KP: usize = 10;
const FD_DELTA: usize = 11;
const BOOST_CE: usize = 12;
const BOOST_MIN: usize = 13;
const PERF_DROP: usize = 14;
const PERF_TC: usize = 15;
const PERF_TC2: usize = 16;
const PERF_KI: usize = 17;
const PERF_KI2: usize = 18;
const PERF_MIN_CLAMP: usize = 19;
const PERF_KP: usize = 20;
const PERF_KP2: usize = 21;
const PERF_RESET: usize = 22;
const PERF_TGT: usize = 23;
const PPM_TC_MS: usize = 24;
const PPM_KI: usize = 25;
const PPM_KP: usize = 26;
const PWR_TC: usize = 27;
const PWR_KI: usize = 28;
const PWR_MIN_CLAMP: usize = 29;
const PWR_MIN_DUTY: usize = 30;
const PWR_KP: usize = 31;
const AIC_CLKS: usize = 32;
const SE_CRIT: usize = 33;
const SE_TC: usize = 34;
const SE_TC1: usize = 35;
const SE_INACT: usize = 36;
const SE_KI: usize = 37;
const SE_KI1: usize = 38;
const SE_KP: usize = 39;
const SE_KP1: usize = 40;
const SE_RESET: usize = 41;
const SE_TARGET: usize = 42;

/// One operating point: MHz, the clusters' voltages in mV (rounded up from microvolts) and how
/// many voltage cells the node had.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub(crate) struct Opp {
    pub(crate) mhz: u32,
    pub(crate) mv: [u32; CLUSTERS],
    pub(crate) cells: usize,
}

impl Opp {
    fn max_mv(&self) -> u32 {
        self.mv[..self.cells].iter().copied().max().unwrap_or(0)
    }
}

/// One IOMapping record: slot, physical address and firmware VA (both with the in-page offset),
/// mapped bytes, element bytes, writable.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub(crate) struct IoSlot {
    pub(crate) slot: u8,
    pub(crate) pa: u64,
    pub(crate) va: u64,
    pub(crate) total: u32,
    pub(crate) element: u32,
    pub(crate) writable: bool,
}

/// Everything the record is built from.
pub(crate) struct Inputs<'a> {
    /// SoC identity of this G15C record.
    pub(crate) chip_id: u32,
    pub(crate) props: [u32; PROPS.len()],
    /// `apple,power-zones` cells (triples).
    pub(crate) zones: &'a [u32],
    pub(crate) main: &'a [Opp],
    pub(crate) cs: &'a [Opp],
    pub(crate) afr: &'a [Opp],
    pub(crate) slots: &'a [IoSlot],
    /// Unmapped slots whose flag word the record nevertheless holds as 1.
    pub(crate) empty_writable: &'a [u8],
    /// The TTBAT page (the `ttbs` region).
    pub(crate) ttbat: u64,
    /// Firmware VAs the record repeats: the USC base, the page past the user range, the
    /// timestamp arena.
    pub(crate) usc_base: u64,
    pub(crate) user_end: u64,
    pub(crate) timestamps: u64,
    /// The six sensor rows of the ADT's `aon-ptd` table ([`sensor_rows`]).
    pub(crate) sensor_rows: [[u8; 12]; SENSOR_ROW_COUNT],
    /// The words neither the device tree nor the ADT carries.
    pub(crate) reference: &'a Reference,
}

/// The record's words that neither the device tree nor the ADT carries, as one machine's
/// driver wrote them ([`REFERENCE_WORDS`]).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) struct Reference {
    /// The maximum GPU power in mW ([`MAX_POWER`]).
    pub(crate) max_power_mw: u32,
    /// The relative powers of DVFS states 1..7, percent of the top state's (state 8 is 100).
    pub(crate) relative_power: [u32; RELATIVE_STATES],
    /// The MTR fast-die sensor mask (written twice) and alarm mask, as the u64 at each place.
    /// The device tree's `apple,fast-die0-sensor-mask` is another value.
    pub(crate) fast_die_mask: u64,
    pub(crate) alarm_mask: u64,
}

/// DVFS states with a relative-power word of their own.
pub(crate) const RELATIVE_STATES: usize = 7;

/// The J516C reference machine's words (its 2419 firmware driver's values on that machine).
/// Another J516C gets them as a J514S gets the T6030 templates' machine's: the operating
/// points, voltages, gains and sensor rows are its own, these are the reference machine's.
pub(crate) const J516C: Reference = Reference {
    max_power_mw: 116739,
    relative_power: [11, 23, 33, 44, 55, 71, 90],
    fast_die_mask: 0x802a002b_802a002b,
    alarm_mask: 0x902a102b_802a002b,
};

/// HardwareDataA's maximum-power word (= `fw::initdata::hwdata::MAX_POWER_A`).
pub(crate) const MAX_POWER: usize = HWDATA_A + 0x6a4;
const RELATIVE_POWER: usize = 0x1060;
const FAST_DIE_MASK: usize = HWDATA_A + 0x8ac;
const FAST_DIE_MASK_2: usize = HWDATA_A + 0x1490;
const ALARM_MASK: usize = HWDATA_A + 0x24a8;

/// The record words [`Reference`] supplies. Record offsets.
#[cfg(test)]
#[rustfmt::skip]
pub(crate) const REFERENCE_WORDS: [usize; 14] = [
    0x1060, 0x1064, 0x1068, 0x106c, 0x1070, 0x1074, 0x1078,
    MAX_POWER,
    FAST_DIE_MASK, FAST_DIE_MASK + 4,
    FAST_DIE_MASK_2, FAST_DIE_MASK_2 + 4,
    ALARM_MASK, ALARM_MASK + 4,
];

/// Words the reference machine's record carries that this one leaves zero: the firmware ran,
/// passed every check and reached its top state without them (one boot each). 27 words in
/// the HardwareDataB +0x38..+0x638 block and two all-ones words
/// after it (zero in the G15S templates); the tail's thirteen readings (a single at
/// HardwareDataA +0x6d34, twelve singles after +0x6d80).
#[cfg(test)]
#[rustfmt::skip]
pub(crate) const UNNEEDED: [usize; 42] = [
    0xe0, 0xe8, 0xf4, 0xf8, 0xfc, 0x100, 0x104, 0x108, 0x10c, 0x2d8, 0x2dc, 0x2e0, 0x2e4,
    0x2e8, 0x2ec, 0x3e0, 0x3e8, 0x3f4, 0x3f8, 0x400, 0x40c, 0x5d8, 0x5dc, 0x5e0, 0x5e4,
    0x5e8, 0x5ec, 0x30d8, 0x30f0,
    HWDATA_A + 0x6d34,
    HWDATA_A + 0x6d8c, HWDATA_A + 0x6d90, HWDATA_A + 0x6d94, HWDATA_A + 0x6d98,
    HWDATA_A + 0x6dcc, HWDATA_A + 0x6dd0, HWDATA_A + 0x6dd4, HWDATA_A + 0x6dd8,
    HWDATA_A + 0x6dec, HWDATA_A + 0x6df4, HWDATA_A + 0x6dfc, HWDATA_A + 0x6e04,
];

/// The tail's sensor rows: six rows of {key, id, kind} from [`SENSOR_ROWS`]; the keys of the
/// rows whose kind byte is `g`, their presence mask, then the `r` and `f` rows' keys with a
/// count of one each.
const SENSOR_ROWS: usize = HWDATA_A + 0x6c58;
pub(crate) const SENSOR_ROW_COUNT: usize = 6;
const SENSOR_G_KEYS: usize = HWDATA_A + 0x6cdc;
const SENSOR_G_MASK: usize = HWDATA_A + 0x6cfc;
const SENSOR_R_KEY: usize = HWDATA_A + 0x6d04;
const SENSOR_R_COUNT: usize = HWDATA_A + 0x6d0c;
const SENSOR_F_KEY: usize = HWDATA_A + 0x6d14;
const SENSOR_F_COUNT: usize = HWDATA_A + 0x6d18;

type R<T> = Result<T, &'static str>;

fn get32(b: &[u8], at: usize) -> R<u32> {
    let w = b.get(at..at + 4).ok_or("record too short")?;
    Ok(u32::from_le_bytes([w[0], w[1], w[2], w[3]]))
}

fn put32(b: &mut [u8], at: usize, v: u32) -> R<()> {
    b.get_mut(at..at + 4)
        .ok_or("record too short")?
        .copy_from_slice(&v.to_le_bytes());
    Ok(())
}

fn put64(b: &mut [u8], at: usize, v: u64) -> R<()> {
    b.get_mut(at..at + 8)
        .ok_or("record too short")?
        .copy_from_slice(&v.to_le_bytes());
    Ok(())
}

/// `1 / n` as a single.
fn inv(n: u32) -> R<u32> {
    f32_div(F_1, f32_from_u32(n)).ok_or("a time constant is 0")
}

/// `(n - 1) / n` as a single (the filter's complement, `1 - 1/n`).
fn complement(n: u32) -> R<u32> {
    if n == 0 {
        return Err("a time constant is 0");
    }
    f32_div(f32_from_u32(n - 1), f32_from_u32(n)).ok_or("a time constant is 0")
}

/// `a * b` of device-tree words: refused, not wrapped (and no overflow panic), when it does not
/// fit the record's u32.
fn mul(a: u32, b: u32) -> R<u32> {
    a.checked_mul(b).ok_or(OVERFLOW)
}

const OVERFLOW: &str = "a device-tree value overflows the hardware record's u32 arithmetic";

/// The G15C HardwareDataA offset of a G15S-layout word.
pub(crate) const fn a(off: usize) -> usize {
    let shifted = if off < 0x8bc {
        off
    } else if off < 0x970 {
        off + 8
    } else if off < 0xb70 {
        // The coefficient tables, 16 instead of 8 singles per row: not written here.
        off + 8
    } else if off < 0x1290 {
        off + 0x208
    } else if off < 0x14b4 {
        off + 0x210
    } else if off < 0x1aa0 {
        off + 0xa10
    } else if off < 0x3aa4 {
        off + 0xa18
    } else {
        off + 0x2a18
    };
    HWDATA_A + shifted
}

/// The two voltage-sorted DVFS tables of the operating points: entry 0 is state 0 (off), then
/// per distinct maximum voltage, ascending, the highest-frequency state (table A, `primary`)
/// and the lowest-frequency one (table B, `secondary`). States sharing a voltage must share
/// their cluster voltages; frequencies must rise along both tables and every cluster's voltage
/// along table A.
pub(crate) struct Tables {
    pub(crate) len: usize,
    pub(crate) primary: [u8; STATES],
    pub(crate) secondary: [u8; STATES],
}

impl Tables {
    pub(crate) fn top(&self) -> usize {
        self.len - 1
    }
}

pub(crate) fn tables(main: &[Opp]) -> R<Tables> {
    if main.len() < 2 || main.len() > u8::MAX as usize {
        return Err("operating points: fewer than 2 or more than 255");
    }
    if main[0].mhz != 0 || main[1..].iter().any(|s| s.mhz == 0) {
        return Err("operating points: state 0 is not the only 0 MHz state");
    }
    let mut t = Tables {
        len: 1,
        primary: [0; STATES],
        secondary: [0; STATES],
    };
    let mut last_mv = 0;
    loop {
        let Some(mv) = main[1..]
            .iter()
            .map(Opp::max_mv)
            .filter(|&mv| mv > last_mv)
            .min()
        else {
            break;
        };
        let group = (1..main.len()).filter(|&i| main[i].max_mv() == mv);
        let (mut first, mut hi, mut lo) = (0usize, 0usize, 0usize);
        for i in group {
            if first == 0 {
                first = i;
            }
            if hi == 0 || main[i].mhz > main[hi].mhz {
                hi = i;
            }
            if lo == 0 || main[i].mhz < main[lo].mhz {
                lo = i;
            }
            if main[i].mv != main[first].mv {
                return Err("operating points at one voltage differ per cluster");
            }
        }
        if t.len >= STATES {
            return Err("operating points: more than 16 voltages");
        }
        let (pp, ps) = (
            &main[t.primary[t.len - 1] as usize],
            &main[t.secondary[t.len - 1] as usize],
        );
        if main[hi].mhz <= pp.mhz || main[lo].mhz <= ps.mhz {
            return Err("operating points: frequency does not rise with voltage");
        }
        if t.len > 1 && (0..main[hi].cells).any(|c| main[hi].mv[c] <= pp.mv[c]) {
            return Err("operating points: a cluster's voltage does not rise with the table");
        }
        t.primary[t.len] = hi as u8;
        t.secondary[t.len] = lo as u8;
        t.len += 1;
        last_mv = mv;
    }
    if t.len < 2 {
        return Err("operating points: no usable state");
    }
    Ok(t)
}

/// A voltage lane of a state: the cluster's voltage, or cluster 0's past the node's cells.
fn lane(s: &Opp, j: usize) -> u32 {
    if j < s.cells {
        s.mv[j]
    } else {
        s.mv[0]
    }
}

fn hwdata_b(out: &mut [u8], i: &Inputs<'_>, t: &Tables) -> R<()> {
    let p = &i.props;
    put64(out, 0x0, 0x6f_0000_0000)?;
    put64(out, 0x8, 0xffc0_0000)?;
    put64(out, 0x10, i.usc_base)?;
    put64(out, 0x18, i.usc_base)?;
    put64(out, 0x20, i.user_end)?;
    put64(out, 0x28, i.timestamps)?;
    for s in i.slots {
        if s.slot as usize >= IOMAP_SLOTS {
            return Err("IOMapping slot past the table");
        }
        let r = IOMAPS + 32 * s.slot as usize;
        put64(out, r, s.pa)?;
        put64(out, r + 8, s.va)?;
        put32(out, r + 0x10, s.total)?;
        put32(out, r + 0x14, s.element)?;
        put32(out, r + 0x18, s.writable as u32)?;
    }
    for &slot in i.empty_writable {
        if slot as usize >= IOMAP_SLOTS || i.slots.iter().any(|s| s.slot == slot) {
            return Err("an empty-writable slot is mapped or past the table");
        }
        put32(out, IOMAPS + 32 * slot as usize + 0x18, 1)?;
    }
    // Identity and flags.
    put32(out, 0xa28, i.chip_id)?;
    put32(out, 0xa2c, 1)?;
    put32(out, 0xa30, 2)?;
    for at in [0xa38, 0xa3c, 0xa40, 0xa50, 0xa58, 0xa60] {
        put32(out, at, 1)?;
    }
    put32(out, 0xa68, BASE_CLOCK_KHZ)?;
    put32(out, 0xa6c, 1)?;
    put32(out, 0xa70, p[PERIOD_MS])?;
    put32(out, 0xa78, 1)?;
    put64(out, 0xa7c, 0x1_0000_0001)?;
    put32(out, 0xa9c, 0x1f)?;
    put64(out, 0xabc, 4)?;
    put32(out, 0xacc, 1)?;
    put32(out, 0xad0, 1)?;
    put32(out, 0xae4, 0x31)?;
    put32(out, 0xb04, 1)?;
    put32(out, 0xb20, 0x28)?;
    put32(out, 0xb24, 0xff)?;
    put32(out, 0xb40, 1)?;
    put64(out, 0xb44, i.ttbat)?;
    // GPU core 24 (G15C), revision 4, 40 core slots.
    put32(out, 0xb4c, 24)?;
    put32(out, 0xb50, 4)?;
    put32(out, 0xb54, 40)?;
    put32(out, 0xb58, t.top() as u32)?;
    let min_sram = p[MIN_SRAM_UV].div_ceil(1000);
    let top = &i.main[t.primary[t.top()] as usize];
    let base_index = p[BASE_PSTATE] as usize;
    let base = i
        .main
        .get(base_index)
        .ok_or("perf-base-pstate past the states")?;
    if t.primary[base_index] as usize != base_index {
        return Err("perf-base-pstate is not its own table-A entry");
    }
    if top.mhz <= base.mhz {
        return Err("the top state is not above the base state");
    }
    for k in 0..t.len {
        let ps = &i.main[t.primary[k] as usize];
        let ss = &i.main[t.secondary[k] as usize];
        put32(out, 0xb5c + 4 * k, ps.mhz)?;
        put32(out, 0xf9c + 4 * k, ss.mhz)?;
        put32(out, 0x1350 + 4 * k, ss.mhz)?;
        for j in 0..LANES {
            let v = lane(ps, j);
            put32(out, 0xb9c + 0x20 * k + 4 * j, v)?;
            put32(out, 0xd9c + 0x20 * k + 4 * j, v.max(min_sram))?;
        }
        for j in 0..ss.cells {
            put32(out, 0x1390 + 0x20 * k + 4 * j, ss.mv[j])?;
            put32(out, 0x1590 + 0x20 * k + 4 * j, ss.mv[j].max(min_sram))?;
        }
        put32(out, 0xfdc + 4 * k, F_1_02)?;
        put32(out, 0x10dc + 4 * k, t.primary[k] as u32)?;
        put32(out, 0x111c + 4 * k, t.secondary[k] as u32)?;
        if k > 0 {
            // Relative power (the reference machine's) and relative boost frequency.
            let relative = match i.reference.relative_power.get(k - 1) {
                Some(&r) if k < t.top() => r,
                None if k == t.top() => 100,
                _ => return Err("the reference relative powers are for another state count"),
            };
            put32(out, RELATIVE_POWER - 4 + 4 * k, relative)?;
            if ps.mhz > base.mhz {
                let span = u64::from(top.mhz - base.mhz);
                put32(
                    out,
                    0x109c + 4 * k,
                    (100 * u64::from(ps.mhz - base.mhz) / span) as u32,
                )?;
            }
        }
    }
    put32(out, 0x134c, t.top() as u32)?;
    // The CS and AFR tables: the top state twice, frequencies, voltages, SRAM voltages.
    let cs_sram = p[CSAFR_MIN_SRAM_UV] / 1000;
    let mut aux = |base: usize, tab: &[Opp], n: usize| -> R<usize> {
        if tab.is_empty() || tab.len() > n {
            return Err("CS/AFR table empty or longer than its layout");
        }
        let top = (tab.len() - 1) as u32;
        put32(out, base, top)?;
        put32(out, base + 4, top)?;
        let (fr, vo, sr) = (base + 8, base + 8 + 4 * n, base + 8 + 12 * n);
        for (k, s) in tab.iter().enumerate() {
            put32(out, fr + 4 * k, s.mhz)?;
            for j in 0..s.cells.min(2) {
                put32(out, vo + 8 * k + 4 * j, s.mv[j])?;
                put32(out, sr + 8 * k + 4 * j, s.mv[j].max(cs_sram))?;
            }
        }
        Ok(sr + 8 * n)
    };
    let end = aux(0x115c, i.cs, CS_STATES)?;
    if end != 0x12a4 {
        return Err("CS table layout");
    }
    let end = aux(0x12a4, i.afr, AFR_STATES)?;
    if end != 0x134c {
        return Err("AFR table layout");
    }
    put32(out, 0x17b8, 5)?;
    put32(out, 0x17c0, 3)?;
    put32(out, 0x17c4, 7)?;
    put32(out, 0x17c8, 7)?;
    for at in [
        0x17e8, 0x17fc, 0x1800, 0x1804, 0x1808, 0x1810, 0x1814, 0x1818, 0x1860,
    ] {
        put32(out, at, 1)?;
    }
    for at in (0x1820..0x1850).step_by(4) {
        put32(out, at, u32::MAX)?;
    }
    Ok(())
}

fn hwdata_a(out: &mut [u8], i: &Inputs<'_>, t: &Tables) -> R<()> {
    let p = &i.props;
    let period_ms = p[PERIOD_MS];
    if period_ms == 0 {
        return Err("the power sample period is 0");
    }
    let period_s = f32_div(f32_from_u32(period_ms), f32_from_u32(1000)).ok_or("period")?;
    let per_period =
        |gain: usize| f32_mul(p[gain], period_s).ok_or("a gain is not a finite normal single");
    let max_scaled = 100 * t.top() as u32;
    let base_scaled = mul(100, p[BASE_PSTATE])?;
    let max_power = i.reference.max_power_mw;
    let max_power_f = f32_from_u32(max_power);
    let mut w = |off: usize, v: u32| put32(out, a(off), v);
    w(0x4, p[AIC_CLKS])?;
    w(0x8, p[AIC_CLKS])?;
    w(0x10, 4)?;
    w(0x14, F_1)?;
    w(0x2c, 1)?;
    w(0x30, 1)?;
    w(0x44, base_scaled)?;
    w(0x48, 1)?;
    w(0x4c, max_scaled)?;
    w(0x54, 100)?;
    for k in 0..t.len {
        w(0x80 + 4 * k, F_1_02)?;
    }
    w(0x658, 625)?;
    // Power controller.
    w(0x664, complement(p[PWR_TC])?)?;
    w(0x66c, inv(p[PWR_TC])?)?;
    w(0x674, p[PWR_KI])?;
    w(0x67c, f32_from_u32(p[PWR_MIN_CLAMP]))?;
    w(0x680, max_power_f)?;
    w(0x684, p[PWR_KP])?;
    let k = f32_div(f32_from_u32(max_scaled), max_power_f).ok_or("maximum power is 0")?;
    w(0x68c, k ^ 0x8000_0000)?;
    w(0x690, p[PWR_MIN_DUTY].wrapping_sub(max_scaled))?;
    w(0x698, max_scaled)?;
    w(0x6a4, max_power)?;
    w(0x6d4, max_scaled)?;
    // PPM controller.
    let ppm_tc = p[PPM_TC_MS] / period_ms;
    w(0x6f0, mul(4, ppm_tc)?)?;
    w(0x6fc, complement(ppm_tc)?)?;
    w(0x704, inv(ppm_tc)?)?;
    w(0x70c, per_period(PPM_KI)?)?;
    w(0x718, F_65536)?;
    w(0x71c, p[PPM_KP])?;
    w(0x728, p[PWR_MIN_DUTY])?;
    w(0x72c, max_scaled)?;
    w(0x73c, max_power)?;
    w(0x744, p[PPM_TC_MS])?;
    w(0x74c, mul(mul(ppm_tc, period_ms)?, BASE_CLOCK_KHZ)?)?;
    // Performance controller.
    w(0x788, p[PERF_TGT])?;
    w(0x790, p[BOOST_MIN])?;
    w(0x794, p[BOOST_CE])?;
    w(0x798, p[PERF_RESET])?;
    w(0x7a0, 6)?;
    w(0x7a4, 1)?;
    w(0x7a8, p[PERF_DROP])?;
    w(0x7ac, complement(p[PERF_TC])?)?;
    w(0x7b0, complement(p[PERF_TC2])?)?;
    w(0x7b4, inv(p[PERF_TC])?)?;
    w(0x7b8, inv(p[PERF_TC2])?)?;
    w(0x7bc, p[PERF_KI])?;
    w(0x7c0, p[PERF_KI2])?;
    w(0x7c4, f32_from_u32(p[PERF_MIN_CLAMP]))?;
    w(0x7c8, F_95)?;
    w(0x7cc, p[PERF_KP])?;
    w(0x7d0, p[PERF_KP2])?;
    let boost = (t.top() as u32)
        .checked_sub(p[BASE_PSTATE])
        .ok_or("perf-base-pstate above the top state")?;
    w(0x7d4, f32_div(f32_from_u32(boost), F_0_95).ok_or("boost")?)?;
    w(0x7d8, base_scaled)?;
    w(0x7dc, max_scaled)?;
    w(0x7e0, base_scaled)?;
    w(0x7ec, p[PERF_TGT])?;
    w(0x81c, base_scaled)?;
    w(0x838, F_65536)?;
    w(0x83c, f32_from_u32(p[PWR_MIN_DUTY]))?;
    w(0x840, f32_from_u32(max_scaled))?;
    w(0x844, i.main.iter().map(|s| s.mhz).max().unwrap_or(0))?;
    w(0x854, p[PWR_MIN_DUTY])?;
    w(0x860, F_100)?;
    w(0x864, max_scaled)?;
    w(0x86c, F_0_8)?;
    w(0x870, F_0_2)?;
    // Fast-die controller (its sensor mask follows below).
    w(0x8bc, 4)?;
    w(0x8d0, F_1)?;
    w(0x8d8, per_period(FD_KI)?)?;
    w(0x8e4, F_65536)?;
    w(0x8e8, p[FD_KP])?;
    w(0x8f4, p[PWR_MIN_DUTY])?;
    w(0x8f8, max_scaled)?;
    w(0x8fc, max_scaled)?;
    w(0x900, mul(100, p[FD_DELTA])?)?;
    w(0x908, 11000)?;
    w(0x938, max_scaled)?;
    w(0xbe0, max_scaled)?;
    // Power zones and the average-power controller.
    w(0xe80, 1)?;
    w(0xe84, (i.zones.len() / 3) as u32)?;
    for at in [0xe88, 0xe8c, 0xe90] {
        w(at, max_power)?;
    }
    let tgt_tc = p[AVG_TGT_TC];
    w(0xe9c, complement(tgt_tc)?)?;
    w(0xea0, inv(tgt_tc)?)?;
    w(0xea4, mul(4, tgt_tc)?)?;
    w(0xea8, mul(period_ms, tgt_tc)?)?;
    w(0xeac, mul(mul(period_ms, tgt_tc)?, BASE_CLOCK_KHZ)?)?;
    if i.zones.len() % 3 != 0 || i.zones.len() > 15 {
        return Err("the power zones are not up to five u32 triples");
    }
    for (z, zone) in i.zones.chunks_exact(3).enumerate() {
        let at = 0xeb4 + 0x28 * z;
        let (target, offset, tc) = (zone[0], zone[1], zone[2]);
        w(at + 0x4, target)?;
        w(
            at + 0x8,
            target
                .checked_sub(offset)
                .ok_or("a power zone's offset is larger than its target")?,
        )?;
        w(at + 0xc, mul(4, tc)?)?;
        w(at + 0x10, mul(period_ms, tc)?)?;
        w(at + 0x14, 1_320_000_000)?;
        w(at + 0x1c, complement(tc)?)?;
        w(at + 0x20, inv(tc)?)?;
    }
    let avg_tc = p[AVG_TC_MS] / period_ms;
    w(0xf7c, mul(4, avg_tc)?)?;
    w(0xf88, complement(avg_tc)?)?;
    w(0xf90, inv(avg_tc)?)?;
    w(0xf98, per_period(AVG_KI)?)?;
    w(0xfa4, F_65536)?;
    w(0xfa8, p[AVG_KP])?;
    w(0xfb4, p[AVG_MIN_DUTY])?;
    w(0xfb8, max_scaled)?;
    w(0xfbc, max_scaled)?;
    w(0xfc4, max_power_f)?;
    w(0xfc8, max_power)?;
    w(0xfd0, p[AVG_TC_MS])?;
    w(0xfd8, mul(mul(avg_tc, period_ms)?, BASE_CLOCK_KHZ)?)?;
    w(0xff8, max_scaled)?;
    // The shader-engine controller block at +0x10b4.
    const SE: usize = 0x10b4;
    let clocks = mul(BASE_CLOCK_KHZ, period_ms)?;
    w(SE + 0x40, 50)?;
    w(SE + 0x44, 1)?;
    w(SE + 0x4c, complement(p[SE_TC])?)?;
    w(SE + 0x50, complement(p[SE_TC1])?)?;
    w(SE + 0x54, inv(p[SE_TC])?)?;
    w(SE + 0x58, inv(p[SE_TC1])?)?;
    w(SE + 0x5c, per_period(SE_KI)?)?;
    w(SE + 0x60, per_period(SE_KI1)?)?;
    w(SE + 0x68, F_65536)?;
    w(SE + 0x6c, p[SE_KP])?;
    w(SE + 0x70, p[SE_KP1])?;
    w(SE + 0x78, 100)?;
    w(SE + 0x7c, max_scaled)?;
    w(SE + 0x80, 100)?;
    w(SE + 0x88, F_8000)?;
    w(SE + 0x8c, p[SE_TARGET])?;
    w(SE + 0x94, mul(p[SE_TC], period_ms)?)?;
    w(SE + 0x98, mul(p[SE_TC1], period_ms)?)?;
    w(SE + 0x9c, mul(p[SE_TC], clocks)?)?;
    w(SE + 0xa4, mul(p[SE_TC1], clocks)?)?;
    w(SE + 0xb0, F_65536)?;
    w(SE + 0xd8, 8000)?;
    w(SE + 0xdc, 4)?;
    w(SE + 0xe0, p[SE_INACT])?;
    w(SE + 0xe4, p[SE_CRIT])?;
    w(SE + 0xe8, 2)?;
    w(SE + 0xec, 4)?;
    w(SE + 0xf0, p[SE_RESET])?;
    w(SE + 0x11c, F_65536)?;
    w(SE + 0x12c, 40)?;
    w(SE + 0x130, max_scaled)?;
    // Past the second sensor mask, the alarm mask and the end flag.
    w(0x1290, 125)?;
    w(0x1294, 1)?;
    w(0x1aa0, 1)?;
    w(0x3aa4, F_5)?;
    // The MTR sensor masks: the fast-die mask at both places, then the alarm mask.
    put64(out, FAST_DIE_MASK, i.reference.fast_die_mask)?;
    put64(out, FAST_DIE_MASK_2, i.reference.fast_die_mask)?;
    put64(out, ALARM_MASK, i.reference.alarm_mask)?;
    Ok(())
}

/// The six sensor rows of the ADT's `aon-ptd` `ptd_data_table`: 12-byte entries of a
/// little-endian four-character key (`RT0g` is stored as `"RT0g"`) and two words. The rows are
/// the `RT` entries of kind `g` (four, in table order), then the one of kind `r` and the one
/// of kind `f`; any other count is refused rather than guessed.
pub(crate) fn sensor_rows(table: &[u8]) -> R<[[u8; 12]; SENSOR_ROW_COUNT]> {
    if table.is_empty() || table.len() % 12 != 0 {
        return Err("the aon-ptd sensor table is not whole 12-byte entries");
    }
    let mut rows = [[0u8; 12]; SENSOR_ROW_COUNT];
    let mut n = 0;
    for (kind, wanted) in [(b'g', 4), (b'r', 1), (b'f', 1)] {
        let mut found = 0;
        for e in table.chunks_exact(12) {
            if e[..2] == *b"RT" && e[3] == kind {
                let row = rows.get_mut(n).ok_or("too many GPU sensor rows in aon-ptd")?;
                row.copy_from_slice(e);
                n += 1;
                found += 1;
            }
        }
        if found != wanted {
            return Err("aon-ptd does not hold four g, one r and one f GPU sensor row");
        }
    }
    Ok(rows)
}

/// The tail's sensor keys from the six rows.
fn tail(out: &mut [u8]) -> R<()> {
    let mut g = 0usize;
    for row in 0..SENSOR_ROW_COUNT {
        let key = get32(out, SENSOR_ROWS + 12 * row)?;
        match (key >> 24) as u8 {
            b'g' => {
                put32(out, SENSOR_G_KEYS + 4 * g, key)?;
                g += 1;
            }
            b'r' => {
                put32(out, SENSOR_R_KEY, key)?;
                put32(out, SENSOR_R_COUNT, 1)?;
            }
            b'f' => {
                put32(out, SENSOR_F_KEY, key)?;
                put32(out, SENSOR_F_COUNT, 1)?;
            }
            _ => return Err("a sensor row has an unknown kind"),
        }
    }
    if g > 4 {
        return Err("more than four g sensor rows");
    }
    put32(out, SENSOR_G_MASK, (1u32 << g) - 1)?;
    put32(out, HWDATA_A + 0x6d80, 1)?;
    put32(out, HWDATA_A + 0x6d84, 1)?;
    Ok(())
}

/// The first word at which a built record differs from a machine's own record (a host test's
/// fixture), except the [`UNNEEDED`] words, where the machine's value is accepted as long as
/// the built word is zero.
#[cfg(test)]
pub(crate) fn cross_check(built: &[u8], file: &[u8]) -> Option<(usize, u32, u32)> {
    let word = |s: &[u8], at: usize| -> u32 {
        let mut w = [0u8; 4];
        for (i, out) in w.iter_mut().enumerate() {
            *out = s.get(at + i).copied().unwrap_or(0);
        }
        u32::from_le_bytes(w)
    };
    (0..built.len().max(file.len()))
        .step_by(4)
        .map(|at| (at, word(built, at), word(file, at)))
        .find(|&(at, b, f)| b != f && !(b == 0 && UNNEEDED.contains(&at)))
}

/// Builds the record into `out` (zeroed first) from the inputs.
pub(crate) fn build(out: &mut [u8], i: &Inputs<'_>) -> R<()> {
    if !matches!(i.chip_id, 0x6031 | 0x6034) {
        return Err("unsupported G15C record SoC");
    }
    if out.len() != SIZE {
        return Err("record of another size");
    }
    out.fill(0);
    let t = tables(i.main)?;
    hwdata_b(out, i, &t)?;
    hwdata_a(out, i, &t)?;
    for (k, row) in i.sensor_rows.iter().enumerate() {
        let at = SENSOR_ROWS + 12 * k;
        out.get_mut(at..at + 12)
            .ok_or("record too short")?
            .copy_from_slice(row);
    }
    tail(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn get(b: &[u8], o: usize) -> u32 {
        get32(b, o).unwrap()
    }

    fn opp(mhz: u32, mv: [u32; 4]) -> Opp {
        Opp { mhz, mv, cells: 4 }
    }

    /// Operating points shaped like the J516C's: eight voltages, the upper five with two states.
    fn main_opps() -> Vec<Opp> {
        let mut v = vec![opp(0, [125; 4])];
        let groups: [(u32, [u32; 4], &[u32]); 8] = [
            (605, [605, 605, 615, 615], &[338]),
            (680, [680, 680, 665, 665], &[618]),
            (730, [730, 730, 715, 715], &[796]),
            (770, [770, 770, 755, 755], &[832, 924]),
            (800, [800, 800, 785, 785], &[952, 1056]),
            (860, [855, 860, 845, 845], &[1064, 1182]),
            (930, [915, 930, 915, 915], &[1182, 1312]),
            (955, [940, 955, 940, 940], &[1242, 1380]),
        ];
        for (_, mv, mhz) in groups {
            for &m in mhz {
                v.push(opp(m, mv));
            }
        }
        v
    }

    fn props() -> [u32; PROPS.len()] {
        let mut p = [0u32; PROPS.len()];
        p[PERIOD_MS] = 8;
        p[BASE_PSTATE] = 1;
        p[MIN_SRAM_UV] = 815000;
        p[CSAFR_MIN_SRAM_UV] = 815000;
        p[AVG_TC_MS] = 32;
        p[AVG_KI] = 0x41480000;
        p[AVG_KP] = 0x3e99999a;
        p[AVG_MIN_DUTY] = 40;
        p[AVG_TGT_TC] = 1;
        p[FD_KI] = 0x443b8000;
        p[FD_KP] = 0x42080000;
        p[PERF_TC] = 5;
        p[PERF_TC2] = 200;
        p[PERF_RESET] = 6;
        p[PPM_TC_MS] = 34;
        p[PPM_KI] = 0x4198cccd;
        p[PPM_KP] = 0x3e99999a;
        p[PWR_TC] = 313;
        p[PWR_MIN_DUTY] = 40;
        p[AIC_CLKS] = 200000;
        p[SE_TC] = 9;
        p[SE_TC1] = 3;
        p[SE_KI] = (-50.0f32).to_bits();
        p[SE_KI1] = (-100.0f32).to_bits();
        p[SE_TARGET] = 1400;
        p
    }

    /// Arbitrary reference words, distinct from the J516C's.
    const TEST_REFERENCE: Reference = Reference {
        max_power_mw: 54321,
        relative_power: [0x1000, 0x1001, 0x1002, 0x1003, 0x1004, 0x1005, 0x1006],
        fast_die_mask: 0x1111_2222_3333_4444,
        alarm_mask: 0x5555_6666_7777_8888,
    };

    /// A sensor row: the key's characters in storage order, then two words.
    fn row(key: &[u8; 4], id: u32, kind: u32) -> [u8; 12] {
        let mut r = [0u8; 12];
        r[..4].copy_from_slice(key);
        r[4..8].copy_from_slice(&id.to_le_bytes());
        r[8..].copy_from_slice(&kind.to_le_bytes());
        r
    }

    fn rows() -> [[u8; 12]; SENSOR_ROW_COUNT] {
        [b"RT0g", b"RT1g", b"RT2g", b"RT3g", b"RT0r", b"RT0f"].map(|k| row(k, 0x10001, 0x1401))
    }

    fn inputs<'a>(main: &'a [Opp], cs: &'a [Opp], slots: &'a [IoSlot]) -> Inputs<'a> {
        Inputs {
            chip_id: 0x6031,
            props: props(),
            zones: &[],
            main,
            cs,
            afr: cs,
            slots,
            empty_writable: &[4, 6],
            ttbat: 0x10bfffb8000,
            usc_base: 0x10_0000_0000,
            user_end: 0x2ff_ffff_8000,
            timestamps: 0xfffffc2ffc000000,
            sensor_rows: rows(),
            reference: &TEST_REFERENCE,
        }
    }

    #[test]
    fn t6034_identity_and_six_memory_channels() {
        let main = main_opps();
        let cs = [opp(24, [635; CLUSTERS])];
        let slots = [IoSlot {
            slot: 11,
            pa: 0x220000000,
            va: 0xfffffc2068000000,
            total: 6 * 0xd8000,
            element: 0xd8000,
            writable: true,
        }];
        let mut input = inputs(&main, &cs, &slots);
        input.chip_id = 0x6034;
        let mut out = vec![0u8; SIZE];
        build(&mut out, &input).unwrap();
        assert_eq!(get(&out, 0xa28), 0x6034);
        assert_eq!(get(&out, IOMAPS + 32 * 11 + 0x10), 6 * 0xd8000);
        assert_eq!(get(&out, IOMAPS + 32 * 11 + 0x14), 0xd8000);
        // The fused 30-core part still has four ten-slot physical clusters.
        assert_eq!(get(&out, 0xb54), 40);
        input.chip_id = 0x6030;
        assert_eq!(build(&mut out, &input), Err("unsupported G15C record SoC"));
    }

    #[test]
    fn factors() {
        assert_eq!(F_1, 1.0f32.to_bits());
        assert_eq!(F_0_2, 0.2f32.to_bits());
        assert_eq!(F_0_8, 0.8f32.to_bits());
        assert_eq!(F_0_95, 0.95f32.to_bits());
        assert_eq!(F_1_02, 1.02f32.to_bits());
        assert_eq!(F_5, 5.0f32.to_bits());
        assert_eq!(F_95, 95.0f32.to_bits());
        assert_eq!(F_100, 100.0f32.to_bits());
        assert_eq!(F_8000, 8000.0f32.to_bits());
        assert_eq!(F_65536, 65536.0f32.to_bits());
        // The filter complement is the correctly rounded (n - 1) / n, also where 1 - 1/n is
        // a rounding tie.
        assert_eq!(complement(3).unwrap(), (2.0f32 / 3.0).to_bits());
        assert_eq!(complement(313).unwrap(), (312.0f32 / 313.0).to_bits());
        assert_eq!(inv(9).unwrap(), (1.0f32 / 9.0).to_bits());
        assert!(inv(0).is_err());
    }

    #[test]
    fn unneeded() {
        // No UNNEEDED word is a reference or sensor-row word; the cross-check accepts the
        // file's value there only against a zero built word, and nothing else.
        let rows = SENSOR_ROWS..SENSOR_ROWS + 12 * SENSOR_ROW_COUNT;
        assert!(UNNEEDED
            .iter()
            .all(|at| !REFERENCE_WORDS.contains(at) && !rows.contains(at)));
        assert!(UNNEEDED.iter().all(|&at| at % 4 == 0 && at + 4 <= SIZE));
        let mut file = vec![0u8; SIZE];
        for (k, &at) in UNNEEDED.iter().enumerate() {
            put32(&mut file, at, 0x2000 + k as u32).unwrap();
        }
        let built = vec![0u8; SIZE];
        assert_eq!(cross_check(&built, &file), None);
        let mut built = built;
        put32(&mut built, UNNEEDED[0], 1).unwrap();
        assert_eq!(cross_check(&built, &file), Some((UNNEEDED[0], 1, 0x2000)));
        let mut built = vec![0u8; SIZE];
        put32(&mut built, 0x10, 5).unwrap();
        assert_eq!(cross_check(&built, &file), Some((0x10, 5, 0)));
    }

    #[test]
    fn layout() {
        assert_eq!(a(0x8b8), HWDATA_A + 0x8b8);
        assert_eq!(a(0x8bc), HWDATA_A + 0x8c4);
        assert_eq!(a(0x938), HWDATA_A + 0x940);
        assert_eq!(a(0xbe0), HWDATA_A + 0xde8);
        assert_eq!(a(0x1288), HWDATA_A + 0x1490);
        assert_eq!(a(0x1290), HWDATA_A + 0x14a0);
        assert_eq!(a(0x1a98), HWDATA_A + 0x24a8);
        assert_eq!(a(0x1aa0), HWDATA_A + 0x24b8);
        assert_eq!(a(0x3aa4), HWDATA_A + 0x64bc);
        assert_eq!(MAX_POWER, a(0x6a4));
        // REFERENCE_WORDS is sorted, unique and inside the record.
        assert!(REFERENCE_WORDS.windows(2).all(|w| w[0] < w[1]));
        assert!(REFERENCE_WORDS.iter().all(|&o| o % 4 == 0 && o + 4 <= SIZE));
        assert!(SENSOR_ROWS + 12 * SENSOR_ROW_COUNT <= SENSOR_G_KEYS);
    }

    #[test]
    fn dvfs_tables() {
        let main = main_opps();
        let t = tables(&main).unwrap();
        assert_eq!(t.len, 9);
        assert_eq!(&t.primary[..9], &[0, 1, 2, 3, 5, 7, 9, 11, 13]);
        assert_eq!(&t.secondary[..9], &[0, 1, 2, 3, 4, 6, 8, 10, 12]);
        // One state: nothing to sort.
        assert!(tables(&main[..1]).is_err());
        // Frequency falling with voltage.
        let mut bad = main.clone();
        bad[2].mhz = 300;
        assert!(tables(&bad).is_err());
        // Two states at one voltage with different cluster voltages.
        let mut bad = main.clone();
        bad[5].mv[2] = 760;
        assert!(tables(&bad).is_err());
        // A cluster whose voltage does not rise.
        let mut bad = main.clone();
        bad[13].mv[3] = 915;
        assert!(tables(&bad).is_err());
    }

    #[test]
    fn record_words() {
        let main = main_opps();
        let cs: Vec<Opp> = [24u32, 480, 624, 810, 1024, 1140, 1236]
            .iter()
            .zip([635, 635, 680, 720, 790, 855, 875])
            .map(|(&mhz, mv)| Opp {
                mhz,
                mv: [mv, 0, 0, 0],
                cells: 1,
            })
            .collect();
        let slots = [
            IoSlot {
                slot: 0,
                pa: 0x408d00000,
                va: 0xfffffc2068000000,
                total: 0x144000,
                element: 0x144000,
                writable: true,
            },
            IoSlot {
                slot: 1,
                pa: 0x20e101000,
                va: 0xfffffc2068149000,
                total: 0x4000,
                element: 0x4000,
                writable: false,
            },
        ];
        let i = inputs(&main, &cs, &slots);
        // Whatever the buffer held before is not part of the record.
        let mut r = vec![0x5au8; SIZE];
        build(&mut r, &i).unwrap();
        let good = r.clone();
        // The reference words: relative powers of states 1..7, the maximum power, the masks.
        for k in 0..RELATIVE_STATES {
            assert_eq!(get(&r, 0x1060 + 4 * k), 0x1000 + k as u32);
        }
        assert_eq!(get(&r, FAST_DIE_MASK), 0x3333_4444);
        assert_eq!(get(&r, FAST_DIE_MASK + 4), 0x1111_2222);
        assert_eq!(get(&r, FAST_DIE_MASK_2), 0x3333_4444);
        assert_eq!(get(&r, FAST_DIE_MASK_2 + 4), 0x1111_2222);
        assert_eq!(get(&r, ALARM_MASK), 0x7777_8888);
        assert_eq!(get(&r, ALARM_MASK + 4), 0x5555_6666);
        // The sensor rows as given.
        assert_eq!(&r[SENSOR_ROWS..SENSOR_ROWS + 12], &rows()[0]);
        assert_eq!(&r[SENSOR_ROWS + 60..SENSOR_ROWS + 72], &rows()[5]);
        assert_eq!(get(&r, 0x30f0), 0);
        // Identity, IOMapping records, flags.
        assert_eq!(get(&r, 0xa28), 0x6031);
        assert_eq!(get(&r, 0x640), 0x08d00000);
        assert_eq!(get(&r, 0x644), 4);
        assert_eq!(get(&r, 0x648), 0x68000000);
        assert_eq!(get(&r, 0x658), 1);
        assert_eq!(get(&r, 0x660), 0x0e101000);
        assert_eq!(get(&r, 0x678), 0);
        assert_eq!(get(&r, 0x640 + 32 * 4 + 0x18), 1);
        assert_eq!(get(&r, 0x640 + 32 * 5 + 0x18), 0);
        assert_eq!(get(&r, 0xb44), 0xfffb8000);
        assert_eq!(get(&r, 0xb48), 0x10b);
        assert_eq!(get(&r, 0xb58), 8);
        // Tables: A frequencies, B frequencies, per-cluster lanes, SRAM floor, index maps.
        assert_eq!(get(&r, 0xb5c + 4 * 8), 1380);
        assert_eq!(get(&r, 0xf9c + 4 * 8), 1242);
        assert_eq!(get(&r, 0x1350 + 4 * 4), 832);
        assert_eq!(get(&r, 0xb9c + 0x20 + 4 * 2), 615);
        assert_eq!(get(&r, 0xb9c + 0x20 + 4 * 5), 605);
        assert_eq!(get(&r, 0xd9c + 0x20 + 4 * 2), 815);
        assert_eq!(get(&r, 0x1390 + 0x20 * 8 + 4 * 1), 955);
        assert_eq!(get(&r, 0x10dc + 4 * 8), 13);
        assert_eq!(get(&r, 0x111c + 4 * 8), 12);
        assert_eq!(get(&r, 0x107c), 100);
        assert_eq!(get(&r, 0x109c + 4 * 8), 100);
        // CS table: top twice, then the arrays with the SRAM floor.
        assert_eq!(get(&r, 0x115c), 6);
        assert_eq!(get(&r, 0x1160), 6);
        assert_eq!(get(&r, 0x1164), 24);
        assert_eq!(get(&r, 0x11a4), 635);
        assert_eq!(get(&r, 0x1224), 815);
        assert_eq!(get(&r, 0x12a4), 6);
        assert_eq!(get(&r, 0x12ac), 24);
        // HardwareDataA: scaled states, the maximum power and its derivatives, filters, SE.
        assert_eq!(get(&r, a(0x4c)), 800);
        assert_eq!(get(&r, a(0x680)), 54321.0f32.to_bits());
        assert_eq!(get(&r, a(0x68c)), (-800.0f32 / 54321.0).to_bits());
        assert_eq!(get(&r, a(0x690)), (40i32 - 800) as u32);
        assert_eq!(get(&r, MAX_POWER), 54321);
        assert_eq!(get(&r, a(0x6fc)), (3.0f32 / 4.0).to_bits());
        assert_eq!(get(&r, a(0x74c)), 4 * 8 * 24000);
        assert_eq!(get(&r, a(0x7d4)), (7.0f32 / 0.95).to_bits());
        assert_eq!(get(&r, a(0x844)), 1380);
        assert_eq!(get(&r, a(0x8d8)), 0x40c00001);
        assert_eq!(get(&r, a(0x8e8)), 0x42080000);
        assert_eq!(get(&r, a(0xe88)), 54321);
        assert_eq!(get(&r, a(0xf88)), 0x3f400000);
        assert_eq!(get(&r, a(0xfd8)), 4 * 8 * 24000);
        assert_eq!(get(&r, a(0x10b4 + 0x4c)), (8.0f32 / 9.0).to_bits());
        assert_eq!(get(&r, a(0x10b4 + 0x5c)), (-50.0f32 * 0.008).to_bits());
        assert_eq!(get(&r, a(0x10b4 + 0x9c)), 9 * 192000);
        assert_eq!(get(&r, a(0x3aa4)), F_5);
        // Tail keys.
        assert_eq!(get(&r, SENSOR_G_KEYS), u32::from_le_bytes(*b"RT0g"));
        assert_eq!(get(&r, SENSOR_G_KEYS + 12), u32::from_le_bytes(*b"RT3g"));
        assert_eq!(get(&r, SENSOR_G_MASK), 0xf);
        assert_eq!(get(&r, SENSOR_R_KEY), u32::from_le_bytes(*b"RT0r"));
        assert_eq!(get(&r, SENSOR_F_COUNT), 1);
        // Refusals.
        let mut short = vec![0u8; SIZE - 4];
        assert!(build(&mut short, &i).is_err());
        let mut j = inputs(&main, &cs, &slots);
        j.props[PERIOD_MS] = 0;
        assert!(build(&mut r, &j).is_err());
        let mut j = inputs(&main, &cs, &slots);
        j.empty_writable = &[0];
        assert!(build(&mut r, &j).is_err());
        let zero = Reference {
            max_power_mw: 0,
            ..TEST_REFERENCE
        };
        let mut j = inputs(&main, &cs, &slots);
        j.reference = &zero;
        assert!(build(&mut r, &j).is_err());
        let mut j = inputs(&main, &cs, &slots);
        j.sensor_rows[0] = row(b"RT0x", 1, 1);
        assert!(build(&mut r, &j).is_err());
        let mut j = inputs(&main, &cs, &slots);
        j.zones = &[100, 101, 1];
        assert!(build(&mut r, &j).is_err());
        // Device-tree values whose arithmetic leaves u32 are refused, not wrapped or panicking:
        // each on a copy of the record that built, so that nothing else refuses it first.
        let refused = |j: &Inputs<'_>| {
            let mut x = good.clone();
            build(&mut x, j)
        };
        assert_eq!(refused(&i), Ok(()));
        for (prop, value) in [
            (AVG_TGT_TC, 0x4000_0000),
            (PPM_TC_MS, u32::MAX),
            (AVG_TC_MS, u32::MAX),
            (SE_TC, 0x2000_0000),
            (SE_TC1, 0x2000_0000),
            (FD_DELTA, 0x0300_0000),
        ] {
            let mut j = inputs(&main, &cs, &slots);
            j.props[prop] = value;
            assert_eq!(refused(&j), Err(OVERFLOW), "prop {prop} = {value:#x}");
        }
        let mut j = inputs(&main, &cs, &slots);
        j.zones = &[5000, 200, 0x4000_0000];
        assert_eq!(refused(&j), Err(OVERFLOW));
        // The seven relative powers fit eight states above state 0 and nothing else.
        let fewer = &main[..PERF_STATES - 2];
        assert_eq!(tables(fewer).unwrap().top(), 7);
        assert_eq!(
            refused(&inputs(fewer, &cs, &slots)),
            Err("the reference relative powers are for another state count")
        );
    }

    #[test]
    fn sensor_rows_from_the_ptd_table() {
        // A table shaped like the J516C's: other sensors around the GPU's, the f row first.
        let mut table = Vec::new();
        for r in [
            row(b"YD0C", 0x5e0001, 0x1040405),
            row(b"RT0a", 0x6b0001, 0x13),
            row(b"RT0f", 0xcb0001, 0x1601),
            row(b"RE0g", 0xaa0001, 0xff01),
            row(b"RT0g", 0xab0001, 0x1401),
            row(b"RT1g", 0x8d0001, 0x1401),
            row(b"RT2g", 0x8b0001, 0x1401),
            row(b"RT3g", 0xad0001, 0x1401),
            row(b"RT0p", 0x6d0001, 0x13),
            row(b"RT0r", 0xeb0001, 0x1401),
            row(b"XEXE", 0x3c0001, 0xff00),
        ] {
            table.extend_from_slice(&r);
        }
        let got = sensor_rows(&table).unwrap();
        let keys: Vec<&[u8]> = got.iter().map(|r| &r[..4]).collect();
        assert_eq!(keys, [b"RT0g", b"RT1g", b"RT2g", b"RT3g", b"RT0r", b"RT0f"]);
        assert_eq!(got[5], row(b"RT0f", 0xcb0001, 0x1601));
        assert_eq!(got[4], row(b"RT0r", 0xeb0001, 0x1401));
        // Refused: no table, a partial entry, a missing row, a fifth g row, a second f row.
        assert!(sensor_rows(&[]).is_err());
        assert!(sensor_rows(&table[..table.len() - 1]).is_err());
        assert!(sensor_rows(&table[12..table.len() - 12]).is_ok());
        assert!(sensor_rows(&table[..9 * 12]).is_err());
        for extra in [row(b"RT4g", 1, 1), row(b"RT1f", 1, 1)] {
            let mut t = table.clone();
            t.extend_from_slice(&extra);
            assert!(sensor_rows(&t).is_err());
        }
    }

    // --- The fixture: a saved copy of a J516C's device tree and its private board file. ---

    fn be32(b: &[u8], o: usize) -> u32 {
        u32::from_be_bytes(b[o..o + 4].try_into().unwrap())
    }

    /// All nodes of a flattened device tree: path -> properties.
    fn fdt(dtb: &[u8]) -> Vec<(String, Vec<(String, Vec<u8>)>)> {
        assert_eq!(be32(dtb, 0), 0xd00dfeed);
        let strings = be32(dtb, 12) as usize;
        let mut at = be32(dtb, 8) as usize;
        let mut path: Vec<String> = Vec::new();
        let mut out: Vec<(String, Vec<(String, Vec<u8>)>)> = Vec::new();
        loop {
            let token = be32(dtb, at);
            at += 4;
            match token {
                1 => {
                    let end = dtb[at..].iter().position(|&b| b == 0).unwrap();
                    path.push(String::from_utf8(dtb[at..at + end].to_vec()).unwrap());
                    at = (at + end + 1).next_multiple_of(4);
                    out.push((path.join("/"), Vec::new()));
                }
                2 => {
                    path.pop();
                }
                3 => {
                    let len = be32(dtb, at) as usize;
                    let name_off = strings + be32(dtb, at + 4) as usize;
                    let name_end = dtb[name_off..].iter().position(|&b| b == 0).unwrap();
                    let name =
                        String::from_utf8(dtb[name_off..name_off + name_end].to_vec()).unwrap();
                    let value = dtb[at + 8..at + 8 + len].to_vec();
                    at = (at + 8 + len).next_multiple_of(4);
                    let node = path.join("/");
                    out.iter_mut()
                        .rev()
                        .find(|(p, _)| *p == node)
                        .unwrap()
                        .1
                        .push((name, value));
                }
                4 => {}
                9 => return out,
                _ => panic!("fdt token {token}"),
            }
        }
    }

    fn prop<'a>(props: &'a [(String, Vec<u8>)], name: &str) -> Option<&'a [u8]> {
        props
            .iter()
            .find(|(n, _)| n == name)
            .map(|(_, v)| v.as_slice())
    }

    fn opp_table(
        nodes: &[(String, Vec<(String, Vec<u8>)>)],
        phandle: u32,
        cells: usize,
    ) -> Vec<Opp> {
        let (table, _) = nodes
            .iter()
            .find(|(_, p)| prop(p, "phandle").map(|v| be32(v, 0)) == Some(phandle))
            .unwrap();
        let mut rows: Vec<(String, Opp)> = nodes
            .iter()
            .filter(|(p, _)| {
                p.starts_with(&format!("{table}/"))
                    && p.matches('/').count() == table.matches('/').count() + 1
            })
            .map(|(p, props)| {
                let hz = u64::from_be_bytes(prop(props, "opp-hz").unwrap().try_into().unwrap());
                let uv = prop(props, "opp-microvolt").unwrap();
                assert_eq!(uv.len(), 4 * cells);
                let mut mv = [0u32; CLUSTERS];
                for (c, m) in mv.iter_mut().enumerate().take(cells) {
                    *m = be32(uv, 4 * c).div_ceil(1000);
                }
                (
                    p.clone(),
                    Opp {
                        mhz: (hz / 1_000_000) as u32,
                        mv,
                        cells,
                    },
                )
            })
            .collect();
        rows.sort_by(|x, y| x.0.cmp(&y.0));
        rows.into_iter().map(|(_, o)| o).collect()
    }

    /// No GPU node property value panics the builder (CONFIG_RUST_OVERFLOW_CHECKS is on in our
    /// kernels): each of the properties at the u32 and single-precision boundaries, on a copy
    /// of a seed record that builds. A value may be refused; it must not panic.
    #[test]
    fn device_tree_u32_boundaries_do_not_panic() {
        let main = main_opps();
        let cs = [Opp {
            mhz: 24,
            mv: [635, 0, 0, 0],
            cells: 1,
        }];
        let mut seed = vec![0u8; SIZE];
        assert_eq!(build(&mut seed, &inputs(&main, &cs, &[])), Ok(()));
        for prop in 0..PROPS.len() {
            for value in [
                0, 1, 7, 8, 31, 0x3f80_0000, 0x4000_0000, 0x7f80_0000, 0x7fc0_0000, 0x8000_0000,
                u32::MAX,
            ] {
                let mut i = inputs(&main, &cs, &[]);
                i.props[prop] = value;
                let mut image = seed.clone();
                let built = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    build(&mut image, &i)
                }));
                assert!(built.is_ok(), "prop {} value {:#x}", prop, value);
            }
        }
    }

    /// A property of an ADT node found by its path of `name`s from the root: nodes of u32
    /// property and child counts, properties of a 32-byte name, a u32 size (top bit a flag)
    /// and the value padded to 4 bytes.
    fn adt_prop<'a>(adt: &'a [u8], path: &[&str], name: &str) -> Option<&'a [u8]> {
        fn le(b: &[u8], o: usize) -> usize {
            u32::from_le_bytes(b[o..o + 4].try_into().unwrap()) as usize
        }
        // The node at `at`: its properties, its children's offsets and the offset past it.
        fn node(adt: &[u8], at: usize) -> (Vec<(&[u8], &[u8])>, Vec<usize>, usize) {
            let (np, nc) = (le(adt, at), le(adt, at + 4));
            let mut o = at + 8;
            let mut props = Vec::new();
            for _ in 0..np {
                let n = &adt[o..o + 32];
                let n = &n[..n.iter().position(|&b| b == 0).unwrap_or(32)];
                let size = le(adt, o + 32) & 0x7fff_ffff;
                props.push((n, &adt[o + 36..o + 36 + size]));
                o += 36 + size.next_multiple_of(4);
            }
            let mut children = Vec::new();
            for _ in 0..nc {
                children.push(o);
                o = node(adt, o).2;
            }
            (props, children, o)
        }
        let mut at = 0;
        for want in path {
            let (_, children, _) = node(adt, at);
            let named = |c: usize| {
                let (props, _, _) = node(adt, c);
                props.iter().any(|(n, v)| {
                    *n == b"name" && v.split(|&b| b == 0).next() == Some(want.as_bytes())
                })
            };
            at = *children.iter().find(|&&c| named(c))?;
        }
        node(adt, at).0.into_iter().find(|(n, _)| *n == name.as_bytes()).map(|(_, v)| v)
    }

    /// Builder on the saved device tree and ADT with the J516C reference words == the J516C
    /// reference machine's own hardware record, byte for byte except the [`UNNEEDED`] words.
    /// Needs `G15_J516C_FIXTURES=<dir>` with `j516c.dtb` (`/sys/firmware/fdt`), `adt.bin` (the
    /// boot loader's ADT) and `t6031-j516c-board.bin` (that machine's record), all off the
    /// tree: ignored by default, `--ignored` runs it.
    #[test]
    #[ignore = "needs the off-tree J516C fixtures (G15_J516C_FIXTURES)"]
    fn j516c_fixture() {
        let dir = std::env::var_os("G15_J516C_FIXTURES")
            .expect("G15_J516C_FIXTURES names the directory with j516c.dtb, adt.bin, the record");
        let dir = std::path::Path::new(&dir);
        let dtb = std::fs::read(dir.join("j516c.dtb")).unwrap();
        let adt = std::fs::read(dir.join("adt.bin")).unwrap();
        let blob = std::fs::read(dir.join("t6031-j516c-board.bin")).unwrap();
        assert_eq!(&blob[..8], b"T6031BRD");
        let at = 16 + 3 * 8;
        let file = &blob[at..at + SIZE];

        let nodes = fdt(&dtb);
        let (_, gpu) = nodes
            .iter()
            .find(|(_, p)| {
                prop(p, "compatible")
                    .is_some_and(|c| c.split(|&b| b == 0).any(|s| s == b"apple,agx-t6031"))
            })
            .expect("a T6031 GPU node");
        let mut props = [0u32; PROPS.len()];
        for (v, (name, default)) in props.iter_mut().zip(PROPS.iter()) {
            *v = match prop(gpu, name.to_str().unwrap()) {
                Some(b) => be32(b, 0),
                None => default.unwrap_or_else(|| panic!("GPU node has no {name:?}")),
            };
        }
        let zones: Vec<u32> = prop(gpu, "apple,power-zones")
            .map(|b| b.chunks(4).map(|c| be32(c, 0)).collect())
            .unwrap_or_default();
        let main = opp_table(
            &nodes,
            be32(prop(gpu, "operating-points-v2").unwrap(), 0),
            4,
        );
        let cs = opp_table(&nodes, be32(prop(gpu, "apple,cs-opp").unwrap(), 0), 1);
        let afr = opp_table(&nodes, be32(prop(gpu, "apple,afr-opp").unwrap(), 0), 1);
        assert_eq!(main.len(), PERF_STATES);
        // The ttbs region: the first memory-region phandle's reserved-memory node.
        let regions = prop(gpu, "memory-region").unwrap();
        let (_, ttbs) = nodes
            .iter()
            .find(|(_, p)| prop(p, "phandle").map(|v| be32(v, 0)) == Some(be32(regions, 0)))
            .unwrap();
        let reg = prop(ttbs, "reg").unwrap();
        let ttbat = u64::from_be_bytes(reg[..8].try_into().unwrap());
        // The IOMapping slots: the T6031 table's addresses and sizes with the file's VAs (the
        // VAs are the driver's; the layout packs them from a base with a page gap).
        let slots_table: [(u8, u64, u32, u32, bool); 17] = [
            (0, 0x408d00000, 0x144000, 0x144000, true),
            (1, 0x20e100000, 0x4000, 0x4000, false),
            (2, 0x292414000, 0x4000, 0x4000, true),
            (3, 0x408000000, 0x20000, 0x20000, true),
            (7, 0x290444000, 0x4000, 0x4000, false),
            (9, 0x408e08000, 0x8000, 0x8000, true),
            (11, 0x220000000, 8 * 0xd8000, 0xd8000, true),
            (12, 0x29244c000, 0x4000, 0x4000, false),
            (18, 0x2903d0000, 0x4000, 0x4000, true),
            (19, 0x2903c0000, 0x4000, 0x4000, false),
            (20, 0x2903d8000, 0x4000, 0x4000, true),
            (21, 0x408eb4000, 0x4000, 0x4000, true),
            (24, 0x409000000, 0x8000, 0x8000, true),
            (25, 0x3c945c000, 0x4000, 0x4000, true),
            (26, 0x290400000, 0x8000, 0x8000, false),
            (28, 0x2a4dc0000, 0x24000, 0x24000, false),
            (29, 0x408e1c000, 0x4000, 0x4000, false),
        ];
        let mut va = 0xfffffc2068000000u64;
        let slots: Vec<IoSlot> = slots_table
            .iter()
            .map(|&(slot, pa, total, element, writable)| {
                let delta = pa & 0x3fff;
                let s = IoSlot {
                    slot,
                    pa,
                    va: va + delta,
                    total,
                    element,
                    writable,
                };
                va += (delta + u64::from(total) + 0x3fff) & !0x3fff;
                va += 0x4000;
                s
            })
            .collect();
        let i = Inputs {
            chip_id: 0x6031,
            props,
            zones: &zones,
            main: &main,
            cs: &cs,
            afr: &afr,
            slots: &slots,
            empty_writable: &[4, 6, 8, 10, 13, 14, 15, 16, 17],
            ttbat,
            usc_base: 0x10_0000_0000,
            user_end: 0x2ff_ffff_8000,
            timestamps: 0xfffffc2ffc000000,
            sensor_rows: sensor_rows(
                adt_prop(&adt, &["arm-io", "aon-ptd"], "ptd_data_table").expect("aon-ptd table"),
            )
            .unwrap(),
            reference: &J516C,
        };
        let mut built = vec![0u8; SIZE];
        build(&mut built, &i).unwrap();
        assert_eq!(cross_check(&built, file), None, "(offset, built, file)");
        for &at in UNNEEDED.iter() {
            assert_eq!(get(&built, at), 0);
        }
    }
}
