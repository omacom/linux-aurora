// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Initialization data of the 25G83 firmware build (macOS 26.6, `mBoot-18000.161.10`).
//!
//! The M4 Pro (T6040) runs this build. It has the interface version of the A18 Pro's
//! 26A428 build and the same objects, with these differences:
//!
//! - [`HwData25G83`]: the A18 Pro's hardware data up to `0x2554`, eight bytes inserted before
//!   `0x2570` and four more before `0x2614`, and the CS and AFR performance tables;
//! - [`PowerConfig25G83`]: the power configuration with two blocks removed (`0xa8` and `0xa0`
//!   bytes), at bundle `+0x5380`, followed by the auxiliary data;
//! - region C and the power state ([`POWER_STATE_WORDS`]) with other values;
//! - one firmware instance carrying both roles, with a larger status B ([`private`]), five
//!   separately allocated views and a firmware area ([`kernel_window`]).
//!
//! The constant contents were derived from the G16 encoders of Gravity Linux, which run this
//! firmware on the M4 Pro, by evaluating them for this SoC; see the commit that added them.

use super::{
    fill_words, perf_states, CompletionRingDescriptor, HwDataArgs, Opp, PerfGroup, RegisterMapEntry,
    Words, COMPLETION_RING_ENTRIES, COMPLETION_RING_ENTRY_SIZE, COMPLETION_RING_INDEX, PERF_STATES,
    REGISTER_MAP_SLOTS,
};
use kernel::prelude::*;

/// Kernel-window objects whose placement differs from the A18 Pro's, as offsets in the upper
/// kernel window.
pub(crate) mod kernel_window {
    /// The power-state object named by the root (`0x6a8` bytes).
    pub(crate) const POWER_STATE: u64 = 0x17_0000;
    /// The five views named by the main configuration.
    pub(crate) const VIEWS: [u64; 5] = [0x41_4000, 0x40_0000, 0x40_4000, 0x40_8000, 0x40_c000];
    /// The firmware area named twice by the main configuration (`0x88` bytes).
    pub(crate) const FIRMWARE_AREA: u64 = 0x41_0000;
    /// The AON temperature-source table named by the power configuration.
    pub(crate) const AON_TABLE: u64 = 0xc050_0000;
}

/// Offsets in the private cluster.
pub(crate) mod private {
    /// Size of the private cluster: the state grid at `+0`, status B and status A.
    pub(crate) const SIZE: usize = 0xd4000;
    /// The primary status B ([`super::super::PrimaryStatusB`] up to its power state).
    pub(crate) const STATUS_B: usize = 0x1_0000;
    /// The primary status A, after the `0xf180` bytes of status B.
    pub(crate) const STATUS_A: usize = 0x1_f180;
    /// The configuration named by the root; its status follows at `+0x80`.
    pub(crate) const CONFIG: usize = 0xd_3440;
    /// The zeroed page named by the main configuration's `unk_18`.
    pub(crate) const MAIN_HEADER: usize = 0x8000;
}

/// Bundle offset of the [`PowerConfig25G83`].
pub(crate) const POWER_CONFIG: usize = 0x5380;
/// Bundle offset of the completion-ring descriptors in the [`HwData25G83`].
pub(crate) const COMPLETION_DESCRIPTORS: usize = 0x2620;
/// Bundle offset of the QoS pointer in the [`HwData25G83`].
pub(crate) const QOS_POINTER: usize = 0x26b8;

/// Most states of a CS or AFR performance table.
const AUX_PERF_STATES: usize = 16;

/// The CS or AFR performance table of the hardware data.
#[repr(C, packed)]
pub(crate) struct AuxPerfTable {
    /// Highest state, twice.
    pub(crate) max_state: [u32; 2],
    /// Frequency of each state in MHz.
    pub(crate) freq_mhz: [u32; AUX_PERF_STATES],
    /// Voltage of each state in mV, in the first word of each pair.
    pub(crate) mv: [[u32; 2]; AUX_PERF_STATES],
    /// SRAM voltage of each state in mV, in the first word of each pair.
    pub(crate) sram_mv: [[u32; 2]; AUX_PERF_STATES],
}

// SAFETY: `AuxPerfTable` consists of integers and arrays of integers only.
unsafe impl Zeroable for AuxPerfTable {}

static_assert!(size_of::<AuxPerfTable>() == 0x148);

impl AuxPerfTable {
    /// Fills the table from its operating points (one voltage cell each), raising each SRAM
    /// voltage to `sram_min_mv`.
    fn fill(&mut self, opps: &[Opp], sram_min_mv: u32) -> Result {
        if !(2..=AUX_PERF_STATES).contains(&opps.len()) {
            return Err(EINVAL);
        }
        self.max_state = [opps.len() as u32 - 1; 2];
        for (index, opp) in opps.iter().enumerate() {
            if opp.cells != 1 {
                return Err(EINVAL);
            }
            let mv = opp.volt_uv[0] / 1000;
            self.freq_mhz[index] = u32::try_from(opp.freq_hz / 1_000_000)?;
            self.mv[index][0] = mv;
            self.sram_mv[index][0] = mv.max(sram_min_mv);
        }
        Ok(())
    }
}

/// Inputs of the 25G83 hardware data in addition to [`HwDataArgs`].
pub(crate) struct HwDataArgs25G83<'a> {
    /// Enabled GPU cores.
    pub(crate) active_cores: u32,
    /// Operating points of the CS clock domain.
    pub(crate) cs_opps: &'a [Opp],
    /// Operating points of the AFR clock domain.
    pub(crate) afr_opps: &'a [Opp],
}

/// The hardware data of the 25G83 build, at the base of the bundle.
#[repr(C, packed)]
pub(crate) struct HwData25G83 {
    /// Required: 0x6f_0000_0000.
    pub(crate) unk_0: u64,
    /// Required: 0xffc0_0000.
    pub(crate) unk_8: u64,
    /// Required: 0x10_0000_0000.
    pub(crate) unk_10: u64,
    /// Required: 0x10_0000_0000.
    pub(crate) unk_18: u64,
    /// Required: 0x2ff_ffff_8000.
    pub(crate) unk_20: u64,
    /// Required: 0xffff_fc21_8140_0000.
    pub(crate) unk_28: u64,
    pub(crate) unk_30: [u8; 0x610],
    /// Register windows the firmware maps.
    pub(crate) register_map: [RegisterMapEntry; REGISTER_MAP_SLOTS],
    pub(crate) unk_e88: [u8; 8],
    /// Chip identifier.
    pub(crate) chip_id: u32,
    /// Required: 1.
    pub(crate) unk_e94: u32,
    /// Required: 1.
    pub(crate) unk_e98: u32,
    pub(crate) unk_e9c: [u8; 4],
    /// Required: 1.
    pub(crate) unk_ea0: u32,
    /// Required: 1.
    pub(crate) unk_ea4: u32,
    pub(crate) unk_ea8: [u8; 0x10],
    /// Required: 1.
    pub(crate) unk_eb8: u32,
    pub(crate) unk_ebc: [u8; 4],
    /// Required: 1.
    pub(crate) unk_ec0: u32,
    pub(crate) unk_ec4: [u8; 4],
    /// Required: 1.
    pub(crate) unk_ec8: u32,
    pub(crate) unk_ecc: [u8; 4],
    /// Required: 0x5dc0 (24000).
    pub(crate) unk_ed0: u32,
    /// Required: 1.
    pub(crate) unk_ed4: u32,
    /// Power sample period in ms: 8.
    pub(crate) unk_ed8: u32,
    pub(crate) unk_edc: [u8; 4],
    /// Required: 1 in every entry.
    pub(crate) unk_ee0: [u32; 3],
    pub(crate) unk_eec: [u8; 0x18],
    /// Required: 0x1f (31).
    pub(crate) unk_f04: u32,
    pub(crate) unk_f08: [u8; 0x1c],
    /// Required: 0x0700_2004.
    pub(crate) unk_f24: u32,
    /// Required: 6.
    pub(crate) unk_f28: u32,
    pub(crate) unk_f2c: [u8; 8],
    /// Required: 1.
    pub(crate) unk_f34: u32,
    /// Required: 1.
    pub(crate) unk_f38: u32,
    pub(crate) unk_f3c: [u8; 0x10],
    /// Required: 0x31 (49).
    pub(crate) unk_f4c: u32,
    pub(crate) unk_f50: [u8; 0x38],
    /// Enabled GPU cores.
    pub(crate) unk_f88: u32,
    /// Required: 0xf (15).
    pub(crate) unk_f8c: u32,
    pub(crate) unk_f90: [u8; 0x28],
    /// Required: 0x1c (28).
    pub(crate) unk_fb8: u32,
    /// Required: 4.
    pub(crate) unk_fbc: u32,
    /// Enabled GPU cores.
    pub(crate) unk_fc0: u32,
    /// Performance table A, one voltage lane per cluster.
    pub(crate) perf_a: PerfGroup,
    pub(crate) unk_16c8: [u8; 0x140],
    /// Frequency of each state of table B in MHz.
    pub(crate) perf_b_freq_mhz: [u32; PERF_STATES],
    pub(crate) unk_1834: [u8; 0x14],
    /// Required: 0x3f82_8f5c (1.02 as an IEEE-754 single) in every entry.
    pub(crate) unk_1848: [u32; 11],
    pub(crate) unk_1874: [u8; 0x54],
    /// Relative performance of each state of table A, in percent.
    pub(crate) perf_a_relative: [u32; PERF_STATES],
    pub(crate) unk_18f4: [u8; 0x14],
    /// Relative performance of each state of table B, in percent.
    pub(crate) perf_b_relative: [u32; PERF_STATES],
    pub(crate) unk_1934: [u8; 0x94],
    /// Operating point of each state of table A.
    pub(crate) perf_a_states: [u32; PERF_STATES],
    pub(crate) unk_19f4: [u8; 0x14],
    /// Operating point of each state of table B.
    pub(crate) perf_b_states: [u32; PERF_STATES],
    pub(crate) unk_1a34: [u8; 0x14],
    /// CS clock-domain performance table.
    pub(crate) cs_perf: AuxPerfTable,
    /// AFR clock-domain performance table.
    pub(crate) afr_perf: AuxPerfTable,
    /// Performance table B, one voltage lane per cluster.
    pub(crate) perf_b: PerfGroup,
    pub(crate) unk_23dc: [u8; 0x168],
    /// Required: 5.
    pub(crate) unk_2544: u32,
    /// Required: 6.
    pub(crate) unk_2548: u32,
    /// Required: 3.
    pub(crate) unk_254c: u32,
    /// Required: 7.
    pub(crate) unk_2550: u32,
    /// Required: 7.
    pub(crate) unk_2554: u32,
    pub(crate) unk_2558: [u8; 0x18],
    /// Enabled GPU cores.
    pub(crate) unk_2570: u32,
    pub(crate) unk_2574: [u8; 4],
    /// Required: 1.
    pub(crate) unk_2578: u32,
    pub(crate) unk_257c: [u8; 0x1c],
    /// Required: 0, 1, 1, 1.
    pub(crate) unk_2598: [u32; 4],
    /// Required: 0, 1, 1, 1.
    pub(crate) unk_25a8: [u32; 4],
    pub(crate) unk_25b8: [u8; 4],
    /// Required: 0xffff_ffff in every entry, one per work channel.
    pub(crate) unk_25bc: [u32; 12],
    pub(crate) unk_25ec: [u8; 8],
    /// Required: 2.
    pub(crate) unk_25f4: u32,
    pub(crate) unk_25f8: [u8; 4],
    /// Required: 1.
    pub(crate) unk_25fc: u32,
    pub(crate) unk_2600: [u8; 0x14],
    /// Required: 1.
    pub(crate) unk_2614: u32,
    pub(crate) unk_2618: [u8; 4],
    /// Required: 0x100.
    pub(crate) unk_261c: u32,
    /// Completion-ring descriptors.
    pub(crate) completion_rings: [CompletionRingDescriptor; 4],
    pub(crate) unk_26a0: [u8; 0x18],
    /// QoS object, zeroed.
    pub(crate) qos_va: u64,
    /// Required: 1.
    pub(crate) unk_26c0: u32,
    /// Required: 1.
    pub(crate) unk_26c4: u32,
    pub(crate) unk_26c8: [u8; 0x2c],
    /// Required: 1.
    pub(crate) unk_26f4: u32,
    pub(crate) unk_26f8: [u8; 0x16bc],
}

static_assert!(core::mem::offset_of!(HwData25G83, register_map) == 0x640);
static_assert!(core::mem::offset_of!(HwData25G83, chip_id) == 0xe90);
static_assert!(core::mem::offset_of!(HwData25G83, perf_a) == 0xfc4);
static_assert!(core::mem::offset_of!(HwData25G83, perf_b_freq_mhz) == 0x1808);
static_assert!(core::mem::offset_of!(HwData25G83, cs_perf) == 0x1a48);
static_assert!(core::mem::offset_of!(HwData25G83, afr_perf) == 0x1b90);
static_assert!(core::mem::offset_of!(HwData25G83, perf_b) == 0x1cd8);
static_assert!(core::mem::offset_of!(HwData25G83, unk_2570) == 0x2570);
static_assert!(core::mem::offset_of!(HwData25G83, completion_rings) == COMPLETION_DESCRIPTORS);
static_assert!(core::mem::offset_of!(HwData25G83, qos_va) == QOS_POINTER);
static_assert!(size_of::<HwData25G83>() == 0x3db4);
static_assert!(size_of::<HwData25G83>() <= POWER_CONFIG);

// SAFETY: `HwData25G83` consists of integers and arrays of integers only.
unsafe impl Zeroable for HwData25G83 {}

impl HwData25G83 {
    /// Initializes the hardware data. Table A and B carry one voltage lane per cluster.
    pub(crate) fn init(&mut self, args: &HwDataArgs<'_>, ext: &HwDataArgs25G83<'_>) -> Result {
        super::clear(self);
        self.unk_0 = 0x6f_0000_0000;
        self.unk_8 = 0xffc0_0000;
        self.unk_10 = 0x10_0000_0000;
        self.unk_18 = 0x10_0000_0000;
        self.unk_20 = 0x2ff_ffff_8000;
        self.unk_28 = 0xffff_fc21_8140_0000;

        let mut used = [false; REGISTER_MAP_SLOTS];
        for window in args.windows {
            match used.get_mut(window.slot) {
                Some(used) if !*used => *used = true,
                _ => return Err(EINVAL),
            }
            self.register_map[window.slot] = RegisterMapEntry::window(window);
        }
        if !args.reserved_slots.is_empty() {
            return Err(EINVAL);
        }

        self.chip_id = args.chip_id;
        self.unk_e94 = 1;
        self.unk_e98 = 1;
        self.unk_ea0 = 1;
        self.unk_ea4 = 1;
        self.unk_eb8 = 1;
        self.unk_ec0 = 1;
        self.unk_ec8 = 1;
        self.unk_ed0 = 0x5dc0;
        self.unk_ed4 = 1;
        self.unk_ed8 = 8;
        self.unk_ee0 = [1; 3];
        self.unk_f04 = 0x1f;
        self.unk_f24 = 0x0700_2004;
        self.unk_f28 = 6;
        self.unk_f34 = 1;
        self.unk_f38 = 1;
        self.unk_f4c = 0x31;
        self.unk_f88 = ext.active_cores;
        self.unk_f8c = 0xf;
        self.unk_fb8 = 0x1c;
        self.unk_fbc = 4;
        self.unk_fc0 = ext.active_cores;

        let perf = args.perf;
        let states_a = perf_states(args.opps, &perf.states_a, perf.sram_min_mv)?;
        let states_b = perf_states(args.opps, &perf.states_b, perf.sram_min_mv)?;
        if states_a[0].cells < 2 {
            return Err(EINVAL);
        }
        self.perf_a.fill(&states_a, 1);
        self.perf_b.fill(&states_b, 1);
        for (index, state) in states_b.iter().enumerate() {
            self.perf_b_freq_mhz[index] = state.freq_mhz;
            self.perf_a_states[index] = perf.states_a[index].into();
            self.perf_b_states[index] = perf.states_b[index].into();
        }
        self.perf_a_relative = perf.relative_a;
        self.perf_b_relative = perf.relative_b;
        self.unk_1848 = [0x3f82_8f5c; 11];
        self.cs_perf.fill(ext.cs_opps, perf.sram_min_mv)?;
        self.afr_perf.fill(ext.afr_opps, perf.sram_min_mv)?;

        self.unk_2544 = 5;
        self.unk_2548 = 6;
        self.unk_254c = 3;
        self.unk_2550 = 7;
        self.unk_2554 = 7;
        self.unk_2570 = ext.active_cores;
        self.unk_2578 = 1;
        self.unk_2598 = [0, 1, 1, 1];
        self.unk_25a8 = [0, 1, 1, 1];
        self.unk_25bc = [u32::MAX; 12];
        self.unk_25f4 = 2;
        self.unk_25fc = 1;
        self.unk_2614 = 1;
        self.unk_261c = 0x100;
        for (ring, index) in args.completion_rings.iter().zip(COMPLETION_RING_INDEX) {
            self.completion_rings[index as usize] = CompletionRingDescriptor {
                low_va: ring.low_va,
                high_va: ring.high_va,
                entries: COMPLETION_RING_ENTRIES,
                entry_size: COMPLETION_RING_ENTRY_SIZE,
                index,
                unk_1c: 0,
            };
        }
        self.qos_va = args.qos_va;
        self.unk_26c0 = 1;
        self.unk_26c4 = 1;
        self.unk_26f4 = 1;
        Ok(())
    }
}

/// Power configuration control period, in 24 MHz clocks (8 ms).
const PERIOD_CLOCKS: u32 = 200_000;
/// Highest GPU power, in mW.
pub(crate) const MAX_POWER_MW: u32 = 69_116;
/// [`MAX_POWER_MW`] as an IEEE-754 single.
const MAX_POWER_MW_F32: u32 = 0x4786_fe00;
/// Temperature sensors read by the die and MTR temperature limiters.
const TEMP_SENSOR_MASK: u32 = 0x2a22_2aa2;

/// Leakage coefficients, IEEE-754 singles: three arrays of sixteen clusters, then four arrays
/// of two dies. Not found in the ADT; constant for the SoC.
const LEAKAGE: [u32; 56] = {
    let mut words = [0; 56];
    words[0] = 0x44f0_3000; // 1921.5
    words[1] = 0x44f3_0000; // 1944.0
    words[32] = 0x4373_0000; // 243.0
    words[33] = 0x4373_0000; // 243.0
    words[52] = 0x437b_0000; // 251.0
    words[54] = 0x43bd_c000; // 379.5
    words
};
/// Offset of [`LEAKAGE`] in the power configuration.
const LEAKAGE_OFFSET: usize = 0x642c;
/// Offset in the power configuration of the GPU address of the configuration's status (the
/// configuration named by the root, `+0x80`).
const CONFIG_STATUS_POINTER: usize = 0x3b50;
/// Offset in the power configuration of the GPU address of the AON table.
const AON_POINTER: usize = 0x6318;
/// Offset in the power configuration of a random word the firmware expects.
const ENTROPY: usize = 0x63d4;

/// Nonzero words of the power configuration and the auxiliary data that follows it, offsets
/// from the start of the power configuration.
const POWER_CONFIG_WORDS: Words = &[
    (0x4, PERIOD_CLOCKS, 2),
    (0x10, 0x4, 1),
    (0x14, 0x3f800000, 1),
    (0x2c, 0x1, 2),
    (0x40, 0x64, 1),
    (0x44, 0x1, 1),
    (0x48, 0x3e8, 1),
    (0x50, 0x64, 1),
    (0x60, 0x4, 1),
    (0x64, 0x3f800000, 1),
    (0x6c, 0x1, 2),
    (0x80, 0x64, 1),
    (0x84, 0x1, 1),
    (0x88, 0x3e8, 1),
    (0x90, 0x64, 1),
    (0xc0, 0x3f828f5c, 11),
    (0x9b8, 0x271, 1),
    (0x9c4, 0x3f7f2e9f, 1),
    (0x9cc, 0x3b516154, 1),
    (0x9d4, 0x3ca59586, 1),
    (0x9e0, MAX_POWER_MW_F32, 1),
    (0x9e4, 0x40a90fdb, 1),
    (0x9ec, 0xbc6d0cfe, 1),
    (0x9f0, 0x64, 1),
    (0x9f4, 0x3e8, 2),
    (0xa04, MAX_POWER_MW, 1),
    (0xa38, 0x3e8, 1),
    (0xa70, 0x8, 1),
    (0xa7c, 0x3f000000, 1),
    (0xa84, 0x3f000000, 1),
    (0xa8c, 0x3e3bd7b3, 1),
    (0xa98, 0x47800000, 1),
    (0xa9c, 0x3ecf2e49, 1),
    (0xaa8, 0x64, 1),
    (0xaac, 0x3e8, 1),
    (0xabc, MAX_POWER_MW, 1),
    (0xac8, 0x10, 1),
    (0xad0, 0x5dc00, 1),
    (0xb28, 0x50, 1),
    (0xb30, 0x55, 1),
    (0xb34, 0x32, 1),
    (0xb38, 0x6, 1),
    (0xb40, 0x6, 1),
    (0xb44, 0x1, 1),
    (0xb4c, 0x3f4ccccd, 1),
    (0xb50, 0x3f7eb852, 1),
    (0xb54, 0x3e4ccccd, 1),
    (0xb58, 0x3ba3d70a, 1),
    (0xb5c, 0x3f64c986, 2),
    (0xb68, 0x42be0000, 1),
    (0xb6c, 0x403ea7f0, 2),
    (0xb74, 0x41179436, 1),
    (0xb78, 0x64, 1),
    (0xb7c, 0x3e8, 1),
    (0xb80, 0x64, 1),
    (0xb8c, 0x50, 1),
    (0xbc0, 0x64, 1),
    (0x1bec, 0xff000000, 1),
    (0x1bf0, 0xff7fff7f, 255),
    (0x1fec, 0x7fff7f, 1),
    (0x21f0, 0x47800000, 1),
    (0x21f4, 0x42c80000, 1),
    (0x21f8, 0x447a0000, 1),
    (0x21fc, 0x62a, 1),
    (0x2210, 0x64, 1),
    (0x2218, 0x42c80000, 1),
    (0x221c, 0x3e8, 1),
    (0x2224, 0x3f4ccccd, 1),
    (0x2228, 0x3e4ccccd, 1),
    (0x2268, TEMP_SENSOR_MASK, 1),
    (0x2270, 0x1f40, 1),
    (0x2278, 0x4, 1),
    (0x228c, 0x3f800000, 1),
    (0x2294, 0x414eeeee, 1),
    (0x22a0, 0x47800000, 1),
    (0x22a4, 0x42140000, 1),
    (0x22b0, 0x64, 1),
    (0x22b4, 0x3e8, 2),
    (0x22c4, 0x2af8, 1),
    (0x22f8, 0x3e8, 1),
    (0x2728, 0xb8520000, 1),
    (0x272c, 0x3f7e, 1),
    (0x2730, 0xd70a0000, 1),
    (0x2734, 0x3ba3, 1),
    (0x273c, 0x43c8, 1),
    (0x2748, 0x4780, 1),
    (0x274c, 0x42c8, 1),
    (0x2754, 0x384bac8, 1),
    (0x2758, 0x3e80000, 2),
    (0x2764, 0x80000000, 1),
    (0x2768, 0x26ac453b, 1),
    (0x279c, 0x3e80000, 1),
    (0x27b8, 0xc80000, 1),
    (0x2870, 0x3e8, 1),
    (0x28c0, 0x1, 1),
    (0x28c8, MAX_POWER_MW, 3),
    (0x28e0, 0x3f800000, 1),
    (0x28e4, 0x4, 1),
    (0x28e8, 0x8, 1),
    (0x28ec, 0x2ee00, 1),
    (0x29bc, 0x7c, 1),
    (0x29c8, 0x3f77bdef, 1),
    (0x29d0, 0x3d042108, 1),
    (0x29d8, 0x3d25bea1, 1),
    (0x29e4, 0x47800000, 1),
    (0x29e8, 0x3f4f2e49, 1),
    (0x29f4, 0x28, 1),
    (0x29f8, 0x3e8, 2),
    (0x2a04, MAX_POWER_MW_F32, 1),
    (0x2a08, MAX_POWER_MW, 1),
    (0x2a14, 0xf8, 1),
    (0x2a1c, 0x5ad200, 1),
    (0x2a3c, 0x3e8, 1),
    (0x2b4c, 0x32, 1),
    (0x2b50, 0x1, 1),
    (0x2b58, 0x3f638e39, 1),
    (0x2b5c, 0x3f2aaaab, 1),
    (0x2b60, 0x3de38e39, 1),
    (0x2b64, 0x3eaaaaab, 1),
    (0x2b68, 0xbecccccd, 2),
    (0x2b74, 0x47800000, 1),
    (0x2b78, 0xc0a00000, 2),
    (0x2b84, 0x64, 1),
    (0x2b88, 0x3e8, 1),
    (0x2b8c, 0x64, 1),
    (0x2b94, 0x45fa0000, 1),
    (0x2b98, 0x578, 1),
    (0x2ba4, 0x48, 1),
    (0x2ba8, 0x18, 1),
    (0x2bac, 0x1a5e00, 1),
    (0x2bb4, 0x8ca00, 1),
    (0x2bc0, 0x47800000, 1),
    (0x2be8, 0x1f40, 1),
    (0x2bec, 0x4, 1),
    (0x2bf0, 0x9c4, 1),
    (0x2bf4, 0x258, 1),
    (0x2bf8, 0x2, 1),
    (0x2bfc, 0x4, 1),
    (0x2c00, 0x32, 1),
    (0x2c30, 0x47800000, 1),
    (0x2c40, 0x64, 1),
    (0x2c44, 0x3e8, 1),
    (0x2e2c, 0x4, 1),
    (0x2e38, 0x3f780000, 1),
    (0x2e40, 0x3d000000, 1),
    (0x2e48, 0x40a00000, 1),
    (0x2e54, 0x47800000, 1),
    (0x2e58, 0x41700000, 1),
    (0x2e64, 0x384, 1),
    (0x2e68, 0x3e8, 2),
    (0x2e78, 0x3e8, 1),
    (0x2eac, 0x3e8, 1),
    (0x2eec, 0x64, 1),
    (0x2f08, TEMP_SENSOR_MASK, 1),
    (0x2f10, 0x7d, 1),
    (0x2f14, 0x1, 1),
    (0x3b58, TEMP_SENSOR_MASK, 1),
    (0x3b60, 0x1, 1),
    (0x5b64, 0x40a00000, 1),
    (0x6320, u32::from_be_bytes(*b"g0TR"), 1),
    (0x6324, u32::from_be_bytes(*b"g1TR"), 1),
    (0x6360, 0x3, 1),
    (0x6368, u32::from_be_bytes(*b"r0TR"), 1),
    (0x6370, 0x1, 1),
    (0x6420, 0x1, 1),
];

/// The power configuration and auxiliary data of the 25G83 build, at bundle
/// [`POWER_CONFIG`]. The main configuration names it.
#[repr(C)]
pub(crate) struct PowerConfig25G83(pub(crate) [u8; 0x6510]);

// SAFETY: `PowerConfig25G83` is a byte array.
unsafe impl Zeroable for PowerConfig25G83 {}

static_assert!(LEAKAGE_OFFSET + size_of::<[u32; 56]>() <= size_of::<PowerConfig25G83>());

impl PowerConfig25G83 {
    /// Pointer and random words, which [`Self::hash`] leaves out.
    const RUNTIME: [(usize, usize); 3] =
        [(CONFIG_STATUS_POINTER, 8), (AON_POINTER, 8), (ENTROPY, 4)];

    /// Initializes the power configuration.
    pub(crate) fn init(&mut self, config_va: u64, aon_va: u64, entropy: u32) {
        fill_words(&mut self.0, POWER_CONFIG_WORDS);
        for (index, word) in LEAKAGE.iter().enumerate() {
            let at = LEAKAGE_OFFSET + 4 * index;
            self.0[at..at + 4].copy_from_slice(&word.to_le_bytes());
        }
        self.0[CONFIG_STATUS_POINTER..CONFIG_STATUS_POINTER + 8]
            .copy_from_slice(&(config_va + 0x80).to_le_bytes());
        self.0[AON_POINTER..AON_POINTER + 8].copy_from_slice(&aon_va.to_le_bytes());
        self.0[ENTROPY..ENTROPY + 4].copy_from_slice(&entropy.to_le_bytes());
    }

    /// Hash of the contents without the pointers and the random word.
    pub(crate) fn hash(&self) -> u64 {
        hash_without(&self.0, &Self::RUNTIME)
    }
}

/// The AON temperature-source rows the power configuration names: the GPU cluster 0, GPU
/// cluster 1 and SoC sensors.
pub(crate) const AON_TABLE: [u32; 9] = [
    u32::from_be_bytes(*b"g0TR"),
    0x006b_0001,
    0x0000_1401,
    u32::from_be_bytes(*b"g1TR"),
    0x00b5_0001,
    0x0000_1401,
    u32::from_be_bytes(*b"r0TR"),
    0x00c1_0001,
    0x0000_1401,
];

/// Nonzero words of the power-state object. The fast-die limiter reads its target
/// (`+0xc8`, 101.00 °C) and release (`+0xcc`, 96.00 °C) temperatures in 0.01 °C; left at zero
/// the limit is 0 °C and the GPU stays at its lowest state whenever the PMP runs.
pub(crate) const POWER_STATE_WORDS: Words = &[(0xc8, 10_100, 1), (0xcc, 9_600, 1)];

impl HwData25G83 {
    /// What [`Self::hash`] leaves out: the runtime words (completion descriptors, QoS pointer)
    /// and what comes from the device: the performance tables built from its operating points,
    /// which are binned per device, and its enabled core count.
    const UNHASHED: [(usize, usize); 10] = [
        (COMPLETION_DESCRIPTORS, 0x80),
        (QOS_POINTER, 8),
        (core::mem::offset_of!(Self, unk_f88), size_of::<u32>()),
        (core::mem::offset_of!(Self, unk_fc0), size_of::<u32>()),
        (core::mem::offset_of!(Self, perf_a), size_of::<PerfGroup>()),
        (core::mem::offset_of!(Self, perf_b_freq_mhz), size_of::<[u32; PERF_STATES]>()),
        (core::mem::offset_of!(Self, cs_perf), size_of::<AuxPerfTable>()),
        (core::mem::offset_of!(Self, afr_perf), size_of::<AuxPerfTable>()),
        (core::mem::offset_of!(Self, perf_b), size_of::<PerfGroup>()),
        (core::mem::offset_of!(Self, unk_2570), size_of::<u32>()),
    ];

    /// Hash of the constant contents (everything but [`Self::UNHASHED`]).
    pub(crate) fn hash(&self) -> u64 {
        // SAFETY: `HwData25G83` consists of integers only and has no padding.
        let bytes = unsafe {
            core::slice::from_raw_parts((self as *const Self).cast::<u8>(), size_of::<Self>())
        };
        hash_without(bytes, &Self::UNHASHED)
    }
}

/// FNV-1a 64 of `bytes` with the `(offset, length)` ranges in `skip` left out.
pub(crate) fn hash_without(bytes: &[u8], skip: &[(usize, usize)]) -> u64 {
    let mut hash = 0xcbf2_9ce4_8422_2325u64;
    for (offset, byte) in bytes.iter().enumerate() {
        if skip.iter().any(|&(at, len)| (at..at + len).contains(&offset)) {
            continue;
        }
        hash = (hash ^ u64::from(*byte)).wrapping_mul(0x100_0000_01b3);
    }
    hash
}

/// Hashes ([`hash_without`]) of the hardware data, power configuration and region C that the
/// Gravity Linux G16 encoders produce for the J616s, which this build's objects must equal.
pub(crate) mod expected {
    pub(crate) const HW_DATA: u64 = 0xfd3b_a26f_570f_57cd;
    pub(crate) const POWER_CONFIG: u64 = 0xb669_5b84_2ad2_36fe;
    pub(crate) const REGION_C: u64 = 0xd2db_fa6b_8976_47b4;
}
