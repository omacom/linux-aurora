// SPDX-License-Identifier: GPL-2.0-only
// Copyright The Gravity Linux Contributors
//
// The power configuration words, leakage coefficients, power limit and AON rows of this
// profile are what the M4 GPU encoders of Gravity Linux (g16_fw.rs, g16_platform.rs and
// g16_power.rs, written by Cody Ho, with the bootloader power limit added by Niklas Sheth)
// built on a 32-core J616C:
// https://github.com/GravityLinux/linux/commit/bd25d194b0a295a75692a0e6caed3793603b9e65
// https://github.com/GravityLinux/linux/commit/3a2ad760d7316813f37c0ddc653ef783827f855b

//! T6041 (M4 Max) with 32 cores on the 25G83 firmware build, as the J616C runs it.
//!
//! The J616C boots through a macOS 26.6.2 stub (iBoot mBoot-18000.161.10), which loads the
//! 25G83 image [`t6041::BUILD_25G83`] admits. Its GPU has 32 of 40 cores in four clusters, so
//! the 40-core configuration of that build refuses it; this one is selected instead.
//!
//! What differs between units comes from the device tree, which the boot loader fills from the
//! ADT: the operating points with one voltage per cluster (`operating-points-v2`, from the
//! ADT `perf-states` tables) and the CS and AFR tables (`apple,cs-opp`, `apple,afr-opp`, from
//! `cs-perf-states` and `afr-perf-states`). The builders turn them into the performance, CS and
//! AFR tables of the hardware data. The enabled cores are probed from the GPU.
//!
//! Everything else is static, because the driver has no source for it on T6041:
//! - [`PERF`]: the selector ladders, see there.
//! - [`MAX_POWER_MW`] and the leakage coefficients of [`CHIP`]: not in the ADT. They are the
//!   M4 Pro's values (its per-core power budget for 32 cores, its per-cluster coefficients for
//!   four clusters), not values of this unit.
//! - [`AON_TABLE`]: the GPU cluster and SoC rows of the ADT's `/arm-io/aon-ptd`
//!   `ptd_data_table`, which the loader does not publish.
//! - [`REGISTER_WINDOWS`]: slot 3 holds this part's six memory controllers (ADT
//!   `/arm-io/sgx` `active-mcc-bit-vector` 0x3f).
//!
//! Built from the device tree of the J616C, the hardware data and region C equal what this
//! unit ran with since October 2026, and the power configuration does with [`CHIP`]'s words.

use super::t6041;
use super::t604x::{instances, window};
use crate::g17::fw::{
    self,
    initdata::{v25g83, PerfConfig, RegionCConfig, RegisterWindow, PERF_STATES},
};
use crate::g17::{Config, CoreLayout};

pub(super) const CONFIG: Config = Config {
    windows: &REGISTER_WINDOWS,
    reserved_slots: &[],
    perf: Some(&PERF),
    region_c: Some(&REGION_C),
    chip: Some(&CHIP),
    firmware: fw::variant::Firmware::Identified(&[t6041::BUILD_25G83, t6041::BUILD_26B123]),
    core_layout: CoreLayout {
        cores: Some((4, 32)),
        alternate_core_masks: None,
        // Banks for all forty core slots: the firmware indexes them by physical slot, and the
        // J616C's enabled slots run to 39 (core mask 0xf6_fdbf6fdb). With banks for the 32
        // enabled cores only, a compute load beside rendering hit reason-0 recoveries.
        operand_state: fw::compute::OperandState::FOUR_BY_TEN,
        deflake: fw::render::Deflake::FOUR_CLUSTERS,
        // The compute pool the J616C has run with since 23 Oct 2026; the M4 Pro's 21 blocks
        // hang this GPU's firmware on grids of more than ten workgroups.
        compute_blocks: 96,
    },
    ..t6041::BASE
};

/// The register map of [`t6041::REGISTER_WINDOWS_25G83`] with six memory controller
/// instances in slot 3 instead of eight, laid out by the same rule, so every window after it
/// lies 0x30000 lower.
const REGISTER_WINDOWS: [RegisterWindow; 25] = [
    window(0, 0x5_0241_4000, 0x20_0000, 0x4000, 0, 2),
    instances(
        window(3, 0x2_201c_4000, 0x20_8000, 0x1_8000, 0, 2),
        6,
        0x200_0000,
    ),
    window(4, 0x5_0244_c000, 0x29_c000, 0x4000, 0, 0),
    window(9, 0x5_003d_0000, 0x2a_4000, 0x4000, 0, 2),
    window(10, 0x5_003c_0000, 0x2a_c000, 0x4000, 0, 0),
    window(11, 0x5_003d_8000, 0x2b_4000, 0x4000, 0, 2),
    window(12, 0x4_8565_c000, 0x2b_c000, 0x4000, 0, 2),
    window(14, 0x5_0040_0000, 0x2c_4000, 0x1_0000, 0, 0),
    window(15, 0x5_0cdc_0000, 0x2d_8000, 0x4000, 0, 0),
    window(17, 0x2_8800_0000, 0x2e_0000, 0x2_1400, 0, 2),
    instances(
        window(20, 0x2_8b00_0000, 0x30_8000, 0x4_0000, 0, 2),
        4,
        0x20_0000,
    ),
    window(22, 0x2_8900_0000, 0x40_c000, 0x8000, 0, 2),
    window(26, 0x2_88d0_4000, 0x41_8000, 0x8000, 0xd0_4000, 2),
    window(28, 0x2_88d5_8000, 0x42_4000, 0x8000, 0xd5_8000, 2),
    window(29, 0x2_88d1_0000, 0x43_0000, 0x4000, 0xd1_0000, 2),
    window(31, 0x2_88d4_0000, 0x43_8000, 0x4000, 0xd4_0000, 2),
    window(32, 0x2_88d6_0000, 0x44_0000, 0x2_0000, 0xd6_0000, 2),
    window(33, 0x2_88d4_4000, 0x46_4000, 0x4000, 0xd4_4000, 2),
    window(34, 0x2_88d4_c000, 0x46_c000, 0x200, 0xd4_c000, 2),
    window(35, 0x2_88e0_0000, 0x47_4000, 0x4000, 0xe0_0000, 2),
    window(37, 0x2_88e4_0000, 0x47_c000, 0x4000, 0xe4_0000, 2),
    window(39, 0x2_88e0_8000, 0x48_4000, 0x8000, 0, 2),
    window(40, 0x2_88e1_c000, 0x49_0000, 0x4000, 0xe1_c000, 2),
    window(41, 0x2_88e1_f800, 0x49_b800, 0x4000, 0, 2),
    window(43, 0x2_88e6_0000, 0x4a_4000, 0x58, 0xe6_0000, 2),
];

// The windows of the 40-core 25G83 map, with six controllers, laid out by its rule.
const _: () = {
    assert!(super::t604x::laid_out_25g83(&REGISTER_WINDOWS));
    let mut index = 0;
    while index < REGISTER_WINDOWS.len() {
        let (max, ours) = (
            &t6041::REGISTER_WINDOWS_25G83[index],
            &REGISTER_WINDOWS[index],
        );
        assert!(
            max.slot == ours.slot
                && max.phys == ours.phys
                && max.size == ours.size
                && max.unk_18 == ours.unk_18
                && max.unk_20 == ours.unk_20
                && max.phys_stride == ours.phys_stride
                && (max.instances == ours.instances || (max.slot == 3 && ours.instances == 6))
        );
        index += 1;
    }
};

/// The selector ladders. `states_a` and `states_b` are the highest and lowest of the J616C's
/// operating points at each of its 11 core voltages, in ADT order; the 40-core 25F84 profile
/// selects OPP 6 for state 5 of `states_b`, this one OPP 5. The relative ladders are the M4
/// Pro's. The driver has no source for them on T6041.
const PERF: PerfConfig = PerfConfig {
    states_a: [0, 1, 2, 3, 4, 6, 8, 10, 12, 14, 15],
    states_b: [0, 1, 2, 3, 4, 5, 7, 9, 11, 13, 15],
    relative_a: [0, 9, 16, 23, 29, 38, 49, 60, 69, 79, 100],
    relative_b: [0, 0, 22, 36, 47, 57, 68, 78, 84, 91, 100],
    sram_min_mv: 810,
};

const _: () = {
    let mut index = 0;
    while index < PERF_STATES {
        assert!((PERF.states_a[index] as usize) < t6041::OPPS);
        assert!((PERF.states_b[index] as usize) < t6041::OPPS);
        index += 1;
    }
};

/// The M4 Pro's power budget per core (69116 mW for 20) for 32 cores.
const MAX_POWER_MW: u32 = 110_586;

const REGION_C: RegionCConfig = RegionCConfig {
    max_power_mw: MAX_POWER_MW,
    ..t6041::REGION_C
};

/// The AON temperature-source rows (12 bytes each) the power configuration names: GPU clusters
/// 0 to 3 and the SoC, as in the ADT's `ptd_data_table`.
const AON_TABLE: [u32; 15] = [
    u32::from_be_bytes(*b"g0TR"),
    0x00c2_0001,
    0x0000_1401,
    u32::from_be_bytes(*b"g1TR"),
    0x0097_0001,
    0x0000_1401,
    u32::from_be_bytes(*b"g2TR"),
    0x0092_0001,
    0x0000_1401,
    u32::from_be_bytes(*b"g3TR"),
    0x00c7_0001,
    0x0000_1401,
    u32::from_be_bytes(*b"r0TR"),
    0x012c_0001,
    0x0000_1401,
];

const CHIP: v25g83::Chip = v25g83::Chip {
    max_power_mw: MAX_POWER_MW,
    // The M4 Pro's per-cluster core coefficients for four clusters, its SRAM, CS and AFR ones.
    leakage: {
        let mut words = [0; 56];
        words[0] = 0x44f0_3000;
        words[1] = 0x44f3_0000;
        words[2] = 0x44f0_3000;
        words[3] = 0x44f3_0000;
        words[32] = 0x4373_0000;
        words[33] = 0x4373_0000;
        words[34] = 0x4373_0000;
        words[35] = 0x4373_0000;
        words[52] = 0x437b_0000;
        words[54] = 0x43bd_c000;
        words
    },
    aon_table: &AON_TABLE,
    hw_data_words: &[],
    power_config_words: POWER_CONFIG_WORDS,
    perf_a_replicate_lanes: false,
    cs_perf_table: true,
};

/// What differs from the M4 Pro's power configuration besides the power limit and leakage.
const POWER_CONFIG_WORDS: &[(usize, u32, usize)] = &[
    // -1000 / MAX_POWER_MW, as the M4 Pro's word is for its 69116 mW.
    (0x9ec, 0xbc14_27fc, 1),
    // The second temperature-sensor mask word.
    (0x3b5c, 0x2a22_2aa2, 1),
    // The temperature sources of GPU clusters 2 and 3.
    (0x6328, u32::from_be_bytes(*b"g2TR"), 1),
    (0x632c, u32::from_be_bytes(*b"g3TR"), 1),
    (0x6360, 0x5, 1),
];
