// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Hardware configuration for t6040 platforms (M4 Pro).
//!
//! The M4 Pro runs the macOS 26.6 GPU firmware (25G83), which has the A18 Pro firmware's
//! interface; [`fw::variant::V25G83`] describes where it differs.

use crate::g17::fw::{
    self,
    initdata::{v25g83, PerfConfig, RegionCConfig, RegisterWindow, PERF_STATES},
};
use crate::g17::{Config, CoreLayout, CrashBuffers, FixedBuffer, Identity};
use crate::hw::{GpuGen, GpuVariant};
use crate::mmu::UatQuirks;

/// Static configuration of the T6040 GPU.
pub(crate) const CONFIG: Config = Config {
    chip_id: CHIP_ID,
    identity: Identity {
        family: 0xa,
        // ID register 0x0a03_1100: family 0xa, variant 3, revision 0x11 (B1).
        variant: 3,
        num_dies: 1,
        gpu_variant: GpuVariant::S,
        usc_generation: 3,
        hal_generation: 200,
    },
    uat_ias: 42,
    uat_oas: 42,
    // One coprocessor carries both firmware roles; `pagetables` holds only its root.
    secondary_root_offset: None,
    crash_buffers: CrashBuffers::FirmwareData,
    // Read by the firmware through the kernel lower root, whose top-level entries it only
    // samples when it starts.
    pm_metrics: FixedBuffer {
        va: 0x10_0008_0000,
        size: 0x8000,
    },
    windows: &REGISTER_WINDOWS,
    reserved_slots: &[],
    perf: &PERF,
    opps: OPPS,
    // The no-map tables are needed until the boot loader adds the UAT ranges to /memory. The
    // rest is the policy the M4 Pro has run with; none of it was tested otherwise.
    uat_quirks: UatQuirks {
        reserved_tables: true,
        clean_tables: true,
        page_tlbi: true,
        handoff_lock: false,
        handoff_slot: 0xffff_ffff,
        native_gpu_permissions: true,
    },
    region_c: &REGION_C,
    adopt_root_entries: false,
    fabric_bridge: false,
    // The firmware's per-slot recovery keys are not known for this build; recovery relies on
    // the firmware's own blame.
    recovery_slot_keys: None,
    // Userspace drives this GPU as a G16 part.
    gpu_generation: GpuGen::G16,
    // The macOS 26.6.2 boot firmware (iBoot mBoot-18000.161.10) loads the 25G83 GPU firmware.
    firmware: fw::variant::Firmware::Identified(&[fw::variant::Build {
        os_version: [26, 6, 2],
        name: "macOS 26.6.2 (25G83, iBoot mBoot-18000.161.10)",
        variant: &fw::variant::V25G83,
    }]),
    // Measured on the 20-core M4 Pro; other core counts are refused at probe.
    core_layout: CoreLayout {
        cores: Some((2, 20)),
        operand_state: fw::compute::OperandState::TWO_BY_TEN,
        deflake: fw::render::Deflake::TWO_CLUSTERS,
    },
};

/// Chip identifier written to the hardware data.
pub(crate) const CHIP_ID: u32 = 0x6040;

/// Base of the firmware MMIO aperture in the kernel window.
const MMIO_APERTURE: u64 = 0xffff_fc21_8000_0000;

const fn window(
    slot: usize,
    phys: u64,
    offset: u64,
    size: u32,
    unk_18: u64,
    unk_20: u32,
) -> RegisterWindow {
    RegisterWindow {
        slot,
        phys,
        va: MMIO_APERTURE + offset,
        size,
        unk_18,
        unk_20,
        instances: 1,
        phys_stride: 0,
    }
}

/// `window` repeated `instances` times, `phys_stride` apart in physical memory.
const fn instances(window: RegisterWindow, instances: u32, phys_stride: u64) -> RegisterWindow {
    RegisterWindow {
        instances,
        phys_stride,
        ..window
    }
}

/// Register windows the firmware maps, by register-map slot. Slot 3 holds the four memory
/// controllers (ADT /arm-io/mcc), slot 20 the four AFR blocks.
pub(crate) const REGISTER_WINDOWS: [RegisterWindow; 25] = [
    window(0, 0x5_0241_4000, 0x20_0000, 0x4000, 0, 2),
    instances(window(3, 0x2_201c_4000, 0x20_8000, 0x1_8000, 0, 2), 4, 0x200_0000),
    window(4, 0x5_0244_c000, 0x26_c000, 0x4000, 0, 0),
    window(9, 0x5_003d_0000, 0x27_4000, 0x4000, 0, 2),
    window(10, 0x5_003c_0000, 0x27_c000, 0x4000, 0, 0),
    window(11, 0x5_003d_8000, 0x28_4000, 0x4000, 0, 2),
    window(12, 0x4_8565_c000, 0x28_c000, 0x4000, 0, 2),
    window(14, 0x5_0040_0000, 0x29_4000, 0x1_0000, 0, 0),
    window(15, 0x5_0cdc_0000, 0x2a_8000, 0x4000, 0, 0),
    window(17, 0x2_8800_0000, 0x2b_0000, 0x2_1400, 0, 2),
    instances(window(20, 0x2_8b00_0000, 0x2d_8000, 0x4_0000, 0, 2), 4, 0x20_0000),
    window(22, 0x2_8900_0000, 0x3d_c000, 0x8000, 0, 2),
    window(26, 0x2_88d0_4000, 0x3e_8000, 0x8000, 0xd0_4000, 2),
    window(28, 0x2_88d5_8000, 0x3f_4000, 0x8000, 0xd5_8000, 2),
    window(29, 0x2_88d1_0000, 0x40_0000, 0x4000, 0xd1_0000, 2),
    window(31, 0x2_88d4_0000, 0x40_8000, 0x4000, 0xd4_0000, 2),
    window(32, 0x2_88d6_0000, 0x41_0000, 0x2_0000, 0xd6_0000, 2),
    window(33, 0x2_88d4_4000, 0x43_4000, 0x4000, 0xd4_4000, 2),
    window(34, 0x2_88d4_c000, 0x43_c000, 0x200, 0xd4_c000, 2),
    window(35, 0x2_88e0_0000, 0x44_4000, 0x4000, 0xe0_0000, 2),
    window(37, 0x2_88e4_0000, 0x44_c000, 0x4000, 0xe4_0000, 2),
    window(39, 0x2_88e0_8000, 0x45_4000, 0x8000, 0, 2),
    window(40, 0x2_88e1_c000, 0x46_0000, 0x4000, 0xe1_c000, 2),
    window(41, 0x2_88e1_f800, 0x46_b800, 0x4000, 0, 2),
    window(43, 0x2_88e6_0000, 0x47_4000, 0x58, 0xe6_0000, 2),
];

/// Performance-table configuration. The device tree lists the sixteen operating points in
/// performance-state order, with one voltage per cluster.
pub(crate) const PERF: PerfConfig = PerfConfig {
    states_a: [0, 1, 2, 3, 4, 6, 8, 10, 12, 14, 15],
    states_b: [0, 1, 2, 3, 4, 6, 7, 9, 11, 13, 15],
    relative_a: [0, 9, 16, 23, 29, 38, 49, 60, 69, 79, 100],
    relative_b: [0, 0, 22, 36, 47, 57, 68, 78, 84, 91, 100],
    sram_min_mv: 810,
};

/// Operating points the performance tables reference.
pub(crate) const OPPS: usize = 16;

const _: () = {
    let mut index = 0;
    while index < PERF_STATES {
        assert!((PERF.states_a[index] as usize) < OPPS);
        assert!((PERF.states_b[index] as usize) < OPPS);
        index += 1;
    }
};

/// Region C.
const REGION_C: RegionCConfig = RegionCConfig {
    unk_54: 0x0fa0,
    unk_78: [1, 0, 0],
    max_power_mw: v25g83::MAX_POWER_MW,
    unk_9c: 0,
    unk_a0: 0,
    unk_7c4: 1500,
    unk_9e0: [
        1,
        0x0102_0200,
        0x010a_2801,
        0x2802_0101,
        0x0101_010a,
        0x010a_2802,
        0x2801_0101,
        0x0001_010a,
    ],
};
