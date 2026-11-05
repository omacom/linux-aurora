// SPDX-License-Identifier: GPL-2.0-only OR MIT
//! M5 Pro, M5 Max and M5 Ultra GPU firmware and address-space configuration.
//!
//! The M5 firmware builds its HW data, performance tables and region C in its own graph
//! ([`crate::g17::fw::m5::initdata`]), so the 26A428/25G83 graph tables of [`Config`] are empty.
use crate::g17::{Config, CoreLayout, CrashBuffers, FirmwareProfile, FixedBuffer, Identity};
use crate::hw::{GpuGen, GpuVariant};
use crate::regs::G17Id;

// The coprocessors request a crash log buffer at these addresses (primary first) when they start and hand
// its contents back when they crash; outside the canonical range of dynamic objects. They are the
// ADT sgx gfx-data-shared-ro-base and gfx1-data-shared-ro-base (each 0x8000), so they differ per board.
const J714_CRASH_BUFFERS: [FixedBuffer; 2] = [
    FixedBuffer { va: 0x100_025b_4000, size: 0x8000 },
    FixedBuffer { va: 0x100_0267_0000, size: 0x8000 },
];

pub(crate) const CONFIG: Config = Config {
    chip_id: 0x6050,
    identity: Identity {
        family: 0xb,
        variant: 3,
        num_dies: 1,
        gpu_variant: GpuVariant::S,
        usc_generation: 3,
        hal_generation: 300,
    },
    uat_ias: 42,
    uat_oas: 42,
    // Unknown until the board has bare-metal page tables.
    secondary_root_offset: None,
    crash_buffers: CrashBuffers::Fixed(J714_CRASH_BUFFERS),
    profile: FirmwareProfile::M5NoBareMetalTables,
    pm_metrics: None,
    windows: &[],
    reserved_slots: &[],
    perf: None,
    opps: 0,
    uat_quirks: crate::mmu::UatQuirks::DEFAULT,
    region_c: None,
    chip: None,
    m5: Some(&crate::g17::fw::m5::initdata::J714),
    adopt_root_entries: true,
    fabric_bridge: false,
    recovery_slot_keys: None,
    gpu_generation: GpuGen::G17,
    // No loaded image is admitted: the board has no bare-metal page tables yet.
    firmware: crate::g17::fw::variant::Firmware::Identified(&[]),
    // Two clusters of ten cores. The buffers are sized for every core slot of the board, so the
    // board's cluster and slot counts admit the GPU, whichever cores are fused off.
    core_layout: CoreLayout {
        cores: None,
        alternate_core_masks: None,
        operand_state: crate::g17::fw::compute::OperandState::TWO_BY_TEN,
        deflake: crate::g17::fw::render::Deflake::TWO_CLUSTERS,
        compute_blocks: crate::g17::fw::freelist::COMPUTE_BLOCKS,
    },
};

// J716c ADT gfx-data-shared-ro-base and gfx1-data-shared-ro-base.
const J716C_CRASH_BUFFERS: [FixedBuffer; 2] = [
    FixedBuffer { va: 0x100_0283_c000, size: 0x8000 },
    FixedBuffer { va: 0x100_028f_8000, size: 0x8000 },
];

/// M5 Max: ID variant 4, GPU variant C, HAL generation 300 like the M5 Pro.
pub(crate) const CONFIG_MAX: Config = Config {
    chip_id: 0x6050,
    identity: Identity {
        family: 0xb,
        variant: 4,
        num_dies: 1,
        gpu_variant: GpuVariant::C,
        usc_generation: 3,
        hal_generation: 300,
    },
    uat_ias: 42,
    uat_oas: 42,
    // Unknown until the board has bare-metal page tables.
    secondary_root_offset: None,
    crash_buffers: CrashBuffers::Fixed(J716C_CRASH_BUFFERS),
    profile: FirmwareProfile::M5NoBareMetalTables,
    pm_metrics: None,
    windows: &[],
    reserved_slots: &[],
    perf: None,
    opps: 0,
    uat_quirks: crate::mmu::UatQuirks::DEFAULT,
    region_c: None,
    chip: None,
    m5: Some(&crate::g17::fw::m5::initdata::J716C),
    adopt_root_entries: true,
    fabric_bridge: false,
    recovery_slot_keys: None,
    gpu_generation: GpuGen::G17,
    // No loaded image is admitted: the board has no bare-metal page tables yet.
    firmware: crate::g17::fw::variant::Firmware::Identified(&[]),
    // Four clusters of ten cores. The buffers are sized for every core slot of the board, so the
    // board's cluster and slot counts admit the GPU, whichever cores are fused off.
    core_layout: CoreLayout {
        cores: None,
        alternate_core_masks: None,
        operand_state: crate::g17::fw::compute::OperandState::FOUR_BY_TEN,
        deflake: crate::g17::fw::render::Deflake::FOUR_CLUSTERS,
        compute_blocks: crate::g17::fw::freelist::COMPUTE_BLOCKS,
    },
};

/// M5 Ultra (J775D, two dies, G17D) on bare metal: both roles publish their kernel roots, the
/// secondary's 0x40000 into `pagetables`, and each asks for its crash log inside `fw-data`. The
/// UAT tables are no-map reservations outside /memory; the rest of the UAT policy is the one
/// this board has run with: every table change written back, page-by-page TLB invalidation, and
/// the handoff values its firmware expects (current slot 0xffff, the byte after the flush records
/// 1, the lock never taken).
pub(crate) const CONFIG_ULTRA: Config = Config {
    chip_id: 0x6050,
    // Family 0xb (G17), variant 6 (G17D), revision 0x11.
    identity: Identity {
        family: 0xb,
        variant: 6,
        num_dies: 2,
        gpu_variant: GpuVariant::D,
        usc_generation: 3,
        hal_generation: 300,
    },
    uat_ias: 42,
    uat_oas: 42,
    secondary_root_offset: Some(0x40000),
    crash_buffers: CrashBuffers::FirmwareData,
    profile: FirmwareProfile::M5PublishedRoots,
    pm_metrics: None,
    windows: &[],
    reserved_slots: &[],
    perf: None,
    opps: 0,
    uat_quirks: crate::mmu::UatQuirks {
        reserved_tables: true,
        clean_tables: true,
        page_tlbi: true,
        handoff_lock: false,
        handoff_slot: 0xffff,
        handoff_unk2: 1,
        native_gpu_permissions: false,
    },
    region_c: None,
    chip: None,
    m5: Some(&crate::g17::fw::m5::initdata::J775D),
    adopt_root_entries: true,
    fabric_bridge: false,
    recovery_slot_keys: None,
    gpu_generation: GpuGen::G17,
    firmware: crate::g17::fw::variant::Firmware::Fixed(&crate::g17::fw::variant::V26A428_M5),
    // Eight clusters of ten cores. M5 renders lay out their tiler context store themselves
    // (`fw::m5::render_registers`), so the deflake layout is the M5 Pro's. As on the M5 Pro, the
    // buffers are sized for all 80 core slots and the board admits the GPU by clusters and slots:
    // parts with fused-off cores (64 enabled) take the same layout.
    core_layout: CoreLayout {
        cores: None,
        alternate_core_masks: None,
        operand_state: crate::g17::fw::compute::OperandState::EIGHT_BY_TEN,
        deflake: crate::g17::fw::render::Deflake::TWO_CLUSTERS,
        compute_blocks: crate::g17::fw::freelist::COMPUTE_BLOCKS,
    },
};

/// Every T6050 board binds the same compatible; the ID registers select the configuration.
pub(crate) fn config(id: &G17Id) -> Option<&'static Config> {
    [&CONFIG, &CONFIG_MAX, &CONFIG_ULTRA].into_iter().find(|cfg| id.family == cfg.identity.family
        && id.variant == cfg.identity.variant && id.num_dies == cfg.identity.num_dies)
}
