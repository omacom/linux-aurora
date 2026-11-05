// SPDX-License-Identifier: GPL-2.0-only OR MIT
//! M5 Pro GPU firmware and address-space configuration.
//!
//! The M5 firmware builds its HW data, performance tables and region C in its own graph
//! ([`crate::g17::fw::m5::initdata`]), so the 26A428/25G83 graph tables of [`Config`] are empty.
use crate::g17::{Config, CoreLayout, CrashBuffers, FirmwareProfile, FixedBuffer, Identity};
use crate::hw::{GpuGen, GpuVariant};

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
    // The coprocessors request a crash log buffer at these addresses (primary first) when they
    // start and hand its contents back when they crash; outside the canonical range of dynamic
    // objects.
    crash_buffers: CrashBuffers::Fixed([
        FixedBuffer { va: 0x100_025b_4000, size: 0x8000 },
        FixedBuffer { va: 0x100_0267_0000, size: 0x8000 },
    ]),
    profile: FirmwareProfile::M5NoBareMetalTables,
    pm_metrics: None,
    windows: &[],
    reserved_slots: &[],
    perf: None,
    opps: 0,
    uat_quirks: crate::mmu::UatQuirks::DEFAULT,
    region_c: None,
    chip: None,
    adopt_root_entries: true,
    fabric_bridge: false,
    recovery_slot_keys: None,
    gpu_generation: GpuGen::G17,
    // No loaded image is admitted: the board has no bare-metal page tables yet.
    firmware: crate::g17::fw::variant::Firmware::Identified(&[]),
    // Two clusters of ten cores.
    core_layout: CoreLayout {
        cores: Some((2, 20)),
        alternate_core_masks: None,
        operand_state: crate::g17::fw::compute::OperandState::TWO_BY_TEN,
        deflake: crate::g17::fw::render::Deflake::TWO_CLUSTERS,
        compute_blocks: crate::g17::fw::freelist::COMPUTE_BLOCKS,
    },
};
