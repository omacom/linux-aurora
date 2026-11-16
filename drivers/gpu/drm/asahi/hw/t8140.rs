// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Hardware configuration for t8140 platforms (A18 Pro).

use crate::g17::{
    Config,
    CrashBuffers,
    FixedBuffer,
    Identity, //
};
use crate::hw::{GpuGen, GpuVariant};

/// Static configuration of the T8140 GPU.
pub(crate) const CONFIG: Config = Config {
    chip_id: CHIP_ID,
    identity: Identity {
        family: 0xa,
        variant: 0,
        num_dies: 1,
        gpu_variant: GpuVariant::P,
        usc_generation: 3,
        hal_generation: 200,
    },
    uat_ias: 42,
    uat_oas: 42,
    // The secondary coprocessor's kernel root follows the primary one's 256 KiB of page tables.
    secondary_root_offset: Some(0x40000),
    // The coprocessors request their crash log buffers at these addresses, in this order, and
    // expect them to be mapped before they start.
    crash_buffers: CrashBuffers::Fixed([
        FixedBuffer {
            va: 0x01e1_c000,
            size: 0x8000,
        },
        FixedBuffer {
            va: 0x01f5_0000,
            size: 0x8000,
        },
    ]),
    // Read by the firmware through the kernel lower root, whose top-level entries it only
    // samples when it starts.
    pm_metrics: FixedBuffer {
        va: 0x10_0008_0000,
        size: 0x8000,
    },
    recovery_slot_keys: Some(recovery::SLOT_KEYS),
    gpu_generation: GpuGen::G17,
    windows: &REGISTER_WINDOWS,
    reserved_slots: &RESERVED_REGISTER_SLOTS,
    perf: &PERF,
    opps: OPPS,
    uat_quirks: crate::mmu::UatQuirks::DEFAULT,
    region_c: &crate::g17::fw::initdata::REGION_C_26A428,
    adopt_root_entries: true,
    fabric_bridge: true,
    firmware: crate::g17::fw::variant::Firmware::Fixed(&crate::g17::fw::variant::V26A428),
    // The layouts the A18 Pro has always run with.
    core_layout: crate::g17::CoreLayout {
        cores: None,
        operand_state: crate::g17::fw::compute::OperandState::ONE_BY_SIX,
        deflake: crate::g17::fw::render::Deflake::ONE_CLUSTER,
    },
};

use crate::g17::fw::initdata::{PerfConfig, RegisterWindow, PERF_STATES};

/// Chip identifier written to the hardware data.
pub(crate) const CHIP_ID: u32 = 0x8140;

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

/// Register windows the firmware maps, by register-map slot.
pub(crate) const REGISTER_WINDOWS: [RegisterWindow; 18] = [
    window(0, 0x3_0101_4000, 0x0_0000, 0x4000, 0, 2),
    window(3, 0x2_2010_4000, 0x0_8000, 0x1_8000, 0, 2),
    window(9, 0x3_003d_0000, 0x2_8000, 0x1000, 0, 2),
    window(10, 0x3_003c_0000, 0x3_0000, 0x2000, 0, 0),
    window(12, 0x4_0165_c000, 0x3_8000, 0x4000, 0, 2),
    window(14, 0x3_0028_0000, 0x4_0000, 0x8000, 0, 0),
    window(17, 0x4_8000_0000, 0x5_0000, 0x2_1400, 0, 2),
    window(22, 0x4_8100_0000, 0x7_8000, 0x8000, 0, 2),
    window(26, 0x4_80d0_4000, 0x8_8000, 0x8000, 0xd0_4000, 2),
    window(27, 0x4_80d0_d000, 0x9_9000, 0x1000, 0xd0_d000, 2),
    window(28, 0x4_80d5_8000, 0xa_0000, 0x8000, 0xd5_8000, 2),
    window(29, 0x4_80d1_0000, 0xb_0000, 0x4000, 0xd1_0000, 2),
    window(31, 0x4_80d4_0000, 0xb_8000, 0x4000, 0xd4_0000, 2),
    window(32, 0x4_80d6_0000, 0xc_0000, 0x4000, 0xd6_0000, 2),
    window(35, 0x4_80e0_0000, 0xc_8000, 0x4000, 0xe0_0000, 2),
    window(39, 0x4_80e0_8000, 0xd_0000, 0x8000, 0, 2),
    window(40, 0x4_80e1_c000, 0xe_0000, 0x4000, 0xe1_c000, 2),
    window(41, 0x4_80e1_f800, 0xe_b800, 0x4000, 0, 2),
];

/// Register-map slots that are reserved but have no window.
pub(crate) const RESERVED_REGISTER_SLOTS: [usize; 17] = [
    2, 5, 6, 7, 8, 30, 33, 34, 37, 38, 42, 43, 46, 47, 48, 49, 52,
];

/// Performance-table configuration. The device tree lists the operating
/// points in performance-state order; table A uses the odd states above
/// state 5, table B the even ones.
pub(crate) const PERF: PerfConfig = PerfConfig {
    states_a: [0, 1, 2, 3, 4, 5, 7, 9, 11, 13, 15],
    states_b: [0, 1, 2, 3, 4, 5, 6, 8, 10, 12, 14],
    relative_a: [0, 15, 22, 29, 42, 56, 74, 86, 90, 92, 100],
    relative_b: [0, 0, 13, 24, 40, 52, 63, 73, 83, 88, 100],
    sram_min_mv: 765,
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

/// Offsets of the fixed objects in the kernel window, in the order the
/// builder creates them.
pub(crate) mod kernel_window {
    /// Private cluster (`initdata::private`).
    pub(crate) const PRIVATE_CLUSTER: u64 = 0x2_0000;
    /// Zeroed firmware-private page.
    pub(crate) const ZERO_PAGE: u64 = 0x15d_8000;
    /// Parameter-buffer descriptor table.
    pub(crate) const PB_DESCRIPTORS: u64 = 0x15e_0000;
    /// page-pool descriptor table.
    pub(crate) const PAGE_POOL_DESCRIPTORS: u64 = 0x15e_8000;
    /// Index object (64 KiB, `initdata::IndexTable` in its first page).
    pub(crate) const INDEX: u64 = 0xc084_8000;
    /// Firmware-control page.
    pub(crate) const FWCTL: u64 = 0x1c_0000;
    /// Render free-list control and state pages.
    pub(crate) const RENDER_FREE_LIST: u64 = 0xc083_0000;
    pub(crate) const RENDER_FREE_LIST_STATE: u64 = 0x160_8000;
    /// Boot free-list control and state pages.
    pub(crate) const BOOT_FREE_LIST: u64 = 0xc082_8000;
    pub(crate) const BOOT_FREE_LIST_STATE: u64 = 0x160_0000;
    /// Zeroed descriptor pages.
    pub(crate) const DESCRIPTOR_PAGE_A: u64 = 0x1c_8000;
    pub(crate) const DESCRIPTOR_PAGE_B: u64 = 0xc07c_0000;
    /// Zeroed TA and 3D context pages (128 KiB each).
    pub(crate) const TA_CONTEXT: u64 = 0x1d_8000;
    pub(crate) const FRAGMENT_CONTEXT: u64 = 0x20_0000;
    /// Compute free-list control page and the state page it names (the
    /// state page is not mapped at boot).
    pub(crate) const COMPUTE_FREE_LIST: u64 = 0xc086_8000;
    pub(crate) const COMPUTE_FREE_LIST_STATE: u64 = 0x163_0000;
    /// Second compute free-list control page and its state page.
    pub(crate) const COMPUTE_READY_FREE_LIST: u64 = 0xc087_8000;
    pub(crate) const COMPUTE_READY_FREE_LIST_STATE: u64 = 0x164_8000;
    /// Further compute free-list state page.
    pub(crate) const COMPUTE_STATE: u64 = 0x165_0000;
    /// Hardware-data bundle (`initdata::bundle`).
    pub(crate) const BUNDLE: u64 = 0xc078_8000;
}

/// Arenas used by dynamically allocated firmware objects.
pub(crate) mod dynamic {
    /// First dynamic byte, relative to the upper kernel window.
    pub(crate) const KERNEL_OFFSET: u64 = 0x168_4000;
    /// Client-private compute and render backing after the shared queue arena.
    pub(crate) const CLIENT_LOWER: core::ops::Range<u64> = 0x70_8000_0000..0x71_0000_0000;
    /// Canonical low mappings shared by hardware queues and their clients.
    /// Client-private allocations start after this arena.
    pub(crate) const LOWER: core::ops::Range<u64> = 0x70_0400_0000..0x70_8000_0000;
}

/// Fixed addresses in the firmware's context-0 and context-1 address spaces.
pub(crate) mod low {
    /// Context-0 alias of the parameter-buffer descriptor table.
    pub(crate) const PB_DESCRIPTORS: u64 = 0x70_0183_8000;
    /// Context-0 alias of the page-pool descriptor table.
    pub(crate) const PAGE_POOL_DESCRIPTORS: u64 = 0x70_0184_0000;
    /// Context-0 alias of the index object.
    pub(crate) const INDEX: u64 = 0x10_0019_0000;
    /// Context-0 aliases of the completion rings.
    pub(crate) const COMPLETION_RINGS: [u64; 2] = [0x70_03f0_0000, 0x70_03f2_8000];
    /// Context-0 run list of the compute free list.
    pub(crate) const COMPUTE_RUN_LIST: u64 = 0x70_03ad_8000;

    /// Render free list (context 1): page list.
    pub(crate) const RENDER_PAGE_LIST: u64 = 0x70_0000_0000;
    /// Size of the render page list.
    pub(crate) const RENDER_PAGE_LIST_SIZE: usize = 0x20_0000;
    /// Render free list: run list (64 KiB, zero at boot).
    pub(crate) const RENDER_RUN_LIST: u64 = 0x70_0020_8000;
    /// Size of the render run list.
    pub(crate) const RENDER_RUN_LIST_SIZE: usize = 0x1_0000;
    /// Render free list: first block.
    pub(crate) const RENDER_BLOCKS: u64 = 0x70_0022_0000;
    /// Distance between consecutive blocks.
    pub(crate) const BLOCK_STRIDE: u64 = 0x10_8000;
    /// Render blocks allocated at boot: the mapped blocks and the block that
    /// holds the compute page list.
    pub(crate) const RENDER_BLOCKS_ALLOCATED: usize = COMPUTE_PAGE_LIST_BLOCK + 1;
    /// Render blocks mapped at boot.
    pub(crate) const RENDER_BLOCKS_MAPPED: usize = 22;
    /// Render blocks in the free list at boot.
    pub(crate) const RENDER_BLOCKS_INITIAL: u32 = 17;
    /// Render block holding the compute page list.
    pub(crate) const COMPUTE_PAGE_LIST_BLOCK: usize = 22;

    /// Distance of the compute pool from the render pool.
    const COMPUTE: u64 = 0x200_0000;
    /// Compute pool: page list.
    pub(crate) const COMPUTE_POOL_PAGE_LIST: u64 = RENDER_PAGE_LIST + COMPUTE;
    /// Compute pool: run list.
    pub(crate) const COMPUTE_POOL_RUN_LIST: u64 = RENDER_RUN_LIST + COMPUTE;
    /// Compute pool: first block.
    pub(crate) const COMPUTE_POOL_BLOCKS: u64 = RENDER_BLOCKS + COMPUTE;
    /// Blocks the compute free list declares.
    pub(crate) const COMPUTE_BLOCKS: u32 = 18;
    /// Compute pool blocks in the second compute free list.
    pub(crate) const COMPUTE_READY_BLOCKS: u32 = 17;
    /// Page list of the compute free list (inside the block
    /// [`COMPUTE_PAGE_LIST_BLOCK`] holds, relocated to the compute pool).
    pub(crate) const COMPUTE_PAGE_LIST: u64 =
        RENDER_BLOCKS + COMPUTE_PAGE_LIST_BLOCK as u64 * BLOCK_STRIDE + COMPUTE;
    /// Two zeroed compute blocks.
    pub(crate) const COMPUTE_ZERO_BLOCKS: [u64; 2] = [
        RENDER_PAGE_LIST + COMPUTE,
        RENDER_PAGE_LIST + BLOCK_STRIDE + COMPUTE,
    ];

    /// Returns block `index` of the render pool.
    pub(crate) const fn render_block(index: usize) -> u64 {
        RENDER_BLOCKS + index as u64 * BLOCK_STRIDE
    }

    /// Returns block `index` of the compute pool.
    pub(crate) const fn compute_block(index: usize) -> u64 {
        COMPUTE_POOL_BLOCKS + index as u64 * BLOCK_STRIDE
    }

    /// Blocks of the compute free list, in run-list order: pool blocks 0 to
    /// 17, the two zeroed blocks, then pool block 18.
    pub(crate) const COMPUTE_FREE_LIST_BLOCKS: [u64; 21] = {
        let mut blocks = [0; 21];
        let mut index = 0;
        while index < 18 {
            blocks[index] = compute_block(index);
            index += 1;
        }
        blocks[18] = COMPUTE_ZERO_BLOCKS[0];
        blocks[19] = COMPUTE_ZERO_BLOCKS[1];
        blocks[20] = compute_block(18);
        blocks
    };
}

/// Client-root reservations and parameter-buffer geometry.
pub(crate) mod resources {
    use core::ops::Range;

    pub(crate) const UAT_PAGE: u64 = 0x4000;
    pub(crate) const USER_BASE: u64 = 0x10_0000_0000;
    pub(crate) const TIMESTAMPS: Range<u64> = 0x70_0000_0000..0x71_0000_0000;
    pub(crate) const COMMAND_TIMESTAMP_HZ: u64 = 24_000_000;
    pub(crate) const USER_END: u64 = USER_BASE + (1 << 32);
    pub(crate) const FIXED_PAIRS: usize = 2;
    pub(crate) const FIXED_PAIR_STRIDE: u64 = 0x8_0000;
    pub(crate) const SCENE: u64 = USER_BASE + 0x17_8000;
    pub(crate) const SCENE_SIZE: u64 = 0x8000;
    pub(crate) const DISCARD: u64 = USER_BASE + 0x18_0000;
    pub(crate) const DISCARD_SIZE: u64 = 0x8000;
    pub(crate) const PAGE_LIST: u64 = USER_BASE + 0x19_0000;
    pub(crate) const FRAGMENT_STATUS: u64 = USER_BASE + 0x1a_8000;
    pub(crate) const RENDER_ALIASES: Range<u64> = 0x10_0200_0000..0x10_0220_0000;
    pub(crate) const TARGETS: Range<u64> = 0x10_5000_0000..0x10_6400_0000;
    pub(crate) const TARGET_RECORD_MIN: usize = TVB_PAGE as usize;
    pub(crate) const TARGET_RECORD_MAX: usize = 0x80_0000;
    pub(crate) const TPC_MIN: usize = 0x3_0000;
    pub(crate) const TPC_MAX: usize = 0x200_0000;
    pub(crate) const AUXILIARY: Range<u64> = 0x100_0200_0000..0x100_0240_0000;
    pub(crate) const METRICS_SIZE: u64 = 0x8000;
    pub(crate) const TVB_PAGE: u64 = 0x8000;
    pub(crate) const TVB_BLOCK_SIZE: usize = 0x2_0000;
    pub(crate) const TVB_BLOCK_STRIDE: u64 = 0x2_8000;
    pub(crate) const TVB_BLOCK_SLOTS: usize = 0xd1a;
    pub(crate) const TVB_MAX_BLOCKS: usize = TVB_BLOCK_SLOTS - 1;
    pub(crate) const TVB_PAGES_PER_BLOCK: usize = 4;
    pub(crate) const TVB_PAGE_LIST_SIZE: usize = TVB_BLOCK_SLOTS * TVB_PAGES_PER_BLOCK * 4;
    pub(crate) const TVB_INITIAL_PAGES: [u32; 32] = [
        0x11, 0x16, 0x1b, 0x20, 0x25, 0x2a, 0x4a, 0x4f, 0x54, 0x59, 0x5e, 0x63, 0x68, 0x6d, 0x72,
        0x77, 0x7c, 0x81, 0x86, 0x8b, 0x90, 0x95, 0x9a, 0x9f, 0xa4, 0xa9, 0xae, 0xb3, 0xb8, 0xbd,
        0xc2, 0xc7,
    ];
    pub(crate) const TVB_GROWTH: u64 = USER_BASE + 0x0400_0000;
    pub(crate) const TVB_PAIR_STRIDE: u64 = 0x2800_0000;
    pub(crate) const RESERVED_RANGES: usize = 48;

    /// Compact allocations search these shared client arenas in this order.
    pub(crate) fn compact_ranges() -> [Range<u64>; 4] {
        let second = TVB_GROWTH + TVB_PAIR_STRIDE;
        [
            RENDER_ALIASES,
            TVB_GROWTH
                ..TVB_GROWTH + (TVB_MAX_BLOCKS - TVB_INITIAL_PAGES.len()) as u64 * TVB_BLOCK_STRIDE,
            second..second + TVB_MAX_BLOCKS as u64 * TVB_BLOCK_STRIDE,
            TARGETS,
        ]
    }

    /// All driver-owned windows reserved before the first userspace bind.
    /// Reserving them does not install pages or allocate backing memory.
    pub(crate) fn client_reserved_ranges(metrics_va: u64) -> Option<[Range<u64>; RESERVED_RANGES]> {
        let mut ranges = core::array::from_fn(|_| 0..0);
        let mut index = 0;
        let mut add = |range| {
            ranges[index] = range;
            index += 1;
        };
        add(0..0x1_0000);
        add(TIMESTAMPS);
        let page_list_size = (TVB_PAGE_LIST_SIZE as u64 + UAT_PAGE - 1) & !(UAT_PAGE - 1);
        for pair in 0..FIXED_PAIRS {
            let offset = pair as u64 * FIXED_PAIR_STRIDE;
            add(SCENE + offset..SCENE + offset + SCENE_SIZE);
            add(DISCARD + offset..DISCARD + offset + DISCARD_SIZE);
            add(PAGE_LIST + offset..PAGE_LIST + offset + page_list_size);
            add(FRAGMENT_STATUS + offset..FRAGMENT_STATUS + offset + UAT_PAGE);
        }
        add(RENDER_ALIASES);
        add(TARGETS);
        for page in TVB_INITIAL_PAGES {
            let start = USER_BASE + u64::from(page) * TVB_PAGE;
            add(start..start + TVB_BLOCK_STRIDE);
        }
        add(TVB_GROWTH
            ..TVB_GROWTH + (TVB_MAX_BLOCKS - TVB_INITIAL_PAGES.len()) as u64 * TVB_BLOCK_STRIDE);
        for pair in 1..FIXED_PAIRS {
            let start = TVB_GROWTH + pair as u64 * TVB_PAIR_STRIDE;
            add(start..start + TVB_MAX_BLOCKS as u64 * TVB_BLOCK_STRIDE);
        }
        add(AUXILIARY);
        add(metrics_va..metrics_va.checked_add(METRICS_SIZE)?);
        Some(ranges)
    }
}

/// Canonical userspace resource and its presentation in an execution context.
#[derive(Clone, Copy)]
pub(crate) struct ContextView {
    pub(crate) source: u64,
    pub(crate) low: u64,
    pub(crate) size: usize,
    pub(crate) write: bool,
}

const fn context_view(low: u64, size: usize, write: bool) -> ContextView {
    ContextView {
        source: 0x10_0000_0000 + low,
        low,
        size,
        write,
    }
}

pub(crate) const CONTEXT_VIEWS: [ContextView; 7] = [
    context_view(0, 0x10000, false),
    context_view(0x18000, 0x8000, false),
    context_view(0x28000, 0x8000, false),
    context_view(0x38000, 0x8000, false),
    context_view(0x48000, 0x8000, false),
    context_view(0x58000, 0x8000, true),
    context_view(0x68000, 0xc000, true),
];

/// Scheduler accounting policy and its firmware clock domain.
pub(crate) mod qos {
    pub(crate) const CLASS: u8 = 0x0f;
    pub(crate) const SHARE: u32 = 0x2000;
    pub(crate) const CLOCK_HZ: u64 = 24_000_000;
}

/// Firmware scheduling profiles of the DRM queue priorities. Native user
/// queues use priority class 2 with policy 2.
pub(crate) mod scheduling {
    use crate::g17::fw::queue::Policy;

    /// `DRM_ASAHI_PRIORITY_HIGH`: class 1, as the G13 tables map HIGH; the
    /// policy word of the user classes is kept (no macOS trace of class 1 yet).
    pub(crate) const HIGH: Policy = Policy::new(1, 2, super::qos::CLASS, super::qos::SHARE);
    /// `DRM_ASAHI_PRIORITY_MEDIUM`.
    pub(crate) const MEDIUM: Policy = Policy::new(2, 2, super::qos::CLASS, super::qos::SHARE);
    /// `DRM_ASAHI_PRIORITY_LOW`.
    pub(crate) const LOW: Policy = Policy::new(3, 2, super::qos::CLASS, super::qos::SHARE);
}

/// Physical workqueue capacity and logical-owner admission limits.
pub(crate) mod queues {
    pub(crate) const RENDER_SLOTS: usize = 64;
    // One physical pair per logical owner leaves room for every execution
    // context in the shared 128-QID namespace. Commands queue at RENDER_DEPTH,
    // as many as a pair has command records (the render memory's RECORDS).
    pub(crate) const RENDER_PAIRS_PER_OWNER: usize = 1;
    pub(crate) const RENDER_DEPTH: u8 = 32;
    pub(crate) const COMPUTE_DEPTH: usize = 16;
}

/// Read-only recovery evidence, sampled only for selected halted slots.
pub(crate) mod recovery {
    pub(crate) const SLOT_KEYS: u64 = 0x0000_0100_01ea_74b8;
}
