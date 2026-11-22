// SPDX-License-Identifier: GPL-2.0-only OR MIT
//! J873 fixed VA reservations. Addresses are policy, never captured pointers.
//! The allocator owns suballocations inside the named legacy arenas. Compact
//! aliases intentionally share physical backing but have disjoint VA extents.
#![allow(dead_code)]

pub(crate) const PAGE: u64 = 0x4000;
pub(crate) const KERNEL_BASE: u64 = 0xffff_fc20_0000_0000;
pub(crate) const COMPACT_BIAS: u64 = 0x10_0000_0000;
// Optional H1 control: the first Gallium VDM address from retained run123.
pub(crate) const GRAPHICS_HIGH_VDM: u64 = 0x5f_ffe2_8000;
// Native PPP page 0x58000 aliases the first Gallium PPP page. Preserve offsets
// between all four native PPP pages; these aliases share original backing.
pub(crate) const GRAPHICS_HIGH_PPP_BASE: u64 = 0x5f_ffea_c000;
pub(crate) const GRAPHICS_HIGH_PPP_PAGES: [u64; 4] = [0, 0x48000, 0x58000, 0x68000];
pub(crate) const PB_TABLE: u64 = KERNEL_BASE + 0x015e_0000;
pub(crate) const UMA_TABLE: u64 = KERNEL_BASE + 0x015e_8000;
pub(crate) const COMPUTE_GRAPH: u64 = KERNEL_BASE + 0xc000_0000;
pub(crate) const COMPUTE_ENTRY_HIGH: u64 = KERNEL_BASE + 0x0039_0000;
pub(crate) const COMPUTE_ENTRY_LOW: u64 = 0x70_005f_0000;
pub(crate) const DYNAMIC_HIGH_START: u64 = KERNEL_BASE + 0x0168_0000;
pub(crate) const DYNAMIC_HIGH_END: u64 = KERNEL_BASE + 0x1000_0000;
pub(crate) const POOL_DIRECTORY: u64 = 0x70_0200_0000;
pub(crate) const POOL_BLOCK_LIST: u64 = 0x70_0400_0000;
pub(crate) const POOL_BLOCKS: u64 = 0x70_0500_0000;
pub(crate) const POOL_BLOCK_STRIDE: u64 = 0x20_0000;
// Job-VM-only 0x1a510 object for g18_app_preempt_mode=1: the first pages of
// the VM_CREATE auxiliary driver arena (m5_vm::auxiliary_range), which
// gem_bind refuses to userspace. Native uses a 48 KiB process object.
pub(crate) const APPLICATION_COMPUTE_PREEMPT_DRIVER: u64 = COMPACT_BIAS + (1 << 32);
pub(crate) const APPLICATION_COMPUTE_PREEMPT_DRIVER_BYTES: u64 = 0xc000;

// Graphics reservations are beyond both the compute graph and its low arena.
// Queue/PB owner integration must consume these, or update this table as one
// unit; the legacy M5 graph/entries collide with COMPUTE_GRAPH/ENTRY_*.
pub(crate) const GRAPHICS_QUEUE_HIGH: u64 = KERNEL_BASE + 0xc200_0000;
pub(crate) const GRAPHICS_QUEUE_BYTES: usize = 0x40000;
pub(crate) const GRAPHICS_ENTRY_HIGH: u64 = KERNEL_BASE + 0x0050_0000;
pub(crate) const GRAPHICS_ENTRY_LOW: u64 = 0x70_1100_0000;
pub(crate) const GRAPHICS_ENTRY_BYTES: usize = 0x40000;
pub(crate) const GRAPHICS_JOB_LIST: u64 = GRAPHICS_QUEUE_HIGH + 0x400;
pub(crate) const GRAPHICS_QUEUE_CONTROL: u64 = GRAPHICS_QUEUE_HIGH + 0x500;
pub(crate) const GRAPHICS_JOB_HIGH: u64 = KERNEL_BASE + 0xc210_0000;
pub(crate) const GRAPHICS_JOB_LOW: u64 = 0x70_1200_0000;
pub(crate) const GRAPHICS_JOB_BYTES: usize = 0x40000;
pub(crate) const GRAPHICS_SIGNALS: u64 = KERNEL_BASE + 0xc218_0000;
pub(crate) const GRAPHICS_SIGNALS_BYTES: usize = 0x10000;
pub(crate) const GRAPHICS_SCHEDULER_START: u64 = KERNEL_BASE + 0xc220_0000;
pub(crate) const GRAPHICS_SCHEDULER_END: u64 = GRAPHICS_SCHEDULER_START + 0x8000;
pub(crate) const GRAPHICS_PB_HIGH: u64 = KERNEL_BASE + 0xc230_0000;
pub(crate) const GRAPHICS_PB_BYTES: usize = 0x100000;
pub(crate) const GRAPHICS_UMA_HIGH: u64 = KERNEL_BASE + 0xc300_0000;
pub(crate) const GRAPHICS_UMA_LIST: u64 = GRAPHICS_UMA_HIGH + 0x34000;
pub(crate) const GRAPHICS_UMA_STATE: u64 = GRAPHICS_UMA_HIGH + 0x38000;
pub(crate) const GRAPHICS_UMA_COUNTER: u64 = GRAPHICS_UMA_HIGH + 0x3c000;
pub(crate) const GRAPHICS_UMA_BACKING: u64 = GRAPHICS_UMA_HIGH + 0x100000;
pub(crate) const GRAPHICS_UMA_BYTES: u64 = 0x3000000;
pub(crate) const GRAPHICS_UMA_GROWTH: u64 = GRAPHICS_UMA_HIGH + 0x2a00000;
pub(crate) const GRAPHICS_IDEAL_BLOCKS: u64 = 0x70_2000_0000;
pub(crate) const GRAPHICS_TARGET_OFFSET: u64 = 0x0100_0000;
pub(crate) const GRAPHICS_TARGET_BYTES: usize = 0x0100_0000;
pub(crate) const GRAPHICS_PB_OFFSET: u64 = 0x0200_0000;
// PB blocks grow to render_policy::PB_MAX_BLOCKS. The biased half stays
// inside the COMPACT_BIAS + [32 MiB, 2 GiB) that Mesa leaves to the kernel.
pub(crate) const GRAPHICS_PB_COMPACT_BYTES: usize = 0x3400_0000;
pub(crate) const PB_GRAPH: u64 = GRAPHICS_PB_HIGH + 0x4000;
pub(crate) const PB_PAGE_LIST_HIGH: u64 = GRAPHICS_PB_HIGH + 0x10000;
pub(crate) const PB_PAGE_LIST_LOW: u64 = GRAPHICS_PB_OFFSET;
pub(crate) const PB_PAGE_LIST_BYTES: usize = 0x1c000;
pub(crate) const PB_METRICS_HIGH: u64 = GRAPHICS_PB_HIGH + 0x30000;
pub(crate) const PB_METRICS_LOW: u64 = GRAPHICS_PB_OFFSET + 0x20000;
pub(crate) const PB_SCENE_SCRATCH: u64 = GRAPHICS_PB_OFFSET + 0x28000;
pub(crate) const PB_DISCARD: u64 = GRAPHICS_PB_OFFSET + 0x30000;
pub(crate) const PB_BLOCKS: u64 = GRAPHICS_PB_OFFSET + 0x100000;
pub(crate) const QUEUE_IDS: [u8; 2] = [9, 10];

#[derive(Clone, Copy, Debug)]
pub(crate) struct Region { pub name: &'static str, pub base: u64, pub bytes: u64 }
const fn r(name: &'static str, base: u64, bytes: u64) -> Region { Region { name, base, bytes } }

pub(crate) const REGIONS: &[Region] = &[
    r("capture-compact", 0, 0x78000),
    r("capture-compact-biased", COMPACT_BIAS, 0x78000),
    r("capture-high-vdm-control", GRAPHICS_HIGH_VDM, PAGE),
    r("capture-high-ppp-0", GRAPHICS_HIGH_PPP_BASE - 0x58000, PAGE),
    r("capture-high-ppp-48000", GRAPHICS_HIGH_PPP_BASE - 0x10000, PAGE),
    r("capture-high-ppp-58000", GRAPHICS_HIGH_PPP_BASE, PAGE),
    r("capture-high-ppp-68000", GRAPHICS_HIGH_PPP_BASE + 0x10000, PAGE),
    r("capture-data", 0x100_0000_0000, 0x3d8000),
    r("init-private-roots-fwctl", KERNEL_BASE + 0x20000, 0x1a4000),
    r("pb-table", PB_TABLE, 0x4000), r("uma-table", UMA_TABLE, 0x8000),
    r("dynamic-high-pool-scheduler-compute", DYNAMIC_HIGH_START,
        DYNAMIC_HIGH_END - DYNAMIC_HIGH_START),
    r("compute-queue", COMPUTE_GRAPH, PAGE),
    r("compute-entry-high", COMPUTE_ENTRY_HIGH, 0x20000),
    r("compute-entry-low", COMPUTE_ENTRY_LOW, 0x20000),
    r("init-shared", KERNEL_BASE + 0xc0788000, 0x28000),
    r("init-sentinel", KERNEL_BASE + 0xc07e0000, PAGE),
    r("completion-high", KERNEL_BASE + 0xc07e8000, 4 * 0x28000),
    r("pb-table-low", 0x70_0183_8000, 0x4000),
    r("uma-table-low", 0x70_0184_0000, 0x8000),
    r("completion-low", 0x70_0184_8000, 4 * 0x28000),
    r("pool-directory-reservation", POOL_DIRECTORY, 0x2000000),
    r("pool-block-list", POOL_BLOCK_LIST, PAGE),
    r("application-compute-directory", POOL_DIRECTORY + 0x40000000, 0x2000000),
    r("application-compute-block-list", POOL_BLOCK_LIST + 0x40000000, PAGE),
    r("application-compute-block0", POOL_BLOCKS + 0x40000000, 0x204000),
    r("application-compute-block1", POOL_BLOCKS + 0x40300000, 0x204000),
    r("application-compute-block2", POOL_BLOCKS + 0x40600000, 0x204000),
    r("application-compute-block3", POOL_BLOCKS + 0x40900000, 0x204000),
    r("application-compute-preempt", 0x70_5000_0000, PAGE),
    r("application-compute-preempt-driver", APPLICATION_COMPUTE_PREEMPT_DRIVER,
        APPLICATION_COMPUTE_PREEMPT_DRIVER_BYTES + PAGE),
    // Replacement views have independent sparse directories and block lists.
    // Each scratch aperture includes every supported 2-MiB block plus guards
    // at a 3-MiB stride; four-GiB bank spacing also covers the full 2-GiB plan.
    r("compute-replacement-a-directory", POOL_DIRECTORY + 0x2_4000_0000, 0x2000000),
    r("compute-replacement-a-list", POOL_BLOCK_LIST + 0x2_4000_0000, PAGE),
    r("compute-replacement-a-blocks", POOL_BLOCKS + 0x2_4000_0000, 0xc0000000),
    r("compute-replacement-b-directory", POOL_DIRECTORY + 0x3_4000_0000, 0x2000000),
    r("compute-replacement-b-list", POOL_BLOCK_LIST + 0x3_4000_0000, PAGE),
    r("compute-replacement-b-blocks", POOL_BLOCKS + 0x3_4000_0000, 0xc0000000),
    r("pool-block0-guard", POOL_BLOCKS, 0x104000),
    r("pool-block1-guard", POOL_BLOCKS + POOL_BLOCK_STRIDE, 0x104000),
    r("pool-block2-guard", POOL_BLOCKS + 2 * POOL_BLOCK_STRIDE, 0x104000),
    r("pool-block3-guard", POOL_BLOCKS + 3 * POOL_BLOCK_STRIDE, 0x104000),
    r("pool-block4-guard", POOL_BLOCKS + 4 * POOL_BLOCK_STRIDE, 0x104000),
    // Compute currently allocates five blocks. Reserve twelve for the pending
    // graphics pool so its additional backing cannot enter another arena.
    r("pool-block5-guard", POOL_BLOCKS + 5 * POOL_BLOCK_STRIDE, 0x104000),
    r("pool-block6-guard", POOL_BLOCKS + 6 * POOL_BLOCK_STRIDE, 0x104000),
    r("pool-block7-guard", POOL_BLOCKS + 7 * POOL_BLOCK_STRIDE, 0x104000),
    r("pool-block8-guard", POOL_BLOCKS + 8 * POOL_BLOCK_STRIDE, 0x104000),
    r("pool-block9-guard", POOL_BLOCKS + 9 * POOL_BLOCK_STRIDE, 0x104000),
    r("pool-block10-guard", POOL_BLOCKS + 10 * POOL_BLOCK_STRIDE, 0x104000),
    r("pool-block11-guard", POOL_BLOCKS + 11 * POOL_BLOCK_STRIDE, 0x104000),
    r("pool-block12-guard", POOL_BLOCKS + 12 * POOL_BLOCK_STRIDE, 0x104000),
    r("pool-block13-guard", POOL_BLOCKS + 13 * POOL_BLOCK_STRIDE, 0x104000),
    r("pool-block14-guard", POOL_BLOCKS + 14 * POOL_BLOCK_STRIDE, 0x104000),
    r("pool-block15-guard", POOL_BLOCKS + 15 * POOL_BLOCK_STRIDE, 0x104000),
    r("pool-block16-guard", POOL_BLOCKS + 16 * POOL_BLOCK_STRIDE, 0x104000),
    r("pool-block17-guard", POOL_BLOCKS + 17 * POOL_BLOCK_STRIDE, 0x104000),
    r("pool-block18-guard", POOL_BLOCKS + 18 * POOL_BLOCK_STRIDE, 0x104000),
    r("pool-block19-guard", POOL_BLOCKS + 19 * POOL_BLOCK_STRIDE, 0x104000),
    r("pool-block20-guard", POOL_BLOCKS + 20 * POOL_BLOCK_STRIDE, 0x104000),
    r("pool-block21-guard", POOL_BLOCKS + 21 * POOL_BLOCK_STRIDE, 0x104000),
    r("pool-block22-guard", POOL_BLOCKS + 22 * POOL_BLOCK_STRIDE, 0x104000),
    r("pool-block23-guard", POOL_BLOCKS + 23 * POOL_BLOCK_STRIDE, 0x104000),
    r("compute-low-arena", 0x70_0800_0000, 0x8000000),
    r("graphics-queues", GRAPHICS_QUEUE_HIGH, GRAPHICS_QUEUE_BYTES as u64),
    r("graphics-entries-high", GRAPHICS_ENTRY_HIGH, GRAPHICS_ENTRY_BYTES as u64),
    r("graphics-entries-low", GRAPHICS_ENTRY_LOW, GRAPHICS_ENTRY_BYTES as u64),
    r("graphics-job-high", GRAPHICS_JOB_HIGH, GRAPHICS_JOB_BYTES as u64),
    r("graphics-job-low", GRAPHICS_JOB_LOW, GRAPHICS_JOB_BYTES as u64),
    r("graphics-signals", GRAPHICS_SIGNALS, GRAPHICS_SIGNALS_BYTES as u64),
    r("graphics-scheduler", GRAPHICS_SCHEDULER_START,
        GRAPHICS_SCHEDULER_END - GRAPHICS_SCHEDULER_START),
    r("graphics-pb-high", GRAPHICS_PB_HIGH, GRAPHICS_PB_BYTES as u64),
    r("graphics-uma-high", GRAPHICS_UMA_HIGH, GRAPHICS_UMA_BYTES),
    r("graphics-ideal-blocks", GRAPHICS_IDEAL_BLOCKS, 40 * POOL_BLOCK_STRIDE),
    r("graphics-target", GRAPHICS_TARGET_OFFSET, GRAPHICS_TARGET_BYTES as u64),
    r("graphics-target-biased", COMPACT_BIAS + GRAPHICS_TARGET_OFFSET, GRAPHICS_TARGET_BYTES as u64),
    r("graphics-pb", GRAPHICS_PB_OFFSET, GRAPHICS_PB_COMPACT_BYTES as u64),
    r("graphics-pb-biased", COMPACT_BIAS + GRAPHICS_PB_OFFSET, GRAPHICS_PB_COMPACT_BYTES as u64),
];

pub(crate) const fn nonoverlapping(regions: &[Region]) -> bool {
    let mut i = 0;
    while i < regions.len() {
        let a = regions[i];
        if a.bytes == 0 || a.base & (PAGE - 1) != 0 || a.bytes & (PAGE - 1) != 0
            || a.base > u64::MAX - a.bytes { return false; }
        let mut j = 0;
        while j < i {
            let b = regions[j];
            if a.base < b.base + b.bytes && b.base < a.base + a.bytes { return false; }
            j += 1;
        }
        i += 1;
    }
    true
}
const _: () = assert!(nonoverlapping(REGIONS));

pub(crate) const GRAPHICS_UMA_REGIONS: &[Region] = &[
    r("uma-control", GRAPHICS_UMA_HIGH, 0x40000),
    r("uma-ideal-backing", GRAPHICS_UMA_BACKING, 40 * 0x100000),
    r("uma-ideal-growth", GRAPHICS_UMA_GROWTH, 12 * 21 * PAGE),
];
const _: () = assert!(nonoverlapping(GRAPHICS_UMA_REGIONS));
const _: () = assert!(GRAPHICS_UMA_GROWTH + 12 * 21 * PAGE <= GRAPHICS_UMA_HIGH + GRAPHICS_UMA_BYTES);

pub(crate) const PB_REGIONS: &[Region] = &[
    r("pb-state", GRAPHICS_PB_HIGH, PAGE), r("pb-graph", PB_GRAPH, 0x8000),
    r("pb-page-list-high", PB_PAGE_LIST_HIGH, PB_PAGE_LIST_BYTES as u64),
    r("pb-metrics-high", PB_METRICS_HIGH, PAGE),
    r("pb-page-list-low", PB_PAGE_LIST_LOW, PB_PAGE_LIST_BYTES as u64),
    r("pb-metrics-low", PB_METRICS_LOW, PAGE), r("pb-scene-scratch", PB_SCENE_SCRATCH, PAGE),
    r("pb-discard", PB_DISCARD, 0x8000),
    r("pb-blocks", PB_BLOCKS, crate::g18::uapi::render_policy::PB_MAX_BLOCKS as u64 * 0x80000),
];
const _: () = assert!(nonoverlapping(PB_REGIONS));

const fn pb_regions_contained() -> bool {
    let mut i = 0;
    while i < PB_REGIONS.len() {
        let child = PB_REGIONS[i];
        let high = child.base >= GRAPHICS_PB_HIGH
            && child.base + child.bytes <= GRAPHICS_PB_HIGH + GRAPHICS_PB_BYTES as u64;
        let low = child.base >= GRAPHICS_PB_OFFSET
            && child.base + child.bytes <= GRAPHICS_PB_OFFSET + GRAPHICS_PB_COMPACT_BYTES as u64;
        if !high && !low { return false; }
        i += 1;
    }
    true
}
const _: () = assert!(pb_regions_contained());
