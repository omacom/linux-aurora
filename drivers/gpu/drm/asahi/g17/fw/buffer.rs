// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Parameter-buffer inventory, scene and control records. These objects are
//! initialized before publication; live access uses the host buffer manager's
//! individual aligned words and packed byte fields.

use super::{clear, offset_va};
use crate::hw::t8140::resources as cfg;
use kernel::prelude::*;

pub(crate) const SCENES: usize = 0x50;
pub(crate) const SCRATCH_SLOTS: usize = 0x24;
pub(crate) const SCRATCH_STRIDE: u64 = 0x20;
pub(crate) const FIRST_SCENE: usize = 1;
pub(crate) const PAGE_LIST: usize = 0x8000;
pub(crate) const PAGE_LIST_BYTES: usize = 0x10000;
pub(crate) const CONTROL: usize = 0x20000;
pub(crate) const COUNTER: usize = 0x28000;
pub(crate) const SCENE_TABLE: usize = 0x38000;
pub(crate) const BLOCK_TABLE: usize = 0x40000;
pub(crate) const SIZE: usize = 0x48000;
pub(crate) const METRICS_STRIDE: usize = SCENES * size_of::<u32>();
pub(crate) const METRICS_SLOTS: usize = cfg::METRICS_SIZE as usize / METRICS_STRIDE;
pub(crate) const NORMAL_MAX_BLOCKS: usize = 0x45e;
pub(crate) const GROW_BLOCKS: usize = 10;

/// Firmware's active parameter-buffer state. Pointers at +0x44, +0x4c,
/// +0x64 and +0x84 are intentionally unaligned.
#[repr(C, packed)]
pub(crate) struct State {
    pub(crate) unk_0: [u8; 0xc],
    pub(crate) buffer_id: u32,
    pub(crate) unk_10: [u8; 0x10],
    pub(crate) page_list_fw: u64,
    pub(crate) page_list_client: u64,
    pub(crate) page_list_size: u32,
    pub(crate) pages: u32,
    pub(crate) block_capacity: u32,
    pub(crate) producer: u32,
    pub(crate) consumer: u32,
    pub(crate) block_table: u64,
    pub(crate) control: u64,
    pub(crate) last_page: u32,
    pub(crate) block_size: u32,
    pub(crate) unk_5c: [u8; 8],
    pub(crate) counter: u64,
    pub(crate) unk_6c: [u8; 0x10],
    pub(crate) maximum_size: u32,
    pub(crate) normal_maximum_size: u32,
    pub(crate) discard: u64,
    pub(crate) unk_8c: [u8; 0x34],
}

/// One scene's accounting and scratch identity.
#[repr(C, packed)]
pub(crate) struct Scene {
    pub(crate) metric_client: u64,
    pub(crate) metric_fw: u64,
    pub(crate) unk_10: [u8; 0x18],
    pub(crate) scratch: u64,
    pub(crate) unk_30: [u8; 0x10],
    pub(crate) statistics: u64,
    pub(crate) used: u32,
    pub(crate) generation: u64,
    pub(crate) unk_54: [u8; 0x2c],
}

/// Circular inventory cursors followed by firmware statistics. Firmware owns
/// `read` and the statistics; the host requests reset by release-storing1.
#[repr(C)]
pub(crate) struct Control {
    pub(crate) total: u32,
    pub(crate) committed: u32,
    pub(crate) read: u32,
    pub(crate) unk_c: [u32; 13],
    pub(crate) max_pages: u32,
    pub(crate) unk_44: [u32; 7],
    pub(crate) reset: u32,
    pub(crate) unk_64: [u32; 7],
}

#[repr(C)]
pub(crate) struct Block {
    pub(crate) first_page: u32,
    pub(crate) unk_4: u32,
}

/// The complete parameter-buffer graph, including reserved spaces between
/// records. The page-list subrange also has a client mapping.
#[repr(C)]
pub(crate) struct Graph {
    pub(crate) state: State,
    pub(crate) pad_c0: [u8; PAGE_LIST - size_of::<State>()],
    pub(crate) pages: [u32; cfg::TVB_BLOCK_SLOTS * cfg::TVB_PAGES_PER_BLOCK],
    pub(crate) pad_151a0: [u8; CONTROL - PAGE_LIST - cfg::TVB_PAGE_LIST_SIZE],
    pub(crate) control: Control,
    pub(crate) pad_20080: [u8; COUNTER - CONTROL - size_of::<Control>()],
    pub(crate) counter: u32,
    pub(crate) pad_28004: [u8; SCENE_TABLE - COUNTER - 4],
    pub(crate) scenes: [Scene; SCENES],
    pub(crate) trailer: [u8; 0x40],
    pub(crate) pad_3a840: [u8; BLOCK_TABLE - SCENE_TABLE - SCENES * size_of::<Scene>() - 0x40],
    pub(crate) blocks: [Block; cfg::TVB_BLOCK_SLOTS],
    pub(crate) pad_468d0: [u8; SIZE - BLOCK_TABLE - cfg::TVB_BLOCK_SLOTS * size_of::<Block>()],
}

// SAFETY: Every field of the complete graph consists solely of integers.
unsafe impl Zeroable for Graph {}

static_assert!(size_of::<State>() == 0xc0);
static_assert!(size_of::<Scene>() == 0x80);
static_assert!(size_of::<Control>() == 0x80);
static_assert!(size_of::<Graph>() == SIZE);
static_assert!(core::mem::offset_of!(Graph, pages) == PAGE_LIST);
static_assert!(core::mem::offset_of!(Graph, control) == CONTROL);
static_assert!(core::mem::offset_of!(Graph, counter) == COUNTER);
static_assert!(core::mem::offset_of!(Graph, scenes) == SCENE_TABLE);
static_assert!(core::mem::offset_of!(Graph, blocks) == BLOCK_TABLE);
static_assert!(core::mem::offset_of!(State, block_table) == 0x44);
static_assert!(core::mem::offset_of!(State, discard) == 0x84);
static_assert!(core::mem::offset_of!(Scene, generation) == 0x4c);

/// The same metrics storage in the firmware and application address spaces.
#[derive(Clone, Copy, Default)]
pub(crate) struct MetricsAddresses {
    pub(crate) firmware: u64,
    pub(crate) client: u64,
}

pub(crate) struct GraphArgs<'a> {
    pub(crate) fw_va: u64,
    pub(crate) page_list_client: u64,
    pub(crate) metrics: MetricsAddresses,
    pub(crate) blocks: &'a [u64],
    pub(crate) scratch: u64,
    pub(crate) discard: u64,
    pub(crate) buffer_id: u32,
    pub(crate) generation: u64,
}

pub(crate) fn compact_page(address: u64) -> Result<u32> {
    let relative = address.checked_sub(cfg::USER_BASE).ok_or(EINVAL)?;
    if relative % cfg::TVB_PAGE != 0
        || address
            .checked_add(cfg::TVB_BLOCK_SIZE as u64)
            .ok_or(EOVERFLOW)?
            > cfg::USER_END
    {
        return Err(EINVAL);
    }
    Ok((relative / cfg::TVB_PAGE).try_into()?)
}

impl Graph {
    pub(crate) fn init(&mut self, args: &GraphArgs<'_>) -> Result {
        if args.generation == 0
            || args.blocks.len() < cfg::TVB_INITIAL_PAGES.len()
            || args.blocks.len() > cfg::TVB_MAX_BLOCKS
        {
            return Err(EINVAL);
        }
        let scratch = args.scratch.checked_sub(cfg::USER_BASE).ok_or(EINVAL)?;
        let discard = args.discard.checked_sub(cfg::USER_BASE).ok_or(EINVAL)?;
        if scratch > u64::from(u32::MAX) || discard > u64::from(u32::MAX) {
            return Err(EINVAL);
        }
        let stats = offset_va(args.fw_va, (CONTROL + 0x40) as u64)?;
        clear(self);
        for (index, &address) in args.blocks.iter().enumerate() {
            let first = compact_page(address)?;
            self.blocks[index].first_page = first;
            for page in 0..cfg::TVB_PAGES_PER_BLOCK {
                self.pages[index * cfg::TVB_PAGES_PER_BLOCK + page] = first + page as u32;
            }
        }
        for (index, scene) in self.scenes.iter_mut().enumerate() {
            scene.metric_client = offset_va(args.metrics.client, (index * 4) as u64)?;
            scene.metric_fw = offset_va(args.metrics.firmware, (index * 4) as u64)?;
            scene.scratch = scratch + (index % SCRATCH_SLOTS) as u64 * SCRATCH_STRIDE;
            scene.statistics = stats;
        }
        self.scenes[FIRST_SCENE].generation = args.generation;
        self.control.total = args.blocks.len() as u32;
        self.control.committed = self.control.total;
        self.control.reset = 1;
        let pages = (args.blocks.len() * cfg::TVB_PAGES_PER_BLOCK) as u32;
        self.state.buffer_id = args.buffer_id;
        self.state.page_list_fw = offset_va(args.fw_va, PAGE_LIST as u64)?;
        self.state.page_list_client = args.page_list_client;
        self.state.page_list_size = PAGE_LIST_BYTES as u32;
        self.state.pages = pages;
        self.state.block_capacity = cfg::TVB_BLOCK_SLOTS as u32;
        self.state.producer = args.blocks.len() as u32;
        self.state.block_table = offset_va(args.fw_va, BLOCK_TABLE as u64)?;
        self.state.control = offset_va(args.fw_va, CONTROL as u64)?;
        self.state.last_page = pages - 1;
        self.state.block_size = cfg::TVB_BLOCK_SIZE as u32;
        self.state.counter = offset_va(args.fw_va, COUNTER as u64)?;
        self.state.maximum_size = (cfg::TVB_BLOCK_SLOTS * 4) as u32;
        self.state.normal_maximum_size = (NORMAL_MAX_BLOCKS * 4) as u32;
        self.state.discard = discard;
        Ok(())
    }
}

/// Rebuilds an idle parameter-buffer descriptor, preserving firmware-owned
/// bits and resetting both ring counters and flags for its rebuilt page list.
pub(crate) fn reload_descriptor(previous: [u32; 4], address: u64, pages: u32) -> [u32; 4] {
    const COUNT: u32 = 0x003f_ffff;
    let bias = if address & (1u64 << 42) == 0 {
        0x70_0000_0000
    } else {
        0
    };
    let segment = (address.wrapping_add(bias) & 0x70_0000_0000) | ((address >> 3) & 0x80_0000_0000);
    [
        ((address >> 4) as u32 & !7) | (previous[0] & 6),
        ((segment >> 8) as u32 & 0xf000_0000) | (previous[1] & 0x0fc0_0000) | (pages & COUNT),
        previous[2] & !COUNT,
        previous[3] & !(COUNT | 0x8000_0000),
    ]
}

/// Compact scratch and tagged metrics addresses carried by scene registers.
pub(crate) fn scene_registers(index: usize, scratch: u64, metrics: u64) -> Result<(u64, u64)> {
    const SEGMENT_SELECT: u64 = 1 << 42;
    const SEGMENT_BIAS: u64 = 0x70_0000_0000;
    const ADDRESS_MASK: u64 = 0x7f_ffff_fffe;
    const HIGH_BIT: u64 = 0x80_0000_0000;
    if index >= SCENES || metrics & 3 != 0 {
        return Err(EINVAL);
    }
    let scratch = scratch
        .checked_sub(cfg::USER_BASE)
        .ok_or(EINVAL)?
        .checked_add((index % SCRATCH_SLOTS) as u64 * SCRATCH_STRIDE)
        .ok_or(EOVERFLOW)?;
    if scratch > u64::from(u32::MAX) || scratch & 0xf != 0 {
        return Err(EINVAL);
    }
    let metric = offset_va(metrics, (index * 4) as u64)?;
    let adjusted = offset_va(
        metric,
        if metric & SEGMENT_SELECT == 0 {
            SEGMENT_BIAS
        } else {
            0
        },
    )?;
    Ok((
        scratch,
        (adjusted & ADDRESS_MASK) | ((metric >> 3) & HIGH_BIT) | 1,
    ))
}
