// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Auxiliary render records and their initial images. Live event counters
//! are accessed individually by the queue owner after publication.

use super::super::{buffer, clear, context, initdata};
use kernel::prelude::*;

pub(crate) const SUPPORT_SIZE: usize = 0x40000;
pub(crate) const EVENT_SLOTS: usize = 0x4000;
pub(crate) const EVENT_COUNT: usize = 35;
// Host-private packing: consumers receive separate GPU pointers to these ranges.
// Keep every accessed range and all generation slots, without unused backing pages.
pub(crate) const STATE_SIZE: usize = 0x10000;
pub(crate) const DEFLAKE: usize = 0;
pub(crate) const TA_STATUS: usize = 0x4000;
pub(crate) const AUXILIARY: usize = 0x8000;
pub(crate) const AUXILIARY_SIZE: usize = 0x8000;
const _: () = {
    assert!(DEFLAKE + 0x4000 <= TA_STATUS);
    assert!(TA_STATUS + 0x4000 <= AUXILIARY);
    assert!(AUXILIARY + AUXILIARY_SIZE == STATE_SIZE);
};

#[repr(C)]
pub(crate) struct Event {
    pub(crate) node: context::WorkNode,
    unk_c8: [u8; 0x38],
}

#[repr(C)]
pub(crate) struct Support {
    secondary_index: [u64; 8],
    unk_40: [u8; 0x3fc0],
    pub(crate) event_slots: [u32; 0x1000],
    scene_slots: [u32; 0x1000],
    shared_slots: [u32; 0x1000],
    flag: u32,
    unk_10004: [u8; 0x3ffc],
    pub(crate) events: [Event; EVENT_COUNT],
    unk_16300: [u8; 0x1d00],
    scenes: [buffer::Scene; 79],
    unk_1a780: [u8; 0x1880],
    shared: buffer::State,
    unk_1c0c0: [u8; 0x7f40],
    control: initdata::FreeListControl,
    unk_24068: [u8; 0x3f98],
    submitted: u32,
    unk_28004: [u8; 0x3ffc],
    scheduler: context::Scheduler,
    unk_30000: [u8; 0x10000],
}
// SAFETY: The entire graph is integer storage; all nested types are Zeroable.
unsafe impl Zeroable for Support {}
impl Support {
    pub(crate) fn init(
        &mut self,
        address: u64,
        primary_index: u64,
        operand_table: u64,
        page_list: u64,
        word: u32,
    ) -> Result {
        clear(self);
        address.checked_add(SUPPORT_SIZE as u64).ok_or(EOVERFLOW)?;
        self.secondary_index = [0x11, 0x16, 0x1b, 0x20, 0x25, 0x2a, 0x3c, 0x41];
        self.event_slots[1] = 1;
        self.shared_slots[0] = 8;
        self.shared_slots[1] = 8;
        self.shared_slots[0x60 / 4] = 1;
        self.flag = 1;
        for (index, event) in self.events.iter_mut().enumerate() {
            event.node.submitted = address + EVENT_SLOTS as u64 + index as u64 * 4;
            if index == 1 {
                event.node.threshold = 0x50;
                event.node.word = word;
            }
        }
        for (index, scene) in self.scenes.iter_mut().enumerate() {
            scene.metric_client = 0x10_0008_0004 + index as u64 * 4;
            scene.metric_fw = address + 0x8004 + index as u64 * 4;
            let phase = index % buffer::SCRATCH_SLOTS;
            scene.scratch = if phase == buffer::SCRATCH_SLOTS - 1 {
                0x178000
            } else {
                0x178020 + phase as u64 * buffer::SCRATCH_STRIDE
            };
            scene.statistics = address + 0xc040;
            if index == 0 {
                scene.generation = 1;
            }
        }
        let shared = &mut self.shared;
        shared.page_list_fw = primary_index;
        shared.page_list_client = 0x10_0019_0000;
        shared.page_list_size = 0x10000;
        shared.pages = 0x20;
        shared.block_capacity = 0xc18;
        shared.producer = 8;
        shared.block_table = address;
        shared.control = address + 0xc000;
        shared.last_page = 0x1f;
        shared.block_size = 0x20000;
        shared.counter = address + 0x10000;
        shared.maximum_size = 0x3060;
        shared.normal_maximum_size = 0x1020;
        shared.discard = 0x180000;
        self.control = initdata::FreeListControl::new(&initdata::FreeListArgs {
            buffer_slot: 0,
            class: initdata::FreeListClass::Two,
            page_list_va: page_list,
            run_list_va: operand_table,
            blocks: 25,
            state_va: address + 0x28000,
        });
        self.control.run_cursor = 0xe0;
        self.submitted = 1;
        self.scheduler.init();
        Ok(())
    }
}

/// The viewport words consumed from the deflake page at +0x900.
#[repr(C)]
#[derive(Copy, Clone)]
pub(crate) struct Viewport {
    unk_0: u32,
    tiles_x: u32,
    tiles_y: u32,
    unk_c: u32,
    scale_x: u32,
    translate_x: u32,
    scale_y: u32,
    translate_y: u32,
    unk_20: u32,
    scale_z: u32,
}
impl Viewport {
    /// Refresh only defined words; firmware-owned gaps retain their contents.
    pub(crate) fn apply(&self, destination: &mut Self) {
        destination.unk_0 = self.unk_0;
        destination.tiles_x = self.tiles_x;
        destination.tiles_y = self.tiles_y;
        destination.scale_x = self.scale_x;
        destination.translate_x = self.translate_x;
        destination.scale_y = self.scale_y;
        destination.translate_y = self.translate_y;
        destination.scale_z = self.scale_z;
    }
    pub(crate) fn new(width: u32, height: u32) -> Result<Self> {
        fn half(value: u32) -> Result<u32> {
            if value == 0 {
                return Err(EINVAL);
            }
            let highest = 31 - value.leading_zeros();
            let shift = 23u32.checked_sub(highest).ok_or(EINVAL)?;
            Ok(((highest + 126) << 23) | ((value << shift) & 0x7f_ffff))
        }
        let x = half(width)?;
        let y = half(height)?;
        Ok(Self {
            unk_0: 0xc00,
            tiles_x: 0x8000_0000 | (width.checked_add(31).ok_or(EOVERFLOW)? / 32 - 1),
            tiles_y: height.checked_add(31).ok_or(EOVERFLOW)? / 32 - 1,
            unk_c: 0,
            scale_x: x,
            translate_x: x,
            scale_y: y,
            translate_y: y | 0x8000_0000,
            unk_20: 0,
            scale_z: 0x3f80_0000,
        })
    }
}

const _: () = {
    assert!(core::mem::size_of::<Event>() == 0x100);
    assert!(core::mem::size_of::<Support>() == SUPPORT_SIZE);
    assert!(core::mem::offset_of!(Support, events) == 0x14000);
    assert!(core::mem::offset_of!(Support, scenes) == 0x18000);
    assert!(core::mem::offset_of!(Support, shared) == 0x1c000);
    assert!(core::mem::offset_of!(Support, control) == 0x24000);
    assert!(core::mem::offset_of!(Support, scheduler) == 0x2c000);
    assert!(core::mem::size_of::<Viewport>() == 0x28);
};
