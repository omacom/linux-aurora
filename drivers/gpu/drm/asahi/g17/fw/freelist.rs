// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! USC free-list page addresses and block run tables.

use super::initdata::{FreeListRunSlot, FREE_LIST_BLOCK_PAGES, FREE_LIST_BLOCK_SIZE};
use kernel::prelude::*;

pub(crate) const PAGE_LIST_SIZE: usize = 0x20_0000;
pub(crate) const RUN_LIST_SIZE: usize = 0x1_0000;
pub(crate) const TABLE_SIZE: usize = PAGE_LIST_SIZE + RUN_LIST_SIZE;
pub(crate) const CONTROL_SIZE: usize = 0x4000;
pub(crate) const RENDER_BLOCKS: usize = 44;
pub(crate) const COMPUTE_BLOCKS: usize = 21;

/// Render reserves eight run words per block; compute packs runs consecutively.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) enum RunLayout {
    Render,
    Compute,
}

impl RunLayout {
    const fn words(self) -> usize {
        match self {
            Self::Render => size_of::<FreeListRunSlot>() / 8,
            Self::Compute => 1,
        }
    }
}

#[repr(C)]
pub(crate) struct PoolTables {
    pages: [u64; PAGE_LIST_SIZE / 8],
    runs: [u64; RUN_LIST_SIZE / 8],
}

// SAFETY: The table consists entirely of integer arrays.
unsafe impl Zeroable for PoolTables {}
static_assert!(size_of::<PoolTables>() == TABLE_SIZE);
static_assert!(core::mem::offset_of!(PoolTables, runs) == PAGE_LIST_SIZE);

impl PoolTables {
    /// Enumerates only the initialized run words and page addresses, in their
    /// publication order. Reserved run words and unused table space stay zero.
    pub(crate) fn for_each_word(
        base: u64,
        blocks: usize,
        layout: RunLayout,
        mut put: impl FnMut(usize, u64),
    ) -> Result {
        if blocks > RUN_LIST_SIZE / (layout.words() * 8)
            || blocks > PAGE_LIST_SIZE / (FREE_LIST_BLOCK_PAGES as usize * 8)
        {
            return Err(EINVAL);
        }
        let buffers = base.checked_add(TABLE_SIZE as u64).ok_or(EOVERFLOW)?;
        buffers
            .checked_add((blocks * FREE_LIST_BLOCK_SIZE) as u64)
            .ok_or(EOVERFLOW)?;
        for block in 0..blocks {
            let address = buffers + (block * FREE_LIST_BLOCK_SIZE) as u64;
            put(
                PAGE_LIST_SIZE / 8 + block * layout.words(),
                FreeListRunSlot::new(address).run,
            );
            for page in 0..FREE_LIST_BLOCK_PAGES as usize {
                put(
                    block * FREE_LIST_BLOCK_PAGES as usize + page,
                    address
                        + (page * (FREE_LIST_BLOCK_SIZE / FREE_LIST_BLOCK_PAGES as usize)) as u64,
                );
            }
        }
        Ok(())
    }

    /// Initializes a zeroed table before its first publication.
    pub(crate) fn init(&mut self, base: u64, blocks: usize, layout: RunLayout) -> Result {
        Self::for_each_word(base, blocks, layout, |index, value| {
            if index < self.pages.len() {
                self.pages[index] = value;
            } else {
                self.runs[index - self.pages.len()] = value;
            }
        })
    }
}

/// Mutable page-pool state carried between the retained control record and
/// its global descriptor. Offsets name the source control-record words.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) struct PagePoolState {
    pub(crate) page_list_va: u64,
    pub(crate) unk_1c: u32,
    pub(crate) unk_20: u32,
    pub(crate) unk_24: u32,
    pub(crate) unk_28: u32,
    pub(crate) page_count: u32,
}

impl PagePoolState {
    /// The fourth descriptor word belongs to firmware and survives every load.
    pub(crate) const fn descriptor(self, unk_18: u64) -> super::initdata::PagePoolDescriptor {
        super::initdata::PagePoolDescriptor {
            page_list: ((self.unk_1c as u64) << 41) | (self.page_list_va >> 7),
            counters: ((self.unk_24 as u64) << 33)
                | ((self.unk_20 as u64) << 5)
                | ((self.unk_28 as u64) << 61),
            page_count: self.page_count as u64,
            unk_18,
        }
    }

    /// Address and allocation unit remain properties of the retained pool.
    pub(crate) const fn updated(self, descriptor: &super::initdata::PagePoolDescriptor) -> Self {
        Self {
            unk_20: ((descriptor.counters >> 5) & 0x0fff_ffff) as u32,
            unk_24: ((descriptor.counters >> 33) & 0x0fff_ffff) as u32,
            unk_28: (descriptor.counters >> 61) as u32,
            page_count: descriptor.page_count as u32,
            ..self
        }
    }
}
