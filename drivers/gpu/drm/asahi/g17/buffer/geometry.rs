// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Parameter-buffer backing geometry. Wire records carry each block's address,
//! so allocation stride and the reserved trailing guard are host policy.

use crate::hw::t8140::resources as cfg;
use kernel::prelude::*;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct BlockGeometry {
    pub(crate) base: u64,
    pub(crate) end: u64,
    pub(crate) block_bytes: usize,
    pub(crate) backing_stride: usize,
    pub(crate) stride: usize,
    pub(crate) capacity: usize,
}

impl BlockGeometry {
    pub(crate) const PAGE: usize = 0x8000;
    /// M5 parameter resources use 32-bit offsets from this compact base.
    pub(crate) const M5_COMPACT: core::ops::Range<u64> = 0x10_0000_0000..0x11_0000_0000;
    pub(crate) const NEO: Self = Self {
        base: cfg::USER_BASE,
        end: cfg::USER_END,
        block_bytes: cfg::TVB_BLOCK_SIZE,
        backing_stride: cfg::TVB_BLOCK_STRIDE as usize,
        stride: cfg::TVB_BLOCK_STRIDE as usize,
        capacity: cfg::TVB_BLOCK_SLOTS,
    };

    pub(crate) fn m5(memory_bytes: u64, initial_bytes: u64) -> Result<Self> {
        let capacity = crate::g17::fw::m5::pool::parameter_capacity(
            memory_bytes, initial_bytes,
        ).map_err(|_| EINVAL)?;
        // Retain one unallocated inventory slot, as in the shared ring manager.
        if capacity < 2 || capacity.checked_mul(16).is_none_or(|n| n > 0x3f_ffff) {
            return Err(EINVAL);
        }
        Ok(Self {
            base: Self::M5_COMPACT.start,
            end: Self::M5_COMPACT.end,
            block_bytes: 0x80000,
            backing_stride: 0x80000,
            stride: 0x88000,
            capacity,
        })
    }

    pub(crate) const fn pages(self) -> usize { self.block_bytes / Self::PAGE }
    pub(crate) const fn max_blocks(self) -> usize { self.capacity - 1 }

    pub(crate) fn validate(self) -> Result {
        if self.capacity < 2 || self.capacity > u32::MAX as usize
            || self.base >= self.end || self.base % Self::PAGE as u64 != 0
            || self.block_bytes == 0 || self.block_bytes % Self::PAGE != 0
            || self.backing_stride < self.block_bytes || self.backing_stride % Self::PAGE != 0
            || self.stride <= self.block_bytes || self.stride % Self::PAGE != 0 {
            return Err(EINVAL);
        }
        Ok(())
    }

    pub(crate) fn backing_bytes(self, blocks: usize) -> Result<usize> {
        self.validate()?;
        if blocks == 0 || blocks > self.max_blocks() { return Err(EINVAL); }
        (blocks - 1).checked_mul(self.backing_stride)
            .and_then(|n| n.checked_add(self.block_bytes)).ok_or(EOVERFLOW)
    }

    pub(crate) fn compact_page(self, address: u64) -> Result<u32> {
        let relative = address.checked_sub(self.base).ok_or(EINVAL)?;
        if relative % Self::PAGE as u64 != 0
            || address.checked_add(self.block_bytes as u64).ok_or(EOVERFLOW)? > self.end {
            return Err(EINVAL);
        }
        (relative / Self::PAGE as u64).try_into().map_err(|_| EOVERFLOW)
    }
}
