// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! M5 compute register programs. The operand and preemption layouts differ
//! from the other G17 profile; sharing a register count does not imply a shared
//! register program. M5 scratch has its own layout and two control writes.

use kernel::prelude::*;
pub(crate) const CHECKPOINTS: [u16; 4] = [37, 39, 39, 40];

/// The SCS scratch allocation. Pages are 4 KiB USC pool pages, independent
/// of the host's CPU page size. Layout sections and control registers are
/// specific to M5; the other G17 profile uses a different fixed region.
#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) struct ScratchRequest {
    layout: u64,
    driver_end: u32,
    pages: u16,
}

impl ScratchRequest {
    pub(crate) const NONE: Self = Self { layout: 0, driver_end: 0, pages: 0 };

    pub(crate) fn new(layout: u64) -> Result<Self> {
        let cliques = (layout >> 24) & 0xff;
        if cliques == 0 { return Ok(Self::NONE); }
        let depth = layout & 0xffff;
        let levels = (layout >> 16) & 0xff;
        let user = (layout >> 32) & 0xffff;
        let driver = layout >> 48;
        let threads = cliques * 32;
        // Four fixed sections followed by per-thread stacks, driver/user
        // payloads and per-level state. All inputs are bounded bitfields.
        let driver_end = 0x68000 + threads * (depth * 8 + driver);
        // The register naming the end of the driver section holds a 24-bit
        // byte offset. Reject truncation even if the total page count fits.
        if driver_end > 0xffffff { return Err(EINVAL); }
        let bytes = driver_end + threads * user
            + cliques * 256 * (levels + u64::from(levels > 1));
        let pages = (bytes + 0x1fff) >> 12;
        Ok(Self { layout, driver_end: driver_end as u32,
            pages: u16::try_from(pages).map_err(|_| EINVAL)? })
    }

    pub(crate) fn enabled(self) -> bool { self.pages != 0 }
    pub(crate) fn layout(self) -> u64 { self.layout }
    pub(crate) fn pages(self) -> u16 { self.pages }
    pub(crate) fn size_word(self) -> u64 {
        let pages = u64::from(self.pages);
        pages | (pages.div_ceil(256) << 16)
    }
    pub(crate) fn window_word(self) -> u64 {
        if self.enabled() { (1 << 24) | (((self.layout >> 24) & 0xff) << 16) }
        else { 0xff0000 }
    }
    pub(crate) fn stack_word(self) -> u64 {
        if self.enabled() { self.layout & 0xfff8 } else { 8 }
    }
    pub(crate) fn driver_word(self) -> u64 {
        if !self.enabled() { return 0; }
        let levels = (self.layout >> 16) & 0xff;
        u64::from(self.driver_end) | ((levels + u64::from(levels > 1)) << 24)
    }
    fn checkpoints(self) -> [u16; 4] {
        CHECKPOINTS.map(|count| count + if self.enabled() { 2 } else { 0 })
    }
}
/// ownership and readable ranges are checked by command validation.
#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) struct IntersectionTables {
    pub(crate) pool: u64,
    pub(crate) pool_bytes: u64,
    first: u32,
    count: u32,
    pub(crate) default_program: u64,
    default_control: u64,
}

impl IntersectionTables {
    pub(crate) const NONE: Self = Self {
        pool: 0, pool_bytes: 0, first: 0, count: 0,
        default_program: 0, default_control: 0,
    };

    pub(crate) fn new(pool: u64, pool_bytes: u64, first: u32, count: u32,
                      default_program: u64, default_control: u64) -> Result<Self> {
        if pool == 0 {
            if pool_bytes != 0 || first != 0 || count != 0
                || default_program != 0 || default_control != 0 {
                return Err(EINVAL);
            }
            return Ok(Self::NONE);
        }
        // Resource sizes are encoded in 16 KiB pages. Table lengths
        // have 18 block-count bits; truncating either field changes addressing.
        let end = pool.checked_add(pool_bytes).ok_or(EINVAL)?;
        let blocks_end = u64::from(first) + u64::from(count);
        let code = default_program & 0x0000_ffff_ffff_ffff;
        if pool & 0x3fff != 0 || pool_bytes == 0 || pool_bytes & 0x3fff != 0
            || end > (1u64 << 48) || (pool_bytes >> 14) >= (1u64 << 25)
            || count == 0 || count >= (1 << 18) || blocks_end * 128 > pool_bytes
            || code & 63 != 0
            || default_control & !0x0000_7fff_0000_3fff != 0
            || (code == 0 && (default_program != 0 || default_control != 0))
            || (code != 0 && default_control & (1u64 << 40) == 0) {
            return Err(EINVAL);
        }
        Ok(Self { pool, pool_bytes, first, count, default_program, default_control })
    }

    pub(crate) fn enabled(self) -> bool { self.pool != 0 }

    pub(crate) fn words(self) -> [u64; 6] {
        if !self.enabled() { return [0; 6]; }
        [self.pool + u64::from(self.first) * 128 | (1u64 << 51),
         0xc0000000 | (u64::from(self.count) << 7), self.pool,
         self.pool_bytes >> 14, self.default_program, self.default_control]
    }
}
