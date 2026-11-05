// SPDX-License-Identifier: GPL-2.0-only

//! Kernel-owned scratch for sparse fills (DRM_ASAHI_BIND_SPARSE_TILE).

use kernel::{prelude::*, sync::Arc};

/// A zeroed, physically contiguous kernel allocation. The page tables repeat
/// its first page across a filled range, as a SINGLE_PAGE bind repeats its
/// GEM's first page.
pub(super) struct SparseTile {
    page: *mut kernel::bindings::page,
    physical: u64,
}

// SAFETY: The pages are exclusively owned until Drop; only their physical
// address is shared, and it is immutable.
unsafe impl Send for SparseTile {}
// SAFETY: See Send. No interior mutability.
unsafe impl Sync for SparseTile {}

impl SparseTile {
    /// 64 pages. The page tables read only the first.
    const ORDER: u32 = 6;

    pub(super) fn new() -> Result<Arc<Self>> {
        // SAFETY: A plain zeroed higher-order page allocation.
        let page = unsafe { kernel::bindings::alloc_pages(
            kernel::bindings::GFP_KERNEL | kernel::bindings::__GFP_ZERO, Self::ORDER) };
        if page.is_null() { return Err(ENOMEM); }
        // SAFETY: page is a live allocation returned above.
        let physical = unsafe { kernel::bindings::page_to_phys(page) } as u64;
        let tile = Arc::new(Self { page, physical }, GFP_KERNEL);
        if tile.is_err() {
            // SAFETY: The allocation above was not published anywhere.
            unsafe { kernel::bindings::__free_pages(page, Self::ORDER) };
        }
        Ok(tile?)
    }

    pub(super) fn physical(&self) -> u64 { self.physical }
}

impl Drop for SparseTile {
    fn drop(&mut self) {
        // SAFETY: The VM that maps the tile owns it and drops it after its
        // page tables.
        unsafe { kernel::bindings::__free_pages(self.page, Self::ORDER) };
    }
}
