// SPDX-License-Identifier: GPL-2.0-only OR MIT
//! Construct a firmware-owned UMA allocator once from its graph's allocations.
//! GPU mappings and counters outlive this initialization and remain owned by
//! Render/Compute. Callers must never invoke this on a live or replayed pool.
use kernel::prelude::*;
use crate::{m3::m3_memory::Buffer, m3::m3_pool_layout as wire, m3::m3_queue_layout::FirmwareVa};
pub(crate) struct Owners {
    pub(crate) backing: core::ops::Range<usize>,
    pub(crate) pages: usize,
    pub(crate) table: usize,
    pub(crate) manager: usize,
    pub(crate) counter: usize,
    pub(crate) table_index: u8,
}
impl Owners {
    pub(crate) fn initialize(&self, objects: &mut [Buffer]) -> Result {
        let buffers = objects.get(self.backing.clone()).ok_or(EINVAL)?;
        let mut backings = KVec::with_capacity(buffers.len(), GFP_KERNEL)?;
        for buffer in buffers {
            backings.push(wire::GpuRegion::new(buffer.gpu_va()?, buffer.size()).map_err(|_| EINVAL)?, GFP_KERNEL)?;
        }
        let pages = objects.get(self.pages).ok_or(EINVAL)?;
        let pages = wire::GpuRegion::new(pages.gpu_va()?, pages.size()).map_err(|_| EINVAL)?;
        let table = objects.get(self.table).ok_or(EINVAL)?;
        let table = wire::GpuRegion::new(table.gpu_va()?, table.size()).map_err(|_| EINVAL)?;
        let counter = FirmwareVa::new(objects.get(self.counter).ok_or(EINVAL)?.va()).map_err(|_| EINVAL)?;
        if objects.get(self.manager).ok_or(EINVAL)?.size() < wire::MANAGER_SIZE { return Err(EINVAL); }
        let pool = wire::Pool::new(&backings, pages, table, counter, self.table_index).map_err(|_| EINVAL)?;
        let mut manager = [0; wire::MANAGER_SIZE];
        pool.encode(&mut manager).map_err(|_| EINVAL)?;
        // Unpopulated entries remain explicitly zero from the fresh DMA owner.
        for page in 0..wire::INITIAL_PAGES {
            objects[self.pages].u64(page * 8, pool.page(page).map_err(|_| EINVAL)?)?;
        }
        for entry in 0..backings.len() {
            objects[self.table].write(entry * wire::POOL_ENTRY_SIZE,
                &pool.table_entry(entry).map_err(|_| EINVAL)?)?;
        }
        objects[self.manager].write(0, &manager)
    }
}

impl Owners {
    pub(crate) fn initialize_extended(&self, objects: &mut [Buffer], slot: u8,
        compute_only: bool) -> Result {
        let mut backings = KVec::new();
        for index in self.backing.clone().take(8) {
            let buffer = objects.get(index).ok_or(EINVAL)?;
            backings.push(wire::GpuRegion::new(buffer.gpu_va()?, buffer.size()).map_err(|_| EINVAL)?, GFP_KERNEL)?;
        }
        let backing = wire::ModernBacking(&backings);
        let populated = backing.pages().map_err(|_| EINVAL)?;
        let pages = objects.get(self.pages).ok_or(EINVAL)?;
        let entries = wire::GpuRegion::new(pages.gpu_va()?, pages.size()).map_err(|_| EINVAL)?;
        let table = objects.get(self.table).ok_or(EINVAL)?;
        let groups = wire::GpuRegion::new(table.gpu_va()?, table.size()).map_err(|_| EINVAL)?;
        if groups.size() < populated / 512 * 64 { return Err(EINVAL); }
        let descriptor = wire::ModernPool { entries, groups,
            submitted: FirmwareVa::new(objects.get(self.counter).ok_or(EINVAL)?.va()).map_err(|_| EINVAL)?,
            identity: FirmwareVa::new(objects.get(self.manager).ok_or(EINVAL)?.va()).map_err(|_| EINVAL)?,
            slot, capacity: (entries.size() / 8).try_into()?, populated: populated.try_into()?, compute_only };
        let mut bytes = [0; wire::ModernPool::SIZE];
        descriptor.encode(&mut bytes).map_err(|_| EINVAL)?;
        for index in 0..populated {
            objects[self.pages].u64(index * 8, backing.page(index).map_err(|_| EINVAL)?)?;
        }
        for index in 0..populated / 512 {
            objects[self.table].write(index * 64, &backing.group(index).map_err(|_| EINVAL)?)?;
        }
        objects[self.manager].write(0, &bytes)
    }
}
