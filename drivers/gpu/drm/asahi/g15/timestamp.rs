// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! The firmware timestamp arena.
//!
//! GEM_BIND_OBJECT with USAGE_TIMESTAMPS maps the BO range into the 64 MiB arena at
//! [`super::hw::va::TIMESTAMPS`] in the host window with the firmware/GPU shared class
//! ([`super::hw::pte::SHARED`]); the firmware writes nanoseconds directly from the start/end
//! microsequence records (four distinct values for a render). The arena is a range of the
//! adopted UAT's `kernel_vm`, so the mapping is a shared-layer `mmu::KernelMapping`, placed in
//! the arena by the shared VA allocator (ENOSPC when full). Its leaves are cleaned to the point
//! of coherency by the shared page-table code; a page table it adds reaches the firmware's view
//! through the host-window sync ([`super::object::Objects::sync_window`]) before the VA is
//! returned.
//!
//! A job retains the mappings it names, and failed jobs are retained until
//! reboot, so an arena alias the firmware may still write is never unmapped. Unmapping clears
//! leaves only: the tables stay linked.

use core::ops::Range;
use core::sync::atomic::Ordering;
use kernel::prelude::*;

use super::{hw, Gpu};
use crate::{gem, mmu};

/// The arena in the host window.
const ARENA: Range<u64> = hw::va::TIMESTAMPS..hw::va::TIMESTAMPS + hw::va::TIMESTAMPS_SIZE;

/// Maps a timestamp buffer into the arena (`gpu::Gpu::map_timestamp_buffer`).
pub(crate) fn map(
    gpu: &Gpu,
    mut bo: gem::ObjectRef,
    range: Range<usize>,
) -> Result<mmu::KernelMapping> {
    if gpu.shared.crashed.load(Ordering::Acquire) {
        return Err(EIO);
    }
    let size = range.end.checked_sub(range.start).ok_or(EINVAL)?;
    if size == 0 || (range.start | size) & mmu::UAT_PGMSK != 0 || range.end > bo.size() {
        return Err(EINVAL);
    }
    let mapping = bo.map_range_into_range(
        gpu.uat.kernel_vm(),
        range,
        ARENA,
        mmu::UAT_PGSZ as u64,
        mmu::Prot::from_pte(hw::pte::SHARED),
        false,
    )?;
    // The firmware walks the host window through its own second-level table.
    let synced = gpu
        .shared
        .state
        .lock()
        .as_deref_mut()
        .ok_or(ENODEV)
        .and_then(|firmware| firmware.objects.sync_window(mapping.iova_range()));
    // On failure the mapping is dropped (unmapped) without the device lock.
    synced?;
    Ok(mapping)
}
