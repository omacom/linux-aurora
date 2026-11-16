// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Bounded read-only slot identity sampling while firmware is halted.

use crate::mmu;
use core::{
    ffi::c_void,
    ptr::NonNull,
    sync::atomic::{AtomicU64, Ordering},
};
use kernel::{bindings, prelude::*};

struct Mapping(NonNull<c_void>);

impl Drop for Mapping {
    fn drop(&mut self) {
        // SAFETY: This is the successful memremap result owned exclusively by this guard.
        unsafe { bindings::memunmap(self.0.as_ptr()) };
    }
}

pub(super) fn key(slot: usize, slot_keys: u64) -> Result<u64> {
    if slot >= super::attribution::SLOTS {
        return Err(EINVAL);
    }
    let physical = slot_keys + slot as u64 * size_of::<u64>() as u64;
    let offset = physical as usize & (mmu::UAT_PGSZ - 1);
    // SAFETY: The fixed recovery-key range is firmware RAM, page aligned here
    // for mapping. Only the selected aligned eight-byte word is read below.
    let raw = unsafe {
        bindings::memremap(
            physical - offset as u64,
            mmu::UAT_PGSZ,
            bindings::MEMREMAP_WB as usize,
        )
    };
    let mapping = Mapping(NonNull::new(raw).ok_or(ENOMEM)?);
    let ctr: u64;
    // SAFETY: CTR_EL0 is readable at EL1. DminLine specifies the cache maintenance granule.
    unsafe {
        core::arch::asm!("mrs {ctr}, ctr_el0", ctr = out(reg) ctr,
            options(nomem, nostack, preserves_flags));
    }
    let line = 4usize << ((ctr >> 16) & 0xf);
    if line > mmu::UAT_PGSZ {
        return Err(EINVAL);
    }
    // SAFETY: The range and offset are eight-byte aligned and lie within the mapped page.
    let pointer = unsafe {
        mapping
            .0
            .as_ptr()
            .cast::<u8>()
            .add(offset)
            .cast::<AtomicU64>()
    };
    let mut address = pointer as usize & !(line - 1);
    let end = (pointer as usize)
        .checked_add(size_of::<u64>())
        .ok_or(EOVERFLOW)?;
    // SAFETY: The mapping stays live and is never written by the host. The
    // invalidated cache line lies within its page; barriers complete the
    // invalidation before the aligned load of the selected firmware-owned word.
    unsafe {
        core::arch::asm!("dsb osh", options(nostack, preserves_flags));
        while address < end {
            core::arch::asm!("dc ivac, {address}", address = in(reg) address,
                options(nostack, preserves_flags));
            address = address.checked_add(line).ok_or(EOVERFLOW)?;
        }
        core::arch::asm!("dsb osh", options(nostack, preserves_flags));
        Ok((*pointer).load(Ordering::Acquire))
    }
}
