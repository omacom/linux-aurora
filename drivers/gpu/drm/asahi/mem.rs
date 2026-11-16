// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! ARM64 low level memory operations.
//!
//! This GPU uses CPU-side `tlbi` outer-shareable instructions to manage its TLBs.
//! Yes, really. Even though the VA address spaces are unrelated.
//!
//! Right now we pick our own ASIDs and don't coordinate with the CPU. This might result
//! in needless TLB shootdowns on the CPU side... TODO: fix this.

use core::arch::asm;
use core::cmp::min;

use kernel::page::PAGE_SIZE;

use crate::debug::*;
use crate::mmu;
use core::sync::atomic::{AtomicBool, Ordering};

type Asid = u8;

/// Invalidate ranges page by page (`mmu::UatQuirks::page_tlbi`): never range operations.
///
/// The setting is global because the invalidation helpers take no UAT. It assumes one GPU per
/// system: `Uat::new` sets it for that GPU before the GPU's first invalidation, and a second
/// UAT with another policy would change it for both.
static PAGE_TLBI: AtomicBool = AtomicBool::new(false);

/// Ranges above this many pages are invalidated by ASID in page mode.
const PAGE_TLBI_MAX_PAGES: usize = 2048;

/// Selects page-by-page range invalidation for the system's GPU ([`PAGE_TLBI`]).
pub(crate) fn set_page_tlbi(enabled: bool) {
    PAGE_TLBI.store(enabled, Ordering::Relaxed);
}

/// Invalidates `len` bytes at `va` one 16 KiB page at a time, or the whole ASID above
/// [`PAGE_TLBI_MAX_PAGES`].
fn tlbi_pages(asid: Asid, va: usize, len: usize) {
    let start = va >> mmu::UAT_PGBIT;
    let end = (va + len + mmu::UAT_PGMSK) >> mmu::UAT_PGBIT;
    if end - start > PAGE_TLBI_MAX_PAGES {
        tlbi_asid(asid);
        return;
    }
    for page in start..end {
        tlbi_page(asid, page << mmu::UAT_PGBIT);
    }
}

/// Invalidate the entire GPU TLB.
#[inline(always)]
pub(crate) fn tlbi_all() {
    // SAFETY: tlbi is always safe by definition
    unsafe {
        asm!(".arch armv8.4-a", "tlbi vmalle1os",);
    }
}

/// Invalidate all TLB entries for a given ASID.
#[inline(always)]
pub(crate) fn tlbi_asid(asid: Asid) {
    if debug_enabled(DebugFlags::ConservativeTlbi) {
        tlbi_all();
        sync();
        return;
    }

    // SAFETY: tlbi is always safe by definition
    unsafe {
        asm!(
            ".arch armv8.4-a",
            "tlbi aside1os, {x}",
            x = in(reg) ((asid as u64) << 48)
        );
    }
}

/// Invalidate a single page for a given ASID.
#[inline(always)]
pub(crate) fn tlbi_page(asid: Asid, va: usize) {
    if debug_enabled(DebugFlags::ConservativeTlbi) {
        tlbi_all();
        sync();
        return;
    }

    let val: u64 = ((asid as u64) << 48) | ((va as u64 >> 12) & 0xffffffffffc);
    // SAFETY: tlbi is always safe by definition
    unsafe {
        asm!(
            ".arch armv8.4-a",
            "tlbi vae1os, {x}",
            x = in(reg) val
        );
    }
}

/// Invalidate a range of pages for a given ASID.
#[inline(always)]
pub(crate) fn tlbi_range(asid: Asid, va: usize, len: usize) {
    if debug_enabled(DebugFlags::ConservativeTlbi) {
        tlbi_all();
        sync();
        return;
    }

    if len == 0 {
        return;
    }
    if PAGE_TLBI.load(Ordering::Relaxed) {
        tlbi_pages(asid, va, len);
        return;
    }

    let start_pg = va >> mmu::UAT_PGBIT;
    let end_pg = (va + len + mmu::UAT_PGMSK) >> mmu::UAT_PGBIT;

    const BADDR_MASK: u64 = (1 << 37) - 1;
    let mut val: u64 = (u64::from(asid) << 48) | (2 << 46) | (start_pg as u64 & BADDR_MASK);
    let pages = end_pg - start_pg;

    // Guess? It's possible that the page count is in terms of 4K pages
    // when the CPU is in 4K mode...
    #[cfg(CONFIG_ARM64_4K_PAGES)]
    let pages = 4 * pages;

    if pages == 1 {
        tlbi_page(asid, va);
        return;
    }

    // Page count is always in units of 2
    let num = ((pages + 1) >> 1) as u64;
    // base: 5 bits
    // exp: 2 bits
    // pages = (base + 1) << (5 * exp + 1)
    // 0:00000 ->                     2 pages = 2 << 0
    // 0:11111 ->                32 * 2 pages = 2 << 5
    // 1:00000 ->            1 * 32 * 2 pages = 2 << 5
    // 1:11111 ->           32 * 32 * 2 pages = 2 << 10
    // 2:00000 ->       1 * 32 * 32 * 2 pages = 2 << 10
    // 2:11111 ->      32 * 32 * 32 * 2 pages = 2 << 15
    // 3:00000 ->  1 * 32 * 32 * 32 * 2 pages = 2 << 15
    // 3:11111 -> 32 * 32 * 32 * 32 * 2 pages = 2 << 20
    let exp = min(3, (64 - num.leading_zeros()) / 5);
    let bits = 5 * exp;
    let mut base = (num + (1 << bits) - 1) >> bits;

    val |= (exp as u64) << 44;

    while base > 32 {
        // SAFETY: tlbi is always safe by definition
        unsafe {
            asm!(
                ".arch armv8.4-a",
                "tlbi rvae1os, {x}",
                x = in(reg) val | (31 << 39)
            );
        }
        base -= 32;
        // Each full operation covers 32 units of 2 << bits GPU granules.
        // Advance only the base-address field, preserving the ASID and scale.
        let next = (val & BADDR_MASK) + (32u64 << (bits + 1));
        val = (val & !BADDR_MASK) | (next & BADDR_MASK);
    }

    // SAFETY: tlbi is always safe by definition
    unsafe {
        asm!(
            ".arch armv8.4-a",
            "tlbi rvae1os, {x}",
            x = in(reg) val | ((base - 1) << 39)
        );
    }
}

/// Range length from which [`tlbi_range_or_asid`] invalidates the whole ASID.
const TLBI_RANGE_MAX: usize = 0x200_0000;
/// Range length from which [`tlbi_range_or_asid`] counts in 1 MiB units (scale 1) instead of
/// 32 KiB units (scale 0).
const TLBI_RANGE_SCALE1_MIN: usize = 0x10_0000;
/// Translation granule field value for 16 KiB pages.
const TLBI_TG_16K: u64 = 2;
/// Base address field of a range TLBI operand, bits 36:0. Bits 38:37 hold the TTL hint.
const TLBI_RANGE_BADDR_MASK: u64 = (1 << 37) - 1;

/// Invalidate a range of pages for a given ASID with a single range operation.
///
/// Unlike [`tlbi_range`], which issues as many range operations as needed, this invalidates
/// ranges that do not fit one operation (and ranges starting at VA 0) by ASID. Single pages use a
/// page operation, at the CPU page size.
#[inline(always)]
pub(crate) fn tlbi_range_or_asid(asid: Asid, va: usize, len: usize) {
    if debug_enabled(DebugFlags::ConservativeTlbi) {
        tlbi_all();
        sync();
        return;
    }

    if len == 0 {
        return;
    }
    if PAGE_TLBI.load(Ordering::Relaxed) {
        tlbi_pages(asid, va, len);
        return;
    }

    if va == 0 || len >= TLBI_RANGE_MAX {
        tlbi_asid(asid);
        return;
    }
    if len <= PAGE_SIZE {
        tlbi_page(asid, va);
        return;
    }

    // The range covers (NUM + 1) << (5 * SCALE + 1) granules of 16 KiB.
    let (scale, unit_shift) = if len >= TLBI_RANGE_SCALE1_MIN {
        (1u64, 20)
    } else {
        (0u64, 15)
    };
    let num = ((len - 1) >> unit_shift) as u64;

    let val = (u64::from(asid) << 48)
        | (TLBI_TG_16K << 46)
        | (scale << 44)
        | (num << 39)
        | (((va >> mmu::UAT_PGBIT) as u64) & TLBI_RANGE_BADDR_MASK);

    // SAFETY: tlbi is always safe by definition
    unsafe {
        asm!(
            ".arch armv8.4-a",
            "tlbi rvae1os, {x}",
            x = in(reg) val
        );
    }
}

/// Issue a memory barrier (`dsb sy`).
#[inline(always)]
pub(crate) fn sync() {
    // SAFETY: Barriers are always safe
    unsafe {
        asm!("dsb sy");
    }
}
