// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! UAT Page Table management
//!
//! AGX GPUs use an MMU called the UAT, which is largely compatible with the ARM64 page table
//! format. This module manages the actual page tables by allocating raw memory pages from
//! the kernel page allocator.

use core::fmt::Debug;
use core::mem::size_of;
use core::ops::Range;
use core::sync::atomic::{
    AtomicU64,
    Ordering, //
};

use kernel::{
    addr::PhysicalAddr,
    error::Result,
    page::Page,
    prelude::*, //
};
#[cfg(CONFIG_DEV_COREDUMP)]
use kernel::{
    types::Owned,
    uapi::{
        PF_R,
        PF_W,
        PF_X, //
    },
};

use crate::debug::*;
use crate::{uat::UatGeometry, util::align};
use crate::pgtable_memory::ReservedTables;

pub(crate) use crate::uat::{UAT_PGBIT, UAT_PGMSK, UAT_PGSZ};

const DEBUG_CLASS: DebugFlags = DebugFlags::PgTable;

type Pte = AtomicU64;

const PTE_BIT: usize = 3; // log2(sizeof(Pte))
const PTE_SIZE: usize = 1 << PTE_BIT;

/// Number of PTEs per page.
const UAT_NPTE: usize = UAT_PGSZ / size_of::<Pte>();

/// Number of address bits to address a level
const UAT_LVBIT: usize = crate::uat::UAT_LVBIT;
/// Number of entries per level
const UAT_LVSZ: usize = UAT_NPTE;
/// Mask of level bits
const UAT_LVMSK: u64 = (UAT_LVSZ - 1) as u64;

const UAT_LEVELS: usize = crate::uat::UAT_LEVELS;

/// Number of address bits below the index of a top-level page table entry.
pub(crate) const UAT_TOP_LEVEL_SHIFT: usize = UAT_PGBIT + (UAT_LEVELS - 1) * UAT_LVBIT;

/// Size of the smallest CPU data cache line (`CTR_EL0.DminLine`).
#[inline(always)]
fn dcache_line_size() -> usize {
    let ctr: u64;
    // SAFETY: CTR_EL0 is readable at EL1 and reading it has no side effects.
    unsafe {
        core::arch::asm!(
            "mrs {ctr}, ctr_el0",
            ctr = out(reg) ctr,
            options(nomem, nostack, preserves_flags)
        )
    };
    4 << ((ctr >> 16) & 0xf)
}

/// Cleans the CPU data cache lines covering `len` bytes at `ptr` to the point of coherency, for an
/// agent that does not snoop the CPU caches. [`poc_barrier`] completes the maintenance.
pub(crate) fn clean_poc(ptr: *const u8, len: usize) {
    if len == 0 {
        return;
    }
    let line = dcache_line_size();
    let end = ptr as usize + len;
    let mut addr = ptr as usize & !(line - 1);
    while addr < end {
        // SAFETY: Cleaning a cache line of a mapped kernel address writes dirty data back without
        // changing memory contents. The asm is a compiler barrier, so earlier stores are emitted
        // first, and the architecture orders them with the clean of the same line.
        unsafe {
            core::arch::asm!(
                "dc cvac, {addr}",
                addr = in(reg) addr,
                options(nostack, preserves_flags)
            )
        };
        addr += line;
    }
}

/// Cleans and invalidates the CPU data cache lines covering `len` bytes at `ptr` to the point of
/// coherency, so that later CPU reads observe what a non-snooping agent wrote. Lines the CPU wrote
/// are written back first. [`poc_barrier`] completes the maintenance.
pub(crate) fn clean_invalidate_poc(ptr: *const u8, len: usize) {
    if len == 0 {
        return;
    }
    let line = dcache_line_size();
    let end = ptr as usize + len;
    let mut addr = ptr as usize & !(line - 1);
    while addr < end {
        // SAFETY: Clean+invalidate of a mapped kernel address never discards dirty data.
        unsafe {
            core::arch::asm!(
                "dc civac, {addr}",
                addr = in(reg) addr,
                options(nostack, preserves_flags)
            )
        };
        addr += line;
    }
}

/// Completes earlier point-of-coherency maintenance for the outer shareable domain (`dsb osh`).
pub(crate) fn poc_barrier() {
    // SAFETY: Barriers are always safe.
    unsafe { core::arch::asm!("dsb osh", options(nostack, preserves_flags)) };
}

/// Runs `f` on the first `count` top-level entries of the root page table at `ttb`.
///
/// # Safety
///
/// `ttb` must be the physical address of a page table root in RAM that stays allocated while `f`
/// runs. Other agents (the GPU firmware) may access the entries concurrently, which is why they
/// are only exposed as atomics.
pub(crate) unsafe fn with_root_entries<R>(
    ttb: PhysicalAddr,
    count: usize,
    f: impl FnOnce(&[AtomicU64]) -> R,
) -> Result<R> {
    if count > UAT_NPTE {
        return Err(EINVAL);
    }
    // SAFETY: The caller guarantees that `ttb` is an allocated page in RAM.
    let page = unsafe { Page::borrow_phys(&ttb) }.ok_or(EIO)?;
    page.with_pointer_into_page(0, count * PTE_SIZE, |p| {
        // SAFETY: with_pointer_into_page() ensures the pointer is valid for `count` PTEs, and page
        // table entries are naturally aligned.
        Ok(f(unsafe {
            core::slice::from_raw_parts(p as *const AtomicU64, count)
        }))
    })
}

const PTE_TYPE_BITS: u64 = 3;
const PTE_TYPE_LEAF_TABLE: u64 = 3;

const UAT_NON_GLOBAL: u64 = 1 << 11;
const UAT_AP_SHIFT: u32 = 6;
const UAT_AP_BITS: u64 = 3 << UAT_AP_SHIFT;
const UAT_HIGH_BITS_SHIFT: u32 = 52;
const UAT_HIGH_BITS: u64 = 0xfff << UAT_HIGH_BITS_SHIFT;
const UAT_MEMATTR_SHIFT: u32 = 2;
const UAT_MEMATTR_BITS: u64 = 7 << UAT_MEMATTR_SHIFT;

const UAT_PROT_BITS: u64 = UAT_AP_BITS | UAT_MEMATTR_BITS | UAT_HIGH_BITS;

const UAT_AF: u64 = 1 << 10;

const MEMATTR_CACHED: u8 = 0;
const MEMATTR_DEV: u8 = 1;
const MEMATTR_UNCACHED: u8 = 2;

const AP_FW_GPU: u8 = 0;
const AP_FW: u8 = 1;
const AP_GPU: u8 = 2;

const HIGH_BITS_PXN: u16 = 1 << 1;
const HIGH_BITS_UXN: u16 = 1 << 2;
const HIGH_BITS_GPU_ACCESS: u16 = 1 << 3;

const fn complete_coverage(expected: u64, visited: u64, permitted: bool) -> bool {
    permitted && visited == expected
}

#[cfg(CONFIG_DEV_COREDUMP)]
pub(crate) const PTE_ADDR_BITS: u64 = (!UAT_PGMSK as u64) & (!UAT_HIGH_BITS);

#[derive(Debug, Copy, Clone)]
pub(crate) struct Prot {
    memattr: u8,
    ap: u8,
    high_bits: u16,
}

// Firmware + GPU access
const PROT_FW_GPU_NA: Prot = Prot::from_bits(AP_FW_GPU, 0, 0);
const _PROT_FW_GPU_RO: Prot = Prot::from_bits(AP_FW_GPU, 0, 1);
const _PROT_FW_GPU_WO: Prot = Prot::from_bits(AP_FW_GPU, 1, 0);
const PROT_FW_GPU_RW: Prot = Prot::from_bits(AP_FW_GPU, 1, 1);

// Firmware only access
const PROT_FW_RO: Prot = Prot::from_bits(AP_FW, 0, 0);
const _PROT_FW_NA: Prot = Prot::from_bits(AP_FW, 0, 1);
const PROT_FW_RW: Prot = Prot::from_bits(AP_FW, 1, 0);
const PROT_FW_RW_GPU_RO: Prot = Prot::from_bits(AP_FW, 1, 1);

// GPU only access
const PROT_GPU_RO: Prot = Prot::from_bits(AP_GPU, 0, 0);
const PROT_GPU_WO: Prot = Prot::from_bits(AP_GPU, 0, 1);
const PROT_GPU_RW: Prot = Prot::from_bits(AP_GPU, 1, 0);
const _PROT_GPU_NA: Prot = Prot::from_bits(AP_GPU, 1, 1);

#[cfg(CONFIG_DEV_COREDUMP)]
const PF_RW: u32 = PF_R | PF_W;
#[cfg(CONFIG_DEV_COREDUMP)]
const PF_RX: u32 = PF_R | PF_X;

// For crash dumps
#[cfg(CONFIG_DEV_COREDUMP)]
const PROT_TO_PERMS_FW: [[u32; 4]; 4] = [
    [0, 0, 0, PF_RW],
    [0, PF_RW, 0, PF_RW],
    [PF_RX, PF_RX, 0, PF_R],
    [PF_RX, PF_RW, 0, PF_R],
];
#[cfg(CONFIG_DEV_COREDUMP)]
const PROT_TO_PERMS_OS: [[u32; 4]; 4] = [
    [0, PF_R, PF_W, PF_RW],
    [PF_R, 0, PF_RW, PF_RW],
    [0, 0, 0, 0],
    [0, 0, 0, 0],
];

pub(crate) mod prot {
    pub(crate) use super::Prot;
    use super::*;

    /// Firmware MMIO R/W
    pub(crate) const PROT_FW_MMIO_RW: Prot = PROT_FW_RW.memattr(MEMATTR_DEV);
    /// Protected firmware register aperture: AP=3, PXN, no GPU access.
    pub(crate) const PROT_FW_PROTECTED_MMIO: Prot = Prot {
        memattr: MEMATTR_DEV, ap: 3, high_bits: HIGH_BITS_PXN,
    };
    /// Firmware MMIO R/O
    pub(crate) const PROT_FW_MMIO_RO: Prot = PROT_FW_RO.memattr(MEMATTR_DEV);
    /// Firmware shared (uncached) RW
    pub(crate) const PROT_FW_SHARED_RW: Prot = PROT_FW_RW.memattr(MEMATTR_UNCACHED);
    /// Firmware shared (uncached) RO
    pub(crate) const PROT_FW_SHARED_RO: Prot = PROT_FW_RO.memattr(MEMATTR_UNCACHED);
    /// Firmware private (cached) RW
    pub(crate) const PROT_FW_PRIV_RW: Prot = PROT_FW_RW.memattr(MEMATTR_CACHED);
    /// Firmware/GPU shared (uncached) RW
    pub(crate) const PROT_GPU_FW_SHARED_RW: Prot = PROT_FW_GPU_RW.memattr(MEMATTR_UNCACHED);
    /// Firmware/GPU shared (private) RW
    pub(crate) const PROT_GPU_FW_PRIV_RW: Prot = PROT_FW_GPU_RW.memattr(MEMATTR_CACHED);
    /// Firmware-RW/GPU-RO shared (private) RW
    pub(crate) const PROT_GPU_RO_FW_PRIV_RW: Prot = PROT_FW_RW_GPU_RO.memattr(MEMATTR_CACHED);
    /// GPU shared/coherent RW
    pub(crate) const PROT_GPU_SHARED_RW: Prot = PROT_GPU_RW.memattr(MEMATTR_UNCACHED);
    /// GPU shared/coherent RO
    pub(crate) const PROT_GPU_SHARED_RO: Prot = PROT_GPU_RO.memattr(MEMATTR_UNCACHED);
    /// GPU shared/coherent WO
    pub(crate) const PROT_GPU_SHARED_WO: Prot = PROT_GPU_WO.memattr(MEMATTR_UNCACHED);
}

impl Prot {
    const fn from_bits(ap: u8, uxn: u16, pxn: u16) -> Self {
        assert!(uxn <= 1);
        assert!(pxn <= 1);
        assert!(ap <= 3);

        Prot {
            high_bits: HIGH_BITS_GPU_ACCESS | (pxn * HIGH_BITS_PXN) | (uxn * HIGH_BITS_UXN),
            memattr: 0,
            ap,
        }
    }

    #[inline]
    pub(crate) const fn from_pte(pte: u64) -> Self {
        Prot {
            high_bits: (pte >> UAT_HIGH_BITS_SHIFT) as u16,
            ap: ((pte & UAT_AP_BITS) >> UAT_AP_SHIFT) as u8,
            memattr: ((pte & UAT_MEMATTR_BITS) >> UAT_MEMATTR_SHIFT) as u8,
        }
    }

    #[cfg(CONFIG_DEV_COREDUMP)]
    pub(crate) const fn elf_flags(&self) -> u32 {
        let ap = (self.ap & 3) as usize;
        let uxn = if self.high_bits & HIGH_BITS_UXN != 0 {
            1
        } else {
            0
        };
        let pxn = if self.high_bits & HIGH_BITS_PXN != 0 {
            1
        } else {
            0
        };
        let gpu = self.high_bits & HIGH_BITS_GPU_ACCESS != 0;

        // Format:
        // [12 top bits of PTE] [12 bottom bits of PTE] [5 bits pad] [ELF RWX]
        let mut perms = if gpu {
            PROT_TO_PERMS_OS[ap][(uxn << 1) | pxn]
        } else {
            PROT_TO_PERMS_FW[ap][(uxn << 1) | pxn]
        };

        perms |= ((self.as_pte() >> 52) << 20) as u32;
        perms |= ((self.as_pte() & 0xfff) << 8) as u32;

        perms
    }

    const fn memattr(&self, memattr: u8) -> Self {
        Self { memattr, ..*self }
    }

    /// The protection bits of a leaf with this protection (no address, table type or non-global
    /// bits), for checking that a firmware leaf word round-trips through [`Self::from_pte`].
    pub(crate) const fn leaf_bits(&self) -> u64 {
        self.as_pte()
    }

    const fn as_pte(&self) -> u64 {
        (self.ap as u64) << UAT_AP_SHIFT
            | (self.high_bits as u64) << UAT_HIGH_BITS_SHIFT
            | (self.memattr as u64) << UAT_MEMATTR_SHIFT
            | UAT_AF
    }

    #[inline]
    pub(crate) const fn allows_gpu(&self, need_read: bool, need_write: bool) -> bool {
        let uxn = self.high_bits & HIGH_BITS_UXN != 0;
        let pxn = self.high_bits & HIGH_BITS_PXN != 0;
        let (readable, writable) = match self.ap {
            AP_FW_GPU => (pxn, uxn),
            AP_FW => (uxn && pxn, false),
            AP_GPU => (!pxn, uxn != pxn),
            _ => (false, false),
        };
        (!need_read || readable) && (!need_write || writable)
    }

    pub(crate) const fn is_cached_noncoherent(&self) -> bool {
        self.ap != AP_GPU && self.memattr == MEMATTR_CACHED
    }

    pub(crate) const fn as_uncached(&self) -> Self {
        self.memattr(MEMATTR_UNCACHED)
    }
}

/// Returns whether a leaf/block descriptor maps normal memory (cached or uncached), as opposed
/// to device memory (MMIO) or an unknown memory attribute index.
pub(crate) const fn pte_is_normal_memory(pte: u64) -> bool {
    let memattr = ((pte & UAT_MEMATTR_BITS) >> UAT_MEMATTR_SHIFT) as u8;
    memattr == MEMATTR_CACHED || memattr == MEMATTR_UNCACHED
}

/// Returns whether a normal-memory leaf/block descriptor uses the "uncached" (shared) memory
/// attribute index (firmware writes bypass the inner caches).
pub(crate) const fn pte_is_uncached(pte: u64) -> bool {
    ((pte & UAT_MEMATTR_BITS) >> UAT_MEMATTR_SHIFT) as u8 == MEMATTR_UNCACHED
}

/// Decodes the memory attribute and access permission fields of a leaf/block descriptor for
/// diagnostics: (memattr name, AP name, AP index, high bits).
pub(crate) const fn pte_describe(pte: u64) -> (&'static str, &'static str, u8, u16) {
    let memattr = ((pte & UAT_MEMATTR_BITS) >> UAT_MEMATTR_SHIFT) as u8;
    let ap = ((pte & UAT_AP_BITS) >> UAT_AP_SHIFT) as u8;
    let high = ((pte & UAT_HIGH_BITS) >> UAT_HIGH_BITS_SHIFT) as u16;
    let m = match memattr {
        MEMATTR_CACHED => "cached",
        MEMATTR_DEV => "device",
        MEMATTR_UNCACHED => "uncached",
        _ => "unknown",
    };
    let a = match ap {
        AP_FW_GPU => "fw+gpu",
        AP_FW => "fw",
        AP_GPU => "gpu",
        _ => "ap3",
    };
    (m, a, ap, high)
}

impl Default for Prot {
    fn default() -> Self {
        PROT_FW_GPU_NA
    }
}

const _: () = {
    assert!(PROT_GPU_RO.allows_gpu(true, false));
    assert!(!PROT_GPU_RO.allows_gpu(false, true));
    assert!(!PROT_GPU_WO.allows_gpu(true, false));
    assert!(PROT_GPU_WO.allows_gpu(false, true));
    assert!(PROT_GPU_RW.allows_gpu(true, false));
    assert!(PROT_GPU_RW.allows_gpu(false, true));
    assert!(PROT_GPU_RW.allows_gpu(true, true));
    assert!(!PROT_FW_RW.allows_gpu(true, false));
    assert!(!PROT_FW_RW.allows_gpu(false, true));
    assert!(!Prot::from_pte(0).allows_gpu(true, false));
    assert!(complete_coverage(3, 3, true));
    assert!(!complete_coverage(3, 2, true));
    assert!(!complete_coverage(3, 3, false));
};

#[cfg(CONFIG_DEV_COREDUMP)]
pub(crate) struct DumpedPage {
    pub(crate) iova: u64,
    pub(crate) pte: u64,
    pub(crate) data: Option<Owned<Page>>,
}

// DMA-backed M3 table owners, sorted by unique DMA address. VM serialization
// protects all accesses. Insertion/removal move descriptors only: table backing
// and returned CPU pointers remain stable as the owning vector changes.
struct DmaTables {
    dev: kernel::sync::aref::ARef<kernel::device::Device>,
    pages: core::cell::RefCell<KVec<kernel::dma::Coherent<[u64]>>>,
}
impl DmaTables {
    fn new(dev:&crate::driver::AsahiDevice)->Self {
        Self {dev:dev.as_ref().into(),pages:core::cell::RefCell::new(KVec::new())}
    }
    fn alloc(&self)->Result<u64> {
        // SAFETY: admitted M3 runtime retains the bound device and all tables
        // until ASC and GPU consumers have been quiesced.
        let page=kernel::dma::Coherent::zeroed_slice(unsafe {self.dev.as_bound()},UAT_NPTE,GFP_KERNEL)?;
        let pa = page.dma_handle();
        let mut pages = self.pages.borrow_mut();
        let index = match pages.binary_search_by_key(&pa, |p| p.dma_handle()) {
            Ok(_) => return Err(EEXIST),
            Err(index) => index,
        };
        pages.reserve(1, GFP_KERNEL)?;
        pages.insert_within_capacity(index, page).map_err(|_| ENOMEM)?;
        Ok(pa)
    }
    fn pointer(&self, pa: u64) -> Option<*mut Pte> {
        let pages = self.pages.borrow();
        let index = pages.binary_search_by_key(&pa, |p| p.dma_handle()).ok()?;
        Some(pages[index].as_mut_ptr().cast::<Pte>())
    }
    fn free(&self,pa:u64) {
        let mut pages = self.pages.borrow_mut();
        if let Ok(index) = pages.binary_search_by_key(&pa, |p| p.dma_handle()) {
            // Removal preserves the physical-address order. Moving a DMA
            // owner moves its descriptor, never the retained table backing.
            drop(pages.remove(index));
        }
    }
}

pub(crate) struct UatPageTable {
    ttb: PhysicalAddr,
    ttb_owned: bool,
    quarantined: bool,
    noncoherent: bool,
    dma_tables: Option<DmaTables>,
    reserved_tables: Option<ReservedTables>,
    va_range: Range<u64>,
    geometry: UatGeometry,
    oas_mask: u64,
    coverage: Option<KBox<crate::m3_coverage::Cache>>,
    /// G17 tables (see [`Self::new`]): every walk, including those of 42-bit roots, uses the
    /// ordered walk with the leaf-level semantics of the 39-bit walk.
    ordered_walk: bool,
    /// Tables below a root the driver did not allocate may belong to the bootloader or the
    /// firmware, so only the tables recorded here are ever freed. `None` if the driver owns the
    /// tables below the root. Only used by the ordered walk.
    driver_tables: Option<KVec<PhysicalAddr>>,
}

impl UatPageTable {
    /// Allocate a root for a G17 user VM. Its tables use the ordered walk.
    pub(crate) fn new(ias: u32, oas: u32) -> Result<Self> {
        mod_pr_debug!("UATPageTable::new: ias={} oas={}\n", ias, oas);
        let geometry = UatGeometry::new(u8::try_from(ias)?).ok_or(EINVAL)?;
        let ttb_page = Page::alloc_page(GFP_KERNEL | __GFP_ZERO)?;
        let ttb = Page::into_phys(ttb_page);
        Ok(UatPageTable {
            ttb,
            ttb_owned: true,
            quarantined: false,
            noncoherent: false,
            dma_tables: None,
            reserved_tables: None,
            va_range: 0..(1u64 << ias),
            geometry,
            oas_mask: (1u64 << oas) - 1,
            coverage: None,
            ordered_walk: true,
            driver_tables: None,
        })
    }

    pub(crate) fn new_by_geometry(ias: u8, oas: u32) -> Result<Self> {
        mod_pr_debug!("UATPageTable::new: ias={} oas={}\n", ias, oas);
        let geometry = UatGeometry::new(ias).ok_or(EINVAL)?;
        let ttb_page = Page::alloc_page(GFP_KERNEL | __GFP_ZERO)?;
        let ttb = Page::into_phys(ttb_page);
        Ok(UatPageTable {
            ttb,
            ttb_owned: true,
            quarantined: false,
            noncoherent: false,
            dma_tables: None,
            reserved_tables: None,
            va_range: 0..geometry.root_size(),
            geometry,
            oas_mask: (1u64 << oas) - 1,
            coverage: None,
            ordered_walk: false,
            driver_tables: None,
        })
    }

    pub(crate) fn new_m3_coherent(ias:u8,oas:u32,dev:&crate::driver::AsahiDevice)->Result<Self> {
        let mut table=Self::new_by_geometry(ias,oas)?;
        let dma=DmaTables::new(dev);let root=dma.alloc()?;
        // SAFETY: this newly allocated, unpublished root is still empty.
        unsafe {Page::from_phys(table.ttb)};
        table.ttb=root;table.dma_tables=Some(dma);
        table.coverage=Some(KBox::new(crate::m3_coverage::Cache::new(), GFP_KERNEL)?);Ok(table)
    }

    /// Resolve one mapped IOVA through this page table without touching the
    /// mapped memory. This is used to prove that separately named aliases
    /// refer to the same backing page before firmware can consume them.
    pub(crate) fn translate_iova(&mut self, iova: u64) -> Result<PhysicalAddr> {
        if self.ordered_walk {
            let entry = self.leaf_entry(iova & !(UAT_PGMSK as u64))?;
            return Ok((entry & self.oas_mask & !(UAT_PGMSK as u64)) | (iova & UAT_PGMSK as u64));
        }
        let page_mask = UAT_PGMSK as u64;
        let page = iova & !page_mask;
        let end = page.checked_add(UAT_PGSZ as u64).ok_or(EOVERFLOW)?;
        let offset = iova & page_mask;
        let oas_mask = self.oas_mask;
        let mut translated = None;
        self.with_pages(page..end, false, false, false, |_, ptes| {
            let pte = ptes.first().ok_or(EFAULT)?.load(Ordering::Acquire);
            if pte & PTE_TYPE_BITS != PTE_TYPE_LEAF_TABLE {
                return Err(EFAULT);
            }
            translated = Some((pte & oas_mask & !page_mask).checked_add(offset).ok_or(EOVERFLOW)?);
            Ok(())
        })?;
        translated.ok_or(EFAULT)
    }

    /// Borrow a root; track individual child ownership only when other agents
    /// can install tables below it. Used for G17 kernel roots (ordered walk).
    pub(crate) fn new_with_ttb(
        ttb: PhysicalAddr,
        va_range: Range<u64>,
        ias: u32,
        oas: u32,
        shared_tables: bool,
    ) -> Result<Self> {
        mod_pr_debug!(
            "UATPageTable::new_with_ttb: ttb={:#x} range={:#x?} ias={} oas={}\n",
            ttb,
            va_range,
            ias,
            oas
        );
        let geometry = UatGeometry::new(u8::try_from(ias)?).ok_or(EINVAL)?;
        if ttb & (UAT_PGMSK as PhysicalAddr) != 0 {
            return Err(EINVAL);
        }
        if (va_range.start | va_range.end) & (UAT_PGMSK as u64) != 0 {
            return Err(EINVAL);
        }
        // SAFETY: The bootloader-managed root stays valid while this page table exists.
        if unsafe { Page::borrow_phys(&ttb) }.is_none() {
            pr_err!(
                "UATPageTable::new_with_ttb: ttb at {:#x} is not mapped (DT using no-map?)\n",
                ttb
            );
            return Err(EIO);
        }

        Ok(UatPageTable {
            ttb,
            ttb_owned: false,
            quarantined: false,
            noncoherent: false,
            dma_tables: None,
            reserved_tables: None,
            va_range,
            geometry,
            oas_mask: (1u64 << oas) - 1,
            coverage: None,
            ordered_walk: true,
            driver_tables: shared_tables.then(KVec::new),
        })
    }

    pub(crate) fn new_with_ttb_by_geometry(
        ttb: PhysicalAddr,
        va_range: Range<u64>,
        ias: u8,
        oas: u32,
    ) -> Result<Self> {
        mod_pr_debug!(
            "UATPageTable::new_with_ttb: ttb={:#x} range={:#x?} ias={} oas={}\n",
            ttb,
            va_range,
            ias,
            oas
        );
        let geometry = UatGeometry::new(ias).ok_or(EINVAL)?;
        if ttb & (UAT_PGMSK as PhysicalAddr) != 0 {
            return Err(EINVAL);
        }
        if (va_range.start | va_range.end) & (UAT_PGMSK as u64) != 0 {
            return Err(EINVAL);
        }
        // SAFETY: The TTB is should remain valid (if properly mapped), as it is bootloader-managed.
        if unsafe { Page::borrow_phys(&ttb) }.is_none() {
            pr_err!(
                "UATPageTable::new_with_ttb: ttb at {:#x} is not mapped (DT using no-map?)\n",
                ttb
            );
            return Err(EIO);
        }

        Ok(UatPageTable {
            ttb,
            ttb_owned: false,
            quarantined: false,
            noncoherent: false,
            dma_tables: None,
            reserved_tables: None,
            va_range,
            geometry,
            oas_mask: (1u64 << oas) - 1,
            coverage: None,
            ordered_walk: false,
            driver_tables: None,
        })
    }

    /// Adopt a reserved root without assuming a linear CPU mapping or page
    /// allocator ownership of any inherited table. Firmware must be stopped,
    /// and callers must serialize all subsequent changes with the UAT owner.
    ///
    /// # Safety
    /// The inherited tables must not have another writer for this object's
    /// lifetime. Before dropping it the caller must stop all hardware walks
    /// and invalidate any cached translations that reference owned tables.
    pub(crate) unsafe fn new_with_reserved_ttb(
        ttb: PhysicalAddr,
        va_range: Range<u64>,
        ias: u8,
        oas: u32,
        reserved_tables: ReservedTables,
    ) -> Result<Self> {
        let geometry = UatGeometry::new(ias).ok_or(EINVAL)?;
        if va_range.is_empty() || (va_range.start | va_range.end) & UAT_PGMSK as u64 != 0 {
            return Err(EINVAL);
        }
        reserved_tables.validate_root(ttb, ias, oas)?;
        Ok(Self {
            ttb,
            ttb_owned: false,
            quarantined: false,
            noncoherent: true,
            dma_tables: None,
            reserved_tables: Some(reserved_tables),
            va_range,
            geometry,
            oas_mask: (1u64 << oas) - 1,
            coverage: None,
            ordered_walk: false,
            driver_tables: None,
        })
    }

    /// # Safety
    /// Only J514S after RTKit wake and before initdata publication. Firmware
    /// owns root entries 0/1; no firmware or GPU consumer holds a host VA yet.
    pub(crate) unsafe fn new_with_m3_live_ttb(ttb:PhysicalAddr,va_range:Range<u64>,tables:ReservedTables,dev:&crate::driver::AsahiDevice)->Result<Self> {
        let geometry=UatGeometry::new(42).ok_or(EINVAL)?;
        if !tables.contains(ttb) || va_range.start!=geometry.kernel_va_base()
            || va_range.end!=geometry.kernel_va_top() {return Err(EINVAL);}
        tables.with_page(ttb,|entries| {
            pr_info!("M3 live root: {:x} {:x} {:x} {:x}\n",entries[0].load(Ordering::Acquire),entries[1].load(Ordering::Acquire),entries[2].load(Ordering::Acquire),entries[3].load(Ordering::Acquire));
            for entry in &entries[2..] {entry.store(0,Ordering::Release);}
            Self::publish_table(&entries[2..]);Ok(())
        })?;
        crate::mem::tlbi_all();crate::mem::sync();
        Ok(Self {ttb,ttb_owned:false,quarantined:false,noncoherent:true,dma_tables:Some(DmaTables::new(dev)),
            reserved_tables:Some(tables),va_range,geometry,oas_mask:(1u64<<42)-1,coverage:None,
            ordered_walk:false,driver_tables:None})
    }

    /// Adopt the loader-designated shared middle table before ASC starts.
    /// Firmware installs this same link when it switches translation regimes;
    /// allocating another table here would leave the host's mappings detached.
    pub(crate) fn install_reserved_middle(&mut self, iova: u64, physical: PhysicalAddr) -> Result {
        if !self.va_range.contains(&iova) || !self.is_reserved_table(physical) || physical == self.ttb {
            return Err(EINVAL);
        }
        let index = self.geometry.root_index(iova);
        let old = self.with_table(self.ttb, index, 1, false, |entries| Ok(entries[0].load(Ordering::Acquire)))?;
        if old != 0 {
            return if old & (self.oas_mask & !(UAT_PGMSK as u64)) == physical && old & 3 == 3 {
                Ok(())
            } else { Err(EBUSY) };
        }
        self.with_table(physical, 0, UAT_NPTE, false, |entries| {
            if entries.iter().any(|entry| entry.load(Ordering::Acquire) != 0) { Err(EBUSY) } else { Ok(()) }
        })?;
        self.with_table(self.ttb, index, 1, true, |entries| {
            entries[0].store(physical | PTE_TYPE_LEAF_TABLE, Ordering::Release);
            Ok(())
        })
    }

    fn is_reserved_table(&self, physical: PhysicalAddr) -> bool {
        self.reserved_tables.as_ref().is_some_and(|tables| tables.contains(physical))
    }

    /// Owned M3 user tables are written only by the host under the VM lock.
    /// Their leaf values do not publish any separately owned CPU data. The
    /// lock already provides acquire ordering between host writers/readers;
    /// acquiring every uncached DMA PTE serializes the whole sparse-VA scan.
    /// Firmware-owned/shared roots retain their existing acquire accesses.
    fn leaf_read_order(&self) -> Ordering {
        if self.ttb_owned && self.dma_tables.is_some() {
            Ordering::Relaxed
        } else {
            Ordering::Acquire
        }
    }

    fn with_table<T>(
        &self,
        physical: PhysicalAddr,
        index: usize,
        count: usize,
        write: bool,
        cb: impl FnOnce(&[Pte]) -> Result<T>,
    ) -> Result<T> {
        if index.checked_add(count).ok_or(EOVERFLOW)? > UAT_NPTE {
            return Err(EINVAL);
        }
        let access = |entries: &[Pte]| {
            let entries = &entries[index..index + count];
            let result = cb(entries);
            if write && self.noncoherent {
                // Publish only changed table spans, never a whole VM at each
                // submission. Also publish partial writes on a callback error.
                Self::publish_table(entries);
            }
            result
        };
        if let Some(pointer)=self.dma_tables.as_ref().and_then(|d|d.pointer(physical)) {
            // SAFETY: retained DMA page and serialized VM table access. No
            // Rust reference to DMA-owned bytes escapes this callback.
            return access(unsafe {core::slice::from_raw_parts(pointer,UAT_NPTE)});
        }
        if let Some(tables) = &self.reserved_tables {
            if tables.contains(physical) {
                return tables.with_page(physical, access);
            }
        }
        // SAFETY: Imported roots were checked before adoption. Other table
        // pointers were published from Page::into_phys by this owner. Use the
        // checked borrow so an invalid physical mapping fails instead of being
        // treated as a direct-map address.
        let page = unsafe { Page::borrow_phys(&physical) }.ok_or(EFAULT)?;
        page.with_pointer_into_page(0, UAT_PGSZ, |pointer| {
            // SAFETY: The retained page mapping covers aligned AtomicU64s.
            access(unsafe { core::slice::from_raw_parts(pointer.cast::<Pte>(), UAT_NPTE) })
        })
    }

    fn publish_table(entries: &[Pte]) {
        if entries.is_empty() { return; }
        let ctr: u64;
        // SAFETY: CTR_EL0 is readable at EL1. All cleaned cachelines belong
        // to the live, page-aligned normal-memory table slice supplied here.
        unsafe {
            core::arch::asm!("mrs {ctr}, ctr_el0", ctr = out(reg) ctr, options(nomem, nostack, preserves_flags));
            let line = 4usize << ((ctr >> 16) & 0xf);
            let mut address = entries.as_ptr() as usize & !(line - 1);
            let end = entries.as_ptr() as usize + core::mem::size_of_val(entries);
            while address < end {
                core::arch::asm!("dc cvac, {address}", address = in(reg) address, options(nostack, preserves_flags));
                address += line;
            }
            core::arch::asm!("dsb osh", options(nostack, preserves_flags));
        }
    }

    fn free_table(&self, physical: PhysicalAddr) {
        if let Some(dma)=&self.dma_tables {dma.free(physical);return;}
        if !self.is_reserved_table(physical) {
            // SAFETY: The initial graph contains only reserved pages. Every
            // other table in this tree was allocated by this owner. The
            // caller detached it and quiesced GPU walks before reclamation.
            unsafe { Page::from_phys(physical) };
        }
    }

    pub(crate) fn quarantine(&mut self) {
        self.quarantined = true;
    }

    pub(crate) fn ttb(&self) -> PhysicalAddr {
        self.ttb
    }

    /// Makes every later change of this page table visible to a UAT walker that does not snoop the
    /// CPU caches (G15), and cleans the root, which was zeroed through the cacheable linear map.
    ///
    /// From now on the `noncoherent` publication of this file applies: a new table page is cleaned
    /// before its parent descriptor names it, and every written table span is cleaned after the
    /// store with a `dsb osh` ([`Self::publish_table`]). Call it before anything is mapped and
    /// before any context names the root.
    pub(crate) fn enable_clean_poc(&mut self) -> Result {
        self.noncoherent = true;
        // The zeroed root: published whole through the write path of `with_table`.
        self.with_table(self.ttb, 0, UAT_NPTE, true, |_| Ok(()))
    }

    /// INT's 42-bit (AGX3) walk with prepared, non-allocating leaf writes. 39-bit roots and G17
    /// tables use the leaf semantics of the 39-bit walk instead.
    fn agx3_walk(&self) -> bool {
        !self.ordered_walk && self.geometry.root_size() != 1u64 << 39
    }

    /// The G17 walk (`ordered_walk`): records driver-allocated tables below shared roots,
    /// completes child-table initialization before linking it, and detaches tables bottom-up.
    fn with_pages_ordered<F>(
        &mut self,
        iova_range: Range<u64>,
        alloc: bool,
        free: bool,
        mut cb: F,
    ) -> Result
    where
        F: FnMut(u64, &[Pte]) -> Result,
    {
        mod_pr_debug!(
            "UATPageTable::with_pages: {:#x?} alloc={} free={}\n",
            iova_range,
            alloc,
            free
        );
        if (iova_range.start | iova_range.end) & (UAT_PGMSK as u64) != 0 {
            pr_err!(
                "UATPageTable::with_pages: iova range not aligned: {:#x?}\n",
                iova_range
            );
            return Err(EINVAL);
        }

        if iova_range.is_empty() {
            return Ok(());
        }

        let mut iova = iova_range.start & self.geometry.root_mask();
        let mut last_iova = iova;
        // Handle the case where iova_range.end is just at the top boundary of the IAS
        let end = ((iova_range.end - 1) & self.geometry.root_mask()) + 1;

        let mut pt_addr: [Option<PhysicalAddr>; UAT_LEVELS] = Default::default();
        // For every child table, retain the parent page and entry that names it.
        // Children are detached before their parents are freed, so these borrowed
        // parent pages remain valid while the descriptor is cleared.
        let mut pt_parent: [Option<(PhysicalAddr, usize)>; UAT_LEVELS - 1] = Default::default();
        pt_addr[UAT_LEVELS - 1] = Some(self.ttb);

        'outer: while iova < end {
            mod_pr_debug!("UATPageTable::with_pages: iova={:#x}\n", iova);
            let addr_diff = last_iova ^ iova;
            // Detach leaves before intermediate tables. The old order freed a
            // parent first and could leave its descriptor naming a freed child.
            for level in 0..UAT_LEVELS - 1 {
                // If the iova has changed at this level or above, invalidate the physaddr
                if addr_diff & !((1 << (UAT_PGBIT + (level + 1) * UAT_LVBIT)) - 1) != 0 {
                    if let Some(phys) = pt_addr[level].take() {
                        if free {
                            mod_pr_debug!(
                                "UATPageTable::with_pages: free level {} {:#x?}\n",
                                level,
                                phys
                            );
                            let parent = pt_parent[level].take();
                            self.free_detached_table(phys, parent)?;
                        } else {
                            pt_parent[level] = None;
                        }
                        mod_pr_debug!("UATPageTable::with_pages: invalidate level {}\n", level);
                    }
                }
            }
            last_iova = iova;
            for level in (0..UAT_LEVELS - 1).rev() {
                // Fetch the page table base address for this level
                if pt_addr[level].is_none() {
                    let phys = pt_addr[level + 1].unwrap();
                    mod_pr_debug!(
                        "UATPageTable::with_pages: need level {}, parent phys {:#x}\n",
                        level,
                        phys
                    );
                    let upidx = ((iova >> (UAT_PGBIT + (level + 1) * UAT_LVBIT) as u64) & UAT_LVMSK)
                        as usize;
                    // SAFETY: Page table addresses are either allocated by us, or
                    // firmware-managed and safe to borrow a struct page from.
                    let upt = unsafe { Page::borrow_phys_unchecked(&phys) };
                    mod_pr_debug!("UATPageTable::with_pages: borrowed phys {:#x}\n", phys);
                    pt_addr[level] =
                        upt.with_pointer_into_page(upidx * PTE_SIZE, PTE_SIZE, |p| {
                            let uptep = p as *const _ as *const Pte;
                            // SAFETY: with_pointer_into_page() ensures the pointer is valid,
                            // and our index is aligned so it is safe to deref as an AtomicU64.
                            let upte = unsafe { &*uptep };
                            let mut upte_val = upte.load(Ordering::Relaxed);
                            // Allocate if requested
                            if upte_val == 0 && alloc {
                                let pt_page = Page::alloc_page(GFP_KERNEL | __GFP_ZERO)?;
                                mod_pr_debug!("UATPageTable::with_pages: alloc PT at {:#x}\n", pt_page.phys());
                                if let Some(tables) = self.driver_tables.as_mut() {
                                    tables.push(pt_page.phys(), GFP_KERNEL)?;
                                }
                                let pt_paddr = Page::into_phys(pt_page);
                                upte_val = pt_paddr | PTE_TYPE_LEAF_TABLE;
                                // Complete child-table initialization before a
                                // concurrent device walk can follow its parent.
                                // A later mapping/TLBI barrier cannot order an
                                // already-visible parent against earlier clears.
                                crate::mem::sync();
                                upte.store(upte_val, Ordering::Relaxed);
                            }
                            if upte_val & PTE_TYPE_BITS == PTE_TYPE_LEAF_TABLE {
                                Ok(Some(upte_val & self.oas_mask & (!UAT_PGMSK as u64)))
                            } else if upte_val == 0 || (!alloc && !free) {
                                mod_pr_debug!("UATPageTable::with_pages: no level {}\n", level);
                                Ok(None)
                            } else {
                                pr_err!("UATPageTable::with_pages: Unexpected Table PTE value {:#x} at iova {:#x} index {} phys {:#x}\n", upte_val,
                                        iova, level + 1, phys + ((upidx * PTE_SIZE) as PhysicalAddr));
                                Ok(None)
                            }
                        })?;
                    if pt_addr[level].is_some() {
                        pt_parent[level] = Some((phys, upidx));
                    }
                    mod_pr_debug!(
                        "UATPageTable::with_pages: level {} PT {:#x?}\n",
                        level,
                        pt_addr[level]
                    );
                }
                // If we don't have a page table, skip this entire level
                if pt_addr[level].is_none() {
                    let block = 1 << (UAT_PGBIT + UAT_LVBIT * (level + 1));
                    let old = iova;
                    iova = align(iova + 1, block);
                    mod_pr_debug!(
                        "UATPageTable::with_pages: skip {:#x} {:#x} -> {:#x}\n",
                        block,
                        old,
                        iova
                    );
                    continue 'outer;
                }
            }

            let idx = ((iova >> UAT_PGBIT as u64) & UAT_LVMSK) as usize;
            let max_count = UAT_NPTE - idx;
            let count = (((end - iova) >> UAT_PGBIT) as usize).min(max_count);
            let phys = pt_addr[0].unwrap();
            mod_pr_debug!(
                "UATPageTable::with_pages: leaf PT at {:#x} idx {:#x} count {:#x} iova {:#x}\n",
                phys,
                idx,
                count,
                iova
            );
            // SAFETY: Page table addresses are either allocated by us, or
            // firmware-managed and safe to borrow a struct page from.
            let pt = unsafe { Page::borrow_phys_unchecked(&phys) };
            pt.with_pointer_into_page(idx * PTE_SIZE, count * PTE_SIZE, |p| {
                let ptep = p as *const _ as *const Pte;
                // SAFETY: We know this is a valid pointer to PTEs and the range is valid and
                // checked by with_pointer_into_page().
                let ptes = unsafe { core::slice::from_raw_parts(ptep, count) };
                cb(iova, ptes)?;
                Ok(())
            })?;

            let block = 1 << (UAT_PGBIT + UAT_LVBIT);
            iova = align(iova + 1, block);
        }

        if free {
            // The final path is detached bottom-up for the same reason as the
            // boundary transitions above.
            for level in 0..UAT_LEVELS - 1 {
                if let Some(phys) = pt_addr[level] {
                    mod_pr_debug!(
                        "UATPageTable::with_pages: free level {} {:#x?}\n",
                        level,
                        phys
                    );
                    self.free_detached_table(phys, pt_parent[level].take())?;
                }
            }
        }

        Ok(())
    }

    /// Free a page table, unless it was not allocated by the driver.
    fn free_detached_table(
        &mut self,
        phys: PhysicalAddr,
        parent: Option<(PhysicalAddr, usize)>,
    ) -> Result {
        let owned = if let Some(tables) = self.driver_tables.as_ref() {
            match tables.iter().position(|&table| table == phys) {
                Some(_) => true,
                None => false,
            }
        } else {
            true
        };
        if !owned {
            return Ok(());
        }
        if let Some((parent_phys, parent_index)) = parent {
            // SAFETY: The bottom-up traversal keeps the parent allocated while
            // its child descriptor is detached. A changed descriptor indicates
            // an ownership violation; fail closed instead of freeing a table
            // that may still be reachable.
            let page = unsafe { Page::borrow_phys(&parent_phys) }.ok_or(EIO)?;
            page.with_pointer_into_page(parent_index * PTE_SIZE, PTE_SIZE, |p| {
                let pte = unsafe { &*(p as *const AtomicU64) };
                let current = pte.load(Ordering::Acquire);
                let expected = phys & self.oas_mask & (!UAT_PGMSK as u64);
                if current & self.oas_mask & (!UAT_PGMSK as u64) != expected {
                    return Err(EIO);
                }
                pte.store(0, Ordering::Release);
                Ok(())
            })?;
        }
        if let Some(tables) = self.driver_tables.as_mut() {
            let index = tables
                .iter()
                .position(|&table| table == phys)
                .ok_or(EIO)?;
            tables.swap_remove(index);
        }
        // SAFETY: Page tables allocated by the driver always come from Page::into_phys(). Without
        // `driver_tables`, every child table belongs to the driver.
        unsafe { Page::from_phys(phys) };
        Ok(())
    }

    fn with_pages_legacy<F>(
        &mut self,
        iova_range: Range<u64>,
        alloc: bool,
        free: bool,
        mut cb: F,
    ) -> Result
    where
        F: FnMut(u64, &[Pte]) -> Result,
    {
        if self.ordered_walk {
            return self.with_pages_ordered(iova_range, alloc, free, cb);
        }
        mod_pr_debug!(
            "UATPageTable::with_pages: {:#x?} alloc={} free={}\n",
            iova_range,
            alloc,
            free
        );
        if (iova_range.start | iova_range.end) & (UAT_PGMSK as u64) != 0 {
            pr_err!(
                "UATPageTable::with_pages: iova range not aligned: {:#x?}\n",
                iova_range
            );
            return Err(EINVAL);
        }

        if iova_range.is_empty() {
            return Ok(());
        }

        let mut iova = iova_range.start & self.geometry.root_mask();
        let mut last_iova = iova;
        // Handle the case where iova_range.end is just at the top boundary of the IAS
        let end = ((iova_range.end - 1) & self.geometry.root_mask()) + 1;

        let mut pt_addr: [Option<PhysicalAddr>; UAT_LEVELS] = Default::default();
        pt_addr[UAT_LEVELS - 1] = Some(self.ttb);

        'outer: while iova < end {
            mod_pr_debug!("UATPageTable::with_pages: iova={:#x}\n", iova);
            let addr_diff = last_iova ^ iova;
            for level in (0..UAT_LEVELS - 1).rev() {
                // If the iova has changed at this level or above, invalidate the physaddr
                if addr_diff & !((1 << (UAT_PGBIT + (level + 1) * UAT_LVBIT)) - 1) != 0 {
                    if let Some(phys) = pt_addr[level].take() {
                        if free {
                            mod_pr_debug!(
                                "UATPageTable::with_pages: free level {} {:#x?}\n",
                                level,
                                phys
                            );
                            // SAFETY: Page tables for our VA ranges always come from Page::into_phys().
                            unsafe { Page::from_phys(phys) };
                        }
                        mod_pr_debug!("UATPageTable::with_pages: invalidate level {}\n", level);
                    }
                }
            }
            last_iova = iova;
            for level in (0..UAT_LEVELS - 1).rev() {
                // Fetch the page table base address for this level
                if pt_addr[level].is_none() {
                    let phys = pt_addr[level + 1].unwrap();
                    mod_pr_debug!(
                        "UATPageTable::with_pages: need level {}, parent phys {:#x}\n",
                        level,
                        phys
                    );
                    let upidx = ((iova >> (UAT_PGBIT + (level + 1) * UAT_LVBIT) as u64) & UAT_LVMSK)
                        as usize;
                    // SAFETY: Page table addresses are either allocated by us, or
                    // firmware-managed and safe to borrow a struct page from.
                    let upt = unsafe { Page::borrow_phys_unchecked(&phys) };
                    mod_pr_debug!("UATPageTable::with_pages: borrowed phys {:#x}\n", phys);
                    pt_addr[level] =
                        upt.with_pointer_into_page(upidx * PTE_SIZE, PTE_SIZE, |p| {
                            let uptep = p as *const _ as *const Pte;
                            // SAFETY: with_pointer_into_page() ensures the pointer is valid,
                            // and our index is aligned so it is safe to deref as an AtomicU64.
                            let upte = unsafe { &*uptep };
                            let mut upte_val = upte.load(Ordering::Relaxed);
                            // Allocate if requested
                            if upte_val == 0 && alloc {
                                let pt_page = Page::alloc_page(GFP_KERNEL | __GFP_ZERO)?;
                                mod_pr_debug!("UATPageTable::with_pages: alloc PT at {:#x}\n", pt_page.phys());
                                let pt_paddr = Page::into_phys(pt_page);
                                upte_val = pt_paddr | PTE_TYPE_LEAF_TABLE;
                                upte.store(upte_val, Ordering::Relaxed);
                            }
                            if upte_val & PTE_TYPE_BITS == PTE_TYPE_LEAF_TABLE {
                                Ok(Some(upte_val & self.oas_mask & (!UAT_PGMSK as u64)))
                            } else if upte_val == 0 || (!alloc && !free) {
                                mod_pr_debug!("UATPageTable::with_pages: no level {}\n", level);
                                Ok(None)
                            } else {
                                pr_err!("UATPageTable::with_pages: Unexpected Table PTE value {:#x} at iova {:#x} index {} phys {:#x}\n", upte_val,
                                        iova, level + 1, phys + ((upidx * PTE_SIZE) as PhysicalAddr));
                                Ok(None)
                            }
                        })?;
                    mod_pr_debug!(
                        "UATPageTable::with_pages: level {} PT {:#x?}\n",
                        level,
                        pt_addr[level]
                    );
                }
                // If we don't have a page table, skip this entire level
                if pt_addr[level].is_none() {
                    let block = 1 << (UAT_PGBIT + UAT_LVBIT * (level + 1));
                    let old = iova;
                    iova = align(iova + 1, block);
                    mod_pr_debug!(
                        "UATPageTable::with_pages: skip {:#x} {:#x} -> {:#x}\n",
                        block,
                        old,
                        iova
                    );
                    continue 'outer;
                }
            }

            let idx = ((iova >> UAT_PGBIT as u64) & UAT_LVMSK) as usize;
            let max_count = UAT_NPTE - idx;
            let count = (((end - iova) >> UAT_PGBIT) as usize).min(max_count);
            let phys = pt_addr[0].unwrap();
            mod_pr_debug!(
                "UATPageTable::with_pages: leaf PT at {:#x} idx {:#x} count {:#x} iova {:#x}\n",
                phys,
                idx,
                count,
                iova
            );
            // SAFETY: Page table addresses are either allocated by us, or
            // firmware-managed and safe to borrow a struct page from.
            let pt = unsafe { Page::borrow_phys_unchecked(&phys) };
            pt.with_pointer_into_page(idx * PTE_SIZE, count * PTE_SIZE, |p| {
                let ptep = p as *const _ as *const Pte;
                // SAFETY: We know this is a valid pointer to PTEs and the range is valid and
                // checked by with_pointer_into_page().
                let ptes = unsafe { core::slice::from_raw_parts(ptep, count) };
                cb(iova, ptes)?;
                Ok(())
            })?;

            let block = 1 << (UAT_PGBIT + UAT_LVBIT);
            iova = align(iova + 1, block);
        }

        if free {
            for level in (0..UAT_LEVELS - 1).rev() {
                if let Some(phys) = pt_addr[level] {
                    mod_pr_debug!(
                        "UATPageTable::with_pages: free level {} {:#x?}\n",
                        level,
                        phys
                    );
                    // SAFETY: Page tables for our VA ranges always come from Page::into_phys().
                    unsafe { Page::from_phys(phys) };
                }
            }
        }

        Ok(())
    }

    fn with_pages<F>(
        &mut self,
        iova_range: Range<u64>,
        alloc: bool,
        free: bool,
        write: bool,
        mut cb: F,
    ) -> Result
    where
        F: FnMut(u64, &[Pte]) -> Result,
    {
        if !self.agx3_walk() {
            return self.with_pages_legacy(iova_range, alloc, free, cb);
        }

        // All leaf mutations use this walk. Invalidate before even a partial
        // or failed write, including allocation/freeing of intermediate tables.
        if write || alloc || free { if let Some(cache) = &mut self.coverage { cache.invalidate(); } }
        mod_pr_debug!(
            "UATPageTable::with_pages: {:#x?} alloc={} free={}\n",
            iova_range,
            alloc,
            free
        );
        if (iova_range.start | iova_range.end) & (UAT_PGMSK as u64) != 0 {
            pr_err!(
                "UATPageTable::with_pages: iova range not aligned: {:#x?}\n",
                iova_range
            );
            return Err(EINVAL);
        }

        if iova_range.is_empty() {
            return Ok(());
        }

        let ias_mask = self.geometry.root_mask();
        let mut iova = iova_range.start & ias_mask;
        let mut last_iova = iova;
        // Handle the case where iova_range.end is just at the top boundary of the IAS
        let end = ((iova_range.end - 1) & ias_mask) + 1;

        let mut pt_addr: [Option<PhysicalAddr>; UAT_LEVELS] = Default::default();
        pt_addr[UAT_LEVELS - 1] = Some(self.ttb);

        'outer: while iova < end {
            mod_pr_debug!("UATPageTable::with_pages: iova={:#x}\n", iova);
            let addr_diff = last_iova ^ iova;
            for level in (0..UAT_LEVELS - 1).rev() {
                // If the iova has changed at this level or above, invalidate the physaddr
                if addr_diff & !((1 << (UAT_PGBIT + (level + 1) * UAT_LVBIT)) - 1) != 0 {
                    if let Some(phys) = pt_addr[level].take() {
                        if free {
                            mod_pr_debug!(
                                "UATPageTable::with_pages: free level {} {:#x?}\n",
                                level,
                                phys
                            );
                            // SAFETY: Page tables for our VA ranges always come from Page::into_phys().
                            self.free_table(phys);
                        }
                        mod_pr_debug!("UATPageTable::with_pages: invalidate level {}\n", level);
                    }
                }
            }
            last_iova = iova;
            for level in (0..UAT_LEVELS - 1).rev() {
                // Fetch the page table base address for this level
                if pt_addr[level].is_none() {
                    let phys = pt_addr[level + 1].unwrap();
                    mod_pr_debug!(
                        "UATPageTable::with_pages: need level {}, parent phys {:#x}\n",
                        level,
                        phys
                    );
                    let upidx = ((iova >> (UAT_PGBIT + (level + 1) * UAT_LVBIT) as u64) & UAT_LVMSK)
                        as usize;
                    pt_addr[level] =
                        self.with_table(phys, upidx, 1, alloc || free, |entries| {
                            let upte = &entries[0];
                            let mut upte_val = upte.load(Ordering::Relaxed);
                            // Allocate if requested
                            if upte_val == 0 && alloc {
                                let pt_paddr=if let Some(dma)=&self.dma_tables {dma.alloc()?} else {
                                let pt_page = Page::alloc_page(GFP_KERNEL | __GFP_ZERO)?;
                                mod_pr_debug!("UATPageTable::with_pages: alloc PT at {:#x}\n", pt_page.phys());
                                if self.noncoherent {
                                    pt_page.with_pointer_into_page(0, UAT_PGSZ, |pointer| {
                                        // SAFETY: Fresh zeroed page, still owned here.
                                        Self::publish_table(unsafe { core::slice::from_raw_parts(pointer.cast::<Pte>(), UAT_NPTE) });
                                        Ok(())
                                    })?;
                                }
                                Page::into_phys(pt_page)
                                };
                                upte_val = pt_paddr | PTE_TYPE_LEAF_TABLE;
                                upte.store(upte_val, Ordering::Relaxed);
                            }
                            if upte_val & PTE_TYPE_BITS == PTE_TYPE_LEAF_TABLE {
                                let child = upte_val & self.oas_mask & (!UAT_PGMSK as u64);
                                if free && self.reserved_tables.is_some() && !self.is_reserved_table(child) {
                                    upte.store(0, Ordering::Relaxed);
                                }
                                Ok(Some(child))
                            } else if upte_val == 0 || (!alloc && !free) {
                                mod_pr_debug!("UATPageTable::with_pages: no level {}\n", level);
                                Ok(None)
                            } else {
                                pr_err!("UATPageTable::with_pages: Unexpected Table PTE value {:#x} at iova {:#x} index {} phys {:#x}\n", upte_val,
                                        iova, level + 1, phys + ((upidx * PTE_SIZE) as PhysicalAddr));
                                Ok(None)
                            }
                        })?;
                    mod_pr_debug!(
                        "UATPageTable::with_pages: level {} PT {:#x?}\n",
                        level,
                        pt_addr[level]
                    );
                }
                // If we don't have a page table, skip this entire level
                if pt_addr[level].is_none() {
                    let block = 1 << (UAT_PGBIT + UAT_LVBIT * (level + 1));
                    let old = iova;
                    iova = align(iova + 1, block);
                    mod_pr_debug!(
                        "UATPageTable::with_pages: skip {:#x} {:#x} -> {:#x}\n",
                        block,
                        old,
                        iova
                    );
                    continue 'outer;
                }
            }

            let idx = ((iova >> UAT_PGBIT as u64) & UAT_LVMSK) as usize;
            let max_count = UAT_NPTE - idx;
            let count = (((end - iova) >> UAT_PGBIT) as usize).min(max_count);
            let phys = pt_addr[0].unwrap();
            mod_pr_debug!(
                "UATPageTable::with_pages: leaf PT at {:#x} idx {:#x} count {:#x} iova {:#x}\n",
                phys,
                idx,
                count,
                iova
            );
            self.with_table(phys, idx, count, write, |ptes| cb(iova, ptes))?;

            let block = 1 << (UAT_PGBIT + UAT_LVBIT);
            iova = align(iova + 1, block);
        }

        if free {
            for level in (0..UAT_LEVELS - 1).rev() {
                if let Some(phys) = pt_addr[level] {
                    mod_pr_debug!(
                        "UATPageTable::with_pages: free level {} {:#x?}\n",
                        level,
                        phys
                    );
                    // SAFETY: Page tables for our VA ranges always come from Page::into_phys().
                    self.free_table(phys);
                }
            }
        }

        Ok(())
    }

    pub(crate) fn alloc_pages(&mut self, iova_range: Range<u64>) -> Result {
        mod_pr_debug!("UATPageTable::alloc_pages: {:#x?}\n", iova_range);
        self.with_pages(iova_range, true, false, false, |_, _| Ok(()))
    }

    /// Reserve every intermediate table required by a new leaf mapping and
    /// prove that the complete destination span is empty before any leaf is
    /// installed.  GPUVM and the driver's fixed-mapping allocator are
    /// intentionally separate address managers, so neither allocator alone
    /// can detect ownership held by the other one.  The page table is the
    /// authoritative final arbiter.
    pub(crate) fn prepare_map(&mut self, iova_range: Range<u64>) -> Result {
        if self.ordered_walk {
            // G17: GPUVM and the fixed-address allocator may share a page table, so check
            // that neither owns a destination leaf before allowing failure rollback.
            if iova_range.is_empty() || (iova_range.start | iova_range.end) & UAT_PGMSK as u64 != 0 {
                return Err(EINVAL);
            }
            self.with_pages(iova_range.clone(), false, false, false, |_, ptes| {
                if ptes.iter().any(|pte| pte.load(Ordering::Acquire) != 0) {
                    return Err(EBUSY);
                }
                Ok(())
            })?;
            return self.alloc_pages(iova_range);
        }
        if self.geometry.root_size() == 1u64 << 39 { return self.alloc_pages(iova_range); }

        if iova_range.is_empty()
            || (iova_range.start | iova_range.end) & UAT_PGMSK as u64 != 0
        {
            return Err(EINVAL);
        }

        let read_order = self.leaf_read_order();
        let mut occupied = None;
        self.with_pages(iova_range.clone(), false, false, false, |iova, ptes| {
            for (index, pte) in ptes.iter().enumerate() {
                let value = pte.load(read_order);
                if value != 0 && occupied.is_none() {
                    occupied = Some((iova + (index * UAT_PGSZ) as u64, value));
                }
            }
            Ok(())
        })?;
        if let Some((iova, pte)) = occupied {
            pr_err!(
                "UATPageTable::prepare_map: Page at IOVA {:#x} is already owned (PTE: {:#x})\n",
                iova,
                pte
            );
            return Err(EBUSY);
        }

        // Do all fallible page-table allocation before installing any leaf.
        // A failed allocation can leave empty intermediate tables behind, but
        // it can never leave a partially visible object mapping.
        self.alloc_pages(iova_range)
    }

    /// Check that every page in an aligned range is mapped with the requested
    /// GPU permissions. Missing intermediate page tables are detected by the
    /// visited-page count because `with_pages` skips those holes.
    pub(crate) fn covers_range(
        &mut self,
        iova_range: Range<u64>,
        need_read: bool,
        need_write: bool,
    ) -> Result<bool> {
        if iova_range.is_empty() || (iova_range.start | iova_range.end) & UAT_PGMSK as u64 != 0 {
            return Ok(false);
        }

        // Only M3's exclusively host-owned coherent user tables qualify.
        // Borrowed/firmware-owned roots always walk the current PTEs. GPUVM's
        // execution lock serializes this check with every host table mutation.
        let cache = self.ttb_owned && self.dma_tables.is_some();
        let (start, end) = (iova_range.start, iova_range.end);
        if cache && self.coverage.as_ref().is_some_and(|cache| cache.covers(start, end, need_read, need_write)) {
            return Ok(true);
        }
        let expected = (iova_range.end - iova_range.start) >> UAT_PGBIT;
        let mut visited = 0u64;
        let mut permitted = true;
        let read_order = self.leaf_read_order();
        self.with_pages(iova_range, false, false, false, |_, ptes| {
            visited = visited.checked_add(ptes.len() as u64).ok_or(EOVERFLOW)?;
            for pte in ptes {
                let value = pte.load(read_order);
                if value & PTE_TYPE_BITS != PTE_TYPE_LEAF_TABLE
                    || !Prot::from_pte(value).allows_gpu(need_read, need_write)
                {
                    permitted = false;
                }
            }
            Ok(())
        })?;
        let covered = complete_coverage(expected, visited, permitted);
        if cache && covered { if let Some(cache) = &mut self.coverage { cache.remember(start, end, need_read, need_write); } }
        Ok(covered)
    }

    fn pte_bits(&self) -> u64 {
        if self.ttb_owned {
            // Owned page tables are userspace, so non-global
            PTE_TYPE_LEAF_TABLE | UAT_NON_GLOBAL
        } else {
            // The sole non-owned page table is kernelspace, so global
            PTE_TYPE_LEAF_TABLE
        }
    }

    pub(crate) fn map_pages_legacy(
        &mut self,
        iova_range: Range<u64>,
        mut phys: PhysicalAddr,
        prot: Prot,
        one_page: bool,
    ) -> Result {
        mod_pr_debug!(
            "UATPageTable::map_pages: {:#x?} {:#x?} {:?}\n",
            iova_range,
            phys,
            prot
        );
        if phys & (UAT_PGMSK as PhysicalAddr) != 0 {
            pr_err!("UATPageTable::map_pages: phys not aligned: {:#x?}\n", phys);
            return Err(EINVAL);
        }

        let pte_bits = self.pte_bits();

        self.with_pages_legacy(iova_range, true, false, |iova, ptes| {
            for (idx, pte) in ptes.iter().enumerate() {
                let ptev = pte.load(Ordering::Relaxed);
                if ptev != 0 {
                    pr_err!(
                        "UATPageTable::map_pages: Page at IOVA {:#x} is mapped (PTE: {:#x})\n",
                        iova + (idx * UAT_PGSZ) as u64,
                        ptev
                    );
                }
                pte.store(phys | prot.as_pte() | pte_bits, Ordering::Relaxed);
                if !one_page {
                    phys += UAT_PGSZ as PhysicalAddr;
                }
            }
            Ok(())
        })
    }

    pub(crate) fn map_pages(
        &mut self,
        iova_range: Range<u64>,
        mut phys: PhysicalAddr,
        prot: Prot,
        one_page: bool,
    ) -> Result {
        if !self.agx3_walk() {
            return self.map_pages_legacy(iova_range, phys, prot, one_page);
        }

        mod_pr_debug!(
            "UATPageTable::map_pages: {:#x?} {:#x?} {:?}\n",
            iova_range,
            phys,
            prot
        );
        if phys & (UAT_PGMSK as PhysicalAddr) != 0 {
            pr_err!("UATPageTable::map_pages: phys not aligned: {:#x?}\n", phys);
            return Err(EINVAL);
        }

        let pte_bits = self.pte_bits();

        // Callers prepare the complete object range while holding the VM
        // execution lock.  Do not allocate here: mapping one scatterlist run
        // and then failing to allocate tables for a later run would create an
        // ownerless partial mapping.
        let expected = (iova_range.end - iova_range.start) >> UAT_PGBIT;
        let mut visited = 0u64;
        let mut occupied = None;
        let read_order = self.leaf_read_order();
        self.with_pages(iova_range.clone(), false, false, false, |iova, ptes| {
            visited = visited.checked_add(ptes.len() as u64).ok_or(EOVERFLOW)?;
            for (idx, pte) in ptes.iter().enumerate() {
                let value = pte.load(read_order);
                if value != 0 && occupied.is_none() {
                    occupied = Some((iova + (idx * UAT_PGSZ) as u64, value));
                }
            }
            Ok(())
        })?;
        if visited != expected {
            pr_err!(
                "UATPageTable::map_pages: destination tables were not prepared ({}/{} pages)\n",
                visited,
                expected
            );
            return Err(EFAULT);
        }
        if let Some((iova, pte)) = occupied {
            pr_err!(
                "UATPageTable::map_pages: refusing to overwrite owned page at IOVA {:#x} (PTE: {:#x})\n",
                iova,
                pte
            );
            return Err(EBUSY);
        }
        self.with_pages(iova_range, false, false, true, |_, ptes| {
            for pte in ptes {
                pte.store(phys | prot.as_pte() | pte_bits, Ordering::Relaxed);
                if !one_page {
                    phys += UAT_PGSZ as PhysicalAddr;
                }
            }
            Ok(())
        })
    }

    pub(crate) fn reprot_pages_legacy(&mut self, iova_range: Range<u64>, prot: Prot) -> Result {
        mod_pr_debug!(
            "UATPageTable::reprot_pages: {:#x?} {:?}\n",
            iova_range,
            prot
        );
        self.with_pages_legacy(iova_range, true, false, |iova, ptes| {
            for (idx, pte) in ptes.iter().enumerate() {
                let ptev = pte.load(Ordering::Relaxed);
                if ptev & PTE_TYPE_BITS != PTE_TYPE_LEAF_TABLE {
                    pr_err!(
                        "UATPageTable::reprot_pages: Page at IOVA {:#x} is unmapped (PTE: {:#x})\n",
                        iova + (idx * UAT_PGSZ) as u64,
                        ptev
                    );
                    continue;
                }
                pte.store((ptev & !UAT_PROT_BITS) | prot.as_pte(), Ordering::Relaxed);
            }
            Ok(())
        })
    }

    pub(crate) fn reprot_pages(&mut self, iova_range: Range<u64>, prot: Prot) -> Result {
        if !self.agx3_walk() {
            return self.reprot_pages_legacy(iova_range, prot);
        }

        mod_pr_debug!(
            "UATPageTable::reprot_pages: {:#x?} {:?}\n",
            iova_range,
            prot
        );
        // Preserve the established zero-length range semantics.  In
        // particular, GPUVM can produce an empty remap hole when the retained
        // prefix and suffix exactly cover the old VA.
        if iova_range.is_empty() {
            return Ok(());
        }
        if !self.covers_range(iova_range.clone(), false, false)? {
            pr_err!(
                "UATPageTable::reprot_pages: refusing partially unmapped range {:#x?}\n",
                iova_range
            );
            return Err(EFAULT);
        }
        self.with_pages(iova_range, false, false, true, |iova, ptes| {
            for (idx, pte) in ptes.iter().enumerate() {
                let ptev = pte.load(Ordering::Relaxed);
                if ptev & PTE_TYPE_BITS != PTE_TYPE_LEAF_TABLE {
                    pr_err!(
                        "UATPageTable::reprot_pages: Page at IOVA {:#x} is unmapped (PTE: {:#x})\n",
                        iova + (idx * UAT_PGSZ) as u64,
                        ptev
                    );
                    return Err(EFAULT);
                }
                pte.store((ptev & !UAT_PROT_BITS) | prot.as_pte(), Ordering::Relaxed);
            }
            Ok(())
        })
    }

    pub(crate) fn unmap_pages_legacy(&mut self, iova_range: Range<u64>) -> Result {
        mod_pr_debug!("UATPageTable::unmap_pages: {:#x?}\n", iova_range);
        self.with_pages_legacy(iova_range, false, false, |iova, ptes| {
            for (idx, pte) in ptes.iter().enumerate() {
                if pte.load(Ordering::Relaxed) & PTE_TYPE_LEAF_TABLE == 0 {
                    pr_err!(
                        "UATPageTable::unmap_pages: Page at IOVA {:#x} already unmapped\n",
                        iova + (idx * UAT_PGSZ) as u64
                    );
                }
                pte.store(0, Ordering::Relaxed);
            }
            Ok(())
        })
    }

    pub(crate) fn unmap_pages(&mut self, iova_range: Range<u64>) -> Result {
        if !self.agx3_walk() {
            return self.unmap_pages_legacy(iova_range);
        }

        mod_pr_debug!("UATPageTable::unmap_pages: {:#x?}\n", iova_range);
        if iova_range.is_empty() {
            return Ok(());
        }
        if !self.covers_range(iova_range.clone(), false, false)? {
            pr_err!(
                "UATPageTable::unmap_pages: refusing partially unmapped range {:#x?}\n",
                iova_range
            );
            return Err(EFAULT);
        }
        self.with_pages(iova_range, false, false, true, |iova, ptes| {
            for (idx, pte) in ptes.iter().enumerate() {
                if pte.load(Ordering::Relaxed) & PTE_TYPE_LEAF_TABLE == 0 {
                    pr_err!(
                        "UATPageTable::unmap_pages: Page at IOVA {:#x} already unmapped\n",
                        iova + (idx * UAT_PGSZ) as u64
                    );
                    return Err(EFAULT);
                }
                pte.store(0, Ordering::Relaxed);
            }
            Ok(())
        })
    }

    /// Reads one mapped leaf entry without allocating missing tables.
    pub(crate) fn leaf_entry(&mut self, iova: u64) -> Result<u64> {
        if iova & UAT_PGMSK as u64 != 0 {
            return Err(EINVAL);
        }
        let end = iova.checked_add(UAT_PGSZ as u64).ok_or(EINVAL)?;
        let mut value = 0;
        self.with_pages(iova..end, false, false, false, |_, ptes| {
            if let Some(pte) = ptes.first() {
                value = pte.load(Ordering::Relaxed);
            }
            Ok(())
        })?;
        if value & PTE_TYPE_BITS != PTE_TYPE_LEAF_TABLE {
            return Err(EFAULT);
        }
        Ok(value)
    }

    /// Discards an aligned map operation that failed before acquiring a GPUVA owner.
    /// Missing tables and leaves are expected; existing table storage stays allocated.
    pub(crate) fn discard_partial_map(&mut self, iova_range: Range<u64>) -> Result {
        self.with_pages(iova_range, false, false, true, |_, ptes| {
            for pte in ptes {
                pte.store(0, Ordering::Relaxed);
            }
            Ok(())
        })
    }

    /// Read one PTE from a page table page, without ever allocating or modifying anything.
    ///
    /// Fails with an error (instead of oopsing) if the table page is not mapped.
    fn read_pte(&self, table: PhysicalAddr, idx: usize) -> Result<u64> {
        self.with_table(table, idx, 1, false, |entries| {
            Ok(entries[0].load(Ordering::Relaxed))
        })
        .inspect_err(|_| {
            pr_err!("UATPageTable::lookup: table at {:#x} is not readable\n", table);
        })
    }

    /// Read-only translation of one IOVA through this page table.
    ///
    /// Returns `Ok(Some((pa, pte)))` for a valid page (level 0) or 32 MiB block (level 1)
    /// descriptor, `Ok(None)` if the address is not mapped, and an error if a table page cannot
    /// be read. Never allocates page tables and never writes a PTE.
    pub(crate) fn lookup(&self, iova: u64) -> Result<Option<(PhysicalAddr, u64)>> {
        let va = iova & self.geometry.root_mask();
        let mut table = self.ttb;
        for level in (0..UAT_LEVELS).rev() {
            let shift = UAT_PGBIT + level * UAT_LVBIT;
            let idx = ((va >> shift) & UAT_LVMSK) as usize;
            let pte = self.read_pte(table, idx)?;
            let ty = pte & PTE_TYPE_BITS;
            if level == 0 {
                if ty != PTE_TYPE_LEAF_TABLE {
                    return Ok(None);
                }
                let pa = (pte & self.oas_mask & !(UAT_PGMSK as u64)) | (va & UAT_PGMSK as u64);
                return Ok(Some((pa as PhysicalAddr, pte)));
            }
            match ty {
                PTE_TYPE_LEAF_TABLE => {
                    table = (pte & self.oas_mask & !(UAT_PGMSK as u64)) as PhysicalAddr;
                }
                // Block descriptor: only valid one level above the leaves with a 16K granule.
                1 if level == 1 => {
                    let bmask = (1u64 << shift) - 1;
                    let pa = (pte & self.oas_mask & !bmask) | (va & bmask);
                    return Ok(Some((pa as PhysicalAddr, pte)));
                }
                _ => return Ok(None),
            }
        }
        Ok(None)
    }

    /// Read-only translation of a UAT-page aligned IOVA range: returns the physical address and
    /// the leaf descriptor of every UAT page in the range, or ENOENT if any page is unmapped.
    /// Never allocates page tables.
    pub(crate) fn translate_range(
        &self,
        iova_range: Range<u64>,
    ) -> Result<KVec<(PhysicalAddr, u64)>> {
        if (iova_range.start | iova_range.end) & (UAT_PGMSK as u64) != 0 || iova_range.is_empty() {
            return Err(EINVAL);
        }
        let count = ((iova_range.end - iova_range.start) >> UAT_PGBIT) as usize;
        let mut pas = KVec::with_capacity(count, GFP_KERNEL)?;
        let mut iova = iova_range.start;
        while iova < iova_range.end {
            match self.lookup(iova)? {
                Some(t) => pas.push(t, GFP_KERNEL)?,
                None => {
                    pr_err!(
                        "UATPageTable::translate_range: IOVA {:#x} is not mapped\n",
                        iova
                    );
                    return Err(ENOENT);
                }
            }
            iova += UAT_PGSZ as u64;
        }
        Ok(pas)
    }

    #[cfg(CONFIG_DEV_COREDUMP)]
    pub(crate) fn dump_pages(&mut self, iova_range: Range<u64>) -> Result<KVVec<DumpedPage>> {
        let mut pages = KVVec::new();
        let oas_mask = self.oas_mask;
        let iova_base = self.va_range.start & !self.geometry.root_mask();
        self.with_pages(iova_range, false, false, false, |iova, ptes| {
            let iova = iova | iova_base;
            for (idx, ppte) in ptes.iter().enumerate() {
                let pte = ppte.load(Ordering::Relaxed);
                if (pte & PTE_TYPE_LEAF_TABLE) != PTE_TYPE_LEAF_TABLE {
                    continue;
                }
                let memattr = ((pte & UAT_MEMATTR_BITS) >> UAT_MEMATTR_SHIFT) as u8;

                if !(memattr == MEMATTR_CACHED || memattr == MEMATTR_UNCACHED) {
                    pages.push(
                        DumpedPage {
                            iova: iova + (idx * UAT_PGSZ) as u64,
                            pte,
                            data: None,
                        },
                        GFP_KERNEL,
                    )?;
                    continue;
                }
                let phys = pte & oas_mask & (!UAT_PGMSK as u64);
                // SAFETY: GPU pages are either firmware/preallocated pages
                // (which the kernel isn't concerned with and are either in
                // the page map or not, and if they aren't, borrow_phys()
                // will fail), or GPU page table pages (which we own),
                // or GEM buffer pages (which are locked while they are
                // mapped in the page table), so they should be safe to
                // borrow.
                //
                // This does trust the firmware not to have any weird
                // mappings in its own internal page tables, but since
                // those are managed by the uPPL which is privileged anyway,
                // this trust does not actually extend any trust boundary.
                let src_page = match unsafe { Page::borrow_phys(&phys) } {
                    Some(page) => page,
                    None => {
                        pages.push(
                            DumpedPage {
                                iova: iova + (idx * UAT_PGSZ) as u64,
                                pte,
                                data: None,
                            },
                            GFP_KERNEL,
                        )?;
                        continue;
                    }
                };
                let dst_page = Page::alloc_page(GFP_KERNEL)?;
                src_page.with_page_mapped(|psrc| -> Result {
                    // SAFETY: This could technically still have a data race with the firmware
                    // or other driver code (or even userspace with timestamp buffers), but while
                    // the Rust language technically says this is UB, in the real world, using
                    // atomic reads for this is guaranteed to never cause any harmful effects
                    // other than possibly reading torn/unreliable data. At least on ARM64 anyway.
                    //
                    // (Yes, I checked with Rust people about this. ~~ Lina)
                    //
                    let src_items = unsafe {
                        core::slice::from_raw_parts(
                            psrc as *const AtomicU64,
                            UAT_PGSZ / core::mem::size_of::<AtomicU64>(),
                        )
                    };
                    dst_page.with_page_mapped(|pdst| -> Result {
                        // SAFETY: We own the destination page, so it is safe to view its contents
                        // as a u64 slice.
                        let dst_items = unsafe {
                            core::slice::from_raw_parts_mut(
                                pdst as *mut u64,
                                UAT_PGSZ / core::mem::size_of::<u64>(),
                            )
                        };
                        for (si, di) in src_items.iter().zip(dst_items.iter_mut()) {
                            *di = si.load(Ordering::Relaxed);
                        }
                        Ok(())
                    })?;
                    Ok(())
                })?;
                pages.push(
                    DumpedPage {
                        iova: iova + (idx * UAT_PGSZ) as u64,
                        pte,
                        data: Some(dst_page),
                    },
                    GFP_KERNEL,
                )?;
            }
            Ok(())
        })?;
        Ok(pages)
    }

}

impl Drop for UatPageTable {
    fn drop(&mut self) {
        if self.quarantined {core::mem::forget(self.dma_tables.take());return;}
        mod_pr_debug!("UATPageTable::drop range: {:#x?}\n", &self.va_range);
        if self
            .with_pages(self.va_range.clone(), false, true, false, |iova, ptes| {
                for (idx, pte) in ptes.iter().enumerate() {
                    if pte.load(Ordering::Relaxed) != 0 {
                        pr_err!(
                            "UATPageTable::drop: Leaked page at IOVA {:#x}\n",
                            iova + (idx * UAT_PGSZ) as u64
                        );
                    }
                }
                Ok(())
            })
            .is_err()
        {
            pr_err!("UATPageTable::drop failed to free page tables\n",);
        }
        if self.ttb_owned && self.dma_tables.is_none() {
            mod_pr_debug!("UATPageTable::drop: Free TTB {:#x}\n", self.ttb);
            // SAFETY: If we own the ttb, it was allocated with Page::into_phys().
            unsafe {
                Page::from_phys(self.ttb);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gpu_permissions_match_ro_wo_and_rw_pte_encodings() {
        assert!(PROT_GPU_RO.allows_gpu(true, false));
        assert!(!PROT_GPU_RO.allows_gpu(false, true));
        assert!(!PROT_GPU_RO.allows_gpu(true, true));

        assert!(!PROT_GPU_WO.allows_gpu(true, false));
        assert!(PROT_GPU_WO.allows_gpu(false, true));
        assert!(!PROT_GPU_WO.allows_gpu(true, true));

        assert!(PROT_GPU_RW.allows_gpu(true, false));
        assert!(PROT_GPU_RW.allows_gpu(false, true));
        assert!(PROT_GPU_RW.allows_gpu(true, true));

        assert!(!PROT_FW_RW.allows_gpu(true, false));
        assert!(!PROT_FW_RW.allows_gpu(false, true));
        assert!(!Prot::from_pte(0).allows_gpu(true, false));
    }

    #[test]
    fn skipped_page_table_hole_never_counts_as_coverage() {
        assert!(complete_coverage(3, 3, true));
        assert!(!complete_coverage(3, 2, true));
        assert!(!complete_coverage(3, 3, false));
    }
}
