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
    prelude::*,
    sync::Arc, //
};

use crate::debug::*;
use crate::util::align;

const DEBUG_CLASS: DebugFlags = DebugFlags::PgTable;

/// Number of bits in a page offset.
pub(crate) const UAT_PGBIT: usize = 14;
/// UAT page size.
pub(crate) const UAT_PGSZ: usize = 1 << UAT_PGBIT;
/// UAT page offset mask.
pub(crate) const UAT_PGMSK: usize = UAT_PGSZ - 1;

type Pte = AtomicU64;

const PTE_BIT: usize = 3; // log2(sizeof(Pte))
const PTE_SIZE: usize = 1 << PTE_BIT;

/// Number of PTEs per page.
const UAT_NPTE: usize = UAT_PGSZ / size_of::<Pte>();

/// Number of address bits to address a level
const UAT_LVBIT: usize = UAT_PGBIT - PTE_BIT;
/// Number of entries per level
const UAT_LVSZ: usize = UAT_NPTE;
/// Mask of level bits
const UAT_LVMSK: u64 = (UAT_LVSZ - 1) as u64;

const UAT_LEVELS: usize = 3;

/// Number of address bits below the index of a top-level page table entry.
pub(crate) const UAT_TOP_LEVEL_SHIFT: usize = UAT_PGBIT + (UAT_LEVELS - 1) * UAT_LVBIT;

/// The system-wide minimum CPU line size, including heterogeneous CPUs. A local
/// CTR_EL0 read can select too large a stride if the task migrates during the loop.
#[inline(always)]
fn dcache_line_size() -> usize {
    use kernel::bindings;
    let ctr = (bindings::SYS_CTR_EL0_Op0 << 19)
        | (bindings::SYS_CTR_EL0_Op1 << 16)
        | (bindings::SYS_CTR_EL0_CRn << 12)
        | (bindings::SYS_CTR_EL0_CRm << 8)
        | (bindings::SYS_CTR_EL0_Op2 << 5);
    // SAFETY: Read-only lookup of the arm64 sanitized feature register. DminLine
    // uses the architecture's LOWER_SAFE merge policy across all admitted CPUs.
    let value = unsafe { bindings::read_sanitised_ftr_reg(ctr) };
    4 << ((value >> bindings::CTR_EL0_DminLine_SHIFT) & 0xf)
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

/// Completes earlier point-of-coherency maintenance for the whole system (`dsb sy`).
pub(crate) fn poc_barrier() {
    // SAFETY: Barriers are always safe.
    unsafe { core::arch::asm!("dsb sy", options(nostack, preserves_flags)) };
}

/// Page tables in bootloader-reserved memory that the kernel does not map ("no-map"), reached
/// through a write-back mapping instead of `struct page` (the `pagetables` and `l2-pagetables`
/// regions, where the firmware builds its root and kernel-window tables).
pub(crate) struct ReservedTables {
    regions: KVec<kernel::io::mem::Mem>,
    starts: KVec<PhysicalAddr>,
}

// SAFETY: The mappings are plain write-back memory owned for the lifetime of this object; table
// entries are only accessed as atomics.
unsafe impl Send for ReservedTables {}
// SAFETY: As above.
unsafe impl Sync for ReservedTables {}

impl ReservedTables {
    pub(crate) fn new() -> Self {
        Self {
            regions: KVec::new(),
            starts: KVec::new(),
        }
    }

    /// Adds a reserved region, mapped write-back.
    ///
    /// # Safety
    ///
    /// `res` must be a bootloader-reserved region outside System RAM that stays reserved.
    pub(crate) unsafe fn add(&mut self, res: kernel::io::resource::Resource) -> Result {
        let start = res.start();
        // SAFETY: The caller guarantees the region; write-back matches how the firmware maps it.
        let map = unsafe {
            kernel::io::mem::Mem::try_new(res, kernel::io::mem::MemFlag::WB.into())?
        };
        self.starts.push(start, GFP_KERNEL)?;
        self.regions.push(map, GFP_KERNEL)?;
        Ok(())
    }

    /// The CPU address of a page-table page in one of the regions.
    fn lookup(&self, phys: PhysicalAddr) -> Option<*mut u8> {
        for (start, map) in self.starts.iter().zip(self.regions.iter()) {
            let Some(offset) = phys.checked_sub(*start) else {
                continue;
            };
            let offset = offset as usize;
            if offset + UAT_PGSZ <= map.size() {
                // SAFETY: The offset and one page lie inside the mapping.
                return Some(unsafe { map.ptr().add(offset) });
            }
        }
        None
    }

    /// Whether `phys` is a page of one of the regions.
    pub(crate) fn contains(&self, phys: PhysicalAddr) -> bool {
        self.lookup(phys).is_some()
    }

    /// Drops the CPU's cached lines of every region, so that later reads see what the firmware
    /// wrote while booting. Lines the host dirtied are written back first, never discarded.
    pub(crate) fn invalidate(&self) {
        for map in self.regions.iter() {
            clean_invalidate_poc(map.ptr(), map.size());
        }
        poc_barrier();
    }
}

/// Runs `f` on a pointer to `len` bytes at `offset` in the page-table page at `phys`, through the
/// reserved mapping when the page lies in a reserved region, else through `struct page`.
///
/// # Safety
///
/// `phys` must be a page-table page that stays allocated while `f` runs.
unsafe fn with_table<T>(
    reserved: Option<&ReservedTables>,
    phys: PhysicalAddr,
    offset: usize,
    len: usize,
    f: impl FnOnce(*mut u8) -> Result<T>,
) -> Result<T> {
    if let Some(base) = reserved.and_then(|r| r.lookup(phys)) {
        if offset.checked_add(len).ok_or(EINVAL)? > UAT_PGSZ {
            return Err(EINVAL);
        }
        // SAFETY: lookup() checked that the whole page is mapped.
        return f(unsafe { base.add(offset) });
    }
    // SAFETY: The caller guarantees an allocated page-table page in RAM.
    let page = unsafe { Page::borrow_phys_unchecked(&phys) };
    page.with_pointer_into_page(offset, len, f)
}

/// Runs `f` on the first `count` top-level entries of the root page table at `ttb`.
///
/// # Safety
///
/// `ttb` must be the physical address of a page table root, in RAM or in one of the `reserved`
/// regions, that stays allocated while `f` runs. Other agents (the GPU firmware) may access the
/// entries concurrently, which is why they are only exposed as atomics.
pub(crate) unsafe fn with_root_entries<R>(
    reserved: Option<&ReservedTables>,
    ttb: PhysicalAddr,
    count: usize,
    f: impl FnOnce(&[AtomicU64]) -> R,
) -> Result<R> {
    if count > UAT_NPTE {
        return Err(EINVAL);
    }
    // SAFETY: A root outside the reserved regions must have a struct page.
    if !reserved.is_some_and(|r| r.contains(ttb)) && unsafe { Page::borrow_phys(&ttb) }.is_none() {
        return Err(EIO);
    }
    // SAFETY: The caller guarantees an allocated root page, in RAM or in a reserved region.
    unsafe {
        with_table(reserved, ttb, 0, count * PTE_SIZE, |p| {
            // SAFETY: with_table() checked that `count` entries are mapped, and page table entries
            // are naturally aligned.
            Ok(f(core::slice::from_raw_parts(p as *const AtomicU64, count)))
        })
    }
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

pub(crate) mod prot {
    pub(crate) use super::Prot;
    use super::*;

    /// Firmware MMIO R/W
    pub(crate) const PROT_FW_MMIO_RW: Prot = PROT_FW_RW.memattr(MEMATTR_DEV);
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

    pub(crate) const fn from_pte(pte: u64) -> Self {
        Prot {
            high_bits: (pte >> UAT_HIGH_BITS_SHIFT) as u16,
            ap: ((pte & UAT_AP_BITS) >> UAT_AP_SHIFT) as u8,
            memattr: ((pte & UAT_MEMATTR_BITS) >> UAT_MEMATTR_SHIFT) as u8,
        }
    }

    const fn memattr(&self, memattr: u8) -> Self {
        Self { memattr, ..*self }
    }

    /// The protection bits of a leaf with this protection (no address, table type or non-global
    /// bits), for checking that a firmware leaf word round-trips through [`Self::from_pte`].
    #[cfg(CONFIG_DRM_ASAHI_G15)]
    pub(crate) const fn leaf_bits(&self) -> u64 {
        self.as_pte()
    }

    const fn as_pte(&self) -> u64 {
        (self.ap as u64) << UAT_AP_SHIFT
            | (self.high_bits as u64) << UAT_HIGH_BITS_SHIFT
            | (self.memattr as u64) << UAT_MEMATTR_SHIFT
            | UAT_AF
    }

    /// Whether this leaf allows the requested GPU accesses.
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

    /// The native form of a GPU-only permission: AP 0 with the access in PXN (read) and UXN
    /// (write), as [`Self::allows_gpu`] reads AP 0. The logical selectors RO, WO, RW, NA map to
    /// the native RO, WO, RW, NA. Firmware permissions are returned unchanged.
    const fn native_gpu(self) -> Self {
        if self.ap != AP_GPU {
            return self;
        }
        let selector = (self.high_bits >> 1) & 3;
        Self {
            ap: AP_FW_GPU,
            high_bits: (self.high_bits & !(HIGH_BITS_PXN | HIGH_BITS_UXN))
                | (((selector + 1) & 3) << 1),
            ..self
        }
    }

    pub(crate) const fn is_cached_noncoherent(&self) -> bool {
        self.ap != AP_GPU && self.memattr == MEMATTR_CACHED
    }

    pub(crate) const fn as_uncached(&self) -> Self {
        self.memattr(MEMATTR_UNCACHED)
    }
}

impl Default for Prot {
    fn default() -> Self {
        PROT_FW_GPU_NA
    }
}

pub(crate) struct UatPageTable {
    ttb: PhysicalAddr,
    ttb_owned: bool,
    /// Ownership could not be relinquished to firmware; physical tables remain until reboot.
    retained: bool,
    /// Reserved regions holding tables this page table may reach.
    reserved: Option<Arc<ReservedTables>>,
    /// Store GPU-only permissions in the native AP 0 form ([`Prot::native_gpu`]).
    native_gpu: bool,
    /// Tables below a root the driver did not allocate may belong to the bootloader or the
    /// firmware, so only the tables recorded here are ever freed. `None` if the driver owns the
    /// tables below the root, as with the handoff firmware interface.
    driver_tables: Option<KVec<PhysicalAddr>>,
    va_range: Range<u64>,
    ias_mask: u64,
    oas_mask: u64,
    /// Clean new tables and changed entries to the point of coherency, for a UAT walker that does
    /// not snoop the CPU caches ([`Self::enable_clean_poc`]).
    clean_poc: bool,
}

impl UatPageTable {
    pub(crate) fn new(ias: u32, oas: u32) -> Result<Self> {
        mod_pr_debug!("UATPageTable::new: ias={} oas={}\n", ias, oas);
        let ttb_page = Page::alloc_page(GFP_KERNEL | __GFP_ZERO)?;
        let ttb = Page::into_phys(ttb_page);
        Ok(UatPageTable {
            ttb,
            ttb_owned: true,
            retained: false,
            reserved: None,
            native_gpu: false,
            driver_tables: None,
            va_range: 0..(1u64 << ias),
            ias_mask: (1u64 << ias) - 1,
            oas_mask: (1u64 << oas) - 1,
            clean_poc: false,
        })
    }

    /// Makes every later change of this page table visible to a UAT walker that does not snoop the
    /// CPU caches (G16, and `UatQuirks::clean_tables`), and cleans the root, which may have been
    /// zeroed through the cacheable linear map.
    ///
    /// From now on, a new table page is cleaned before its parent descriptor names it, a changed
    /// table or leaf descriptor is cleaned after the store, and every mutating walk ends with a
    /// `dsb sy`. Call it before anything is mapped and before any context names the root.
    pub(crate) fn enable_clean_poc(&mut self) -> Result {
        self.clean_poc = true;
        // SAFETY: The root is owned or borrowed by this page table for its lifetime, in RAM or in
        // a reserved region.
        unsafe {
            with_table(self.reserved.as_deref(), self.ttb, 0, UAT_PGSZ, |p| {
                clean_poc(p, UAT_PGSZ);
                Ok(())
            })
        }?;
        poc_barrier();
        Ok(())
    }

    /// Whether changes are cleaned to the point of coherency.
    #[inline(always)]
    fn cleans_poc(&self) -> bool {
        self.clean_poc
    }

    /// Stores GPU-only permissions of this (client or kernel-lower) table in the native AP 0 form
    /// (`UatQuirks::native_gpu_permissions`). Firmware roots never hold GPU-only mappings.
    pub(crate) fn with_native_gpu(mut self, native_gpu: bool) -> Self {
        self.native_gpu = native_gpu;
        self
    }

    /// Borrow a root; track individual child ownership only when other agents
    /// can install tables below it. Handoff roots have driver-owned children.
    pub(crate) fn new_with_ttb(
        ttb: PhysicalAddr,
        va_range: Range<u64>,
        ias: u32,
        oas: u32,
        shared_tables: bool,
        reserved: Option<Arc<ReservedTables>>,
    ) -> Result<Self> {
        mod_pr_debug!(
            "UATPageTable::new_with_ttb: ttb={:#x} range={:#x?} ias={} oas={}\n",
            ttb,
            va_range,
            ias,
            oas
        );
        if ttb & (UAT_PGMSK as PhysicalAddr) != 0 {
            return Err(EINVAL);
        }
        if (va_range.start | va_range.end) & (UAT_PGMSK as u64) != 0 {
            return Err(EINVAL);
        }
        // SAFETY: The bootloader-managed root stays valid while this page table exists.
        let reachable = reserved.as_ref().is_some_and(|r| r.contains(ttb))
            || unsafe { Page::borrow_phys(&ttb) }.is_some();
        if !reachable {
            pr_err!(
                "UATPageTable::new_with_ttb: ttb at {:#x} is not mapped (DT using no-map?)\n",
                ttb
            );
            return Err(EIO);
        }

        Ok(UatPageTable {
            ttb,
            ttb_owned: false,
            retained: false,
            reserved,
            native_gpu: false,
            driver_tables: shared_tables.then(KVec::new),
            va_range,
            ias_mask: (1u64 << ias) - 1,
            oas_mask: (1u64 << oas) - 1,
            clean_poc: false,
        })
    }

    /// Retains every physical table after an uncertain M4 handoff. No other agent may reclaim
    /// this tree merely because a userspace handle closed or a fence reported an error.
    pub(crate) fn retain(&mut self) {
        self.retained = true;
    }

    pub(crate) fn ttb(&self) -> PhysicalAddr {
        self.ttb
    }

    /// Walks the leaves of `iova_range` without changing them.
    fn with_pages<F>(&mut self, iova_range: Range<u64>, alloc: bool, free: bool, cb: F) -> Result
    where
        F: FnMut(u64, &[Pte]) -> Result,
    {
        self.walk(iova_range, alloc, free, false, cb)
    }

    /// Walks the leaves of `iova_range`, which `cb` may change (`mutate`).
    ///
    /// With point-of-coherency cleaning enabled, the changed leaves are cleaned after `cb`, and a
    /// walk that may have changed any descriptor ends with a barrier, also on failure.
    fn walk<F>(
        &mut self,
        iova_range: Range<u64>,
        alloc: bool,
        free: bool,
        mutate: bool,
        cb: F,
    ) -> Result
    where
        F: FnMut(u64, &[Pte]) -> Result,
    {
        let result = self.walk_inner(iova_range, alloc, free, mutate, cb);
        if self.cleans_poc() && (alloc || free || mutate) {
            poc_barrier();
        }
        result
    }

    fn walk_inner<F>(
        &mut self,
        iova_range: Range<u64>,
        alloc: bool,
        free: bool,
        mutate: bool,
        mut cb: F,
    ) -> Result
    where
        F: FnMut(u64, &[Pte]) -> Result,
    {
        let clean = self.cleans_poc();
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

        let mut iova = iova_range.start & self.ias_mask;
        let mut last_iova = iova;
        // Handle the case where iova_range.end is just at the top boundary of the IAS
        let end = ((iova_range.end - 1) & self.ias_mask) + 1;

        let reserved = self.reserved.clone();
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
                            self.free_table(phys, parent)?;
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
                    mod_pr_debug!("UATPageTable::with_pages: borrowed phys {:#x}\n", phys);
                    let driver_tables = &mut self.driver_tables;
                    let oas_mask = self.oas_mask;
                    let entry = |p: *mut u8| -> Result<Option<PhysicalAddr>> {
                        let uptep = p as *const _ as *const Pte;
                        // SAFETY: with_table() ensures the pointer is valid, and our index is
                        // aligned so it is safe to deref as an AtomicU64.
                        let upte = unsafe { &*uptep };
                        let mut upte_val = upte.load(Ordering::Relaxed);
                        // Allocate if requested
                        if upte_val == 0 && alloc {
                            let pt_page = Page::alloc_page(GFP_KERNEL | __GFP_ZERO)?;
                            mod_pr_debug!("UATPageTable::with_pages: alloc PT at {:#x}\n", pt_page.phys());
                            if clean {
                                // A non-snooping walker must never follow a parent to a
                                // table whose zeroes are still in the CPU caches.
                                pt_page.with_pointer_into_page(0, UAT_PGSZ, |p| {
                                    clean_poc(p, UAT_PGSZ);
                                    Ok(())
                                })?;
                            }
                            if let Some(tables) = driver_tables.as_mut() {
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
                            if clean {
                                clean_poc(uptep.cast(), PTE_SIZE);
                            }
                        }
                        if upte_val & PTE_TYPE_BITS == PTE_TYPE_LEAF_TABLE {
                            Ok(Some(upte_val & oas_mask & (!UAT_PGMSK as u64)))
                        } else if upte_val == 0 || (!alloc && !free) {
                            mod_pr_debug!("UATPageTable::with_pages: no level {}\n", level);
                            Ok(None)
                        } else {
                            pr_err!("UATPageTable::with_pages: Unexpected Table PTE value {:#x} at iova {:#x} index {} phys {:#x}\n", upte_val,
                                    iova, level + 1, phys + ((upidx * PTE_SIZE) as PhysicalAddr));
                            Ok(None)
                        }
                    };
                    // SAFETY: Page table addresses are either allocated by us, or
                    // firmware-managed tables in RAM or in a reserved region.
                    pt_addr[level] = unsafe {
                        with_table(reserved.as_deref(), phys, upidx * PTE_SIZE, PTE_SIZE, entry)
                    }?;
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
            let leaves = |p: *mut u8| -> Result {
                let ptep = p as *const _ as *const Pte;
                // SAFETY: We know this is a valid pointer to PTEs and the range is valid and
                // checked by with_table().
                let ptes = unsafe { core::slice::from_raw_parts(ptep, count) };
                let result = cb(iova, ptes);
                if clean && mutate {
                    // Also after a partial change: the callback may have stored some leaves.
                    clean_poc(p, count * PTE_SIZE);
                }
                result
            };
            // SAFETY: Page table addresses are either allocated by us, or
            // firmware-managed tables in RAM or in a reserved region.
            unsafe {
                with_table(reserved.as_deref(), phys, idx * PTE_SIZE, count * PTE_SIZE, leaves)
            }?;

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
                    self.free_table(phys, pt_parent[level].take())?;
                }
            }
        }

        Ok(())
    }

    /// Free a page table, unless it was not allocated by the driver.
    fn free_table(&mut self, phys: PhysicalAddr, parent: Option<(PhysicalAddr, usize)>) -> Result {
        let owned = if let Some(tables) = self.driver_tables.as_ref() {
            tables.contains(&phys)
        } else {
            true
        };
        if !owned {
            return Ok(());
        }
        if let Some((parent_phys, parent_index)) = parent {
            let oas_mask = self.oas_mask;
            let in_reserved = self.reserved.as_ref().is_some_and(|r| r.contains(parent_phys));
            // SAFETY: Only looks the page up; nothing is accessed through it.
            if !in_reserved && unsafe { Page::borrow_phys(&parent_phys) }.is_none() {
                return Err(EIO);
            }
            let reserved = self.reserved.as_deref();
            let cleans = self.cleans_poc();
            let detach = |p: *mut u8| -> Result {
                // SAFETY: The live page-table page contains aligned u64 PTEs, and with_table()
                // bounds this access to one PTE.
                let pte = unsafe { &*(p as *const AtomicU64) };
                let current = pte.load(Ordering::Acquire);
                let expected = phys & oas_mask & (!UAT_PGMSK as u64);
                if current & oas_mask & (!UAT_PGMSK as u64) != expected {
                    return Err(EIO);
                }
                pte.store(0, Ordering::Release);
                if cleans {
                    clean_poc(p, PTE_SIZE);
                }
                Ok(())
            };
            // SAFETY: The bottom-up traversal keeps the parent allocated while
            // its child descriptor is detached. A changed descriptor indicates
            // an ownership violation; fail closed instead of freeing a table
            // that may still be reachable.
            unsafe { with_table(reserved, parent_phys, parent_index * PTE_SIZE, PTE_SIZE, detach) }?;
        }
        if let Some(tables) = self.driver_tables.as_mut() {
            let index = tables.iter().position(|&table| table == phys).ok_or(EIO)?;
            tables.swap_remove(index);
        }
        if self.cleans_poc() {
            // Complete the parent descriptor clear before its detached child can be reused.
            // Callers have already removed this tree from live contexts before freeing tables.
            poc_barrier();
        }
        // SAFETY: Page tables allocated by the driver always come from Page::into_phys(). Without
        // `driver_tables`, every child table belongs to the driver.
        unsafe { Page::from_phys(phys) };
        Ok(())
    }

    /// Checks every page, including holes skipped by the page-table walker.
    pub(crate) fn covers_range(
        &mut self,
        range: Range<u64>,
        read: bool,
        write: bool,
    ) -> Result<bool> {
        if range.is_empty() || (range.start | range.end) & UAT_PGMSK as u64 != 0 {
            return Ok(false);
        }
        let expected = (range.end - range.start) >> UAT_PGBIT;
        let mut visited = 0u64;
        let mut permitted = true;
        self.with_pages(range, false, false, |_, ptes| {
            visited = visited.checked_add(ptes.len() as u64).ok_or(EOVERFLOW)?;
            for pte in ptes {
                let value = pte.load(Ordering::Acquire);
                if value & PTE_TYPE_BITS != PTE_TYPE_LEAF_TABLE
                    || !Prot::from_pte(value).allows_gpu(read, write)
                {
                    permitted = false;
                }
            }
            Ok(())
        })?;
        Ok(expected == visited && permitted)
    }

    /// Resolves an existing leaf without creating page tables.
    pub(crate) fn translate_iova(&mut self, iova: u64) -> Result<u64> {
        let entry = self.leaf_entry(iova & !(UAT_PGMSK as u64))?;
        Ok((entry & self.oas_mask & !(UAT_PGMSK as u64)) | (iova & UAT_PGMSK as u64))
    }

    /// Reads one mapped leaf entry without allocating missing tables.
    pub(crate) fn leaf_entry(&mut self, iova: u64) -> Result<u64> {
        if iova & UAT_PGMSK as u64 != 0 {
            return Err(EINVAL);
        }
        let end = iova.checked_add(UAT_PGSZ as u64).ok_or(EINVAL)?;
        let mut value = 0;
        self.with_pages(iova..end, false, false, |_, ptes| {
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

    pub(crate) fn alloc_pages(&mut self, iova_range: Range<u64>) -> Result {
        mod_pr_debug!("UATPageTable::alloc_pages: {:#x?}\n", iova_range);
        self.with_pages(iova_range, true, false, |_, _| Ok(()))
    }

    /// Reserve all tables for a fixed mapping before publishing any leaves.
    /// GPUVM and the fixed-address allocator may share a page table, so check
    /// that neither owns a destination leaf before allowing failure rollback.
    pub(crate) fn prepare_map(&mut self, range: Range<u64>) -> Result {
        if range.is_empty() || (range.start | range.end) & UAT_PGMSK as u64 != 0 {
            return Err(EINVAL);
        }
        self.with_pages(range.clone(), false, false, |_, ptes| {
            if ptes.iter().any(|pte| pte.load(Ordering::Acquire) != 0) {
                return Err(EBUSY);
            }
            Ok(())
        })?;
        self.alloc_pages(range)
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

    /// The permission bits as this table stores them.
    fn stored(&self, prot: Prot) -> Prot {
        if self.native_gpu {
            prot.native_gpu()
        } else {
            prot
        }
    }

    pub(crate) fn map_pages(
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
        let prot = self.stored(prot);

        self.walk(iova_range, true, false, true, |iova, ptes| {
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

    pub(crate) fn reprot_pages(&mut self, iova_range: Range<u64>, prot: Prot) -> Result {
        mod_pr_debug!(
            "UATPageTable::reprot_pages: {:#x?} {:?}\n",
            iova_range,
            prot
        );
        let prot = self.stored(prot);
        self.walk(iova_range, true, false, true, |iova, ptes| {
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

    pub(crate) fn unmap_pages(&mut self, iova_range: Range<u64>) -> Result {
        mod_pr_debug!("UATPageTable::unmap_pages: {:#x?}\n", iova_range);
        self.walk(iova_range, false, false, true, |iova, ptes| {
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

    /// Discards an aligned map operation that failed before acquiring a GPUVA owner.
    /// Missing tables and leaves are expected; existing table storage stays allocated.
    pub(crate) fn discard_partial_map(&mut self, iova_range: Range<u64>) -> Result {
        self.walk(iova_range, false, false, true, |_, ptes| {
            for pte in ptes {
                pte.store(0, Ordering::Relaxed);
            }
            Ok(())
        })
    }
}

impl Drop for UatPageTable {
    fn drop(&mut self) {
        if self.retained {
            return;
        }
        mod_pr_debug!("UATPageTable::drop range: {:#x?}\n", &self.va_range);
        if self
            .with_pages(self.va_range.clone(), false, true, |iova, ptes| {
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
        if self.ttb_owned {
            mod_pr_debug!("UATPageTable::drop: Free TTB {:#x}\n", self.ttb);
            // SAFETY: If we own the ttb, it was allocated with Page::into_phys().
            unsafe {
                Page::from_phys(self.ttb);
            }
        }
    }
}
