// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! The adopted UAT: PPL handoff, TTBAT context 0, the host window and TLB maintenance.
//!
//! The firmware's kernel root (the TABLES region, ADT `gpu-region`) and the shared L2 page are
//! not System RAM, so the shared page-table code cannot walk them. The design:
//!
//! 1. Map handoff, TTBAT, kernel root page and shared L2 (write-combined carveouts, or the
//!    kernel's mapping of reserved System RAM, [`Mapping::region`]). PPL exchange: write the
//!    AP magic, `cur_slot = 0xffffffff`, zero the 65 flush entries and `+0x640`, then poll the
//!    firmware magic (1 s).
//! 2. `mmu::Uat::new_adopted` with `kernel_root = TABLES`: user VMs and the kernel lower VM get
//!    driver-owned RAM roots; `kernel_vm` manages the host window under a RAM shadow root.
//! 3. Under the PPL lock: zero root entries 2..63 of the firmware kernel root and all of the
//!    shared L2, TLBI all, root[2] = shared L2 | 3, zero TTBAT 2..127, TTBAT context 0 =
//!    (kernel lower root | 1, TABLES | 1). Root entries 0..1 belong to the firmware.
//! 4. [`Window::sync_range`]: after any mapping in `kernel_vm`, copy the shadow root's window L2
//!    entries (they name RAM L3 tables) into the shared L2, before the firmware uses the new
//!    addresses. Tables are never freed on unmap.
//!
//! Point of coherency: the tables are RAM pages the shared code writes through the cacheable
//! linear map, and the UAT walker does not snoop the CPU caches, so every table the firmware can
//! walk is cleaned to the point of coherency before it becomes reachable, and changed leaves are
//! cleaned before the new addresses are published: [`Window::sync_range`] for the host window,
//! [`clean_lower`] for the context-0 lower tree. (The cleaning in `pgtable.rs` makes these
//! redundant but harmless.)
//!
//! User VMs are bound to exclusive TTBAT slots for their lifetime through `mmu::Uat::bind`
//! (`vm.rs`); the shared layer writes TTBAT `(root | 1 | slot << 48, TABLES | 1 |
//! slot << 48)` under the PPL lock and invalidates the slot's ASID.

use core::ops::Range;
use core::sync::atomic::Ordering;

use kernel::{
    bindings, device,
    io::Io,
    prelude::*,
    sync::Arc,
    time::{delay::fsleep, Delta, Instant, Monotonic},
};

use super::hw;
use super::regs::{mb, rmb, wmb, Mapping};
use crate::driver::AsahiDevice;
use crate::{mmu, pgtable};

/// Entries of a UAT table page.
const ENTRIES: usize = 2048;
/// Entries of the top-level table used with a 42-bit input range.
const ROOT_ENTRIES: usize = 64;
/// Input address mask of a 42-bit UAT.
const IAS_MASK: u64 = (1 << 42) - 1;
/// Valid table/leaf descriptor bits.
const VALID: u64 = 3;
/// Root entry of the host window (2).
const WINDOW_INDEX: usize = ((hw::HOST & IAS_MASK) >> 36) as usize;

/// Root, L2 and L3 indices of `va`.
const fn indices(va: u64) -> [usize; 3] {
    let va = va & IAS_MASK;
    [
        ((va >> 36) & 63) as usize,
        ((va >> 25) & 2047) as usize,
        ((va >> 14) & 2047) as usize,
    ]
}

/// Creates the adopted UAT over the ADT regions (step 2 above).
pub(crate) fn adopt(
    dev: &AsahiDevice,
    cfg: &'static super::Config,
    adt: &super::adt::GpuAdt,
) -> Result<Arc<mmu::Uat>> {
    let geometry = mmu::UatGeometry {
        ias: cfg.uat_ias,
        oas: cfg.uat_oas,
    };
    let regions = mmu::AdoptedRegions {
        handoff: adt.handoff.clone(),
        ttbs: adt.ttbs.clone(),
        kernel_root: adt.tables.start,
    };
    Ok(Arc::new(
        mmu::Uat::new_adopted(dev, geometry, &regions)?,
        GFP_KERNEL,
    )?)
}

/// Runs `f` on the entries of the RAM table page at `pa`.
fn with_table<R>(
    pa: u64,
    count: usize,
    f: impl FnOnce(&[core::sync::atomic::AtomicU64]) -> R,
) -> Result<R> {
    // SAFETY: Every table walked here is a page the shared page-table code allocated in RAM
    // (`Page::alloc_page`) and never frees while the firmware runs (tables are not freed on
    // unmap, and these VMs are retained until reboot).
    unsafe { pgtable::with_root_entries(pa, count, f) }
}

/// Cleans `len` bytes of the RAM table page at `pa`, from entry `first`, to the point of
/// coherency.
fn clean_entries(pa: u64, first: usize, count: usize) -> Result {
    if first + count > ENTRIES {
        return Err(EINVAL);
    }
    with_table(pa, ENTRIES, |entries| {
        let start = entries[first..].as_ptr() as usize;
        // SAFETY: `start..start + count * 8` lies inside the borrowed table page; cleaning a
        // cacheable range has no other effect.
        unsafe {
            bindings::dcache_clean_poc(start as _, (start + count * 8) as _);
        }
    })
}

/// Cleans a whole RAM table page to the point of coherency.
fn clean_page(pa: u64) -> Result {
    clean_entries(pa, 0, ENTRIES)
}

/// Reads entry `index` of the RAM table page at `pa`.
fn entry(pa: u64, index: usize) -> Result<u64> {
    with_table(pa, index + 1, |entries| {
        entries[index].load(Ordering::Acquire)
    })
}

/// Reads the leaf word that translates `va` under the RAM root `root`, or `None` if a level is
/// missing.
pub(crate) fn read_leaf(root: u64, va: u64) -> Result<Option<u64>> {
    let [i0, i1, i2] = indices(va);
    let l1 = entry(root, i0)?;
    if l1 & VALID != VALID {
        return Ok(None);
    }
    let l2 = entry(l1 & hw::PA_MASK, i1)?;
    if l2 & VALID != VALID {
        return Ok(None);
    }
    Ok(Some(entry(l2 & hw::PA_MASK, i2)?))
}

/// Cleans every table of the context-0 lower tree (the kernel lower VM) to the point of
/// coherency: its root, then each second-level and leaf table. Called before TTBAT context 0
/// names the root and after every lower mapping.
pub(crate) fn clean_lower(uat: &mmu::Uat) -> Result {
    let root = uat.kernel_lower_root();
    let mut tables = KVec::new();
    for i in 0..ROOT_ENTRIES {
        let e = entry(root, i)?;
        if e & VALID == VALID {
            tables.push(e & hw::PA_MASK, GFP_KERNEL)?;
        }
    }
    let mut leaves = KVec::new();
    for &l2 in tables.iter() {
        with_table(l2, ENTRIES, |entries| -> Result {
            for e in entries {
                let e = e.load(Ordering::Acquire);
                if e & VALID == VALID {
                    leaves.push(e & hw::PA_MASK, GFP_KERNEL)?;
                }
            }
            Ok(())
        })??;
    }
    // Leaves first, then the tables naming them, then the root.
    for &pa in Iterator::chain(leaves.iter(), tables.iter()) {
        clean_page(pa)?;
    }
    clean_page(root)?;
    mb();
    Ok(())
}

/// Invalidates every TLB entry of the GPU UAT (`TLBI VMALLE1OS`).
pub(crate) fn tlbi_all() {
    crate::mem::sync();
    crate::mem::tlbi_all();
    crate::mem::sync();
}

/// The firmware's view of the host window (root entry 2 -> shared L2) and the PPL handoff.
pub(crate) struct Window {
    /// PPL handoff page.
    handoff: Mapping,
    /// TTBAT page.
    ttbs: Mapping,
    /// First page of the firmware kernel root tables.
    root: Mapping,
    /// The shared L2 page.
    shared_l2: Mapping,
    /// The shared L2 entries last written, and a snapshot buffer of the shadow L2.
    image: KVec<u64>,
    snapshot: KVec<u64>,
}

impl Window {
    /// Maps the UAT handoff regions ([`Mapping::region`]).
    pub(crate) fn new(adt: &super::adt::GpuAdt) -> Result<Self> {
        Ok(Self {
            handoff: Mapping::region(adt.handoff.start, hw::HANDOFF_SIZE as usize)?,
            ttbs: Mapping::region(adt.ttbs.start, hw::TTBS_SIZE as usize)?,
            root: Mapping::region(adt.tables.start, hw::PAGE as usize)?,
            shared_l2: Mapping::region(adt.shared_l2.start, hw::SHARED_L2_SIZE as usize)?,
            image: KVec::from_elem(0u64, ENTRIES, GFP_KERNEL)?,
            snapshot: KVec::from_elem(0u64, ENTRIES, GFP_KERNEL)?,
        })
    }

    /// Cold-start check: all 128 TTBAT words are zero.
    pub(crate) fn ttbat_is_clear(&self) -> Result<bool> {
        let t = self.ttbs.io();
        for i in 0..128 {
            if t.try_read64(i * 8)? != 0 {
                return Ok(false);
            }
        }
        Ok(true)
    }

    /// Takes the PPL lock: AP interest, then wait while the firmware holds it, yielding the turn
    /// when the firmware asks for it; 100 ms.
    pub(crate) fn ppl_lock(&self) -> Result {
        let h = self.handoff.io();
        let start = Instant::<Monotonic>::now();
        let expired = || start.elapsed() > Delta::from_millis(100);
        h.try_write8(1, 0x10)?;
        wmb();
        while h.try_read8(0x11)? != 0 {
            if h.try_read32(0x14)? != 0 {
                h.try_write8(0, 0x10)?;
                wmb();
                while h.try_read32(0x14)? != 0 {
                    if expired() {
                        return Err(ETIMEDOUT);
                    }
                }
                h.try_write8(1, 0x10)?;
                wmb();
            }
            if expired() {
                h.try_write8(0, 0x10)?;
                return Err(ETIMEDOUT);
            }
            core::hint::spin_loop();
        }
        rmb();
        Ok(())
    }

    /// Releases the PPL lock.
    pub(crate) fn ppl_unlock(&self) -> Result {
        let h = self.handoff.io();
        wmb();
        h.try_write32(1, 0x14)?;
        h.try_write8(0, 0x10)?;
        wmb();
        Ok(())
    }

    /// The PPL exchange and the exclusive startup epoch: host window and TTBAT context 0 (step 3
    /// above).
    pub(crate) fn bootstrap(&mut self, dev: &device::Device, uat: &mmu::Uat) -> Result {
        let h = self.handoff.io();
        // AP side of the PPL exchange; never overwrite firmware interest/magic.
        h.try_write64(hw::PPL_MAGIC, 0x00)?;
        h.try_write32(0xffffffff, 0x18)?;
        for i in 0..=64 {
            for w in 0..3 {
                h.try_write64(0, 0x20 + 24 * i + 8 * w)?;
            }
        }
        h.try_write64(0, 0x640)?;
        wmb();
        // Wait up to 1 s for the firmware magic.
        let start = Instant::<Monotonic>::now();
        loop {
            let magic = h.try_read64(0x08)?;
            if magic == hw::PPL_MAGIC {
                break;
            }
            if start.elapsed() > Delta::from_millis(1000) {
                let magic = h.try_read64(0x08)?;
                if magic == hw::PPL_MAGIC {
                    break;
                }
                dev_err!(dev, "PPL firmware magic missing: {:#x}\n", magic);
                return Err(ETIMEDOUT);
            }
            fsleep(Delta::from_micros(250));
        }
        // The context-0 lower root is the kernel lower VM's RAM root, cleaned before TTBAT names
        // it.
        let lower = uat.kernel_lower_root();
        clean_lower(uat)?;
        self.ppl_lock()?;
        let result = self.epoch(lower);
        self.ppl_unlock()?;
        result?;
        self.image.fill(0);
        let r = self.root.io();
        dev_info!(
            dev,
            "MMU: initialized, root[0..2]={:#x} {:#x} {:#x}\n",
            r.try_read64(0)?,
            r.try_read64(8)?,
            r.try_read64(16)?
        );
        Ok(())
    }

    /// The writes of the startup epoch, under the PPL lock.
    fn epoch(&self, lower: u64) -> Result {
        let root = self.root.io();
        let l2 = self.shared_l2.io();
        let ttbs = self.ttbs.io();
        // Exclusive startup epoch: host entries 2.. of the upper root go first.
        for i in 2..ROOT_ENTRIES {
            root.try_write64(0, i * 8)?;
        }
        for i in 0..ENTRIES {
            l2.try_write64(0, i * 8)?;
        }
        tlbi_all();
        root.try_write64(self.shared_l2.phys() | hw::pte::TABLE, WINDOW_INDEX * 8)?;
        // TTBAT context0: owned lower root, upper root = pagetables base.
        for i in 2..128 {
            ttbs.try_write64(0, i * 8)?;
        }
        ttbs.try_write64(lower | 1, 0)?;
        ttbs.try_write64(self.root.phys() | 1, 8)?;
        Ok(())
    }

    /// The RAM L2 table of the shadow window, if anything is mapped in the window.
    fn shadow_l2(uat: &mmu::Uat) -> Result<Option<u64>> {
        let e = entry(uat.kernel_window_root(), WINDOW_INDEX)?;
        Ok((e & VALID == VALID).then_some(e & hw::PA_MASK))
    }

    /// Copies the shadow window's L2 entries into the shared L2 and cleans the leaves of `range`
    /// to the point of coherency, in leaf tables the firmware can already reach. New leaf tables
    /// are always cleaned whole before they are linked.
    pub(crate) fn sync_range(&mut self, uat: &mmu::Uat, range: Range<u64>) -> Result {
        if range.start < hw::HOST || range.end > hw::HOST + hw::HOST_SIZE || range.is_empty() {
            return Err(EINVAL);
        }
        self.sync_inner(uat, Some(range))
    }

    fn sync_inner(&mut self, uat: &mmu::Uat, range: Option<Range<u64>>) -> Result {
        let Some(l2) = Self::shadow_l2(uat)? else {
            return Ok(());
        };
        let snapshot = &mut self.snapshot;
        with_table(l2, ENTRIES, |entries| {
            for (out, e) in snapshot.iter_mut().zip(entries) {
                *out = e.load(Ordering::Acquire);
            }
        })?;
        let shared = self.shared_l2.io();
        for i in 0..ENTRIES {
            let e = self.snapshot[i];
            if e & VALID != VALID {
                if self.image[i] != 0 {
                    // Tables are never freed, so a linked entry cannot disappear.
                    return Err(EIO);
                }
                continue;
            }
            let l3 = e & hw::PA_MASK;
            if self.image[i] != e {
                // A new leaf table: complete it in memory before the firmware can walk into it.
                clean_page(l3)?;
                mb();
                shared.try_write64(e, i * 8)?;
                self.image[i] = e;
                continue;
            }
            match &range {
                None => clean_page(l3)?,
                Some(r) => {
                    let base = hw::HOST + ((i as u64) << 25);
                    let start = r.start.max(base);
                    let end = r.end.min(base + (1 << 25));
                    if start < end {
                        let first = ((start - base) >> 14) as usize;
                        let last = ((end - 1 - base) >> 14) as usize;
                        clean_entries(l3, first, last - first + 1)?;
                    }
                }
            }
        }
        mb();
        Ok(())
    }

    /// Leaf tables linked into the shared L2.
    pub(crate) fn l3_count(&self) -> usize {
        self.image.iter().filter(|&&e| e != 0).count()
    }
}
