// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! GPU UAT (MMU) management
//!
//! AGX GPUs use an MMU called the UAT, which is largely compatible with the ARM64 page table
//! format. This module manages the global MMU structures, including a shared handoff structure
//! that is used to coordinate VM management operations with the firmware, the TTBAT which points
//! to currently active GPU VM contexts, as well as the individual `Vm` operations to map and
//! unmap buffer objects into a single user or kernel address space.
//!
//! The actual page table management is in the `pt` module.
//!
//! G17 firmware does not take part in the handoff protocol. Instead, each of its two coprocessor
//! instances publishes the top-level entries it owns into its own kernel root while it boots, and
//! TTBAT contexts other than a VM's bind slot may name a VM's root. See [`UatFirmware`].

use core::fmt::Debug;
use core::mem::{size_of, ManuallyDrop};
use core::num::NonZeroUsize;
use core::ops::Range;
use core::sync::atomic::{
    fence,
    AtomicBool,
    AtomicU32,
    AtomicU64,
    AtomicU8,
    Ordering, //
};

use kernel::{
    addr::{
        PhysicalAddr, //
    },
    c_str,
    device::{
        self, //
        Bound,
    },
    dma::DmaAddress,
    drm::{
        gpuvm::{
            self,
            GpuVaAlloc,
            GpuVaFlag,
            GpuVaFlags,
            OpMap,
            OpMapRequest,
            OpMapped,
            OpRemap,
            OpRemapped,
            OpUnmap,
            OpUnmapped, //
        },
        mm,
        Registered, //
    },
    error::Result,
    io::{
        self,
        ResourceSize, //
    },
    new_mutex,
    prelude::*,
    // scatterlist::SGTable,
    static_lock_class,
    sync::{
        aref::ARef,
        lock::{
            mutex::MutexBackend,
            Guard, //
        },
        Arc,
        Mutex, //
    },
    time::{
        delay::fsleep,
        Delta,
        Instant,
        Monotonic, //
    }, //
};

use crate::debug::*;
use crate::module_parameters;
use crate::no_debug;
use crate::{
    driver::{
        self,
        AsahiDrmDevice,
        AsahiDrmDriver, //
    },
    fw,
    gem,
    mem,
    pgtable,
    slotalloc,
    util::RangeExt, //
};

// KernelMapping protection types
pub(crate) use crate::pgtable::Prot;
pub(crate) use pgtable::prot::*;
pub(crate) use pgtable::{
    UatPageTable,
    UAT_PGBIT,
    UAT_PGMSK,
    UAT_PGSZ,
    UAT_TOP_LEVEL_SHIFT, //
};

use pin_init;

const DEBUG_CLASS: DebugFlags = DebugFlags::Mmu;

mod bind;
mod context;
mod lifetime;
mod ppl;
#[cfg(CONFIG_DRM_ASAHI_M3_T8122)]
mod m3;
mod residency;
mod shared;
pub(crate) use bind::PreparedUserBindBatch;
pub(crate) use lifetime::VmJobGuard;
pub(crate) use residency::{Gate as ResidencyGate, VmShrinker};

/// PPL magic number for the handoff region
const PPL_MAGIC: u64 = 0x4b1d000000000002;

/// Number of supported context entries in the TTBAT
const UAT_NUM_CTX: usize = 64;

/// Application contexts owned by independent logical queues. Contexts 0..4
/// and 63 remain reserved for the firmware and device-global resources.
const EXECUTION_CONTEXT_MASK: u64 = ((1u64 << 63) - 1) & !((1u64 << 5) - 1);
/// Maximum simultaneously retained logical queue contexts.
pub(crate) const MAX_EXECUTION_CONTEXTS: u32 = EXECUTION_CONTEXT_MASK.count_ones();
/// First context available for users
const UAT_USER_CTX_START: usize = 1;
/// Number of available user contexts
const UAT_USER_CTX: usize = UAT_NUM_CTX - UAT_USER_CTX_START;

/// Lower/user base VA
pub(crate) const IOVA_USER_BASE: u64 = UAT_PGSZ as u64;

/// Top-level page table entry of the upper half that holds the driver-managed kernel VA range.
const IOVA_KERN_TOP_LEVEL_INDEX: u64 = 2;

/// TTBAT context that starts out as an alias of the kernel context (with its own ASID) with
/// [`UatFirmware::PublishedRoots`].
const KERNEL_ALIAS_CTX: usize = 1;

/// How the GPU firmware shares UAT management with the driver.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) enum UatFirmware {
    /// One coprocessor that publishes its magic number in the handoff region, arbitrates the
    /// handoff lock with the driver, and acknowledges cache flush requests (G13/G14).
    Handoff,
    /// One or two coprocessor instances that never touch the handoff region (G17).
    ///
    /// Each instance has its own kernel root and writes the top-level entries below and including
    /// the driver-managed kernel window into it while it boots: the lower ones are private to the
    /// instance, and the kernel window entry points at a table that the bootloader reserved. The
    /// primary instance uses the `pagetables` root, a secondary one (A18 Pro) the root at
    /// `secondary_root_offset` in the same region; a single-instance firmware (M4 Pro) has none.
    /// TTBAT context 1 starts as an alias of the kernel context.
    PublishedRoots {
        /// Offset of the secondary instance's kernel root in the `pagetables` region.
        secondary_root_offset: Option<u64>,
    },
}

/// UAT policy of a SoC where it differs from the A18 Pro's.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) struct UatQuirks {
    /// The firmware builds its root and kernel-window tables in the no-map `pagetables` and
    /// `l2-pagetables` regions: reach tables there through a write-back mapping instead of
    /// `struct page`.
    ///
    /// Temporary: once the boot loader adds these regions to `/memory`, as m1n1 does, the
    /// regions lose `no-map` and this path goes away.
    pub(crate) reserved_tables: bool,
    // The following settings keep a SoC on the only policy it was tested with. Each one is
    // a candidate for the A18 Pro's value once a test on that SoC shows it is not needed.
    /// Write back every page-table and context-table change: the GPU table walker is not
    /// assumed to snoop the CPU caches.
    pub(crate) clean_tables: bool,
    /// Invalidate ranges one page at a time, never with range TLBI operations.
    pub(crate) page_tlbi: bool,
    /// Take the handoff lock around context-table edits. Where the firmware leaves its half of
    /// the handoff page alone, its lock bytes are not initialized and are never taken.
    pub(crate) handoff_lock: bool,
    /// Handoff "current slot" written at initialization.
    pub(crate) handoff_slot: u32,
    /// Store GPU-only mappings of client and kernel-lower tables as AP 0 with the access in
    /// PXN (read) and UXN (write), the form the firmware itself uses, instead of AP 2.
    pub(crate) native_gpu_permissions: bool,
}

impl UatQuirks {
    /// The A18 Pro and earlier policy.
    pub(crate) const DEFAULT: UatQuirks = UatQuirks {
        reserved_tables: false,
        clean_tables: false,
        page_tlbi: false,
        handoff_lock: true,
        handoff_slot: 0,
        native_gpu_permissions: false,
    };
}

const TTBR_VALID: u64 = 0x1; // BIT(0)
const TTBR_ASID_SHIFT: usize = 48;

/// M4's nonzero wire incarnation of a host-owned root installation.
const fn binding_wire_generation(incarnation: u64) -> u8 {
    if incarnation == 0 {
        0
    } else {
        (1 + (incarnation - 1) % 255) as u8
    }
}

/// Upper word of an application TTBAT slot. M4 exposes firmware upper mappings only in
/// context 0; the existing G15 handoff shares them with user contexts.
const fn application_ttb1(root: u64, asid: usize, map_kernel_to_user: bool) -> u64 {
    if map_kernel_to_user {
        root | TTBR_VALID | (asid as u64) << TTBR_ASID_SHIFT
    } else {
        0
    }
}

/// UAT address space geometry of a given SoC.
#[derive(Debug, Copy, Clone)]
pub(crate) struct UatGeometry {
    /// Input address bits translated by each page table root.
    pub(crate) ias: u32,
    /// Output (physical) address bits.
    pub(crate) oas: u32,
}

impl UatGeometry {
    /// Lower/user top VA
    const fn user_top(&self) -> u64 {
        1 << self.ias
    }

    /// Lower/user VA range
    pub(crate) const fn user_range(&self) -> Range<u64> {
        IOVA_USER_BASE..self.user_top()
    }

    /// Address of a special dummy page?
    pub(crate) const fn unk_page(&self) -> u64 {
        self.user_top() - 2 * UAT_PGSZ as u64
    }

    /// User VA range excluding the unk page
    pub(crate) const fn user_usable_range(&self) -> Range<u64> {
        IOVA_USER_BASE..self.unk_page()
    }

    /// Upper/kernel base VA
    const fn upper_base(&self) -> u64 {
        !(self.user_top() - 1)
    }

    /// Driver-managed kernel VA range
    pub(crate) const fn kernel_range(&self) -> Range<u64> {
        let start = self.upper_base() + (IOVA_KERN_TOP_LEVEL_INDEX << UAT_TOP_LEVEL_SHIFT);
        start..(start + (1 << UAT_TOP_LEVEL_SHIFT))
    }
}

/// A pre-allocated memory region for UAT management
struct UatRegion {
    base: PhysicalAddr,
    map: io::mem::Mem,
}

/// SAFETY: It's safe to share UAT region records across threads.
unsafe impl Send for UatRegion {}
/// SAFETY: It's safe to share UAT region records across threads.
unsafe impl Sync for UatRegion {}

/// Handoff region flush info structure
#[repr(C)]
struct FlushInfo {
    state: AtomicU64,
    addr: AtomicU64,
    size: AtomicU64,
}

/// UAT Handoff region layout
#[repr(C)]
struct Handoff {
    magic_ap: AtomicU64,
    magic_fw: AtomicU64,

    lock_ap: AtomicU8,
    lock_fw: AtomicU8,
    // Implicit padding: 2 bytes
    turn: AtomicU32,
    cur_slot: AtomicU32,
    // Implicit padding: 4 bytes
    flush: [FlushInfo; UAT_NUM_CTX + 1],

    unk2: AtomicU8,
    // Implicit padding: 7 bytes
    unk3: AtomicU64,
}

const HANDOFF_SIZE: usize = size_of::<Handoff>();

/// One VM slot in the TTBAT
#[repr(C)]
struct SlotTTBS {
    ttb0: AtomicU64,
    ttb1: AtomicU64,
}

const SLOTS_SIZE: usize = UAT_NUM_CTX * size_of::<SlotTTBS>();

// We need at least page 0 (ttb0)
const PAGETABLES_SIZE: usize = UAT_PGSZ;

/// Lock-free visibility of the mapping epoch. Writers run under the GPUVM
/// exec lock; nested mutations keep cached validation unavailable until the
/// outermost mutation has finished its leaf writes and invalidations.
struct MappingEpoch {
    visible: AtomicU64,
    pending: AtomicU64,
    writers: AtomicU32,
}

impl MappingEpoch {
    fn new() -> Self {
        Self {
            visible: AtomicU64::new(0),
            pending: AtomicU64::new(0),
            writers: AtomicU32::new(0),
        }
    }
    fn snapshot(&self) -> Option<u64> {
        let value = self.visible.load(Ordering::Acquire);
        (value != u64::MAX).then_some(value)
    }
}

#[must_use = "retain until the mapping mutation and its invalidations finish"]
struct MappingMutation(Option<Arc<MappingEpoch>>);

impl Drop for MappingMutation {
    fn drop(&mut self) {
        if let Some(epoch) = self.0.as_ref() {
            if epoch.writers.fetch_sub(1, Ordering::Relaxed) == 1 {
                epoch
                    .visible
                    .store(epoch.pending.load(Ordering::Relaxed), Ordering::Release);
            }
        }
    }
}

/// Inner data for a Vm instance.
struct VmInner {
    dev: driver::AsahiDevRef,
    is_kernel: bool,
    va_range: Range<u64>,
    page_table: UatPageTable,
    mapping_epoch: Option<Arc<MappingEpoch>>,
    bind_sequence: Arc<AtomicU64>,
    epoch: Option<u64>,
    mm: mm::Allocator<(), KernelMappingInner>,
    uat_inner: Arc<UatInner>,
    binding: Arc<Mutex<VmBinding>>,
    id: u64,
}

/// Exec-locked access to the [`VmInner`] of a GPUVM.
///
/// `GpuVm::exec_lock()` returns a `GpuVmExec`; this guard gives direct access to the driver data
/// it protects.
struct VmExecGuard(gpuvm::GpuVmExec<VmInner>);

impl core::ops::Deref for VmExecGuard {
    type Target = VmInner;

    fn deref(&self) -> &VmInner {
        self.0.unique_gpuvm.data_ref()
    }
}

impl core::ops::DerefMut for VmExecGuard {
    fn deref_mut(&mut self) -> &mut VmInner {
        self.0.unique_gpuvm.data()
    }
}

impl VmExecGuard {
    /// Mark the exact retained logical mapping after page-table restoration.
    fn set_invalidated(&mut self, addr: u64, range: u64, invalidated: bool) -> Result {
        self.0.set_invalidated(addr, range, invalidated)
    }
}

/// Take the exec lock of a VM's GPUVM.
fn lock_vm(
    gpuvm: &gpuvm::GpuVm<VmInner>,
    obj: Option<&gem::Object>,
    interruptible: bool,
) -> Result<VmExecGuard> {
    // SAFETY: As in the rest of this file, the `UniqueRefGpuVm` returned by `GpuVm::new()` is not
    // retained, so the exec lock is the only way to reach the VM's data.
    Ok(VmExecGuard(unsafe { gpuvm.exec_lock(obj, interruptible)? }))
}

#[cfg(CONFIG_DRM_ASAHI_UAT_ADOPTION)]
pub(crate) struct VmReadGuard(VmExecGuard);

#[cfg(CONFIG_DRM_ASAHI_UAT_ADOPTION)]
impl VmReadGuard {
    #[cfg(CONFIG_DRM_ASAHI_G15)]
    pub(crate) fn covers_range(&mut self, address: u64, size: u64, read: bool, write: bool) -> bool {
        Vm::covers_range_locked(&mut self.0, address, size, read, write)
    }

    pub(crate) fn read_user(&mut self, va: u64, buf: &mut [u8]) -> Result {
        Vm::read_user_locked(&mut self.0, va, buf)
    }
}

/// Slot binding-related inner data for a Vm instance.
struct VmBinding {
    active_users: usize,
    binding: Option<slotalloc::Guard<SlotInner>>,
    bind_token: Option<slotalloc::SlotToken>,
    /// Nonzero M4 wire incarnation of the root currently owning this slot.
    generation: u8,
    ttb: u64,
}

/// Data associated with a VM <=> BO pairing
#[pin_data]
struct VmBoData {
    /// Highest sequence assigned to a GPUVA linked to this VM/BO pair.
    ///
    /// Map steps update this monotonically under the VM reservation before
    /// linking the GPUVA. Remap pieces keep their original sequence.
    newest_bind: AtomicU64,
    #[pin]
    segments: Mutex<Option<SgSegments>>,
}

/// Copied DMA addresses and the lease covering every alias of this VM/BO association.
/// The lease is dropped only after all aliases have been unmapped and invalidated.
struct SgSegments {
    segments: KVVec<(DmaAddress, ResourceSize)>,
    _lease: kernel::drm::gem::shmem::SGLease<gem::AsahiObject>,
}

impl SgSegments {
    fn iter(&self) -> core::slice::Iter<'_, (DmaAddress, ResourceSize)> {
        self.segments.iter()
    }
}

/// Preallocated resources needed to execute a VM operation.
///
/// VM operations may require allocating new GPUVA objects to track mappings.
/// To avoid allocation failures during the operation, preallocate the
/// maximum number of GPUVAs that might be needed.
struct VmOpResources {
    /// Preallocated GPUVA objects for remap operations.
    ///
    /// Partial unmap requests or map requests overlapping existing mappings
    /// will trigger a remap call, which needs to register up to three VA
    /// objects (one for the new mapping, and two for the previous and next
    /// mappings).
    preallocated_gpuvas: [Option<GpuVaAlloc<VmInner>>; 3],
}

impl VmOpResources {
    /// Resources for unmapping a range.
    ///
    /// An unmap splits at most the two mappings that straddle its ends. Each split takes two
    /// GPUVAs before it knows which pieces survive and returns the unused one, so cutting the tail
    /// of one mapping and the head of the next needs three GPUVAs at the second split.
    fn for_unmap() -> Result<Self> {
        Ok(Self {
            preallocated_gpuvas: [
                Some(GpuVaAlloc::<VmInner>::new(GFP_KERNEL)?),
                Some(GpuVaAlloc::<VmInner>::new(GFP_KERNEL)?),
                Some(GpuVaAlloc::<VmInner>::new(GFP_KERNEL)?),
            ],
        })
    }
}

struct StepContext<'ctx> {
    dev: Option<&'ctx device::Device<Bound>>,
    prot: Prot,

    /// Preallocated resources that can be used when executing the request.
    resources: &'ctx mut VmOpResources,

    /// Sequence assigned by the successful map for its related metadata.
    bind_sequence: u64,
}

impl<'ctx> StepContext<'ctx> {
    fn new(
        dev: Option<&'ctx device::Device<Bound>>,
        prot: Prot,
        resources: &'ctx mut VmOpResources,
    ) -> Result<StepContext<'ctx>> {
        Ok(Self {
            dev,
            prot,
            resources,
            bind_sequence: 0,
        })
    }

    /// Finds one of our pre-allocated VAs.
    fn preallocated_gpuva(&mut self) -> Result<GpuVaAlloc<VmInner>> {
        self.resources
            .preallocated_gpuvas
            .iter_mut()
            .find_map(|f| f.take())
            .ok_or(EINVAL)
    }

    /// Returns an unused GPUVA object to the preallocated pool.
    /// If the pool is already full, the unused allocation is simply dropped.
    fn return_preallocated_gpuva(&mut self, gpuva: GpuVaAlloc<VmInner>) {
        if let Some(slot) = self
            .resources
            .preallocated_gpuvas
            .iter_mut()
            .find(|slot| slot.is_none())
        {
            *slot = Some(gpuva);
        }
    }
}

/// Per-mapping metadata that survives backing eviction and GPUVA splits.
#[derive(Default, Clone)]
struct VmGpuVa {
    prot: Option<Prot>,
    bind_sequence: u64,
}

impl gpuvm::DriverGpuVm for VmInner {
    type Driver = AsahiDrmDriver;
    type Object = gem::Object;
    type VaData = VmGpuVa;
    type VmBoData = VmBoData;
    type SmContext<'ctx>
        = StepContext<'ctx>
    where
        Self: 'ctx;

    /// Create a new mapping.
    fn sm_step_map<'op, 'ctx>(
        &mut self,
        op: OpMap<'op, Self>,
        ctx: &mut Self::SmContext<'ctx>,
    ) -> Result<OpMapped<'op, Self>, Error> {
        let bind_sequence = self.bind_sequence.fetch_update(
            Ordering::AcqRel,
            Ordering::Acquire,
            |value| value.checked_add(1),
        ).map_err(|_| EOVERFLOW)? + 1;
        let _mutation = self.mapping_mutation();
        let (map_addr, map_length) = (op.addr(), op.length() as usize);
        let mut iova = op.addr();
        let mut left = op.length() as usize;
        let mut offset = op.gem_offset() as usize;

        let one_page = op.flags().contains(gpuvm::GpuVaFlag::REPEAT);

        if ctx.dev.is_none() {
            return Err(ENODEV);
        }
        // Bind preparation retained the table's backing before taking the VM reservation.
        // Never acquire the object reservation from inside the GPUVM map callback.
        let segments = op.vm_bo().data().segments.lock();
        let sgt = segments.as_ref().ok_or(EFAULT)?;
        for &(dma_address, dma_len) in sgt.iter() {
            let mut addr = dma_address as usize;
            let mut len = dma_len as usize;

            if left == 0 {
                break;
            }

            if offset > 0 {
                let skip = len.min(offset);
                addr += skip;
                len -= skip;
                offset -= skip;
            }
            if len == 0 {
                continue;
            }

            assert!(offset == 0);

            if one_page {
                len = left;
            } else {
                len = len.min(left);
            }

            mod_dev_dbg!(
                self.dev,
                "MMU: map: {:#x}:{:#x} -> {:#x} [OP={}]\n",
                addr,
                len,
                iova,
                one_page
            );

            if let Err(error) = self.page_table.map_pages(
                iova..(iova + len as u64),
                addr as PhysicalAddr,
                ctx.prot,
                one_page,
            ) {
                self.rollback_failed_map(map_addr, map_length as u64);
                return Err(error);
            }

            left -= len;
            iova += len as u64;
        }

        drop(segments);
        let gpuva = match ctx.preallocated_gpuva() {
            Ok(gpuva) => gpuva,
            Err(error) => {
                self.rollback_failed_map(map_addr, map_length as u64);
                return Err(error);
            }
        };
        op.vm_bo()
            .data()
            .newest_bind
            .fetch_max(bind_sequence, Ordering::Relaxed);
        let op = op.insert(
            gpuva,
            VmGpuVa {
                prot: Some(ctx.prot),
                bind_sequence,
            },
        );
        ctx.bind_sequence = bind_sequence;

        self.tlbi_contexts_naming_root(map_addr, map_length);
        Ok(op)
    }

    /// Unmap existing mapping.
    fn sm_step_unmap<'op, 'ctx>(
        &mut self,
        op: OpUnmap<'op, Self>,
        _ctx: &mut Self::SmContext<'ctx>,
    ) -> Result<OpUnmapped<'op, Self>, Error> {
        self.unmap_gpuva(op.va())?;

        let (op_unmapped, _va_removed) = op.remove();

        Ok(op_unmapped)
    }

    /// Truncate or split exisiting mapping.
    fn sm_step_remap<'op, 'ctx>(
        &mut self,
        op: OpRemap<'op, Self>,
        ctx: &mut Self::SmContext<'ctx>,
    ) -> Result<OpRemapped<'op, Self>, Error> {
        let orig_prot = op.va_to_unmap().data().prot.ok_or(EINVAL)?;
        let orig_bind_sequence = op.va_to_unmap().data().bind_sequence;
        let orig_invalidated = op.va_to_unmap().flags().contains(GpuVaFlag::INVALIDATED);

        // Only unmap the hole between prev/next, if they exist
        let unmap_start = if let Some(op) = op.prev() {
            op.addr() + op.length()
        } else {
            op.va_to_unmap().addr()
        };

        let unmap_end = if let Some(op) = op.next() {
            op.addr()
        } else {
            op.va_to_unmap().addr() + op.va_to_unmap().length()
        };

        mod_dev_dbg!(
            self.dev,
            "MMU: unmap for remap: {:#x}..{:#x} (from {:#x}:{:#x})\n",
            unmap_start,
            unmap_end,
            op.va_to_unmap().addr(),
            op.va_to_unmap().addr() + op.va_to_unmap().length()
        );

        let unmap_range = unmap_end - unmap_start;

        // GPUVM must be able to record the surviving pieces once their hole is gone from the page
        // tables, so take the allocations first. `remap()` takes one for each side and hands back
        // the one for a side that does not survive.
        let prev_gpuva = ctx.preallocated_gpuva()?;
        let next_gpuva = match ctx.preallocated_gpuva() {
            Ok(gpuva) => gpuva,
            Err(error) => {
                ctx.return_preallocated_gpuva(prev_gpuva);
                return Err(error);
            }
        };

        let _mutation = self.mapping_mutation();
        if orig_invalidated {
            self.page_table
                .discard_partial_map(unmap_start..unmap_end)?;
        } else {
            self.page_table.unmap_pages(unmap_start..unmap_end)?;
        }

        if let Some(asid) = self.slot() {
            mem::sync();
            self.tlbi_range(asid as u8, unmap_start, unmap_range as usize);
            mod_dev_dbg!(
                self.dev,
                "MMU: flush range: asid={:#x} start={:#x} len={:#x}\n",
                asid,
                unmap_start,
                unmap_range,
            );
            mem::sync();
        }

        self.tlbi_contexts_naming_root(unmap_start, unmap_range as usize);

        let (op_remapped, remap_ret) = op.remap(
            [prev_gpuva, next_gpuva],
            VmGpuVa {
                prot: Some(orig_prot),
                bind_sequence: orig_bind_sequence,
            },
            VmGpuVa {
                prot: Some(orig_prot),
                bind_sequence: orig_bind_sequence,
            },
        );

        if let Some(unused_va) = remap_ret.unused_va {
            ctx.return_preallocated_gpuva(unused_va);
        }

        Ok(op_remapped)
    }
}

impl VmInner {
    /// Remove hardware leaves for one complete user GPUVA without allocating.
    fn unmap_gpuva(&mut self, va: &gpuvm::GpuVa<Self>) -> Result {
        mod_dev_dbg!(
            self.dev,
            "MMU: unmap: {:#x}:{:#x}\n",
            va.addr(),
            va.length()
        );

        let _mutation = self.mapping_mutation();
        if va.flags().contains(GpuVaFlag::INVALIDATED) {
            self.page_table
                .discard_partial_map(va.addr()..(va.addr() + va.length()))?;
        } else {
            self.page_table
                .unmap_pages(va.addr()..(va.addr() + va.length()))?;
        }

        if let Some(asid) = self.slot() {
            mem::sync();
            self.tlbi_range(asid as u8, va.addr(), va.length() as usize);
            mod_dev_dbg!(
                self.dev,
                "MMU: flush range: asid={:#x} start={:#x} len={:#x}\n",
                asid,
                va.addr(),
                va.length(),
            );
            mem::sync();
        }

        self.tlbi_contexts_naming_root(va.addr(), va.length() as usize);

        Ok(())
    }

    /// A failed map has no GPUVA owner to retain its backing or remove its leaves.
    /// GPUVM has already split or removed overlaps throughout this operation's range.
    fn rollback_failed_map(&mut self, addr: u64, size: u64) {
        // Include the current segment: map_pages can fail after writing some of its leaves.
        // The checked GPUVM operation supplies an aligned range without overflow.
        let result = self.page_table.discard_partial_map(addr..addr + size);
        self.finish_failed_map(addr, size as usize);
        if let Err(error) = result {
            dev_err!(
                self.dev.as_ref(),
                "Could not discard failed mapping: {:?}\n",
                error
            );
        }
    }

    /// Complete discarded stores and invalidate both the legacy bound slot and
    /// every M4/G17 context naming this root. Call only after releasing PPL.
    fn finish_failed_map(&self, addr: u64, size: usize) {
        if self.uat_inner.firmware == UatFirmware::Handoff {
            if let Some(asid) = self.slot() {
                mem::sync();
                self.tlbi_range(asid as u8, addr, size);
                mem::sync();
            }
        }
        self.tlbi_contexts_naming_root(addr, size);
    }

    fn mapping_mutation(&mut self) -> MappingMutation {
        let state = self.mapping_epoch.clone();
        if let Some(state) = state.as_ref() {
            state.writers.fetch_add(1, Ordering::Relaxed);
            state.visible.swap(u64::MAX, Ordering::AcqRel);
            self.epoch = self.epoch.and_then(|epoch| epoch.checked_add(1));
            state
                .pending
                .store(self.epoch.unwrap_or(u64::MAX), Ordering::Relaxed);
        }
        MappingMutation(state)
    }

    /// Returns the slot index, if this VM is bound.
    fn slot(&self) -> Option<u32> {
        if self.is_kernel {
            // The GFX ASC does not care about the ASID. Pick an arbitrary one.
            // TODO: This needs to be a persistently reserved ASID once we integrate
            // with the ARM64 kernel ASID machinery to avoid overlap.
            Some(0)
        } else {
            // We don't check whether we lost the slot, which could cause unnecessary
            // invalidations against another Vm. However, this situation should be very
            // rare (e.g. a Vm lost its slot, which means 63 other Vms bound in the
            // interim, and then it gets killed / drops its mappings without doing any
            // final rendering). Anything doing active maps/unmaps is probably also
            // rendering and therefore likely bound.
            self.binding
                .lock()
                .bind_token
                .as_ref()
                .map(|token| token.last_slot() + UAT_USER_CTX_START as u32)
        }
    }

    /// Returns the translation table base for this Vm
    fn ttb(&self) -> u64 {
        self.page_table.ttb()
    }

    /// Invalidate a VA range of this Vm's address space in the TLB of one ASID.
    fn tlbi_range(&self, asid: u8, iova: u64, size: usize) {
        match self.uat_inner.firmware {
            UatFirmware::Handoff => mem::tlbi_range(asid, iova as usize, size),
            UatFirmware::PublishedRoots { .. } => {
                mem::tlbi_range_or_asid(asid, iova as usize, size)
            }
        }
    }

    /// Invalidate a VA range in every TTBAT context that names this Vm's root as its lower half.
    ///
    /// With [`UatFirmware::PublishedRoots`], contexts other than a Vm's bind slot name its root
    /// (the kernel lower root sits in contexts 0 and 1 and has no slot at all), so a mapping
    /// change must reach all of them. Adopted M4 roots also remain in the TTBAT after a bind
    /// lease is dropped, until the slot is reassigned or the root is destroyed. Include those
    /// contexts before allowing an unmapped object's backing to be released.
    fn tlbi_contexts_naming_root(&self, iova: u64, size: usize) {
        if self.uat_inner.firmware == UatFirmware::Handoff && !self.uat_inner.m4_adopted {
            mem::sync();
            return;
        }

        let contexts = self
            .uat_inner
            .lock()
            .contexts_naming_root(self.ttb(), self.uat_inner.m4_adopted);
        self.tlbi_context_mask(iova, size, contexts);
    }

    fn tlbi_context_mask(&self, iova: u64, size: usize, contexts: u64) {
        // Complete new leaves even if no context currently names this root:
        // a later context can publish it without repeating the map operation.
        // DMB orders accesses but does not complete stores before TLBI; the
        // GPU uses outer-shareable invalidation and the barrier covers it.
        mem::sync();
        if contexts == 0 {
            return;
        }
        for ctx in 0..UAT_NUM_CTX {
            if contexts & (1 << ctx) != 0 {
                mem::tlbi_range_or_asid(ctx as u8, iova as usize, size);
            }
        }
        mem::sync();
    }

    /// Map an `mm::Node` representing an mapping in VA space.
    fn map_node(&mut self, node: &mm::Node<(), KernelMappingInner>, prot: Prot) -> Result {
        let end = node
            .start()
            .checked_add(node.mapped_size as u64)
            .ok_or(EOVERFLOW)?;
        let _mutation = self.mapping_mutation();
        // Allocate detached RAM tables before excluding the firmware. The shadow L2 is not
        // the reserved shared-L2; only final leaf stores below already linked L3s are live.
        self.page_table.prepare_map(node.start()..end)?;
        let uat_inner = self.uat_inner.clone();
        let handoff = uat_inner.lock_m4_upper(self.is_kernel)?;
        let result = self.map_node_prepared(node, prot);
        let discard = if result.is_err() {
            // Rollback still changes live leaves and must remain under the PPL lease.
            self.page_table.discard_partial_map(node.start()..end)
        } else {
            Ok(())
        };
        // The M4 handoff guard owns UatShared's mutex. Invalidation follows its release.
        drop(handoff);
        if result.is_err() {
            self.finish_failed_map(node.start(), node.mapped_size);
        } else {
            self.tlbi_contexts_naming_root(node.start(), node.mapped_size);
        }
        if let Err(error) = discard {
            if self.uat_inner.m4_adopted {
                // The caller retains its node and backing when this latch is set.
                self.uat_inner.failed.store(true, Ordering::Release);
            }
            dev_err!(
                self.dev.as_ref(),
                "Could not discard failed kernel mapping: {:?}\n",
                error
            );
        }
        result
    }

    /// Install only into the empty range reserved by `map_node`.
    fn map_node_prepared(&mut self, node: &mm::Node<(), KernelMappingInner>, prot: Prot) -> Result {
        let mut iova = node.start();
        let segments = node
            .bo
            .as_ref()
            .ok_or(EINVAL)?
            .get()?
            .data()
            .segments
            .lock();
        let mut offset = node.offset;
        let mut left = node.mapped_size;

        if let Some(segments) = &*segments {
            for (dma_address, dma_len) in segments.iter() {
                if left == 0 {
                    break;
                }

                // TODO: proper DMA address/length handling
                let mut addr = *dma_address as usize;
                let mut len: usize = *dma_len as usize;

                if (offset | addr | len | iova as usize) & UAT_PGMSK != 0 {
                    dev_err!(
                        self.dev.as_ref(),
                        "MMU: KernelMapping {:#x}:{:#x} -> {:#x} is not page-aligned\n",
                        addr,
                        len,
                        iova
                    );
                    return Err(EINVAL);
                }

                if offset > 0 {
                    let skip = len.min(offset);
                    addr += skip;
                    len -= skip;
                    offset -= skip;
                }

                len = len.min(left);

                if len == 0 {
                    continue;
                }

                mod_dev_dbg!(
                    self.dev,
                    "MMU: map: {:#x}:{:#x} -> {:#x}\n",
                    addr,
                    len,
                    iova
                );

                self.page_table.map_pages(
                    iova..(iova + len as u64),
                    addr as PhysicalAddr,
                    prot,
                    false,
                )?;

                iova += len as u64;
                left -= len;
            }
        }

        // The caller invalidates negative translations after releasing its PPL lease.
        Ok(())
    }
}

/// Shared reference to a virtual memory address space ([`Vm`]).
#[derive(Clone)]
pub(crate) struct Vm {
    dev: ARef<device::Device>,
    id: u64,
    mapping_epoch: Option<Arc<MappingEpoch>>,
    bind_sequence: Arc<AtomicU64>,
    lifetime: Option<Arc<lifetime::VmLifetime>>,
    residency: Option<Arc<residency::Gate>>,
    context_bindings: Option<Arc<context::ContextBindings>>,
    shared_bindings: Option<Arc<shared::SharedBindings>>,
    status: Option<Arc<crate::g17::status::VmStatus>>,
    gpuvm: ARef<gpuvm::GpuVm<VmInner>>,
    dummy_obj: ARef<gem::Object>,
    binding: Arc<Mutex<VmBinding>>,
}
no_debug!(Vm);

/// An application TTBAT identity retained by a logical queue and its jobs.
/// The root lease is dropped before the VM, so no page table can be freed
/// while an active context still names it. Installed firmware queues retain
/// this object through replacement retirement or processor stop; a consumed
/// closed-scheduler release may return its root identity while keeping storage.
pub(crate) struct ExecutionContext {
    root: ExecutionRoot,
    vm: Vm,
}

struct ExecutionRoot {
    released: AtomicBool,
    inner: Arc<UatInner>,
    id: u8,
    generation: u8,
    low: u64,
    high: u64,
}

impl ExecutionContext {
    pub(crate) fn id(&self) -> u32 {
        self.root.id.into()
    }
    pub(crate) fn generation(&self) -> u8 {
        self.root.generation
    }
    pub(crate) fn vm(&self) -> &Vm {
        &self.vm
    }

    /// Called only after the closed scheduler's release was consumed and all
    /// its commands retired. Installed storage may outlive this root lease.
    pub(crate) fn release(&self) {
        self.root.release();
    }

    pub(crate) fn is_current(&self) -> bool {
        let inner = self.root.inner.lock();
        if self.root.released.load(Ordering::Acquire) {
            return false;
        }
        let roots = &inner.ttbs()[usize::from(self.root.id)];
        roots.ttb0.load(Ordering::Acquire) == self.root.low
            && roots.ttb1.load(Ordering::Acquire) == self.root.high
    }
}

impl ExecutionRoot {
    fn release(&self) {
        if self.released.swap(true, Ordering::AcqRel) {
            return;
        }
        let id = usize::from(self.id);
        let release = {
            let inner = self.inner.lock();
            inner.handoff_lock();
            let roots = &inner.ttbs()[id];
            let low = roots.ttb0.load(Ordering::Acquire);
            let high = roots.ttb1.load(Ordering::Acquire);
            let owned = low == self.low && high == self.high;
            if owned {
                roots.ttb0.store(0, Ordering::Release);
                roots.ttb1.store(0, Ordering::Release);
                inner.publish_slot(id);
            }
            inner.handoff_unlock();
            owned || (low == 0 && high == 0)
        };
        if release {
            mem::sync();
            mem::tlbi_asid(self.id);
            mem::sync();
            self.inner.lock().execution_contexts &= !(1u64 << id);
        } else {
            pr_err!(
                "MMU: application context {} changed owners while leased\n",
                id
            );
        }
    }
}

impl Drop for ExecutionRoot {
    fn drop(&mut self) {
        self.release();
    }
}

/// Slot data for a [`Vm`] slot (nothing, we only care about the indices).
pub(crate) struct SlotInner();

impl slotalloc::SlotItem for SlotInner {
    type Data = ();
}

/// Represents a single user of a binding of a [`Vm`] to a slot.
///
/// The number of users is counted, and the slot will be freed when it drops to 0.
#[derive(Debug)]
pub(crate) struct VmBind(Vm, u32);

impl VmBind {
    /// Returns the slot that this `Vm` is bound to.
    pub(crate) fn slot(&self) -> u32 {
        self.1
    }

    /// Stable wire incarnation for this active root lease. M4 reserves zero as unbound.
    #[cfg(CONFIG_DRM_ASAHI_G16)]
    pub(crate) fn generation(&self) -> u8 {
        self.0.binding.lock().generation
    }
}

impl Drop for VmBind {
    fn drop(&mut self) {
        let mut binding = self.0.binding.lock();

        assert_ne!(binding.active_users, 0);
        binding.active_users -= 1;
        mod_pr_debug!(
            "MMU: slot {} active users {}\n",
            self.1,
            binding.active_users
        );
        if binding.active_users == 0 {
            binding.binding = None;
        }
    }
}

impl Clone for VmBind {
    fn clone(&self) -> VmBind {
        let mut binding = self.0.binding.lock();

        binding.active_users += 1;
        mod_pr_debug!(
            "MMU: slot {} active users {}\n",
            self.1,
            binding.active_users
        );
        VmBind(self.0.clone(), self.1)
    }
}

/// A mapping BO reference released under the immediate-mode GPUVA lock.
/// Retain the owner until deferred cleanup completes, including its final BO.
struct MappingBo {
    bo: Option<ARef<gpuvm::GpuVmBo<VmInner>>>,
    owner: ARef<gpuvm::GpuVm<VmInner>>,
}
impl MappingBo {
    fn new(bo: ARef<gpuvm::GpuVmBo<VmInner>>, owner: ARef<gpuvm::GpuVm<VmInner>>) -> Self {
        Self {
            bo: Some(bo),
            owner,
        }
    }
    fn get(&self) -> Result<&gpuvm::GpuVmBo<VmInner>> {
        self.bo.as_deref().ok_or(EINVAL)
    }
}
impl Drop for MappingBo {
    fn drop(&mut self) {
        let Some(bo) = self.bo.take() else {
            return;
        };
        let raw = ARef::into_raw(bo);
        // SAFETY: GpuVmBo is repr(C) with drm_gpuvm_bo first. into_raw
        // transfers one live reference. Every Vm here uses IMMEDIATE_MODE;
        // deferred put selects its GPUVA lock and retains cleanup ownership.
        unsafe {
            kernel::bindings::drm_gpuvm_bo_put_deferred(
                raw.as_ptr().cast::<kernel::bindings::drm_gpuvm_bo>(),
            )
        };
        self.owner.deferred_cleanup();
    }
}

/// Inner data required for an object mapping into a [`Vm`].
pub(crate) struct KernelMappingInner {
    // Drop order matters:
    // - Deferred-put the GpuVmBo under its immediate-mode GPUVA lock
    // - Drop the GEM BO next, since BO free can take the resv lock itself
    // - Drop the owner GpuVm last, since that again can take resv locks when the refcount drops to 0
    bo: Option<MappingBo>,
    _gem: Option<gem::KernelMappingPin>,
    owner: ARef<gpuvm::GpuVm<VmInner>>,
    uat_inner: Arc<UatInner>,
    prot: Prot,
    offset: usize,
    mapped_size: usize,
}

/// A kernel CPU mapping of a timestamp buffer's GEM backing. Dropping it takes the object's
/// reservation lock, so it must not be dropped where fence producers run.
pub(crate) struct TimestampVmap(kernel::drm::gem::shmem::VMapOwned<gem::AsahiObject>);

impl TimestampVmap {
    pub(crate) fn new(gem: &gem::Object) -> Result<Self> {
        Ok(Self(gem.owned_vmap::<0>()?))
    }

    fn size(&self) -> usize {
        use kernel::drm::gem::BaseObject;
        self.0.owner().size()
    }

    /// Writes a timestamp at byte `offset` of the GEM object. Aligned results use a complete
    /// atomic store; byte stores support unaligned destinations without creating mutable
    /// aliases. An unaligned complete timestamp is not atomic. Out-of-range offsets are refused
    /// when the destination is prepared, and ignored here.
    pub(crate) fn write(&self, offset: usize, value: u64) {
        if offset.checked_add(size_of::<u64>()).is_none_or(|end| end > self.size()) {
            return;
        }
        // Completion runs in task context. Bindings of the same BO share this lock,
        // so aligned u64 and overlapping unaligned byte stores never race in Rust.
        let _guard = self.0.owner().timestamp_lock.lock();
        // SAFETY: The complete extent was checked above, inside this pinned vmap.
        let base = self.0.addr.cast::<u8>();
        let pointer = unsafe { base.add(offset) };
        if pointer as usize % core::mem::align_of::<AtomicU64>() == 0 {
            // SAFETY: All eight bytes were checked above; the vmap pins the backing.
            // A naturally aligned query result becomes available in one complete store.
            let destination = unsafe { &*pointer.cast::<AtomicU64>() };
            destination.store(value.to_le(), Ordering::Relaxed);
        } else {
            for (index, byte) in value.to_le_bytes().into_iter().enumerate() {
                // SAFETY: The complete eight-byte extent was checked above. The owned
                // vmap pins its backing; byte atomics permit unaligned and overlapping
                // timestamp destinations without mutable slice aliases.
                let destination = unsafe { &*base.add(offset + index).cast::<AtomicU8>() };
                destination.store(byte, Ordering::Relaxed);
            }
        }
        // A CPU-retired query can be consumed by a later GPU command. WRITEBACK
        // objects use a cacheable vmap, while GPU mappings need not snoop that cache.
        // Publish the bytes before signalling the completion fence or queue frontier.
        if self.0.owner().is_writeback() {
            crate::pgtable::clean_poc(pointer, size_of::<u64>());
        }
        crate::pgtable::poc_barrier();
    }
}

/// Prepared CPU timestamp access with its own vmap; destruction happens after completion
/// signalling.
#[cfg(CONFIG_DRM_ASAHI_G16)]
pub(crate) struct TimestampMapping {
    mapping: TimestampVmap,
    offset: usize,
}

#[cfg(CONFIG_DRM_ASAHI_G16)]
impl TimestampMapping {
    /// Prepare one bounded CPU timestamp destination on retained GEM backing.
    pub(crate) fn new(gem: &gem::Object, offset: usize) -> Result<Self> {
        use kernel::drm::gem::BaseObject;
        if offset.checked_add(size_of::<u64>()).ok_or(EOVERFLOW)? > gem.size() {
            return Err(ERANGE);
        }
        Ok(Self {
            mapping: TimestampVmap::new(gem)?,
            offset,
        })
    }

    pub(crate) fn write(&self, value: u64) {
        self.mapping.write(self.offset, value);
    }
}

/// An object mapping into a [`Vm`], which reserves the address range from use by other mappings.
pub(crate) struct KernelMapping(ManuallyDrop<mm::Node<(), KernelMappingInner>>);

impl KernelMapping {
    /// Returns the IOVA base of this mapping
    pub(crate) fn iova(&self) -> u64 {
        self.0.start()
    }

    /// Returns the size of this mapping in bytes
    pub(crate) fn size(&self) -> usize {
        self.0.mapped_size
    }

    /// Returns the IOVA base of this mapping
    pub(crate) fn iova_range(&self) -> Range<u64> {
        self.0.start()..(self.0.start() + self.0.mapped_size as u64)
    }

    /// Resolves a byte in this retained mapping without creating page tables.
    pub(crate) fn translate_offset(&self, offset: usize) -> Result<u64> {
        if offset >= self.size() {
            return Err(EINVAL);
        }
        let address = self.iova().checked_add(offset as u64).ok_or(EOVERFLOW)?;
        lock_vm(&self.0.owner, None, false)?
            .page_table
            .translate_iova(address)
    }

    /// Writes a userspace timestamp through the retained GEM mapping. Aligned results
    /// use a complete atomic store; byte stores support unaligned destinations without
    /// creating mutable aliases. An unaligned complete timestamp is not atomic.
    /// Retain CPU access before this destination's completion fence is published.
    #[cfg(CONFIG_DRM_ASAHI_G16)]
    pub(crate) fn timestamp_mapping(&self, offset: usize) -> Result<TimestampMapping> {
        let gem = self.0._gem.as_ref().ok_or(EINVAL)?;
        TimestampMapping::new(gem, self.timestamp_offset(offset)?)
    }

    /// The GEM object offset of the timestamp at `offset` in this mapping, whose complete
    /// extent lies inside both the mapping and the object.
    pub(crate) fn timestamp_offset(&self, offset: usize) -> Result<usize> {
        use kernel::drm::gem::BaseObject;
        let gem = self.0._gem.as_ref().ok_or(EINVAL)?;
        if offset.checked_add(size_of::<u64>()).ok_or(EOVERFLOW)? > self.size() {
            return Err(ERANGE);
        }
        let start = self.0.offset.checked_add(offset).ok_or(EOVERFLOW)?;
        let end = start.checked_add(size_of::<u64>()).ok_or(EOVERFLOW)?;
        if end > gem.size() {
            return Err(ERANGE);
        }
        Ok(start)
    }

    /// A kernel CPU mapping of this mapping's GEM backing, for timestamp writes.
    pub(crate) fn timestamp_vmap(&self) -> Result<TimestampVmap> {
        TimestampVmap::new(self.0._gem.as_ref().ok_or(EINVAL)?)
    }

    /// Gives a fresh kernel mapping a firmware-cached prefix, preserving its tail.
    /// The result includes whether any page-table entry may have changed; after
    /// mutation the caller must retain the mapping until processor stop even if
    /// the readback fails. The original protection is retained for teardown.
    ///
    /// # Safety
    /// The mapping must be unpublished. After mutation it must not be unmapped
    /// while a firmware instance can retain a cached translation or contents.
    pub(crate) unsafe fn set_fw_cached_prefix(&self, prefix: usize) -> (Result<[u64; 2]>, bool) {
        const MAX_PAGES: usize = 32;
        const ATTRIBUTE_SHIFT: u32 = 2;
        const ATTRIBUTE_MASK: u64 = 7;
        const UNCACHED: u64 = 2;
        const CACHE_ATTRIBUTE_CHANGE: u64 = UNCACHED << ATTRIBUTE_SHIFT;
        let mut mutated = false;
        let result = (|| {
            if self.0.uat_inner.firmware == UatFirmware::Handoff
                || prefix == 0
                || prefix > self.size()
                || (prefix | self.size()) & UAT_PGMSK != 0
                || self.size() > MAX_PAGES * UAT_PGSZ
            {
                return Err(EINVAL);
            }
            let mut owner = lock_vm(&self.0.owner, None, false)?;
            if !owner.is_kernel {
                return Err(EINVAL);
            }
            let pages = self.size() / UAT_PGSZ;
            let cached_pages = prefix / UAT_PGSZ;
            let mut before = [0; MAX_PAGES];
            for (index, value) in before[..pages].iter_mut().enumerate() {
                *value = owner
                    .page_table
                    .leaf_entry(self.iova() + (index * UAT_PGSZ) as u64)?;
                if (*value >> ATTRIBUTE_SHIFT) & ATTRIBUTE_MASK != UNCACHED {
                    return Err(EINVAL);
                }
            }
            let _mutation = owner.mapping_mutation();
            mutated = true;
            owner.page_table.reprot_pages(
                self.iova()..self.iova() + prefix as u64,
                PROT_GPU_FW_PRIV_RW,
            )?;
            mem::sync();
            mem::tlbi_all();
            mem::sync();
            let mut after = [0; MAX_PAGES];
            for (index, value) in after[..pages].iter_mut().enumerate() {
                *value = owner
                    .page_table
                    .leaf_entry(self.iova() + (index * UAT_PGSZ) as u64)?;
                let change = if index < cached_pages {
                    CACHE_ATTRIBUTE_CHANGE
                } else {
                    0
                };
                if before[index] ^ *value != change {
                    return Err(EFAULT);
                }
            }
            Ok([after[0], after[cached_pages.min(pages - 1)]])
        })();
        (result, mutated)
    }

    /// Remap a cached mapping as uncached, then synchronously flush that range of VAs from the
    /// coprocessor cache. This is required to safely unmap cached/private mappings.
    fn remap_uncached_and_flush(&mut self) {
        let mut owner = self.0.owner.lock_inner();

        mod_dev_dbg!(
            owner.dev,
            "MMU: remap as uncached {:#x}:{:#x}\n",
            self.iova(),
            self.size()
        );

        // Remap in-place as uncached.
        // Do not try to unmap the guard page (-1)
        let prot = self.0.prot.as_uncached();
        let _mutation = owner.mapping_mutation();
        if owner
            .page_table
            .reprot_pages(self.iova_range(), prot)
            .is_err()
        {
            dev_err!(
                owner.dev.as_ref(),
                "MMU: remap {:#x}:{:#x} failed\n",
                self.iova(),
                self.size()
            );
        }
        mem::sync();

        // If we don't have (and have never had) a VM slot, just return
        let slot = match owner.slot() {
            None => return,
            Some(slot) => slot,
        };

        let flush_slot = if owner.is_kernel {
            // If this is a kernel mapping, always flush on index 64
            UAT_NUM_CTX as u32
        } else {
            // Otherwise, check if this slot is the active one, otherwise return
            // Also check that we actually own this slot
            let ttb = owner.ttb() | TTBR_VALID | (slot as u64) << TTBR_ASID_SHIFT;

            let uat_inner = self.0.uat_inner.lock();
            uat_inner.handoff_lock();
            let cur_slot = uat_inner.handoff().current_slot();
            let ttb_cur = uat_inner.ttbs()[slot as usize].ttb0.load(Ordering::Relaxed);
            uat_inner.handoff_unlock();
            if cur_slot == Some(slot) && ttb_cur == ttb {
                slot
            } else {
                return;
            }
        };

        // FIXME: There is a race here, though it'll probably never happen in practice.
        // In theory, it's possible for the ASC to finish using our slot, whatever command
        // it was processing to complete, the slot to be lost to another context, and the ASC
        // to begin using it again with a different page table, thus faulting when it gets a
        // flush request here. In practice, the chance of this happening is probably vanishingly
        // small, as all 62 other slots would have to be recycled or in use before that slot can
        // be reused, and the ASC using user contexts at all is very rare.

        // Still, the locking around UAT/Handoff/TTBs should probably be redesigned to better
        // model the interactions with the firmware and avoid these races.
        // Possibly TTB changes should be tied to slot locks:

        // Flush:
        //  - Can early check handoff here (no need to lock).
        //      If user slot and it doesn't match the active ASC slot,
        //      we can elide the flush as the ASC guarantees it flushes
        //      TLBs/caches when it switches context. We just need a
        //      barrier to ensure ordering.
        //  - Lock TTB slot
        //      - If user ctx:
        //          - Lock handoff AP-side
        //              - Lock handoff dekker
        //                  - Check TTB & handoff cur ctx
        //      - Perform flush if necessary
        //          - This implies taking the fwring lock
        //
        // TTB change:
        //  - lock TTB slot
        //      - lock handoff AP-side
        //          - lock handoff dekker
        //              change TTB

        // Lock this flush slot, and write the range to it
        let flush = self.0.uat_inner.lock_flush(flush_slot);
        let pages = self.size() >> UAT_PGBIT;
        flush.begin_flush(self.iova(), self.size() as u64);
        if pages >= 0x10000 {
            dev_err!(
                owner.dev.as_ref(),
                "MMU: Flush too big ({:#x} pages))\n",
                pages
            );
        }

        let cmd = fw::channels::FwCtlMsg {
            addr: fw::types::U64(self.iova()),
            unk_8: 0,
            slot: flush_slot,
            page_count: pages as u16,
            unk_12: 2, // ?
        };

        let dev = owner.dev.clone();
        let guard = dev.registration_guard().expect("Device is not registered!");

        // Tell the firmware to do a cache flush
        guard.registration_data_with(|reg_data| {
            let ret = match reg_data.gpu.manager() {
                Some(gpu) => gpu.fwctl(cmd),
                None => Err(ENODEV),
            };
            if let Err(e) = ret {
                dev_err!(
                    owner.dev.as_ref(),
                    "MMU: ASC cache flush {:#x}:{:#x} failed (err: {:?})\n",
                    self.iova(),
                    self.size(),
                    e
                );
            }
        });

        // Finish the flush
        flush.end_flush();

        // Slot is unlocked here
    }
}
no_debug!(KernelMapping);

impl Drop for KernelMapping {
    fn drop(&mut self) {
        // An ASC-cached M4 mapping has no qualified cache-flush teardown. Even an unexpected
        // failed construction must retain its complete node/backing instead of invoking the
        // unrelated G13/G14 FwCtl protocol.
        if self.0.uat_inner.m4_adopted && self.0.prot.is_cached_noncoherent() {
            self.0.uat_inner.failed.store(true, Ordering::Release);
            pr_err!(
                "MMU: retaining cached M4 mapping {:#x}+{:#x} until reboot\n",
                self.iova(),
                self.size()
            );
            return;
        }

        // This is the main unmap function for UAT mappings.
        // The sequence of operations here is finicky, due to the interaction
        // between cached GFX ASC mappings and the page tables. These mappings
        // always have to be flushed from the cache before being unmapped.

        // For uncached mappings, just unmapping and flushing the TLB is sufficient.

        // For cached mappings, this is the required sequence:
        // 1. Remap it as uncached
        // 2. Flush the TLB range
        // 3. If kernel VA mapping OR user VA mapping and handoff.current_slot() == slot:
        //    a. Take a lock for this slot
        //    b. Write the flush range to the right context slot in handoff area
        //    c. Issue a cache invalidation request via FwCtl queue
        //    d. Poll for completion via queue
        //    e. Check for completion flag in the handoff area
        //    f. Drop the lock
        // 4. Unmap
        // 5. Flush the TLB range again

        // Firmware that does not use the handoff protocol cannot be asked to flush its cache.
        // Such firmware only caches kernel mappings that live until its coprocessors stop.
        if self.0.prot.is_cached_noncoherent() && self.0.uat_inner.firmware == UatFirmware::Handoff
        {
            mod_pr_debug!(
                "MMU: remap as uncached {:#x}:{:#x}\n",
                self.iova(),
                self.size()
            );
            self.remap_uncached_and_flush();
        }

        // Only VM page tables/binding state are touched here. The shared
        // reservation is sufficient and avoids fallible exec allocation in Drop.
        let mut owner = self.0.owner.lock_inner();
        mod_dev_dbg!(
            owner.dev,
            "MMU: unmap {:#x}:{:#x}\n",
            self.iova(),
            self.size()
        );

        let upper_handoff = match self.0.uat_inner.lock_m4_upper(owner.is_kernel) {
            Ok(guard) => guard,
            Err(error) => {
                dev_err!(
                    owner.dev.as_ref(),
                    "MMU: retaining M4 mapping after PPL failure: {:?}\n",
                    error
                );
                return;
            }
        };
        let _mutation = owner.mapping_mutation();
        if owner.page_table.unmap_pages(self.iova_range()).is_err() {
            if self.0.uat_inner.m4_adopted {
                self.0.uat_inner.failed.store(true, Ordering::Release);
                dev_err!(
                    owner.dev.as_ref(),
                    "MMU: retaining M4 mapping after partial unmap failure\n"
                );
                return;
            }
            dev_err!(
                owner.dev.as_ref(),
                "MMU: unmap {:#x}:{:#x} failed\n",
                self.iova(),
                self.size()
            );
        }

        drop(upper_handoff);

        if let Some(asid) = owner.slot() {
            mem::sync();
            owner.tlbi_range(asid as u8, self.iova(), self.size());
            mod_dev_dbg!(
                owner.dev,
                "MMU: flush range: asid={:#x} start={:#x} len={:#x}\n",
                asid,
                self.iova(),
                self.size()
            );
            mem::sync();
        }

        // The range returns to the VA allocator once this mapping is gone, so no context that
        // names this root may keep a stale translation for it.
        owner.tlbi_contexts_naming_root(self.iova(), self.size());
        drop(_mutation);
        drop(owner);
        // SAFETY: Every normal path reaches this exactly once, after all GPUVM locks have
        // been released. Uncertain M4 paths return earlier, retaining the node and its pins.
        unsafe { ManuallyDrop::drop(&mut self.0) };
    }
}

/// Shared UAT global data structures
struct UatShared {
    kernel_ttb1: u64,
    execution_contexts: u64,
    context_generations: [u8; UAT_NUM_CTX],
    /// Firmware-observed incarnations of application roots installed by the VM slot allocator.
    binding_generations: [u64; UAT_NUM_CTX],
    binding_published: [bool; UAT_NUM_CTX],
    /// Expected lower words of user roots owned by the VM slot allocator. Replaced only by
    /// successful host publications, never learned by accepting observed hardware words.
    owned_user_roots: [u64; UAT_NUM_CTX],
    /// Kernel root of the secondary firmware instance, if there is one.
    secondary_ttb1: Option<PhysicalAddr>,
    map_kernel_to_user: bool,
    handoff_rgn: UatRegion,
    ttbs_rgn: UatRegion,
    quirks: UatQuirks,
}

impl UatShared {
    /// Takes the handoff lock, where this SoC uses it.
    fn handoff_lock(&self) {
        if self.quirks.handoff_lock {
            self.handoff().lock();
        }
    }

    /// Releases the handoff lock taken by [`Self::handoff_lock`].
    fn handoff_unlock(&self) {
        if self.quirks.handoff_lock {
            self.handoff().unlock();
        }
    }

    /// Writes back context-table slot `idx` after it changed, where this SoC needs it.
    fn publish_slot(&self, idx: usize) {
        if self.quirks.clean_tables {
            let slot = &self.ttbs()[idx] as *const SlotTTBS as *const u8;
            pgtable::clean_poc(slot, size_of::<SlotTTBS>());
            pgtable::poc_barrier();
        }
    }

    /// Returns the handoff region area
    fn handoff(&self) -> &Handoff {
        // SAFETY: pointer is non-null per the type invariant
        unsafe { (self.handoff_rgn.map.ptr() as *mut Handoff).as_ref() }.unwrap()
    }

    /// Returns the TTBAT area
    fn ttbs(&self) -> &[SlotTTBS; UAT_NUM_CTX] {
        // SAFETY: pointer is non-null per the type invariant
        unsafe { (self.ttbs_rgn.map.ptr() as *mut [SlotTTBS; UAT_NUM_CTX]).as_ref() }.unwrap()
    }

    /// Returns the mask of TTBAT contexts whose lower half is the root `ttb`.
    fn contexts_naming_root(&self, ttb: u64, adopted: bool) -> u64 {
        self.ttbs()
            .iter()
            .enumerate()
            .filter(|(ctx, slot)| {
                let expected = tagged_root(ttb, *ctx);
                let _ = adopted; // Context 0 is naturally untagged; user contexts carry their ASID.
                slot.ttb0.load(Ordering::Acquire) == expected
            })
            .fold(0, |mask, (ctx, _)| mask | (1 << ctx))
    }
}

/// Returns the TTBAT value naming root `ttb` in context `ctx`.
const fn tagged_root(ttb: u64, ctx: usize) -> u64 {
    ttb | TTBR_VALID | (ctx as u64) << TTBR_ASID_SHIFT
}

// SAFETY: Nothing here is unsafe to send across threads.
unsafe impl Send for UatShared {}

/// Inner data for the top-level UAT instance.
#[pin_data]
struct UatInner {
    firmware: UatFirmware,
    /// The stopped J713 handoff: bounded full-barrier PPL and upper words zero in users.
    m4_adopted: bool,
    /// An uncertain mapping/root is retained; future M4 allocation and publication fail.
    failed: AtomicBool,
    /// Clean page tables to the point of coherency for a walker that does not snoop the CPU
    /// caches: [`Uat::new_adopted`], and [`UatQuirks::clean_tables`] otherwise.
    clean_poc: bool,
    /// This SoC's UAT policy.
    quirks: UatQuirks,
    /// No-map reserved regions holding firmware tables (`UatQuirks::reserved_tables`).
    reserved: Option<Arc<pgtable::ReservedTables>>,
    #[pin]
    shared: Mutex<UatShared>,
    #[pin]
    handoff_flush: [Mutex<HandoffFlush>; UAT_NUM_CTX + 1],
}

/// A transport owns the sole host-side PPL mutex before touching AP interest.
struct M4Transport<'a> {
    shared: Guard<'a, UatShared, MutexBackend>,
    start: Instant<Monotonic>,
    failed: &'a AtomicBool,
}

impl ppl::Memory for M4Transport<'_> {
    fn firmware_interested(&self) -> bool {
        self.shared.handoff().lock_fw.load(Ordering::Relaxed) != 0
    }
    fn firmware_turn(&self) -> bool {
        self.shared.handoff().turn.load(Ordering::Relaxed) != 0
    }
    fn ap_interest(&self, interested: bool) {
        self.shared
            .handoff()
            .lock_ap
            .store(u8::from(interested), Ordering::Relaxed);
    }
    fn give_firmware_turn(&self) {
        self.shared.handoff().turn.store(1, Ordering::Relaxed);
    }
    fn barrier(&self) {
        mem::sync();
    }
    fn expired(&self) -> bool {
        self.start.elapsed() > Delta::from_millis(100)
    }
    fn relax(&self) {
        core::hint::spin_loop();
    }
    fn fail(&self) {
        self.failed.store(true, Ordering::Release);
    }
}

/// Every AP firmware transaction owns the same mutex, including G16 Window operations.
/// M4 drops its protocol guard before its transport releases the host mutex. The legacy
/// handoff path keeps the preexisting protocol and lock ordering for other generations.
#[must_use = "retain the guard until protected table mutations have completed"]
enum HandoffGuard<'a> {
    M4(ppl::Guard<M4Transport<'a>>),
    Legacy(Guard<'a, UatShared, MutexBackend>),
}

impl HandoffGuard<'_> {
    fn shared_mut(&mut self) -> &mut UatShared {
        match self {
            Self::M4(guard) => &mut guard.memory_mut().shared,
            Self::Legacy(shared) => shared,
        }
    }
}

impl Drop for HandoffGuard<'_> {
    fn drop(&mut self) {
        if let Self::Legacy(shared) = self {
            shared.handoff_unlock();
        }
    }
}

/// Opaque host-and-firmware lease for G16 window transactions. No caller may
/// independently manipulate the AP-interest or turn words.
#[cfg(CONFIG_DRM_ASAHI_G16)]
#[must_use = "retain the guard throughout the reserved-table window transaction"]
pub(crate) struct M4HandoffGuard<'a> {
    _transaction: HandoffGuard<'a>,
}

impl UatInner {
    /// Take the lock on the shared data and return the guard.
    fn lock(&self) -> Guard<'_, UatShared, MutexBackend> {
        self.shared.lock()
    }

    fn check_m4(&self) -> Result {
        if self.m4_adopted && self.failed.load(Ordering::Acquire) {
            Err(EIO)
        } else {
            Ok(())
        }
    }

    /// Serialize AP callers before taking the firmware exclusion. Timeouts withdraw
    /// AP interest while still holding the mutex and latch M4 failure before releasing it.
    fn lock_handoff(&self) -> Result<HandoffGuard<'_>> {
        self.check_m4()?;
        let shared = self.lock();
        self.check_m4()?;
        if !self.m4_adopted {
            shared.handoff_lock();
            return Ok(HandoffGuard::Legacy(shared));
        }
        let transport = M4Transport {
            shared,
            start: Instant::<Monotonic>::now(),
            failed: &self.failed,
        };
        match ppl::Guard::acquire(transport) {
            Ok(guard) => Ok(HandoffGuard::M4(guard)),
            Err(()) => Err(ETIMEDOUT),
        }
    }

    /// No leaf is changed on a failed acquisition. Uncertain mappings retain their backing.
    fn lock_m4_upper(&self, upper: bool) -> Result<Option<HandoffGuard<'_>>> {
        if upper && self.m4_adopted {
            self.lock_handoff().map(Some)
        } else {
            Ok(None)
        }
    }

    /// Take a lock on a handoff flush slot and return the guard.
    fn lock_flush(&self, slot: u32) -> Guard<'_, HandoffFlush, MutexBackend> {
        self.handoff_flush[slot as usize].lock()
    }
}

/// Top-level UAT manager object
pub(crate) struct Uat {
    dev: driver::AsahiDevRef,
    geometry: UatGeometry,

    inner: Arc<UatInner>,
    slots: slotalloc::SlotAllocator<SlotInner>,

    kernel_vm: Vm,
    kernel_lower_vm: Vm,
}

impl Handoff {
    /// Lock the handoff region from firmware access
    fn lock(&self) {
        self.lock_ap.store(1, Ordering::Relaxed);
        fence(Ordering::SeqCst);

        while self.lock_fw.load(Ordering::Relaxed) != 0 {
            if self.turn.load(Ordering::Relaxed) != 0 {
                self.lock_ap.store(0, Ordering::Relaxed);
                while self.turn.load(Ordering::Relaxed) != 0 {}
                self.lock_ap.store(1, Ordering::Relaxed);
                fence(Ordering::SeqCst);
            }
        }
        fence(Ordering::Acquire);
    }

    /// Unlock the handoff region, allowing firmware access
    fn unlock(&self) {
        self.turn.store(1, Ordering::Relaxed);
        self.lock_ap.store(0, Ordering::Release);
    }

    /// Returns the current Vm slot mapped by the firmware for lower/unprivileged access, if any.
    fn current_slot(&self) -> Option<u32> {
        let slot = self.cur_slot.load(Ordering::Relaxed);
        if slot == 0 || slot == u32::MAX {
            None
        } else {
            Some(slot)
        }
    }

    /// Initialize the handoff region
    fn init(&self, firmware: UatFirmware, slot: u32) -> Result {
        self.magic_ap.store(PPL_MAGIC, Ordering::Relaxed);
        self.cur_slot.store(slot, Ordering::Relaxed);
        self.unk3.store(0, Ordering::Relaxed);
        fence(Ordering::SeqCst);

        if firmware == UatFirmware::Handoff {
            self.wait_for_firmware()?;
        }

        for i in 0..=UAT_NUM_CTX {
            self.flush[i].state.store(0, Ordering::Relaxed);
            self.flush[i].addr.store(0, Ordering::Relaxed);
            self.flush[i].size.store(0, Ordering::Relaxed);
        }
        fence(Ordering::SeqCst);
        Ok(())
    }

    /// Wait for the firmware to publish its half of the handoff region.
    fn wait_for_firmware(&self) -> Result {
        let start = Instant::<Monotonic>::now();
        const TIMEOUT: Delta = Delta::from_millis(1000);

        self.lock();
        while start.elapsed() < TIMEOUT {
            if self.magic_fw.load(Ordering::Relaxed) == PPL_MAGIC {
                break;
            } else {
                self.unlock();
                fsleep(Delta::from_millis(10));
                self.lock();
            }
        }

        if self.magic_fw.load(Ordering::Relaxed) != PPL_MAGIC {
            self.unlock();
            pr_err!("Handoff: Failed to initialize (firmware not running?)\n");
            return Err(EIO);
        }

        self.unlock();
        Ok(())
    }
}

/// Represents a single flush info slot in the handoff region.
///
/// # Invariants
/// The pointer is valid and there is no aliasing HandoffFlush instance.
struct HandoffFlush(*const FlushInfo);

// SAFETY: These pointers are safe to send across threads.
unsafe impl Send for HandoffFlush {}

impl HandoffFlush {
    /// Set up a flush operation for the coprocessor
    fn begin_flush(&self, start: u64, size: u64) {
        // SAFETY: Per the type invariant, this is safe
        let flush = unsafe { self.0.as_ref().unwrap() };

        let state = flush.state.load(Ordering::Relaxed);
        if state != 0 {
            pr_err!("Handoff: expected flush state 0, got {}\n", state);
        }
        flush.addr.store(start, Ordering::Relaxed);
        flush.size.store(size, Ordering::Relaxed);
        flush.state.store(1, Ordering::Relaxed);
    }

    /// Complete a flush operation for the coprocessor
    fn end_flush(&self) {
        // SAFETY: Per the type invariant, this is safe
        let flush = unsafe { self.0.as_ref().unwrap() };
        let state = flush.state.load(Ordering::Relaxed);
        if state != 2 {
            pr_err!("Handoff: expected flush state 2, got {}\n", state);
        }
        flush.state.store(0, Ordering::Relaxed);
    }
}

fn prefetch_sgt_segments(
    dev: &device::Device<Bound>,
    gem: &gem::Object,
) -> Result<SgSegments> {
    let sgt = crate::gem::held_sg_table(gem, dev)?;

    let mut segments = KVVec::with_capacity(sgt.iter().size_hint().0, GFP_KERNEL)?;
    for sgt_entry in sgt.iter() {
        segments.push((sgt_entry.dma_address(), sgt_entry.dma_len()), GFP_KERNEL)?;
    }

    Ok(SgSegments { segments, _lease: sgt.into_lease() })
}

/// Reuse cached segments without allocating another copy for each bind. The bind lease was
/// acquired before BO locking: its getter synchronized with any earlier reclaim under resv,
/// and its extra count excludes new eviction until preparation/commit finishes.
fn prepare_sgt_segments(
    dev: &device::Device<Bound>,
    gem: &gem::Object,
    bo: &gpuvm::GpuVmBo<VmInner>,
    _bind_lease: &kernel::drm::gem::shmem::SGLease<gem::AsahiObject>,
) -> Result {
    let needs_segments = bo.data().segments.lock().is_none();
    if needs_segments {
        // Never enter the object reservation while holding the BO mutex.
        let prefetched = prefetch_sgt_segments(dev, gem)?;
        let mut segments = bo.data().segments.lock();
        if segments.is_none() {
            segments.replace(prefetched);
        }
    }
    Ok(())
}

impl Vm {
    /// Create a new virtual memory address space
    fn new(
        dev: driver::AsahiDevice,
        uat_inner: Arc<UatInner>,
        kernel_range: Range<u64>,
        geometry: UatGeometry,
        ttb: Option<PhysicalAddr>,
        id: u64,
    ) -> Result<Vm> {
        let dummy_obj = gem::new_kernel_object(&dev, UAT_PGSZ)?;
        let is_kernel = ttb.is_some();

        let page_table = if let Some(ttb) = ttb {
            UatPageTable::new_with_ttb(
                ttb,
                geometry.kernel_range(),
                geometry.ias,
                geometry.oas,
                uat_inner.firmware != UatFirmware::Handoff,
                uat_inner.reserved.clone(),
            )?
        } else {
            UatPageTable::new(geometry.ias, geometry.oas)?
                .with_native_gpu(uat_inner.quirks.native_gpu_permissions)
        };
        let page_table = {
            let mut page_table = page_table;
            if uat_inner.clean_poc {
                page_table.enable_clean_poc()?;
            }
            page_table
        };

        let (va_range, gpuvm_range) = if is_kernel {
            (geometry.kernel_range(), kernel_range.clone())
        } else {
            let mut range = geometry.user_range();
            if uat_inner.firmware != UatFirmware::Handoff {
                range.start = 0;
            }
            (range, geometry.user_usable_range())
        };

        let mm = mm::Allocator::new(va_range.start, va_range.range(), ())?;

        let binding = Arc::pin_init(
            new_mutex!(
                VmBinding {
                    binding: None,
                    bind_token: None,
                    generation: 0,
                    active_users: 0,
                    ttb: page_table.ttb(),
                },
                "VmBinding",
            ),
            GFP_KERNEL,
        )?;

        let mapping_epoch = if uat_inner.firmware != UatFirmware::Handoff {
            Some(Arc::new(MappingEpoch::new(), GFP_KERNEL)?)
        } else {
            None
        };
        let bind_sequence = Arc::new(AtomicU64::new(0), GFP_KERNEL)?;
        let unique_gpuvm = gpuvm::GpuVm::new::<Error, _>(
            c"Asahi::GpuVm",
            &dev,
            &*dummy_obj.gem,
            gpuvm_range,
            kernel_range,
            VmInner {
                dev: dev.clone(),
                is_kernel,
                va_range: va_range.clone(),
                page_table,
                mapping_epoch: mapping_epoch.clone(),
                bind_sequence: bind_sequence.clone(),
                epoch: Some(0),
                mm,
                uat_inner,
                binding: binding.clone(),
                id,
            },
        )?;
        let gpuvm = ARef::from(&*unique_gpuvm);

        Ok(Vm {
            dev: dev.as_ref().as_ref().into(),
            id,
            gpuvm,
            mapping_epoch,
            bind_sequence,
            lifetime: None,
            residency: None,
            context_bindings: None,
            shared_bindings: None,
            status: None,
            dummy_obj: dummy_obj.gem.clone(),
            binding,
        })
    }

    /// Attach a status before sharing a newly created user VM with queues.
    pub(crate) fn with_status(mut self) -> Result<Self> {
        self.status = Some(Arc::pin_init(
            crate::g17::status::VmStatus::new(),
            GFP_KERNEL,
        )?);
        self.lifetime = Some(lifetime::VmLifetime::new()?);
        self.residency = Some(residency::Gate::new()?);
        self.context_bindings = Some(context::ContextBindings::new()?);
        self.shared_bindings = Some(shared::SharedBindings::new()?);
        Ok(self)
    }

    /// Attach a status and job lifetime tracking before sharing a newly created user VM with
    /// queues, without the G17 canonical context views, shared-object footprints and idle
    /// residency reclaim of [`Self::with_status`]. Used by G15, whose user objects stay pinned.
    #[cfg(CONFIG_DRM_ASAHI_UAT_ADOPTION)]
    pub(crate) fn with_tracked_status(mut self) -> Result<Self> {
        self.status = Some(Arc::pin_init(
            crate::g17::status::VmStatus::new(),
            GFP_KERNEL,
        )?);
        self.lifetime = Some(lifetime::VmLifetime::new()?);
        Ok(self)
    }

    /// The identifier this VM was created with (G15 keeps per-VM state keyed by it; G17 timeout
    /// attribution compares and logs VMs by it).
    pub(crate) fn id(&self) -> u64 {
        self.id
    }

    /// Copies `buf.len()` bytes of GPU-readable memory at `va` of this VM into `buf`, for admission
    /// checks of streams the GPU will execute (G15 stream gates).
    ///
    /// Every page must be mapped GPU-readable and be RAM; otherwise EFAULT. The lines are cleaned
    /// and invalidated to the point of coherency first (never only invalidated: CPU-written lines
    /// may be dirty), so the copy sees what the GPU or a write-combined CPU view stored. The
    /// mappings cannot change during the copy (GPUVM exec lock).
    #[cfg(CONFIG_DRM_ASAHI_UAT_ADOPTION)]
    pub(crate) fn read_user(&self, va: u64, buf: &mut [u8]) -> Result {
        self.read_lock()?.read_user(va, buf)
    }

    #[cfg(CONFIG_DRM_ASAHI_UAT_ADOPTION)]
    pub(crate) fn read_lock(&self) -> Result<VmReadGuard> {
        Ok(VmReadGuard(lock_vm(&self.gpuvm, None, false)?))
    }

    #[cfg(CONFIG_DRM_ASAHI_UAT_ADOPTION)]
    fn read_user_locked(inner: &mut VmInner, va: u64, buf: &mut [u8]) -> Result {
        let mut done = 0usize;
        while done < buf.len() {
            let address = va.checked_add(done as u64).ok_or(EFAULT)?;
            let page = address & !(UAT_PGMSK as u64);
            let within = (address - page) as usize;
            let count = (buf.len() - done).min(UAT_PGSZ - within);
            if page < inner.va_range.start || page >= inner.va_range.end {
                return Err(EFAULT);
            }
            let leaf = inner.page_table.leaf_entry(page).map_err(|_| EFAULT)?;
            if !Prot::from_pte(leaf).allows_gpu(true, false) {
                return Err(EFAULT);
            }
            let phys = inner.page_table.translate_iova(page).map_err(|_| EFAULT)?;
            // SAFETY: `phys` is checked to be a RAM page; it stays mapped (and its backing pinned)
            // while the exec lock is held, and is only read here.
            let ram = unsafe { kernel::page::Page::borrow_phys(&phys) }.ok_or(EFAULT)?;
            let target = &mut buf[done..done + count];
            ram.with_pointer_into_page(within, count, |p| {
                pgtable::clean_invalidate_poc(p, count);
                pgtable::poc_barrier();
                // SAFETY: with_pointer_into_page() checked that `count` bytes at `p` are inside the
                // page, and `target` holds exactly `count` bytes.
                unsafe { core::ptr::copy_nonoverlapping(p, target.as_mut_ptr(), count) };
                Ok(())
            })?;
            done += count;
        }
        Ok(())
    }

    /// Status shared by every queue using this VM, if enabled by its GPU.
    pub(crate) fn status(&self) -> Option<&Arc<crate::g17::status::VmStatus>> {
        self.status.as_ref()
    }

    fn covers_range_locked(
        inner: &mut VmInner,
        address: u64,
        size: u64,
        read: bool,
        write: bool,
    ) -> bool {
        if size == 0 {
            return false;
        }
        let Some(end) = address.checked_add(size) else {
            return false;
        };
        let mask = UAT_PGMSK as u64;
        let start = address & !mask;
        let end = if end & mask == 0 {
            end
        } else {
            let Some(end) = (end | mask).checked_add(1) else {
                return false;
            };
            end
        };
        if start < inner.va_range.start || end > inner.va_range.end {
            return false;
        }
        inner
            .page_table
            .covers_range(start..end, read, write)
            .unwrap_or(false)
    }

    pub(crate) fn covers_range(&self, address: u64, size: u64, read: bool, write: bool) -> bool {
        let Ok(mut inner) = lock_vm(&self.gpuvm, None, false) else {
            return false;
        };
        Self::covers_range_locked(&mut inner, address, size, read, write)
    }

    /// Checks all writable sparse blocks and their guard coverage under one
    /// GPUVM lock. A fully mapped guard is not a valid sparse allocation.
    pub(crate) fn covers_sparse_blocks_with_guards(
        &self,
        addresses: &[u64],
        block_size: u64,
        stride: u64,
    ) -> bool {
        if stride <= block_size {
            return false;
        }
        let Ok(mut inner) = lock_vm(&self.gpuvm, None, false) else {
            return false;
        };
        addresses.iter().all(|&address| {
            let Some(guard) = address.checked_add(block_size) else {
                return false;
            };
            Self::covers_range_locked(&mut inner, address, block_size, true, true)
                && !Self::covers_range_locked(&mut inner, guard, stride - block_size, false, false)
        })
    }

    /// Checks all exact request tuples under one GPUVM exec lock. Access bit0
    /// requests reading and bit1 writing. The epoch names this same snapshot.
    pub(crate) fn covers_ranges_batch(&self, ranges: &[(u64, u64, u8)]) -> (bool, Option<u64>) {
        let Ok(mut inner) = lock_vm(&self.gpuvm, None, false) else {
            return (false, None);
        };
        let epoch = inner.epoch;
        let covered = ranges.iter().all(|&(address, size, access)| {
            Self::covers_range_locked(&mut inner, address, size, access & 1 != 0, access & 2 != 0)
        });
        (covered, epoch)
    }

    /// Returns None while a leaf mutation is in progress or after epoch overflow.
    pub(crate) fn mapping_validation_epoch(&self) -> Option<u64> {
        self.mapping_epoch.as_ref()?.snapshot()
    }

    fn current_bind_sequence(&self) -> u64 {
        self.bind_sequence.load(Ordering::Acquire)
    }

    /// Get the translation table base for this Vm
    fn ttb(&self) -> u64 {
        self.binding.lock().ttb
    }

    /// Map a GEM object (using its `SGTable`) into this Vm at a free address in a given range.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn map_in_range(
        &self,
        gem: &gem::Object,
        object_range: Range<usize>,
        alignment: u64,
        range: Range<u64>,
        prot: Prot,
        guard: bool,
    ) -> Result<KernelMapping> {
        self.map_in_range_with_guard_size(
            gem,
            object_range,
            alignment,
            range,
            prot,
            if guard { UAT_PGSZ } else { 0 },
        )
    }

    /// Reserves a whole-page unmapped guard without adding it to the mapped backing.
    pub(crate) fn map_in_range_with_guard_size(
        &self,
        gem: &gem::Object,
        object_range: Range<usize>,
        alignment: u64,
        range: Range<u64>,
        prot: Prot,
        guard_size: usize,
    ) -> Result<KernelMapping> {
        self.map_in_range_with_mode(
            gem, object_range, alignment, range, prot, guard_size, mm::InsertMode::Best,
        )
    }

    /// Lowest-address placement for a monotonic search across several VMs.
    pub(crate) fn map_in_range_lowest(
        &self,
        gem: &gem::Object,
        object_range: Range<usize>,
        alignment: u64,
        range: Range<u64>,
        prot: Prot,
        guard_size: usize,
    ) -> Result<KernelMapping> {
        self.map_in_range_with_mode(
            gem, object_range, alignment, range, prot, guard_size, mm::InsertMode::Low,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn map_in_range_with_mode(
        &self,
        gem: &gem::Object,
        object_range: Range<usize>,
        alignment: u64,
        range: Range<u64>,
        prot: Prot,
        guard_size: usize,
        mode: mm::InsertMode,
    ) -> Result<KernelMapping> {
        use kernel::drm::gem::BaseObject;
        if guard_size % UAT_PGSZ != 0 {
            return Err(EINVAL);
        }
        let size = object_range
            .end
            .checked_sub(object_range.start)
            .ok_or(EINVAL)?;
        if size == 0 || object_range.end > gem.size() {
            return Err(EINVAL);
        }
        let reserved_size = size.checked_add(guard_size).ok_or(EOVERFLOW)?;
        let _residency = self.enter_residency()?;
        // Pin and copy backing before taking BO or DMA reservation locks.
        let dev = unsafe { self.dev.as_bound() };
        let _bind_lease = crate::gem::held_sg_table(gem, dev)?.into_lease();
        let vm_bo = self
            .gpuvm
            .obtain(gem, pin_init!(VmBoData {
                newest_bind: AtomicU64::new(0),
                segments <- new_mutex!(None),
            }))?;
        prepare_sgt_segments(dev, gem, &vm_bo, &_bind_lease)?;

        let size = object_range.range();
        let mut vm_exec = unsafe {
            self.gpuvm.exec_lock(/*Some(gem)*/ None, false)?
        };
        let inner = vm_exec.unique_gpuvm.data();

        let mut vm_bo = Some(MappingBo::new(vm_bo, self.gpuvm.clone()));
        let uat_inner = inner.uat_inner.clone();
        // A failed reservation drops its payload while exec still owns GEM
        // reservations. Attach BO/GEM references only after success, since
        // their destructors may acquire the same reservation locks.
        let result = inner.mm.insert_node_in_range(
            KernelMappingInner {
                owner: self.gpuvm.clone(),
                uat_inner,
                prot,
                bo: None,
                _gem: None,
                offset: object_range.start,
                mapped_size: size,
            },
            reserved_size as u64,
            alignment,
            0,
            range.start,
            range.end,
            mode,
        );
        let mut node = match result {
            Ok(node) => node,
            Err(error) => {
                drop(vm_exec);
                return Err(error);
            }
        };
        {
            let payload = node.as_mut().inner_mut();
            payload.bo = vm_bo.take();
            payload._gem = Some(gem::KernelMappingPin::new(gem));
        }

        let ret = inner.map_node(&node, prot);
        // Drop the exec_lock first, so that if map_node failed the
        // KernelMappingInner destructur does not deadlock.
        let retain = ret.is_err()
            && inner.uat_inner.m4_adopted
            && inner.uat_inner.failed.load(Ordering::Acquire);
        core::mem::drop(vm_exec);
        if retain {
            core::mem::forget(node);
            return Err(ret.err().unwrap_or(EIO));
        }
        ret?;
        Ok(KernelMapping(ManuallyDrop::new(node)))
    }

    /// Map a GEM object into this Vm at a specific address.
    #[allow(clippy::too_many_arguments)]
    #[inline(never)]
    pub(crate) fn map_at(
        &self,
        addr: u64,
        size: usize,
        gem: &gem::Object,
        prot: Prot,
        guard: bool,
    ) -> Result<KernelMapping> {
        let reserved_size = size
            .checked_add(if guard { UAT_PGSZ } else { 0 })
            .ok_or(EOVERFLOW)?;
        let _residency = self.enter_residency()?;
        // Pin and copy backing before taking BO or DMA reservation locks.
        let dev = unsafe { self.dev.as_bound() };
        let _bind_lease = crate::gem::held_sg_table(gem, dev)?.into_lease();
        let vm_bo = self
            .gpuvm
            .obtain(&gem, pin_init!(VmBoData {
                newest_bind: AtomicU64::new(0),
                segments <- new_mutex!(None),
            }))?;
        prepare_sgt_segments(dev, gem, &vm_bo, &_bind_lease)?;

        // SAFETY: no other UniqRefGpuVm exists so `GpuVm::exec_lock()` is safe
        let mut vm_exec = unsafe {
            self.gpuvm
                .exec_lock(/*Some(&gem)*/ None, false)
                .inspect_err(|_| {
                    dev_err!(self.dev, "map_at(): Failed to exec_lock gpuvm\n");
                })?
        };
        let inner = vm_exec.unique_gpuvm.data();

        let mut vm_bo = Some(MappingBo::new(vm_bo, self.gpuvm.clone()));
        let uat_inner = inner.uat_inner.clone();
        // Keep reservation-locking destructors out of the allocator payload
        // until reservation succeeds.
        let result = inner.mm.reserve_node(
            KernelMappingInner {
                owner: self.gpuvm.clone(),
                uat_inner,
                prot,
                bo: None,
                _gem: None,
                offset: 0,
                mapped_size: size,
            },
            addr,
            reserved_size as u64,
            0,
        );
        let mut node = match result {
            Ok(node) => node,
            Err(error) => {
                drop(vm_exec);
                return Err(error);
            }
        };
        {
            let payload = node.as_mut().inner_mut();
            payload.bo = vm_bo.take();
            payload._gem = Some(gem::KernelMappingPin::new(gem));
        }

        let ret = inner.map_node(&node, prot);
        // Drop the exec_lock first, so that if map_node failed the
        // KernelMappingInner destructur does not deadlock.
        let retain = ret.is_err()
            && inner.uat_inner.m4_adopted
            && inner.uat_inner.failed.load(Ordering::Acquire);
        core::mem::drop(vm_exec);
        if retain {
            core::mem::forget(node);
            return Err(ret.err().unwrap_or(EIO));
        }
        ret?;
        Ok(KernelMapping(ManuallyDrop::new(node)))
    }

    /// Map a range of a GEM object into this Vm using GPUVM.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn bind_object<'drm>(
        &self,
        device: &'drm AsahiDrmDevice<Registered>,
        gem: &gem::Object,
        addr: u64,
        size: u64,
        offset: u64,
        prot: Prot,
        single_page: bool,
    ) -> Result {
        let _residency = self.enter_residency_metadata()?;
        gem::validate_vm_binding(gem, self)?;
        self.wait_for_user_map_admission()?;
        // Pin and copy backing before taking BO or DMA reservation locks.
        let dev = unsafe { self.dev.as_bound() };
        let _bind_lease = crate::gem::held_sg_table(gem, dev)?.into_lease();
        let vm_bo = self
            .gpuvm
            .obtain(gem, pin_init!(VmBoData {
                newest_bind: AtomicU64::new(0),
                segments <- new_mutex!(None),
            }))?;
        prepare_sgt_segments(dev, gem, &vm_bo, &_bind_lease)?;

        let mut resources = VmOpResources {
            preallocated_gpuvas: [
                Some(GpuVaAlloc::<VmInner>::new(GFP_KERNEL)?),
                Some(GpuVaAlloc::<VmInner>::new(GFP_KERNEL)?),
                Some(GpuVaAlloc::<VmInner>::new(GFP_KERNEL)?),
            ],
        };

        // Mapping needs a complete context
        let mut ctx = StepContext::new(Some(device.as_ref().as_ref()), prot, &mut resources)?;

        let mut vm_exec = unsafe {
            self.gpuvm.exec_lock(/*Some(&gem)*/ None, false)?
        };
        let inner = vm_exec.unique_gpuvm.data();

        // Preallocate the page tables, to fail early if we ENOMEM
        inner.page_table.alloc_pages(addr..(addr + size))?;

        if (addr | size | offset) & (UAT_PGMSK as u64) != 0 {
            dev_err!(
                inner.dev.as_ref(),
                "MMU: Map step {:#x} [{:#x}] -> {:#x} is not page-aligned\n",
                offset,
                size,
                addr
            );
            return Err(EINVAL);
        }

        let (flags, gem_range) = if single_page {
            ((GpuVaFlag::REPEAT).into(), UAT_PGSZ as u32)
        } else {
            (GpuVaFlags::empty(), 0u32)
        };
        let _gem_range = 0u32;

        mod_dev_dbg!(
            inner.dev,
            "MMU: sm_map: {:#x} [{:#x}] -> {:#x}\n",
            offset,
            size,
            addr
        );
        let _commit = self.mapping_commit()?;

        let req = OpMapRequest {
            addr,
            range: size,
            gem_offset: offset,
            gem_range,
            vm_bo: &vm_bo,
            context: &mut ctx,
            flags,
        };
        vm_exec.unique_gpuvm.sm_map(req)
    }

    /// Spend the binding generation only for an incarnation exposed to firmware.
    #[cfg(CONFIG_DRM_ASAHI_G16)]
    pub(crate) fn mark_binding_published(&self) -> Result {
        let uat = self.gpuvm.lock_inner().uat_inner.clone();
        let binding = self.binding.lock();
        let slot = binding.binding.as_ref().ok_or(EIO)?.slot() as usize + UAT_USER_CTX_START;
        let mut shared = uat.lock();
        if shared.owned_user_roots[slot] != tagged_root(binding.ttb, slot)
            || shared.ttbs()[slot].ttb0.load(Ordering::Acquire) != shared.owned_user_roots[slot]
        {
            return Err(EIO);
        }
        shared.binding_published[slot] = true;
        Ok(())
    }

    /// Add a direct MMIO mapping to this Vm at a free address.
    pub(crate) fn map_io(
        &self,
        dev: &device::Device,
        iova: u64,
        phys: usize,
        size: usize,
        prot: Prot,
    ) -> Result<KernelMapping> {
        let mut vm_exec = unsafe { self.gpuvm.exec_lock(None, false)? };

        if (iova as usize | phys | size) & UAT_PGMSK != 0 {
            dev_err!(
                dev,
                "MMU: KernelMapping {:#x}:{:#x} -> {:#x} is not page-aligned\n",
                phys,
                size,
                iova
            );
            return Err(EINVAL);
        }

        dev_info!(dev, "MMU: IO map: {:#x}:{:#x} -> {:#x}\n", phys, size, iova);

        let inner = vm_exec.unique_gpuvm.data();
        let uat_inner = inner.uat_inner.clone();
        let node = inner.mm.reserve_node(
            KernelMappingInner {
                owner: self.gpuvm.clone(),
                uat_inner,
                prot,
                bo: None,
                _gem: None,
                offset: 0,
                mapped_size: size,
            },
            iova,
            size as u64,
            0,
        )?;

        let end = iova.checked_add(size as u64).ok_or(EOVERFLOW)?;
        inner.page_table.prepare_map(iova..end)?;
        let transaction_uat = inner.uat_inner.clone();
        let handoff = transaction_uat.lock_m4_upper(inner.is_kernel)?;
        let mutation = inner.mapping_mutation();
        let ret = inner
            .page_table
            .map_pages(iova..end, phys as PhysicalAddr, prot, false);
        let discard = if ret.is_err() {
            inner.page_table.discard_partial_map(iova..end)
        } else {
            Ok(())
        };
        drop(handoff);
        if ret.is_err() {
            inner.finish_failed_map(iova, size);
        } else {
            inner.tlbi_contexts_naming_root(iova, size);
        }
        let retain = discard.is_err() && inner.uat_inner.m4_adopted;
        if retain {
            inner.uat_inner.failed.store(true, Ordering::Release);
        }
        drop(mutation);
        drop(vm_exec);
        if retain {
            core::mem::forget(node);
            return Err(ret.err().unwrap_or(EIO));
        }
        ret?;
        Ok(KernelMapping(ManuallyDrop::new(node)))
    }

    /// Unmap everything in an address range.
    pub(crate) fn unmap_range<'drm>(
        &self,
        dev: Option<&'drm device::Device<Bound>>,
        iova: u64,
        size: u64,
    ) -> Result {
        if self.defer_unmap(iova, size)? {
            return Ok(());
        }
        self.unmap_range_commit(dev, iova, size, true)
    }

    fn unmap_prepared_cleanup(&self, unmap: &mut bind::PreparedUserUnmap) -> Result {
        let mut inner = self.gpuvm.lock_inner();
        let mut ctx = StepContext::new(None, Prot::default(), &mut unmap.resources)?;
        // SAFETY: step_unmap/step_remap modify this VM's translations and
        // immediate GPUVA metadata only, using the retained split nodes. They
        // do not access external BO backing and defer all BO reference puts.
        let result = unsafe { inner.sm_unmap_inner(unmap.iova, unmap.size, &mut ctx) };
        if result.is_ok() {
            self.untrack_shared_range(unmap.iova, unmap.size);
        }
        drop(inner);
        self.bo_deferred_cleanup();
        result
    }

    fn unmap_range_commit<'drm>(
        &self,
        dev: Option<&'drm device::Device<Bound>>,
        iova: u64,
        size: u64,
        user: bool,
    ) -> Result {
        let _residency = self.enter_retirement_residency();
        let mut resources = VmOpResources::for_unmap()?;
        let mut ctx = StepContext::new(dev, Prot::default(), &mut resources)?;

        let mut vm_exec = unsafe { self.gpuvm.exec_lock(None, false)? };

        mod_dev_dbg!(self.dev, "MMU: sm_unmap: {:#x}:{:#x}\n", iova, size);
        let _commit = if user {
            match self.mapping_commit() {
                Ok(guard) => guard,
                Err(error) => {
                    drop(vm_exec);
                    if self.defer_unmap(iova, size)? {
                        return Ok(());
                    }
                    return Err(error);
                }
            }
        } else {
            None
        };
        vm_exec.unique_gpuvm.sm_unmap(iova, size, &mut ctx)?;
        self.untrack_shared_range(iova, size);
        Ok(())
    }

    fn drop_mappings_now(&self, gem: &gem::Object) -> Result {
        let _residency = self.enter_retirement_residency();
        // Removing whole mappings only does unmaps, so no preallocated VAs
        let mut resources = VmOpResources {
            preallocated_gpuvas: [None, None, None],
        };
        let mut ctx = StepContext::new(None, Prot::default(), &mut resources)?;

        // SAFETY: See `lock_vm()`.
        let mut vm_exec = match unsafe { self.gpuvm.exec_lock(None, false) } {
            Err(error) if error == ENOMEM => return self.drop_mappings_low_memory(gem),
            result => result?,
        };

        if let Some(bo) = vm_exec.unique_gpuvm.find_bo(gem) {
            mod_dev_dbg!(self.dev, "MMU: bo_unmap\n");
            let result = vm_exec.unique_gpuvm.bo_unmap(&mut ctx, &bo);
            if result.is_ok() {
                // The close callback or deferred object list retains this GEM,
                // so clearing its candidate cannot release its final reference.
                self.untrack_context_object(gem);
                self.untrack_shared_object(gem);
            }
            mod_dev_dbg!(self.dev, "MMU: bo_unmap done\n");
            // BO release can drain unrelated deferred GEMs, including on an
            // unmap error. Always release reservations before dropping it.
            core::mem::drop(vm_exec);
            core::mem::drop(bo);
            if result == Err(ENOMEM) {
                return self.drop_mappings_low_memory(gem);
            }
            result?;
        }
        Ok(())
    }

    fn drop_mappings_before(&self, gem: &gem::Object, sequence: u64) -> Result {
        let _residency = self.enter_retirement_residency();
        // Removing whole mappings only does unmaps, so no preallocated VAs
        // are needed for the indexed path.
        let mut resources = VmOpResources {
            preallocated_gpuvas: [None, None, None],
        };
        let mut ctx = StepContext::new(None, Prot::default(), &mut resources)?;

        // The BO's newest sequence is checked under the same lock as binds. If
        // it is not newer than the close, every mapping belongs to that close
        // and the per-object list is safe and substantially cheaper to walk.
        let mut vm_exec = match unsafe { self.gpuvm.exec_lock(None, false) } {
            Err(error) if error == ENOMEM => {
                return self.drop_mappings_filtered(gem, sequence);
            }
            result => result?,
        };
        let Some(bo) = vm_exec.unique_gpuvm.find_bo(gem) else {
            return Ok(());
        };
        if bo.data().newest_bind.load(Ordering::Relaxed) > sequence {
            drop(vm_exec);
            drop(bo);
            return self.drop_mappings_filtered(gem, sequence);
        }

        mod_dev_dbg!(self.dev, "MMU: deferred bo_unmap\n");
        let result = vm_exec.unique_gpuvm.bo_unmap(&mut ctx, &bo);
        if result.is_ok() {
            self.untrack_context_object_before(gem, sequence);
            self.untrack_shared_object_before(gem, sequence);
        }
        drop(vm_exec);
        drop(bo);
        if result == Err(ENOMEM) {
            return self.drop_mappings_filtered(gem, sequence);
        }
        result
    }

    /// A newer binding shares the BO list, so select the closed generation by
    /// walking the VM without allocating. This is also the ENOMEM fallback.
    fn drop_mappings_filtered(&self, gem: &gem::Object, sequence: u64) -> Result {
        let mut inner = self.gpuvm.lock_inner();
        let result = inner.unmap_object_where(
            gem,
            |va| va.data().bind_sequence <= sequence,
            VmInner::unmap_gpuva,
        );
        if result.is_ok() {
            self.untrack_context_object_before(gem, sequence);
            self.untrack_shared_object_before(gem, sequence);
        }
        drop(inner);
        self.bo_deferred_cleanup();
        result
    }

    /// Preserve the indexed fast path normally; memory pressure must not lose
    /// accepted object cleanup just because an exec or operation list failed.
    fn drop_mappings_low_memory(&self, gem: &gem::Object) -> Result {
        let mut inner = self.gpuvm.lock_inner();
        let result = inner.unmap_object(gem, VmInner::unmap_gpuva);
        if result.is_ok() {
            self.untrack_context_object(gem);
            self.untrack_shared_object(gem);
        }
        drop(inner);
        self.bo_deferred_cleanup();
        result
    }

    /// Returns the dummy GEM object used to hold the shared DMA reservation locks
    pub(crate) fn get_resv_obj(&self) -> ARef<gem::Object> {
        self.dummy_obj.clone()
    }

    /// Check whether an object is external to this GpuVm
    pub(crate) fn is_extobj(&self, gem: &gem::Object) -> bool {
        self.gpuvm.is_extobj(gem)
    }

    /// Perform deferred cleanup for this GpuVm
    pub(crate) fn bo_deferred_cleanup(&self) {
        self.gpuvm.deferred_cleanup()
    }
}

impl Drop for VmInner {
    fn drop(&mut self) {
        let mut binding = self.binding.lock();
        assert_eq!(binding.active_users, 0);

        mod_pr_debug!(
            "VmInner::Drop [{}]: bind_token={:?}\n",
            self.id,
            binding.bind_token
        );

        // Make sure this VM is not mapped to a TTB if it was
        if let Some(token) = binding.bind_token.take() {
            let idx = (token.last_slot() as usize) + UAT_USER_CTX_START;
            let ttb = self.ttb() | TTBR_VALID | (idx as u64) << TTBR_ASID_SHIFT;

            let mut transaction = match self.uat_inner.lock_handoff() {
                Ok(guard) => guard,
                Err(_) => {
                    self.page_table.retain();
                    pr_err!(
                        "MMU: retaining M4 VM {} page tables after PPL failure\n",
                        self.id
                    );
                    return;
                }
            };
            let uat_inner = transaction.shared_mut();
            let handoff_cur = uat_inner.handoff().current_slot();
            let ttb_cur = uat_inner.ttbs()[idx].ttb0.load(Ordering::SeqCst);
            let inval = ttb_cur == ttb;
            if inval {
                if handoff_cur == Some(idx as u32) {
                    pr_err!(
                        "VmInner::drop owning slot {}, but it is currently in use by the ASC?\n",
                        idx
                    );
                }
                uat_inner.ttbs()[idx].ttb0.store(0, Ordering::SeqCst);
                uat_inner.ttbs()[idx].ttb1.store(0, Ordering::SeqCst);
                uat_inner.owned_user_roots[idx] = 0;
                uat_inner.publish_slot(idx);
            }
            drop(transaction);

            // In principle we dropped all the KernelMappings already, but we might as
            // well play it safe and invalidate the whole ASID.
            if inval {
                mod_pr_debug!(
                    "VmInner::Drop [{}]: need inval for ASID {:#x}\n",
                    self.id,
                    idx
                );
                mem::sync();
                mem::tlbi_asid(idx as u8);
                mem::sync();
            }
        }
    }
}

/// Physical regions of a UAT whose handoff the generation has already established itself
/// ([`Uat::new_adopted`]). None of them needs to be in the kernel linear map.
#[cfg(CONFIG_DRM_ASAHI_UAT_ADOPTION)]
pub(crate) struct AdoptedRegions {
    /// The handoff region (PPL magic, lock and flush requests).
    pub(crate) handoff: Range<PhysicalAddr>,
    /// The TTBAT.
    pub(crate) ttbs: Range<PhysicalAddr>,
    /// The firmware kernel (upper) root, which every user context names as its upper half.
    pub(crate) kernel_root: PhysicalAddr,
}

impl Uat {
    /// Common AP and firmware exclusion for G16's reserved-table window. Acquire
    /// immutable root addresses before this guard, preserving binding -> UAT lock order.
    #[cfg(CONFIG_DRM_ASAHI_G16)]
    pub(crate) fn lock_m4_handoff(&self) -> Result<M4HandoffGuard<'_>> {
        if !self.inner.m4_adopted {
            return Err(EINVAL);
        }
        Ok(M4HandoffGuard {
            _transaction: self.inner.lock_handoff()?,
        })
    }

    #[cfg(CONFIG_DRM_ASAHI_UAT_ADOPTION)]
    /// Map a region given by its physical range, write-combined.
    fn map_physical(range: &Range<PhysicalAddr>, min_size: usize) -> Result<UatRegion> {
        let size: usize = (range.end.checked_sub(range.start).ok_or(EINVAL)?).try_into()?;
        if size < min_size || (range.start | range.end) & (UAT_PGMSK as PhysicalAddr) != 0 {
            return Err(EINVAL);
        }
        // SAFETY: All-zero is a valid `struct resource` (no name, parent or children).
        let mut raw: kernel::bindings::resource = unsafe { core::mem::zeroed() };
        raw.start = range.start;
        raw.end = range.end - 1;
        raw.flags = kernel::bindings::IORESOURCE_MEM as _;
        // SAFETY: `Resource` is a transparent wrapper around `Opaque<bindings::resource>`, and
        // `raw` describes a valid memory range.
        let res: io::resource::Resource =
            unsafe { core::mem::transmute(kernel::types::Opaque::new(raw)) };
        // A carveout outside System RAM is mapped write-combined; a reserved-memory region
        // inside System RAM (mapped by the kernel) through the kernel's own mapping, as
        // `map_region` does for the other generations.
        let mode = crate::g15::memory_mode(range.start, size as u64)?;
        let ram = mode == kernel::bindings::MEMREMAP_WB as u32;
        let flags = if ram {
            io::mem::MemFlag::WB
        } else {
            io::mem::MemFlag::WC
        };
        // SAFETY: The UAT regions are only accessed through this mapping by the CPU; the GPU
        // reaches them through its own translation.
        let map = unsafe { io::mem::Mem::try_new(res, flags.into()) }?;
        Ok(UatRegion {
            base: range.start,
            map,
        })
    }

    #[cfg(CONFIG_DRM_ASAHI_UAT_ADOPTION)]
    /// Creates a `Uat` for a handoff that the caller establishes itself (G15).
    ///
    /// This only maps the handoff and the TTBAT (write-combined); neither is written here. The
    /// caller performs the PPL magic exchange and installs TTBAT context 0 before any user `Vm` is
    /// bound, naming [`Self::kernel_lower_vm`] as its lower half and `regions.kernel_root` as its
    /// upper half. Bound user `Vm`s also name `regions.kernel_root` as their upper half.
    ///
    /// The firmware kernel root is not in the kernel linear map, so the driver cannot walk it.
    /// [`Self::kernel_vm`] instead manages the driver window (the kernel range) below a
    /// driver-owned shadow root that no context names: the caller links the window's second-level
    /// entries ([`Self::kernel_window_root`]) into the firmware's own table for that window after
    /// mapping and before the firmware uses the new addresses. Page tables are never freed on
    /// unmap, so linked entries stay valid.
    #[allow(dead_code)]
    pub(crate) fn new_adopted(
        dev: &driver::AsahiDrmDevice,
        geometry: UatGeometry,
        regions: &AdoptedRegions,
    ) -> Result<Self> {
        Self::new_adopted_with_user_upper(dev, geometry, regions, true)
    }

    /// Creates the same owned-RAM adopted UAT, choosing whether application contexts also
    /// name the firmware upper root. M4 qualifies only context 0 for firmware-upper access;
    /// its application TTBAT upper words stay zero. G15 preserves its existing shared upper.
    #[cfg(CONFIG_DRM_ASAHI_UAT_ADOPTION)]
    pub(crate) fn new_adopted_with_user_upper(
        dev: &driver::AsahiDrmDevice,
        geometry: UatGeometry,
        regions: &AdoptedRegions,
        map_kernel_to_user: bool,
    ) -> Result<Self> {
        let handoff_rgn = Self::map_physical(&regions.handoff, HANDOFF_SIZE)?;
        let ttbs_rgn = Self::map_physical(&regions.ttbs, SLOTS_SIZE)?;

        // SAFETY: The Handoff struct layout matches the firmware's view of memory at this
        // address, and the region is at least large enough per the check above.
        let handoff = unsafe { &(handoff_rgn.map.ptr() as *mut Handoff).as_ref().unwrap() };

        let inner = Arc::pin_init(
            try_pin_init!(UatInner {
                firmware: UatFirmware::Handoff,
                m4_adopted: !map_kernel_to_user,
                failed: AtomicBool::new(false),
                // The G15 UAT walker does not snoop the CPU caches.
                clean_poc: true,
                reserved: None,
                quirks: UatQuirks::DEFAULT,
                handoff_flush <- pin_init::pin_init_array_from_fn(|i| {
                    new_mutex!(HandoffFlush(&handoff.flush[i]), "handoff_flush")
                }),
                shared <- new_mutex!(
                    UatShared {
                        kernel_ttb1: regions.kernel_root,
                        execution_contexts: !EXECUTION_CONTEXT_MASK,
                        context_generations: [0; UAT_NUM_CTX],
                        binding_generations: [0; UAT_NUM_CTX],
                        binding_published: [false; UAT_NUM_CTX],
                        owned_user_roots: [0; UAT_NUM_CTX],
                        secondary_ttb1: None,
                        map_kernel_to_user,
                        handoff_rgn,
                        ttbs_rgn,
                        quirks: UatQuirks::DEFAULT,
                    },
                    "uat_shared"
                ),
            }),
            GFP_KERNEL,
        )?;

        // The shadow root is retained for the lifetime of the driver window it describes.
        let shadow = kernel::page::Page::alloc_page(GFP_KERNEL | __GFP_ZERO)?;
        let shadow_root = kernel::page::Page::into_phys(shadow);

        let kernel_lower_vm = Vm::new(
            dev.into(),
            inner.clone(),
            geometry.user_range(),
            geometry,
            None,
            1,
        )?;
        let kernel_vm = Vm::new(
            dev.into(),
            inner.clone(),
            geometry.kernel_range(),
            geometry,
            Some(shadow_root),
            0,
        )?;

        Ok(Self {
            dev: dev.into(),
            geometry,
            kernel_vm,
            kernel_lower_vm,
            inner,
            slots: slotalloc::SlotAllocator::new(
                UAT_USER_CTX as u32,
                (),
                |_inner, _slot| Some(SlotInner()),
                c_str!("Uat::SlotAllocator"),
                static_lock_class!(),
                static_lock_class!(),
            )?,
        })
    }

    #[cfg(CONFIG_DRM_ASAHI_UAT_ADOPTION)]
    /// Root of the kernel lower `Vm`, for TTBAT context 0 of an adopted handoff.
    pub(crate) fn kernel_lower_root(&self) -> PhysicalAddr {
        self.kernel_lower_vm.ttb()
    }

    #[cfg(CONFIG_DRM_ASAHI_UAT_ADOPTION)]
    /// Root whose driver-window entry names the second-level table of [`Self::kernel_vm`]'s
    /// driver window. With [`Self::new_adopted`] this is the shadow root that no context names.
    pub(crate) fn kernel_window_root(&self) -> PhysicalAddr {
        self.kernel_vm.ttb()
    }

    /// Map a bootloader-preallocated memory region
    fn map_region(
        dev: &device::Device,
        name: &CStr,
        size: usize,
        cached: bool,
    ) -> Result<UatRegion> {
        let of_node = dev.of_node().ok_or(EINVAL)?;
        let res = of_node.reserved_mem_region_to_resource_byname(name)?;
        let base = res.start();
        let res_size = res.size().try_into()?;

        if size > res_size {
            dev_err!(
                dev,
                "Region {} is too small (expected {}, got {})\n",
                name,
                size,
                res_size
            );
            return Err(ENOMEM);
        }

        let flags = if cached {
            io::mem::MemFlag::WB
        } else {
            io::mem::MemFlag::WC
        };

        // SAFETY: The safety of this operation hinges on the correctness of
        // much of this file and also the `pgtable` module, so it is difficult
        // to prove in a single safety comment. Such is life with raw GPU
        // page table management...
        let map = unsafe { io::mem::Mem::try_new(res, flags.into()) }.inspect_err(|_| {
            dev_err!(dev, "Failed to remap {} mem resource\n", name);
        })?;

        Ok(UatRegion { base, map })
    }

    /// Returns a reference to the global kernel (upper half) `Vm`
    pub(crate) fn kernel_vm(&self) -> &Vm {
        &self.kernel_vm
    }

    /// Returns a reference to the local kernel (lower half) `Vm`
    pub(crate) fn kernel_lower_vm(&self) -> &Vm {
        &self.kernel_lower_vm
    }

    /// Installed descriptor and bind contexts rooted in this user VM. A graph
    /// constructor checks this before changing a retained pool's mappings.
    pub(crate) fn vm_context_mask(&self, vm: &Vm) -> Result<u64> {
        if !matches!(self.inner.firmware, UatFirmware::PublishedRoots { .. }) {
            return Err(EINVAL);
        }
        let root = {
            let owner = lock_vm(&vm.gpuvm, None, false)?;
            if owner.is_kernel || !Arc::ptr_eq(&owner.uat_inner, &self.inner) {
                return Err(EINVAL);
            }
            owner.ttb()
        };
        Ok(self
            .inner
            .lock()
            .contexts_naming_root(root, self.inner.m4_adopted))
    }

    /// Publish a completed alias set before its descriptors become visible.
    /// Match actual roots, including independently allocated execution contexts.
    pub(crate) fn flush_vm_contexts(&self, vm: &Vm) -> Result<u64> {
        let contexts = self.vm_context_mask(vm)?;
        if contexts == 0 {
            return Err(ENOENT);
        }
        mem::sync();
        for context in 0..UAT_NUM_CTX {
            if contexts & (1u64 << context) != 0 {
                mem::tlbi_asid(context as u8);
            }
        }
        mem::sync();
        Ok(contexts)
    }

    /// Returns the base physical address of the TTBAT region.
    pub(crate) fn ttb_base(&self) -> u64 {
        let inner = self.inner.lock();

        inner.ttbs_rgn.base
    }

    /// Binds a `Vm` to a slot, preferring the last used one.
    pub(crate) fn bind(&self, vm: &Vm) -> Result<VmBind> {
        self.bind_inner(vm, true)
    }

    /// Binds a VM without waiting for another slot owner to retire.
    #[cfg(CONFIG_DRM_ASAHI_G15)]
    pub(crate) fn try_bind(&self, vm: &Vm) -> Result<VmBind> {
        self.bind_inner(vm, false)
    }

    fn bind_inner(&self, vm: &Vm, wait: bool) -> Result<VmBind> {
        self.inner.check_m4()?;
        let mut binding = vm.binding.lock();
        self.inner.check_m4()?;

        if binding.binding.is_none() {
            assert_eq!(binding.active_users, 0);

            let isolation = *module_parameters::robust_isolation.value() != 0;

            self.slots.set_limit(if isolation {
                NonZeroUsize::new(1)
            } else {
                None
            });

            let slot = if wait {
                self.slots.get(binding.bind_token)?
            } else {
                self.slots.try_get(binding.bind_token)?
            };
            if slot.changed() {
                mod_pr_debug!("Vm Bind [{}]: bind_token={:?}\n", vm.id, slot.token(),);
                let idx = (slot.slot() as usize) + UAT_USER_CTX_START;
                let ttb = binding.ttb | TTBR_VALID | (idx as u64) << TTBR_ASID_SHIFT;

                let mut transaction = self.inner.lock_handoff()?;
                let uat_inner = transaction.shared_mut();
                // On M4, empty bind/close churn must not wrap the wire generation
                // back to firmware's last observed identity for a different root.
                let previous = uat_inner.binding_generations[idx];
                let incarnation = if !self.inner.m4_adopted
                    || previous == 0
                    || uat_inner.binding_published[idx]
                {
                    previous.checked_add(1).ok_or(EOVERFLOW)?
                } else {
                    previous
                };

                let ttb1 =
                    application_ttb1(uat_inner.kernel_ttb1, idx, uat_inner.map_kernel_to_user);

                let ttbs = uat_inner.ttbs();
                if uat_inner.handoff().current_slot() == Some(idx as u32) {
                    pr_err!(
                        "Vm::bind to slot {}, but it is currently in use by the ASC?\n",
                        idx
                    );
                }
                ttbs[idx].ttb0.store(ttb, Ordering::Release);
                ttbs[idx].ttb1.store(ttb1, Ordering::Release);
                uat_inner.binding_generations[idx] = incarnation;
                uat_inner.binding_published[idx] = false;
                uat_inner.owned_user_roots[idx] = ttb;
                binding.generation = binding_wire_generation(incarnation);
                uat_inner.publish_slot(idx);
                drop(transaction);

                // Make sure all TLB entries from the previous owner of this ASID are gone
                mem::sync();
                mem::tlbi_asid(idx as u8);
                mem::sync();
            }

            binding.bind_token = Some(slot.token());
            binding.binding = Some(slot);
        }

        binding.active_users += 1;

        let slot = binding.binding.as_ref().unwrap().slot() + UAT_USER_CTX_START as u32;
        mod_pr_debug!("MMU: slot {} active users {}\n", slot, binding.active_users);
        Ok(VmBind(vm.clone(), slot))
    }

    /// Confirms the exact stopped-adopted M4 TTBAT contract. The same mutex serializes every
    /// host root installation/removal; firmware does not write these words. An unexpected
    /// observed root is an error, never a new ownership grant.
    #[cfg(CONFIG_DRM_ASAHI_G16)]
    pub(crate) fn check_adopted_bindings(&self) -> Result {
        self.inner.check_m4()?;
        // Binding locks precede the global UAT lock, as in bind().
        let lower = self.kernel_lower_vm.ttb() | TTBR_VALID;
        let inner = self.inner.lock();
        self.inner.check_m4()?;
        if self.inner.firmware != UatFirmware::Handoff || inner.map_kernel_to_user {
            return Err(EINVAL);
        }
        let ttbs = inner.ttbs();
        if ttbs[0].ttb0.load(Ordering::Acquire) != lower
            || ttbs[0].ttb1.load(Ordering::Acquire) != inner.kernel_ttb1 | TTBR_VALID
        {
            return Err(EIO);
        }
        for index in UAT_USER_CTX_START..UAT_NUM_CTX {
            if ttbs[index].ttb0.load(Ordering::Acquire) != inner.owned_user_roots[index]
                || ttbs[index].ttb1.load(Ordering::Acquire) != 0
            {
                return Err(EIO);
            }
        }
        Ok(())
    }

    /// Allocates a stable application context without changing the VM bind slot.
    /// Both roots and the generation remain unchanged for the lease's lifetime.
    pub(crate) fn new_execution_context(&self, vm: &Vm) -> Result<Arc<ExecutionContext>> {
        if self.inner.firmware == UatFirmware::Handoff {
            return Err(EINVAL);
        }
        {
            let owner = lock_vm(&vm.gpuvm, None, false)?;
            if owner.is_kernel
                || !core::ptr::eq(Arc::as_ptr(&owner.uat_inner), Arc::as_ptr(&self.inner))
            {
                return Err(EINVAL);
            }
        }
        let (id, generation) = {
            let mut inner = self.inner.lock();
            let free = !inner.execution_contexts & EXECUTION_CONTEXT_MASK;
            if free == 0 {
                return Err(ENOSPC);
            }
            let id = free.trailing_zeros() as usize;
            inner.execution_contexts |= 1u64 << id;
            inner.context_generations[id] = inner.context_generations[id].wrapping_add(1);
            (id, inner.context_generations[id])
        };
        let low = tagged_root(vm.ttb(), id);
        let high = tagged_root(self.kernel_vm.ttb(), id);
        // Allocate the complete host owner before either TTBAT half is published.
        // ExecutionRoot releases an empty claimed slot if allocation fails.
        let context = Arc::new(
            ExecutionContext {
                root: ExecutionRoot {
                    released: AtomicBool::new(false),
                    inner: self.inner.clone(),
                    id: id as u8,
                    generation,
                    low,
                    high,
                },
                vm: vm.clone(),
            },
            GFP_KERNEL,
        )?;
        {
            let inner = self.inner.lock();
            let roots = &inner.ttbs()[id];
            if roots.ttb0.load(Ordering::Acquire) != 0 || roots.ttb1.load(Ordering::Acquire) != 0 {
                return Err(EBUSY);
            }
            roots.ttb0.store(low, Ordering::Release);
            roots.ttb1.store(high, Ordering::Release);
            inner.publish_slot(id);
        }
        mem::sync();
        mem::tlbi_asid(id as u8);
        mem::sync();
        Ok(context)
    }

    /// Creates a new `Vm` linked to this UAT.
    pub(crate) fn new_vm(&self, id: u64, kernel_range: Range<u64>) -> Result<Vm> {
        self.inner.check_m4()?;
        let vm = Vm::new(
            self.dev.clone(),
            self.inner.clone(),
            kernel_range,
            self.geometry,
            None,
            id,
        )?;
        self.inner.check_m4()?;
        Ok(vm)
    }

    /// Returns the address space geometry of this UAT.
    pub(crate) fn geometry(&self) -> UatGeometry {
        self.geometry
    }

    /// Adopts the top-level entries that the primary firmware instance published in the kernel
    /// root while booting ([`UatFirmware::PublishedRoots`]).
    ///
    /// The entries below the kernel window belong to the firmware, and the kernel window entry
    /// points at a table that the bootloader reserved, so the driver maps kernel objects beneath
    /// it and never frees it. Entries above the kernel window do not belong to anyone and are
    /// cleared. Must run after the primary instance booted and before anything is mapped into the
    /// kernel `Vm`.
    pub(crate) fn adopt_firmware_root_entries(&self) -> Result<[u64; 3]> {
        let owned = IOVA_KERN_TOP_LEVEL_INDEX as usize + 1;
        let mut snapshot = [0; 3];
        // SAFETY: The kernel root is the bootloader-reserved `pagetables` region, which outlives
        // this `Uat`.
        unsafe {
            pgtable::with_root_entries(self.inner.reserved.as_deref(), self.kernel_vm.ttb(), UAT_PGSZ / size_of::<u64>(), |ptes| {
                for (value, pte) in snapshot.iter_mut().zip(ptes) {
                    *value = pte.load(Ordering::Relaxed);
                }
                for pte in &ptes[owned..] {
                    pte.store(0, Ordering::Relaxed);
                }
            })
        }?;

        mem::sync();
        mem::tlbi_all();
        mem::sync();
        Ok(snapshot)
    }

    /// Checks that allocation under the shared subtree did not replace a firmware-owned entry.
    pub(crate) fn confirm_firmware_root_entries(&self, snapshot: [u64; 3]) -> Result {
        // SAFETY: The root is a bootloader-reserved page retained for this UAT's lifetime.
        let matches = unsafe {
            pgtable::with_root_entries(self.inner.reserved.as_deref(), self.kernel_vm.ttb(), snapshot.len(), |ptes| {
                ptes.iter()
                    .zip(snapshot)
                    .all(|(pte, expected)| pte.load(Ordering::Relaxed) == expected)
            })
        }?;
        if !matches {
            return Err(EIO);
        }
        Ok(())
    }

    /// Copies the kernel root's top-level entries from the kernel window up into the root of the
    /// secondary firmware instance ([`UatFirmware::PublishedRoots`]), whose lower entries are
    /// private to that instance.
    pub(crate) fn mirror_secondary_root(&self) -> Result {
        let Some(secondary) = self.inner.lock().secondary_ttb1 else {
            // A single-instance firmware has no secondary root to mirror.
            return Ok(());
        };
        let count = UAT_PGSZ / size_of::<u64>();
        let first = IOVA_KERN_TOP_LEVEL_INDEX as usize;

        let reserved = self.inner.reserved.as_deref();
        // SAFETY: Both roots are in the bootloader-reserved `pagetables` region, which outlives
        // this `Uat`.
        unsafe {
            pgtable::with_root_entries(reserved, self.kernel_vm.ttb(), count, |primary| {
                pgtable::with_root_entries(reserved, secondary, count, |secondary| {
                    for (src, dst) in primary.iter().zip(secondary).skip(first) {
                        let pte = src.load(Ordering::Relaxed);
                        if pte != 0 && dst.load(Ordering::Relaxed) != pte {
                            dst.store(pte, Ordering::Relaxed);
                        }
                    }
                })
            })
        }??;

        mem::sync();
        mem::tlbi_all();
        mem::sync();
        Ok(())
    }

    /// Discards the CPU's cached lines of the no-map table regions after the firmware built its
    /// root there during boot, before the first host edit (`UatQuirks::reserved_tables`).
    pub(crate) fn invalidate_reserved_tables(&self) {
        if let Some(reserved) = self.inner.reserved.as_ref() {
            reserved.invalidate();
        }
    }

    /// Activates the device-global render root in context 1 after the boot records retire.
    /// Both contexts must still contain their original kernel roots.
    pub(crate) fn activate_render_context(&self, vm: &Vm) -> Result {
        let inner = self.inner.lock();
        inner.handoff_lock();
        let ttbs = inner.ttbs();
        let ready = [0, KERNEL_ALIAS_CTX].into_iter().all(|ctx| {
            ttbs[ctx].ttb0.load(Ordering::Acquire) == tagged_root(self.kernel_lower_vm.ttb(), ctx)
                && ttbs[ctx].ttb1.load(Ordering::Acquire) == tagged_root(self.kernel_vm.ttb(), ctx)
        });
        if !ready {
            inner.handoff_unlock();
            return Err(EIO);
        }
        ttbs[KERNEL_ALIAS_CTX]
            .ttb0
            .store(tagged_root(vm.ttb(), KERNEL_ALIAS_CTX), Ordering::Release);
        inner.publish_slot(KERNEL_ALIAS_CTX);
        inner.handoff_unlock();
        core::mem::drop(inner);
        mem::sync();
        mem::tlbi_asid(KERNEL_ALIAS_CTX as u8);
        mem::sync();
        Ok(())
    }

    /// Creates the reference-counted inner data for a new `Uat` instance.
    #[inline(never)]
    fn make_inner(
        dev: driver::AsahiDevice,
        firmware: UatFirmware,
        quirks: UatQuirks,
        reserved: Option<Arc<pgtable::ReservedTables>>,
    ) -> Result<Arc<UatInner>> {
        let handoff_rgn = Self::map_region(dev.as_ref().as_ref(), c"handoff", HANDOFF_SIZE, true)?;
        let ttbs_rgn = Self::map_region(dev.as_ref().as_ref(), c"ttbs", SLOTS_SIZE, true)?;

        // SAFETY: The Handoff struct layout matches the firmware's view of memory at this address,
        // and the region is at least large enough per the size specified above.
        let handoff = unsafe { &(handoff_rgn.map.ptr() as *mut Handoff).as_ref().unwrap() };

        dev_info!(dev.as_ref(), "MMU: Initializing kernel page table\n");
        let shared_quirks = quirks;

        Arc::pin_init(
            try_pin_init!(UatInner {
                firmware,
                m4_adopted: false,
                failed: AtomicBool::new(false),
                clean_poc: quirks.clean_tables,
                quirks,
                reserved,
                handoff_flush <- pin_init::pin_init_array_from_fn(|i| {
                    new_mutex!(HandoffFlush(&handoff.flush[i]), "handoff_flush")
                }),
                shared <- new_mutex!(
                    UatShared {
                        kernel_ttb1: 0,
                        execution_contexts: !EXECUTION_CONTEXT_MASK,
                        context_generations: [0; UAT_NUM_CTX],
                        binding_generations: [0; UAT_NUM_CTX],
                        binding_published: [false; UAT_NUM_CTX],
                        owned_user_roots: [0; UAT_NUM_CTX],
                        secondary_ttb1: None,
                        map_kernel_to_user: false,
                        handoff_rgn,
                        ttbs_rgn,
                        quirks: shared_quirks,
                    },
                    "uat_shared"
                ),
            }),
            GFP_KERNEL,
        )
    }

    /// Creates a new `Uat` instance for the given address space geometry and firmware.
    #[inline(never)]
    pub(crate) fn new(
        dev: driver::AsahiDevice,
        geometry: UatGeometry,
        map_kernel_to_user: bool,
        firmware: UatFirmware,
        quirks: UatQuirks,
    ) -> Result<Self> {
        dev_info!(dev.as_ref(), "MMU: Initializing...\n");

        mem::set_page_tlbi(quirks.page_tlbi);
        let of_node = dev.as_ref().as_ref().of_node().ok_or(EINVAL)?;
        let reserved = if quirks.reserved_tables {
            let mut tables = pgtable::ReservedTables::new();
            for name in [c"pagetables", c"l2-pagetables"] {
                let res = of_node.reserved_mem_region_to_resource_byname(name)?;
                // SAFETY: The boot loader reserved these regions (no-map) for the firmware's
                // page tables; they stay reserved until reboot.
                unsafe { tables.add(res)? };
            }
            Some(Arc::new(tables, GFP_KERNEL)?)
        } else {
            None
        };

        let inner = Self::make_inner(dev.clone(), firmware, quirks, reserved)?;

        let res = of_node.reserved_mem_region_to_resource_byname(c_str!("pagetables"))?;
        let ttb1 = res.start();
        let ttb1size: usize = res.size().try_into()?;

        let (pagetables_size, secondary_ttb1) = match firmware {
            UatFirmware::Handoff => (PAGETABLES_SIZE, None),
            UatFirmware::PublishedRoots {
                secondary_root_offset: Some(offset),
            } => (offset as usize + UAT_PGSZ, Some(ttb1 + offset)),
            UatFirmware::PublishedRoots {
                secondary_root_offset: None,
            } => (UAT_PGSZ, None),
        };
        if ttb1size < pagetables_size {
            dev_err!(dev.as_ref(), "MMU: Pagetables region is too small\n");
            return Err(ENOMEM);
        }

        dev_info!(dev.as_ref(), "MMU: Creating kernel page tables\n");
        let kernel_lower_vm = Vm::new(
            dev.clone(),
            inner.clone(),
            geometry.user_range(),
            geometry,
            None,
            1,
        )?;
        let kernel_vm = Vm::new(
            dev.clone(),
            inner.clone(),
            geometry.kernel_range(),
            geometry,
            Some(ttb1),
            0,
        )?;

        dev_info!(dev.as_ref(), "MMU: Kernel page tables created\n");

        let ttb0 = kernel_lower_vm.ttb();

        let uat = Self {
            dev: dev.clone(),
            geometry,
            kernel_vm,
            kernel_lower_vm,
            inner,
            slots: slotalloc::SlotAllocator::new(
                UAT_USER_CTX as u32,
                (),
                |_inner, _slot| Some(SlotInner()),
                c_str!("Uat::SlotAllocator"),
                static_lock_class!(),
                static_lock_class!(),
            )?,
        };

        let mut inner = uat.inner.lock();

        inner.map_kernel_to_user = map_kernel_to_user;
        inner.kernel_ttb1 = ttb1;
        inner.secondary_ttb1 = secondary_ttb1;

        inner.handoff().init(firmware, quirks.handoff_slot)?;
        if quirks.clean_tables {
            pgtable::clean_poc(inner.handoff() as *const Handoff as *const u8, HANDOFF_SIZE);
            pgtable::poc_barrier();
        }

        dev_info!(dev.as_ref(), "MMU: Initializing TTBs\n");

        inner.handoff_lock();

        let ttbs = inner.ttbs();

        ttbs[0].ttb0.store(ttb0 | TTBR_VALID, Ordering::SeqCst);
        ttbs[0].ttb1.store(ttb1 | TTBR_VALID, Ordering::SeqCst);

        for ctx in &ttbs[1..] {
            ctx.ttb0.store(0, Ordering::Relaxed);
            ctx.ttb1.store(0, Ordering::Relaxed);
        }

        if matches!(firmware, UatFirmware::PublishedRoots { .. }) {
            let alias = &ttbs[KERNEL_ALIAS_CTX];
            alias
                .ttb0
                .store(tagged_root(ttb0, KERNEL_ALIAS_CTX), Ordering::SeqCst);
            alias
                .ttb1
                .store(tagged_root(ttb1, KERNEL_ALIAS_CTX), Ordering::SeqCst);
        }
        if quirks.clean_tables {
            pgtable::clean_poc(ttbs.as_ptr() as *const u8, SLOTS_SIZE);
            pgtable::poc_barrier();
        }

        inner.handoff_unlock();

        core::mem::drop(inner);

        dev_info!(dev.as_ref(), "MMU: initialized\n");

        Ok(uat)
    }
}

impl Drop for Uat {
    fn drop(&mut self) {
        // Make sure we flush the TLBs
        mem::sync();
        mem::tlbi_all();
        mem::sync();
    }
}

impl Vm {
    /// Diagnostic reservation occupancy (including guards), not a placement promise.
    /// Removal can progress between queries; allocations are excluded by the VM lock.
    pub(crate) fn capacity(&self, range: Range<u64>) -> Result<(u64, u64)> {
        let inner = lock_vm(&self.gpuvm, None, false)?;
        let mut next = range.start;
        let mut used = 0;
        let mut largest = 0;
        while next < range.end {
            let Some(node) = inner.mm.first_overlap(next..range.end)? else {
                largest = largest.max(range.end - next);
                break;
            };
            largest = largest.max(node.start.saturating_sub(next));
            let end = node.end.min(range.end);
            used += end - node.start.max(next);
            next = end;
        }
        Ok((used, largest))
    }
}
impl Uat {
    pub(crate) fn execution_contexts_used(&self) -> u32 {
        (self.inner.lock().execution_contexts & EXECUTION_CONTEXT_MASK).count_ones()
    }
}
