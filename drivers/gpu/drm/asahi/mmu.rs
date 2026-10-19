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
//! TTBAT contexts other than a VM's bind slot may name a VM's root. See [`UatFirmware`]. G17 UATs
//! and their VMs ("published-roots" UATs) run the G17 paths of this module; every other UAT keeps
//! the handoff/M3/J613 paths.

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

use core::cmp;

use kernel::{
    addr::PhysicalAddr,
    bindings::drm_gpuvm_flags_DRM_GPUVM_IMMEDIATE_MODE,
    c_str,
    device,
    drm::{
        gem::shmem,
        gpuvm,
        mm, //
    },
    error::Result,
    io,
    new_mutex,
    page::Page,
    prelude::*,
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
    driver,
    fw,
    gem,
    hw,
    mem,
    pgtable,
    slotalloc,
    uat::UatGeometry as RootGeometry,
    util::RangeExt, //
};

// KernelMapping protection types
pub(crate) use crate::pgtable::Prot;
pub(crate) use pgtable::prot::*;
pub(crate) use pgtable::{
    pte_describe,
    pte_is_normal_memory,
    pte_is_uncached,
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
mod shared;
mod lifetime;
mod residency;
pub(crate) use residency::{Gate as ResidencyGate, VmShrinker};
pub(crate) use bind::PreparedUserBindBatch;
pub(crate) use lifetime::VmJobGuard;

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
/// Lower/user top VA for the 39-bit (AGX2) roots. Geometry-aware paths must
/// use [`iova_user_range`]/[`iova_user_usable_range`] instead; this const
/// remains for the AGX2 initdata builder, which is 39-bit-only.
pub(crate) const IOVA_USER_TOP: u64 = 1 << 39;

const AGX3_FIRMWARE_HANDOFF_VALIDATED: bool = false;

/// How the firmware-shared UAT handoff region behaves for the target SoC.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) enum HandoffMode {
    FirmwareDekker,
    /// The M3 runtime's RTKit is running before the AP publishes its host mappings (observed
    /// on J514S).
    FirmwareM3,
    /// 25G83 lazy handoff, initialized while ASC is stopped.
    FirmwareG16,

}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
struct UatConfig {
    chip_id: u32,
    ias: u8,
    oas: u32,
    /// Kernel tables are the bootloader-reserved handoff tables (the M3 runtime owner, on any
    /// SoC it admits); false for the HwConfig-driven GpuManager paths.
    reserved_tables: bool,
}

impl UatConfig {
    const fn from_hw(cfg: &hw::HwConfig) -> Self {
        Self {
            chip_id: cfg.chip_id,
            ias: cfg.uat_ias,
            oas: cfg.uat_oas,
            reserved_tables: false,
        }
    }

    /// The M3 runtime's UAT on the admitted `soc`: the bootloader-reserved handoff tables, with
    /// the root geometry of that SoC (42-bit roots and output addresses on every AGX3 part). On
    /// T6030 this is chip 0x6030, 42/42, the configuration its runtime always used.
    ///
    /// The caller passes the SoC its resource admission accepted, rather than this looking the
    /// chip up in the runtime table: an armed T6031 start is admitted without being in that
    /// table, which stays T6030 and T8122 only.
    fn m3_runtime(soc: &'static crate::m3_soc::Soc) -> Result<Self> {
        let cfg = soc.hwcfg.ok_or(ENODEV)?;
        if cfg.chip_id != soc.chip_id || cfg.uat_ias != 42 || cfg.uat_oas != 42 {
            return Err(ENODEV);
        }
        Ok(Self {
            chip_id: cfg.chip_id,
            ias: cfg.uat_ias,
            oas: cfg.uat_oas,
            reserved_tables: true,
        })
    }

    /// The UAT of a published-roots (G17) GPU. No chip-specific owner applies to it.
    fn published(geometry: UatGeometry) -> Result<Self> {
        let ias = u8::try_from(geometry.ias)?;
        RootGeometry::new(ias).ok_or(ENODEV)?;
        Ok(Self {
            chip_id: 0,
            ias,
            oas: geometry.oas,
            reserved_tables: false,
        })
    }

    /// Whether the M3 runtime (reserved tables) holds this UAT. That owner looks its reserved
    /// regions up through `m3_resources`, tracks faults and VM status, and keeps the AP side of
    /// the handoff itself.
    const fn m3_owner(&self) -> bool {
        self.reserved_tables
    }
}

fn uat_geometry(cfg: UatConfig) -> Result<RootGeometry> {
    // 39-bit (G13/G14) and 42-bit (G15+) roots are the two recovered
    // geometries; anything else is undefined.
    RootGeometry::new(cfg.ias).ok_or(ENODEV)
}

/// Lower/user VA range for this SoC's root geometry (one page up to the
/// per-root translation top: `1 << 39` on AGX2, `1 << 42` on AGX3).
#[allow(dead_code)]
pub(crate) fn iova_user_range(cfg: &hw::HwConfig) -> Result<Range<u64>> {
    Ok(IOVA_USER_BASE..uat_geometry(UatConfig::from_hw(cfg))?.user_va_top())
}

pub(crate) fn iova_unk_page(cfg: &hw::HwConfig) -> Result<u64> {
    Ok(uat_geometry(UatConfig::from_hw(cfg))?.user_va_top() - 2 * UAT_PGSZ as u64)
}

/// Usable lower/user VA range for this SoC (excludes the unknown page).
pub(crate) fn iova_user_usable_range(cfg: &hw::HwConfig) -> Result<Range<u64>> {
    Ok(IOVA_USER_BASE..iova_unk_page(cfg)?)
}

fn iova_kern_range(cfg: UatConfig) -> Result<Range<u64>> {
    let geometry = uat_geometry(cfg)?;
    Ok(geometry.kernel_va_base()..geometry.kernel_va_top())
}

/// Upper/kernel (TTBR1) base VA of the 39-bit (G13/G14) layout. The driver's `IOVA_KERN_*`
/// constants are expressed in this layout; [`kern_iova`] rebases them for the running GPU.
const IOVA_TTBR1_BASE_39: u64 = 0xffffff8000000000;

/// Returns the upper/kernel (TTBR1) base VA for this GPU's root geometry.
pub(crate) fn iova_ttbr1_base(cfg: &hw::HwConfig) -> u64 {
    RootGeometry::new(cfg.uat_ias)
        .map(|g| g.upper_canonical_base())
        .unwrap_or(IOVA_TTBR1_BASE_39)
}

/// Rebase a kernel (TTBR1) VA from the G13/G14 layout to the layout of this GPU generation.
///
/// The kernel ranges keep the same offset from the TTBR1 base on every generation, right after
/// the 128 GiB firmware-private space: 0xffffffa0_00000000 on G13/G14 and 0xfffffc20_00000000
/// on G15. This is the identity on G13/G14.
pub(crate) fn kern_iova(cfg: &hw::HwConfig, iova: u64) -> u64 {
    iova - IOVA_TTBR1_BASE_39 + iova_ttbr1_base(cfg)
}

/// Rebase a kernel (TTBR1) VA range, see [`kern_iova`].
pub(crate) fn kern_iova_range(cfg: &hw::HwConfig, range: Range<u64>) -> Range<u64> {
    // An exclusive end of 0 would mean "top of the address space"; no range here uses it.
    kern_iova(cfg, range.start)..kern_iova(cfg, range.end)
}

const TTBR_VALID: u64 = 0x1; // BIT(0)
const TTBR_ASID_SHIFT: usize = 48;

/// Address of a special dummy page?
//const IOVA_UNK_PAGE: u64 = 0x6f_ffff8000;
pub(crate) const IOVA_UNK_PAGE: u64 = IOVA_USER_TOP - 2 * UAT_PGSZ as u64;

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
    /// Two coprocessor instances that never touch the handoff region (G17).
    ///
    /// Each instance has its own kernel root and writes the top-level entries below and including
    /// the driver-managed kernel window into it while it boots: the lower ones are private to the
    /// instance, and the kernel window entry points at a table that the bootloader reserved. The
    /// primary instance uses the `pagetables` root, the secondary one the root at
    /// `secondary_root_offset` in the same region. TTBAT context 1 starts as an alias of the kernel
    /// context.
    PublishedRoots {
        /// Offset of the secondary instance's kernel root in the `pagetables` region.
        secondary_root_offset: u64,
    },
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

    /// Full kernel VA range
    #[cfg(CONFIG_DEV_COREDUMP)]
    const fn upper_range(&self) -> Range<u64> {
        self.upper_base()..(!UAT_PGMSK as u64)
    }
}

// The G17 geometry (`UatGeometry`) and the root geometry used by the other UATs
// (`RootGeometry`) describe the same windows for the two supported root widths.
const _: () = {
    let ias = [39u32, 42];
    let mut i = 0;
    while i < ias.len() {
        let g = UatGeometry { ias: ias[i], oas: 42 };
        let r = match RootGeometry::new(ias[i] as u8) {
            Some(r) => r,
            None => panic!("unsupported root geometry"),
        };
        assert!(g.user_top() == r.user_va_top());
        assert!(g.upper_base() == r.upper_canonical_base());
        assert!(g.kernel_range().start == r.kernel_va_base());
        assert!(g.kernel_range().end == r.kernel_va_top());
        i += 1;
    }
    assert!(UatGeometry { ias: 39, oas: 40 }.unk_page() == IOVA_UNK_PAGE);
};

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

/// Read-only snapshot of the handoff region, for diagnostics.
#[derive(Debug)]
#[allow(dead_code)] // Fields are only read through Debug.
pub(crate) struct HandoffSnapshot {
    pub(crate) magic_ap: u64,
    pub(crate) magic_fw: u64,
    pub(crate) magic_fw_ok: bool,
    pub(crate) lock_ap: u8,
    pub(crate) lock_fw: u8,
    pub(crate) turn: u32,
    pub(crate) cur_slot: u32,
    pub(crate) unk2: u8,
    pub(crate) unk3: u64,
    pub(crate) flush0_state: u64,
    pub(crate) flush_kernel_state: u64,
}

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

/// Inner data for a Vm instance. This is reference-counted by the outer Vm object.
struct VmInner {
    dev: driver::AsahiDevRef,
    is_kernel: bool,
    /// Permanently installed context without a dynamic VmBind lease.
    fixed_slot: Option<u32>,
    va_range: Range<u64>,
    page_table: UatPageTable,
    /// Mapping epoch of a published-roots (G17) VM; `None` on every other VM.
    mapping_epoch: Option<Arc<MappingEpoch>>,
    epoch: Option<u64>,
    mm: mm::Allocator<(), KernelMappingInner>,
    uat_inner: Arc<UatInner>,
    binding: Arc<Mutex<VmBinding>>,
    id: u64,
}

/// Slot binding-related inner data for a Vm instance.
struct VmBinding {
    active_users: usize,
    binding: Option<slotalloc::Guard<SlotInner>>,
    bind_token: Option<slotalloc::SlotToken>,
    ttb: u64,
}

struct VmBoInner {
    sgt: Option<shmem::SGTable<gem::AsahiObject>>,
    sg_vec: Option<KVVec<(usize, Range<usize>)>>,
}

/// Data associated with a VM <=> BO pairing
#[pin_data]
struct VmBo {
    #[pin]
    inner: Mutex<VmBoInner>,
}

impl gpuvm::DriverGpuVmBo for VmBo {
    fn new() -> impl PinInit<Self> {
        pin_init!(VmBo {
            inner <- new_mutex!(VmBoInner {
                sgt: None,
                sg_vec: None,
            }, "VmBinding"),
        })
    }
}

/// Per-mapping metadata that survives backing eviction and GPUVA splits.
#[derive(Default, Clone)]
struct VmGpuVa {
    prot: Option<Prot>,
}
impl gpuvm::DriverGpuVa for VmGpuVa {}

#[derive(Default)]
struct StepContext {
    new_va: Option<Pin<KBox<gpuvm::GpuVa<VmInner>>>>,
    prev_va: Option<Pin<KBox<gpuvm::GpuVa<VmInner>>>>,
    next_va: Option<Pin<KBox<gpuvm::GpuVa<VmInner>>>>,
    vm_bo: Option<ARef<gpuvm::GpuVmBo<VmInner>>>,
    prot: Prot,
}

impl gpuvm::DriverGpuVm for VmInner {
    type Driver = driver::AsahiDriver;
    type GpuVmBo = VmBo;
    type GpuVa = VmGpuVa;
    type StepContext = StepContext;

    fn step_map(
        self: &mut gpuvm::UpdatingGpuVm<'_, Self>,
        op: &mut gpuvm::OpMap<Self>,
        ctx: &mut Self::StepContext,
    ) -> Result {
        if self.published() {
            return Self::step_map_published(self, op, ctx);
        }
        if self.uat_inner.fault.as_ref().is_some_and(|fault| fault.load(Ordering::Acquire)) { return Err(EIO); }
        let mut iova = op.addr();
        let mut left = op.range() as usize;
        let mut offset = op.offset() as usize;

        let bo = ctx.vm_bo.as_ref().expect("step_map with no BO");

        let one_page = op.flags().contains(gpuvm::GpuVaFlags::REPEAT);

        // GPUVM cannot see driver-owned KernelMappings.  Preflight the full
        // object range in the authoritative page table and allocate every
        // intermediate table before the first scatterlist leaf is written.
        if self.uat_inner.fault.is_some() {
            self.page_table
                .prepare_map(op.addr()..op.addr().checked_add(op.range()).ok_or(EOVERFLOW)?)?;
        }

        let mut do_map = |mut addr: usize, mut len: usize, offset: &mut usize| -> Result<bool> {
            if left == 0 {
                return Ok(false);
            }

            if *offset > 0 {
                let skip = len.min(*offset);
                addr += skip;
                len -= skip;
                *offset -= skip;
            }
            if len == 0 {
                return Ok(true);
            }
            assert!(*offset == 0);

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

            self.page_table.map_pages(
                iova..(iova + len as u64),
                addr as PhysicalAddr,
                ctx.prot,
                one_page,
            )?;

            left -= len;
            iova += len as u64;
            Ok(true)
        };

        let guard = bo.inner().inner.lock();
        if let Some(sg_vec) = guard.sg_vec.as_ref() {
            let start_idx = sg_vec
                .binary_search_by(|range| {
                    if range.0 > offset {
                        cmp::Ordering::Greater
                    } else if (range.0 + range.1.len()) <= offset {
                        cmp::Ordering::Less
                    } else {
                        cmp::Ordering::Equal
                    }
                })
                .expect("sg_vec does not contain offset???");

            offset -= sg_vec[start_idx].0 as usize;

            for cur in start_idx..sg_vec.len() {
                let addr = sg_vec[cur].1.start as usize;
                let len: usize = sg_vec[cur].1.len() as usize;
                if do_map(addr, len, &mut offset)? == false {
                    break;
                }
            }
        } else {
            for range in guard.sgt.as_ref().expect("step_map with no SGT").iter() {
                // TODO: proper DMA address/length handling
                let addr = range.dma_address() as usize;
                let len: usize = range.dma_len() as usize;
                if do_map(addr, len, &mut offset)? == false {
                    break;
                }
            }
        }

        let gpuva = ctx.new_va.take().expect("Multiple step_map calls");

        if op
            .map_and_link_va(
                self,
                gpuva,
                ctx.vm_bo.as_ref().expect("step_map with no BO"),
            )
            .is_err()
        {
            dev_err!(
                self.dev.as_ref(),
                "map_and_link_va failed: {:#x} [{:#x}] -> {:#x}\n",
                op.offset(),
                op.range(),
                op.addr()
            );
            return Err(EINVAL);
        }
        Ok(())
    }
    fn step_unmap(
        self: &mut gpuvm::UpdatingGpuVm<'_, Self>,
        op: &mut gpuvm::OpUnMap<Self>,
        _ctx: &mut Self::StepContext,
    ) -> Result {
        if self.published() {
            return Self::step_unmap_published(self, op, _ctx);
        }
        if self.uat_inner.fault.as_ref().is_some_and(|fault| fault.load(Ordering::Acquire)) { return Err(EIO); }
        let va = op.va().expect("step_unmap: missing VA");

        mod_dev_dbg!(self.dev, "MMU: unmap: {:#x}:{:#x}\n", va.addr(), va.range());

        self.page_table
            .unmap_pages(va.addr()..(va.addr() + va.range()))?;

        if let Some(asid) = self.slot() {
            fence(Ordering::SeqCst);
            mem::tlbi_range(asid as u8, va.addr() as usize, va.range() as usize);
            mod_dev_dbg!(
                self.dev,
                "MMU: flush range: asid={:#x} start={:#x} len={:#x}\n",
                asid,
                va.addr(),
                va.range(),
            );
            mem::sync();
        }

        if op.unmap_and_unlink_va_defer().is_none() {
            dev_err!(self.dev.as_ref(), "step_unmap: could not unlink gpuva");
        }
        Ok(())
    }
    fn step_remap(
        self: &mut gpuvm::UpdatingGpuVm<'_, Self>,
        op: &mut gpuvm::OpReMap<Self>,
        vm_bo: &gpuvm::GpuVmBo<Self>,
        ctx: &mut Self::StepContext,
    ) -> Result {
        if self.published() {
            return Self::step_remap_published(self, op, vm_bo, ctx);
        }
        if self.uat_inner.fault.as_ref().is_some_and(|fault| fault.load(Ordering::Acquire)) { return Err(EIO); }
        let va = op.unmap().va().expect("No previous VA");
        let orig_addr = va.addr();
        let orig_range = va.range();

        // Only unmap the hole between prev/next, if they exist
        let unmap_start = if let Some(op) = op.prev_map() {
            op.addr() + op.range()
        } else {
            orig_addr
        };

        let unmap_end = if let Some(op) = op.next_map() {
            op.addr()
        } else {
            orig_addr + orig_range
        };

        mod_dev_dbg!(
            self.dev,
            "MMU: unmap for remap: {:#x}..{:#x} (from {:#x}:{:#x})\n",
            unmap_start,
            unmap_end,
            orig_addr,
            orig_range
        );

        let unmap_range = unmap_end - unmap_start;

        self.page_table.unmap_pages(unmap_start..unmap_end)?;

        if let Some(asid) = self.slot() {
            fence(Ordering::SeqCst);
            mem::tlbi_range(asid as u8, unmap_start as usize, unmap_range as usize);
            mod_dev_dbg!(
                self.dev,
                "MMU: flush range: asid={:#x} start={:#x} len={:#x}\n",
                asid,
                unmap_start,
                unmap_range,
            );
            mem::sync();
        }

        if op.unmap().unmap_and_unlink_va_defer().is_none() {
            dev_err!(self.dev.as_ref(), "step_unmap: could not unlink gpuva");
        }

        if let Some(prev_op) = op.prev_map() {
            let prev_gpuva = ctx
                .prev_va
                .take()
                .expect("Multiple step_remap calls with prev_op");
            if prev_op.map_and_link_va(self, prev_gpuva, vm_bo).is_err() {
                dev_err!(self.dev.as_ref(), "step_remap: could not relink prev gpuva");
                return Err(EINVAL);
            }
        }

        if let Some(next_op) = op.next_map() {
            let next_gpuva = ctx
                .next_va
                .take()
                .expect("Multiple step_remap calls with next_op");
            if next_op.map_and_link_va(self, next_gpuva, vm_bo).is_err() {
                dev_err!(self.dev.as_ref(), "step_remap: could not relink next gpuva");
                return Err(EINVAL);
            }
        }

        Ok(())
    }
}

impl VmInner {
    /// Whether this VM belongs to a published-roots (G17) UAT.
    fn published(&self) -> bool {
        self.uat_inner.firmware != UatFirmware::Handoff
    }

    /// `DriverGpuVm::step_map` of a published-roots (G17) VM.
    fn step_map_published(
        self: &mut gpuvm::UpdatingGpuVm<'_, Self>,
        op: &mut gpuvm::OpMap<Self>,
        ctx: &mut StepContext,
    ) -> Result {
        let _mutation = self.mapping_mutation();
        let mut iova = op.addr();
        let mut left = op.range() as usize;
        let mut offset = op.offset() as usize;

        let bo = ctx.vm_bo.as_ref().expect("step_map with no BO");

        let one_page = op.flags().contains(gpuvm::GpuVaFlags::REPEAT);

        let mut do_map = |mut addr: usize, mut len: usize, offset: &mut usize| -> Result<bool> {
            if left == 0 {
                return Ok(false);
            }

            if *offset > 0 {
                let skip = len.min(*offset);
                addr += skip;
                len -= skip;
                *offset -= skip;
            }
            if len == 0 {
                return Ok(true);
            }
            assert!(*offset == 0);

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
                self.rollback_failed_map(op.addr(), op.range());
                return Err(error);
            }

            left -= len;
            iova += len as u64;
            Ok(true)
        };

        let guard = bo.inner().inner.lock();
        if let Some(sg_vec) = guard.sg_vec.as_ref() {
            let start_idx = sg_vec
                .binary_search_by(|range| {
                    if range.0 > offset {
                        cmp::Ordering::Greater
                    } else if (range.0 + range.1.len()) <= offset {
                        cmp::Ordering::Less
                    } else {
                        cmp::Ordering::Equal
                    }
                })
                .expect("sg_vec does not contain offset???");

            offset -= sg_vec[start_idx].0 as usize;

            for cur in start_idx..sg_vec.len() {
                let addr = sg_vec[cur].1.start as usize;
                let len: usize = sg_vec[cur].1.len() as usize;
                if do_map(addr, len, &mut offset)? == false {
                    break;
                }
            }
        } else {
            for range in guard.sgt.as_ref().expect("step_map with no SGT").iter() {
                // TODO: proper DMA address/length handling
                let addr = range.dma_address() as usize;
                let len: usize = range.dma_len() as usize;
                if do_map(addr, len, &mut offset)? == false {
                    break;
                }
            }
        }

        let mut gpuva = ctx.new_va.take().expect("Multiple step_map calls");
        gpuva.as_mut().inner_mut().get_mut().prot = Some(ctx.prot);

        if op
            .map_and_link_va(
                self,
                gpuva,
                ctx.vm_bo.as_ref().expect("step_map with no BO"),
            )
            .is_err()
        {
            dev_err!(
                self.dev.as_ref(),
                "map_and_link_va failed: {:#x} [{:#x}] -> {:#x}\n",
                op.offset(),
                op.range(),
                op.addr()
            );
            self.rollback_failed_map(op.addr(), op.range());
            return Err(EINVAL);
        }
        self.tlbi_contexts_naming_root(op.addr(), op.range() as usize);
        Ok(())
    }
    /// `DriverGpuVm::step_unmap` of a published-roots (G17) VM.
    fn step_unmap_published(
        self: &mut gpuvm::UpdatingGpuVm<'_, Self>,
        op: &mut gpuvm::OpUnMap<Self>,
        _ctx: &mut StepContext,
    ) -> Result {
        let va = op.va().expect("step_unmap: missing VA");

        self.unmap_gpuva(va)?;

        if op.unmap_and_unlink_va_defer().is_none() {
            dev_err!(self.dev.as_ref(), "step_unmap: could not unlink gpuva");
        }
        Ok(())
    }
    /// `DriverGpuVm::step_remap` of a published-roots (G17) VM.
    fn step_remap_published(
        self: &mut gpuvm::UpdatingGpuVm<'_, Self>,
        op: &mut gpuvm::OpReMap<Self>,
        vm_bo: &gpuvm::GpuVmBo<Self>,
        ctx: &mut StepContext,
    ) -> Result {
        let va = op.unmap().va().expect("No previous VA");
        let orig_addr = va.addr();
        let orig_range = va.range();
        let orig_prot = va.inner().prot.ok_or(EINVAL)?;
        let orig_invalidated = va.flags().contains(gpuvm::GpuVaFlags::INVALIDATED);

        // Only unmap the hole between prev/next, if they exist
        let unmap_start = if let Some(op) = op.prev_map() {
            op.addr() + op.range()
        } else {
            orig_addr
        };

        let unmap_end = if let Some(op) = op.next_map() {
            op.addr()
        } else {
            orig_addr + orig_range
        };

        mod_dev_dbg!(
            self.dev,
            "MMU: unmap for remap: {:#x}..{:#x} (from {:#x}:{:#x})\n",
            unmap_start,
            unmap_end,
            orig_addr,
            orig_range
        );

        let unmap_range = unmap_end - unmap_start;

        let _mutation = self.mapping_mutation();
        if orig_invalidated {
            self.page_table.discard_partial_map(unmap_start..unmap_end)?;
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

        if op.unmap().unmap_and_unlink_va_defer().is_none() {
            dev_err!(self.dev.as_ref(), "step_unmap: could not unlink gpuva");
        }

        if let Some(prev_op) = op.prev_map() {
            let mut prev_gpuva = ctx
                .prev_va
                .take()
                .expect("Multiple step_remap calls with prev_op");
            prev_gpuva.as_mut().inner_mut().get_mut().prot = Some(orig_prot);
            if prev_op.map_and_link_va(self, prev_gpuva, vm_bo).is_err() {
                dev_err!(self.dev.as_ref(), "step_remap: could not relink prev gpuva");
                return Err(EINVAL);
            }
        }

        if let Some(next_op) = op.next_map() {
            let mut next_gpuva = ctx
                .next_va
                .take()
                .expect("Multiple step_remap calls with next_op");
            next_gpuva.as_mut().inner_mut().get_mut().prot = Some(orig_prot);
            if next_op.map_and_link_va(self, next_gpuva, vm_bo).is_err() {
                dev_err!(self.dev.as_ref(), "step_remap: could not relink next gpuva");
                return Err(EINVAL);
            }
        }

        Ok(())
    }

    /// Remove hardware leaves for one complete user GPUVA without allocating.
    fn unmap_gpuva(&mut self, va: &gpuvm::GpuVa<Self>) -> Result {
        mod_dev_dbg!(self.dev, "MMU: unmap: {:#x}:{:#x}\n", va.addr(), va.range());

        let _mutation = self.mapping_mutation();
        if va.flags().contains(gpuvm::GpuVaFlags::INVALIDATED) {
            self.page_table.discard_partial_map(va.addr()..(va.addr() + va.range()))?;
        } else {
            self.page_table.unmap_pages(va.addr()..(va.addr() + va.range()))?;
        }

        if let Some(asid) = self.slot() {
            mem::sync();
            self.tlbi_range(asid as u8, va.addr(), va.range() as usize);
            mod_dev_dbg!(
                self.dev,
                "MMU: flush range: asid={:#x} start={:#x} len={:#x}\n",
                asid,
                va.addr(),
                va.range(),
            );
            mem::sync();
        }

        self.tlbi_contexts_naming_root(va.addr(), va.range() as usize);

        Ok(())
    }

    /// A failed map has no GPUVA owner to retain its backing or remove its leaves.
    /// GPUVM has already split or removed overlaps throughout this operation's range.
    fn rollback_failed_map(&mut self, addr: u64, size: u64) {
        // Include the current segment: map_pages can fail after writing some of its leaves.
        // The checked GPUVM operation supplies an aligned range without overflow.
        let result = self.page_table.discard_partial_map(addr..addr + size);
        if self.uat_inner.firmware == UatFirmware::Handoff {
            if let Some(asid) = self.slot() {
                mem::sync();
                self.tlbi_range(asid as u8, addr, size as usize);
                mem::sync();
            }
        }
        self.tlbi_contexts_naming_root(addr, size as usize);
        if let Err(error) = result {
            dev_err!(
                self.dev.as_ref(),
                "Could not discard failed mapping: {:?}\n",
                error
            );
        }
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
    /// change must reach all of them. Other firmware only uses the bind slot, which the callers
    /// already invalidate.
    fn tlbi_contexts_naming_root(&self, iova: u64, size: usize) {
        if self.uat_inner.firmware == UatFirmware::Handoff {
            mem::sync();
            return;
        }

        let contexts = self.uat_inner.lock().contexts_naming_root(self.ttb());
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

    /// `map_node` of a published-roots (G17) VM.
    fn map_node_published(&mut self, node: &mm::Node<(), KernelMappingInner>, prot: Prot) -> Result {
        let end = node
            .start()
            .checked_add(node.mapped_size as u64)
            .ok_or(EOVERFLOW)?;
        let _mutation = self.mapping_mutation();
        self.page_table.prepare_map(node.start()..end)?;
        let result = self.map_node_prepared(node, prot);
        if result.is_err() {
            self.rollback_failed_map(node.start(), node.mapped_size as u64);
        }
        result
    }

    /// Install only into the empty range reserved by `map_node`.
    fn map_node_prepared(&mut self, node: &mm::Node<(), KernelMappingInner>, prot: Prot) -> Result {
        let mut iova = node.start();
        let guard = node.bo.as_ref().ok_or(EINVAL)?.get()?.inner().inner.lock();
        let sgt = guard.sgt.as_ref().ok_or(EINVAL)?;
        let mut offset = node.offset;
        let mut left = node.mapped_size;

        for range in sgt.iter() {
            if left == 0 {
                break;
            }

            // TODO: proper DMA address/length handling
            let mut addr = range.dma_address() as usize;
            let mut len: usize = range.dma_len() as usize;

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

        // A context naming this root may be live and hold a negative translation for the range.
        self.tlbi_contexts_naming_root(node.start(), node.mapped_size);
        Ok(())
    }

    /// Returns the slot index, if this VM is bound.
    fn slot(&self) -> Option<u32> {
        if let Some(slot) = self.fixed_slot { return Some(slot); }
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

    /// Check that a byte range is fully mapped with the requested GPU access.
    /// The caller holds the VM execution lock.
    fn covers_mapped_range(
        &mut self,
        address: u64,
        size: u64,
        need_read: bool,
        need_write: bool,
    ) -> bool {
        if size == 0 {
            return false;
        }
        let Some(end) = address.checked_add(size) else {
            return false;
        };
        let page_mask = UAT_PGMSK as u64;
        let start = address & !page_mask;
        let aligned_end = if end & page_mask == 0 {
            end
        } else {
            match (end | page_mask).checked_add(1) {
                Some(end) => end,
                None => return false,
            }
        };
        if start < self.va_range.start || aligned_end > self.va_range.end {
            return false;
        }
        self.page_table
            .covers_range(start..aligned_end, need_read, need_write)
            .unwrap_or(false)
    }

    /// Copy `out.len()` bytes at `iova` from the pages this VM maps, through
    /// the kernel's linear map. `before_copy` receives the kernel address and
    /// length of each chunk, which never crosses a page, before it is copied.
    ///
    /// The caller holds the VM execution lock for the whole call. Every PTE
    /// change takes that lock, and a page leaves the page table before the GEM
    /// reference that keeps it allocated is released, so a concurrent unbind
    /// or GEM close cannot free a page while it is being read.
    fn copy_from_mapped_pages(
        &mut self,
        iova: u64,
        out: &mut [u8],
        mut before_copy: impl FnMut(*mut u8, usize),
    ) -> Result {
        let total = out.len();
        let mut done = 0usize;
        while done < total {
            let addr = iova.checked_add(done as u64).ok_or(EOVERFLOW)?;
            let phys = self.page_table.translate_iova(addr)?;
            let page_mask = (kernel::page::PAGE_SIZE as PhysicalAddr) - 1;
            let page_phys = phys & !page_mask;
            let offset = (phys - page_phys) as usize;
            let chunk = cmp::min(kernel::page::PAGE_SIZE - offset, total - done);
            // SAFETY: `translate_iova` resolved this address through a leaf
            // PTE of this VM while the VM execution lock is held. The lock
            // stays held until the copy below finishes, so the PTE cannot be
            // removed and the GEM object it maps keeps the page allocated.
            // `borrow_phys` additionally rejects anything without a struct
            // page.
            let page = unsafe { Page::borrow_phys(&page_phys) }.ok_or(EFAULT)?;
            let dst = &mut out[done..done + chunk];
            page.with_pointer_into_page(offset, chunk, |ptr| {
                before_copy(ptr, chunk);
                // SAFETY: `with_pointer_into_page` bounds-checked
                // `offset..offset + chunk` inside the page.
                let src = unsafe { core::slice::from_raw_parts(ptr as *const u8, chunk) };
                dst.copy_from_slice(src);
                Ok(())
            })?;
            done += chunk;
        }
        Ok(())
    }

    /// Map an `mm::Node` representing an mapping in VA space.
    fn map_node(&mut self, node: &mm::Node<(), KernelMappingInner>, prot: Prot) -> Result {
        if self.published() {
            return self.map_node_published(node, prot);
        }
        let mut iova = node.start();
        let mapping_end = if self.uat_inner.fault.is_some() {
            iova.checked_add(node.mapped_size as u64).ok_or(EOVERFLOW)?
        } else { iova + node.mapped_size as u64 };
        // The private fixed-mapping allocator cannot see GPUVM-owned leaves.
        // Reserve the complete page-table walk before installing any SG run.
        if self.uat_inner.fault.is_some() { self.page_table.prepare_map(iova..mapping_end)?; }
        let guard = node.bo.as_ref().ok_or(EINVAL)?.get()?.inner().inner.lock();
        let sgt = guard.sgt.as_ref().ok_or(EINVAL)?;
        let mut offset = node.offset;
        let mut left = node.mapped_size;

        for range in sgt.iter() {
            if left == 0 {
                break;
            }

            // TODO: proper DMA address/length handling
            let mut addr = range.dma_address() as usize;
            let mut len: usize = range.dma_len() as usize;

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
        Ok(())
    }
}

/// Shared reference to a virtual memory address space ([`Vm`]).
#[derive(Clone)]
pub(crate) struct Vm {
    fault: Option<Arc<AtomicBool>>,
    id: u64,
    inner: ARef<gpuvm::GpuVm<VmInner>>,
    dummy_obj: ARef<gem::Object>,
    binding: Arc<Mutex<VmBinding>>,
    driver_mappings: Option<Arc<Mutex<Option<VmDriverMappings>>>>,
    agx_status: Option<Arc<crate::agx_status::VmStatus>>,
    job_lifetime: Option<Arc<Mutex<M3VmJobLifetime>>>,
    // Published-roots (G17) VM state; `None` on every other VM.
    mapping_epoch: Option<Arc<MappingEpoch>>,
    lifetime: Option<Arc<lifetime::VmLifetime>>,
    residency: Option<Arc<residency::Gate>>,
    context_bindings: Option<Arc<context::ContextBindings>>,
    shared_bindings: Option<Arc<shared::SharedBindings>>,
    status: Option<Arc<crate::g17::status::VmStatus>>,
}
impl Drop for Vm {
    fn drop(&mut self) {
        if self.fault.as_ref().is_some_and(|fault| fault.load(Ordering::Acquire)) {
            // Keep the GPUVM and its user BO bindings until a future reset
            // implementation can prove that firmware no longer references it.
            core::mem::forget(self.inner.clone());
        }
    }
}
no_debug!(Vm);

struct M3VmJobLifetime {
    active: usize,
    closed: bool,
    close_ranges: Option<(Range<u64>, Range<u64>)>,
    closed_objects: KVec<ARef<gem::Object>>,
}

/// GPU ownership outlives a DRM handle or file. Deferred user mappings retain
/// their GEM backing until all accepted jobs retire; quarantined packets keep
/// this guard until the processors have provably stopped.
pub(crate) struct M3VmJobGuard { vm: Vm }
impl Drop for M3VmJobGuard {
    fn drop(&mut self) {
        let (ranges, objects) = {
            let mut state = self.vm.job_lifetime.as_ref().expect("M3 VM job state").lock();
            state.active -= 1;
            if state.active != 0 { return; }
            (state.close_ranges.take(), core::mem::replace(&mut state.closed_objects, KVec::new()))
        };
        if let Some((user, kernel)) = ranges {
            if self.vm.unmap_user_ranges(user, kernel).is_err() {
                pr_err!("M3 VM job retirement: deferred user unmap failed\n");
            }
        }
        for object in objects {
            if self.vm.drop_mappings(&object).is_err() {
                pr_err!("M3 VM job retirement: deferred GEM unmap failed\n");
            }
        }
        self.vm.bo_deferred_cleanup();
    }
}

struct VmDriverMappings {
    mappings: KVec<KernelMapping>,
    reserved_ranges: KVec<Range<u64>>,
}
no_debug!(VmDriverMappings);

impl VmDriverMappings {
    fn iter_mappings(&self) -> impl Iterator<Item = &KernelMapping> {
        self.mappings.iter()
    }
}

fn any_range_overlaps<I>(target: Range<u64>, mut ranges: I) -> bool
where
    I: Iterator<Item = Range<u64>>,
{
    ranges.any(|range| range.overlaps(target.clone()))
}

/// Allocation generation of a [`Vm`] hardware slot.
pub(crate) struct SlotInner { generation: u8 }

impl slotalloc::SlotItem for SlotInner {
    type Data = ();
}

/// Represents a single user of a binding of a [`Vm`] to a slot.
///
/// The number of users is counted, and the slot will be freed when it drops to 0.
#[derive(Debug)]
pub(crate) struct VmBind(Vm, u32, u8);

impl VmBind {
    /// Returns the slot that this `Vm` is bound to.
    pub(crate) fn slot(&self) -> u32 {
        self.1
    }

    /// Allocation generation retained with this ASID.
    pub(crate) fn generation(&self) -> u8 { self.2 }

    /// The bound `Vm` itself, for diagnostics that need to walk its tables.
    pub(crate) fn vm(&self) -> &Vm {
        &self.0
    }

    /// Whether this retained binding belongs to the supplied DRM VM.
    pub(crate) fn matches(&self, vm: &Vm) -> bool {
        core::ptr::eq(&*self.0.inner, &*vm.inner)
    }

    /// Physical base of the bound VM's page-table root.
    pub(crate) fn root(&self) -> u64 {
        self.0.ttb()
    }

    /// The bound VM's driver-assigned id.
    pub(crate) fn vm_id(&self) -> u64 {
        self.0.id
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
        VmBind(self.0.clone(), self.1, self.2)
    }
}

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
            let handoff = inner.handoff().lock_legacy();
            let roots = &inner.ttbs()[id];
            let low = roots.ttb0.load(Ordering::Acquire);
            let high = roots.ttb1.load(Ordering::Acquire);
            let owned = low == self.low && high == self.high;
            if owned {
                roots.ttb0.store(0, Ordering::Release);
                roots.ttb1.store(0, Ordering::Release);
            }
            drop(handoff);
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

/// A mapping BO reference. On published-roots (G17) VMs it is released under the
/// immediate-mode GPUVA lock, retaining the owner until deferred cleanup completes, including
/// its final BO. Other VMs drop the reference synchronously.
struct MappingBo {
    bo: Option<ARef<gpuvm::GpuVmBo<VmInner>>>,
    owner: Option<ARef<gpuvm::GpuVm<VmInner>>>,
}
impl MappingBo {
    fn new(
        bo: ARef<gpuvm::GpuVmBo<VmInner>>,
        owner: ARef<gpuvm::GpuVm<VmInner>>,
    ) -> Self {
        Self {
            bo: Some(bo),
            owner: Some(owner),
        }
    }
    fn synchronous(bo: ARef<gpuvm::GpuVmBo<VmInner>>) -> Self {
        Self { bo: Some(bo), owner: None }
    }
    fn get(&self) -> Result<&gpuvm::GpuVmBo<VmInner>> {
        self.bo.as_deref().ok_or(EINVAL)
    }
}
impl Drop for MappingBo {
    fn drop(&mut self) {
        let Some(owner) = self.owner.as_ref() else {
            // The `bo` field drops normally.
            return;
        };
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
        owner.bo_deferred_cleanup();
    }
}

/// The GEM object of a mapping: published-roots (G17) mappings pin it against idle reclaim.
enum MappingObject {
    Shared(ARef<gem::Object>),
    Pinned(gem::KernelMappingPin),
}

impl core::ops::Deref for MappingObject {
    type Target = gem::Object;
    fn deref(&self) -> &gem::Object {
        match self {
            Self::Shared(object) => object,
            Self::Pinned(pin) => pin,
        }
    }
}

/// Inner data required for an object mapping into a [`Vm`].
pub(crate) struct KernelMappingInner {
    // Drop order matters:
    // - Drop the GpuVmBo first, which locks its BO GPUVA list and drops a GpuVm reference
    //   (published-roots VMs: deferred-put it under its immediate-mode GPUVA lock)
    // - Drop the GEM BO next, since BO free can take the resv lock itself
    // - Drop the owner GpuVm last, since that again can take resv locks when the refcount drops to 0
    bo: Option<MappingBo>,
    _gem: Option<MappingObject>,
    owner: ARef<gpuvm::GpuVm<VmInner>>,
    uat_inner: Arc<UatInner>,
    prot: Prot,
    offset: usize,
    mapped_size: usize,
}

/// Prepared CPU timestamp access; destruction happens after completion signalling.
pub(crate) struct TimestampMapping {
    mapping: shmem::VMap<gem::AsahiObject, u8>,
    offset: usize,
}

impl TimestampMapping {
    pub(crate) fn write(&self, value: u64) {
        let map = self.mapping.get();
        for (index, byte) in value.to_le_bytes().into_iter().enumerate() {
            // SAFETY: timestamp_mapping checked the complete eight-byte extent.
            // The owned vmap pins its backing; byte atomics permit unaligned and
            // overlapping timestamp destinations without mutable slice aliases.
            let destination = unsafe {
                &*map.as_mut_ptr().add(self.offset + index).cast::<AtomicU8>()
            };
            destination.store(byte, Ordering::Relaxed);
        }
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
        self.0.owner.exec_lock(None, false)?.page_table.translate_iova(address)
    }

    /// Writes a userspace timestamp through the retained GEM mapping. Byte
    /// stores support unaligned destinations and concurrent completion writes
    /// without creating mutable aliases. The complete timestamp is not atomic.
    /// Retain CPU access before this destination's completion fence is published.
    pub(crate) fn timestamp_mapping(&self, offset: usize) -> Result<TimestampMapping> {
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
        Ok(TimestampMapping { mapping: gem.owned_vmap::<u8>()?, offset: start })
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
            let mut owner = self.0.owner.exec_lock(None, false)?;
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

    /// `Drop` of a published-roots (G17) mapping. That firmware does not use the handoff
    /// protocol and cannot be asked to flush its cache; it only caches kernel mappings that live
    /// until its coprocessors stop.
    fn drop_published(&mut self) {
        // Only VM page tables/binding state are touched here. The shared
        // reservation is sufficient and avoids fallible exec allocation in Drop.
        let mut owner = self.0.owner.lock_inner();
        mod_dev_dbg!(
            owner.dev,
            "MMU: unmap {:#x}:{:#x}\n",
            self.iova(),
            self.size()
        );

        let mutation = owner.mapping_mutation();
        if owner.page_table.unmap_pages(self.iova_range()).is_err() {
            dev_err!(
                owner.dev.as_ref(),
                "MMU: unmap {:#x}:{:#x} failed\n",
                self.iova(),
                self.size()
            );
        }

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
        drop(mutation);
        drop(owner);
        // SAFETY: The PTEs are gone and invalidated, and the VM reservation is released before
        // the node drops its BO references.
        unsafe { ManuallyDrop::drop(&mut self.0) };
    }

    fn remap_uncached_and_flush_legacy(&mut self) {
        let mut owner = self
            .0
            .owner
            .exec_lock(None, false)
            .expect("Failed to exec_lock in remap_uncached_and_flush");

        mod_dev_dbg!(
            owner.dev,
            "MMU: remap as uncached {:#x}:{:#x}\n",
            self.iova(),
            self.size()
        );

        // Remap in-place as uncached.
        // Do not try to unmap the guard page (-1)
        let prot = self.0.prot.as_uncached();
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
        fence(Ordering::SeqCst);

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
            let handoff_guard = uat_inner.handoff().lock_legacy();
            let cur_slot = uat_inner.handoff().current_slot();
            let ttb_cur = uat_inner.ttbs()[slot as usize].ttb0.load(Ordering::Relaxed);
            drop(handoff_guard);
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
        flush.begin_flush_legacy(self.iova(), self.size() as u64);
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

        // Tell the firmware to do a cache flush
        if let Err(e) = (*owner.dev).gpu().and_then(|gpu| gpu.fwctl(cmd)) {
            dev_err!(
                owner.dev.as_ref(),
                "MMU: ASC cache flush {:#x}:{:#x} failed (err: {:?})\n",
                self.iova(),
                self.size(),
                e
            );
        }

        // Finish the flush
        flush.end_flush_legacy();

        // Slot is unlocked here
    }

    fn drop_legacy(&mut self) {
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

        if self.0.prot.is_cached_noncoherent() {
            mod_pr_debug!(
                "MMU: remap as uncached {:#x}:{:#x}\n",
                self.iova(),
                self.size()
            );
            self.remap_uncached_and_flush_legacy();
        }

        let mut owner = self
            .0
            .owner
            .exec_lock(None, false)
            .expect("exec_lock failed in KernelMapping::drop");
        mod_dev_dbg!(
            owner.dev,
            "MMU: unmap {:#x}:{:#x}\n",
            self.iova(),
            self.size()
        );

        if owner.page_table.unmap_pages(self.iova_range()).is_err() {
            dev_err!(
                owner.dev.as_ref(),
                "MMU: unmap {:#x}:{:#x} failed\n",
                self.iova(),
                self.size()
            );
        }

        if let Some(asid) = owner.slot() {
            fence(Ordering::SeqCst);
            mem::tlbi_range(asid as u8, self.iova() as usize, self.size());
            mod_dev_dbg!(
                owner.dev,
                "MMU: flush range: asid={:#x} start={:#x} len={:#x}\n",
                asid,
                self.iova(),
                self.size()
            );
            mem::sync();
        }
        drop(owner);
        // SAFETY: This is the original legacy teardown order. The node was
        // automatically dropped after the old Drop callback returned.
        unsafe { ManuallyDrop::drop(&mut self.0) };
    }

    /// Remap a cached mapping as uncached, then synchronously flush that range of VAs from the
    /// coprocessor cache. This is required to safely unmap cached/private mappings.
    fn remap_uncached_and_flush(&mut self) -> Result {
        let mut owner = self
            .0
            .owner
            .exec_lock(None, false)?;

        mod_dev_dbg!(
            owner.dev,
            "MMU: remap as uncached {:#x}:{:#x}\n",
            self.iova(),
            self.size()
        );

        // Remap in-place as uncached.
        // Do not try to unmap the guard page (-1)
        let prot = self.0.prot.as_uncached();
        if let Err(error) = owner.page_table.reprot_pages(self.iova_range(), prot) {
            dev_err!(
                owner.dev.as_ref(),
                "MMU: remap {:#x}:{:#x} failed\n",
                self.iova(),
                self.size()
            );
            return Err(error);
        }
        fence(Ordering::SeqCst);

        // M3 builds its UAT and object graph before AsahiData.gpu is
        // initialized. A constructor error can therefore drop cached mappings
        // while owner.dev has no DrmGpu yet. Those mappings were never
        // published to firmware, so the PTE downgrade above is sufficient.
        if !self.0.uat_inner.firmware_cache_flush_ready() {
            mod_dev_dbg!(
                owner.dev,
                "MMU: firmware cache flush not armed for {:#x}:{:#x}\n",
                self.iova(),
                self.size()
            );
            return Ok(());
        }

        // If we don't have (and have never had) a VM slot, just return
        let slot = match owner.slot() {
            None => return Ok(()),
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
            let handoff_guard = uat_inner.lock_handoff()?;
            let cur_slot = uat_inner.handoff().current_slot();
            let ttb_cur = uat_inner.ttbs()[slot as usize].ttb0.load(Ordering::Relaxed);
            drop(handoff_guard);
            if cur_slot == Some(slot) && ttb_cur == ttb {
                slot
            } else {
                return Ok(());
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
        if pages >= 0x10000 {
            dev_err!(
                owner.dev.as_ref(),
                "MMU: Flush too big ({:#x} pages))\n",
                pages
            );
            return Err(E2BIG);
        }

        flush.begin_flush(self.iova(), self.size() as u64)?;
        let cmd = fw::channels::FwCtlMsg {
            addr: fw::types::U64(self.iova()),
            unk_8: 0,
            slot: flush_slot,
            page_count: pages as u16,
            unk_12: 2, // ?
        };

        // Tell the firmware to do a cache flush
        if let Err(e) = (*owner.dev).gpu().and_then(|gpu| gpu.fwctl(cmd)) {
            dev_err!(
                owner.dev.as_ref(),
                "MMU: ASC cache flush {:#x}:{:#x} failed (err: {:?})\n",
                self.iova(),
                self.size(),
                e
            );
            return Err(e);
        }

        // Finish the flush
        flush.end_flush()?;

        // Slot is unlocked here
        Ok(())
    }
}
no_debug!(KernelMapping);

impl Drop for KernelMapping {
    fn drop(&mut self) {
        if self.0.uat_inner.firmware != UatFirmware::Handoff {
            self.drop_published();
            return;
        }
        if self.0.uat_inner.fault.is_none() {
            self.drop_legacy();
            return;
        }

        if self.0.uat_inner.fault.as_ref().is_some_and(|fault| fault.load(Ordering::Acquire)) { return; }
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

        if self.0.prot.is_cached_noncoherent() {
            mod_pr_debug!(
                "MMU: remap as uncached {:#x}:{:#x}\n",
                self.iova(),
                self.size()
            );
            if let Err(error) = self.remap_uncached_and_flush() {
                if let Some(fault) = &self.0.uat_inner.fault { fault.store(true, Ordering::Release); }
                pr_err!("UAT: retaining mapping after failed firmware flush ({:?})\n", error);
                return;
            }
        }

        let mut owner = match self.0.owner.exec_lock(None, false) {
            Ok(owner) => owner,
            Err(_) => {
                if let Some(fault) = &self.0.uat_inner.fault { fault.store(true, Ordering::Release); }
                return;
            }
        };
        mod_dev_dbg!(
            owner.dev,
            "MMU: unmap {:#x}:{:#x}\n",
            self.iova(),
            self.size()
        );

        if owner.page_table.unmap_pages(self.iova_range()).is_err() {
            dev_err!(
                owner.dev.as_ref(),
                "MMU: unmap {:#x}:{:#x} failed\n",
                self.iova(),
                self.size()
            );
            if let Some(fault) = &self.0.uat_inner.fault { fault.store(true, Ordering::Release); }
            return;
        }

        if let Some(asid) = owner.slot() {
            fence(Ordering::SeqCst);
            mem::tlbi_range(asid as u8, self.iova() as usize, self.size());
            mod_dev_dbg!(
                owner.dev,
                "MMU: flush range: asid={:#x} start={:#x} len={:#x}\n",
                asid,
                self.iova(),
                self.size()
            );
            mem::sync();
        }
        drop(owner);
        // SAFETY: The PTEs are gone, required flushes completed, and the VM
        // execution lock is released before the node drops BO references.
        unsafe { ManuallyDrop::drop(&mut self.0) };
    }
}

/// Shared UAT global data structures
struct UatShared {
    fault: Option<Arc<AtomicBool>>,
    kernel_ttb1: u64,
    /// Execution contexts leased to logical queues (published-roots UATs).
    execution_contexts: u64,
    context_generations: [u8; UAT_NUM_CTX],
    /// Kernel root of the secondary firmware instance, if there is one.
    secondary_ttb1: Option<PhysicalAddr>,
    map_kernel_to_user: bool,
    handoff_rgn: UatRegion,
    ttbs_rgn: UatRegion,
}

impl UatShared {
    fn lock_handoff(&self) -> Result<HandoffGuard<'_>> {
        if self.fault.is_none() { return Ok(self.handoff().lock_legacy()); }

        if self.fault.as_ref().is_some_and(|fault| fault.load(Ordering::Acquire)) { return Err(EIO); }
        self.handoff().try_lock().inspect_err(|_| {
            if let Some(fault) = &self.fault { fault.store(true, Ordering::Release); }
        })
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
    fn contexts_naming_root(&self, ttb: u64) -> u64 {
        self.ttbs()
            .iter()
            .enumerate()
            .filter(|(ctx, slot)| slot.ttb0.load(Ordering::Acquire) == tagged_root(ttb, *ctx))
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
    m3_running: bool,
    fault: Option<Arc<AtomicBool>>,
    firmware_cache_flush_ready: AtomicBool,
    /// Clean page tables to the point of coherency for a walker that does not snoop the CPU
    /// caches. Set only by [`Uat::new_adopted`].
    clean_poc: bool,
    #[pin]
    shared: Mutex<UatShared>,
    #[pin]
    handoff_flush: [Mutex<HandoffFlush>; UAT_NUM_CTX + 1],
}

impl UatInner {
    fn firmware_cache_flush_ready(&self) -> bool {
        self.firmware_cache_flush_ready.load(Ordering::Acquire)
    }

    /// Take the lock on the shared data and return the guard.
    fn lock(&self) -> Guard<'_, UatShared, MutexBackend> {
        self.shared.lock()
    }

    /// Take a lock on a handoff flush slot and return the guard.
    fn lock_flush(&self, slot: u32) -> Guard<'_, HandoffFlush, MutexBackend> {
        self.handoff_flush[slot as usize].lock()
    }
}

/// Top-level UAT manager object
pub(crate) struct Uat {
    dev: driver::AsahiDevRef,
    cfg: UatConfig,
    geometry: UatGeometry,

    inner: Arc<UatInner>,
    slots: slotalloc::SlotAllocator<SlotInner>,

    kernel_vm: Vm,
    kernel_lower_vm: Vm,
}

struct HandoffGuard<'a>(&'a Handoff, bool);

impl Drop for HandoffGuard<'_> {
    fn drop(&mut self) {
        if self.1 {
            self.0.turn.store(1, Ordering::Relaxed);
            self.0.lock_ap.store(0, Ordering::Release);
        } else {
            self.0.unlock();
        }
    }
}

impl Handoff {
    fn lock_legacy(&self) -> HandoffGuard<'_> {
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
        HandoffGuard(self, true)
    }

    /// Acquire firmware exclusion or return an error after a bounded wait.
    /// The guard releases exclusion on every return path, including errors.
    fn try_lock(&self) -> Result<HandoffGuard<'_>> {
        let start = Instant::<Monotonic>::now();
        self.try_lock_until(|| start.elapsed() >= Delta::from_millis(100))
            .inspect_err(|_| {
                pr_err!("UAT handoff: firmware exclusion timed out; AP interest withdrawn\n");
            })
    }

    /// `try_lock` with a caller-supplied expiry test. A timeout returns
    /// ETIMEDOUT without logging, so the probe self-check stays quiet.
    fn try_lock_until(&self, expired: impl FnMut() -> bool) -> Result<HandoffGuard<'_>> {
        if !crate::handoff_lock::acquire(&self.lock_ap, &self.lock_fw, &self.turn,
            expired,
            || fsleep(Delta::from_micros(20))) {
            return Err(ETIMEDOUT);
        }
        Ok(HandoffGuard(self, false))
    }

    /// Unlock the handoff region, allowing firmware access
    fn unlock(&self) {
        crate::handoff_lock::release(&self.lock_ap, &self.turn);
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

    /// Initialize the handoff region.
    ///
    /// In [`HandoffMode::FirmwareDekker`] this blocks (bounded) until the
    /// firmware publishes `magic_fw`. The stopped M3 owner initializes
    /// its AP state before firmware starts.
    fn init(&self, mode: HandoffMode, m3: bool, firmware: UatFirmware) -> Result {
        self.magic_ap.store(PPL_MAGIC, Ordering::Relaxed);
        self.cur_slot.store(if m3 && mode != HandoffMode::FirmwareDekker { u32::MAX } else { 0 }, Ordering::Relaxed);
        self.unk3.store(0, Ordering::Relaxed);
        if mode == HandoffMode::FirmwareG16 {
            // This mode is constructed only with ASC stopped. Preserve the
            // firmware magic but initialize the AP producer's lock/flush state.
            self.lock_ap.store(0, Ordering::Relaxed);
            self.lock_fw.store(0, Ordering::Relaxed);
            self.turn.store(0, Ordering::Relaxed);
            if (!m3 || mode == HandoffMode::FirmwareG16) && self.magic_fw.load(Ordering::Relaxed) != PPL_MAGIC {
                self.unk2.store(1, Ordering::Relaxed);
            }
        }
        fence(Ordering::SeqCst);

        // Published-roots (G17) firmware never touches the handoff region.
        if matches!(mode, HandoffMode::FirmwareDekker | HandoffMode::FirmwareM3)
            && firmware == UatFirmware::Handoff
        {
            let start = Instant::<Monotonic>::now();
            const TIMEOUT: Delta = Delta::from_millis(1000);

            let mut guard = if m3 { self.try_lock()? } else { self.lock_legacy() };
            while start.elapsed() < TIMEOUT {
                if self.magic_fw.load(Ordering::Relaxed) == PPL_MAGIC {
                    break;
                } else {
                    drop(guard);
                    fsleep(Delta::from_millis(10));
                    guard = if m3 { self.try_lock()? } else { self.lock_legacy() };
                }
            }

            if self.magic_fw.load(Ordering::Relaxed) != PPL_MAGIC {
                drop(guard);
                pr_err!("Handoff: Failed to initialize (firmware not running?)\n");
                return Err(EIO);
            }

            drop(guard);
        }

        for i in 0..=UAT_NUM_CTX {
            self.flush[i].state.store(0, Ordering::Relaxed);
            self.flush[i].addr.store(0, Ordering::Relaxed);
            self.flush[i].size.store(0, Ordering::Relaxed);
        }
        fence(Ordering::SeqCst);
        Ok(())
    }
}

/// Exercise the exact guard implementation on private CPU memory, including
/// timeout unwinding. This does not touch the firmware handoff reservation.
pub(crate) fn check_handoff_guard() -> Result {
    // SAFETY: Every field is an integer atomic, for which zero is valid.
    let handoff: Handoff = unsafe { core::mem::zeroed() };
    {
        let _guard = handoff.try_lock()?;
        if handoff.lock_ap.load(Ordering::Acquire) != 1 { return Err(EIO); }
    }
    if handoff.lock_ap.load(Ordering::Acquire) != 0
        || handoff.turn.load(Ordering::Acquire) != 1 { return Err(EIO); }
    handoff.cur_slot.store(u32::MAX, Ordering::Release);
    if handoff.current_slot().is_some() { return Err(EIO); }
    handoff.lock_fw.store(1, Ordering::Release);
    for turn in [0, 1] {
        handoff.turn.store(turn, Ordering::Release);
        // Expire on the second poll instead of waiting out the 100 ms bound.
        // Turn 1 then times out while yielding to the firmware, and turn 0
        // while waiting for the firmware flag, so both timeout paths still run.
        let mut polls = 0u32;
        if handoff.try_lock_until(|| { polls += 1; polls >= 2 }).err() != Some(ETIMEDOUT)
            || handoff.lock_ap.load(Ordering::Acquire) != 0
            || handoff.lock_fw.load(Ordering::Acquire) != 1 { return Err(EIO); }
    }
    handoff.lock_fw.store(0, Ordering::Release);
    let _guard = handoff.try_lock()?;
    Ok(())
}

/// Represents a single flush info slot in the handoff region.
///
/// # Invariants
/// The pointer is valid and there is no aliasing HandoffFlush instance.
struct HandoffFlush(*const FlushInfo);

// SAFETY: These pointers are safe to send across threads.
unsafe impl Send for HandoffFlush {}

impl HandoffFlush {
    fn end_flush_legacy(&self) {
        // SAFETY: Per the type invariant, this is safe
        let flush = unsafe { self.0.as_ref().unwrap() };
        let state = flush.state.load(Ordering::Relaxed);
        if state != 2 {
            pr_err!("Handoff: expected flush state 2, got {}\n", state);
        }
        flush.state.store(0, Ordering::Relaxed);
    }

    fn begin_flush_legacy(&self, start: u64, size: u64) {
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

    /// Set up a flush operation for the coprocessor
    fn begin_flush(&self, start: u64, size: u64) -> Result {
        // SAFETY: Per the type invariant, this is safe
        let flush = unsafe { self.0.as_ref().unwrap() };

        let state = flush.state.load(Ordering::Relaxed);
        if state != 0 {
            pr_err!("Handoff: expected flush state 0, got {}\n", state);
            return Err(EBUSY);
        }
        flush.addr.store(start, Ordering::Relaxed);
        flush.size.store(size, Ordering::Relaxed);
        flush.state.store(1, Ordering::Relaxed);
        Ok(())
    }

    /// Complete a flush operation for the coprocessor
    fn end_flush(&self) -> Result {
        // SAFETY: Per the type invariant, this is safe
        let flush = unsafe { self.0.as_ref().unwrap() };
        let state = flush.state.load(Ordering::Relaxed);
        // Firmware acknowledges a consumed flush by moving state to 2.
        let expected = 2;
        if state != expected {
            pr_err!(
                "Handoff: expected flush state {}, got {}\n",
                expected,
                state
            );
            return Err(EIO);
        }
        flush.state.store(0, Ordering::Relaxed);
        Ok(())
    }
}

impl Vm {
    /// Whether `other` is the same DRM VM.
    pub(crate) fn same(&self, other: &Vm) -> bool { self.id == other.id }
    /// The DRM VM's id.
    pub(crate) fn id(&self) -> u64 { self.id }
    pub(crate) fn agx_status(&self) -> Result<&Arc<crate::agx_status::VmStatus>> { self.agx_status.as_ref().ok_or(EINVAL) }
    pub(crate) fn is_m3(&self) -> bool { self.fault.is_some() }
    /// Whether this VM belongs to a published-roots (G17) UAT.
    fn published(&self) -> bool { self.mapping_epoch.is_some() }

    /// Create a new virtual memory address space of a published-roots (G17) UAT.
    fn new_published(
        dev: &driver::AsahiDevice,
        uat_inner: Arc<UatInner>,
        kernel_range: Range<u64>,
        geometry: UatGeometry,
        ttb: Option<PhysicalAddr>,
        id: u64,
    ) -> Result<Vm> {
        let dummy_obj = gem::new_kernel_object(dev, UAT_PGSZ)?;
        let is_kernel = ttb.is_some();

        let page_table = if let Some(ttb) = ttb {
            UatPageTable::new_with_ttb(
                ttb,
                geometry.kernel_range(),
                geometry.ias,
                geometry.oas,
                uat_inner.firmware != UatFirmware::Handoff,
            )?
        } else {
            UatPageTable::new(geometry.ias, geometry.oas)?
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
        let binding_clone = binding.clone();
        Ok(Vm {
            fault: uat_inner.fault.clone(),
            driver_mappings: None,
            agx_status: None,
            job_lifetime: None,
            id,
            mapping_epoch: mapping_epoch.clone(),
            lifetime: None,
            residency: None,
            context_bindings: None,
            shared_bindings: None,
            status: None,
            dummy_obj: dummy_obj.gem.clone(),
            inner: gpuvm::GpuVm::new_deferred_bo_put(
                c_str!("Asahi::GpuVm"),
                // TODO: should we using DRM_GPUVM_RESV_PROTECTED as well?
                drm_gpuvm_flags_DRM_GPUVM_IMMEDIATE_MODE,
                dev,
                dummy_obj.gem.clone(),
                gpuvm_range,
                kernel_range,
                init!(VmInner {
                    dev: dev.into(),
                    va_range,
                    is_kernel,
                    fixed_slot: None,
                    page_table,
                    mapping_epoch,
                    epoch: Some(0),
                    mm,
                    uat_inner,
                    binding: binding_clone,
                    id,
                }),
            )?,
            binding,
        })
    }

    /// Attach a status before sharing a newly created user VM with queues.
    pub(crate) fn with_status(mut self) -> Result<Self> {
        self.status = Some(Arc::pin_init(crate::g17::status::VmStatus::new(), GFP_KERNEL)?);
        self.lifetime = Some(lifetime::VmLifetime::new()?);
        self.residency = Some(residency::Gate::new()?);
        self.context_bindings = Some(context::ContextBindings::new()?);
        self.shared_bindings = Some(shared::SharedBindings::new()?);
        Ok(self)
    }

    /// Attach a status and job lifetime tracking before sharing a newly created user VM with
    /// queues, without the G17 canonical context views, shared-object footprints and idle
    /// residency reclaim of [`Self::with_status`]. Used by G15, whose user objects stay pinned.
    pub(crate) fn with_tracked_status(mut self) -> Result<Self> {
        self.status = Some(Arc::pin_init(crate::g17::status::VmStatus::new(), GFP_KERNEL)?);
        self.lifetime = Some(lifetime::VmLifetime::new()?);
        Ok(self)
    }

    /// Copies `buf.len()` bytes of GPU-readable memory at `va` of this VM into `buf`, for admission
    /// checks of streams the GPU will execute (G15 stream gates).
    ///
    /// Every page must be mapped GPU-readable and be RAM; otherwise EFAULT. The lines are cleaned
    /// and invalidated to the point of coherency first (never only invalidated: CPU-written lines
    /// may be dirty), so the copy sees what the GPU or a write-combined CPU view stored. The
    /// mappings cannot change during the copy (GPUVM exec lock).
    pub(crate) fn read_user(&self, va: u64, buf: &mut [u8]) -> Result {
        let mut inner = self.inner.exec_lock(None, false)?;
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
        let Ok(mut inner) = self.inner.exec_lock(None, false) else {
            return false;
        };
        addresses.iter().all(|&address| {
            let Some(guard) = address.checked_add(block_size) else {
                return false;
            };
            Self::covers_range_locked(&mut inner, address, block_size, true, true)
                && !Self::covers_range_locked(
                    &mut inner, guard, stride - block_size, false, false,
                )
        })
    }

    /// Checks all exact request tuples under one GPUVM exec lock. Access bit0
    /// requests reading and bit1 writing. The epoch names this same snapshot.
    pub(crate) fn covers_ranges_batch(&self, ranges: &[(u64, u64, u8)]) -> (bool, Option<u64>) {
        let Ok(mut inner) = self.inner.exec_lock(None, false) else {
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

    /// Create a new virtual memory address space
    fn new(
        dev: &driver::AsahiDevice,
        uat_inner: Arc<UatInner>,
        kernel_range: Range<u64>,
        cfg: UatConfig,
        ttb: Option<PhysicalAddr>,
        id: u64,
    ) -> Result<Vm> {
        if uat_inner.fault.as_ref().is_some_and(|fault| fault.load(Ordering::Acquire)) { return Err(EIO); }
        let fault = uat_inner.fault.clone();
        let dummy_obj = gem::new_kernel_object(dev, UAT_PGSZ)?;
        let is_kernel = ttb.is_some();
        let iova_kern_range = iova_kern_range(cfg)?;

        let page_table = if let Some(ttb) = ttb {
            if cfg.reserved_tables {
                let node = dev.as_ref().of_node().ok_or(ENODEV)?;
                let mut resources = KVec::new();
                for name in [c_str!("pagetables"), c_str!("shared-l2")] {
                    resources.push(crate::m3_resources::reserved_resource(&node, name)?, GFP_KERNEL)?;
                }
                // SAFETY: the owner retains these admitted reservations until ASC stops.
                // Running firmware roots are preserved until host InitData is published.
                let wc = cfg.reserved_tables
                    && [c_str!("pagetables"), c_str!("shared-l2")].iter().all(|name| crate::m3_board::region_is_nomap(&node, name));
                let tables = unsafe { if wc {crate::pgtable_memory::ReservedTables::new_wc(resources)} else {crate::pgtable_memory::ReservedTables::new(resources)} }?;
                if cfg.reserved_tables && uat_inner.m3_running {
                    // RTKit has switched its private mapping regime. Preserve
                    // firmware root entries 0/1; the host window is unused
                    // until this owner publishes initdata.
                    unsafe { UatPageTable::new_with_m3_live_ttb(ttb,iova_kern_range.clone(),tables,dev) }?
                } else {
                    let mut table = unsafe { UatPageTable::new_with_reserved_ttb(ttb, iova_kern_range.clone(), cfg.ias, cfg.oas, tables) }?;
                    let shared = crate::m3_resources::reserved_resource(&node, c_str!("shared-l2"))?;
                    table.install_reserved_middle(iova_kern_range.start, shared.start())?;
                    table
                }
            } else {
                UatPageTable::new_with_ttb_by_geometry(ttb, iova_kern_range.clone(), cfg.ias, cfg.oas)?
            }
        } else if cfg.reserved_tables {
            UatPageTable::new_m3_coherent(cfg.ias,cfg.oas,dev)?

        } else {
            UatPageTable::new_by_geometry(cfg.ias, cfg.oas)?
        };
        let page_table = {
            let mut page_table = page_table;
            if uat_inner.clean_poc {
                page_table.enable_clean_poc()?;
            }
            page_table
        };

        let lower_start = IOVA_USER_BASE;
        let driver_start = lower_start;
        let (va_range, gpuvm_range) = if is_kernel {
            (iova_kern_range, kernel_range.clone())
        } else {
            (
                driver_start..uat_geometry(cfg)?.user_va_top(),
                lower_start..(uat_geometry(cfg)?.user_va_top() - 2 * UAT_PGSZ as u64),
            )
        };

        let mm = mm::Allocator::new(va_range.start, va_range.range(), ())?;

        let binding = Arc::pin_init(
            new_mutex!(
                VmBinding {
                    binding: None,
                    bind_token: None,
                    active_users: 0,
                    ttb: page_table.ttb(),
                },
                "VmBinding",
            ),
            GFP_KERNEL,
        )?;

        let binding_clone = binding.clone();
        Ok(Vm {
            fault,
            id,
            dummy_obj: dummy_obj.gem.clone(),
            inner: gpuvm::GpuVm::new(
                c_str!("Asahi::GpuVm"),
                // TODO: should we using DRM_GPUVM_RESV_PROTECTED as well?
                drm_gpuvm_flags_DRM_GPUVM_IMMEDIATE_MODE,
                dev,
                dummy_obj.gem.clone(),
                gpuvm_range,
                kernel_range,
                init!(VmInner {
                    dev: dev.into(),
                    va_range,
                    is_kernel,
                    fixed_slot: None,
                    page_table,
                    mapping_epoch: None,
                    epoch: Some(0),
                    mm,
                    uat_inner,
                    binding: binding_clone,
                    id,
                }),
            )?,
            binding,
            mapping_epoch: None,
            lifetime: None,
            residency: None,
            context_bindings: None,
            shared_bindings: None,
            status: None,
            driver_mappings: if cfg.m3_owner() {
                Some(Arc::pin_init(new_mutex!(None, "VmDriverMappings"), GFP_KERNEL)?)
            } else { None },
            agx_status: if cfg.m3_owner() {
                Some(Arc::new(crate::agx_status::VmStatus::new(), GFP_KERNEL)?)
            } else { None },
            job_lifetime: if cfg.m3_owner() {
                Some(Arc::pin_init(new_mutex!(M3VmJobLifetime {
                    active: 0, closed: false, close_ranges: None, closed_objects: KVec::new(),
                }, "M3VmJobLifetime"), GFP_KERNEL)?)
            } else { None },

        })
    }

    /// Get the translation table base for this Vm
    fn ttb(&self) -> u64 {
        self.binding.lock().ttb
    }

    /// Retain hardware-required fixed aliases for this VM's complete
    /// lifetime. Installing them before the VM reaches userspace also makes
    /// the GPUVM allocator reserve the addresses against user mappings.
    ///
    /// `install_driver_mappings` of every VM except published-roots (G17) ones.
    fn m3_install_driver_mappings(
        &self,
        mappings: KVec<KernelMapping>,
        reserved_ranges: KVec<Range<u64>>,
    ) -> Result {
        let mut retained = self.driver_mappings.as_ref().ok_or(EINVAL)?.lock();
        if retained.is_some() {
            return Err(EBUSY);
        }
        *retained = Some(VmDriverMappings {
            mappings,
            reserved_ranges,
        });
        Ok(())
    }

    /// `driver_range_overlaps` of every VM except published-roots (G17) ones.
    fn m3_driver_range_overlaps(&self, range: Range<u64>) -> bool {
        let Some(driver_mappings) = &self.driver_mappings else { return false; };
        driver_mappings.lock().as_ref().is_some_and(|driver| {
            any_range_overlaps(
                range.clone(),
                driver.iter_mappings().map(KernelMapping::iova_range),
            ) || any_range_overlaps(range, driver.reserved_ranges.iter().cloned())
        })
    }

    pub(crate) fn retain_m3_job(&self) -> Result<M3VmJobGuard> {
        let mut state = self.job_lifetime.as_ref().ok_or(EINVAL)?.lock();
        if state.closed { return Err(ENOENT); }
        state.active = state.active.checked_add(1).ok_or(EOVERFLOW)?;
        Ok(M3VmJobGuard { vm: self.clone() })
    }

    /// Remove userspace mappings while preserving VM-lifetime driver aliases.
    ///
    /// `unmap_user_ranges` of every VM except published-roots (G17) ones.
    fn m3_unmap_user_ranges(
        &self,
        user_range: Range<u64>,
        kernel_range: Range<u64>,
    ) -> Result {
        if let Some(job_lifetime) = &self.job_lifetime {
            let mut state = job_lifetime.lock();
            state.closed = true;
            if state.active != 0 {
                state.close_ranges = Some((user_range, kernel_range));
                return Ok(());
            }
        }
        let Some(driver_mappings) = &self.driver_mappings else {
            let left = user_range.start..kernel_range.start;
            let right = kernel_range.end..user_range.end;
            if !left.is_empty() { self.unmap_range(left.start, left.range())?; }
            if !right.is_empty() { self.unmap_range(right.start, right.range())?; }
            return Ok(());
        };

        let retained = driver_mappings.lock();
        let driver = retained.as_ref();
        let mut cursor = user_range.start;

        while cursor < user_range.end {
            if kernel_range.contains(&cursor) {
                cursor = kernel_range.end.min(user_range.end);
                continue;
            }

            let mut next = user_range.end;
            let mut covering_end = cursor;
            if let Some(driver) = driver {
                for mapping in driver.iter_mappings() {
                    let range = mapping.iova_range();
                    if range.contains(&cursor) {
                        covering_end = covering_end.max(range.end);
                    } else if cursor < range.start {
                        next = next.min(range.start);
                    }
                }
                for range in driver.reserved_ranges.iter() {
                    if range.contains(&cursor) {
                        covering_end = covering_end.max(range.end);
                    } else if cursor < range.start {
                        next = next.min(range.start);
                    }
                }
            }
            if covering_end != cursor {
                cursor = covering_end.min(user_range.end);
                continue;
            }
            if cursor < kernel_range.start {
                next = next.min(kernel_range.start);
            }
            if next <= cursor {
                return Err(EIO);
            }
            self.unmap_range(cursor, next - cursor)?;
            cursor = next;
        }
        Ok(())
    }

    /// Check that a byte range is fully mapped with the requested GPU access.
    pub(crate) fn covers_range(
        &self,
        address: u64,
        size: u64,
        need_read: bool,
        need_write: bool,
    ) -> bool {
        let mut inner = match self.inner.exec_lock(None, false) {
            Ok(inner) => inner,
            Err(_) => return false,
        };
        inner.covers_mapped_range(address, size, need_read, need_write)
    }

    // `translate_iova` (the same exec-locked page-table lookup) is defined in `context.rs`.

    /// Read GPU-visible bytes out of this VM.
    ///
    /// Fail-closed: the whole span must first pass `covers_range` with GPU
    /// read permission, and every page is then translated and borrowed
    /// individually.  This only ever touches ordinary DRAM backing a GEM
    /// object that is mapped into this VM, never GPU MMIO, so it is safe with
    /// the cores gated.
    ///
    /// The VM execution lock is held from the coverage check until the last
    /// byte is copied, so a concurrent unbind or GEM close cannot free a page
    /// while it is being read.
    ///
    /// M3 compute validates the client control stream before publishing it.
    pub(crate) fn read_bytes(&self, iova: u64, out: &mut [u8]) -> Result {
        if out.is_empty() {
            return Err(EINVAL);
        }
        let mut inner = self.inner.exec_lock(None, false)?;
        if !inner.covers_mapped_range(iova, out.len() as u64, true, false) {
            return Err(EFAULT);
        }
        inner.copy_from_mapped_pages(iova, out, |_, _| {})
    }

    /// Read the bytes this VM maps at `iova`, calling `before_copy` with the
    /// kernel address and length of each chunk, within one page, before it is
    /// copied.
    ///
    /// Unlike `read_bytes`, this checks no GPU access permission: an unmapped
    /// address simply fails translation. The VM execution lock is held from
    /// the first translation until the last copy, so a concurrent unbind or
    /// GEM close cannot free a page while it is being read.
    pub(crate) fn read_mapped_bytes(
        &self,
        iova: u64,
        out: &mut [u8],
        before_copy: impl FnMut(*mut u8, usize),
    ) -> Result {
        if out.is_empty() {
            return Ok(());
        }
        let mut inner = self.inner.exec_lock(None, false)?;
        inner.copy_from_mapped_pages(iova, out, before_copy)
    }

    /// Snapshot the first `out.len()` hardware context-table (TTBAT) entries
    /// as `(ttb0, ttb1)` pairs.
    ///
    /// The TTBAT is ordinary host DRAM, so this reads no GPU MMIO
    /// and is safe with the cores gated. M3 fault diagnostics use it to
    /// identify the tables visible to each hardware context.
    pub(crate) fn context_roots(&self, out: &mut [(u64, u64)]) -> Result {
        let inner = self.inner.exec_lock(None, false)?;
        let shared = inner.uat_inner.lock();
        let ttbs = shared.ttbs();
        for (slot, entry) in out.iter_mut().enumerate() {
            if slot >= UAT_NUM_CTX {
                break;
            }
            *entry = (
                ttbs[slot].ttb0.load(Ordering::Acquire),
                ttbs[slot].ttb1.load(Ordering::Acquire),
            );
        }
        Ok(())
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
        if self.published() {
            return self.map_in_range_with_guard_size(
                gem,
                object_range,
                alignment,
                range,
                prot,
                if guard { UAT_PGSZ } else { 0 },
            );
        }
        let size = object_range.range();
        let sgt = gem.owned_sg_table()?;
        let mut inner = self.inner.exec_lock(Some(gem), false)?;
        let vm_bo = self.inner.obtain_bo(gem)?;

        let mut vm_bo_guard = vm_bo.inner().inner.lock();
        if vm_bo_guard.sgt.is_none() {
            vm_bo_guard.sgt.replace(sgt);
        }
        core::mem::drop(vm_bo_guard);

        let uat_inner = inner.uat_inner.clone();
        // Reserve the VA before attaching objects whose destructors take the
        // GEM reservation lock.  The allocator drops its payload internally
        // on an insertion error, while exec_lock still owns that lock.
        let node_result = inner.mm.insert_node_in_range(
            KernelMappingInner {
                owner: self.inner.clone(),
                uat_inner,
                prot,
                bo: None,
                _gem: None,
                offset: object_range.start,
                mapped_size: size,
            },
            (size + if guard { UAT_PGSZ } else { 0 }) as u64, // Add guard page
            alignment,
            0,
            range.start,
            range.end,
            mm::InsertMode::Best,
        );
        let mut node = match node_result {
            Ok(node) => node,
            Err(err) => {
                core::mem::drop(inner);
                return Err(err);
            }
        };
        {
            let node_inner = node.as_mut().inner_mut();
            node_inner.bo = Some(MappingBo::synchronous(vm_bo));
            node_inner._gem = Some(MappingObject::Shared(gem.into()));
        }

        let ret = inner.map_node(&node, prot);
        // Drop the exec_lock first, so that if map_node failed the
        // KernelMappingInner destructur does not deadlock.
        core::mem::drop(inner);
        ret?;
        Ok(KernelMapping(ManuallyDrop::new(node)))
    }

    /// Map a GEM object into this Vm at a specific address.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn map_at(
        &self,
        addr: u64,
        size: usize,
        gem: ARef<gem::Object>,
        prot: Prot,
        guard: bool,
    ) -> Result<KernelMapping> {
        if self.published() {
            return self.map_at_published(addr, size, gem, prot, guard);
        }
        let sgt = gem.owned_sg_table()?;
        let mut inner = self.inner.exec_lock(Some(&gem), false)?;

        let vm_bo = self.inner.obtain_bo(&gem)?;

        let mut vm_bo_guard = vm_bo.inner().inner.lock();
        if vm_bo_guard.sgt.is_none() {
            vm_bo_guard.sgt.replace(sgt);
        }
        core::mem::drop(vm_bo_guard);

        let uat_inner = inner.uat_inner.clone();
        // See map_in_range(): keep dma_resv-owning references out of the
        // allocator payload until the fixed-address reservation succeeds.
        let node_result = inner.mm.reserve_node(
            KernelMappingInner {
                owner: self.inner.clone(),
                uat_inner,
                prot,
                bo: None,
                _gem: None,
                offset: 0,
                mapped_size: size,
            },
            addr,
            (size + if guard { UAT_PGSZ } else { 0 }) as u64, // Add guard page
            0,
        );
        let mut node = match node_result {
            Ok(node) => node,
            Err(err) => {
                core::mem::drop(inner);
                return Err(err);
            }
        };
        {
            let node_inner = node.as_mut().inner_mut();
            node_inner.bo = Some(MappingBo::synchronous(vm_bo));
            node_inner._gem = Some(MappingObject::Shared(gem.clone()));
        }

        let ret = inner.map_node(&node, prot);
        // Drop the exec_lock first, so that if map_node failed the
        // KernelMappingInner destructur does not deadlock.
        core::mem::drop(inner);
        ret?;
        Ok(KernelMapping(ManuallyDrop::new(node)))
    }

    /// Map a range of a GEM object into this Vm using GPUVM.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn bind_object(
        &self,
        gem: &gem::Object,
        addr: u64,
        size: u64,
        offset: u64,
        prot: Prot,
        single_page: bool,
    ) -> Result {
        if self.published() {
            return self.bind_object_published(gem, addr, size, offset, prot, single_page);
        }
        // Mapping needs a complete context
        let mut ctx = StepContext {
            new_va: Some(gpuvm::GpuVa::<VmInner>::new(pin_init::default())?),
            prev_va: Some(gpuvm::GpuVa::<VmInner>::new(pin_init::default())?),
            next_va: Some(gpuvm::GpuVa::<VmInner>::new(pin_init::default())?),
            prot,
            ..Default::default()
        };

        let vm_bo = self.inner.obtain_bo(gem)?;
        {
            let mut vm_bo_guard = vm_bo.inner().inner.lock();
            if vm_bo_guard.sgt.is_none() {
                let sgt = gem.owned_sg_table()?;

                if vm_bo_guard.sg_vec.is_none() {
                    let mut sg_vec = KVVec::new();
                    let mut offset = 0;
                    for range in sgt.iter() {
                        let addr = range.dma_address() as usize;
                        let len = range.dma_len() as usize;
                        sg_vec.push((offset, addr..(addr + len)), GFP_KERNEL)?;
                        offset += len;
                    }
                    vm_bo_guard.sg_vec.replace(sg_vec);
                }
                vm_bo_guard.sgt.replace(sgt);
            }
            core::mem::drop(vm_bo_guard);
        }

        let mut inner = self.inner.exec_lock(Some(gem), true)?;

        // Legacy preallocation happens before GPUVM can unmap an old VA.
        if self.fault.is_none() {
            inner.page_table.alloc_pages(addr..(addr + size))?;
        }

        ctx.vm_bo = Some(vm_bo);

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
            (gpuvm::GpuVaFlags::REPEAT, UAT_PGSZ as u32)
        } else {
            (gpuvm::GpuVaFlags::NONE, 0u32)
        };

        mod_dev_dbg!(
            inner.dev,
            "MMU: sm_map: {:#x} [{:#x}] -> {:#x}\n",
            offset,
            size,
            addr
        );
        inner.sm_map(&mut ctx, addr, size, offset, gem_range, flags)
    }

    /// Add a direct MMIO mapping to this Vm at a free address.
    pub(crate) fn map_io(
        &self,
        iova: u64,
        phys: usize,
        size: usize,
        prot: Prot,
    ) -> Result<KernelMapping> {
        if self.published() {
            return self.map_io_published(iova, phys, size, prot);
        }
        self.map_io_internal(Some(iova), 0..0, phys, size, prot)
    }

    /// Allocate a firmware MMIO VA through the same DRM range allocator as GEM.
    pub(crate) fn map_io_in_range(&self, range: Range<u64>, phys: usize,
        size: usize, prot: Prot) -> Result<KernelMapping> {
        self.map_io_internal(None, range, phys, size, prot)
    }

    fn map_io_internal(&self, fixed: Option<u64>, range: Range<u64>, phys: usize,
        size: usize, prot: Prot) -> Result<KernelMapping> {
        let mut inner = self.inner.exec_lock(None, false)?;
        let iova = fixed.unwrap_or(0);
        if size == 0 || (iova as usize | phys | size) & UAT_PGMSK != 0 {
            dev_err!(
                inner.dev.as_ref(),
                "MMU: KernelMapping {:#x}:{:#x} -> {:#x} is not page-aligned\n",
                phys,
                size,
                iova
            );
            return Err(EINVAL);
        }

        // Legacy GPUs only map fixed addresses here, and keep logging each
        // firmware MMIO mapping at boot as they always have.
        if self.fault.is_none() {
            dev_info!(
                inner.dev.as_ref(),
                "MMU: IO map: {:#x}:{:#x} -> {:#x}\n",
                phys,
                size,
                iova
            );
        }

        let payload = KernelMappingInner {
            owner: self.inner.clone(), uat_inner: inner.uat_inner.clone(), prot,
            bo: None, _gem: None, offset: 0, mapped_size: size,
        };
        let node = if let Some(iova) = fixed {
            inner.mm.reserve_node(payload, iova, size as u64, 0)?
        } else {
            inner.mm.insert_node_in_range(payload, size as u64, UAT_PGSZ as u64,
                0, range.start, range.end, mm::InsertMode::Best)?
        };
        let iova = node.start();
        if self.fault.is_some() {
            mod_dev_dbg!(inner.dev, "MMU: IO map: {:#x}:{:#x} -> {:#x}\n", phys, size, iova);
        }

        let prepared = if self.fault.is_some() {
            inner.page_table.prepare_map(iova..iova.checked_add(size as u64).ok_or(EOVERFLOW)?)
        } else { Ok(()) };
        if let Err(error) = prepared {
            // Drop the VM lock before the reserved node and its owner refs.
            core::mem::drop(inner);
            return Err(error);
        }
        let ret = inner.page_table.map_pages(
            iova..(iova + size as u64),
            phys as PhysicalAddr,
            prot,
            false,
        );
        // Drop the exec_lock first, so that if map_node failed the
        // KernelMappingInner destructur does not deadlock.
        core::mem::drop(inner);
        ret?;
        Ok(KernelMapping(ManuallyDrop::new(node)))
    }

    /// Unmap everything in an address range.
    pub(crate) fn unmap_range(&self, iova: u64, size: u64) -> Result {
        if self.published() {
            if self.defer_unmap(iova, size)? {
                return Ok(());
            }
            return self.unmap_range_commit(iova, size, true);
        }
        // Unmapping a range can only do a single split, so just preallocate
        // the prev and next GpuVas
        let mut ctx = StepContext {
            prev_va: Some(gpuvm::GpuVa::<VmInner>::new(pin_init::default())?),
            next_va: Some(gpuvm::GpuVa::<VmInner>::new(pin_init::default())?),
            ..Default::default()
        };

        let mut inner = self.inner.exec_lock(None, false)?;

        mod_dev_dbg!(inner.dev, "MMU: sm_unmap: {:#x}:{:#x}\n", iova, size);
        inner.sm_unmap(&mut ctx, iova, size)
    }

    /// Drop mappings for a given bo.
    ///
    /// `drop_mappings` of every VM except published-roots (G17) ones.
    fn m3_drop_mappings(&self, gem: &gem::Object) -> Result {
        if let Some(job_lifetime) = &self.job_lifetime {
            let mut state = job_lifetime.lock();
            if state.active != 0 {
                if !state.closed_objects.iter().any(|object| core::ptr::eq(&**object, gem)) {
                    state.closed_objects.push(gem.into(), GFP_KERNEL)?;
                }
                return Ok(());
            }
        }

        // Removing whole mappings only does unmaps, so no preallocated VAs
        let mut ctx = Default::default();

        let inner = self.inner.exec_lock(Some(gem), false)?;

        if let Some(bo) = self.inner.find_bo(gem) {
            mod_dev_dbg!(inner.dev, "MMU: bo_unmap\n");
            self.inner.bo_unmap(&mut ctx, &bo)?;
            mod_dev_dbg!(inner.dev, "MMU: bo_unmap done\n");
            // We need to drop the exec_lock first, then the GpuVmBo since that will take the lock itself.
            core::mem::drop(inner);
            core::mem::drop(bo);
        }

        Ok(())
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
        let sgt = crate::gem::held_sg_table(&gem)?;
        let mut inner = self.inner.exec_lock(Some(gem), false)?;
        let vm_bo = self.inner.obtain_bo(gem)?;

        let mut vm_bo_guard = vm_bo.inner().inner.lock();
        if vm_bo_guard.sgt.is_none() {
            vm_bo_guard.sgt.replace(sgt);
        }
        core::mem::drop(vm_bo_guard);

        let mut vm_bo = Some(MappingBo::new(vm_bo, self.inner.clone()));
        let uat_inner = inner.uat_inner.clone();
        // A failed reservation drops its payload while exec still owns GEM
        // reservations. Attach BO/GEM references only after success, since
        // their destructors may acquire the same reservation locks.
        let result = inner.mm.insert_node_in_range(
            KernelMappingInner {
                owner: self.inner.clone(),
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
            mm::InsertMode::Best,
        );
        let mut node = match result {
            Ok(node) => node,
            Err(error) => {
                drop(inner);
                return Err(error);
            }
        };
        {
            let payload = node.as_mut().inner_mut();
            payload.bo = vm_bo.take();
            payload._gem = Some(MappingObject::Pinned(gem::KernelMappingPin::new(gem)));
        }

        let ret = inner.map_node(&node, prot);
        // Drop the exec_lock first, so that if map_node failed the
        // KernelMappingInner destructur does not deadlock.
        core::mem::drop(inner);
        ret?;
        Ok(KernelMapping(ManuallyDrop::new(node)))
    }

    /// `map_at` of a published-roots (G17) VM.
    #[allow(clippy::too_many_arguments)]
    fn map_at_published(
        &self,
        addr: u64,
        size: usize,
        gem: ARef<gem::Object>,
        prot: Prot,
        guard: bool,
    ) -> Result<KernelMapping> {
        let reserved_size = size
            .checked_add(if guard { UAT_PGSZ } else { 0 })
            .ok_or(EOVERFLOW)?;
        let _residency = self.enter_residency()?;
        let sgt = crate::gem::held_sg_table(&gem)?;
        let mut inner = self.inner.exec_lock(Some(&gem), false)?;

        let vm_bo = self.inner.obtain_bo(&gem)?;

        let mut vm_bo_guard = vm_bo.inner().inner.lock();
        if vm_bo_guard.sgt.is_none() {
            vm_bo_guard.sgt.replace(sgt);
        }
        core::mem::drop(vm_bo_guard);

        let mut vm_bo = Some(MappingBo::new(vm_bo, self.inner.clone()));
        let uat_inner = inner.uat_inner.clone();
        // Keep reservation-locking destructors out of the allocator payload
        // until reservation succeeds.
        let result = inner.mm.reserve_node(
            KernelMappingInner {
                owner: self.inner.clone(),
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
                drop(inner);
                return Err(error);
            }
        };
        {
            let payload = node.as_mut().inner_mut();
            payload.bo = vm_bo.take();
            payload._gem = Some(MappingObject::Pinned(gem::KernelMappingPin::new(&gem)));
        }

        let ret = inner.map_node(&node, prot);
        // Drop the exec_lock first, so that if map_node failed the
        // KernelMappingInner destructur does not deadlock.
        core::mem::drop(inner);
        ret?;
        Ok(KernelMapping(ManuallyDrop::new(node)))
    }

    /// `bind_object` of a published-roots (G17) VM.
    #[allow(clippy::too_many_arguments)]
    fn bind_object_published(
        &self,
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
        // Mapping needs a complete context
        let mut ctx = StepContext {
            new_va: Some(gpuvm::GpuVa::<VmInner>::new(pin_init::default())?),
            prev_va: Some(gpuvm::GpuVa::<VmInner>::new(pin_init::default())?),
            next_va: Some(gpuvm::GpuVa::<VmInner>::new(pin_init::default())?),
            prot,
            ..Default::default()
        };

        let vm_bo = self.inner.obtain_bo(gem)?;
        {
            let needs_sgt = vm_bo.inner().inner.lock().sgt.is_none();
            if needs_sgt {
                // Pin without the BO mutex: alias construction can hold the GEM
                // reservation while acquiring this mutex.
                let sgt = crate::gem::held_sg_table(&gem)?;
                let mut vm_bo_guard = vm_bo.inner().inner.lock();
                if vm_bo_guard.sgt.is_none() {
                    if vm_bo_guard.sg_vec.is_none() {
                        let mut sg_vec = KVVec::new();
                        let mut offset = 0;
                        for range in sgt.iter() {
                            let addr = range.dma_address() as usize;
                            let len = range.dma_len() as usize;
                            sg_vec.push((offset, addr..(addr + len)), GFP_KERNEL)?;
                            offset += len;
                        }
                        vm_bo_guard.sg_vec.replace(sg_vec);
                    }
                    vm_bo_guard.sgt.replace(sgt);
                }
                // Release the mutex before a losing initializer drops its SG
                // owner. The declaration order also preserves this on errors.
                drop(vm_bo_guard);
            }
        }

        let mut inner = self.inner.exec_lock(Some(gem), true)?;

        // Preallocate the page tables, to fail early if we ENOMEM
        inner.page_table.alloc_pages(addr..(addr + size))?;

        ctx.vm_bo = Some(vm_bo);

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
            (gpuvm::GpuVaFlags::REPEAT, UAT_PGSZ as u32)
        } else {
            (gpuvm::GpuVaFlags::NONE, 0u32)
        };

        mod_dev_dbg!(
            inner.dev,
            "MMU: sm_map: {:#x} [{:#x}] -> {:#x}\n",
            offset,
            size,
            addr
        );
        let _commit = self.mapping_commit()?;
        inner.sm_map(&mut ctx, addr, size, offset, gem_range, flags)
    }

    /// `map_io` of a published-roots (G17) VM.
    fn map_io_published(
        &self,
        iova: u64,
        phys: usize,
        size: usize,
        prot: Prot,
    ) -> Result<KernelMapping> {
        let mut inner = self.inner.exec_lock(None, false)?;

        if (iova as usize | phys | size) & UAT_PGMSK != 0 {
            dev_err!(
                inner.dev.as_ref(),
                "MMU: KernelMapping {:#x}:{:#x} -> {:#x} is not page-aligned\n",
                phys,
                size,
                iova
            );
            return Err(EINVAL);
        }

        dev_info!(
            inner.dev.as_ref(),
            "MMU: IO map: {:#x}:{:#x} -> {:#x}\n",
            phys,
            size,
            iova
        );

        let uat_inner = inner.uat_inner.clone();
        let node = inner.mm.reserve_node(
            KernelMappingInner {
                owner: self.inner.clone(),
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
        let mutation = inner.mapping_mutation();
        let ret = inner.page_table.map_pages(
            iova..end,
            phys as PhysicalAddr,
            prot,
            false,
        );
        if ret.is_err() {
            inner.rollback_failed_map(iova, size as u64);
        }
        // Drop the exec_lock first, so that if map_node failed the
        // KernelMappingInner destructur does not deadlock.
        drop(mutation);
        core::mem::drop(inner);
        ret?;
        Ok(KernelMapping(ManuallyDrop::new(node)))
    }

    fn unmap_prepared_cleanup(&self, unmap: &mut bind::PreparedUserUnmap) -> Result {
        let mut inner = self.inner.lock_inner();
        // SAFETY: step_unmap/step_remap modify this VM's translations and
        // immediate GPUVA metadata only, using the retained split nodes. They
        // do not access external BO backing and defer all BO reference puts.
        let result = unsafe { inner.sm_unmap_inner(&mut unmap.ctx, unmap.iova, unmap.size) };
        if result.is_ok() {
            self.untrack_shared_range(unmap.iova, unmap.size);
        }
        drop(inner);
        self.bo_deferred_cleanup();
        result
    }

    fn unmap_range_commit(&self, iova: u64, size: u64, user: bool) -> Result {
        let _residency = self.enter_retirement_residency();
        // Unmapping a range can only do a single split, so just preallocate
        // the prev and next GpuVas
        let mut ctx = StepContext {
            prev_va: Some(gpuvm::GpuVa::<VmInner>::new(pin_init::default())?),
            next_va: Some(gpuvm::GpuVa::<VmInner>::new(pin_init::default())?),
            ..Default::default()
        };

        let mut inner = self.inner.exec_lock(None, false)?;

        mod_dev_dbg!(inner.dev, "MMU: sm_unmap: {:#x}:{:#x}\n", iova, size);
        let _commit = if user {
            match self.mapping_commit() {
                Ok(guard) => guard,
                Err(error) => {
                    drop(inner);
                    if self.defer_unmap(iova, size)? {
                        return Ok(());
                    }
                    return Err(error);
                }
            }
        } else {
            None
        };
        inner.sm_unmap(&mut ctx, iova, size)?;
        self.untrack_shared_range(iova, size);
        Ok(())
    }

    fn drop_mappings_now(&self, gem: &gem::Object) -> Result {
        // A tracked VM of a handoff UAT (G15) drops like a published one.
        if !self.published() && self.lifetime.is_none() {
            return self.m3_drop_mappings(gem);
        }
        let _residency = self.enter_retirement_residency();
        // Removing whole mappings only does unmaps, so no preallocated VAs
        let mut ctx = Default::default();

        let inner = match self.inner.exec_lock(Some(gem), false) {
            Err(error) if error == ENOMEM => return self.drop_mappings_low_memory(gem),
            result => result?,
        };

        if let Some(bo) = self.inner.find_bo(gem) {
            mod_dev_dbg!(inner.dev, "MMU: bo_unmap\n");
            let result = self.inner.bo_unmap(&mut ctx, &bo);
            if result.is_ok() {
                // The close callback or deferred object list retains this GEM,
                // so clearing its candidate cannot release its final reference.
                self.untrack_context_object(gem);
                self.untrack_shared_object(gem);
            }
            mod_dev_dbg!(inner.dev, "MMU: bo_unmap done\n");
            // BO release can drain unrelated deferred GEMs, including on an
            // unmap error. Always release reservations before dropping it.
            core::mem::drop(inner);
            core::mem::drop(bo);
            if result == Err(ENOMEM) {
                return self.drop_mappings_low_memory(gem);
            }
            result?;
        }

        Ok(())
    }

    /// Preserve the indexed fast path normally; memory pressure must not lose
    /// accepted object cleanup just because an exec or operation list failed.
    fn drop_mappings_low_memory(&self, gem: &gem::Object) -> Result {
        let mut inner = self.inner.lock_inner();
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
        self.inner.is_extobj(gem)
    }

    /// Check whether an object is external to this GpuVm
    pub(crate) fn bo_deferred_cleanup(&self) {
        self.inner.bo_deferred_cleanup()
    }
}

impl VmInner {
    /// `Drop` of a published-roots (G17) VM.
    fn drop_published(&mut self) {
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

            let uat_inner = self.uat_inner.lock();
            let handoff = uat_inner.handoff().lock_legacy();
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
            }
            drop(handoff);
            core::mem::drop(uat_inner);

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

impl Drop for VmInner {
    fn drop(&mut self) {
        if self.published() {
            self.drop_published();
            return;
        }
        if self.uat_inner.fault.as_ref().is_some_and(|fault| fault.load(Ordering::Acquire)) {
            self.page_table.quarantine();
            core::mem::forget(self.uat_inner.clone());
            return;
        }
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

            let uat_inner = self.uat_inner.lock();
            let handoff_guard = match uat_inner.lock_handoff() {
                Ok(guard) => guard,
                Err(_) => {
                    self.page_table.quarantine();
                    core::mem::forget(self.uat_inner.clone());
                    pr_err!("UAT: retaining VM root after handoff failure\n");
                    return;
                }
            };
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
            }
            drop(handoff_guard);
            core::mem::drop(uat_inner);
            fence(Ordering::SeqCst);

            // In principle we dropped all the KernelMappings already, but we might as
            // well play it safe and invalidate the whole ASID.
            if inval {
                mod_pr_debug!(
                    "VmInner::Drop [{}]: need inval for ASID {:#x}\n",
                    self.id,
                    idx
                );
                mem::tlbi_asid(idx as u8);
                mem::sync();
            }
        }
    }
}

/// Physical regions of a UAT whose handoff the generation has already established itself
/// ([`Uat::new_adopted`]). None of them needs to be in the kernel linear map.
pub(crate) struct AdoptedRegions {
    /// The handoff region (PPL magic, lock and flush requests).
    pub(crate) handoff: Range<PhysicalAddr>,
    /// The TTBAT.
    pub(crate) ttbs: Range<PhysicalAddr>,
    /// The firmware kernel (upper) root, which every user context names as its upper half.
    pub(crate) kernel_root: PhysicalAddr,
}

impl Uat {
    /// Map a region given by its physical range, write-combined.
    fn map_physical(range: &Range<PhysicalAddr>, min_size: usize) -> Result<UatRegion> {
        let size: usize = (range.end.checked_sub(range.start).ok_or(EINVAL)?).try_into()?;
        if size < min_size || range.start & (UAT_PGMSK as PhysicalAddr) != 0 {
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
        extern "C" {
            fn asahi_g15_ram_within(start: u64, size: u64) -> bool;
        }
        // SAFETY: Only queries the resource tree (`g15/glue.c`).
        let ram = unsafe { asahi_g15_ram_within(range.start, size as u64) };
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
    ///
    /// No M3 owner applies: the `UatConfig` is the published-roots one (chip 0, no reserved
    /// tables), so the VMs take the plain handoff paths and the legacy PPL lock.
    pub(crate) fn new_adopted(
        dev: &driver::AsahiDevice,
        geometry: UatGeometry,
        regions: &AdoptedRegions,
    ) -> Result<Self> {
        let cfg = UatConfig::published(geometry)?;
        let handoff_rgn = Self::map_physical(&regions.handoff, HANDOFF_SIZE)?;
        let ttbs_rgn = Self::map_physical(&regions.ttbs, SLOTS_SIZE)?;

        // SAFETY: The Handoff struct layout matches the firmware's view of memory at this
        // address, and the region is at least large enough per the check above.
        let handoff = unsafe { &(handoff_rgn.map.ptr() as *mut Handoff).as_ref().unwrap() };

        let inner = Arc::pin_init(
            try_pin_init!(UatInner {
                firmware: UatFirmware::Handoff,
                m3_running: false,
                fault: None,
                // No handoff cache-flush requests: the generation keeps the PPL protocol itself.
                firmware_cache_flush_ready: AtomicBool::new(false),
                // The G15 UAT walker does not snoop the CPU caches.
                clean_poc: true,
                handoff_flush <- pin_init::pin_init_array_from_fn(|i| {
                    new_mutex!(HandoffFlush(&handoff.flush[i]), "handoff_flush")
                }),
                shared <- new_mutex!(
                    UatShared {
                        fault: None,
                        kernel_ttb1: regions.kernel_root,
                        execution_contexts: !EXECUTION_CONTEXT_MASK,
                        context_generations: [0; UAT_NUM_CTX],
                        secondary_ttb1: None,
                        map_kernel_to_user: true,
                        handoff_rgn,
                        ttbs_rgn,
                    },
                    "uat_shared"
                ),
            }),
            GFP_KERNEL,
        )?;

        // The shadow root is retained for the lifetime of the driver window it describes.
        let shadow = Page::alloc_page(GFP_KERNEL | __GFP_ZERO)?;
        let shadow_root = Page::into_phys(shadow);

        let kernel_lower_vm = Vm::new(
            dev,
            inner.clone(),
            IOVA_USER_BASE..uat_geometry(cfg)?.user_va_top(),
            cfg,
            None,
            1,
        )?;
        let kernel_vm = Vm::new(
            dev,
            inner.clone(),
            iova_kern_range(cfg)?,
            cfg,
            Some(shadow_root),
            0,
        )?;

        Ok(Self {
            dev: dev.into(),
            cfg,
            geometry,
            kernel_vm,
            kernel_lower_vm,
            inner,
            slots: slotalloc::SlotAllocator::new(
                UAT_USER_CTX as u32,
                (),
                |_inner, _slot| Some(SlotInner { generation: 0 }),
                c_str!("Uat::SlotAllocator"),
                static_lock_class!(),
                static_lock_class!(),
            )?,
        })
    }

    /// Root of the kernel lower `Vm`, for TTBAT context 0 of an adopted handoff.
    pub(crate) fn kernel_lower_root(&self) -> PhysicalAddr {
        self.kernel_lower_vm.ttb()
    }

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
        m3: bool,
    ) -> Result<UatRegion> {
        let of_node = dev.of_node().ok_or(EINVAL)?;
        // Only an M3 owner may fall back to its static reserved-memory
        // nodes. Every other GPU keeps the plain lookup and its errors.
        let res = if m3 {
            crate::m3_resources::reserved_resource(&of_node, name)?
        } else {
            of_node.reserved_mem_region_to_resource_byname(name)?
        };
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

        let flags = if cached || !crate::m3_board::region_is_nomap(&of_node, name) {
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
            let owner = vm.inner.exec_lock(None, false)?;
            if owner.is_kernel || !Arc::ptr_eq(&owner.uat_inner, &self.inner) {
                return Err(EINVAL);
            }
            owner.ttb()
        };
        Ok(self.inner.lock().contexts_naming_root(root))
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

    #[cfg(CONFIG_DEV_COREDUMP)]
    pub(crate) fn dump_kernel_pages(&self) -> Result<KVVec<pgtable::DumpedPage>> {
        // Full upper-half range for this SoC's root geometry.
        let full_range = if self.inner.firmware != UatFirmware::Handoff {
            self.geometry.upper_range()
        } else {
            uat_geometry(self.cfg)?.upper_canonical_base()..(!UAT_PGMSK as u64)
        };
        let mut inner = self.kernel_vm.inner.exec_lock(None, false)?;
        inner.page_table.dump_pages(full_range)
    }

    /// Read-only translation of a kernel (TTBR1) VA range through the kernel page tables, which
    /// on G15 are the bootloader-built firmware tree. Returns the physical address and leaf
    /// descriptor of every UAT page in the (UAT-page aligned) range; fails with ENOENT if any
    /// page is unmapped and with an error if a table page is not readable. Never allocates page
    /// tables or modifies a PTE.
    pub(crate) fn translate_kernel_range(
        &self,
        iova_range: Range<u64>,
    ) -> Result<KVec<(PhysicalAddr, u64)>> {
        let base = uat_geometry(self.cfg)?.upper_canonical_base();
        if iova_range.start < base || iova_range.end <= iova_range.start {
            return Err(EINVAL);
        }
        let inner = self.kernel_vm.inner.exec_lock(None, false)?;
        inner.page_table.translate_range(iova_range)
    }

    /// Read-only translation of the single kernel (TTBR1) VA `iova` (G15 bring-up diagnostics).
    /// Returns the physical address of `iova` (page PA plus offset) and the leaf descriptor.
    pub(crate) fn translate_kernel_va(&self, iova: u64) -> Result<(PhysicalAddr, u64)> {
        let pgmsk = UAT_PGMSK as u64;
        let start = iova & !pgmsk;
        let end = start.checked_add(UAT_PGSZ as u64).ok_or(EINVAL)?;
        let pages = self.translate_kernel_range(start..end)?;
        let (pa, pte) = *pages.first().ok_or(ENOENT)?;
        Ok((pa + (iova & pgmsk) as PhysicalAddr, pte))
    }

    /// Returns a read-only snapshot of the firmware handoff region (no handoff lock is taken).
    pub(crate) fn handoff_snapshot(&self) -> HandoffSnapshot {
        let inner = self.inner.lock();
        let h = inner.handoff();
        let magic_fw = h.magic_fw.load(Ordering::Relaxed);
        HandoffSnapshot {
            magic_ap: h.magic_ap.load(Ordering::Relaxed),
            magic_fw,
            magic_fw_ok: magic_fw == PPL_MAGIC,
            lock_ap: h.lock_ap.load(Ordering::Relaxed),
            lock_fw: h.lock_fw.load(Ordering::Relaxed),
            turn: h.turn.load(Ordering::Relaxed),
            cur_slot: h.cur_slot.load(Ordering::Relaxed),
            unk2: h.unk2.load(Ordering::Relaxed),
            unk3: h.unk3.load(Ordering::Relaxed),
            flush0_state: h.flush[0].state.load(Ordering::Relaxed),
            flush_kernel_state: h.flush[UAT_NUM_CTX].state.load(Ordering::Relaxed),
        }
    }

    /// Returns the base physical address of the TTBAT region.
    pub(crate) fn ttb_base(&self) -> u64 {
        let inner = self.inner.lock();

        inner.ttbs_rgn.base
    }

    /// Binds a `Vm` to a slot, preferring the last used one.
    pub(crate) fn bind(&self, vm: &Vm) -> Result<VmBind> {
        let mut binding = vm.binding.lock();

        if binding.binding.is_none() {
            assert_eq!(binding.active_users, 0);

            let isolation = *module_parameters::robust_isolation.value() != 0;

            self.slots.set_limit(if isolation {
                NonZeroUsize::new(1)
            } else {
                None
            });

            let mut slot = self.slots.get(binding.bind_token)?;
            if slot.changed() {
                mod_pr_debug!("Vm Bind [{}]: bind_token={:?}\n", vm.id, slot.token(),);
                let idx = (slot.slot() as usize) + UAT_USER_CTX_START;
                let ttb = binding.ttb | TTBR_VALID | (idx as u64) << TTBR_ASID_SHIFT;

                let uat_inner = self.inner.lock();

                let ttb1 = if uat_inner.map_kernel_to_user {
                    uat_inner.kernel_ttb1 | TTBR_VALID | (idx as u64) << TTBR_ASID_SHIFT
                } else {
                    0
                };

                let ttbs = uat_inner.ttbs();
                let handoff_guard = uat_inner.lock_handoff()?;
                if uat_inner.handoff().current_slot() == Some(idx as u32) {
                    pr_err!(
                        "Vm::bind to slot {}, but it is currently in use by the ASC?\n",
                        idx
                    );
                }
                ttbs[idx].ttb0.store(ttb, Ordering::Release);
                ttbs[idx].ttb1.store(ttb1, Ordering::Release);
                drop(handoff_guard);
                core::mem::drop(uat_inner);

                // Make sure all TLB entries from the previous owner of this ASID are gone
                mem::tlbi_asid(idx as u8);
                mem::sync();
                slot.generation = slot.generation.wrapping_add(1).max(1);
            }

            binding.bind_token = Some(slot.token());
            binding.binding = Some(slot);
        }

        binding.active_users += 1;

        let slot = binding.binding.as_ref().unwrap().slot() + UAT_USER_CTX_START as u32;
        mod_pr_debug!("MMU: slot {} active users {}\n", slot, binding.active_users);
        Ok(VmBind(vm.clone(), slot, binding.binding.as_ref().unwrap().generation))
    }

    /// Allocates a stable application context without changing the VM bind slot.
    /// Both roots and the generation remain unchanged for the lease's lifetime.
    pub(crate) fn new_execution_context(&self, vm: &Vm) -> Result<Arc<ExecutionContext>> {
        if self.inner.firmware == UatFirmware::Handoff {
            return Err(EINVAL);
        }
        {
            let owner = vm.inner.exec_lock(None, false)?;
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
        }
        mem::sync();
        mem::tlbi_asid(id as u8);
        mem::sync();
        Ok(context)
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
            pgtable::with_root_entries(self.kernel_vm.ttb(), UAT_PGSZ / size_of::<u64>(), |ptes| {
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
            pgtable::with_root_entries(self.kernel_vm.ttb(), snapshot.len(), |ptes| {
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
        let secondary = self.inner.lock().secondary_ttb1.ok_or(EINVAL)?;
        let count = UAT_PGSZ / size_of::<u64>();
        let first = IOVA_KERN_TOP_LEVEL_INDEX as usize;

        // SAFETY: Both roots are in the bootloader-reserved `pagetables` region, which outlives
        // this `Uat`.
        unsafe {
            pgtable::with_root_entries(self.kernel_vm.ttb(), count, |primary| {
                pgtable::with_root_entries(secondary, count, |secondary| {
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

    /// Activates the device-global render root in context 1 after the boot records retire.
    /// Both contexts must still contain their original kernel roots.
    pub(crate) fn activate_render_context(&self, vm: &Vm) -> Result {
        let inner = self.inner.lock();
        let handoff = inner.handoff().lock_legacy();
        let ttbs = inner.ttbs();
        let ready = [0, KERNEL_ALIAS_CTX].into_iter().all(|ctx| {
            ttbs[ctx].ttb0.load(Ordering::Acquire) == tagged_root(self.kernel_lower_vm.ttb(), ctx)
                && ttbs[ctx].ttb1.load(Ordering::Acquire) == tagged_root(self.kernel_vm.ttb(), ctx)
        });
        if !ready {
            drop(handoff);
            return Err(EIO);
        }
        ttbs[KERNEL_ALIAS_CTX]
            .ttb0
            .store(tagged_root(vm.ttb(), KERNEL_ALIAS_CTX), Ordering::Release);
        drop(handoff);
        core::mem::drop(inner);
        mem::sync();
        mem::tlbi_asid(KERNEL_ALIAS_CTX as u8);
        mem::sync();
        Ok(())
    }

    /// Creates a new `Vm` linked to this UAT.
    pub(crate) fn new_vm(&self, id: u64, kernel_range: Range<u64>) -> Result<Vm> {
        if self.inner.firmware != UatFirmware::Handoff {
            return Vm::new_published(
                &self.dev,
                self.inner.clone(),
                kernel_range,
                self.geometry,
                None,
                id,
            );
        }
        Vm::new(
            &self.dev,
            self.inner.clone(),
            kernel_range,
            self.cfg,
            None,
            id,
        )
    }

    /// Creates the reference-counted inner data for a new `Uat` instance.
    #[inline(never)]
    fn make_inner(
        dev: &driver::AsahiDevice,
        handoff_mode: HandoffMode,
        m3: bool,
        firmware: UatFirmware,
    ) -> Result<Arc<UatInner>> {
        let cached = !matches!(handoff_mode, HandoffMode::FirmwareM3 | HandoffMode::FirmwareG16);
        let handoff_rgn =
            Self::map_region(dev.as_ref(), c_str!("handoff"), HANDOFF_SIZE, cached, m3)?;
        let ttbs_rgn = Self::map_region(dev.as_ref(), c_str!("ttbs"), SLOTS_SIZE, cached, m3)?;

        // SAFETY: The Handoff struct layout matches the firmware's view of memory at this address,
        // and the region is at least large enough per the size specified above.
        let handoff = unsafe { &(handoff_rgn.map.ptr() as *mut Handoff).as_ref().unwrap() };

        dev_info!(dev.as_ref(), "MMU: Initializing kernel page table\n");

        let fault = if !m3 {
            None
        } else {
            Some(Arc::new(AtomicBool::new(false), GFP_KERNEL)?)
        };
        let shared_fault = fault.clone();
        Arc::pin_init(
            try_pin_init!(UatInner {
                firmware,
                m3_running: handoff_mode == HandoffMode::FirmwareM3,
                fault,
                firmware_cache_flush_ready: AtomicBool::new(
                    handoff_mode == HandoffMode::FirmwareDekker
                ),
                clean_poc: false,
                handoff_flush <- pin_init::pin_init_array_from_fn(|i| {
                    new_mutex!(HandoffFlush(&handoff.flush[i]), "handoff_flush")
                }),
                shared <- new_mutex!(
                    UatShared {
                        fault: shared_fault,
                        kernel_ttb1: 0,
                        execution_contexts: !EXECUTION_CONTEXT_MASK,
                        context_generations: [0; UAT_NUM_CTX],
                        secondary_ttb1: None,
                        map_kernel_to_user: false,
                        handoff_rgn,
                        ttbs_rgn,
                    },
                    "uat_shared"
                ),
            }),
            GFP_KERNEL,
        )
    }

    /// Construct the exact admitted 25G83 UAT while the owner holds ASC stopped.
    ///
    /// # Safety
    /// The caller owns stopped ASC and retains the identified firmware reservations.
    pub(crate) unsafe fn new_j613_25g83(dev: &driver::AsahiDevice,
        firmware: &crate::g16_firmware::Firmware) -> Result<Self> {
        if firmware.board.chip_id != 0x8122 { return Err(ENODEV); }
        let node = dev.as_ref().of_node().ok_or(ENODEV)?;
        for (i, name) in [c_str!("ttbs"), c_str!("pagetables"), c_str!("handoff"),
            c_str!("shared-l2"), c_str!("fw-text"), c_str!("fw-data")].iter().enumerate() {
            let region = crate::m3_resources::reserved_resource(&node, name)?;
            if region.start() != firmware.resources.regions[i].base
                || region.size() != firmware.resources.regions[i].size { return Err(EINVAL); }
        }
        Self::new_with_config(dev, UatConfig::m3_runtime(&crate::m3_soc::T8122)?,
            false, HandoffMode::FirmwareG16)
    }

    /// After firmware publishes its handoff magic, mapping teardown may flush its caches.
    pub(crate) fn enable_g16_flushes(&self) -> Result {
        if !self.cfg.reserved_tables || self.cfg.chip_id != 0x8122 { return Err(EINVAL); }
        let inner = self.inner.lock();
        if inner.handoff().magic_fw.load(Ordering::Acquire) != PPL_MAGIC { return Err(EIO); }
        self.inner.firmware_cache_flush_ready.store(true, Ordering::Release);
        Ok(())
    }

    /// Cached firmware objects can be released after firmware ownership ends.
    ///
    /// # Safety
    /// The 25G83 owner has verified ASC stopped and all submitted GPU work retired.
    pub(crate) unsafe fn disarm_g16_flushes_after_stop(&self) {
        self.inner.firmware_cache_flush_ready.store(false, Ordering::Release);
    }

    /// The M3 runtime's UAT on the admitted `soc`, with its `m3_soc::Soc::hwcfg`.
    ///
    /// # Safety
    /// The admitted RTKit (J514S on T6030) has completed wake, but has not received an initdata
    /// root. The owner retains power and excludes GPU jobs until publication.
    pub(crate) unsafe fn new_m3_running(
        dev: &driver::AsahiDevice,
        soc: &'static crate::m3_soc::Soc,
    ) -> Result<Self> {
        Self::new_with_config(dev, UatConfig::m3_runtime(soc)?, true, HandoffMode::FirmwareM3)
    }

    #[inline(never)]
    pub(crate) fn new_for_config(
        dev: &driver::AsahiDevice,
        cfg: &'static hw::HwConfig,
        map_kernel_to_user: bool,
    ) -> Result<Self> {
        Self::new_with_handoff_mode(dev, cfg, map_kernel_to_user, HandoffMode::FirmwareDekker)
    }

    #[inline(never)]
    pub(crate) fn new_with_handoff_mode(
        dev: &driver::AsahiDevice,
        cfg: &'static hw::HwConfig,
        map_kernel_to_user: bool,
        handoff_mode: HandoffMode,
    ) -> Result<Self> {
        Self::new_with_config(
            dev,
            UatConfig::from_hw(cfg),
            map_kernel_to_user,
            handoff_mode,
        )
    }

    #[inline(never)]
    fn new_with_config(
        dev: &driver::AsahiDevice,
        cfg: UatConfig,
        map_kernel_to_user: bool,
        handoff_mode: HandoffMode,
    ) -> Result<Self> {
        dev_info!(dev.as_ref(), "MMU: Initializing...\n");

        // Validate the geometry first: 39- and 42-bit roots are the two
        // recovered layouts, and the page-table walk, kernel-half VA window,
        // and leaf PTE policy are implemented for both (the AGX3 leaf PTE
        // encoder is byte-identical to AGX2). What must remain fail-closed
        // per mode is the firmware-shared state this constructor touches
        // next: the Handoff/FlushInfo protocol is per-firmware-build ABI
        // observed on AGX2 (Dekker) and, separately, observed *absent* on
        // M3.
        let kernel_range = iova_kern_range(cfg)?;
        match handoff_mode {
            HandoffMode::FirmwareM3 => { if !cfg.reserved_tables {return Err(ENODEV);} }
            HandoffMode::FirmwareG16 => {
                if !cfg.reserved_tables || cfg.chip_id != 0x8122 { return Err(ENODEV); }
            }
            HandoffMode::FirmwareDekker => {
                if cfg.ias != 39 && !AGX3_FIRMWARE_HANDOFF_VALIDATED {
                    dev_err!(
                        dev.as_ref(),
                        "MMU: {}-bit UAT geometry is implemented, but the AGX3 firmware Handoff ABI and kernel-VA map are unvalidated\n",
                        cfg.ias
                    );
                    return Err(ENODEV);
                }
            }

        }

        let inner = Self::make_inner(dev, handoff_mode, cfg.m3_owner(), UatFirmware::Handoff)?;

        let of_node = dev.as_ref().of_node().ok_or(EINVAL)?;
        let res = if cfg.m3_owner() {
            crate::m3_resources::reserved_resource(&of_node, c_str!("pagetables"))?
        } else {
            of_node.reserved_mem_region_to_resource_byname(c_str!("pagetables"))?
        };
        let ttb1 = res.start();
        let ttb1size: usize = res.size().try_into()?;

        let required_pagetables_size = PAGETABLES_SIZE;
        if ttb1size < required_pagetables_size {
            dev_err!(dev.as_ref(), "MMU: Pagetables region is too small\n");
            return Err(ENOMEM);
        }

        dev_info!(dev.as_ref(), "MMU: Creating kernel page tables\n");
        let kernel_lower_vm = Vm::new(
            dev,
            inner.clone(),
            IOVA_USER_BASE..uat_geometry(cfg)?.user_va_top(),
            cfg,
            None,
            1,
        )?;
        if cfg.reserved_tables {
            // M3's bootstrap command register lists are fetched through this
            // low root in context zero. It never acquires a VmBind, but its
            // mappings are live: dropping/remapping a command must invalidate
            // context-zero translations before releasing the old GEM pages.
            // Otherwise reusing the VA can fetch a previous command's ASID,
            // register list and CDM after those client mappings are gone.
            kernel_lower_vm.inner.exec_lock(None, false)?.fixed_slot = Some(0);
        }
        let kernel_vm = Vm::new(dev, inner.clone(), kernel_range, cfg, Some(ttb1), 0)?;

        dev_info!(dev.as_ref(), "MMU: Kernel page tables created\n");

        let ttb0 = kernel_lower_vm.ttb();
        let uat = Self {
            dev: dev.into(),
            cfg,
            geometry: UatGeometry {
                ias: u32::from(cfg.ias),
                oas: cfg.oas,
            },
            kernel_vm,
            kernel_lower_vm,
            inner,
            slots: slotalloc::SlotAllocator::new(
                UAT_USER_CTX as u32,
                (),
                |_inner, _slot| Some(SlotInner { generation: 0 }),
                c_str!("Uat::SlotAllocator"),
                static_lock_class!(),
                static_lock_class!(),
            )?,
        };

        let mut inner = uat.inner.lock();

        inner.map_kernel_to_user = map_kernel_to_user;
        inner.kernel_ttb1 = ttb1;

        inner.handoff().init(handoff_mode, cfg.m3_owner(), UatFirmware::Handoff)?;

        dev_info!(dev.as_ref(), "MMU: Initializing TTBs\n");

        let handoff_guard = inner.lock_handoff()?;

        let ttbs = inner.ttbs();

        ttbs[0].ttb0.store(ttb0 | TTBR_VALID, Ordering::SeqCst);
        ttbs[0].ttb1.store(ttb1 | TTBR_VALID, Ordering::SeqCst);

        for ctx in &ttbs[1..] {
            ctx.ttb0.store(0, Ordering::Relaxed);
            ctx.ttb1.store(0, Ordering::Relaxed);
        }

        drop(handoff_guard);

        core::mem::drop(inner);

        dev_info!(dev.as_ref(), "MMU: initialized\n");

        Ok(uat)
    }

    /// Creates a new published-roots (G17) `Uat` for the given address space geometry.
    #[inline(never)]
    pub(crate) fn new(
        dev: &driver::AsahiDevice,
        geometry: UatGeometry,
        map_kernel_to_user: bool,
        firmware: UatFirmware,
    ) -> Result<Self> {
        dev_info!(dev.as_ref(), "MMU: Initializing...\n");

        if firmware == UatFirmware::Handoff {
            // Handoff UATs are built by `new_for_config` and its siblings.
            return Err(EINVAL);
        }
        let cfg = UatConfig::published(geometry)?;
        let inner = Self::make_inner(dev, HandoffMode::FirmwareDekker, false, firmware)?;

        let of_node = dev.as_ref().of_node().ok_or(EINVAL)?;
        let res = of_node.reserved_mem_region_to_resource_byname(c_str!("pagetables"))?;
        let ttb1 = res.start();
        let ttb1size: usize = res.size().try_into()?;

        let (pagetables_size, secondary_ttb1) = match firmware {
            UatFirmware::Handoff => (PAGETABLES_SIZE, None),
            UatFirmware::PublishedRoots {
                secondary_root_offset,
            } => (
                secondary_root_offset as usize + UAT_PGSZ,
                Some(ttb1 + secondary_root_offset),
            ),
        };
        if ttb1size < pagetables_size {
            dev_err!(dev.as_ref(), "MMU: Pagetables region is too small\n");
            return Err(ENOMEM);
        }

        dev_info!(dev.as_ref(), "MMU: Creating kernel page tables\n");
        let kernel_lower_vm =
            Vm::new_published(dev, inner.clone(), geometry.user_range(), geometry, None, 1)?;
        let kernel_vm = Vm::new_published(
            dev,
            inner.clone(),
            geometry.kernel_range(),
            geometry,
            Some(ttb1),
            0,
        )?;

        dev_info!(dev.as_ref(), "MMU: Kernel page tables created\n");

        let ttb0 = kernel_lower_vm.ttb();

        let uat = Self {
            dev: dev.into(),
            cfg,
            geometry,
            kernel_vm,
            kernel_lower_vm,
            inner,
            slots: slotalloc::SlotAllocator::new(
                UAT_USER_CTX as u32,
                (),
                |_inner, _slot| Some(SlotInner { generation: 0 }),
                c_str!("Uat::SlotAllocator"),
                static_lock_class!(),
                static_lock_class!(),
            )?,
        };

        let mut inner = uat.inner.lock();

        inner.map_kernel_to_user = map_kernel_to_user;
        inner.kernel_ttb1 = ttb1;
        inner.secondary_ttb1 = secondary_ttb1;

        inner.handoff().init(HandoffMode::FirmwareDekker, false, firmware)?;

        dev_info!(dev.as_ref(), "MMU: Initializing TTBs\n");

        let handoff = inner.handoff().lock_legacy();

        let ttbs = inner.ttbs();

        ttbs[0].ttb0.store(ttb0 | TTBR_VALID, Ordering::SeqCst);
        ttbs[0].ttb1.store(ttb1 | TTBR_VALID, Ordering::SeqCst);

        for ctx in &ttbs[1..] {
            ctx.ttb0.store(0, Ordering::Relaxed);
            ctx.ttb1.store(0, Ordering::Relaxed);
        }

        if secondary_ttb1.is_some() {
            let alias = &ttbs[KERNEL_ALIAS_CTX];
            alias
                .ttb0
                .store(tagged_root(ttb0, KERNEL_ALIAS_CTX), Ordering::SeqCst);
            alias
                .ttb1
                .store(tagged_root(ttb1, KERNEL_ALIAS_CTX), Ordering::SeqCst);
        }

        drop(handoff);

        core::mem::drop(inner);

        dev_info!(dev.as_ref(), "MMU: initialized\n");

        Ok(uat)
    }
}

impl Drop for Uat {
    fn drop(&mut self) {
        if self.inner.firmware != UatFirmware::Handoff {
            // Make sure we flush the TLBs
            mem::sync();
            mem::tlbi_all();
            mem::sync();
            return;
        }
        if self.cfg.reserved_tables {
            self.inner.firmware_cache_flush_ready.store(false, Ordering::Release);
            let inner = self.inner.lock();
            let _guard = match inner.lock_handoff() {
                Ok(guard) => guard,
                Err(_) => return, // Faulted VM owners retain their backing.
            };
            for slot in inner.ttbs() {
                slot.ttb0.store(0, Ordering::Release);
                slot.ttb1.store(0, Ordering::Release);
            }
        }

        // Make sure we flush the TLBs
        fence(Ordering::SeqCst);
        mem::tlbi_all();
        mem::sync();
    }
}
