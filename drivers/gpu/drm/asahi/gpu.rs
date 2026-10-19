// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Top-level GPU manager
//!
//! This module is the root of all GPU firmware management for a given driver instance. It is
//! responsible for initialization, owning the top-level managers (events, UAT, etc.), and
//! communicating with the raw RtKit endpoints to send and receive messages to/from the GPU
//! firmware.
//!
//! It is also the point where diverging driver firmware/GPU variants (using the versions macro)
//! are unified, so that the top level of the driver itself (in `driver`) does not have to concern
//! itself with version dependence.

use core::any::Any;
use core::ops::Range;
use core::ptr::NonNull;
use core::slice;
use core::sync::atomic::{
    AtomicBool,
    AtomicI64,
    AtomicU32,
    AtomicU64,
    Ordering, //
};

use kernel::{
    addr::PhysicalAddr,
    bindings,
    c_str,
    drm::gem::shmem,
    error::code::*,
    io::mem::{
        Mem,
        MemFlag, //
    },
    iosys_map::IoSysMapRef,
    macros::versions,
    new_mutex,
    page::{
        Page,
        PAGE_SIZE, //
    },
    prelude::*,
    soc::apple::rtkit,
    sync::{
        lock::{
            mutex::MutexBackend,
            Guard, //
        },
        Arc,
        Mutex,
        UniqueArc, //
    },
    time::{
        delay::fsleep,
        Delta,
        Instant,
        Monotonic, //
    },
    types::ForeignOwnable, //
};
#[cfg(CONFIG_DEV_COREDUMP)]
use kernel::{
    devcoredump,
    time::msecs_to_jiffies, //
};

use crate::alloc::Allocator;
use crate::debug::*;
use crate::driver::{
    AsahiDevRef,
    AsahiDevice, //
};
use crate::fw::channels::{
    ChannelErrorType,
    PipeType, //
};
use crate::fw::types::{
    U32,
    U64, //
};
use crate::{
    alloc,
    buffer,
    channel,
    event,
    fw,
    gem,
    hw,
    initdata,
    mem,
    mmu,
    queue,
    regs,
    workqueue, //
};

const DEBUG_CLASS: DebugFlags = DebugFlags::Gpu;

/// Firmware endpoint for init & incoming notifications.
const EP_FIRMWARE: u8 = 0x20;

/// Doorbell endpoint for work/message submissions.
const EP_DOORBELL: u8 = 0x21;

/// Initialize the GPU firmware.
const MSG_INIT: u64 = 0x81 << 48;
const INIT_DATA_MASK: u64 = (1 << 44) - 1;

/// TX channel doorbell.
const MSG_TX_DOORBELL: u64 = 0x83 << 48;
/// Firmware control channel doorbell.
const MSG_FWCTL: u64 = 0x84 << 48;
// /// Halt the firmware (?).
// const MSG_HALT: u64 = 0x85 << 48;

/// Receive channel doorbell notification.
const MSG_RX_DOORBELL: u64 = 0x42 << 48;

/// Doorbell number for firmware kicks/wakeups.
const DOORBELL_KICKFW: u64 = 0x10;
/// Doorbell number for device control channel kicks.
const DOORBELL_DEVCTRL: u64 = 0x11;

// Upper kernel half VA address ranges.
//
// These are the G13/G14 (39-bit IAS) addresses. Every use goes through `mmu::kern_iova()` /
// `mmu::kern_iova_range()`, which rebases them to 0xfffffc20_... on G15 (42-bit IAS), keeping the
// same 0x20_0000_0000 offset from the TTBR1 base.
/// Private (cached) firmware structure VA range base.
const IOVA_KERN_PRIV_RANGE: Range<u64> = 0xffffffa000000000..0xffffffa600000000;
/// Private (cached) GPU-RO firmware structure VA range base.
const IOVA_KERN_GPU_RO_RANGE: Range<u64> = 0xffffffa600000000..0xffffffa800000000;
/// Shared (uncached) firmware structure VA range base.
const IOVA_KERN_SHARED_RANGE: Range<u64> = 0xffffffa800000000..0xffffffaa00000000;
/// Shared (uncached) read-only firmware structure VA range base.
const IOVA_KERN_SHARED_RO_RANGE: Range<u64> = 0xffffffaa00000000..0xffffffac00000000;
/// GPU/FW shared structure VA range base.
const IOVA_KERN_GPU_RANGE: Range<u64> = 0xffffffac00000000..0xffffffae00000000;
/// GPU/FW shared structure VA range base.
const IOVA_KERN_RTKIT_RANGE: Range<u64> = 0xffffffae00000000..0xffffffae10000000;
/// Shared (uncached) timestamp region.
pub(crate) const IOVA_KERN_TIMESTAMP_RANGE: Range<u64> = 0xffffffae10000000..0xffffffae14000000;
/// FW MMIO VA range base.
const IOVA_KERN_MMIO_RANGE: Range<u64> = 0xffffffaf00000000..0xffffffb000000000;

/// GPU/FW buffer manager control address (context 0 low)
pub(crate) const IOVA_KERN_GPU_BUFMGR_LOW: u64 = 0x20_0000_0000;
/// GPU/FW buffer manager control address (context 0 high)
pub(crate) const IOVA_KERN_GPU_BUFMGR_HIGH: u64 = 0xffffffaeffff0000;

/// G15: value written into the P2 debug block init word (+0x14) before MSG_INIT. The firmware
/// overwrites it with 0 when it consumes the INIT handshake and with 1 when its post-INIT
/// initialisation is done.
const G15_INIT_SENTINEL: u32 = 0xa5a5_a5a5;
/// G15: how long init() polls the P2 init word after MSG_INIT.
const G15_INIT_POLL_TIMEOUT_MS: i64 = 2000;
/// G15: devctl liveness probe sample points (ms after the doorbell).
const G15_PROBE_CHECKPOINTS_MS: [i64; 5] = [10, 50, 100, 250, 500];
/// G15: ring the devctl doorbell once more if the read index has not moved by this point.
const G15_PROBE_RERING_MS: i64 = 100;

/// Returns the kernel monotonic time in ns (the same clock as the rtkit.c trace lines).
fn ktime_ns() -> i64 {
    // SAFETY: ktime_get() is always safe to call outside NMI context.
    unsafe { bindings::ktime_get() }
}

/// Timeout for entering the halt state after a fault or request.
const HALT_ENTER_TIMEOUT: Delta = Delta::from_millis(100);

/// Maximum amount of firmware-private memory garbage allowed before collection.
/// Collection flushes the FW cache and is expensive, so this needs to be
/// reasonably high.
const MAX_FW_ALLOC_GARBAGE_BYTES: usize = 16 * 1024 * 1024;
/// Maximum count of firmware-private memory garbage objects allowed before collection.
/// This works out to 16K of memory in the garbage list (8 bytes each), which keeps us
/// within the safe range for kmalloc (on 16K page systems).
const MAX_FW_ALLOC_GARBAGE_OBJECTS: usize = 2048;

/// Global allocators used for kernel-half structures.
pub(crate) struct KernelAllocators {
    pub(crate) private: alloc::DefaultAllocator,
    pub(crate) shared: alloc::DefaultAllocator,
    pub(crate) shared_ro: alloc::DefaultAllocator,
    #[allow(dead_code)]
    pub(crate) gpu: alloc::DefaultAllocator,
    pub(crate) gpu_ro: alloc::DefaultAllocator,
}

/// Receive (GPU->driver) ring buffer channels.
#[versions(AGX)]
#[pin_data]
struct RxChannels {
    event: channel::EventChannel::ver,
    fw_log: channel::FwLogChannel,
    ktrace: channel::KTraceChannel,
    stats: channel::StatsChannel::ver,
}

/// GPU work submission pipe channels (driver->GPU).
#[versions(AGX)]
struct PipeChannels {
    pub(crate) vtx: KVec<Pin<KBox<Mutex<channel::PipeChannel::ver>>>>,
    pub(crate) frag: KVec<Pin<KBox<Mutex<channel::PipeChannel::ver>>>>,
    pub(crate) comp: KVec<Pin<KBox<Mutex<channel::PipeChannel::ver>>>>,
}

/// Misc command transmit (driver->GPU) channels.
#[versions(AGX)]
#[pin_data]
struct TxChannels {
    pub(crate) device_control: channel::DeviceControlChannel::ver,
}

/// Number of work submission pipes per type, one for each priority level.
const NUM_PIPES: usize = 4;

/// A generic monotonically incrementing ID used to uniquely identify object instances within the
/// driver.
pub(crate) struct ID(AtomicU64);

impl ID {
    /// Create a new ID counter with a given value.
    fn new(val: u64) -> ID {
        ID(AtomicU64::new(val))
    }

    /// Fetch the next unique ID.
    pub(crate) fn next(&self) -> u64 {
        self.0.fetch_add(1, Ordering::Relaxed)
    }
}

impl Default for ID {
    /// IDs default to starting at 2, as 0/1 are considered reserved for the system.
    fn default() -> Self {
        Self::new(2)
    }
}

/// A guard representing one active submission on the GPU. When dropped, decrements the active
/// submission count.
pub(crate) struct OpGuard(Arc<dyn GpuManagerPriv>);

impl Drop for OpGuard {
    fn drop(&mut self) {
        self.0.end_op();
    }
}

/// Set of global sequence IDs used in the driver.
#[derive(Default)]
pub(crate) struct SequenceIDs {
    /// `File` instance ID.
    pub(crate) file: ID,
    /// `Vm` instance ID.
    pub(crate) vm: ID,
    /// Submission instance ID.
    pub(crate) submission: ID,
    /// `Queue` instance ID.
    pub(crate) queue: ID,
}

/// Top-level GPU manager that owns all the global state relevant to the driver instance.
#[versions(AGX)]
#[pin_data]
pub(crate) struct GpuManager {
    dev: AsahiDevRef,
    cfg: &'static hw::HwConfig,
    dyncfg: hw::DynConfig,
    pub(crate) initdata: fw::types::GpuObject<fw::initdata::InitData::ver>,
    uat: mmu::Uat,
    crashed: AtomicBool,
    #[pin]
    alloc: Mutex<KernelAllocators>,
    io_mappings: KVec<mmu::KernelMapping>,
    next_mmio_iova: u64,
    #[pin]
    rtkit: Mutex<Option<rtkit::RtKit<GpuManager::ver>>>,
    #[ver(V >= V14_8_3)]
    /// Diagnostics: RTKit messages received on the non-system endpoints.
    rtk_rx_msgs: AtomicU64,
    #[ver(V >= V14_8_3)]
    /// Diagnostics: firmware-provided (mapped) RTKit buffers mapped / refused.
    rtk_shmem_mapped: AtomicU64,
    #[ver(V >= V14_8_3)]
    rtk_shmem_map_failed: AtomicU64,
    #[ver(V >= V14_8_3)]
    /// G15 diagnostics: ktime (ns) at which MSG_INIT was sent, 0 before.
    g15_init_ns: AtomicI64,
    #[ver(V >= V14_8_3)]
    /// G15 diagnostics: last P2+0x14 value seen by the init poll.
    g15_init_state: AtomicU32,
    #[pin]
    rx_channels: Mutex<RxChannels::ver>,
    #[pin]
    tx_channels: Mutex<TxChannels::ver>,
    #[pin]
    fwctl_channel: Mutex<channel::FwCtlChannel>,
    pipes: PipeChannels::ver,
    event_manager: Arc<event::EventManager>,
    buffer_mgr: buffer::BufferManager::ver,
    ids: SequenceIDs,
    #[allow(clippy::vec_box)]
    #[pin]
    garbage_contexts: Mutex<KVec<KBox<fw::types::GpuObject<fw::workqueue::GpuContextData>>>>,
}

/// Operations the G17P runtime (`g17::Gpu`) provides to the DRM driver.
///
/// `drm_gpu` adapts it to [`crate::drm_gpu::DrmGpu`]; the G13/G14 and M3 runtimes use that
/// trait directly.
pub(crate) trait Gpu: Send + Sync {
    // `g17::Gpu` implements these three; the DRM layer selects that runtime through
    // `drm_gpu::Backend::G17` instead of downcasting, so nothing calls them.
    /// Cast as an Any type.
    #[allow(dead_code)]
    fn as_any(&self) -> &dyn Any;
    /// Cast Arc<Self> as an Any type.
    #[allow(dead_code)]
    fn arc_as_any(self: Arc<Self>) -> Arc<dyn Any + Sync + Send>;
    /// Return the G13/G14 firmware manager, if this GPU uses that firmware interface.
    #[allow(dead_code)]
    fn manager(&self) -> Option<&dyn GpuManager>;
    /// Initialize the GPU.
    fn init(&self) -> Result;
    /// Return a reference to the global `SequenceIDs` instance.
    fn ids(&self) -> &SequenceIDs;
    /// Check whether the GPU is crashed
    fn is_crashed(&self) -> bool;
    /// Fill in the hardware description reported to userspace.
    fn get_params(&self, params: &mut kernel::uapi::drm_asahi_params_global) -> Result;
    /// Whether user VMs retain a sticky error and support a userspace mirror.
    fn supports_vm_status(&self) -> bool {
        false
    }
    /// Limits for independently executing queues, if supported by this GPU.
    fn queue_limits(&self) -> Option<kernel::uapi::drm_asahi_queue_limits> {
        None
    }
    /// Temporary capacity hint and validity in milliseconds for subsequent syncobj waits.
    fn syncobj_wait_hint(&self) -> Option<(u32, u32)> {
        None
    }
    // The five hooks below are read through the G15 `drm_gpu::DrmGpu` adapter when that
    // generation is configured; without it nothing calls them.
    /// The VA range that user VMs may use (`vm_start..vm_end` of GET_PARAMS).
    #[allow(dead_code)]
    fn user_range(&self) -> Range<u64> {
        self.uat_geometry().user_usable_range()
    }
    /// Whether this GPU uses the tracked submission model of `file.rs` (besides G17, which
    /// `file.rs` recognizes by its backend): VM_BIND batches are validated as a whole against
    /// driver-reserved ranges before any change, and SUBMIT copies the command buffer before
    /// resolving syncs because the queue installs a fence in every output sync even on failure.
    #[allow(dead_code)]
    fn tracked_submission(&self) -> bool {
        false
    }
    /// Whether SOFT_FAULTS may be advertised when `fault_control` selects soft faults.
    #[allow(dead_code)]
    fn soft_faults(&self) -> bool {
        true
    }
    /// Whether new user objects must be populated, cleaned to the point of coherency and pinned
    /// at creation, before any CPU view exists (GPUs whose device is not DMA-coherent and whose
    /// default CPU mappings are write-combined).
    #[allow(dead_code)]
    fn prepare_new_objects(&self) -> bool {
        false
    }
    /// The largest VM_BIND batch, if the GPU's UAPI contract bounds it; with a bound, a batch also
    /// needs at least one operation, a non-null pointer and a stride that is a multiple of 8 and at
    /// most a page (EINVAL otherwise). `None` keeps the shared rules.
    #[allow(dead_code)]
    fn bind_batch_limit(&self) -> Option<u32> {
        None
    }
    /// Frequency of the CPU timer that GPU timestamps are derived from.
    fn base_clock_hz(&self) -> u32;
    /// Return the GPU address space geometry.
    fn uat_geometry(&self) -> mmu::UatGeometry;
    /// Create a new `Vm` with the given range reserved for kernel-managed objects.
    fn new_vm(&self, kernel_range: Range<u64>) -> Result<mmu::Vm>;
    /// Create a new user command queue.
    fn new_queue(
        &self,
        vm: mmu::Vm,
        ualloc: Arc<Mutex<alloc::DefaultAllocator>>,
        ualloc_priv: Arc<Mutex<alloc::DefaultAllocator>>,
        priority: u32,
        usc_exec_base: u64,
    ) -> Result<KBox<dyn queue::Queue>>;
    /// Map a BO as a timestamp buffer
    fn map_timestamp_buffer(
        &self,
        bo: gem::ObjectRef,
        range: Range<usize>,
    ) -> Result<mmu::KernelMapping>;
}

/// Trait used to abstract the firmware/GPU-dependent variants of the GpuManager.
pub(crate) trait GpuManager: Send + Sync {
    /// Cast as an Any type.
    fn as_any(&self) -> &dyn Any;
    /// Cast Arc<Self> as an Any type.
    fn arc_as_any(self: Arc<Self>) -> Arc<dyn Any + Sync + Send>;
    /// Initialize the GPU.
    fn init(&self) -> Result;
    /// Update the GPU globals from global info
    ///
    /// TODO: Unclear what can and cannot be updated like this.
    fn update_globals(&self);
    /// Get a reference to the KernelAllocators.
    fn alloc(&self) -> Guard<'_, KernelAllocators, MutexBackend>;
    /// Create a new `Vm` given a unique `File` ID.
    fn new_vm(&self, kernel_range: Range<u64>) -> Result<mmu::Vm>;
    /// Bind a `Vm` to an available slot and return the `VmBind`.
    fn bind_vm(&self, vm: &mmu::Vm) -> Result<mmu::VmBind>;
    /// Create a new user command queue.
    fn new_queue(
        &self,
        vm: mmu::Vm,
        ualloc: Arc<Mutex<alloc::DefaultAllocator>>,
        ualloc_priv: Arc<Mutex<alloc::DefaultAllocator>>,
        priority: u32,
        usc_exec_base: u64,
    ) -> Result<KBox<dyn queue::Queue>>;
    /// Return a reference to the global `SequenceIDs` instance.
    fn ids(&self) -> &SequenceIDs;
    /// Kick the firmware (wake it up if asleep).
    ///
    /// This should be useful to reduce latency on work submission, so we can ask the firmware to
    /// wake up while we do some preparatory work for the work submission.
    fn kick_firmware(&self) -> Result;
    /// Flush the entire firmware cache.
    ///
    /// TODO: Does this actually work?
    fn flush_fw_cache(&self) -> Result;
    /// Handle a GPU work timeout event.
    fn handle_timeout(&self, counter: u32, event_slot: i32, unk: u32);
    /// Handle a GPU fault event.
    fn handle_fault(&self);
    /// Handle a channel error event.
    fn handle_channel_error(
        &self,
        error_type: ChannelErrorType,
        pipe_type: u32,
        event_slot: u32,
        event_value: u32,
    );
    /// Acknowledge a Buffer grow op.
    fn ack_grow(&self, buffer_slot: u32, vm_slot: u32, counter: u32);
    /// Send a firmware control command (secure cache flush).
    fn fwctl(&self, msg: fw::channels::FwCtlMsg) -> Result;
    /// Log a read-only firmware health sample labelled `tag` (G15 bring-up diagnostics).
    fn health_report(&self, tag: &str);
    /// Get the static GPU configuration for this SoC.
    fn get_cfg(&self) -> &'static hw::HwConfig;
    /// Get the dynamic GPU configuration for this SoC.
    fn get_dyncfg(&self) -> &hw::DynConfig;
    /// Register an unused context as garbage
    fn free_context(&self, data: KBox<fw::types::GpuObject<fw::workqueue::GpuContextData>>);
    /// Check whether the GPU is crashed
    fn is_crashed(&self) -> bool;
    /// Map a BO as a timestamp buffer
    fn map_timestamp_buffer(
        &self,
        bo: gem::ObjectRef,
        range: Range<usize>,
    ) -> Result<mmu::KernelMapping>;
}

/// Private generic trait for functions that don't need to escape this module.
trait GpuManagerPriv {
    /// Decrement the pending submission counter.
    fn end_op(&self);
}

/// Returns true (and warns once) because no context-release / cache-flush command is known yet
/// for the 14.x firmware interface, so firmware allocations are never reclaimed.
/// TODO: use the 14.x DeviceControl 0x11 release-resource command once its payload is known.
#[allow(dead_code)]
fn g15_fw_release_unsupported(dev: &kernel::device::Device) -> bool {
    static WARNED: AtomicBool = AtomicBool::new(false);
    if !WARNED.swap(true, Ordering::Relaxed) {
        dev_warn!(
            dev,
            "G15: no firmware context-release/cache-flush command yet; firmware objects are leaked\n"
        );
    }
    true
}

pub(crate) struct RtkitObject {
    vmap: shmem::VMap<gem::AsahiObject, u8>,
    mapping: mmu::KernelMapping,
}

/// How a firmware-provided RTKit buffer is mapped for the CPU.
enum RtkitMapKind {
    /// `memremap(MEMREMAP_WB)` of a physically contiguous range (the return value).
    Memremap(NonNull<core::ffi::c_void>),
    /// `vm_map_ram()` of this many RAM pages (the return value).
    VmMapRam(NonNull<core::ffi::c_void>, u32),
}

/// A buffer the firmware placed at a VA of its own choosing (already mapped in its page tables
/// by the bootloader), mapped read-mostly for the CPU so that the RTKit core can read the
/// syslog / crashlog / ioreport contents. No reply is sent to the firmware for these.
pub(crate) struct RtkitMapped {
    /// The IOVA exactly as the RTKit core passed it (and expects back).
    iova: usize,
    /// CPU address of the first byte of the buffer.
    cpu: NonNull<u8>,
    size: usize,
    kind: RtkitMapKind,
}

impl Drop for RtkitMapped {
    fn drop(&mut self) {
        match self.kind {
            // SAFETY: `p` was returned by a successful memremap() and is unmapped only here.
            RtkitMapKind::Memremap(p) => unsafe { bindings::memunmap(p.as_ptr()) },
            // SAFETY: `p`/`n` come from a successful vm_map_ram() and are unmapped only here.
            RtkitMapKind::VmMapRam(p, n) => unsafe { bindings::vm_unmap_ram(p.as_ptr(), n) },
        }
    }
}

/// RTKit buffer: either allocated by the driver (G13/G14 and G15 requests with IOVA 0), or
/// provided by the firmware at a fixed VA (G15).
pub(crate) enum RtkitBuffer {
    Alloc(RtkitObject),
    Mapped(RtkitMapped),
}

impl rtkit::Buffer for RtkitBuffer {
    fn iova(&self) -> Result<usize> {
        match self {
            RtkitBuffer::Alloc(o) => Ok(o.mapping.iova() as usize),
            RtkitBuffer::Mapped(m) => Ok(m.iova),
        }
    }
    fn buf(&mut self) -> Result<IoSysMapRef<'_, u8>> {
        match self {
            RtkitBuffer::Alloc(o) => Ok(o.vmap.get()),
            // SAFETY: `cpu` is a normal-memory kernel mapping (memremap WB / vm_map_ram) of
            // `size` bytes that lives as long as `self`.
            RtkitBuffer::Mapped(m) => {
                Ok(unsafe { IoSysMapRef::from_vaddr(m.cpu.as_ptr(), m.size) })
            }
        }
    }
}

/// Largest firmware-provided RTKit buffer we are willing to map (the RTKit size fields allow
/// ~1 MiB; anything much larger is a bogus request).
const RTKIT_MAP_MAX_SIZE: usize = 16 << 20;

/// Map a firmware-provided RTKit buffer (G15): translate the firmware VA through the kernel
/// (TTBR1) UAT page tables, which on G15 are the bootloader-built firmware tree, check that every
/// page is mapped normal memory, and map the physical pages for the CPU. Never allocates or
/// changes GPU page tables; any bad input fails with an error.
fn g15_rtkit_map(
    dev: &AsahiDevice,
    uat: &mmu::Uat,
    cfg: &'static hw::HwConfig,
    iova: usize,
    size: usize,
) -> Result<RtkitMapped> {
    let raw = iova as u64;
    let ias = cfg.uat_ias as usize;
    if size == 0 || size > RTKIT_MAP_MAX_SIZE {
        dev_err!(
            dev.as_ref(),
            "G15 RTKit map: IOVA {:#x}: bad size {:#x}\n",
            raw,
            size
        );
        return Err(EINVAL);
    }
    // The RTKit core only passes the low bits of the firmware VA (44 bits for buffer requests,
    // 48 for oslog). Any bit at or above the IAS means an upper-half (TTBR1) firmware address;
    // restore the sign extension. Lower-half addresses cannot be translated here.
    if raw >> ias == 0 {
        // Observed on J516S: the crashlog endpoint asks with a PHYSICAL address inside the
        // firmware carveout (ADT gfx-data, e.g. 0x10001a58000 in 0x10001950000+0x114000), not a
        // firmware VA. Accept page-aligned physical ranges inside the t6030 DRAM window only (so an
        // MMIO address can never get a cacheable mapping) and memremap them WB for reading.
        const G15_DRAM_BASE: u64 = 0x100_0000_0000; // ADT /chosen dram-base on t6030
        const G15_DRAM_WINDOW: u64 = 0x10_0000_0000; // 64 GiB, above any M3 Pro configuration
        let pa_end = raw.checked_add(size as u64).ok_or(EINVAL)?;
        if raw & (PAGE_SIZE as u64 - 1) != 0
            || raw < G15_DRAM_BASE
            || pa_end > G15_DRAM_BASE + G15_DRAM_WINDOW
        {
            dev_err!(
                dev.as_ref(),
                "G15 RTKit map: IOVA {:#x} (size {:#x}) is neither a TTBR1 firmware address nor a DRAM physical address\n",
                raw,
                size
            );
            return Err(EINVAL);
        }
        // SAFETY: memremap() validates the range and returns NULL on failure. The range is inside
        // DRAM (checked above): linear-map RAM gives the linear address, the firmware carveout gets
        // a normal cacheable mapping.
        let p = unsafe { bindings::memremap(raw as _, size, bindings::MEMREMAP_WB as _) };
        let p = NonNull::new(p).ok_or_else(|| {
            dev_err!(
                dev.as_ref(),
                "G15 RTKit map: memremap(PA {:#x}, {:#x}) failed\n",
                raw,
                size
            );
            ENOMEM
        })?;
        dev_info!(
            dev.as_ref(),
            "G15 RTKit map: IOVA {:#x} size {:#x} treated as a DRAM physical address (firmware carveout), CPU mapping WB\n",
            raw,
            size
        );
        return Ok(RtkitMapped {
            iova,
            cpu: p.cast(),
            size,
            kind: RtkitMapKind::Memremap(p),
        });
    }
    let fw_va = raw | mmu::iova_ttbr1_base(cfg);
    let end = fw_va.checked_add(size as u64).ok_or(EINVAL)?;
    let pgmsk = mmu::UAT_PGMSK as u64;
    let start_pg = fw_va & !pgmsk;
    let end_pg = end.checked_add(pgmsk).ok_or(EINVAL)? & !pgmsk;
    let offset = (fw_va - start_pg) as usize;

    let pages = uat
        .translate_kernel_range(start_pg..end_pg)
        .inspect_err(|e| {
            dev_err!(
                dev.as_ref(),
                "G15 RTKit map: FW VA {:#x}..{:#x} not fully mapped in the firmware page tables ({:?})\n",
                fw_va,
                end,
                e
            )
        })?;
    let first_pa = pages.first().ok_or(EINVAL)?.0;

    for (i, (pa, pte)) in pages.iter().enumerate() {
        if !mmu::pte_is_normal_memory(*pte) {
            dev_err!(
                dev.as_ref(),
                "G15 RTKit map: FW VA {:#x} -> PA {:#x} is not normal memory (PTE {:#x}), refusing\n",
                start_pg + (i * mmu::UAT_PGSZ) as u64,
                pa,
                pte
            );
            return Err(ENXIO);
        }
    }

    let contiguous = pages
        .iter()
        .enumerate()
        .all(|(i, (pa, _))| *pa == first_pa + (i * mmu::UAT_PGSZ) as PhysicalAddr);

    dev_info!(
        dev.as_ref(),
        "G15 RTKit map: IOVA {:#x} -> FW VA {:#x} size {:#x} ({} UAT pages, {}), PA {:#x}{}, PTE {:#x}; endpoint not reported by the RTKit core\n",
        raw,
        fw_va,
        size,
        pages.len(),
        if contiguous { "contiguous" } else { "scattered" },
        first_pa + offset as PhysicalAddr,
        if contiguous { "" } else { " (first page)" },
        pages[0].1
    );
    if !contiguous {
        for (i, (pa, _)) in pages.iter().enumerate().take(16) {
            dev_info!(dev.as_ref(), "G15 RTKit map:   page {}: PA {:#x}\n", i, pa);
        }
    }

    if contiguous {
        let pa = first_pa + offset as PhysicalAddr;
        // Only map physical memory the kernel knows about (System RAM or a memblock-described
        // carveout, including no-map reserved memory). A PTE that claims "normal memory" for an
        // MMIO/SRAM address would otherwise get a cacheable mapping of device memory (SError).
        let pa_end = pa + size as PhysicalAddr;
        let mut p_chk = pa & !((PAGE_SIZE - 1) as PhysicalAddr);
        while p_chk < pa_end {
            // SAFETY: pfn_valid() only inspects the memory model's section tables.
            if !unsafe { bindings::pfn_valid((p_chk >> bindings::PAGE_SHIFT) as _) } {
                dev_err!(
                    dev.as_ref(),
                    "G15 RTKit map: PA {:#x} (FW VA {:#x}) is not kernel-described memory, refusing\n",
                    p_chk,
                    fw_va
                );
                return Err(ENXIO);
            }
            p_chk += PAGE_SIZE as PhysicalAddr;
        }

        // Firmware-"uncached" (shared) buffers: prefer a write-combining (non-cacheable) CPU
        // mapping so reads are not served from stale cache lines. arm64 refuses WC for pages in
        // the linear map; fall back to WB then (and say so).
        let uncached = pages.iter().any(|(_, pte)| mmu::pte_is_uncached(*pte));
        let mut p: *mut core::ffi::c_void = core::ptr::null_mut();
        let mut how = "WB";
        if uncached {
            // SAFETY: memremap() validates the range itself and returns NULL on failure.
            p = unsafe { bindings::memremap(pa as _, size, bindings::MEMREMAP_WC as _) };
            how = "WC";
            if p.is_null() {
                dev_warn!(
                    dev.as_ref(),
                    "G15 RTKit map: PA {:#x} is firmware-uncached but memremap(WC) failed; using WB (reads may be stale)\n",
                    pa
                );
            }
        }
        if p.is_null() {
            // SAFETY: memremap() validates the range itself and returns NULL on failure. For RAM
            // in the linear map it returns the linear address; otherwise (the firmware carveout)
            // it creates a normal cacheable mapping, so plain memcpy() reads are fine.
            p = unsafe { bindings::memremap(pa as _, size, bindings::MEMREMAP_WB as _) };
            how = "WB";
        }
        let p = NonNull::new(p).ok_or_else(|| {
            dev_err!(
                dev.as_ref(),
                "G15 RTKit map: memremap({:#x}, {:#x}) failed\n",
                pa,
                size
            );
            ENOMEM
        })?;
        dev_info!(
            dev.as_ref(),
            "G15 RTKit map: FW VA {:#x}: memattr {}, CPU mapping {}\n",
            fw_va,
            if uncached { "uncached" } else { "cached" },
            how
        );
        return Ok(RtkitMapped {
            iova,
            cpu: p.cast(),
            size,
            kind: RtkitMapKind::Memremap(p),
        });
    }

    // Scattered pages: only RAM with struct pages can be vmapped.
    if PAGE_SIZE > mmu::UAT_PGSZ || mmu::UAT_PGSZ % PAGE_SIZE != 0 {
        return Err(ENXIO);
    }
    let per = mmu::UAT_PGSZ / PAGE_SIZE;
    let mut ptrs: KVec<*mut bindings::page> = KVec::with_capacity(pages.len() * per, GFP_KERNEL)?;
    for (pa, _) in pages.iter() {
        for k in 0..per {
            let p = *pa + (k * PAGE_SIZE) as PhysicalAddr;
            // SAFETY: Only used to get the struct page pointer; borrow_phys() checks that the
            // pfn is valid RAM. The pages belong to the firmware and stay allocated while it runs.
            match unsafe { Page::borrow_phys(&p) } {
                Some(page) => ptrs.push(page.as_ptr(), GFP_KERNEL)?,
                None => {
                    dev_err!(
                        dev.as_ref(),
                        "G15 RTKit map: scattered page at PA {:#x} is not RAM, cannot vmap\n",
                        p
                    );
                    return Err(ENXIO);
                }
            }
        }
    }
    let count: u32 = ptrs.len().try_into()?;
    // SAFETY: `ptrs` holds `count` valid struct page pointers.
    let v = unsafe { bindings::vm_map_ram(ptrs.as_mut_ptr(), count, bindings::NUMA_NO_NODE) };
    let v = NonNull::new(v).ok_or_else(|| {
        dev_err!(
            dev.as_ref(),
            "G15 RTKit map: vm_map_ram({} pages) failed\n",
            count
        );
        ENOMEM
    })?;
    let uncached = pages.iter().any(|(_, pte)| mmu::pte_is_uncached(*pte));
    dev_info!(
        dev.as_ref(),
        "G15 RTKit map: FW VA {:#x}: memattr {}, CPU mapping vm_map_ram (WB){}\n",
        fw_va,
        if uncached { "uncached" } else { "cached" },
        if uncached { ", reads may be stale" } else { "" }
    );
    // SAFETY: `offset` < UAT_PGSZ and the mapping covers `offset + size` bytes.
    let cpu = unsafe { NonNull::new_unchecked((v.as_ptr() as *mut u8).add(offset)) };
    Ok(RtkitMapped {
        iova,
        cpu,
        size,
        kind: RtkitMapKind::VmMapRam(v, count),
    })
}

#[versions(AGX)]
#[vtable]
impl rtkit::Operations for GpuManager::ver {
    type Data = Arc<GpuManager::ver>;
    type Buffer = RtkitBuffer;

    fn recv_message(data: <Self::Data as ForeignOwnable>::Borrowed<'_>, ep: u8, msg: u64) {
        let dev = &data.dev;
        //dev_info!(dev.as_ref(), "RtKit message: {:#x}:{:#x}\n", ep, msg);
        #[ver(V >= V14_8_3)]
        {
        let n = data.rtk_rx_msgs.fetch_add(1, Ordering::Relaxed);
        // G15 bring-up: log the first firmware messages (bounded).
        if data.cfg.gpu_gen == hw::GpuGen::G15 && n < 32 {
            dev_info!(
                dev.as_ref(),
                "G15 RTKit rx #{}: ep {:#x} msg {:#x}\n",
                n + 1,
                ep,
                msg
            );
        }

        }

        if ep != EP_FIRMWARE || msg != MSG_RX_DOORBELL {
            dev_err!(dev.as_ref(), "Unknown message: {:#x}:{:#x}\n", ep, msg);
            return;
        }

        let mut ch = data.rx_channels.lock();

        ch.fw_log.poll();
        ch.ktrace.poll();
        ch.stats.poll();
        ch.event.poll();
    }

    fn crashed(data: <Self::Data as ForeignOwnable>::Borrowed<'_>, crashlog: Option<&[u8]>) {
        let dev = &data.dev;

        data.crashed.store(true, Ordering::Relaxed);

        #[cfg(CONFIG_DEV_COREDUMP)]
        if let Err(e) = data.generate_crashdump(crashlog) {
            dev_err!(dev.as_ref(), "Could not generate crashdump: {:?}\n", e);
        }
        #[cfg(not(CONFIG_DEV_COREDUMP))]
        let _ = crashlog;

        if debug_enabled(DebugFlags::OopsOnGpuCrash) {
            panic!("GPU firmware crashed");
        } else {
            dev_err!(dev.as_ref(), "GPU firmware crashed, failing all jobs\n");
            data.event_manager.fail_all(workqueue::WorkError::NoDevice);
        }
    }

    fn shmem_alloc(
        data: <Self::Data as ForeignOwnable>::Borrowed<'_>,
        size: usize,
    ) -> Result<Self::Buffer> {
        let dev = &data.dev;
        mod_dev_dbg!(dev, "shmem_alloc() {:#x} bytes\n", size);

        let mut obj = gem::new_kernel_object(dev, size)?;
        let vmap = obj.gem.owned_vmap()?;
        let mapping = obj.map_into_range(
            data.uat.kernel_vm(),
            mmu::kern_iova_range(data.cfg, IOVA_KERN_RTKIT_RANGE),
            mmu::UAT_PGSZ as u64,
            mmu::PROT_FW_SHARED_RW,
            true,
        )?;
        mod_dev_dbg!(dev, "shmem_alloc() -> VA {:#x}\n", mapping.iova());
        Ok(RtkitBuffer::Alloc(RtkitObject { vmap, mapping }))
    }

    #[ver(V < V14_8_3)]
    // Map a buffer the firmware placed at its own VA. G15 only: on G13/G14 this keeps the
    // previous behaviour (the default implementation's EINVAL).
    fn shmem_map(_data: <Self::Data as ForeignOwnable>::Borrowed<'_>, _iova: usize, _size: usize) -> Result<Self::Buffer> {
        Err(EINVAL)
    }

    #[ver(V >= V14_8_3)]
    fn shmem_map(
        data: <Self::Data as ForeignOwnable>::Borrowed<'_>,
        iova: usize,
        size: usize,
    ) -> Result<Self::Buffer> {
        if data.cfg.gpu_gen != hw::GpuGen::G15 {
            return Err(EINVAL);
        }
        match g15_rtkit_map(&data.dev, &data.uat, data.cfg, iova, size) {
            Ok(m) => {
                data.rtk_shmem_mapped.fetch_add(1, Ordering::Relaxed);
                Ok(RtkitBuffer::Mapped(m))
            }
            Err(e) => {
                data.rtk_shmem_map_failed.fetch_add(1, Ordering::Relaxed);
                Err(e)
            }
        }
    }
}

#[versions(AGX)]
impl GpuManager::ver {
    /// Create a new GpuManager of this version/GPU combination.
    #[inline(never)]
    pub(crate) fn new(
        dev: &AsahiDevice,
        res: &regs::Resources,
        cfg: &'static hw::HwConfig,
    ) -> Result<Arc<GpuManager::ver>> {
        let uat = Self::make_uat(dev, cfg)?;
        let dyncfg = Self::make_dyncfg(dev, res, cfg, &uat)?;

        let mut alloc = KernelAllocators {
            private: alloc::DefaultAllocator::new(
                dev,
                uat.kernel_vm(),
                mmu::kern_iova_range(cfg, IOVA_KERN_PRIV_RANGE),
                0x80,
                mmu::PROT_FW_PRIV_RW,
                1024 * 1024,
                true,
                fmt!("Kernel Private"),
                true,
            )?,
            shared: alloc::DefaultAllocator::new(
                dev,
                uat.kernel_vm(),
                mmu::kern_iova_range(cfg, IOVA_KERN_SHARED_RANGE),
                0x80,
                mmu::PROT_FW_SHARED_RW,
                1024 * 1024,
                true,
                fmt!("Kernel Shared"),
                false,
            )?,
            shared_ro: alloc::DefaultAllocator::new(
                dev,
                uat.kernel_vm(),
                mmu::kern_iova_range(cfg, IOVA_KERN_SHARED_RO_RANGE),
                0x80,
                mmu::PROT_FW_SHARED_RO,
                64 * 1024,
                true,
                fmt!("Kernel RO Shared"),
                false,
            )?,
            gpu: alloc::DefaultAllocator::new(
                dev,
                uat.kernel_vm(),
                mmu::kern_iova_range(cfg, IOVA_KERN_GPU_RANGE),
                0x80,
                mmu::PROT_GPU_FW_SHARED_RW,
                64 * 1024,
                true,
                fmt!("Kernel GPU Shared"),
                false,
            )?,
            gpu_ro: alloc::DefaultAllocator::new(
                dev,
                uat.kernel_vm(),
                mmu::kern_iova_range(cfg, IOVA_KERN_GPU_RO_RANGE),
                0x80,
                mmu::PROT_GPU_RO_FW_PRIV_RW,
                1024 * 1024,
                true,
                fmt!("Kernel GPU RO Shared"),
                true,
            )?,
        };

        let event_manager = Self::make_event_manager(&mut alloc)?;
        let mut initdata = Self::make_initdata(dev, cfg, &dyncfg, &mut alloc)?;

        initdata.runtime_pointers.buffer_mgr_ctl_low_mapping =
            Some(initdata.runtime_pointers.buffer_mgr_ctl.map_at(
                uat.kernel_lower_vm(),
                IOVA_KERN_GPU_BUFMGR_LOW,
                mmu::PROT_GPU_SHARED_RW,
                false,
            )?);
        initdata.runtime_pointers.buffer_mgr_ctl_high_mapping =
            Some(initdata.runtime_pointers.buffer_mgr_ctl.map_at(
                uat.kernel_vm(),
                mmu::kern_iova(cfg, IOVA_KERN_GPU_BUFMGR_HIGH),
                mmu::PROT_FW_SHARED_RW,
                false,
            )?);

        let mut mgr = Self::make_mgr(dev, cfg, dyncfg, uat, alloc, event_manager, initdata)?;

        {
            let fwctl = mgr.fwctl_channel.lock();
            let p_fwctl = fwctl.to_raw();
            core::mem::drop(fwctl);

            mgr.as_mut()
                .initdata_mut()
                .fw_status
                .with_mut(|raw, _inner| {
                    raw.fwctl_channel = p_fwctl;
                });
        }

        {
            let txc = mgr.tx_channels.lock();
            let p_device_control = txc.device_control.to_raw();
            core::mem::drop(txc);

            let rxc = mgr.rx_channels.lock();
            let p_event = rxc.event.to_raw();
            let p_fw_log = rxc.fw_log.to_raw();
            let p_ktrace = rxc.ktrace.to_raw();
            let p_stats = rxc.stats.to_raw();
            let p_fwlog_buf = rxc.fw_log.get_buf();
            core::mem::drop(rxc);

            mgr.as_mut()
                .initdata_mut()
                .runtime_pointers
                .with_mut(|raw, _inner| {
                    raw.device_control = p_device_control;
                    raw.event = p_event;
                    raw.fw_log = p_fw_log;
                    raw.ktrace = p_ktrace;
                    raw.stats = p_stats;
                    raw.fwlog_buf = Some(p_fwlog_buf);
                });
        }

        let mut p_pipes: KVec<fw::initdata::raw::PipeChannels::ver> = KVec::new();

        for ((v, f), c) in mgr
            .pipes
            .vtx
            .iter()
            .zip(&mgr.pipes.frag)
            .zip(&mgr.pipes.comp)
        {
            p_pipes.push(
                fw::initdata::raw::PipeChannels::ver {
                    vtx: v.lock().to_raw(),
                    frag: f.lock().to_raw(),
                    comp: c.lock().to_raw(),
                },
                GFP_KERNEL,
            )?;
        }

        mgr.as_mut()
            .initdata_mut()
            .runtime_pointers
            .with_mut(|raw, _inner| {
                for (i, p) in p_pipes.into_iter().enumerate() {
                    raw.pipes[i].vtx = p.vtx;
                    raw.pipes[i].frag = p.frag;
                    raw.pipes[i].comp = p.comp;
                }
            });

        for (i, map) in cfg.io_mappings.iter().enumerate() {
            if let Some(map) = map.as_ref() {
                Self::iomap(&mut mgr, cfg, i, map)?;
            }
        }

        #[ver(V >= V13_0B4)]
        if let Some(base) = cfg.sram_base {
            let size = cfg.sram_size.unwrap();
            let iova = mgr.as_mut().alloc_mmio_iova(size);

            let mapping = mgr
                .uat
                .kernel_vm()
                .map_io(iova, base, size, mmu::PROT_FW_SHARED_RW)?;

            // G15 leaves HwDataB+0xa20 (sgx_sram_ptr) at 0. With 0 the firmware uses
            // io_mappings[0] + 0x74000 (Fender SRAM part B) for its 0xc000-byte
            // power-transition save/restore; with the SRAM base it would copy over part A,
            // where the devctl and pipe write indices live.
            #[ver(V < V14_8_3)]
            mgr.as_mut()
                .initdata_mut()
                .runtime_pointers
                .hwdata_b
                .with_mut(|raw, _| {
                    raw.sgx_sram_ptr = U64(mapping.iova());
                });

            // G15: the host->FW ring indices and the pipe rings live in this SRAM.
            #[ver(V >= V14_8_3)]
            Self::place_rings_in_sram(mgr.as_mut(), base, size, mapping.iova())?;

            mgr.as_mut().io_mappings_mut().push(mapping, GFP_KERNEL)?;
        }

        let mgr = Arc::from(mgr);

        let rtkit = rtkit::RtKit::<GpuManager::ver>::new(dev.as_ref(), None, 0, mgr.clone())?;

        *mgr.rtkit.lock() = Some(rtkit);

        {
            let mut rxc = mgr.rx_channels.lock();
            rxc.event.set_manager(mgr.clone());
            // G15 bring-up: log every FWLog message at info level (bounded).
            rxc.fw_log.set_log_all_info(cfg.gpu_gen == hw::GpuGen::G15);
        }

        Ok(mgr)
    }

    // Move the host->FW ring indices and the pipe rings into the Fender scratch SRAM and point
    // RuntimePointers at them. Runs before the firmware is booted.
    // TODO: SRAM backup/restore around GPU power-down.
    #[ver(V >= V14_8_3)]
    fn place_rings_in_sram(
        mut this: Pin<&mut Self>,
        base: usize,
        size: usize,
        iova: u64,
    ) -> Result {
        // SAFETY: `base`/`size` is the Fender scratch SRAM from the HwConfig, and the caller
        // keeps the FW mapping at `iova` in `io_mappings` for the life of the GpuManager.
        let mut sram = unsafe { channel::ScratchRam::new(base, size, iova)? };

        let p_device_control = {
            let mut txc = this.tx_channels.lock();
            txc.device_control.place_in_sram(&mut sram)?;
            txc.device_control.to_raw()
        };

        let mut p_pipes: KVec<fw::initdata::raw::PipeChannels::ver> = KVec::new();
        for ((v, f), c) in this
            .pipes
            .vtx
            .iter()
            .zip(&this.pipes.frag)
            .zip(&this.pipes.comp)
        {
            let (mut v, mut f, mut c) = (v.lock(), f.lock(), c.lock());
            v.place_in_sram(&mut sram)?;
            f.place_in_sram(&mut sram)?;
            c.place_in_sram(&mut sram)?;
            p_pipes.push(
                fw::initdata::raw::PipeChannels::ver {
                    vtx: v.to_raw(),
                    frag: f.to_raw(),
                    comp: c.to_raw(),
                },
                GFP_KERNEL,
            )?;
        }

        this.as_mut()
            .initdata_mut()
            .runtime_pointers
            .with_mut(|raw, _inner| {
                raw.device_control = p_device_control;
                for (i, p) in p_pipes.into_iter().enumerate() {
                    raw.pipes[i] = p;
                }
            });
        Ok(())
    }

    /// Return a mutable reference to the initdata member
    fn initdata_mut(
        self: Pin<&mut Self>,
    ) -> &mut fw::types::GpuObject<fw::initdata::InitData::ver> {
        // SAFETY: initdata does not require structural pinning.
        unsafe { &mut self.get_unchecked_mut().initdata }
    }

    /// Return a mutable reference to the io_mappings member
    fn io_mappings_mut(self: Pin<&mut Self>) -> &mut KVec<mmu::KernelMapping> {
        // SAFETY: io_mappings does not require structural pinning.
        unsafe { &mut self.get_unchecked_mut().io_mappings }
    }

    /// Allocate an MMIO iova range
    fn alloc_mmio_iova(self: Pin<&mut Self>, size: usize) -> u64 {
        let mmio_end = mmu::kern_iova(self.cfg, IOVA_KERN_MMIO_RANGE.end);
        // SAFETY: next_mmio_iova does not require structural pinning.
        let next_ref = unsafe { &mut self.get_unchecked_mut().next_mmio_iova };

        let addr = *next_ref;
        let next = addr + (size + mmu::UAT_PGSZ) as u64;

        assert!(next <= mmio_end);

        *next_ref = next;

        addr
    }

    /// Build the entire GPU InitData structure tree and return it as a boxed GpuObject.
    fn make_initdata(
        dev: &AsahiDevice,
        cfg: &'static hw::HwConfig,
        dyncfg: &hw::DynConfig,
        alloc: &mut KernelAllocators,
    ) -> Result<KBox<fw::types::GpuObject<fw::initdata::InitData::ver>>> {
        let mut builder = initdata::InitDataBuilder::ver::new(dev, alloc, cfg, dyncfg);
        builder.build()
    }

    /// Create a fresh boxed Uat instance.
    ///
    /// Force disable inlining to avoid blowing up the stack.
    #[inline(never)]
    fn make_uat(dev: &AsahiDevice, cfg: &'static hw::HwConfig) -> Result<KBox<mmu::Uat>> {
        // G14X has a new thing in the Scene structure that unfortunately requires
        // write access from user contexts. Hopefully it's not security-sensitive.
        #[ver(G >= G14X)]
        let map_kernel_to_user = true;
        #[ver(G < G14X)]
        let map_kernel_to_user = false;

        Ok(KBox::new(
            mmu::Uat::new_for_config(dev, cfg, map_kernel_to_user)?,
            GFP_KERNEL,
        )?)
    }

    /// Actually create the final GpuManager instance, as a UniqueArc.
    ///
    /// Force disable inlining to avoid blowing up the stack.
    #[inline(never)]
    fn make_mgr(
        dev: &AsahiDevice,
        cfg: &'static hw::HwConfig,
        dyncfg: KBox<hw::DynConfig>,
        uat: KBox<mmu::Uat>,
        mut alloc: KernelAllocators,
        event_manager: Arc<event::EventManager>,
        initdata: KBox<fw::types::GpuObject<fw::initdata::InitData::ver>>,
    ) -> Result<Pin<UniqueArc<GpuManager::ver>>> {
        let mut pipes = PipeChannels::ver {
            vtx: KVec::new(),
            frag: KVec::new(),
            comp: KVec::new(),
        };

        for _i in 0..=NUM_PIPES - 1 {
            pipes.vtx.push(
                KBox::pin_init(
                    new_mutex!(channel::PipeChannel::ver::new(dev, &mut alloc)?, "pipe_vtx",),
                    GFP_KERNEL,
                )?,
                GFP_KERNEL,
            )?;
            pipes.frag.push(
                KBox::pin_init(
                    new_mutex!(
                        channel::PipeChannel::ver::new(dev, &mut alloc)?,
                        "pipe_frag",
                    ),
                    GFP_KERNEL,
                )?,
                GFP_KERNEL,
            )?;
            pipes.comp.push(
                KBox::pin_init(
                    new_mutex!(
                        channel::PipeChannel::ver::new(dev, &mut alloc)?,
                        "pipe_comp",
                    ),
                    GFP_KERNEL,
                )?,
                GFP_KERNEL,
            )?;
        }

        #[ver(V < V14_8_3)]
        let fwctl_channel = channel::FwCtlChannel::new(dev, &mut alloc)?;
        // 14.x FWCtl entries are 0x18 bytes
        #[ver(V >= V14_8_3)]
        let fwctl_channel = channel::FwCtlChannel::new_padded(dev, &mut alloc)?;

        let buffer_mgr = buffer::BufferManager::ver::new()?;
        let event_manager_clone = event_manager.clone();
        let buffer_mgr_clone = buffer_mgr.clone();
        let alloc_ref = &mut alloc;
        let rx_channels = KBox::init(
            try_init!(RxChannels::ver {
                event: channel::EventChannel::ver::new(
                    dev,
                    alloc_ref,
                    event_manager_clone,
                    buffer_mgr_clone,
                )?,
                fw_log: channel::FwLogChannel::new(dev, alloc_ref)?,
                ktrace: channel::KTraceChannel::new(dev, alloc_ref)?,
                stats: channel::StatsChannel::ver::new(dev, alloc_ref)?,
            }),
            GFP_KERNEL,
        )?;

        let alloc_ref = &mut alloc;
        let tx_channels = KBox::init(
            try_init!(TxChannels::ver {
                device_control: channel::DeviceControlChannel::ver::new(dev, alloc_ref)?,
            }),
            GFP_KERNEL,
        )?;

        let x = UniqueArc::pin_init(
            try_pin_init!(GpuManager::ver {
                dev: dev.into(),
                cfg,
                dyncfg: KBox::<hw::DynConfig>::into_inner(dyncfg),
                initdata: KBox::<fw::types::GpuObject<fw::initdata::InitData::ver>>::into_inner(initdata),
                uat: KBox::<mmu::Uat>::into_inner(uat),
                io_mappings: KVec::new(),
                next_mmio_iova: mmu::kern_iova(cfg, IOVA_KERN_MMIO_RANGE.start),
                rtkit <- new_mutex!(None, "rtkit"),
                #[ver(V >= V14_8_3)]
                rtk_rx_msgs: AtomicU64::new(0),
                #[ver(V >= V14_8_3)]
                rtk_shmem_mapped: AtomicU64::new(0),
                #[ver(V >= V14_8_3)]
                rtk_shmem_map_failed: AtomicU64::new(0),
                #[ver(V >= V14_8_3)]
                g15_init_ns: AtomicI64::new(0),
                #[ver(V >= V14_8_3)]
                g15_init_state: AtomicU32::new(0),
                crashed: AtomicBool::new(false),
                event_manager,
                alloc <- new_mutex!(alloc, "alloc"),
                fwctl_channel <- new_mutex!(fwctl_channel, "fwctl_channel"),
                rx_channels <- new_mutex!(KBox::<RxChannels::ver>::into_inner(rx_channels), "rx_channels"),
                tx_channels <- new_mutex!(KBox::<TxChannels::ver>::into_inner(tx_channels), "tx_channels"),
                pipes,
                buffer_mgr,
                ids: Default::default(),
                garbage_contexts <- new_mutex!(KVec::new(), "garbage_contexts"),
            }),
            GFP_KERNEL,
        )?;

        Ok(x)
    }

    fn load_hwdata_blob(dev: &AsahiDevice, name: &CStr, size_name: &CStr) -> Result<KVVec<u8>> {
        let of_node = dev.as_ref().of_node().ok_or(EINVAL)?;
        let size: usize = dev
            .as_ref()
            .fwnode()
            .ok_or(ENOENT)?
            .property_read::<u32>(size_name)
            .or(0)
            .try_into()?;
        let res = of_node.reserved_mem_region_to_resource_byname(name)?;
        // SAFETY: No dma here, just loading init data.
        let mem = unsafe { Mem::try_new(res, (MemFlag::WB).into())? };
        if size > mem.size() {
            return Err(ENOENT);
        }
        // SAFETY: trusting the bootloader to fill it out correctly
        let blob_sl = unsafe { slice::from_raw_parts(mem.ptr(), size) };
        let mut blob = KVVec::new();
        blob.extend_from_slice(blob_sl, GFP_KERNEL)?;
        Ok(blob)
    }

    /// Fetch and validate the GPU dynamic configuration from the device tree and hardware.
    ///
    /// Force disable inlining to avoid blowing up the stack.
    #[inline(never)]
    fn make_dyncfg(
        dev: &AsahiDevice,
        res: &regs::Resources,
        cfg: &'static hw::HwConfig,
        uat: &mmu::Uat,
    ) -> Result<KBox<hw::DynConfig>> {
        let gpu_id = res.get_gpu_id()?;

        dev_info!(dev.as_ref(), "GPU Information:\n");
        dev_info!(
            dev.as_ref(),
            "  Type: {:?}{:?}\n",
            gpu_id.gpu_gen,
            gpu_id.gpu_variant
        );
        dev_info!(dev.as_ref(), "  Clusters: {}\n", gpu_id.num_clusters);
        dev_info!(
            dev.as_ref(),
            "  Cores: {} ({})\n",
            gpu_id.num_cores,
            gpu_id.num_cores * gpu_id.num_clusters
        );
        dev_info!(
            dev.as_ref(),
            "  Frags: {} ({})\n",
            gpu_id.num_frags,
            gpu_id.num_frags * gpu_id.num_clusters
        );
        dev_info!(
            dev.as_ref(),
            "  GPs: {} ({})\n",
            gpu_id.num_gps,
            gpu_id.num_gps * gpu_id.num_clusters
        );
        dev_info!(dev.as_ref(), "  Core masks: {:#x?}\n", gpu_id.core_masks);
        dev_info!(
            dev.as_ref(),
            "  Active cores: {}\n",
            gpu_id.total_active_cores
        );

        dev_info!(dev.as_ref(), "Getting configuration from device tree...\n");
        let pwr_cfg = hw::PwrConfig::load(dev.as_ref(), cfg)?;
        dev_info!(dev.as_ref(), "Dynamic configuration fetched\n");

        if gpu_id.gpu_gen != cfg.gpu_gen || gpu_id.gpu_variant != cfg.gpu_variant {
            dev_err!(
                dev.as_ref(),
                "GPU type mismatch (expected {:?}{:?}, found {:?}{:?})\n",
                cfg.gpu_gen,
                cfg.gpu_variant,
                gpu_id.gpu_gen,
                gpu_id.gpu_variant
            );
            return Err(EIO);
        }
        if gpu_id.num_clusters > cfg.max_num_clusters {
            dev_err!(
                dev.as_ref(),
                "Too many clusters ({} > {})\n",
                gpu_id.num_clusters,
                cfg.max_num_clusters
            );
            return Err(EIO);
        }
        if gpu_id.num_cores > cfg.max_num_cores {
            dev_err!(
                dev.as_ref(),
                "Too many cores ({} > {})\n",
                gpu_id.num_cores,
                cfg.max_num_cores
            );
            return Err(EIO);
        }
        if gpu_id.num_frags > cfg.max_num_frags {
            dev_err!(
                dev.as_ref(),
                "Too many frags ({} > {})\n",
                gpu_id.num_frags,
                cfg.max_num_frags
            );
            return Err(EIO);
        }
        if gpu_id.num_gps > cfg.max_num_gps {
            dev_err!(
                dev.as_ref(),
                "Too many GPs ({} > {})\n",
                gpu_id.num_gps,
                cfg.max_num_gps
            );
            return Err(EIO);
        }

        let fwnode = dev.as_ref().fwnode().ok_or(ENOENT)?;

        Ok(KBox::new(
            hw::DynConfig {
                pwr: pwr_cfg,
                uat_ttb_base: uat.ttb_base(),
                id: gpu_id,
                firmware_version: fwnode
                    .property_read_array_vec(c_str!("apple,firmware-version"), 3)?
                    .or(kernel::kvec![0; 3]?),

                hw_data_a: Self::load_hwdata_blob(
                    dev,
                    c_str!("hw-cal-a"),
                    c_str!("debug,hw-cal-a-size"),
                )
                .unwrap_or(KVVec::new()),
                hw_data_b: Self::load_hwdata_blob(
                    dev,
                    c_str!("hw-cal-b"),
                    c_str!("debug,hw-cal-b-size"),
                )
                .unwrap_or(KVVec::new()),
                hw_globals: Self::load_hwdata_blob(
                    dev,
                    c_str!("globals"),
                    c_str!("debug,globals-size"),
                )
                .unwrap_or(KVVec::new()),
            },
            GFP_KERNEL,
        )?)
    }

    /// Create the global GPU event manager, and return an `Arc<>` to it.
    fn make_event_manager(alloc: &mut KernelAllocators) -> Result<Arc<event::EventManager>> {
        Ok(Arc::new(event::EventManager::new(alloc)?, GFP_KERNEL)?)
    }

    /// Create a new MMIO mapping and add it to the mappings list in initdata at the specified
    /// index.
    fn iomap(
        this: &mut Pin<UniqueArc<GpuManager::ver>>,
        cfg: &'static hw::HwConfig,
        index: usize,
        map: &hw::IOMapping,
    ) -> Result {
        let dies = if map.per_die {
            cfg.num_dies as usize
        } else {
            1
        };

        let off = map.base & mmu::UAT_PGMSK;
        let base = map.base - off;
        let end = (map.base + map.size + mmu::UAT_PGMSK) & !mmu::UAT_PGMSK;
        let map_size = end - base;

        // Array mappings must be aligned
        assert!((off == 0 && map_size == map.size) || (map.count == 1 && !map.per_die));
        assert!(map.count > 0);

        let iova = this.as_mut().alloc_mmio_iova(map_size * map.count * dies);
        let mut cur_iova = iova;

        for die in 0..dies {
            for i in 0..map.count {
                let phys_off = die * 0x20_0000_0000 + i * map.stride;

                let mapping = this.uat.kernel_vm().map_io(
                    cur_iova,
                    base + phys_off,
                    map_size,
                    if map.writable {
                        mmu::PROT_FW_MMIO_RW
                    } else {
                        mmu::PROT_FW_MMIO_RO
                    },
                )?;

                this.as_mut().io_mappings_mut().push(mapping, GFP_KERNEL)?;
                cur_iova += map_size as u64;
            }
        }

        let in_range = this
            .as_mut()
            .initdata_mut()
            .runtime_pointers
            .hwdata_b
            .with_mut(|raw, _| {
                // The HwConfig table must fit the firmware's slot count (31 on G15).
                // Fail the probe instead of panicking if the two disagree.
                let Some(slot) = raw.io_mappings.get_mut(index) else {
                    return false;
                };
                *slot = fw::initdata::raw::IOMapping {
                    phys_addr: U64(map.base as u64),
                    virt_addr: U64(iova + off as u64),
                    total_size: (map.size * map.count * dies) as u32,
                    element_size: map.size as u32,
                    readwrite: U64(map.writable as u64),
                };
                true
            });

        if !in_range {
            dev_err!(
                this.dev.as_ref(),
                "IO mapping {} does not fit the firmware io_mappings table\n",
                index
            );
            return Err(EINVAL);
        }

        Ok(())
    }

    /// Mark work associated with currently in-progress event slots as failed, after a fault or
    /// timeout.
    fn mark_pending_events(&self, culprit_slot: Option<u32>, error: workqueue::WorkError) {
        dev_err!(self.dev.as_ref(), "  Pending events:\n");

        self.initdata.globals.with(|raw, _inner| {
            for (index, i) in raw.pending_stamps.iter().enumerate() {
                let info = i.info.load(Ordering::Relaxed);
                let wait_value = i.wait_value.load(Ordering::Relaxed);

                if info & 1 != 0 {
                    #[ver(V >= V13_5)]
                    let slot = (info >> 4) & 0x7f;
                    #[ver(V < V13_5)]
                    let slot = (info >> 3) & 0x7f;
                    #[ver(V >= V13_5)]
                    let flags = info & 0xf;
                    #[ver(V < V13_5)]
                    let flags = info & 0x7;
                    dev_err!(
                        self.dev.as_ref(),
                        "    [{}:{}] flags={} value={:#x}\n",
                        index,
                        slot,
                        flags,
                        wait_value
                    );
                    let error = if culprit_slot.is_some() && culprit_slot != Some(slot) {
                        workqueue::WorkError::Killed
                    } else {
                        error
                    };
                    self.event_manager.mark_error(slot, wait_value, error);
                    i.info.store(0, Ordering::Relaxed);
                    i.wait_value.store(0, Ordering::Relaxed);
                }
            }
        });
    }

    /// Fetch the GPU MMU fault information from the hardware registers.
    fn get_fault_info(&self) -> Option<regs::FaultInfo> {
        let res = (*self.dev)
            .resources
            .as_ref()
            .expect("legacy manager requires register resources");

        let info = res.get_fault_info(self.cfg);
        if info.is_some() {
            dev_err!(
                self.dev.as_ref(),
                "  Fault info: {:#x?}\n",
                info.as_ref().unwrap()
            );
        }
        info
    }

    /// Resume the GPU firmware after it halts (due to a timeout, fault, or request).
    fn recover(&self) {
        self.initdata.fw_status.with(|raw, _inner| {
            let halt_count = raw.flags.halt_count.load(Ordering::Relaxed);
            let mut halted = raw.flags.halted.load(Ordering::Relaxed);
            dev_err!(self.dev.as_ref(), "  Halt count: {}\n", halt_count);
            dev_err!(self.dev.as_ref(), "  Halted: {}\n", halted);

            if halted == 0 {
                let start = Instant::<Monotonic>::now();
                while start.elapsed() < HALT_ENTER_TIMEOUT {
                    halted = raw.flags.halted.load(Ordering::Relaxed);
                    if halted != 0 {
                        break;
                    }
                    mem::sync();
                }
                halted = raw.flags.halted.load(Ordering::Relaxed);
            }

            if debug_enabled(DebugFlags::NoGpuRecovery) {
                dev_crit!(
                    self.dev.as_ref(),
                    "  GPU recovery is disabled, wedging forever!\n"
                );
            } else if halted != 0 {
                dev_err!(self.dev.as_ref(), "  Attempting recovery...\n");
                raw.flags.halted.store(0, Ordering::SeqCst);
                raw.flags.resume.store(1, Ordering::SeqCst);
            } else {
                dev_err!(self.dev.as_ref(), "  Cannot recover.\n");
            }
        });
    }

    /// Return the packed GPU enabled core masks.
    // Only used for some versions
    #[allow(dead_code)]
    pub(crate) fn core_masks_packed(&self) -> &[u32] {
        self.dyncfg.id.core_masks_packed.as_slice()
    }

    /// Kick a submission pipe for a submitted job to tell the firmware to start processing it.
    pub(crate) fn run_job(&self, job: workqueue::JobSubmission::ver<'_>) -> Result {
        mod_dev_dbg!(self.dev, "GPU: run_job\n");

        let pipe_type = job.pipe_type();
        mod_dev_dbg!(self.dev, "GPU: run_job: pipe_type={:?}\n", pipe_type);

        let pipes = match pipe_type {
            PipeType::Vertex => &self.pipes.vtx,
            PipeType::Fragment => &self.pipes.frag,
            PipeType::Compute => &self.pipes.comp,
        };

        let index: usize = job.priority() as usize;
        let mut pipe = pipes.get(index).ok_or(EIO)?.lock();

        mod_dev_dbg!(self.dev, "GPU: run_job: run()\n");
        job.run(&mut pipe);
        mod_dev_dbg!(self.dev, "GPU: run_job: ring doorbell\n");

        let mut guard = self.rtkit.lock();
        let rtk = guard.as_mut().as_pin_mut().unwrap();
        rtk.send_message(
            EP_DOORBELL,
            MSG_TX_DOORBELL | pipe_type as u64 | ((index as u64) << 2),
        )?;
        mod_dev_dbg!(self.dev, "GPU: run_job: done\n");

        Ok(())
    }

    pub(crate) fn start_op(self: &Arc<GpuManager::ver>) -> Result<OpGuard> {
        if self.is_crashed() {
            return Err(ENODEV);
        }

        let val = self
            .initdata
            .globals
            .with(|raw, _inner| raw.pending_submissions.fetch_add(1, Ordering::Acquire));

        mod_dev_dbg!(self.dev, "OP start (pending: {})\n", val + 1);
        self.kick_firmware()?;
        Ok(OpGuard(self.clone()))
    }

    fn invalidate_context(
        &self,
        context: &fw::types::GpuObject<fw::workqueue::GpuContextData>,
    ) -> Result {
        mod_dev_dbg!(
            self.dev,
            "Invalidating GPU context @ {:?}\n",
            context.weak_pointer()
        );

        if self.is_crashed() {
            return Err(ENODEV);
        }

        // 14.x has no DestroyContext and DeviceControlChannel::send() drops it, so this cannot
        // invalidate anything or flush the firmware cache. Report success to the caller (the
        // context object only moves to the allocator garbage list), but do NOT collect garbage:
        // without a firmware cache flush, reusing that memory could be a firmware
        // use-after-free. Firmware objects therefore leak on G15 until the 14.x
        // release-resource command is known.
        // TODO: implement with the 14.x DeviceControl 0x11 release-resource command.
        #[ver(V >= V14_8_3)]
        if g15_fw_release_unsupported(self.dev.as_ref()) {
            return Ok(());
        }

        let mut guard = self.alloc.lock();
        let (garbage_count, _) = guard.private.garbage();
        let (garbage_count_gpuro, _) = guard.gpu_ro.garbage();

        let dc = context.with(
            |raw, _inner| fw::channels::DeviceControlMsg::ver::DestroyContext {
                unk_4: 0,
                ctx_23: raw.unk_23,
                #[ver(V < V13_3)]
                __pad0: Default::default(),
                unk_c: U32(0),
                unk_10: U32(0),
                ctx_0: raw.unk_0,
                ctx_1: raw.unk_1,
                ctx_4: raw.unk_4,
                #[ver(V < V13_3)]
                __pad1: Default::default(),
                #[ver(V < V13_3)]
                unk_18: 0,
                gpu_context: Some(context.weak_pointer()),
                __pad2: Default::default(),
            },
        );

        mod_dev_dbg!(self.dev, "Context invalidation command: {:?}\n", &dc);

        let mut txch = self.tx_channels.lock();

        let token = txch.device_control.send(&dc);

        {
            let mut guard = self.rtkit.lock();
            let rtk = guard.as_mut().as_pin_mut().unwrap();
            rtk.send_message(EP_DOORBELL, MSG_TX_DOORBELL | DOORBELL_DEVCTRL)?;
        }

        txch.device_control.wait_for(token)?;

        mod_dev_dbg!(
            self.dev,
            "GPU context invalidated: {:?}\n",
            context.weak_pointer()
        );

        // The invalidation does a cache flush, so it is okay to collect garbage
        guard.private.collect_garbage(garbage_count);
        guard.gpu_ro.collect_garbage(garbage_count_gpuro);

        Ok(())
    }

    #[ver(V >= V14_8_3)]
    // Microseconds since MSG_INIT was sent (0 if it was not sent yet).
    fn us_since_init(&self) -> i64 {
        let t = self.g15_init_ns.load(Ordering::Relaxed);
        if t == 0 {
            0
        } else {
            (ktime_ns() - t) / 1000
        }
    }

    #[ver(V >= V14_8_3)]
    // G15: read the P2 debug block init word (+0x14).
    fn g15_p2_init_word(&self) -> u32 {
        self.initdata
            .debug_block
            .with(|raw, _| raw.init_state.load(Ordering::Acquire))
    }

    #[ver(V >= V14_8_3)]
    // G15: poll P2+0x14 every ~1 ms for up to [`G15_INIT_POLL_TIMEOUT_MS`] after MSG_INIT.
    // Returns true once the firmware reports post-INIT init done (1). Never fails the probe.
    fn g15_wait_init_done(&self) -> bool {
        let dev = self.dev.as_ref();
        let start = Instant::<Monotonic>::now();
        let timeout = Delta::from_millis(G15_INIT_POLL_TIMEOUT_MS);
        let mut last = G15_INIT_SENTINEL;
        let mut t_zero: Option<i64> = None;
        let mut polls = 0u32;
        loop {
            let v = self.g15_p2_init_word();
            let us = self.us_since_init();
            polls += 1;
            if v != last {
                dev_info!(
                    dev,
                    "G15 init: P2+0x14 {:#x} -> {:#x} at {} us after MSG_INIT (poll {})\n",
                    last,
                    v,
                    us,
                    polls
                );
                last = v;
                self.g15_init_state.store(v, Ordering::Relaxed);
            }
            if v == 0 && t_zero.is_none() {
                t_zero = Some(us);
            }
            if v == 1 {
                match t_zero {
                    Some(z) => dev_info!(
                        dev,
                        "G15 init: DONE. INIT consumed (P2+0x14 = 0) by {} us, init done (1) at {} us after MSG_INIT\n",
                        z,
                        us
                    ),
                    None => dev_info!(
                        dev,
                        "G15 init: DONE at {} us after MSG_INIT (the intermediate 0 passed within one poll)\n",
                        us
                    ),
                }
                return true;
            }
            if self.is_crashed() {
                dev_err!(
                    dev,
                    "G15 init: firmware crashed while waiting for init (P2+0x14 = {:#x}, {} us)\n",
                    v,
                    us
                );
                return false;
            }
            if start.elapsed() >= timeout {
                let what = match v {
                    G15_INIT_SENTINEL => {
                        "sentinel still present: INIT was not consumed (check the rtkit tx trace for EP 0x20), or CPU-side caching of the shared block"
                    }
                    0 => "INIT consumed but post-INIT init never finished (enable the gfx-data peek to dump the firmware threads)",
                    _ => "unexpected value",
                };
                dev_warn!(
                    dev,
                    "G15 init: TIMEOUT after {} ms: P2+0x14 = {:#x}: {}\n",
                    G15_INIT_POLL_TIMEOUT_MS,
                    v,
                    what
                );
                return false;
            }
            fsleep(Delta::from_micros(1000));
        }
    }

    #[ver(V >= V14_8_3)]
    // G15: ring the devctl doorbell once and log the result.
    fn g15_ring_devctl(&self, why: &str) -> Result {
        let mut guard = self.rtkit.lock();
        let rtk = guard.as_mut().as_pin_mut().ok_or(ENODEV)?;
        let ret = rtk.send_message(EP_DOORBELL, MSG_TX_DOORBELL | DOORBELL_DEVCTRL);
        core::mem::drop(guard);
        dev_info!(
            self.dev.as_ref(),
            "G15 devctl probe: doorbell {:#x} ({}) at {} us after MSG_INIT: {:?}\n",
            MSG_TX_DOORBELL | DOORBELL_DEVCTRL,
            why,
            self.us_since_init(),
            ret
        );
        ret
    }

    #[ver(V >= V14_8_3)]
    // G15 liveness probe: ring the devctl doorbell once (the ring holds the pre-INIT 0x13
    // entry) and watch the read index, read-only, at fixed points. Re-rings once if nothing
    // moved by [`G15_PROBE_RERING_MS`]. Never uses the unbounded wait path.
    fn g15_devctl_probe(&self) {
        let dev = self.dev.as_ref();
        let before = self.tx_channels.lock().device_control.indices();
        dev_info!(
            dev,
            "G15 devctl probe: before the doorbell: devctl {:?}\n",
            before
        );
        let rptr0 = before.map(|i| i.rptr).unwrap_or(0);
        let start = Instant::<Monotonic>::now();
        if self.g15_ring_devctl("probe").is_err() {
            return;
        }
        let mut rerung = false;
        for &cp in G15_PROBE_CHECKPOINTS_MS.iter() {
            let target = Delta::from_millis(cp);
            let el = start.elapsed();
            if el < target {
                fsleep(Delta::from_nanos(target.as_nanos() - el.as_nanos()));
            }
            let idx = self.tx_channels.lock().device_control.indices();
            let (ev, fl) = {
                let rxc = self.rx_channels.lock();
                (rxc.event.ring_snapshot(), rxc.fw_log.ring_snapshot())
            };
            dev_info!(
                dev,
                "G15 devctl probe +{}ms: devctl {:?} event {:?} fwlog {:?} rtk_rx={} crashed={} P2+0x14={:#x}\n",
                cp,
                idx,
                &ev[..],
                &fl[..],
                self.rtk_rx_msgs.load(Ordering::Relaxed),
                self.is_crashed(),
                self.g15_p2_init_word()
            );
            let moved = idx.map(|i| i.rptr != rptr0).unwrap_or(false);
            if !rerung && !moved && cp >= G15_PROBE_RERING_MS && !self.is_crashed() {
                rerung = true;
                let _ = self.g15_ring_devctl("re-ring, rptr unchanged");
            }
        }
        let after = self.tx_channels.lock().device_control.indices();
        let verdict = match after {
            Some(i) if i.rptr == i.wptr_host => {
                "rptr caught up with wptr: firmware consumed the devctl queue (doorbell handler and devctl path work)"
            }
            Some(i) if i.rptr != rptr0 => "rptr moved but did not catch up",
            Some(_) => {
                "rptr did not move: doorbell lost/ignored or devctl waiting on GPU power (try g15_debug bit 43, then bit 41)"
            }
            None => "no ring indices",
        };
        dev_info!(dev, "G15 devctl probe: result {:?}: {}\n", after, verdict);
    }

    #[ver(V >= V14_8_3)]
    // G15: log the identity of this boot's firmware objects (VAs, the INIT message, the InitData
    // bytes) and their UAT translations. Read-only.
    fn g15_identity_dump(&self) {
        let dev = self.dev.as_ref();
        let initdata = self.initdata.gpu_va().get();
        dev_info!(
            dev,
            "G15 identity: InitData VA {:#x}, INIT message {:#x}\n",
            initdata,
            MSG_INIT | (initdata & INIT_DATA_MASK)
        );
        self.initdata.with(|raw, _| {
            let p = raw as *const _ as *const u8;
            for row in 0..(0xc0 / 0x10) {
                let mut q = [0u64; 2];
                for (j, w) in q.iter_mut().enumerate() {
                    let mut b = [0u8; 8];
                    for (k, x) in b.iter_mut().enumerate() {
                        // SAFETY: InitData is 0xc0 bytes (static_assert in fw/initdata.rs).
                        *x = unsafe { core::ptr::read_volatile(p.add(row * 0x10 + j * 8 + k)) };
                    }
                    *w = u64::from_le_bytes(b);
                }
                dev_info!(
                    dev,
                    "G15 identity: InitData +0x{:02x}: {:016x} {:016x}\n",
                    row * 0x10,
                    q[0],
                    q[1]
                );
            }
        });

        let rp = &self.initdata.runtime_pointers;
        let desc = self.tx_channels.lock().device_control.to_raw();
        let sram = self.io_mappings.last().map(|m| m.iova()).unwrap_or(0);
        let objs: [(&str, u64); 16] = [
            ("InitData", initdata),
            ("RuntimePointers", rp.gpu_va().get()),
            ("Globals", self.initdata.globals.gpu_va().get()),
            ("HwDataA", rp.hwdata_a.gpu_va().get()),
            ("HwDataB", rp.hwdata_b.gpu_va().get()),
            ("P0 status", self.initdata.fw_status.gpu_va().get()),
            ("P1 power", self.initdata.power_ctl_block.gpu_va().get()),
            ("P2 debug", self.initdata.debug_block.gpu_va().get()),
            ("E5", rp.unkptr_e5.gpu_va().get()),
            ("E7", rp.unkptr_e7.gpu_va().get()),
            ("BRN table", rp.brn_table.gpu_va().get()),
            ("SRAM map", sram),
            ("devctl rptr", desc.read_index.0),
            ("devctl cfi", desc.cfi_index.0),
            ("devctl wptr", desc.write_index.0),
            ("devctl ring", desc.ring.0),
        ];
        for (name, va) in objs.iter() {
            match self.uat.translate_kernel_va(*va) {
                Ok((pa, pte)) => {
                    let (m, a, ap, hi) = mmu::pte_describe(pte);
                    dev_info!(
                        dev,
                        "G15 identity: {:16} VA {:#x} -> PA {:#x} PTE {:#x} ({}, AP {} ({}), high {:#x})\n",
                        name,
                        va,
                        pa,
                        pte,
                        m,
                        ap,
                        a,
                        hi
                    );
                }
                Err(e) => dev_warn!(
                    dev,
                    "G15 identity: {:16} VA {:#x} -> NOT TRANSLATED ({:?})\n",
                    name,
                    va,
                    e
                ),
            }
        }
    }

    #[ver(V >= V14_8_3)]
    // G15: the extra firmware words logged by each health sample. Read-only.
    fn g15_sample_words(&self, tag: &str) {
        let dev = self.dev.as_ref();
        let p2 = self.initdata.debug_block.with(|raw, _| {
            let p = raw as *const _ as *const u32;
            let mut w = [0u32; 8];
            for (i, x) in w.iter_mut().enumerate() {
                // SAFETY: DebugBlock is 0x20 bytes (static_assert), 4-byte aligned.
                *x = unsafe { core::ptr::read_volatile(p.add(i)) };
            }
            w
        });
        dev_info!(
            dev,
            "  [{}] P2 debug words (+0x00..+0x1c): {:#x?}\n",
            tag,
            p2
        );
        let (energy, power) = self.initdata.power_ctl_block.with(|raw, _| {
            // SAFETY: Plain reads of firmware-written fields through valid references.
            unsafe {
                (
                    core::ptr::read_volatile(core::ptr::addr_of!(raw.accumulated_energy)).0,
                    core::ptr::read_volatile(core::ptr::addr_of!(raw.filtered_power)),
                )
            }
        });
        dev_info!(
            dev,
            "  [{}] P1: accumulated_energy (+0x1d8) = {:#x}, filtered_power (+0x1e0) = {:#x}\n",
            tag,
            energy,
            power
        );
        let (rp200, rp230, e5) = self.initdata.runtime_pointers.with(|raw, inner| {
            // SAFETY: Byte-wise volatile reads of the RuntimePointers +0x200..+0x233 words and
            // of the 0x60-byte E5 buffer, both inside live allocations.
            unsafe {
                let p = core::ptr::addr_of!(raw.__pad_200) as *const u8;
                let mut a = [0u32; 12];
                for (i, x) in a.iter_mut().enumerate() {
                    let mut b = [0u8; 4];
                    for (k, y) in b.iter_mut().enumerate() {
                        *y = core::ptr::read_volatile(p.add(i * 4 + k));
                    }
                    *x = u32::from_le_bytes(b);
                }
                let u230 = core::ptr::read_volatile(core::ptr::addr_of!(raw.unk_230));
                let e = inner.unkptr_e5.as_slice();
                let mut ew = [0u32; 8];
                for (i, x) in ew.iter_mut().enumerate() {
                    let mut b = [0u8; 4];
                    for (k, y) in b.iter_mut().enumerate() {
                        *y = core::ptr::read_volatile(e.as_ptr().add(i * 4 + k));
                    }
                    *x = u32::from_le_bytes(b);
                }
                (a, u230, ew)
            }
        });
        dev_info!(
            dev,
            "  [{}] RP+0x200..+0x22f (FWLog seq/drop): {:x?} RP+0x230 (FWLog enable) = {}\n",
            tag,
            rp200,
            rp230
        );
        dev_info!(
            dev,
            "  [{}] E5 (RP+0x24c) +0x00..+0x1c (fw-private cached, may be stale): {:x?}\n",
            tag,
            e5
        );
    }

    #[cfg(CONFIG_DEV_COREDUMP)]
    fn generate_crashdump(&self, crashlog: Option<&[u8]>) -> Result {
        // Lock the allocators, to block kernel/FW memory mutations (mostly)
        let kalloc = self.alloc();
        let pages = self.uat.dump_kernel_pages()?;
        core::mem::drop(kalloc);

        let mut crashdump = crate::crashdump::CrashDumpBuilder::new(pages)?;
        let initdata_addr = self.initdata.gpu_va().get();
        crashdump.add_agx_info(self.cfg, &self.dyncfg, initdata_addr)?;
        if let Some(crashlog) = crashlog {
            crashdump.add_crashlog(crashlog)?;
        }
        let crashdump = KBox::new(crashdump.finalize()?, GFP_KERNEL)?;

        devcoredump::dev_coredump(
            self.dev.as_ref(),
            &crate::THIS_MODULE,
            crashdump,
            GFP_KERNEL,
            msecs_to_jiffies(60 * 60 * 1000),
        );

        Ok(())
    }
}

#[versions(AGX)]
impl GpuManager for GpuManager::ver {
    fn as_any(&self) -> &dyn Any {
        self
    }

    fn arc_as_any(self: Arc<Self>) -> Arc<dyn Any + Sync + Send> {
        self as Arc<dyn Any + Sync + Send>
    }

    #[ver(V < V14_8_3)]
    fn init(&self) -> Result {
        self.tx_channels.lock().device_control.send(
            &fw::channels::DeviceControlMsg::ver::Initialize(Default::default()),
        );

        let initdata = self.initdata.gpu_va().get();
        let mut guard = self.rtkit.lock();
        let mut rtk = guard.as_mut().as_pin_mut().unwrap();

        rtk.as_mut().boot()?;
        rtk.as_mut().start_endpoint(EP_FIRMWARE)?;
        rtk.as_mut().start_endpoint(EP_DOORBELL)?;
        rtk.as_mut()
            .send_message(EP_FIRMWARE, MSG_INIT | (initdata & INIT_DATA_MASK))?;
        rtk.as_mut()
            .send_message(EP_DOORBELL, MSG_TX_DOORBELL | DOORBELL_DEVCTRL)?;
        core::mem::drop(guard);

        self.kick_firmware()?;
        Ok(())
    }

    #[ver(V >= V14_8_3)]
    fn init(&self) -> Result {
        #[ver(V < V14_8_3)]
        self.tx_channels.lock().device_control.send(
            &fw::channels::DeviceControlMsg::ver::Initialize(Default::default()),
        );
        // G15: the 14.x interface has no Initialize. Before INIT, on every (re)init, queue one
        // DeviceControl 0x13 (ForceConfigSnapshot), which the firmware consumes during its own
        // initialisation. No doorbell: EP 0x21 is not started yet.
        #[ver(V >= V14_8_3)]
        {
            let wptr = self.tx_channels.lock().device_control.send(
                &fw::channels::DeviceControlMsg::ver::ForceConfigSnapshot(Default::default()),
            );
            dev_info!(
                self.dev.as_ref(),
                "G15: queued DeviceControl 0x13 ForceConfigSnapshot before INIT (devctl wptr {})\n",
                wptr
            );
            self.initdata
                .debug_block
                .with(|raw, _| raw.init_state.store(G15_INIT_SENTINEL, Ordering::SeqCst));
            self.g15_init_state
                .store(G15_INIT_SENTINEL, Ordering::Relaxed);
            mem::sync();
            self.g15_identity_dump();
        }

        let initdata = self.initdata.gpu_va().get();
        let init_msg = MSG_INIT | (initdata & INIT_DATA_MASK);
        let mut guard = self.rtkit.lock();
        let mut rtk = guard.as_mut().as_pin_mut().unwrap();

        rtk.as_mut().boot()?;
        rtk.as_mut().start_endpoint(EP_FIRMWARE)?;
        rtk.as_mut().start_endpoint(EP_DOORBELL)?;
        let ret = rtk.as_mut().send_message(EP_FIRMWARE, init_msg);
        let init_ns = ktime_ns();
        self.g15_init_ns.store(init_ns, Ordering::Relaxed);
        if self.cfg.gpu_gen == hw::GpuGen::G15 || ret.is_err() {
            dev_info!(
                self.dev.as_ref(),
                "INIT: EP {:#x} msg {:#x} (InitData VA {:#x}) sent at {} ns: {:?}\n",
                EP_FIRMWARE,
                init_msg,
                initdata,
                init_ns,
                ret
            );
        }
        ret?;

        #[ver(V < V14_8_3)]
        {
            rtk.as_mut()
                .send_message(EP_DOORBELL, MSG_TX_DOORBELL | DOORBELL_DEVCTRL)?;
            core::mem::drop(guard);

            self.kick_firmware()?;
        }

        // G15: no doorbell at boot. The firmware registers its EP 0x21 handler only at the end
        // of its post-INIT initialisation; a doorbell that arrives earlier is queued where
        // nothing drains it (observed on J516S: the firmware went quiet after MSG_INIT with the
        // old doorbell sequence). So wait for the firmware's own "init done" word, then ring the
        // devctl doorbell once as a liveness probe.
        #[ver(V >= V14_8_3)]
        {
            core::mem::drop(guard);

            let done = self.g15_wait_init_done();
            self.health_report("init-done");

            if !done {
                dev_warn!(
                    self.dev.as_ref(),
                    "G15: firmware init not confirmed (P2+0x14 = {:#x}); sending no doorbell\n",
                    self.g15_init_state.load(Ordering::Relaxed)
                );
            } else {
                self.g15_devctl_probe();
                self.health_report("probe+500ms");
            }
        }
        Ok(())
    }

    fn update_globals(&self) {
        let mut timeout: u32 = 2;
        if debug_enabled(DebugFlags::WaitForPowerOff) {
            timeout = 0;
        } else if debug_enabled(DebugFlags::KeepGpuPowered) {
            timeout = 5000;
        }
        // TODO: the rings live in Fender scratch SRAM and there is no SRAM backup/restore yet,
        // so keep the GPU from idling off, the same value that initdata.rs puts in Globals at
        // build time. WaitForPowerOff (timeout 0) would
        // ask for immediate idle-off, so it is ignored on G15.
        #[ver(V >= V14_8_3)]
        let timeout = {
            if timeout == 0 {
                static WARNED: AtomicBool = AtomicBool::new(false);
                if !WARNED.swap(true, Ordering::Relaxed) {
                    dev_warn!(
                        self.dev.as_ref(),
                        "G15: WaitForPowerOff ignored (no Fender SRAM backup/restore yet)\n"
                    );
                }
            }
            crate::initdata::G15_KEEP_POWERED_MS
        };

        self.initdata.globals.with(|raw, _inner| {
            raw.idle_off_delay_ms.store(timeout, Ordering::Relaxed);
        });
    }

    #[ver(V < V14_8_3)]
    fn health_report(&self, _tag: &str) {}

    #[ver(V >= V14_8_3)]
    fn health_report(&self, tag: &str) {
        let dev = self.dev.as_ref();
        dev_info!(
            dev,
            "G15 health report [{}] ({} us after MSG_INIT):\n",
            tag,
            self.us_since_init()
        );
        #[ver(V >= V14_8_3)]
        self.g15_sample_words(tag);

        // Handoff region: magic_fw == PPL magic means the firmware reached its MMU handoff init.
        dev_info!(dev, "  handoff: {:x?}\n", self.uat.handoff_snapshot());

        // FwStatus flags (G15: embedded in the P0 status block at +0x4580).
        self.initdata.fw_status.with(|raw, _inner| {
            let f = &raw.flags;
            // SAFETY: Plain reads of firmware-shared u32 words through valid references.
            let (unk_40, unk_ctr, unk_60, unk_70) = unsafe {
                (
                    core::ptr::read_volatile(&f.unk_40),
                    core::ptr::read_volatile(&f.unk_ctr),
                    core::ptr::read_volatile(&f.unk_60),
                    core::ptr::read_volatile(&f.unk_70),
                )
            };
            dev_info!(
                dev,
                "  FwStatus: halt_count={} halted={} resume={} unk_40={:#x} unk_ctr={:#x} unk_60={:#x} unk_70={:#x}\n",
                f.halt_count.load(Ordering::Relaxed),
                f.halted.load(Ordering::Relaxed),
                f.resume.load(Ordering::Relaxed),
                unk_40,
                unk_ctr,
                unk_60,
                unk_70
            );
            #[ver(V >= V14_8_3)]
            {
                // SAFETY: Plain reads of firmware-written words through valid references.
                let (ra, rb, u44c0, u4558, u4560) = unsafe {
                    (
                        core::ptr::read_volatile(&raw.fw_recovery_info_a),
                        core::ptr::read_volatile(&raw.fw_recovery_info_b),
                        core::ptr::read_volatile(&raw.unk_44c0).0,
                        core::ptr::read_volatile(&raw.unk_4558).0,
                        core::ptr::read_volatile(&raw.unk_4560).0,
                    )
                };
                dev_info!(
                    dev,
                    "  P0: recovery_info={:#x}/{:#x} +0x44c0={:#x} +0x4558={:#x} +0x4560={:#x}\n",
                    ra,
                    rb,
                    u44c0,
                    u4558,
                    u4560
                );
            }
        });

        // Firmware -> host traffic so far.
        {
            let rxc = self.rx_channels.lock();
            let crashed = self.crashed.load(Ordering::Relaxed);
            // The RTKit core treats a crash message as another buffer request while the crashlog
            // buffer is unmapped, so crashed=false means nothing if a buffer request was refused.
            let refused = self.rtk_shmem_map_failed.load(Ordering::Relaxed);
            dev_info!(
                dev,
                "  rx: RTKit msgs={} events={} fwlog={} crashed={}{}\n",
                self.rtk_rx_msgs.load(Ordering::Relaxed),
                rxc.event.received(),
                rxc.fw_log.received(),
                crashed,
                if !crashed && refused > 0 {
                    " (UNKNOWN: a firmware buffer request was refused, the crashlog buffer may be unmapped so crashes cannot be detected)"
                } else {
                    ""
                }
            );
        }
        // Firmware -> host rings (DRAM). A nonzero firmware wptr with no messages received means the
        // firmware wrote but the host was never notified.
        {
            let rxc = self.rx_channels.lock();
            dev_info!(
                dev,
                "  rx rings (fw wptr, host rptr): event {:?} fwlog {:?}\n",
                &rxc.event.ring_snapshot()[..],
                &rxc.fw_log.ring_snapshot()[..]
            );
        }
        dev_info!(
            dev,
            "  RTKit shmem: firmware-provided buffers mapped={} refused={}\n",
            self.rtk_shmem_mapped.load(Ordering::Relaxed),
            self.rtk_shmem_map_failed.load(Ordering::Relaxed)
        );

        // Host -> firmware ring indices (G15: in Fender SRAM; None before 14.x).
        {
            dev_info!(
                dev,
                "  devctl ring: {:?}\n",
                self.tx_channels.lock().device_control.indices()
            );
            for (i, ((v, f), c)) in self
                .pipes
                .vtx
                .iter()
                .zip(&self.pipes.frag)
                .zip(&self.pipes.comp)
                .enumerate()
            {
                dev_info!(
                    dev,
                    "  pipe[{}]: vtx {:?} frag {:?} comp {:?}\n",
                    i,
                    v.lock().indices(),
                    f.lock().indices(),
                    c.lock().indices()
                );
            }
        }

        // RTKit endpoint state. `crashed` is the flag set by the RTKit crash callback.
        let mut guard = self.rtkit.lock();
        match guard.as_mut().as_pin_mut() {
            Some(mut rtk) => {
                let mut eps: [(u8, bool); 7] = [
                    (1, false),
                    (2, false),
                    (3, false),
                    (4, false),
                    (8, false),
                    (EP_FIRMWARE, false),
                    (EP_DOORBELL, false),
                ];
                for e in eps.iter_mut() {
                    e.1 = rtk.as_mut().has_endpoint(e.0);
                }
                dev_info!(
                    dev,
                    "  RTKit: running={} crashed={} endpoints (crashlog 1, syslog 2, debug 3, ioreport 4, oslog 8, fw 0x20, doorbell 0x21) present: {:?}\n",
                    rtk.as_ref().is_running(),
                    self.is_crashed(),
                    eps
                );
            }
            None => dev_info!(dev, "  RTKit: not initialized\n"),
        }
    }

    fn alloc(&self) -> Guard<'_, KernelAllocators, MutexBackend> {
        /* Clean up idle contexts */
        let mut garbage_ctx = KVec::new();
        core::mem::swap(&mut *self.garbage_contexts.lock(), &mut garbage_ctx);

        for ctx in garbage_ctx {
            if self.invalidate_context(&ctx).is_err() {
                dev_err!(
                    self.dev.as_ref(),
                    "GpuContext: Failed to invalidate GPU context!\n"
                );
                if debug_enabled(DebugFlags::OopsOnGpuCrash) {
                    panic!("GPU firmware timed out");
                }
            }
        }

        let mut guard = self.alloc.lock();
        let (garbage_count, garbage_bytes) = guard.private.garbage();
        let (ro_garbage_count, ro_garbage_bytes) = guard.gpu_ro.garbage();

        if garbage_bytes > MAX_FW_ALLOC_GARBAGE_BYTES
            || ro_garbage_bytes > MAX_FW_ALLOC_GARBAGE_BYTES
            || garbage_count > MAX_FW_ALLOC_GARBAGE_OBJECTS
            || ro_garbage_count > MAX_FW_ALLOC_GARBAGE_OBJECTS
        {
            mod_dev_dbg!(
                self.dev,
                "Collecting kalloc garbage (private: {} objects, {} bytes, gpuro: {} objects, {} bytes)\n",
                garbage_count,
                garbage_bytes,
                ro_garbage_count,
                ro_garbage_bytes
            );
            // G15: flush_fw_cache() always fails (no 14.x flush command yet), so the garbage is
            // kept; the one-time warning from g15_fw_release_unsupported() is enough.
            let flushed = self.flush_fw_cache().is_ok();
            #[ver(V < V14_8_3)]
            if !flushed {
                dev_err!(self.dev.as_ref(), "Failed to flush FW cache\n");
            }
            if flushed {
                guard.private.collect_garbage(garbage_count);
                guard.gpu_ro.collect_garbage(ro_garbage_count);
            }
        }

        guard
    }

    fn new_vm(&self, kernel_range: Range<u64>) -> Result<mmu::Vm> {
        self.uat.new_vm(self.ids.vm.next(), kernel_range)
    }

    fn bind_vm(&self, vm: &mmu::Vm) -> Result<mmu::VmBind> {
        self.uat.bind(vm)
    }

    fn new_queue(
        &self,
        vm: mmu::Vm,
        ualloc: Arc<Mutex<alloc::DefaultAllocator>>,
        ualloc_priv: Arc<Mutex<alloc::DefaultAllocator>>,
        priority: u32,
        usc_exec_base: u64,
    ) -> Result<KBox<dyn queue::Queue>> {
        let mut kalloc = self.alloc();
        let id = self.ids.queue.next();
        Ok(KBox::new(
            queue::Queue::ver::new(
                &self.dev,
                vm,
                &mut kalloc,
                ualloc,
                ualloc_priv,
                self.event_manager.clone(),
                &self.buffer_mgr,
                id,
                priority,
                usc_exec_base,
            )?,
            GFP_KERNEL,
        )?)
    }

    fn kick_firmware(&self) -> Result {
        if self.is_crashed() {
            return Err(ENODEV);
        }

        let mut guard = self.rtkit.lock();
        let rtk = guard.as_mut().as_pin_mut().unwrap();
        rtk.send_message(EP_DOORBELL, MSG_TX_DOORBELL | DOORBELL_KICKFW)?;

        Ok(())
    }

    fn flush_fw_cache(&self) -> Result {
        mod_dev_dbg!(self.dev, "Flushing coprocessor data cache\n");

        if self.is_crashed() {
            return Err(ENODEV);
        }

        // No 14.x equivalent of the DestroyContext(0xff) cache flush yet: fail, so the caller
        // keeps the garbage instead of reusing memory the firmware may have cached.
        // TODO: replace with the 14.x release-resource / flush command.
        #[ver(V >= V14_8_3)]
        if g15_fw_release_unsupported(self.dev.as_ref()) {
            return Err(ENOTSUPP);
        }

        // ctx_0 == 0xff or ctx_1 == 0xff cause no effect on context,
        // but this command does a full cache flush too, so abuse it
        // for that.

        let dc = fw::channels::DeviceControlMsg::ver::DestroyContext {
            unk_4: 0,

            ctx_23: 0,
            #[ver(V < V13_3)]
            __pad0: Default::default(),
            unk_c: U32(0),
            unk_10: U32(0),
            ctx_0: 0xff,
            ctx_1: 0xff,
            ctx_4: 0,
            #[ver(V < V13_3)]
            __pad1: Default::default(),
            #[ver(V < V13_3)]
            unk_18: 0,
            gpu_context: None,
            __pad2: Default::default(),
        };

        let mut txch = self.tx_channels.lock();

        let token = txch.device_control.send(&dc);
        {
            let mut guard = self.rtkit.lock();
            let rtk = guard.as_mut().as_pin_mut().unwrap();
            rtk.send_message(EP_DOORBELL, MSG_TX_DOORBELL | DOORBELL_DEVCTRL)?;
        }

        txch.device_control.wait_for(token)?;
        Ok(())
    }

    fn ids(&self) -> &SequenceIDs {
        &self.ids
    }

    fn handle_timeout(&self, counter: u32, event_slot: i32, unk: u32) {
        dev_err!(self.dev.as_ref(), " (\\________/) \n");
        dev_err!(self.dev.as_ref(), "  |        |  \n");
        dev_err!(self.dev.as_ref(), "'.| \\  , / |.'\n");
        dev_err!(self.dev.as_ref(), "--| / (( \\ |--\n");
        dev_err!(self.dev.as_ref(), ".'|  _-_-  |'.\n");
        dev_err!(self.dev.as_ref(), "  |________|  \n");
        dev_err!(self.dev.as_ref(), "** GPU timeout nya~!!!!! **\n");
        dev_err!(self.dev.as_ref(), "  Event slot: {}\n", event_slot);
        dev_err!(self.dev.as_ref(), "  Timeout count: {}\n", counter);
        dev_err!(self.dev.as_ref(), "  Unk: {}\n", unk);

        // If we have fault info, consider it a fault.
        let error = match self.get_fault_info() {
            Some(info) => workqueue::WorkError::Fault(info),
            None => workqueue::WorkError::Timeout,
        };
        self.mark_pending_events(event_slot.try_into().ok(), error);
        self.recover();
    }

    fn handle_fault(&self) {
        dev_err!(self.dev.as_ref(), " (\\________/) \n");
        dev_err!(self.dev.as_ref(), "  |        |  \n");
        dev_err!(self.dev.as_ref(), "'.| \\  , / |.'\n");
        dev_err!(self.dev.as_ref(), "--| / (( \\ |--\n");
        dev_err!(self.dev.as_ref(), ".'|  _-_-  |'.\n");
        dev_err!(self.dev.as_ref(), "  |________|  \n");
        dev_err!(self.dev.as_ref(), "GPU fault nya~!!!!!\n");
        let error = match self.get_fault_info() {
            Some(info) => workqueue::WorkError::Fault(info),
            None => workqueue::WorkError::Unknown,
        };
        self.mark_pending_events(None, error);
        self.recover();
    }

    fn handle_channel_error(
        &self,
        error_type: ChannelErrorType,
        pipe_type: u32,
        event_slot: u32,
        event_value: u32,
    ) {
        dev_err!(self.dev.as_ref(), " (\\________/) \n");
        dev_err!(self.dev.as_ref(), "  |        |  \n");
        dev_err!(self.dev.as_ref(), "'.| \\  , / |.'\n");
        dev_err!(self.dev.as_ref(), "--| / (( \\ |--\n");
        dev_err!(self.dev.as_ref(), ".'|  _-_-  |'.\n");
        dev_err!(self.dev.as_ref(), "  |________|  \n");
        dev_err!(self.dev.as_ref(), "GPU channel error nya~!!!!!\n");
        dev_err!(self.dev.as_ref(), "  Error type: {:?}\n", error_type);
        dev_err!(self.dev.as_ref(), "  Pipe type: {}\n", pipe_type);
        dev_err!(self.dev.as_ref(), "  Event slot: {}\n", event_slot);
        dev_err!(self.dev.as_ref(), "  Event value: {:#x?}\n", event_value);

        self.event_manager.mark_error(
            event_slot,
            event_value,
            workqueue::WorkError::ChannelError(error_type),
        );

        let wq = match self.event_manager.get_owner(event_slot) {
            Some(wq) => wq,
            None => {
                dev_err!(
                    self.dev.as_ref(),
                    "Workqueue not found for this event slot!\n"
                );
                return;
            }
        };

        let wq = match wq.as_any().downcast_ref::<workqueue::WorkQueue::ver>() {
            Some(wq) => wq,
            None => {
                dev_crit!(self.dev.as_ref(), "GpuManager mismatched with WorkQueue!\n");
                return;
            }
        };

        if debug_enabled(DebugFlags::VerboseFaults) {
            wq.dump_info();
        }

        let dc = fw::channels::DeviceControlMsg::ver::RecoverChannel {
            pipe_type,
            work_queue: wq.info_pointer(),
            event_value,
            __pad: Default::default(),
        };

        mod_dev_dbg!(self.dev, "Recover Channel command: {:?}\n", &dc);
        let mut txch = self.tx_channels.lock();

        let token = txch.device_control.send(&dc);
        {
            let mut guard = self.rtkit.lock();
            let rtk = guard.as_mut().as_pin_mut().unwrap();
            if rtk
                .send_message(EP_DOORBELL, MSG_TX_DOORBELL | DOORBELL_DEVCTRL)
                .is_err()
            {
                dev_err!(
                    self.dev.as_ref(),
                    "Failed to send Recover Channel command\n"
                );
            }
        }

        if txch.device_control.wait_for(token).is_err() {
            dev_err!(
                self.dev.as_ref(),
                "Timed out waiting for Recover Channel command\n"
            );
        }

        if debug_enabled(DebugFlags::VerboseFaults) {
            wq.dump_info();
        }
    }

    fn ack_grow(&self, buffer_slot: u32, vm_slot: u32, counter: u32) {
        let halt_count = self
            .initdata
            .fw_status
            .with(|raw, _inner| raw.flags.halt_count.load(Ordering::Relaxed));

        let dc = fw::channels::DeviceControlMsg::ver::GrowTVBAck {
            unk_4: 1,
            buffer_slot,
            vm_slot,
            counter,
            subpipe: 0, // TODO
            halt_count: U64(halt_count),
            __pad: Default::default(),
        };

        mod_dev_dbg!(self.dev, "TVB Grow Ack command: {:?}\n", &dc);

        let mut txch = self.tx_channels.lock();

        txch.device_control.send(&dc);
        {
            let mut guard = self.rtkit.lock();
            let rtk = guard.as_mut().as_pin_mut().unwrap();
            if rtk
                .send_message(EP_DOORBELL, MSG_TX_DOORBELL | DOORBELL_DEVCTRL)
                .is_err()
            {
                dev_err!(self.dev.as_ref(), "Failed to send TVB Grow Ack command\n");
            }
        }
    }

    fn fwctl(&self, msg: fw::channels::FwCtlMsg) -> Result {
        if self.is_crashed() {
            return Err(ENODEV);
        }

        // The G15 firmware takes the size in 4 KiB units (size >> 12) at +0x08, where the G13
        // path sends 0. mmu.rs passes the 16 KiB page count in page_count.
        // TODO: +0x10 may be a shift-derived value rather than a count, and the +0x12 flags are
        // left at the G13 value 2; confirm both with a trace.
        #[ver(V >= V14_8_3)]
        let msg = fw::channels::FwCtlMsg {
            unk_8: (msg.page_count as u32) << (crate::pgtable::UAT_PGBIT - 12),
            ..msg
        };

        let mut fwctl = self.fwctl_channel.lock();
        let token = fwctl.send(&msg);
        {
            let mut guard = self.rtkit.lock();
            let rtk = guard.as_mut().as_pin_mut().unwrap();
            rtk.send_message(EP_DOORBELL, MSG_FWCTL)?;
        }
        fwctl.wait_for(token)?;
        Ok(())
    }

    fn get_cfg(&self) -> &'static hw::HwConfig {
        self.cfg
    }

    fn get_dyncfg(&self) -> &hw::DynConfig {
        &self.dyncfg
    }

    fn free_context(&self, ctx: KBox<fw::types::GpuObject<fw::workqueue::GpuContextData>>) {
        let mut garbage = self.garbage_contexts.lock();

        if garbage.push(ctx, GFP_KERNEL).is_err() {
            dev_err!(
                self.dev.as_ref(),
                "Failed to reserve space for freed context, deadlock possible.\n"
            );
        }
    }

    fn is_crashed(&self) -> bool {
        self.crashed.load(Ordering::Relaxed)
    }

    fn map_timestamp_buffer(
        &self,
        mut bo: gem::ObjectRef,
        range: Range<usize>,
    ) -> Result<mmu::KernelMapping> {
        bo.map_range_into_range(
            self.uat.kernel_vm(),
            range,
            mmu::kern_iova_range(self.cfg, IOVA_KERN_TIMESTAMP_RANGE),
            mmu::UAT_PGSZ as u64,
            mmu::PROT_FW_SHARED_RW,
            false,
        )
    }
}

#[versions(AGX)]
impl GpuManagerPriv for GpuManager::ver {
    fn end_op(&self) {
        let val = self
            .initdata
            .globals
            .with(|raw, _inner| raw.pending_submissions.fetch_sub(1, Ordering::Release));

        mod_dev_dbg!(self.dev, "OP end (pending: {})\n", val - 1);
    }
}
