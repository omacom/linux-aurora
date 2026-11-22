// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(clippy::unusual_byte_groupings)]

//! File implementation, which represents a single DRM client.
//!
//! This is in charge of managing the resources associated with one GPU client, including an
//! arbitrary number of submission queues and Vm objects, and reporting hardware/driver
//! information to userspace and accepting submissions.

use crate::debug::*;
use crate::{
    alloc,
    buffer,
    driver::{
        self,
        AsahiDrmDevice, //
    },
    gem,
    gpu,
    memacct,
    mmu,
    queue,
    queue_share,
    util::{
        align,
        align_down,
        gcd,
        AnyBitPattern,
        RangeExt,
        Reader, //
    }, //
};
use core::mem::MaybeUninit;
use core::ops::{Deref, DerefMut};
use core::ops::Range;
use core::ptr::addr_of_mut;
use core::sync::atomic::{AtomicBool, Ordering};
use kernel::bindings;
use kernel::dma_fence::RawDmaFence;
use kernel::drm::gem::BaseObject;
use kernel::drm::Registered;
use kernel::error::code::*;
use kernel::new_mutex;
use kernel::prelude::*;
use kernel::sync::{
    Arc,
    Mutex, //
};
use kernel::time::NSEC_PER_SEC;
use kernel::types::ScopeGuard;
use kernel::uaccess::{
    UserPtr,
    UserSlice, //
};
use kernel::{
    dma_fence,
    drm,
    io::Io,
    uapi,
    xarray, //
};

// The M5 runtime's UAPI numbers. The header's values are Mesa's; the M5 queue and
// GET_PARAMS paths use these names.

/// DRM_ASAHI_BIND_SPARSE_TILE: with SINGLE_PAGE, fill the range
/// from kernel sparse tiles (read-only: global zero tile; writable: per-VM
/// sink) instead of the GEM's first page. The GEM only anchors the GPUVA.
pub(crate) const DRM_ASAHI_BIND_SPARSE_TILE: u32 = uapi::drm_asahi_bind_flags_DRM_ASAHI_BIND_SPARSE_TILE;
/// DRM_ASAHI_FEATURE_SPARSE_TILES: BIND_SPARSE_TILE accepted.
pub(crate) const DRM_ASAHI_FEATURE_SPARSE_TILES: u64 =
    uapi::drm_asahi_feature_DRM_ASAHI_FEATURE_SPARSE_TILES as u64;
pub(crate) const DRM_ASAHI_FEATURE_COMPUTE_MATRIX: u64 =
    uapi::drm_asahi_feature_DRM_ASAHI_FEATURE_COMPUTE_MATRIX as u64;
/// DRM_ASAHI_FEATURE_TASK_MESH_STORAGE: render programs accept
/// DRM_ASAHI_RENDER_PROGRAM_TASK and such task (object-stage) kicks get the
/// TA mesh rendering buffer, which backs the object stage's payload address.
/// Reported only while m5_mesh_storage_chunks != 0.
pub(crate) const DRM_ASAHI_FEATURE_TASK_MESH_STORAGE: u64 =
    uapi::drm_asahi_feature_DRM_ASAHI_FEATURE_TASK_MESH_STORAGE as u64;
pub(crate) const DRM_ASAHI_FEATURE_RENDER_SHADER_WRITES: u64 =
    uapi::drm_asahi_feature_DRM_ASAHI_FEATURE_RENDER_SHADER_WRITES as u64;
/// D32 2x/4x ZLS admission; no compressed stencil or resolve bits.
pub(crate) const DRM_ASAHI_FEATURE_M5_DEPTH_MSAA_COMPRESSION: u64 =
    uapi::drm_asahi_feature_DRM_ASAHI_FEATURE_M5_DEPTH_MSAA_COMPRESSION as u64;
/// DRM_ASAHI_FEATURE_M5_STENCIL_MSAA_COMPRESSION: single-layer
/// 2x/4x ZLS admission in the compressed Z/S form, stencil included.
pub(crate) const DRM_ASAHI_FEATURE_M5_STENCIL_MSAA_COMPRESSION: u64 =
    uapi::drm_asahi_feature_DRM_ASAHI_FEATURE_M5_STENCIL_MSAA_COMPRESSION as u64;
#[allow(dead_code)]
pub(crate) const DRM_ASAHI_RENDER_SHADER_WRITES: u32 =
    uapi::drm_asahi_render_flags_DRM_ASAHI_RENDER_SHADER_WRITES;
/// Renders may carry separate tiling prefixes of both subqueues
/// (DRM_ASAHI_RENDER_STAGE_PREFIXES), as the complete struct drm_asahi_cmd_render_m5.
pub(crate) const DRM_ASAHI_FEATURE_RENDER_STAGE_PREFIXES_V2: u64 =
    uapi::DRM_ASAHI_FEATURE_RENDER_STAGE_PREFIXES_V2 as u64;
/// Compute commands may declare a stream that ends with a memory-scoped flush
/// (DRM_ASAHI_COMPUTE_VISIBLE_COMPLETION).
pub(crate) const DRM_ASAHI_FEATURE_COMPUTE_VISIBLE_COMPLETION: u64 =
    uapi::DRM_ASAHI_FEATURE_COMPUTE_VISIBLE_COMPLETION as u64;
/// The compute command's writes are visible when it completes, so another queue's compute
/// command may wait for it in firmware.
pub(crate) const DRM_ASAHI_COMPUTE_VISIBLE_COMPLETION: u32 =
    uapi::drm_asahi_compute_flags_DRM_ASAHI_COMPUTE_VISIBLE_COMPLETION as u32;
/// Queues take a watchdog allowance for their compute commands
/// (`drm_asahi_queue_create::compute_timeout_ms`).
pub(crate) const DRM_ASAHI_FEATURE_COMPUTE_TIMEOUT: u64 =
    uapi::DRM_ASAHI_FEATURE_COMPUTE_TIMEOUT as u64;
/// The tiling half waits only on the render and compute prefixes of the command's stage-prefix
/// fields, its fragment half on the whole ones.
#[allow(dead_code)]
pub(crate) const DRM_ASAHI_RENDER_STAGE_PREFIXES: u32 =
    uapi::drm_asahi_render_flags_DRM_ASAHI_RENDER_STAGE_PREFIXES;
/// Renders may declare that they write whole attachments only
/// (DRM_ASAHI_RENDER_WHOLE_ATTACHMENTS).
pub(crate) const DRM_ASAHI_FEATURE_WHOLE_ATTACHMENT_RENDERS: u64 =
    uapi::drm_asahi_feature_DRM_ASAHI_FEATURE_WHOLE_ATTACHMENT_RENDERS as u64;
/// The render area covers every attachment's full extent, so its tile stores
/// write no pixel outside it: an overlap with another such render without an
/// ordering dependency is the application's own race.
#[allow(dead_code)]
pub(crate) const DRM_ASAHI_RENDER_WHOLE_ATTACHMENTS: u32 =
    uapi::drm_asahi_render_flags_DRM_ASAHI_RENDER_WHOLE_ATTACHMENTS;
/// Render commands accept DRM_ASAHI_RENDER_MESH_OUTPUT.
pub(crate) const DRM_ASAHI_FEATURE_RENDER_MESH_OUTPUT: u64 =
    uapi::DRM_ASAHI_FEATURE_RENDER_MESH_OUTPUT as u64;
/// The render contains mesh shading: its tiler heap may be sized for mesh output ahead of it.
pub(crate) const DRM_ASAHI_RENDER_MESH_OUTPUT: u32 =
    uapi::drm_asahi_render_flags_DRM_ASAHI_RENDER_MESH_OUTPUT;
/// DRM_ASAHI_RENDER_PROGRAM_TASK, only with DRM_ASAHI_RENDER_PROGRAM_OBJECT: the object
/// program is an API task stage.
pub(crate) const DRM_ASAHI_RENDER_PROGRAM_TASK: u32 = uapi::DRM_ASAHI_RENDER_PROGRAM_TASK;
/// Bit 16, as Mesa defines it.
#[allow(dead_code)]
pub(crate) const DRM_ASAHI_COMPUTE_USES_MATRIX: u32 =
    uapi::drm_asahi_compute_flags_DRM_ASAHI_COMPUTE_USES_MATRIX as u32;
/// The compute command neither copies nor resets queries, so it cannot observe
/// timestamps the host publishes after earlier commands complete
/// (DRM_ASAHI_COMPUTE_NO_QUERY_ACCESS). Accepted, and reported with
/// DRM_ASAHI_FEATURE_COMPUTE_NO_QUERY_ACCESS by the G17 command sets.
pub(crate) const DRM_ASAHI_COMPUTE_NO_QUERY_ACCESS: u32 =
    uapi::drm_asahi_compute_flags_DRM_ASAHI_COMPUTE_NO_QUERY_ACCESS as u32;

const DEBUG_CLASS: DebugFlags = DebugFlags::File;

pub(crate) const MAX_COMMANDS_PER_SUBMISSION: u32 = 64;

/// A client instance of an `mmu::Vm` address space.
struct Vm {
    shrinker: Option<KBox<mmu::VmShrinker>>,
    ualloc: Arc<Mutex<alloc::DefaultAllocator>>,
    ualloc_priv: Arc<Mutex<alloc::DefaultAllocator>>,
    vm: mmu::Vm,
    user_range: Range<u64>,
    kernel_range: Range<u64>,
    _dummy_mapping: mmu::KernelMapping,
    gpu: Arc<dyn gpu::Gpu>,
}

impl Drop for Vm {
    fn drop(&mut self) {
        self.shrinker = None;
        self.vm.close_idle_reclaim();
        if let Some(status) = self.vm.status() {
            // Retained graphs may keep this status alive; the client's mirror
            // object must not outlive the client's VM handle.
            status.release_mirror();
        }
        if self.vm.status().is_some() {
            // Tracked jobs can outlive their file's VM handle. Their last guard performs
            // these unmaps once no accepted work can still dereference the mappings.
            if let Err(error) = self
                .vm
                .unmap_user_ranges(self.user_range.clone(), self.kernel_range.clone())
            {
                pr_err!("Vm::Drop: deferred user unmap failed: {:?}\n", error);
            }
            self.vm.bo_deferred_cleanup();
            self.gpu.vm_closed(&self.vm);
            return;
        }
        // When the user Vm is dropped, verify that no user mappings exists.
        let left_range = self.user_range.start..self.kernel_range.start;
        let right_range = self.kernel_range.end..self.user_range.end;

        if !left_range.is_empty()
            && self
                .vm
                .unmap_range(None, left_range.start, left_range.range())
                .is_err()
        {
            pr_err!("Vm::Drop: vm.unmap_range() failed\n");
        }
        if !right_range.is_empty()
            && self
                .vm
                .unmap_range(None, right_range.start, right_range.range())
                .is_err()
        {
            pr_err!("Vm::Drop: vm.unmap_range() failed\n");
        }

        self.vm.bo_deferred_cleanup();
    }
}

/// Sync object from userspace.
pub(crate) struct SyncItem {
    pub(crate) syncobj: drm::syncobj::SyncObj,
    pub(crate) fence: Option<dma_fence::Fence>,
    pub(crate) chain_fence: Option<dma_fence::FenceChain>,
    pub(crate) timeline_value: u64,
    pub(crate) cumulative: Option<crate::g17::sync_output::SignalBundle>,
    pub(crate) timeline: bool,
}

impl SyncItem {
    fn parse_one(
        file: &DrmFile,
        mut data: uapi::drm_asahi_sync,
        out: bool,
        bundled: bool,
    ) -> Result<SyncItem> {
        if data.sync_type == crate::g17::sync_output::SYNC_CUMULATIVE {
            if !out || !bundled || data.timeline_value == 0 {
                return Err(EINVAL);
            }
            data.sync_type = uapi::drm_asahi_sync_type_DRM_ASAHI_SYNC_TIMELINE_SYNCOBJ;
        }
        match data.sync_type {
            uapi::drm_asahi_sync_type_DRM_ASAHI_SYNC_SYNCOBJ => {
                if data.timeline_value != 0 {
                    cls_pr_debug!(Errors, "Non-timeline sync object with a nonzero value\n");
                    return Err(EINVAL);
                }
                let syncobj = drm::syncobj::SyncObj::lookup_handle(file, data.handle)?;

                Ok(SyncItem {
                    fence: if out {
                        None
                    } else {
                        Some(syncobj.fence_get().ok_or_else(|| {
                            cls_pr_debug!(Errors, "Failed to get fence from sync object\n");
                            EINVAL
                        })?)
                    },
                    syncobj,
                    chain_fence: None,
                    timeline_value: data.timeline_value,
                    cumulative: None,
                    timeline: false,
                })
            }
            uapi::drm_asahi_sync_type_DRM_ASAHI_SYNC_TIMELINE_SYNCOBJ => {
                let syncobj = drm::syncobj::SyncObj::lookup_handle(file, data.handle)?;
                let fence = if out {
                    None
                } else {
                    syncobj
                        .fence_get()
                        .ok_or_else(|| {
                            cls_pr_debug!(
                                Errors,
                                "Failed to get fence from timeline sync object\n"
                            );
                            EINVAL
                        })?
                        .chain_find_seqno(data.timeline_value)?
                };

                Ok(SyncItem {
                    fence,
                    syncobj,
                    chain_fence: None,
                    timeline_value: data.timeline_value,
                    cumulative: None,
                    timeline: true,
                })
            }
            _ => {
                cls_pr_debug!(Errors, "Invalid sync type {}\n", data.sync_type);
                Err(EINVAL)
            }
        }
    }

    fn parse_array(
        file: &DrmFile,
        ptr: u64,
        in_count: u32,
        out_count: u32,
        allow_cumulative: bool,
    ) -> Result<KVec<SyncItem>> {
        let count = in_count.checked_add(out_count).ok_or(EOVERFLOW)?;

        const STRIDE: usize = core::mem::size_of::<uapi::drm_asahi_sync>();
        let size = STRIDE.checked_mul(count as usize).ok_or(EOVERFLOW)?;
        let mut vec = KVec::with_capacity(count as usize, GFP_KERNEL)?;
        let mut anchor = None;

        // Read each record once so validation and use see the same retained bytes.
        let mut reader = UserSlice::new(UserPtr::from_addr(ptr as usize), size).reader();

        for i in 0..count {
            let mut sync: MaybeUninit<uapi::drm_asahi_sync> = MaybeUninit::uninit();

            // SAFETY: The size of `sync` is STRIDE
            reader.read_raw(unsafe {
                core::slice::from_raw_parts_mut(sync.as_mut_ptr().cast::<MaybeUninit<u8>>(), STRIDE)
            })?;

            // SAFETY: All bit patterns in the struct are valid
            let sync = unsafe { sync.assume_init() };

            if sync.sync_type == crate::g17::sync_output::SYNC_CUMULATIVE {
                if !allow_cumulative || i < in_count || anchor.is_some() {
                    return Err(EINVAL);
                }
                anchor = Some(i as usize);
            }
            vec.push(
                Self::parse_one(file, sync, i >= in_count, allow_cumulative)?,
                GFP_KERNEL,
            )?;
        }
        if let Some(anchor) = anchor {
            let bundle = crate::g17::sync_output::SignalBundle::new(
                &vec[in_count as usize..],
                anchor - in_count as usize,
            )?;
            vec[anchor].cumulative = Some(bundle);
        } else {
            for sync in &mut vec[in_count as usize..] {
                if sync.timeline {
                    sync.chain_fence = Some(dma_fence::FenceChain::new()?);
                }
            }
        }

        Ok(vec)
    }
}

#[derive(Clone)]
pub(crate) enum Object {
    TimestampBuffer(Arc<crate::timestamp::Buffer>),
}

/// A queue in a file's table. Its count in the creating process's share of the device's
/// queues is given back once the queue itself is gone.
pub(crate) struct TableQueue {
    queue: KBox<dyn queue::Queue>,
    _share: Option<queue_share::Share>,
}

impl Deref for TableQueue {
    type Target = dyn queue::Queue;

    fn deref(&self) -> &Self::Target {
        &*self.queue
    }
}

impl DerefMut for TableQueue {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut *self.queue
    }
}

/// State associated with a client.
// #[pin_data]
pub(crate) struct File {
    id: u64,
    /// Available VM range for the user
    user_range: Range<u64>,
    // Fields drop in order: the queues go first, so that a closing client's submitted work
    // can finish before its VMs unmap the user ranges.
    // #[pin]
    queues: xarray::XArray<Arc<Mutex<TableQueue>>>,
    // #[pin]
    vms: xarray::XArray<KBox<Vm>>,
    // #[pin]
    objects: xarray::XArray<KBox<Object>>,
    /// GPU memory this client created and holds.
    mem: Arc<memacct::Client>,
    /// Completed engine time, retained by queues after handle destruction.
    usage: Arc<crate::usage::Client>,
    /// Largest size this file has asked GET_PARAMS for the global parameters with
    /// ([`gpu::Gpu::admit_vm`]).
    params_size: core::sync::atomic::AtomicU64,
    /// The device's statistics, for the GPU frequency in fdinfo.
    stats: Option<Arc<crate::stats::StatsSnapshot>>,
    /// An exiting task's close of this file began ([`File::close_on_exit`]).
    drained: AtomicBool,
}

/// Convenience type alias for our DRM `File` type.
pub(crate) type DrmFile = drm::File<File>;

/// Minimum reserved AS for kernel mappings
const VM_KERNEL_MIN_SIZE: u64 = 0x20000000;

impl drm::file::DriverFile for File {
    type Driver = driver::AsahiDrmDriver;

    /// Create a new `File` instance for a fresh client.
    fn open(device: &drm::Device<Self::Driver, Registered>) -> Result<Pin<KBox<Self>>> {
        debug::update_debug_flags();

        device.registration_data_with(|reg_data| {
            let gpu = &reg_data.gpu;
            let id = gpu.ids().file.next();
            let user_range = gpu.user_range();
            let mem = memacct::Client::new()?;
            let usage = Arc::new(crate::usage::Client::new(), GFP_KERNEL)?;
            let stats = gpu.stats_snapshot();

            mod_dev_dbg!(device, "[File {}]: DRM device opened\n", id);
            KBox::pin_init(File::new(id, user_range, mem, usage, stats), GFP_KERNEL)
        })
    }

    /// The objects this client created, which `memacct` attributes to it, not every object it
    /// has a handle to: an imported or shared buffer is counted once, by its creator.
    fn memory_stats(&self) -> Option<drm::file::MemoryStats> {
        Some(drm::file::MemoryStats {
            private: self.mem.committed(),
            resident: Some(self.mem.resident()),
            ..Default::default()
        })
    }

    fn engine_stats(&self, mut emit: impl FnMut(&CStr, u64)) {
        let names = [
            kernel::c_str!("vertex"),
            kernel::c_str!("fragment"),
            kernel::c_str!("compute"),
        ];
        for (name, ns) in names.into_iter().zip(self.usage.snapshot()) {
            emit(name, ns);
        }
    }

    /// One clock drives every engine.
    fn engine_frequency(&self) -> Option<(Option<u64>, u64)> {
        self.stats.as_ref()?.engine_frequency()
    }

    fn as_raw(&self) -> *mut bindings::drm_file {
        todo!()
    }
}

// SAFETY: All bit patterns are valid by construction.
unsafe impl AnyBitPattern for uapi::drm_asahi_gem_bind_op {}

impl File {
    fn new(
        id: u64,
        user_range: Range<u64>,
        mem: Arc<memacct::Client>,
        usage: Arc<crate::usage::Client>,
        stats: Option<Arc<crate::stats::StatsSnapshot>>,
    ) -> impl PinInit<Self, Error> {
        // SAFETY: The initializer supplies aligned, pinned storage for Self.
        // XArray::new is infallible; each pinned array is initialized in place,
        // then every remaining field is written before returning success.
        unsafe {
            pin_init::pin_init_from_closure(move |slot: *mut Self| {
                let raw_vms = addr_of_mut!((*slot).vms);
                pin_init::raw_try_init(
                    raw_vms,
                    xarray::XArray::<KBox<Vm>>::new(xarray::AllocKind::Alloc1),
                )?;

                let raw_queues = addr_of_mut!((*slot).queues);
                pin_init::raw_try_init(
                    raw_queues,
                    xarray::XArray::<Arc<Mutex<TableQueue>>>::new(
                        xarray::AllocKind::Alloc1,
                    ),
                )?;

                let raw_objects = addr_of_mut!((*slot).objects);
                pin_init::raw_try_init(
                    raw_objects,
                    xarray::XArray::<KBox<Object>>::new(xarray::AllocKind::Alloc1),
                )?;

                (*slot).id = id;
                (*slot).user_range = user_range;
                addr_of_mut!((*slot).mem).write(mem);
                addr_of_mut!((*slot).usage).write(usage);
                addr_of_mut!((*slot).params_size).write(core::sync::atomic::AtomicU64::new(0));
                addr_of_mut!((*slot).stats).write(stats);
                addr_of_mut!((*slot).drained).write(AtomicBool::new(false));
                Ok(())
            })
        }
    }

    fn vms(self: Pin<&Self>) -> Pin<&xarray::XArray<KBox<Vm>>> {
        // SAFETY: Structural pinned projection for vms.
        // We never move out of this field.
        unsafe { self.map_unchecked(|s| &s.vms) }
    }

    #[allow(clippy::type_complexity)]
    fn queues(self: Pin<&Self>) -> Pin<&xarray::XArray<Arc<Mutex<TableQueue>>>> {
        // SAFETY: Structural pinned projection for queues.
        // We never move out of this field.
        unsafe { self.map_unchecked(|s| &s.queues) }
    }

    fn objects(self: Pin<&Self>) -> Pin<&xarray::XArray<KBox<Object>>> {
        // SAFETY: Structural pinned projection for objects.
        // We never move out of this field.
        unsafe { self.map_unchecked(|s| &s.objects) }
    }

    /// IOCTL: get_param: Get a driver parameter value.
    pub(crate) fn get_params(
        device: &AsahiDrmDevice,
        reg_data: &driver::AsahiDrmRegistrationData<'_>,
        data: &mut uapi::drm_asahi_get_params,
        file: &DrmFile,
    ) -> Result<u32> {
        mod_dev_dbg!(device, "[File {}]: IOCTL: get_params\n", file.inner().id);

        let gpu = &reg_data.gpu;

        match data.param_group {
            uapi::DRM_ASAHI_PARAM_GROUP_VM_STATUS if gpu.supports_vm_status() => {
                return Self::get_vm_status(data, file);
            }
            uapi::DRM_ASAHI_PARAM_GROUP_VM_STATUS_MIRROR if gpu.supports_vm_status() => {
                return Self::register_vm_status_mirror(data, file);
            }
            uapi::DRM_ASAHI_PARAM_GROUP_QUEUE_LIMITS => {
                if data.pad != 0
                    || data.size < core::mem::size_of::<uapi::drm_asahi_queue_limits>() as u64
                {
                    return Err(EINVAL);
                }
                let limits = gpu.queue_limits().ok_or(EINVAL)?;
                let words = [limits.max_queues, limits.max_in_flight_per_queue, 0, 0];
                UserSlice::new(
                    UserPtr::from_addr(data.pointer as usize),
                    core::mem::size_of_val(&words),
                )
                .writer()
                .write(&words)?;
                return Ok(0);
            }
            _ => {}
        }

        if data.param_group != 0 || data.pad != 0 {
            cls_pr_debug!(Errors, "get_params: Invalid arguments\n");
            return Err(EINVAL);
        }

        if gpu.is_crashed() {
            return Err(ENODEV);
        }

        let memory_limits = memacct::LEDGER.limits();
        let mut params = uapi::drm_asahi_params_global {
            features: 0,
            memory_limit: memory_limits.device,
            client_memory_limit: memory_limits.client,

            vm_start: file.inner().user_range.start,
            vm_end: file.inner().user_range.end,
            vm_kernel_min_size: VM_KERNEL_MIN_SIZE,

            max_commands_per_submission: MAX_COMMANDS_PER_SUBMISSION,
            max_attachments: crate::microseq::MAX_ATTACHMENTS as u32,

            command_timestamp_frequency_hz: 1_000_000_000, // User timestamps always in nanoseconds

            ..pin_init::zeroed()
        };

        gpu.get_params(&mut params)?;

        params.features |= gpu.feature_flags();
        if Self::supports_user_memory(reg_data) {
            params.features |= uapi::DRM_ASAHI_FEATURE_USERPTR as u64;
        }
        if Self::supports_residency_control(reg_data) {
            params.features |= uapi::DRM_ASAHI_FEATURE_GEM_RESIDENCY as u64;
        }

        let size = core::mem::size_of::<uapi::drm_asahi_params_global>().min(data.size.try_into()?);

        // Write the retained result to this user slice once.
        let mut params_writer =
            UserSlice::new(UserPtr::from_addr(data.pointer as usize), size).writer();

        // SAFETY: `size` is at most the sizeof of `params`
        params_writer.write_slice(unsafe {
            core::slice::from_raw_parts(core::ptr::from_ref(&params).cast::<u8>(), size)
        })?;

        file.inner()
            .params_size
            .fetch_max(data.size, core::sync::atomic::Ordering::Relaxed);
        Ok(0)
    }

    /// Query only file-owned VM state; this does not start recovery or wait for work.
    fn get_vm_status(data: &uapi::drm_asahi_get_params, file: &DrmFile) -> Result<u32> {
        let size = core::mem::size_of::<uapi::drm_asahi_vm_status>();
        if data.pad != 0 || data.size < size as u64 {
            return Err(EINVAL);
        }
        let user = UserPtr::from_addr(data.pointer as usize);
        let [vm_id, flags, _, pad] = UserSlice::new(user, size).reader().read::<[u32; 4]>()?;
        if flags != 0 || pad != 0 {
            return Err(EINVAL);
        }
        let status = Self::vm_status(file, vm_id)?;
        UserSlice::new(user, size).writer().write(&[
            vm_id,
            0,
            status.reported_error() as u32,
            0,
        ])?;
        Ok(0)
    }

    /// Register a file-owned GEM word as the mirror of a file-owned VM's status.
    fn register_vm_status_mirror(data: &uapi::drm_asahi_get_params, file: &DrmFile) -> Result<u32> {
        let size = core::mem::size_of::<uapi::drm_asahi_vm_status_mirror>();
        if data.pad != 0 || data.size < size as u64 {
            return Err(EINVAL);
        }
        let [vm_id, handle, offset, flags] =
            UserSlice::new(UserPtr::from_addr(data.pointer as usize), size)
                .reader()
                .read::<[u32; 4]>()?;
        if flags != 0 || handle == 0 || offset % core::mem::align_of::<i32>() as u32 != 0 {
            return Err(EINVAL);
        }
        let status = Self::vm_status(file, vm_id)?;
        let owner = gem::ObjectRef::new(gem::Object::lookup_handle(file, handle)?);
        status.attach_mirror(owner, offset as usize)?;
        Ok(0)
    }

    fn vm_status(file: &DrmFile, vm_id: u32) -> Result<Arc<crate::g17::status::VmStatus>> {
        let vms = file.inner().vms();
        let guard = vms.lock();
        Ok(guard
            .get(vm_id as usize)
            .ok_or(ENOENT)?
            .vm
            .status()
            .ok_or(EINVAL)?
            .clone())
    }

    /// IOCTL: vm_create: Create a new `Vm`.
    pub(crate) fn vm_create(
        device: &AsahiDrmDevice<drm::Registered>,
        reg_data: &driver::AsahiDrmRegistrationData<'_>,
        data: &mut uapi::drm_asahi_vm_create,
        file: &DrmFile,
    ) -> Result<u32> {
        if reg_data.gpu.as_any().is::<crate::g17::Gpu>() {
            // G17P reports HAL200; it accepts the acknowledgement but does not require it.
            if data.flags & !uapi::drm_asahi_vm_create_flags_DRM_ASAHI_VM_CREATE_HAL200 != 0 {
                return Err(EINVAL);
            }
        } else if data.flags != 0 {
            // Every other runtime here reports DRM_ASAHI_GPU_HAL_LEGACY.
            return Err(EINVAL);
        }
        reg_data.gpu.admit_vm(
            device.as_ref().as_ref(),
            file.inner()
                .params_size
                .load(core::sync::atomic::Ordering::Relaxed),
            data.flags,
        )?;

        let kernel_range = data.kernel_start..data.kernel_end;
        let user_range = file.inner().user_range.clone();

        // Validate requested kernel range
        if !user_range.is_superset(kernel_range.clone())
            || kernel_range.range() < VM_KERNEL_MIN_SIZE
            || kernel_range.start & (mmu::UAT_PGMSK as u64) != 0
            || kernel_range.end & (mmu::UAT_PGMSK as u64) != 0
        {
            cls_pr_debug!(Errors, "vm_create: Invalid kernel range\n");
            return Err(EINVAL);
        }

        // Align to buffer::PAGE_SIZE so the allocators are happy
        let kernel_range = align(kernel_range.start, buffer::PAGE_SIZE as u64)
            ..align_down(kernel_range.end, buffer::PAGE_SIZE as u64);

        let kernel_half_size = align_down(kernel_range.range() >> 1, buffer::PAGE_SIZE as u64);
        let kernel_gpu_range = kernel_range.start..(kernel_range.start + kernel_half_size);
        let kernel_gpufw_range = kernel_gpu_range.end..kernel_range.end;

        let gpu = &reg_data.gpu;
        let file_id = file.inner().id;
        let vm = gpu
            .new_vm_for_file(kernel_range.clone(), file_id)?
            .with_memory_client(file.inner().mem.clone());
        // G16 registers a retained backend owner before returning this VM. Any later
        // allocation can fail before file::Vm exists to run its normal close path.
        let failed_vm = ScopeGuard::new_with_data(
            (vm.clone(), user_range.clone(), kernel_range.clone()),
            |(vm, user_range, kernel_range)| {
                if gpu.tracked_submission() && vm.status().is_some() && !vm.is_closed() {
                    vm.close_idle_reclaim();
                    if let Err(error) = vm.unmap_user_ranges(user_range, kernel_range) {
                        pr_err!("VM create: closing unpublished VM failed: {:?}\n", error);
                    }
                    vm.bo_deferred_cleanup();
                    gpu.vm_closed(&vm);
                }
            },
        );

        let vm_xa = file.inner().vms();
        let resv = vm_xa.lock().reserve_limit(1..=u32::MAX, GFP_KERNEL)?;
        let id: u32 = resv.index().try_into()?;

        mod_dev_dbg!(device, "[File {} VM {}]: VM Create\n", file_id, id);
        mod_dev_dbg!(
            device,
            "[File {} VM {}]: Creating allocators\n",
            file_id,
            id
        );
        let ualloc = Arc::pin_init(
            new_mutex!(alloc::DefaultAllocator::new(
                device,
                &vm,
                kernel_gpu_range,
                buffer::PAGE_SIZE,
                mmu::PROT_GPU_SHARED_RW,
                512 * 1024,
                true,
                fmt!("File {} VM {} GPU Shared", file_id, id),
                false,
            )?),
            GFP_KERNEL,
        )?;
        let ualloc_priv = Arc::pin_init(
            new_mutex!(alloc::DefaultAllocator::new(
                device,
                &vm,
                kernel_gpufw_range,
                buffer::PAGE_SIZE,
                mmu::PROT_GPU_FW_PRIV_RW,
                64 * 1024,
                true,
                fmt!("File {} VM {} GPU FW Private", file_id, id),
                false,
            )?),
            GFP_KERNEL,
        )?;

        mod_dev_dbg!(
            device,
            "[File {} VM {}]: Creating dummy object\n",
            file_id,
            id
        );
        let mut dummy_obj = gem::new_kernel_object(device, 0x4000)?;
        // TODO: aren't new GEM shmem object zero initialized?
        let obj_size = dummy_obj.size();
        let vmap = dummy_obj.vmap()?;
        for off in 0..obj_size {
            vmap.try_write8(0x0, off)?;
        }
        let dummy_mapping = dummy_obj.map_at(
            &vm,
            gpu.unknown_page(),
            mmu::PROT_GPU_SHARED_RW,
            true,
        )?;

        mod_dev_dbg!(device, "[File {} VM {}]: VM created\n", file_id, id);
        resv.fill(KBox::new(
            Vm {
                shrinker: if vm.residency_reclaim_enabled() {
                    Some(mmu::VmShrinker::new(&vm)?)
                } else {
                    None
                },
                ualloc,
                ualloc_priv,
                vm,
                user_range,
                kernel_range,
                _dummy_mapping: dummy_mapping,
                gpu: gpu.clone(),
            },
            GFP_KERNEL,
        )?)?;

        drop(failed_vm.dismiss());
        data.vm_id = id;

        Ok(0)
    }

    /// IOCTL: vm_destroy: Destroy a `Vm`.
    pub(crate) fn vm_destroy(
        _device: &AsahiDrmDevice<drm::Registered>,
        _reg_data: &driver::AsahiDrmRegistrationData<'_>,
        data: &mut uapi::drm_asahi_vm_destroy,
        file: &DrmFile,
    ) -> Result<u32> {
        let vm = file.inner().vms().remove(data.vm_id as usize);
        if vm.is_none() {
            Err(ENOENT)
        } else {
            drop(vm);
            Ok(0)
        }
    }

    /// IOCTL: gem_create: Create a new GEM object.
    pub(crate) fn gem_create(
        device: &AsahiDrmDevice,
        reg_data: &driver::AsahiDrmRegistrationData<'_>,
        data: &mut uapi::drm_asahi_gem_create,
        file: &DrmFile,
    ) -> Result<u32> {
        mod_dev_dbg!(
            device,
            "[File {}]: IOCTL: gem_create size={:#x?}\n",
            file.inner().id,
            data.size
        );

        if (data.flags
            & !(uapi::drm_asahi_gem_flags_DRM_ASAHI_GEM_WRITEBACK
                | uapi::drm_asahi_gem_flags_DRM_ASAHI_GEM_VM_PRIVATE))
            != 0
            || (data.flags & uapi::drm_asahi_gem_flags_DRM_ASAHI_GEM_VM_PRIVATE == 0
                && data.vm_id != 0)
        {
            cls_pr_debug!(Errors, "gem_create: Invalid arguments\n");
            return Err(EINVAL);
        }

        let resv_gem;
        let mut reclaim_cpu = false;
        let resv_obj = if data.flags & uapi::drm_asahi_gem_flags_DRM_ASAHI_GEM_VM_PRIVATE != 0 {
            let vms = file.inner().vms();
            let vms = vms.lock();
            let vm = &vms.get(data.vm_id.try_into()?).ok_or(ENOENT)?.vm;
            reclaim_cpu = vm.residency_reclaim_enabled();
            resv_gem = vm.get_resv_obj();
            Some(resv_gem.deref())
        } else {
            None
        };

        let size = gem::checked_object_size(data.size.try_into()?)?;
        let commit = memacct::LEDGER
            .admit(&file.inner().mem, size as u64)
            .inspect_err(|_| {
                cls_pr_debug!(
                    Errors,
                    "gem_create: {:#x} bytes refused by the GPU memory limits\n",
                    size
                );
            })?;
        // Large G17 user objects are populated in full by the first VM_BIND.
        // Let them release clean page cache, but do not swap out the rest of
        // the system for them folio by folio. Userspace can evict its BO
        // cache and retry after ENOMEM.
        let pages_bounded_reclaim =
            gem::user_pages_bounded_reclaim(size, reg_data.gpu.as_any().is::<crate::g17::Gpu>());
        let gem = gem::new_object(
            device,
            size,
            data.flags,
            resv_obj,
            reclaim_cpu,
            pages_bounded_reclaim,
            commit,
        )?;
        if reg_data.gpu.prepare_new_objects() {
            #[cfg(CONFIG_DRM_ASAHI_G15)]
            if reg_data.gpu.as_any().is::<crate::g15::Gpu>() {
                gem::prepare_new_object_g15(&gem)?;
            } else {
                let registered = device.registration_guard().ok_or(ENODEV)?;
                let parent: &kernel::platform::Device<kernel::device::Bound> = registered.as_ref();
                gem::prepare_new_object(&gem, parent.as_ref())?;
            }
            #[cfg(not(CONFIG_DRM_ASAHI_G15))]
            let registered = device.registration_guard().ok_or(ENODEV)?;
            #[cfg(not(CONFIG_DRM_ASAHI_G15))]
            {
                let parent: &kernel::platform::Device<kernel::device::Bound> = registered.as_ref();
                gem::prepare_new_object(&gem, parent.as_ref())?;
            }
        }

        let handle = gem.create_handle(file)?;
        data.handle = handle;

        mod_dev_dbg!(
            device,
            "[File {}]: IOCTL: gem_create size={:#x} handle={:#x?}\n",
            file.inner().id,
            data.size,
            data.handle
        );

        Ok(0)
    }

    /// Whether this GPU takes objects over user memory (DRM_IOCTL_ASAHI_GEM_USERPTR).
    ///
    /// The bindings of G17 user VMs are the only ones that admit such an object's pages. They
    /// are GPU-only page-table entries, and the GPU's accesses through those are taken to be
    /// coherent with the CPU caches, as they are for the cacheable CPU mappings of ordinary
    /// objects (DRM_ASAHI_GEM_WRITEBACK): see `gem::new_user_memory_object()`.
    fn supports_user_memory(reg_data: &driver::AsahiDrmRegistrationData<'_>) -> bool {
        reg_data.gpu.as_any().is::<crate::g17::Gpu>()
    }

    /// IOCTL: gem_userptr: Create a GEM object over the caller's own memory.
    pub(crate) fn gem_userptr(
        device: &AsahiDrmDevice,
        reg_data: &driver::AsahiDrmRegistrationData<'_>,
        data: &mut uapi::drm_asahi_gem_userptr,
        file: &DrmFile,
    ) -> Result<u32> {
        mod_dev_dbg!(
            device,
            "[File {}]: IOCTL: gem_userptr addr={:#x} size={:#x}\n",
            file.inner().id,
            data.userptr,
            data.size
        );

        if !Self::supports_user_memory(reg_data) {
            return Err(EOPNOTSUPP);
        }
        let private = data.flags & uapi::drm_asahi_gem_flags_DRM_ASAHI_GEM_VM_PRIVATE != 0;
        if data.flags
            & !(uapi::drm_asahi_gem_flags_DRM_ASAHI_GEM_VM_PRIVATE
                | uapi::drm_asahi_gem_flags_DRM_ASAHI_GEM_USERPTR_READ_ONLY)
            != 0
            || (!private && data.vm_id != 0)
            || data.pad != 0
            || data.size == 0
            || (data.userptr | data.size) & (mmu::UAT_PGMSK | (kernel::page::PAGE_SIZE - 1)) as u64
                != 0
        {
            cls_pr_debug!(Errors, "gem_userptr: Invalid arguments\n");
            return Err(EINVAL);
        }
        let addr = usize::try_from(data.userptr)?;
        let size = gem::checked_object_size(data.size.try_into()?)?;
        let writable = data.flags & uapi::drm_asahi_gem_flags_DRM_ASAHI_GEM_USERPTR_READ_ONLY == 0;

        let resv_gem;
        let resv_obj = if private {
            let vms = file.inner().vms();
            let vms = vms.lock();
            resv_gem = vms
                .get(data.vm_id.try_into()?)
                .ok_or(ENOENT)?
                .vm
                .get_resv_obj();
            Some(resv_gem.deref())
        } else {
            None
        };

        let commit = memacct::LEDGER
            .admit(&file.inner().mem, size as u64)
            .inspect_err(|_| {
                cls_pr_debug!(
                    Errors,
                    "gem_userptr: {:#x} bytes refused by the GPU memory limits\n",
                    size
                );
            })?;
        let pages = gem::UserPages::pin(addr, size, writable).inspect_err(|e| {
            cls_pr_debug!(Errors, "gem_userptr: pinning failed: {:?}\n", e);
        })?;
        let registered = device.registration_guard().ok_or(ENODEV)?;
        let parent: &kernel::platform::Device<kernel::device::Bound> = registered.as_ref();
        let gem = gem::new_user_memory_object(
            device,
            parent.as_ref(),
            pages,
            data.flags,
            resv_obj,
            commit,
        )?;
        drop(registered);

        data.handle = gem.create_handle(file)?;

        mod_dev_dbg!(
            device,
            "[File {}]: IOCTL: gem_userptr size={:#x} handle={:#x?}\n",
            file.inner().id,
            size,
            data.handle
        );

        Ok(0)
    }

    /// Whether this GPU takes residency requests (DRM_IOCTL_ASAHI_GEM_RESIDENCY): only G17 user
    /// VMs release idle memory under pressure.
    fn supports_residency_control(reg_data: &driver::AsahiDrmRegistrationData<'_>) -> bool {
        reg_data.gpu.as_any().is::<crate::g17::Gpu>()
    }

    /// IOCTL: gem_residency: Keep an object's memory resident, or return it to the default
    /// policy.
    pub(crate) fn gem_residency(
        _device: &AsahiDrmDevice,
        reg_data: &driver::AsahiDrmRegistrationData<'_>,
        data: &mut uapi::drm_asahi_gem_residency,
        file: &DrmFile,
    ) -> Result<u32> {
        if !Self::supports_residency_control(reg_data) {
            return Err(EOPNOTSUPP);
        }
        if data.flags & !uapi::drm_asahi_gem_residency_flags_DRM_ASAHI_GEM_RESIDENCY_KEEP != 0 {
            return Err(EINVAL);
        }
        let gem = gem::Object::lookup_handle(file, data.handle)?;
        gem::set_keep_resident(
            &gem,
            data.flags & uapi::drm_asahi_gem_residency_flags_DRM_ASAHI_GEM_RESIDENCY_KEEP != 0,
        )?;
        Ok(0)
    }

    /// IOCTL: gem_mmap_offset: Assign an mmap offset to a GEM object.
    pub(crate) fn gem_mmap_offset(
        device: &AsahiDrmDevice,
        _reg_data: &driver::AsahiDrmRegistrationData<'_>,
        data: &mut uapi::drm_asahi_gem_mmap_offset,
        file: &DrmFile,
    ) -> Result<u32> {
        mod_dev_dbg!(
            device,
            "[File {}]: IOCTL: gem_mmap_offset handle={:#x?}\n",
            file.inner().id,
            data.handle
        );

        if data.flags != 0 {
            cls_pr_debug!(Errors, "gem_mmap_offset: Unexpected flags\n");
            return Err(EINVAL);
        }

        let gem = gem::Object::lookup_handle(file, data.handle)?;
        // The caller maps a user-memory object's memory itself.
        if gem.is_user_memory() {
            return Err(EINVAL);
        }
        data.offset = gem.create_mmap_offset()?;
        Ok(0)
    }

    /// IOCTL: gem_madvise: Mark a VM-private object purgeable or needed.
    pub(crate) fn gem_madvise(
        _device: &AsahiDrmDevice,
        _reg_data: &driver::AsahiDrmRegistrationData<'_>,
        data: &mut uapi::drm_asahi_gem_madvise,
        file: &DrmFile,
    ) -> Result<u32> {
        if data.pad != 0 || data.madv > uapi::drm_asahi_gem_madv_DRM_ASAHI_GEM_MADV_DONTNEED {
            return Err(EINVAL);
        }
        let gem = gem::Object::lookup_handle(file, data.handle)?;
        data.retained = gem::madvise(
            &gem,
            data.madv == uapi::drm_asahi_gem_madv_DRM_ASAHI_GEM_MADV_DONTNEED,
        )?;
        Ok(0)
    }

    /// IOCTL: vm_bind: Map or unmap memory into a Vm.
    pub(crate) fn vm_bind(
        device: &AsahiDrmDevice<drm::Registered>,
        reg_data: &driver::AsahiDrmRegistrationData<'_>,
        data: &mut uapi::drm_asahi_vm_bind,
        file: &DrmFile,
    ) -> Result<u32> {
        mod_dev_dbg!(
            device,
            "[File {} VM {}]: IOCTL: vm_bind\n",
            file.inner().id,
            data.vm_id,
        );

        if data.stride == 0 || data.pad != 0 {
            cls_pr_debug!(Errors, "vm_bind: Unexpected headers\n");
            return Err(EINVAL);
        }

        if let Some(limit) = reg_data.gpu.bind_batch_limit() {
            if data.num_binds == 0
                || data.num_binds > limit
                || data.userptr == 0
                || data.stride % 8 != 0
                || data.stride as usize > kernel::page::PAGE_SIZE
            {
                cls_pr_debug!(Errors, "vm_bind: Batch outside the GPU's limits\n");
                return Err(EINVAL);
            }
        }

        if reg_data.gpu.as_any().is::<crate::g17::Gpu>() || reg_data.gpu.tracked_submission() {
            return Self::vm_bind_g17(data, file);
        }

        let vm_id = data.vm_id.try_into()?;

        let mut vec = KVec::new();
        let size = usize::try_from(data.stride)?
            .checked_mul(usize::try_from(data.num_binds)?)
            .ok_or(EOVERFLOW)?;
        let reader = UserSlice::new(UserPtr::from_addr(data.userptr as usize), size).reader();
        reader.read_all(&mut vec, GFP_KERNEL)?;
        let mut reader = Reader::new(&vec);

        for _i in 0..data.num_binds {
            let bind: uapi::drm_asahi_gem_bind_op = reader.read_up_to(data.stride as usize)?;
            Self::do_gem_bind_unbind(device, vm_id, &bind, file)?;
        }

        Ok(0)
    }

    /// Validate all operations and retain their objects before changing a tracked VM.
    fn vm_bind_g17(data: &uapi::drm_asahi_vm_bind, file: &DrmFile) -> Result<u32> {
        let count = usize::try_from(data.num_binds)?;
        let stride = usize::try_from(data.stride)?;
        let size = stride.checked_mul(count).ok_or(EOVERFLOW)?;
        let mut bytes = KVec::new();
        UserSlice::new(UserPtr::from_addr(data.userptr as usize), size)
            .reader()
            .read_all(&mut bytes, GFP_KERNEL)?;
        let (vm, user_range, kernel_range) = {
            let vms = file.inner().vms();
            let guard = vms.lock();
            let entry = guard.get(data.vm_id as usize).ok_or(ENOENT)?;
            (
                entry.vm.clone(),
                entry.user_range.clone(),
                entry.kernel_range.clone(),
            )
        };
        let mut batch = mmu::PreparedUserBindBatch::new(count)?;
        let mut unmaps = 0;
        // A fresh reader per declared stride also skips extension bytes in longer records.
        for bytes in bytes.chunks_exact(stride) {
            let op: uapi::drm_asahi_gem_bind_op = Reader::new(bytes).read_up_to(stride)?;
            if op.range == 0 || (op.addr | op.range | op.offset) & mmu::UAT_PGMSK as u64 != 0 {
                return Err(EINVAL);
            }
            let range = op.addr..op.addr.checked_add(op.range).ok_or(EINVAL)?;
            if !user_range.is_superset(range.clone())
                || kernel_range.overlaps(range.clone())
                || vm.driver_range_overlaps(range.clone())
            {
                return Err(EINVAL);
            }
            if op.flags & uapi::drm_asahi_bind_flags_DRM_ASAHI_BIND_UNBIND != 0 {
                if op.offset != 0
                    || op.flags != uapi::drm_asahi_bind_flags_DRM_ASAHI_BIND_UNBIND
                    || op.handle != 0
                {
                    return Err(EINVAL);
                }
                batch.push_unmap(vm.prepare_user_unmap(op.addr, op.range)?)?;
                unmaps += 1;
                continue;
            }
            vm.wait_for_user_map_admission()?;
            if op.flags
                & !(uapi::drm_asahi_bind_flags_DRM_ASAHI_BIND_READ
                    | uapi::drm_asahi_bind_flags_DRM_ASAHI_BIND_WRITE
                    | uapi::drm_asahi_bind_flags_DRM_ASAHI_BIND_SINGLE_PAGE
                    | DRM_ASAHI_BIND_SPARSE_TILE)
                != 0
            {
                return Err(EINVAL);
            }
            let sparse_tile = op.flags & DRM_ASAHI_BIND_SPARSE_TILE != 0;
            let read = op.flags & uapi::drm_asahi_bind_flags_DRM_ASAHI_BIND_READ != 0;
            let write = op.flags & uapi::drm_asahi_bind_flags_DRM_ASAHI_BIND_WRITE != 0;
            let prot = match (read, write) {
                (true, true) => mmu::PROT_GPU_SHARED_RW,
                (true, false) => mmu::PROT_GPU_SHARED_RO,
                (false, true) => mmu::PROT_GPU_SHARED_WO,
                (false, false) => return Err(EINVAL),
            };
            let single_page = op.flags & uapi::drm_asahi_bind_flags_DRM_ASAHI_BIND_SINGLE_PAGE != 0;
            let object = gem::Object::lookup_handle(file, op.handle)?;
            let accessed = if single_page {
                mmu::UAT_PGSZ as u64
            } else {
                op.range
            };
            if op.offset.checked_add(accessed).ok_or(EINVAL)? > u64::try_from(object.size())? {
                return Err(EINVAL);
            }
            vm.validate_context_binding(op.addr, op.range, single_page)?;
            batch.push_map(vm.prepare_bind_object(
                &object,
                op.addr,
                op.range,
                op.offset,
                prot,
                single_page,
                sparse_tile,
            )?)?;
        }
        vm.reserve_deferred_user_unmaps(unmaps)?;
        vm.commit_prepared_user_bind_batch(&mut batch)?;
        vm.bo_deferred_cleanup();
        Ok(0)
    }

    pub(crate) fn do_gem_bind_unbind(
        device: &AsahiDrmDevice<drm::Registered>,
        vm_id: usize,
        data: &uapi::drm_asahi_gem_bind_op,
        file: &DrmFile,
    ) -> Result<u32> {
        if data.range == 0 {
            return Err(EINVAL);
        }
        if (data.flags & uapi::drm_asahi_bind_flags_DRM_ASAHI_BIND_UNBIND) != 0 {
            Self::do_gem_unbind(device, vm_id, data, file)
        } else {
            Self::do_gem_bind(device, vm_id, data, file)
        }
    }

    pub(crate) fn do_gem_bind(
        device: &AsahiDrmDevice<drm::Registered>,
        vm_id: usize,
        data: &uapi::drm_asahi_gem_bind_op,
        file: &DrmFile,
    ) -> Result<u32> {
        if (data.addr | data.range | data.offset) as usize & mmu::UAT_PGMSK != 0 {
            cls_pr_debug!(
                Errors,
                "gem_bind: Addr/range/offset not page aligned: {:#x} {:#x}\n",
                data.addr,
                data.range
            );
            return Err(EINVAL); // Must be page aligned
        }

        if (data.flags
            & !(uapi::drm_asahi_bind_flags_DRM_ASAHI_BIND_READ
                | uapi::drm_asahi_bind_flags_DRM_ASAHI_BIND_WRITE
                | uapi::drm_asahi_bind_flags_DRM_ASAHI_BIND_SINGLE_PAGE))
            != 0
        {
            cls_pr_debug!(Errors, "gem_bind: Invalid flags {:#x}\n", data.flags);
            return Err(EINVAL);
        }

        let single_page = data.flags & uapi::drm_asahi_bind_flags_DRM_ASAHI_BIND_SINGLE_PAGE != 0;

        let bo = gem::Object::lookup_handle(file, data.handle)?;

        let start = data.addr;
        let end = data.addr.checked_add(data.range).ok_or(EINVAL)?;
        let range = start..end;

        let bo_accessed_size = if single_page {
            mmu::UAT_PGMSK as u64
        } else {
            data.range
        };
        let end_off = data.offset.checked_add(bo_accessed_size).ok_or(EINVAL)?;
        if end_off as usize > bo.size() {
            return Err(EINVAL);
        }

        if !file.inner().user_range.is_superset(range.clone()) {
            cls_pr_debug!(
                Errors,
                "gem_bind: Invalid map range {:#x}..{:#x} (not contained in user range)\n",
                start,
                end
            );
            return Err(EINVAL); // Invalid map range
        }

        let prot = if data.flags & uapi::drm_asahi_bind_flags_DRM_ASAHI_BIND_READ != 0 {
            if data.flags & uapi::drm_asahi_bind_flags_DRM_ASAHI_BIND_WRITE != 0 {
                mmu::PROT_GPU_SHARED_RW
            } else {
                mmu::PROT_GPU_SHARED_RO
            }
        } else if data.flags & uapi::drm_asahi_bind_flags_DRM_ASAHI_BIND_WRITE != 0 {
            mmu::PROT_GPU_SHARED_WO
        } else {
            cls_pr_debug!(
                Errors,
                "gem_bind: Must specify read or write (flags: {:#x})\n",
                data.flags
            );
            return Err(EINVAL); // Must specify one of DRM_ASAHI_BIND_{READ,WRITE}
        };

        let vms_xa = file.inner().vms();
        let guard = vms_xa.lock();
        let guarded_vm = guard.get(vm_id).ok_or(ENOENT)?;

        // Clone it immediately so we aren't holding the XArray lock
        let vm = guarded_vm.vm.clone();
        let kernel_range = guarded_vm.kernel_range.clone();
        let _ = guarded_vm;
        core::mem::drop(guard);

        if kernel_range.overlaps(range) {
            cls_pr_debug!(
                Errors,
                "gem_bind: Invalid map range {:#x}..{:#x} (intrudes in kernel range)\n",
                start,
                end
            );
            return Err(EINVAL);
        }

        vm.bind_object(
            device,
            &bo,
            data.addr,
            data.range,
            data.offset,
            prot,
            single_page,
        )?;

        vm.bo_deferred_cleanup();

        Ok(0)
    }

    pub(crate) fn do_gem_unbind(
        device: &AsahiDrmDevice<drm::Registered>,
        vm_id: usize,
        data: &uapi::drm_asahi_gem_bind_op,
        file: &DrmFile,
    ) -> Result<u32> {
        if data.offset != 0
            || data.flags != uapi::drm_asahi_bind_flags_DRM_ASAHI_BIND_UNBIND
            || data.handle != 0
        {
            cls_pr_debug!(Errors, "gem_unbind: offset/flags/handle not zero\n");
            return Err(EINVAL);
        }

        if (data.addr | data.range) as usize & mmu::UAT_PGMSK != 0 {
            cls_pr_debug!(
                Errors,
                "gem_bind: Addr/range/offset not page aligned: {:#x} {:#x}\n",
                data.addr,
                data.range
            );
            return Err(EINVAL); // Must be page aligned
        }

        let start = data.addr;
        let end = data.addr.checked_add(data.range).ok_or(EINVAL)?;
        let range = start..end;

        if !file.inner().user_range.is_superset(range.clone()) {
            cls_pr_debug!(
                Errors,
                "gem_bind: Invalid unmap range {:#x}..{:#x} (not contained in user range)\n",
                start,
                end
            );
            return Err(EINVAL); // Invalid map range
        }

        let vms_xa = file.inner().vms();
        let guard = vms_xa.lock();
        let guarded_vm = guard.get(vm_id).ok_or(ENOENT)?;

        // Clone it immediately so we aren't holding the XArray lock
        let vm = guarded_vm.vm.clone();
        let kernel_range = guarded_vm.kernel_range.clone();
        let _ = guarded_vm;
        core::mem::drop(guard);

        if kernel_range.overlaps(range.clone()) {
            cls_pr_debug!(
                Errors,
                "gem_bind: Invalid unmap range {:#x}..{:#x} (intrudes in kernel range)\n",
                start,
                end
            );
            return Err(EINVAL);
        }

        vm.unmap_range(Some(device.as_ref().as_ref()), range.start, range.range())?;

        vm.bo_deferred_cleanup();

        Ok(0)
    }

    /// Runs once, at the first GEM handle close of an exiting task's file.
    ///
    /// Handles close through GEM_CLOSE, which an exiting task cannot issue, or when the file is
    /// released, so an exiting task here is releasing the file.
    ///
    /// Such a file closes its GEM handles before the driver sees it go. The client is gone:
    /// its G17 VMs are marked closed first, as `File::drop` does, so work that fails once an
    /// object leaves is unobserved teardown. Closing a handle also unmaps the object at
    /// once from VMs that do not track jobs (G13/G14), where work left running would fault,
    /// so the queues are drained. A live client unmaps only what its work no longer uses.
    fn close_on_exit(file: &DrmFile) {
        // SAFETY: `current` is valid for the running task.
        let exiting =
            unsafe { (*kernel::task::Task::current_raw()).flags } & bindings::PF_EXITING != 0;
        if !exiting || file.inner().drained.swap(true, Ordering::AcqRel) {
            return;
        }
        // M5 VMs keep their mark at file close.
        Self::mark_clients_closed(file.inner(), |vm| {
            vm.gpu.as_any().is::<crate::g17::Gpu>() && !vm.vm.is_m5()
        });
        let mut index = 0;
        loop {
            // Clone since we can't hold the xarray spinlock while waiting.
            let Some((idx, queue)) =
                file.inner()
                    .queues()
                    .find(index, usize::MAX)
                    .map(|(idx, guard)| {
                        let queue: Arc<Mutex<TableQueue>> = guard.borrow().into();
                        (idx, queue)
                    })
            else {
                break;
            };
            queue.lock().drain();
            if idx == usize::MAX {
                break;
            }
            index = idx + 1;
        }
    }

    /// Tells the status of each VM `which` selects that its client is gone: nobody sees the
    /// failures of the client's remaining work any more.
    fn mark_clients_closed(this: Pin<&Self>, which: impl Fn(&Vm) -> bool) {
        let mut index = 0;
        while let Some((idx, entry)) = this.vms().find(index, usize::MAX) {
            let vm: &Vm = entry.borrow();
            if which(vm) {
                if let Some(status) = vm.vm.status() {
                    status.mark_client_closed();
                }
            }
            if idx == usize::MAX {
                break;
            }
            index = idx + 1;
        }
    }

    pub(crate) fn unbind_gem_object(file: &DrmFile, bo: &gem::Object) -> Result {
        Self::close_on_exit(file);
        // TODO: use iter()
        let mut index = 0;
        loop {
            let vms = file.inner().vms();
            let item = vms.find(index, usize::MAX);
            match item {
                Some((idx, file_vm)) => {
                    // Clone since we can't hold the xarray spinlock while
                    // calling drop_mappings()
                    let vm = file_vm.borrow().vm.clone();
                    core::mem::drop(file_vm);
                    vm.drop_mappings(bo)?;
                    if idx == usize::MAX {
                        break;
                    }
                    index = idx + 1;
                }
                None => break,
            }
        }
        Ok(())
    }

    /// IOCTL: gem_bind_object: Map or unmap a GEM object as a special object.
    pub(crate) fn gem_bind_object(
        device: &AsahiDrmDevice<drm::Registered>,
        reg_data: &driver::AsahiDrmRegistrationData<'_>,
        data: &mut uapi::drm_asahi_gem_bind_object,
        file: &DrmFile,
    ) -> Result<u32> {
        mod_dev_dbg!(
            device,
            "[File {} VM {}]: IOCTL: gem_bind_object op={:?} handle={:#x?} flags={:#x?} {:#x?}:{:#x?} object_handle={:#x?}\n",
            file.inner().id,
            data.vm_id,
            data.op,
            data.handle,
            data.flags,
            data.offset,
            data.range,
            data.object_handle
        );

        if data.pad != 0 {
            cls_pr_debug!(Errors, "gem_bind_object: Unexpected pad\n");
            return Err(EINVAL);
        }

        if data.vm_id != 0 {
            cls_pr_debug!(Errors, "gem_bind_object: Unexpected vm_id\n");
            return Err(EINVAL);
        }

        match data.op {
            uapi::drm_asahi_bind_object_op_DRM_ASAHI_BIND_OBJECT_OP_BIND => {
                Self::do_gem_bind_object(reg_data, data, file)
            }
            uapi::drm_asahi_bind_object_op_DRM_ASAHI_BIND_OBJECT_OP_UNBIND => {
                Self::do_gem_unbind_object(device, data, file)
            }
            _ => {
                cls_pr_debug!(Errors, "gem_bind_object: Invalid op {}\n", data.op);
                Err(EINVAL)
            }
        }
    }

    pub(crate) fn do_gem_bind_object(
        reg_data: &driver::AsahiDrmRegistrationData<'_>,
        // device: &AsahiDrmDevice,
        data: &mut uapi::drm_asahi_gem_bind_object,
        file: &DrmFile,
    ) -> Result<u32> {
        if (data.range | data.offset) as usize & mmu::UAT_PGMSK != 0 {
            cls_pr_debug!(
                Errors,
                "gem_bind_object: Range/offset not page aligned: {:#x} {:#x}\n",
                data.range,
                data.offset
            );
            return Err(EINVAL); // Must be page aligned
        }

        if data.flags != uapi::drm_asahi_bind_object_flags_DRM_ASAHI_BIND_OBJECT_USAGE_TIMESTAMPS {
            cls_pr_debug!(Errors, "gem_bind_object: Invalid flags {:#x}\n", data.flags);
            return Err(EINVAL);
        }

        let offset = data.offset.try_into()?;
        let end_offset = data
            .offset
            .checked_add(data.range)
            .ok_or(EINVAL)?
            .try_into()?;
        let bo = gem::ObjectRef::new(gem::Object::lookup_handle(file, data.handle)?);

        let mapping = Arc::new(
            reg_data.gpu.map_timestamp_buffer(bo, offset..end_offset)?,
            GFP_KERNEL,
        )?;
        let obj = KBox::new(Object::TimestampBuffer(mapping), GFP_KERNEL)?;
        let handle = file
            .inner()
            .objects()
            .lock()
            .insert_limit(1..=u32::MAX, obj, GFP_KERNEL);
        // StoreError owns the rejected object. Convert it only after the
        // temporary XArray guard is gone: dropping a mapping may sleep.
        let handle = handle? as u64;

        data.object_handle = handle as u32;
        Ok(0)
    }

    pub(crate) fn do_gem_unbind_object(
        _device: &AsahiDrmDevice,
        data: &mut uapi::drm_asahi_gem_bind_object,
        file: &DrmFile,
    ) -> Result<u32> {
        if data.range != 0 || data.offset != 0 {
            cls_pr_debug!(
                Errors,
                "gem_unbind_object: Range/offset not zero: {:#x} {:#x}\n",
                data.range,
                data.offset
            );
            return Err(EINVAL);
        }

        if data.flags != 0 {
            cls_pr_debug!(
                Errors,
                "gem_unbind_object: Invalid flags {:#x}\n",
                data.flags
            );
            return Err(EINVAL);
        }

        if data.handle != 0 {
            cls_pr_debug!(
                Errors,
                "gem_unbind_object: Invalid handle {}\n",
                data.handle
            );
            return Err(EINVAL);
        }

        let object = file.inner().objects().remove(data.object_handle as usize);
        if object.is_none() {
            Err(ENOENT)
        } else {
            Ok(0)
        }
    }

    /// IOCTL: queue_create: Create a new command submission queue of a given type.
    pub(crate) fn queue_create(
        device: &AsahiDrmDevice<drm::Registered>,
        reg_data: &driver::AsahiDrmRegistrationData<'_>,
        data: &mut uapi::drm_asahi_queue_create,
        file: &DrmFile,
    ) -> Result<u32> {
        let file_id = file.inner().id;

        mod_dev_dbg!(
            device,
            "[File {} VM {}]: Creating queue prio={:?} flags={:#x?}\n",
            file_id,
            data.vm_id,
            data.priority,
            data.flags,
        );

        if data.pad != 0 {
            cls_pr_debug!(Errors, "queue_create: Invalid arguments\n");
            return Err(EINVAL);
        }
        if data.flags & !uapi::drm_asahi_queue_create_flags_DRM_ASAHI_QUEUE_CREATE_ASYNC_SUBMIT != 0
            || data.priority > uapi::drm_asahi_priority_DRM_ASAHI_PRIORITY_REALTIME
        {
            cls_pr_debug!(Errors, "queue_create: Invalid arguments\n");
            return Err(EINVAL);
        }

        // REALTIME stays reserved. HIGH needs CAP_SYS_NICE or DRM master, as amdgpu and
        // panthor require for raised queue priority; EACCES lets userspace fall back to MEDIUM.
        if data.priority == uapi::drm_asahi_priority_DRM_ASAHI_PRIORITY_REALTIME {
            cls_pr_debug!(Errors, "queue_create: Invalid priority\n");
            return Err(EINVAL);
        }
        // Keep support tied to a backend and admitted profile. G13/G14 and
        // unqualified M3 profiles retain their previous rejection.
        let high_supported = reg_data.gpu.as_any().is::<crate::g17::Gpu>();
        #[cfg(CONFIG_DRM_ASAHI_M3_T8122)]
        let high_supported = high_supported || reg_data.gpu.as_any()
            .downcast_ref::<crate::m3::m3_drm::Backend>()
            .is_some_and(|gpu| gpu.supports_high_priority());
        if data.priority == uapi::drm_asahi_priority_DRM_ASAHI_PRIORITY_HIGH
            && !high_supported
        {
            cls_pr_debug!(
                Errors,
                "queue_create: HIGH priority is not supported on this GPU\n"
            );
            return Err(EINVAL);
        }
        if data.priority == uapi::drm_asahi_priority_DRM_ASAHI_PRIORITY_HIGH
            // SAFETY: capable() reads the current ioctl task's credentials.
            && !unsafe { bindings::capable(bindings::CAP_SYS_NICE as i32) }
            && !file.is_current_master()
        {
            cls_pr_debug!(
                Errors,
                "queue_create: HIGH priority needs CAP_SYS_NICE or DRM master\n"
            );
            return Err(EACCES);
        }

        // The queue counts against the creating process's share of the device's independent
        // queues, whichever of its files it is created on, so no process can take every
        // execution context and leave the others, the compositor included, unable to create
        // one (see [`queue_share`]); a refusal fails with EBUSY.
        let share = reg_data
            .gpu
            .queue_limits()
            .map(|limits| queue_share::Share::take_current(limits.max_queues))
            .transpose()?;
        let queues_xa = file.inner().queues();
        let resv = queues_xa.lock().reserve_limit(1..=u32::MAX, GFP_KERNEL)?;
        let vms_xa = file.inner().vms();
        let guard = vms_xa.lock();
        let file_vm = guard.get(data.vm_id.try_into()?).ok_or(ENOENT)?;
        let vm = file_vm.vm.clone();
        let ualloc = file_vm.ualloc.clone();
        let ualloc_priv = file_vm.ualloc_priv.clone();
        // Drop the vms lock eagerly
        let _ = file_vm;
        core::mem::drop(guard);

        let mut queue = reg_data.gpu.new_queue(
            vm,
            ualloc,
            ualloc_priv,
            // TODO: Plumb deeper the enum
            uapi::drm_asahi_priority_DRM_ASAHI_PRIORITY_REALTIME - data.priority,
            data.usc_exec_base,
            file.inner().usage.clone(),
        )?;

        queue.reserve_render_scs(data.render_scs_layout)?;
        if data.compute_timeout_ms != 0 {
            data.compute_timeout_ms = queue.set_compute_timeout(data.compute_timeout_ms)?;
        }
        if data.flags & uapi::drm_asahi_queue_create_flags_DRM_ASAHI_QUEUE_CREATE_ASYNC_SUBMIT != 0 {
            queue.set_async_submit()?;
        }

        data.queue_id = resv.index().try_into()?;
        resv.fill(Arc::pin_init(
            new_mutex!(TableQueue {
                queue,
                _share: share,
            }),
            GFP_KERNEL,
        )?)?;

        Ok(0)
    }

    /// IOCTL: queue_destroy: Destroy a command submission queue.
    pub(crate) fn queue_destroy(
        _device: &AsahiDrmDevice,
        _reg_data: &driver::AsahiDrmRegistrationData<'_>,
        data: &mut uapi::drm_asahi_queue_destroy,
        file: &DrmFile,
    ) -> Result<u32> {
        // grab the queue so the xarray spinlock is dropped first
        let queue = file.inner().queues().remove(data.queue_id as usize);
        if queue.is_none() {
            Err(ENOENT)
        } else {
            Ok(0)
        }
    }

    /// IOCTL: submit: Submit GPU work to a command submission queue.
    pub(crate) fn submit(
        device: &AsahiDrmDevice,
        reg_data: &driver::AsahiDrmRegistrationData<'_>,
        data: &mut uapi::drm_asahi_submit,
        file: &DrmFile,
    ) -> Result<u32> {
        debug::update_debug_flags();

        if data.flags != 0 || data.pad != 0 {
            cls_pr_debug!(Errors, "submit: Invalid arguments\n");
            return Err(EINVAL);
        }

        let gpu = &reg_data.gpu;
        if data.cmdbuf_size as usize > gpu.max_cmdbuf_size() {
            return Err(E2BIG);
        }
        if let Some(manager) = gpu.manager() {
            manager.update_globals();
        }

        // Upgrade to Arc<T> to drop the XArray lock early
        let queue: Arc<Mutex<TableQueue>> = file
            .inner()
            .queues()
            .lock()
            .get(data.queue_id.try_into()?)
            .ok_or(ENOENT)?
            .into();

        let id = gpu.ids().submission.next();
        mod_dev_dbg!(
            device,
            "[File {} Queue {}]: IOCTL: submit (submission ID: {})\n",
            file.inner().id,
            data.queue_id,
            id
        );

        mod_dev_dbg!(
            device,
            "[File {} Queue {}]: IOCTL: submit({}): Parsing syncs\n",
            file.inner().id,
            data.queue_id,
            id
        );
        let mut vec = KVec::new();
        let copy_commands = |bytes: &mut KVec<u8>| {
            UserSlice::new(
                UserPtr::from_addr(data.cmdbuf as usize),
                data.cmdbuf_size as usize,
            )
            .reader()
            .read_all(bytes, GFP_KERNEL)
        };
        let guarded = gpu.as_any().is::<crate::g17::Gpu>() || gpu.tracked_submission();
        // Once G17 output syncobjs are resolved, its queue must install a fence
        // on every failure. Take the fallible userspace snapshot before that point.
        if guarded {
            copy_commands(&mut vec)?;
        }
        let syncs = SyncItem::parse_array(
            file,
            data.syncs,
            data.in_sync_count,
            data.out_sync_count,
            gpu.as_any().is::<crate::g17::Gpu>(),
        )?;

        mod_dev_dbg!(
            device,
            "[File {} Queue {}]: IOCTL: submit({}): Parsing commands\n",
            file.inner().id,
            data.queue_id,
            id
        );

        // Copy the command buffer into the kernel. Because we need to iterate
        // the command buffer twice, we do this in one big copy_from_user to
        // avoid TOCTOU issues.
        if !guarded {
            copy_commands(&mut vec)?;
        }

        let objects = file.inner().objects();
        let ret = queue.lock().submit(
            reg_data,
            id,
            syncs,
            data.in_sync_count as usize,
            &vec,
            objects,
        );

        if ret.is_ok() {
            if let Some((util_min, duration_ms)) = gpu.syncobj_wait_hint() {
                file.set_syncobj_wait_hint(util_min, duration_ms);
            }
        }

        match ret {
            Err(ERESTARTSYS) => Err(ERESTARTSYS),
            Err(e) => {
                dev_info!(
                    device.as_ref(),
                    "[File {} Queue {}]: IOCTL: submit failed! (submission ID: {} err: {:?})\n",
                    file.inner().id,
                    data.queue_id,
                    id,
                    e
                );
                Err(e)
            }
            Ok(()) => Ok(0),
        }
    }

    /// IOCTL: get_time: Get the current GPU timer value.
    pub(crate) fn get_time(
        _device: &AsahiDrmDevice,
        reg_data: &driver::AsahiDrmRegistrationData<'_>,
        data: &mut uapi::drm_asahi_get_time,
        _file: &DrmFile,
    ) -> Result<u32> {
        if data.flags != 0 {
            cls_pr_debug!(Errors, "get_time: Unexpected flags\n");
            return Err(EINVAL);
        }

        // TODO: Do this on device-init for perf.
        let gpu = &reg_data.gpu;
        let frequency_hz = u64::from(gpu.base_clock_hz());
        let ts_gcd = gcd(frequency_hz, NSEC_PER_SEC as u64);

        let num = (NSEC_PER_SEC as u64) / ts_gcd;
        let den = frequency_hz / ts_gcd;

        let raw: u64;

        // SAFETY: Assembly only loads the timer
        unsafe {
            core::arch::asm!(
                "mrs {x}, CNTPCT_EL0",
                x = out(reg) raw
            );
        }

        data.gpu_timestamp = (raw * num) / den;

        Ok(0)
    }
}

impl Drop for File {
    fn drop(&mut self) {
        mod_pr_debug!("[File {}]: Closing...\n", self.id);
        // The queues drop before the VMs (see the field order), and destroying a queue fails
        // its queued work with ENOENT or ECANCELED. Nobody sees those any more: the client
        // closed its file. Say so before any queue goes, as a VM handle closed first did.
        // SAFETY: `drop` runs on the pinned file in place, and nothing here moves it.
        let this = unsafe { Pin::new_unchecked(&*self) };
        Self::mark_clients_closed(this, |_| true);
    }
}
