// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(clippy::unusual_byte_groupings)]

//! File implementation, which represents a single DRM client.
//!
//! This is in charge of managing the resources associated with one GPU client, including an
//! arbitrary number of submission queues and Vm objects, and reporting hardware/driver
//! information to userspace and accepting submissions.

use crate::debug::*;
use crate::driver::AsahiDevice;
use crate::{
    alloc,
    buffer,
    driver,
    gem,
    hw,
    mmu,
    module_parameters,
    queue,
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
use core::ops::Deref;
use core::ops::Range;
use core::ptr::addr_of_mut;
use kernel::bindings;
use kernel::dma_fence::RawDmaFence;
use kernel::drm::gem::BaseObject;
use kernel::error::code::*;
use kernel::new_mutex;
use kernel::prelude::*;
use kernel::sync::{
    Arc,
    Mutex, //
};
use kernel::time::NSEC_PER_SEC;
use kernel::uaccess::{
    UserPtr,
    UserSlice, //
};
use kernel::{
    dma_fence,
    drm,
    uapi,
    xarray, //
};

const DEBUG_CLASS: DebugFlags = DebugFlags::File;

pub(crate) const MAX_COMMANDS_PER_SUBMISSION: u32 = 64;

const LEGACY_VM_USER_RANGE: Range<u64> = mmu::IOVA_USER_BASE..mmu::IOVA_UNK_PAGE;



/// A client instance of an `mmu::Vm` address space.
struct Vm {
    /// Residency reclaim of a G17 VM; None elsewhere.
    shrinker: Option<KBox<mmu::VmShrinker>>,
    ualloc: Arc<Mutex<alloc::DefaultAllocator>>,
    ualloc_priv: Arc<Mutex<alloc::DefaultAllocator>>,
    vm: mmu::Vm,
    kernel_range: Range<u64>,
    user_range: Range<u64>,
    _dummy_mapping: mmu::KernelMapping,
}

impl Drop for Vm {
    fn drop(&mut self) {
        if self.vm.status().is_some() {
            // G17 VM.
            self.shrinker = None;
            self.vm.close_idle_reclaim();
            if let Some(status) = self.vm.status() {
                // Retained graphs may keep this status alive; the client's mirror
                // object must not outlive the client's VM handle.
                status.release_mirror();
            }
            // Tracked jobs can outlive their file's VM handle. Their last guard performs
            // these unmaps once no accepted work can still dereference the mappings.
            if let Err(error) = self
                .vm
                .unmap_user_ranges(self.user_range.clone(), self.kernel_range.clone())
            {
                pr_err!("Vm::Drop: deferred user unmap failed: {:?}\n", error);
            }
            self.vm.bo_deferred_cleanup();
            return;
        }

        if !self.vm.is_m3() {

        // When the user Vm is dropped, unmap everything in the user range
        let left_range = self.user_range.start..self.kernel_range.start;
        let right_range = self.kernel_range.end..self.user_range.end;

        if !left_range.is_empty()
            && self
                .vm
                .unmap_range(left_range.start, left_range.range())
                .is_err()
        {
            pr_err!("Vm::Drop: vm.unmap_range() failed\n");
        }
        if !right_range.is_empty()
            && self
                .vm
                .unmap_range(right_range.start, right_range.range())
                .is_err()
        {
            pr_err!("Vm::Drop: vm.unmap_range() failed\n");
        }

        self.vm.bo_deferred_cleanup();
                return;
        }

        if self
            .vm
            .unmap_user_ranges(self.user_range.clone(), self.kernel_range.clone())
            .is_err()
        {
            pr_err!("Vm::Drop: vm.unmap_user_ranges() failed\n");
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
}

impl SyncItem {
    fn parse_one(file: &DrmFile, data: uapi::drm_asahi_sync, out: bool) -> Result<SyncItem> {
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
                    chain_fence: if out {
                        Some(dma_fence::FenceChain::new()?)
                    } else {
                        None
                    },
                    timeline_value: data.timeline_value,
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
    ) -> Result<KVec<SyncItem>> {
        let count = in_count.checked_add(out_count).ok_or(EOVERFLOW)?;

        const STRIDE: usize = core::mem::size_of::<uapi::drm_asahi_sync>();
        let size = STRIDE.checked_mul(count as usize).ok_or(EOVERFLOW)?;
        let mut vec = KVec::with_capacity(count as usize, GFP_KERNEL)?;

        // SAFETY: We only read this once, so there are no TOCTOU issues.
        let mut reader = UserSlice::new(UserPtr::from_addr(ptr as _), size).reader();

        for i in 0..count {
            let mut sync: MaybeUninit<uapi::drm_asahi_sync> = MaybeUninit::uninit();

            // SAFETY: The size of `sync` is STRIDE
            reader.read_raw(unsafe {
                core::slice::from_raw_parts_mut(sync.as_mut_ptr() as *mut MaybeUninit<u8>, STRIDE)
            })?;

            // SAFETY: All bit patterns in the struct are valid
            let sync = unsafe { sync.assume_init() };

            vec.push(SyncItem::parse_one(file, sync, i >= in_count)?, GFP_KERNEL)?;
        }

        Ok(vec)
    }
}

#[derive(Clone)]
pub(crate) enum Object {
    TimestampBuffer(Arc<mmu::KernelMapping>),
}

/// State associated with a client.
// #[pin_data]
pub(crate) struct File {
    id: u64,
    // #[pin]
    vms: xarray::XArray<KBox<Vm>>,
    // #[pin]
    queues: xarray::XArray<Arc<Mutex<KBox<dyn queue::Queue>>>>,
    // #[pin]
    objects: xarray::XArray<KBox<Object>>,
    m3_client: Option<crate::m3_client::ClientGate>,
}

/// Convenience type alias for our DRM `File` type.
pub(crate) type DrmFile = drm::File<File>;

// The DRM adapter supplies the per-device user range and reserved dummy page.

/// Minimum reserved AS for kernel mappings
const VM_KERNEL_MIN_SIZE: u64 = 0x20000000;

impl drm::file::DriverFile for File {
    type Driver = driver::AsahiDriver;

    /// Create a new `File` instance for a fresh client.
    fn open(device: &AsahiDevice) -> Result<Pin<KBox<Self>>> {
        debug::update_debug_flags();

        let gpu = device.gpu()?;
        let id = gpu.ids().file.next();

        mod_dev_dbg!(device, "[File {}]: DRM device opened\n", id);
        Ok(KBox::pin_init(File::new(id, device.is_m3), GFP_KERNEL)?)
    }

    fn as_raw(&self) -> *mut bindings::drm_file {
        todo!()
    }
}

// SAFETY: All bit patterns are valid by construction.
unsafe impl AnyBitPattern for uapi::drm_asahi_gem_bind_op {}

impl File {
    fn new(id: u64, is_m3: bool) -> impl PinInit<Self, Error> {
        unsafe {
            pin_init::pin_init_from_closure(move |slot: *mut Self| {
                let raw_vms = addr_of_mut!((*slot).vms);
                xarray::XArray::<KBox<Vm>>::new(xarray::AllocKind::Alloc1)
                    .__pinned_init(raw_vms)?;

                let raw_queues = addr_of_mut!((*slot).queues);
                xarray::XArray::<Arc<Mutex<KBox<dyn queue::Queue>>>>::new(
                    xarray::AllocKind::Alloc1,
                )
                .__pinned_init(raw_queues)?;

                let raw_objects = addr_of_mut!((*slot).objects);
                xarray::XArray::<KBox<Object>>::new(xarray::AllocKind::Alloc1)
                    .__pinned_init(raw_objects)?;

                addr_of_mut!((*slot).m3_client).write(if is_m3 { Some(crate::m3_client::ClientGate::new()) } else { None });
                (*slot).id = id;
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
    fn queues(self: Pin<&Self>) -> Pin<&xarray::XArray<Arc<Mutex<KBox<dyn queue::Queue>>>>> {
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
        device: &AsahiDevice,
        data: &uapi::drm_asahi_get_params,
        file: &DrmFile,
    ) -> Result<u32> {
        mod_dev_dbg!(device, "[File {}]: IOCTL: get_params\n", file.inner().id);

        let gpu = device.gpu()?;

        match data.param_group {
            crate::agx_status::PARAM_GROUP_VM_STATUS => {
                return Self::get_vm_status(device, data, file);
            }
            uapi::DRM_ASAHI_PARAM_GROUP_VM_STATUS_MIRROR
                if device.g17().is_some() || gpu.tracked_submission() =>
            {
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
                    UserPtr::from_addr(data.pointer as _),
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

        let facts = gpu.params()?;
        let mut params = uapi::drm_asahi_params_global {
            features: gpu.extra_features(),

            gpu_generation: facts.gpu_generation,
            gpu_variant: facts.gpu_variant,
            gpu_revision: facts.gpu_revision,
            chip_id: facts.chip_id,

            num_dies: facts.num_dies,
            num_clusters_total: facts.num_clusters_total,
            num_cores_per_cluster: facts.num_cores_per_cluster,
            core_masks: facts.core_masks.map(|mask| mask.into()),

            vm_start: gpu.user_range()?.start,
            vm_end: gpu.user_range()?.end,
            vm_kernel_min_size: VM_KERNEL_MIN_SIZE,

            max_commands_per_submission: facts.max_commands_per_submission,
            max_attachments: crate::microseq::MAX_ATTACHMENTS as u32,
            max_frequency_khz: facts.max_frequency_khz,

            command_timestamp_frequency_hz: 1_000_000_000, // User timestamps always in nanoseconds

            usc_generation: facts.usc_generation,
            gpu_hal_generation: facts.gpu_hal_generation,
        };

        if gpu.supports_vm_status() {
            params.features |= crate::agx_status::FEATURE_VM_STATUS;
        }

        if gpu.queue_limits().is_some() {
            params.features |= uapi::drm_asahi_feature_DRM_ASAHI_FEATURE_INDEPENDENT_QUEUES as u64;
        }

        if gpu.supports_scheduled_queues() {
            params.features |= uapi::drm_asahi_feature_DRM_ASAHI_FEATURE_SCHEDULED_QUEUES as u64;
        }
        // The M3 runtime's optional features are per SoC (`m3_soc::Soc::features`); the G15
        // generation reports its own feature word on the same chips.
        let m3 = (facts.gpu_hal_generation == hw::GpuHalGeneration::Legacy as u32
            && device.g15().is_none())
            .then(|| crate::m3_soc::by_chip(facts.chip_id)).flatten();
        if m3.is_some_and(|soc| soc.features.fragment_dependency)
            && *module_parameters::m3_early_tiling.value()!=0
        {
            params.features |= uapi::drm_asahi_feature_DRM_ASAHI_FEATURE_FRAGMENT_DEPENDENCY as u64;
        }
        if m3.is_some_and(|soc| soc.features.compute_wide_visibility) {
            params.features |= uapi::drm_asahi_feature_DRM_ASAHI_FEATURE_COMPUTE_WIDE_VISIBILITY as u64;
        }
        // The owned M3/M4 runtimes do not apply the legacy InitData fault_control
        // parameter. Do not invite speculative invalid accesses until its
        // firmware/MMU soft-fault configuration is implemented and qualified:
        // not on any SoC with an M3 runtime table, nor on the M3 Max or M4.
        if facts.gpu_hal_generation == hw::GpuHalGeneration::Legacy as u32 && m3.is_none()
            && !matches!(facts.chip_id, 0x6031 | 0x6034 | 0x8132)
            && gpu.soft_faults()
            && *module_parameters::fault_control.value() == 0xb
        {
            params.features |= uapi::drm_asahi_feature_DRM_ASAHI_FEATURE_SOFT_FAULTS as u64;
        }
        // The G17P runtime keeps its own soft-fault rule.
        if device.g17().is_some() && *module_parameters::fault_control.value() == 0xb {
            params.features |= uapi::drm_asahi_feature_DRM_ASAHI_FEATURE_SOFT_FAULTS as u64;
        }

        let size = core::mem::size_of::<uapi::drm_asahi_params_global>().min(data.size.try_into()?);

        // SAFETY: We only write to this userptr once, so there are no TOCTOU issues.
        let mut params_writer =
            UserSlice::new(UserPtr::from_addr(data.pointer as _), size).writer();

        // SAFETY: `size` is at most the sizeof of `params`
        params_writer.write_slice(unsafe {
            core::slice::from_raw_parts(&params as *const _ as *const u8, size)
        })?;

        if let Some(gate) = &file.inner().m3_client { gate.note_params(data.size); }
        Ok(0)
    }

    /// Status queries never enter firmware recovery or wait for a GPU job.
    /// File-local lookup enforces VM ownership. Status survives queue teardown.
    fn get_vm_status(device: &AsahiDevice, data: &uapi::drm_asahi_get_params,
                     file: &DrmFile) -> Result<u32> {
        use crate::agx_status::{decode_request, encode_response, VM_STATUS_SIZE};
        if !device.gpu()?.supports_vm_status() { return Err(EINVAL); }
        if data.pad != 0 || data.size < VM_STATUS_SIZE as u64 { return Err(EINVAL); }
        let user = UserPtr::from_addr(data.pointer as _);
        let mut bytes = [0u8; VM_STATUS_SIZE];
        UserSlice::new(user, VM_STATUS_SIZE).reader().read_slice(&mut bytes)?;
        let vm_id = decode_request(&bytes).ok_or(EINVAL)?;
        let (g17_status, status) = {
            let vms = file.inner().vms();
            let guard = vms.lock();
            let vm = guard.get(vm_id as usize).ok_or(ENOENT)?;
            match vm.vm.status() {
                // G17 VMs keep their own sticky status object.
                Some(status) => (Some(status.clone()), None),
                None => (None, Some(vm.vm.agx_status()?.clone())),
            }
        };
        let error = match (g17_status, status) {
            (Some(status), _) => status.reported_error(),
            (None, Some(status)) => {
                status.record(device.gpu()?.submission_error());
                status.get()
            }
            (None, None) => return Err(EINVAL),
        };
        let bytes = encode_response(vm_id, error);
        UserSlice::new(user, VM_STATUS_SIZE).writer().write_slice(&bytes)?;
        Ok(0)
    }

    /// Register a file-owned GEM word as the mirror of a file-owned VM's status.
    fn register_vm_status_mirror(data: &uapi::drm_asahi_get_params, file: &DrmFile) -> Result<u32> {
        let size = core::mem::size_of::<uapi::drm_asahi_vm_status_mirror>();
        if data.pad != 0 || data.size < size as u64 {
            return Err(EINVAL);
        }
        let [vm_id, handle, offset, flags] =
            UserSlice::new(UserPtr::from_addr(data.pointer as _), size)
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
        device: &AsahiDevice,
        data: &mut uapi::drm_asahi_vm_create,
        file: &DrmFile,
    ) -> Result<u32> {
        if device.g17().is_some() {
            // G17P reports HAL200; it accepts the acknowledgement but does not require it.
            if data.flags & !uapi::drm_asahi_vm_create_flags_DRM_ASAHI_VM_CREATE_HAL200 != 0 {
                return Err(EINVAL);
            }
        } else {
            let hal = device.gpu()?.params()?.gpu_hal_generation;
            let expected = if hal == 200 { uapi::drm_asahi_vm_create_flags_DRM_ASAHI_VM_CREATE_HAL200 } else { 0 };
            if data.flags != expected { return Err(EINVAL); }
        }
        if let Some(gate) = &file.inner().m3_client { gate.admit_vm(device, data.flags)?; }
        let kernel_range = data.kernel_start..data.kernel_end;
        let user_range = device.gpu()?.user_range()?;

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

        let gpu = device.gpu()?;
        let file_id = file.inner().id;
        let vm = gpu.new_vm(kernel_range.clone())?;

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
        dummy_obj.vmap()?.memset(0);
        let dummy_mapping = dummy_obj.map_at(
            &vm,
            device.gpu()?.unknown_page()?,
            mmu::PROT_GPU_SHARED_RW,
            true,
        )?;

        mod_dev_dbg!(device, "[File {} VM {}]: VM created\n", file_id, id);
        let profile_vm_id = vm.id();
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
                kernel_range,
                user_range,
                _dummy_mapping: dummy_mapping,
            },
            GFP_KERNEL,
        )?)?;

        data.vm_id = id;
        if crate::debug::debug_enabled(crate::debug::DebugFlags::M3PassTiming) {
            dev_info!(device, "M3_VM_PROFILE pid={} file={} handle={} vm={}\n",
                kernel::current!().group_leader().pid(), file_id, id, profile_vm_id);
        }

        Ok(0)
    }

    /// IOCTL: vm_destroy: Destroy a `Vm`.
    pub(crate) fn vm_destroy(
        device: &AsahiDevice,
        data: &mut uapi::drm_asahi_vm_destroy,
        file: &DrmFile,
    ) -> Result<u32> {
        let vm = file.inner().vms().remove(data.vm_id as usize);
        if vm.is_none() {
            return Err(ENOENT);
        }
        drop(vm);
        // The generation may release per-VM state now that the client's handle is gone.
        device.gpu()?.vm_closed();
        Ok(0)
    }

    /// IOCTL: gem_create: Create a new GEM object.
    pub(crate) fn gem_create(
        device: &AsahiDevice,
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
            // Only G17 VMs enable residency reclaim.
            reclaim_cpu = vm.residency_reclaim_enabled();
            resv_gem = vm.get_resv_obj();
            Some(resv_gem.deref())
        } else {
            None
        };

        let gem = gem::new_object(device, data.size.try_into()?, data.flags, resv_obj, reclaim_cpu)?;
        if device.gpu()?.prepare_new_objects() {
            gem::prepare_new_object(&gem)?;
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

    /// IOCTL: gem_mmap_offset: Assign an mmap offset to a GEM object.
    pub(crate) fn gem_mmap_offset(
        device: &AsahiDevice,
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
        data.offset = gem.create_mmap_offset()?;
        Ok(0)
    }

    /// IOCTL: gem_madvise: Mark a VM-private object purgeable or needed.
    pub(crate) fn gem_madvise(
        device: &AsahiDevice,
        data: &mut uapi::drm_asahi_gem_madvise,
        file: &DrmFile,
    ) -> Result<u32> {
        // Only the G17P runtime implements purgeable objects.
        if device.g17().is_none() {
            return Err(EINVAL);
        }
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
        device: &AsahiDevice,
        data: &uapi::drm_asahi_vm_bind,
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

        let gpu = device.gpu()?;
        if let Some(limit) = gpu.bind_batch_limit() {
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

        if device.g17().is_some() || gpu.tracked_submission() {
            return Self::vm_bind_g17(data, file);
        }

        let vm_id = data.vm_id.try_into()?;

        let mut vec = KVec::new();
        let size = (data.stride * data.num_binds) as usize;
        let reader = UserSlice::new(UserPtr::from_addr(data.userptr as _), size).reader();
        reader.read_all(&mut vec, GFP_KERNEL)?;
        let mut reader = Reader::new(&vec);

        for _i in 0..data.num_binds {
            let bind: uapi::drm_asahi_gem_bind_op = reader.read_up_to(data.stride as usize)?;
            // Preserve the bind diagnostic without formatting every normal
            // desktop allocation into the kernel journal.
            cls_pr_debug!(Mmu,
                "GPU bind: vm={} addr={:#x} range={:#x} offset={:#x} flags={:#x} handle={}\n",
                data.vm_id,
                bind.addr,
                bind.range,
                bind.offset,
                bind.flags,
                bind.handle,
            );
            Self::do_gem_bind_unbind(vm_id, &bind, file)?;
        }

        Ok(0)
    }

    /// Validate all operations and retain their objects before changing a tracked VM.
    fn vm_bind_g17(data: &uapi::drm_asahi_vm_bind, file: &DrmFile) -> Result<u32> {
        let count = usize::try_from(data.num_binds)?;
        let stride = usize::try_from(data.stride)?;
        let size = stride.checked_mul(count).ok_or(EOVERFLOW)?;
        let mut bytes = KVec::new();
        UserSlice::new(UserPtr::from_addr(data.userptr as _), size)
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
                    | uapi::drm_asahi_bind_flags_DRM_ASAHI_BIND_SINGLE_PAGE)
                != 0
            {
                return Err(EINVAL);
            }
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
            )?)?;
        }
        vm.reserve_deferred_user_unmaps(unmaps)?;
        vm.commit_prepared_user_bind_batch(&mut batch)?;
        vm.bo_deferred_cleanup();
        Ok(0)
    }

    pub(crate) fn do_gem_bind_unbind(
        vm_id: usize,
        data: &uapi::drm_asahi_gem_bind_op,
        file: &DrmFile,
    ) -> Result<u32> {
        if (data.flags & uapi::drm_asahi_bind_flags_DRM_ASAHI_BIND_UNBIND) != 0 {
            Self::do_gem_unbind(vm_id, data, file)
        } else {
            Self::do_gem_bind(vm_id, data, file)
        }
    }

    pub(crate) fn do_gem_bind(
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

        if file.inner().m3_client.is_none()
            && !LEGACY_VM_USER_RANGE.is_superset(range.clone())
        {
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
        let user_range = guarded_vm.user_range.clone();
        let _ = guarded_vm;
        core::mem::drop(guard);

        if file.inner().m3_client.is_some() && !user_range.is_superset(range.clone()) {
            cls_pr_debug!(
                Errors,
                "gem_bind: Invalid map range {:#x}..{:#x} (not contained in user range)\n",
                start,
                end
            );
            return Err(EINVAL); // Invalid map range
        }

        if kernel_range.overlaps(range.clone()) {
            cls_pr_debug!(
                Errors,
                "gem_bind: Invalid map range {:#x}..{:#x} (intrudes in kernel range)\n",
                start,
                end
            );
            return Err(EINVAL);
        }
        if vm.driver_range_overlaps(range.clone()) {
            cls_pr_debug!(
                Errors,
                "gem_bind: map range {:#x}..{:#x} overlaps a device-owned VA\n",
                start,
                end
            );
            return Err(EINVAL);
        }

        vm.bind_object(&bo, data.addr, data.range, data.offset, prot, single_page)?;
        vm.bo_deferred_cleanup();

        Ok(0)
    }

    pub(crate) fn do_gem_unbind(
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

        if file.inner().m3_client.is_none()
            && !LEGACY_VM_USER_RANGE.is_superset(range.clone())
        {
            cls_pr_debug!(
                Errors,
                "gem_bind: Invalid unmap range {:#x}..{:#x} (not contained in user range)\n",
                start,
                end
            );
            return Err(EINVAL); // Invalid unmap range
        }

        let vms_xa = file.inner().vms();
        let guard = vms_xa.lock();
        let guarded_vm = guard.get(vm_id).ok_or(ENOENT)?;

        // Clone it immediately so we aren't holding the XArray lock
        let vm = guarded_vm.vm.clone();
        let kernel_range = guarded_vm.kernel_range.clone();
        let user_range = guarded_vm.user_range.clone();
        let _ = guarded_vm;
        core::mem::drop(guard);

        if file.inner().m3_client.is_some() && !user_range.is_superset(range.clone()) {
            cls_pr_debug!(
                Errors,
                "gem_bind: Invalid unmap range {:#x}..{:#x} (not contained in user range)\n",
                start,
                end
            );
            return Err(EINVAL); // Invalid map range
        }

        if kernel_range.overlaps(range.clone()) {
            cls_pr_debug!(
                Errors,
                "gem_bind: Invalid unmap range {:#x}..{:#x} (intrudes in kernel range)\n",
                start,
                end
            );
            return Err(EINVAL);
        }
        if vm.driver_range_overlaps(range.clone()) {
            cls_pr_debug!(
                Errors,
                "gem_unbind: range {:#x}..{:#x} overlaps a device-owned VA\n",
                start,
                end
            );
            return Err(EINVAL);
        }

        vm.unmap_range(range.start, range.range())?;


        vm.bo_deferred_cleanup();

        Ok(0)
    }

    pub(crate) fn unbind_gem_object(file: &DrmFile, bo: &gem::Object) -> Result {
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
        device: &AsahiDevice,
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
                Self::do_gem_bind_object(device, data, file)
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
        device: &AsahiDevice,
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
            device.gpu()?.map_timestamp_buffer(bo, offset..end_offset)?,
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
        _device: &AsahiDevice,
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
        device: &AsahiDevice,
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

        if data.flags != 0 || data.priority > uapi::drm_asahi_priority_DRM_ASAHI_PRIORITY_REALTIME {
            cls_pr_debug!(Errors, "queue_create: Invalid arguments\n");
            return Err(EINVAL);
        }

        // TODO: Allow with CAP_SYS_NICE
        if data.priority >= uapi::drm_asahi_priority_DRM_ASAHI_PRIORITY_HIGH {
            cls_pr_debug!(Errors, "queue_create: Invalid priority\n");
            return Err(EINVAL);
        }

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

        let mut queue = device.gpu()?.new_queue(
            vm,
            ualloc,
            ualloc_priv,
            // TODO: Plumb deeper the enum
            uapi::drm_asahi_priority_DRM_ASAHI_PRIORITY_REALTIME - data.priority,
            data.usc_exec_base,
        )?;

        // Only G17P queues accept a nonzero render ray query scratch layout.
        queue.reserve_render_scs(data.render_scs_layout)?;

        data.queue_id = resv.index().try_into()?;
        resv.fill(Arc::pin_init(new_mutex!(queue), GFP_KERNEL)?)?;

        Ok(0)
    }

    /// IOCTL: queue_destroy: Destroy a command submission queue.
    pub(crate) fn queue_destroy(
        _device: &AsahiDevice,
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
        device: &AsahiDevice,
        data: &mut uapi::drm_asahi_submit,
        file: &DrmFile,
    ) -> Result<u32> {
        debug::update_debug_flags();

        if data.flags != 0 || data.pad != 0 {
            cls_pr_debug!(Errors, "submit: Invalid arguments\n");
            return Err(EINVAL);
        }

        let gpu = device.gpu()?;
        gpu.update_globals();

        // Upgrade to Arc<T> to drop the XArray lock early
        let queue: Arc<Mutex<KBox<dyn queue::Queue>>> = file
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
                UserPtr::from_addr(data.cmdbuf as _),
                data.cmdbuf_size as usize,
            )
            .reader()
            .read_all(bytes, GFP_KERNEL)
        };
        let guarded = device.g17().is_some() || gpu.tracked_submission();
        // Once G17 output syncobjs are resolved, its queue must install a fence
        // on every failure. Take the fallible userspace snapshot before that point.
        if guarded {
            copy_commands(&mut vec)?;
        }
        let syncs =
            SyncItem::parse_array(file, data.syncs, data.in_sync_count, data.out_sync_count)?;

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
        let ret = queue
            .lock()
            .submit(id, syncs, data.in_sync_count as usize, &vec, objects);

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
        device: &AsahiDevice,
        data: &mut uapi::drm_asahi_get_time,
        _file: &DrmFile,
    ) -> Result<u32> {
        if data.flags != 0 {
            cls_pr_debug!(Errors, "get_time: Unexpected flags\n");
            return Err(EINVAL);
        }

        // TODO: Do this on device-init for perf.
        let gpu = device.gpu()?;
        let frequency_hz = gpu.base_clock_hz() as u64;
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
    }
}
