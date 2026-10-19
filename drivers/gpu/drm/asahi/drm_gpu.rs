// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Generation-neutral DRM-facing GPU operations.
//!
//! The legacy manager remains unchanged behind [`LegacyDrmGpu`]. M3 uses
//! the same file and ioctl layer through its own adapter without pretending to
//! be an AGX2 firmware manager. The G17P runtime reaches this layer through its
//! [`gpu::Gpu`] implementation, adapted below.

use core::ops::Range;

use kernel::{
    prelude::*,
    sync::{Arc, Mutex},
    uapi,
};

use crate::g15;
use crate::{alloc, file, fw, g17, gem, gpu, mmu, queue};

/// Hardware facts returned through `DRM_ASAHI_GET_PARAMS`.
pub(crate) struct DrmGpuParams {
    pub(crate) gpu_generation: u32,
    pub(crate) gpu_variant: u32,
    pub(crate) gpu_revision: u32,
    pub(crate) chip_id: u32,
    pub(crate) num_dies: u32,
    pub(crate) num_clusters_total: u32,
    pub(crate) num_cores_per_cluster: u32,
    pub(crate) core_masks: [u32; uapi::DRM_ASAHI_MAX_CLUSTERS as usize],
    pub(crate) max_frequency_khz: u32,
    pub(crate) usc_generation: u32,
    pub(crate) gpu_hal_generation: u32,
    pub(crate) max_commands_per_submission: u32,
}

/// Operations needed by the DRM file layer, independent of firmware ABI.
pub(crate) trait DrmGpu: Send + Sync {
    fn init(&self) -> Result;
    fn ids(&self) -> &gpu::SequenceIDs;
    fn is_crashed(&self) -> bool;
    fn supports_vm_status(&self) -> bool { false }
    fn supports_scheduled_queues(&self) -> bool { false }
    /// Permanent submission-domain failure; querying must not reset firmware.
    fn submission_error(&self) -> i32 { 0 }
    fn update_globals(&self);
    fn service_g16_jobs(&self) {}
    fn extra_features(&self) -> u64 { 0 }
    fn params(&self) -> Result<DrmGpuParams>;
    fn user_range(&self) -> Result<Range<u64>>;
    fn unknown_page(&self) -> Result<u64>;
    fn base_clock_hz(&self) -> u32;
    fn new_vm(&self, kernel_range: Range<u64>) -> Result<mmu::Vm>;
    fn new_queue(
        &self,
        vm: mmu::Vm,
        ualloc: Arc<Mutex<alloc::DefaultAllocator>>,
        ualloc_priv: Arc<Mutex<alloc::DefaultAllocator>>,
        priority: u32,
        usc_exec_base: u64,
    ) -> Result<KBox<dyn queue::Queue>>;
    fn map_timestamp_buffer(
        &self,
        bo: gem::ObjectRef,
        range: Range<usize>,
    ) -> Result<mmu::KernelMapping>;

    /// Legacy-only operations reached by objects that cannot exist on M3.
    fn legacy_manager(&self) -> Option<&Arc<dyn gpu::GpuManager>> {
        None
    }
    fn free_context(&self, _data: KBox<fw::types::GpuObject<fw::workqueue::GpuContextData>>) {}
    fn fwctl(&self, _msg: fw::channels::FwCtlMsg) -> Result {
        Err(ENODEV)
    }

    /// Independent-queue limits; only the G17P runtime has them.
    fn queue_limits(&self) -> Option<uapi::drm_asahi_queue_limits> {
        None
    }
    /// Temporary capacity hint and validity in milliseconds for subsequent syncobj waits;
    /// only the G17P runtime gives one.
    fn syncobj_wait_hint(&self) -> Option<(u32, u32)> {
        None
    }
    /// Whether this GPU uses the tracked submission model of `file.rs` (besides G17, which
    /// `file.rs` recognizes by its backend): VM_BIND batches are validated as a whole against
    /// driver-reserved ranges before any change, and SUBMIT copies the command buffer before
    /// resolving syncs because the queue installs a fence in every output sync even on failure.
    fn tracked_submission(&self) -> bool {
        false
    }
    /// Whether SOFT_FAULTS may be advertised when `fault_control` selects soft faults.
    fn soft_faults(&self) -> bool {
        true
    }
    /// Whether new user objects must be populated, cleaned to the point of coherency and pinned
    /// at creation, before any CPU view exists.
    fn prepare_new_objects(&self) -> bool {
        false
    }
    /// The largest VM_BIND batch, if the GPU's UAPI contract bounds it; with a bound, a batch
    /// also needs at least one operation, a non-null pointer and a stride that is a multiple of 8
    /// and at most a page (EINVAL otherwise). `None` keeps the shared rules.
    fn bind_batch_limit(&self) -> Option<u32> {
        None
    }
    /// A client's VM handle was destroyed: a generation that keeps per-VM state beyond the
    /// client's handle may release it now.
    fn vm_closed(&self) {}
}

/// Publish an existing manager without allocating another reference-counted adapter.
pub(crate) enum Backend {
    Legacy(LegacyDrmGpu),
    M3(Arc<dyn DrmGpu>),
    G16(Arc<dyn DrmGpu>),
    G17(Arc<g17::Gpu>),
    /// The G15 generation (T6030, M3 Pro).
    G15(Arc<g15::Gpu>),
}

impl Backend {
    pub(crate) fn gpu(&self) -> &dyn DrmGpu {
        match self {
            Self::Legacy(manager) => manager,
            Self::M3(runtime) | Self::G16(runtime) => &**runtime,
            Self::G17(runtime) => &**runtime,
            Self::G15(gpu) => &**gpu,
        }
    }
}

/// The G15 generation's `gpu::Gpu` operations, as the file layer sees them.
impl DrmGpu for g15::Gpu {
    fn init(&self) -> Result {
        gpu::Gpu::init(self)
    }

    fn ids(&self) -> &gpu::SequenceIDs {
        gpu::Gpu::ids(self)
    }

    fn is_crashed(&self) -> bool {
        gpu::Gpu::is_crashed(self)
    }

    fn supports_vm_status(&self) -> bool {
        gpu::Gpu::supports_vm_status(self)
    }

    /// G15 has no G13/G14 globals.
    fn update_globals(&self) {}

    fn extra_features(&self) -> u64 {
        // The out-of-line bindgen `Default` is not exported to modules; zero-fill instead.
        let mut params: uapi::drm_asahi_params_global = pin_init::zeroed();
        if gpu::Gpu::get_params(self, &mut params).is_err() {
            return 0;
        }
        let mut features = params.features;
        if gpu::Gpu::supports_vm_status(self) {
            features |= uapi::drm_asahi_feature_DRM_ASAHI_FEATURE_VM_STATUS_MIRROR as u64;
        }
        features
    }

    fn params(&self) -> Result<DrmGpuParams> {
        // The out-of-line bindgen `Default` is not exported to modules; zero-fill instead.
        let mut params: uapi::drm_asahi_params_global = pin_init::zeroed();
        gpu::Gpu::get_params(self, &mut params)?;
        let mut core_masks = [0u32; uapi::DRM_ASAHI_MAX_CLUSTERS as usize];
        for (mask, reported) in core_masks.iter_mut().zip(params.core_masks.iter()) {
            *mask = u32::try_from(*reported).map_err(|_| EIO)?;
        }
        Ok(DrmGpuParams {
            gpu_generation: params.gpu_generation,
            gpu_variant: params.gpu_variant,
            gpu_revision: params.gpu_revision,
            chip_id: params.chip_id,
            num_dies: params.num_dies,
            num_clusters_total: params.num_clusters_total,
            num_cores_per_cluster: params.num_cores_per_cluster,
            core_masks,
            max_frequency_khz: params.max_frequency_khz,
            usc_generation: params.usc_generation,
            gpu_hal_generation: params.gpu_hal_generation,
            max_commands_per_submission: file::MAX_COMMANDS_PER_SUBMISSION,
        })
    }

    fn user_range(&self) -> Result<Range<u64>> {
        Ok(gpu::Gpu::user_range(self))
    }

    fn unknown_page(&self) -> Result<u64> {
        Ok(gpu::Gpu::uat_geometry(self).unk_page())
    }

    fn base_clock_hz(&self) -> u32 {
        gpu::Gpu::base_clock_hz(self)
    }

    fn new_vm(&self, kernel_range: Range<u64>) -> Result<mmu::Vm> {
        gpu::Gpu::new_vm(self, kernel_range)
    }

    fn new_queue(
        &self,
        vm: mmu::Vm,
        ualloc: Arc<Mutex<alloc::DefaultAllocator>>,
        ualloc_priv: Arc<Mutex<alloc::DefaultAllocator>>,
        priority: u32,
        usc_exec_base: u64,
    ) -> Result<KBox<dyn queue::Queue>> {
        gpu::Gpu::new_queue(self, vm, ualloc, ualloc_priv, priority, usc_exec_base)
    }

    fn map_timestamp_buffer(
        &self,
        bo: gem::ObjectRef,
        range: Range<usize>,
    ) -> Result<mmu::KernelMapping> {
        gpu::Gpu::map_timestamp_buffer(self, bo, range)
    }

    fn queue_limits(&self) -> Option<uapi::drm_asahi_queue_limits> {
        gpu::Gpu::queue_limits(self)
    }

    fn tracked_submission(&self) -> bool {
        gpu::Gpu::tracked_submission(self)
    }

    fn soft_faults(&self) -> bool {
        gpu::Gpu::soft_faults(self)
    }

    fn prepare_new_objects(&self) -> bool {
        gpu::Gpu::prepare_new_objects(self)
    }

    fn bind_batch_limit(&self) -> Option<u32> {
        gpu::Gpu::bind_batch_limit(self)
    }

    fn vm_closed(&self) {
        g15::vm::closed(self)
    }
}

/// The G17P runtime's `gpu::Gpu` operations, as the file layer sees them.
impl DrmGpu for g17::Gpu {
    fn init(&self) -> Result {
        gpu::Gpu::init(self)
    }

    fn ids(&self) -> &gpu::SequenceIDs {
        gpu::Gpu::ids(self)
    }

    fn is_crashed(&self) -> bool {
        gpu::Gpu::is_crashed(self)
    }

    fn supports_vm_status(&self) -> bool {
        gpu::Gpu::supports_vm_status(self)
    }

    /// G17P has no G13/G14 globals.
    fn update_globals(&self) {}

    fn extra_features(&self) -> u64 {
        // The out-of-line bindgen `Default` is not exported to modules; zero-fill instead.
        let mut params: uapi::drm_asahi_params_global = pin_init::zeroed();
        if gpu::Gpu::get_params(self, &mut params).is_err() {
            return 0;
        }
        let mut features = params.features;
        if gpu::Gpu::supports_vm_status(self) {
            features |= uapi::drm_asahi_feature_DRM_ASAHI_FEATURE_VM_STATUS_MIRROR as u64;
        }
        features
    }

    fn params(&self) -> Result<DrmGpuParams> {
        // The out-of-line bindgen `Default` is not exported to modules; zero-fill instead.
        let mut params: uapi::drm_asahi_params_global = pin_init::zeroed();
        gpu::Gpu::get_params(self, &mut params)?;
        let mut core_masks = [0u32; uapi::DRM_ASAHI_MAX_CLUSTERS as usize];
        for (mask, reported) in core_masks.iter_mut().zip(params.core_masks.iter()) {
            *mask = u32::try_from(*reported).map_err(|_| EIO)?;
        }
        Ok(DrmGpuParams {
            gpu_generation: params.gpu_generation,
            gpu_variant: params.gpu_variant,
            gpu_revision: params.gpu_revision,
            chip_id: params.chip_id,
            num_dies: params.num_dies,
            num_clusters_total: params.num_clusters_total,
            num_cores_per_cluster: params.num_cores_per_cluster,
            core_masks,
            max_frequency_khz: params.max_frequency_khz,
            usc_generation: params.usc_generation,
            gpu_hal_generation: params.gpu_hal_generation,
            max_commands_per_submission: file::MAX_COMMANDS_PER_SUBMISSION,
        })
    }

    fn user_range(&self) -> Result<Range<u64>> {
        Ok(gpu::Gpu::uat_geometry(self).user_usable_range())
    }

    fn unknown_page(&self) -> Result<u64> {
        Ok(gpu::Gpu::uat_geometry(self).unk_page())
    }

    fn base_clock_hz(&self) -> u32 {
        gpu::Gpu::base_clock_hz(self)
    }

    fn new_vm(&self, kernel_range: Range<u64>) -> Result<mmu::Vm> {
        gpu::Gpu::new_vm(self, kernel_range)
    }

    fn new_queue(
        &self,
        vm: mmu::Vm,
        ualloc: Arc<Mutex<alloc::DefaultAllocator>>,
        ualloc_priv: Arc<Mutex<alloc::DefaultAllocator>>,
        priority: u32,
        usc_exec_base: u64,
    ) -> Result<KBox<dyn queue::Queue>> {
        gpu::Gpu::new_queue(self, vm, ualloc, ualloc_priv, priority, usc_exec_base)
    }

    fn map_timestamp_buffer(
        &self,
        bo: gem::ObjectRef,
        range: Range<usize>,
    ) -> Result<mmu::KernelMapping> {
        gpu::Gpu::map_timestamp_buffer(self, bo, range)
    }

    fn queue_limits(&self) -> Option<uapi::drm_asahi_queue_limits> {
        gpu::Gpu::queue_limits(self)
    }

    fn syncobj_wait_hint(&self) -> Option<(u32, u32)> {
        gpu::Gpu::syncobj_wait_hint(self)
    }
}

/// DRM adapter for the existing AGX2 manager implementations.
pub(crate) struct LegacyDrmGpu {
    manager: Arc<dyn gpu::GpuManager>,
}

impl LegacyDrmGpu {
    pub(crate) fn new(manager: Arc<dyn gpu::GpuManager>) -> Self {
        Self { manager }
    }
}

impl DrmGpu for LegacyDrmGpu {
    fn init(&self) -> Result {
        self.manager.init()
    }

    fn ids(&self) -> &gpu::SequenceIDs {
        self.manager.ids()
    }

    fn is_crashed(&self) -> bool {
        self.manager.is_crashed()
    }

    fn update_globals(&self) {
        self.manager.update_globals()
    }

    fn params(&self) -> Result<DrmGpuParams> {
        let cfg = self.manager.get_cfg();
        let dyncfg = self.manager.get_dyncfg();
        let mut core_masks = [0u32; uapi::DRM_ASAHI_MAX_CLUSTERS as usize];
        for (index, mask) in dyncfg.id.core_masks.iter().enumerate() {
            *core_masks.get_mut(index).ok_or(EIO)? = *mask;
        }
        Ok(DrmGpuParams {
            gpu_generation: dyncfg.id.gpu_gen as u32,
            gpu_variant: dyncfg.id.gpu_variant as u32,
            gpu_revision: dyncfg.id.gpu_rev as u32,
            chip_id: cfg.chip_id,
            num_dies: if cfg.gpu_gen == crate::hw::GpuGen::G15 { dyncfg.id.num_dies } else { cfg.num_dies },
            num_clusters_total: dyncfg.id.num_clusters,
            num_cores_per_cluster: dyncfg.id.num_cores,
            core_masks,
            max_frequency_khz: dyncfg.pwr.max_frequency_khz(),
            usc_generation: dyncfg.id.usc_generation,
            gpu_hal_generation: dyncfg.id.gpu_hal_generation as u32,
            max_commands_per_submission: file::MAX_COMMANDS_PER_SUBMISSION,
        })
    }

    fn user_range(&self) -> Result<Range<u64>> {
        mmu::iova_user_usable_range(self.manager.get_cfg())
    }

    fn unknown_page(&self) -> Result<u64> {
        mmu::iova_unk_page(self.manager.get_cfg())
    }

    fn base_clock_hz(&self) -> u32 {
        self.manager.get_cfg().base_clock_hz
    }

    fn new_vm(&self, kernel_range: Range<u64>) -> Result<mmu::Vm> {
        self.manager.new_vm(kernel_range)
    }

    fn new_queue(
        &self,
        vm: mmu::Vm,
        ualloc: Arc<Mutex<alloc::DefaultAllocator>>,
        ualloc_priv: Arc<Mutex<alloc::DefaultAllocator>>,
        priority: u32,
        usc_exec_base: u64,
    ) -> Result<KBox<dyn queue::Queue>> {
        self.manager
            .new_queue(vm, ualloc, ualloc_priv, priority, usc_exec_base)
    }

    fn map_timestamp_buffer(
        &self,
        bo: gem::ObjectRef,
        range: Range<usize>,
    ) -> Result<mmu::KernelMapping> {
        self.manager.map_timestamp_buffer(bo, range)
    }

    fn legacy_manager(&self) -> Option<&Arc<dyn gpu::GpuManager>> {
        Some(&self.manager)
    }

    fn free_context(&self, data: KBox<fw::types::GpuObject<fw::workqueue::GpuContextData>>) {
        self.manager.free_context(data)
    }

    fn fwctl(&self, msg: fw::channels::FwCtlMsg) -> Result {
        self.manager.fwctl(msg)
    }
}
