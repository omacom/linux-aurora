// SPDX-License-Identifier: GPL-2.0-only OR MIT
//! M3 through the common Asahi GEM, GPUVM and render-node frontend.
use core::ops::Range;
use kernel::{device::Core, drm, new_mutex, platform, prelude::*, sync::{Arc, Mutex}};
use crate::{alloc, driver, drm_gpu::{DrmGpu, DrmGpuParams}, gem, gpu, hw, mmu, queue};

pub(crate) type Shared = Arc<Mutex<Option<crate::m3_runtime::Runtime>>>;
struct ProgressView(Arc<crate::m3_rtkit::Health>);
impl kernel::debugfs::Writer for ProgressView {
    fn write(&self, f: &mut kernel::fmt::Formatter<'_>) -> kernel::fmt::Result {
        let (generation, completed, last_ns, healthy) = self.0.progress_snapshot();
        writeln!(f, "version=1 generation_ns={} completed={} last_completion_ns={} healthy={}",
            generation, completed, last_ns, u8::from(healthy))
    }
}
// Separate endpoint preserves the strict completion-v1 ABI for old guards.
struct ActivityView(Arc<crate::m3_rtkit::Health>);
impl kernel::debugfs::Writer for ActivityView {
    fn write(&self, f: &mut kernel::fmt::Formatter<'_>) -> kernel::fmt::Result {
        let (generation, epoch, healthy) = self.0.activity_snapshot();
        writeln!(f, "version=1 generation_ns={} epoch={} pending={} healthy={}",
            generation, epoch, epoch & 1, u8::from(healthy))
    }
}

struct TimingView(Arc<crate::m3_rtkit::Health>);
impl kernel::debugfs::Writer for TimingView {
    fn write(&self, f: &mut kernel::fmt::Formatter<'_>) -> kernel::fmt::Result {
        let (generation, _, _, healthy) = self.0.progress_snapshot();
        self.0.timing.write(f, generation, healthy)
    }
}

// Root-only writable debugfs policy; no mutable module-parameter references.
// Keep ownership independent of the runtime lock and remove the file on stop.
struct CompletionView(Arc<crate::m3_rtkit::EventWait>);
impl kernel::debugfs::Writer for CompletionView {
    fn write(&self, f: &mut kernel::fmt::Formatter<'_>) -> kernel::fmt::Result {
        let (us, irqs, sleeps, timers, preparations) = self.0.snapshot();
        writeln!(f, "version=1 wait_us={} notifications={} sleeps={} timer_wakes={} cpu_preparations={}",
                 us, irqs, sleeps, timers, preparations)
    }
}
impl kernel::debugfs::Reader for CompletionView {
    fn read_from_slice(&self, reader: &mut kernel::uaccess::UserSliceReader) -> Result {
        let len = reader.len();
        if len == 0 || len > 8 { return Err(EINVAL); }
        let mut bytes = [0u8; 8];
        reader.read_slice(&mut bytes[..len])?;
        let text = core::str::from_utf8(&bytes[..len]).map_err(|_| EINVAL)?;
        let us = text.trim().parse::<u64>().map_err(|_| EINVAL)?;
        self.0.set_wait_us(us)
    }
}

struct SubmitOverlapView(Arc<crate::m3_rtkit::EventWait>);
impl kernel::debugfs::Writer for SubmitOverlapView {
    fn write(&self, f: &mut kernel::fmt::Formatter<'_>) -> kernel::fmt::Result {
        writeln!(f, "{}", u8::from(self.0.cpu_overlap()))
    }
}
impl kernel::debugfs::Reader for SubmitOverlapView {
    fn read_from_slice(&self, reader: &mut kernel::uaccess::UserSliceReader) -> Result {
        let len=reader.len();
        if len==0 || len>2 {return Err(EINVAL);}
        let mut bytes=[0u8;2];reader.read_slice(&mut bytes[..len])?;
        let text=core::str::from_utf8(&bytes[..len]).map_err(|_|EINVAL)?.trim();
        match text {
            "0" => self.0.set_cpu_overlap(false),
            "1" => self.0.set_cpu_overlap(true),
            _ => return Err(EINVAL),
        }
        Ok(())
    }
}

pub(crate) struct Registered {
    registration: Pin<KBox<Mutex<Option<drm::driver::Registration<driver::AsahiDriver>>>>>,
    shared: Shared,
    health: Arc<crate::m3_rtkit::Health>,
    _progress: Pin<KBox<kernel::debugfs::File<ProgressView>>>,
    _activity: Pin<KBox<kernel::debugfs::File<ActivityView>>>,
    _memory: Pin<KBox<kernel::debugfs::File<crate::agx_memory_stats::View>>>,
    _timing: Pin<KBox<kernel::debugfs::File<TimingView>>>,
    _completion: Pin<KBox<kernel::debugfs::File<CompletionView>>>,
    _overlap: Pin<KBox<kernel::debugfs::File<SubmitOverlapView>>>,
}
impl Registered {
    pub(crate) fn start(pdev: &platform::Device<Core>, soc: &'static crate::m3_soc::Soc) -> Result<Self> {
        let resources = crate::m3_resources::from_device(pdev, soc).inspect_err(|_| {
            crate::t8122_start::not_admitted(pdev.as_ref(), soc);
        })?;
        dev_info!(pdev.as_ref(), "M3: resource admission complete\n");
        // An armed T8122 start supplies the values its table leaves out. Any other SoC, or an
        // unarmed T8122, needs a complete table.
        let experiment = crate::t8122_start::arm(pdev.as_ref(), soc)?;
        if experiment.is_none() {
            soc.require_complete(pdev.as_ref())?;
        }
        let experiment = experiment.as_ref();
        let refused = |stage: &'static str| {
            move |e: &Error| {
                crate::t8122_start::refused(pdev.as_ref(), experiment, stage, *e);
            }
        };
        let firmware = crate::m3_firmware::identify_loaded(pdev, soc, resources, experiment)
            .inspect_err(refused("the firmware identity"))?;
        dev_info!(pdev.as_ref(), "M3: firmware identity accepted\n");
        // InitData source and contents, before any GPU register access.
        let contents = crate::m3_adt_config::Contents::select(pdev, &firmware, soc, experiment)
            .inspect_err(refused("the InitData contents"))?;
        let device = crate::m3_device::Device::new(pdev, firmware, soc, experiment)
            .inspect_err(refused("the GPU registers and identity"))?;
        device.check_drm(pdev).inspect_err(refused("the DRM setup"))?;
        let core_mask = device.core_mask();
        let max_frequency_khz = 1000 * contents.pstates.reported_max_mhz();
        let mut runtime = crate::m3_runtime::Runtime::new(pdev, device, contents)?;
        runtime.boot(pdev)?;
        dev_info!(pdev.as_ref(), "M3: runtime switches: asahi.m3_unlocked_wait={} asahi.m3_timeout_nohang={} resume-after-fault={}\n",
            u8::from(crate::m3_params::unlocked_wait()), u8::from(crate::m3_params::timeout_nohang()),
            u8::from(crate::m3_params::g15_debug(crate::m3_params::G15Debug::M3ResumeAfterFault)));
        let drm = runtime.drm();
        let health = runtime.health();
        // Health holds no device references. The read-only file owns an Arc
        // and is removed with registration; teardown latches failure first.
        let directory = kernel::debugfs::Dir::new(c"asahi-m3");
        let progress = KBox::pin_init(directory.read_only_file(c"progress",
            ProgressView(health.clone())), GFP_KERNEL)?;
        let activity = KBox::pin_init(directory.read_only_file(c"activity",
            ActivityView(health.clone())), GFP_KERNEL)?;
        let memory = KBox::pin_init(directory.read_only_file(c"memory",
            crate::agx_memory_stats::View), GFP_KERNEL)?;
        let timing = KBox::pin_init(directory.read_only_file(c"timing",
            TimingView(health.clone())), GFP_KERNEL)?;
        let completion = KBox::pin_init(directory.read_write_file(c"completion_wait",
            CompletionView(runtime.completion_wait())), GFP_KERNEL)?;
        let overlap = KBox::pin_init(directory.read_write_file(c"submit_overlap",
            SubmitOverlapView(runtime.completion_wait())), GFP_KERNEL)?;
        let shared = Arc::pin_init(new_mutex!(Some(runtime)), GFP_KERNEL)?;
        if !drm.completion.populate(crate::m3_completion::Completion::new(shared.clone())?) {
            return Err(EBUSY);
        }
        let owner = Self { registration: KBox::pin_init(new_mutex!(None), GFP_KERNEL)?, shared: shared.clone(), health: health.clone(), _progress: progress, _activity: activity, _memory: memory, _timing: timing, _completion: completion, _overlap: overlap };
        let scheduler=Arc::new(drm::sched::Scheduler::new(drm.as_ref(),4,8,0,3000,kernel::c_str!("asahi_m3_sched"))?,GFP_KERNEL)?;
        let backend: Arc<dyn DrmGpu> = Arc::new(Backend { shared, health, scheduler, ids: gpu::SequenceIDs::default(),
            core_mask, max_frequency_khz, soc }, GFP_KERNEL)?;
        if !drm.gpu.populate(crate::drm_gpu::Backend::M3(backend)) { return Err(EBUSY); }
        if crate::m3_board::expose_render_node(soc) {
            *owner.registration.lock() = Some(drm::driver::Registration::new(&drm, 0)?);
            dev_info!(pdev.as_ref(), "M3: firmware ready; common DRM GEM/VM frontend registered\n");
        } else {
            dev_info!(pdev.as_ref(), "M3: firmware ready; render node not registered on this board (asahi.m3_expose=1 registers it)\n");
        }
        Ok(owner)
    }
    pub(crate) fn stop(&self) {
        self.health.mark_failed();
        drop(self.registration.lock().take());
        drop(self.shared.lock().take());
    }
}
impl Drop for Registered { fn drop(&mut self) { self.stop(); } }
struct Backend { shared: Shared, health: Arc<crate::m3_rtkit::Health>, ids: gpu::SequenceIDs,
    scheduler:Arc<drm::sched::Scheduler<crate::m3_submit::Job>>, core_mask: u64, max_frequency_khz: u32,
    soc: &'static crate::m3_soc::Soc }
impl DrmGpu for Backend {
    fn init(&self) -> Result { if self.is_crashed() { Err(ENODEV) } else { Ok(()) } }
    fn ids(&self) -> &gpu::SequenceIDs { &self.ids }
    fn is_crashed(&self) -> bool { !self.health.healthy() }
    fn update_globals(&self) {}

    fn supports_vm_status(&self) -> bool { true }
    fn supports_scheduled_queues(&self)->bool {true}
    fn submission_error(&self) -> i32 { if self.is_crashed() { EIO.to_errno() } else { 0 } }
    fn params(&self) -> Result<DrmGpuParams> {
        let soc = self.soc;
        let masks = crate::m3_board::core_masks(soc, self.core_mask);
        Ok(DrmGpuParams { gpu_generation: hw::GpuGen::G15 as u32,
            gpu_variant: soc.gpu_variant as u32, gpu_revision: soc.gpu_revision as u32,
            chip_id: soc.chip_id, num_dies: 1, num_clusters_total: soc.clusters,
            num_cores_per_cluster: soc.cores_per_cluster,
            core_masks: masks, max_frequency_khz: self.max_frequency_khz, usc_generation: 3,
            gpu_hal_generation: hw::GpuHalGeneration::Legacy as u32,
            max_commands_per_submission: crate::file::MAX_COMMANDS_PER_SUBMISSION })
    }
    fn user_range(&self) -> Result<Range<u64>> { Ok(mmu::UAT_PGSZ as u64..0x2ff_ffff_8000) }
    fn unknown_page(&self) -> Result<u64> { Ok(0x2ff_ffff_8000) }
    fn base_clock_hz(&self) -> u32 {
        let value: u64;
        // SAFETY: read-only architectural counter frequency.
        unsafe { core::arch::asm!("mrs {x}, CNTFRQ_EL0", x=out(reg) value) };
        value as u32
    }
    fn new_vm(&self, range: Range<u64>) -> Result<mmu::Vm> {
        Option::as_mut(&mut *self.shared.lock()).ok_or(ENODEV)?.new_vm(self.ids.vm.next(), range)
    }
    fn new_queue(&self, vm: mmu::Vm, _ualloc: Arc<Mutex<alloc::DefaultAllocator>>,
        _ualloc_priv: Arc<Mutex<alloc::DefaultAllocator>>, priority: u32,
        usc_exec_base: u64) -> Result<KBox<dyn queue::Queue>> {
        Ok(KBox::new(crate::m3_submit::Queue::new(self.shared.clone(),self.scheduler.clone(),vm,priority,usc_exec_base,
            self.soc.features.fragment_dependency)?,GFP_KERNEL)?)
    }
    fn map_timestamp_buffer(&self, bo: gem::ObjectRef, range: Range<usize>) -> Result<mmu::KernelMapping> {
        self.shared.lock().as_ref().ok_or(ENODEV)?.map_timestamp(bo,range)
    }
}
