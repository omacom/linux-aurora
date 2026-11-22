// SPDX-License-Identifier: GPL-2.0-only OR MIT
//! G18 (J873, T8152/G18G) DRM render-node adapter.
//!
//! A structural copy of the M5 adapter (`m5_drm.rs`): one hardware command
//! per submit, out-fences signalled by a publication guard. It differs where
//! G18 differs:
//! - submissions run in submission order on a job worker ([`JobQueue`]);
//!   on a queue created with DRM_ASAHI_QUEUE_CREATE_ASYNC_SUBMIT, a submit
//!   with output syncs returns once accepted, every other submit once it
//!   has retired;
//! - render commands go through `g18_application` to a job sink the G18
//!   runtime implements, not through a captured reference profile;
//! - `map_timestamp_buffer` is implemented (contract K8) with the shared
//!   CPU-written timestamp buffers, so Mesa's G18G screen exposes timer
//!   queries and with them GL 3.3-4.6;
//! - compute is refused unless the adapter is built with `admit_compute`.
//!
//! Commands use the shared command layout (`include/uapi/drm/asahi_drm.h`);
//! `g18::uapi::command` parses it. The adapter holds no firmware state.
//! Everything it needs from the live G18 session goes through
//! [`G18DrmRuntime`].

use core::ops::Range;
use core::sync::atomic::{AtomicI32, Ordering};
use kernel::{bindings, macros::vtable, new_condvar, new_mutex, prelude::*, static_lock_class,
    dma_fence::{Fence, FenceContexts, FenceObject, FenceOps, RawDmaFence, UserFence},
    sync::{Arc, CondVar, Mutex, SpinLock}, uapi, workqueue, xarray};
use crate::{alloc, driver::AsahiRegData, file, g18::uapi::command as g17_uapi, g18::hw,
    g18::uapi::application::{self as g18_application, G18ApplicationRefusal, G18RenderJobSink, G18RenderRetirement},
    gem, gpu, g18::fw::vm_layout as m5_vm, mmu, queue, stats, stats_raw, timestamp};

const USER_END: u64 = (1 << 42) - 2 * mmu::UAT_PGSZ as u64;
/// Largest compute control stream admitted, in bytes. The kernel does not read
/// the stream (the CDM does); this only bounds what the firmware is pointed at.
/// 64 KiB is a sizing choice, not a hardware limit: no decode names one. It
/// admits a stream of many launches, which cost 2.10 ms per op on the M6 against
/// 10.45 ms when each op is its own stream. Only reachable with `admit_compute`.
pub(crate) const G18_COMPUTE_STREAM_BYTES: u64 = 64 * 1024;
/// `DRM_ASAHI_COMPUTE_USES_TEXTURE_STATE`: the dispatch binds texture or image
/// state, so the command takes the native texture compute mode (d62a,
/// `g18_application_compute_resources.rs`).
const G18_COMPUTE_TEXTURE_STATE: u32 =
    uapi::drm_asahi_compute_flags_DRM_ASAHI_COMPUTE_USES_TEXTURE_STATE as u32;

// `g18::uapi::command` parses by byte offset (it is also built on the host);
// keep its constants equal to the UAPI header.
const _: () = {
    use core::mem::{offset_of, size_of};
    assert!(g17_uapi::UAPI_RENDER_SIZE == size_of::<uapi::drm_asahi_cmd_render>());
    assert!(g17_uapi::UAPI_COMPUTE_SIZE == size_of::<uapi::drm_asahi_cmd_compute>());
    assert!(g17_uapi::UAPI_G18_RENDER_SIZE == size_of::<uapi::drm_asahi_cmd_render_g18>());
    assert!(g17_uapi::UAPI_G18_COMPUTE_SIZE == size_of::<uapi::drm_asahi_cmd_compute_g18>());
    assert!(offset_of!(uapi::drm_asahi_cmd_render_g18, resources) == g17_uapi::UAPI_RENDER_SIZE);
    assert!(offset_of!(uapi::drm_asahi_cmd_compute_g18, resources) == g17_uapi::UAPI_COMPUTE_SIZE);
    assert!(size_of::<uapi::drm_asahi_g18_render_resources>() == 80);
    assert!(size_of::<uapi::drm_asahi_g18_compute_resources>() == 64);
    assert!(offset_of!(uapi::drm_asahi_cmd_render, ray_query) == 0x150);
    assert!(offset_of!(uapi::drm_asahi_cmd_compute, prior_vdm) == 0x88);
    assert!(offset_of!(uapi::drm_asahi_cmd_render, ts_frag_end_copies) == 0x100);
    assert!(offset_of!(uapi::drm_asahi_cmd_render, prior_vdm) == 0x140);
    assert!(offset_of!(uapi::drm_asahi_cmd_render, prior_cdm) == 0x148);
    assert!(offset_of!(uapi::drm_asahi_cmd_compute, ts_end_copies) == 0x40);
    assert!(offset_of!(uapi::drm_asahi_cmd_compute, scs_layout) == 0x80);
    assert!(offset_of!(uapi::drm_asahi_cmd_compute, prior_cdm) == 0x90);
    assert!(offset_of!(uapi::drm_asahi_cmd_compute, cdm_auxiliary_allocation) == 0x98);
    assert!(g17_uapi::RENDER_G18_RESOURCES
        == uapi::drm_asahi_render_flags_DRM_ASAHI_RENDER_G18_RESOURCES as u32);
    assert!(g17_uapi::COMPUTE_G18_RESOURCES
        == uapi::drm_asahi_compute_flags_DRM_ASAHI_COMPUTE_G18_RESOURCES as u32);
};

// Identity Mesa's G18G bring-up gate accepts (`agx_chip.c:471-477`) and the
// drm-shim `g18g` profile publishes (`asahi_noop.c:129-147`), measured on
// J873: two clusters of six cores (ioreg core_mask_list=(63,63)).
const G18_CHIP_ID: u32 = 0x8152;
const G18_CLUSTERS: u32 = 2;
const G18_CORES_PER_CLUSTER: u32 = 6;
const G18_CORE_MASK: u32 = 0x3f;
const G18_USC_GENERATION: u32 = 3;
/// Revision B1, what the Linux SGX ID probe decodes on J873. Used only when
/// the runtime supplies no observed identity.
const G18_REVISION_B1: u32 = hw::GpuRevision::B1 as u32;
/// `DRM_ASAHI_GPU_HAL_300`: the UAPI has no HAL400 value yet and Mesa's
/// G18G gate requires 300. Apple's kext is PI_400.
const G18_UAPI_HAL_GENERATION: u32 = uapi::drm_asahi_gpu_hal_generation_DRM_ASAHI_GPU_HAL_300 as u32;

static QUEUE_NAME: &CStr = c"g18_queue";
static QUEUE_CLASS_KEY: Pin<&'static kernel::sync::LockClassKey> = static_lock_class!();

/// What the adapter needs from the live G18 session. The conductor
/// implements it over the G18 runtime once firmware and the graphics
/// queues are up (see the G18 DRM adapter notes).
pub(crate) trait G18DrmRuntime: G18RenderJobSink {
    /// Any crash, fault or quarantine that ends submission for this boot.
    fn is_crashed(&self) -> bool;
    /// VM_CREATE: a user VM in the session UAT. The adapter has already
    /// checked `kernel_range` against the M5/Mesa compact contract.
    fn new_user_vm(&self, id: u64, kernel_range: Range<u64>) -> Result<mmu::Vm>;
    /// Compute (K1). Only called when the adapter admits compute.
    fn submit_compute(&self, _vm: &mmu::Vm, _command: &g17_uapi::TranslatedComputeCommand)
        -> Result<G18RenderRetirement> {
        Err(ENOTSUPP)
    }
}

/// Adapter options fixed at registration.
#[derive(Debug, Copy, Clone, Default)]
pub(crate) struct G18DrmConfig {
    /// Admit compute commands (module parameter `g18_admit_compute`, default 0).
    pub(crate) admit_compute: bool,
    /// Admit exactly the native 4x MSAA render set (module parameter
    /// `g18_admit_msaa4`, default 0; msaa163). Off refuses every
    /// multisampled render as before.
    pub(crate) admit_msaa4: bool,
    /// Admit renders carrying `DRM_ASAHI_RENDER_G18_RESOURCES` (module
    /// parameter `g18_admit_graphics_scratch`, default 0). Off leaves the
    /// feature unadvertised and refuses the flag, so Mesa keeps refusing
    /// draws whose programs need a thread-private frame.
    pub(crate) admit_graphics_scratch: bool,
}

pub(crate) struct G18DrmGpu {
    runtime: Arc<dyn G18DrmRuntime>,
    /// Serializes job-sink calls across every queue of this device.
    submit_lock: Arc<Mutex<()>>,
    jobs: Arc<JobQueue>,
    config: G18DrmConfig,
    revision: u32,
    ids: gpu::SequenceIDs,
}

impl G18DrmGpu {
    /// `observed` is the SGX identity read after firmware start, if the
    /// runtime has one. It must match the published topology exactly.
    pub(crate) fn new(runtime: Arc<dyn G18DrmRuntime>, observed: Option<&hw::GpuIdConfig>,
        config: G18DrmConfig) -> Result<Self> {
        let revision = match observed {
            Some(id) => {
                if id.gpu_gen != hw::GpuGen::G18 || id.gpu_variant != hw::GpuVariant::G
                    || id.num_dies != 1 || id.num_clusters != G18_CLUSTERS
                    || id.num_cores != G18_CORES_PER_CLUSTER
                    || id.usc_generation != G18_USC_GENERATION
                    || id.core_masks.as_slice() != [G18_CORE_MASK; G18_CLUSTERS as usize] {
                    pr_err!("G18_DRM: observed identity {:?} is not the J873 G18G topology\n", id);
                    return Err(ENODEV);
                }
                id.gpu_rev as u32
            }
            None => G18_REVISION_B1,
        };
        let submit_lock = Arc::pin_init(new_mutex!(()), GFP_KERNEL)?;
        let jobs = JobQueue::new()?;
        Ok(Self { runtime, submit_lock, jobs, config, revision, ids: Default::default() })
    }
}

// Module 1134 reached G18 through its `DrmGpu` trait (drm_gpu.rs) and
// computed GET_PARAMS in file.rs from `DrmGpu::params`; here the same facts
// fill `drm_asahi_params_global` through `gpu::Gpu::get_params`. The shared
// file layer adds VM_STATUS and VM_STATUS_MIRROR (supports_vm_status) and
// SOFT_FAULTS when fault_control is 0xb; there are no queue limits.
impl gpu::Gpu for G18DrmGpu {
    fn stats_snapshot(&self) -> Option<Arc<stats::StatsSnapshot>> { None }
    fn raw_stats_snapshot(&self) -> Option<Arc<SpinLock<stats_raw::RawStatsSnapshot>>> { None }
    fn as_any(&self) -> &dyn core::any::Any { self }
    fn arc_as_any(self: Arc<Self>) -> Arc<dyn core::any::Any + Sync + Send> { self }
    fn manager(&self) -> Option<&dyn gpu::GpuManager> { None }
    fn init(&self) -> Result {
        if self.is_crashed() { Err(ENODEV) } else { Ok(()) }
    }
    fn ids(&self) -> &gpu::SequenceIDs { &self.ids }
    fn is_crashed(&self) -> bool { self.runtime.is_crashed() }
    fn get_params(&self, params: &mut uapi::drm_asahi_params_global) -> Result {
        let mut core_masks = [0; uapi::DRM_ASAHI_MAX_CLUSTERS as usize];
        for mask in core_masks.get_mut(..G18_CLUSTERS as usize).ok_or(EIO)? {
            *mask = G18_CORE_MASK;
        }
        params.gpu_generation = hw::GpuGen::G18 as u32;
        params.gpu_variant = hw::GpuVariant::G as u32;
        params.gpu_revision = self.revision;
        params.chip_id = G18_CHIP_ID;
        params.num_dies = 1;
        params.num_clusters_total = G18_CLUSTERS;
        params.num_cores_per_cluster = G18_CORES_PER_CLUSTER;
        params.core_masks = core_masks.map(|mask| mask.into());
        // No G18 clock policy is implemented by this adapter.
        params.max_frequency_khz = 0;
        params.usc_generation = G18_USC_GENERATION;
        params.gpu_hal_generation = G18_UAPI_HAL_GENERATION;
        params.max_commands_per_submission = 1;
        if self.config.admit_graphics_scratch {
            params.features |= uapi::drm_asahi_feature_DRM_ASAHI_FEATURE_G18_GRAPHICS_SCRATCH as u64;
        }
        params.features |= uapi::DRM_ASAHI_FEATURE_G18_NATIVE_TILE_SAMPLES as u64;
        params.features |= uapi::DRM_ASAHI_FEATURE_ASYNC_SUBMIT as u64;
        Ok(())
    }
    fn supports_vm_status(&self) -> bool { true }
    /// VM_BIND validates every op before committing any, and SUBMIT copies
    /// the command buffer before it resolves the syncs (module 1134 did both).
    fn tracked_submission(&self) -> bool { true }
    /// `DRM_IOCTL_ASAHI_GET_TIME` scales CNTPCT_EL0 by this rate; report the
    /// counter's own frequency, as G17P does. J873 runs it at 24 MHz.
    fn base_clock_hz(&self) -> u32 { counter_frequency() as u32 }
    /// 42-bit T8152 user space: user range UAT_PGSZ..USER_END with the dummy
    /// page at USER_END, as in module 1134.
    fn uat_geometry(&self) -> mmu::UatGeometry { mmu::UatGeometry { ias: 42, oas: 42 } }
    fn new_vm(&self, kernel_range: Range<u64>) -> Result<mmu::Vm> {
        if self.is_crashed() { return Err(ENODEV); }
        m5_vm::compact_range(&kernel_range).ok_or(EINVAL)?;
        let vm = self.runtime.new_user_vm(self.ids.vm.next(), kernel_range)?;
        // Module 1134 VM_CREATE: a simplefb alias is T8140-only; any other
        // GPU refuses the VM when the diagnostic parameter is set.
        if crate::g18::params::g17p_simplefb_iova.value() != 0 {
            return Err(ENOTSUPP);
        }
        Ok(vm)
    }
    fn new_queue(&self, vm: mmu::Vm,
        _ualloc: Arc<Mutex<alloc::DefaultAllocator>>,
        _ualloc_priv: Arc<Mutex<alloc::DefaultAllocator>>,
        priority: u32, usc_exec_base: u64,
        _usage: Arc<crate::usage::Client>) -> Result<KBox<dyn queue::Queue>> {
        if priority > 3 { return Err(EINVAL); }
        let window = g17_uapi::QueueUscWindow {
            base: usc_exec_base, user_start: m5_vm::DRIVER_BASE, user_end: USER_END,
        };
        window.validate().map_err(|_| EINVAL)?;
        let queue = G18Queue {
            runtime: self.runtime.clone(), submit_lock: self.submit_lock.clone(),
            config: self.config, vm, window,
            fences: FenceContexts::new(1, QUEUE_NAME, QUEUE_CLASS_KEY)?,
            jobs: self.jobs.clone(), failed: Arc::new(AtomicI32::new(0), GFP_KERNEL)?,
            asynchronous: false, recent: [0; QUEUE_DEPTH], next: 0, last: 0,
        };
        Ok(KBox::new(queue, GFP_KERNEL)?)
    }
    /// K8. Mesa binds `128 * sizeof(struct agx_timestamps)` of a WRITEBACK BO
    /// at screen creation and enables timer queries only if this works. G18
    /// publishes timestamps from the CPU after retirement, so the buffer needs
    /// no firmware mapping.
    fn map_timestamp_buffer(&self, bo: gem::ObjectRef, range: Range<usize>)
        -> Result<timestamp::Buffer> {
        if self.is_crashed() { return Err(ENODEV); }
        if range.is_empty() { return Err(EINVAL); }
        timestamp::Buffer::cpu(bo, range)
    }
}

/// CNTFRQ_EL0, or Apple's 24 MHz if boot firmware left it unprogrammed:
/// GET_TIME divides by this rate (`file.rs:1225-1229`) and must never see 0.
fn counter_frequency() -> u64 {
    let frequency: u64;
    // SAFETY: CNTFRQ_EL0 is a read-only system register.
    unsafe { core::arch::asm!("mrs {x}, CNTFRQ_EL0", x = out(reg) frequency) };
    if frequency == 0 { 24_000_000 } else { frequency }
}

/// The counter `DRM_IOCTL_ASAHI_GET_TIME` reads, ordered after prior code.
fn counter_ticks() -> u64 {
    let ticks: u64;
    // SAFETY: the ISB only orders the read; CNTPCT_EL0 is read-only.
    unsafe { core::arch::asm!("isb", "mrs {x}, CNTPCT_EL0", x = out(reg) ticks) };
    ticks
}

struct G18SubmissionFence;
#[vtable]
impl FenceOps for G18SubmissionFence {
    fn get_driver_name<'a>(self: &'a FenceObject<Self>) -> &'a CStr { c"aurora" }
    fn get_timeline_name<'a>(self: &'a FenceObject<Self>) -> &'a CStr { c"g18-submit" }
}

/// Keep the original sync vector: creating a second output list would add an
/// allocation failure after userspace has already supplied its wait targets.
/// Dropped unaccepted, it signals the fence with `result` and installs it.
struct OutputPublication {
    fence: Option<UserFence<G18SubmissionFence>>,
    syncs: KVec<file::SyncItem>,
    input_count: usize,
    result: Result,
}

impl OutputPublication {
    fn install(&mut self) {
        let Some(fence) = &self.fence else { return };
        let fence = Fence::from_fence(fence);
        for mut sync in self.syncs.drain(self.input_count..) {
            if let Some(chain) = sync.chain_fence.take() {
                sync.syncobj.add_point(chain, &fence, sync.timeline_value);
            } else { sync.syncobj.replace_fence(Some(&fence)); }
        }
    }

    /// The input fences, for a job to check.
    fn input_fences(&self) -> Result<KVec<Fence>> {
        let mut fences = KVec::new();
        for sync in &self.syncs[..self.input_count] {
            if let Some(fence) = &sync.fence {
                fences.push(Fence::from_fence(fence), GFP_KERNEL)?;
            }
        }
        Ok(fences)
    }

    /// Install the fence in every output syncobj and hand it to the job,
    /// which signals it. Only [`JobQueue::push`] calls this.
    fn accept(mut self) -> JobFence {
        self.install();
        JobFence { fence: self.fence.take(), result: Err(ECANCELED) }
    }
}

impl Drop for OutputPublication {
    fn drop(&mut self) {
        let Some(fence) = &self.fence else { return };
        if let Err(error) = self.result { fence.set_error(error); }
        // Signalled before it is installed: a submission that finds one of
        // this adapter's fences in a syncobj does not wait for it.
        fence.signal();
        self.install();
    }
}

/// An accepted job's installed output fence. Signals, with `result`, when
/// the job drops.
struct JobFence {
    fence: Option<UserFence<G18SubmissionFence>>,
    result: Result,
}

impl Drop for JobFence {
    fn drop(&mut self) {
        let Some(fence) = &self.fence else { return };
        if let Err(error) = self.result { fence.set_error(error); }
        fence.signal();
    }
}

/// Accepted jobs a queue may have waiting or running; a further submit
/// waits for the oldest.
const QUEUE_DEPTH: usize = 32;

/// One accepted submission. Dropping it signals its output fence.
struct G18Job {
    runtime: Arc<dyn G18DrmRuntime>,
    submit_lock: Arc<Mutex<()>>,
    vm: mmu::Vm,
    vm_job: Option<mmu::VmJobGuard>,
    commands: KVec<Admitted>,
    /// Set for a submission without outputs: its ioctl waits for the job
    /// and returns this (0 or a negative errno).
    result: Option<Arc<AtomicI32>>,
    inputs: KVec<Fence>,
    fence: JobFence,
    /// The queue's [`G18Queue::failed`].
    queue_failed: Arc<AtomicI32>,
    /// The submit ioctl that accepted the job, for its failure report.
    submission: u64,
    pid: i32,
    comm: [u8; 16],
}

impl G18Job {
    fn run(mut job: KBox<Self>) {
        let result = job.execute();
        // Release the mappings before the fence signals, as a synchronous
        // submit did.
        job.vm_job = None;
        match (&job.result, result) {
            (Some(slot), result) =>
                slot.store(result.err().map_or(0, |error| error.to_errno()), Ordering::Release),
            // The ioctl returned long ago and cannot report this; the output
            // fence carries the error and the queue's next submit returns it.
            (None, Err(error)) => {
                let len = job.comm.iter().position(|&c| c == 0).unwrap_or(job.comm.len());
                pr_warn!("G18 job failed: submission {} pid {} ({}): {:?}\n", job.submission,
                    job.pid, kernel::str::BStr::from_bytes(&job.comm[..len]), error);
                let _ = job.queue_failed.compare_exchange(0, error.to_errno(),
                    Ordering::AcqRel, Ordering::Acquire);
            }
            (None, Ok(())) => {}
        }
        job.fence.result = result;
    }

    fn execute(&mut self) -> Result {
        // Every input has signalled: the submit waited for those that no job
        // of this adapter signals (see [`wait_foreign_inputs`]), and the
        // others belong to earlier jobs, which retired first. A failed one
        // fails this job, also inside a timeline point or a fence array,
        // which do not carry their fences' errors themselves.
        for fence in &self.inputs {
            // SAFETY: the input fence is strongly held for the call.
            let status = unsafe { asahi_g18_fence_error(fence.raw()) };
            if status < 0 { return Err(Error::from_errno(status)); }
        }
        let _serialized = self.submit_lock.lock();
        if self.vm.status().map_or(0, |s| s.get()) != 0 { return Err(EIO); }
        if self.runtime.is_crashed() { return Err(ENODEV); }
        // An empty command buffer is a synchronization-only submission.
        // Never manufacture a hardware job or relax hardware retirement checks.
        if self.commands.is_empty() {
            pr_debug!("G18_SYNC_ONLY inputs={}\n", self.inputs.len());
            return Ok(());
        }
        // Commands run in submission order and each retires before the next
        // starts, so every barrier (an earlier command of this submission, or
        // any earlier submission) has completed when its waiter begins.
        for command in self.commands.drain_all() {
            if self.vm.status().map_or(0, |s| s.get()) != 0 { return Err(EIO); }
            if self.runtime.is_crashed() { return Err(ENODEV); }
            // Without job stage ticks, the CPU bracket `[t0, t1]` around the
            // job bounds every stage: starts get t0, ends get t1.
            let t0 = counter_ticks();
            match command {
                Admitted::Render(render, slots) => {
                    let retired = self.runtime.submit_render(&self.vm, &render)?;
                    let t1 = counter_ticks();
                    slots.publish(retired.stage_ticks.unwrap_or([t0, t1, t0, t1]),
                        counter_frequency());
                }
                Admitted::Compute(command, slots) => {
                    let retired = self.runtime.submit_compute(&self.vm, &command)?;
                    let t1 = counter_ticks();
                    let [start, _, _, end] = retired.stage_ticks.unwrap_or([t0, t1, t0, t1]);
                    slots.publish([start, end, 0, 0], counter_frequency());
                }
            }
        }
        Ok(())
    }
}

/// The accepted jobs of every queue of the device, run one at a time in
/// submission order on a worker. The GPU runs one job at a time anyway; the
/// worker lets a submit return once its job is accepted instead of when it
/// retires. Running in submission order across queues keeps the ordering
/// the synchronous adapter gave: every fence of an earlier submission has
/// signalled when a job starts.
#[pin_data]
struct JobQueue {
    #[pin]
    state: Mutex<JobQueueState>,
    /// Notified whenever `completed` advances.
    #[pin]
    retired: CondVar,
}

struct JobQueueState {
    pending: KVec<KBox<G18Job>>,
    /// A worker is draining `pending`.
    running: bool,
    /// Sequence numbers of the last accepted and the last finished job.
    submitted: u64,
    completed: u64,
}

impl JobQueue {
    fn new() -> Result<Arc<Self>> {
        Arc::pin_init(pin_init!(Self {
            state <- new_mutex!(JobQueueState {
                pending: KVec::new(), running: false, submitted: 0, completed: 0,
            }, "G18 jobs"),
            retired <- new_condvar!("G18 job retired"),
        }), GFP_KERNEL)
    }

    /// Accept the job that `job` builds around the publication's fence and
    /// return its sequence number, or give the publication back if the job
    /// cannot be queued. The fence is installed under the lock, once the
    /// job has its place: a submission that finds it in a syncobj is
    /// accepted later and so runs after the job that signals it.
    fn push(self: &Arc<Self>, publication: OutputPublication,
        job: impl FnOnce(JobFence) -> KBox<G18Job>)
        -> core::result::Result<u64, OutputPublication> {
        let mut state = self.state.lock();
        if state.pending.reserve(1, GFP_KERNEL).is_err() { return Err(publication); }
        if !state.running {
            let this = self.clone();
            if workqueue::system_unbound().try_spawn(GFP_KERNEL, move || this.drain()).is_err() {
                return Err(publication);
            }
            state.running = true;
        }
        let job = job(publication.accept());
        // Cannot fail: room was reserved above.
        let _ = state.pending.push_within_capacity(job);
        state.submitted += 1;
        Ok(state.submitted)
    }

    fn drain(&self) {
        loop {
            let job = {
                let mut state = self.state.lock();
                match state.pending.remove(0) {
                    Ok(job) => job,
                    Err(_) => { state.running = false; return; }
                }
            };
            G18Job::run(job);
            self.state.lock().completed += 1;
            self.retired.notify_all();
        }
    }

    /// Wait until job `seq` has finished and its fence has signalled.
    fn wait(&self, seq: u64, interruptible: bool) -> Result {
        let mut state = self.state.lock();
        while state.completed < seq {
            if !interruptible {
                self.retired.wait(&mut state);
            } else if self.retired.wait_interruptible(&mut state) {
                return Err(ERESTARTSYS);
            }
        }
        Ok(())
    }
}

struct UserAddressSpace<'a> { vm: &'a mmu::Vm }
impl g17_uapi::GpuAddressSpace for UserAddressSpace<'_> {
    fn covers(&self, address: u64, size: u64, access: g17_uapi::GpuAccess) -> bool {
        let (read, write) = match access {
            g17_uapi::GpuAccess::Read => (true, false),
            g17_uapi::GpuAccess::Write => (false, true),
            g17_uapi::GpuAccess::ReadWrite => (true, true),
        };
        m5_vm::user_range_contains(&self.vm.kernel_range(), address, size, USER_END)
            && self.vm.covers_range(address, size, read, write)
    }
}

/// A parsed timestamp pair in the UAPI's own type, for the shared resolver.
fn uapi_timestamps(timestamps: g17_uapi::UapiTimestamps) -> uapi::drm_asahi_timestamps {
    uapi::drm_asahi_timestamps {
        start: uapi::drm_asahi_timestamp {
            handle: timestamps.start.handle,
            offset: timestamps.start.offset,
        },
        end: uapi::drm_asahi_timestamp {
            handle: timestamps.end.handle,
            offset: timestamps.end.offset,
        },
    }
}

struct G18Queue {
    runtime: Arc<dyn G18DrmRuntime>,
    submit_lock: Arc<Mutex<()>>,
    config: G18DrmConfig,
    vm: mmu::Vm,
    window: g17_uapi::QueueUscWindow,
    fences: FenceContexts,
    jobs: Arc<JobQueue>,
    /// The error of a job of this queue that failed after its submit
    /// returned (0: none). The next submit returns and clears it: Mesa learns
    /// of a GPU fault only from a submit error, and then treats the context
    /// as lost, as it did when a submit returned the error of its own job.
    failed: Arc<AtomicI32>,
    /// Created with DRM_ASAHI_QUEUE_CREATE_ASYNC_SUBMIT.
    asynchronous: bool,
    /// Sequence numbers of this queue's last [`QUEUE_DEPTH`] jobs, oldest at
    /// `next`, and of its last job.
    recent: [u64; QUEUE_DEPTH],
    next: usize,
    last: u64,
}

impl queue::Queue for G18Queue {
    /// Userspace that reads GPU results right after a submit returns (Mesa's
    /// G18G indirect dispatch sizes, before its fix) opts out by not setting
    /// the flag; its submissions keep returning once they have retired.
    fn set_async_submit(&mut self) -> Result {
        self.asynchronous = true;
        Ok(())
    }

    /// On an asynchronous queue, a submission with output syncs returns once
    /// its job is accepted; the outputs signal when it retires. Every other
    /// submission returns once it has retired: one without outputs gives
    /// userspace nothing to wait on (Mesa's G18G split compute prepass reads
    /// GPU results after such submissions).
    fn submit(&mut self, _reg_data: &AsahiRegData<'_>, submission_id: u64,
        syncs: KVec<file::SyncItem>,
        in_sync_count: usize, cmdbuf_raw: &[u8],
        objects: Pin<&xarray::XArray<KBox<file::Object>>>) -> Result {
        if in_sync_count > syncs.len() { return Err(EINVAL); }
        // Nothing is published until these waits are done, so a signal
        // simply restarts the ioctl.
        self.jobs.wait(self.recent[self.next], true)?;
        let fence: UserFence<G18SubmissionFence> =
            self.fences.new_fence(0, G18SubmissionFence)?.into();
        wait_foreign_inputs(&fence, &syncs[..in_sync_count])?;
        let synchronous = !self.asynchronous || syncs.len() == in_sync_count;
        let mut publication = OutputPublication {
            fence: Some(fence), syncs, input_count: in_sync_count, result: Err(ECANCELED),
        };
        let accepted = (|| {
            let slot = KBox::<G18Job>::new_uninit(GFP_KERNEL)?;
            let result = if synchronous {
                Some(Arc::new(AtomicI32::new(0), GFP_KERNEL)?)
            } else {
                None
            };
            let inputs = publication.input_fences()?;
            let (vm_job, commands) = self.prepare(cmdbuf_raw, objects)?;
            Ok((slot, result, inputs, vm_job, commands))
        })();
        let (slot, result, inputs, vm_job, commands) = match accepted {
            Ok(accepted) => accepted,
            Err(error) => {
                // The runtime records and quarantines a failed published VM
                // before this output fence is installed/signalled by the
                // publication guard, after the queue's earlier fences.
                publication.result = Err(error);
                self.jobs.wait(self.last, false)?;
                return Err(error);
            }
        };
        let task = kernel::current!();
        // SAFETY: `comm` of the current task is a NUL-terminated array that
        // only the task itself renames; a torn copy is only a log string.
        let comm = unsafe { (*task.as_ptr()).comm }.map(|c| c as u8);
        let job = |fence| slot.write(G18Job {
            runtime: self.runtime.clone(), submit_lock: self.submit_lock.clone(),
            vm: self.vm.clone(), vm_job, commands, result: result.clone(), inputs,
            fence, queue_failed: self.failed.clone(),
            submission: submission_id, pid: task.pid(), comm,
        });
        let seq = match self.jobs.push(publication, job) {
            Ok(seq) => seq,
            Err(mut publication) => {
                // As above: the fence signals after the queue's earlier ones.
                publication.result = Err(ENOMEM);
                self.jobs.wait(self.last, false)?;
                return Err(ENOMEM);
            }
        };
        self.recent[self.next] = seq;
        self.next = (self.next + 1) % QUEUE_DEPTH;
        self.last = seq;
        let Some(result) = result else { return Ok(()) };
        self.jobs.wait(seq, false)?;
        match result.load(Ordering::Acquire) {
            0 => Ok(()),
            errno => Err(Error::from_errno(errno)),
        }
    }

    fn drain(&mut self) {
        let _ = self.jobs.wait(self.last, false);
    }
}

unsafe extern "C" {
    fn asahi_g18_wait_foreign_fences(head: *mut bindings::dma_fence,
        own: *const bindings::dma_fence_ops) -> i32;
    fn asahi_g18_fence_error(head: *mut bindings::dma_fence) -> i32;
}

/// Wait for every input fence that no job of this adapter signals, inside
/// chains and arrays too (Mesa's timeline points wrap this adapter's
/// fences). The job worker waits for nothing: an input that it waited for
/// would hold every queue of the device. This adapter's own fences are
/// skipped, since they belong to earlier jobs, which retire first.
fn wait_foreign_inputs(ours: &UserFence<G18SubmissionFence>, inputs: &[file::SyncItem])
    -> Result {
    // SAFETY: `ours` is held and not yet signalled, so its ops are still set;
    // the pointer is only compared.
    let own = unsafe { (*ours.raw()).ops };
    for sync in inputs {
        let Some(fence) = &sync.fence else { continue };
        // SAFETY: the input fence is strongly held for the call.
        let error = unsafe { asahi_g18_wait_foreign_fences(fence.raw(), own) };
        if error != 0 { return Err(Error::from_errno(error)); }
    }
    Ok(())
}

impl G18Queue {
    /// Check and admit a submission. An empty command buffer is a
    /// synchronization-only submission and holds no VM job.
    fn prepare(&self, cmdbuf_raw: &[u8], objects: Pin<&xarray::XArray<KBox<file::Object>>>)
        -> Result<(Option<mmu::VmJobGuard>, KVec<Admitted>)> {
        match self.failed.swap(0, Ordering::AcqRel) {
            0 => {}
            errno => return Err(Error::from_errno(errno)),
        }
        if self.vm.status().map_or(0, |s| s.get()) != 0 { return Err(EIO); }
        if self.runtime.is_crashed() { return Err(ENODEV); }
        if cmdbuf_raw.is_empty() { return Ok((None, KVec::new())); }
        let vm_job = self.vm.retain_t8140_job()?;
        let admitted = self.admit(cmdbuf_raw, objects)?;
        Ok((Some(vm_job), admitted))
    }

    /// Parse every hardware command of a submission, apply the G18 admission
    /// rules and resolve their timestamp destinations, all before any runs:
    /// a submission is admitted whole or not at all. Nothing here touches
    /// the GPU. A compute prepass and the render that consumes it (transform
    /// feedback, geometry and tessellation draws) arrive as one submission.
    #[inline(never)]
    fn admit(&self, cmdbuf_raw: &[u8], objects: Pin<&xarray::XArray<KBox<file::Object>>>)
        -> Result<KVec<Admitted>> {
        let malformed = |error: g17_uapi::UapiTranslateError| {
            pr_err!("G18 submit admission rejected: {:?}\n", error);
            EINVAL
        };
        let mut parser = g17_uapi::UapiCommandParser::new(cmdbuf_raw);
        let mut admitted = KVec::new();
        while let Some(command) = parser.next_hardware().map_err(malformed)? {
            let command = match command {
                g17_uapi::ParsedHardwareCommand::Render {
                    payload, vertex_attachments, fragment_attachments, ..
                } => self.admit_render(&payload, vertex_attachments, fragment_attachments, objects)?,
                g17_uapi::ParsedHardwareCommand::Compute { payload, attachments, .. } =>
                    self.admit_compute(&payload, attachments, objects)?,
            };
            admitted.push(command, GFP_KERNEL)?;
        }
        parser.finish().map_err(malformed)?;
        Ok(admitted)
    }

    #[inline(never)]
    fn admit_render(&self, payload: &g17_uapi::UapiRenderCommand,
        vertex_attachments: g17_uapi::UapiAttachmentList,
        fragment_attachments: g17_uapi::UapiAttachmentList,
        objects: Pin<&xarray::XArray<KBox<file::Object>>>) -> Result<Admitted> {
        let address_space = UserAddressSpace { vm: &self.vm };
        let scratch = if payload.flags & g17_uapi::RENDER_G18_RESOURCES != 0 {
            Some(self.admit_render_scratch(payload)?)
        } else {
            None
        };
        // The shared translator refuses the G18 flag and never sees the
        // suffix. Only this adapter consumes and strips a validated suffix.
        let mut untagged = *payload;
        untagged.flags &= !g17_uapi::RENDER_G18_RESOURCES;
        untagged.g18_resources = [0; 20];
        let mut render = g18_application::translate_g18_render_admitting(&untagged,
            vertex_attachments, fragment_attachments, self.window, &address_space,
            self.config.admit_msaa4)
            .map_err(|refusal| refuse_render(payload, refusal))?;
        render.scratch = scratch;
        let render = KBox::new(render, GFP_KERNEL)?;
        if !g18_application::job_owned_fields_clear(&render.parameters, &render.controls) {
            pr_err!("G18 render admission rejected: job-owned fields set\n");
            return Err(EINVAL);
        }
        let slots = timestamp::Destinations::resolve_unaligned(objects, &[
            uapi_timestamps(payload.vertex_timestamps),
            uapi_timestamps(payload.fragment_timestamps),
        ])?;
        Ok(Admitted::Render(render, slots))
    }

    /// Derive the TA/3D private-memory selection for a flagged render. Only
    /// single-sample, single-layer renders with one-page tile storage are in
    /// the evidenced envelope (g18_graphics_scratch.rs). The selection never
    /// resizes the owned pool: ideal when both kicks fit it, else the
    /// native minimum result, else ENOSPC. Nothing is published on refusal.
    #[inline(never)]
    fn admit_render_scratch(&self, payload: &g17_uapi::UapiRenderCommand)
        -> Result<crate::g18::uapi::graphics_scratch::G18RenderScratchSelection> {
        use crate::{g18::uapi::graphics_scratch::G18RenderMetadata, g18::uapi::render_policy as policy};
        if !self.config.admit_graphics_scratch {
            pr_err!("G18 render admission rejected: graphics scratch is disabled (g18_admit_graphics_scratch=0)\n");
            return Err(ENOTSUPP);
        }
        if payload.samples != 1 || payload.layers != 1
            || (payload.utile_width, payload.utile_height) != (32, 32) {
            pr_err!("G18 render admission rejected: graphics scratch outside single-sample 32x32 envelope samples={} layers={} utiles={}x{}\n",
                payload.samples, payload.layers, payload.utile_width, payload.utile_height);
            return Err(ENOTSUPP);
        }
        let tile_bytes = 32 * 32 * u32::from(payload.sample_size);
        let metadata = G18RenderMetadata::decode(&payload.g18_resources, tile_bytes)
            .ok_or_else(|| {
                pr_err!("G18 render admission rejected: invalid graphics scratch metadata words={:x?} tile_bytes={}\n",
                    payload.g18_resources, tile_bytes);
                EINVAL
            })?;
        let plan = metadata.plan(policy::RENDER_POOL_BLOCK_BYTES).ok_or_else(|| {
            pr_err!("G18 render admission rejected: graphics scratch exceeds the native pool capacity\n");
            ENOSPC
        })?;
        let (owned_bytes, owned_chunks) = policy::owned_scratch_capacity();
        let selected = plan.select(owned_bytes, owned_chunks).ok_or_else(|| {
            pr_err!("G18 render admission rejected: graphics scratch needs {:?} beyond owned pool {} bytes / {} chunks\n",
                plan.minimum, owned_bytes, owned_chunks);
            ENOSPC
        })?;
        pr_info!("G18_RENDER_SCRATCH vertex={:?} fragment={:?} tiles={:?} tile_bytes={} mode={:?} ta={:?} d3={:?}\n",
            metadata.vertex, metadata.fragment, metadata.tiles, tile_bytes, selected.mode,
            selected.stages[0], selected.stages[1]);
        Ok(selected)
    }

    #[inline(never)]
    fn admit_compute(&self, payload: &g17_uapi::UapiComputeCommand,
        attachments: g17_uapi::UapiAttachmentList,
        objects: Pin<&xarray::XArray<KBox<file::Object>>>) -> Result<Admitted> {
        if !self.config.admit_compute {
            pr_err!("G18 compute admission rejected: compute is disabled (g18_admit_compute=0)\n");
            return Err(ENOTSUPP);
        }
        let stream_bytes = payload.control_stream_end.saturating_sub(payload.control_stream_base);
        let g18_flags = G18_COMPUTE_TEXTURE_STATE | g17_uapi::COMPUTE_G18_RESOURCES;
        // ADD3 and every flag outside the G18 contract remain refused.
        if payload.flags & !g18_flags != 0 || payload.sampler_heap != 0
            || payload.sampler_count != 0 || stream_bytes > G18_COMPUTE_STREAM_BYTES {
            pr_err!("G18 compute admission rejected flags={:#x} sampler_heap={:#x} samplers={} stream_bytes={} (admits at most {} bytes)\n",
                payload.flags, payload.sampler_heap, payload.sampler_count, stream_bytes,
                G18_COMPUTE_STREAM_BYTES);
            return Err(ENOTSUPP);
        }
        let resource_descriptor = if payload.flags & g17_uapi::COMPUTE_G18_RESOURCES != 0 {
            let metadata = crate::g18::uapi::compute_metadata::G18ComputeMetadata::decode(payload.g18_resources)
                .ok_or_else(|| {
                    pr_err!("G18 compute admission rejected: invalid resource metadata\n");
                    EINVAL
                })?;
            if crate::g18::uapi::compute_plan::G18ComputePlan::from_descriptor(&metadata.descriptor).is_none() {
                pr_err!("G18 compute admission rejected: resource plan exceeds supported capacity\n");
                return Err(ENOSPC);
            }
            // Main static TPS uses the metadata-derived owned pool and SMA
            // launch pair. Mesa's G18 compute encoder carries the same aligned
            // frame in both SPECTPR scratch fields. Constant-program scratch
            // and SCS still need their separate launch contract.
            if metadata.demand.constant_scratch_bytes != 0 || payload.scs_layout != 0
            {
                pr_err!("G18 compute admission rejected: constant scratch or SCS integration pending\n");
                return Err(ENOTSUPP);
            }
            crate::g18_kick_pr_info!("G18_COMPUTE_METADATA stream={:#x} registers={} scratch_bytes={} spectpr_bytes={} constant_scratch={} local={:?}\n",
                payload.control_stream_base, payload.g18_resources[2],
                metadata.demand.scratch_bytes, payload.g18_resources[4],
                metadata.demand.constant_scratch_bytes, &payload.g18_resources[12..15]);
            Some(metadata.descriptor)
        } else {
            None
        };
        // The shared G17 translator refuses G18 flags and nonzero resource
        // words. Only this adapter may consume and strip a validated suffix.
        let mut untagged = *payload;
        untagged.flags &= !g18_flags;
        if resource_descriptor.is_some() { untagged.g18_resources = [0; 16]; }
        let address_space = UserAddressSpace { vm: &self.vm };
        let mut translated = g17_uapi::translate_compute_command(untagged, attachments,
            self.window, &address_space).map_err(|error| {
                pr_err!("G18 compute admission rejected {:?}: helper={:#x}/{:#x}/{:#x} flags={:#x}\n",
                    error, payload.helper.binary, payload.helper.config, payload.helper.data, payload.flags);
                EINVAL
            })?;
        translated.g18_texture_state = payload.flags & G18_COMPUTE_TEXTURE_STATE != 0;
        translated.g18_resource_descriptor = resource_descriptor;
        let slots = timestamp::Destinations::resolve_unaligned(objects,
            &[uapi_timestamps(payload.timestamps)])?;
        Ok(Admitted::Compute(KBox::new(translated, GFP_KERNEL)?, slots))
    }
}

/// An admitted command, heap-held so the submit frame stays small.
enum Admitted {
    Render(KBox<g18_application::G18ApplicationRender>, timestamp::Destinations),
    Compute(KBox<g17_uapi::TranslatedComputeCommand>, timestamp::Destinations),
}

/// Log a render refusal with the fields that decided it and map it to the
/// M5 errno convention: EINVAL for a malformed command, ENOTSUPP otherwise.
fn refuse_render(payload: &g17_uapi::UapiRenderCommand, refusal: G18ApplicationRefusal) -> Error {
    pr_err!("G18 render admission rejected {:?}: size={}x{} layers={} utiles={}x{} samples={} msaa={:#x} flags={:#x} zls={:#x} depth={:#x}/{:#x} stencil={:#x} bgobjvals={:#x} samplers={}\n",
        refusal, payload.width, payload.height, payload.layers, payload.utile_width,
        payload.utile_height, payload.samples, payload.multisample_control, payload.flags,
        payload.zls_control, payload.depth.base, payload.depth.compression_base,
        payload.stencil.base, payload.stencil_clear, payload.sampler_count);
    if refusal.is_invalid() { EINVAL } else { ENOTSUPP }
}
