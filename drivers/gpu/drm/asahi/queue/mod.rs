// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Submission queue management
//!
//! This module implements the userspace view of submission queues and the logic to map userspace
//! submissions to firmware queues.

use kernel::dma_fence::*;
use kernel::prelude::*;
use kernel::{
    c_str,
    dma_fence,
    drm::sched,
    macros::versions,
    sync::{
        Arc,
        LockClassKey,
        Mutex, //
    },
    uapi,
    xarray, //
};

use crate::alloc::Allocator;
use crate::debug::*;
use crate::driver::{AsahiDevRef, AsahiDevice};
use crate::file::MAX_COMMANDS_PER_SUBMISSION;
use crate::fw::types::*;
use crate::gpu::{
    Gpu,
    GpuManager, //
};
use crate::inner_weak_ptr;
use crate::microseq;
use crate::module_parameters;
use crate::stats;
use crate::util::{
    AnyBitPattern,
    Reader, //
};
use crate::{
    alloc,
    buffer,
    channel,
    driver::AsahiRegData,
    event,
    file,
    fw,
    gpu,
    mmu,
    workqueue, //
};

use core::sync::atomic::{
    AtomicBool,
    AtomicU64,
    Ordering, //
};

const DEBUG_CLASS: DebugFlags = DebugFlags::Queue;

const WQ_SIZE: u32 = 0x500;

mod common;
mod compute;
mod render;

fn can_submit_inline(dependencies_ready: bool, previous_status: Option<i32>) -> bool {
    dependencies_ready && previous_status.is_none_or(|status| status != 0)
}

/// TVB state that a queue shares with its render jobs.
struct TvbShared {
    /// Sequence number of the last render job that carried a slot init and failed in `run()`
    /// before any of its commands reached the firmware, or 0.
    lost_init: AtomicU64,
}

/// What a render job knows about the TVB buffer it uses.
struct TvbJob {
    shared: Arc<TvbShared>,
    /// This job's render sequence number on its queue (from 1).
    seq: u64,
    /// Sequence number of the render job whose slot init this job relies on (`seq` if it carries
    /// the init itself), or 0 if no init was needed since the queue was created.
    init_seq: u64,
    /// Set by `run()` before the job's first command is published to the firmware.
    published: AtomicBool,
}

impl TvbJob {
    fn carries_init(&self) -> bool {
        self.init_seq != 0 && self.init_seq == self.seq
    }

    /// Whether the slot init this job relies on never reached the firmware.
    fn init_lost(&self) -> bool {
        init_was_lost(self.shared.lost_init.load(Ordering::Acquire), self.init_seq)
    }

    /// Called when `run()` fails. An init carrier that published nothing lost its init.
    fn note_run_failed(&self) {
        if self.carries_init() && !self.published.load(Ordering::Acquire) {
            self.shared.lost_init.store(self.seq, Ordering::Release);
        }
    }
}

/// Whether the init carried by render job `init_seq` is the one that `lost_init` reports lost.
fn init_was_lost(lost_init: u64, init_seq: u64) -> bool {
    init_seq != 0 && lost_init == init_seq
}

/// A render submission as the TVB grow sees it.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum TvbUse {
    /// Its finished fence has not signalled.
    Pending,
    /// It completed without error: it and every render before it are done.
    Done,
    /// It failed in `run()` before any command reached the firmware.
    FailedUnpublished,
    /// It failed after some of its commands reached the firmware. Nothing signals when those
    /// retire (the job's other commands were rolled back), so only a later clean completion
    /// proves them done.
    FailedPublished,
}

impl TvbUse {
    fn from_status(status: i32, published: bool) -> Self {
        match status {
            0 => Self::Pending,
            1.. => Self::Done,
            _ if published => Self::FailedPublished,
            _ => Self::FailedUnpublished,
        }
    }
}

/// Drops the entries of `users` (oldest first, as classified by `seen`) that cannot hold the TVB
/// any more: a clean completion proves itself and every older entry done (renders complete in
/// order on the fragment queue), an entry that failed before reaching the firmware never held it.
/// Pending entries stay, and of the entries that failed after reaching the firmware the newest one
/// stays (it stands in for the older ones, since whatever proves it done proves them done too).
fn prune_tvb_uses<T>(users: &mut KVec<T>, seen: impl Fn(&T) -> TvbUse) {
    if let Some(done) = users.iter().rposition(|u| seen(u) == TvbUse::Done) {
        users.drain(..=done);
    }
    let newest_failed = users
        .iter()
        .rposition(|u| seen(u) == TvbUse::FailedPublished);
    let mut index = 0;
    users.retain(|u| {
        let keep = match seen(u) {
            TvbUse::Pending => true,
            TvbUse::FailedPublished => Some(index) == newest_failed,
            TvbUse::Done | TvbUse::FailedUnpublished => false,
        };
        index += 1;
        keep
    });
}

/// An entry of [`Queue::ver::tvb_users`].
struct TvbUser {
    /// Finished fence of the submission.
    fence: Fence,
    job: Arc<TvbJob>,
    /// The state last read by [`Queue::ver::prune_tvb_users`].
    seen: TvbUse,
}

/// Trait implemented by all versioned queues.
pub(crate) trait Queue: Send + Sync {
    /// Sets the watchdog allowance of the queue's compute commands to `requested_ms`
    /// (`drm_asahi_queue_create::compute_timeout_ms`, nonzero) and returns the allowance in
    /// effect. Only queues of GPUs that advertise DRM_ASAHI_FEATURE_COMPUTE_TIMEOUT take one.
    fn set_compute_timeout(&mut self, _requested_ms: u32) -> Result<u32> {
        Err(EINVAL)
    }

    /// Lets submissions with output syncs return once accepted
    /// (DRM_ASAHI_QUEUE_CREATE_ASYNC_SUBMIT). Only queues of GPUs that advertise
    /// DRM_ASAHI_FEATURE_ASYNC_SUBMIT take it.
    fn set_async_submit(&mut self) -> Result {
        Err(EINVAL)
    }

    fn reserve_render_scs(&mut self, layouts: [u64; 2]) -> Result {
        if layouts != [0; 2] {
            return Err(EINVAL);
        }
        Ok(())
    }

    fn submit(
        &mut self,
        reg_data: &AsahiRegData<'_>,
        id: u64,
        syncs: KVec<file::SyncItem>,
        in_sync_count: usize,
        cmdbuf_raw: &[u8],
        objects: Pin<&xarray::XArray<KBox<file::Object>>>,
    ) -> Result;

    /// Waits for the work submitted so far to finish, before the mappings of a closing
    /// client go. Queues whose VMs keep user mappings for accepted jobs need not wait.
    fn drain(&mut self) {}
}

#[versions(AGX)]
struct SubQueue {
    wq: Arc<workqueue::WorkQueue::ver>,
}

#[versions(AGX)]
impl SubQueue::ver {
    fn new_job(&mut self, fence: dma_fence::Fence) -> SubQueueJob::ver {
        SubQueueJob::ver {
            wq: self.wq.clone(),
            fence: Some(fence),
            job: None,
        }
    }
}

#[versions(AGX)]
struct SubQueueJob {
    wq: Arc<workqueue::WorkQueue::ver>,
    job: Option<workqueue::Job::ver>,
    fence: Option<dma_fence::Fence>,
}

#[versions(AGX)]
impl SubQueueJob::ver {
    fn get(&mut self) -> Result<&mut workqueue::Job::ver> {
        if self.job.is_none() {
            mod_pr_debug!("SubQueueJob: Creating {:?} job\n", self.wq.pipe_type());
            self.job
                .replace(self.wq.new_job(self.fence.take().unwrap())?);
        }
        Ok(self.job.as_mut().expect("expected a Job"))
    }

    fn commit(&mut self) -> Result {
        match self.job.as_mut() {
            Some(job) => job.commit(),
            None => Ok(()),
        }
    }

    fn can_submit(&self) -> Option<Fence> {
        self.job.as_ref().and_then(|job| job.can_submit())
    }

    fn can_submit_inline(&self) -> bool {
        self.job.as_ref().is_none_or(|job| job.can_submit_inline())
    }
}

#[versions(AGX)]
pub(crate) struct Queue {
    dev: AsahiDevRef,
    _sched: sched::Scheduler<QueueJob::ver>,
    entity: sched::Entity<QueueJob::ver>,
    vm: mmu::Vm,
    q_vtx: Option<SubQueue::ver>,
    q_frag: Option<SubQueue::ver>,
    q_comp: Option<SubQueue::ver>,
    fence_ctx: FenceContexts,
    inner: QueueInner::ver,
    stats: Arc<stats::StatsSnapshot>,
    /// Finished fence of the last submitted job, waited for when the queue goes away.
    last_out: Option<Fence>,
    /// The submissions with a render that may still use the TVB buffer: those after the newest one
    /// that completed without error. See [`Queue::ver::wait_tvb_idle`].
    tvb_users: KVec<TvbUser>,
    /// The TVB grew (or the render that re-initialised its slot never reached the firmware), so the
    /// next render must re-initialise the buffer slot. Cleared once a render with that init is
    /// pushed.
    tvb_init_pending: bool,
    /// Shared with this queue's render jobs: reports a slot init that never reached the firmware.
    tvb_shared: Arc<TvbShared>,
    /// Sequence number of the last render job pushed (0 before the first).
    tvb_seq: u64,
    /// Sequence number of the render job that carried the newest slot init, or 0.
    tvb_init_seq: u64,
}

#[versions(AGX)]
pub(crate) struct QueueInner {
    dev: AsahiDevRef,
    ualloc: Arc<Mutex<alloc::DefaultAllocator>>,
    buffer: buffer::Buffer::ver,
    gpu_context: Arc<workqueue::GpuContext>,
    notifier_list: Arc<GpuObject<fw::event::NotifierList>>,
    notifier: Arc<GpuObject<fw::event::Notifier::ver>>,
    usc_exec_base: u64,
    usage: Arc<crate::usage::Client>,
    id: u64,
    #[ver(V >= V13_0B4)]
    counter: AtomicU64,
}

#[versions(AGX)]
#[derive(Default)]
pub(crate) struct JobFence {
    id: u64,
    pending: AtomicU64,
    stats: Option<Arc<stats::StatsSnapshot>>,
}

#[versions(AGX)]
impl JobFence::ver {
    fn add_command(self: &FenceObject<Self>) {
        self.pending.fetch_add(1, Ordering::Relaxed);
    }

    fn command_complete(self: &FenceObject<Self>) {
        let remain = self.pending.fetch_sub(1, Ordering::Relaxed) - 1;
        mod_pr_debug!(
            "JobFence[{}]: Command complete (remain: {})\n",
            self.id,
            remain
        );
        if remain == 0 {
            if let Some(stats) = &self.stats {
                stats.note_job();
            }
            mod_pr_debug!("JobFence[{}]: Signaling\n", self.id);
            self.signal();
        }
    }
}

#[versions(AGX)]
#[vtable]
impl dma_fence::FenceOps for JobFence::ver {
    fn get_driver_name<'a>(self: &'a FenceObject<Self>) -> &'a CStr {
        c_str!("aurora")
    }
    fn get_timeline_name<'a>(self: &'a FenceObject<Self>) -> &'a CStr {
        c_str!("queue")
    }
}

#[versions(AGX)]
pub(crate) struct QueueJob {
    dev: AsahiDevRef,
    vm_bind: mmu::VmBind,
    op_guard: Option<gpu::OpGuard>,
    sj_vtx: Option<SubQueueJob::ver>,
    sj_frag: Option<SubQueueJob::ver>,
    sj_comp: Option<SubQueueJob::ver>,
    fence: UserFence<JobFence::ver>,
    notifier: Arc<GpuObject<fw::event::Notifier::ver>>,
    notification_count: u32,
    /// Zero until the ioctl tries publication, one for success, or a negative
    /// errno for a failed attempt. The scheduler consumes the recorded result.
    inline_result: i32,
    /// For a job with a render: the TVB slot init it relies on and whether it reached the firmware.
    tvb: Option<Arc<TvbJob>>,
    id: u64,
}

pub(crate) struct QueueTimeout {
    dev: AsahiDevRef,
    id: u64,
}

#[versions(AGX)]
impl QueueJob::ver {
    fn get_vtx(&mut self) -> Result<&mut workqueue::Job::ver> {
        self.sj_vtx
            .as_mut()
            .ok_or_else(|| {
                cls_pr_debug!(Errors, "No vertex queue\n");
                EINVAL
            })?
            .get()
    }
    fn get_frag(&mut self) -> Result<&mut workqueue::Job::ver> {
        self.sj_frag
            .as_mut()
            .ok_or_else(|| {
                cls_pr_debug!(Errors, "No fragment queue\n");
                EINVAL
            })?
            .get()
    }
    fn get_comp(&mut self) -> Result<&mut workqueue::Job::ver> {
        self.sj_comp
            .as_mut()
            .ok_or_else(|| {
                cls_pr_debug!(Errors, "No compute queue\n");
                EINVAL
            })?
            .get()
    }

    fn commit(&mut self) -> Result {
        mod_dev_dbg!(self.dev, "QueueJob {}: Committing\n", self.id);

        self.sj_vtx.as_mut().map(|a| a.commit()).unwrap_or(Ok(()))?;
        self.sj_frag
            .as_mut()
            .map(|a| a.commit())
            .unwrap_or(Ok(()))?;
        self.sj_comp.as_mut().map(|a| a.commit()).unwrap_or(Ok(()))
    }

    fn can_submit_inline(&self) -> bool {
        self.sj_vtx
            .as_ref()
            .is_none_or(|job| job.can_submit_inline())
            && self
                .sj_frag
                .as_ref()
                .is_none_or(|job| job.can_submit_inline())
            && self
                .sj_comp
                .as_ref()
                .is_none_or(|job| job.can_submit_inline())
    }
}

#[versions(AGX)]
impl sched::JobImpl for QueueJob::ver {
    type TimeoutData = QueueTimeout;
    const MODULE: Option<&'static kernel::ThisModule> =
        Some(kernel::module::this_module::<crate::LocalModule>());

    fn timeout_data(&self) -> QueueTimeout {
        QueueTimeout {
            dev: self.dev.clone(),
            id: self.id,
        }
    }

    fn prepare(job: &mut Self) -> Option<Fence> {
        mod_dev_dbg!(job.dev, "QueueJob {}: Checking runnability\n", job.id);

        if let Some(sj) = job.sj_vtx.as_ref() {
            if let Some(fence) = sj.can_submit() {
                mod_dev_dbg!(
                    job.dev,
                    "QueueJob {}: Blocking due to vertex queue full\n",
                    job.id
                );
                return Some(fence);
            }
        }
        if let Some(sj) = job.sj_frag.as_ref() {
            if let Some(fence) = sj.can_submit() {
                mod_dev_dbg!(
                    job.dev,
                    "QueueJob {}: Blocking due to fragment queue full\n",
                    job.id
                );
                return Some(fence);
            }
        }
        if let Some(sj) = job.sj_comp.as_ref() {
            if let Some(fence) = sj.can_submit() {
                mod_dev_dbg!(
                    job.dev,
                    "QueueJob {}: Blocking due to compute queue full\n",
                    job.id
                );
                return Some(fence);
            }
        }
        None
    }

    #[allow(unused_assignments)]
    fn run(job: &mut Self) -> Result<Option<dma_fence::Fence>> {
        if job.inline_result != 0 {
            return if job.inline_result == 1 {
                Ok(Some(Fence::from_fence(&job.fence)))
            } else {
                Err(Error::from_errno(job.inline_result))
            };
        }

        // A render that relies on a slot init which never reached the firmware would tile with a
        // stale slot. Fail it before publishing anything; the next submission re-initialises.
        if job.tvb.as_ref().is_some_and(|tvb| tvb.init_lost()) {
            cls_pr_debug!(
                Errors,
                "QueueJob {}: Cancelled, its TVB slot init never reached the firmware\n",
                job.id
            );
            return Err(ECANCELED);
        }

        mod_dev_dbg!(job.dev, "QueueJob {}: Running Job\n", job.id);

        // We can only increase the notifier threshold here, now that we are
        // actually running the job. We cannot increase it while queueing the
        // job without introducing subtle race conditions. Suppose we did, as
        // early versions of drm/asahi did:
        //
        // 1. When processing the ioctl submit, a job is queued to drm_sched.
        //    Incorrectly, the notifier threshold is increased, gating firmware
        //    events.
        // 2. When DRM schedules an event, the hardware is kicked.
        // 3. When the number of processed jobs equals the threshold, the
        //    firmware signals the complete event to the kernel
        // 4. When the kernel gets a complete event, we signal the out-syncs.
        //
        // Does that work? There are a few scenarios.
        //
        // 1. There is nothing else ioctl submitted before the job completes.
        //    The job is scheduled, completes, and signals immediately.
        //    Everything works.
        // 2. There is nontrivial sync across different queues. Since each queue
        //    has a separate own notifier threshold, submitting one does not
        //    block scheduling of the other. Everything works the way you'd
        //    expect. drm/sched handles the wait/signal ordering.
        // 3. Two ioctls are submitted back-to-back. The first signals a fence
        //    that the second waits on. Due to the notifier threshold increment,
        //    the first job's completion event is deferred. But in good
        //    conditions, drm/sched will schedule the second submit anyway
        //    because it kills the pointless intra-queue sync. Then both
        //    commands execute and are signalled together.
        // 4. Two ioctls are submitted back-to-back as above, but conditions are
        //    bad. Reporting completion of the first job is still masked by the
        //    notifier threshold, but the intra-queue fences are not optimized
        //    out in drm/sched... drm/sched doesn't schedule the second job
        //    until the first is signalled, but the first isn't signalled until
        //    the second is completed, but the second can't complete until it's
        //    scheduled. We hang!
        //
        // In good conditions, everything works properly and/or we win the race
        // to mask the issue. So the issue here is challenging to hit.
        // Nevertheless, we do need to get it right.
        //
        // The intention with drm/sched is that jobs that are not yet scheduled
        // are "invisible" to the firmware. Incrementing the notifier threshold
        // earlier than this violates that which leads to circles like the
        // above. Deferring the increment to submit solves the race.
        let dev = job.dev.clone();
        let Some(guard) = dev.registration_guard() else {
            if let Some(tvb) = job.tvb.as_ref() {
                tvb.note_run_failed();
            }
            return Err(ENODEV);
        };

        let ret = guard.registration_data_with(|reg_data| {
            let gpu = match reg_data
                .gpu
                .clone()
                .arc_as_any()
                .downcast::<gpu::GpuManager::ver>()
            {
                Ok(gpu) => gpu,
                Err(_) => {
                    dev_crit!(job.dev.as_ref(), "GpuManager mismatched with QueueJob!\n");
                    return Err(EIO);
                }
            };

            if job.op_guard.is_none() {
                job.op_guard = Some(gpu.start_op()?);
            }

            // First submit all the commands for each queue. This can fail.

            let mut frag_job = None;
            let mut frag_sub = None;
            if let Some(sj) = job.sj_frag.as_mut() {
                frag_job = sj.job.take();
                if let Some(wqjob) = frag_job.as_mut() {
                    mod_dev_dbg!(job.dev, "QueueJob {}: Submit fragment\n", job.id);
                    frag_sub = Some(wqjob.submit()?);
                }
            }

            let mut vtx_job = None;
            let mut vtx_sub = None;
            if let Some(sj) = job.sj_vtx.as_mut() {
                vtx_job = sj.job.take();
                if let Some(wqjob) = vtx_job.as_mut() {
                    mod_dev_dbg!(job.dev, "QueueJob {}: Submit vertex\n", job.id);
                    vtx_sub = Some(wqjob.submit()?);
                }
            }

            let mut comp_job = None;
            let mut comp_sub = None;
            if let Some(sj) = job.sj_comp.as_mut() {
                comp_job = sj.job.take();
                if let Some(wqjob) = comp_job.as_mut() {
                    mod_dev_dbg!(job.dev, "QueueJob {}: Submit compute\n", job.id);
                    comp_sub = Some(wqjob.submit()?);
                }
            }

            // Prepared submissions still hold their workqueue locks and have not
            // published cpu_wptr. Only count them after every fallible submit()
            // succeeded, before the first run_job exposes commands to firmware.
            job.notifier.threshold.with(|raw, _inner| {
                raw.increase(job.notification_count);
            });

            // Now we fully commit to running the job. From here on the firmware may see the
            // job's commands (`run_job()` publishes them before it rings the doorbell).
            if let Some(tvb) = job.tvb.as_ref() {
                tvb.published.store(true, Ordering::Release);
            }
            mod_dev_dbg!(job.dev, "QueueJob {}: Run fragment\n", job.id);
            frag_sub.map(|a| gpu.run_job(a)).transpose()?;

            mod_dev_dbg!(job.dev, "QueueJob {}: Run vertex\n", job.id);
            vtx_sub.map(|a| gpu.run_job(a)).transpose()?;

            mod_dev_dbg!(job.dev, "QueueJob {}: Run compute\n", job.id);
            comp_sub.map(|a| gpu.run_job(a)).transpose()?;

            mod_dev_dbg!(job.dev, "QueueJob {}: Drop compute job\n", job.id);
            core::mem::drop(comp_job);
            mod_dev_dbg!(job.dev, "QueueJob {}: Drop vertex job\n", job.id);
            core::mem::drop(vtx_job);
            mod_dev_dbg!(job.dev, "QueueJob {}: Drop fragment job\n", job.id);
            core::mem::drop(frag_job);

            Ok(Some(Fence::from_fence(&job.fence)))
        });
        if ret.is_err() {
            if let Some(tvb) = job.tvb.as_ref() {
                tvb.note_run_failed();
            }
        }
        ret
    }

    fn timed_out(job: &mut QueueTimeout, _finished: bool) -> sched::Status {
        // FIXME: Handle timeouts properly
        dev_err!(
            job.dev.as_ref(),
            "QueueJob {}: Job timed out on the DRM scheduler, things will probably break\n",
            job.id
        );
        sched::Status::NoDevice
    }

    fn cancel(job: &mut Self) {
        dev_info!(
            job.dev.as_ref(),
            "QueueJob {}: Job canceled on DRM scheduler teardown\n",
            job.id
        );
    }
}

#[versions(AGX)]
impl Drop for QueueJob::ver {
    fn drop(&mut self) {
        mod_dev_dbg!(self.dev, "QueueJob {}: Dropping\n", self.id);
    }
}

static QUEUE_NAME: &CStr = c_str!("aurora_fence");
static QUEUE_CLASS_KEY: Pin<&LockClassKey> = kernel::static_lock_class!();

#[versions(AGX)]
impl Queue::ver {
    /// Create a new user queue.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn new(
        dev: AsahiDevice,
        gpu: &dyn GpuManager,
        vm: mmu::Vm,
        alloc: &mut gpu::KernelAllocators,
        ualloc: Arc<Mutex<alloc::DefaultAllocator>>,
        ualloc_priv: Arc<Mutex<alloc::DefaultAllocator>>,
        event_manager: Arc<event::EventManager>,
        stats: Arc<stats::StatsSnapshot>,
        mgr: &buffer::BufferManager::ver,
        id: u64,
        priority: u32,
        usc_exec_base: u64,
        usage: Arc<crate::usage::Client>,
    ) -> Result<Queue::ver> {
        mod_dev_dbg!(dev, "[Queue {}] Creating queue\n", id);

        // Must be shared, no cache management on this one!
        let mut notifier_list = alloc.shared.new_default::<fw::event::NotifierList>()?;

        let self_ptr = notifier_list.weak_pointer();
        notifier_list.with_mut(|raw, _inner| {
            raw.list_head.next = Some(inner_weak_ptr!(self_ptr, list_head));
        });

        let threshold = alloc.shared.new_default::<fw::event::Threshold>()?;

        let notifier: Arc<GpuObject<fw::event::Notifier::ver>> = Arc::new(
            alloc.private.new_init(
                /*try_*/ init!(fw::event::Notifier::ver { threshold }),
                |inner, _p| {
                    try_init!(fw::event::raw::Notifier::ver {
                        threshold: inner.threshold.gpu_pointer(),
                        generation: AtomicU32::new(id as u32),
                        cur_count: AtomicU32::new(0),
                        unk_10: AtomicU32::new(0x50),
                        state: Default::default()
                    })
                },
            )?,
            GFP_KERNEL,
        )?;

        // Priorities are handled by the AGX scheduler, there is no meaning within a
        // per-queue scheduler. Use a single run queue wth Kernel priority.
        let sched = sched::Scheduler::new(
            dev.as_ref().as_ref(),
            1,
            WQ_SIZE,
            0,
            100000,
            c_str!("aurora_sched"),
        )?;
        let entity = sched::Entity::new(&sched, sched::Priority::Kernel)?;

        let buffer = buffer::Buffer::ver::new(gpu, alloc, ualloc.clone(), ualloc_priv, mgr)?;

        let gpu_context = Arc::new(
            workqueue::GpuContext::new(dev.clone(), alloc, buffer.any_ref())?,
            GFP_KERNEL,
        )?;
        // Every command of this queue points the firmware at the notifier, and the notifier links
        // into the notifier list of the work queues. Keep it as long as they are kept, until the
        // context is invalidated, instead of freeing it with the last command.
        gpu_context.retain(KBox::new(notifier.clone(), GFP_KERNEL)?)?;

        let mut ret = Queue::ver {
            dev: dev.clone(),
            _sched: sched,
            entity,
            vm,
            q_vtx: None,
            q_frag: None,
            q_comp: None,
            fence_ctx: FenceContexts::new(1, QUEUE_NAME, QUEUE_CLASS_KEY)?,
            stats,
            last_out: None,
            tvb_users: KVec::new(),
            tvb_init_pending: false,
            tvb_shared: Arc::new(
                TvbShared {
                    lost_init: AtomicU64::new(0),
                },
                GFP_KERNEL,
            )?,
            tvb_seq: 0,
            tvb_init_seq: 0,
            inner: QueueInner::ver {
                dev: dev.clone(),
                ualloc,
                gpu_context,
                buffer,
                notifier_list: Arc::new(notifier_list, GFP_KERNEL)?,
                notifier,
                usc_exec_base,
                usage,
                id,
                #[ver(V >= V13_0B4)]
                counter: AtomicU64::new(0),
            },
        };

        // Rendering structures
        let tvb_blocks = module_parameters::initial_tvb_size.value();

        ret.inner.buffer.ensure_blocks(tvb_blocks)?;

        ret.q_vtx = Some(SubQueue::ver {
            wq: workqueue::WorkQueue::ver::new(
                dev.clone(),
                gpu.get_cfg(),
                alloc,
                event_manager.clone(),
                ret.inner.gpu_context.clone(),
                ret.inner.notifier_list.clone(),
                channel::PipeType::Vertex,
                id,
                priority,
                WQ_SIZE,
            )?,
        });

        ret.q_frag = Some(SubQueue::ver {
            wq: workqueue::WorkQueue::ver::new(
                dev.clone(),
                gpu.get_cfg(),
                alloc,
                event_manager.clone(),
                ret.inner.gpu_context.clone(),
                ret.inner.notifier_list.clone(),
                channel::PipeType::Fragment,
                id,
                priority,
                WQ_SIZE,
            )?,
        });

        // Compute structures
        ret.q_comp = Some(SubQueue::ver {
            wq: workqueue::WorkQueue::ver::new(
                dev.clone(),
                gpu.get_cfg(),
                alloc,
                event_manager,
                ret.inner.gpu_context.clone(),
                ret.inner.notifier_list.clone(),
                channel::PipeType::Compute,
                id,
                priority,
                WQ_SIZE,
            )?,
        });

        mod_dev_dbg!(dev, "[Queue {}] Queue created\n", id);
        Ok(ret)
    }
}

const SQ_RENDER: usize = 0;
const SQ_COMPUTE: usize = 1;
const SQ_COUNT: usize = 2;

/// Bytes of a render command this queue implements. The fields from `ts_frag_end_copies` on
/// belong to other runtimes and must be zero here, as must any bytes beyond the kernel's
/// structure.
const RENDER_SUPPORTED_SIZE: usize =
    core::mem::offset_of!(uapi::drm_asahi_cmd_render, ts_frag_end_copies);
/// Bytes of a compute command this queue implements; see [`RENDER_SUPPORTED_SIZE`].
const COMPUTE_SUPPORTED_SIZE: usize =
    core::mem::offset_of!(uapi::drm_asahi_cmd_compute, ts_end_copies);

// SAFETY: All bit patterns are valid by construction.
unsafe impl AnyBitPattern for uapi::drm_asahi_cmd_header {}
// SAFETY: This UAPI record consists of integer fields and nested integer records.
unsafe impl AnyBitPattern for uapi::drm_asahi_cmd_render {}
// SAFETY: This UAPI record consists of integer fields and nested integer records.
unsafe impl AnyBitPattern for uapi::drm_asahi_cmd_compute {}
// SAFETY: This UAPI record consists of integer fields and nested integer records.
unsafe impl AnyBitPattern for uapi::drm_asahi_attachment {}

fn build_attachments(reader: &mut Reader<'_>, size: usize) -> Result<microseq::Attachments> {
    const STRIDE: usize = core::mem::size_of::<uapi::drm_asahi_attachment>();
    let count = size / STRIDE;

    if count > microseq::MAX_ATTACHMENTS {
        return Err(EINVAL);
    }

    let mut attachments = microseq::Attachments {
        count: count as u32,
        ..Default::default()
    };

    for i in 0..count {
        let att: uapi::drm_asahi_attachment = reader.read()?;

        if att.flags != 0 || att.pad != 0 {
            return Err(EINVAL);
        }

        // Some kind of power-of-2 exponent related to attachment size, in
        // bounds [1, 6]? We don't know what this is exactly yet.
        let unk_e = 1;

        let cache_lines = (att.size + 127) >> 7;
        attachments.list[i] = microseq::Attachment {
            address: U64(att.pointer),
            size: cache_lines.try_into()?,
            unk_c: 0x17,
            unk_e: unk_e as u16,
        };
    }

    Ok(attachments)
}

/// Status of `fence`: 0 while pending, 1 once signalled without error, a negative errno once
/// signalled with an error.
fn fence_status(fence: &Fence) -> i32 {
    // SAFETY: `fence` owns a live dma_fence reference for the duration of the call.
    unsafe { kernel::bindings::dma_fence_get_status(fence.raw()) }
}

/// Waits for `fence` to signal. Returns `Ok(false)` if `timeout` (in jiffies) ran out first, and
/// `ERESTARTSYS` if `interruptible` and a signal arrived.
fn wait_fence(fence: &Fence, interruptible: bool, timeout: kernel::ffi::c_long) -> Result<bool> {
    // SAFETY: `fence` holds a reference for the duration of the call. The callers hold no lock
    // that its completion needs.
    let ret = unsafe { kernel::bindings::dma_fence_wait_timeout(fence.raw(), interruptible, timeout) };
    if ret < 0 {
        Err(Error::from_errno(ret as i32))
    } else {
        Ok(ret > 0)
    }
}

#[versions(AGX)]
impl Queue::ver {
    /// Add a barrier to `queue_job` that waits for `event`.
    fn add_explicit_barrier(
        dev: &AsahiDevRef,
        gpu: &gpu::GpuManager::ver,
        queue_job: &mut workqueue::Job::ver,
        event: &workqueue::QueueEventInfo::ver,
        vm_slot: u32,
        id: u64,
    ) -> Result {
        let mut alloc = gpu.alloc();
        mod_dev_dbg!(dev, "[Submission {}] Create Explicit Barrier\n", id);
        let barrier = alloc.private.new_init(
            pin_init::zeroed::<fw::workqueue::Barrier>(),
            |_inner, _p| {
                let queue_job = &*queue_job;
                try_init!(fw::workqueue::raw::Barrier {
                    tag: fw::workqueue::CommandType::Barrier,
                    wait_stamp: event.fw_stamp_pointer,
                    wait_value: event.value,
                    wait_slot: event.slot,
                    stamp_self: queue_job.event_info().value.next(),
                    uuid: 0xffffbbbb,
                    external_barrier: 0,
                    internal_barrier_type: 1,
                    padding: Default::default(),
                })
            },
        )?;
        mod_dev_dbg!(dev, "[Submission {}] Add Explicit Barrier\n", id);
        queue_job.add(barrier, vm_slot)
    }

    /// Waits, for at most [`QUEUE_CLOSE_WAIT_MS`], for the last submitted job to finish. The
    /// jobs of one queue complete in order, so its finished fence covers all of them.
    fn wait_submitted(&self) {
        let Some(fence) = self.last_out.as_ref() else {
            return;
        };
        let timeout = kernel::time::msecs_to_jiffies(QUEUE_CLOSE_WAIT_MS) as kernel::ffi::c_long;
        if let Ok(false) = wait_fence(fence, false, timeout) {
            dev_warn!(
                self.dev.as_ref(),
                "[Queue {}] Closing with work still running after {} ms\n",
                self.inner.id,
                QUEUE_CLOSE_WAIT_MS
            );
        }
    }

    /// Drops the entries of [`Self::tvb_users`] that can no longer be using the TVB buffer (see
    /// [`prune_tvb_uses`]). Each entry's fence status is read once per call.
    fn prune_tvb_users(&mut self) {
        for user in self.tvb_users.iter_mut() {
            user.seen = TvbUse::from_status(
                fence_status(&user.fence),
                user.job.published.load(Ordering::Acquire),
            );
        }
        prune_tvb_uses(&mut self.tvb_users, |user| user.seen);
    }

    /// Waits until no earlier submission of this queue can still be using its TVB buffer.
    ///
    /// The wait is interruptible and has no time limit of its own: earlier work may legitimately
    /// run long (or wait on its own in-fences), and the scheduler's timeout covers a hung job. A
    /// signal returns `ERESTARTSYS`. Nothing of the current submission is published yet when this
    /// runs, so the ioctl restarts cleanly.
    ///
    /// If the newest such submission failed after some of its work reached the firmware, nothing
    /// will signal when that work retires, so the buffer cannot be proven idle and this returns
    /// `ECANCELED` (the submission that needs the growth fails, the buffer is left alone).
    fn wait_tvb_idle(&mut self) -> Result {
        loop {
            self.prune_tvb_users();
            let Some(user) = self.tvb_users.last() else {
                return Ok(());
            };
            if user.seen == TvbUse::FailedPublished {
                cls_pr_debug!(
                    Errors,
                    "submit: TVB cannot grow: render {} failed after reaching the firmware\n",
                    user.job.seq
                );
                return Err(ECANCELED);
            }
            let fence = user.fence.clone();
            wait_fence(&fence, true, kernel::task::MAX_SCHEDULE_TIMEOUT)?;
        }
    }
}

#[versions(AGX)]
impl Queue for Queue::ver {
    fn drain(&mut self) {
        self.wait_submitted();
    }

    fn submit<'drm>(
        &mut self,
        reg_data: &AsahiRegData<'drm>,
        id: u64,
        mut syncs: KVec<file::SyncItem>,
        in_sync_count: usize,
        cmdbuf_raw: &[u8],
        objects: Pin<&xarray::XArray<KBox<file::Object>>>,
    ) -> Result {
        let gpu = match reg_data
            .gpu
            .clone()
            .arc_as_any()
            .downcast::<gpu::GpuManager::ver>()
        {
            Ok(gpu) => gpu,
            Err(_) => {
                dev_crit!(self.dev.as_ref(), "GpuManager mismatched with JobImpl!\n");
                return Err(EIO);
            }
        };

        mod_dev_dbg!(self.dev, "[Submission {}] Submit job\n", id);

        if gpu.is_crashed() {
            dev_err!(
                self.dev.as_ref(),
                "[Submission {}] GPU is crashed, cannot submit\n",
                id
            );
            return Err(ENODEV);
        }

        let mut cmdbuf = Reader::new(cmdbuf_raw);

        // First, parse the headers to determine the number of compute/render
        // commands. This will be used to determine when to flush stamps.
        //
        // We also use it to determine how many notifications the job will
        // generate. We could calculate that in the second pass since we don't
        // need until much later, but it's convenient to gather everything at
        // the same time.
        let mut nr_commands = 0;
        let mut last_compute = 0;
        let mut last_render = 0;
        let mut nr_render = 0;
        let mut nr_compute = 0;
        let mut max_tvb_blocks = 0usize;

        while !cmdbuf.is_empty() {
            let header: uapi::drm_asahi_cmd_header = cmdbuf.read()?;
            let payload_size = header.size as usize;
            match u32::from(header.cmd_type) {
                uapi::drm_asahi_cmd_type_DRM_ASAHI_CMD_RENDER => {
                    let render: uapi::drm_asahi_cmd_render = cmdbuf.read_up_to(payload_size)?;
                    if render
                        .ts_frag_end_copies
                        .iter()
                        .any(|ts| ts.handle != 0 || ts.offset != 0)
                    {
                        return Err(EINVAL);
                    }
                    max_tvb_blocks = max_tvb_blocks
                        .max(QueueInner::ver::render_min_tvb_blocks(&gpu, &render)?);
                }
                uapi::drm_asahi_cmd_type_DRM_ASAHI_CMD_COMPUTE => {
                    let compute: uapi::drm_asahi_cmd_compute = cmdbuf.read_up_to(payload_size)?;
                    if compute
                        .ts_end_copies
                        .iter()
                        .any(|ts| ts.handle != 0 || ts.offset != 0)
                    {
                        return Err(EINVAL);
                    }
                }
                _ => cmdbuf.skip(payload_size),
            }
            nr_commands += 1;

            match u32::from(header.cmd_type) {
                uapi::drm_asahi_cmd_type_DRM_ASAHI_CMD_RENDER => {
                    last_render = nr_commands;
                    nr_render += 1;
                }
                uapi::drm_asahi_cmd_type_DRM_ASAHI_CMD_COMPUTE => {
                    last_compute = nr_commands;
                    nr_compute += 1;
                }
                _ => {}
            }
        }

        // Validate the number of hardware commands, ignoring software commands
        let nr_hw_commands = nr_render + nr_compute;
        if nr_hw_commands == 0 || nr_hw_commands > MAX_COMMANDS_PER_SUBMISSION {
            cls_pr_debug!(
                Errors,
                "submit: Command count {} out of valid range [1, {}]\n",
                nr_hw_commands,
                MAX_COMMANDS_PER_SUBMISSION - 1
            );
            return Err(EINVAL);
        }

        // The firmware buffer manager owns a TVB buffer while scenes on it are in flight. Growing
        // it then (new blocks in its block list plus an InitBuffer of its bound slot) corrupts the
        // tiling of those scenes: on G13 clustered rendering the tiler writes to VA 0. So size the
        // buffer for every render of this submission before any of it is published, and only once
        // no earlier render of this queue (the only other user of the buffer) can still use it.
        // The size the overflow statistics ask for is folded in, so that growth from them also
        // happens on a busy queue, with one wait per growth step (3x each).
        if nr_render > 0 {
            // The render that carried the newest slot init failed before reaching the firmware,
            // and the renders after it that rely on that init fail in `run()` too. Nothing on the
            // firmware uses the slot since then, so this render initialises it again.
            if init_was_lost(
                self.tvb_shared.lost_init.load(Ordering::Acquire),
                self.tvb_init_seq,
            ) {
                self.tvb_init_pending = true;
            }
            let cur_blocks = self.inner.buffer.block_count() as usize;
            let stats_blocks = self.inner.buffer.stats_target_blocks();
            let want_blocks = max_tvb_blocks.max(stats_blocks.unwrap_or(0));
            if want_blocks > cur_blocks {
                self.wait_tvb_idle()?;
                if self.inner.buffer.ensure_blocks_idle(want_blocks)? {
                    self.tvb_init_pending = true;
                    if stats_blocks.is_some() {
                        self.inner.buffer.reset_stats();
                    }
                    cls_dev_dbg!(
                        TVBStats,
                        &self.dev,
                        "[Submission {}] TVB grew from {} to {} blocks ({} bytes): dimensions need {}, overflows ask {}\n",
                        id,
                        cur_blocks,
                        want_blocks,
                        want_blocks * buffer::BLOCK_SIZE,
                        max_tvb_blocks,
                        stats_blocks.unwrap_or(0),
                    );
                }
            }
            // Room for this submission's entry, so that recording it cannot fail once published.
            self.tvb_users.reserve(1, GFP_KERNEL)?;
        }

        let op_guard = if in_sync_count > 0 {
            Some(gpu.start_op()?)
        } else {
            None
        };

        let mut events: [KVec<Option<workqueue::QueueEventInfo::ver>>; SQ_COUNT] =
            Default::default();

        events[SQ_RENDER].push(
            self.q_frag.as_ref().and_then(|a| a.wq.event_info()),
            GFP_KERNEL,
        )?;
        events[SQ_COMPUTE].push(
            self.q_comp.as_ref().and_then(|a| a.wq.event_info()),
            GFP_KERNEL,
        )?;

        let vm_bind = gpu.bind_vm(&self.vm)?;
        let vm_slot = vm_bind.slot();

        mod_dev_dbg!(self.dev, "[Submission {}] Creating job\n", id);

        // FIXME: I think this can violate the fence seqno ordering contract.
        // If we have e.g. a render submission with no barriers and then a compute submission
        // with no barriers, it's possible for the compute submission to complete first, and
        // therefore its fence. Maybe we should have separate fence contexts for render
        // and compute, and then do a ? (Vert+frag should be fine since there is no vert
        // without frag, and frag always serializes.)
        let fence: UserFence<JobFence::ver> = self
            .fence_ctx
            .new_fence::<JobFence::ver>(
                0,
                JobFence::ver {
                    id,
                    pending: Default::default(),
                    stats: Some(self.stats.clone()),
                },
            )?
            .into();

        let mut init_slot = self.tvb_init_pending;
        let tvb = if nr_render > 0 {
            let seq = self.tvb_seq + 1;
            Some(Arc::new(
                TvbJob {
                    shared: self.tvb_shared.clone(),
                    seq,
                    init_seq: if init_slot { seq } else { self.tvb_init_seq },
                    published: AtomicBool::new(false),
                },
                GFP_KERNEL,
            )?)
        } else {
            None
        };

        let mut job = self.entity.new_job(
            1,
            QueueJob::ver {
                dev: self.dev.clone(),
                vm_bind,
                op_guard,
                sj_vtx: self
                    .q_vtx
                    .as_mut()
                    .map(|a| a.new_job(Fence::from_fence(&fence))),
                sj_frag: self
                    .q_frag
                    .as_mut()
                    .map(|a| a.new_job(Fence::from_fence(&fence))),
                sj_comp: self
                    .q_comp
                    .as_mut()
                    .map(|a| a.new_job(Fence::from_fence(&fence))),
                fence,
                notifier: self.inner.notifier.clone(),

                // Each render command generates 2 notifications: 1 for the
                // vertex part, 1 for the fragment part. Each compute command
                // generates 1 notification. Sum up to calculate the total
                // notification count for the job.
                notification_count: (2 * nr_render) + nr_compute,

                inline_result: 0,

                tvb: tvb.clone(),

                id,
            },
        )?;

        mod_dev_dbg!(
            self.dev,
            "[Submission {}] Adding {} in_syncs\n",
            id,
            in_sync_count
        );
        let mut dependencies_ready = true;
        for sync in syncs.drain(0..in_sync_count) {
            if let Some(fence) = sync.fence {
                // SAFETY: `fence` owns the live dma_fence reference used for
                // this concurrent status query.
                dependencies_ready &=
                    unsafe { kernel::bindings::dma_fence_get_status(fence.raw()) != 0 };
                job.add_dependency(fence)?;
            }
        }

        cmdbuf.rewind();

        let mut command_index = 0;
        let mut vertex_attachments: microseq::Attachments = Default::default();
        let mut fragment_attachments: microseq::Attachments = Default::default();
        let mut compute_attachments: microseq::Attachments = Default::default();

        // Parse the full command buffer submitting as we go
        while !cmdbuf.is_empty() {
            let header: uapi::drm_asahi_cmd_header = cmdbuf.read()?;
            let header_size = header.size as usize;

            // Pre-increment command index to match last_compute/last_render
            command_index += 1;

            // A render's wait for earlier renders is added once its flags are
            // known: without DRM_ASAHI_RENDER_FRAGMENT_BARRIERS it goes on the
            // vertex queue; with it no barrier is added, since the wait names
            // only earlier fragment work, which the fragment queue already runs
            // first, so the vertex half can run beside it. Each fragment half
            // still waits for its own vertex half.
            let is_render =
                u32::from(header.cmd_type) == uapi::drm_asahi_cmd_type_DRM_ASAHI_CMD_RENDER;
            let mut render_wait = None;
            for (queue_idx, index) in [header.vdm_barrier, header.cdm_barrier].iter().enumerate() {
                if *index == uapi::DRM_ASAHI_BARRIER_NONE as u16 {
                    continue;
                }
                if let Some(event) = events[queue_idx].get(*index as usize).ok_or_else(|| {
                    cls_pr_debug!(Errors, "Invalid barrier #{}: {}\n", queue_idx, index);
                    EINVAL
                })? {
                    if is_render && queue_idx == SQ_RENDER {
                        render_wait = Some(*event);
                        continue;
                    }
                    let queue_job = match u32::from(header.cmd_type) {
                        uapi::drm_asahi_cmd_type_DRM_ASAHI_CMD_RENDER => job.get_vtx()?,
                        uapi::drm_asahi_cmd_type_DRM_ASAHI_CMD_COMPUTE => job.get_comp()?,
                        _ => return Err(EINVAL),
                    };
                    Self::add_explicit_barrier(&self.dev, &gpu, queue_job, event, vm_slot, id)?;
                } else {
                    assert!(*index == 0);
                }
            }

            match u32::from(header.cmd_type) {
                uapi::drm_asahi_cmd_type_DRM_ASAHI_CMD_RENDER => {
                    let render: uapi::drm_asahi_cmd_render = cmdbuf
                        .read_supported(header_size, RENDER_SUPPORTED_SIZE)
                        .inspect_err(|_| {
                            cls_pr_debug!(Errors, "Unsupported render command fields\n")
                        })?;

                    // With FRAGMENT_BARRIERS the wait names only earlier fragment work of
                    // this queue (index 0 is its last fragment event, later indices are
                    // renders of this submission), which the fragment workqueue already
                    // runs first, so no barrier is needed at all.
                    let fragment_only = render.flags
                        & uapi::drm_asahi_render_flags_DRM_ASAHI_RENDER_FRAGMENT_BARRIERS
                        != 0;
                    if let Some(event) = render_wait.take().filter(|_| !fragment_only) {
                        Self::add_explicit_barrier(
                            &self.dev,
                            &gpu,
                            job.get_vtx()?,
                            &event,
                            vm_slot,
                            id,
                        )?;
                    }

                    self.inner.submit_render(
                        &gpu,
                        &mut job,
                        &render,
                        &vertex_attachments,
                        &fragment_attachments,
                        objects,
                        id,
                        command_index == last_render,
                        core::mem::take(&mut init_slot),
                    )?;
                    events[SQ_RENDER].push(
                        Some(
                            job.sj_frag
                                .as_ref()
                                .expect("No frag queue?")
                                .job
                                .as_ref()
                                .expect("No frag job?")
                                .event_info(),
                        ),
                        GFP_KERNEL,
                    )?;
                }
                uapi::drm_asahi_cmd_type_DRM_ASAHI_CMD_COMPUTE => {
                    let compute: uapi::drm_asahi_cmd_compute = cmdbuf
                        .read_supported(header_size, COMPUTE_SUPPORTED_SIZE)
                        .inspect_err(|_| {
                            cls_pr_debug!(Errors, "Unsupported compute command fields\n")
                        })?;

                    self.inner.submit_compute(
                        &gpu,
                        &mut job,
                        &compute,
                        &compute_attachments,
                        objects,
                        id,
                        command_index == last_compute,
                    )?;
                    events[SQ_COMPUTE].push(
                        Some(
                            job.sj_comp
                                .as_ref()
                                .expect("No comp queue?")
                                .job
                                .as_ref()
                                .expect("No comp job?")
                                .event_info(),
                        ),
                        GFP_KERNEL,
                    )?;
                }
                uapi::drm_asahi_cmd_type_DRM_ASAHI_SET_VERTEX_ATTACHMENTS => {
                    vertex_attachments = build_attachments(&mut cmdbuf, header_size)?;
                }
                uapi::drm_asahi_cmd_type_DRM_ASAHI_SET_FRAGMENT_ATTACHMENTS => {
                    fragment_attachments = build_attachments(&mut cmdbuf, header_size)?;
                }
                uapi::drm_asahi_cmd_type_DRM_ASAHI_SET_COMPUTE_ATTACHMENTS => {
                    compute_attachments = build_attachments(&mut cmdbuf, header_size)?;
                }
                _ => {
                    cls_pr_debug!(Errors, "Unknown command type {}\n", header.cmd_type);
                    return Err(EINVAL);
                }
            }
        }

        mod_dev_dbg!(
            self.dev,
            "Queue {}: Committing job {}\n",
            self.inner.id,
            job.id
        );
        job.commit()?;

        mod_dev_dbg!(self.dev, "Queue {}: Arming job {}\n", self.inner.id, job.id);
        let mut job = job.arm();
        let out_fence = job.fences().finished();
        let previous_status = self.last_out.as_ref().map(|fence| {
            // SAFETY: `last_out` owns the live dma_fence reference used for
            // this concurrent status query.
            unsafe { kernel::bindings::dma_fence_get_status(fence.raw()) }
        });
        if module_parameters::legacy_inline_submit.value() != 0
            && can_submit_inline(dependencies_ready, previous_status)
            && job.can_submit_inline()
        {
            // All fallible construction and scheduler arming has completed.
            // Regardless of this result the scheduler takes ownership below,
            // preserving timeout, credit, error, and teardown bookkeeping.
            let result = <QueueJob::ver as sched::JobImpl>::run(&mut job).map(|_| ());
            job.inline_result = result.map_or_else(|error| error.to_errno(), |()| 1);
        }
        self.last_out = Some(out_fence.clone());
        mod_dev_dbg!(
            self.dev,
            "Queue {}: Pushing job {}\n",
            self.inner.id,
            job.id
        );
        job.push();

        if let Some(tvb) = tvb {
            self.tvb_seq = tvb.seq;
            if tvb.carries_init() {
                // The first render of the job carries the slot init.
                self.tvb_init_pending = false;
                self.tvb_init_seq = tvb.seq;
            }
            self.prune_tvb_users();
            let user = TvbUser {
                fence: out_fence.clone(),
                job: tvb,
                seen: TvbUse::Pending,
            };
            if self.tvb_users.push_within_capacity(user).is_err() {
                // The space was reserved above and pruning only removes entries.
                unreachable!("push_within_capacity() failed after reserve()");
            }
        }

        mod_dev_dbg!(
            self.dev,
            "Queue {}: Adding {} out_syncs\n",
            self.inner.id,
            syncs.len()
        );
        for mut sync in syncs {
            if let Some(chain) = sync.chain_fence.take() {
                sync.syncobj
                    .add_point(chain, &out_fence, sync.timeline_value);
            } else {
                sync.syncobj.replace_fence(Some(&out_fence));
            }
        }

        Ok(())
    }
}

#[kunit_tests(asahi_legacy_inline_queue)]
mod tests {
    use super::{can_submit_inline, init_was_lost, prune_tvb_uses, TvbUse};
    use kernel::prelude::*;

    use TvbUse::{Done, FailedPublished as FailPub, FailedUnpublished as FailUnpub, Pending};

    fn pruned(uses: &[TvbUse]) -> KVec<TvbUse> {
        let mut v = KVec::new();
        for u in uses {
            v.push(*u, GFP_KERNEL).expect("alloc");
        }
        prune_tvb_uses(&mut v, |u| *u);
        v
    }

    #[test]
    fn tvb_use_from_fence_status() {
        assert_eq!(TvbUse::from_status(0, false), Pending);
        assert_eq!(TvbUse::from_status(0, true), Pending);
        assert_eq!(TvbUse::from_status(1, true), Done);
        assert_eq!(TvbUse::from_status(-125, false), FailUnpub);
        assert_eq!(TvbUse::from_status(-22, true), FailPub);
    }

    #[test]
    fn tvb_clean_completion_proves_older_renders() {
        assert_eq!(&pruned(&[FailPub, Pending, Done, Pending])[..], &[Pending]);
        assert!(pruned(&[Pending, Done]).is_empty());
    }

    #[test]
    fn tvb_unpublished_failure_drops_only_itself() {
        assert_eq!(
            &pruned(&[Pending, FailUnpub, Pending])[..],
            &[Pending, Pending]
        );
        assert!(pruned(&[FailUnpub]).is_empty());
    }

    #[test]
    fn tvb_published_failure_stays_until_proven() {
        // Never dropped on its own, and the newest one stands in for the older ones.
        assert_eq!(&pruned(&[FailPub])[..], &[FailPub]);
        assert_eq!(
            &pruned(&[FailPub, Pending, FailPub, FailUnpub])[..],
            &[Pending, FailPub]
        );
        // A later clean completion proves it done.
        assert!(pruned(&[FailPub, Done]).is_empty());
    }

    #[test]
    fn tvb_lost_init_matches_only_its_carrier() {
        assert!(!init_was_lost(0, 0));
        assert!(!init_was_lost(5, 0));
        assert!(init_was_lost(5, 5));
        assert!(!init_was_lost(5, 7));
    }

    #[test]
    fn depth_192_only_publishes_the_idle_head_inline() {
        const DEPTH: usize = 192;
        let mut previous_status = None;
        let mut inline = 0;

        for _ in 0..DEPTH {
            if can_submit_inline(true, previous_status) {
                inline += 1;
            }
            // Every later ioctl sees the preceding logical queue fence. A
            // burst cannot publish around that unsignalled scheduler owner.
            previous_status = Some(0);
        }

        assert_eq!(inline, 1);
    }

    #[test]
    fn serial_round_trips_stay_eligible() {
        const SUBMITS: usize = 192;

        for index in 0..SUBMITS {
            let previous_status = (index != 0).then_some(1);
            assert!(can_submit_inline(true, previous_status));
        }
    }

    #[test]
    fn unsignalled_dependency_or_predecessor_falls_back() {
        assert!(!can_submit_inline(false, None));
        assert!(!can_submit_inline(true, Some(0)));
        assert!(can_submit_inline(true, Some(-5)));
    }
}

#[versions(AGX)]
impl Drop for Queue::ver {
    fn drop(&mut self) {
        mod_dev_dbg!(self.dev, "[Queue {}] Dropping queue\n", self.inner.id);
        // A client that exits or is killed drops its queues with jobs still queued or running
        // on the GPU, and its VMs right after (see `file::File`). Those VMs do not track jobs:
        // dropping one unmaps the user range at once, and the firmware then faults on the
        // next access of the running job and times out. Let the submitted work finish first.
        self.wait_submitted();
    }
}

/// How long a closing queue waits for its submitted work to finish.
const QUEUE_CLOSE_WAIT_MS: u32 = 5000;
