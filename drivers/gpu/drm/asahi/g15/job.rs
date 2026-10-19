// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Jobs: one per SUBMIT, scheduled by a per-queue DRM scheduler and executed by the engine.
//!
//! - One `drm_sched` scheduler and entity per logical queue ([`Scheduler`], credit limit
//!   [`super::hw::sizes::HORIZON_JOBS`] = 16, one credit per job); every input sync is a
//!   dependency of the whole job (PER_COMMAND_INPUT_SYNCS holds a fortiori). The host priority
//!   lives in [`super::queue::QueueState::priority`] for the engine's pick: with one entity per
//!   scheduler, the entity priority orders nothing.
//! - Timeout [`super::hw::tuning::SCHED_TIMEOUT_MS`], always "no hang": the engine's
//!   progress-based deadlines own failure.
//! - Dependency errors from another VM do not fail a job (per-client isolation); an input error
//!   fails the job only if its own VM already has an error.
//! - `run` hands the job to the engine ([`super::Firmware::enqueue`], then
//!   [`super::Firmware::publish`]) under the device lock and returns the job's fence; FINISHED
//!   fences stay in queue order ([`super::fence::Timeline`]).
//! - A failed job latches its VM error before its fence signals ([`Job::complete`]).
//!
//! # For the engine
//!
//! The engine owns an `Arc<Job>` from [`super::Firmware::enqueue`] on. It completes the job
//! exactly once with [`Job::complete`] (later calls are ignored), through [`Completions`]
//! collected under the device lock and [`complete_all`] after it is dropped. A job whose storage
//! the firmware may still reference stays in `Engine::retained` (failed jobs until reboot): its
//! [`mmu::VmJobGuard`] keeps the VM's mappings and its VM state keeps the VM's TTBAT slot until
//! the last `Arc<Job>` drops.

use core::sync::atomic::{AtomicBool, Ordering};
use kernel::{
    bindings, c_str, device,
    dma_fence::{Fence, RawDmaFence},
    drm::sched,
    prelude::*,
    sync::Arc,
};

use super::{command::Command, fence::JobFence, hw, queue::QueueState, vm::VmState, Shared};
use crate::mmu;

/// A job as the engine sees it.
pub(crate) struct Job {
    /// Submission ID (debugging).
    pub(crate) id: u64,
    /// The submitting queue (priority, service turn, horizon history).
    pub(crate) queue: Arc<QueueState>,
    /// The job's VM (TTBAT slot, status).
    pub(crate) vm: Arc<VmState>,
    /// Validated commands, in submission order (at most 64).
    pub(crate) commands: KVec<Command>,
    /// The job's hardware fence.
    pub(crate) fence: JobFence,
    /// The engine's bookkeeping; `progress.signaled` makes [`Job::complete`] act once.
    pub(crate) progress: super::engine::Progress,
    /// The timestamp buffers the commands name (their arena VAs stay mapped while the job lives).
    _timestamps: KVec<Arc<mmu::KernelMapping>>,
    /// Retains the VM's mappings (deferred unmaps, deferred VM close) while the firmware may
    /// access them.
    _guard: mmu::VmJobGuard,
    /// Pushed to the scheduler: from now on the job counts as submitted work of its VM.
    accepted: AtomicBool,
    /// Handed to the engine (or refused at `run`): the scheduler no longer cancels it.
    started: AtomicBool,
}

impl Job {
    pub(crate) fn new(
        id: u64,
        queue: Arc<QueueState>,
        vm: Arc<VmState>,
        commands: KVec<Command>,
        fence: JobFence,
        timestamps: KVec<Arc<mmu::KernelMapping>>,
        guard: mmu::VmJobGuard,
    ) -> Result<Arc<Self>> {
        let progress = super::engine::Progress::new(commands.len());
        Ok(Arc::new(
            Self {
                id,
                queue,
                vm,
                commands,
                fence,
                progress,
                _timestamps: timestamps,
                _guard: guard,
                accepted: AtomicBool::new(false),
                started: AtomicBool::new(false),
            },
            GFP_KERNEL,
        )?)
    }

    /// Completes the job: latches the VM status on failure, then signals the fence (after every
    /// earlier job of the queue). Called by the engine, without the device lock. Only the first
    /// call has an effect.
    ///
    /// Any failure makes the VM's error sticky: later SUBMITs, QUEUE_CREATEs and mapping changes
    /// of the VM are refused (EIO), and mapping changes stay blocked while the job lives (the
    /// engine retains a job the firmware may still reference).
    #[track_caller]
    pub(crate) fn complete(&self, result: Result) {
        self.finish(result, true);
    }

    /// Completes the job; `engine`: the engine had it, so a failure quarantines its mappings.
    /// ECANCELED (never-run work at queue teardown) is only reported, not sticky.
    #[track_caller]
    fn finish(&self, result: Result, engine: bool) {
        if self.progress.signaled.swap(true, Ordering::AcqRel) {
            return;
        }
        if let Err(error) = result {
            if error == ECANCELED {
                self.vm.status.report_failure(error);
            } else {
                if engine {
                    // Before the error fence becomes visible (G17's rule).
                    self._guard.quarantine();
                }
                self.vm.status.record(error);
            }
        }
        self.fence.signal(result);
    }

    /// Hands the job to the engine, or completes it when the GPU failed or the job's own VM
    /// failed with an input error.
    fn start(self: &Arc<Self>, shared: &Arc<Shared>, inputs: &[Fence]) {
        self.started.store(true, Ordering::Release);
        if shared.crashed.load(Ordering::Acquire) {
            self.finish(Err(EIO), false);
            return;
        }
        if self.vm.status.get() != 0 {
            // SAFETY: Each retained fence owns its pointer for this read.
            let failed = inputs
                .iter()
                .map(|fence| unsafe { bindings::dma_fence_get_status(fence.raw()) })
                .find(|&status| status < 0);
            if let Some(status) = failed {
                self.finish(Err(Error::from_errno(status)), false);
                return;
            }
        }
        // `Shared::submit`: enqueue, publish and rearm under the device lock,
        // completions after it.
        if let Err(error) = shared.submit(self.clone()) {
            // An engine that refuses work is a device failure for the job's VM.
            self.finish(Err(if error == ENODEV { EIO } else { error }), false);
        }
    }

    /// Cancels a job the scheduler never ran (queue teardown, or a failed submission).
    fn cancel_unstarted(&self) {
        if self.started.swap(true, Ordering::AcqRel) {
            return;
        }
        if self.accepted.load(Ordering::Acquire) {
            self.finish(Err(ECANCELED), false);
        } else if !self.progress.signaled.swap(true, Ordering::AcqRel) {
            // A refused submission's job: its fence orders nothing and was never published.
            self.fence.signal(Err(ECANCELED));
        }
    }
}

/// Jobs retired or failed under the device lock, completed after it is dropped.
pub(crate) type Completions = KVec<(Arc<Job>, Result)>;

/// Completes jobs in order.
#[track_caller]
pub(crate) fn complete_all(done: Completions) {
    for (job, result) in done {
        job.complete(result);
    }
}

/// The scheduler's view of a job.
struct SchedJob {
    job: Arc<Job>,
    shared: Arc<Shared>,
    /// The input fences, kept for the dependency-error check at `run`.
    inputs: KVec<Fence>,
}

impl sched::JobImpl for SchedJob {
    type TimeoutData = ();

    const MODULE: Option<&'static kernel::ThisModule> = Some(&crate::THIS_MODULE);

    fn timeout_data(&self) {}

    fn run(job: &mut Self) -> Result<Option<Fence>> {
        let fence = job.job.fence.fence();
        job.job.start(&job.shared, &job.inputs);
        Ok(Some(fence))
    }

    /// The engine's own deadlines own failure; late delivery is not a hang.
    fn timed_out(_data: &mut (), _finished: bool) -> sched::Status {
        sched::Status::NoHang
    }

    fn cancel(job: &mut Self) {
        job.job.cancel_unstarted();
    }
}

impl Drop for SchedJob {
    fn drop(&mut self) {
        // Entity teardown frees jobs it never ran without calling `cancel`.
        self.job.cancel_unstarted();
    }
}

/// The scheduler and entity of one logical queue.
pub(crate) struct Scheduler {
    // Dropped first: the entity flush waits until every queued job has run (QUEUE_DESTROY
    // drains), and only then the scheduler is finalized.
    entity: sched::Entity<SchedJob>,
    _scheduler: sched::Scheduler<SchedJob>,
}

impl Scheduler {
    pub(crate) fn new(dev: &device::Device) -> Result<Self> {
        let scheduler = sched::Scheduler::new(
            dev,
            1,
            hw::sizes::HORIZON_JOBS,
            0,
            hw::tuning::SCHED_TIMEOUT_MS,
            c_str!("asahi_g15"),
        )?;
        // With one run queue, only the kernel priority is valid.
        let entity = sched::Entity::new(&scheduler, sched::Priority::Kernel)?;
        Ok(Self {
            entity,
            _scheduler: scheduler,
        })
    }

    /// Pushes a job that waits for `inputs`. On failure the job is cancelled (not counted as
    /// submitted work of its VM) and the error is returned.
    pub(crate) fn push(
        &mut self,
        job: Arc<Job>,
        shared: Arc<Shared>,
        inputs: KVec<Fence>,
    ) -> Result {
        let mut dependencies = KVec::with_capacity(inputs.len(), GFP_KERNEL)?;
        for fence in inputs.iter() {
            dependencies.push(fence.clone(), GFP_KERNEL)?;
        }
        let handle = job.clone();
        let mut pending = self.entity.new_job(
            1,
            SchedJob {
                job,
                shared,
                inputs,
            },
        )?;
        for fence in dependencies {
            pending.add_dependency(fence)?;
        }
        let armed = pending.arm();
        handle.accepted.store(true, Ordering::Release);
        armed.push();
        Ok(())
    }
}
