// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Immutable submissions and the DRM scheduler boundary for independent engines.

use super::{command::Validated, completion::Completion, context::Context};
use crate::driver::AsahiDrmDevice;
use core::sync::atomic::{AtomicBool, AtomicI32, AtomicU8, Ordering};
use kernel::{
    bindings, c_str,
    dma_fence::{Fence, RawDmaFence},
    drm::sched,
    prelude::*,
    sync::Arc,
};

pub(super) const TIMEOUT_MS: kernel::time::Msecs = super::timeout::POLL_MS;
/// One logical render queue owns one physical pair, whose active ring holds
/// `RENDER_DEPTH` commands. A deeper scheduler would only park run_job in
/// admission until a retirement; the scheduler waits for the credit instead.
const RENDER_CREDITS: u32 = crate::hw::t8140::queues::RENDER_DEPTH as u32;
const COMPUTE_CREDITS: u32 = 16;

#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) enum Engine {
    Render,
    Compute,
}

impl Engine {
    pub(crate) fn index(self) -> usize {
        match self {
            Self::Render => 0,
            Self::Compute => 1,
        }
    }
}

/// Logical sequence numbers do not wrap with firmware timestamps. A zero barrier
/// includes preceding ioctls; the sentinel leaves only an explicit historical prefix.
#[derive(Clone, Copy)]
pub(crate) struct Order {
    pub(crate) sequence: u64,
    pub(crate) wait_through: [Option<u64>; 2],
}

impl Order {
    pub(crate) fn new(
        base: [u64; 2],
        preceding: [u16; 2],
        barriers: [u16; 2],
        prior: [u64; 2],
        engine: Engine,
    ) -> Result<Self> {
        let mut sequences = [0; 2];
        let mut wait_through = [None; 2];
        for index in 0..2 {
            // Earlier commands in this ioctl are not historical commands.
            if prior[index] > base[index] {
                return Err(EINVAL);
            }
            sequences[index] = base[index]
                .checked_add(u64::from(preceding[index]))
                .and_then(|value| value.checked_add(1))
                .ok_or(EOVERFLOW)?;
            if barriers[index] != u16::MAX {
                if barriers[index] > preceding[index] {
                    return Err(EINVAL);
                }
                wait_through[index] = Some(
                    base[index]
                        .checked_add(u64::from(barriers[index]))
                        .ok_or(EOVERFLOW)?,
                );
            }
            if prior[index] != 0 {
                wait_through[index] = Some(
                    wait_through[index].map_or(prior[index], |last| last.max(prior[index])),
                );
            }
        }
        Ok(Self {
            sequence: sequences[engine.index()],
            wait_through,
        })
    }

    pub(crate) fn contains(prefix: Option<u64>, sequence: u64) -> bool {
        sequence != 0 && prefix.is_some_and(|last| sequence <= last)
    }
}

pub(crate) struct Packet {
    pub(crate) id: u64,
    pub(crate) context: Arc<Context>,
    pub(crate) command: Validated,
    pub(crate) order: Order,
    pub(crate) completion: Arc<Completion>,
    pub(crate) footprint: Option<Arc<super::hazard::Footprint>>,
    inputs: KVec<Fence>,
    firmware: KVec<Fence>,
    publication: AtomicU8,
    cancelled: AtomicBool,
    early: AtomicI32,
    backlog: AtomicBool,
}

impl Packet {
    pub(crate) fn new(
        id: u64,
        context: Arc<Context>,
        command: Validated,
        order: Order,
        completion: Arc<Completion>,
        inputs: KVec<Fence>,
        firmware: KVec<Fence>,
    ) -> Result<Arc<Self>> {
        let footprint = match &command {
            Validated::Render { attachments, .. } => {
                context.vm().render_footprint(&attachments[1])?
            }
            Validated::Compute { .. } => None,
        };
        Ok(Arc::new(
            Self {
                id,
                context,
                command,
                order,
                completion,
                footprint,
                inputs,
                firmware,
                publication: AtomicU8::new(0),
                cancelled: AtomicBool::new(false),
                early: AtomicI32::new(0),
                backlog: AtomicBool::new(false),
            },
            GFP_KERNEL,
        )?)
    }

    pub(crate) fn engine(&self) -> Engine {
        match &self.command {
            Validated::Render { .. } => Engine::Render,
            Validated::Compute { .. } => Engine::Compute,
        }
    }

    /// Replay holds the device mutex. Its original inputs were satisfied before
    /// first publication; never sleep on a fence that needs event service here.
    pub(crate) fn check_replay_dependencies(&self) -> Result {
        for fence in &self.inputs {
            super::dependency::replay_status(fence_status(fence))?;
        }
        for fence in &self.firmware { check_status(fence)?; }
        self.completion.check_source_inputs()?;
        Ok(())
    }

    /// Scheduler dependency readiness includes failed fences. Consume the actual
    /// status before publication, and retain the references through that check.
    pub(crate) fn check_dependencies(&self) -> Result {
        let polls = if self.command.long_job() { 5 } else { 1 };
        let deadline = super::now_ns().saturating_add(u64::from(TIMEOUT_MS) * polls * 1_000_000);
        for fence in &self.inputs {
            loop {
                if self.cancel_requested() || self.completion.status().get() != 0 {
                    return Err(ECANCELED);
                }
                if fence_status(fence) != 0 { break; }
                if super::now_ns() >= deadline { return Err(ECANCELED); }
                // SAFETY: The retained fence owns the pointer throughout the bounded wait.
                let waited = unsafe {
                    bindings::dma_fence_wait_timeout(
                        fence.raw(),
                        true,
                        kernel::time::msecs_to_jiffies(10) as isize,
                    )
                };
                if waited < 0 {
                    return Err(super::dependency::inherited_error(Error::from_errno(waited as i32)));
                }
                if waited > 0 { break; }
            }
            check_status(fence)?;
        }
        for fence in &self.firmware {
            check_status(fence)?;
        }
        self.completion.check_source_inputs()?;
        Ok(())
    }

    /// Dependency-bearing packets remain on the scheduler publication path.
    pub(crate) fn has_dependencies(&self) -> bool {
        !self.inputs.is_empty() || !self.firmware.is_empty()
    }

    pub(crate) fn begin_publication(&self) -> bool {
        self.publication
            .compare_exchange(0, 1, Ordering::AcqRel, Ordering::Acquire)
            .is_ok()
    }
    /// Gives up a claim taken from the submitting ioctl before the job was
    /// pushed: nothing became visible, and the scheduler claims it again.
    pub(crate) fn abandon_publication(&self) {
        let _ = self
            .publication
            .compare_exchange(1, 0, Ordering::AcqRel, Ordering::Acquire);
    }
    pub(crate) fn mark_published(&self) {
        self.publication.store(2, Ordering::Release);
    }
    pub(crate) fn is_published(&self) -> bool {
        self.publication.load(Ordering::Acquire) == 2
    }
    /// A published group retired by firmware without executing is published
    /// again from the same packet. Publication state returns to "begun".
    pub(crate) fn reset_publication_for_replay(&self) {
        self.publication.store(1, Ordering::Release);
    }
    pub(crate) fn cancel_requested(&self) -> bool {
        self.cancelled.load(Ordering::Acquire)
    }
    pub(crate) fn request_cancel(&self) -> bool {
        self.cancelled.store(true, Ordering::Release);
        self.publication
            .compare_exchange(0, 3, Ordering::AcqRel, Ordering::Acquire)
            .is_ok()
    }
    pub(crate) fn set_early_result(&self, result: Result) {
        self.early.store(
            result.map_or_else(|error| error.to_errno(), |()| 1),
            Ordering::Release,
        );
    }
    pub(crate) fn early_result(&self) -> Option<Result> {
        match self.early.load(Ordering::Acquire) {
            0 => None,
            1 => Some(Ok(())),
            error => Some(Err(Error::from_errno(error))),
        }
    }
    pub(crate) fn count_backlog(&self) {
        self.backlog.store(true, Ordering::Release);
    }
    pub(crate) fn take_backlog(&self) -> bool {
        self.backlog.swap(false, Ordering::AcqRel)
    }
}

pub(crate) fn fence_status(fence: &Fence) -> i32 {
    // SAFETY: The owned fence keeps the raw DMA fence alive for this read.
    unsafe { bindings::dma_fence_get_status(fence.raw()) }
}

fn check_status(fence: &Fence) -> Result {
    let status = fence_status(fence);
    if status < 0 {
        Err(super::dependency::inherited_error(Error::from_errno(status)))
    } else {
        Ok(())
    }
}

/// The backend retains visible packets through exact retirement or processor stop.
/// It performs all completion signalling after releasing the device mutex.
pub(crate) trait Backend: Send + Sync {
    /// Device detector retained by render completions after queue teardown.
    fn input_sources(&self, _inputs: &[crate::file::SyncItem])
        -> Option<Arc<super::sync_input::Sources>> { None }

    fn feed(&self) -> Option<Arc<super::feed::Feed>> {
        None
    }

    /// Validate compute sizing without reserving a physical channel. Queued
    /// commands may wait on a host signal submitted after this ioctl returns.
    fn ensure_compute(&self, scs_pages: u16) -> Result;
    fn render_scs_blocks(&self, pages: [u16; 2]) -> Result<usize> {
        if pages != [0; 2] { return Err(EINVAL); }
        Ok(super::fw::freelist::RENDER_BLOCKS)
    }
    /// Settle outstanding work of a terminal VM before an ioctl error is published.
    fn fail_vm(&self, error: Error);
    /// Called after both scheduler entities have cancelled their queued jobs.
    fn release_owner(&self);
    /// A fence to wait for before `publish`, signalled by the release of the capacity the
    /// packet lacks. Called again once it signals.
    fn prepare(&self, packet: &Arc<Packet>) -> Option<Fence>;
    fn publish(&self, packet: Arc<Packet>) -> Result<Fence>;
    fn timed_out(&self, packet: Arc<Packet>) -> sched::Status;
    fn cancel(&self, packet: Arc<Packet>);
    /// Transfer a timeout to an outstanding off-lock graph transaction, if any.
    fn defer_timeout(&self, packet: &Arc<Packet>, renewals: u8) -> bool;
    /// A long job of another queue is running beside `packet`.
    fn long_job_beside(&self, _packet: &Arc<Packet>) -> bool { false }
    /// Return true only after taking publication ownership and recording its result.
    fn try_early(&self, packet: &Arc<Packet>) -> bool;
    fn count_backlog(&self, packet: &Packet);
}

struct Job<B: Backend> {
    backend: Arc<B>,
    packet: Arc<Packet>,
}

struct Timeout<B: Backend> {
    backend: Arc<B>,
    victim_renewals: u8,
    execution_renewals: u8,
    shared_renewals: u8,
    packet: Arc<Packet>,
}

impl<B: Backend> Drop for Job<B> {
    fn drop(&mut self) {
        // Entity teardown can free a queued job without invoking cancel.
        // Its independent fence may still order another engine. A claimed
        // publisher keeps responsibility for visible work and its VM pin.
        if self.packet.publication.load(Ordering::Acquire) == 0 {
            self.backend.cancel(self.packet.clone());
        }
    }
}

impl<B: Backend> sched::JobImpl for Job<B> {
    type TimeoutData = Timeout<B>;
    const MODULE: Option<&'static kernel::ThisModule> =
        Some(&crate::THIS_MODULE);

    fn timeout_data(&self) -> Timeout<B> {
        Timeout {
            backend: self.backend.clone(),
            victim_renewals: 0,
            execution_renewals: 0,
            shared_renewals: 0,
            packet: self.packet.clone(),
        }
    }

    fn false_timeout(job: &mut Timeout<B>, finished: bool) -> bool {
        if finished || job.packet.completion.take_replay_timeout() {
            return true;
        }
        if super::timeout::renew_execution(
            job.packet.is_published(), job.packet.command.long_job(), &mut job.execution_renewals,
        ) {
            return true;
        }
        // Other work retiring means the GPU is shared, not hung. So does another long job
        // running beside a long job, though neither retires until it ends.
        let shared = super::timeout::retiring(super::recovery::since_retired(super::now_ns()))
            || (job.packet.command.long_job() && job.backend.long_job_beside(&job.packet));
        if super::timeout::renew_shared(job.packet.is_published(), shared, &mut job.shared_renewals) {
            return true;
        }
        let deferred = job.backend.defer_timeout(&job.packet, job.victim_renewals);
        if deferred {
            job.victim_renewals = job.victim_renewals.saturating_add(1);
        }
        deferred
    }

    fn prepare(job: &mut Self) -> Option<Fence> {
        job.backend.prepare(&job.packet)
    }
    fn run(job: &mut Self) -> Result<Option<Fence>> {
        job.backend.publish(job.packet.clone()).map(Some)
    }
    fn timed_out(job: &mut Timeout<B>, finished: bool) -> sched::Status {
        // The scheduler retains the detached job throughout this callback. Keep
        // its parent callback intact while the driver settles the exact owner;
        // reinsertion also handles completion racing with timeout observation.
        if !finished {
            job.backend.timed_out(job.packet.clone());
        }
        sched::Status::NoHang
    }
    fn cancel(job: &mut Self) {
        job.backend.cancel(job.packet.clone());
    }
}

pub(crate) struct Scheduler<B: Backend> {
    // Cancel entity jobs before dropping the scheduler's owning reference.
    entity: sched::Entity<Job<B>>,
    _scheduler: sched::Scheduler<Job<B>>,
    backend: Arc<B>,
}

#[derive(Clone)]
pub(crate) struct Fences {
    pub(crate) ready: Fence,
    pub(crate) completed: Fence,
    pub(crate) publication: Option<Arc<super::completion::Publication>>,
}

impl Fences {
    /// Scheduler::enqueue always supplies our explicit completion fence here.
    /// It has no polling callback, so pending frontier checks need no lock.
    pub(crate) fn status(&self) -> i32 {
        unsafe extern "C" {
            fn asahi_g17_completion_fence_status(fence: *mut bindings::dma_fence) -> i32;
        }
        // SAFETY: completed owns the live private fence throughout this read.
        unsafe { asahi_g17_completion_fence_status(self.completed.raw()) }
    }
}

impl<B: Backend> Scheduler<B> {
    pub(crate) fn new(dev: &AsahiDrmDevice, backend: Arc<B>, engine: Engine) -> Result<Self> {
        let (credits, name) = match engine {
            Engine::Render => (RENDER_CREDITS, c_str!("asahi_g17_render")),
            Engine::Compute => (COMPUTE_CREDITS, c_str!("asahi_g17_compute")),
        };
        let scheduler = sched::Scheduler::new(dev.as_ref().as_ref(), 1, credits, 0, TIMEOUT_MS, name)?;
        let entity = sched::Entity::new(&scheduler, sched::Priority::Kernel)?;
        Ok(Self {
            entity,
            _scheduler: scheduler,
            backend,
        })
    }

    pub(crate) fn enqueue(
        &mut self,
        packet: Arc<Packet>,
        dependencies: KVec<Fence>,
        early: bool,
        publication: Option<super::completion::PreparedPublication>,
    ) -> Result<Fences> {
        let completed = packet.completion.fence();
        let host_timestamps = packet.completion.has_timestamps();
        // Only commands which actually write timestamps become host-prefix
        // producers. Ordinary consumers retain the scheduled ready fence and
        // their existing firmware dependency path for cross-engine overlap.
        let host_publication = if host_timestamps {
            packet.completion.publication()
        } else {
            None
        };
        let mut job = self.entity.new_job(
            1,
            Job {
                backend: self.backend.clone(),
                packet: packet.clone(),
            },
        )?;
        for dependency in dependencies {
            job.add_dependency(dependency)?;
        }
        let mut job = job.arm();
        packet.completion.mark_accepted();
        if let Some(publication) = publication {
            publication.activate();
        }
        let ready = if host_timestamps {
            completed.clone()
        } else {
            job.fences().scheduled()
        };
        // All fallible scheduler setup precedes early hardware publication. The
        // scheduler still owns watchdog, credits and cancellation after push.
        if !(early && self.backend.try_early(&packet)) {
            if packet.completion.needs_source_ready() {
                packet.completion.set_source_ready(ready.clone());
            }
            self.backend.count_backlog(&packet);
        }
        job.push();
        Ok(Fences { ready, completed, publication: host_publication })
    }
}
