// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Per-command fences, timestamp destinations and VM pins retained through retirement.
//!
//! Runtime queues own completions before making work visible. They defer fence signalling
//! until after the device mutex is released, and retain failed visible work until a retirement
//! witness or processor stop. An error fence never substitutes for that ownership boundary.

use super::{
    context::{Context, WorkStateLease},
    fence::{self, CompletionFence, Member},
    object::{CpuMap, KernelObject},
    recovery::Class,
    status::VmStatus,
};
use crate::{driver::AsahiDrmDevice, file, hw::t8140::{dynamic, resources as cfg}, mmu};
use core::sync::atomic::{fence as barrier, AtomicBool, AtomicI32, AtomicU8, Ordering};
use kernel::{
    dma_fence::{Fence, FenceContexts, RawDmaFence, UserFence},
    new_mutex,
    prelude::*,
    sync::{Arc, Mutex},
    uapi, xarray,
};

mod publication;
pub(crate) use publication::{Prepared as PreparedPublication, Publication};

struct Destination {
    // The buffer keeps its GPU alias and its CPU mapping alive.
    buffer: Arc<crate::timestamp::Buffer>,
    offset: usize,
}

impl Destination {
    fn resolve(
        objects: Pin<&xarray::XArray<KBox<file::Object>>>,
        timestamp: uapi::drm_asahi_timestamp,
    ) -> Result<Option<Self>> {
        if timestamp.handle == 0 {
            return Ok(None);
        }
        let object = objects
            .lock()
            .get(timestamp.handle.try_into()?)
            .ok_or(ENOENT)?
            .clone();
        let file::Object::TimestampBuffer(buffer) = object;
        let offset: usize = timestamp.offset.try_into()?;
        if offset.checked_add(size_of::<u64>()).ok_or(EINVAL)? > buffer.size() {
            return Err(ERANGE);
        }
        let offset = buffer.timestamp_offset(offset)?;
        Ok(Some(Self { buffer, offset }))
    }

    fn write(&self, ticks: u64) {
        const NSEC_PER_SEC: u64 = 1_000_000_000;
        let ns = (ticks / cfg::COMMAND_TIMESTAMP_HZ)
            .wrapping_mul(NSEC_PER_SEC)
            .wrapping_add(
                ticks % cfg::COMMAND_TIMESTAMP_HZ * NSEC_PER_SEC / cfg::COMMAND_TIMESTAMP_HZ,
            );
        self.buffer.write_timestamp(self.offset, ns);
        barrier(Ordering::Release);
    }
}

/// Timestamp destinations resolved and retained before any command is enqueued. Dropping
/// one releases only a buffer reference; the buffer defers its own mapping release.
pub(crate) struct Destinations([Option<Destination>; 4]);

impl Destinations {
    pub(crate) fn resolve(
        objects: Pin<&xarray::XArray<KBox<file::Object>>>,
        timestamps: &[uapi::drm_asahi_timestamps],
    ) -> Result<Self> {
        if timestamps.is_empty() || timestamps.len() > 2 {
            return Err(EINVAL);
        }
        let mut destinations = core::array::from_fn(|_| None);
        for (index, timestamp) in timestamps.iter().enumerate() {
            destinations[2 * index] = Destination::resolve(objects, timestamp.start)?;
            destinations[2 * index + 1] = Destination::resolve(objects, timestamp.end)?;
        }
        Ok(Self(destinations))
    }

    pub(crate) fn has_outputs(&self) -> bool { self.0.iter().any(Option::is_some) }

    fn iter(&self) -> core::slice::Iter<'_, Option<Destination>> {
        self.0.iter()
    }
}

/// Public outputs do not own physical command resources or a scheduler job.
struct Output {
    fence: UserFence<CompletionFence>,
    member: Member,
    destinations: Destinations,
}

impl Output {
    fn publish(&self, result: Result<[u64; 4]>) {
        match result {
            Ok(ticks) => {
                for (destination, ticks) in self.destinations.iter().zip(ticks) {
                    if let Some(destination) = destination {
                        destination.write(ticks);
                    }
                }
            }
            Err(error) => self.fence.set_error(error),
        }
        self.fence.signal();
        self.member.complete(result.map(|_| ()));
    }
}

enum HostOutput {
    Immediate(Output),
    Ordered(Arc<Publication>),
}

impl HostOutput {
    fn output(&self) -> &Output {
        match self {
            Self::Immediate(output) => output,
            Self::Ordered(publication) => publication.output(),
        }
    }

    fn finish(&self, result: Result<[u64; 4]>) {
        match self {
            Self::Immediate(output) => output.publish(result),
            Self::Ordered(publication) => publication.finish(result),
        }
    }
}

/// Firmware writes two aligned words in a write-combined page of its own per
/// compute command.
struct ComputeTimestamps {
    // Remove the GPU alias before releasing its backing.
    mapping: mmu::KernelMapping,
    object: KernelObject,
}

impl ComputeTimestamps {
    fn new(dev: &AsahiDrmDevice, vm: &mmu::Vm) -> Result<Self> {
        let object = KernelObject::backing(dev, mmu::UAT_PGSZ, CpuMap::WriteCombined)?;
        let mapping = object.alias_in(
            vm,
            // Canonical shared queue aliases may be installed in this VM later.
            // Private timestamp pages must never occupy their retained addresses.
            dynamic::CLIENT_LOWER,
            mmu::UAT_PGSZ as u64,
            mmu::PROT_GPU_FW_SHARED_RW,
        )?;
        Ok(Self { mapping, object })
    }
}

/// Compute timestamp pages of one context, kept mapped for its next commands.
///
/// A fresh page costs a GEM object, its write-combined kernel mapping and a
/// GPU mapping on the submission path, and their teardown when the command's
/// completion is dropped. A page comes back here only from that drop, which
/// follows exact retirement or a kill witness, so the firmware no longer
/// writes it; both words are cleared before the page is handed out again.
#[pin_data]
pub(crate) struct ComputeTimestampCache {
    #[pin]
    pages: Mutex<KVec<ComputeTimestamps>>,
}

impl ComputeTimestampCache {
    /// Pages retained per context; deeper pipelines allocate the rest.
    const RETAINED: usize = 8;

    pub(crate) fn new() -> Result<Arc<Self>> {
        let pages = KVec::with_capacity(Self::RETAINED, GFP_KERNEL)?;
        Arc::pin_init(
            pin_init!(Self {
                pages <- new_mutex!(pages, "G17 compute timestamp pages"),
            }),
            GFP_KERNEL,
        )
    }

    fn take(self: &Arc<Self>, dev: &AsahiDrmDevice, vm: &mmu::Vm) -> Result<PooledTimestamps> {
        let reused = self.pages.lock().pop();
        let page = match reused {
            Some(page) => {
                page.object.dword(0)?.store(0, Ordering::Relaxed);
                page.object.dword(8)?.store(0, Ordering::Relaxed);
                barrier(Ordering::SeqCst);
                page
            }
            None => ComputeTimestamps::new(dev, vm)?,
        };
        Ok(PooledTimestamps {
            page: Some(page),
            cache: self.clone(),
        })
    }

    fn put(&self, page: ComputeTimestamps) {
        let rejected = {
            let mut pages = self.pages.lock();
            pages.push_within_capacity(page).err()
        };
        // Teardown of a page beyond the retained count runs unlocked.
        drop(rejected);
    }
}

/// A compute timestamp page lent to one command, returned to its context's
/// cache when the command's completion is dropped.
struct PooledTimestamps {
    page: Option<ComputeTimestamps>,
    cache: Arc<ComputeTimestampCache>,
}

impl PooledTimestamps {
    fn page(&self) -> Result<&ComputeTimestamps> {
        self.page.as_ref().ok_or(EINVAL)
    }
}

impl Drop for PooledTimestamps {
    fn drop(&mut self) {
        if let Some(page) = self.page.take() {
            self.cache.put(page);
        }
    }
}

/// Republications of one command whose earlier publications firmware dropped unexecuted.
pub(crate) const MAX_REPLAYS: u8 = 3;

#[pin_data]
pub(crate) struct Completion {
    status: Arc<VmStatus>,
    output: HostOutput,
    // Present only when this command's public result may wait for a render
    // predecessor's host writes. DRM credits must follow the terminal result.
    terminal: Option<UserFence<CompletionFence>>,
    compute: Option<PooledTimestamps>,
    feed: Option<Arc<super::feed::Feed>>,
    sources: Option<Arc<super::sync_input::Sources>>,
    #[pin]
    vm_job: Mutex<Option<mmu::VmJobGuard>>,
    #[pin]
    render: Mutex<Option<super::queue::admission::RenderLease>>,
    signalled: AtomicBool,
    retirement_queued: AtomicBool,
    accepted: AtomicBool,
    timeout_error: AtomicI32,
    replays: AtomicU8,
    replay_timeout: AtomicBool,
    /// A recovery blamed this command. Unlike `class`, which a later
    /// classification can change, this never clears.
    blamed: AtomicBool,
    /// A firmware restart listed this command as in flight. Until a replay
    /// republishes it from the start, it never reports success.
    interrupted: AtomicBool,
    firmware_consumer: AtomicBool,
    class: AtomicU8,
}

impl Completion {
    #[expect(
        clippy::too_many_arguments,
        reason = "Completion construction takes distinct queue, context, and lifetime owners."
    )]
    pub(crate) fn new(
        dev: &AsahiDrmDevice,
        contexts: &FenceContexts,
        context: &Arc<Context>,
        member: Member,
        vm_job: mmu::VmJobGuard,
        destinations: Destinations,
        compute: bool,
        host_predecessors: bool,
        feed: Option<Arc<super::feed::Feed>>,
        sources: Option<Arc<super::sync_input::Sources>>,
    ) -> Result<Arc<Self>> {
        let fence = fence::independent(contexts)?;
        let ordered = !compute
            && (host_predecessors || destinations.iter().any(Option::is_some));
        let terminal = if host_predecessors {
            Some(fence::independent(contexts)?)
        } else {
            None
        };
        let output = Output { fence, member, destinations };
        let output = if ordered {
            HostOutput::Ordered(Publication::new(output, context.status().clone())?)
        } else {
            HostOutput::Immediate(output)
        };
        let compute = if compute {
            Some(context.compute_timestamps().take(dev, context.vm())?)
        } else {
            None
        };
        Arc::pin_init(
            pin_init!(Self {
                status: context.status().clone(),
                output,
                terminal,
                compute,
                feed,
                sources,
                vm_job <- new_mutex!(Some(vm_job), "G17 command VM pin"),
                render <- new_mutex!(None, "G17 command render lease"),
                signalled: AtomicBool::new(false),
                retirement_queued: AtomicBool::new(false),
                accepted: AtomicBool::new(false),
                timeout_error: AtomicI32::new(0),
                replays: AtomicU8::new(0),
                replay_timeout: AtomicBool::new(false),
                blamed: AtomicBool::new(false),
                interrupted: AtomicBool::new(false),
                firmware_consumer: AtomicBool::new(false),
                class: AtomicU8::new(0),
            }),
            GFP_KERNEL,
        )
    }

    pub(crate) fn fence(&self) -> Fence {
        Fence::from_fence(&self.output.output().fence)
    }
    pub(crate) fn scheduler_fence(&self) -> Fence {
        self.terminal.as_ref().map_or_else(|| self.fence(), |fence| Fence::from_fence(fence))
    }
    pub(crate) fn publication(&self) -> Option<Arc<Publication>> {
        match &self.output {
            HostOutput::Ordered(publication) => Some(publication.clone()),
            HostOutput::Immediate(_) => None,
        }
    }
    fn signal_terminal(&self, result: Result<[u64; 4]>) {
        if let Some(fence) = &self.terminal {
            if let Err(error) = result {
                fence.set_error(error);
            }
            fence.signal();
        }
    }
    /// All fallible scheduler setup has succeeded. Failures from this point
    /// must be visible through VM_STATUS before the accepted job's fence.
    pub(crate) fn mark_accepted(&self) {
        self.accepted.store(true, Ordering::Release);
    }
    pub(crate) fn status(&self) -> &Arc<VmStatus> {
        &self.status
    }
    pub(super) fn sources(&self) -> Option<&Arc<super::sync_input::Sources>> {
        self.sources.as_ref()
    }
    pub(super) fn is_input_source(&self, source: &Arc<super::fence::Submission>) -> bool {
        self.output.output().member.is_source(source)
    }
    pub(super) fn set_source_ready(&self, ready: Fence) {
        self.output.output().member.set_source_ready(ready);
    }
    pub(super) fn needs_source_ready(&self) -> bool {
        self.output.output().member.needs_source_ready()
    }
    pub(super) fn note_input_error(&self, error: Error) -> bool {
        self.output.output().member.note_error(error)
    }
    /// Called under the device mutex as a successful retirement is queued.
    pub(super) fn note_source_retired(&self) {
        if self.interrupted() {
            // complete() fails it; consumers must not take its output as valid.
            self.note_input_error(ECANCELED);
            return;
        }
        match self.check_source_inputs() {
            Ok(()) => self.output.output().member.note_retired(),
            Err(error) => { self.note_input_error(error); }
        }
    }
    pub(super) fn check_source_inputs(&self) -> Result {
        if let Some(sources) = &self.sources { sources.check()?; }
        Ok(())
    }

    pub(crate) fn work_state(&self) -> Result<Arc<WorkStateLease>> {
        self.output.output().member.work_state()
    }
    pub(crate) fn has_timestamps(&self) -> bool {
        self.output.output().destinations.iter().any(Option::is_some)
    }

    pub(crate) fn compute_timestamp_va(&self) -> Result<u64> {
        Ok(self.compute.as_ref().ok_or(EINVAL)?.page()?.mapping.iova())
    }

    /// The caller has proved that firmware retired this command, or has
    /// received its scheduler kill acknowledgement with recovery closed.
    /// These are the command's public timestamp words, not queue-private state.
    pub(crate) fn compute_words(&self) -> Result<[u64; 2]> {
        let object = &self.compute.as_ref().ok_or(EINVAL)?.page()?.object;
        let words = [
            object.dword(0)?.load(Ordering::Relaxed),
            object.dword(8)?.load(Ordering::Relaxed),
        ];
        barrier(Ordering::Acquire);
        Ok(words)
    }

    pub(crate) fn replay_witness_words(&self) -> Option<[u64; 2]> {
        if self.signalled.load(Ordering::Acquire) {
            return None;
        }
        self.compute_words().ok()
    }

    pub(crate) fn begin_replay(&self) -> bool {
        let accepted = self
            .replays
            .fetch_update(Ordering::AcqRel, Ordering::Acquire, |count| {
                (count < MAX_REPLAYS).then_some(count + 1)
            })
            .is_ok();
        if accepted {
            self.replay_timeout.store(true, Ordering::Release);
            // Callers replay only work proven not to have started; the new
            // publication runs from the beginning and reports its own result.
            self.interrupted.store(false, Ordering::Release);
        }
        accepted
    }

    pub(crate) fn replay_count(&self) -> u8 {
        self.replays.load(Ordering::Acquire)
    }

    pub(crate) fn note_firmware_consumer(&self) {
        self.firmware_consumer.store(true, Ordering::Release);
    }

    pub(crate) fn has_firmware_consumer(&self) -> bool {
        self.firmware_consumer.load(Ordering::Acquire)
    }

    pub(crate) fn take_replay_timeout(&self) -> bool {
        !self.signalled.load(Ordering::Acquire) && self.replay_timeout.swap(false, Ordering::AcqRel)
    }

    /// Only a witnessed, retained command can be reset before republishing it.
    pub(crate) fn reset_for_replay(&self) -> Result {
        if self.signalled.load(Ordering::Acquire) {
            return Err(ECANCELED);
        }
        let object = &self.compute.as_ref().ok_or(EINVAL)?.page()?.object;
        let start = object.dword(0)?;
        let end = object.dword(8)?;
        start.store(0, Ordering::Relaxed);
        end.store(0, Ordering::Relaxed);
        barrier(Ordering::SeqCst);
        Ok(())
    }

    /// A retirement witness has been accepted by an off-lock completion batch.
    /// This is separate from fence signalling and from a failure still owned by firmware.
    pub(crate) fn note_retirement_queued(&self) {
        self.retirement_queued.store(true, Ordering::Release);
    }

    pub(crate) fn retirement_queued(&self) -> bool {
        self.retirement_queued.load(Ordering::Acquire)
    }

    /// The first timeout request retains its errno while the physical graph is
    /// borrowed. Duplicate watchdog or cancellation callbacks cannot replace it.
    pub(crate) fn defer_timeout(&self, error: Error) {
        let _ = self.timeout_error.compare_exchange(
            0,
            error.to_errno(),
            Ordering::AcqRel,
            Ordering::Acquire,
        );
    }

    pub(crate) fn take_deferred_timeout(&self) -> Option<Error> {
        let error = self.timeout_error.swap(0, Ordering::AcqRel);
        (error != 0).then(|| Error::from_errno(error))
    }

    pub(crate) fn classify(&self, class: Class) {
        self.class.store(
            match class {
                Class::Spared => 1,
                Class::Guilty => 2,
            },
            Ordering::Release,
        );
    }

    pub(crate) fn spared(&self) -> bool {
        self.class.load(Ordering::Acquire) == 1
    }

    pub(crate) fn guilty(&self) -> bool {
        self.class.load(Ordering::Acquire) == 2
    }

    /// A recovery blamed this command, whatever class it keeps: it may have caused the fault,
    /// so it is never published again.
    pub(crate) fn note_blamed(&self) {
        self.blamed.store(true, Ordering::Release);
    }

    pub(crate) fn blamed(&self) -> bool {
        self.blamed.load(Ordering::Acquire)
    }

    /// A firmware restart listed this command in its recovery table: it was in
    /// flight, so it may have run partly. Its stamps and completion record may
    /// still look like success. Unless a replay republishes it from the start,
    /// it fails and the failure is always reported to the client.
    pub(crate) fn note_interrupted(&self) {
        self.interrupted.store(true, Ordering::Release);
    }

    pub(crate) fn interrupted(&self) -> bool {
        self.interrupted.load(Ordering::Acquire)
    }

    /// Whether this failure makes the VM unusable. Spared work, and interrupted
    /// work no recovery found guilty, fail without poisoning it.
    pub(crate) fn poisons(&self, error: Error) -> bool {
        let interrupted = self.interrupted();
        (error != ECANCELED || interrupted) && !self.spared() && !(interrupted && !self.guilty())
    }

    /// ECANCELED stays silent only for work that was never interrupted (closed
    /// or cancelled by its owner); every other failure reaches the client.
    fn report(&self, error: Error) {
        if self.poisons(error) {
            self.status.record(error);
        } else if error != ECANCELED || self.interrupted() {
            self.status.report_failure(error);
        }
    }

    /// Installed before the first firmware-visible prefix. Cancellation takes
    /// the same lock after claiming the signal, so it cannot miss a late lease.
    pub(crate) fn attach_render_lease(
        &self,
        lease: super::queue::admission::RenderLease,
    ) -> Result {
        let mut render = self.render.lock();
        if self.signalled.load(Ordering::Acquire) {
            return Err(ECANCELED);
        }
        if render.is_some() {
            return Err(EIO);
        }
        *render = Some(lease);
        Ok(())
    }

    /// An ioctl publication that met contention returns its lease before the
    /// scheduler publishes the command. Nothing referred to it yet.
    pub(crate) fn detach_render_lease(&self) -> Option<super::queue::admission::RenderLease> {
        self.render.lock().take()
    }

    pub(crate) fn render_slot(&self) -> Option<u8> {
        self.render
            .lock()
            .as_ref()
            .map(super::queue::admission::RenderLease::slot)
    }

    /// Called under the device mutex once the render's kicks are committed.
    pub(crate) fn note_render_publication(&self, fragment: super::fw::kick::KickDependency) {
        if let Some(lease) = self.render.lock().as_ref() {
            lease.note_publication(fragment);
        }
    }

    /// Called under the device mutex before deferring the error signal.
    pub(crate) fn quarantine_render(&self) {
        if let Some(lease) = self.render.lock().as_ref() {
            lease.quarantine(self.spared());
        }
    }

    /// An exact whole-pair drain or processor stop allows rehabilitation. The
    /// VM pin goes first; the still-held lease excludes reuse until both settle.
    pub(crate) fn release_after_retirement(&self) {
        if let Some(sources) = &self.sources { sources.release(); }
        drop(self.vm_job.lock().take());
        let lease = self.render.lock().take();
        if let Some(lease) = lease.as_ref() {
            lease.rehabilitate_after_firmware_drain();
        }
        drop(lease);
    }

    fn release_healthy(&self) {
        if let Some(sources) = &self.sources { sources.release(); }
        drop(self.render.lock().take());
        drop(self.vm_job.lock().take());
    }

    /// Exact retirement permits the physical lease and VM pin to be released
    /// before waking userspace. A failure fence alone never establishes this.
    #[track_caller]
    pub(crate) fn complete(&self, mut result: Result<[u64; 4]>) {
        if result.is_ok() {
            if let Err(error) = self.check_source_inputs() { result = Err(error); }
        }
        if self.signalled.swap(true, Ordering::AcqRel) {
            return;
        }
        if result.is_ok() && self.interrupted() {
            result = Err(ECANCELED);
        }
        if let (Some(feed), Ok(ticks)) = (&self.feed, &result) {
            // Compute reports its dispatch envelope in the first pair, a
            // render its fragment pass in the second.
            if self.compute.is_some() {
                feed.note_pass(ticks[0], ticks[1]);
            } else {
                feed.note_pass(ticks[2], ticks[3]);
            }
        }
        if let Err(error) = result {
            // Spared work cancelled after another VM's recovery is not this client's device loss.
            self.report(error);
        }
        self.release_healthy();
        self.signal_terminal(result);
        self.output.finish(result);
    }

    /// A rejected, never-published command cannot poison the VM for contention.
    #[track_caller]
    pub(crate) fn fail_unpublished(&self, error: Error) {
        if self.signalled.swap(true, Ordering::AcqRel) {
            return;
        }
        self.status.record_if_device_loss(error);
        if self.accepted.load(Ordering::Acquire) {
            self.status.report_failure(error);
        }
        self.release_healthy();
        self.signal_terminal(Err(error));
        self.output.finish(Err(error));
    }

    /// The physical queue must already be quarantined. Keep mappings pinned while
    /// waking waiters, even if queue destruction requested cancellation.
    #[track_caller]
    pub(crate) fn fail_while_owned(&self, error: Error) {
        if self.signalled.swap(true, Ordering::AcqRel) {
            return;
        }
        if let Some(guard) = self.vm_job.lock().as_ref() {
            guard.quarantine();
        }
        if self.spared() {
            self.status.report_failure(error);
        } else {
            self.report(error);
        }
        self.signal_terminal(Err(error));
        self.output.finish(Err(error));
    }
}

/// Collected under the device mutex and executed only after releasing it.
/// Runtime queues establish the ownership witness before constructing `Retired`
/// or `Release`; `FailedOwned` leaves their physical and VM ownership intact.
pub(crate) enum Deferred {
    Retired(Arc<Completion>, Result<[u64; 4]>),
    Drained(Arc<Completion>, Result<[u64; 4]>),
    FailedOwned(Arc<Completion>, Error),
    Unpublished(Arc<Completion>, Error),
    Release(Arc<Completion>),
}

impl Deferred {
    pub(crate) fn finish(self) {
        match self {
            Self::Retired(completion, result) => {
                completion.complete(result);
                // An earlier failure may already have signalled this fence while
                // retaining the pin. Retirement is a separate release witness.
                completion.release_after_retirement();
            }
            Self::Drained(completion, result) => {
                completion.release_after_retirement();
                completion.complete(result);
            }
            Self::FailedOwned(completion, error) => completion.fail_while_owned(error),
            Self::Unpublished(completion, error) => completion.fail_unpublished(error),
            Self::Release(completion) => completion.release_after_retirement(),
        }
    }
}
