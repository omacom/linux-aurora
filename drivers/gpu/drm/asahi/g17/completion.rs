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
use crate::{
    driver::AsahiDrmDevice,
    file,
    hw::t8140::resources as cfg,
    mmu,
};
use core::sync::atomic::{
    fence as barrier, AtomicBool, AtomicI32, AtomicU32, AtomicU64, AtomicU8, Ordering,
};
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
    /// The destination's address in the firmware timestamp window, if its buffer is mapped there.
    window_va: Option<u64>,
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
        let window_va = buffer.window_va(offset);
        let offset = buffer.timestamp_offset(offset)?;
        Ok(Some(Self { buffer, offset, window_va }))
    }

    fn write(&self, ticks: u64) {
        self.buffer.write_timestamp(self.offset, ticks_to_ns(ticks));
        barrier(Ordering::Release);
    }

    /// An end stamp the firmware writes itself, through the window, as the same nanoseconds the
    /// host would write. Wait for it without writing: a host write, or a firmware write landing
    /// after the fence, could undo a reset userspace may make once the fence has signalled. If
    /// it does not land in time, the host writes the value, still before the fence.
    fn write_window(&self, ticks: u64) {
        let ns = ticks_to_ns(ticks);
        let mut polls = 0;
        while self.buffer.read_timestamp(self.offset) != Some(ns) {
            if polls == WINDOW_POLLS {
                WINDOW_FALLBACKS.fetch_add(1, Ordering::Relaxed);
                WINDOW_MISSES.fetch_add(1, Ordering::Relaxed);
                self.write(ticks);
                return;
            }
            kernel::time::delay::udelay(kernel::time::Delta::from_micros(WINDOW_POLL_US));
            polls += 1;
        }
        WINDOW_KEPT.fetch_add(1, Ordering::Relaxed);
        WINDOW_MISSES.store(0, Ordering::Relaxed);
    }
}

fn ticks_to_ns(ticks: u64) -> u64 {
    const NSEC_PER_SEC: u64 = 1_000_000_000;
    (ticks / cfg::COMMAND_TIMESTAMP_HZ)
        .wrapping_mul(NSEC_PER_SEC)
        .wrapping_add(ticks % cfg::COMMAND_TIMESTAMP_HZ * NSEC_PER_SEC / cfg::COMMAND_TIMESTAMP_HZ)
}

/// The firmware writes the stamp ~20 us after the pass ends; macOS saw it up to 36 us after the
/// event ring. In practice it has always landed by retirement's first read.
const WINDOW_POLLS: u32 = 40;
const WINDOW_POLL_US: i64 = 5;
static WINDOW_KEPT: AtomicU64 = AtomicU64::new(0);
static WINDOW_FALLBACKS: AtomicU64 = AtomicU64::new(0);
/// Host fallbacks in a row. At [`WINDOW_GIVE_UP`] the firmware is taken not to write through
/// the window at all (mapping refused, other firmware), and no render is handed a window
/// address again, so none pays the full wait.
static WINDOW_MISSES: AtomicU32 = AtomicU32::new(0);
const WINDOW_GIVE_UP: u32 = 8;

/// `capacity`: firmware-written end stamps kept, and those the host had to write.
pub(crate) fn window_counters() -> [u64; 2] {
    [WINDOW_KEPT.load(Ordering::Relaxed), WINDOW_FALLBACKS.load(Ordering::Relaxed)]
}

/// Only the fragment end stamp goes through the window (slots: vertex start, vertex end,
/// fragment start, fragment end). The firmware latches a start stamp before the command's
/// barriers resolve, so a reset of the same query ordered before the command can still land
/// on top of it. A vertex end handed to the firmware never held the host's value by the end
/// of the wait, so it would only cost every render that stamps it the full wait.
const fn window_slot(index: usize) -> bool {
    index == 3
}

/// Timestamp destinations resolved and retained before any command is enqueued. Dropping
/// one releases only a buffer reference; the buffer defers its own mapping release.
pub(crate) struct Destinations {
    stamps: [Option<Destination>; 4],
    /// A render: the firmware gets its fragment end destination through the window. A compute
    /// command's only stamp pair stays kernel-private (recovery trusts it).
    render: bool,
    /// The firmware was handed window destinations ([`Self::window_vas`]).
    handed: AtomicBool,
}

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
        Ok(Self {
            stamps: destinations,
            render: timestamps.len() == 2,
            handed: AtomicBool::new(false),
        })
    }

    /// Window addresses for a render's vertex and fragment start/end destinations: the fragment
    /// end's if it is mapped in the window, 0 for the others.
    fn window_vas(&self) -> [u64; 4] {
        if !self.render || WINDOW_MISSES.load(Ordering::Relaxed) >= WINDOW_GIVE_UP {
            return [0; 4];
        }
        let vas: [u64; 4] = core::array::from_fn(|i| {
            let destination = self.stamps[i].as_ref().filter(|_| window_slot(i));
            destination.and_then(|d| d.window_va).unwrap_or(0)
        });
        if vas.iter().any(|&va| va != 0) {
            self.handed.store(true, Ordering::Release);
        }
        vas
    }

    pub(crate) fn has_outputs(&self) -> bool {
        self.stamps.iter().any(Option::is_some)
    }

    fn iter(&self) -> core::slice::Iter<'_, Option<Destination>> {
        self.stamps.iter()
    }

    fn write(&self, ticks: [u64; 4]) {
        let handed = self.handed.load(Ordering::Acquire);
        for (index, (destination, ticks)) in self.stamps.iter().zip(ticks).enumerate() {
            match destination {
                Some(destination) if handed && window_slot(index) && destination.window_va.is_some() => {
                    destination.write_window(ticks)
                }
                Some(destination) => destination.write(ticks),
                None => {}
            }
        }
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
            Ok(ticks) => self.destinations.write(ticks),
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
            vm.client_private_range(),
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
    /// Pages retained per context; deeper pipelines allocate the rest. Every queued command
    /// holds a page from submission on, and a game keeps a couple of hundred compute commands
    /// queued, so a smaller cache refills and frees a page for nearly every command.
    const RETAINED: usize = 256;
    /// The cache grows to `RETAINED` only in contexts that queue that deep.
    const INITIAL: usize = 8;

    pub(crate) fn new() -> Result<Arc<Self>> {
        let pages = KVec::with_capacity(Self::INITIAL, GFP_KERNEL)?;
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
            if pages.len() < Self::RETAINED && pages.reserve(1, GFP_KERNEL).is_ok() {
                pages.push_within_capacity(page).err().map(|rejected| rejected.0)
            } else {
                Some(page)
            }
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

/// M5 timestamp pairs share a retained page of the physical compute queue.
/// A completion keeps its backing alive through quarantine and replay.
struct M5ComputeTimestamps {
    object: Arc<KernelObject>,
    offset: usize,
}

impl M5ComputeTimestamps {
    fn clear(&self) -> Result {
        let start = self.object.dword(self.offset)?;
        let end = self.object.dword(self.offset + 8)?;
        start.store(0, Ordering::Relaxed);
        end.store(0, Ordering::Relaxed);
        barrier(Ordering::SeqCst);
        Ok(())
    }

    fn words(&self) -> Result<[u64; 2]> {
        let words = [self.object.dword(self.offset)?.load(Ordering::Relaxed),
            self.object.dword(self.offset + 8)?.load(Ordering::Relaxed)];
        barrier(Ordering::Acquire);
        Ok(words)
    }
}

#[pin_data]
pub(crate) struct Completion {
    status: Arc<VmStatus>,
    output: HostOutput,
    // Present only when this command's public result may wait for a render
    // predecessor's host writes. DRM credits must follow the terminal result.
    terminal: Option<UserFence<CompletionFence>>,
    compute: Option<PooledTimestamps>,
    #[pin]
    m5_compute: Mutex<Option<M5ComputeTimestamps>>,
    /// A compute command. M5 compute binds its timestamps at publication, so
    /// `compute` alone does not identify the engine.
    is_compute: bool,
    feed: Option<Arc<super::feed::Feed>>,
    /// The command's execution context, whose busy run its execution extends
    /// ([`super::feed::Feed::note_work`]).
    context_id: u32,
    usage: Arc<crate::usage::Lane<Arc<crate::usage::Client>>>,
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
        usage: Arc<crate::usage::Lane<Arc<crate::usage::Client>>>,
    ) -> Result<Arc<Self>> {
        let fence = fence::independent(contexts)?;
        let ordered = !compute && (host_predecessors || destinations.iter().any(Option::is_some));
        let terminal = if host_predecessors {
            Some(fence::independent(contexts)?)
        } else {
            None
        };
        let output = Output {
            fence,
            member,
            destinations,
        };
        let output = if ordered {
            HostOutput::Ordered(Publication::new(output, context.status().clone())?)
        } else {
            HostOutput::Immediate(output)
        };
        // M5 compute binds its timestamp pair in its physical queue's page at publication.
        let is_compute = compute;
        let compute = if compute && !context.is_m5() {
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
                m5_compute <- new_mutex!(None, "M5 queue timestamps"),
                is_compute,
                feed,
                context_id: context.id(),
                usage,
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
        self.terminal
            .as_ref()
            .map_or_else(|| self.fence(), |fence| Fence::from_fence(fence))
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
            Err(error) => {
                self.note_input_error(error);
            }
        }
    }
    pub(super) fn check_source_inputs(&self) -> Result {
        if let Some(sources) = &self.sources {
            sources.check()?;
        }
        Ok(())
    }

    pub(crate) fn work_state(&self) -> Result<Arc<WorkStateLease>> {
        self.output.output().member.work_state()
    }
    pub(crate) fn has_timestamps(&self) -> bool {
        self.output
            .output()
            .destinations
            .iter()
            .any(Option::is_some)
    }

    /// The firmware window addresses of a render's vertex and fragment start/end stamps.
    pub(crate) fn window_vas(&self) -> [u64; 4] {
        self.output.output().destinations.window_vas()
    }

    pub(crate) fn compute_timestamp_va(&self) -> Result<u64> {
        if let Some(compute) = &self.compute { return Ok(compute.page()?.mapping.iova()); }
        let compute = self.m5_compute.lock();
        let compute = compute.as_ref().ok_or(EINVAL)?;
        Ok(compute.object.gpu_va() + compute.offset as u64)
    }

    /// The queue checked its record-slot retirement before calling this.
    /// Clearing and binding precede descriptor publication under the device lock.
    pub(crate) fn bind_m5_compute_timestamps(&self, object: &Arc<KernelObject>, offset: usize) -> Result<u64> {
        if self.compute.is_some() || self.signalled.load(Ordering::Acquire) {
            return Err(EINVAL);
        }
        let timestamps = M5ComputeTimestamps { object: object.clone(), offset };
        timestamps.clear()?;
        let address = timestamps.object.gpu_va() + offset as u64;
        *self.m5_compute.lock() = Some(timestamps);
        Ok(address)
    }

    /// The caller has proved that firmware retired this command, or has
    /// received its scheduler kill acknowledgement with recovery closed.
    /// These are the command's public timestamp words, not queue-private state.
    pub(crate) fn compute_words(&self) -> Result<[u64; 2]> {
        let Some(compute) = self.compute.as_ref() else {
            return self.m5_compute.lock().as_ref().ok_or(EINVAL)?.words();
        };
        let object = &compute.page()?.object;
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
        let Some(compute) = self.compute.as_ref() else {
            return self.m5_compute.lock().as_ref().ok_or(EINVAL)?.clear();
        };
        let object = &compute.page()?.object;
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
    /// it fails: its fence carries ECANCELED, and only a guilty VM is poisoned.
    pub(crate) fn note_interrupted(&self) {
        self.interrupted.store(true, Ordering::Release);
    }

    pub(crate) fn interrupted(&self) -> bool {
        self.interrupted.load(Ordering::Acquire)
    }

    /// Whether the fence has been signalled (completed or failed).
    pub(crate) fn signalled(&self) -> bool {
        self.signalled.load(Ordering::Acquire)
    }

    /// Whether this failure makes the VM unusable. Spared work, interrupted
    /// work no recovery found guilty, and work that never ran (ENODATA) fail
    /// without poisoning it.
    pub(crate) fn poisons(&self, error: Error) -> bool {
        // A command that never ran cannot have faulted the GPU. The queue's own failed
        // command also comes back as ENODATA (a zero-duration record); it is classified
        // guilty before it fails, by the firmware's fault report or by a recovery.
        if error == ENODATA && !self.guilty() {
            return false;
        }
        let interrupted = self.interrupted();
        (error != ECANCELED || interrupted) && !self.spared() && !(interrupted && !self.guilty())
    }

    /// A non-poisoning ECANCELED or ENODATA (work a recovery discarded or dropped) stays out of
    /// the VM status, as the UAPI documents for work another VM's recovery discarded:
    /// Borealis reads any status error as a lost device. Every other failure reaches the client.
    fn report(&self, error: Error) {
        if self.poisons(error) {
            self.status.record(error);
        } else if !discarded(error) {
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
        if let Some(sources) = &self.sources {
            sources.release();
        }
        drop(self.vm_job.lock().take());
        let lease = self.render.lock().take();
        if let Some(lease) = lease.as_ref() {
            lease.rehabilitate_after_firmware_drain();
        }
        drop(lease);
    }

    /// Whether the command still holds its VM pin: firmware may still own or replay it.
    pub(crate) fn retains_vm(&self) -> bool {
        self.vm_job.lock().is_some()
    }

    fn release_healthy(&self) {
        if let Some(sources) = &self.sources {
            sources.release();
        }
        drop(self.render.lock().take());
        drop(self.vm_job.lock().take());
    }

    /// Exact retirement permits the physical lease and VM pin to be released
    /// before waking userspace. A failure fence alone never establishes this.
    #[track_caller]
    pub(crate) fn complete(&self, result: Result<[u64; 4]>) {
        // M5 compute feeds the refill detector only from its exact execution record
        // (RetiredCompute); M5 compute is the compute without pooled timestamps.
        let execution = if self.is_compute && self.compute.is_none() {
            None
        } else {
            result.as_ref().ok().map(|ticks| [ticks[0], ticks[1]])
        };
        self.complete_with_compute(result, execution);
    }

    /// Compute feedback is separate from public timestamps, which may contain
    /// a synthesized retirement envelope when the execution record is absent.
    #[track_caller]
    fn complete_with_compute(&self, mut result: Result<[u64; 4]>, compute: Option<[u64; 2]>) {
        if result.is_ok() {
            if let Err(error) = self.check_source_inputs() {
                result = Err(error);
            }
        }
        if self.signalled.swap(true, Ordering::AcqRel) {
            return;
        }
        if result.is_ok() && self.interrupted() {
            result = Err(ECANCELED);
        }
        if let Ok(ticks) = &result {
            use crate::usage::Engine;
            let hz = cfg::COMMAND_TIMESTAMP_HZ as u32;
            if self.is_compute {
                self.usage.add(Engine::Compute, ticks[0], ticks[1], hz);
            } else {
                self.usage.add(Engine::Vertex, ticks[0], ticks[1], hz);
                self.usage.add(Engine::Fragment, ticks[2], ticks[3], hz);
            }
        }
        if let (Some(feed), Ok(ticks)) = (&self.feed, &result) {
            // Compute reports its dispatch envelope in the first pair, a
            // render its fragment pass in the second.
            if self.is_compute {
                if let Some([start, end]) = compute {
                    feed.note_pass(start, end);
                    feed.note_work(self.context_id, start, end);
                }
            } else {
                feed.note_pass(ticks[2], ticks[3]);
                // The whole render, from the first of its two halves that ran to the last.
                let start = [ticks[0], ticks[2]].into_iter().filter(|&tick| tick != 0).min();
                feed.note_work(self.context_id, start.unwrap_or(0), ticks[1].max(ticks[3]));
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
        // As in complete(): a cancellation is not this client's device loss. A guilty VM
        // already carries its own sticky error.
        if self.accepted.load(Ordering::Acquire) && error != ECANCELED {
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
            if !discarded(error) {
                self.status.report_failure(error);
            }
        } else {
            self.report(error);
        }
        self.signal_terminal(Err(error));
        self.output.finish(Err(error));
    }
}

/// The errors of work a recovery discarded (ECANCELED) or dropped before it ran (ENODATA).
fn discarded(error: Error) -> bool {
    error == ECANCELED || error == ENODATA
}

/// Collected under the device mutex and executed only after releasing it.
/// Runtime queues establish the ownership witness before constructing `Retired`
/// or `Release`; `FailedOwned` leaves their physical and VM ownership intact.
pub(crate) enum Deferred {
    Retired(Arc<Completion>, Result<[u64; 4]>),
    /// Only the optional interval comes from an exact compute execution record.
    RetiredCompute(Arc<Completion>, [u64; 4], Option<[u64; 2]>),
    Drained(Arc<Completion>, Result<[u64; 4]>),
    FailedOwned(Arc<Completion>, Error),
    Unpublished(Arc<Completion>, Error),
    Release(Arc<Completion>),
}

impl Deferred {
    pub(crate) fn finish(self) {
        match self {
            Self::RetiredCompute(completion, ticks, execution) => {
                completion.complete_with_compute(Ok(ticks), execution);
                completion.release_after_retirement();
            }
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

#[kernel::macros::kunit_tests(asahi_g17_timestamp_window)]
mod window_tests {
    use super::*;

    #[test]
    fn only_the_fragment_end_goes_through_the_window() {
        assert!(!window_slot(0) && !window_slot(1) && !window_slot(2) && window_slot(3));
    }

    #[test]
    fn ticks_convert_to_nanoseconds_at_24_mhz() {
        assert_eq!(ticks_to_ns(24_000_000), 1_000_000_000);
        assert_eq!(ticks_to_ns(24), 1_000);
        assert_eq!(ticks_to_ns(24_000_025), 1_000_001_041);
    }
}
