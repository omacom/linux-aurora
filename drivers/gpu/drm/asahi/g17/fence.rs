// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Hardware completion fences and the aggregate fence of one submit ioctl.
//!
//! These fences can complete out of order, so each has an independent timeline.
//! Signalling belongs outside the device lock, after timestamp and VM-status writes.

use super::context::{Context, WorkStateLease};
use crate::file::SyncItem;
use core::sync::atomic::{AtomicBool, AtomicI32, AtomicU8, AtomicUsize, Ordering};
use kernel::{
    bindings, c_str,
    dma_fence::{Fence, FenceContexts, FenceObject, FenceOps, RawDmaFence, UniqueFence, UserFence},
    new_mutex,
    prelude::*,
    str::CStr,
    sync::{Arc, Mutex},
};

pub(crate) struct CompletionFence;

unsafe extern "C" {
    fn asahi_g17_signal_fence(fence: *mut bindings::dma_fence);
}

fn signal_exported(fence: &dyn RawDmaFence) {
    // SAFETY: The borrowed fence remains owned throughout synchronous signaling.
    // Ready completion scopes are drained iteratively after its lock is released.
    unsafe { asahi_g17_signal_fence(fence.raw()) };
}

#[vtable]
impl FenceOps for CompletionFence {
    fn get_driver_name<'a>(self: &'a FenceObject<Self>) -> &'a CStr {
        c_str!("asahi")
    }

    fn get_timeline_name<'a>(self: &'a FenceObject<Self>) -> &'a CStr {
        c_str!("g17-submit")
    }
}

/// Give independently retired work a timeline that fence merging cannot subsume.
pub(crate) fn independent(contexts: &FenceContexts) -> Result<UserFence<CompletionFence>> {
    let fence = contexts.new_fence(0, CompletionFence)?;
    // SAFETY: The fence is unique and unpublished: there are no waiters, callbacks or
    // shared references. The DMA-fence allocator supplies a globally unique context.
    unsafe { (*fence.raw()).context = bindings::dma_fence_context_alloc(1) };
    Ok(fence.into())
}

/// An immutable failure fence allocated before the queue is exposed to ioctls.
/// Reusing it for rejected submissions cannot retain or subsume accepted work.
pub(crate) fn allocation_failure(contexts: &FenceContexts) -> Result<Fence> {
    let fence = independent(contexts)?;
    fence.set_error(ENOMEM);
    fence.signal();
    Ok(Fence::from_fence(&fence))
}

/// The existing aggregate-fence allocation owns the existing submission state.
/// Submission never owns this fence, so exported references cannot form a cycle.
pub(crate) struct ExportFence {
    source: Option<Arc<Submission>>,
    scoped: bool,
}
static EXPORT_OPS: AtomicUsize = AtomicUsize::new(0);

#[vtable]
impl FenceOps for ExportFence {
    fn get_driver_name<'a>(self: &'a FenceObject<Self>) -> &'a CStr { c_str!("asahi") }
    fn get_timeline_name<'a>(self: &'a FenceObject<Self>) -> &'a CStr {
        c_str!("g17-aggregate")
    }
}

#[inline(never)]
fn export_fallback(contexts: &FenceContexts) -> Result<UniqueFence<ExportFence>> {
    let fence = contexts.new_fence(0, ExportFence { source: None, scoped: false })?;
    // SAFETY: Unique, initialized and still private. Only this constructor
    // publishes ExportFence objects; its payload is immutable after sharing.
    let ops = unsafe {
        (*fence.raw()).context = bindings::dma_fence_context_alloc(1);
        (*fence.raw()).ops as usize
    };
    let previous = EXPORT_OPS.load(Ordering::Acquire);
    if previous == 0 {
        match EXPORT_OPS.compare_exchange(0, ops, Ordering::AcqRel, Ordering::Acquire) {
            Ok(_) => {},
            Err(previous) if previous == ops => {},
            Err(_) => return Err(EIO),
        }
    } else if previous != ops {
        return Err(EIO);
    }
    Ok(fence)
}

/// Caller retains the leaf fence throughout lookup. Exact registered operations
/// identify the repr(C) FenceObject payload, not a driver or timeline name.
pub(super) unsafe fn exported_source_raw(raw: *mut bindings::dma_fence)
    -> Option<Arc<Submission>>
{
    let expected = EXPORT_OPS.load(Ordering::Acquire);
    if expected == 0 { return None; }
    // SAFETY: The caller owns a live fence reference. ExportFence has a unique
    // timeline callback and nontrivial payload drop. No cast precedes the exact
    // registered vtable check. FenceObject begins with its dma_fence.
    let object = unsafe {
        if (*raw).ops as usize != expected { return None; }
        &*(raw as *const FenceObject<ExportFence>)
    };
    object.source.as_ref().filter(|source| source.firmware_source).cloned()
}

struct State {
    work: Option<Arc<WorkStateLease>>,
    ready: Option<Fence>,
}

// One Member per parsed command, plus the enqueue guard. Exported references
// retain the allocation but do not contribute to either of these counters.
kernel::static_assert!(crate::file::MAX_COMMANDS_PER_SUBMISSION < u8::MAX as u32);

/// The initial member keeps the aggregate unsignalled until enqueue finishes.
#[pin_data]
pub(crate) struct Submission {
    #[pin]
    state: Mutex<State>,
    remaining: AtomicU8,
    error: AtomicI32,
    /// Live Member objects, plus the enqueue guard, independently of signalling.
    owners: AtomicU8,
    firmware_source: bool,
    /// Set under the device mutex when the only member's successful retirement
    /// is queued, before its fence is signalled off-lock: a consumer published
    /// in between needs no wait on it.
    retired: AtomicBool,
}

impl Submission {
    fn new(context: Arc<Context>, firmware_source: bool) -> Result<Arc<Self>> {
        let work = Arc::new(WorkStateLease::deferred(context), GFP_KERNEL)?;
        Self::with_work(Some(work), firmware_source)
    }
    fn with_work(work: Option<Arc<WorkStateLease>>, firmware_source: bool) -> Result<Arc<Self>> {
        Arc::pin_init(pin_init!(Self {
            state <- new_mutex!(State { work, ready: None }, "G17 submission work state"),
            remaining: AtomicU8::new(1),
            error: AtomicI32::new(0),
            owners: AtomicU8::new(1),
            firmware_source,
            retired: AtomicBool::new(false),
        }), GFP_KERNEL)
    }
    pub(super) fn source_status(&self) -> Result<bool> {
        let complete = self.remaining.load(Ordering::Acquire) == 0
            || self.retired.load(Ordering::Acquire);
        let error = self.error.load(Ordering::Acquire);
        if error != 0 { return Err(super::dependency::inherited_error(Error::from_errno(error))); }
        Ok(complete)
    }
    pub(super) fn same_vm(&self, status: &Arc<super::status::VmStatus>) -> bool {
        self.state.lock().work.as_ref().is_some_and(|work| Arc::ptr_eq(work.status(), status))
    }
    pub(super) fn ready(&self) -> Option<Fence> { self.state.lock().ready.clone() }
    fn set_ready(&self, ready: Fence) {
        if !self.firmware_source { return; }
        let mut state = self.state.lock();
        if state.work.is_some() && state.ready.is_none() { state.ready = Some(ready); }
    }
    fn note_error(&self, error: Error) -> bool {
        self.error.compare_exchange(0, error.to_errno(), Ordering::AcqRel, Ordering::Acquire).is_ok()
    }
    fn release_state(&self) {
        let old = {
            let mut state = self.state.lock();
            (state.work.take(), state.ready.take())
        };
        drop(old);
    }
    fn owner_done(&self) {
        // Every hardware owner retains its Completion/Member through exact
        // retirement or stop. Exported fence FDs must not retain failed pins.
        if self.owners.fetch_sub(1, Ordering::AcqRel) == 1 { self.release_state(); }
    }
    fn member_done(&self, fence: &UserFence<ExportFence>, result: Result) {
        if let Err(error) = result { self.note_error(error); }
        if self.remaining.fetch_sub(1, Ordering::AcqRel) == 1 {
            let error = self.error.load(Ordering::Acquire);
            if error != 0 { fence.set_error(Error::from_errno(error)); }
            else { self.release_state(); }
            if fence.scoped { signal_exported(fence); } else { fence.signal(); }
        }
    }
}

#[derive(Clone)]
pub(crate) struct Aggregate(UserFence<ExportFence>);
impl Aggregate {
    fn source(&self) -> &Arc<Submission> {
        // Only Outputs::publish constructs Aggregate, after setting the payload.
        self.0.source.as_ref().expect("published aggregate has submission state")
    }
    fn fence(&self) -> Fence { Fence::from_fence(&self.0) }
    pub(crate) fn member(&self) -> Member {
        self.source().remaining.fetch_add(1, Ordering::Relaxed);
        self.source().owners.fetch_add(1, Ordering::Relaxed);
        Member { aggregate: self.clone(), complete: AtomicBool::new(false) }
    }
    fn finish_enqueue(&self, result: Result) {
        if let Err(error) = result { self.source().error.store(error.to_errno(), Ordering::Release); }
        self.source().member_done(&self.0, result);
        self.source().owner_done();
    }
}

pub(crate) struct Member {
    aggregate: Aggregate,
    complete: AtomicBool,
}
impl Member {
    pub(super) fn needs_source_ready(&self) -> bool { self.aggregate.source().firmware_source }
    pub(crate) fn work_state(&self) -> Result<Arc<WorkStateLease>> {
        self.aggregate.source().state.lock().work.clone().ok_or(ECANCELED)
    }
    pub(super) fn is_source(&self, source: &Arc<Submission>) -> bool {
        self.aggregate.source().firmware_source && Arc::ptr_eq(self.aggregate.source(), source)
    }
    pub(super) fn set_source_ready(&self, ready: Fence) { self.aggregate.source().set_ready(ready); }
    pub(super) fn note_error(&self, error: Error) -> bool { self.aggregate.source().note_error(error) }
    pub(super) fn note_retired(&self) {
        let source = self.aggregate.source();
        if source.firmware_source { source.retired.store(true, Ordering::Release); }
    }
    pub(crate) fn complete(&self, result: Result) {
        if !self.complete.swap(true, Ordering::AcqRel) {
            self.aggregate.source().member_done(&self.aggregate.0, result);
        }
    }
}

#[kunit_tests(asahi_g17_aggregate_promise)]
mod tests {
    use super::*;
    use kernel::sync::LockClassKey;
    static KEY: Pin<&LockClassKey> = kernel::static_lock_class!();

    fn contexts() -> FenceContexts {
        FenceContexts::new(1, c_str!("asahi_promise_test"), KEY).unwrap()
    }
    fn aggregate(contexts: &FenceContexts) -> Aggregate {
        let mut fence = export_fallback(contexts).unwrap();
        fence.source = Some(Submission::with_work(None, true).unwrap());
        Aggregate(fence.into())
    }

    #[test]
    fn exact_export_identity_recognizes_unpublished_source() {
        let contexts = contexts();
        let aggregate = aggregate(&contexts);
        let plain = independent(&contexts).unwrap();
        // SAFETY: Both references remain owned throughout the checked lookup.
        assert!(unsafe { exported_source_raw(plain.raw()) }.is_none());
        let source = unsafe { exported_source_raw(aggregate.0.raw()) }.unwrap();
        assert!(Arc::ptr_eq(&source, aggregate.source()));
        assert_eq!(source.source_status().unwrap(), false);
        aggregate.finish_enqueue(Ok(()));
        assert_eq!(source.source_status().unwrap(), true);
    }

    #[test]
    fn failed_export_releases_state_after_last_runtime_owner() {
        let contexts = contexts();
        let aggregate = aggregate(&contexts);
        let source = aggregate.source().clone();
        source.state.lock().ready = Some(Fence::from_fence(&independent(&contexts).unwrap()));
        let member = aggregate.member();
        aggregate.finish_enqueue(Ok(()));
        member.complete(Err(EIO));
        assert_eq!(super::super::job::fence_status(&aggregate.fence()), EIO.to_errno());
        assert!(source.state.lock().ready.is_some());
        drop(member);
        // Keep both the exported fence and source state alive, as userspace can.
        assert!(source.state.lock().ready.is_none());
        assert_eq!(source.owners.load(Ordering::Acquire), 0);
    }

    #[test]
    fn actual_enqueue_error_wins_over_dropped_member_cancellation() {
        let aggregate = aggregate(&contexts());
        drop(aggregate.member());
        aggregate.finish_enqueue(Err(ENOMEM));
        assert_eq!(super::super::job::fence_status(&aggregate.fence()), ENOMEM.to_errno());
    }

    #[test]
    fn rejected_private_fence_never_exports_source_state() {
        let fence = export_fallback(&contexts()).unwrap();
        fence.set_error(ENOMEM);
        fence.signal();
        // SAFETY: The unique fence owns its initialized allocation.
        assert!(unsafe { exported_source_raw(fence.raw()) }.is_none());
    }

    #[test]
    fn maximum_submission_keeps_all_members_and_enqueue_guard() {
        let aggregate = aggregate(&contexts());
        let source = aggregate.source().clone();
        let mut members: [Option<Member>; crate::file::MAX_COMMANDS_PER_SUBMISSION as usize] =
            core::array::from_fn(|_| Some(aggregate.member()));
        assert_eq!(source.remaining.load(Ordering::Acquire), 65);
        assert_eq!(source.owners.load(Ordering::Acquire), 65);
        aggregate.finish_enqueue(Ok(()));
        for member in &members {
            assert_eq!(super::super::job::fence_status(&aggregate.fence()), 0);
            member.as_ref().unwrap().complete(Ok(()));
        }
        assert!(super::super::job::fence_status(&aggregate.fence()) > 0);
        assert_eq!(source.remaining.load(Ordering::Acquire), 0);
        for member in &mut members { drop(member.take()); }
        assert_eq!(source.owners.load(Ordering::Acquire), 0);
    }
}
impl Drop for Member {
    fn drop(&mut self) {
        self.complete(Err(ECANCELED));
        self.aggregate.source().owner_done();
    }
}

fn install(outputs: impl Iterator<Item = SyncItem>, fence: &Fence) {
    for mut sync in outputs {
        if let Some(chain) = sync.chain_fence.take() {
            sync.syncobj.add_point(chain, fence, sync.timeline_value);
        } else { sync.syncobj.replace_fence(Some(fence)); }
    }
}

pub(crate) struct Outputs {
    fallback: Option<UniqueFence<ExportFence>>,
    syncs: KVec<SyncItem>,
    input_count: usize,
    submission: Option<Aggregate>,
    result: Result,
    cumulative: Option<super::sync_output::SignalBundle>,
}
impl Outputs {
    pub(crate) fn new(contexts: &FenceContexts, mut syncs: KVec<SyncItem>,
        input_count: usize, allocation_failure: &Fence) -> Result<Self>
    {
        if input_count > syncs.len() { return Err(EINVAL); }
        let mut cumulative = syncs.iter_mut().find_map(|sync| sync.cumulative.take());
        let fallback = match export_fallback(contexts) {
            Ok(fence) => fence,
            Err(error) => {
                if let Some(bundle) = &mut cumulative {
                    bundle.fail(allocation_failure);
                } else {
                    install(syncs.drain(input_count..), allocation_failure);
                }
                return Err(error);
            }
        };
        Ok(Self { fallback: Some(fallback), syncs, input_count,
            submission: None, result: Err(ECANCELED), cumulative })
    }
    pub(crate) fn inputs(&self) -> &[SyncItem] { &self.syncs[..self.input_count] }
    pub(crate) fn publish(&mut self, context: Arc<Context>, firmware_source: bool) -> Result<Aggregate> {
        let source = Submission::new(context, firmware_source)?;
        let mut fence = self.fallback.take().ok_or(EINVAL)?;
        fence.source = Some(source);
        fence.scoped = self.cumulative.is_some();
        let submission = Aggregate(fence.into());
        self.submission = Some(submission.clone());
        if let Some(bundle) = &mut self.cumulative {
            bundle.publish(&submission.fence())?;
            self.syncs.truncate(self.input_count);
        } else {
            install(self.syncs.drain(self.input_count..), &submission.fence());
        }
        Ok(submission)
    }
    pub(crate) fn finish(mut self, result: Result) -> Result { self.result = result; result }
}
impl Drop for Outputs {
    fn drop(&mut self) {
        if let Some(submission) = self.submission.take() {
            if let Some(bundle) = &mut self.cumulative {
                bundle.fail(&submission.fence());
            }
            submission.finish_enqueue(self.result);
            return;
        }
        if let Some(fallback) = self.fallback.take() {
            if let Err(error) = self.result { fallback.set_error(error); }
            if let Some(bundle) = &mut self.cumulative {
                bundle.fail(&Fence::from_fence(&fallback));
            } else {
                install(self.syncs.drain(self.input_count..), &Fence::from_fence(&fallback));
            }
            fallback.signal();
        }
    }
}
