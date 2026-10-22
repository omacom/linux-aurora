// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Device ownership of physical queues and off-lock construction reservations.
//!
//! The device mutex protects this registry. Logical queues may disappear while
//! their published graphs remain here; only exact retirement permits rebinding,
//! and only processor stop permits dropping installed firmware addresses.
//! A preparation lease keeps Shared::state populated: shutdown and loss join
//! every lease before taking the session, including builders asleep on changed.

mod backend;
mod compute_control;
mod compute_replay;
mod compute_exit;
mod events;
mod feed;
mod render;
pub(super) mod teardown;

pub(super) use super::now_ns;
use super::{
    buffer::{BufferIds, MetricsIds},
    context::{Context, QosIds},
    fw::queue::DataMaster,
    kick, qos,
    queue::{self, admission, compute},
};
use core::sync::atomic::{fence, Ordering};
use kernel::{prelude::*, sync::Arc};

const QIDS: usize = 128;
const FIRST_COMPUTE_QID: u8 = 4;
const RECOVERY_SETTLE_POLLS: usize = 200;
const WORK_RING_WAIT_NS: u64 = super::job::TIMEOUT_MS as u64 * 1_000_000;
/// Bound on all contention waits of one publication together: capacity and
/// identities held by other live owners, borrowed pairs, work states, queue
/// binding and ring backpressure. Each frees when published work retires or
/// times out, when warming finishes or is reclaimed, or when an owner closes.
const PUBLICATION_WAIT_NS: u64 = 4 * WORK_RING_WAIT_NS;

/// Internal result of a publication attempted from the submitting ioctl that
/// met contention before anything became visible: the scheduler publishes the
/// command instead. Never returned to userspace.
pub(super) const FALLBACK: Error = EINPROGRESS;

/// How a publication may wait for contention.
#[derive(Clone, Copy)]
pub(super) enum Wait {
    /// From the scheduler's worker, until one deadline for the whole publication.
    Until(u64),
    /// From the submitting ioctl, which holds the logical queue's mutex: never
    /// sleep for contention, return `FALLBACK`.
    Never,
}

impl Wait {
    pub(super) fn scheduled() -> Self {
        Self::Until(now_ns().saturating_add(PUBLICATION_WAIT_NS))
    }
    /// Before a contention wait: `FALLBACK` when it may not sleep at all,
    /// otherwise whether the publication's deadline has passed.
    pub(super) fn expired(self) -> Result<bool> {
        match self {
            Self::Never => Err(FALLBACK),
            Self::Until(deadline) => Ok(now_ns() >= deadline),
        }
    }
    pub(super) fn deadline(self) -> Option<u64> {
        match self {
            Self::Never => None,
            Self::Until(deadline) => Some(deadline),
        }
    }
}

/// Whether a publication with no queue may take another live owner's idle one:
/// when neither a fresh QID nor surplus render capacity remains, or once its
/// own off-lock attempt (`contended`) found no released queue, fresh QID or
/// reclaimable render spare. Surplus held by installed render pairs never
/// becomes compute capacity, so it alone must not keep a publication waiting.
fn takeover_allowed(compute_available: bool, render_surplus: usize, contended: bool) -> bool {
    contended || (!compute_available && render_surplus < 2)
}

/// A reservation remains identifiable while its backing is built without the
/// device mutex. Failed construction returns the slot, never its host identity.
#[derive(Clone, Copy)]
pub(super) struct ComputeReservation {
    slot: usize,
    pub(super) id: kick::Id,
    owner: u64,
}

struct ComputeEntry {
    reservation: ComputeReservation,
    queue: Option<KBox<compute::Queue>>,
}

pub(super) struct Registry {
    pub(super) qids: kick::QueueIds,
    pub(super) buffers: BufferIds,
    pub(super) accounting: qos::Accounting,
    pub(super) metrics: Arc<MetricsIds>,
    pub(super) qos_ids: Arc<QosIds>,
    pub(super) admission: Arc<admission::Pool>,
    render: render::RenderState,
    identities: queue::Identities,
    compute: KVec<Option<ComputeEntry>>,
    pub(super) compute_building: bool,
    compute_scheduler_building: bool,
    replay_outer_wait: Option<u64>,
    pub(super) observations: compute::retirement::Batch,
    worker_deferred: Option<DeferredBatch>,
    teardown: super::teardown::Pending,
}

impl Registry {
    /// Initialize inside the firmware allocation instead of staging the whole
    /// registry on the probe stack. No additional allocation is required.
    pub(super) fn new(
        clusters: u32,
        descriptor_flags: [u32; 2],
    ) -> impl Init<Self, Error> {
        kernel::try_init!(Self {
            qids <- kick::QueueIds::new().chain(|qids| {
                qids.reserve_initial_render()
            }),
            buffers <- BufferIds::new(),
            accounting: qos::Accounting::new(),
            metrics: Arc::new(MetricsIds::new(), GFP_KERNEL)?,
            qos_ids: QosIds::new()?,
            admission: admission::Pool::new()?,
            render: render::RenderState::new(clusters, descriptor_flags)?,
            identities: queue::Identities::new(),
            compute: {
                let mut compute = KVec::with_capacity(QIDS, GFP_KERNEL)?;
                for _ in 0..QIDS {
                    compute.push(None, GFP_KERNEL)?;
                }
                compute
            },
            compute_building: false,
            compute_scheduler_building: false,
            replay_outer_wait: None,
            observations <- compute::retirement::Batch::new(),
            worker_deferred: Some(DeferredBatch::worker()?),
            teardown: super::teardown::Pending::new()?,
        })
    }

    pub(super) fn compute(&self, owner: u64) -> Option<&compute::Queue> {
        self.compute
            .iter()
            .flatten()
            .filter_map(|entry| entry.queue.as_deref())
            .find(|queue| queue.owner() == Some(owner))
    }

    pub(super) fn compute_mut(&mut self, owner: u64) -> Option<&mut compute::Queue> {
        self.compute
            .iter_mut()
            .flatten()
            .filter_map(|entry| entry.queue.as_deref_mut())
            .find(|queue| queue.owner() == Some(owner))
    }

    /// An installed idle queue keeps its original QID and backing. Mapping a
    /// replacement context must succeed before its old binding is displaced.
    /// A vacant queue (USC backing given back) is offered for re-arming only
    /// when no populated idle queue exists.
    pub(super) fn rebind_compute(&mut self, owner: u64, context: &Arc<Context>, required: usize) -> Result<Rebind> {
        if let Some(queue) = self.compute(owner) {
            return Ok(if queue.matches(owner, queue.qid(), context)? {
                if queue.pool_fits(required) { Rebind::Done } else { Rebind::Grow }
            } else {
                Rebind::Fresh
            });
        }
        let mut vacant = None;
        let teardown = &self.teardown;
        for queue in self
            .compute
            .iter_mut()
            .flatten()
            .filter_map(|entry| entry.queue.as_deref_mut())
            .filter(|queue| queue.reusable())
        {
            // The release of the scheduler these records were bound to is armed
            // or not yet consumed: the firmware may process it against this
            // queue, which therefore must not change owner until it has.
            if queue.context().is_some_and(|bound| teardown.releasing(bound)) {
                continue;
            }
            if queue.pool_vacant() {
                vacant.get_or_insert(queue.qid());
                continue;
            }
            if !queue.pool_fits(required) { continue; }
            queue.bind_owner(owner, context.clone())?;
            return Ok(Rebind::Done);
        }
        Ok(vacant.map_or(Rebind::Fresh, Rebind::Rearm))
    }

    /// No fresh QID and no released queue remain: move the least recently used
    /// idle queue of another live owner to `owner`, preferring one bound to the
    /// same VM. Only a pool that already fits is taken; growth would have to
    /// wait for a publication of the new owner. The previous owner keeps its
    /// context and takes a queue again at its next publication.
    pub(super) fn take_over_idle_compute(
        &mut self,
        owner: u64,
        context: &Arc<Context>,
        required: usize,
    ) -> Result<bool> {
        if !context.is_current() {
            return Err(EFAULT);
        }
        let mut selected = None::<(bool, u64, usize)>;
        for (index, entry) in self.compute.iter().enumerate() {
            let Some(queue) = entry.as_ref().and_then(|entry| entry.queue.as_deref()) else {
                continue;
            };
            if queue.owner() == Some(owner)
                || !queue.idle_owned()
                || !queue.pool_fits(required)
                || queue.context().is_some_and(|bound| self.teardown.releasing(bound))
            {
                continue;
            }
            let foreign = !queue
                .bound_status()
                .is_some_and(|status| Arc::ptr_eq(status, context.status()));
            let key = (foreign, queue.last_end(), index);
            if selected.is_none_or(|best| key < best) {
                selected = Some(key);
            }
        }
        let Some((_, _, index)) = selected else {
            return Ok(false);
        };
        let queue = self.compute[index]
            .as_mut()
            .and_then(|entry| entry.queue.as_deref_mut())
            .ok_or(EIO)?;
        queue.yield_idle_owner()?;
        queue.bind_owner(owner, context.clone())?;
        Ok(true)
    }

    /// Publication's device-locked binding: a released queue that fits, or a
    /// takeover when neither a fresh QID nor surplus render capacity remains,
    /// or once an off-lock attempt of this publication (`contended`) found no
    /// released queue, fresh QID or reclaimable render spare. Render surplus
    /// held by installed pairs cannot become compute capacity, so it must not
    /// keep a publication from taking an idle queue. False leaves
    /// construction, re-arming or growth to the off-lock path.
    pub(super) fn rebind_or_take_over_compute(
        &mut self,
        owner: u64,
        context: &Arc<Context>,
        required: usize,
        contended: bool,
    ) -> Result<bool> {
        match self.rebind_compute(owner, context, required)? {
            Rebind::Done => Ok(true),
            Rebind::Fresh
                if takeover_allowed(
                    self.qids.compute_available(self.admission.queues()),
                    self.qids.render_surplus(self.admission.queues()),
                    contended,
                ) =>
            {
                self.take_over_idle_compute(owner, context, required)
            }
            _ => Ok(false),
        }
    }

    /// Installs fresh USC backing into the vacant queue `qid`; the caller writes
    /// the returned descriptor row before binding. EAGAIN if the queue was taken.
    pub(super) fn rearm_compute(
        &mut self,
        qid: u8,
        pages: crate::g17::freelist::Pages,
        ring: compute::DescriptorRing,
    ) -> Result<(u16, u64)> {
        let queue = self
            .compute
            .iter_mut()
            .flatten()
            .filter_map(|entry| entry.queue.as_deref_mut())
            .find(|queue| queue.qid() == qid)
            .ok_or(EAGAIN)?;
        queue.rearm(pages, ring)
    }

    pub(super) fn bind_rearmed(&mut self, qid: u8, owner: u64, context: &Arc<Context>) -> Result {
        let teardown = &self.teardown;
        let queue = self
            .compute
            .iter_mut()
            .flatten()
            .filter_map(|entry| entry.queue.as_deref_mut())
            .find(|queue| queue.qid() == qid && queue.reusable())
            .ok_or(EAGAIN)?;
        // Its last owner may have closed while the backing was rebuilt.
        if queue.context().is_some_and(|bound| teardown.releasing(bound)) {
            return Err(EAGAIN);
        }
        queue.bind_owner(owner, context.clone())
    }

    /// Delay before retrying pools a turn could not release.
    const POOL_RETRY_NS: u64 = 100_000_000;
    /// Publishes releases for the oldest idle retained pools beyond `cap` and
    /// for every pool released at least `expiry` before `now`, at most four
    /// per pass. Returns the number published and the delay until the next
    /// retained pool expires or a retry is due.
    pub(super) fn vacate_idle_pools(
        &mut self,
        init: &super::initdata::InitData,
        cap: usize,
        now: u64,
        expiry: u64,
    ) -> Result<(usize, Option<u64>)> {
        // Count first: event turns are frequent and the steady state has no excess.
        let mut forced = 0;
        let mut count = 0;
        let mut next = None::<u64>;
        for queue in self
            .compute
            .iter()
            .flatten()
            .filter_map(|entry| entry.queue.as_deref())
            .filter(|queue| queue.vacate_candidate())
        {
            count += 1;
            let released = queue.released_at();
            let age = now.saturating_sub(released);
            if queue.needs_fresh_backing() || (released != 0 && age >= expiry) {
                forced += 1;
            } else if released != 0 {
                let left = expiry - age;
                next = Some(next.map_or(left, |next| next.min(left)));
            }
        }
        if count <= cap && forced == 0 {
            return Ok((0, next));
        }
        let mut candidates: KVec<(u64, u8)> = KVec::with_capacity(count, GFP_KERNEL)?;
        for queue in self
            .compute
            .iter()
            .flatten()
            .filter_map(|entry| entry.queue.as_deref())
            .filter(|queue| queue.vacate_candidate())
        {
            // Recycled killed queues sort first (time 0) and count beyond the cap.
            let key = if queue.needs_fresh_backing() { 0 } else { queue.released_at() };
            candidates.push((key, queue.qid()), GFP_KERNEL)?;
        }
        // Recycled and expired pools are the oldest keys and sort first.
        candidates.sort_unstable();
        let excess = candidates.len().saturating_sub(cap).max(forced);
        let mut published = 0;
        for (_, qid) in candidates.iter().take(excess.min(4)) {
            let queue = self
                .compute
                .iter_mut()
                .flatten()
                .filter_map(|entry| entry.queue.as_deref_mut())
                .find(|queue| queue.qid() == *qid)
                .ok_or(EIO)?;
            match queue.vacate(init) {
                Err(error) if error == EAGAIN => break,
                result => result?,
            }
            published += 1;
        }
        if published < excess {
            // Beyond this turn's bound, or not yet releasable: retry soon.
            next = Some(next.map_or(Self::POOL_RETRY_NS, |next| next.min(Self::POOL_RETRY_NS)));
        }
        Ok((published, next))
    }

    /// Frees the backing of every pool whose release the firmware consumed.
    pub(super) fn observe_pool_releases(
        &mut self,
        init: &super::initdata::InitData,
        deferred: &mut DeferredBatch,
    ) -> Result<usize> {
        fence(Ordering::Acquire);
        let consumer = init.control_consumer()?.load(Ordering::Relaxed);
        let producer = init.control_producer()?.load(Ordering::Relaxed);
        let mut freed = 0;
        for queue in self
            .compute
            .iter_mut()
            .flatten()
            .filter_map(|entry| entry.queue.as_deref_mut())
        {
            if let Some(vacated) = queue.observe_vacancy(consumer, producer)? {
                deferred.defer_vacated(vacated);
                freed += 1;
            }
        }
        Ok(freed)
    }

    /// A retained queue in a pool-release transaction temporarily cannot bind.
    /// Distinguish that state from permanent physical-identity exhaustion.
    fn compute_admission_error(&self, error: Error) -> Error {
        if error == ENOSPC
            && self.compute.iter().flatten()
                .filter_map(|entry| entry.queue.as_deref())
                .any(|queue| queue.pool_release_pending())
        {
            EAGAIN
        } else {
            error
        }
    }

    pub(super) fn reserve_compute(&mut self, owner: u64) -> Result<ComputeReservation> {
        if owner == 0
            || self.compute.iter().flatten().any(|entry| {
                entry
                    .queue
                    .as_ref()
                    .map_or(entry.reservation.owner == owner, |queue| {
                        queue.owner() == Some(owner)
                    })
            })
        {
            return Err(EINVAL);
        }
        let slot = self
            .compute
            .iter()
            .position(Option::is_none)
            .ok_or(ENOSPC)?;
        let identity = self.identities.reserve(1)?.start;
        let id = self
            .qids
            .reserve_compute(identity, Some(FIRST_COMPUTE_QID), self.admission.queues())
            .map_err(|error| self.compute_admission_error(error))?;
        let reservation = ComputeReservation { slot, id, owner };
        self.compute[slot] = Some(ComputeEntry {
            reservation,
            queue: None,
        });
        Ok(reservation)
    }

    fn reserved(&mut self, reservation: ComputeReservation) -> Result<&mut ComputeEntry> {
        let entry = self
            .compute
            .get_mut(reservation.slot)
            .and_then(Option::as_mut)
            .ok_or(EIO)?;
        if entry.reservation.id != reservation.id
            || entry.reservation.owner != reservation.owner
            || entry.queue.is_some()
        {
            return Err(EIO);
        }
        Ok(entry)
    }

    /// The caller checked its preparation epoch and retained the same device
    /// session before placing this graph where publication can find it.
    pub(super) fn install_compute(
        &mut self,
        reservation: ComputeReservation,
        queue: &mut Option<KBox<compute::Queue>>,
    ) -> Result {
        let built = queue.as_ref().ok_or(EINVAL)?;
        if built.qid() != reservation.id.qid() || built.owner() != Some(reservation.owner) {
            return Err(EIO);
        }
        let entry = self.reserved(reservation)?;
        entry.queue = queue.take();
        Ok(())
    }

    pub(super) fn cancel_compute(&mut self, reservation: ComputeReservation) -> Result {
        self.reserved(reservation)?;
        self.qids.cancel_unpublished(reservation.id)?;
        self.compute[reservation.slot] = None;
        Ok(())
    }

    pub(super) fn release_compute(&mut self, owner: u64) -> Result {
        let Some(slot) = self.compute.iter().position(|entry| {
            entry
                .as_ref()
                .and_then(|entry| entry.queue.as_ref())
                .is_some_and(|queue| queue.owner() == Some(owner))
        }) else {
            return Ok(());
        };
        let entry = self.compute[slot].as_mut().ok_or(EIO)?;
        if entry.queue.as_mut().ok_or(EIO)?.release_owner()? {
            self.qids.cancel_unpublished(entry.reservation.id)?;
            self.compute[slot] = None;
        }
        Ok(())
    }
}

/// Outcome of offering an owner a retained compute queue.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(super) enum Rebind {
    /// The owner now holds an installed, populated queue.
    Done,
    Grow,
    /// Only a vacant queue is available; re-arm `qid` with fresh backing.
    Rearm(u8),
    /// No retained queue suits the owner: reserve a fresh QID.
    Fresh,
}

/// An immutable logical owner shared by its two scheduler entities. Installed
/// physical graphs retain the context independently of this frontend lifetime.
pub(super) struct Backend {
    shared: Arc<super::Shared>,
    pub(super) context: Arc<Context>,
    owner: u64,
    /// Render packets queued to the scheduler and not yet published.
    backlog: core::sync::atomic::AtomicU32,
    /// Compute packets queued to the scheduler and not yet published.
    compute_backlog: core::sync::atomic::AtomicU32,
    /// One compute binder per owner: submission and publication can both bind,
    /// and two concurrent binders could leave the owner with two queues.
    binding: core::sync::atomic::AtomicBool,
}

/// Clears the owner's compute-binder flag on every exit.
struct Binder<'a>(&'a core::sync::atomic::AtomicBool);
impl Drop for Binder<'_> {
    fn drop(&mut self) {
        self.0.store(false, Ordering::Release);
    }
}

impl Backend {
    pub(super) fn new(
        shared: &Arc<super::Shared>,
        vm: &crate::mmu::Vm,
        owner: u64,
        policy: super::fw::queue::Policy,
    ) -> Result<(Arc<Self>, admission::QueueSlot)> {
        // A closed queue keeps its application context until its scheduler
        // release is consumed. When only such contexts block this one, wait
        // for the teardown (bounded) instead of failing a live client; the
        // preparation lease is not held while sleeping.
        const RETAINED_CONTEXT_WAIT_NS: u64 = 4 * WORK_RING_WAIT_NS;
        let started = now_ns();
        let (lease, uat, pool, qos, execution, slot) = loop {
            let lease = shared.preparations.enter(true)?;
            let mut state = shared.state.lock();
            let firmware = (*state).as_deref_mut().ok_or(ENODEV)?;
            if shared.crashed.load(core::sync::atomic::Ordering::Acquire) {
                return Err(EIO);
            }
            let slot = firmware.queues.admission.reserve_queue().map_err(|error| {
                if error == ENOSPC {
                    resource_exhausted!("logical queues")
                } else {
                    error
                }
            })?;
            match firmware.uat.new_execution_context(vm) {
                Ok(execution) => {
                    break (
                        lease,
                        firmware.uat.clone(),
                        firmware.pool.clone(),
                        firmware.queues.qos_ids.clone(),
                        execution,
                        slot,
                    )
                }
                Err(error) if error == ENOSPC => {
                    // Every open queue holds one context; this one is counted
                    // in `queues()` already but holds none yet.
                    let open = firmware.queues.admission.queues().saturating_sub(1);
                    let closing = firmware.uat.execution_contexts_in_use().saturating_sub(open);
                    if closing == 0 || now_ns().saturating_sub(started) >= RETAINED_CONTEXT_WAIT_NS {
                        return Err(resource_exhausted!("execution contexts"));
                    }
                    drop(slot);
                    drop(state);
                    drop(lease);
                    // Nothing is visible yet: a signalled creator restarts.
                    if kernel::current!().signal_pending() {
                        return Err(ERESTARTSYS);
                    }
                    shared.queue_events();
                    kernel::time::delay::fsleep(kernel::time::Delta::from_millis(1));
                }
                Err(error) => return Err(error),
            }
        };
        let dev = shared.drm()?;
        let alloc = super::object::Allocator {
            dev: &dev,
            uat: &uat,
        };
        let owner_pid =
            u32::try_from(kernel::current!().group_leader().pid()).map_err(|_| EOVERFLOW)?;
        let context = Arc::new(
            Context::new(&alloc, &pool, &qos, execution, owner_pid, policy)?,
            GFP_KERNEL,
        )?;
        let state = shared.state.lock();
        if !lease.is_current() || state.is_none() {
            return Err(EAGAIN);
        }
        drop(state);
        Ok((
            Arc::new(
                Self {
                    shared: shared.clone(),
                    context,
                    owner,
                    backlog: core::sync::atomic::AtomicU32::new(0),
                    compute_backlog: core::sync::atomic::AtomicU32::new(0),
                    binding: core::sync::atomic::AtomicBool::new(false),
                },
                GFP_KERNEL,
            )?,
            slot,
        ))
    }

    /// The shared aperture has exactly one builder. Other queues sleep without
    /// holding the device mutex; recovery joins its lease before closing roots.
    fn shared_compute(&self, wait: Wait) -> Result {
        loop {
            let lease = self.enter_preparation(wait)?;
            let dev = self.shared.drm()?;
            let uat = {
                let mut state = self.shared.state.lock();
                loop {
                    let firmware = (*state).as_deref_mut().ok_or(ENODEV)?;
                    if self
                        .shared
                        .crashed
                        .load(core::sync::atomic::Ordering::Acquire)
                    {
                        return Err(EIO);
                    }
                    if !lease.is_current() {
                        return Err(EAGAIN);
                    }
                    if firmware.init.has_compute_shared() {
                        return firmware.init.cache_compute_vm(self.context.vm());
                    }
                    if !firmware.queues.compute_building {
                        firmware.queues.compute_building = true;
                        break firmware.uat.clone();
                    }
                    if wait.expired()? {
                        return Err(self.wait_expired(EBUSY, "the shared compute aperture"));
                    }
                    self.shared.changed.wait(&mut state);
                }
            };
            let alloc = super::object::Allocator {
                dev: &dev,
                uat: &uat,
            };
            let mut built = Some(super::initdata::ComputeShared::new(&alloc));
            let result = {
                let mut state = self.shared.state.lock();
                let firmware = (*state).as_deref_mut().ok_or(ENODEV)?;
                if !lease.is_current() {
                    Err(EAGAIN)
                } else {
                    built
                        .take()
                        .ok_or(EIO)?
                        .and_then(|built| firmware.init.install_compute_shared(built))
                        .and_then(|()| firmware.init.cache_compute_vm(self.context.vm()))
                }
            };
            drop(built);
            {
                let mut state = self.shared.state.lock();
                (*state)
                    .as_deref_mut()
                    .ok_or(ENODEV)?
                    .queues
                    .compute_building = false;
            }
            self.shared.changed.notify_all();
            drop(lease);
            if result != Err(EAGAIN) {
                return result;
            }
        }
    }

    /// Uses the queue builder's existing lease. Entering another closed epoch
    /// here would make recovery wait for the very builder blocked on reopening.
    fn compute_scheduler(
        &self,
        alloc: &super::object::Allocator<'_>,
        lease: &super::preparation::Lease,
    ) -> Result {
        {
            let mut state = self.shared.state.lock();
            loop {
                let firmware = (*state).as_deref_mut().ok_or(ENODEV)?;
                if !lease.is_current() {
                    return Err(EAGAIN);
                }
                if firmware.init.has_compute_scheduler() {
                    return Ok(());
                }
                if !firmware.queues.compute_scheduler_building {
                    firmware.queues.compute_scheduler_building = true;
                    break;
                }
                self.shared.changed.wait(&mut state);
            }
        }
        let mut built = Some(super::initdata::ComputeScheduler::new(alloc));
        let result = {
            let mut state = self.shared.state.lock();
            let firmware = (*state).as_deref_mut().ok_or(ENODEV)?;
            if !lease.is_current() {
                Err(EAGAIN)
            } else {
                built
                    .take()
                    .ok_or(EIO)?
                    .and_then(|built| firmware.init.install_compute_scheduler(built))
            }
        };
        drop(built);
        {
            let mut state = self.shared.state.lock();
            (*state)
                .as_deref_mut()
                .ok_or(ENODEV)?
                .queues
                .compute_scheduler_building = false;
        }
        self.shared.changed.notify_all();
        result
    }

    fn grow_compute(&self, required: usize, wait: Wait) -> Result {
        use kernel::time::{delay::fsleep, Delta};
        {
            let state = self.shared.state.lock();
            let firmware = state.as_ref().ok_or(ENODEV)?;
            if firmware.queues.compute(self.owner).is_some_and(|queue| queue.pool_fits(required)) {
                return Ok(());
            }
        }
        let lease = self.enter_preparation(wait)?;
        let dev = self.shared.drm()?;
        let uat = self.shared.state.lock().as_ref().ok_or(ENODEV)?.uat.clone();
        let alloc = super::object::Allocator { dev: &dev, uat: &uat };
        let mut pages = Some(super::freelist::Pages::compute_sized(&alloc, required)?);
        let mut alias = Some(pages.as_ref().ok_or(EIO)?.map_client(self.context.vm())?);
        let mut deferred = DeferredBatch::publication()?;
        loop {
            let result = (|| {
                let mut state = self.shared.state.lock();
                let firmware = (*state).as_deref_mut().ok_or(ENODEV)?;
                if !lease.is_current() { return Err(EAGAIN); }
                if self.shared.crashed.load(Ordering::Acquire) { return Err(EIO); }
                firmware.scan_compute(None, &mut deferred)?;
                let queue = firmware.queues.compute_mut(self.owner).ok_or(EIO)?;
                if queue.pool_fits(required) { return Ok(None); }
                if queue.begin_growth(&firmware.init, required)? {
                    fence(Ordering::SeqCst);
                    firmware.primary.notify(crate::g17::MSG_CONTROL_NOTIFY)?;
                }
                queue.finish_growth(&firmware.init, &mut pages, &mut alias, required).map(Some)
            })();
            deferred.finish();
            match result {
                Ok(old) => { drop(old); return Ok(()); }
                Err(error) if error == EAGAIN && lease.is_current() => {
                    if wait.expired()? {
                        return Err(self.wait_expired(error, "compute pool growth"));
                    }
                    self.shared.queue_events();
                    fsleep(Delta::from_millis(1));
                }
                Err(error) => return Err(error),
            }
        }
    }

    /// Off-lock binding: a released queue, a re-armed vacant one, growth, or a
    /// fresh QID. ENOSPC leaves the owner unbound. Taking over another live
    /// owner's idle queue happens only in publication's device-locked rebind,
    /// which publishes under the same mutex hold. Submission never waits for
    /// or fails on physical capacity: it leaves binding to publication.
    fn build_compute(&self, required: usize, wait: Wait) -> Result {
        loop {
            match self.shared_compute(wait) {
                Err(error) if error == EAGAIN => continue,
                result => {
                    result?;
                    break;
                }
            }
        }
        loop {
            let lease = self.enter_preparation(wait)?;
            let dev = self.shared.drm()?;
            let (uat, reservation) = {
                let mut state = self.shared.state.lock();
                let firmware = (*state).as_deref_mut().ok_or(ENODEV)?;
                if self
                    .shared
                    .crashed
                    .load(core::sync::atomic::Ordering::Acquire)
                {
                    return Err(EIO);
                }
                match firmware.queues.rebind_compute(self.owner, &self.context, required)? {
                    Rebind::Done => return Ok(()),
                    Rebind::Grow => { drop(state); return self.grow_compute(required, wait); }
                    Rebind::Rearm(qid) => (firmware.uat.clone(), Err(qid)),
                    Rebind::Fresh => {
                        let reservation = match firmware.queues.reserve_compute(self.owner) {
                            Ok(reservation) => reservation,
                            // A pool release is still in flight: the event worker
                            // observes it and drops the vacated backing off this
                            // mutex. The caller retries after it.
                            Err(error) if error == EAGAIN => {
                                drop(state);
                                self.shared.queue_events();
                                return Err(ENOSPC);
                            }
                            Err(error) if error == ENOSPC => {
                                // Cancelling an unused render reservation frees
                                // compute capacity only beyond the render reserve.
                                let surplus = firmware
                                    .queues
                                    .qids
                                    .render_surplus(firmware.queues.admission.queues());
                                let reclaimed = if surplus >= 2 {
                                    firmware.queues.reclaim_unused_render_spare(None, true)?
                                } else {
                                    None
                                };
                                if reclaimed.is_none() {
                                    return Err(error);
                                }
                                drop(state);
                                drop(reclaimed);
                                continue;
                            }
                            Err(error) => return Err(error),
                        };
                        (firmware.uat.clone(), Ok(reservation))
                    }
                }
            };
            let alloc = super::object::Allocator {
                dev: &dev,
                uat: &uat,
            };
            let reservation = match reservation {
                Ok(reservation) => reservation,
                Err(qid) => {
                    // Fresh zeroed backing is built without the device mutex; the
                    // queue may have been taken or re-armed by another owner since.
                    let pages = crate::g17::freelist::Pages::compute_sized(&alloc, required)?;
                    let ring = compute::DescriptorRing::new(&alloc)?;
                    let mut state = self.shared.state.lock();
                    let firmware = (*state).as_deref_mut().ok_or(ENODEV)?;
                    if !lease.is_current() {
                        continue;
                    }
                    let (slot, page_list) = match firmware.queues.rearm_compute(qid, pages, ring) {
                        Err(error) if error == EAGAIN => continue,
                        result => result?,
                    };
                    // The firmware consumed this slot's release before the backing
                    // was dropped; the row is rewritten before the queue is kicked.
                    firmware.init.initialize_compute_pool_blocks(slot, page_list, required)?;
                    firmware.queues.bind_rearmed(qid, self.owner, &self.context)?;
                    return Ok(());
                }
            };
            let built = (|| {
                let pool = compute::PoolBacking::new(&alloc, reservation.id, required)?;
                {
                    let mut state = self.shared.state.lock();
                    let firmware = (*state).as_deref_mut().ok_or(ENODEV)?;
                    if !lease.is_current() {
                        return Err(EAGAIN);
                    }
                    firmware.queues.reserved(reservation)?;
                    let (slot, page_list) = pool.pool_descriptor()?;
                    firmware.init.initialize_compute_pool_blocks(slot, page_list, required)?;
                }
                let backing = compute::Backing::new(&alloc, pool)?;
                // The global scheduler is created after pool/kick backing and
                // retained even if this queue's later descriptor graph fails.
                self.compute_scheduler(&alloc, &lease)?;
                let queue = compute::Queue::new(&alloc, backing, self.owner, self.context.clone())?;
                Ok(KBox::new(queue, GFP_KERNEL)?)
            })();
            let (mut built, build_error) = match built {
                Ok(queue) => (Some(queue), None),
                Err(error) => (None, Some(error)),
            };
            let result = {
                let mut state = self.shared.state.lock();
                let firmware = (*state).as_deref_mut().ok_or(ENODEV)?;
                if !lease.is_current() {
                    Err(EAGAIN)
                } else if let Some(error) = build_error {
                    Err(error)
                } else {
                    firmware.queues.install_compute(reservation, &mut built)
                }
            };
            // The old aliases must disappear before a waiting constructor can
            // reuse the QID and its queue-local free-list identity. Keep the
            // preparation lease until both mappings and reservation are gone.
            drop(built);
            if result.is_err() {
                let mut state = self.shared.state.lock();
                (*state)
                    .as_mut()
                    .ok_or(ENODEV)?
                    .queues
                    .cancel_compute(reservation)?;
            }
            drop(lease);
            if result != Err(EAGAIN) {
                return result;
            }
        }
    }
}

/// One device transaction can fail each retained kick once and then retire it.
/// The physical namespace bounds the batch independently of open logical queues.
/// Collection failure leaves the engine's Active owner intact for a stopped retry.
pub(super) struct DeferredBatch {
    values: KVVec<super::completion::Deferred>,
    /// Exited owners detached from idle retained compute queues and the backing
    /// of vacated pools: unmapping and freeing happen in `finish`, off-lock.
    bindings: KVVec<super::queue::compute::Binding>,
    vacated: KVVec<super::queue::compute::Vacated>,
    /// Backing of render pools vacated for evicted idle clients.
    render: KVVec<super::freelist::RenderBacking>,
    /// Tiler-heap extensions removed from idle heaps, with their aliases.
    tvb: KVVec<super::buffer::TvbRelease>,
}
impl DeferredBatch {
    const DEPTH: usize = if crate::hw::t8140::queues::RENDER_DEPTH as usize
        > crate::hw::t8140::queues::COMPUTE_DEPTH
    {
        crate::hw::t8140::queues::RENDER_DEPTH as usize
    } else {
        crate::hw::t8140::queues::COMPUTE_DEPTH
    };

    /// Reused by the sole event work item; no allocation on a completion pass.
    fn worker() -> Result<Self> {
        Self::with_capacity(2 * QIDS * Self::DEPTH, QIDS)
    }
    /// One publication can quarantine only its selected physical queue/pair.
    /// VM-wide settlement is a separate transaction before publishing its error.
    fn publication() -> Result<Self> {
        Self::with_capacity(2 * Self::DEPTH, 0)
    }
    fn with_capacity(capacity: usize, detached: usize) -> Result<Self> {
        let render_slots = if detached != 0 {
            crate::hw::t8140::queues::RENDER_SLOTS
        } else {
            0
        };
        Ok(Self {
            values: KVVec::with_capacity(capacity, GFP_KERNEL)?,
            bindings: KVVec::with_capacity(detached, GFP_KERNEL)?,
            vacated: KVVec::with_capacity(detached, GFP_KERNEL)?,
            render: KVVec::with_capacity(render_slots, GFP_KERNEL)?,
            tvb: KVVec::with_capacity(render_slots, GFP_KERNEL)?,
        })
    }
    /// Keeps the detached owner's unmapping and VM teardown out of the device
    /// mutex. Without reserved room the binding is dropped here instead.
    pub(super) fn defer_binding(&mut self, binding: super::queue::compute::Binding) {
        if let Err(binding) = self.bindings.push_within_capacity(binding) {
            drop(binding);
        }
    }
    /// Room for one more vacated render backing; the worker vacates no more.
    pub(super) fn render_backing_room(&self) -> bool {
        self.render.len() < self.render.capacity()
    }
    pub(super) fn defer_render_backing(&mut self, backing: super::freelist::RenderBacking) {
        if let Err(backing) = self.render.push_within_capacity(backing) {
            drop(backing);
        }
    }
    pub(super) fn tvb_room(&self) -> bool {
        self.tvb.len() < self.tvb.capacity()
    }
    pub(super) fn defer_tvb(&mut self, release: super::buffer::TvbRelease) {
        if let Err(release) = self.tvb.push_within_capacity(release) {
            drop(release);
        }
    }
    pub(super) fn defer_vacated(&mut self, vacated: super::queue::compute::Vacated) {
        if let Err(vacated) = self.vacated.push_within_capacity(vacated) {
            drop(vacated);
        }
    }
    /// A recovery pass can fail and retire each retained command at most once.
    /// Reserve before changing queues, plus a full publication for its caller.
    /// Healthy submissions keep the small publication collector unchanged.
    fn reserve_replays(&mut self, commands: usize) -> Result {
        self.values
            .reserve(2 * commands + 2 * Self::DEPTH, GFP_KERNEL)?;
        Ok(())
    }
    pub(super) fn push(&mut self, value: super::completion::Deferred) -> Result {
        use super::completion::Deferred;
        // Publishers may acquire the device mutex as soon as this pass drops
        // it. Expose terminal VM status now, before off-lock fence callbacks.
        // Recovery-spared and cancelled commands never poison their VM.
        if let Deferred::FailedOwned(completion, error) = &value {
            if *error != ECANCELED && !completion.spared() {
                completion.status().record(*error);
            }
        }
        self.values.push_within_capacity(value).map_err(|_| EIO)?;
        // A later publisher can borrow this pair before finish signals its old
        // fence. Publish the exact completion's witness only after collection
        // succeeds; failure fences and rejected events are not retirement.
        if let Some(
            Deferred::Retired(completion, _)
            | Deferred::Drained(completion, _)
            | Deferred::Release(completion),
        ) = self.values.last()
        {
            completion.note_retirement_queued();
        }
        Ok(())
    }
    /// Called only after releasing the device mutex.
    pub(super) fn finish(&mut self) {
        for value in self.values.drain_all() {
            value.finish();
        }
        for binding in self.bindings.drain_all() {
            drop(binding);
        }
        for vacated in self.vacated.drain_all() {
            drop(vacated);
        }
        for backing in self.render.drain_all() {
            drop(backing);
        }
        for release in self.tvb.drain_all() {
            drop(release);
        }
    }
}

struct ComputeHost<'a> {
    init: &'a super::initdata::InitData,
    primary: &'a mut super::Coprocessor,
    effort: &'a mut super::power::Effort,
    qids: &'a mut kick::QueueIds,
    accounting: &'a mut qos::Accounting,
    generation: u64,
}
impl compute::Host for ComputeHost<'_> {
    fn next_compute_publication(&mut self) -> Result<u64> {
        self.qids.next_compute_publication()
    }
    fn epoch(&self) -> Result<(u64, u32)> {
        Ok((
            self.generation,
            super::recovery::Memory::recovery_state(self.init)?,
        ))
    }
    fn prepare_compute_shared(&mut self) -> Result {
        self.init.activate_compute()
    }
    fn qos_publish(
        &mut self,
        owner: qos::Owner,
        scheduler: u64,
        policy: super::fw::queue::Policy,
    ) -> Result<qos::Publication> {
        self.accounting.publish(
            &qos::View::new(self.init.qos()?)?,
            owner,
            scheduler,
            policy,
            qos::clock,
        )
    }
    fn qos_cancel(&mut self, publication: qos::Publication) -> Result {
        self.accounting
            .cancel(&qos::View::new(self.init.qos()?)?, publication)
    }
    fn publish_qid(&mut self, id: kick::Id) -> Result {
        self.qids.publish(id)
    }
    fn publish_outer(&mut self, priority: u8, slot: &super::fw::channels::WorkSlot) -> Result {
        self.init.publish_work(priority, slot).map(|_| ())
    }
    fn notify(&mut self, message: u64) -> Result {
        self.primary.notify(message)
    }
    fn note_submission(&mut self) -> Result {
        self.effort.submit();
        super::Firmware::apply_shared_effort(self.init, self.effort, super::power::FULL_EFFORT)
    }
}

impl super::Firmware {
    /// One snapshot of every finite queue resource.
    pub(in crate::g17) fn capacity(&self) -> render::Capacity {
        self.queues.capacity(self.render_ids.in_use(), self.uat.execution_contexts_in_use())
    }

    fn publish_compute(
        &mut self,
        owner: u64,
        packet: Arc<super::job::Packet>,
        dependencies: &[super::fw::kick::KickDependency],
        deferred: &mut DeferredBatch,
    ) -> Result {
        if self.recovery.pending() {
            return Err(EAGAIN);
        }
        // No descriptor, accounting or inner producer changes precede this
        // capacity check. The same device lock covers queue.publish below.
        if !self
            .init
            .work_ready(packet.context.policy().priority(), DataMaster::Compute)?
        {
            return Err(EAGAIN);
        }
        let queues = &mut self.queues;
        let queue = queues
            .compute
            .iter_mut()
            .flatten()
            .filter_map(|entry| entry.queue.as_deref_mut())
            .find(|queue| queue.owner() == Some(owner))
            .ok_or(EIO)?;
        // The first command after an owner change waits while the scheduler
        // the queue was bound to is being released: a live previous owner may
        // have closed after giving up the queue.
        if queue
            .pending_switch_from()
            .is_some_and(|previous| queues.teardown.releasing(previous))
        {
            return Err(EAGAIN);
        }
        let mut host = ComputeHost {
            init: &self.init,
            primary: &mut self.primary,
            effort: &mut self.effort,
            qids: &mut queues.qids,
            accounting: &mut queues.accounting,
            generation: self.recovery.generation(),
        };
        queue.publish(packet, dependencies, &mut host, &mut |event| {
            deferred.push(event)
        })
    }
}

impl Registry {
    fn release_compute_witness(
        &mut self,
        init: &super::initdata::InitData,
        owner: u64,
        deferred: &mut DeferredBatch,
    ) -> Result<bool> {
        let Some(queue) = self
            .compute
            .iter_mut()
            .flatten()
            .filter_map(|entry| entry.queue.as_deref_mut())
            .find(|queue| queue.owner() == Some(owner))
        else {
            return Ok(false);
        };
        if !queue.spared_pending() {
            return Ok(false);
        }
        let view = qos::View::new(init.qos()?)?;
        queue.release_witnessed(
            super::recovery::Memory::recovery_state(init)?,
            |owner| self.accounting.complete(&view, &[owner]),
            &mut |event| deferred.push(event),
        )?;
        Ok(queue.spared_pending())
    }

    fn fail_compute_vm(
        &mut self,
        status: &Arc<super::status::VmStatus>,
        error: Error,
        deferred: &mut DeferredBatch,
    ) -> Result {
        for queue in self
            .compute
            .iter_mut()
            .flatten()
            .filter_map(|entry| entry.queue.as_deref_mut())
        {
            if queue.in_flight()
                && queue
                    .context()
                    .is_some_and(|context| Arc::ptr_eq(context.status(), status))
            {
                queue.quarantine(error, &mut |event| deferred.push(event))?;
            }
        }
        Ok(())
    }
}

impl super::Firmware {
    /// Latch one shared snapshot for all selected queues before settling any VM.
    /// A later queue's exact success in this batch wins over a peer's failure.
    fn scan_compute(&mut self, masks: Option<[u64; 2]>, deferred: &mut DeferredBatch) -> Result {
        let state = super::recovery::Memory::recovery_state(&self.init)?;
        let generation = self.recovery.generation();
        let registry = &mut self.queues;
        registry.observations.clear();
        let mut requested = false;
        for queue in registry
            .compute
            .iter()
            .flatten()
            .filter_map(|entry| entry.queue.as_deref())
        {
            requested |= queue.request_observations(&mut registry.observations, masks)?;
        }
        if requested {
            self.init
                .scan_compute_completions(|record| registry.observations.fold(record))?;
        }
        let view = qos::View::new(self.init.qos()?)?;
        for queue in registry
            .compute
            .iter_mut()
            .flatten()
            .filter_map(|entry| entry.queue.as_deref_mut())
        {
            queue.latch_observations(&mut registry.observations, masks)?;
            queue.retire(
                masks,
                generation,
                state,
                |owner| registry.accounting.complete(&view, &[owner]),
                &mut |event| deferred.push(event),
            )?;
        }
        // All compute observations in this batch are latched before a VM
        // failure can settle its render peers. Exact successes win first.
        for index in 0..self.queues.compute.len() {
            let terminal = self.queues.compute[index]
                .as_mut()
                .and_then(|entry| entry.queue.as_deref_mut())
                .and_then(compute::Queue::take_terminal);
            if let Some((status, error)) = terminal {
                status.record(error);
                self.settle_render_vm(&status, error, deferred)?;
                self.queues.fail_compute_vm(&status, error, deferred)?;
            }
        }
        let view = qos::View::new(self.init.qos()?)?;
        let registry = &mut self.queues;
        for queue in registry
            .compute
            .iter_mut()
            .flatten()
            .filter_map(|entry| entry.queue.as_deref_mut())
        {
            queue.release_witnessed(
                state,
                |owner| registry.accounting.complete(&view, &[owner]),
                &mut |event| deferred.push(event),
            )?;
        }
        Ok(())
    }
}

impl Backend {
    /// Submission-time binding: never takes over another owner's queue and
    /// reports ENOSPC when no physical queue is free, leaving publication to bind.
    fn ensure_compute(&self, scs_pages: u16) -> Result {
        let required = super::fw::compute_pool::required_blocks(self.shared.active_cores, scs_pages)?;
        // The submit ioctl never sleeps for contention: publication binds.
        match self.bind_compute(required, Wait::Never) {
            Err(error) if error == FALLBACK => Err(EAGAIN),
            result => result,
        }
    }

    /// A publication gave up waiting for `what` at its deadline and fails with
    /// `error`. Names the wait, its source line and the owner, at most once a
    /// second, so a lost device reported for contention is attributable from
    /// the kernel log alone. Only reached on that failure path.
    #[track_caller]
    pub(super) fn wait_expired(&self, error: Error, what: impl kernel::fmt::Display) -> Error {
        use core::sync::atomic::AtomicU64;
        static NEXT: AtomicU64 = AtomicU64::new(0);
        static SUPPRESSED: AtomicU64 = AtomicU64::new(0);
        let now = now_ns();
        let next = NEXT.load(Ordering::Relaxed);
        if now < next
            || NEXT
                .compare_exchange(next, now + 1_000_000_000, Ordering::Relaxed, Ordering::Relaxed)
                .is_err()
        {
            SUPPRESSED.fetch_add(1, Ordering::Relaxed);
            return error;
        }
        let at = core::panic::Location::caller();
        dev_warn!(
            self.shared.dev.as_ref(),
            "G17 publication of pid {} gave up waiting for {} ({:?}, {}:{}); {} similar since the last report\n",
            self.context.owner_pid(),
            what,
            error,
            at.file().rsplit("asahi/").next().unwrap_or(at.file()),
            at.line(),
            SUPPRESSED.swap(0, Ordering::Relaxed)
        );
        error
    }

    /// The preparation lease. From the ioctl, a recovery that has closed the
    /// preparation epoch is contention.
    fn enter_preparation(&self, wait: Wait) -> Result<super::preparation::Lease> {
        match wait {
            Wait::Never => self
                .shared
                .preparations
                .try_enter()
                .map_err(|error| if error == EAGAIN { FALLBACK } else { error }),
            Wait::Until(_) => self.shared.preparations.enter(false),
        }
    }

    /// EAGAIN while another binder of this owner runs; it binds for both.
    fn bind_compute(&self, required: usize, wait: Wait) -> Result {
        use kernel::time::{delay::fsleep, Delta};
        if self
            .binding
            .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
            .is_err()
        {
            return Err(EAGAIN);
        }
        let _binder = Binder(&self.binding);
        let mut deferred = DeferredBatch::publication()?;
        loop {
            let pending = (|| {
                let mut state = self.shared.state.lock();
                let firmware = (*state).as_deref_mut().ok_or(ENODEV)?;
                firmware.replay_compute(&mut deferred)?;
                firmware
                    .queues
                    .release_compute_witness(&firmware.init, self.owner, &mut deferred)
            })();
            deferred.finish();
            let pending = pending?;
            if !pending {
                return self.build_compute(required, wait);
            }
            // Keep the spared queue and its retained work intact until a later
            // admission observes its witness. Other owners continue submitting
            // while this owner gives its queue-local drain witness time to arrive.
            if wait.expired()? {
                return Err(self.wait_expired(EBUSY, "a spared compute queue's drain witness"));
            }
            fsleep(Delta::from_millis(1));
        }
    }

    /// Publishes a dependency-free compute command from the submitting ioctl,
    /// as renders are, instead of waking the scheduler's worker for it. Only a
    /// queue already bound to this owner that fits the command and has room
    /// qualifies, and only while no earlier compute command of this owner
    /// still waits in the scheduler, so publication order stays submission
    /// order. The ioctl never binds, grows, replays or sleeps: any of those
    /// returns false before anything became visible and leaves the command to
    /// the scheduler.
    pub(in crate::g17) fn try_compute_early(&self, packet: &Arc<super::job::Packet>) -> bool {
        if self.compute_backlog.load(Ordering::Acquire) != 0 || packet.has_dependencies() {
            return false;
        }
        let super::command::Validated::Compute { scratch, .. } = &packet.command else {
            return false;
        };
        let Ok(required) =
            super::fw::compute_pool::required_blocks(self.shared.active_cores, scratch.pages())
        else {
            return false;
        };
        {
            let state = self.shared.state.lock();
            let Some(firmware) = (*state).as_deref() else {
                return false;
            };
            if firmware.recovery.pending()
                || self.binding.load(Ordering::Acquire)
                || !firmware
                    .queues
                    .compute(self.owner)
                    .is_some_and(|queue| queue.pool_fits(required) && queue.room())
            {
                return false;
            }
        }
        if !packet.begin_publication() {
            packet.completion.fail_unpublished(ECANCELED);
            packet.set_early_result(Err(ECANCELED));
            return true;
        }
        let mut deferred = match DeferredBatch::publication() {
            Ok(deferred) => deferred,
            Err(error) => {
                if packet.completion.status().get() != 0 {
                    <Self as super::job::Backend>::fail_vm(self, error);
                }
                packet.completion.fail_unpublished(error);
                packet.set_early_result(Err(error));
                return true;
            }
        };
        let result = self.publish_compute_early(packet, required, &mut deferred);
        if matches!(result, Err(error) if error == FALLBACK) && !packet.is_published() {
            deferred.finish();
            packet.abandon_publication();
            return false;
        }
        let result = self
            .finish_publication(packet, result, &mut deferred)
            .map(|_| ());
        packet.set_early_result(result);
        true
    }

    /// The ioctl's single publication attempt, under one device mutex hold on
    /// the owner's bound queue. Contention returns `FALLBACK` unpublished.
    fn publish_compute_early(
        &self,
        packet: &Arc<super::job::Packet>,
        required: usize,
        deferred: &mut DeferredBatch,
    ) -> Result {
        if packet.completion.status().get() != 0 {
            return Err(EIO);
        }
        let lease = self.enter_preparation(Wait::Never)?;
        match packet.completion.work_state()?.acquire() {
            Err(error) if error == EBUSY => return Err(FALLBACK),
            result => {
                result?;
            }
        }
        let mut state = self.shared.state.lock();
        let firmware = (*state).as_deref_mut().ok_or(ENODEV)?;
        if packet.cancel_requested() {
            return Err(ECANCELED);
        }
        if self.shared.crashed.load(Ordering::Acquire) {
            return Err(EIO);
        }
        if !lease.is_current()
            || firmware.recovery.pending()
            || super::recovery::Memory::recovery_state(&firmware.init)? != 0
            || self.binding.load(Ordering::Acquire)
            // Replays of recovered commands publish first, from the scheduler.
            || firmware
                .queues
                .compute
                .iter()
                .flatten()
                .filter_map(|entry| entry.queue.as_deref())
                .any(|queue| queue.replay_commands() != 0)
            || !firmware
                .queues
                .compute(self.owner)
                .is_some_and(|queue| queue.pool_fits(required) && queue.room())
        {
            return Err(FALLBACK);
        }
        let dependencies = firmware.queues.render_prefix(self.owner, packet)?;
        match firmware.publish_compute(self.owner, packet.clone(), dependencies.as_slice(), deferred)
        {
            Err(error) if !packet.is_published() && [EAGAIN, EBUSY].contains(&error) => {
                Err(FALLBACK)
            }
            result => result,
        }
    }

    /// Called after taking this packet's publication claim. All waits and work
    /// state allocation precede the device mutex; only the checked epoch commits.
    fn compute_packet(
        &self,
        packet: &Arc<super::job::Packet>,
        deferred: &mut DeferredBatch,
        wait: Wait,
    ) -> Result {
        packet.check_dependencies()?;
        let super::command::Validated::Compute { scratch, .. } = &packet.command else {
            return Err(EINVAL);
        };
        let required =
            super::fw::compute_pool::required_blocks(self.shared.active_cores, scratch.pages())?;
        let mut recovery_polls = 0;
        let mut state_delay = 20i64;
        let mut contended = false;
        loop {
            if packet.completion.status().get() != 0 {
                return Err(EIO);
            }
            let lease = self.enter_preparation(wait)?;
            // Work states free as this context's earlier commands retire.
            match packet.completion.work_state()?.acquire() {
                Err(error) if error == EBUSY => {
                    drop(lease);
                    if wait.expired()? {
                        return Err(self.wait_expired(error, "a free compute work state"));
                    }
                    self.shared.queue_events();
                    kernel::time::delay::fsleep(kernel::time::Delta::from_micros(state_delay));
                    state_delay = (state_delay * 2).min(1000);
                    continue;
                }
                result => {
                    result?;
                }
            }
            let mut state = self.shared.state.lock();
            let firmware = (*state).as_deref_mut().ok_or(ENODEV)?;
            if !lease.is_current() {
                drop(state);
                continue;
            }
            if packet.cancel_requested() {
                return Err(ECANCELED);
            }
            if self
                .shared
                .crashed
                .load(core::sync::atomic::Ordering::Acquire)
            {
                return Err(EIO);
            }
            let recovery_state = super::recovery::Memory::recovery_state(&firmware.init)?;
            if firmware.recovery.pending() || recovery_state != 0 {
                if wait.expired()? || recovery_polls == RECOVERY_SETTLE_POLLS {
                    return Err(self.wait_expired(EBUSY, "GPU recovery to settle"));
                }
                recovery_polls += 1;
                drop(state);
                drop(lease);
                self.shared.queue_events();
                kernel::time::delay::fsleep(kernel::time::Delta::from_millis(1));
                continue;
            }
            firmware.replay_compute(deferred)?;
            if !firmware
                .queues
                .compute(self.owner)
                .is_some_and(|queue| queue.pool_fits(required))
            {
                // Submission found no free physical queue, or another owner
                // took this idle one. Bind again; a takeover publishes under
                // the same mutex hold. Capacity frees whenever any queue's
                // published work retires or times out, so the wait is bounded.
                if wait.expired()? {
                    return Err(self.wait_expired(EBUSY, "a compute queue to bind"));
                }
                // An off-lock binder of this owner may hold a reservation or
                // a queue being re-armed: binding here could give the owner a
                // second queue. Its flag is raised before its first decision.
                let taken = !self.binding.load(Ordering::Acquire)
                    && firmware.queues.compute(self.owner).is_none()
                    && match firmware.queues.rebind_or_take_over_compute(
                        self.owner,
                        &self.context,
                        required,
                        contended,
                    ) {
                        Err(error) if error == ENOSPC => false,
                        result => result?,
                    };
                if !taken {
                    drop(state);
                    drop(lease);
                    // Replays collected above belong to other packets; do not
                    // hold their fences while this one waits for a queue.
                    deferred.finish();
                    match self.bind_compute(required, wait) {
                        Ok(()) => {}
                        Err(error) if [ENOSPC, EBUSY, EAGAIN].contains(&error) => {
                            // No released queue, fresh QID or spare: the next
                            // locked attempt may take an idle queue over.
                            contended |= error != EAGAIN;
                            self.shared.queue_events();
                            kernel::time::delay::fsleep(kernel::time::Delta::from_millis(1));
                        }
                        Err(error) => return Err(error),
                    }
                    continue;
                }
            }
            let dependencies = firmware.queues.render_prefix(self.owner, packet)?;
            let result = firmware.publish_compute(
                self.owner,
                packet.clone(),
                dependencies.as_slice(),
                deferred,
            );
            if matches!(result, Err(EBUSY) | Err(EAGAIN))
                && !packet.is_published()
                && (firmware.recovery.pending()
                    || super::recovery::Memory::recovery_state(&firmware.init)? != 0)
            {
                if wait.expired()? || recovery_polls == RECOVERY_SETTLE_POLLS {
                    return Err(self.wait_expired(EBUSY, "GPU recovery to settle"));
                }
                recovery_polls += 1;
                drop(state);
                drop(lease);
                self.shared.queue_events();
                kernel::time::delay::fsleep(kernel::time::Delta::from_millis(1));
                continue;
            }
            if matches!(result, Err(EAGAIN)) && !packet.is_published() {
                if wait.expired()? {
                    let releasing = firmware.queues.compute(self.owner).is_some_and(|queue| {
                        queue
                            .pending_switch_from()
                            .is_some_and(|previous| firmware.queues.teardown.releasing(previous))
                    });
                    return Err(self.wait_expired(
                        EBUSY,
                        if releasing {
                            "the previous owner's scheduler release"
                        } else {
                            "compute ring room"
                        },
                    ));
                }
                // Backpressure owns no firmware-visible attempt. Other queues
                // can submit and the event worker can retire while we sleep.
                drop(state);
                drop(lease);
                self.shared.queue_events();
                kernel::time::delay::fsleep(kernel::time::Delta::from_millis(1));
                continue;
            }
            return result;
        }
    }
}

impl super::Firmware {
    /// Both processors are stopped and their owner detached before this call.
    /// Error fences release their command mappings without the device mutex
    /// or an allocation-dependent collector.
    fn finish_stopped_queues(&mut self) {
        self.stop_render();
        for queue in self
            .queues
            .compute
            .iter_mut()
            .flatten()
            .filter_map(|entry| entry.queue.as_deref_mut())
        {
            let result = queue.stopped(&mut |event| {
                event.finish();
                Ok(())
            });
            if let Err(error) = result {
                dev_err!(
                    self.primary.state.shared.dev.as_ref(),
                    "Could not settle stopped compute queue: {:?}\n",
                    error
                );
            }
        }
        self.queues
            .admission
            .release_retained_owners_after_processor_stop();
    }
}

impl super::Shared {
    /// Joins construction and complete host teardown, including RTKit drops.
    /// The stop-only mutex outlives detached firmware destruction, so unbind
    /// cannot return while another stop caller still owns mailbox clients.
    /// Completion callbacks remain outside the device mutex. Direct handlers
    /// do not synchronously request or join stop; this path does not join
    /// scheduler workers.
    pub(super) fn stop_queues(&self) {
        self.preparations.wait_drained();
        let _stopping = self.stop.lock();
        let firmware = {
            let mut state = self.state.lock();
            if let Some(firmware) = (*state).as_deref_mut() {
                firmware.secondary.stop();
                firmware.primary.stop();
            }
            state.take()
        };
        if let Some(mut firmware) = firmware {
            firmware.finish_stopped_queues();
            // RTKit destruction still dereferences the mailbox provider.
            drop(firmware);
        }
    }
}

/// Idle retained compute queues that keep their USC pool populated; older idle
/// pools give their backing back.
const COMPUTE_WARM_CAP: usize = 4;
/// A retained pool idle this long gives its backing back regardless of the
/// cap: a burst of short-lived clients still finds warm pools, but the pools
/// of clients that are gone do not stay resident. The macOS driver frees a
/// client's USC pools with the client and has an idle shrink; ten seconds
/// matches its idle parameter-buffer shrink.
const COMPUTE_WARM_EXPIRY_NS: u64 = 10_000_000_000;

impl super::Firmware {
    /// Event-worker service step: witness consumed pool releases, then release the
    /// oldest idle retained pools beyond the warm cap.
    /// Detached owner bindings and vacated pool backing go to `deferred`, which
    /// the worker finishes after releasing the device mutex.
    /// Returns the delay until a retained pool expires, for the worker's wake.
    pub(in crate::g17) fn service_compute_pools(
        &mut self,
        deferred: &mut DeferredBatch,
        now: u64,
    ) -> Result<Option<u64>> {
        for queue in self
            .queues
            .compute
            .iter_mut()
            .flatten()
            .filter_map(|entry| entry.queue.as_deref_mut())
        {
            queue.recycle_killed()?;
            queue.recycle_quarantined_idle()?;
            if let Some(binding) = queue.detach_exited_owner() {
                deferred.defer_binding(binding);
            }
        }
        self.queues.observe_pool_releases(&self.init, deferred)?;
        if self.render_control_backpressured() {
            return Ok(None);
        }
        // While the shared lower window refuses free-list backing, no idle
        // retained compute pool stays warm: render and new compute need it more.
        let cap = if super::freelist::lower_pressure() {
            0
        } else {
            COMPUTE_WARM_CAP
        };
        let (published, next) = self.queues.vacate_idle_pools(
            &self.init,
            cap,
            now,
            COMPUTE_WARM_EXPIRY_NS,
        )?;
        if published != 0 {
            fence(Ordering::SeqCst);
            self.primary.notify(crate::g17::MSG_CONTROL_NOTIFY)?;
        }
        Ok(next)
    }
}

