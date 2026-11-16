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
use super::preparation::Admission;
use core::sync::atomic::{fence, Ordering};
use kernel::{prelude::*, sync::Arc};

const QIDS: usize = 128;
const FIRST_COMPUTE_QID: u8 = 4;
const WORK_RING_WAIT_NS: u64 = super::job::TIMEOUT_MS as u64 * 1_000_000;

/// Pending admission keeps exactly retired compute backing available for reuse.
/// The guard spans retries and disappears on publication, cancellation or error.
/// It borrows Shared, so the accounting cannot retain the device after teardown.
struct ComputeDemand<'a>(&'a Arc<super::Shared>);

impl<'a> ComputeDemand<'a> {
    fn new(shared: &'a Arc<super::Shared>) -> Self {
        // Each admitted logical queue has at most one scheduler publisher.
        // The execution-context bound is far below the counter's range.
        shared.compute_demand.fetch_add(1, Ordering::AcqRel);
        Self(shared)
    }
}

impl Drop for ComputeDemand<'_> {
    fn drop(&mut self) {
        if self.0.compute_demand.fetch_sub(1, Ordering::AcqRel) == 1 {
            // Reinstate normal idle retention even if the last publisher was
            // cancelled before a kick could produce another event.
            self.0.queue_events();
        }
    }
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
    queue: ComputeSlot,
}

/// A physical compute queue behind its own lock. Publication into a bound queue takes only
/// this lock; device-mutex holders take it to observe or change the queue (device mutex ->
/// cell). A logical queue may keep the cell after the registry gave the queue up: the cell is
/// then empty, so the cached reference never extends any firmware-visible lifetime.
#[pin_data]
pub(super) struct ComputeCell {
    #[pin]
    queue: kernel::sync::Mutex<Option<KBox<compute::Queue>>>,
}

impl ComputeCell {
    fn new() -> Result<Arc<Self>> {
        Arc::pin_init(
            pin_init!(Self {
                queue <- kernel::new_mutex!(None, "g17::ComputeCell::queue"),
            }),
            GFP_KERNEL,
        )
    }

    /// The installed queue, locked; None once the registry gave it up.
    pub(super) fn lock(&self) -> Option<QueueGuard<'_>> {
        QueueGuard::new(self.queue.lock())
    }
}

/// A locked installed queue.
pub(super) struct QueueGuard<'a> {
    queue: core::ptr::NonNull<compute::Queue>,
    _guard: kernel::sync::MutexGuard<'a, Option<KBox<compute::Queue>>>,
}

impl<'a> QueueGuard<'a> {
    fn new(mut guard: kernel::sync::MutexGuard<'a, Option<KBox<compute::Queue>>>) -> Option<Self> {
        let queue = core::ptr::NonNull::from((*guard).as_deref_mut()?);
        Some(Self { queue, _guard: guard })
    }
}

impl core::ops::Deref for QueueGuard<'_> {
    type Target = compute::Queue;
    fn deref(&self) -> &compute::Queue {
        // SAFETY: The boxed queue cannot be taken or dropped while the guard holds its lock.
        unsafe { self.queue.as_ref() }
    }
}

impl core::ops::DerefMut for QueueGuard<'_> {
    fn deref_mut(&mut self) -> &mut compute::Queue {
        // SAFETY: As for `deref`; the lock makes this the only reference.
        unsafe { self.queue.as_mut() }
    }
}

/// The registry's handle on one compute cell. Dropping it drops the queue in the cell, so a
/// cached cell never keeps a removed queue (or its QID's aliases) alive.
struct ComputeSlot(Arc<ComputeCell>);

impl ComputeSlot {
    fn new() -> Result<Self> {
        Ok(Self(ComputeCell::new()?))
    }
    fn as_deref(&self) -> Option<QueueGuard<'_>> {
        self.0.lock()
    }
    fn as_deref_mut(&mut self) -> Option<QueueGuard<'_>> {
        self.0.lock()
    }
    fn is_some(&self) -> bool {
        self.0.queue.lock().is_some()
    }
    fn is_none(&self) -> bool {
        !self.is_some()
    }
    /// Gives the queue up to the caller, which drops it outside the cell lock.
    fn take(&mut self) -> Option<KBox<compute::Queue>> {
        self.0.queue.lock().take()
    }
    fn put(&mut self, queue: Option<KBox<compute::Queue>>) {
        let old = core::mem::replace(&mut *self.0.queue.lock(), queue);
        drop(old);
    }
}

impl Drop for ComputeSlot {
    fn drop(&mut self) {
        let queue = self.0.queue.lock().take();
        drop(queue);
    }
}

pub(super) struct Registry {
    pub(super) qids: kick::QueueIds,
    pub(super) buffers: BufferIds,
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
    /// Completion-ring slot where the next compute scan starts.
    completion_cursor: usize,
    /// Control word of each completion-ring slot as of the last scan that read it.
    completion_seen: KVVec<u64>,
    worker_deferred: Option<DeferredBatch>,
    teardown: super::teardown::Pending,
}

impl Registry {
    /// Initialize inside the firmware allocation instead of staging the whole
    /// registry on the probe stack. No additional allocation is required.
    #[cfg(CONFIG_KUNIT = "y")]
    pub(super) fn new(
        clusters: u32,
        deflake: crate::g17::fw::render::Deflake,
        descriptor_flags: [u32; 2],
        variant: &'static crate::g17::fw::Variant,
    ) -> impl Init<Self, Error> {
        Self::with_waiters(clusters, deflake, descriptor_flags, variant, None, None)
    }

    /// `queues` wakes logical-queue creation when a logical queue slot is returned;
    /// `renders` wakes render publication when a render pair or lease returns.
    pub(super) fn with_waiters(
        clusters: u32,
        deflake: crate::g17::fw::render::Deflake,
        descriptor_flags: [u32; 2],
        variant: &'static crate::g17::fw::Variant,
        queues: Option<Arc<super::availability::Availability>>,
        renders: Option<Arc<super::availability::Availability>>,
    ) -> impl Init<Self, Error> {
        kernel::try_init!(Self {
            qids <- kick::QueueIds::new(variant.qid_floor).chain(|qids| {
                qids.reserve_initial_render()
            }),
            buffers <- BufferIds::new(),
            metrics: Arc::new(MetricsIds::new(), GFP_KERNEL)?,
            qos_ids: QosIds::new()?,
            admission: admission::Pool::with_waiters(kick::queue_limit(variant.qid_floor), queues, renders)?,
            render: render::RenderState::new(clusters, deflake, descriptor_flags)?,
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
            observations: compute::retirement::Batch::new()?,
            completion_cursor: 0,
            completion_seen: {
                let slots = super::initdata::COMPUTE_COMPLETION_SLOTS;
                let mut seen = KVVec::with_capacity(slots, GFP_KERNEL)?;
                for _ in 0..slots {
                    seen.push_within_capacity(0).map_err(|_| ENOMEM)?;
                }
                seen
            },
            worker_deferred: Some(DeferredBatch::worker()?),
            teardown: super::teardown::Pending::new()?,
        })
    }

    pub(super) fn compute(&self, owner: u64) -> Option<QueueGuard<'_>> {
        self.compute
            .iter()
            .flatten()
            .filter_map(|entry| entry.queue.as_deref())
            .find(|queue| queue.owner() == Some(owner))
    }

    pub(super) fn compute_mut(&mut self, owner: u64) -> Option<QueueGuard<'_>> {
        self.compute(owner)
    }

    /// The cell of `owner`'s bound queue, which its publisher may lock without the device mutex.
    pub(super) fn compute_cell(&self, owner: u64) -> Option<Arc<ComputeCell>> {
        self.compute.iter().flatten().find_map(|entry| {
            entry.queue.as_deref()?.owner().filter(|bound| *bound == owner)?;
            Some(entry.queue.0.clone())
        })
    }

    /// An installed idle queue keeps its original QID and backing. Mapping a
    /// replacement context must succeed before its old binding is displaced.
    /// A vacant queue (USC backing given back) is offered for re-arming only
    /// when no populated idle queue exists.
    pub(super) fn rebind_compute(&mut self, owner: u64, context: &Arc<Context>, required: usize, admission: &Admission<'_>) -> Result<Rebind> {
        admission.check()?;
        if let Some(queue) = self.compute(owner) {
            return Ok(if queue.matches(owner, queue.qid(), context)? {
                if queue.pool_fits(required) { Rebind::Done } else { Rebind::Grow }
            } else {
                Rebind::Fresh
            });
        }
        // A binding persists for its logical queue's lifetime while physical
        // QIDs allow: take a channel of a closed owner, then a fresh QID, and
        // only then the least recently idle channel of one open owner.
        if let Some(rebind) = self.bind_reusable(owner, context, required, admission)? {
            return Ok(rebind);
        }
        if self.qids.compute_fits(self.admission.queues() as usize) {
            return Ok(Rebind::Fresh);
        }
        let teardown = &self.teardown;
        let idle = self
            .compute
            .iter()
            .flatten()
            .filter_map(|entry| entry.queue.as_deref())
            .filter(|queue| queue.shareable())
            .filter(|queue| !queue.context().is_some_and(|bound| teardown.releasing(bound)))
            .map(|queue| ((!queue.pool_fits(required), queue.last_published()), queue.qid()))
            .min_by_key(|(key, _)| *key)
            .map(|(_, qid)| qid);
        let Some(qid) = idle else { return Err(ENOSPC); };
        admission.check()?;
        let shared = self
            .compute
            .iter_mut()
            .flatten()
            .filter_map(|entry| entry.queue.as_deref_mut())
            .find(|queue| queue.qid() == qid)
            .ok_or(EIO)?
            .share_idle()?;
        if !shared {
            return Err(ENOSPC);
        }
        Ok(self.bind_reusable(owner, context, required, admission)?.unwrap_or(Rebind::Fresh))
    }

    /// Whether `owner` can bind a channel now, without changing any binding. Owner 0 asks
    /// only whether some channel is free for a new owner.
    fn compute_readiness(&self, owner: u64, context: Option<&Arc<Context>>, required: usize) -> Readiness {
        if let (Some(context), Some(queue)) = (context, self.compute(owner).filter(|_| owner != 0)) {
            return if queue.spared_pending() {
                Readiness::Progress
            } else if !matches!(queue.matches(owner, queue.qid(), context), Ok(true)) {
                // Publication reports the mismatch.
                Readiness::Ready
            } else if queue.pool_fits(required) || !queue.growing() {
                Readiness::Ready
            } else {
                Readiness::Progress
            };
        }
        let teardown = &self.teardown;
        let available = self
            .compute
            .iter()
            .flatten()
            .filter_map(|entry| entry.queue.as_deref())
            .any(|queue| {
                (queue.reusable() || queue.shareable() || queue.can_rearm())
                    && !queue.context().is_some_and(|bound| teardown.releasing(bound))
            });
        if available || self.qids.compute_fits(self.admission.queues() as usize) {
            Readiness::Ready
        } else {
            Readiness::Channel
        }
    }

    /// Binds `owner` to a reusable channel whose last owner released it, or
    /// names a vacant one to re-arm. None if no channel is reusable.
    fn bind_reusable(&mut self, owner: u64, context: &Arc<Context>, required: usize, admission: &Admission<'_>) -> Result<Option<Rebind>> {
        let mut vacant = None;
        let teardown = &self.teardown;
        for mut queue in self
            .compute
            .iter_mut()
            .flatten()
            .filter_map(|entry| entry.queue.as_deref_mut())
            .filter(|queue| queue.reusable())
        {
            admission.check()?;
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
            return Ok(Some(Rebind::Done));
        }
        Ok(vacant.map(Rebind::Rearm))
    }

    /// Installs fresh USC backing into the vacant queue `qid`; the caller writes
    /// the returned descriptor row before binding. EAGAIN if the queue was taken.
    pub(super) fn rearm_compute(
        &mut self,
        qid: u8,
        pages: crate::g17::freelist::Pages,
        ring: compute::DescriptorRing,
    ) -> Result<(u16, u64)> {
        let mut queue = self
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
        let mut queue = self
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
            let mut queue = self
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
        for mut queue in self
            .compute
            .iter_mut()
            .flatten()
            .filter_map(|entry| entry.queue.as_deref_mut())
        {
            if !deferred.vacated_room() { break; }
            if let Some(vacated) = queue.observe_vacancy(consumer, producer)? {
                deferred.defer_vacated(vacated);
                freed += 1;
            }
        }
        Ok(freed)
    }

    /// Builders and released queues waiting for their exact retirement or pool
    /// witness temporarily occupy identities. A failed admission must let those
    /// transactions finish rather than fail a healthy successor immediately.
    fn compute_admission_error(&self, error: Error) -> Error {
        if error == ENOSPC {
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
                    .as_deref()
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
            .ok_or_else(|| self.compute_admission_error(
                crate::g17::capacity::refused(crate::g17::capacity::Site::ComputeSlots)))?;
        let identity = self.identities.reserve(1)?.start;
        let queues = self.admission.queues() as usize;
        let id = self
            .qids
            .reserve_for(identity, DataMaster::Compute, Some(FIRST_COMPUTE_QID), queues)
            .map_err(|error| self.compute_admission_error(error))?;
        self.note_build_room();
        let reservation = ComputeReservation { slot, id, owner };
        self.compute[slot] = Some(ComputeEntry {
            reservation,
            queue: ComputeSlot::new()?,
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
        entry.queue.put(queue.take());
        Ok(())
    }

    pub(super) fn cancel_compute(&mut self, reservation: ComputeReservation) -> Result {
        self.reserved(reservation)?;
        self.qids.cancel_unpublished(reservation.id)?;
        self.compute[reservation.slot] = None;
        self.note_build_room();
        Ok(())
    }

    pub(super) fn release_compute(&mut self, owner: u64) -> Result {
        let Some(slot) = self.compute.iter().position(|entry| {
            entry
                .as_ref()
                .and_then(|entry| entry.queue.as_deref())
                .is_some_and(|queue| queue.owner() == Some(owner))
        }) else {
            return Ok(());
        };
        let entry = self.compute[slot].as_mut().ok_or(EIO)?;
        let unpublished = entry.queue.as_deref_mut().ok_or(EIO)?.release_owner()?;
        if unpublished {
            self.qids.cancel_unpublished(entry.reservation.id)?;
            self.compute[slot] = None;
            self.note_build_room();
        }
        Ok(())
    }
}

/// What a compute packet still lacks before publication.
#[derive(Copy, Clone, PartialEq, Eq)]
pub(super) enum Readiness {
    Ready,
    /// A physical channel: every one is busy and no QID fits.
    Channel,
    /// Retirement-bound capacity or reopened publication.
    Progress,
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


#[kunit_tests(asahi_g17_uninstalled_cleanup)]
mod uninstalled_cleanup_tests {
    use super::*;
    use crate::g17::fw::render::Deflake;
    use crate::g17::fw::variant::V26A428;
    #[test]
    fn empty_reservation_prevents_reuse_until_explicit_cancellation() {
        let mut registry = KBox::init(
            Registry::new(1, Deflake::ONE_CLUSTER, [0; 2], &V26A428),
            GFP_KERNEL,
        )
        .unwrap();
        let first = registry.reserve_compute(1).unwrap();
        assert!(registry.reserved(first).is_ok());
        assert!(registry.reserve_compute(1).is_err());
        let second = registry.reserve_compute(2).unwrap();
        assert_ne!(first.id.qid(), second.id.qid());
        registry.cancel_compute(first).unwrap();
        assert!(registry.reserved(first).is_err());
        let next = registry.reserve_compute(3).unwrap();
        assert_eq!(first.id.qid(), next.id.qid());
        assert!(first.id != next.id);
    }
}

/// An immutable logical owner shared by its two scheduler entities. Installed
/// physical graphs retain the context independently of this frontend lifetime.
#[pin_data]
pub(super) struct Backend {
    shared: Arc<super::Shared>,
    pub(super) context: Arc<Context>,
    owner: u64,
    backlog: core::sync::atomic::AtomicU32,
    compute_backlog: core::sync::atomic::AtomicU32,
    closed: core::sync::atomic::AtomicBool,
    /// The cell of the compute queue this owner last published to under the device mutex,
    /// with the shared compute objects mapped into its VM. Direct publication revalidates the
    /// binding under the cell lock; a stale cell only sends the packet back to that path.
    #[pin]
    direct: kernel::sync::Mutex<Option<Arc<ComputeCell>>>,
    /// The channel a prepared packet is about to publish into. Its claim keeps the idle
    /// channel from being shared away between prepare_job and run_job.
    #[pin]
    claimed: kernel::sync::Mutex<Option<Arc<ComputeCell>>>,
    /// Render pair admission, shared by every logical queue of the session.
    admission: Arc<admission::Pool>,
    /// The render pair this owner last published to without the device mutex
    /// (zero for none): a hint, revalidated by every lend.
    direct_slot: core::sync::atomic::AtomicU32,
}

impl Backend {
    pub(super) fn new(
        shared: &Arc<super::Shared>,
        vm: &crate::mmu::Vm,
        owner: u64,
        policy: super::fw::queue::Policy,
    ) -> Result<(Arc<Self>, admission::QueueSlot)> {
        // A closed queue keeps its application context until its scheduler
        // release is consumed and the context is returned. When only such
        // contexts block this one, wait (bounded) for the teardown instead of
        // failing a live client; the preparation lease is not held while
        // sleeping, and nothing of the new queue is visible yet.
        const RETAINED_CONTEXT_WAIT_NS: u64 = 4 * WORK_RING_WAIT_NS;
        let started = now_ns();
        let (lease, uat, pool, qos, execution, slot) = loop {
            let observed = shared.queues.observe();
            let lease = shared.preparations.enter(true)?;
            let mut state = shared.state.lock();
            let firmware = (*state).as_deref_mut().ok_or(ENODEV)?;
            if shared.crashed.load(core::sync::atomic::Ordering::Acquire) {
                return Err(EIO);
            }
            let slot = firmware.queues.admission.reserve_queue()?;
            // Installed compute channels narrow render's QIDs: every open
            // logical queue keeps a dedicated render pair within reach.
            if !firmware.queues.qids.render_fits(firmware.queues.admission.queues() as usize) {
                return Err(super::capacity::refused(super::capacity::Site::LogicalQueues));
            }
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
                    let closing = firmware.uat.execution_contexts_used().saturating_sub(open);
                    if closing == 0 || now_ns().saturating_sub(started) >= RETAINED_CONTEXT_WAIT_NS {
                        return Err(super::capacity::refused(super::capacity::Site::ExecutionContexts));
                    }
                    drop(slot);
                    drop(state);
                    drop(lease);
                    // Nothing is visible yet: a signalled creator restarts.
                    if kernel::current!().signal_pending() {
                        return Err(ERESTARTSYS);
                    }
                    shared.queue_events();
                    // A closing queue returns its context when its release
                    // is reclaimed; that reclaim wakes this creator.
                    let left = RETAINED_CONTEXT_WAIT_NS.saturating_sub(now_ns().saturating_sub(started));
                    shared.queues.block(observed, left.div_ceil(1_000_000) as u32, true)?;
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
            Context::new(
                &alloc,
                &pool,
                &qos,
                execution,
                owner_pid,
                policy,
                shared.renders.clone(),
                shared.progress.clone(),
            )?,
            GFP_KERNEL,
        )?;
        let state = shared.state.lock();
        if !lease.is_current() || state.is_none() {
            return Err(EAGAIN);
        }
        drop(state);
        let admission = slot.pool().clone();
        Ok((
            Arc::pin_init(
                pin_init!(Self {
                    shared: shared.clone(),
                    context,
                    owner,
                    backlog: core::sync::atomic::AtomicU32::new(0),
                    compute_backlog: core::sync::atomic::AtomicU32::new(0),
                    closed: core::sync::atomic::AtomicBool::new(false),
                    direct <- kernel::new_mutex!(None, "g17::Backend::direct"),
                    claimed <- kernel::new_mutex!(None, "g17::Backend::claimed"),
                    admission,
                    direct_slot: core::sync::atomic::AtomicU32::new(0),
                }),
                GFP_KERNEL,
            )?,
            slot,
        ))
    }

    /// The shared aperture has exactly one builder. Other queues sleep without
    /// holding the device mutex; recovery joins its lease before closing roots.
    fn shared_compute(&self, admission: &Admission<'_>) -> Result {
        loop {
            admission.check()?;
            let lease = self.shared.preparations.enter_until(admission)?;
            let dev = self.shared.drm()?;
            let uat = {
                let mut state = self.shared.state.lock();
                loop {
                    admission.check()?;
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
                    let _ = self.shared.changed.wait_timeout(
                        &mut state, kernel::time::msecs_to_jiffies(admission.slice_ms()?)
                    );
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
                if let Err(error) = admission.check() {
                    Err(error)
                } else if !lease.is_current() {
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
            self.shared.note_changed();
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
        admission: &Admission<'_>,
    ) -> Result {
        {
            let mut state = self.shared.state.lock();
            loop {
                admission.check()?;
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
                let _ = self.shared.changed.wait_timeout(
                    &mut state, kernel::time::msecs_to_jiffies(admission.slice_ms()?)
                );
            }
        }
        let mut built = Some(super::initdata::ComputeScheduler::new(alloc));
        let result = {
            let mut state = self.shared.state.lock();
            let firmware = (*state).as_deref_mut().ok_or(ENODEV)?;
            if let Err(error) = admission.check() {
                Err(error)
            } else if !lease.is_current() {
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
        self.shared.note_changed();
        result
    }

    /// The caller keeps its preparation lease through this two-phase rollback.
    /// An empty registry entry prevents reusing the QID/free-list identity while
    /// its old aliases still occupy canonical or client addresses. Recovery joins
    /// this lease before removing the session, even if its epoch was invalidated.
    fn finish_uninstalled_compute(
        &self, retired: Option<(ComputeReservation, KBox<compute::Queue>)>,
    ) -> Result {
        let Some((reservation, queue)) = retired else { return Ok(()); };
        drop(queue);
        let mut state = self.shared.state.lock();
        let firmware = state.as_deref_mut().ok_or(ENODEV)?;
        firmware.queues.cancel_compute(reservation)?;
        drop(state);
        // The slot and its never-published QID are free again.
        self.shared.channels.notify();
        Ok(())
    }

    fn grow_compute(&self, required: usize, admission: &Admission<'_>) -> Result {
        admission.check()?;
        {
            let state = self.shared.state.lock();
            let firmware = state.as_ref().ok_or(ENODEV)?;
            if firmware.queues.compute(self.owner).is_some_and(|queue| queue.pool_fits(required)) {
                return Ok(());
            }
        }
        let lease = self.shared.preparations.enter_until(admission)?;
        let dev = self.shared.drm()?;
        let uat = self.shared.state.lock().as_ref().ok_or(ENODEV)?.uat.clone();
        let alloc = super::object::Allocator { dev: &dev, uat: &uat };
        let mut pages = Some(super::freelist::Pages::compute_sized(&alloc, required)?);
        let mut alias = Some(pages.as_ref().ok_or(EIO)?.map_client(self.context.vm())?);
        let mut deferred = DeferredBatch::publication()?;
        let mut growth = None;
        let result = (|| loop {
            // Growth waits for this queue's retirement and for firmware to consume the pool
            // release; both are progress the event worker reports.
            let observed = self.shared.progress.observe();
            let result = (|| {
                let mut state = self.shared.state.lock();
                admission.check()?;
                let firmware = (*state).as_deref_mut().ok_or(ENODEV)?;
                if !lease.is_current() { return Err(EAGAIN); }
                if self.shared.crashed.load(Ordering::Acquire) { return Err(EIO); }
                firmware.scan_compute(None, &mut deferred)?;
                let mut queue = firmware.queues.compute_mut(self.owner).ok_or(EAGAIN)?;
                if queue.pool_fits(required) { return Ok(None); }
                if growth.is_none() {
                    let notify = queue.begin_growth(&firmware.init, required)?;
                    // Capture physical identity before any following fallible
                    // operation. Close can remove the owner-keyed lookup.
                    growth = queue.growth_token();
                    if notify {
                        fence(Ordering::SeqCst);
                        firmware.primary.notify(crate::g17::MSG_CONTROL_NOTIFY)?;
                    }
                }
                queue.finish_growth(&firmware.init, &mut pages, &mut alias, required).map(Some)
            })();
            deferred.finish();
            match result {
                Ok(old) => { drop(old); return Ok(()); }
                Err(error) if error == EAGAIN && lease.is_current() => {
                    self.shared.queue_events();
                    self.wait_release(&self.shared.progress, observed, admission)?;
                }
                Err(error) => return Err(error),
            }
        })();
        if result.is_err() {
            if let Some(token) = growth {
                let mut retired = None;
                let mut state = self.shared.state.lock();
                if let Some(firmware) = state.as_deref_mut() {
                    for index in 0..firmware.queues.compute.len() {
                        let Some(entry) = firmware.queues.compute[index].as_mut() else { continue; };
                        let Some(aborted) = entry.queue.as_deref_mut()
                            .map(|mut queue| queue.abort_growth(token)).transpose()? else { continue; };
                        if aborted == Some(true) {
                            // Keep the empty entry and QID reserved until all
                            // graph aliases have been destroyed outside the lock.
                            retired = entry.queue.take().map(|queue| (entry.reservation, queue));
                            break;
                        }
                    }
                }
                drop(state);
                // Tentative replacement mappings also belong to this builder.
                // Discard them before a fresh QID can acquire a successor.
                drop(alias.take());
                drop(pages.take());
                self.finish_uninstalled_compute(retired)?;
                self.shared.queue_events();
            }
        }
        result
    }

    fn build_compute(&self, required: usize, admission: &Admission<'_>) -> Result {
        admission.check()?;
        self.admission_open()?;
        loop {
            match self.shared_compute(admission) {
                Err(error) if error == EAGAIN => continue,
                result => {
                    result?;
                    break;
                }
            }
        }
        loop {
            admission.check()?;
            // Sampled before any capacity check of this attempt.
            let observed = self.shared.channels.observe();
            let lease = self.shared.preparations.enter_until(admission)?;
            let dev = self.shared.drm()?;
            let (uat, objects, reservation) = {
                let mut state = self.shared.state.lock();
                admission.check()?;
                let firmware = (*state).as_deref_mut().ok_or(ENODEV)?;
                if self
                    .shared
                    .crashed
                    .load(core::sync::atomic::Ordering::Acquire)
                {
                    return Err(EIO);
                }
                self.admission_open()?;
                let rebound = match firmware.queues.rebind_compute(self.owner, &self.context, required, admission) {
                    Err(error) if error == ENOSPC => {
                        drop(state);
                        drop(lease);
                        self.wait_channels(observed, admission, true)?;
                        continue;
                    }
                    result => result?,
                };
                match rebound {
                    Rebind::Done => return Ok(()),
                    Rebind::Grow => { drop(state); drop(lease); return self.grow_compute(required, admission); }
                    Rebind::Rearm(qid) => (firmware.uat.clone(), firmware.pool.clone(), Err(qid)),
                    Rebind::Fresh => {
                        let reservation = match firmware.queues.reserve_compute(self.owner) {
                            Ok(reservation) => reservation,
                            Err(error) if error == EAGAIN => {
                                // All physical channels are occupied. Retain their
                                // USC backing for exact-retirement handoff; releasing
                                // it cannot create a QID and forces fresh allocation.
                                drop(state);
                                drop(lease);
                                self.wait_channels(observed, admission, false)?;
                                continue;
                            }
                            Err(error) if error == ENOSPC => {
                                let reclaimed = firmware.queues.reclaim_unused_render_spare(None, true)?;
                                drop(state);
                                if reclaimed.is_none() {
                                    return Err(error);
                                }
                                drop(reclaimed);
                                continue;
                            }
                            Err(error) => return Err(error),
                        };
                        (firmware.uat.clone(), firmware.pool.clone(), Ok(reservation))
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
                    let built = (|| -> Result<_> {
                        let pages = crate::g17::freelist::Pages::compute_sized(&alloc, required)?;
                        Ok((pages, compute::DescriptorRing::new(&alloc, &objects)?))
                    })();
                    let (pages, ring) = match built {
                        Err(error) if error == ENOSPC => {
                            drop(lease);
                            self.wait_channels(observed, admission, true)?;
                            continue;
                        }
                        result => result?,
                    };
                    let mut state = self.shared.state.lock();
                    admission.check()?;
                    let firmware = (*state).as_deref_mut().ok_or(ENODEV)?;
                    if !lease.is_current() {
                        continue;
                    }
                    if let Err(error) = self.admission_open() { drop(state); return Err(error); }
                    // A competing ready owner may have taken/rearmed it while
                    // backing was built off-lock. Leave rejected backing local
                    // so it is dropped after the device guard, not in rearm().
                    let ready = firmware.queues.compute.iter().flatten()
                        .filter_map(|entry| entry.queue.as_deref())
                        .any(|queue| queue.qid() == qid && queue.can_rearm());
                    if !ready { drop(state); continue; }
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
                    admission.check()?;
                    let firmware = (*state).as_deref_mut().ok_or(ENODEV)?;
                    if !lease.is_current() {
                        return Err(EAGAIN);
                    }
                    self.admission_open()?;
                    firmware.queues.reserved(reservation)?;
                    let (slot, page_list) = pool.pool_descriptor()?;
                    firmware.init.initialize_compute_pool_blocks(slot, page_list, required)?;
                }
                let backing = compute::Backing::new(&alloc, pool)?;
                // The global scheduler is created after pool/kick backing and
                // retained even if this queue's later descriptor graph fails.
                self.compute_scheduler(&alloc, &lease, admission)?;
                let queue = compute::Queue::new(
                    &alloc,
                    &objects,
                    backing,
                    self.owner,
                    self.context.clone(),
                    self.shared.variant.pointers,
                    self.shared.operand_state,
                )?;
                Ok(KBox::new(queue, GFP_KERNEL)?)
            })();
            let (mut built, build_error) = match built {
                Ok(queue) => (Some(queue), None),
                Err(error) => (None, Some(error)),
            };
            let result = {
                let mut state = self.shared.state.lock();
                let firmware = (*state).as_deref_mut().ok_or(ENODEV)?;
                if let Err(error) = admission.check() {
                    Err(error)
                } else if !lease.is_current() {
                    Err(EAGAIN)
                } else if let Some(error) = build_error {
                    Err(error)
                } else {
                    admission.check().and_then(|()| firmware.queues.install_compute(reservation, &mut built))
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
            if result == Err(ENOSPC) {
                self.wait_channels(observed, admission, true)?;
                continue;
            }
            if result != Err(EAGAIN) { return result; }
        }
    }
}

/// One device transaction can fail each retained kick once and then retire it.
/// The physical namespace bounds the batch independently of open logical queues.
/// Collection failure leaves the engine's Active owner intact for a stopped retry.
pub(super) struct DeferredBatch {
    has_source_completions: bool,
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
            has_source_completions: false,
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
    pub(super) fn vacated_room(&self) -> bool {
        self.vacated.len() < self.vacated.capacity()
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
            if completion.poisons(*error) {
                completion.status().record(*error);
            }
        }
        match &value {
            Deferred::Retired(completion, Err(error)) | Deferred::Drained(completion, Err(error))
            | Deferred::FailedOwned(completion, error) | Deferred::Unpublished(completion, error) =>
                { completion.note_input_error(*error); },
            Deferred::Retired(completion, Ok(_)) | Deferred::Drained(completion, Ok(_)) =>
                completion.note_source_retired(),
            _ => {},
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
            self.has_source_completions |= completion.sources().is_some();
        }
        Ok(())
    }
    /// Completions collected so far.
    pub(super) fn pending(&self) -> usize {
        self.values.len()
    }
    /// Called only after releasing the device mutex.
    pub(super) fn finish(&mut self) {
        use super::completion::Deferred;
        if self.has_source_completions {
            for _ in 0..self.values.len() {
                let mut changed = false;
                for value in &self.values {
                    if let Deferred::Retired(completion, Ok(_))
                        | Deferred::Drained(completion, Ok(_)) = value
                    {
                        if let Err(error) = completion.check_source_inputs() {
                            changed |= completion.note_input_error(error);
                        }
                    }
                }
                if !changed { break; }
            }
        }
        self.has_source_completions = false;
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
    port: &'a super::port::Port,
    outer: Option<super::port::OuterClaim<'a>>,
    effort: &'a mut super::power::Effort,
    qids: &'a mut kick::QueueIds,
    generation: u64,
}
impl compute::Host for ComputeHost<'_> {
    fn next_compute_publication(&mut self) -> Result<u64> {
        Ok(self.port.next_publication())
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
        self.port.qos_publish(owner, scheduler, policy)
    }
    fn qos_cancel(&mut self, publication: qos::Publication) -> Result {
        self.port.qos_cancel(publication)
    }
    fn publish_qid(&mut self, id: kick::Id) -> Result {
        self.qids.publish(id)
    }
    fn publish_outer(&mut self, _priority: u8, slot: &super::fw::channels::WorkSlot) -> Result {
        self.port.publish_compute(self.outer.take().ok_or(EIO)?, slot).map(|_| ())
    }
    fn notify(&mut self, message: u64) -> Result {
        self.port.notify(message)
    }
    fn note_submission(&mut self) -> Result {
        self.effort.submit();
        super::Firmware::apply_shared_effort(self.init, self.effort, self.port, super::power::FULL_EFFORT)
    }
}

/// Publication into a bound, installed queue through the port alone. The caller checked the
/// recovery state, full effort and the queue's binding; first installation never comes here.
struct DirectHost<'a> {
    port: &'a super::port::Port,
    outer: Option<super::port::OuterClaim<'a>>,
    generation: u64,
}
impl compute::Host for DirectHost<'_> {
    fn next_compute_publication(&mut self) -> Result<u64> {
        Ok(self.port.next_publication())
    }
    fn epoch(&self) -> Result<(u64, u32)> {
        Ok((self.generation, self.port.recovery_state()?))
    }
    fn prepare_compute_shared(&mut self) -> Result {
        Err(EAGAIN)
    }
    fn qos_publish(
        &mut self,
        owner: qos::Owner,
        scheduler: u64,
        policy: super::fw::queue::Policy,
    ) -> Result<qos::Publication> {
        self.port.qos_publish(owner, scheduler, policy)
    }
    fn qos_cancel(&mut self, publication: qos::Publication) -> Result {
        self.port.qos_cancel(publication)
    }
    fn publish_qid(&mut self, _id: kick::Id) -> Result {
        // An installed queue published its identity with its first command.
        Ok(())
    }
    fn publish_outer(&mut self, _priority: u8, slot: &super::fw::channels::WorkSlot) -> Result {
        self.port.publish_compute(self.outer.take().ok_or(EIO)?, slot).map(|_| ())
    }
    fn notify(&mut self, message: u64) -> Result {
        self.port.notify(message)
    }
    fn note_submission(&mut self) -> Result {
        Ok(())
    }
}

impl super::Firmware {
    #[inline]
    fn publish_compute(&mut self, owner: u64, packet: Arc<super::job::Packet>,
        dependencies: &[super::fw::kick::KickDependency], deferred: &mut DeferredBatch) -> Result
    {
        if packet.completion.sources().is_some() {
            self.publish_compute_inputs(owner, packet, dependencies, deferred)
        } else {
            self.publish_compute_raw(owner, packet, dependencies, &[], deferred)
        }
    }

    #[inline(never)]
    fn publish_compute_inputs(&mut self, owner: u64, packet: Arc<super::job::Packet>,
        dependencies: &[super::fw::kick::KickDependency], deferred: &mut DeferredBatch) -> Result
    {
        if self.recovery.pending() || super::recovery::Memory::recovery_state(&self.init)? != 0 {
            return Err(EAGAIN);
        }
        let sources = packet.completion.sources().ok_or(EINVAL)?.clone();
        sources.check()?;
        let mut merged = super::dependency::Frontier::new();
        for dependency in dependencies { merged.insert(0, *dependency)?; }
        let mut holds = KVec::with_capacity(sources.sources.len(), GFP_KERNEL)?;
        let mut native: KVec<super::sync_input::NativeWait> =
            KVec::with_capacity(sources.sources.len(), GFP_KERNEL)?;
        for source in &sources.sources {
            if source.submission.source_status()? { continue; }
            let found = self.queues.compute.iter_mut().flatten()
                .filter_map(|entry| entry.queue.as_deref_mut())
                .find_map(|mut queue| match queue.source_dependency(source, packet.completion.status(),
                    self.recovery.generation()) {
                    Ok(None) => None,
                    result => Some(result),
                }).ok_or(EAGAIN)??;
            let Some((sequence, dependency, stamp_va, event_slot, hold)) = found else {
                return Err(EAGAIN);
            };
            // The completion-wait record orders the consumer behind its
            // source; naming the source in the kick entry as well only adds
            // a second, slower wait and a predecessor record.
            let wait = super::sync_input::NativeWait { sequence, dependency, stamp_va, event_slot };
            if let Some(old) = native.iter_mut().find(|old|
                old.dependency.qid() == dependency.qid()) {
                if sequence > old.sequence { *old = wait; }
            } else {
                native.push(wait, GFP_KERNEL)?;
            }
            holds.push(hold, GFP_KERNEL)?;
        }
        let result = self.publish_compute_raw(owner, packet.clone(), merged.as_slice(),
            native.as_slice(), deferred);
        if packet.is_published() {
            for queue in self.queues.compute.iter().flatten()
                .filter_map(|entry| entry.queue.as_deref())
                .filter(|queue| native.iter().any(|wait| wait.dependency.qid() == queue.qid()))
            {
                for source in &sources.sources { queue.note_input_consumer(source); }
            }
            sources.set_holds(holds);
            super::sync_input::note_published();
            super::sync_input::note_native(native.len());
        }
        result
    }

    fn publish_compute_raw(
        &mut self,
        owner: u64,
        packet: Arc<super::job::Packet>,
        dependencies: &[super::fw::kick::KickDependency],
        native: &[super::sync_input::NativeWait],
        deferred: &mut DeferredBatch,
    ) -> Result {
        if self.recovery.pending() {
            return Err(EAGAIN);
        }
        // No descriptor, accounting or inner producer changes precede this
        // reservation; dropping it unused returns the work-ring slot.
        let outer = self.port.reserve_compute(packet.context.policy().priority())?;
        let queues = &mut self.queues;
        let mut queue = queues
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
            port: &self.port,
            outer: Some(outer),
            effort: &mut self.effort,
            qids: &mut queues.qids,
            generation: self.recovery.generation(),
        };
        if native.is_empty() {
            queue.publish(packet, dependencies, &mut host, &mut |event| deferred.push(event))
        } else {
            queue.publish_with_native_waits(packet, dependencies, native,
                &mut host, &mut |event| deferred.push(event))
        }
    }
}

impl Registry {
    /// Everything ensure_compute establishes for `owner`: the shared compute
    /// objects mapped into its VM, and its queue bound to `context` with a
    /// pool that fits and no spared witness to release.
    fn compute_bound(&self, owner: u64, context: &Arc<Context>, required: usize) -> Result<bool> {
        let Some(queue) = self.compute(owner) else {
            return Ok(false);
        };
        Ok(!queue.spared_pending()
            && queue.pool_fits(required)
            && queue.matches(owner, queue.qid(), context)?)
    }

    fn release_compute_witness(
        &mut self,
        init: &super::initdata::InitData,
        port: &super::port::Port,
        owner: u64,
        deferred: &mut DeferredBatch,
    ) -> Result<bool> {
        let Some(mut queue) = self
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
        queue.release_witnessed(
            super::recovery::Memory::recovery_state(init)?,
            |owner| port.qos_complete(&[owner]),
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
        for mut queue in self
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
        // Every queue's pending identities are staged, not only the ones this batch
        // retires: the scan passes each new record once, and a queue retired by a later
        // batch finds its observation already latched.
        for queue in registry
            .compute
            .iter()
            .flatten()
            .filter_map(|entry| entry.queue.as_deref())
        {
            requested |= queue.request_observations(&mut registry.observations)?;
        }
        // Completion records are appended in order: fold only the records written since
        // the previous scan. A request whose stamp already covers it has its record in the
        // ring; if the scan ended before finding one, read the whole ring once.
        if requested {
            let observations = &mut registry.observations;
            let seen = &mut registry.completion_seen;
            registry.completion_cursor = self.init.fold_compute_completions(
                registry.completion_cursor,
                seen,
                |record| observations.fold(record),
            )?;
            if observations.covered_pending() {
                registry.completion_cursor = self
                    .init
                    .resync_compute_completions(seen, |record| observations.fold_any(record))?;
            }
        }
        let port = &self.port;
        let retired = deferred.pending();
        let mut freed = false;
        for mut queue in registry
            .compute
            .iter_mut()
            .flatten()
            .filter_map(|entry| entry.queue.as_deref_mut())
        {
            let busy = !queue.channel_free();
            queue.latch_observations(&mut registry.observations)?;
            queue.retire(
                masks,
                generation,
                state,
                |owner| port.qos_complete(&[owner]),
                &mut |event| deferred.push(event),
            )?;
            freed |= busy && queue.channel_free();
        }
        // All compute observations in this batch are latched before a VM
        // failure can settle its render peers. Exact successes win first.
        for index in 0..self.queues.compute.len() {
            let terminal = self.queues.compute[index]
                .as_mut()
                .and_then(|entry| entry.queue.as_deref_mut())
                .and_then(|mut queue| queue.take_terminal());
            if let Some((status, error)) = terminal {
                status.record(error);
                self.settle_render_vm(&status, error, deferred)?;
                self.queues.fail_compute_vm(&status, error, deferred)?;
            }
        }
        let port = &self.port;
        let registry = &mut self.queues;
        for mut queue in registry
            .compute
            .iter_mut()
            .flatten()
            .filter_map(|entry| entry.queue.as_deref_mut())
        {
            let busy = !queue.channel_free();
            queue.release_witnessed(
                state,
                |owner| port.qos_complete(&[owner]),
                &mut |event| deferred.push(event),
            )?;
            freed |= busy && queue.channel_free();
        }
        // Wake only the admission waits this batch can end. The fences gate
        // scheduler entities alone; their callbacks never take the device mutex.
        let shared = &self.primary.state.shared;
        if freed {
            shared.channels.notify();
        }
        if deferred.pending() != retired {
            shared.progress.notify();
        }
        Ok(())
    }
}

impl Backend {
    fn ensure_compute(&self, scs_pages: u16, admission: &Admission<'_>) -> Result {
        admission.check()?;
        let required = super::fw::compute_pool::required_blocks(self.shared.active_cores, scs_pages)?;
        let mut deferred = DeferredBatch::publication()?;
        loop {
            admission.check()?;
            let observed = self.shared.progress.observe();
            let pending = (|| {
                let mut state = self.shared.state.lock();
                let firmware = (*state).as_deref_mut().ok_or(ENODEV)?;
                firmware.replay_compute(&mut deferred)?;
                firmware
                    .queues
                    .release_compute_witness(&firmware.init, &firmware.port, self.owner, &mut deferred)
            })();
            deferred.finish();
            let pending = pending?;
            if !pending {
                match self.build_compute(required, admission) {
                    Err(error) if error == EAGAIN => continue,
                    result => return result,
                }
            }
            // Other owners continue submitting while this owner waits for its
            // queue-local drain witness. No new command has been accepted: the
            // spared queue and its retained work stay intact past the deadline.
            self.shared.queue_events();
            self.wait_release(&self.shared.progress, observed, admission)?;
        }
    }

    /// Steady-state publication into this owner's bound channel without the device mutex: only
    /// the channel's own lock and the port's small locks are taken, and nothing device-wide is
    /// scanned. None leaves the packet untouched for the device-mutex path, which owns binding,
    /// first installation, input sources, render prefixes, replay and recovery.
    fn compute_direct(
        &self,
        packet: &Arc<super::job::Packet>,
        required: usize,
        deferred: &mut DeferredBatch,
    ) -> Option<Result> {
        if packet.completion.sources().is_some()
            || self.context.render_seen()
            || packet.cancel_requested()
            || packet.completion.status().get() != 0
            || self.admission_open().is_err()
        {
            return None;
        }
        let cell = self.direct.lock().clone()?;
        let lease = self.shared.preparations.try_enter().ok()?;
        let port = self.shared.port(&lease)?;
        if self.shared.crashed.load(Ordering::Acquire) || port.recovery_state().ok()? != 0 {
            return None;
        }
        // Full effort is applied by the device-mutex path when it is not already.
        if !port.note_submission() {
            return None;
        }
        match packet.completion.work_state().ok()?.acquire(crate::g17::job::Engine::Compute) {
            Ok(_) => {}
            Err(error) if error == EBUSY => return None,
            Err(error) => return Some(Err(error)),
        }
        let mut queue = cell.lock()?;
        if !queue.installed()
            || queue.spared_pending()
            || !queue.pool_fits(required)
            || queue.pending_switch_from().is_some()
            || !matches!(queue.matches(self.owner, queue.qid(), &self.context), Ok(true))
            || !lease.is_current()
        {
            return None;
        }
        let outer = port.reserve_compute(packet.context.policy().priority()).ok()?;
        let mut host = DirectHost {
            port,
            outer: Some(outer),
            generation: port.generation(),
        };
        match queue.publish(packet.clone(), &[], &mut host, &mut |event| deferred.push(event)) {
            Err(error) if (error == EAGAIN || error == EBUSY) && !packet.is_published() => None,
            result => Some(result),
        }
    }

    /// Scheduler admission of a compute packet. Shared capacity the packet lacks (a channel, a
    /// work state, retirement-bound room, reopened publication) becomes a fence that the release
    /// providing it signals, so publication finds it ready instead of sleeping for it. Errors
    /// and anything prepare cannot judge are left to publication.
    pub(super) fn prepare_compute(&self, packet: &Arc<super::job::Packet>) -> Option<kernel::dma_fence::Fence> {
        if packet.early_result().is_some()
            || packet.cancel_requested()
            || packet.completion.status().get() != 0
            || self.admission_open().is_err()
        {
            return None;
        }
        let super::command::Validated::Compute { scratch, .. } = &packet.command else { return None; };
        let required = super::fw::compute_pool::required_blocks(self.shared.active_cores, scratch.pages()).ok()?;
        loop {
            let channels = self.shared.channels.observe();
            let progress = self.shared.progress.observe();
            let (domain, observed) = match self.compute_readiness(packet, required) {
                Readiness::Ready => return None,
                Readiness::Channel => (&self.shared.channels, channels),
                Readiness::Progress => (&self.shared.progress, progress),
            };
            self.shared.queue_events();
            match domain.wait(observed) {
                Ok(Some(fence)) => return Some(fence),
                Ok(None) => continue,
                Err(_) => return None,
            }
        }
    }

    fn compute_readiness(&self, packet: &Arc<super::job::Packet>, required: usize) -> Readiness {
        match packet.completion.work_state().and_then(|work| work.acquire(crate::g17::job::Engine::Compute)) {
            Err(error) if error == EBUSY => return Readiness::Progress,
            Err(_) => return Readiness::Ready,
            Ok(_) => {}
        }
        // The steady state: this owner's channel is bound and installed.
        if packet.completion.sources().is_none() {
            if let Some(cell) = self.direct.lock().clone() {
                let bound = cell.lock().is_some_and(|mut queue| {
                    let bound = queue.installed()
                        && !queue.spared_pending()
                        && queue.pool_fits(required)
                        && matches!(queue.matches(self.owner, queue.qid(), &self.context), Ok(true));
                    if bound {
                        queue.claim();
                    }
                    bound
                });
                if bound {
                    self.hold_claim(Some(cell));
                    return Readiness::Ready;
                }
            }
        }
        match self.sources_published(packet) {
            Ok(false) => return Readiness::Progress,
            Err(_) => return Readiness::Ready,
            Ok(true) => {}
        }
        let lease = match self.shared.preparations.try_enter() {
            Ok(lease) => lease,
            Err(error) if error == EAGAIN => return Readiness::Progress,
            Err(_) => return Readiness::Ready,
        };
        let state = self.shared.state.lock();
        let Some(firmware) = state.as_deref() else { return Readiness::Ready; };
        if !lease.is_current()
            || firmware.recovery.pending()
            || super::recovery::Memory::recovery_state(&firmware.init).unwrap_or(0) != 0
        {
            return Readiness::Progress;
        }
        firmware.queues.compute_readiness(self.owner, Some(&self.context), required)
    }

    /// Replaces the prepared packet's channel claim; None releases it after publication,
    /// cancellation or close.
    pub(super) fn hold_claim(&self, cell: Option<Arc<ComputeCell>>) {
        let old = core::mem::replace(&mut *self.claimed.lock(), cell);
        let freed = old.as_ref().and_then(|old| old.lock()).is_some_and(|mut queue| {
            let busy = !queue.channel_free();
            queue.release_claim();
            busy && queue.channel_free()
        });
        if freed {
            // A channel nobody prepares into any more is shareable now.
            self.shared.channels.notify();
        }
    }

    /// Called with the device mutex held after a successful publication: remembers the bound
    /// channel for direct publication.
    fn remember_direct(&self, registry: &Registry) {
        let cell = registry.compute_cell(self.owner);
        let mut direct = self.direct.lock();
        let current = match (&*direct, &cell) {
            (Some(old), Some(new)) => Arc::ptr_eq(old, new),
            (None, None) => true,
            _ => false,
        };
        if !current {
            let old = core::mem::replace(&mut *direct, cell);
            drop(direct);
            drop(old);
        }
    }

    /// Publishes a dependency-free compute command from its submit ioctl when
    /// its queue is already bound and nothing has to wait, as renders do. Any
    /// wait leaves the command, unpublished, to its scheduler.
    pub(super) fn try_compute_early(&self, packet: &Arc<super::job::Packet>) -> bool {
        if self.compute_backlog.load(Ordering::Acquire) != 0 || packet.has_dependencies() {
            return false;
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
        let result = match self.compute_direct_packet(packet, &mut deferred) {
            Some(result) => result,
            None => self.compute_inline(packet, &mut deferred),
        };
        if matches!(result, Err(error) if error == render::FALLBACK) && !packet.is_published() {
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

    fn compute_direct_packet(
        &self,
        packet: &Arc<super::job::Packet>,
        deferred: &mut DeferredBatch,
    ) -> Option<Result> {
        let super::command::Validated::Compute { scratch, .. } = &packet.command else {
            return Some(Err(EINVAL));
        };
        let required = match super::fw::compute_pool::required_blocks(self.shared.active_cores, scratch.pages()) {
            Ok(required) => required,
            Err(error) => return Some(Err(error)),
        };
        self.compute_direct(packet, required, deferred)
    }

    /// compute_packet's publication section without its waits: every condition
    /// it would sleep or retry for returns FALLBACK instead.
    fn compute_inline(
        &self,
        packet: &Arc<super::job::Packet>,
        deferred: &mut DeferredBatch,
    ) -> Result {
        let super::command::Validated::Compute { scratch, .. } = &packet.command else { return Err(EINVAL); };
        let required = super::fw::compute_pool::required_blocks(self.shared.active_cores, scratch.pages())?;
        if packet.cancel_requested() || self.admission_open().is_err() || packet.completion.status().get() != 0 {
            return Err(render::FALLBACK);
        }
        let lease = self.enter_preparation(false)?;
        match packet.completion.work_state()?.acquire(crate::g17::job::Engine::Compute) {
            Err(error) if error == EBUSY => return Err(render::FALLBACK),
            result => {
                result?;
            }
        }
        // A lone admission keeps the original latency-sensitive probe.
        // Competing publishers otherwise spin on a binding they do not own.
        if !self.context.compute_binding_hint()
            && self.shared.compute_demand.load(Ordering::Relaxed) > 1
        {
            return Err(render::FALLBACK);
        }
        let mut state = self.shared.state.lock();
        let firmware = (*state).as_deref_mut().ok_or(ENODEV)?;
        if !lease.is_current()
            || self.shared.crashed.load(Ordering::Acquire)
            || firmware.recovery.pending()
            || super::recovery::Memory::recovery_state(&firmware.init)? != 0
        {
            return Err(render::FALLBACK);
        }
        firmware.replay_compute(deferred)?;
        if !firmware.init.compute_vm_cached(self.context.vm())
            || !firmware.queues.compute_bound(self.owner, &self.context, required)?
        {
            return Err(render::FALLBACK);
        }
        let dependencies = firmware.queues.render_prefix(self.owner, packet)?;
        match firmware.publish_compute(self.owner, packet.clone(), dependencies.as_slice(), deferred) {
            Err(error) if (error == EBUSY || error == EAGAIN) && !packet.is_published() => {
                Err(render::FALLBACK)
            }
            result => {
                if result.is_ok() {
                    self.remember_direct(&firmware.queues);
                }
                result
            }
        }
    }

    /// A scheduled dependency may refer to a previous recovery epoch. Do not
    /// acquire consumer capacity until every live source is published now.
    fn sources_published(&self, packet: &super::job::Packet) -> Result<bool> {
        let Some(sources) = packet.completion.sources() else { return Ok(true); };
        sources.check()?;
        let state = self.shared.state.lock();
        let firmware = state.as_deref().ok_or(ENODEV)?;
        if firmware.recovery.pending()
            || super::recovery::Memory::recovery_state(&firmware.init)? != 0
        { return Ok(false); }
        for source in &sources.sources {
            if source.submission.source_status()? { continue; }
            if !firmware.queues.compute.iter().flatten()
                .filter_map(|entry| entry.queue.as_deref())
                .any(|queue| queue.has_input_source(&source.submission,
                    packet.completion.status(), firmware.recovery.generation()))
            { return Ok(false); }
        }
        Ok(true)
    }

    /// Called after taking this packet's publication claim. All waits and work
    /// state allocation precede the device mutex; only the checked epoch commits.
    fn compute_packet(
        &self,
        packet: &Arc<super::job::Packet>,
        deferred: &mut DeferredBatch,
    ) -> Result {
        packet.check_dependencies()?;
        let super::command::Validated::Compute { scratch, .. } = &packet.command else { return Err(EINVAL); };
        let required = super::fw::compute_pool::required_blocks(self.shared.active_cores, scratch.pages())?;
        if let Some(result) = self.compute_direct(packet, required, deferred) {
            return result;
        }
        // A bound queue needs none of ensure_compute's admission passes. Try
        // without them; the publication section below checks the same state.
        // Every retry blocks on the release that ends its wait: retirement
        // progress (work states, ring room, replays) and the reopening of
        // publication after recovery both signal `progress`.
        let mut bound = true;
        let mut demand = None;
        let deadline = now_ns().saturating_add(WORK_RING_WAIT_NS);
        let check = || {
            if packet.cancel_requested() { return Err(ECANCELED); }
            self.admission_open()?;
            if packet.completion.status().get() != 0 { return Err(EIO); }
            Ok(())
        };
        let admission = Admission::new(deadline, &check);
        let progress = &self.shared.progress;
        loop {
            admission.check()?;
            let observed = progress.observe();
            if !self.sources_published(packet)? {
                self.shared.queue_events();
                self.wait_release(progress, observed, &admission)?;
                continue;
            }
            // All DRM dependencies were satisfied before this scheduler callback.
            // Recheck on every retry: pressure may yield an idle binding between
            // admission and publication without changing this logical queue.
            if !bound {
                demand.get_or_insert_with(|| ComputeDemand::new(&self.shared));
                Backend::ensure_compute(self, scratch.pages(), &admission)?;
                admission.check()?;
            }
            let lease = self.shared.preparations.enter_until(&admission)?;
            // Work states free as this context's earlier commands retire.
            match packet.completion.work_state()?.acquire(crate::g17::job::Engine::Compute) {
                Err(error) if error == EBUSY => {
                    drop(lease);
                    self.shared.queue_events();
                    self.wait_release(progress, observed, &admission)?;
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
            admission.check()?;
            if self
                .shared
                .crashed
                .load(core::sync::atomic::Ordering::Acquire)
            {
                return Err(EIO);
            }
            let recovery_state = super::recovery::Memory::recovery_state(&firmware.init)?;
            if firmware.recovery.pending() || recovery_state != 0 {
                drop(state);
                drop(lease);
                self.shared.queue_events();
                self.wait_release(progress, observed, &admission)?;
                continue;
            }
            firmware.replay_compute(deferred)?;
            if !firmware.init.compute_vm_cached(self.context.vm())
                || !firmware.queues.compute_bound(self.owner, &self.context, required)?
            {
                bound = false;
                drop(state);
                drop(lease);
                continue;
            }
            let dependencies = firmware.queues.render_prefix(self.owner, packet)?;
            admission.check()?;
            let result = firmware.publish_compute(
                self.owner,
                packet.clone(),
                dependencies.as_slice(),
                deferred,
            );
            if !packet.is_published()
                && (matches!(result, Err(EAGAIN))
                    || (matches!(result, Err(EBUSY))
                        && (firmware.recovery.pending()
                            || super::recovery::Memory::recovery_state(&firmware.init)? != 0)))
            {
                // Recovery or backpressure: nothing is firmware-visible. Other
                // queues keep submitting and the event worker retires meanwhile.
                drop(state);
                drop(lease);
                self.shared.queue_events();
                self.wait_release(progress, observed, &admission)?;
                continue;
            }
            if result.is_ok() {
                self.remember_direct(&firmware.queues);
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
        for mut queue in self
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
    /// Charge only real watchdog expirations, after the per-job allowance. A
    /// changing client or recovery generation cannot mint a fresh storm budget.
    pub(super) fn watchdog_timeout(self: &Arc<Self>) -> bool {
        if self.crashed.load(Ordering::Acquire) { return true; }
        let stop = {
            let mut state = self.state.lock();
            state.as_deref_mut().is_none_or(|firmware| firmware.recovery.timeout_limit(now_ns()))
        };
        if stop { self.lose_device_quietly(); }
        stop
    }

    /// Workload-triggered stop: halt producers first, then join only for a
    /// bounded interval. A stalled borrower leaves stopped firmware and all its
    /// mappings owned in state. Only a later joined stop may detach/free them.
    pub(super) fn stop_runtime(&self) {
        let Some(_stopping) = self.stop.try_lock() else { return; };
        {
            let mut state = self.state.lock();
            if let Some(firmware) = state.as_deref_mut() {
                if let Some(secondary) = firmware.secondary.as_mut() {
                    secondary.stop();
                }
                firmware.primary.stop();
            }
        }
        if self.preparations.wait_drained_until(now_ns().saturating_add(WORK_RING_WAIT_NS)).is_err() {
            dev_warn!(self.dev.as_ref(), "Stopped GPU retains outstanding preparation owners\n");
            return;
        }
        self.forget_port();
        let firmware = self.state.lock().take();
        if let Some(mut firmware) = firmware { firmware.finish_stopped_queues(); }
    }

    /// Joins construction and complete host teardown, including RTKit drops.
    /// The stop-only mutex outlives detached firmware destruction, so unbind
    /// cannot return while another stop caller still owns mailbox clients.
    /// Completion callbacks remain outside the device mutex. Direct handlers
    /// do not synchronously request or join stop; this path does not join
    /// scheduler workers.
    pub(super) fn stop_queues(&self) {
        self.preparations.wait_drained();
        let _stopping = self.stop.lock();
        self.forget_port();
        let firmware = {
            let mut state = self.state.lock();
            if let Some(firmware) = (*state).as_deref_mut() {
                if let Some(secondary) = firmware.secondary.as_mut() {
                    secondary.stop();
                }
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
        let mut freed = false;
        let mut releasing = false;
        for mut queue in self
            .queues
            .compute
            .iter_mut()
            .flatten()
            .filter_map(|entry| entry.queue.as_deref_mut())
        {
            let busy = !queue.channel_free();
            queue.recycle_killed()?;
            queue.recycle_quarantined_idle()?;
            if let Some(binding) = queue.detach_exited_owner() {
                deferred.defer_binding(binding);
            }
            freed |= busy && queue.channel_free();
            releasing |= queue.pool_release_pending();
        }
        freed |= self.queues.observe_pool_releases(&self.init, deferred)? != 0;
        let shared = &self.primary.state.shared;
        // Edge notifications cover the transitions this driver makes; every
        // event pass also rechecks the level, so capacity that appeared through
        // another engine (render QIDs, closed logical queues, render-held work
        // states) still ends a scheduler wait that has no timeout of its own.
        if freed
            || (shared.channels.has_waiters()
                && self.queues.compute_readiness(0, None, 0) == Readiness::Ready)
        {
            shared.channels.notify();
        }
        // A pool release has no completion event: each observation pass is
        // the progress a growing owner waits for.
        if releasing || shared.progress.has_waiters() {
            shared.progress.notify();
        }
        if self.queues.compute.iter().flatten().filter_map(|entry| entry.queue.as_deref())
            .any(|queue| queue.pool_release_stalled(now)) {
            return Err(ETIMEDOUT);
        }
        if self.render_control_backpressured() {
            return Ok(None);
        }
        // While the shared lower window refuses free-list backing, no idle
        // retained compute pool stays warm: render and new compute need it more.
        let cap = if super::freelist::lower_pressure() {
            0
        } else if self.primary.state.shared.compute_demand.load(Ordering::Acquire) != 0 {
            // A retired channel being requested by another runnable publisher
            // is not an unused warm pool. Keep its existing backing through
            // handoff; real allocation pressure above still forces reclamation.
            super::kick::COMPUTE_CHANNELS
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


impl Registry {
    pub(super) fn write_capacity(&self, f: &mut kernel::fmt::Formatter<'_>) -> kernel::fmt::Result {
        self.qids.write_capacity(f)?;
        self.admission.write_capacity(f)?;
        self.buffers.write_capacity(f)?;
        writeln!(f, "qos_ids used={} total=126", self.qos_ids.used())?;
        writeln!(f, "metrics_ids used={} total={}", self.metrics.used(), super::fw::buffer::METRICS_SLOTS)?;
        self.render.write_capacity(f)?;
        let now = now_ns();
        let waits = || self.compute.iter().flatten().filter_map(|entry| entry.queue.as_deref())
            .filter_map(|queue| queue.kill_waiting(now));
        compute_exit::write_kill_waits(f, waits().count(), waits().max().unwrap_or(0))?;
        compute_exit::write_render_kills(f, self.teardown.kill_waits(now).count(),
                                         self.teardown.kill_waits(now).max().unwrap_or(0))?;
        writeln!(f, "compute_slots used={} total={} reusable={} building={}",
                 self.compute.iter().flatten().count(), self.compute.len(),
                 self.compute.iter().flatten().filter_map(|entry| entry.queue.as_deref()).filter(|queue| queue.reusable()).count(),
                 self.compute.iter().flatten().filter(|entry| entry.queue.is_none()).count())?;
        writeln!(f, "compute_channels used={} fresh_fits={}",
                 self.compute.iter().flatten().count(),
                 u8::from(self.qids.compute_fits(self.admission.queues() as usize)))?;
        writeln!(f, "scheduler_teardown pending={}", self.teardown.len())?;
        for queue in self.compute.iter().flatten().filter_map(|entry| entry.queue.as_deref()) {
            queue.write_capacity(f)?;
        }
        Ok(())
    }
}

impl Backend {
    /// Compute admission without a channel: give back what pressure allows, then block until a
    /// channel is released after `observed` (sampled before the failed check).
    fn wait_channels(&self, observed: u64, admission: &Admission<'_>, reclaim_backing: bool) -> Result {
        self.reclaim_capacity(admission, reclaim_backing)?;
        self.wait_release(&self.shared.channels, observed, admission)
    }

    /// Blocks until `domain` is signalled after `observed`, this packet's admission ends, or one
    /// admission slice passes: cancellation and logical close have no release to wait for.
    fn wait_release(
        &self,
        domain: &super::availability::Availability,
        observed: u64,
        admission: &Admission<'_>,
    ) -> Result {
        use kernel::dma_fence::RawDmaFence;
        let slice = admission.slice_ms()?;
        if let Some(fence) = domain.wait(observed)? {
            // SAFETY: The owned fence keeps the raw DMA fence alive throughout the wait.
            let waited = unsafe {
                kernel::bindings::dma_fence_wait_timeout(
                    fence.raw(),
                    false,
                    kernel::time::msecs_to_jiffies(slice) as isize,
                )
            };
            if waited < 0 {
                return Err(Error::from_errno(waited as i32));
            }
        }
        admission.check()
    }

    /// Reclaim steps of a capacity wait that never sleep: observe consumed pool releases and,
    /// under backing pressure, give idle pools back.
    fn reclaim_capacity(&self, admission: &Admission<'_>, reclaim_backing: bool) -> Result {
        admission.check()?;
        let mut deferred = DeferredBatch::publication()?;
        let result: Result = (|| {
            // A short reclaim transaction obeys the same session/epoch gate as
            // publication. Never keep this lease while sleeping for capacity.
            let lease = self.shared.preparations.try_enter()?;
            let mut state = self.shared.state.lock();
            let firmware = state.as_deref_mut().ok_or(ENODEV)?;
            if !lease.is_current() || self.shared.crashed.load(Ordering::Acquire) {
                return Err(EAGAIN);
            }
            if firmware.recovery.pending() || super::recovery::Memory::recovery_state(&firmware.init)? != 0 {
                return Ok(());
            }
            // Consumption, not publication or an elapsed timeout, frees backing.
            firmware.queues.observe_pool_releases(&firmware.init, &mut deferred)?;
            // Pool eviction addresses backing allocation pressure, not channel
            // occupancy. Normal idle expiry and the retained-pool cap still run
            // in the event worker for either kind of admission wait.
            if reclaim_backing {
                firmware.queues.reclaim_idle_render_pools(&firmware.init, &mut deferred)?;
            }
            if reclaim_backing && !firmware.render_control_backpressured() {
                let (published, _) = firmware.queues.vacate_idle_pools(&firmware.init, 0, now_ns(), 0)?;
                if published != 0 { firmware.primary.notify(super::MSG_CONTROL_NOTIFY)?; }
            }
            Ok(())
        })();
        deferred.finish();
        self.shared.queue_events();
        match result {
            Err(error) if error == EAGAIN => {}
            result => result?,
        }
        admission.check()
    }
}

#[kunit_tests(asahi_g17_shared_capacity)]
mod shared_capacity_tests {
    use super::*;
    use crate::g17::fw::render::Deflake;
    use crate::g17::fw::variant::V26A428;

    #[test]
    fn all_logical_render_owners_fit_beside_shared_compute() {
        let mut registry = KBox::init(
            Registry::new(1, Deflake::ONE_CLUSTER, [0; 2], &V26A428),
            GFP_KERNEL,
        )
        .unwrap();
        let mut leases = KVec::with_capacity(crate::mmu::MAX_EXECUTION_CONTEXTS as usize, GFP_KERNEL).unwrap();
        let mut slots = KVec::with_capacity(crate::mmu::MAX_EXECUTION_CONTEXTS as usize, GFP_KERNEL).unwrap();
        for _ in 0..crate::mmu::MAX_EXECUTION_CONTEXTS {
            slots.push(registry.admission.reserve_queue().unwrap(), GFP_KERNEL).unwrap();
        }
        // With every execution context open, compute keeps only the channels
        // that leave each logical queue a dedicated render pair.
        for owner in 1..=super::super::kick::COMPUTE_FLOOR as u64 {
            registry.reserve_compute(owner).unwrap();
        }
        assert!(matches!(registry.reserve_compute(1000), Err(EAGAIN)));
        assert!(registry.qids.render_fits(slots.len()));
        for owner in 1..=u64::from(crate::mmu::MAX_EXECUTION_CONTEXTS) {
            let lease = registry.admission.acquire_render(owner, None, u64::MAX, || Ok(())).unwrap();
            registry.ensure_render_ids(lease.slot(), owner).unwrap();
            leases.push(lease, GFP_KERNEL).unwrap();
        }
        assert_eq!(leases.len(), crate::mmu::MAX_EXECUTION_CONTEXTS as usize);
        // Every accepted logical render owner now has its physical pair; no
        // published identity was cancelled or retyped to achieve the fit.
    }

    #[test]
    fn a_qid_floor_admits_only_the_render_pairs_it_leaves() {
        use crate::g17::fw::variant::V25G83;
        let limit = super::super::kick::queue_limit(V25G83.qid_floor);
        assert_eq!(limit, 42);
        let mut registry = KBox::init(
            Registry::new(2, Deflake::TWO_CLUSTERS, [0; 2], &V25G83),
            GFP_KERNEL,
        )
        .unwrap();
        let mut slots = KVec::with_capacity(limit as usize, GFP_KERNEL).unwrap();
        let mut leases = KVec::with_capacity(limit as usize, GFP_KERNEL).unwrap();
        for owner in 1..=u64::from(limit) {
            slots.push(registry.admission.reserve_queue().unwrap(), GFP_KERNEL).unwrap();
            let lease = registry.admission.acquire_render(owner, None, u64::MAX, || Ok(())).unwrap();
            registry.ensure_render_ids(lease.slot(), owner).unwrap();
            leases.push(lease, GFP_KERNEL).unwrap();
        }
        // The next logical queue is refused when it is created, not left waiting for a pair.
        assert!(matches!(registry.admission.reserve_queue(), Err(ENOSPC)));
    }

    #[test]
    fn four_clients_have_capacity_before_any_owner_closes() {
        let mut registry = KBox::init(
            Registry::new(1, Deflake::ONE_CLUSTER, [0; 2], &V26A428),
            GFP_KERNEL,
        )
        .unwrap();
        let mut leases = KVec::with_capacity(36, GFP_KERNEL).unwrap();
        for client in 0..4u64 {
            for queue in 0..9u64 {
                let owner = 1 + client * 9 + queue;
                let lease = registry.admission.acquire_render(owner, None, u64::MAX, || Ok(())).unwrap();
                registry.ensure_render_ids(lease.slot(), owner).unwrap();
                leases.push(lease, GFP_KERNEL).unwrap();
            }
        }
        for owner in 1..=super::super::kick::COMPUTE_FLOOR as u64 {
            registry.reserve_compute(owner).unwrap();
        }
        assert_eq!(leases.len(), 36);
    }

    #[test]
    fn compute_grows_with_free_qids_and_narrows_queue_admission() {
        let mut registry = KBox::init(Registry::new(1, Deflake::ONE_CLUSTER, [0; 2], &V26A428), GFP_KERNEL).unwrap();
        let mut slots = KVec::with_capacity(20, GFP_KERNEL).unwrap();
        for _ in 0..20 {
            slots.push(registry.admission.reserve_queue().unwrap(), GFP_KERNEL).unwrap();
        }
        // Sixteen open compute clients each keep their own channel.
        for owner in 1..=16u64 {
            registry.reserve_compute(owner).unwrap();
        }
        // Growth stops where open queues plus headroom would lose their pairs.
        let mut owner = 17;
        while registry.reserve_compute(owner).is_ok() {
            owner += 1;
        }
        assert!(registry.qids.render_fits(slots.len() + 24));
        assert!(!registry.qids.render_fits(crate::mmu::MAX_EXECUTION_CONTEXTS as usize));
    }
}

impl Backend {
    /// Logical close is independent of whether an off-lock builder has already
    /// installed a physical graph. No constructor can resurrect a closed owner.
    fn admission_open(&self) -> Result {
        if self.closed.load(Ordering::Acquire) { return Err(ECANCELED); }
        if self.context.status().get() != 0 || self.shared.crashed.load(Ordering::Acquire) {
            return Err(EIO);
        }
        if !self.context.is_current() { return Err(EFAULT); }
        Ok(())
    }
}
