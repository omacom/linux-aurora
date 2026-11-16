// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Device ownership of retained render pairs and their shared free-list leases.
//! The device mutex protects identities and publication. A preparation gate
//! lease permits graph construction outside that mutex without session teardown.

mod exit;
mod dependencies;
mod direct;
mod grow;
mod observe;
mod pools;
mod prepare;
mod recovery;
mod replay;
mod settle;
mod submit;
pub(super) mod teardown;

pub(super) use submit::FALLBACK;
use super::{DeferredBatch, Registry};
use crate::g17::{context::Context, fw::queue::DataMaster, job::Packet, kick, queue::render::Pair};
use crate::hw::t8140::queues::RENDER_SLOTS;
use kernel::{prelude::*, sync::Arc};

#[derive(Copy, Clone)]
struct Config {
    clusters: u32,
    deflake: crate::g17::fw::render::Deflake,
    descriptor_flags: [u32; 2],
}

#[derive(Copy, Clone)]
struct Reservation {
    slot: u8,
    ids: [kick::Id; 2],
    owner: u64,
    buffer: (u8, u64),
}

struct Entry {
    reservation: Reservation,
    pair: Option<KBox<Pair>>,
    parked: Option<crate::g17::queue::render::parked::ParkedPair>,
    preparing: Option<Arc<Packet>>,
    context: Option<Arc<Context>>,
    pool: Option<Arc<crate::g17::freelist::RenderPool>>,
    pending_owner: Option<u64>,
    pending_owner_closed: bool,
    borrowed_active: bool,
    borrowed_dependencies: Option<crate::g17::queue::render::Dependencies>,
    /// `Pair::switch_in_flight` of a borrowed pair. A borrowed pair publishes
    /// nothing, so the snapshot can only be stale towards an older switch.
    borrowed_switch: Option<u64>,
    event_masks: [u64; 2],
    /// A completion event for this pair arrived while it was borrowed. Event
    /// passes skip borrowed pairs, so the returning borrower requests another.
    missed_event: bool,
    closed: bool,
    pending_error: Option<(Arc<crate::g17::status::VmStatus>, Error)>,
    pending_timeout: bool,
    constructing: bool,
    growing: bool,
    published: bool,
}

/// Unpublished objects removed together, then destroyed outside the device lock.
pub(super) struct UnusedRenderSpare {
    _entry: Option<Entry>,
    _binding: Option<crate::g17::queue::render::RetiredBinding>,
}

pub(super) struct RenderState {
    config: Config,
    entries: KVec<Option<Entry>>,
    pools: pools::Pools,
    grow: grow::Grow,
    retired: KVec<crate::g17::queue::render::RetiredBinding>,
    /// Graphs of released contexts the firmware may still walk (see `detach_context`): kept
    /// until the firmware stops, when this state is dropped.
    kept_graphs: KVec<crate::g17::queue::render::RetiredBinding>,
    next_pm_generation: u64,
}

impl RenderState {
    pub(super) fn new(
        clusters: u32,
        deflake: crate::g17::fw::render::Deflake,
        descriptor_flags: [u32; 2],
    ) -> Result<Self> {
        if clusters == 0 {
            return Err(EINVAL);
        }
        let mut entries = KVec::with_capacity(RENDER_SLOTS, GFP_KERNEL)?;
        for _ in 0..RENDER_SLOTS {
            entries.push(None, GFP_KERNEL)?;
        }
        Ok(Self {
            config: Config {
                clusters,
                deflake,
                descriptor_flags,
            },
            entries,
            pools: pools::Pools::new()?,
            grow: grow::Grow::new()?,
            retired: KVec::with_capacity(128, GFP_KERNEL)?,
            kept_graphs: KVec::new(),
            next_pm_generation: 1,
        })
    }

    fn reserve_pm_generation(&mut self) -> Result<u64> {
        let generation = self.next_pm_generation;
        self.next_pm_generation = generation.checked_add(1).ok_or(EOVERFLOW)?;
        Ok(generation)
    }

    fn collect_retired(&mut self, slot: usize) -> Result {
        loop {
            if self.retired.len() == self.retired.capacity() {
                return Ok(());
            }
            let retired = self.entries[slot]
                .as_mut()
                .and_then(|entry| entry.pair.as_mut())
                .and_then(|pair| pair.take_retired_binding());
            let Some(retired) = retired else {
                return Ok(());
            };
            self.retired
                .push_within_capacity(retired)
                .map_err(|_| EIO)?;
        }
    }

    fn entry(&self, slot: u8) -> Result<&Entry> {
        self.entries
            .get(usize::from(slot))
            .and_then(Option::as_ref)
            .ok_or(EIO)
    }

    fn entry_mut(&mut self, slot: u8) -> Result<&mut Entry> {
        self.entries
            .get_mut(usize::from(slot))
            .and_then(Option::as_mut)
            .ok_or(EIO)
    }

    /// Whether a completion event arrived while the pair in `slot` was
    /// borrowed; clears the record. The returned pair is installed again, so
    /// the next event pass observes it.
    fn take_missed_event(&mut self, slot: u8) -> bool {
        self.entry_mut(slot)
            .is_ok_and(|entry| core::mem::take(&mut entry.missed_event))
    }

    fn reserved(&mut self, reservation: Reservation) -> Result<&mut Entry> {
        let entry = self.entry_mut(reservation.slot)?;
        if entry.reservation.ids != reservation.ids
            || entry.reservation.owner != reservation.owner
            || entry.reservation.buffer != reservation.buffer
            || entry.pair.is_some()
            || entry.preparing.is_some()
            || entry.context.is_some()
            || !entry.constructing
        {
            return Err(EIO);
        }
        Ok(entry)
    }

    /// Keep metadata while graph preparation owns the pair outside the mutex.
    /// Completion scans skip it; cancellation is recorded and applied on return.
    fn take_pair(&mut self, slot: u8, owner: u64, packet: &Arc<Packet>) -> Result<KBox<Pair>> {
        let entry = self.entry_mut(slot)?;
        if entry.closed
            || entry.reservation.owner != owner
            || entry.preparing.is_some()
            || entry.pending_error.is_some()
            || entry.pending_timeout
        {
            return Err(EBUSY);
        }
        let pair = entry.pair.as_ref().ok_or(EIO)?;
        if !pair.matches(owner, &packet.context)? {
            return Err(EIO);
        }
        entry.borrowed_active = pair.published_in_flight();
        entry.borrowed_dependencies = Some(pair.dependency_snapshot());
        entry.borrowed_switch = pair.switch_in_flight();
        entry.preparing = Some(packet.clone());
        entry.pair.take().ok_or(EIO)
    }

    /// Validate before taking the owner from the caller. Even rejected returns
    /// keep their backing alive until the caller resolves the reservation.
    fn return_pair(&mut self, slot: u8, pair: &mut Option<KBox<Pair>>) -> Result {
        let returning = pair.as_ref().ok_or(EINVAL)?;
        let entry = self.entry_mut(slot)?;
        if entry.pair.is_some()
            || entry.preparing.is_none()
            || returning.slot() != slot
            || returning.qids() != entry.reservation.ids.map(kick::Id::qid)
            || returning.owner() != Some(entry.reservation.owner)
        {
            return Err(EIO);
        }
        entry.pair = pair.take();
        entry.borrowed_dependencies = None;
        entry.borrowed_switch = None;
        entry.preparing = None;
        Ok(())
    }

    fn save_detaching_pool(&self, init: &crate::g17::initdata::InitData, slot: u8) -> Result {
        let pool = self.entry(slot)?.pool.as_ref().ok_or(EIO)?;
        let shared = self.entries.iter().enumerate().any(|(index, entry)| {
            index != usize::from(slot)
                && entry.as_ref().is_some_and(|entry| {
                    entry.pool.as_ref().is_some_and(|other| {
                        other.id() == pool.id() && other.control_va() == pool.control_va()
                    })
                })
        });
        if !shared {
            self.pools.save(init, pool)?;
        }
        Ok(())
    }
}

impl Registry {
    /// Yield unused warming only when admission has run out of physical slots.
    /// Previously published graphs keep their existing ownership and retirement
    /// rules. The caller destroys returned backing after dropping the device lock.
    pub(super) fn reclaim_unused_render_spare(
        &mut self,
        owner: Option<u64>,
        require_ids: bool,
    ) -> Result<Option<UnusedRenderSpare>> {
        // Prefer an empty reservation or an already parked channel.
        for with_backing in [false, true] {
            for slot in 1..RENDER_SLOTS {
                let mut warm = false;
                let expected = if let Some(entry) = self.render.entries[slot].as_ref() {
                    if entry.constructing
                        || entry.preparing.is_some()
                        || entry.growing
                        || entry.pending_error.is_some()
                        || entry.pending_timeout
                    {
                        continue;
                    }
                    // Compute can gain identities only from physical channels
                    // that never entered command publication. Render can reuse
                    // a retained render identity without changing its lifetime.
                    if owner.is_none()
                        && !entry
                            .reservation
                            .ids
                            .iter()
                            .all(|id| self.qids.can_cancel_unpublished(*id))
                    {
                        continue;
                    }
                    if let Some(pair) = entry.pair.as_ref() {
                        if !with_backing {
                            continue;
                        }
                        if owner.is_some() && pair.reusable()? {
                            None
                        } else if pair.can_yield_unused()? {
                            warm = true;
                            Some(entry.reservation.owner)
                        } else {
                            continue;
                        }
                    } else {
                        if with_backing || entry.context.is_some() {
                            continue;
                        }
                        None
                    }
                } else {
                    if with_backing || require_ids || owner.is_none() {
                        continue;
                    }
                    None
                };
                if !self
                    .admission
                    .reassign_unused_spare(slot as u8, expected, owner)
                {
                    continue;
                }
                if owner.is_none() {
                    let entry = self.render.entries[slot].take();
                    if let Some(entry) = entry.as_ref() {
                        for id in entry.reservation.ids {
                            self.qids.cancel_unpublished(id)?;
                        }
                        if let Some(pair) = entry.pair.as_ref() {
                            self.buffers.unreserve_owner(pair.buffer_id())?;
                        }
                        self.note_build_room();
                    }
                    return Ok(Some(UnusedRenderSpare {
                        _entry: entry,
                        _binding: None,
                    }));
                }
                let mut binding = None;
                if warm {
                    let entry = self.render.entries[slot].as_mut().ok_or(EIO)?;
                    let mut pair = entry.pair.take().ok_or(EIO)?;
                    self.buffers.unreserve_owner(pair.buffer_id())?;
                    pair.release_owner();
                    let (parked, retired) = KBox::into_inner(pair).park();
                    entry.parked = Some(parked);
                    entry.context = None;
                    entry.pool = None;
                    binding = Some(retired);
                }
                return Ok(Some(UnusedRenderSpare {
                    _entry: None,
                    _binding: binding,
                }));
            }
        }
        Ok(None)
    }

    /// The installed pair in `slot` belongs to `owner` and its context, is
    /// neither borrowed nor settling a failure, and its pool has backing: the
    /// state in which building the pair would do nothing.
    pub(super) fn render_ready(&self, slot: u8, owner: u64, context: &Arc<Context>) -> Result<bool> {
        let Ok(entry) = self.render.entry(slot) else {
            return Ok(false);
        };
        if entry.pending_error.is_some()
            || entry.pending_timeout
            || entry.constructing
            || entry.preparing.is_some()
            || entry.growing
            || entry.parked.is_some()
        {
            return Ok(false);
        }
        let Some(pair) = entry.pair.as_ref() else {
            return Ok(false);
        };
        Ok(pair.matches(owner, context)? && !pair.pool().vacant())
    }

    fn has_render_ids(&self, owner: u64) -> bool {
        self.render
            .entries
            .iter()
            .enumerate()
            .any(|(slot, entry)| entry.is_some() && self.admission.owns_slot(owner, slot))
    }

    /// Tell admission whether another render pair can still be built.
    pub(super) fn note_build_room(&self) {
        self.admission.note_build_room(self.qids.render_pair_fits());
    }

    pub(super) fn ensure_render_ids(&mut self, slot: u8, owner: u64) -> Result {
        if owner == 0 || slot == 0 || usize::from(slot) >= RENDER_SLOTS {
            return Err(EINVAL);
        }
        if self.render.entries[usize::from(slot)].is_some() {
            return Ok(());
        }
        let built = self.build_render_ids(slot, owner);
        self.note_build_room();
        built
    }

    fn build_render_ids(&mut self, slot: u8, owner: u64) -> Result {
        {
            let identity = self.identities.reserve(2)?.start;
            let tiling = self.qids.reserve(identity, DataMaster::Tiling, None)?;
            let fragment = match self.qids.reserve(identity + 1, DataMaster::Fragment, None) {
                Ok(id) => id,
                Err(error) => {
                    self.qids.cancel_unpublished(tiling)?;
                    return Err(error);
                }
            };
            self.render.entries[usize::from(slot)] = Some(Entry {
                reservation: Reservation {
                    slot,
                    ids: [tiling, fragment],
                    owner,
                    buffer: (0, 0),
                },
                pair: None,
                parked: None,
                preparing: None,
                context: None,
                pool: None,
                pending_owner: None,
                pending_owner_closed: false,
                borrowed_active: false,
                borrowed_dependencies: None,
                borrowed_switch: None,
                event_masks: [0; 2],
                missed_event: false,
                closed: false,
                pending_error: None,
                pending_timeout: false,
                constructing: false,
                growing: false,
                published: false,
            });
        }
        Ok(())
    }

    /// Only an optional sibling's unpublished QID-capacity refusal is soft.
    /// ensure_render_ids cancels a first ID if the second cannot be reserved;
    /// validation, identity overflow, and cancellation errors still propagate.
    fn prepare_render_ids(&mut self, slot: u8, owner: u64, optional: bool) -> Result<bool> {
        // A full queue table refuses a sibling without scanning it on every preparation.
        if optional
            && self.render.entries.get(usize::from(slot)).is_some_and(Option::is_none)
            && !self.qids.render_pair_fits()
        {
            return Ok(false);
        }
        match self.ensure_render_ids(slot, owner) {
            Ok(()) => Ok(true),
            Err(error) if optional && error == ENOSPC => Ok(false),
            Err(error) => Err(error),
        }
    }

    fn reserve_render(&mut self, slot: u8, owner: u64) -> Result<Reservation> {
        self.ensure_render_ids(slot, owner)?;
        let entry = self.render.entry(slot)?;
        if entry.constructing
            || entry.pair.is_some()
            || entry.parked.is_some()
            || entry.preparing.is_some()
            || entry.growing
        {
            return Err(EBUSY);
        }
        // Construction keeps identities reserved. Publication pins them at the
        // first firmware-visible command, so unused warming can still yield.
        let buffer = self.buffers.allocate_render()?;
        let entry = self.render.entry_mut(slot)?;
        entry.reservation.owner = owner;
        entry.reservation.buffer = buffer;
        entry.constructing = true;
        entry.closed = false;
        Ok(entry.reservation)
    }

    fn install_render(
        &mut self,
        reservation: Reservation,
        pair: &mut Option<KBox<Pair>>,
    ) -> Result {
        let built = pair.as_ref().ok_or(EINVAL)?;
        if built.slot() != reservation.slot
            || built.qids() != reservation.ids.map(kick::Id::qid)
            || built.owner() != Some(reservation.owner)
            || built.buffer_id() != reservation.buffer.0
        {
            return Err(EIO);
        }
        if self.render.reserved(reservation)?.closed {
            return Err(ECANCELED);
        }
        // Construction holds one PB reference; retain the owner independently
        // and return that reference before the first command acquires its own.
        self.buffers.reserve_owner(reservation.buffer.0)?;
        self.buffers.release(reservation.buffer.0)?;
        let entry = self.render.reserved(reservation)?;
        entry.context = Some(built.context().clone());
        entry.pool = Some(built.pool().clone());
        entry.pair = pair.take();
        entry.constructing = false;
        Ok(())
    }

    /// Drop the rejected graph outside the device mutex before returning its
    /// temporary PB reference. The physical queue identities remain reserved.
    fn cancel_render_reservation(&mut self, reservation: Reservation) -> Result {
        self.render.reserved(reservation)?;
        self.buffers.release(reservation.buffer.0)?;
        self.render.entry_mut(reservation.slot)?.constructing = false;
        Ok(())
    }

    pub(super) fn fail_render_vm(
        &mut self,
        status: &Arc<crate::g17::status::VmStatus>,
        error: Error,
        deferred: &mut DeferredBatch,
    ) -> Result {
        for entry in self.render.entries.iter_mut().flatten() {
            if !entry
                .context
                .as_ref()
                .is_some_and(|context| Arc::ptr_eq(context.status(), status))
            {
                continue;
            }
            if let Some(pair) = entry.pair.as_mut() {
                pair.quarantine(error, &mut |event| deferred.push(event))?;
            } else if entry.preparing.is_some() || entry.growing {
                entry.pending_error = Some((status.clone(), error));
            }
        }
        Ok(())
    }
}

struct Host<'a> {
    firmware: &'a mut crate::g17::Firmware,
    registration_powered: bool,
}
impl Host<'_> {
    fn release_registration(&mut self) {
        if !self.registration_powered && !self.firmware.registration_power_release_pending {
            return;
        }
        if let Err(error) = self.firmware.release_registration_power() {
            dev_warn!(
                self.firmware.primary.state.shared.dev.as_ref(),
                "Could not restore render idle policy: {:?}\n",
                error
            );
        } else {
            self.registration_powered = false;
        }
    }
}
impl crate::g17::queue::render::Host for Host<'_> {
    fn epoch(&self) -> Result<(u64, u32)> {
        Ok((
            self.firmware.recovery.generation(),
            crate::g17::recovery::Memory::recovery_state(&self.firmware.init)?,
        ))
    }
    fn stamps(&self, qids: [u8; 2]) -> Result<[u32; 2]> {
        use core::sync::atomic::Ordering;
        let stamps = [
            self.firmware.init.stamp(qids[0])?.load(Ordering::Relaxed),
            self.firmware.init.stamp(qids[1])?.load(Ordering::Relaxed),
        ];
        core::sync::atomic::fence(Ordering::Acquire);
        Ok(stamps)
    }
    fn qos_publish_pair(
        &mut self,
        owners: [crate::g17::qos::Owner; 2],
        scheduler: u64,
        policy: crate::g17::fw::queue::Policy,
    ) -> Result<[crate::g17::qos::Publication; 2]> {
        self.firmware.port.qos_publish_pair(owners, scheduler, policy)
    }
    fn qos_cancel(&mut self, publication: crate::g17::qos::Publication) -> Result {
        self.firmware.port.qos_cancel(publication)
    }
    /// Idle descent is held for the whole first install. A point-in-time power
    /// check let the firmware gate the cores between the host's configuration
    /// writes while several clients installed their first pairs at once.
    fn install_pair(&mut self, registration: &crate::g17::fw::kick::RenderRegistration) -> Result {
        // Firmware builds without MMIO registration install the pair from its queue
        // configuration alone.
        if !self.firmware.variant.mmio_pair_registration {
            return Ok(());
        }
        if !self.registration_powered {
            self.firmware.acquire_registration_power()?;
            self.registration_powered = true;
        }
        for _ in 0..3 {
            if self
                .firmware
                .bridge
                .regs
                .install_render_pair(registration)?
            {
                return Ok(());
            }
        }
        Err(EAGAIN)
    }
    fn publish_qid(&mut self, id: kick::Id) -> Result {
        self.firmware.queues.qids.publish(id)
    }
    fn next_publication(&mut self) -> Result<u64> {
        Ok(self.firmware.port.next_publication())
    }
    fn publish_outer_pair(
        &mut self,
        priority: u8,
        fragment: &crate::g17::fw::channels::WorkSlot,
        tiling: &crate::g17::fw::channels::WorkSlot,
        commit: impl FnOnce() -> Result,
        late_tiling: impl FnOnce(),
    ) -> Result<[u32; 2]> {
        self.firmware
            .init
            .publish_work_pair(priority, fragment, tiling, true, commit, late_tiling)
    }
    fn activate_render_pool(&mut self, id: u16, generation: u64) -> Result {
        self.release_registration();
        let result = self.firmware.queues.render.pools.activate(id, generation);
        if result.is_err() {
            self.firmware.primary.state.shared.lose_device_quietly();
        }
        result
    }
    fn publish_retained_pair(
        &mut self,
        priority: u8,
        fragment: &crate::g17::fw::channels::WorkSlot,
        tiling: &crate::g17::fw::channels::WorkSlot,
        next: [u32; 2],
    ) -> Result {
        let result = self
            .firmware
            .init
            .publish_retained_work_pair(priority, fragment, tiling, next);
        if result.is_err() {
            self.firmware.primary.state.shared.lose_device_quietly();
        }
        result
    }
    /// Pipe 0 (tiling) in bits 1:0, priority class in bits 3:2.
    fn notify_render(&mut self, priority: u8) -> Result {
        self.release_registration();
        crate::g17::trace::doorbell(crate::g17::trace::RENDER, || {
            self.firmware.bridge.regs.g17_core_power()
        });
        let result = self
            .firmware
            .primary
            .notify((0x83 << crate::g17::MSG_TYPE_SHIFT) | u64::from(priority) << 2);
        if result.is_err() {
            self.firmware.primary.state.shared.lose_device_quietly();
        }
        result
    }
    fn note_submission(&mut self) -> Result {
        self.firmware.note_submission()
    }
    fn complete_render(
        &mut self,
        owners: Option<[crate::g17::qos::Owner; 2]>,
        pool_id: u16,
        generation: u64,
        buffer_id: u8,
    ) -> Result {
        let queues = &mut self.firmware.queues;
        // Each final release is a control record the firmware processes before
        // it returns to the work rings. While other pairs keep it busy, a
        // registration left idle lingers instead, and the owner's next render
        // reuses it. On an otherwise idle device the release is cheaper than
        // reusing a registration that went idle, so it is sent at once.
        // The pair retiring here is lent out of its entry meanwhile.
        let linger = queues.render.entries.iter().flatten().any(|entry| {
            entry.pair.as_ref().is_some_and(|pair| pair.published_in_flight())
        });
        let next = queues.render.pools.completed(pool_id, generation, linger)?;
        queues.buffers.can_release(buffer_id)?;
        if let Some(owners) = owners {
            self.firmware.port.qos_complete(&owners)?;
        }
        queues.render.pools.commit_completed(pool_id, next);
        queues.buffers.release(buffer_id)
    }
}

impl crate::g17::Firmware {
    /// The pair is returned even when transport, accounting or collection fails.
    /// No off-lock operation or callback which can re-enter the device runs here.
    fn with_render<R>(
        &mut self,
        slot: usize,
        operation: impl FnOnce(&mut Pair, &mut Host<'_>) -> Result<R>,
    ) -> Result<R> {
        let mut pair = self
            .queues
            .render
            .entries
            .get_mut(slot)
            .and_then(Option::as_mut)
            .and_then(|entry| entry.pair.take())
            .ok_or(EBUSY)?;
        let mut host = Host {
            firmware: self,
            registration_powered: false,
        };
        let result = operation(&mut pair, &mut host);
        host.release_registration();
        self.queues.render.entries[slot].as_mut().ok_or(EIO)?.pair = Some(pair);
        result
    }

    /// Event-worker turn: ends free-list lingers that elapsed or whose owner
    /// closed or moved on, publishes their releases, and returns the delay to
    /// the next expiry, which the worker turns into a timed wake.
    pub(in crate::g17) fn expire_render_lingers(&mut self, now: u64) -> Result<Option<u64>> {
        let render = &mut self.queues.render;
        let entries = &render.entries;
        let (ended, next) = render.pools.expire_lingers(now, |owner| {
            entries.iter().flatten().any(|entry| {
                !entry.closed
                    && entry
                        .pool
                        .as_ref()
                        .is_some_and(|pool| pools::Pools::owner(pool) == *owner)
            })
        });
        if ended {
            match self.release_render_lists() {
                Err(error) if error == EAGAIN => {}
                result => result?,
            }
        }
        Ok(next)
    }

    fn release_render_lists(&mut self) -> Result {
        if self.queues.render.grow.reply.is_some() {
            return Err(EAGAIN);
        }
        let primary = &mut self.primary;
        self.queues.render.pools.publish_releases(&self.init, || {
            primary.notify(crate::g17::MSG_CONTROL_NOTIFY)
        })
    }

    /// Events have already been consumed by the caller. Each healthy completion
    /// services its final pool release before examining the next retained job.
    fn retire_render_slot(
        &mut self,
        slot: usize,
        masks: Option<[u64; 2]>,
        deferred: &mut DeferredBatch,
    ) -> Result {
        let generation = self.recovery.generation();
        let state = crate::g17::recovery::Memory::recovery_state(&self.init)?;
        loop {
            let progressed = self.with_render(slot, |pair, host| {
                pair.retire(masks, generation, state, host, &mut |event| {
                    deferred.push(event)
                })
            })?;
            if !progressed {
                break;
            }
            match self.release_render_lists() {
                Err(error) if error == EAGAIN => break,
                result => result?,
            }
        }
        Ok(())
    }

    /// A final retirement pass before a render watchdog attributes its timeout, as compute runs
    /// scan_compute() first: a head that finished but whose completion the event worker has not
    /// processed yet must not defer another VM's timeout. Pairs lent out, or holding a deferred
    /// timeout or error, stay with the worker, which observes and settles them in its order.
    pub(in crate::g17::runtime) fn scan_renders(&mut self, deferred: &mut DeferredBatch) -> Result {
        if self.recovery.pending()
            || self
                .primary
                .state
                .shared
                .crashed
                .load(core::sync::atomic::Ordering::Acquire)
        {
            return Ok(());
        }
        for slot in 1..self.queues.render.entries.len() {
            if !self.queues.render.entries[slot]
                .as_ref()
                .is_some_and(|entry| {
                    entry
                        .pair
                        .as_ref()
                        .is_some_and(|pair| pair.published_in_flight())
                        && entry.preparing.is_none()
                        && !entry.growing
                        && !entry.pending_timeout
                        && entry.pending_error.is_none()
                })
            {
                continue;
            }
            self.retire_render_slot(slot, None, deferred)?;
            self.queues.render.collect_retired(slot)?;
            self.settle_render_terminal(slot, deferred)?;
        }
        Ok(())
    }

    pub(in crate::g17) fn settle_render_vm(
        &mut self,
        status: &Arc<crate::g17::status::VmStatus>,
        error: Error,
        deferred: &mut DeferredBatch,
    ) -> Result {
        for slot in 1..self.queues.render.entries.len() {
            if self.queues.render.entries[slot]
                .as_ref()
                .is_some_and(|entry| {
                    entry.pair.is_some()
                        && entry
                            .context
                            .as_ref()
                            .is_some_and(|context| Arc::ptr_eq(context.status(), status))
                })
            {
                self.retire_render_slot(slot, None, deferred)?;
            }
        }
        self.queues.fail_render_vm(status, error, deferred)
    }

    pub(in crate::g17) fn stop_render(&mut self) {
        for entry in self.queues.render.entries.iter_mut().flatten() {
            if let Some(pair) = entry.pair.as_mut() {
                if let Err(error) = pair.stopped(&mut |event| {
                    event.finish();
                    Ok(())
                }) {
                    dev_err!(
                        self.primary.state.shared.dev.as_ref(),
                        "Could not settle stopped render pair: {:?}\n",
                        error
                    );
                }
            }
        }
    }
}

impl Registry {
    pub(in crate::g17) fn render_oldest_spared(&self, qid: u8) -> Option<bool> {
        self.render
            .entries
            .iter()
            .flatten()
            .filter_map(|entry| entry.pair.as_ref())
            .find(|pair| pair.qids().contains(&qid))
            .and_then(|pair| pair.oldest_spared())
    }
    /// Marks the interrupted group of the pair that owns `qid`.
    pub(in crate::g17) fn note_render_interrupted(
        &self,
        qid: u8,
        stamp: u32,
    ) -> crate::g17::recovery::Interrupted {
        self.render
            .entries
            .iter()
            .flatten()
            .filter_map(|entry| entry.pair.as_ref())
            .find(|pair| pair.qids().contains(&qid))
            .map_or(crate::g17::recovery::Interrupted::None, |pair| {
                pair.note_interrupted(stamp)
            })
    }
    pub(in crate::g17) fn visit_render_owners(
        &self,
        qid: u8,
        visit: &mut dyn FnMut(&Arc<crate::g17::status::VmStatus>),
    ) {
        for pair in self
            .render
            .entries
            .iter()
            .flatten()
            .filter_map(|entry| entry.pair.as_ref())
        {
            if pair.qids().contains(&qid) {
                pair.visit_owners(visit);
            }
        }
    }
    pub(in crate::g17) fn classify_render(
        &self,
        classify: &dyn Fn(&Arc<crate::g17::status::VmStatus>) -> crate::g17::recovery::Class,
    ) -> u128 {
        let mut spared = 0;
        for pair in self
            .render
            .entries
            .iter()
            .flatten()
            .filter_map(|entry| entry.pair.as_ref())
        {
            pair.classify(classify);
            if pair.oldest_spared() == Some(true) {
                for qid in pair.qids() {
                    spared |= 1u128 << qid;
                }
            }
        }
        spared
    }

    pub(in crate::g17) fn render_scheduler_active(&self) -> bool {
        self.render.entries.iter().any(Option::is_some)
            || !self.render.retired.is_empty()
            || !self.teardown.is_empty()
    }
    /// Applies the native tiler-heap shrink policy (`buffer::Manager::
    /// shrink_target`) to every grown heap at `now` and rebuilds the heaps of
    /// idle pairs that are due, removing whole growth extensions. The removed
    /// aliases and backing go to the worker's deferred batch, which frees them
    /// after the device mutex. A shrink is best effort: any failure leaves the
    /// heap as it was. Returns whether a heap still holds growth it could give
    /// back, so the worker keeps a periodic wake.
    pub(in crate::g17) fn shrink_render_heaps(
        &mut self,
        init: &crate::g17::initdata::InitData,
        now: u64,
        deferred: &mut super::DeferredBatch,
    ) -> bool {
        let mut pending = false;
        for (slot, entry) in self.render.entries.iter_mut().enumerate() {
            let Some(entry) = entry.as_mut() else {
                continue;
            };
            let borrowed = entry.constructing || entry.growing || entry.preparing.is_some();
            let Some(pair) = entry.pair.as_mut() else {
                continue;
            };
            if !pair.manager().grown() {
                continue;
            }
            let busy = borrowed || !pair.idle_for_context(pair.context()).unwrap_or(false);
            if let Ok(Some(target)) = pair.manager().shrink_target(now, busy) {
                if deferred.tvb_room() {
                    if let Ok(slot) = u8::try_from(slot) {
                        // Ignoring the result keeps the heap unchanged on failure.
                        let _ = Self::shrink_pair(init, slot, pair, target, deferred);
                    }
                }
            }
            pending |= pair.manager().shrinkable();
        }
        pending
    }

    fn shrink_pair(
        init: &crate::g17::initdata::InitData,
        slot: u8,
        pair: &mut Pair,
        target: usize,
        deferred: &mut super::DeferredBatch,
    ) -> Result {
        use core::sync::atomic::{fence, Ordering};
        let offset = usize::from(pair.buffer_id()) * 16;
        let table = init.pb_descriptor_table()?;
        let words = [
            table.word(offset)?,
            table.word(offset + 4)?,
            table.word(offset + 8)?,
            table.word(offset + 12)?,
        ];
        fence(Ordering::Acquire);
        let descriptor = words.each_ref().map(|word| word.load(Ordering::Relaxed));
        let vm = pair.context().vm().clone();
        let pool = pair.pool_id();
        let shrunk = pair
            .manager()
            .shrink(target, descriptor, |blocks, kept, out| {
                vm.truncate_render_pool_mappings(slot, pool, blocks, kept, out)
            })?;
        if let Some((reloaded, release)) = shrunk {
            for (word, value) in words.into_iter().zip(reloaded) {
                word.store(value, Ordering::Relaxed);
            }
            fence(Ordering::SeqCst);
            deferred.defer_tvb(release);
        }
        Ok(())
    }

    /// Gives back the USC backing of render pools whose client VM pressure
    /// reclaim evicted (an idle VM) and whose final free-list release was
    /// consumed. The vacated backings go to the worker's deferred batch, which
    /// frees them after the device mutex; a later job of that client re-arms
    /// the pool first.
    pub(in crate::g17) fn vacate_evicted_render_pools(
        &mut self,
        init: &crate::g17::initdata::InitData,
        deferred: &mut super::DeferredBatch,
    ) -> Result {
        let render = &mut self.render;
        render.pools.observe(init)?;
        for entry in render.entries.iter().flatten() {
            if !deferred.render_backing_room() {
                break;
            }
            if entry.constructing || entry.growing || entry.preparing.is_some() {
                continue;
            }
            let Some(pool) = entry.pool.as_ref() else {
                continue;
            };
            if pool.vacant() || !render.pools.idle(pool.id()) || !pool.client_evicted() {
                continue;
            }
            if let Some(backing) = pool.vacate() {
                deferred.defer_render_backing(backing);
            }
        }
        Ok(())
    }
    pub(in crate::g17) fn render_in_flight(&self) -> bool {
        self.render.entries.iter().flatten().any(|entry| {
            entry.pair.as_ref().map_or(
                (entry.preparing.is_some() || entry.growing) && entry.borrowed_active,
                |pair| pair.published_in_flight(),
            )
        })
    }
    pub(in crate::g17) fn retain_render_completions(&mut self, masks: [u64; 2]) {
        for entry in self.render.entries.iter_mut().flatten() {
            let mut hit = false;
            for id in entry.reservation.ids {
                let qid = usize::from(id.qid());
                let bit = masks[qid / 64] & (1u64 << (qid % 64));
                entry.event_masks[qid / 64] |= bit;
                hit |= bit != 0;
            }
            // Event passes skip a pair while preparation, growth or an
            // observation has borrowed it. Without a record, a completion
            // that arrived meanwhile waits for an unrelated later event or the
            // one-millisecond rescan.
            if hit && (entry.preparing.is_some() || entry.growing) {
                entry.missed_event = true;
            }
        }
    }
    pub(in crate::g17) fn render_needs_rescan(&self) -> bool {
        self.render.entries.iter().flatten().any(|entry| {
            let active = entry
                .pair
                .as_ref()
                .map_or(
                    (entry.preparing.is_some() || entry.growing) && entry.borrowed_active,
                    |pair| pair.published_in_flight(),
                );
            active
                && crate::g17::queue::render::completion_noted(
                    entry.reservation.ids.map(kick::Id::qid),
                    entry.event_masks,
                )
        })
    }
    pub(in crate::g17) fn defer_render_timeout(
        &mut self,
        packet: &Arc<Packet>,
        error: Error,
    ) -> bool {
        // Its tracker may already be gone while off-lock completion still
        // owns the fence and admission lease. A loan for a subsequent packet
        // cannot turn this exact accepted retirement into a missing-owner error.
        if packet.completion.retirement_queued() {
            return true;
        }
        let Some(slot) = packet.completion.render_slot() else {
            return false;
        };
        let Ok(entry) = self.render.entry_mut(slot) else {
            return false;
        };
        if entry.preparing.is_none() && !entry.growing {
            return false;
        }
        if !entry
            .borrowed_dependencies
            .as_ref()
            .is_some_and(|dependencies| dependencies.contains_packet(packet))
        {
            return false;
        }
        packet.completion.defer_timeout(error);
        entry.pending_timeout = true;
        true
    }

    /// The oldest published group of every render pair, for cross-VM timeout attribution
    /// (crate::g17::timeout).
    pub(in crate::g17) fn render_timeout_heads(
        &self,
    ) -> impl Iterator<Item = crate::g17::timeout::Head> + '_ {
        self.render
            .entries
            .iter()
            .flatten()
            .filter_map(|entry| entry.pair.as_ref())
            .filter_map(|pair| pair.timeout_head())
    }

    /// A render watchdog blocked behind another VM's older unfinished work gets the same
    /// bounded grace as compute; the oldest unfinished work is never excused. When the GPU
    /// stalls on one client, the others' renders then outlive the culprit's teardown instead
    /// of all timing out together.
    pub(in crate::g17) fn render_timeout_decision(
        &self,
        packet: &Arc<Packet>,
        renewals: u8,
    ) -> Option<crate::g17::timeout::Verdict> {
        let own = self
            .render
            .entries
            .iter()
            .flatten()
            .filter_map(|entry| entry.pair.as_ref())
            .find(|pair| pair.contains_packet(packet))
            .and_then(|pair| pair.timeout_head());
        let computes = self
            .compute
            .iter()
            .flatten()
            .filter_map(|entry| entry.queue.as_deref())
            .filter_map(|queue| queue.timeout_head());
        // Without a head of its own the timeout is its own.
        own.map(|own| {
            crate::g17::timeout::attribute(
                own,
                renewals,
                Iterator::chain(self.render_timeout_heads(), computes),
            )
        })
    }
}

impl RenderState {
    pub(super) fn write_capacity(&self, f: &mut kernel::fmt::Formatter<'_>) -> kernel::fmt::Result {
        writeln!(f, "retired_render_bindings used={} total={}", self.retired.len(), self.retired.capacity())?;
        writeln!(f, "firmware_linked_render_graphs kept={} leaked={}", self.kept_graphs.len(),
                 teardown::LEAKED_GRAPHS.load(core::sync::atomic::Ordering::Relaxed))?;
        self.pools.write_capacity(f)?;
        for (slot, entry) in self.entries.iter().enumerate() {
            if let Some(context) = entry.as_ref().and_then(|entry| entry.context.as_ref()) {
                context.write_capacity(f, "render", slot)?;
            }
        }
        Ok(())
    }
}

impl Registry {
    /// Give idle private backing to another ready client under capacity pressure.
    /// Keep graph/QID/context ownership; the next use rearms the same pool after
    /// its consumed release, exactly as eviction-driven reclaim does.
    pub(super) fn reclaim_idle_render_pools(
        &mut self, init: &crate::g17::initdata::InitData, deferred: &mut DeferredBatch,
    ) -> Result {
        let render = &mut self.render;
        render.pools.observe(init)?;
        for entry in render.entries.iter().flatten() {
            if !deferred.render_backing_room() { break; }
            let Some(pool) = entry.pool.as_ref() else { continue; };
            // None means this pool never acquired a firmware registration;
            // otherwise require its exact final release to have been consumed.
            if pool.vacant() || !render.pools.released(u32::from(pool.id()), pool.control_va())
                || self.admission.owner_active(entry.reservation.owner) { continue; }
            // A sibling or replacement may be building off-lock before it has
            // installed its pool reference. Its logical owner also excludes us.
            let mut idle = true;
            for other in render.entries.iter().flatten().filter(|other| {
                other.reservation.owner == entry.reservation.owner
                    || other.pool.as_ref().is_some_and(|bound| Arc::ptr_eq(bound, pool))
            }) {
                if other.constructing || other.growing || other.preparing.is_some()
                    || other.pending_error.is_some() || other.pending_timeout {
                    idle = false;
                    break;
                }
                if let Some(pair) = other.pair.as_ref() {
                    if !pair.idle_for_context(pair.context())? { idle = false; break; }
                }
            }
            if idle {
                if let Some(backing) = pool.vacate() { deferred.defer_render_backing(backing); }
            }
        }
        Ok(())
    }
}
