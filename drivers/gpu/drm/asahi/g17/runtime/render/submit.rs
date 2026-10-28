// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Submission admission, preparation and the firmware visibility boundary.

use super::super::{Backend, DeferredBatch, Registry};
use crate::g17::{
    job::Packet,
    queue::admission::{RenderLease, Request},
};
use core::sync::atomic::Ordering;
use kernel::{prelude::*, sync::Arc};

/// Internal result of a publication attempted from the submitting ioctl that
/// met contention before anything became visible: the scheduler publishes the
/// command instead. Never returned to userspace.
pub(in crate::g17::runtime) const FALLBACK: Error = EINPROGRESS;

impl Backend {
    /// The submitting ioctl holds the logical queue's mutex and never sleeps
    /// for contention or a closed preparation gate: it returns FALLBACK, and
    /// the scheduler's worker publishes the command and owns the waits.
    pub(in crate::g17::runtime) fn enter_preparation(&self, sleep: bool) -> Result<crate::g17::preparation::Lease> {
        if sleep {
            return self.shared.preparations.enter(false);
        }
        // A closed gate (recovery) is the only case enter() would sleep for.
        self.shared.preparations.try_enter().map_err(|error| {
            if error == EAGAIN {
                FALLBACK
            } else {
                error
            }
        })
    }

    pub(in crate::g17) fn render_packet(
        &self,
        packet: &Arc<Packet>,
        deferred: &mut DeferredBatch,
    ) -> Result {
        self.render_with_lease(packet, None, deferred)
    }

    /// Reserve identities before attaching a lease to the packet. An empty
    /// logical slot must not hide reusable physical channels when the shared
    /// queue-ID namespace is full.
    /// Also reports whether the owner's installed pair is ready to take the
    /// command, so that the publication skips graph construction.
    fn acquire_render_with_ids(
        &self,
        packet: &Arc<Packet>,
        mut next: Option<RenderLease>,
        sleep: bool,
    ) -> Result<(RenderLease, bool)> {
        let pool = &self.admission;
        let started = super::super::now_ns();
        let mut deadline_ns =
            started.saturating_add(u64::from(crate::g17::job::TIMEOUT_MS) * 1_000_000);
        let recovery_deadline_ns = started.saturating_add(recovery_wait_ns());
        // A capacity refusal samples the render domain and retries once; only
        // a refusal after that sample waits, so no release can be missed.
        let mut observed = None;
        let mut capacity = WaitClock::default();
        loop {
            self.admission_open()?;
            if super::super::now_ns() >= deadline_ns {
                if deadline_ns < recovery_deadline_ns && self.waiting_for_recovery()? {
                    deadline_ns = recovery_deadline_ns;
                } else {
                    return Err(EBUSY);
                }
            }
            if packet.cancel_requested() {
                return Err(ECANCELED);
            }
            if packet.completion.status().get() != 0 {
                return Err(EIO);
            }
            let lease = if let Some(lease) = next.take() {
                lease
            } else {
                // Admission sleeps for a pick; the ioctl leaves that to the scheduler.
                if !sleep {
                    return Err(FALLBACK);
                }
                match pool.acquire_render(self.owner, Request::of(packet), deadline_ns, || {
                    // Atomics only: the callback runs under the pool mutex.
                    if packet.cancel_requested() || self.closed.load(Ordering::Acquire) {
                        return Err(ECANCELED);
                    }
                    if packet.completion.status().get() != 0 || self.shared.crashed.load(Ordering::Acquire) {
                        return Err(EIO);
                    }
                    Ok(())
                }) {
                    Ok(lease) => lease,
                    Err(error) if error == ENOSPC => {
                        let reclaimed = {
                            let mut state = self.shared.state.lock();
                            (*state)
                                .as_deref_mut()
                                .ok_or(ENODEV)?
                                .queues
                                .reclaim_unused_render_spare(Some(self.owner), false)?
                        };
                        if reclaimed.is_none() {
                            self.wait_render_capacity(&mut observed, &mut capacity, sleep)?;
                        }
                        drop(reclaimed);
                        continue;
                    }
                    // The deadline check above decides whether a recovery extends the wait.
                    Err(error) if error == EBUSY => continue,
                    Err(error) => return Err(error),
                }
            };
            let slot = lease.slot();
            let mut state = self.shared.state.lock();
            let firmware = (*state).as_deref_mut().ok_or(ENODEV)?;
            match firmware.queues.ensure_render_ids(slot, self.owner) {
                Ok(()) => {
                    let ready = firmware.queues.render_ready(slot, self.owner, &self.context)?;
                    return Ok((lease, ready));
                }
                Err(error) if error == ENOSPC => {
                    let reclaimed = firmware.queues.reclaim_unused_render_spare(None, true)?;
                    if reclaimed.is_some() {
                        drop(state);
                        drop(reclaimed);
                        next = Some(lease);
                        continue;
                    }
                    // ensure_render_ids leaves no partial entry on refusal.
                    if firmware.queues.render.entries[usize::from(slot)].is_some() {
                        return Err(error);
                    }
                    if !lease.cancel_empty(self.owner) {
                        continue;
                    }
                    let reclaimed = firmware
                        .queues
                        .reclaim_unused_render_spare(Some(self.owner), true)?;
                    let ready = firmware.queues.has_render_ids(self.owner);
                    drop(state);
                    // A full queue table makes admission reuse a free killed pair instead.
                    if reclaimed.is_none() && !ready && !pool.killed_pair_free() {
                        self.wait_render_capacity(&mut observed, &mut capacity, sleep)?;
                    }
                    drop(reclaimed);
                }
                Err(error) => return Err(error),
            }
        }
    }

    /// Scheduler admission of a render packet. A work state of its context or
    /// a pair admission it lacks becomes a fence that the return providing it
    /// signals, so run_job finds both ready instead of waiting for them.
    /// Errors, and anything else publication may wait for, are left to it.
    pub(in crate::g17::runtime) fn admit_render(&self, packet: &Arc<Packet>) -> Option<kernel::dma_fence::Fence> {
        if packet.early_result().is_some()
            || packet.cancel_requested()
            || packet.completion.status().get() != 0
            || self.admission_open().is_err()
        {
            return None;
        }
        // Each check runs once more after sampling its domain, so the common
        // case takes no domain lock and no return between check and wait is lost.
        let work = packet.completion.work_state().ok()?;
        loop {
            match work.acquire(crate::g17::job::Engine::Render) {
                Err(error) if error == EBUSY => {
                    let states = work.returns().observe();
                    match work.acquire(crate::g17::job::Engine::Render) {
                        Err(error) if error == EBUSY => match work.returns().wait(states) {
                            Ok(Some(fence)) => return Some(fence),
                            Ok(None) => continue,
                            Err(_) => return None,
                        },
                        Err(_) => return None,
                        Ok(_) => {}
                    }
                }
                Err(_) => return None,
                Ok(_) => {}
            }
            let request = Request::of(packet);
            if self.admission.admits_render(self.owner, request) {
                return None;
            }
            let observed = self.shared.renders.observe();
            if self.admission.admits_render(self.owner, request) {
                return None;
            }
            self.shared.queue_events();
            match self.shared.renders.wait(observed) {
                Ok(Some(fence)) => return Some(fence),
                Ok(None) => continue,
                Err(_) => return None,
            }
        }
    }

    /// Conditions which end a publication that has not become visible yet.
    fn publication_open(&self, packet: &Packet) -> Result {
        self.admission_open()?;
        if packet.cancel_requested() {
            return Err(ECANCELED);
        }
        if packet.completion.status().get() != 0 {
            return Err(EIO);
        }
        Ok(())
    }

    /// Blocks on `domain` until a notification after `observed`, or the
    /// deadline, which only bounds the wait. Slices of at most 10 ms let the
    /// publisher observe cancellation, which has no notification here.
    fn wait_domain(
        &self,
        domain: &crate::g17::availability::Availability,
        observed: u64,
        deadline_ns: u64,
        packet: &Packet,
    ) -> Result {
        self.publication_open(packet)?;
        let remaining = deadline_ns.saturating_sub(super::super::now_ns());
        if remaining == 0 {
            return Err(ETIMEDOUT);
        }
        domain.block(observed, remaining.div_ceil(1_000_000).min(10) as u32, false)
    }

    /// A recovery is pending, or this owner's pair is spared and draining after one: an
    /// innocent publication waits up to RECOVERY_WAIT_MS for it instead of failing.
    fn waiting_for_recovery(&self) -> Result<bool> {
        let state = self.shared.state.lock();
        let firmware = (*state).as_deref().ok_or(ENODEV)?;
        Ok(firmware.recovery.pending()
            || crate::g17::recovery::Memory::recovery_state(&firmware.init)? != 0
            || firmware.queues.admission.spared_quarantined(self.owner))
    }

    /// The deadline of the wait `clock` measures (WaitClock::deadline).
    fn wait_deadline(&self, clock: &mut WaitClock, bound_ns: u64) -> Result<u64> {
        let recovering = self.waiting_for_recovery()?;
        Ok(clock.deadline(super::super::now_ns(), recovering, bound_ns))
    }

    /// A pair is borrowed, still settling, waiting for firmware room or for the
    /// release of the scheduler it last ran for. Every pair return, retirement
    /// and teardown step signals the render domain, and each event pass does
    /// while a publisher waits; the deadline is only the failure bound.
    fn wait_render_busy(
        &self,
        clock: &mut WaitClock,
        observed: &mut Option<u64>,
        sleep: bool,
        packet: &Packet,
    ) -> Result {
        if !sleep {
            return Err(FALLBACK);
        }
        let deadline = self.wait_deadline(clock, super::super::WORK_RING_WAIT_NS)?;
        // Sample, then retry once: only a refusal after the sample waits.
        let Some(sample) = observed.take() else {
            *observed = Some(self.shared.renders.observe());
            return Ok(());
        };
        self.shared.queue_events();
        self.wait_domain(&self.shared.renders, sample, deadline, packet)
    }

    /// Render capacity is short: return what idle owners hold, then block until
    /// a pair, lease or owner returns. No packet is firmware-visible here, and
    /// neither the device mutex nor a preparation lease is held while waiting.
    fn wait_render_capacity(
        &self,
        observed: &mut Option<u64>,
        clock: &mut WaitClock,
        sleep: bool,
    ) -> Result {
        if !sleep {
            return Err(FALLBACK);
        }
        if self.reclaim_render_capacity()? {
            *observed = None;
            return Ok(());
        }
        let Some(sample) = observed.take() else {
            *observed = Some(self.shared.renders.observe());
            return Ok(());
        };
        self.shared.queue_events();
        self.admission_open()?;
        let remaining = self
            .wait_deadline(clock, super::super::WORK_RING_WAIT_NS)?
            .saturating_sub(super::super::now_ns());
        if remaining == 0 {
            return Err(EBUSY);
        }
        self.shared
            .renders
            .block(sample, remaining.div_ceil(1_000_000).min(10) as u32, false)
    }

    /// Gives idle render backing back under capacity pressure. Returns whether
    /// anything was returned, in which case the caller retries at once.
    fn reclaim_render_capacity(&self) -> Result<bool> {
        let mut deferred = DeferredBatch::publication()?;
        let result: Result<bool> = (|| {
            let lease = self.shared.preparations.try_enter()?;
            let mut state = self.shared.state.lock();
            let firmware = (*state).as_deref_mut().ok_or(ENODEV)?;
            if !lease.is_current() || self.shared.crashed.load(Ordering::Acquire) {
                return Err(EAGAIN);
            }
            if firmware.recovery.pending()
                || crate::g17::recovery::Memory::recovery_state(&firmware.init)? != 0
            {
                return Ok(false);
            }
            firmware.queues.observe_pool_releases(&firmware.init, &mut deferred)?;
            firmware.queues.reclaim_idle_render_pools(&firmware.init, &mut deferred)?;
            let mut published = 0;
            if !firmware.render_control_backpressured() {
                (published, _) = firmware.queues.vacate_idle_pools(
                    &firmware.init,
                    0,
                    super::super::now_ns(),
                    0,
                )?;
                if published != 0 {
                    firmware.primary.notify(crate::g17::MSG_CONTROL_NOTIFY)?;
                }
            }
            Ok(published != 0
                || !deferred.render.is_empty()
                || !deferred.vacated.is_empty()
                || !deferred.bindings.is_empty())
        })();
        deferred.finish();
        match result {
            Err(error) if error == EAGAIN => Ok(false),
            result => result,
        }
    }

    fn render_with_lease(
        &self,
        packet: &Arc<Packet>,
        early: Option<RenderLease>,
        deferred: &mut DeferredBatch,
    ) -> Result {
        packet.check_dependencies()?;
        self.admission_open()?;
        if packet.cancel_requested() {
            return Err(ECANCELED);
        }
        if packet.completion.status().get() != 0 {
            return Err(EIO);
        }
        let interruptible = early.is_none();
        // A publication from the submitting ioctl (an early lease) never
        // sleeps for contention; it falls back to the scheduler instead.
        let sleep = early.is_none();
        let mut early = early;
        // The pair this owner last published to directly: a lease on it goes
        // straight to the direct path, whose lend checks everything the
        // identity check would.
        let known = self.direct_slot.load(Ordering::Relaxed);
        let lease = match early.take() {
            Some(lease) => Some(lease),
            None if known != 0 => self
                .admission
                .try_acquire_render(self.owner, Request::of(packet))
                .ok()
                .flatten(),
            None => None,
        };
        let early = match lease {
            Some(lease) if u32::from(lease.slot()) == known => {
                let slot = lease.slot();
                packet.completion.attach_render_lease(lease)?;
                if let Some(result) = self.render_direct(packet, slot, deferred) {
                    return result;
                }
                packet.completion.detach_render_lease()
            }
            lease => lease,
        };
        let (admission, mut ready) = self.acquire_render_with_ids(packet, early, sleep)?;
        let slot = admission.slot();
        packet.completion.attach_render_lease(admission)?;
        if ready {
            if let Some(result) = self.render_direct(packet, slot, deferred) {
                return result;
            }
        }
        let mut busy = WaitClock::default();
        let mut capacity = WaitClock::default();
        let mut state_clock = WaitClock::default();
        let mut recovery_since = None;
        let mut seen = None;
        let mut states = None;
        loop {
            if packet.cancel_requested() {
                return Err(ECANCELED);
            }
            if packet.completion.status().get() != 0 || self.shared.crashed.load(Ordering::Acquire)
            {
                return Err(EIO);
            }
            if !self.context.is_current() {
                return Err(EFAULT);
            }
            let work_state = {
                let preparation = if sleep {
                    self.shared.preparations.enter(interruptible)?
                } else {
                    self.enter_preparation(false)?
                };
                let result = packet.completion.work_state().and_then(|state| state.acquire(crate::g17::job::Engine::Render));
                if preparation.is_current() {
                    result
                } else {
                    Err(EAGAIN)
                }
            };
            match work_state {
                Err(error) if error == EAGAIN => continue,
                Err(error) if error == EBUSY => {
                    if !sleep {
                        return Err(FALLBACK);
                    }
                    // Every node of this context belongs to unretired work:
                    // sample, retry once, then wait for the next returned one.
                    let work = packet.completion.work_state()?;
                    match states.take() {
                        None => states = Some(work.returns().observe()),
                        // Work states free as earlier, possibly spared, work retires.
                        Some(sample) => {
                            let deadline = self
                                .wait_deadline(&mut state_clock, super::super::WORK_RING_WAIT_NS)?;
                            self.wait_domain(work.returns(), sample, deadline, packet)?
                        }
                    }
                    continue;
                }
                result => {
                    result?;
                }
            }
            // A pair found ready under the identity check needs no build; any
            // retry revalidates through ensure_render.
            let ensured = if core::mem::take(&mut ready) {
                Ok(())
            } else {
                self.ensure_render(packet, slot, sleep)
            };
            match ensured {
                Err(error) if error == ENOSPC => {
                    self.wait_render_capacity(&mut seen, &mut capacity, sleep)?;
                    continue;
                }
                Err(error) if error == EAGAIN => continue,
                Err(error) if error == EBUSY => {
                    self.wait_render_busy(&mut busy, &mut seen, sleep, packet)?;
                    continue;
                }
                result => result?,
            }
            match self.prepare_render(slot, packet, deferred, sleep) {
                Err(error) if !packet.is_published() && error == ENOSPC => {
                    self.wait_render_capacity(&mut seen, &mut capacity, sleep)?;
                    continue;
                }
                Err(error) if !packet.is_published() && [EAGAIN, EBUSY].contains(&error) => {
                    let (recovering, gate) = {
                        let state = self.shared.state.lock();
                        let firmware = (*state).as_deref().ok_or(ENODEV)?;
                        (
                            firmware.recovery.pending()
                                || crate::g17::recovery::Memory::recovery_state(&firmware.init)? != 0,
                            self.shared.preparations.snapshot(),
                        )
                    };
                    if recovering {
                        if !sleep {
                            return Err(FALLBACK);
                        }
                        // Recovery closes the preparation gate before touching
                        // any queue; the next pass blocks in enter() until it
                        // reopens. The worker raises the closure.
                        let since = *recovery_since.get_or_insert(super::super::now_ns());
                        self.shared.queue_events();
                        self.shared
                            .preparations
                            .wait_closure(gate, since.saturating_add(recovery_wait_ns()))?;
                    } else if error == EBUSY {
                        self.wait_render_busy(&mut busy, &mut seen, sleep, packet)?;
                    } else if !sleep {
                        return Err(FALLBACK);
                    }
                }
                result => return result,
            }
        }
    }

    pub(in crate::g17) fn try_render_early(&self, packet: &Arc<Packet>) -> bool {
        if self.backlog.load(Ordering::Acquire) != 0 || packet.has_dependencies() {
            return false;
        }
        let admission = match self.admission.try_acquire_render(self.owner, Request::of(packet)) {
            Ok(Some(lease)) => lease,
            _ => return false,
        };
        if !packet.begin_publication() {
            packet.completion.fail_unpublished(ECANCELED);
            packet.set_early_result(Err(ECANCELED));
            return true;
        }
        let mut deferred = match DeferredBatch::publication() {
            Ok(deferred) => deferred,
            Err(error) => {
                if packet.completion.status().get() != 0 {
                    <Self as crate::g17::job::Backend>::fail_vm(self, error);
                }
                packet.completion.fail_unpublished(error);
                packet.set_early_result(Err(error));
                return true;
            }
        };
        let result = self.render_with_lease(packet, Some(admission), &mut deferred);
        if matches!(result, Err(error) if error == FALLBACK) && !packet.is_published() {
            // Contention before anything became visible: the ioctl does not
            // sleep. Return the lease and the claim; the scheduler publishes
            // the command, and later ones wait behind it in the backlog.
            let lease = packet.completion.detach_render_lease();
            deferred.finish();
            drop(lease);
            packet.abandon_publication();
            return false;
        }
        let result = self
            .finish_publication(packet, result, &mut deferred)
            .map(|_| ());
        packet.set_early_result(result);
        true
    }
}

impl Registry {
    fn render_dependencies(
        &self,
        owner: u64,
        packet: &Arc<Packet>,
        state: u32,
    ) -> Result<crate::g17::queue::render::RenderDependencies> {
        let mut dependencies = crate::g17::queue::render::RenderDependencies::EMPTY;
        let frontier = self.render_prefix(owner, packet)?;
        for dependency in frontier.as_slice() {
            *dependencies
                .render
                .get_mut(dependencies.render_count)
                .ok_or(EOVERFLOW)? = *dependency;
            dependencies.render_count += 1;
        }
        if dependencies.render_count != 0
            && matches!(&packet.command, crate::g17::command::Validated::Render { pass, .. }
                if pass.fragment_barriers)
        {
            let tiling = self.tiling_prefix(owner, packet)?;
            for dependency in tiling.as_slice() {
                *dependencies
                    .tiling
                    .get_mut(dependencies.tiling_count)
                    .ok_or(EOVERFLOW)? = *dependency;
                dependencies.tiling_count += 1;
            }
        }
        if let Some(queue) = self.compute(owner) {
            dependencies.compute = queue.dependency(
                owner,
                packet.completion.status(),
                packet.order.wait_through[1],
                state,
            )?;
        }
        Ok(dependencies)
    }
}

impl crate::g17::Firmware {
    pub(super) fn apply_render_deferred(
        &mut self,
        slot: u8,
        deferred: &mut DeferredBatch,
    ) -> Result {
        let observe_before_failure = {
            let entry = self.queues.render.entry_mut(slot)?;
            let pair = entry.pair.as_ref().ok_or(EIO)?;
            entry.pending_error.as_ref().is_some_and(|(status, _)| {
                Arc::ptr_eq(pair.context().status(), status)
            })
        };
        if observe_before_failure {
            // VM failure could not observe this pair while preparation owned
            // it. Restore the same final-observation boundary used by
            // settle_render_vm before quarantining any still-owned commands.
            // Keep pending_error intact if observation itself fails.
            self.retire_render_slot(usize::from(slot), None, deferred)?;
        }
        let entry = self.queues.render.entry_mut(slot)?;
        let pair = entry.pair.as_mut().ok_or(EIO)?;
        if let Some((status, error)) = entry.pending_error.take() {
            if Arc::ptr_eq(pair.context().status(), &status) {
                pair.quarantine(error, &mut |event| deferred.push(event))?;
            }
        }
        if entry.closed {
            pair.release_owner();
        }
        if !entry.pending_timeout {
            return Ok(());
        }
        for _ in 0..crate::hw::t8140::queues::RENDER_DEPTH {
            if self.recovery.pending() || self.primary.state.shared.crashed.load(Ordering::Acquire)
            {
                return Ok(());
            }
            let Some((packet, error)) = self
                .queues
                .render
                .entry(slot)?
                .pair
                .as_ref()
                .ok_or(EIO)?
                .take_deferred_timeout()
            else {
                self.queues.render.entry_mut(slot)?.pending_timeout = false;
                return Ok(());
            };
            if let Err(failure) = self.fail_render_packet(&packet, error, deferred) {
                packet.completion.defer_timeout(error);
                return Err(failure);
            }
        }
        self.queues.render.entry_mut(slot)?.pending_timeout = false;
        Ok(())
    }

    pub(super) fn publish_render(
        &mut self,
        owner: u64,
        slot: u8,
        packet: &Arc<Packet>,
        deferred: &mut DeferredBatch,
    ) -> Result {
        use crate::g17::queue::render::PublicationLease;
        let state = crate::g17::recovery::Memory::recovery_state(&self.init)?;
        if self.recovery.pending() || state != 0 {
            return Err(EAGAIN);
        }
        if packet.completion.status().get() != 0 {
            return Err(EIO);
        }
        let mut dependencies = self.queues.render_dependencies(owner, packet, state)?;
        // Every render publication, with or without the device mutex, holds the
        // port's render lock: the render work rings have one host producer at a
        // time and attachment overlaps are ordered by publication order.
        let port = self.port.clone();
        let _render = port.render_lock();
        // Both outer destinations must have room before pool accounting or
        // either inner queue becomes visible.
        let priority = packet.context.policy().priority();
        if !self.init.work_ready(priority, crate::g17::fw::queue::DataMaster::Fragment)?
            || !self.init.work_ready(priority, crate::g17::fw::queue::DataMaster::Tiling)?
        {
            return Err(EBUSY);
        }
        if let Some(footprint) = packet.footprint.as_ref() {
            dependencies.hazard_count = self.queues.admission.publication_barriers(
                owner,
                slot,
                footprint,
                &mut dependencies.hazards,
            )?;
        }
        let pair = self.queues.render.entry(slot)?.pair.as_ref().ok_or(EIO)?;
        // An owner change taken from a live owner which closed meanwhile: its
        // first command waits until that scheduler's release has been consumed.
        if pair
            .owner_switch_from(&packet.context)
            .is_some_and(|scheduler| self.queues.teardown.releasing_scheduler(scheduler))
        {
            return Err(EBUSY);
        }
        let pool = pair.pool().clone();
        let buffer_id = pair.buffer_id();
        let (buffer_id, buffer_token) = self.queues.buffers.acquire_owner(buffer_id)?;
        let preparation = match self.queues.render.pools.prepare(&self.init, &pool) {
            Ok(preparation) => preparation,
            Err(error) => {
                self.queues.buffers.release(buffer_id)?;
                return Err(error);
            }
        };
        let lease = PublicationLease {
            free_list_generation: preparation.generation,
            buffer_token,
        };
        let result = self.with_render(usize::from(slot), |pair, host| {
            pair.publish(
                packet.clone(),
                &dependencies,
                lease,
                host,
                &mut |event| deferred.push(event),
            )
        });
        if packet.is_published() {
            self.queues.render.entry_mut(slot)?.published = true;
        } else if result.is_err() {
            self.queues.render.pools.rollback(preparation);
            self.queues.buffers.release(buffer_id)?;
            self.queues
                .render
                .entry_mut(slot)?
                .pair
                .as_mut()
                .ok_or(EIO)?
                .discard_prepared(packet)?;
        }
        result
    }
}

fn recovery_wait_ns() -> u64 {
    u64::from(crate::g17::timeout::RECOVERY_WAIT_MS) * 1_000_000
}

/// One bounded wait (Backend::wait_deadline).
#[derive(Default)]
struct WaitClock {
    first: Option<u64>,
    calm: u64,
}

impl WaitClock {
    /// `bound_ns` after the later of the first call and the last call that saw a recovery
    /// pending or this owner's spared pair draining (so the usual bound applies afresh once a
    /// recovery ends), and at most RECOVERY_WAIT_MS after the first call.
    fn deadline(&mut self, now: u64, recovering: bool, bound_ns: u64) -> u64 {
        let first = *self.first.get_or_insert(now);
        self.calm = if recovering { now } else { self.calm.max(first) };
        first
            .saturating_add(recovery_wait_ns())
            .min(self.calm.saturating_add(bound_ns))
    }
}

#[kunit_tests(asahi_g17_wait_clock)]
mod tests {
    use super::*;

    const BOUND: u64 = 2_000_000_000;

    #[test]
    fn a_calm_wait_keeps_its_first_deadline() {
        let mut clock = WaitClock::default();
        assert_eq!(clock.deadline(100, false, BOUND), 100 + BOUND);
        assert_eq!(clock.deadline(500, false, BOUND), 100 + BOUND);
    }

    #[test]
    fn the_bound_starts_afresh_once_a_recovery_ends() {
        let mut clock = WaitClock::default();
        clock.deadline(100, false, BOUND);
        assert_eq!(clock.deadline(1_000, true, BOUND), 1_000 + BOUND);
        assert_eq!(clock.deadline(2_000, false, BOUND), 1_000 + BOUND);
    }

    #[test]
    fn a_recovery_cannot_extend_past_the_recovery_wait() {
        let mut clock = WaitClock::default();
        clock.deadline(100, true, BOUND);
        let limit = 100 + recovery_wait_ns();
        assert_eq!(clock.deadline(limit - 1, true, BOUND), limit);
    }
}
