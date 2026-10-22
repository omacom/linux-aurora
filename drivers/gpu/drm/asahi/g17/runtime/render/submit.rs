// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Submission admission, preparation and the firmware visibility boundary.

use super::super::{Backend, DeferredBatch, Registry, Wait, FALLBACK};
use crate::g17::{
    job::Packet,
    queue::admission::{RenderLease, Request},
};
use core::sync::atomic::Ordering;
use kernel::{
    prelude::*,
    sync::Arc,
    time::{delay::fsleep, Delta},
};

impl Backend {
    /// Render capacity is held by other live owners: admission slots, queue
    /// IDs, parameter-buffer and counter identities, retired-binding room or
    /// shared GPU VA, whose spares are still being built or whose siblings are
    /// busy. Retry reclamation after their next release instead of failing
    /// the packet; fail only once the publication's deadline has passed.
    fn wait_render_capacity(&self, wait: Wait, started: &mut bool) -> Result {
        use core::sync::atomic::AtomicBool;
        static STARTED: AtomicBool = AtomicBool::new(false);
        static EXPIRED: AtomicBool = AtomicBool::new(false);
        if wait.expired()? {
            if !EXPIRED.swap(true, Ordering::Relaxed) {
                self.log_render_census("wait expired");
            }
            return Err(resource_exhausted!("render capacity held by open owners"));
        }
        if !core::mem::replace(started, true) && !STARTED.swap(true, Ordering::Relaxed) {
            self.log_render_census("wait started");
        }
        self.shared.queue_events();
        fsleep(Delta::from_millis(1));
        Ok(())
    }

    /// A pair is borrowed, still settling, or waiting for its previous
    /// scheduler's release to be consumed and parked (the event worker does
    /// both). Poll briefly until the publication's deadline, then name the
    /// state that kept pair `slot` busy.
    ///
    /// A borrower (an event pass observing the pair, a preparation or a
    /// growth) returns the pair under the device mutex and then notifies
    /// `changed`, usually within microseconds: wait for that notification,
    /// bounded by one tick, instead of the fixed sleep.
    fn wait_render_busy(&self, slot: u8, wait: Wait) -> Result {
        if wait.expired()? {
            let cause = {
                let state = self.shared.state.lock();
                (*state).as_deref().map_or("the device stopped", |firmware| {
                    firmware.render_busy_cause(slot, self.owner, &self.context)
                })
            };
            return Err(self.wait_expired(
                ETIMEDOUT,
                format_args!("a busy render pair (slot {}: {})", slot, cause),
            ));
        }
        self.shared.queue_events();
        {
            let mut state = self.shared.state.lock();
            let borrowed = (*state)
                .as_deref()
                .is_some_and(|firmware| firmware.queues.render.borrowed(slot));
            if borrowed {
                let _ = self.shared.changed.wait_timeout(&mut state, 1);
                return Ok(());
            }
        }
        fsleep(Delta::from_micros(200));
        Ok(())
    }

    /// Once-per-boot capacity snapshot taken with the device mutex.
    fn log_render_census(&self, reason: &str) {
        let state = self.shared.state.lock();
        if let Some(firmware) = (*state).as_deref() {
            pr_warn!("asahi: G17 render capacity {}: {}\n", reason, firmware.capacity());
        }
    }

    pub(in crate::g17) fn render_packet(
        &self,
        packet: &Arc<Packet>,
        deferred: &mut DeferredBatch,
    ) -> Result {
        self.render_with_lease(packet, None, Wait::scheduled(), deferred)
    }

    /// Reserve identities before attaching a lease to the packet. An empty
    /// logical slot must not hide reusable physical channels when the shared
    /// queue-ID namespace is full.
    fn acquire_render_with_ids(
        &self,
        packet: &Arc<Packet>,
        mut next: Option<RenderLease>,
        wait: Wait,
    ) -> Result<RenderLease> {
        let pool = {
            let state = self.shared.state.lock();
            (*state).as_deref().ok_or(ENODEV)?.queues.admission.clone()
        };
        let mut capacity_wait = false;
        loop {
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
                let Some(deadline) = wait.deadline() else {
                    return Err(FALLBACK);
                };
                match pool.acquire_render(self.owner, Request::of(packet), deadline) {
                    Ok(lease) => lease,
                    Err(error) if error == EBUSY => {
                        return Err(self.wait_expired(error, "a render admission slot"))
                    }
                    Err(error) if error == ENOSPC => {
                        // Live owners hold every slot. Prefer unused warming,
                        // then an idle sibling of an owner keeping another pair.
                        let (reclaimed, forfeited) = {
                            let mut state = self.shared.state.lock();
                            let queues = &mut (*state).as_deref_mut().ok_or(ENODEV)?.queues;
                            let reclaimed =
                                queues.reclaim_unused_render_spare(Some(self.owner), false)?;
                            let forfeited = reclaimed.is_none()
                                && queues.forfeit_idle_render_sibling(self.owner)?;
                            (reclaimed, forfeited)
                        };
                        if reclaimed.is_none() && !forfeited {
                            self.wait_render_capacity(wait, &mut capacity_wait)?;
                            continue;
                        }
                        drop(reclaimed);
                        continue;
                    }
                    Err(error) => return Err(error),
                }
            };
            let slot = lease.slot();
            let mut state = self.shared.state.lock();
            let firmware = (*state).as_deref_mut().ok_or(ENODEV)?;
            match firmware.queues.ensure_render_ids(slot, self.owner) {
                Ok(()) => return Ok(lease),
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
                        drop(state);
                        self.wait_render_capacity(wait, &mut capacity_wait)?;
                        continue;
                    }
                    let reclaimed = firmware
                        .queues
                        .reclaim_unused_render_spare(Some(self.owner), true)?;
                    let ready = firmware.queues.has_render_ids(self.owner);
                    // Every remaining QID belongs to an installed graph: take an
                    // idle installed sibling, whose identities move with it.
                    let forfeited = reclaimed.is_none()
                        && !ready
                        && firmware.queues.forfeit_idle_render_sibling(self.owner)?;
                    drop(state);
                    if reclaimed.is_none() && !ready && !forfeited {
                        self.wait_render_capacity(wait, &mut capacity_wait)?;
                        continue;
                    }
                    drop(reclaimed);
                }
                Err(error) => return Err(error),
            }
        }
    }

    fn render_with_lease(
        &self,
        packet: &Arc<Packet>,
        early: Option<RenderLease>,
        wait: Wait,
        deferred: &mut DeferredBatch,
    ) -> Result {
        packet.check_dependencies()?;
        if packet.cancel_requested() {
            return Err(ECANCELED);
        }
        if packet.completion.status().get() != 0 {
            return Err(EIO);
        }
        let admission = self.acquire_render_with_ids(packet, early, wait)?;
        let slot = admission.slot();
        packet.completion.attach_render_lease(admission)?;
        let mut state_delay = 20i64;
        let mut recovery_polls = 0;
        let mut capacity_wait = false;
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
                let preparation = self.enter_preparation(wait)?;
                let result = packet.completion.work_state().and_then(|state| state.acquire());
                if preparation.is_current() {
                    result
                } else {
                    Err(EAGAIN)
                }
            };
            match work_state {
                Err(error) if error == EAGAIN => continue,
                Err(error) if error == EBUSY => {
                    if wait.expired()? {
                        return Err(self.wait_expired(ETIMEDOUT, "a free render work state"));
                    }
                    fsleep(Delta::from_micros(state_delay));
                    state_delay = (state_delay * 2).min(1000);
                    continue;
                }
                result => {
                    result?;
                }
            }
            match self.ensure_render(packet, slot, wait) {
                Err(error) if error == EAGAIN => continue,
                Err(error) if error == EBUSY => {
                    self.wait_render_busy(slot, wait)?;
                    continue;
                }
                // Graph or binding resources held by other owners: every failed
                // build left its reservation unchanged, so retry after a release.
                Err(error) if error == ENOSPC => {
                    self.wait_render_capacity(wait, &mut capacity_wait)?;
                    continue;
                }
                result => result?,
            }
            match self.prepare_render(slot, packet, deferred, wait) {
                Err(error) if !packet.is_published() && [EAGAIN, EBUSY].contains(&error) => {
                    let recovering = {
                        let state = self.shared.state.lock();
                        let firmware = (*state).as_deref().ok_or(ENODEV)?;
                        firmware.recovery.pending()
                            || crate::g17::recovery::Memory::recovery_state(&firmware.init)? != 0
                    };
                    if recovering {
                        if wait.expired()? || recovery_polls == super::super::RECOVERY_SETTLE_POLLS {
                            return Err(self.wait_expired(ETIMEDOUT, "GPU recovery to settle"));
                        }
                        recovery_polls += 1;
                        self.shared.queue_events();
                        fsleep(Delta::from_millis(1));
                    } else if error == EBUSY {
                        self.wait_render_busy(slot, wait)?;
                    } else if wait.expired()? {
                        return Err(self.wait_expired(ETIMEDOUT, "render preparation"));
                    }
                }
                // Unpublished: the preparation was discarded and the pair returned.
                Err(error) if !packet.is_published() && error == ENOSPC => {
                    self.wait_render_capacity(wait, &mut capacity_wait)?;
                }
                result => return result,
            }
        }
    }

    pub(in crate::g17) fn try_render_early(&self, packet: &Arc<Packet>) -> bool {
        if self.backlog.load(Ordering::Acquire) != 0 || packet.has_dependencies() {
            return false;
        }
        let admission = {
            let state = self.shared.state.lock();
            let Some(firmware) = (*state).as_deref() else {
                return false;
            };
            match firmware
                .queues
                .admission
                .try_acquire_render(self.owner, Request::of(packet))
            {
                Ok(Some(lease)) => lease,
                _ => return false,
            }
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
        let result = self.render_with_lease(packet, Some(admission), Wait::Never, &mut deferred);
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
    /// The state of pair `slot` that refuses a publication of `owner` with
    /// EBUSY, for the report of a busy wait that reached its deadline. Checks
    /// follow the order of the refusals in construction and preparation.
    fn render_busy_cause(
        &self,
        slot: u8,
        owner: u64,
        context: &Arc<crate::g17::context::Context>,
    ) -> &'static str {
        use crate::g17::fw::queue::DataMaster;
        let Ok(entry) = self.queues.render.entry(slot) else {
            return "no queue identities";
        };
        if entry.pending_error.is_some() || entry.pending_timeout {
            return "a deferred failure or timeout awaits the event worker";
        }
        if entry.constructing {
            return "its graph is being constructed";
        }
        if entry.growing {
            return "its parameter buffer is growing";
        }
        if entry.preparing.is_some() {
            return if entry.pending_owner.is_some_and(|other| other != owner) {
                "another owner is preparing it"
            } else {
                "a preparation has borrowed it"
            };
        }
        let Some(pair) = entry.pair.as_ref() else {
            return if entry.parked.is_some() {
                "parked"
            } else {
                "no installed pair"
            };
        };
        if pair.matches(owner, context).unwrap_or(false) {
            let priority = context.policy().priority();
            return if pair
                .owner_switch_from(context)
                .is_some_and(|scheduler| self.queues.teardown.releasing_scheduler(scheduler))
            {
                "the release of the scheduler it last ran is not consumed yet"
            } else if !self.init.work_ready(priority, DataMaster::Fragment).unwrap_or(false)
                || !self.init.work_ready(priority, DataMaster::Tiling).unwrap_or(false)
            {
                "the outer work ring is full"
            } else if !pair.room() {
                "its command ring is full"
            } else {
                "installed for this owner"
            };
        }
        if self.queues.teardown.releasing(pair.context()) {
            "the release of its previous owner's scheduler is in progress"
        } else if pair.owner().is_some() {
            "another owner holds it"
        } else if !pair.reusable().unwrap_or(false) {
            "its previous owner's work or displaced graph has not settled"
        } else {
            "reusable"
        }
    }

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
        // Both outer destinations must have room before pool accounting or
        // either inner queue becomes visible. All host producers share this lock.
        let priority = packet.context.policy().priority();
        if !self.init.work_ready(priority, crate::g17::fw::queue::DataMaster::Fragment)?
            || !self.init.work_ready(priority, crate::g17::fw::queue::DataMaster::Tiling)?
        {
            return Err(EBUSY);
        }
        let dependencies = self.queues.render_dependencies(owner, packet, state)?;
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
