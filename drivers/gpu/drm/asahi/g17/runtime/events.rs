// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Event and restart callbacks borrow the complete firmware owner. The reusable
//! completion collector belongs to the worker and is drained outside its mutex.

use super::DeferredBatch;
use crate::g17::{
    channel, event,
    fw::channels::{Event, FreeListCompletion, TvbGrowRequest},
    recovery,
    status::VmStatus,
    Firmware, Role,
};
use kernel::{prelude::*, sync::Arc};

/// Wake interval while a tiler heap may still shrink: the native check period.
const SHRINK_WAKE_NS: u64 = 1_000_000_000;
/// Longest interval between two event passes that run every housekeeping step while
/// completion records alone keep the worker busy.
const HOUSEKEEPING_NS: u64 = 1_000_000;

struct Service<'a> {
    firmware: &'a mut Firmware,
    deferred: &'a mut DeferredBatch,
    render_masks: [u64; 2],
    settlement_error: Option<Error>,
}

impl event::Transport for Service<'_> {
    fn roles(&self) -> &'static [Role] {
        self.firmware.roles()
    }
    fn trace(&mut self, role: Role) {
        let firmware = &mut *self.firmware;
        firmware
            .rings
            .drain_trace(&firmware.primary.state.shared.dev, &firmware.init, role);
    }
    fn statistics(&mut self, role: Role) {
        let firmware = &mut *self.firmware;
        firmware.observe_statistics_power();
        firmware
            .rings
            .drain_statistics(&firmware.primary.state.shared.dev, &firmware.init, role);
        firmware.observe_statistics_power();
    }
    fn drain(
        &mut self,
        role: Role,
        mut consume: impl FnMut(&mut Self, Event) -> Result<bool>,
    ) -> Result {
        let mut events =
            channel::Events::new(role, self.firmware.primary.state.shared.variant.graph);
        while let Some(event) =
            events.next(&self.firmware.primary.state.shared.dev, &self.firmware.init)?
        {
            if !consume(self, event)? {
                break;
            }
        }
        Ok(())
    }
    fn recovery_state(&self) -> Result<u32> {
        recovery::Memory::recovery_state(&self.firmware.init)
    }
}

impl event::Host for Service<'_> {
    fn recovery(&self) -> &recovery::State {
        &self.firmware.recovery
    }
    fn recovery_mut(&mut self) -> &mut recovery::State {
        &mut self.firmware.recovery
    }
}

impl event::Queues for Service<'_> {
    fn context_killed(&mut self, cookie: u64) {
        self.firmware.queues.acknowledge_compute_exit(cookie);
    }
    fn compute_fault(&mut self, qid: u8, stamp: u32) {
        for entry in self.firmware.queues.compute.iter_mut().flatten() {
            if let Some(mut queue) = entry.queue.as_deref_mut() {
                if queue.qid() == qid {
                    queue.note_fault(stamp);
                }
            }
        }
    }
    fn flush_tvb_reply(&mut self) -> Result<bool> {
        self.firmware.flush_tvb_reply()
    }
    fn flush_freelist_release(&mut self) -> Result<bool> {
        self.firmware.flush_freelist_release()
    }
    fn observe_render_releases(&mut self) -> Result {
        self.firmware.observe_render_releases()
    }
    fn control_backpressured(&self) -> bool {
        self.firmware.render_control_backpressured()
    }
    fn queue_tvb_grow(&mut self, request: TvbGrowRequest) -> Result {
        self.firmware.queue_tvb_grow(request)
    }
    fn complete_freelist_grow(&mut self, completion: FreeListCompletion) -> Result {
        self.firmware.complete_freelist_grow(completion)
    }
    fn retain_render_completions(&mut self, masks: [u64; 2]) {
        // Inline timeout and drain callers may discard the worker hints. The
        // queue owner retains stamps until its borrowed pair returns to scan.
        self.firmware.queues.retain_render_completions(masks);
        self.render_masks[0] |= masks[0];
        self.render_masks[1] |= masks[1];
    }
    fn complete_compute(&mut self, masks: [u64; 2]) -> Result {
        // A batch without a queue identity still observes completions whose
        // notification was coalesced; identified batches retain QID filtering.
        let selected = (masks != [0; 2]).then_some(masks);
        self.firmware.scan_compute(selected, self.deferred)
    }
}

impl recovery::Host for Service<'_> {
    fn recovery(&self) -> &recovery::State {
        &self.firmware.recovery
    }
    fn recovery_mut(&mut self) -> &mut recovery::State {
        &mut self.firmware.recovery
    }
}

impl recovery::Memory for Service<'_> {
    fn recovery_state(&self) -> Result<u32> {
        self.firmware.init.recovery_state()
    }
    fn host_recovery(&self) -> Result<u32> {
        self.firmware.init.host_recovery()
    }
    fn fault_sources(&self) -> Result<recovery::Sources> {
        self.firmware.init.fault_sources()
    }

    fn transition(&self, expected: u32, next: u32) -> Result {
        self.firmware.init.transition(expected, next)
    }
    fn clear_timestamps(&self) -> Result {
        self.firmware.init.clear_timestamps()
    }
    fn entry(&self, index: usize) -> Result<(u32, u32)> {
        self.firmware.init.entry(index)
    }
    fn wait_tick(&self) {
        self.firmware.init.wait_tick();
    }
}

impl recovery::Queues for Service<'_> {
    fn render_pass_started(&self, qid: u8, stamp: u64) -> Option<bool> {
        self.firmware.queues.render_pass_started(qid, stamp)
    }
    fn publish_recovery_targets(&self) -> Option<Error> {
        let mut failure = self.firmware.queues.publish_render_recovery_targets();
        for queue in self
            .firmware
            .queues
            .compute
            .iter()
            .flatten()
            .filter_map(|entry| entry.queue.as_deref())
        {
            if let Err(error) = queue.publish_recovery_target() {
                failure = failure.or(Some(error));
            }
        }
        core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);
        failure
    }
    fn render_scheduler_active(&self) -> bool {
        self.firmware.queues.render_scheduler_active() || !self.firmware.queues.teardown.is_empty()
    }
    fn oldest_spared(&self, qid: u8) -> Option<bool> {
        self.firmware.queues.render_oldest_spared(qid).or_else(|| {
            self.firmware
                .queues
                .compute
                .iter()
                .flatten()
                .filter_map(|entry| entry.queue.as_deref())
                .find(|queue| queue.qid() == qid)
                .and_then(|queue| queue.oldest_spared())
        })
    }
    fn visit_owners(&self, qid: u8, visit: &mut dyn FnMut(&Arc<VmStatus>)) {
        self.firmware.queues.visit_render_owners(qid, visit);
        for queue in self
            .firmware
            .queues
            .compute
            .iter()
            .flatten()
            .filter_map(|entry| entry.queue.as_deref())
        {
            if queue.qid() == qid {
                queue.visit_owners(visit);
            }
        }
    }
    fn classify_work(&mut self, class: &dyn Fn(&Arc<VmStatus>) -> recovery::Class) -> u128 {
        for queue in self
            .firmware
            .queues
            .compute
            .iter()
            .flatten()
            .filter_map(|entry| entry.queue.as_deref())
        {
            queue.classify(class);
        }
        self.firmware.queues.classify_render(class)
    }
    fn note_interrupted(&mut self, qid: u8, stamp: u32) -> recovery::Interrupted {
        let render = self.firmware.queues.note_render_interrupted(qid, stamp);
        if render != recovery::Interrupted::None {
            return render;
        }
        self.firmware
            .queues
            .compute
            .iter()
            .flatten()
            .filter_map(|entry| entry.queue.as_deref())
            .find(|queue| queue.qid() == qid)
            .map_or(recovery::Interrupted::None, |queue| {
                queue.note_interrupted(stamp)
            })
    }
    fn report_blamed_render(&self, qid: u8) {
        self.firmware
            .queues
            .report_render_recovery(qid, &self.firmware.port);
    }
    fn settle_blamed_render(&mut self, qid: u8, error: Error) {
        let result = self
            .firmware
            .settle_blamed_render(qid, error, self.deferred);
        self.settlement_error = self.settlement_error.or(result.err());
    }
    fn settle_cancelled_renders(&mut self, qids: u128) {
        let result = self.firmware.settle_cancelled_renders(qids, self.deferred);
        self.settlement_error = self.settlement_error.or(result.err());
    }
    fn rescan_compute(&mut self) -> Result {
        self.firmware.scan_compute(None, self.deferred)
    }
    fn release_retired_compute(&mut self) {
        let result = self.firmware.release_compute_witnesses(self.deferred);
        self.settlement_error = self.settlement_error.or(result.err());
    }
}

impl Firmware {
    fn release_compute_witnesses(&mut self, deferred: &mut DeferredBatch) -> Result {
        let state = recovery::Memory::recovery_state(&self.init)?;
        let port = &self.port;
        let registry = &mut self.queues;
        for mut queue in registry
            .compute
            .iter_mut()
            .flatten()
            .filter_map(|entry| entry.queue.as_deref_mut())
        {
            queue.release_witnessed(state, |owner| port.qos_complete(&[owner]), &mut |event| {
                deferred.push(event)
            })?;
        }
        Ok(())
    }

    pub(super) fn service_runtime_events(
        &mut self,
        deferred: &mut DeferredBatch,
    ) -> Result<(event::Pending, [u64; 2])> {
        let mut service = Service {
            firmware: self,
            deferred,
            render_masks: [0; 2],
            settlement_error: None,
        };
        let pending = event::service(&mut service)?;
        if pending.recovery {
            // Inline timeout/drain service can consume the restart record too.
            // Close publication in this same critical section and ensure the
            // event worker will join builders before the restart handshake.
            let shared = &service.firmware.primary.state.shared;
            shared.preparations.close();
            shared.queue_events();
        }
        Ok((pending, service.render_masks))
    }

    /// Preparation admission is already closed and all builders have joined.
    pub(super) fn service_runtime_recovery(&mut self, deferred: &mut DeferredBatch) -> Result {
        self.rings.invalidate_statistics();
        if self.recovery.restart_limit(super::now_ns()) {
            pr_warn!("G17: restart storm limit reached\n");
            return Err(ETIMEDOUT);
        }
        let shared = self.primary.state.shared.clone();
        let mut service = Service {
            firmware: self,
            deferred,
            render_masks: [0; 2],
            settlement_error: None,
        };
        let result = recovery::service(&mut service, || {
            shared.crashed.load(core::sync::atomic::Ordering::Acquire)
        });
        // Publishers without the device mutex stamp their tickets with this
        // mirror; they rejoin only after preparation admission reopens.
        service
            .firmware
            .port
            .set_generation(service.firmware.recovery.generation());
        service.firmware.rings.invalidate_statistics();
        let result = result?;
        if let Some(error) = result.target_error {
            dev_warn!(
                shared.dev.as_ref(),
                "Recovery target copy failed: {:?}\n",
                error
            );
        }
        if let Some(error) = result.scan_error {
            dev_warn!(
                shared.dev.as_ref(),
                "Post-recovery compute scan failed: {:?}\n",
                error
            );
        }
        if let Some(error) = service.settlement_error {
            return Err(error);
        }
        Ok(())
    }
}

impl crate::g17::Shared {
    /// The single event work item owns this preallocated collector until all
    /// callbacks are finished. Shutdown can detach Firmware during an off-lock
    /// boundary; the collector itself still owns every deferred completion.
    pub(in crate::g17) fn run_runtime_events(self: &Arc<Self>) {
        if self.crashed.load(core::sync::atomic::Ordering::Acquire) {
            self.stop_runtime();
            return;
        }
        let mut deferred = {
            let mut state = self.state.lock();
            let Some(firmware) = (*state).as_deref_mut() else {
                return;
            };
            let Some(deferred) = firmware.queues.worker_deferred.take() else {
                drop(state);
                self.lose_device_quietly();
                return;
            };
            deferred
        };
        crate::g17::trace::event(0);
        let result = self.runtime_event_turn(&mut deferred);
        deferred.finish();
        crate::g17::trace::event(2);
        self.queue_feed();
        if let Some(firmware) = (*self.state.lock()).as_deref_mut() {
            firmware.queues.worker_deferred = Some(deferred);
        }
        if let Err(error) = result {
            dev_err!(self.dev.as_ref(), "GPU event service failed: {:?}\n", error);
            self.lose_device_quietly();
            self.stop_runtime();
        }
    }

    fn runtime_event_turn(self: &Arc<Self>, deferred: &mut DeferredBatch) -> Result {
        // Taken before consuming records: a request made during this pass gets the next one.
        let requested = self
            .housekeeping
            .swap(false, core::sync::atomic::Ordering::AcqRel);
        let (recover, masks) = {
            let mut state = self.state.lock();
            let Some(firmware) = (*state).as_deref_mut() else {
                return Ok(());
            };
            let (pending, masks) = firmware.service_runtime_events(deferred)?;
            let recover = pending.recovery || firmware.recovery.pending();
            if recover {
                // Epoch closure and event consumption are serialized with every
                // publisher. Joining builders must happen after unlocking.
                self.preparations.close();
            }
            (recover, masks)
        };
        deferred.finish();
        crate::g17::trace::event(1);
        if recover {
            self.preparations
                .wait_drained_until(super::now_ns().saturating_add(super::WORK_RING_WAIT_NS))?;
        }
        let mut state = self.state.lock();
        let Some(firmware) = (*state).as_deref_mut() else {
            return Ok(());
        };
        if self.crashed.load(core::sync::atomic::Ordering::Acquire) {
            return Err(EIO);
        }
        if recover {
            firmware.service_runtime_recovery(deferred)?;
            if !firmware.recovery.pending() {
                if !self.preparations.reopen() {
                    return Ok(());
                }
                // Restart stopped consumption before its following record.
                // Resume builders before observing pairs outside this lock.
                self.queue_events();
            }
        }
        // Replay only at this outer boundary, after recovery has settled.
        // Publication may service events but must never recurse into replay.
        firmware.replay_compute(deferred)?;
        firmware.replay_render(deferred)?;
        drop(state);
        self.observe_renders(masks, deferred)?;
        let mut state = self.state.lock();
        let Some(firmware) = (*state).as_deref_mut() else {
            return Ok(());
        };
        if self.crashed.load(core::sync::atomic::Ordering::Acquire) {
            return Err(EIO);
        }
        let now = super::now_ns();
        let compute = firmware
            .queues
            .compute
            .iter()
            .flatten()
            .filter_map(|entry| entry.queue.as_deref())
            .any(|queue| queue.in_flight());
        let in_flight = compute || firmware.queues.render_in_flight();
        let idle = firmware.idle_check(now, in_flight)?;
        // Expiries, drained pairs, exits, pool and heap trimming and teardown give back what
        // retirement freed; a completion needs none of them to retire or to signal. While work
        // is in flight and nothing waits for capacity, a pass for firmware records alone skips
        // them. Any requested pass, recovery, teardown, a waiter, the last retirement and, at
        // the latest, the first pass after HOUSEKEEPING_NS run them all.
        let housekeeping = requested
            || recover
            || !in_flight
            || firmware.recovery.pending()
            || !firmware.queues.teardown.is_empty()
            || self.channels.has_waiters()
            || self.progress.has_waiters()
            || self.queues.has_waiters()
            || self.renders.has_waiters()
            || now.saturating_sub(
                self.housekeeping_ns
                    .load(core::sync::atomic::Ordering::Relaxed),
            ) >= HOUSEKEEPING_NS;
        let mut lingers = None;
        let mut pools = None;
        let mut shrinking = false;
        if housekeeping {
            self.housekeeping_ns
                .store(now, core::sync::atomic::Ordering::Relaxed);
            if !firmware.recovery.pending() {
                lingers = firmware.expire_render_lingers(now)?;
            }
            firmware.reclaim_drained_renders(deferred)?;
            firmware.service_compute_exit(deferred)?;
            pools = firmware.service_compute_pools(deferred, now)?;
            if !firmware.recovery.pending() {
                firmware
                    .queues
                    .vacate_evicted_render_pools(&firmware.init, deferred)?;
                shrinking = firmware
                    .queues
                    .shrink_render_heaps(&firmware.init, now, deferred);
            }
            let mut reclaim = self.reclaim.lock();
            firmware.service_teardown(&mut reclaim)?;
            if !reclaim.is_empty() {
                self.queue_reclaims();
            }
        }
        let backpressured = firmware.render_control_backpressured();
        let polling = backpressured
            || firmware.queues.replay_outer_wait.is_some()
            || firmware.queues.render.replay_polling()
            || firmware.queues.teardown.polling(now)
            || firmware.compute_exit_polling(now)
            || firmware
                .queues
                .compute
                .iter()
                .flatten()
                .filter_map(|entry| entry.queue.as_deref())
                .any(|queue| queue.retirement_polling() || queue.pool_release_pending());
        if polling {
            self.queue_poll();
        }
        // A heap with growth to give back is re-evaluated at least once per
        // shrink period, also while the device is otherwise idle.
        let idle = if shrinking {
            Some(idle.map_or(SHRINK_WAKE_NS, |delay| delay.min(SHRINK_WAKE_NS)))
        } else {
            idle
        };
        // A retained compute pool gives its backing back when it expires, and
        // so does an idle render free-list registration.
        let idle = [pools, lingers]
            .into_iter()
            .fold(idle, |idle, expiry| match (idle, expiry) {
                (Some(delay), Some(expiry)) => Some(delay.min(expiry)),
                (delay, expiry) => delay.or(expiry),
            });
        if let Some(delay) = idle {
            self.queue_idle(delay);
        }
        if firmware.recovery.pending() {
            self.preparations.close();
            self.queue_events();
        } else if !recover
            && !polling
            && (channel::Events::pending(&firmware.init, Role::Primary)?
                || (firmware.secondary.is_some()
                    && channel::Events::pending(&firmware.init, Role::Secondary)?))
        {
            // A full bounded pass must not depend on a fresh notification for
            // records that arrived while it was consuming the same ring.
            self.queue_event_records();
        }
        let grow_ready = firmware.render_grow_ready();
        drop(state);
        if grow_ready {
            self.queue_grow();
        }
        // Firmware ring room, consumed releases and teardown steps have no
        // event of their own: recheck render publishers' conditions every pass.
        if self.renders.has_waiters() {
            self.renders.notify();
        }
        Ok(())
    }
}
