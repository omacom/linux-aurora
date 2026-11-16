// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Scheduler callbacks and the off-lock fence boundary of a logical queue.

use super::{Backend, DeferredBatch};
use crate::g17::{
    completion::Deferred,
    job::{self, Engine, Packet},
};
use core::sync::atomic::Ordering;
use kernel::{dma_fence::Fence, drm::sched, prelude::*, sync::Arc};

impl Backend {
    pub(super) fn release_backlog(&self, packet: &Packet) {
        if packet.take_backlog() {
            self.backlog_of(packet).fetch_sub(1, Ordering::AcqRel);
        }
    }

    pub(super) fn backlog_of(&self, packet: &Packet) -> &core::sync::atomic::AtomicU32 {
        match packet.engine() {
            Engine::Render => &self.backlog,
            Engine::Compute => &self.compute_backlog,
        }
    }

    fn stop_after_error(&self) {
        // Scheduler/fence callbacks never synchronously join builders or RTKit
        // receive work. The event worker performs the bounded runtime stop.
        self.shared.lose_device_quietly();
    }

    fn rollback_unpublished_compute(&self) -> Result {
        // No outer preparation lease survives compute_packet's return. Hold a
        // fresh epoch through alias destruction so recovery can join this work.
        loop {
        let lease = self.shared.preparations.enter(false)?;
        let mut retired = None;
        let mut state = self.shared.state.lock();
        let firmware = state.as_deref_mut().ok_or(ENODEV)?;
        if !lease.is_current() { drop(state); drop(lease); continue; }
        let mut binding = None;
        if let Some(index) = firmware.queues.compute.iter().position(|entry| {
            entry.as_ref().and_then(|entry| entry.queue.as_deref())
                .is_some_and(|queue| queue.owner() == Some(self.owner))
        }) {
            let entry = firmware.queues.compute[index].as_mut().ok_or(EIO)?;
            let (cancel, replaced) = entry.queue.as_deref_mut().ok_or(EIO)?
                .rollback_unpublished(self.owner, &self.context)?;
            binding = replaced;
            if cancel {
                retired = entry.queue.take().map(|queue| (entry.reservation, queue));
            }
        }
        drop(state);
        drop(binding);
        self.finish_uninstalled_compute(retired)?;
        self.shared.channels.notify();
        self.shared.queue_events();
        return Ok(());
        }
    }

    /// Complete all failure fanout before signalling this packet's unpublished
    /// error. Visible failures keep the engine's retained owner through a witness.
    pub(super) fn finish_publication(
        &self,
        packet: &Arc<Packet>,
        result: Result,
        deferred: &mut DeferredBatch,
    ) -> Result<Fence> {
        if let Err(error) = result {
            if [EIO, ETIMEDOUT, ENODEV].contains(&error) || packet.completion.status().get() != 0 {
                <Self as job::Backend>::fail_vm(self, error);
            }
            deferred.finish();
            if !packet.is_published() {
                if packet.engine() == Engine::Compute {
                    if self.rollback_unpublished_compute().is_err() {
                        self.stop_after_error();
                    }
                }
                packet.completion.fail_unpublished(error);
            }
            Err(error)
        } else {
            deferred.finish();
            Ok(packet.completion.scheduler_fence())
        }
    }

    fn release_queue(&self) -> Result {
        // A builder may own only an off-lock reservation when close arrives.
        // Record intent before scanning queues; every later install/rearm checks it.
        self.closed.store(true, Ordering::Release);
        self.hold_claim(None);
        let mut state = self.shared.state.lock();
        let Some(firmware) = (*state).as_deref_mut() else {
            return Ok(());
        };
        // Retain a published scheduler before a never-kicked physical graph is
        // removed. The teardown entry owns its root until control consumption.
        firmware
            .queues
            .teardown
            .close(&self.context, super::now_ns())?;
        firmware.queues.release_compute(self.owner)?;
        firmware.queues.release_render(self.owner)?;
        firmware.queues.admission.release_owner(self.owner);
        drop(state);
        // Its idle channel, or its slot and never-published QID, is free.
        self.shared.channels.notify();
        self.shared.queue_events();
        Ok(())
    }
}

impl job::Backend for Backend {
    fn input_sources(&self, inputs: &[crate::file::SyncItem])
        -> Option<Arc<crate::g17::sync_input::Sources>>
    {
        use crate::g17::{fence, sync_input};
        let mut heads = KVec::new();
        let mut sources: KVec<sync_input::Source> = KVec::new();
        let mut publication_waits = KVec::new();
        for input in inputs.iter().filter_map(|input| input.fence.as_ref()) {
            let status = job::fence_status(input);
            if status > 0 { continue; }
            if status < 0 { return None; }
            let leaves = sync_input::collect(input)?;
            for leaf in &leaves {
                if leaf.status > 0 { continue; }
                if leaf.status < 0 { return None; }
                // SAFETY: Collection owns each leaf until this lookup finishes.
                let submission = unsafe { fence::exported_source_raw(leaf.fence) }?;
                if submission.source_status().ok()? { continue; }
                if !submission.same_vm(self.context.status()) { return None; }
                if sources.iter().any(|known| Arc::ptr_eq(&known.submission, &submission)) {
                    continue;
                }
                if sources.len() == crate::g17::fw::kick::KICK_BARRIERS_MAX { return None; }
                let published = {
                    let state = self.shared.state.lock();
                    let firmware = state.as_deref()?;
                    if self.shared.crashed.load(Ordering::Acquire) || firmware.recovery.pending()
                        || crate::g17::recovery::Memory::recovery_state(&firmware.init).ok()? != 0
                    { return None; }
                    firmware.queues.compute.iter().flatten()
                        .filter_map(|entry| entry.queue.as_deref())
                        .any(|queue| queue.has_input_source(&submission, self.context.status(),
                            firmware.recovery.generation()))
                };
                if !published {
                    if submission.source_status().ok()? { continue; }
                    // The existing scheduler fence becomes ready after actual
                    // publication, not GPU completion. No binding loan or
                    // consumer hardware capacity is acquired while waiting.
                    publication_waits.push(submission.ready()?, GFP_KERNEL).ok()?;
                }
                sources.push(sync_input::Source { submission }, GFP_KERNEL).ok()?;
            }
            heads.push(input.clone(), GFP_KERNEL).ok()?;
        }
        if sources.is_empty() { return None; }
        sync_input::Sources::new(heads, sources, publication_waits).ok()
    }

    fn feed(&self) -> Option<Arc<crate::g17::feed::Feed>> {
        Some(self.shared.feed.clone())
    }

    fn render_scs_blocks(&self, pages: [u16; 2]) -> Result<usize> {
        crate::g17::fw::render_rt::required_blocks(self.shared.active_cores, pages)
    }

    fn ensure_compute(&self, scs_pages: u16) -> Result {
        // A reverse timeline chain must be submitted before the host signal.
        // Validate here, but do not consume physical capacity behind unsignalled
        // DRM dependencies. compute_packet admits it when the job can run.
        crate::g17::fw::compute_pool::required_blocks(self.shared.active_cores, scs_pages)
            .map(|_| ())
    }

    fn fail_vm(&self, error: Error) {
        self.context.status().record(error);
        let mut deferred = match DeferredBatch::worker() {
            Ok(batch) => batch,
            Err(_) => {
                self.stop_after_error();
                return;
            }
        };
        let result = (|| {
            let mut state = self.shared.state.lock();
            let Some(firmware) = (*state).as_deref_mut() else {
                return Ok(());
            };
            // Final observations win over this VM's terminal fanout.
            firmware.scan_compute(None, &mut deferred)?;
            firmware.settle_render_vm(self.context.status(), error, &mut deferred)?;
            firmware
                .queues
                .fail_compute_vm(self.context.status(), error, &mut deferred)
        })();
        deferred.finish();
        if result.is_err() {
            self.stop_after_error();
        }
    }

    fn release_owner(&self) {
        if self.release_queue().is_err() {
            self.stop_after_error();
        }
    }

    /// Firmware kick parents order the same engine. Explicit cross-engine and
    /// input dependencies were already attached to the DRM scheduled job.
    /// Compute and render admission wait for their shared capacity here, as a
    /// fence.
    fn prepare(&self, packet: &Arc<Packet>) -> Option<Fence> {
        match packet.engine() {
            Engine::Compute => self.prepare_compute(packet),
            Engine::Render => self.admit_render(packet),
        }
    }

    fn publish(&self, packet: Arc<Packet>) -> Result<Fence> {
        if let Some(result) = packet.early_result() {
            return result.map(|()| packet.completion.scheduler_fence());
        }
        let mut deferred = match DeferredBatch::publication() {
            Ok(batch) => batch,
            Err(error) => {
                self.release_backlog(&packet);
                if packet.completion.status().get() != 0 {
                    <Self as job::Backend>::fail_vm(self, error);
                }
                packet.completion.fail_unpublished(error);
                return Err(error);
            }
        };
        let result = if !packet.begin_publication() {
            Err(ECANCELED)
        } else if packet.completion.status().get() != 0 {
            Err(EIO)
        } else {
            match packet.engine() {
                Engine::Compute => self.compute_packet(&packet, &mut deferred),
                Engine::Render => self.render_packet(&packet, &mut deferred),
            }
        };
        if packet.engine() == Engine::Compute {
            self.hold_claim(None);
        }
        self.release_backlog(&packet);
        self.finish_publication(&packet, result, &mut deferred)
    }

    fn timed_out(&self, packet: Arc<Packet>) -> sched::Status {
        // A packet still in publication, for example waiting for capacity or
        // for its render pair's graph, has nothing visible to firmware and may
        // not even have an installed pair. Its publisher owns its bounded wait
        // and result, and it keeps its place in the backlog so that a later
        // dependency-free command is not published from its ioctl ahead of it.
        // It is not a hang and does not count towards the timeout storm limit.
        if !packet.is_published() {
            return sched::Status::NoHang;
        }
        self.release_backlog(&packet);
        if self.shared.watchdog_timeout() { return sched::Status::NoHang; }
        let result = match packet.engine() {
            Engine::Compute => self.timeout_compute(&packet),
            Engine::Render => self.timeout_render(&packet),
        };
        if result.is_err() {
            self.stop_after_error();
        }
        sched::Status::NoHang
    }

    fn cancel(&self, packet: Arc<Packet>) {
        self.release_backlog(&packet);
        if packet.engine() == Engine::Compute {
            self.hold_claim(None);
        }
        if packet.request_cancel() {
            Deferred::Unpublished(packet.completion.clone(), ECANCELED).finish();
            return;
        }
        let result = match packet.engine() {
            Engine::Compute => self.cancel_compute(&packet),
            Engine::Render => self.cancel_render(&packet),
        };
        if result.is_err() {
            self.stop_after_error();
        }
    }

    fn long_job_beside(&self, _packet: &Arc<Packet>) -> bool {
        let state = self.shared.state.lock();
        let Some(firmware) = (*state).as_deref() else {
            return false;
        };
        firmware.queues.compute.iter().flatten()
            .filter_map(|entry| entry.queue.as_deref())
            .any(|other| other.owner() != Some(self.owner) && other.long_job_running())
    }

    fn defer_timeout(&self, packet: &Arc<Packet>, renewals: u8) -> bool {
        if packet.engine() == Engine::Compute {
            return self.defer_compute_timeout(packet, renewals);
        }
        use crate::g17::timeout::{self, Decision};
        // As for compute, a final retirement pass precedes the attribution, and all completion
        // callbacks run after releasing the device mutex. Without a collector the attribution
        // goes ahead without that pass, as it did before it existed.
        let mut deferred = DeferredBatch::worker().ok();
        let result = (|| -> Result<bool> {
            let mut state = self.shared.state.lock();
            let Some(firmware) = (*state).as_deref_mut() else {
                return Ok(false);
            };
            if let Some(deferred) = deferred.as_mut() {
                firmware.scan_renders(deferred)?;
            }
            // Borrowed-graph and queued-retirement deferrals share the renewal bound.
            if timeout::transaction_grace(renewals)
                && firmware.queues.defer_render_timeout(packet, ETIMEDOUT)
            {
                return Ok(true);
            }
            let verdict = firmware.queues.render_timeout_decision(packet, renewals);
            let decision = verdict.map_or(Decision::Own, |verdict| verdict.decision);
            // As compute does: work still blocked behind another VM's older unfinished work
            // after the whole grace is innocent, so its failure must stay non-sticky. The pair's
            // quarantine extends the class to the pair's other groups.
            if decision == Decision::Spare {
                packet.completion.classify(crate::g17::recovery::Class::Spared);
            }
            if let Some(verdict) = verdict {
                if let Some(blocker) = verdict.reported_blocker(renewals) {
                    dev_warn!(self.shared.dev.as_ref(),
                        "Render watchdog owner={} vm={} publication={} renewals={} decision={:?} behind earlier work of another VM: blocker_vm={} blocker_publication={} blocker_failed={}\n",
                        self.owner, verdict.own.vm, verdict.own.published, renewals, decision,
                        blocker.vm, blocker.published, u8::from(blocker.failed));
                }
            }
            Ok(decision == Decision::Defer)
        })();
        if let Some(deferred) = deferred.as_mut() {
            deferred.finish();
        }
        matches!(result, Ok(true))
    }

    fn try_early(&self, packet: &Arc<Packet>) -> bool {
        match packet.engine() {
            Engine::Render => self.try_render_early(packet),
            Engine::Compute => self.try_compute_early(packet),
        }
    }

    fn count_backlog(&self, packet: &Packet) {
        packet.count_backlog();
        self.backlog_of(packet).fetch_add(1, Ordering::AcqRel);
    }
}
