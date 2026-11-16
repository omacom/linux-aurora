// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Scheduler cancellation and final retirement checks for retained compute work.

use super::{Backend, DeferredBatch};
use crate::g17::job::{self, Packet};
use kernel::{prelude::*, sync::Arc};

impl Backend {
    /// A final retirement pass precedes bounded cross-VM timeout attribution.
    /// All completion callbacks run after releasing the device mutex.
    pub(super) fn defer_compute_timeout(&self, packet: &Arc<Packet>, renewals: u8) -> bool {
        use crate::g17::{recovery::Class, timeout::{self, Decision}};
        let Ok(mut deferred) = DeferredBatch::worker() else {
            return false;
        };
        let result = (|| -> Result<Decision> {
            let mut state = self.shared.state.lock();
            let Some(firmware) = (*state).as_deref_mut() else {
                return Ok(Decision::Own);
            };
            firmware.scan_compute(None, &mut deferred)?;
            // Each queue is locked on its own: release this one before visiting the others.
            let Some(own) = firmware.queues.compute(self.owner)
                .filter(|queue| queue.can_classify_timeout(packet))
                .and_then(|queue| queue.timeout_head()) else {
                return Ok(Decision::Own);
            };
            let computes = firmware.queues.compute.iter().flatten()
                .filter_map(|entry| entry.queue.as_deref())
                .filter(|other| other.owner() != Some(self.owner))
                .filter_map(|other| other.timeout_head());
            let others = Iterator::chain(computes, firmware.queues.render_timeout_heads());
            let verdict = timeout::attribute(own, renewals, others);
            let decision = verdict.decision;
            if decision == Decision::Spare {
                firmware.queues.compute(self.owner).ok_or(EIO)?
                    .classify(&|_| Class::Spared);
            }
            if let Some(blocker) = verdict.reported_blocker(renewals) {
                dev_warn!(self.shared.dev.as_ref(),
                    "Compute watchdog owner={} vm={} publication={} renewals={} decision={:?} behind earlier work of another VM: blocker_vm={} blocker_publication={} blocker_failed={}\n",
                    self.owner, own.vm, own.published, renewals, decision,
                    blocker.vm, blocker.published, u8::from(blocker.failed));
            }
            Ok(decision)
        })();
        deferred.finish();
        matches!(result, Ok(Decision::Defer))
    }

    /// The caller owns cancellation of a packet that never claimed publication.
    /// A missing retained packet may already have a healthy callback waiting off-lock.
    pub(super) fn timeout_compute(&self, packet: &Arc<Packet>) -> Result {
        // Without a collector the watchdog expires again and retries.
        let Ok(mut deferred) = DeferredBatch::worker() else {
            return Ok(());
        };
        let result = (|| {
            let mut state = self.shared.state.lock();
            let Some(firmware) = (*state).as_deref_mut() else {
                // Teardown owns the detached queues and will settle their callbacks.
                return Ok(None);
            };
            firmware.scan_compute(None, &mut deferred)?;
            let Some(mut queue) = firmware.queues.compute_mut(self.owner) else {
                return Ok(None);
            };
            if !queue.owns_packet(packet) {
                return Ok(None);
            }
            queue.timeout(&mut |event| deferred.push(event))
        })();
        if result.as_ref().is_ok_and(|status| status.is_some()) {
            // VM status and peer settlement precede signalling this timeout. The backend
            // owns the same logical VM as the retained packet matched under the mutex.
            <Self as job::Backend>::fail_vm(self, ETIMEDOUT);
        }
        deferred.finish();
        result.map(|_| ())
    }

    /// Cancel only a scheduler waiter. Visible mappings stay pinned until their witness;
    /// cancellation neither quarantines hardware admission nor fans out a VM failure.
    pub(super) fn cancel_compute(&self, packet: &Arc<Packet>) -> Result {
        let mut deferred = DeferredBatch::worker()?;
        let result = (|| {
            let mut state = self.shared.state.lock();
            let Some(firmware) = (*state).as_deref_mut() else {
                return Ok(());
            };
            firmware.scan_compute(None, &mut deferred)?;
            if let Some(mut queue) = firmware.queues.compute_mut(self.owner) {
                queue.cancel(packet, &mut |event| deferred.push(event))?;
            }
            Ok(())
        })();
        deferred.finish();
        result
    }
}
