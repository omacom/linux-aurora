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
            let Some(queue) = firmware.queues.compute(self.owner)
                .filter(|queue| queue.can_classify_timeout(packet)) else {
                return Ok(Decision::Own);
            };
            let Some(own) = queue.timeout_head() else {
                return Ok(Decision::Own);
            };
            let others = firmware.queues.compute.iter().flatten()
                .filter_map(|entry| entry.queue.as_deref())
                .filter(|other| other.owner() != Some(self.owner))
                .filter_map(|other| other.timeout_head());
            let decision = timeout::classify(own, renewals, others);
            if decision == Decision::Spare {
                firmware.queues.compute(self.owner).ok_or(EIO)?
                    .classify(&|_| Class::Spared);
            }
            if decision != Decision::Own && (renewals == 0 || decision == Decision::Spare) {
                dev_warn!(self.shared.dev.as_ref(),
                    "Compute watchdog owner={} publication={} renewals={} decision={:?} behind earlier work of another VM\n",
                    self.owner, own.published, renewals, decision);
            }
            Ok(decision)
        })();
        deferred.finish();
        matches!(result, Ok(Decision::Defer))
    }

    /// The caller owns cancellation of a packet that never claimed publication.
    /// A missing retained packet may already have a healthy callback waiting off-lock.
    pub(super) fn timeout_compute(&self, packet: &Arc<Packet>) -> Result {
        // Unpublished packets wait in publication, which bounds them. Without a
        // collector the watchdog expires again and retries.
        if !packet.is_published() {
            return Ok(());
        }
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
            let Some(queue) = firmware.queues.compute_mut(self.owner) else {
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
            if let Some(queue) = firmware.queues.compute_mut(self.owner) {
                queue.cancel(packet, &mut |event| deferred.push(event))?;
            }
            Ok(())
        })();
        deferred.finish();
        result
    }
}
