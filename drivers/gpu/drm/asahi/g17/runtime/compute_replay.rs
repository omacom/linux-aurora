// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Ordered republication of witnessed, unexecuted compute commands.

use super::DeferredBatch;
use crate::g17::{completion::Deferred, recovery, Firmware};
use kernel::prelude::*;

impl Firmware {
    /// Called only from an outer worker or client submission, never from event
    /// dispatch or a recovery callback nested within publication. All candidate
    /// queues are observed before any retained packet is published again.
    pub(in crate::g17) fn replay_compute(&mut self, deferred: &mut DeferredBatch) -> Result {
        if self.recovery.pending() {
            return Ok(());
        }
        let state = recovery::Memory::recovery_state(&self.init)?;
        if state != 0 {
            return Ok(());
        }
        let commands = self
            .queues
            .compute
            .iter()
            .flatten()
            .filter_map(|entry| entry.queue.as_deref())
            .map(|queue| queue.replay_commands())
            .sum::<usize>();
        if commands == 0 {
            self.queues.replay_outer_wait = None;
            return Ok(());
        }
        deferred.reserve_replays(commands)?;
        let port = &self.port;
        for entry in self.queues.compute.iter_mut().flatten() {
            if let Some(mut queue) = entry.queue.as_deref_mut() {
                queue.prepare_replays(
                    state,
                    |owner| port.qos_complete(&[owner]),
                    &mut |event| deferred.push(event),
                )?;
            }
        }
        for index in 0..self.queues.compute.len() {
            loop {
                if self.recovery.pending() || recovery::Memory::recovery_state(&self.init)? != 0 {
                    return Ok(());
                }
                let Some((packet, owner)) = self.queues.compute[index]
                    .as_ref()
                    .and_then(|entry| entry.queue.as_deref())
                    .and_then(|queue| Some((queue.replay_front()?, queue.owner())))
                else {
                    break;
                };
                let outer_ready = self.port.compute_ready(packet.context.policy().priority())?;
                if !outer_ready {
                    let now = super::now_ns();
                    let since = *self.queues.replay_outer_wait.get_or_insert(now);
                    if now.saturating_sub(since) < super::WORK_RING_WAIT_NS {
                        // Keep the FIFO and its replay credit untouched. The
                        // event worker schedules the existing delayed poll.
                        return Ok(());
                    }
                } else {
                    self.queues.replay_outer_wait = None;
                }

                let result = if !outer_ready {
                    // This replay has no live firmware ownership. Persistent
                    // backpressure uses the existing failed-replay retirement.
                    Err(EBUSY)
                } else { (|| {
                    self.queues.compute[index]
                        .as_mut()
                        .and_then(|entry| entry.queue.as_deref_mut())
                        .ok_or(EIO)?
                        .prepare_replay(&packet)?;
                    let owner = owner.ok_or(EIO)?;
                    packet.check_replay_dependencies()?;
                    let dependencies = self.queues.render_prefix(owner, &packet)?;
                    self.publish_compute(owner, packet.clone(), dependencies.as_slice(), deferred)
                })() };
                let recovery_wait = matches!(result, Err(EAGAIN) | Err(EBUSY))
                    && (self.recovery.pending()
                        || recovery::Memory::recovery_state(&self.init)? != 0);
                let mut queue = self.queues.compute[index]
                    .as_mut()
                    .and_then(|entry| entry.queue.as_deref_mut())
                    .ok_or(EIO)?;
                if let Err(error) = result {
                    if recovery_wait && !queue.owns_packet(&packet) {
                        // Keep this prepared attempt and its one timeout credit
                        // while the outer worker closes the new recovery.
                        return Ok(());
                    }
                    queue.fail_replays(error);
                    if !queue.owns_packet(&packet) {
                        deferred
                            .push(Deferred::Retired(packet.completion.clone(), Err(ENODATA)))?;
                    }
                }
                queue.finish_replay(&packet)?;
            }
        }
        self.queues.replay_outer_wait = None;
        // Publication behind these replays may proceed.
        self.primary.state.shared.progress.notify();
        Ok(())
    }
}
