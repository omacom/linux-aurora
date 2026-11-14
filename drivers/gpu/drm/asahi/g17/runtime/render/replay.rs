// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Republication of render groups that firmware retired without executing.
//! The pair proves the abandonment (retired, unlinked, stamps covering it, no
//! recovery, fragment end never written) and parks the packets in order; this
//! pass prepares and publishes the same commands again under the device mutex.

use super::super::DeferredBatch;
use crate::g17::{object::Allocator, queue::render::RenderAddresses, Firmware};
use kernel::prelude::*;

impl super::RenderState {
    /// (an executed group of the owner retired in `generation` or later,
    ///  a published group of the owner is still live and not parked).
    pub(in crate::g17) fn owner_render_state(&self, owner: u64, generation: u64) -> (bool, bool) {
        let mut executed = false;
        let mut live = false;
        for pair in self.entries.iter().flatten().filter_map(|entry| entry.pair.as_ref()) {
            if pair.owner() != Some(owner) {
                continue;
            }
            executed |= pair.executed_since(generation);
            live |= pair.live_unparked();
        }
        (executed, live)
    }
    /// A parked render of the owner precedes a command waiting through `through`.
    pub(in crate::g17) fn owner_parked_through(&self, owner: u64, through: Option<u64>) -> bool {
        self.entries
            .iter()
            .flatten()
            .filter_map(|entry| entry.pair.as_ref())
            .any(|pair| pair.owner() == Some(owner) && pair.replay_through(through))
    }

    pub(in crate::g17) fn replay_polling(&self) -> bool {
        self.entries
            .iter()
            .flatten()
            .filter_map(|entry| entry.pair.as_ref())
            .any(|pair| pair.replay_polling())
    }
}

impl Firmware {
    pub(in crate::g17) fn replay_render(&mut self, deferred: &mut DeferredBatch) -> Result {
        if self.recovery.pending()
            || crate::g17::recovery::Memory::recovery_state(&self.init)? != 0
        {
            return Ok(());
        }
        for slot in 0..self.queues.render.entries.len() {
            // Publish the parked groups front first; a pause keeps the rest parked.
            while self.replay_render_front(slot, deferred)? {}
        }
        Ok(())
    }

    /// Publishes the oldest parked group of `slot`; returns whether it was published.
    fn replay_render_front(&mut self, slot: usize, deferred: &mut DeferredBatch) -> Result<bool> {
        let Some((packet, owner, qids, closed, busy, stalled)) =
            self.queues.render.entries[slot].as_ref().and_then(|entry| {
                let pair = entry.pair.as_ref()?;
                let packet = pair.replay_front()?;
                Some((
                    packet,
                    entry.reservation.owner,
                    pair.qids(),
                    entry.closed,
                    entry.preparing.is_some()
                        || entry.growing
                        || entry.pending_timeout
                        || entry.pending_error.is_some(),
                    pair.replay_stalled(crate::g17::now_ns()),
                ))
            })
        else {
            return Ok(false);
        };
        if stalled {
            return Err(ETIMEDOUT);
        }
        if busy {
            return Ok(false);
        }
        if closed || packet.cancel_requested() || !packet.context.is_current() {
            self.with_render(slot, |pair, _| {
                pair.fail_replay(ECANCELED, &mut |event| deferred.push(event))
            })?;
            return Ok(false);
        }
        if packet.completion.status().get() != 0 {
            self.with_render(slot, |pair, _| {
                pair.fail_replay(EIO, &mut |event| deferred.push(event))
            })?;
            return Ok(false);
        }
        let addresses = RenderAddresses {
            stamps: [
                self.init.stamp_addresses(qids[0])?,
                self.init.stamp_addresses(qids[1])?,
            ],
        };
        let dev = self.primary.state.shared.drm()?;
        let uat = self.uat.clone();
        let prepared = self.with_render(slot, |pair, _| {
            if !pair.room() {
                return Err(EBUSY);
            }
            let alloc = Allocator {
                dev: &dev,
                uat: &uat,
            };
            pair.prepare(&alloc, &packet, addresses)
        });
        match prepared {
            Ok(()) => {}
            Err(error) if error == EBUSY || error == EAGAIN => return Ok(false),
            Err(error) => {
                self.with_render(slot, |pair, _| {
                    pair.fail_replay(error, &mut |event| deferred.push(event))
                })?;
                return Ok(false);
            }
        }
        match self.publish_render(owner, slot as u8, &packet, deferred) {
            Ok(()) => {
                self.with_render(slot, |pair, _| pair.finish_replay(&packet))?;
                Ok(true)
            }
            Err(error) if error == EBUSY || error == EAGAIN => Ok(false),
            Err(error) => {
                self.with_render(slot, |pair, _| {
                    pair.fail_replay(error, &mut |event| deferred.push(event))
                })?;
                Ok(false)
            }
        }
    }
}
