// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Event-driven render snapshots outside the device mutex. The preparation
//! epoch and exact pair owner survive every read, cancellation and retry.

use super::super::DeferredBatch;
use crate::g17::queue::render::Pair;
use core::sync::atomic::Ordering;
use kernel::{prelude::*, sync::Arc};

impl crate::g17::Firmware {
    fn take_render_observation(&mut self, start: usize) -> Result<Option<(usize, KBox<Pair>)>> {
        if self.recovery.pending() {
            return Ok(None);
        }
        let state = crate::g17::recovery::Memory::recovery_state(&self.init)?;
        for slot in start..self.queues.render.entries.len() {
            let Some(entry) = self.queues.render.entries[slot].as_mut() else {
                continue;
            };
            if entry.preparing.is_some() || entry.growing {
                continue;
            }
            let Some(pair) = entry.pair.as_ref() else {
                continue;
            };
            let qids = pair.qids();
            if !pair.published_in_flight()
                || !qids
                    .into_iter()
                    .all(|qid| entry.event_masks[usize::from(qid) / 64] & (1u64 << (qid % 64)) != 0)
            {
                continue;
            }
            let stamps = [
                self.init.stamp(qids[0])?.load(Ordering::Relaxed),
                self.init.stamp(qids[1])?.load(Ordering::Relaxed),
            ];
            core::sync::atomic::fence(Ordering::Acquire);
            let Some(packet) = pair.observation_packet(entry.event_masks, stamps, state)? else {
                continue;
            };
            entry.borrowed_active = pair.published_in_flight();
            entry.borrowed_dependencies = Some(pair.dependency_snapshot());
            entry.borrowed_switch = pair.switch_in_flight();
            entry.preparing = Some(packet);
            return Ok(Some((slot, entry.pair.take().ok_or(EIO)?)));
        }
        Ok(None)
    }

    fn return_render_observation(&mut self, slot: usize, pair: &mut Option<KBox<Pair>>) -> Result {
        let returning = pair.as_ref().ok_or(EINVAL)?;
        let entry = self.queues.render.entry_mut(slot as u8)?;
        if entry.pair.is_some()
            || entry.preparing.is_none()
            || returning.slot() as usize != slot
            || returning.qids() != entry.reservation.ids.map(crate::g17::kick::Id::qid)
        {
            return Err(EIO);
        }
        entry.pair = pair.take();
        entry.borrowed_dependencies = None;
        entry.borrowed_switch = None;
        entry.preparing = None;
        Ok(())
    }

    pub(super) fn settle_render_terminal(
        &mut self,
        slot: usize,
        deferred: &mut DeferredBatch,
    ) -> Result {
        let terminal = self.queues.render.entries[slot]
            .as_mut()
            .and_then(|entry| entry.pair.as_mut())
            .and_then(|pair| pair.take_terminal());
        if let Some((status, error)) = terminal {
            status.record(error);
            self.scan_compute(None, deferred)?;
            self.settle_render_vm(&status, error, deferred)?;
            self.queues.fail_compute_vm(&status, error, deferred)?;
        }
        Ok(())
    }
}

impl crate::g17::Shared {
    pub(in crate::g17) fn observe_renders(
        self: &Arc<Self>,
        masks: [u64; 2],
        deferred: &mut DeferredBatch,
    ) -> Result {
        {
            let mut state = self.state.lock();
            let Some(firmware) = (*state).as_deref_mut() else {
                return Ok(());
            };
            firmware.queues.retain_render_completions(masks);
            // Returned preparers can have deferred a VM failure or timeout.
            // Only this full worker collector runs their final observations.
            for slot in 1..firmware.queues.render.entries.len() {
                if firmware.queues.render.entries[slot]
                    .as_ref()
                    .is_some_and(|entry| {
                        entry.pair.is_some()
                            && (entry.pending_error.is_some()
                                || entry.pending_timeout
                                || entry.closed)
                    })
                {
                    firmware.apply_render_deferred(slot as u8, deferred)?;
                }
            }
        }
        deferred.finish();
        let mut cursor = 1;
        let mut remaining = crate::hw::t8140::queues::RENDER_DEPTH;
        let mut retries = 0;
        loop {
            let epoch = match self.preparations.try_enter() {
                Err(error) if error == EAGAIN || error == ENODEV => break,
                result => result?,
            };
            let loan = {
                let mut state = self.state.lock();
                let Some(firmware) = (*state).as_deref_mut() else {
                    return Ok(());
                };
                if !epoch.is_current() || self.crashed.load(Ordering::Acquire) {
                    break;
                }
                firmware.take_render_observation(cursor)?
            };
            let Some((slot, pair)) = loan else {
                break;
            };
            if slot != cursor {
                remaining = crate::hw::t8140::queues::RENDER_DEPTH;
                retries = 0;
            }
            let observed = pair.observe_front();
            let mut pair = Some(pair);
            let mut state = self.state.lock();
            // Return installed ownership before examining read errors or epoch.
            let restored = (*state)
                .as_deref_mut()
                .ok_or(ENODEV)
                .and_then(|firmware| firmware.return_render_observation(slot, &mut pair));
            if let Err(error) = restored {
                // A failed invariant must not drop firmware-owned storage. Join
                // the other preparers only after releasing our own epoch, and
                // keep this detached owner until both processors have stopped.
                drop(state);
                drop(epoch);
                self.lose_device_quietly();
                self.stop_queues();
                if let Some(mut pair) = pair {
                    pair.stopped(&mut |event| {
                        event.finish();
                        Ok(())
                    })?;
                }
                return Err(error);
            }
            let firmware = (*state).as_deref_mut().ok_or(EIO)?;
            self.changed.notify_all();
            if firmware.render_grow_pending() || firmware.queues.render.take_missed_event(slot as u8) {
                self.queue_events();
            }
            if !epoch.is_current()
                || firmware.recovery.pending()
                || self.crashed.load(Ordering::Acquire)
            {
                break;
            }
            let result = (|| {
                let state = crate::g17::recovery::Memory::recovery_state(&firmware.init)?;
                let progressed = match observed {
                    Ok(observed) => firmware.with_render(slot, |pair, host| {
                        pair.retire_observed(observed, state, host, &mut |event| {
                            deferred.push(event)
                        })
                    })?,
                    Err(error) if error == EAGAIN => false,
                    Err(error) => return Err(error),
                };
                if progressed {
                    match firmware.release_render_lists() {
                        Err(error) if error == EAGAIN => {}
                        result => result?,
                    }
                }
                firmware.queues.render.collect_retired(slot)?;
                // An observation of the old front is committed before a queued
                // exact timeout is permitted to advance that same tracker.
                firmware.apply_render_deferred(slot as u8, deferred)?;
                firmware.settle_render_terminal(slot, deferred)?;
                if firmware
                    .queues
                    .render
                    .entry(slot as u8)?
                    .pair
                    .as_ref()
                    .is_some_and(|pair| !pair.in_flight())
                {
                    firmware.queues.render.entry_mut(slot as u8)?.event_masks = [0; 2];
                }
                Ok(progressed)
            })();
            drop(state);
            drop(epoch);
            deferred.finish();
            if result? {
                remaining -= 1;
                retries = 0;
                cursor = if remaining == 0 { slot + 1 } else { slot };
            } else {
                retries += 1;
                cursor = if retries >= 2 { slot + 1 } else { slot };
            }
            if cursor != slot {
                remaining = crate::hw::t8140::queues::RENDER_DEPTH;
                retries = 0;
            }
        }
        {
            let state = self.state.lock();
            if (*state).as_deref().is_some_and(|firmware| {
                !firmware.recovery.pending() && firmware.queues.render_needs_rescan()
            }) {
                self.queue_poll();
            }
        }
        Ok(())
    }
}
