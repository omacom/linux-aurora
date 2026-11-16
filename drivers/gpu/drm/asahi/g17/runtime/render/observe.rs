// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Event-driven render snapshots. A pair's completion memory is read and the
//! pair retired in one device-mutex hold, inside the preparation epoch.

use super::super::DeferredBatch;
use core::sync::atomic::Ordering;
use kernel::{prelude::*, sync::Arc};

impl crate::g17::Firmware {
    /// The first slot from `start` whose pair has news to observe: published work in flight, the
    /// fragment queue's notification seen (`completion_noted`), and an observation the pair accepts.
    fn next_render_observation(&self, start: usize) -> Result<Option<usize>> {
        if self.recovery.pending() {
            return Ok(None);
        }
        let state = crate::g17::recovery::Memory::recovery_state(&self.init)?;
        for slot in start..self.queues.render.entries.len() {
            let Some(entry) = self.queues.render.entries[slot].as_ref() else {
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
                || !crate::g17::queue::render::completion_noted(qids, entry.event_masks)
            {
                continue;
            }
            let stamps = [
                self.init.stamp(qids[0])?.load(Ordering::Relaxed),
                self.init.stamp(qids[1])?.load(Ordering::Relaxed),
            ];
            core::sync::atomic::fence(Ordering::Acquire);
            if pair.observation_packet(entry.event_masks, stamps, state)?.is_some() {
                return Ok(Some(slot));
            }
        }
        Ok(None)
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
            // Selecting, reading and retiring the front share one hold: a
            // separate hold for the read would let publishers and the
            // retirement contend twice per observed pair.
            let mut state = self.state.lock();
            let Some(firmware) = (*state).as_deref_mut() else {
                return Ok(());
            };
            if !epoch.is_current()
                || firmware.recovery.pending()
                || self.crashed.load(Ordering::Acquire)
            {
                break;
            }
            let Some(slot) = firmware.next_render_observation(cursor)? else {
                break;
            };
            if slot != cursor {
                remaining = crate::hw::t8140::queues::RENDER_DEPTH;
                retries = 0;
            }
            let observed = firmware.queues.render.entries[slot]
                .as_ref()
                .and_then(|entry| entry.pair.as_ref())
                .ok_or(EIO)?
                .observe_front();
            // Publishers waiting for render capacity recheck once this hold ends.
            self.note_changed();
            if firmware.render_grow_pending() || firmware.queues.render.take_missed_event(slot as u8) {
                // Observation and the growth check run on every pass.
                self.queue_event_records();
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
