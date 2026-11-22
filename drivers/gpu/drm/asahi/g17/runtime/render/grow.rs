// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Deferred parameter-buffer growth. Event ingress retains the entire request;
//! allocation borrows one pair under a preparation lease and runs off-lock.

use super::RenderState;
use crate::g17::{
    channel::Rings,
    fw::channels::{
        ChannelErrorAck, ControlRecord, Engine, FreeListCompletion, TvbGrowReply, TvbGrowRequest,
        EVENT_SLOTS,
    },
};
use core::sync::atomic::{fence, Ordering};
use kernel::prelude::*;

const RETRY_LIMIT: u16 = 1000;

/// A channel-error acknowledgement: its record, and once published the control cursor the
/// consumer must pass before the queue it names may be reused.
pub(super) struct ChannelAck {
    record: ChannelErrorAck,
    qid: u8,
    cursor: Option<u32>,
}

pub(super) struct Grow {
    pub(super) requests: KVec<TvbGrowRequest>,
    pub(super) active: Option<TvbGrowRequest>,
    pub(super) reply: Option<(TvbGrowRequest, bool, u16)>,
    release_attempts: u16,
    /// Channel errors not yet consumed by the firmware, oldest first.
    pub(super) acks: KVec<ChannelAck>,
    ack_attempts: u16,
}
impl Grow {
    pub(super) fn new() -> Result<Self> {
        Ok(Self {
            requests: KVec::with_capacity(2 * EVENT_SLOTS as usize, GFP_KERNEL)?,
            active: None,
            reply: None,
            release_attempts: 0,
            acks: KVec::new(),
            ack_attempts: 0,
        })
    }
}

impl RenderState {
    pub(in crate::g17::runtime) fn control_backpressured(&self) -> bool {
        self.grow.reply.is_some() || self.pools.pending() || !self.grow.acks.is_empty()
    }
    fn grow_owner(&self, request: TvbGrowRequest) -> Option<usize> {
        self.entries.iter().position(|entry| {
            entry.as_ref().is_some_and(|entry| {
                entry.published
                    && u32::from(entry.reservation.buffer.0) == request.buffer_slot
                    && entry.context.as_ref().is_some_and(|context| {
                        request.vm_slot == -1 || request.vm_slot as u32 == context.id()
                    })
            })
        })
    }
    /// The queue record the firmware runs physical queue `qid` from, if a pair holds it.
    pub(in crate::g17::runtime) fn queue_record_va(&self, qid: u8) -> Option<u64> {
        self.entries
            .iter()
            .flatten()
            .filter_map(|entry| entry.pair.as_deref())
            .find_map(|pair| pair.queue_record_va(qid))
    }
    fn grow_owner_busy(&self, request: TvbGrowRequest) -> bool {
        self.grow_owner(request).is_some_and(|slot| {
            self.entries[slot]
                .as_ref()
                .is_some_and(|entry| entry.preparing.is_some() || entry.growing)
        })
    }
}

impl crate::g17::Firmware {
    pub(in crate::g17) fn queue_tvb_grow(&mut self, request: TvbGrowRequest) -> Result {
        let render = &mut self.queues.render;
        if request.halt_count != self.recovery.generation()
            || render.grow.active == Some(request)
            || render.grow.requests.contains(&request)
        {
            return Ok(());
        }
        render
            .grow
            .requests
            .push_within_capacity(request)
            .map_err(|_| EOVERFLOW)
    }
    pub(in crate::g17) fn render_grow_pending(&self) -> bool {
        !self.queues.render.grow.requests.is_empty()
    }
    /// Borrowed owners and control publication each have their own retry wake.
    /// A stale request is runnable so the worker can discard it immediately.
    pub(in crate::g17) fn render_grow_ready(&self) -> bool {
        let render = &self.queues.render;
        !render.grow.requests.is_empty()
            && !self.recovery.pending()
            && !self.render_control_backpressured()
            && render.grow.active.is_none()
            && render.grow.requests.iter().any(|request| {
                request.halt_count != self.recovery.generation()
                    || !render.grow_owner_busy(*request)
            })
    }
    pub(in crate::g17) fn render_control_backpressured(&self) -> bool {
        self.queues.render.grow.reply.is_some()
            || self.queues.render.pools.pending()
            || !self.queues.render.grow.acks.is_empty()
    }

    /// Whether a channel error of physical queue `qid` still waits for the firmware to consume
    /// its acknowledgement: the queue must not be vacated, rebound or released until then.
    pub(in crate::g17) fn channel_error_pending(&self, qid: u8) -> bool {
        self.queues.render.grow.acks.iter().any(|ack| ack.qid == qid)
    }

    /// Queues the acknowledgement of a channel error. The queue record is the one the firmware
    /// runs the named engine of `qid` from; an error of a queue no longer bound is still logged.
    pub(in crate::g17) fn queue_channel_error(&mut self, engine: Engine, qid: u8, stamp: u32) -> Result {
        let queue_va = match engine {
            Engine::Compute => self
                .queues
                .compute
                .iter()
                .flatten()
                .filter_map(|entry| entry.queue.as_deref())
                .find(|queue| queue.qid() == qid)
                .map(|queue| queue.queue_record_va()),
            Engine::Tiling | Engine::Fragment => self.queues.render.queue_record_va(qid),
        };
        let dev = self.primary.state.shared.dev.as_ref();
        dev_warn!(
            dev,
            "G17: firmware channel error on queue {} {:?} stamp {:#x}{}\n",
            qid,
            engine,
            stamp,
            if queue_va.is_some() { "" } else { " (queue not bound)" }
        );
        let Some(queue_va) = queue_va else {
            return Ok(());
        };
        let record = ChannelErrorAck::new(engine, queue_va, stamp);
        let acks = &mut self.queues.render.grow.acks;
        if acks.iter().any(|ack| ack.qid == qid && ack.record == record) {
            return Ok(());
        }
        acks.push(ChannelAck { record, qid, cursor: None }, GFP_ATOMIC)?;
        self.mark_channel_error(qid, true);
        Ok(())
    }

    /// Holds (or frees) the queue or pair that runs physical queue `qid` for reuse.
    fn mark_channel_error(&mut self, qid: u8, pending: bool) {
        for mut queue in self
            .queues
            .compute
            .iter_mut()
            .flatten()
            .filter_map(|entry| entry.queue.as_deref_mut())
            .filter(|queue| queue.qid() == qid)
        {
            queue.set_channel_error(pending);
        }
        for pair in self
            .queues
            .render
            .entries
            .iter_mut()
            .flatten()
            .filter_map(|entry| entry.pair.as_deref_mut())
            .filter(|pair| pair.qids().contains(&qid))
        {
            pair.set_channel_error(pending);
        }
    }

    /// Publishes every unpublished acknowledgement in order and forgets the ones the firmware
    /// consumed. False means ring backpressure: the rest is retried on the next pass.
    pub(in crate::g17) fn flush_channel_errors(&mut self) -> Result<bool> {
        if self.queues.render.grow.acks.is_empty() {
            return Ok(true);
        }
        let mut published = false;
        let mut full = false;
        for ack in self.queues.render.grow.acks.iter_mut() {
            if ack.cursor.is_some() {
                continue;
            }
            match Rings::publish_control(&self.init, &ControlRecord::ChannelErrorAck(ack.record)) {
                Err(error) if error == EAGAIN => {
                    full = true;
                    break;
                }
                result => ack.cursor = Some(result?),
            }
            published = true;
        }
        if published {
            fence(Ordering::SeqCst);
            self.primary.notify(crate::g17::MSG_CONTROL_NOTIFY)?;
        }
        let consumer = self.init.control_consumer()?.load(Ordering::Acquire);
        let producer = self.init.control_producer()?.load(Ordering::Acquire);
        let mut index = 0;
        while index < self.queues.render.grow.acks.len() {
            let ack = &self.queues.render.grow.acks[index];
            let done = match ack.cursor {
                Some(cursor) => crate::g17::freelist::control_consumed(consumer, producer, cursor)?,
                None => false,
            };
            if !done {
                index += 1;
                continue;
            }
            let qid = ack.qid;
            self.queues.render.grow.acks.remove(index).map_err(|_| EINVAL)?;
            if !self.channel_error_pending(qid) {
                self.mark_channel_error(qid, false);
            }
        }
        if self.queues.render.grow.acks.is_empty() {
            self.queues.render.grow.ack_attempts = 0;
        }
        Ok(!full)
    }
    pub(in crate::g17) fn complete_freelist_grow(&mut self, event: FreeListCompletion) -> Result {
        self.queues.render.pools.complete_grow(event)
    }
    pub(in crate::g17) fn observe_render_releases(&mut self) -> Result {
        self.queues.render.pools.observe(&self.init)
    }
    pub(in crate::g17) fn flush_tvb_reply(&mut self) -> Result<bool> {
        let Some((request, grew, _)) = self.queues.render.grow.reply else {
            return Ok(true);
        };
        if request.halt_count != self.recovery.generation() {
            return Err(EIO);
        }
        let record = ControlRecord::TvbGrow(TvbGrowReply::new(&request, grew));
        match Rings::publish_control(&self.init, &record) {
            Err(error) if error == EAGAIN => return Ok(false),
            result => {
                result?;
            }
        }
        // Producer ownership is irreversible, even if announcement fails.
        self.queues.render.grow.reply = None;
        fence(Ordering::SeqCst);
        self.primary
            .notify((0x84 << crate::g17::MSG_TYPE_SHIFT) | 0x11)?;
        Ok(true)
    }
    pub(in crate::g17) fn flush_freelist_release(&mut self) -> Result<bool> {
        if self.queues.render.grow.reply.is_some() {
            return Ok(false);
        }
        match self.release_render_lists() {
            Err(error) if error == EAGAIN => Ok(false),
            result => {
                result?;
                self.queues.render.grow.release_attempts = 0;
                Ok(true)
            }
        }
    }
    /// Only the sleeping event worker consumes this budget. Inline submission
    /// probes cannot turn a transient full control ring into a rapid timeout.
    pub(in crate::g17) fn note_render_control_backpressure(&mut self) -> Result {
        let render = &mut self.queues.render;
        let attempts = if let Some((_, _, attempts)) = render.grow.reply.as_mut() {
            attempts
        } else if !render.grow.acks.is_empty() {
            &mut render.grow.ack_attempts
        } else if render.pools.pending() {
            &mut render.grow.release_attempts
        } else {
            return Err(EINVAL);
        };
        *attempts = attempts.checked_add(1).ok_or(ETIMEDOUT)?;
        if *attempts > RETRY_LIMIT {
            return Err(ETIMEDOUT);
        }
        Ok(())
    }
}

impl crate::g17::Firmware {
    /// Commits a host growth prepared for the pair in `slot`. A `busy` pair's
    /// growth only appends blocks to the inventory, as a firmware grow request's
    /// does; if it cannot be committed the pair keeps its heap and the render
    /// is published as it would have been without it.
    pub(super) fn commit_render_growth(
        &mut self,
        slot: u8,
        prepared: &mut Option<crate::g17::buffer::PreparedGrowth>,
        busy: bool,
    ) -> Result {
        if prepared.is_none() {
            return Ok(());
        }
        let pair = self
            .queues
            .render
            .entry_mut(slot)?
            .pair
            .as_mut()
            .ok_or(EIO)?;
        let previous = pair.manager().blocks();
        let source = if busy {
            crate::g17::trace::HEAP_HOST_BUSY
        } else {
            crate::g17::trace::HEAP_HOST_IDLE
        };
        let committed: Result = (|| {
            let vm = pair.context().vm().clone();
            let pool = pair.pool_id();
            let buffer_id = pair.buffer_id();
            // Preflight every destination before the graph starts publishing.
            let table = self.init.pb_descriptor_table()?;
            let offset = usize::from(buffer_id) * 16;
            let words = [
                table.word(offset)?,
                table.word(offset + 4)?,
                table.word(offset + 8)?,
                table.word(offset + 12)?,
            ];
            let descriptor = pair
                .manager()
                .commit_growth(prepared, |previous, blocks, mappings| {
                    vm.append_render_pool_mappings_prepared(slot, pool, previous, blocks, mappings)
                })?;
            if let Some(descriptor) = descriptor {
                for (word, value) in words.into_iter().zip(descriptor) {
                    word.store(value, Ordering::Relaxed);
                }
                fence(Ordering::SeqCst);
            }
            Ok(())
        })();
        let manager = pair.manager();
        crate::g17::trace::heap(source, committed.is_ok(), previous, manager.blocks());
        match committed {
            Ok(()) if busy => manager.note_busy_growth(previous),
            // The prepared growth is dropped by the caller after the device mutex.
            Err(_) if busy => manager.defer_busy_growth(crate::g17::now_ns()),
            committed => committed?,
        }
        Ok(())
    }
}

impl crate::g17::Shared {
    pub(in crate::g17) fn grow_render(&self) -> Result {
        loop {
            let lease = self.preparations.enter(false)?;
            let dev = self.drm()?;
            let (request, slot, mut pair) = {
                let mut state = self.state.lock();
                let firmware = (*state).as_deref_mut().ok_or(ENODEV)?;
                if firmware.render_control_backpressured() {
                    return Ok(());
                }
                let generation = firmware.recovery.generation();
                let render = &mut firmware.queues.render;
                if render.grow.active.is_some() {
                    return Ok(());
                }
                let Some(index) = render.grow.requests.iter().position(|request| {
                    request.halt_count != generation || !render.grow_owner_busy(*request)
                }) else {
                    return Ok(());
                };
                let request = render.grow.requests.remove(index).map_err(|_| EIO)?;
                if request.halt_count != generation {
                    continue;
                }
                let slot = render.grow_owner(request);
                let pair = match slot {
                    Some(slot) => {
                        let entry = render.entries[slot].as_mut().ok_or(EIO)?;
                        if entry.preparing.is_some() || entry.growing {
                            return Err(EIO);
                        }
                        entry.borrowed_active = entry
                            .pair
                            .as_ref()
                            .is_some_and(|pair| pair.published_in_flight());
                        entry.borrowed_dependencies =
                            entry.pair.as_ref().map(|pair| pair.dependency_snapshot());
                        entry.borrowed_switch =
                            entry.pair.as_ref().and_then(|pair| pair.switch_in_flight());
                        entry.growing = true;
                        Some(entry.pair.take().ok_or(EIO)?)
                    }
                    None => None,
                };
                render.grow.active = Some(request);
                (request, slot, pair)
            };
            let before = pair.as_mut().map_or(0, |pair| pair.manager().blocks());
            let prepared = match pair.as_mut() {
                Some(pair) => {
                    let vm = pair.context().vm().clone();
                    let pool = pair.pool_id();
                    let slot = pair.slot();
                    let manager = pair.manager();
                    if vm.render_pool_tvb_blocks(slot, pool) != Some(before) {
                        Err(EFAULT)
                    } else {
                        manager.prepare_growth(
                            &dev,
                            &vm,
                            manager.firmware_target(),
                            None,
                        )
                    }
                }
                None => Err(ENOENT),
            };
            let (mut prepared, prepare_error) = match prepared {
                Ok(plan) => (plan, None),
                Err(error) => (None, Some(error)),
            };
            let mut state = self.state.lock();
            let firmware = (*state).as_deref_mut().ok_or(ENODEV)?;
            if let Some(slot) = slot {
                let entry = firmware.queues.render.entries[slot].as_mut().ok_or(EIO)?;
                if entry.pair.is_some() || !entry.growing {
                    return Err(EIO);
                }
                entry.pair = pair.take();
                entry.borrowed_dependencies = None;
                entry.borrowed_switch = None;
                entry.growing = false;
                // The grow work item requests an event pass when it finishes.
                entry.missed_event = false;
                firmware.queues.hold_render_slot(slot);
            }
            if firmware.queues.render.grow.active != Some(request) {
                return Err(EIO);
            }
            firmware.queues.render.grow.active = None;
            self.note_changed();
            if !lease.is_current() || request.halt_count != firmware.recovery.generation() {
                continue;
            }
            let result = if let Some(error) = prepare_error {
                Err(error)
            } else if prepared.is_none() {
                Ok(false)
            } else if let Some(slot) = slot {
                let pair = firmware.queues.render.entries[slot]
                    .as_mut()
                    .and_then(|entry| entry.pair.as_mut())
                    .ok_or(EIO)?;
                let vm = pair.context().vm().clone();
                let pool = pair.pool_id();
                let slot = pair.slot();
                let grew = prepared.is_some();
                pair.manager()
                    .commit_growth(&mut prepared, |previous, blocks, mappings| {
                        vm.append_render_pool_mappings_prepared(
                            slot, pool, previous, blocks, mappings,
                        )
                    })
                    .map(|_| grew)
            } else {
                Err(ENOENT)
            };
            let grew = match result {
                Ok(grew) => grew,
                Err(error) => {
                    dev_warn!(
                        self.dev.as_ref(),
                        "Could not grow render parameter buffer {}: {:?}\n",
                        request.buffer_slot,
                        error
                    );
                    false
                }
            };
            // A refused or capped request leaves the tiling pass to partial-render
            // within the heap it has.
            let after = slot
                .and_then(|slot| firmware.queues.render.entries[slot].as_mut())
                .and_then(|entry| entry.pair.as_mut())
                .map_or(before, |pair| pair.manager().blocks());
            crate::g17::trace::heap(
                if grew {
                    crate::g17::trace::HEAP_FIRMWARE_GRANTED
                } else {
                    crate::g17::trace::HEAP_FIRMWARE_REFUSED
                },
                grew,
                before,
                after,
            );
            firmware.queues.render.grow.reply = Some((request, grew, 0));
            if !firmware.flush_tvb_reply()? {
                return Ok(());
            }
        }
    }
}
