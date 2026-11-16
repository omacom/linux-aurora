// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Firmware-produced rings and the device-control producer.
//!
//! The device mutex serializes every caller. Entries are copied before a
//! single aligned consumer store returns their slots to the firmware. Trace
//! and statistics transport errors quarantine only that ring; event errors
//! propagate to the device owner. Device-control publication reserves one
//! empty slot and orders the record, power tally and producer together.

use core::sync::atomic::{fence, Ordering};
use kernel::{device::Device, prelude::*};

use super::fw::{channels::*, initdata::status};
use super::initdata::InitData;
use super::Role;

/// Firmware rings whose contents do not affect host scheduling.
#[derive(Copy, Clone, Debug)]
enum ReportRing {
    Trace = 0,
    Statistics = 1,
}

impl ReportRing {
    fn geometry(self) -> (usize, usize, u32, u32) {
        match self {
            Self::Trace => (
                status::TRACE_STATE as usize,
                status::TRACE_RING as usize,
                TRACE_SLOTS,
                TRACE_TYPES,
            ),
            Self::Statistics => (
                status::STATS_STATE as usize,
                status::STATS_RING as usize,
                STATISTICS_SLOTS,
                STATISTICS_TYPES,
            ),
        }
    }
}

/// Host-side receive-ring state. A quarantined ring stays stopped until the
/// device is destroyed; firmware restart does not clear it.
#[derive(Debug)]
pub(crate) struct Rings {
    quarantined: [[bool; 2]; 2],
}

impl Rings {
    pub(crate) const fn new() -> Self {
        Self {
            quarantined: [[false; 2]; 2],
        }
    }

    pub(crate) fn drain_trace(&mut self, dev: &Device, init: &InitData, role: Role) {
        self.drain_report(dev, init, role, ReportRing::Trace);
    }

    pub(crate) fn drain_statistics(&mut self, dev: &Device, init: &InitData, role: Role) {
        self.drain_report(dev, init, role, ReportRing::Statistics);
    }

    fn drain_report(&mut self, dev: &Device, init: &InitData, role: Role, ring: ReportRing) {
        let quarantined = &mut self.quarantined[role as usize][ring as usize];
        if *quarantined {
            return;
        }
        if let Err(error) = Self::consume_report(init, role, ring) {
            *quarantined = true;
            dev_err!(
                dev.as_ref(),
                "G17 {:?} {:?} ring stopped: {:?}\n",
                role,
                ring,
                error
            );
        }
    }

    fn consume_report(init: &InitData, role: Role, ring: ReportRing) -> Result {
        let (state, entries, capacity, allowed_types) = ring.geometry();
        let consumer_word = init.status_word(role, state)?;
        let producer_word = init.status_word(role, state + RX_PRODUCER_OFFSET)?;
        fence(Ordering::Acquire);
        let mut consumer = consumer_word.load(Ordering::Relaxed);
        let producer = producer_word.load(Ordering::Relaxed);
        // The producer makes the entries it covers available to the host.
        fence(Ordering::Acquire);
        if consumer >= capacity || producer >= capacity {
            return Err(EIO);
        }
        while consumer != producer {
            let offset = entries + consumer as usize * EVENT_RECORD_SIZE;
            let tag = match ring {
                // Copy the entire trace entry before returning its slot.
                ReportRing::Trace => init.status_read_words::<EVENT_WORDS>(role, offset)?[0],
                ReportRing::Statistics => init.status_read_words::<1>(role, offset)?[0],
            };
            if tag >= u32::BITS || allowed_types & (1 << tag) == 0 {
                return Err(EIO);
            }
            consumer = (consumer + 1) & (capacity - 1);
            fence(Ordering::SeqCst);
            consumer_word.store(consumer, Ordering::Relaxed);
            if matches!(ring, ReportRing::Trace) {
                fence(Ordering::SeqCst);
            }
        }
        if matches!(ring, ReportRing::Statistics) {
            fence(Ordering::SeqCst);
        }
        Ok(())
    }

    /// Publishes one primary device-control record. On EAGAIN the caller
    /// retains the request and retries before consuming more dependent events.
    /// The returned cursor is the value the consumer must reach to retire it.
    /// The caller announces the publication through the coprocessor mailbox.
    pub(crate) fn publish_control(init: &InitData, record: &ControlRecord) -> Result<u32> {
        fence(Ordering::Acquire);
        let consumer = init.control_consumer()?.load(Ordering::Relaxed);
        let producer = init.control_producer()?.load(Ordering::Relaxed);
        if consumer >= DEVICE_CONTROL_SLOTS || producer >= DEVICE_CONTROL_SLOTS {
            return Err(EIO);
        }
        let next = (producer + 1) & (DEVICE_CONTROL_SLOTS - 1);
        if next == consumer {
            return Err(EAGAIN);
        }
        // InitData validates every mapping before the first write, then writes
        // record, packed tally, barrier, producer, barrier without failure.
        init.publish_control(
            producer,
            record.bytes(),
            record.opcode().asserts_gpu_power(),
            next,
        )?;
        super::trace::message(super::trace::CONTROL, record.opcode() as u64);
        Ok(next)
    }
}

/// One bounded event-ring pass. No shared-memory borrow survives `next`, so
/// the caller can service the copied event using the complete device owner.
pub(crate) struct Events {
    role: Role,
    remaining: u32,
    unknown_reported: bool,
}

impl Events {
    /// A bounded pass can leave records written while its earlier slots were
    /// consumed. Check both cursors again before the worker goes idle.
    pub(crate) fn pending(init: &InitData, role: Role) -> Result<bool> {
        let state = status::EVENT_STATE as usize;
        fence(Ordering::Acquire);
        let consumer = init.status_word(role, state)?.load(Ordering::Relaxed);
        let producer = init
            .status_word(role, state + RX_PRODUCER_OFFSET)?
            .load(Ordering::Relaxed);
        if consumer >= EVENT_SLOTS || producer >= EVENT_SLOTS {
            return Err(EIO);
        }
        Ok(consumer != producer)
    }

    pub(crate) fn new(role: Role) -> Self {
        Self {
            role,
            remaining: EVENT_SLOTS,
            unknown_reported: false,
        }
    }

    pub(crate) fn next(&mut self, dev: &Device, init: &InitData) -> Result<Option<Event>> {
        let role = self.role;
        let state = status::EVENT_STATE as usize;
        let consumer_word = init.status_word(role, state)?;
        let producer_word = init.status_word(role, state + RX_PRODUCER_OFFSET)?;
        while self.remaining != 0 {
            init.validate_event_ring(role)?;
            self.remaining -= 1;
            let consumer = consumer_word.load(Ordering::Relaxed);
            let producer = producer_word.load(Ordering::Relaxed);
            if consumer >= EVENT_SLOTS || producer >= EVENT_SLOTS {
                return Err(EIO);
            }
            if consumer == producer {
                self.remaining = 0;
                break;
            }
            // Observe the producer before reading the record it publishes.
            fence(Ordering::Acquire);
            let offset = status::EVENT_RING as usize + consumer as usize * EVENT_RECORD_SIZE;
            let record = EventRecord::from_words(init.status_read_words(role, offset)?);
            let event = record.decode();
            fence(Ordering::SeqCst);
            consumer_word.store((consumer + 1) & (EVENT_SLOTS - 1), Ordering::Relaxed);
            fence(Ordering::SeqCst);
            match event? {
                Event::Unknown(tag) => {
                    if !self.unknown_reported {
                        dev_warn!(
                            dev.as_ref(),
                            "G17 {:?} unknown firmware event {}\n",
                            role,
                            tag
                        );
                        self.unknown_reported = true;
                    }
                }
                event => return Ok(Some(event)),
            }
        }
        Ok(None)
    }
}
