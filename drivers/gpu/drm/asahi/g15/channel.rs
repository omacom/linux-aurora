// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Submit channels, the device-control ring, the receive rings and the doorbells.
//!
//! - Submit channels 0 (TA), 1 (FS), 2 (compute) of 12 allocated, 256 x 0x18 records each.
//!   Posting writes the record, a write barrier, then the producer (+0x20 of the state block),
//!   and another barrier. Doorbell: EP 0x21 `0x0083 << 48 | channel`. Records may be posted
//!   behind unconsumed ones (submit-ahead, at most [`super::hw::sizes::AHEAD_RECORDS`]), never
//!   over an unconsumed slot ([`Channels::post_ahead`]).
//! - Device-control ring: 256 x 0x38 records; doorbell EP 0x21 `0x0083000000000011`.
//!   [`Firmware::devctl`] with `wait` posts and waits for consumption and completion (2 s);
//!   without it, it does not wait (the kill request). Both return EBUSY while Control+0x4590
//!   says the firmware is halted, and wait for an earlier posted record's consumption first.
//! - Receive rings (event, 6 log sub-rings, trace, stats): consumed by setting read = producer;
//!   the event ring is decoded by [`super::event`].
//! - FwCtl ring: 256 x 0x18 = 0x1800 bytes (the 2419 firmware uses a 0x18 stride). Never used
//!   by the host: no firmware cache flush path exists.

use kernel::{
    prelude::*,
    sync::Arc,
    time::{delay::fsleep, Delta, Instant, Monotonic},
};

use super::boot::Session;
use super::event::Events;
use super::fw::channels::{
    DevCtl, SubmitRecord, RING_SLOTS, STATE_COMPLETION, STATE_READ, STATE_WRITE,
};
use super::hw::{self, rings, va};
use super::initdata::InitData;
use super::object::{Class, FwObject, Objects, Placement, Request};
use super::regs::wmb;
use super::Firmware;

/// A submit channel.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) enum Channel {
    Ta = 0,
    Fs = 1,
    Compute = 2,
}

/// A ring and its state block.
pub(crate) struct Ring {
    pub(crate) state: Arc<FwObject>,
    pub(crate) ring: Arc<FwObject>,
}

impl Ring {
    /// The consumer's read index (state +0x00).
    pub(crate) fn read(&self) -> Result<u32> {
        self.state.r32(STATE_READ)
    }

    /// The completion index (state +0x10).
    pub(crate) fn completion(&self) -> Result<u32> {
        self.state.r32(STATE_COMPLETION)
    }

    /// The producer's write index (state +0x20).
    pub(crate) fn write(&self) -> Result<u32> {
        self.state.r32(STATE_WRITE)
    }
}

/// The channels and rings.
pub(crate) struct Channels {
    /// The 12 submit channels (TA, FS and compute are used).
    pub(crate) submit: KVec<Ring>,
    pub(crate) devctl: Ring,
    pub(crate) event: Ring,
    /// The six log sub-rings share one state object (6 x 0x30) and one ring object.
    pub(crate) log: Ring,
    pub(crate) trace: Ring,
    pub(crate) stats: Ring,
    /// Allocated for the firmware; the host never uses it.
    _fwctl: Ring,
    /// Host producers of channels 0 TA, 1 FS, 2 compute.
    producer: [u32; 3],
    /// Host producer of the device-control ring.
    devctl_producer: u32,
}

fn fw_object(objects: &mut Objects, at: u64, size: usize) -> Result<Arc<FwObject>> {
    objects.alloc(Request {
        size,
        class: Class::Fw,
        placement: Placement::At(at),
        offset: 0,
        client: None,
        lower: None,
        cpu: true,
    })
}

fn ring(
    objects: &mut Objects,
    state: u64,
    state_size: usize,
    ring: u64,
    ring_size: usize,
) -> Result<Ring> {
    Ok(Ring {
        state: fw_object(objects, state, state_size)?,
        ring: fw_object(objects, ring, ring_size)?,
    })
}

impl Channels {
    /// Allocates every channel and ring of the initdata graph, firmware-only and uncached.
    pub(crate) fn alloc(objects: &mut Objects) -> Result<Self> {
        let mut submit = KVec::with_capacity(rings::CHANNELS as usize, GFP_KERNEL)?;
        for i in 0..rings::CHANNELS {
            submit.push(
                ring(
                    objects,
                    va::ch_state(i),
                    rings::STATE_SIZE,
                    va::ch_ring(i),
                    rings::CHANNEL_RING_SIZE,
                )?,
                GFP_KERNEL,
            )?;
        }
        Ok(Self {
            submit,
            devctl: ring(
                objects,
                va::DEVCTL_STATE,
                rings::STATE_SIZE,
                va::DEVCTL_RING,
                rings::DEVCTL_RING_SIZE,
            )?,
            event: ring(
                objects,
                va::EVENT_STATE,
                rings::STATE_SIZE,
                va::EVENT_RING,
                rings::EVENT_RING_SIZE,
            )?,
            log: ring(
                objects,
                va::LOG_STATE,
                rings::LOG_STATE_SIZE,
                va::LOG_RING,
                rings::LOG_RING_SIZE,
            )?,
            trace: ring(
                objects,
                va::TRACE_STATE,
                rings::STATE_SIZE,
                va::TRACE_RING,
                rings::TRACE_RING_SIZE,
            )?,
            stats: ring(
                objects,
                va::STATS_STATE,
                rings::STATE_SIZE,
                va::STATS_RING,
                rings::STATS_RING_SIZE,
            )?,
            _fwctl: ring(
                objects,
                va::FWCTL_STATE,
                rings::STATE_SIZE,
                va::FWCTL_RING,
                rings::FWCTL_RING_SIZE,
            )?,
            producer: [0; 3],
            devctl_producer: 0,
        })
    }

    /// A submit channel's ring.
    pub(crate) fn channel(&self, channel: Channel) -> &Ring {
        &self.submit[channel as usize]
    }

    /// The host producer of `channel`.
    pub(crate) fn producer(&self, channel: Channel) -> u32 {
        self.producer[channel as usize]
    }

    /// The firmware's read index of `channel`.
    pub(crate) fn read(&self, channel: Channel) -> Result<u32> {
        self.channel(channel).read()
    }

    /// The channel's write cell (state +0x20).
    pub(crate) fn write_cell(&self, channel: Channel) -> Result<u32> {
        self.channel(channel).write()
    }

    /// Writes one submit record at the channel's producer and advances it, only if every earlier
    /// record was consumed. Returns the new producer: the read index at which this record has
    /// been consumed.
    pub(crate) fn post(&mut self, channel: Channel, record: &SubmitRecord) -> Result<u32> {
        self.post_inner(channel, record, false)
    }

    /// Like [`Self::post`], but behind unconsumed records while fewer than
    /// [`hw::sizes::AHEAD_RECORDS`] are outstanding.
    pub(crate) fn post_ahead(&mut self, channel: Channel, record: &SubmitRecord) -> Result<u32> {
        self.post_inner(channel, record, true)
    }

    fn post_inner(&mut self, channel: Channel, record: &SubmitRecord, ahead: bool) -> Result<u32> {
        let ch = channel as usize;
        let producer = self.producer[ch];
        let ring = &self.submit[ch];
        let consumer = ring.read()?;
        // Never over an unconsumed record: behind them only with room (ahead).
        let busy = if ahead {
            producer.wrapping_sub(consumer) % RING_SLOTS >= hw::sizes::AHEAD_RECORDS
        } else {
            consumer != producer
        };
        if busy || ring.write()? != producer {
            return Err(EBUSY);
        }
        ring.ring.write(
            producer as usize * rings::CHANNEL_RECORD,
            &record.to_bytes(),
        )?;
        wmb();
        let producer = (producer + 1) % RING_SLOTS;
        ring.state.w32(STATE_WRITE, producer)?;
        wmb();
        self.producer[ch] = producer;
        Ok(producer)
    }

    /// Consumes a receive ring to its firmware producer.
    fn drain_one(ring: &Ring, offset: usize) -> Result {
        let producer = ring.state.r32(offset + STATE_WRITE)?;
        if ring.state.r32(offset + STATE_READ)? != producer {
            ring.state.w32(offset + STATE_READ, producer)?;
            wmb();
        }
        Ok(())
    }

    /// Drains every receive ring: events (decoded into `events`), the six log sub-rings, trace
    /// and stats.
    pub(crate) fn drain_rings(&self, dev: &kernel::device::Device, events: &mut Events) -> Result {
        events.drain(dev, &self.event)?;
        for i in 0..rings::LOG_RINGS {
            Self::drain_one(&self.log, i * 0x30)?;
        }
        Self::drain_one(&self.trace, 0)?;
        Self::drain_one(&self.stats, 0)
    }
}

/// Posts one 0x38-byte device-control record (EP 0x21 `0x0083000000000011`) behind the earlier
/// ones (one posted without waiting may still be pending: wait for its consumption first) and,
/// with `wait`, waits for its consumption and completion. A firmware halted for recovery consumes
/// nothing until the host's resume: EBUSY then, so the caller can answer the halt instead of
/// timing out. Callers serialize (the device lock once the backend runs).
pub(crate) fn devctl_submit(
    session: &mut Session,
    channels: &mut Channels,
    events: &mut Events,
    init: &InitData,
    record: &DevCtl,
    wait: bool,
) -> Result {
    let dev = session.dev.clone();
    let producer = channels.devctl_producer;
    let next = (producer + 1) % RING_SLOTS;
    let start = Instant::<Monotonic>::now();
    let expired = || start.elapsed() > Delta::from_millis(2000);
    let opcode = record.opcode();
    while channels.devctl.read()? != producer {
        channels.drain_rings(&dev, events)?;
        session.check()?;
        if init.halted()? {
            return Err(EBUSY);
        }
        if expired() {
            dev_err!(
                dev,
                "control {:#x}: earlier record not consumed (read={} producer={})\n",
                opcode,
                channels.devctl.read()?,
                producer
            );
            return Err(ETIMEDOUT);
        }
        fsleep(Delta::from_micros(1000));
    }
    channels
        .devctl
        .ring
        .write(producer as usize * rings::DEVCTL_RECORD, &record.to_bytes())?;
    wmb();
    channels.devctl.state.w32(STATE_WRITE, next)?;
    wmb();
    channels.devctl_producer = next;
    session.send(hw::ep::DOORBELL, hw::engine::DEVCTL_DOORBELL)?;
    if !wait {
        return Ok(());
    }
    loop {
        channels.drain_rings(&dev, events)?;
        session.check()?;
        if channels.devctl.read()? == next && channels.devctl.completion()? == next {
            return Ok(());
        }
        if init.halted()? {
            return Err(EBUSY);
        }
        if expired() {
            dev_err!(
                dev,
                "control {:#x}: read={} completion={} producer={}\n",
                opcode,
                channels.devctl.read()?,
                channels.devctl.completion()?,
                next
            );
            return Err(ETIMEDOUT);
        }
        fsleep(Delta::from_micros(1000));
    }
}

impl Firmware {
    /// Posts a device-control record; with `wait` waits for its consumption and completion,
    /// otherwise returns once posted. EBUSY while the firmware is halted; the transport error if
    /// the session failed.
    pub(crate) fn devctl(&mut self, record: &DevCtl, wait: bool) -> Result {
        devctl_submit(
            &mut self.session,
            &mut self.channels,
            &mut self.events,
            &self.init,
            record,
            wait,
        )
    }

    /// The latched transport error: an RTKit crash or a refused RTKit buffer.
    pub(crate) fn transport_error(&self) -> Option<Error> {
        self.session.check().err()
    }

    /// Drains the receive rings and decodes events. Called first in every poll pass.
    pub(crate) fn drain_rings(&mut self) -> Result {
        let dev = self.session.dev.clone();
        self.channels.drain_rings(&dev, &mut self.events)
    }
}
