// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Decoded firmware events.
//!
//! Event ring records are 0x38 bytes: type 1 stage flags (u64 at +4: bit 0 TA, bit 1 FS, bit 2
//! compute), type 0 fault, type 4 timeout (i32 culprit slot at +0xc), type 7 PB growth request
//! (copied whole into an 8-entry queue; never answered by the drain itself), type 8 channel
//! error. The drain runs under the device lock from the poll work (and inline in the boot and
//! device-control waits); the EP 0x20 interrupt only kicks the poll. Notifications only:
//! retirement has its own receipts.

use kernel::{bindings, device, prelude::*};

use super::channel::Ring;
use super::fw::channels::{event, RING_SLOTS, STATE_READ, STATE_WRITE};
use super::hw::rings;
use super::regs::{mb, rmb, wmb};

/// Number of queued PB growth requests.
pub(crate) const GROW_QUEUE: usize = 8;

/// A copied firmware error event (type 0 fault or type 4 timeout).
#[derive(Copy, Clone, Default)]
pub(crate) struct ErrorEvent {
    pub(crate) kind: u32,
    pub(crate) raw: [u32; 14],
    /// Type 4: the culprit event slot (possibly -1); -1 for type 0.
    pub(crate) slot: i32,
}

/// Event state consumed by the engine, recovery and growth.
pub(crate) struct Events {
    /// Compute completions and stage events seen.
    pub(crate) compute_events: u32,
    pub(crate) stage_events: u32,
    /// Error events seen, and the last one.
    pub(crate) errors: u32,
    pub(crate) last_error: Option<ErrorEvent>,
    /// Channel errors seen.
    pub(crate) channel_errors: u32,
    /// Last firing time per stage bit (TA, FS, compute), ns.
    pub(crate) event_ns: [u64; 3],
    /// PB growth requests, oldest first; at most [`GROW_QUEUE`].
    pub(crate) grow: KVec<[u32; 14]>,
    /// Requests dropped because the queue was full.
    pub(crate) grow_lost: u32,
}

impl Events {
    pub(crate) fn new() -> Self {
        Self {
            compute_events: 0,
            stage_events: 0,
            errors: 0,
            last_error: None,
            channel_errors: 0,
            event_ns: [0; 3],
            grow: KVec::new(),
            grow_lost: 0,
        }
    }

    /// Reserves the growth queue, so that the drain never allocates.
    pub(crate) fn reserve(&mut self) -> Result {
        self.grow.reserve(GROW_QUEUE, GFP_KERNEL)?;
        Ok(())
    }

    fn words(ring: &Ring, index: u32) -> Result<[u32; 14]> {
        let base = index as usize * rings::EVENT_RECORD;
        let mut w = [0u32; 14];
        for (j, word) in w.iter_mut().enumerate() {
            *word = ring.ring.r32(base + 4 * j)?;
        }
        Ok(w)
    }

    /// Decodes and consumes the event ring.
    pub(crate) fn drain(&mut self, dev: &device::Device, ring: &Ring) -> Result {
        let producer = ring.state.r32(STATE_WRITE)?;
        let mut read = ring.state.r32(STATE_READ)?;
        // SAFETY: ktime_get() has no preconditions.
        let now = unsafe { bindings::ktime_get() } as u64;
        if producer >= RING_SLOTS || read >= RING_SLOTS {
            return Ok(());
        }
        rmb();
        while read != producer {
            let w = Self::words(ring, read)?;
            let kind = w[0];
            let flags = w[1] as u64 | (w[2] as u64) << 32;
            if kind == event::STAGE {
                for b in 0..3 {
                    if flags & (1 << b) != 0 {
                        self.event_ns[b] = now;
                    }
                }
                if flags & (1 << 2) != 0 {
                    self.compute_events = self.compute_events.wrapping_add(1);
                }
                self.stage_events = self.stage_events.wrapping_add((flags & 7).count_ones());
            }
            if kind == event::FAULT || kind == event::TIMEOUT {
                self.last_error = Some(ErrorEvent {
                    kind,
                    raw: w,
                    slot: if kind == event::TIMEOUT {
                        w[3] as i32
                    } else {
                        -1
                    },
                });
                self.errors = self.errors.wrapping_add(1);
                dev_err!(
                    dev,
                    "firmware event {}: {:08x} {:08x} {:08x} {:08x} {:08x} {:08x}\n",
                    kind,
                    w[1],
                    w[2],
                    w[3],
                    w[4],
                    w[5],
                    w[6]
                );
            } else if kind == event::PB_GROW {
                if self.grow.len() < GROW_QUEUE {
                    self.grow.push(w, GFP_KERNEL)?;
                } else {
                    self.grow_lost = self.grow_lost.wrapping_add(1);
                }
                dev_dbg!(
                    dev,
                    "firmware event 7 (PB growth): ctx {} slot {} counter {} position {} epoch {:#x}\n",
                    w[1],
                    w[2],
                    w[3],
                    w[4],
                    w[5] as u64 | (w[6] as u64) << 32
                );
            } else if kind == event::CHANNEL_ERROR {
                // Channel error (DMKill): needs a DEVCTL 8 acknowledgement.
                dev_warn!(
                    dev,
                    "firmware event 8: {:08x} {:08x} {:08x} {:08x} {:08x}\n",
                    w[1],
                    w[2],
                    w[3],
                    w[4],
                    w[5]
                );
                self.channel_errors = self.channel_errors.wrapping_add(1);
            } else {
                dev_dbg!(dev, "firmware event {} flags {:#x}\n", kind, flags);
            }
            read = (read + 1) % RING_SLOTS;
        }
        mb();
        ring.state.w32(STATE_READ, producer)?;
        wmb();
        Ok(())
    }
}
