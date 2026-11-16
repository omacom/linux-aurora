// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Event service ordering under the device mutex. A restart ends its role's batch; a worker
//! joins preparation leases and services it on a separate turn before continuing the rings.

use super::{
    fw::channels::{Event, FreeListCompletion, TvbGrowRequest},
    recovery, Role,
};
use kernel::prelude::*;

/// The bounded shared-ring transport; recognized malformed entries return errors after consume.
pub(super) trait Transport {
    /// The firmware instances of this GPU: the primary, and the secondary where one exists.
    fn roles(&self) -> &'static [Role];
    fn trace(&mut self, role: Role);
    fn statistics(&mut self, role: Role);
    fn drain(
        &mut self,
        role: Role,
        consume: impl FnMut(&mut Self, Event) -> Result<bool>,
    ) -> Result;
    fn recovery_state(&self) -> Result<u32>;
}

/// Runtime callbacks with the device mutex held. None signals fences or allocates grow backing.
pub(super) trait Queues {
    /// Flush the oldest deferred reply/release. False means ring backpressure; each pending
    /// transaction retains its own bounded worker-attempt count across service turns.
    fn flush_tvb_reply(&mut self) -> Result<bool>;
    fn flush_freelist_release(&mut self) -> Result<bool>;
    fn observe_render_releases(&mut self) -> Result;
    fn control_backpressured(&self) -> bool;
    fn queue_tvb_grow(&mut self, request: TvbGrowRequest) -> Result;
    fn complete_freelist_grow(&mut self, completion: FreeListCompletion) -> Result;
    fn retain_render_completions(&mut self, masks: [u64; 2]);
    fn complete_compute(&mut self, masks: [u64; 2]) -> Result;
    fn context_killed(&mut self, cookie: u64);
    /// The firmware failed a compute command (see [`Event::ComputeFault`]).
    fn compute_fault(&mut self, qid: u8, stamp: u32);
}

/// Work requested after releasing the device mutex.
#[derive(Default)]
pub(super) struct Pending {
    pub(super) recovery: bool,
}

/// Access to the single recovery owner while event handlers borrow the device.
pub(super) trait Host: Transport + Queues {
    fn recovery(&self) -> &recovery::State;
    fn recovery_mut(&mut self) -> &mut recovery::State;
}

fn batch<H: Host>(host: &mut H, role: Role, masks: &mut [u64; 2]) -> Result {
    host.drain(role, |host, event| {
        match event {
            Event::Stamp(bits) => {
                masks[0] |= bits[0];
                masks[1] |= bits[1];
            }
            Event::Restart { generation, stamp } => {
                if host.recovery_mut().request(generation, stamp)? {
                    return Ok(false);
                }
            }
            Event::TvbGrow(request) => {
                if request.halt_count == host.recovery().generation() {
                    host.queue_tvb_grow(request)?;
                }
                if host.control_backpressured() {
                    return Ok(false);
                }
            }
            Event::FreeListGrow(completion) => host.complete_freelist_grow(completion)?,
            Event::ContextKilled(cookie) => host.context_killed(cookie),
            Event::ComputeFault { qid, stamp } => {
                crate::g17::trace::teardown(1, u64::from(qid), u64::from(stamp), 0);
                host.compute_fault(qid, stamp);
                // Its queue retires or settles its quarantine on this pass.
                masks[usize::from(qid) / 64] |= 1 << (qid % 64);
            }
            Event::Unknown(_) => {}
        }
        Ok(true)
    })
}

pub(super) fn service<H: Host>(host: &mut H) -> Result<Pending> {
    if !host.flush_tvb_reply()? || !host.flush_freelist_release()? {
        return Ok(Pending::default());
    }
    host.observe_render_releases()?;
    for &role in host.roles() {
        host.trace(role);
    }
    for &role in host.roles() {
        host.statistics(role);
    }
    let mut masks = [0; 2];
    if !host.recovery().pending() {
        let mut settled = false;
        // The firmware publishes restart before state 1, but its producer can race the snapshot.
        for _ in 0..256 {
            batch(host, Role::Primary, &mut masks)?;
            if host.recovery().pending() || host.control_backpressured() {
                settled = true;
                break;
            }
            match host.recovery_state()? {
                0 => {
                    settled = true;
                    break;
                }
                1 => {}
                _ => return Err(EIO),
            }
        }
        if !settled {
            return Err(EIO);
        }
    }
    if !host.control_backpressured() && host.roles().contains(&Role::Secondary) {
        batch(host, Role::Secondary, &mut masks)?;
    }
    host.retain_render_completions(masks);
    host.complete_compute(masks)?;
    Ok(Pending {
        recovery: host.recovery().pending(),
    })
}
