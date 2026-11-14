// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Pending capacity claims for M5's per-priority shared outer ring. Firmware
//! consumption frees posted slots; RAII cancellation frees unposted claims.

use crate::g17::{availability::Availability, fw::channels::WORK_RING_SLOTS};
use core::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};
use kernel::{prelude::*, sync::Arc};

const PRIORITIES: usize = 4;

pub(super) struct Credits {
    reserved: [AtomicU32; PRIORITIES],
    published: [AtomicU64; PRIORITIES],
    consumers: [AtomicU64; PRIORITIES],
    scoped: AtomicBool,
    available: Arc<Availability>,
}

pub(crate) struct OuterClaim {
    domain: Arc<Credits>,
    priority: usize,
    remaining: u32,
}

/// Borrows the claim only. The caller must hold the device mutex from entry
/// through Drop: its temporarily lent credits are available to that publisher.
pub(crate) struct OuterScope<'a> {
    claim: &'a mut OuterClaim,
    before: u64,
}

pub(super) fn free([a, b, producer]: [u32; 3]) -> Result<u32> {
    if [a, b, producer].iter().any(|&v| v >= WORK_RING_SLOTS) {
        return Err(EINVAL);
    }
    Ok(((a + WORK_RING_SLOTS - producer - 1) % WORK_RING_SLOTS)
        .min((b + WORK_RING_SLOTS - producer - 1) % WORK_RING_SLOTS))
}

impl Credits {
    pub(super) fn new() -> Result<Arc<Self>> {
        Ok(Arc::new(Self {
            reserved: [const { AtomicU32::new(0) }; PRIORITIES],
            published: [const { AtomicU64::new(0) }; PRIORITIES],
            consumers: [const { AtomicU64::new(u64::MAX) }; PRIORITIES],
            scoped: AtomicBool::new(false),
            available: Availability::new()?,
        }, GFP_KERNEL)?)
    }

    pub(super) fn available(&self) -> Arc<Availability> { self.available.clone() }

    pub(super) fn waiting(&self) -> bool { self.available.has_waiters() }

    pub(super) fn capacity(&self, priority: u8, cursors: [u32; 3]) -> Result<u32> {
        let pending = self.reserved.get(usize::from(priority)).ok_or(EINVAL)?
            .load(Ordering::Acquire);
        Ok(free(cursors)?.saturating_sub(pending))
    }

    /// Reserve under the sole publisher mutex. Off-lock cancellation may only
    /// reduce reserved counts, so the sampled capacity is conservative.
    pub(super) fn reserve(self: &Arc<Self>, priority: u8, count: u32,
        cursors: [u32; 3]) -> Result<OuterClaim>
    {
        if count == 0 || count >= WORK_RING_SLOTS { return Err(EINVAL); }
        if self.scoped.load(Ordering::Acquire) { return Err(EBUSY); }
        let index = usize::from(priority);
        let counter = self.reserved.get(index).ok_or(EINVAL)?;
        let seen = (u64::from(cursors[0]) << 32) | u64::from(cursors[1]);
        self.consumers[index].compare_exchange(u64::MAX, seen, Ordering::AcqRel, Ordering::Acquire).ok();
        if self.capacity(priority, cursors)? < count { return Err(EBUSY); }
        counter.fetch_add(count, Ordering::AcqRel);
        Ok(OuterClaim { domain: self.clone(), priority: index, remaining: count })
    }

    pub(super) fn scope<'a>(self: &Arc<Self>, claim: &'a mut OuterClaim) -> Result<OuterScope<'a>> {
        if !Arc::ptr_eq(self, &claim.domain) { return Err(EINVAL); }
        if self.scoped.swap(true, Ordering::AcqRel) { return Err(EBUSY); }
        self.reserved[claim.priority].fetch_sub(claim.remaining, Ordering::AcqRel);
        let before = self.published[claim.priority].load(Ordering::Acquire);
        Ok(OuterScope { claim, before })
    }

    pub(super) fn counter(&self, priority: u8) -> Result<&AtomicU64> {
        self.published.get(usize::from(priority)).ok_or(EINVAL)
    }

    pub(super) fn progressed(&self, priority: u8, cursors: [u32; 3]) -> Result<bool> {
        free(cursors)?;
        let counter = self.consumers.get(usize::from(priority)).ok_or(EINVAL)?;
        let current = (u64::from(cursors[0]) << 32) | u64::from(cursors[1]);
        let previous = counter.swap(current, Ordering::AcqRel);
        Ok(previous != u64::MAX && previous != current)
    }
}

impl Drop for OuterScope<'_> {
    fn drop(&mut self) {
        let domain = &self.claim.domain;
        let published = domain.published[self.claim.priority].load(Ordering::Acquire)
            .wrapping_sub(self.before);
        // Every publication also checks unreserved physical capacity. If an
        // enclosing transaction posts extra records, only this claim's own
        // credits are consumed here; another claim can never be debited.
        self.claim.remaining -= published.min(u64::from(self.claim.remaining)) as u32;
        domain.reserved[self.claim.priority].fetch_add(self.claim.remaining, Ordering::AcqRel);
        domain.scoped.store(false, Ordering::Release);
    }
}

impl OuterClaim {
    /// Credits not yet consumed by a publication. A claim with none left can be dropped
    /// anywhere: only returned credits notify waiters.
    pub(crate) fn remaining(&self) -> u32 {
        self.remaining
    }
}

impl Drop for OuterClaim {
    /// As for physical queue claims, drop outside the device mutex because
    /// availability notification can run DMA-fence callbacks.
    fn drop(&mut self) {
        if self.remaining != 0 {
            self.domain.reserved[self.priority].fetch_sub(self.remaining, Ordering::AcqRel);
            self.domain.available.notify();
        }
    }
}
