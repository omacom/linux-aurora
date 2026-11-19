// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Two render queues sharing one outer ring. Capacity and both destinations
//! are checked before the host-port transaction can make either queue runnable.

use crate::g17::fw::channels::{WorkSlot, WORK_RING_SLOTS};
use core::sync::atomic::{fence, AtomicU32, AtomicU64, Ordering};
use kernel::prelude::*;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) struct Window {
    pub(super) start: u32,
    pub(super) next: [u32; 2],
}

impl Window {
    pub(super) fn new([consumer, consumer_b, start]: [u32; 3]) -> Result<Self> {
        if [consumer, consumer_b, start].iter().any(|&p| p >= WORK_RING_SLOTS) {
            return Err(EINVAL);
        }
        let next = [(start + 1) % WORK_RING_SLOTS, (start + 2) % WORK_RING_SLOTS];
        if next.iter().any(|&p| p == consumer || p == consumer_b) {
            return Err(EBUSY);
        }
        Ok(Self { start, next })
    }
}

pub(in crate::g17) struct Publication<'a> {
    published: Option<&'a AtomicU64>,
    records: [WorkSlot; 2],
    destinations: [*mut u8; 2],
    producer: &'a AtomicU32,
    window: Window,
}

impl<'a> Publication<'a> {
    /// SAFETY: Both destinations must identify distinct, writable WorkSlot
    /// storage in the retained ring at window.start and window.next[0]. The
    /// caller holds the sole host publication lock until this value is dropped.
    pub(super) unsafe fn new(
        records: [WorkSlot; 2],
        destinations: [*mut u8; 2],
        producer: &'a AtomicU32,
        window: Window,
        published: Option<&'a AtomicU64>,
    ) -> Self {
        Self { records, destinations, producer, window, published }
    }

    pub(in crate::g17) fn next(&self) -> [u32; 2] { self.window.next }

    pub(in crate::g17) fn prepare(&self) {
        for (record, destination) in self.records.iter().zip(self.destinations) {
            // SAFETY: Constructor checked both live destinations. Records are
            // initialized, padding-free integer structures owned by this value.
            unsafe {
                core::ptr::copy_nonoverlapping(
                    (record as *const WorkSlot).cast::<u8>(), destination, size_of::<WorkSlot>(),
                );
            }
        }
    }

    /// Publish both records with one cursor update, after both inner queues and
    /// their accounting are visible. No fallible operation follows the port write.
    pub(in crate::g17) fn publish(self) -> [u32; 2] {
        self.prepare();
        fence(Ordering::SeqCst);
        self.producer.store(self.window.next[1], Ordering::Relaxed);
        if let Some(published) = self.published { published.fetch_add(2, Ordering::Release); }
        fence(Ordering::SeqCst);
        self.window.next
    }
}
