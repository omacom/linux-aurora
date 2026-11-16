// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Bounded input-fence components. Collection happens before the device mutex,
//! because polling a container's status can signal it and run callbacks.

use kernel::{bindings, dma_fence::{Fence, RawDmaFence}, prelude::*};

#[repr(C)]
pub(super) struct Leaf {
    pub(super) fence: *mut bindings::dma_fence,
    pub(super) status: i32,
}

unsafe extern "C" {
    fn asahi_g17_collect_fences(head: *mut bindings::dma_fence,
        out: *mut Leaf, capacity: usize) -> i32;
    fn asahi_g17_put_fence(fence: *mut bindings::dma_fence);
}

impl Drop for Leaf {
    fn drop(&mut self) {
        // SAFETY: Successful collection transfers exactly one owned reference.
        unsafe { asahi_g17_put_fence(self.fence) };
    }
}

/// Allocation, unsupported containers, or a large prefix retain normal waits.
pub(super) fn collect(input: &Fence) -> Option<KVec<Leaf>> {
    let mut capacity = 4;
    loop {
        let mut leaves = KVec::with_capacity(capacity, GFP_KERNEL).ok()?;
        // SAFETY: The input owns its fence. The C helper writes at most capacity
        // fully initialized records and releases every reference on failure.
        let count = unsafe {
            asahi_g17_collect_fences(input.raw(), leaves.as_mut_ptr(), capacity)
        };
        if count == ENOSPC.to_errno() && capacity < 128 {
            capacity = (capacity * 4).min(128);
            continue;
        }
        if count < 0 { return None; }
        // SAFETY: The helper returned this many initialized, owned records.
        unsafe { leaves.set_len(count as usize) };
        return Some(leaves);
    }
}


use super::fence::Submission;
use core::sync::atomic::{AtomicU32, Ordering};
use kernel::{new_mutex, sync::{Arc, Mutex}};

pub(super) struct Source {
    pub(super) submission: Arc<Submission>,
}

/// The counter is independent of a physical queue's allocation and survives
/// processor stop until the last consumer releases its binding loan.
pub(super) struct Hold(Arc<AtomicU32>);
impl Hold {
    fn new(readers: &Arc<AtomicU32>) -> Result<Self> {
        readers.fetch_update(Ordering::AcqRel, Ordering::Acquire,
            |count| count.checked_add(1)).map_err(|_| EOVERFLOW)?;
        Ok(Self(readers.clone()))
    }
}
impl Drop for Hold {
    fn drop(&mut self) { self.0.fetch_sub(1, Ordering::AcqRel); }
}

/// A new owner starts with no loans. Acquiring a loan requires exclusive access
/// and arms the hint; eligibility queries stay read-only. A validated owner
/// transition can clear the hint after all independent loans have been released.
#[repr(transparent)]
pub(super) struct ReaderHint {
    possible: bool,
}
impl ReaderHint {
    pub(super) const fn new() -> Self {
        Self { possible: false }
    }
    pub(super) fn hold(&mut self, count: &Arc<AtomicU32>) -> Result<Hold> {
        let hold = Hold::new(count)?;
        self.possible = true;
        Ok(hold)
    }
    pub(super) fn has_readers(&self, count: &AtomicU32) -> bool {
        self.possible && count.load(Ordering::Acquire) != 0
    }
    pub(super) fn reset_for_owner(&mut self, count: &AtomicU32) -> Result {
        if self.has_readers(count) { return Err(EBUSY); }
        self.possible = false;
        Ok(())
    }
}

#[pin_data]
pub(super) struct Sources {
    inputs: KVec<Fence>,
    publication_waits: KVec<Fence>,
    pub(super) sources: KVec<Source>,
    #[pin]
    holds: Mutex<KVec<Hold>>,
}
impl Sources {
    pub(super) fn new(inputs: KVec<Fence>, sources: KVec<Source>,
        publication_waits: KVec<Fence>) -> Result<Arc<Self>> {
        CONVERTED.fetch_add(1, Ordering::Relaxed);
        if !publication_waits.is_empty() { PENDING.fetch_add(1, Ordering::Relaxed); }
        Arc::pin_init(pin_init!(Self {
            inputs, sources, publication_waits,
            holds <- new_mutex!(KVec::new(), "G17 source binding loans"),
        }), GFP_KERNEL)
    }
    pub(super) fn contains_input(&self, input: &Fence) -> bool {
        self.inputs.iter().any(|fence| fence.raw() == input.raw())
    }
    pub(super) fn publication_waits(&self) -> &[Fence] { &self.publication_waits }
    pub(super) fn check(&self) -> Result {
        for source in &self.sources { source.submission.source_status()?; }
        Ok(())
    }
    pub(super) fn set_holds(&self, holds: KVec<Hold>) {
        let old = core::mem::replace(&mut *self.holds.lock(), holds);
        drop(old);
    }
    pub(super) fn release(&self) { self.set_holds(KVec::new()); }
}


static CONVERTED: core::sync::atomic::AtomicU64 = core::sync::atomic::AtomicU64::new(0);
static PUBLISHED: core::sync::atomic::AtomicU64 = core::sync::atomic::AtomicU64::new(0);
static PENDING: core::sync::atomic::AtomicU64 = core::sync::atomic::AtomicU64::new(0);
pub(super) fn pending_count() -> u64 { PENDING.load(Ordering::Relaxed) }
pub(super) fn note_published() { PUBLISHED.fetch_add(1, Ordering::Relaxed); }
pub(super) fn counters() -> (u64, u64) {
    (CONVERTED.load(Ordering::Relaxed), PUBLISHED.load(Ordering::Relaxed))
}

/// A revalidated compute completion dependency for a direct work descriptor.
#[derive(Clone, Copy)]
pub(super) struct NativeWait {
    pub(super) sequence: u64,
    pub(super) dependency: super::fw::kick::KickDependency,
    pub(super) stamp_va: u64,
    pub(super) event_slot: u32,
}
impl NativeWait {
    pub(super) fn bytes(&self, consumer_stamp: u32, uuid: u64) -> [u8; 0x48] {
        let mut out = [0; 0x48];
        let timestamp = self.dependency.word() & ((1u64 << 40) - 1);
        for (offset, value) in [
            (0, 4u32),
            (0x14, u32::from(self.dependency.qid())),
            (0x18, super::fw::stamp_value(timestamp as u32)),
            (0x24, self.event_slot),
            (0x28, consumer_stamp),
            (0x34, 1),
        ] {
            out[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
        }
        for (offset, value) in [(4, self.stamp_va), (0xc, timestamp), (0x2c, uuid)] {
            out[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
        }
        out
    }
}

static NATIVE: core::sync::atomic::AtomicU64 = core::sync::atomic::AtomicU64::new(0);
pub(super) fn note_native(count: usize) { NATIVE.fetch_add(count as u64, Ordering::Relaxed); }
pub(super) fn native_count() -> u64 { NATIVE.load(Ordering::Relaxed) }

#[kunit_tests(asahi_g17_input_wait)]
mod tests {
    use super::*;
    use crate::g17::fw::kick::{KickDependency, KickTimestamp};

    #[test]
    fn completion_record_preserves_distinct_event_and_queue_domains() {
        let wait = NativeWait {
            sequence: 1,
            dependency: KickDependency::new(37,
                KickTimestamp::new(0x12_3456_789a).unwrap()).unwrap(),
            stamp_va: 0x1122_3344_5566_7788,
            event_slot: 7,
        };
        // Protocol fixture includes the unaligned stamp pointer, full 40-bit
        // timestamp, wrapped 24-bit stamp and independent legacy event slot.
        let expected: [u8; 0x48] = [
            4, 0, 0, 0, 0x88, 0x77, 0x66, 0x55,
            0x44, 0x33, 0x22, 0x11, 0x9a, 0x78, 0x56, 0x34,
            0x12, 0, 0, 0, 37, 0, 0, 0,
            0, 0x9a, 0x78, 0x56, 0, 0, 0, 0,
            0, 0, 0, 0, 7, 0, 0, 0,
            0, 0xcc, 0xbb, 0xaa, 8, 7, 6, 5,
            4, 3, 2, 1, 1, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0,
        ];
        assert_eq!(wait.bytes(0xaabb_cc00, 0x0102_0304_0506_0708), expected);
    }

    #[test]
    fn failed_loan_acquisition_never_wraps_or_releases_another_reader() {
        let readers = Arc::new(AtomicU32::new(u32::MAX - 1), GFP_KERNEL).unwrap();
        let last = Hold::new(&readers).unwrap();
        assert!(matches!(Hold::new(&readers), Err(error) if error == EOVERFLOW));
        assert_eq!(readers.load(Ordering::Acquire), u32::MAX);
        drop(last);
        assert_eq!(readers.load(Ordering::Acquire), u32::MAX - 1);
    }

    #[test]
    fn loan_outlives_original_queue_counter_reference() {
        let readers = Arc::new(AtomicU32::new(0), GFP_KERNEL).unwrap();
        let mut hint = ReaderHint::new();
        let first = hint.hold(&readers).unwrap();
        let second = hint.hold(&readers).unwrap();
        drop(readers);
        assert_eq!(second.0.load(Ordering::Acquire), 2);
        drop(first);
        assert_eq!(second.0.load(Ordering::Acquire), 1);
        drop(second);
    }

    #[test]
    fn reader_guard_tracks_partial_release_and_rearms_after_zero() {
        let readers = Arc::new(AtomicU32::new(0), GFP_KERNEL).unwrap();
        let mut hint = ReaderHint::new();
        assert!(!hint.has_readers(&readers));
        let first = hint.hold(&readers).unwrap();
        let second = hint.hold(&readers).unwrap();
        assert!(hint.has_readers(&readers));
        assert!(matches!(hint.reset_for_owner(&readers), Err(error) if error == EBUSY));
        drop(first);
        assert!(hint.has_readers(&readers));
        drop(second);
        assert!(!hint.has_readers(&readers));
        hint.reset_for_owner(&readers).unwrap();
        let next = hint.hold(&readers).unwrap();
        assert!(hint.has_readers(&readers));
        drop(next);
        assert!(!hint.has_readers(&readers));
    }
}
