// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Observational copies of firmware statistics, independent of admission and power policy.
//!
//! The payload is deliberately unscaled until its units have been established for this
//! firmware. Readers never touch the firmware rings or take the device mutex.

use core::sync::atomic::{fence, AtomicU32, AtomicU64, Ordering};

pub(crate) const WORDS: usize = 18;
const FIRST_TAG: u32 = 16;
const TAGS: usize = 15;
const ROLES: usize = 2;
pub(crate) const CAPTURE_NS: u64 = 2_000_000_000;

struct Slot {
    sequence: AtomicU64,
    count: AtomicU64,
    received_ns: AtomicU64,
    words: [AtomicU32; WORDS],
}

impl Slot {
    const fn new() -> Self {
        Self {
            sequence: AtomicU64::new(0),
            count: AtomicU64::new(0),
            received_ns: AtomicU64::new(0),
            words: [const { AtomicU32::new(0) }; WORDS],
        }
    }
}

pub(crate) struct Statistics {
    capture_until_ns: AtomicU64,
    pub(crate) load: super::load::Load,
    slots: [Slot; ROLES * TAGS],
}

pub(crate) struct Report {
    pub(crate) count: u64,
    pub(crate) received_ns: u64,
    pub(crate) words: [u32; WORDS],
}

impl Statistics {
    pub(crate) const fn new() -> Self {
        Self {
            capture_until_ns: AtomicU64::new(0),
            load: super::load::Load::new(),
            slots: [const { Slot::new() }; ROLES * TAGS],
        }
    }

    /// A diagnostic read requests a bounded full-payload capture; normal load polling does not.
    pub(crate) fn request_capture(&self, now_ns: u64) {
        self.capture_until_ns
            .fetch_max(now_ns.saturating_add(CAPTURE_NS), Ordering::Relaxed);
    }

    pub(crate) fn capture_pending(&self) -> bool {
        self.capture_until_ns.load(Ordering::Relaxed) != 0
    }

    pub(crate) fn capture_enabled(&self, now_ns: u64) -> bool {
        let deadline = self.capture_until_ns.load(Ordering::Relaxed);
        if deadline == 0 {
            return false;
        }
        if now_ns < deadline {
            return true;
        }
        // A reader can renew the request concurrently with expiry.
        match self.capture_until_ns.compare_exchange(
            deadline,
            0,
            Ordering::Relaxed,
            Ordering::Relaxed,
        ) {
            Ok(_) => false,
            Err(renewed) => now_ns < renewed,
        }
    }

    /// Called only by the serialized firmware event worker, before returning the ring slot.
    pub(crate) fn record(&self, role: usize, received_ns: u64, words: &[u32; WORDS]) {
        let Some(tag) = words[0].checked_sub(FIRST_TAG) else {
            return;
        };
        if role >= ROLES || tag as usize >= TAGS {
            return;
        }
        self.load.record(role, received_ns, words);
        let slot = &self.slots[role * TAGS + tag as usize];
        slot.sequence.fetch_add(1, Ordering::AcqRel);
        for (out, value) in slot.words.iter().zip(words) {
            out.store(*value, Ordering::Relaxed);
        }
        slot.received_ns.store(received_ns, Ordering::Relaxed);
        slot.count.fetch_add(1, Ordering::Relaxed);
        slot.sequence.fetch_add(1, Ordering::Release);
    }

    /// A bounded, nonblocking snapshot. Contention is distinct from an unseen record.
    pub(crate) fn snapshot(&self, role: usize, tag: u32) -> Option<Report> {
        let tag = tag.checked_sub(FIRST_TAG)? as usize;
        if role >= ROLES || tag >= TAGS {
            return None;
        }
        let slot = &self.slots[role * TAGS + tag];
        for _ in 0..8 {
            let sequence = slot.sequence.load(Ordering::Acquire);
            if sequence & 1 != 0 {
                continue;
            }
            let report = Report {
                count: slot.count.load(Ordering::Relaxed),
                received_ns: slot.received_ns.load(Ordering::Relaxed),
                words: core::array::from_fn(|i| slot.words[i].load(Ordering::Relaxed)),
            };
            fence(Ordering::Acquire);
            if sequence == slot.sequence.load(Ordering::Relaxed) {
                return Some(report);
            }
        }
        None
    }
}
