// SPDX-License-Identifier: GPL-2.0-only OR MIT
//! Host-verified retirement counters, independent of logs and firmware IRQs.
//!
//! One serialized runtime writer calls record_completion only after proving
//! retirement. Readers never touch MMIO, allocate, or acquire the runtime lock.
use core::sync::atomic::{AtomicU64, Ordering};
use kernel::time::{ClockSource, Monotonic};

pub(crate) struct Progress {
    // Odd epochs conservatively mean potentially outstanding device work.
    // Start busy until boot has proved that its initial work is retired.
    activity_epoch: AtomicU64,
    completed: AtomicU64,
    last_completion_ns: AtomicU64,
    generation_ns: u64,
}
impl Progress {
    pub(crate) fn new() -> Self {
        Self { activity_epoch: AtomicU64::new(1), completed: AtomicU64::new(0), last_completion_ns: AtomicU64::new(0),
            generation_ns: Monotonic::ktime_get() as u64 }
    }
    /// Serialized runtime writer: set before any publication can reach firmware,
    /// clear only after all published work has proven retirement. An error must
    /// never clear this state merely because a software queue was discarded.
    pub(crate) fn set_pending(&self, pending: bool) {
        let old = self.activity_epoch.load(Ordering::Relaxed);
        if (old & 1 != 0) != pending {
            // Exhaustion remains permanently busy, never wraps back to idle.
            self.activity_epoch.store(old.saturating_add(1), Ordering::Release);
            // PMP CPU memory floor: report the same idle/busy edge. The
            // callee takes only a spinlock; it is a no-op unless a report
            // with apple,cpu-memory-floor admitted an owner.
            #[cfg(CONFIG_APPLE_PMP_REPORT)]
            // SAFETY: callable in any context but NMI; takes no GPU state.
            unsafe { kernel::bindings::apple_pmp_floor_gpu_activity(pending) };
            if pending {
                // Order the new marker before device queue publication. While
                // already pending there is no new idle-to-busy state to order.
                core::sync::atomic::fence(Ordering::SeqCst);
            }
        }
    }
    pub(crate) fn activity_snapshot(&self) -> (u64, u64) {
        (self.generation_ns, self.activity_epoch.load(Ordering::Acquire))
    }
    pub(crate) fn record_completion(&self) {
        // A reader observing a new count sees this timestamp or a later REAL
        // completion. A timestamp alone must never authorize watchdog renewal.
        self.last_completion_ns.store(Monotonic::ktime_get() as u64, Ordering::Release);
        self.completed.fetch_add(1, Ordering::Release);
    }
    pub(crate) fn snapshot(&self) -> (u64, u64, u64) {
        let count = self.completed.load(Ordering::Acquire);
        let timestamp = self.last_completion_ns.load(Ordering::Acquire);
        (self.generation_ns, count, timestamp)
    }
}
