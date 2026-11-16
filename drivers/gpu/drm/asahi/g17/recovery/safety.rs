// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Fail closed on repeated watchdog/restart traffic, even across client loss.

use core::sync::atomic::{AtomicU64, Ordering};

const WINDOW_NS: u64 = 30_000_000_000;
const LIMIT: u8 = 8;

/// Commands retired successfully. A watchdog storm means the GPU stopped making progress;
/// timeouts of overloaded clients while other work keeps retiring are not one.
static RETIRED: AtomicU64 = AtomicU64::new(0);
/// [`crate::g17::now_ns`] of the latest successful retirement, zero before the first.
static LAST_RETIRED_NS: AtomicU64 = AtomicU64::new(0);

/// Called for every command that retires without error.
pub(crate) fn note_retired() {
    RETIRED.fetch_add(1, Ordering::Relaxed);
    LAST_RETIRED_NS.store(crate::g17::now_ns(), Ordering::Relaxed);
}

pub(super) fn retired() -> u64 {
    RETIRED.load(Ordering::Relaxed)
}

/// Nanoseconds since any command last retired successfully, or `None` before the first.
pub(crate) fn since_retired(now: u64) -> Option<u64> {
    match LAST_RETIRED_NS.load(Ordering::Relaxed) {
        0 => None,
        last => Some(now.saturating_sub(last)),
    }
}

pub(super) struct Storm {
    start: Option<u64>,
    count: u8,
    stopped: bool,
    progress: u64,
}
impl Storm {
    pub(super) const fn new() -> Self {
        Self { start: None, count: 0, stopped: false, progress: 0 }
    }
    /// Like [`Self::note`], but work retired since the previous event starts a new window.
    pub(super) fn note_unless_progress(&mut self, now: u64, progress: u64) -> bool {
        if !self.stopped && progress != self.progress {
            self.progress = progress;
            self.start = None;
        }
        self.note(now)
    }
    pub(super) fn note(&mut self, now: u64) -> bool {
        if self.stopped { return true; }
        if self.start.is_none_or(|start| now.saturating_sub(start) >= WINDOW_NS) {
            self.start = Some(now);
            self.count = 0;
        }
        self.count += 1;
        self.stopped = self.count >= LIMIT;
        self.stopped
    }
}

#[kernel::macros::kunit_tests(asahi_g17_timeout_recovery_safety)]
mod tests {
    use super::*;
    use crate::g17::{preparation::Gate, recovery::State};
    use kernel::prelude::*;

    #[test]
    fn consecutive_timeouts_during_recovery_are_bounded_and_lock_ordered() {
        let gate = Gate::new().unwrap();
        let lease = gate.try_enter().unwrap();
        let state = KBox::pin_init(kernel::new_mutex!(State::new()), GFP_KERNEL).unwrap();
        gate.close();
        for n in 0..64 {
            let stopped = {
                let mut state = state.lock();
                state.request(0, -1).unwrap();
                assert!(state.pending());
                let stopped = state.timeout_limit(n * 2_000_000_000);
                if stopped { gate.fail(); } // production state -> gate order
                stopped
            };
            assert_eq!(stopped, n >= u64::from(LIMIT - 1));
        }
        // Recovery/stop must release state before joining the retained builder.
        assert!(matches!(gate.wait_drained_until(crate::g17::now_ns() + 2_000_000),
            Err(error) if error == ETIMEDOUT));
        assert!(!gate.reopen()); // failure never gives back an owned reservation
        drop(lease);
        gate.wait_drained_until(crate::g17::now_ns() + 100_000_000).unwrap();
        assert!(!gate.reopen());
    }
    #[test]
    fn successful_handshakes_do_not_reset_a_restart_storm() {
        let mut state = State::new();
        for n in 0..64 {
            assert_eq!(state.restart_limit(n), n >= u64::from(LIMIT - 1));
        }
        assert!(state.restart_limit(u64::MAX));
    }
    #[test]
    fn separated_faults_do_not_accumulate_and_latched_loss_cannot_wrap() {
        let mut storm = Storm::new();
        for n in 0..64 { assert!(!storm.note(n * WINDOW_NS)); }
        for _ in 1..LIMIT { storm.note(64 * WINDOW_NS); }
        assert!(storm.note(64 * WINDOW_NS));
        for _ in 0..1024 { assert!(storm.note(u64::MAX)); }
    }
    #[test]
    fn recovery_table_ends_at_its_first_invalid_record() {
        use crate::g17::recovery::Entries;
        let mut entries = Entries::default();
        assert!(entries.include(1 | (12 << 4)));
        assert!(entries.include(1 | 2 | (17 << 4)));
        assert!(!entries.include(0));
        assert_eq!(entries.listed, 2);
        assert_eq!(entries.guilty, 1u128 << 17);
        assert_eq!(entries.cancelled, 0);
    }
    #[test]
    fn replay_dependency_checks_never_wait_for_event_service() {
        use crate::g17::dependency::replay_status;
        assert_eq!(replay_status(0), Err(EBUSY));
        assert_eq!(replay_status(1), Ok(()));
        assert_eq!(replay_status(EIO.to_errno()), Err(ECANCELED));
        assert_eq!(replay_status(ETIMEDOUT.to_errno()), Err(ECANCELED));
    }
}
