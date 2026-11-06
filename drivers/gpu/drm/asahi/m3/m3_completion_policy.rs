// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Bounded fallback for witnesses which can become visible after the last IRQ.
//!
//! Notifications request a fresh snapshot, but do not prove witness progress.
//! Only publication or retirement resets this policy. In particular, neither a
//! timer wake nor a stream of early firmware notifications renews a spin budget.

pub(crate) struct WaitPolicy {
    next_us: u64,
}

impl WaitPolicy {
    pub(crate) const fn new() -> Self {
        Self { next_us: 25 }
    }

    pub(crate) fn progress(&mut self) {
        self.next_us = 25;
    }

    /// No deliberate busy delay: IRQs still wake each sleep immediately.
    /// The caller supplies the root-selected ceiling (25..=1000 us).
    pub(crate) fn next_wait_us(&mut self, ceiling_us: u64) -> u64 {
        let wait = self.next_us.min(ceiling_us.clamp(25, 1000));
        self.next_us = (wait * 2).min(1000);
        wait
    }
}

#[cfg_attr(not(test), kernel::macros::kunit_tests(asahi_m3_completion_policy))]
mod tests {
    use super::WaitPolicy;

    #[test]
    fn no_progress_backs_off_and_stays_bounded() {
        let mut policy = WaitPolicy::new();
        for expected in [25, 50, 100, 200, 400, 800, 1000, 1000] {
            assert_eq!(policy.next_wait_us(1000), expected);
        }
        for _ in 0..100_000 {
            assert_eq!(policy.next_wait_us(1000), 1000);
        }
    }

    #[test]
    fn publication_or_retirement_restarts_fast_detection() {
        let mut policy = WaitPolicy::new();
        for _ in 0..8 { policy.next_wait_us(1000); }
        policy.progress();
        assert_eq!(policy.next_wait_us(1000), 25);
        assert_eq!(policy.next_wait_us(1000), 50);
    }

    #[test]
    fn live_ceiling_changes_apply_to_the_next_wait() {
        let mut policy = WaitPolicy::new();
        for _ in 0..8 { policy.next_wait_us(1000); }
        assert_eq!(policy.next_wait_us(25), 25);
        assert_eq!(policy.next_wait_us(1000), 50);
        assert_eq!(policy.next_wait_us(100), 100);
        assert_eq!(policy.next_wait_us(100), 100);
    }

    #[test]
    fn fallback_keeps_timeout_checks_frequent_without_any_irq() {
        let mut policy = WaitPolicy::new();
        let mut elapsed_us = 0;
        while elapsed_us < 2_000_000 {
            let wait = policy.next_wait_us(1000);
            assert!(wait <= 1000);
            elapsed_us += wait;
        }
        // This bounds the requested delay, not scheduler dispatch latency.
        assert!(elapsed_us < 2_001_000);
    }
}
