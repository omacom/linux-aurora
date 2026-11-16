// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Bounded isolation for a compute watchdog blocked behind another VM's older kick.
//! Publication order is evidence of a possible blocker, not proof of a firmware
//! dependency. Renewals are bounded; the oldest unfinished queue is never excused.

pub(crate) const MAX_RENEWALS: u8 = 8;

/// A borrowed render graph or queued off-lock retirement is not evidence of
/// progress. Share the finite deferral bound with cross-VM attribution.
pub(crate) fn transaction_grace(renewals: u8) -> bool { renewals < MAX_RENEWALS }

/// Keep the scheduler poll, control-ring waits and ordinary jobs at two seconds.
/// Native ray traversal and AS builds can exceed one poll (CTS stress). Four
/// extra polls give it ten seconds, still one tenth of the legacy scheduler's
/// fallback. This is an execution allowance, not evidence of GPU progress, and
/// is consumed once per scheduler job, independently of cross-VM/replay grace.
pub(crate) const POLL_MS: u32 = 2000;
const EXECUTION_RENEWALS: u8 = 4;

/// A background control/replay poll has no scheduler fence to expire it.
/// Its timestamp belongs to the exact transaction and is never renewed by work.
pub(crate) fn background_stalled(since: Option<u64>, now: u64) -> bool {
    since.is_some_and(|since| now.saturating_sub(since) >= 10_000_000_000)
}

/// Input comes only from fully validated commands. Render metadata covers the
/// VDM half (vertex/tessellation/geometry) and the fragment half independently.
/// AS builders do not traverse rays and need an explicit, bounded UAPI hint.
pub(crate) fn select(compute_ray: bool, render_ray: [bool; 2], as_build: bool) -> bool {
    compute_ray || render_ray.into_iter().any(|enabled| enabled) || as_build
}

/// Firmware dependencies can hold an untagged helper behind a long command on
/// the other engine. Classify the complete validated ioctl before enqueueing
/// anything; no state or allowance leaks into a later submission.
pub(crate) fn select_submission(commands: impl Iterator<Item = bool>) -> bool {
    commands.into_iter().any(|enabled| enabled)
}

/// Only the explicit AS-build bit is newly accepted; reserved and unrelated
/// flags remain invalid on G17P. Return None before any command is published.
pub(crate) fn as_build(flags: u32) -> Option<bool> {
    const AS_BUILD: u32 = 1 << 2;
    if flags & !AS_BUILD != 0 { None } else { Some(flags & AS_BUILD != 0) }
}

pub(crate) fn renew_execution(published: bool, enabled: bool, renewals: &mut u8) -> bool {
    if !published || !enabled || *renewals >= EXECUTION_RENEWALS {
        return false;
    }
    *renewals += 1;
    true
}

/// Polls a published job may be renewed while the GPU keeps retiring work. A job
/// shares the GPU's cores with every other runnable queue, so its wall time grows
/// with the other clients' work: beside four other clients a 1.5 s dispatch took up
/// to 5 s, and lower-priority work waits behind higher-priority work. It has not
/// hung while the GPU keeps completing commands. Fourteen renewals bound the wait at
/// thirty seconds, after which a job that still has not finished fails as before.
const SHARED_RENEWALS: u8 = 14;

/// The GPU counts as live while some command retired this recently. Under heavy
/// sharing the longest gap between retirements measured 1.2 s; a GPU that retires
/// nothing for this long has stopped.
pub(crate) const LIVENESS_NS: u64 = 5_000_000_000;

/// Whether some command retired within [`LIVENESS_NS`] (`since_retired`).
pub(crate) fn retiring(since_retired: Option<u64>) -> bool {
    since_retired.is_some_and(|since| since < LIVENESS_NS)
}

/// Renews a published job's deadline while the GPU is `shared`, at most
/// [`SHARED_RENEWALS`] times. It is shared while some command retires
/// ([`retiring`]), or while a long job of another queue runs beside a long job:
/// two ray traversal dispatches of about 6 s each took 10 to 17 s side by side, and
/// neither retires anything until it ends.
pub(crate) fn renew_shared(published: bool, shared: bool, renewals: &mut u8) -> bool {
    if !published || !shared || *renewals >= SHARED_RENEWALS {
        return false;
    }
    *renewals += 1;
    true
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct Head {
    /// The owning VM's [`crate::mmu::Vm::id`], one per VM status; the watchdog lines print it.
    pub(crate) vm: u64,
    pub(crate) published: u64,
    pub(crate) finished: bool,
    /// Its queue already failed (quarantined) and holds the GPU only until the firmware drains
    /// it. No later watchdog of its own will settle it.
    pub(crate) failed: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Decision {
    Own,
    Defer,
    Spare,
}

/// Another VM's older unfinished work that can block `own`: live work before work whose queue
/// already failed, the oldest of each first.
pub(crate) fn blocker(own: Head, others: impl Iterator<Item = Head>) -> Option<Head> {
    others
        .filter(|other| other.vm != own.vm && !other.finished && other.published < own.published)
        .min_by_key(|other| (other.failed, other.published))
}

/// Work behind another VM's older unfinished work is renewed up to MAX_RENEWALS times, so that
/// the blocker's own failure and teardown can free the GPU. After that it is spared only behind
/// a live blocker, which its own watchdog still settles. A blocker that already failed waits only
/// for the firmware to drain it and no longer excuses later work: otherwise an innocent frame
/// published before the culprit, which timed out first, would spare the culprit, and a pair that
/// never drains would spare every later timeout.
pub(crate) fn decide(own: Head, renewals: u8, blocker: Option<Head>) -> Decision {
    match blocker {
        _ if own.finished => Decision::Own,
        None => Decision::Own,
        Some(_) if renewals < MAX_RENEWALS => Decision::Defer,
        Some(blocker) if blocker.failed => Decision::Own,
        Some(_) => Decision::Spare,
    }
}

/// A watchdog's decision with the heads it rests on, for its log line.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct Verdict {
    pub(crate) decision: Decision,
    pub(crate) own: Head,
    pub(crate) blocker: Option<Head>,
}

impl Verdict {
    /// The blocker a watchdog line names: at the first deferral and once the grace has run out.
    pub(crate) fn reported_blocker(&self, renewals: u8) -> Option<Head> {
        self.blocker.filter(|_| renewals == 0 || self.decision != Decision::Defer)
    }
}

pub(crate) fn attribute(own: Head, renewals: u8, others: impl Iterator<Item = Head>) -> Verdict {
    let blocker = blocker(own, others);
    Verdict {
        decision: decide(own, renewals, blocker),
        own,
        blocker,
    }
}

#[cfg_attr(not(test), kernel::macros::kunit_tests(asahi_g17_execution_allowance))]
mod tests {
    use super::*;
    #[test]
    fn selects_compute_both_render_halves_and_builds() {
        assert!(!select(false, [false; 2], false));
        assert!(select(true, [false; 2], false));
        assert!(select(false, [true, false], false));
        assert!(select(false, [false, true], false));
        assert!(select(false, [true; 2], false));
        assert!(select(false, [false; 2], true));
    }
    #[test]
    fn helpers_share_only_the_current_submission_allowance() {
        assert!(select_submission([false, true, false].into_iter()));
        assert!(select_submission([true, false].into_iter()));
        assert!(select_submission([false, true].into_iter()));
        assert!(!select_submission([false, false].into_iter()));
        assert!(!select_submission([].into_iter()));
        let mut helper = 0;
        for _ in 0..4 { assert!(renew_execution(true, true, &mut helper)); }
        assert!(!renew_execution(true, true, &mut helper));
        assert!(!renew_execution(true, false, &mut 0));
    }
    #[test]
    fn build_hint_rejects_reserved_and_unknown_flags() {
        assert_eq!(as_build(0), Some(false));
        assert_eq!(as_build(1 << 2), Some(true));
        for bit in 0..32 {
            if bit == 2 { continue; }
            assert_eq!(as_build(1 << bit), None);
            assert_eq!(as_build((1 << bit) | (1 << 2)), None);
        }
        assert_eq!(as_build(u32::MAX), None);
    }
    #[test]
    fn borrowed_render_and_queued_retirement_cannot_defer_forever() {
        for count in 0..=u8::MAX {
            assert_eq!(transaction_grace(count), count < MAX_RENEWALS);
        }
    }
    #[test]
    fn background_control_and_replay_polls_have_an_absolute_deadline() {
        let since = 42;
        assert!(!background_stalled(None, u64::MAX));
        assert!(!background_stalled(Some(since), since - 1));
        assert!(!background_stalled(Some(since), since + 9_999_999_999));
        for _ in 0..1024 {
            assert!(background_stalled(Some(since), since + 10_000_000_000));
        }
        assert!(background_stalled(Some(since), u64::MAX));
    }
    fn classify(own: Head, renewals: u8, others: impl Iterator<Item = Head>) -> Decision {
        attribute(own, renewals, others).decision
    }
    fn head(vm: u64, published: u64, finished: bool) -> Head {
        Head { vm, published, finished, failed: false }
    }
    fn failed(vm: u64, published: u64) -> Head {
        Head { vm, published, finished: false, failed: true }
    }
    #[test]
    fn ray_query_execution_is_bounded_at_ten_seconds() {
        let mut renewals = 0;
        for poll in 1..=4 {
            assert!(renew_execution(true, true, &mut renewals));
            assert_eq!(renewals, poll);
        }
        for _ in 0..256 {
            assert!(!renew_execution(true, true, &mut renewals));
        }
        assert_eq!(POLL_MS * (u32::from(renewals) + 1), 10_000);
    }
    #[test]
    fn ordinary_and_unpublished_jobs_keep_the_first_deadline() {
        for (published, enabled) in [(false, false), (false, true), (true, false)] {
            let mut renewals = 0;
            assert!(!renew_execution(published, enabled, &mut renewals));
            assert_eq!(renewals, 0);
        }
    }
    #[test]
    fn ray_query_and_cross_vm_credits_are_independent_and_finite() {
        let mut ray = 0;
        let own = head(5, 101, false);
        for _ in 0..4 {
            assert!(renew_execution(true, true, &mut ray));
        }
        for victim in 0..MAX_RENEWALS {
            assert!(!renew_execution(true, true, &mut ray));
            assert_eq!(classify(own, victim, [head(7, 100, false)].into_iter()), Decision::Defer);
        }
        assert_eq!(classify(own, MAX_RENEWALS, [head(7, 100, false)].into_iter()), Decision::Spare);
        assert_eq!(POLL_MS * (1 + u32::from(ray) + u32::from(MAX_RENEWALS)), 26_000);
    }
    #[test]
    fn a_live_gpu_renews_a_published_job_for_thirty_seconds() {
        let mut renewals = 0;
        for poll in 1..=14u8 {
            assert!(renew_shared(true, retiring(Some(4_900_000_000)), &mut renewals));
            assert_eq!(renewals, poll);
        }
        assert!(!renew_shared(true, true, &mut renewals));
        assert_eq!(POLL_MS * (u32::from(renewals) + 1), 30_000);
    }
    #[test]
    fn a_stopped_gpu_or_unpublished_job_keeps_the_deadline() {
        let mut renewals = 0;
        assert!(!renew_shared(true, retiring(Some(LIVENESS_NS)), &mut renewals));
        assert!(!renew_shared(true, retiring(None), &mut renewals));
        assert!(!renew_shared(false, true, &mut renewals));
        assert_eq!(renewals, 0);
    }
    #[test]
    fn a_long_job_beside_another_shares_the_gpu_without_retirements() {
        // Two traversal dispatches side by side: nothing retires, yet both progress.
        let mut renewals = 0;
        let long_job_beside = true;
        let shared = retiring(Some(LIVENESS_NS + 1)) || long_job_beside;
        for poll in 1..=14u8 {
            assert!(renew_shared(true, shared, &mut renewals));
            assert_eq!(renewals, poll);
        }
        // The thirty-second bound still ends a pair that never finishes.
        assert!(!renew_shared(true, true, &mut renewals));
    }
    #[test]
    fn exhausted_or_invalid_credit_never_wraps() {
        for value in 4..=u8::MAX {
            let mut renewals = value;
            assert!(!renew_execution(true, true, &mut renewals));
            assert_eq!(renewals, value);
        }
    }
    #[test]
    fn earlier_other_vm_gets_bounded_grace() {
        let own = head(5, 101, false);
        for count in 0..MAX_RENEWALS {
            assert_eq!(classify(own, count, [head(7, 100, false)].into_iter()), Decision::Defer);
        }
        assert_eq!(classify(own, MAX_RENEWALS, [head(7, 100, false)].into_iter()), Decision::Spare);
        assert_eq!(classify(own, 2, [].into_iter()), Decision::Own);
    }
    #[test]
    fn oldest_unfinished_queue_is_never_excused() {
        assert_eq!(classify(head(7, 100, false), 0, [head(5, 101, false)].into_iter()), Decision::Own);
    }
    #[test]
    fn a_blocker_that_already_failed_defers_then_never_spares() {
        // An innocent frame published before the culprit timed out first and failed; its
        // quarantined pair keeps its head until the firmware drains it.
        let culprit = head(7, 101, false);
        for count in 0..MAX_RENEWALS {
            assert_eq!(classify(culprit, count, [failed(5, 100)].into_iter()), Decision::Defer);
        }
        assert_eq!(classify(culprit, MAX_RENEWALS, [failed(5, 100)].into_iter()), Decision::Own);
        assert_eq!(classify(culprit, u8::MAX, [failed(5, 100)].into_iter()), Decision::Own);
        // A pair that never drains does not spare any later timeout either.
        let drained_never = [failed(5, 100), failed(5, 102)];
        for own in [head(7, 105, false), head(9, 200, false)] {
            assert_eq!(classify(own, MAX_RENEWALS, drained_never.into_iter()), Decision::Own);
        }
    }
    #[test]
    fn a_live_blocker_still_spares_beside_a_failed_one() {
        let own = head(9, 103, false);
        let others = [failed(5, 100), head(7, 101, false), failed(8, 102)];
        assert_eq!(blocker(own, others.into_iter()), Some(head(7, 101, false)));
        assert_eq!(classify(own, MAX_RENEWALS, others.into_iter()), Decision::Spare);
        // Failed or not, a blocker must be older, unfinished and of another VM.
        let late = [failed(5, 104), failed(9, 100), Head { finished: true, ..failed(5, 100) }];
        assert_eq!(blocker(own, late.into_iter()), None);
        assert_eq!(classify(own, 0, late.into_iter()), Decision::Own);
        // Among failed blockers the oldest is reported.
        let both_failed = [failed(8, 102), failed(5, 100)];
        assert_eq!(blocker(own, both_failed.into_iter()), Some(failed(5, 100)));
    }
    #[test]
    fn the_watchdog_line_names_the_blocker_at_the_first_deferral_and_the_end() {
        let own = head(9, 103, false);
        let verdict = attribute(own, 0, [failed(5, 100)].into_iter());
        let expected = Verdict { decision: Decision::Defer, own, blocker: Some(failed(5, 100)) };
        assert_eq!(verdict, expected);
        assert_eq!(verdict.reported_blocker(0), Some(failed(5, 100)));
        assert_eq!(attribute(own, 1, [failed(5, 100)].into_iter()).reported_blocker(1), None);
        let last = attribute(own, MAX_RENEWALS, [failed(5, 100)].into_iter());
        assert_eq!(last.decision, Decision::Own);
        assert_eq!(last.reported_blocker(MAX_RENEWALS), Some(failed(5, 100)));
        let spared = attribute(own, MAX_RENEWALS, [head(5, 100, false)].into_iter());
        assert_eq!(spared.decision, Decision::Spare);
        assert_eq!(spared.reported_blocker(MAX_RENEWALS), Some(head(5, 100, false)));
        // Nothing blocks: nothing to report.
        assert_eq!(attribute(own, 0, [].into_iter()).reported_blocker(0), None);
    }
    #[test]
    fn same_vm_finished_and_later_work_do_not_block() {
        let own = head(5, 101, false);
        assert_eq!(classify(own, 0, [head(5, 100, false), head(7, 100, true), head(7, 101, false), head(7, 102, false)].into_iter()), Decision::Own);
        assert_eq!(classify(head(5, 101, true), 0, [head(7, 100, false)].into_iter()), Decision::Own);
    }
}
