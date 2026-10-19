// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Pure hang-detection and recovery decisions.
//!
//! The arithmetic and branch structure of the poll pass, free of locks, objects and clocks so
//! host tests can exercise them: receipt comparisons, the started-compute hang test, the kill
//! divisor choice and the queue-drain test of the recovery handshake, the deadlines and the
//! hang branches. Times are monotonic nanoseconds; a deadline `d` has passed when `now > d`.
//!
//! Core-only (no `kernel` imports): this file is compiled by the host test harness as well.

/// Entries of a firmware work ring (= `hw::sizes::QUEUE_CAP`).
pub(crate) const QUEUE_CAP: u32 = 0x500;
/// Records of a submit channel ring (= `fw::channels::RING_SLOTS`).
pub(crate) const CHANNEL_SLOTS: u32 = 256;
/// Channel indices.
pub(crate) const TA: usize = 0;
pub(crate) const FS: usize = 1;
pub(crate) const COMPUTE: usize = 2;

/// Globals divisors of the firmware's periodic context-store request:
/// compute (+0x1c, ends a running class-2 compute) and fragment (+0x24). Mirrors
/// `fw::initdata::globals::{COMPUTE_KILL, FRAGMENT_KILL}`.
pub(crate) const COMPUTE_KILL: usize = 0x1c;
pub(crate) const FRAGMENT_KILL: usize = 0x24;
/// Globals unchanged-work compute monitor (`fw::initdata::globals::COMPUTE_MONITOR`).
pub(crate) const COMPUTE_MONITOR: usize = 0x9b0;

/// Outcome budget of an armed kill: the prefix retires or an error event arrives within it.
pub(crate) const KILL_BUDGET_MS: u64 = 2000;
/// Recovery handshake bounds: polls for the firmware's halt, polls for the resume to be
/// consumed, polls for the drain, and further halts answered during the drain.
/// Every poll sleeps 1-1.5 ms.
pub(crate) const HALT_POLLS: u32 = 200;
pub(crate) const RESUME_POLLS: u32 = 500;
pub(crate) const DRAIN_POLLS: u32 = 1000;
pub(crate) const FURTHER_HALTS: u32 = 4;

/// Stamp cell `obs` reached the own target `own`, bounded by the newest published target
/// `tail`. Stamps are seeded so that a written stamp is never 0 and has a zero low byte.
pub(crate) const fn stamp_reached(obs: u32, own: u32, tail: u32) -> bool {
    obs != 0 && obs & 0xff == 0 && obs.wrapping_sub(own) <= tail.wrapping_sub(own)
}

/// Cursor `c` of a ring of `m` entries passed the end `h` of entries starting at `b`, within
/// the tail `t`.
///
/// `c` is a firmware-written cursor: the u32 arithmetic wraps (a garbage cursor must not trip
/// `CONFIG_RUST_OVERFLOW_CHECKS`).
pub(crate) const fn ring_passed(c: u32, b: u32, h: u32, t: u32, m: u32) -> bool {
    let dc = c.wrapping_add(m).wrapping_sub(b) % m;
    let dh = h.wrapping_add(m).wrapping_sub(b) % m;
    let dt = t.wrapping_add(m).wrapping_sub(b) % m;
    dh <= dc && dc <= dt
}

/// The receipts of one published prefix, filled at publication.
#[derive(Copy, Clone, Default, Debug, PartialEq, Eq)]
pub(crate) struct Receipts {
    /// Compute and render members of the prefix.
    pub(crate) computes: u32,
    pub(crate) renders: u32,
    /// Compute: final FW stamp target, work-ring end after the prefix and before it.
    pub(crate) target: u32,
    pub(crate) head: u32,
    pub(crate) c_start: u32,
    /// Render: final TA/FS stamp targets, ring ends after and before the prefix.
    pub(crate) ta_target: u32,
    pub(crate) fs_target: u32,
    pub(crate) ta_head: u32,
    pub(crate) fs_head: u32,
    pub(crate) ta_start: u32,
    pub(crate) fs_start: u32,
    /// Host channel producers (TA, FS, compute) after the prefix's records.
    pub(crate) rec: [u32; 3],
    /// Compute member j's FW stamp target, and the FS target it waits on directly (0: none).
    pub(crate) cstamp: [u32; 16],
    pub(crate) cwait: [u32; 16],
}

/// The newest published receipts.
#[derive(Copy, Clone, Default, Debug, PartialEq, Eq)]
pub(crate) struct Tail {
    pub(crate) target: u32,
    pub(crate) head: u32,
    pub(crate) ta_target: u32,
    pub(crate) fs_target: u32,
    pub(crate) ta_head: u32,
    pub(crate) fs_head: u32,
    pub(crate) producer: [u32; 3],
}

/// Firmware-written words read in one observation (single aligned loads of uncached cells).
#[derive(Copy, Clone, Default, Debug, PartialEq, Eq)]
pub(crate) struct Observed {
    /// FW stamps: compute (the compute graph's `fw_stamp`), TA and FS (the render graph's
    /// `fw_stamp[0..1]`).
    pub(crate) compute_stamp: u32,
    pub(crate) ta_stamp: u32,
    pub(crate) fs_stamp: u32,
    /// Work-ring DONE (qstate +0) and READ (qstate +0x30) of the compute, TA and FS queues.
    pub(crate) compute_done: u32,
    pub(crate) compute_read: u32,
    pub(crate) ta_done: u32,
    pub(crate) ta_read: u32,
    pub(crate) fs_done: u32,
    pub(crate) fs_read: u32,
    /// Channel consumers (the read cell of the TA, FS and compute channel state blocks).
    pub(crate) consumer: [u32; 3],
}

/// Engine receipts from the SGX status window: busy is bit 0 of C020 | C120; fault is the D8C0
/// status of fault banks 0 and 1.
#[derive(Copy, Clone, Default, Debug, PartialEq, Eq)]
pub(crate) struct EngineState {
    pub(crate) busy: bool,
    pub(crate) fault: u64,
}

/// The oldest prefix `f`'s first unfinished compute has started and makes no progress: its
/// predecessors in the prefix are done (earlier compute members by the reached FW stamp, a
/// render it waits on directly by the FS stamp), the firmware consumed the prefix's compute
/// record and its work-ring READ passed the prefix's entries (the private start timestamp is
/// written only at completion). Later members, renders waiting on it included, may be
/// unfinished.
pub(crate) fn compute_hang(f: &Receipts, t: &Tail, o: &Observed) -> bool {
    let fw = o.compute_stamp;
    if f.computes == 0 || stamp_reached(fw, f.target, t.target) {
        return false; // no compute, or its computes are done
    }
    let computes = (f.computes as usize).min(f.cstamp.len());
    let mut j = 0;
    while j < computes && stamp_reached(fw, f.cstamp[j], t.target) {
        j += 1;
    }
    if j == computes {
        return false;
    }
    if f.cwait[j] != 0 && !stamp_reached(o.fs_stamp, f.cwait[j], t.fs_target) {
        return false; // still waiting on a render of the prefix
    }
    if o.consumer[COMPUTE] != f.rec[COMPUTE] {
        return false; // its record not consumed yet
    }
    ring_passed(o.compute_read, f.c_start, f.head, t.head, QUEUE_CAP)
}

/// The divisor the host deadline arms: the compute request while the oldest prefix's computes
/// are not done, else the fragment request for a prefix with renders.
pub(crate) fn kill_offset(f: &Receipts, t: &Tail, o: &Observed) -> usize {
    if f.computes != 0 && !stamp_reached(o.compute_stamp, f.target, t.target) {
        COMPUTE_KILL
    } else if f.renders != 0 {
        FRAGMENT_KILL
    } else {
        COMPUTE_KILL
    }
}

/// Firmware DONE/READ reached the newest published ends of every queue and every channel
/// record was consumed (the caller adds the engine-idle test).
pub(crate) fn queues_drained(t: &Tail, o: &Observed) -> bool {
    o.ta_done == t.ta_head
        && o.ta_read == t.ta_head
        && o.fs_done == t.fs_head
        && o.fs_read == t.fs_head
        && o.consumer[TA] == t.producer[TA]
        && o.consumer[FS] == t.producer[FS]
        && o.compute_done == t.head
        && o.compute_read == t.head
        && o.consumer[COMPUTE] == t.producer[COMPUTE]
}

/// What a firmware error means for the failed prefixes.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) enum Failure {
    /// A GPU fault (EIO).
    Fault,
    /// A firmware timeout (ETIMEDOUT).
    Timeout,
}

/// Classifies a recovery: a pending fault status or a type-0 event is a fault. `event_type` is
/// the last error event's type; with none seen, type 0 applies.
pub(crate) fn failure(fault: u64, event_type: Option<u32>) -> Failure {
    if fault != 0 || event_type.unwrap_or(0) == 0 {
        Failure::Fault
    } else {
        Failure::Timeout
    }
}

const NSEC_PER_MSEC: u64 = 1_000_000;

/// Host and compute deadlines `deadline_ms`/`compute_deadline_ms` from `now` (both budgets
/// are at least 100 ms).
pub(crate) fn renew(now: u64, deadline_ms: u32, compute_deadline_ms: u32) -> (u64, u64) {
    (
        now + deadline_ms.max(100) as u64 * NSEC_PER_MSEC,
        now + compute_deadline_ms.max(100) as u64 * NSEC_PER_MSEC,
    )
}

/// Host deadline after arming a kill: the outcome budget, or at once (`EXPIRED`) when the
/// request could not be posted.
pub(crate) fn kill_deadline(now: u64, posted: bool) -> u64 {
    if posted {
        now + KILL_BUDGET_MS * NSEC_PER_MSEC
    } else {
        EXPIRED
    }
}

/// A deadline that has passed. Monotonic ns since boot never wrap, so 0 is in the past for
/// every later check.
pub(crate) const EXPIRED: u64 = 0;

/// `time_after(now, deadline)`.
pub(crate) const fn expired(now: u64, deadline: u64) -> bool {
    now > deadline
}

/// The state the hang branches of the poll pass decide on.
#[derive(Copy, Clone, Default, Debug)]
pub(crate) struct HangInputs {
    /// A prefix is published.
    pub(crate) flight: bool,
    /// A kill request is armed.
    pub(crate) armed: bool,
    /// The retirement check saw a fault.
    pub(crate) fault: bool,
    /// The RTKit transport failed.
    pub(crate) transport: bool,
    /// Firmware error events not yet answered.
    pub(crate) new_errors: bool,
    /// The compute no-progress and host deadlines passed.
    pub(crate) compute_deadline_expired: bool,
    pub(crate) deadline_expired: bool,
    /// The compute queue's scheduling class (2: killable), recovery and hang preemption
    /// enabled.
    pub(crate) class: u32,
    pub(crate) recovery: bool,
    pub(crate) hang_preempt: bool,
}

/// What the hang branches do.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) enum HangAction {
    /// Nothing published or armed and no error: go straight to publication.
    Idle,
    /// Nothing to do in this pass; poll again.
    Wait,
    /// The oldest prefix's started compute made no progress for its budget, class 2: arm the
    /// compute request (+0x1c); publication stays frozen.
    ArmCompute,
    /// The host deadline passed with no event: arm the request [`kill_offset`] chooses.
    ArmDeadline,
    /// Failure: answer the firmware halt first when `recover`, else fail-stop.
    Fail { recover: bool },
}

/// The decision of the hang branches.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) struct Decision {
    pub(crate) action: HangAction,
    /// The started-compute hang test matched (reported as "compute hang").
    pub(crate) compute_hang: bool,
    /// ... on the class-0 queue, which cannot be ended: the host deadline is expired so the
    /// failure branch fail-stops now. The caller persists the expired deadline.
    pub(crate) class0_hang: bool,
}

/// The hang branches of the poll pass, after the waited disarm and the progress update.
/// `compute_hang` evaluates [`compute_hang`] for the oldest prefix; it is called only once its
/// deadline passed (and again for a class-0 queue at the host deadline), never on the fast
/// path.
pub(crate) fn decide(i: &HangInputs, mut compute_hang: impl FnMut() -> bool) -> Decision {
    let mut d = Decision {
        action: HangAction::Wait,
        compute_hang: false,
        class0_hang: false,
    };
    let quiet = !i.fault && !i.transport && !i.new_errors;
    if !i.flight && !i.armed && quiet {
        d.action = HangAction::Idle;
        return d;
    }
    let mut deadline_expired = i.deadline_expired;
    if i.flight && quiet && !i.armed && i.compute_deadline_expired && compute_hang() {
        d.compute_hang = true;
        if i.class != 0 && i.recovery {
            d.action = HangAction::ArmCompute;
            return d;
        }
        d.class0_hang = true;
        deadline_expired = true;
    }
    if i.flight
        && quiet
        && deadline_expired
        && !i.armed
        && (i.class != 0 || !compute_hang())
        && i.hang_preempt
        && i.recovery
    {
        d.action = HangAction::ArmDeadline;
    } else if !quiet || ((i.flight || i.armed) && deadline_expired) {
        d.action = HangAction::Fail {
            recover: i.recovery && !i.transport && (i.new_errors || i.fault),
        };
    }
    d
}

#[cfg(test)]
mod tests {
    use super::*;

    const C0: u32 = 0xc1000000;

    fn stamp(n: u32) -> u32 {
        C0 + (n << 8)
    }

    #[test]
    fn stamps() {
        assert!(!stamp_reached(0, stamp(1), stamp(3)));
        assert!(!stamp_reached(stamp(1) + 1, stamp(1), stamp(3)));
        assert!(stamp_reached(stamp(1), stamp(1), stamp(3)));
        assert!(stamp_reached(stamp(3), stamp(1), stamp(3)));
        assert!(!stamp_reached(stamp(0), stamp(1), stamp(3)));
        assert!(!stamp_reached(stamp(4), stamp(1), stamp(3)));
        // Across the u32 wrap.
        let own = 0xffffff00u32;
        assert!(stamp_reached(0x00000100, own, 0x00000200));
        assert!(!stamp_reached(0xfffffe00, own, 0x00000200));
    }

    #[test]
    fn rings() {
        // Entries 10..14 of a 0x500 ring, tail at 20.
        assert!(!ring_passed(12, 10, 14, 20, QUEUE_CAP));
        assert!(ring_passed(14, 10, 14, 20, QUEUE_CAP));
        assert!(ring_passed(20, 10, 14, 20, QUEUE_CAP));
        assert!(!ring_passed(21, 10, 14, 20, QUEUE_CAP));
        // Wrapped.
        assert!(ring_passed(2, 0x4fe, 1, 5, QUEUE_CAP));
        assert!(!ring_passed(0x4ff, 0x4fe, 1, 5, QUEUE_CAP));
        // A channel record: consumed when the cursor passed rec - 1 .. rec.
        let rec = 7;
        assert!(!ring_passed(6, (rec + 255) % 256, rec, 9, CHANNEL_SLOTS));
        assert!(ring_passed(7, (rec + 255) % 256, rec, 9, CHANNEL_SLOTS));
        assert!(ring_passed(0, 255, 0, 3, CHANNEL_SLOTS));
    }

    /// A compute-only prefix of three computes, stamps 1..3, ring entries 10..13 (with the
    /// compute->compute barriers), compute record producer 5.
    fn prefix() -> (Receipts, Tail, Observed) {
        let mut f = Receipts {
            computes: 3,
            target: stamp(3),
            head: 15,
            c_start: 10,
            rec: [0, 0, 5],
            ..Default::default()
        };
        f.cstamp[..3].copy_from_slice(&[stamp(1), stamp(2), stamp(3)]);
        let t = Tail {
            target: stamp(3),
            head: 15,
            producer: [0, 0, 5],
            ..Default::default()
        };
        let o = Observed {
            compute_stamp: stamp(0),
            compute_read: 15,
            consumer: [0, 0, 5],
            ..Default::default()
        };
        (f, t, o)
    }

    #[test]
    fn compute_hangs() {
        let (mut f, t, mut o) = prefix();
        // Started (record consumed, READ past the prefix), first compute unfinished.
        assert!(compute_hang(&f, &t, &o));
        // Second compute running after the first finished.
        o.compute_stamp = stamp(1);
        assert!(compute_hang(&f, &t, &o));
        // All done: no hang.
        o.compute_stamp = stamp(3);
        assert!(!compute_hang(&f, &t, &o));
        o.compute_stamp = stamp(1);
        // Record not consumed yet.
        o.consumer[COMPUTE] = 4;
        assert!(!compute_hang(&f, &t, &o));
        o.consumer[COMPUTE] = 5;
        // READ not past the prefix's entries.
        o.compute_read = 12;
        assert!(!compute_hang(&f, &t, &o));
        o.compute_read = 15;
        // The running compute waits on an FS of the prefix that has not completed.
        f.cwait[1] = 0x3d000300;
        o.fs_stamp = 0x3d000200;
        let mut t2 = t;
        t2.fs_target = 0x3d000400;
        assert!(!compute_hang(&f, &t2, &o));
        o.fs_stamp = 0x3d000300;
        assert!(compute_hang(&f, &t2, &o));
        // No compute in the prefix.
        f.computes = 0;
        assert!(!compute_hang(&f, &t, &o));
    }

    #[test]
    fn kill_offsets() {
        let (mut f, t, mut o) = prefix();
        assert_eq!(kill_offset(&f, &t, &o), COMPUTE_KILL);
        o.compute_stamp = stamp(3);
        f.renders = 1;
        assert_eq!(kill_offset(&f, &t, &o), FRAGMENT_KILL);
        f.renders = 0;
        assert_eq!(kill_offset(&f, &t, &o), COMPUTE_KILL);
        f.computes = 0;
        f.renders = 2;
        assert_eq!(kill_offset(&f, &t, &o), FRAGMENT_KILL);
    }

    #[test]
    fn drained() {
        let t = Tail {
            head: 15,
            ta_head: 7,
            fs_head: 9,
            producer: [3, 4, 5],
            ..Default::default()
        };
        let mut o = Observed {
            compute_done: 15,
            compute_read: 15,
            ta_done: 7,
            ta_read: 7,
            fs_done: 9,
            fs_read: 9,
            consumer: [3, 4, 5],
            ..Default::default()
        };
        assert!(queues_drained(&t, &o));
        o.fs_read = 8;
        assert!(!queues_drained(&t, &o));
        o.fs_read = 9;
        o.consumer[COMPUTE] = 4;
        assert!(!queues_drained(&t, &o));
    }

    #[test]
    fn failures_and_deadlines() {
        assert_eq!(failure(0, Some(4)), Failure::Timeout);
        assert_eq!(failure(1, Some(4)), Failure::Fault);
        assert_eq!(failure(0, Some(0)), Failure::Fault);
        assert_eq!(failure(0, None), Failure::Fault);
        assert_eq!(
            renew(1000, 5000, 1000),
            (1000 + 5_000_000_000, 1000 + 1_000_000_000)
        );
        assert_eq!(renew(0, 10, 0), (100_000_000, 100_000_000));
        assert_eq!(kill_deadline(7, true), 7 + 2_000_000_000);
        assert!(expired(1, kill_deadline(7, false)));
        assert!(!expired(5, 5));
        assert!(expired(6, 5));
    }

    fn base() -> HangInputs {
        HangInputs {
            flight: true,
            class: 2,
            recovery: true,
            hang_preempt: true,
            ..Default::default()
        }
    }

    #[test]
    fn decisions() {
        let never = || -> bool { panic!("compute_hang evaluated on the fast path") };
        // Idle: nothing published, nothing armed, no error.
        let i = HangInputs {
            flight: false,
            ..base()
        };
        assert_eq!(decide(&i, never).action, HangAction::Idle);
        // Published, within the deadlines: wait (no compute-hang evaluation).
        assert_eq!(decide(&base(), never).action, HangAction::Wait);
        // Compute deadline passed but no started compute hangs: wait.
        let i = HangInputs {
            compute_deadline_expired: true,
            ..base()
        };
        assert_eq!(decide(&i, || false).action, HangAction::Wait);
        // ... with a hang on class 2: arm the compute request.
        let d = decide(&i, || true);
        assert_eq!(d.action, HangAction::ArmCompute);
        assert!(d.compute_hang && !d.class0_hang);
        // ... with recovery off: no request; the host deadline is expired, the failure
        // branch fail-stops (no event, so no handshake).
        let i0 = HangInputs {
            recovery: false,
            ..i
        };
        let d = decide(&i0, || true);
        assert_eq!(d.action, HangAction::Fail { recover: false });
        assert!(d.class0_hang);
        // ... on class 0: fail-stop at once.
        let i0 = HangInputs { class: 0, ..i };
        let d = decide(&i0, || true);
        assert_eq!(d.action, HangAction::Fail { recover: false });
        assert!(d.compute_hang && d.class0_hang);
        // Host deadline with no event: arm the request.
        let i = HangInputs {
            deadline_expired: true,
            ..base()
        };
        assert_eq!(decide(&i, never).action, HangAction::ArmDeadline);
        // ... without hang preemption: fail-stop.
        let i2 = HangInputs {
            hang_preempt: false,
            ..i
        };
        assert_eq!(
            decide(&i2, never).action,
            HangAction::Fail { recover: false }
        );
        // ... class 0 with a compute hang: no request (it cannot end it).
        let i2 = HangInputs { class: 0, ..i };
        assert_eq!(
            decide(&i2, || true).action,
            HangAction::Fail { recover: false }
        );
        assert_eq!(decide(&i2, || false).action, HangAction::ArmDeadline);
        // Armed, budget running: wait; budget over: fail-stop.
        let i = HangInputs {
            armed: true,
            ..base()
        };
        assert_eq!(decide(&i, never).action, HangAction::Wait);
        let i = HangInputs {
            armed: true,
            deadline_expired: true,
            ..base()
        };
        assert_eq!(
            decide(&i, never).action,
            HangAction::Fail { recover: false }
        );
        // Armed with nothing published (the ended prefix retired, disarm pending): wait.
        let i = HangInputs {
            flight: false,
            armed: true,
            ..base()
        };
        assert_eq!(decide(&i, never).action, HangAction::Wait);
        // An error event: handshake, also with nothing published.
        for flight in [true, false] {
            let i = HangInputs {
                flight,
                new_errors: true,
                ..base()
            };
            assert_eq!(decide(&i, never).action, HangAction::Fail { recover: true });
        }
        // A receipt fault: handshake.
        let i = HangInputs {
            fault: true,
            ..base()
        };
        assert_eq!(decide(&i, never).action, HangAction::Fail { recover: true });
        // Transport failure: fail-stop, never a handshake.
        let i = HangInputs {
            transport: true,
            new_errors: true,
            ..base()
        };
        assert_eq!(
            decide(&i, never).action,
            HangAction::Fail { recover: false }
        );
        // Recovery off: an error fail-stops.
        let i = HangInputs {
            new_errors: true,
            recovery: false,
            ..base()
        };
        assert_eq!(
            decide(&i, never).action,
            HangAction::Fail { recover: false }
        );
    }
}
