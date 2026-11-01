// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(dead_code, unused_variables, unused_imports)]
use std::cell::{RefCell, RefMut};
use std::sync::atomic::{AtomicU64, Ordering::Relaxed};

/* PRODUCTION */

#[derive(Default)]
struct Mock {
    ms: u64,
    irq: bool,
    spurious: bool,
    events: Vec<(u64, Option<u8>, u32)>,
    cancel_at: Option<u64>,
    epoch_at: Option<u64>,
    status_cost: u64,
    reads: usize,
    arms: usize,
    guides: Vec<bio::Guidance>,
}
thread_local! { static MOCK: RefCell<Mock> = RefCell::new(Mock::default()); }
fn clock() -> u64 { MOCK.with(|m| m.borrow().ms) }
mod shim {
    pub fn monotonic_ns() -> u64 { super::clock() * 1_000_000 }
}
mod kernel {
    pub mod time {
        pub struct Delta(pub i64);
        impl Delta { pub fn from_millis(ms: i64) -> Self { Self(ms) } }
        pub mod delay {
            pub fn fsleep(delta: super::Delta) {
                crate::MOCK.with(|m| m.borrow_mut().ms += delta.0 as u64);
            }
        }
    }
}
mod bio {
    #[derive(Clone, Copy, Debug, PartialEq)]
    pub enum Guidance { Place, HoldStill }
    pub struct Session;
    pub fn capture_is_live(_: &Session, epoch: u64) -> bool {
        super::MOCK.with(|m| {
            let m = m.borrow();
            !m.cancel_at.is_some_and(|at| m.ms >= at)
                && epoch == if m.epoch_at.is_some_and(|at| m.ms >= at) { 2 } else { 1 }
        })
    }
    pub fn capture_guide(s: &mut Session, epoch: u64, guide: Guidance) -> bool {
        if !capture_is_live(s, epoch) { return false; }
        super::MOCK.with(|m| {
            let mut m = m.borrow_mut();
            if m.guides.last() == Some(&guide) { false }
            else { m.guides.push(guide); true }
        })
    }
}
mod sensor {
    pub const STATUS_LEN: usize = 16;
    pub const STATE_ARMED: u8 = 17;
    pub const STATE_READING: u8 = 19;
    pub const STATE_NEEDS_PATCH: u8 = 9;
    pub const STATE_IDLE: u8 = 0;
    pub const STATE_DATA_READY: u8 = 7;
    pub enum Offset12 { Available(u32), Other }
    pub struct Status { pub state: u8, pub raw: [u8; STATUS_LEN], pub count: u32 }
    impl Status {
        pub fn offset12(&self) -> Offset12 {
            if self.state == STATE_DATA_READY { Offset12::Available(self.count) }
            else { Offset12::Other }
        }
    }
    pub fn irq_available() -> bool { super::MOCK.with(|m| m.borrow().irq) }
    pub fn irq_arm() { super::MOCK.with(|m| m.borrow_mut().arms += 1); }
    pub fn irq_wait(ms: u32) -> bool {
        super::MOCK.with(|m| {
            let mut m = m.borrow_mut();
            m.ms += if m.spurious { 1 } else { u64::from(ms) };
        });
        true
    }
    pub fn status() -> Result<Status, ()> {
        super::MOCK.with(|m| {
            let mut m = m.borrow_mut();
            m.reads += 1;
            assert!(m.reads < 2_000_000, "wait did not terminate");
            m.ms += m.status_cost;
            let (_, state, count) = *m.events.iter().rev().find(|v| v.0 <= m.ms).expect("initial state");
            state.map(|state| Status { state, raw: [state; STATUS_LEN], count }).ok_or(())
        })
    }
}
struct SessionLock(RefCell<bio::Session>);
impl SessionLock { fn lock(&self) -> RefMut<'_, bio::Session> { self.0.borrow_mut() } }
struct Driver { bio_session: SessionLock, capture_epoch: AtomicU64 }
impl Driver {
    fn new() -> Self {
        Self { bio_session: SessionLock(RefCell::new(bio::Session)), capture_epoch: AtomicU64::new(1) }
    }
    fn bio_wake(&self) {}
    fn wait(&self, verify: bool) -> CaptureWait { /* CALL */ }
    /* METHOD */
}
fn begin(irq: bool, events: &[(u64, Option<u8>, u32)]) -> Driver {
    MOCK.with(|m| *m.borrow_mut() = Mock { irq, events: events.to_vec(), ..Mock::default() });
    Driver::new()
}
fn both(mut test: impl FnMut(bool)) { for irq in [false, true] { test(irq); } }

#[test]
fn immediate_frame_is_accepted() {
    both(|irq| {
        let d = begin(irq, &[(0, Some(7), 128)]);
        assert!(matches!(d.wait(true), CaptureWait::Ready(128)));
    });
}
#[test]
fn delayed_finger_stays_in_one_capture() {
    both(|irq| {
        let d = begin(irq, &[(0, Some(17), 0), (185_125, Some(19), 0), (185_375, Some(7), 512)]);
        assert!(matches!(d.wait(true), CaptureWait::Ready(512)));
        assert!(clock() >= 185_375);
        MOCK.with(|m| assert_eq!(m.borrow().guides, vec![bio::Guidance::Place, bio::Guidance::HoldStill]));
    });
}
#[test]
fn enrollment_keeps_absolute_deadline() {
    both(|irq| {
        let d = begin(irq, &[(0, Some(17), 0), (70_000, Some(7), 512)]);
        assert!(matches!(d.wait(false), CaptureWait::Timeout));
        assert_eq!(clock(), 60_000);
    });
}
#[test]
fn enrollment_progress_does_not_reset_deadline() {
    both(|irq| {
        let d = begin(irq, &[(0, Some(17), 0), (59_000, Some(19), 0), (70_000, Some(7), 512)]);
        assert!(matches!(d.wait(false), CaptureWait::Timeout));
        assert_eq!(clock(), 60_000);
    });
}
#[test]
fn cancellation_and_epoch_change_are_bounded() {
    both(|irq| {
        for epoch in [false, true] {
            let d = begin(irq, &[(0, Some(17), 0), (180_000, Some(7), 512)]);
            MOCK.with(|m| {
                if epoch { m.borrow_mut().epoch_at = Some(120_001); }
                else { m.borrow_mut().cancel_at = Some(120_001); }
            });
            assert!(matches!(d.wait(true), CaptureWait::Abandon));
            assert!(clock() <= 120_001 + if irq { 250 } else { 2 });
        }
    });
}
#[test]
fn cancellation_during_status_drops_ready_frame() {
    let d = begin(false, &[(0, Some(7), 512)]);
    MOCK.with(|m| { let mut m=m.borrow_mut(); m.status_cost=10; m.cancel_at=Some(1); });
    assert!(matches!(d.wait(true), CaptureWait::Abandon));
}
#[test]
fn cancellation_before_first_read_avoids_spi() {
    let d = begin(true, &[(0, Some(7), 512)]);
    MOCK.with(|m| m.borrow_mut().cancel_at=Some(0));
    assert!(matches!(d.wait(true), CaptureWait::Abandon));
    MOCK.with(|m| assert_eq!(m.borrow().reads, 0));
}
#[test]
fn status_failure_is_not_retried() {
    both(|irq| {
        let d = begin(irq, &[(0, Some(17), 0), (120_000, None, 0)]);
        assert!(matches!(d.wait(true), CaptureWait::Abandon));
        assert_eq!(clock(), 120_000);
    });
}
#[test]
fn lost_patch_is_a_fault() {
    both(|irq| {
        let d = begin(irq, &[(0, Some(17), 0), (120_000, Some(9), 0)]);
        assert!(matches!(d.wait(true), CaptureWait::Fault(9)));
    });
}
#[test]
fn hardware_zero_frame_is_still_a_timeout() {
    both(|irq| {
        let d = begin(irq, &[(0, Some(17), 0), (120_000, Some(7), 0)]);
        assert!(matches!(d.wait(true), CaptureWait::Timeout));
        assert_eq!(clock(), 120_000);
    });
}
#[test]
fn stuck_reading_is_bounded_after_long_idle() {
    both(|irq| {
        let d = begin(irq, &[(0, Some(17), 0), (120_000, Some(19), 0)]);
        assert!(matches!(d.wait(true), CaptureWait::Timeout));
        assert!(clock() >= 179_750 && clock() <= 180_000);
    });
}
#[test]
fn returning_to_armed_does_not_hide_stalled_progress() {
    both(|irq| {
        for state in [0, 19, 42] {
            let d = begin(irq, &[(0, Some(17), 0), (120_000, Some(state), 0), (120_500, Some(17), 0)]);
            assert!(matches!(d.wait(true), CaptureWait::Timeout));
            assert!(clock() <= 180_000);
        }
    });
}
#[test]
fn never_armed_is_bounded() {
    both(|irq| {
        let d = begin(irq, &[(0, Some(0), 0)]);
        assert!(matches!(d.wait(true), CaptureWait::Timeout));
        assert_eq!(clock(), 60_000);
    });
}
#[test]
fn spurious_irq_does_not_spend_deadline_early() {
    let d = begin(true, &[(0, Some(17), 0), (180_000, Some(7), 512)]);
    MOCK.with(|m| m.borrow_mut().spurious = true);
    assert!(matches!(d.wait(true), CaptureWait::Ready(512)));
    assert_eq!(clock(), 180_000);
}

#[test]
fn unattended_verification_has_absolute_upper_bound() {
    both(|irq| {
        let d = begin(irq, &[(0, Some(17), 0)]);
        // This later cancellation also bounds the deliberate no-cap mutant.
        MOCK.with(|m| m.borrow_mut().cancel_at = Some(360_000));
        assert!(matches!(d.wait(true), CaptureWait::Timeout));
        assert_eq!(clock(), 300_000);
    });
}
#[test]
fn frame_at_verification_deadline_is_not_accepted() {
    both(|irq| {
        let d = begin(irq, &[(0, Some(17), 0), (300_000, Some(7), 512)]);
        assert!(matches!(d.wait(true), CaptureWait::Timeout));
        assert_eq!(clock(), 300_000);
    });
}
#[test]
fn frame_just_before_verification_deadline_is_accepted() {
    both(|irq| {
        let d = begin(irq, &[(0, Some(17), 0), (299_750, Some(7), 512)]);
        assert!(matches!(d.wait(true), CaptureWait::Ready(512)));
        assert_eq!(clock(), 299_750);
    });
}
#[test]
fn progress_cannot_extend_the_absolute_verification_deadline() {
    both(|irq| {
        let d = begin(irq, &[(0, Some(17), 0), (299_000, Some(19), 0), (300_500, Some(7), 512)]);
        assert!(matches!(d.wait(true), CaptureWait::Timeout));
        assert_eq!(clock(), 300_000);
    });
}
#[test]
fn cancellation_during_expiring_status_read_takes_priority() {
    let d = begin(false, &[(0, Some(7), 512)]);
    MOCK.with(|m| {
        let mut m = m.borrow_mut();
        m.status_cost = 300_000;
        m.cancel_at = Some(300_000);
    });
    assert!(matches!(d.wait(true), CaptureWait::Abandon));
}

#[test]
fn status_read_crossing_absolute_deadline_discards_ready_frame() {
    let d = begin(false, &[(0, Some(7), 512)]);
    MOCK.with(|m| m.borrow_mut().status_cost = 300_001);
    assert!(matches!(d.wait(true), CaptureWait::Timeout));
}

#[test]
fn late_initial_arming_cannot_reopen_the_progress_budget() {
    both(|irq| {
        let d = begin(irq, &[(0, Some(0), 0), (60_000, Some(17), 0), (60_500, Some(7), 512)]);
        assert!(matches!(d.wait(true), CaptureWait::Timeout));
        assert_eq!(clock(), 60_000);
    });
}
#[test]
fn initial_arming_before_deadline_can_extend_idle_wait() {
    both(|irq| {
        let d = begin(irq, &[(0, Some(0), 0), (59_750, Some(17), 0), (185_500, Some(7), 512)]);
        assert!(matches!(d.wait(true), CaptureWait::Ready(512)));
        assert_eq!(clock(), 185_500);
    });
}
#[test]
fn status_read_crossing_arming_deadline_cannot_extend_idle_wait() {
    both(|irq| {
        let d = begin(irq, &[(0, Some(0), 0), (60_000, Some(17), 0), (60_500, Some(7), 512)]);
        MOCK.with(|m| m.borrow_mut().status_cost = 60_001);
        assert!(matches!(d.wait(true), CaptureWait::Timeout));
        assert_eq!(clock(), 60_001);
        MOCK.with(|m| assert_eq!(m.borrow().reads, 1));
    });
}
#[test]
fn expired_progress_cannot_be_hidden_by_return_to_armed() {
    both(|irq| {
        let deadline = 60_000 + if irq { 750 } else { 998 };
        let d = begin(irq, &[(0, Some(17), 0), (1_000, Some(19), 0), (deadline, Some(17), 0), (deadline + 500, Some(7), 512)]);
        assert!(matches!(d.wait(true), CaptureWait::Timeout));
        assert_eq!(clock(), deadline);
    });
}
#[test]
fn ready_frame_at_progress_deadline_is_refused() {
    both(|irq| {
        let deadline = 60_000 + if irq { 750 } else { 998 };
        let d = begin(irq, &[(0, Some(17), 0), (1_000, Some(19), 0), (deadline, Some(7), 512)]);
        assert!(matches!(d.wait(true), CaptureWait::Timeout));
        assert_eq!(clock(), deadline);
    });
}
#[test]
fn ready_frame_just_before_progress_deadline_is_accepted() {
    both(|irq| {
        let deadline = 60_000 + if irq { 750 } else { 998 };
        let ready = deadline - if irq { 250 } else { 2 };
        let d = begin(irq, &[(0, Some(17), 0), (1_000, Some(19), 0), (ready, Some(7), 512)]);
        assert!(matches!(d.wait(true), CaptureWait::Ready(512)));
        assert_eq!(clock(), ready);
    });
}
#[test]
fn enrollment_ready_frame_at_deadline_is_refused() {
    both(|irq| {
        let d = begin(irq, &[(0, Some(17), 0), (60_000, Some(7), 512)]);
        assert!(matches!(d.wait(false), CaptureWait::Timeout));
        assert_eq!(clock(), 60_000);
    });
}
#[test]
fn enrollment_ready_frame_before_deadline_is_accepted() {
    both(|irq| {
        let d = begin(irq, &[(0, Some(17), 0), (59_750, Some(7), 512)]);
        assert!(matches!(d.wait(false), CaptureWait::Ready(512)));
        assert_eq!(clock(), 59_750);
    });
}
#[test]
fn status_read_crossing_enrollment_deadline_discards_ready_frame() {
    both(|irq| {
        let d = begin(irq, &[(0, Some(7), 512)]);
        MOCK.with(|m| m.borrow_mut().status_cost = 60_001);
        assert!(matches!(d.wait(false), CaptureWait::Timeout));
        assert_eq!(clock(), 60_001);
    });
}
#[test]
fn status_read_crossing_progress_deadline_discards_ready_frame() {
    both(|irq| {
        let d = begin(irq, &[(0, Some(17), 0), (60_000, Some(19), 0), (90_000, Some(7), 512)]);
        MOCK.with(|m| m.borrow_mut().status_cost = 30_000);
        assert!(matches!(d.wait(true), CaptureWait::Timeout));
        assert_eq!(clock(), 90_000 + if irq { 500 } else { 4 });
    });
}
#[test]
fn cancellation_during_arming_deadline_read_takes_priority() {
    both(|irq| {
        let d = begin(irq, &[(0, Some(17), 0)]);
        MOCK.with(|m| {
            let mut m = m.borrow_mut();
            m.status_cost = 60_000;
            m.cancel_at = Some(60_000);
        });
        assert!(matches!(d.wait(true), CaptureWait::Abandon));
    });
}
