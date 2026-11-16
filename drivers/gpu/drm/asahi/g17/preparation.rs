// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Admission and lifetime of off-lock graph construction.
//!
//! Entering and leaving an open epoch use only atomics. Recovery closes the epoch while holding
//! the device mutex, then drops that mutex before waiting for builders. A builder's lease is
//! stored after its tentative mappings so those mappings are destroyed before the join completes.

use core::sync::atomic::{AtomicU64, Ordering};
use kernel::{
    new_condvar, new_mutex,
    prelude::*,
    sync::{Arc, CondVar, Mutex},
};

const COUNT_BITS: u32 = 20;
const COUNT_MASK: u64 = (1 << COUNT_BITS) - 1;
const CLOSED: u64 = 1 << COUNT_BITS;
const REMOVED: u64 = CLOSED << 1;
const FAILED: u64 = CLOSED << 2;
const EPOCH: u64 = CLOSED << 3;

/// One ready packet's monotonic compute-admission budget. The callback checks
/// packet cancellation, logical close, VM status and device loss even when no
/// capacity/recovery event arrives. Wakes and nested constructors never renew it.
pub(super) struct Admission<'a> {
    deadline: u64,
    check: &'a dyn Fn() -> Result,
}

impl<'a> Admission<'a> {
    pub(super) fn new(deadline: u64, check: &'a dyn Fn() -> Result) -> Self {
        Self { deadline, check }
    }

    pub(super) fn check(&self) -> Result {
        (self.check)()?;
        if super::now_ns() >= self.deadline { return Err(EBUSY); }
        Ok(())
    }

    /// Ignore condvar remaining-time reports: only the monotonic deadline counts.
    pub(super) fn slice_ms(&self) -> Result<u32> {
        self.check()?;
        let remaining = self.deadline.saturating_sub(super::now_ns());
        if remaining == 0 { return Err(EBUSY); }
        Ok(remaining.div_ceil(1_000_000).min(10) as u32)
    }
}

#[pin_data]
pub(super) struct Gate {
    state: AtomicU64,
    #[pin]
    wait: Mutex<()>,
    #[pin]
    changed: CondVar,
    /// Scheduler admission waiting for publication to reopen.
    available: Option<Arc<super::availability::Availability>>,
}

impl Gate {
    #[cfg(CONFIG_KUNIT = "y")]
    pub(super) fn new() -> Result<Arc<Self>> {
        Self::with_waiters(None)
    }

    /// A gate whose reopening, failure and removal also wake `available`.
    pub(super) fn with_waiters(
        available: Option<Arc<super::availability::Availability>>,
    ) -> Result<Arc<Self>> {
        Arc::pin_init(
            pin_init!(Self {
                state: AtomicU64::new(EPOCH),
                wait <- new_mutex!((), "g17::Gate::wait"),
                changed <- new_condvar!("g17::Gate::changed"),
                available,
            }),
            GFP_KERNEL,
        )
    }

    fn wake_waiters(&self) {
        if let Some(available) = &self.available {
            available.notify();
        }
    }

    pub(super) fn try_enter(self: &Arc<Self>) -> Result<Lease> {
        let mut previous = self.state.load(Ordering::Acquire);
        loop {
            if previous & (REMOVED | FAILED) != 0 {
                return Err(ENODEV);
            }
            if previous & CLOSED != 0 {
                return Err(EAGAIN);
            }
            if previous & COUNT_MASK == COUNT_MASK {
                return Err(ENOSPC);
            }
            match self.state.compare_exchange_weak(
                previous,
                previous + 1,
                Ordering::AcqRel,
                Ordering::Acquire,
            ) {
                Ok(_) => {
                    return Ok(Lease {
                        gate: self.clone(),
                        epoch: previous & !COUNT_MASK,
                    })
                }
                Err(current) => previous = current,
            }
        }
    }

    /// Wait only for a temporary recovery closure. Called with no device lock or existing lease.
    pub(super) fn enter(self: &Arc<Self>, interruptible: bool) -> Result<Lease> {
        match self.try_enter() {
            Err(error) if error == EAGAIN => {}
            result => return result,
        }
        let mut wait = self.wait.lock();
        loop {
            match self.try_enter() {
                Err(error) if error == EAGAIN => {}
                result => return result,
            }
            if interruptible {
                if self.changed.wait_interruptible(&mut wait) {
                    return Err(ERESTARTSYS);
                }
            } else {
                self.changed.wait(&mut wait);
            }
        }
    }

    /// Packet admission may wait for recovery only within its original budget.
    /// Called with no device mutex or existing preparation lease.
    pub(super) fn enter_until(self: &Arc<Self>, admission: &Admission<'_>) -> Result<Lease> {
        admission.check()?;
        match self.try_enter() {
            Err(error) if error == EAGAIN => {}
            result => return result,
        }
        let mut wait = self.wait.lock();
        loop {
            admission.check()?;
            match self.try_enter() {
                Err(error) if error == EAGAIN => {}
                result => return result,
            }
            let _ = self.changed.wait_timeout(
                &mut wait, kernel::time::msecs_to_jiffies(admission.slice_ms()?)
            );
        }
    }

    fn invalidate(&self, flags: u64) {
        let mut previous = self.state.load(Ordering::Acquire);
        loop {
            let mut next = previous | CLOSED | flags;
            if previous & CLOSED == 0 {
                next = next
                    .checked_add(EPOCH)
                    .unwrap_or(previous | CLOSED | REMOVED | flags);
            }
            match self.state.compare_exchange_weak(
                previous,
                next,
                Ordering::AcqRel,
                Ordering::Acquire,
            ) {
                Ok(_) => return,
                Err(current) => previous = current,
            }
        }
    }

    /// Also wakes publishers in wait_closure(); they then block in enter().
    pub(super) fn close(&self) {
        self.invalidate(0);
        let _wait = self.wait.lock();
        self.changed.notify_all();
    }

    /// Opaque snapshot for wait_closure(), taken where recovery was observed.
    pub(super) fn snapshot(&self) -> u64 {
        self.state.load(Ordering::Acquire)
    }

    /// A publisher that observed recovery (host pending, or the firmware's state
    /// ahead of the event worker, which always follows with a restart and the
    /// closure) sleeps until this gate has closed since `seen`. enter() then
    /// blocks until recovery reopens it. The deadline is only a failure bound.
    pub(super) fn wait_closure(&self, seen: u64, deadline: u64) -> Result {
        let mut wait = self.wait.lock();
        loop {
            let state = self.state.load(Ordering::Acquire);
            if state & (REMOVED | FAILED) != 0 {
                return Err(ENODEV);
            }
            if state & CLOSED != 0 || state & !COUNT_MASK != seen & !COUNT_MASK {
                return Ok(());
            }
            let remaining = deadline.saturating_sub(super::now_ns());
            if remaining == 0 {
                return Err(ETIMEDOUT);
            }
            let ms = u32::try_from(remaining.div_ceil(1_000_000)).unwrap_or(u32::MAX);
            let _ = self.changed.wait_timeout(&mut wait, kernel::time::msecs_to_jiffies(ms));
        }
    }

    pub(super) fn remove(&self) {
        self.invalidate(REMOVED);
        let wait = self.wait.lock();
        self.changed.notify_all();
        drop(wait);
        self.wake_waiters();
    }

    pub(super) fn fail(&self) {
        self.invalidate(FAILED);
        let wait = self.wait.lock();
        self.changed.notify_all();
        drop(wait);
        self.wake_waiters();
    }

    /// Runtime failure/recovery must not join an indefinitely stalled builder.
    /// Timeout retains every graph and reservation; it never authorizes freeing.
    pub(super) fn wait_drained_until(&self, deadline: u64) -> Result {
        let mut wait = self.wait.lock();
        while self.state.load(Ordering::Acquire) & COUNT_MASK != 0 {
            if super::now_ns() >= deadline { return Err(ETIMEDOUT); }
            let _ = self.changed.wait_timeout(&mut wait, kernel::time::msecs_to_jiffies(10));
        }
        Ok(())
    }

    /// Join builders with no device, queue or mapping lock held.
    pub(super) fn wait_drained(&self) {
        let mut wait = self.wait.lock();
        while self.state.load(Ordering::Acquire) & COUNT_MASK != 0 {
            self.changed.wait(&mut wait);
        }
    }

    /// Reopen only an empty, temporary closure. Failed or removed devices remain closed.
    pub(super) fn reopen(&self) -> bool {
        let wait = self.wait.lock();
        let mut previous = self.state.load(Ordering::Acquire);
        loop {
            if previous & (REMOVED | FAILED | COUNT_MASK) != 0 {
                return false;
            }
            match self.state.compare_exchange_weak(
                previous,
                previous & !CLOSED,
                Ordering::AcqRel,
                Ordering::Acquire,
            ) {
                Ok(_) => {
                    self.changed.notify_all();
                    drop(wait);
                    self.wake_waiters();
                    return true;
                }
                Err(current) => previous = current,
            }
        }
    }
}

/// An off-lock builder's admission epoch. Keep after all of its tentative mappings.
pub(super) struct Lease {
    gate: Arc<Gate>,
    epoch: u64,
}

impl Lease {
    /// Must be checked under the device mutex immediately before publishing the built graph.
    pub(super) fn is_current(&self) -> bool {
        self.gate.state.load(Ordering::Acquire) & !COUNT_MASK == self.epoch
    }
}

impl Drop for Lease {
    fn drop(&mut self) {
        let previous = self.gate.state.fetch_sub(1, Ordering::AcqRel);
        if previous & COUNT_MASK == 1 && previous & CLOSED != 0 {
            // Pair the final decrement with the joiner's check/sleep lock to avoid a lost wakeup.
            let _wait = self.gate.wait.lock();
            self.gate.changed.notify_all();
        }
    }
}

#[kunit_tests(asahi_g17_growth_gate)]
mod growth_gate_tests {
    use super::*;
    #[test]
    fn recovery_can_join_between_growth_admissions() {
        let gate = Gate::new().unwrap();
        let outer = gate.try_enter().unwrap();
        // The growth handoff must drop this lease before enter can wait.
        drop(outer);
        gate.close();
        assert!(matches!(gate.try_enter(), Err(error) if error == EAGAIN));
        gate.wait_drained();
        assert!(gate.reopen());
        let growth = gate.enter(false).unwrap();
        assert!(growth.is_current());
        drop(growth);
    }
}

#[kunit_tests(asahi_g17_compute_deadline)]
mod compute_deadline_tests {
    use super::*;

    #[test]
    fn spent_budget_refuses_even_an_open_gate() {
        let gate = Gate::new().unwrap();
        let check = || Ok(());
        let admission = Admission::new(super::super::now_ns(), &check);
        assert!(matches!(gate.enter_until(&admission), Err(error) if error == EBUSY));
        assert_eq!(gate.state.load(Ordering::Acquire) & COUNT_MASK, 0);
    }

    #[test]
    fn closed_gate_expires_without_any_recovery_wake() {
        let gate = Gate::new().unwrap();
        gate.close();
        let start = super::super::now_ns();
        let check = || Ok(());
        let admission = Admission::new(start + 20_000_000, &check);
        assert!(matches!(gate.enter_until(&admission), Err(error) if error == EBUSY));
        assert!(super::super::now_ns() - start < 500_000_000);
        gate.wait_drained();
        assert!(gate.reopen());
    }

    #[test]
    fn cancellation_without_capacity_or_recovery_events() {
        let gate = Gate::new().unwrap();
        gate.close();
        let start = super::super::now_ns();
        let check = || {
            if super::super::now_ns() - start >= 10_000_000 { Err(ECANCELED) } else { Ok(()) }
        };
        let admission = Admission::new(start + 2_000_000_000, &check);
        assert!(matches!(gate.enter_until(&admission), Err(error) if error == ECANCELED));
        assert!(super::super::now_ns() - start < 500_000_000);
        assert_eq!(gate.state.load(Ordering::Acquire) & COUNT_MASK, 0);
    }

    #[test]
    fn recovery_and_binding_retry_churn_cannot_renew_budget() {
        let gate = Gate::new().unwrap();
        let deadline = super::super::now_ns() + 20_000_000;
        let check = || Ok(());
        let admission = Admission::new(deadline, &check);
        loop {
            // Re-entry models a channel taken between admission/publication.
            // Recovery joins each dropped lease, then wakes the same budget.
            gate.close();
            gate.changed.notify_all();
            gate.wait_drained();
            assert!(gate.reopen());
            // However often the gate reopened, an attempt that starts at the deadline
            // is refused. Timing the whole loop would also count time descheduled.
            let attempted = super::super::now_ns();
            match gate.enter_until(&admission) {
                Ok(lease) => {
                    assert!(attempted < deadline);
                    drop(lease);
                }
                Err(error) => { assert_eq!(error, EBUSY); break; }
            }
        }
        assert_eq!(gate.state.load(Ordering::Acquire) & COUNT_MASK, 0);
    }

    #[test]
    fn packet_status_and_cancel_precede_available_capacity() {
        let gate = Gate::new().unwrap();
        for error in [ECANCELED, EIO] {
            let check = || Err(error);
            let admission = Admission::new(u64::MAX, &check);
            assert!(matches!(gate.enter_until(&admission), Err(got) if got == error));
        }
        assert_eq!(gate.state.load(Ordering::Acquire) & COUNT_MASK, 0);
    }
}
