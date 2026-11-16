// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Work-based firmware performance effort. Idle power gating is managed separately by the
//! device-control ring; this input never keeps an otherwise idle GPU powered.

use core::sync::atomic::AtomicU32;

pub(super) const FULL_EFFORT: u32 = 0x10000;
pub(super) const IDLE_HOLD_NS: u64 = 16_000_000;

/// The firmware's idle power-off delay as measured on T6040/25G83: the cores power off 2 ms after
/// the last job. Not measured on other firmware; at or below it the hold stays off.
pub(crate) const FIRMWARE_IDLE_OFF_MS: u32 = 2;

/// The `idle_off_delay_ms` sysfs knob. Above [`FIRMWARE_IDLE_OFF_MS`] the driver keeps the GPU
/// cores powered until the GPU has been idle this long ([`Hold`]).
pub(crate) static IDLE_HOLD_MS: AtomicU32 = AtomicU32::new(FIRMWARE_IDLE_OFF_MS);

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(super) enum IdleAction {
    None,
    Clear,
    Recheck(u64),
}

pub(super) struct Effort {
    streaming: bool,
    idle_since: Option<u64>,
    applied: u32,
    submissions: u64,
}

impl Effort {
    pub(super) const fn new() -> Self {
        Self {
            streaming: false,
            idle_since: None,
            applied: 0,
            submissions: 0,
        }
    }

    pub(super) fn submit(&mut self) {
        self.streaming = true;
        self.idle_since = None;
        self.submissions = self.submissions.wrapping_add(1);
    }

    /// Count of published submissions, for [`Hold::step`].
    pub(super) fn submissions(&self) -> u64 {
        self.submissions
    }

    pub(super) fn applied(&self) -> u32 {
        self.applied
    }
    pub(super) fn did_apply(&mut self, value: u32) {
        self.applied = value;
    }

    /// Quarantined jobs do not count as work in flight; they retain mappings but cannot justify
    /// retaining the effort input. New submissions restart the continuous-idle interval.
    pub(super) fn idle(&mut self, now_ns: u64, in_flight: bool) -> IdleAction {
        if !self.streaming {
            return IdleAction::None;
        }
        if in_flight {
            self.idle_since = None;
            return IdleAction::None;
        }
        let since = *self.idle_since.get_or_insert(now_ns.max(1));
        let idle = now_ns.saturating_sub(since);
        if idle >= IDLE_HOLD_NS {
            self.streaming = false;
            self.idle_since = None;
            IdleAction::Clear
        } else {
            IdleAction::Recheck(IDLE_HOLD_NS - idle)
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(super) enum HoldAction {
    None,
    /// Send the keep-powered idle policy.
    Assert,
    /// Send the power-off idle policy again.
    Release,
    Recheck(u64),
}

/// Host-side idle hold. On T6040/25G83 the firmware powers the cores off 2 ms after the last job
/// and reads that delay only at boot; every wake-up then costs about 1 ms. With a longer
/// [`IDLE_HOLD_MS`] the host sends the keep-powered idle policy once work appears and the power-off
/// policy once no work has been submitted or in flight for that long. Checked from the event
/// worker, which runs on every completion, well inside the firmware's 2 ms.
pub(super) struct Hold {
    active: bool,
    seen: u64,
    since: Option<u64>,
}

impl Hold {
    pub(super) const fn new() -> Self {
        Self {
            active: false,
            seen: 0,
            since: None,
        }
    }

    pub(super) fn active(&self) -> bool {
        self.active
    }

    pub(super) fn step(
        &mut self,
        now_ns: u64,
        in_flight: bool,
        submissions: u64,
        hold_ms: u32,
    ) -> HoldAction {
        let enabled = hold_ms > FIRMWARE_IDLE_OFF_MS;
        let fresh = submissions != self.seen;
        self.seen = submissions;
        if in_flight || fresh {
            self.since = None;
        }
        if !self.active {
            return if enabled && (in_flight || fresh) {
                HoldAction::Assert
            } else {
                HoldAction::None
            };
        }
        if in_flight {
            return HoldAction::None;
        }
        let hold_ns = if enabled { u64::from(hold_ms) * 1_000_000 } else { 0 };
        let since = *self.since.get_or_insert(now_ns.max(1));
        let idle = now_ns.saturating_sub(since);
        if idle >= hold_ns {
            HoldAction::Release
        } else {
            HoldAction::Recheck(hold_ns - idle)
        }
    }

    pub(super) fn asserted(&mut self) {
        self.active = true;
    }

    pub(super) fn released(&mut self) {
        self.active = false;
        self.since = None;
    }
}

/// Single-producer control path under the device mutex. Event draining must not publish another
/// control between `set_idle` and the exact three-counter retirement witness.
pub(super) trait Control {
    fn crashed(&self) -> bool;
    fn powered(&self) -> bool;
    fn counters(&self) -> kernel::error::Result<[u32; 3]>;
    fn set_idle(&mut self, enabled: bool) -> kernel::error::Result<u32>;
    fn wait_tick(&self);
    fn registration_release_failed(&mut self) {}
    fn registration_release_succeeded(&mut self) {}
}

fn wait<C: Control>(control: &C, target: u32, require_powered: bool) -> kernel::error::Result {
    use kernel::prelude::*;
    const RETIRE_POLLS: usize = 20;
    const POWER_POLLS: usize = 250;
    let mut retired = false;
    for elapsed in 0..RETIRE_POLLS + POWER_POLLS {
        if control.crashed() {
            return Err(EIO);
        }
        if !retired {
            retired = control.counters()? == [target; 3];
            if !retired && elapsed + 1 >= RETIRE_POLLS {
                break;
            }
        }
        if retired && (!require_powered || control.powered()) {
            return Ok(());
        }
        control.wait_tick();
    }
    Err(ETIMEDOUT)
}

/// Disable idle descent through fresh pair registration, including a post-wake reassertion.
pub(super) fn acquire<C: Control>(control: &mut C) -> kernel::error::Result {
    use kernel::prelude::*;
    let result = (|| {
        for _ in 0..3 {
            let powered = control.powered();
            let target = control.set_idle(false)?;
            wait(control, target, true)?;
            if !powered {
                let target = control.set_idle(false)?;
                wait(control, target, true)?;
            }
            if control.powered() {
                return Ok(());
            }
        }
        Err(EAGAIN)
    })();
    if result.is_err() && release(control).is_err() {
        control.registration_release_failed();
    }
    result
}

pub(super) fn release<C: Control>(control: &mut C) -> kernel::error::Result {
    let target = match control.set_idle(true) {
        Ok(target) => target,
        Err(error) => {
            control.registration_release_failed();
            return Err(error);
        }
    };
    let result = wait(control, target, false);
    if result.is_ok() {
        control.registration_release_succeeded();
    } else {
        control.registration_release_failed();
    }
    result
}

#[cfg_attr(not(test), kernel::macros::kunit_tests(asahi_g17_idle_hold))]
mod tests {
    use super::*;

    #[test]
    fn holds_until_idle_for_the_delay_then_releases() {
        let mut hold = Hold::new();
        assert_eq!(hold.step(0, true, 1, 10), HoldAction::Assert);
        hold.asserted();
        assert_eq!(hold.step(1_000_000, false, 1, 10), HoldAction::Recheck(10_000_000));
        assert_eq!(hold.step(6_000_000, false, 1, 10), HoldAction::Recheck(5_000_000));
        // A new submission restarts the idle period.
        assert_eq!(hold.step(7_000_000, false, 2, 10), HoldAction::Recheck(10_000_000));
        assert_eq!(hold.step(17_000_000, false, 2, 10), HoldAction::Release);
        hold.released();
        assert!(!hold.active());
    }

    #[test]
    fn the_default_delay_never_holds_and_lowering_it_releases() {
        let mut hold = Hold::new();
        assert_eq!(hold.step(0, true, 1, FIRMWARE_IDLE_OFF_MS), HoldAction::None);
        assert_eq!(hold.step(0, true, 2, 10), HoldAction::Assert);
        hold.asserted();
        assert_eq!(hold.step(1, false, 2, FIRMWARE_IDLE_OFF_MS), HoldAction::Release);
    }
}
