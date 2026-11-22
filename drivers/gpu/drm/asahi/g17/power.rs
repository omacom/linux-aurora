// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Work-based firmware performance effort. Idle power gating is managed separately by the
//! device-control ring; this input never keeps an otherwise idle GPU powered.

use core::sync::atomic::AtomicU32;

pub(super) const FULL_EFFORT: u32 = 0x10000;
pub(super) const IDLE_HOLD_NS: u64 = 16_000_000;

/// The firmware's idle power-off delay as measured on T6040/25G83: the cores power off 2 ms after
/// the last job. Not measured on other firmware; at or below it the hold stays off.
pub(crate) const FIRMWARE_IDLE_OFF_MS: u32 = 2;

/// Idle policy initialized from the existing DT `apple,idle-off-delay-ms` property.
/// The existing `idle_off_delay_ms` sysfs attribute may override it. Above
/// [`FIRMWARE_IDLE_OFF_MS`] the driver keeps the GPU cores powered until it has been idle
/// this long ([`Hold`]).
pub(crate) static IDLE_HOLD_MS: AtomicU32 = AtomicU32::new(FIRMWARE_IDLE_OFF_MS);

/// Use the same bounds as the existing idle-delay attribute. Missing DT policy
/// preserves the native delay; malformed or out-of-range values fail probe.
pub(super) fn idle_delay(requested: Option<u32>) -> kernel::error::Result<u32> {
    let delay = requested.unwrap_or(FIRMWARE_IDLE_OFF_MS);
    if !(FIRMWARE_IDLE_OFF_MS..=1000).contains(&delay) {
        return Err(kernel::error::code::EINVAL);
    }
    Ok(delay)
}

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
    /// The firmware's effort word is known to hold `applied`: the host wrote it, or read it.
    confirmed: bool,
    submissions: u64,
    /// A host thread waits, or waited within [`IDLE_HOLD_NS`], for submitted work
    /// ([`Self::note_waiters`]).
    waited: bool,
    /// The waits begun when the waiters were last noted.
    seen_waits: u64,
    /// When a waiter was last seen.
    last_wait_ns: u64,
    /// The GPU has run longer than [`IDLE_HOLD_NS`] without an idle gap ([`BusyRun`]).
    bound: bool,
}

impl Effort {
    pub(super) const fn new() -> Self {
        Self {
            streaming: false,
            idle_since: None,
            applied: 0,
            confirmed: false,
            submissions: 0,
            waited: false,
            seen_waits: 0,
            last_wait_ns: 0,
            bound: false,
        }
    }

    pub(super) fn streaming(&self) -> bool {
        self.streaming
    }

    /// Whether a host thread waits for the work ([`Self::note_waiters`]).
    pub(super) fn waited(&self) -> bool {
        self.waited
    }

    /// Notes the waiters on submitted work at `now_ns` ([`super::fence::waiters`]): a thread
    /// waits now or began a wait since the last note. The note holds for [`IDLE_HOLD_NS`] after
    /// the last waiter, across a submit-and-wait loop's turns between its waits. Returns whether
    /// that changed.
    pub(super) fn note_waiters(&mut self, now_ns: u64, (waiters, waits): (u32, u64)) -> bool {
        if waiters > 0 || waits != self.seen_waits {
            self.last_wait_ns = now_ns.max(1);
        }
        self.seen_waits = waits;
        let waited = self.last_wait_ns != 0 && now_ns.saturating_sub(self.last_wait_ns) < IDLE_HOLD_NS;
        let changed = waited != self.waited;
        self.waited = waited;
        changed
    }

    /// Whether the GPU has run without an idle gap for longer than the effort is held after
    /// work stops ([`BusyRun::bound`]).
    pub(super) fn bound(&self) -> bool {
        self.bound
    }

    /// Notes [`BusyRun::bound`]; returns whether that changed.
    pub(super) fn note_bound(&mut self, bound: bool) -> bool {
        let changed = bound != self.bound;
        self.bound = bound;
        changed
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
        self.confirmed = true;
    }
    pub(super) fn confirmed(&self) -> bool {
        self.confirmed
    }
    pub(super) fn confirm(&mut self) {
        self.confirmed = true;
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
            self.waited = false;
            self.bound = false;
            self.last_wait_ns = 0;
            self.idle_since = None;
            IdleAction::Clear
        } else {
            IdleAction::Recheck(IDLE_HOLD_NS - idle)
        }
    }
}

/// How long each client's commands have kept the GPU running without going idle, from the start
/// and end timestamps the firmware writes for every command it executes.
///
/// The firmware raises its performance state for work that queues up behind the running command,
/// not for how busy the GPU is: a client that submits one command and waits for it before the
/// next keeps the GPU busy with nothing queued, and the state stays at the bottom. Such a loop is
/// told apart from paced work (a frame, then idle until the next one) by its idle gaps. Its GPU
/// only idles for the host's turnaround, shorter than the idle time after which the firmware
/// powers the GPU off ([`FIRMWARE_IDLE_OFF_MS`]), and a gap at least that long ends a run. The
/// host's own hold ([`IDLE_HOLD_MS`]) can be longer than a frame, so it doesn't mark idle. Once
/// a run lasts longer than the effort is held after work stops ([`IDLE_HOLD_NS`]), the GPU is the
/// bottleneck and gets the full effort ([`Effort::note_bound`]). A shorter run would not be worth
/// a floor that outlasts it.
///
/// Runs are kept per client (DRM context). Commands of different clients run side by side and
/// complete in any order: a short command of one client that starts part-way through another
/// client's long render completes first, while the render that keeps the GPU busy through the
/// gap before it has not been noted yet. One run for the whole GPU took that for idle and
/// restarted, the more often the longer the commands, and could lock a GPU-bound client at the
/// floor. A client's own commands follow each other, so its own gaps are real.
///
/// A run counts while its client is still running: until it has been silent for longer than its
/// last command took plus the idle power-off delay, measured on the clock the firmware stamps
/// commands with. A client that stops submitting does not keep the full effort for others.
pub(super) struct BusyRuns {
    runs: [BusyRun; CLIENT_RUNS],
}

/// Clients followed at once; a new client takes the place of the one that ran least recently.
const CLIENT_RUNS: usize = 16;

#[derive(Copy, Clone)]
struct BusyRun {
    /// The entry follows a client.
    used: bool,
    /// The client's execution context id.
    client: u32,
    /// Start of the run, in firmware timestamp ticks.
    start: u64,
    /// The latest end noted.
    last_end: u64,
    /// How long the client's latest command ran.
    last_busy: u64,
}

impl BusyRun {
    const EMPTY: Self = Self {
        used: false,
        client: 0,
        start: 0,
        last_end: 0,
        last_busy: 0,
    };
}

impl BusyRuns {
    pub(super) const fn new() -> Self {
        Self {
            runs: [BusyRun::EMPTY; CLIENT_RUNS],
        }
    }

    /// Notes a command of `client` that executed from `start` to `end`. A start at least
    /// `idle_ticks` (the firmware's idle power-off delay) after every end noted for the client
    /// begins a new run.
    pub(super) fn note(&mut self, client: u32, start: u64, end: u64, idle_ticks: u64) {
        if start == 0 || end <= start {
            return;
        }
        let index = self
            .runs
            .iter()
            .position(|run| run.used && run.client == client)
            .or_else(|| {
                (0..CLIENT_RUNS).min_by_key(|&index| {
                    let run = &self.runs[index];
                    (run.used, run.last_end)
                })
            })
            .unwrap_or(0);
        let run = &mut self.runs[index];
        if !run.used || run.client != client || start >= run.last_end.saturating_add(idle_ticks) {
            *run = BusyRun {
                used: true,
                client,
                start,
                last_end: end,
                last_busy: end - start,
            };
            return;
        }
        run.last_end = run.last_end.max(end);
        run.last_busy = end - start;
    }

    /// Whether some client, still running at `now`, has kept the GPU running for at least
    /// `hold_ticks`.
    pub(super) fn bound(&self, now: u64, hold_ticks: u64, idle_ticks: u64) -> bool {
        self.runs.iter().any(|run| {
            run.used
                && run.last_end.saturating_sub(run.start) >= hold_ticks
                && now.saturating_sub(run.last_end) < run.last_busy.saturating_add(idle_ticks)
        })
    }

    /// The GPU went idle for longer than the effort hold: every client's next command begins a
    /// new run.
    pub(super) fn end(&mut self) {
        *self = Self::new();
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

    // 24 MHz ticks: a millisecond, the 2 ms idle power-off delay and the 16 ms hold.
    const MS: u64 = 24_000;
    const IDLE: u64 = 2 * MS;
    const HOLD: u64 = 16 * MS;

    /// `count` commands of `client` of `busy` ticks from `start`, each `gap` ticks after the
    /// previous one ended. Returns where the next one would start.
    fn commands(
        runs: &mut BusyRuns,
        client: u32,
        mut start: u64,
        count: u64,
        busy: u64,
        gap: u64,
    ) -> u64 {
        for _ in 0..count {
            runs.note(client, start, start + busy, IDLE);
            start += busy + gap;
        }
        start
    }

    #[test]
    fn a_submit_and_wait_loop_becomes_bound_once_it_outlasts_the_hold() {
        // 1 ms commands, 100 us host turnarounds.
        let mut runs = BusyRuns::new();
        let next = commands(&mut runs, 1, 1000, 10, MS, MS / 10);
        assert!(!runs.bound(next, HOLD, IDLE));
        let next = commands(&mut runs, 1, next, 20, MS, MS / 10);
        assert!(runs.bound(next, HOLD, IDLE));
    }

    #[test]
    fn paced_work_never_becomes_bound() {
        // 2 ms of work every 16.7 ms: each gap is longer than the idle power-off delay.
        let mut runs = BusyRuns::new();
        let next = commands(&mut runs, 1, 1000, 100, 2 * MS, 14 * MS);
        assert!(!runs.bound(next, HOLD, IDLE));
        // A frame whose work leaves less idle than the power-off delay is bound: the GPU is.
        let next = commands(&mut runs, 1, next + 14 * MS, 20, 15 * MS, MS);
        assert!(runs.bound(next, HOLD, IDLE));
    }

    #[test]
    fn another_clients_command_does_not_end_a_run() {
        // Back-to-back 6 ms renders of client 1 keep the GPU busy. 4 ms into each, a 0.2 ms
        // command of client 2 starts and completes first, more than the power-off delay after
        // client 1's previous render ended.
        let mut runs = BusyRuns::new();
        let mut at = 1000;
        for _ in 0..4 {
            runs.note(2, at + 4 * MS, at + 4 * MS + MS / 5, IDLE);
            assert!(!runs.bound(at + 4 * MS + MS / 5, 4 * 6 * MS, IDLE));
            runs.note(1, at, at + 6 * MS, IDLE);
            at += 6 * MS;
        }
        assert!(runs.bound(at, HOLD, IDLE));
        // Still bound while the next render runs and client 2's command completes.
        runs.note(2, at + 4 * MS, at + 4 * MS + MS / 5, IDLE);
        assert!(runs.bound(at + 5 * MS, HOLD, IDLE));
    }

    #[test]
    fn a_client_that_stops_does_not_stay_bound() {
        let mut runs = BusyRuns::new();
        let next = commands(&mut runs, 1, 1000, 30, MS, MS / 10);
        assert!(runs.bound(next, HOLD, IDLE));
        // Silent for its last command plus the power-off delay while another client streams.
        let quiet = next + MS + IDLE;
        commands(&mut runs, 2, next, 4, MS, 4 * MS);
        assert!(!runs.bound(quiet, HOLD, IDLE));
    }

    #[test]
    fn the_least_recent_client_is_replaced_and_end_resets_every_run() {
        let mut runs = BusyRuns::new();
        let next = commands(&mut runs, 1, 1000, 30, MS, MS / 10);
        for client in 2..=CLIENT_RUNS as u32 + 1 {
            runs.note(client, next, next + 1, IDLE);
        }
        // Client 1's entry was the least recent and went to the last new client.
        assert!(!runs.bound(next, HOLD, IDLE));
        let next = commands(&mut runs, 1, next, 30, MS, MS / 10);
        assert!(runs.bound(next, HOLD, IDLE));
        runs.end();
        assert!(!runs.bound(next, 0, IDLE));
        runs.note(1, 0, 10, IDLE);
        runs.note(1, 20, 20, IDLE);
        assert!(!runs.bound(20, 0, IDLE));
    }

    #[test]
    fn effort_bound_is_cleared_with_the_effort() {
        let mut effort = Effort::new();
        effort.submit();
        assert!(effort.note_bound(true));
        assert!(!effort.note_bound(true));
        assert_eq!(effort.idle(1, false), IdleAction::Recheck(IDLE_HOLD_NS));
        assert_eq!(effort.idle(1 + IDLE_HOLD_NS, false), IdleAction::Clear);
        assert!(!effort.bound());
    }

    #[test]
    fn dt_idle_delay_is_bounded_and_absence_preserves_native_policy() {
        assert_eq!(idle_delay(None), Ok(FIRMWARE_IDLE_OFF_MS));
        for delay in [2, 16, 1000] {
            assert_eq!(idle_delay(Some(delay)), Ok(delay));
        }
        for delay in [0, 1, 1001, u32::MAX] {
            assert!(idle_delay(Some(delay)).is_err());
        }
    }

    #[test]
    fn frame_hold_survives_depth_one_gaps_but_releases_at_idle() {
        let mut hold = Hold::new();
        assert_eq!(hold.step(0, true, 1, 16), HoldAction::Assert);
        hold.asserted();
        for submission in 1..=8 {
            // A completed depth-one dispatch followed by another short gap.
            let now = submission * 3_000_000;
            assert_eq!(hold.step(now, false, submission, 16), HoldAction::Recheck(16_000_000));
            assert!(hold.active());
        }
        assert_eq!(hold.step(39_999_999, false, 8, 16), HoldAction::Recheck(1));
        assert_eq!(hold.step(40_000_000, false, 8, 16), HoldAction::Release);
        hold.released();
        assert!(!hold.active());
    }

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

unsafe extern "C" {
    fn asahi_clpc_gpu_get(dev: *mut core::ffi::c_void, out: *mut *mut core::ffi::c_void) -> core::ffi::c_int;
    fn asahi_clpc_gpu_set_active(gpu: *mut core::ffi::c_void, active: bool);
    fn asahi_clpc_gpu_put(gpu: *mut core::ffi::c_void);
}

/// DT-selected memory-demand provider. Its independent allocation outlives a
/// detached provider; a device link orders supplier removal and suspend.
pub(super) struct Demand(*mut core::ffi::c_void);

// SAFETY: the provider's leaf spinlock serializes all access; no GPU callback.
unsafe impl Send for Demand {}
// SAFETY: shared access only changes that locked request, never its lifetime.
unsafe impl Sync for Demand {}

impl Demand {
    pub(super) fn new(dev: *mut kernel::bindings::device) -> kernel::error::Result<Self> {
        let mut out = core::ptr::null_mut();
        // SAFETY: live platform device and writable result; C returns an owned reference.
        let ret = unsafe { asahi_clpc_gpu_get(dev.cast(), &mut out) };
        if ret < 0 {
            Err(kernel::error::Error::from_errno(ret))
        } else {
            Ok(Self(out))
        }
    }

    pub(super) fn set_active(&self, active: bool) {
        // SAFETY: the retained reference is live; NULL denotes no DT provider.
        unsafe { asahi_clpc_gpu_set_active(self.0, active) };
    }
}

impl Drop for Demand {
    fn drop(&mut self) {
        // SAFETY: releases exactly the reference returned by get, clearing demand first.
        unsafe { asahi_clpc_gpu_put(self.0) };
    }
}
