// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Hang detection, kills, the halt/resume handshake and fail-stop.
//!
//! The pure parts are in [`logic`] (host-tested); [`seams`] adapts the engine, register and VM
//! state they decide on.
//!
//! - Host deadline `g15_deadline_ms` (5000) from the oldest prefix's start or last progress
//!   (a firmware stamp moving on an engine where that prefix still has work), renewed at every
//!   retirement. At expiry with `HANG_PREEMPT`: arm the kill for the oldest prefix (compute
//!   divisor +0x1c while its computes are unfinished, else fragment +0x24).
//! - Compute deadline `g15_compute_deadline_ms` (1000: a compute may run for up to 1 s): the
//!   oldest prefix's first unfinished compute has its predecessors done, its FS wait reached,
//!   its record consumed and the work ring READ past it, with no progress: class 2 -> kill;
//!   class 0 -> fail-stop. Measured from the started compute: the total order never blames a
//!   job for another VM's older work.
//! - Kill: Globals divisor = 1, DEVCTL 0x13 posted without waiting; publication frozen; 2 s
//!   budget: (a) the prefix retires -> ETIMEDOUT, VM lost, later same-VM prefixes EIO; render
//!   allocators re-initialized only for a fragment kill; waited disarm before publishing
//!   anything; (b) an error event -> handshake, then disarm; (c) timeout -> fail-stop.
//! - Handshake on an error event or receipt fault: drop pending growth requests; wait up to
//!   200 x ~1-1.5 ms for Control+0x4590 = 1 while draining; write halted = 0, barrier,
//!   resume = 1; wait <= 500 ms for the firmware to clear resume; wait <= 1000 polls for a full
//!   drain, answering up to 4 further halts the same way. Then fail every published prefix
//!   (EIO on fault, ETIMEDOUT on timeout) and latch the VM error; if any had renders, set
//!   fs_resync and reinit so the next first TA waits on the FS stamp actually reached and an
//!   exclusive priming prefix rebuilds the render allocators. A firmware error with nothing
//!   published is answered and blames nobody.
//! - Fail-stop (latched until reboot, every VM EIO, published owners retained): publication
//!   error after a queue WRITE advanced; recovery off, RTKit crash, no halt, resume not
//!   consumed, not drained, more than 4 further halts, failed disarm; deadline without event
//!   and no effective kill; class-0 compute hang; unanswerable TA stall.
//!
//! # The poll pass (poll.rs)
//!
//! 1. Failed ([`Recovery::failed`]): nothing.
//! 2. Nothing published and [`Firmware::recovery_idle`]: nothing, no repoll.
//! 3. Drain and retire. For each retired oldest prefix, [`Firmware::retire_outcome`] gives its
//!    result (the kill outcome (a), or EIO for a doomed prefix); a receipt fault sets
//!    [`Recovery::receipt_fault`].
//! 4. [`Firmware::answer_growth`] (pools.rs).
//! 5. [`Firmware::check_hangs`]: waited disarm after an ended kill, progress, compute hang,
//!    host deadline, recovery or fail-stop. `Err` = fail-stop done (flights failed, VMs failed,
//!    pending drained): return it so `Shared::fail_stop` latches.
//! 6. Publish (frozen while [`Recovery::armed`]); repoll while anything is published or armed.

pub(crate) mod logic;
mod seams;

use kernel::{
    bindings, device,
    prelude::*,
    sync::Arc,
    time::{ClockSource, Monotonic},
};

use logic::{Failure, HangAction, HangInputs};

use super::fw::channels::{devctl, DevCtl, RING_SLOTS};
use super::fw::initdata::{control, globals};
use super::hw::{sizes, tuning};
use super::job::{Completions, Job};
use super::{params, Firmware};

static_assert!(logic::COMPUTE_KILL == globals::COMPUTE_KILL);
static_assert!(logic::FRAGMENT_KILL == globals::FRAGMENT_KILL);
static_assert!(logic::COMPUTE_MONITOR == globals::COMPUTE_MONITOR);
static_assert!(logic::QUEUE_CAP == sizes::QUEUE_CAP);
static_assert!(logic::CHANNEL_SLOTS == RING_SLOTS);
static_assert!(logic::TA == super::channel::Channel::Ta as usize);
static_assert!(logic::FS == super::channel::Channel::Fs as usize);
static_assert!(logic::COMPUTE == super::channel::Channel::Compute as usize);

/// Monotonic time, ns.
pub(crate) fn now_ns() -> u64 {
    <Monotonic as ClockSource>::ktime_get() as u64
}

/// One recovery poll interval, 1-1.5 ms.
fn poll_sleep() {
    // SAFETY: Sleeping is allowed here: the poll work and the scheduler's run path hold only
    // the device mutex.
    unsafe { bindings::usleep_range_state(1000, 1500, bindings::TASK_UNINTERRUPTIBLE) };
}

/// Orders firmware-visible stores and loads: a full `dsb sy`, at least as strong as `wmb()`,
/// `mb()` and `smp_rmb()`.
fn barrier() {
    crate::mem::sync();
}

/// A result as logged: 0 or a negative errno.
fn errno(result: &Result) -> i32 {
    match result {
        Ok(()) => 0,
        Err(error) => error.to_errno(),
    }
}

/// Queues `job`'s completion once. Without memory for the entry the job stays referenced
/// forever and never completes (logged).
pub(crate) fn push_completion(done: &mut Completions, job: &Arc<Job>, result: Result) {
    if done.iter().any(|(queued, _)| Arc::ptr_eq(queued, job)) {
        return;
    }
    if done.push((job.clone(), result), GFP_KERNEL).is_err() {
        pr_err!(
            "G15: no memory to complete job {}; it is retained\n",
            job.id
        );
        core::mem::forget(job.clone());
    }
}

/// Hang and recovery state.
pub(crate) struct Recovery {
    /// Host deadline and compute no-progress deadline, monotonic ns; [`logic::EXPIRED`] (0)
    /// has passed; renewed by [`Firmware::renew_deadlines`].
    pub(crate) deadline_ns: u64,
    pub(crate) compute_deadline_ns: u64,
    /// An armed kill: the Globals divisor offset (0x1c compute, 0x24 fragment) and when it was
    /// armed. Publication is frozen while it is set (`Engine::frozen` mirrors it).
    pub(crate) armed: Option<(usize, u64)>,
    /// The prefix the kill was armed for retired: the disarm is pending.
    pub(crate) kill_ended: bool,
    /// After a recovered render abort: the next first TA waits on the FS stamp reached, and an
    /// exclusive priming prefix rebuilds the render allocators. The engine clears both when
    /// it publishes renders.
    pub(crate) fs_resync: bool,
    pub(crate) reinit: bool,
    /// Firmware error events already answered.
    pub(crate) errors_seen: u32,
    /// The retirement check of this poll pass saw a fault; set by the poll pass, consumed by
    /// [`Firmware::check_hangs`].
    pub(crate) receipt_fault: bool,
    /// The latched backend failure: nothing is accepted or published again.
    pub(crate) failed: Option<Error>,
    /// Statistics (debugfs keys `recoveries`, `hang_kills`).
    pub(crate) recoveries: u64,
    pub(crate) hang_kills: u64,
}

impl Recovery {
    pub(crate) fn new() -> Self {
        Self {
            deadline_ns: logic::EXPIRED,
            compute_deadline_ns: logic::EXPIRED,
            armed: None,
            kill_ended: false,
            fs_resync: false,
            reinit: false,
            errors_seen: 0,
            receipt_fault: false,
            failed: None,
            recoveries: 0,
            hang_kills: 0,
        }
    }

    /// The latched backend failure, if any.
    pub(crate) fn failed(&self) -> Option<Error> {
        self.failed
    }

    /// The poll must run again soon besides any published prefix: a kill is armed.
    pub(crate) fn needs_repoll(&self) -> bool {
        self.armed.is_some()
    }
}

impl Firmware {
    /// The GPU device, for logging.
    pub(crate) fn dev(&self) -> &device::Device {
        &self.pools.dev
    }

    /// Renews the host and compute deadlines from now: at the publication of a prefix with
    /// nothing in flight, at every retirement, and on progress.
    pub(crate) fn renew_deadlines(&mut self) {
        let (deadline, compute) = logic::renew(
            now_ns(),
            params::deadline_ms(),
            params::compute_deadline_ms(),
        );
        self.recovery.deadline_ns = deadline;
        self.recovery.compute_deadline_ns = compute;
    }

    /// Firmware error events arrived that recovery has not answered.
    fn new_errors(&self) -> bool {
        self.events.errors != self.recovery.errors_seen
    }

    /// Nothing for the poll pass to do when nothing is published: no kill armed (whose prefix
    /// ended) and no firmware error event to answer (the pass is skipped without a repoll).
    pub(crate) fn recovery_idle(&self) -> bool {
        self.recovery.armed.is_none() && !self.new_errors()
    }

    /// `(recoveries, hang_kills)` for `asahi-g15/stats`.
    pub(crate) fn recovery_counts(&self) -> (u64, u64) {
        (self.recovery.recoveries, self.recovery.hang_kills)
    }

    /// The result of the oldest published prefix, which just retired.
    /// Retired after an armed request (the first such retirement): the firmware ended it.
    /// Its job fails with ETIMEDOUT and its client's VM is lost, like a hang reset; later
    /// prefixes (same VM) may have been ended by the request too before it is restored: they
    /// fail with EIO. A compute request ends compute work only: the prefix's renders completed
    /// with their own receipts and the render allocators stay; only a fragment kill of a
    /// prefix with renders re-initializes them. Otherwise a doomed prefix fails with EIO.
    pub(crate) fn retire_outcome(&mut self) -> Result {
        let doomed = self.engine.flights.first().is_some_and(|f| f.doomed);
        let offset = match self.recovery.armed {
            Some((offset, _)) if !self.recovery.kill_ended => offset,
            _ => return if doomed { Err(EIO) } else { Ok(()) },
        };
        self.recovery.kill_ended = true;
        self.recovery.hang_kills += 1;
        if self.receipts(0).renders != 0 && offset == logic::FRAGMENT_KILL {
            // Aborted scene: PB state untrusted.
            self.recovery.reinit = true;
        }
        for flight in self.engine.flights.iter_mut().skip(1) {
            flight.doomed = true;
        }
        let ctx = self.engine.flights.first().map_or(0, |f| f.job.vm.slot);
        let kind = if offset == logic::FRAGMENT_KILL {
            "fragment"
        } else {
            "compute"
        };
        dev_warn!(
            self.dev(),
            "hung submission of ctx {} ended by the firmware's {} request\n",
            ctx,
            kind
        );
        Err(ETIMEDOUT)
    }

    /// The hang and recovery branches of the poll pass, after retirement and growth. `Err`
    /// means the backend fail-stopped here: published prefixes failed and retained, every VM
    /// failed, pending jobs failed; the caller latches [`crate::g15::Shared::fail_stop`] and
    /// publishes nothing.
    pub(crate) fn check_hangs(&mut self, done: &mut Completions) -> Result {
        if let Some(error) = self.recovery.failed {
            return Err(error);
        }
        let fault = core::mem::take(&mut self.recovery.receipt_fault);
        // The request's prefix ended: restore the divisor (and wait for its import) before
        // anything is published, unless a firmware error arrived meanwhile: the firmware may
        // have halted, and the recovery below answers it first, then restores the divisor.
        if self.recovery.kill_ended
            && !fault
            && self.transport_error().is_none()
            && !self.new_errors()
            && self.preempt_disarm().is_err()
        {
            // Not restored: fail below.
            self.recovery.deadline_ns = logic::EXPIRED;
        }
        let flight = !self.engine.flights.is_empty();
        if flight && self.recovery.armed.is_none() {
            self.progress();
        }
        let now = now_ns();
        let inputs = HangInputs {
            flight,
            armed: self.recovery.armed.is_some(),
            fault,
            transport: self.transport_error().is_some(),
            new_errors: self.new_errors(),
            compute_deadline_expired: logic::expired(now, self.recovery.compute_deadline_ns),
            deadline_expired: logic::expired(now, self.recovery.deadline_ns),
            class: tuning::COMPUTE_QUEUE_CLASS,
            recovery: params::recovery(),
            hang_preempt: tuning::HANG_PREEMPT,
        };
        let decision = logic::decide(&inputs, || self.compute_hang_now());
        if decision.compute_hang {
            self.report("compute hang");
        }
        let ctx = self.engine.flights.first().map_or(0, |f| f.job.vm.slot);
        if decision.class0_hang {
            dev_warn!(
                self.dev(),
                "class-0 compute hang of ctx {} (not preemptible): failing\n",
                ctx
            );
            // Take the fail-stop branch.
            self.recovery.deadline_ns = logic::EXPIRED;
        }
        match decision.action {
            HangAction::Idle | HangAction::Wait => Ok(()),
            HangAction::ArmCompute => {
                // The started compute made no progress for its budget, with no firmware
                // event: ask the firmware to end it; publication stays frozen.
                dev_warn!(
                    self.dev(),
                    "compute of ctx {} made no progress for {} ms: asking the firmware to end it\n",
                    ctx,
                    params::compute_deadline_ms()
                );
                let _ = self.kill_arm(logic::COMPUTE_KILL);
                Ok(())
            }
            HangAction::ArmDeadline => {
                // Host deadline, no firmware event: let the firmware kill it.
                self.report("hung, requesting preemption");
                let offset = self.kill_offset_now();
                let _ = self.kill_arm(offset);
                Ok(())
            }
            HangAction::Fail { recover } => self.fail_or_recover(recover, fault, done),
        }
    }

    /// The failure branch: a firmware error event or a receipt fault with a live transport
    /// waits for the firmware's own halt and answers it; every published prefix fails with it.
    /// A hang in the render-only allocator rebuild prefix recovers like any render hang and
    /// re-primes. An armed request is restored after the resume, before
    /// anything is published; if it cannot be, the GPU stops. Anything else fail-stops.
    fn fail_or_recover(&mut self, recover: bool, fault: bool, done: &mut Completions) -> Result {
        let flight = !self.engine.flights.is_empty();
        let ctx = self.engine.flights.first().map_or(0, |f| f.job.vm.slot);
        self.report(if flight {
            "prefix FAILED"
        } else {
            "firmware error with no prefix published"
        });
        let render = (0..self.engine.flights.len()).any(|i| self.receipts(i).renders != 0);
        if recover {
            let (error, mut recovered) = self.recover();
            if recovered && self.preempt_disarm().is_err() {
                recovered = false;
            }
            if recovered {
                self.recovery.errors_seen = self.events.errors;
                self.recovery.recoveries += 1;
                if render {
                    self.recovery.fs_resync = true;
                    self.recovery.reinit = true;
                }
                dev_warn!(
                    self.dev(),
                    "recovered: ctx {} submission failed with {}\n",
                    ctx,
                    error.to_errno()
                );
                // The failing client's VM is lost; other clients continue.
                while !self.engine.flights.is_empty() {
                    self.retire_oldest(Err(error), done);
                }
                return Ok(());
            }
        }
        let error = if fault {
            EIO
        } else {
            self.transport_error().unwrap_or(ETIMEDOUT)
        };
        self.fail_stop(error, done);
        Err(error)
    }

    /// Stops the backend until reboot: restores an armed divisor if possible, latches the
    /// failure, fails and retains every published prefix, fails every live VM with EIO and
    /// fails the pending jobs (retaining those partly published). The caller returns the error
    /// so that [`crate::g15::Shared::fail_stop`] latches the device; a job the caller holds
    /// outside `engine.pending` (the job whose publication failed) is the caller's to fail and
    /// retain.
    pub(crate) fn fail_stop(&mut self, error: Error, done: &mut Completions) {
        let _ = self.preempt_disarm();
        if self.recovery.failed.is_none() {
            self.recovery.failed = Some(error);
        }
        // Published and uncertain: never release their owners.
        self.fail_flights(error, done);
        // No recovery: every VM is lost, also those with no failed job.
        self.fail_all_vms(EIO);
        let pending = core::mem::take(&mut self.engine.pending);
        for job in pending.iter() {
            push_completion(done, job, Err(EIO));
            // A partly published job already ran commands: its owners stay retained.
            if self.job_partly_published(job) {
                self.retain(job);
            }
        }
    }

    /// Fails every published prefix: their jobs fail (each once) and, with every owner the
    /// firmware may still reference, are retained. The prefixes' storage is never released:
    /// the flights are leaked.
    pub(crate) fn fail_flights(&mut self, error: Error, done: &mut Completions) {
        let flights = core::mem::take(&mut self.engine.flights);
        for flight in flights.iter() {
            for job in Iterator::chain(core::iter::once(&flight.job), flight.group.iter()) {
                push_completion(done, job, Err(error));
                self.retain(job);
            }
        }
        core::mem::forget(flights);
    }

    /// Keeps `job` until reboot: its owners (VM mappings, storage) stay alive because the
    /// firmware may still reference them.
    pub(crate) fn retain(&mut self, job: &Arc<Job>) {
        if self
            .engine
            .retained
            .iter()
            .any(|kept| Arc::ptr_eq(kept, job))
        {
            return;
        }
        if self.engine.retained.push(job.clone(), GFP_KERNEL).is_err() {
            core::mem::forget(job.clone());
        }
    }

    /// The started-compute hang test on the oldest prefix.
    fn compute_hang_now(&self) -> bool {
        if self.engine.flights.is_empty() {
            return false;
        }
        match self.observe() {
            Ok(observed) => logic::compute_hang(&self.receipts(0), &self.tail(), &observed),
            Err(_) => false,
        }
    }

    /// The divisor the host deadline arms for the oldest prefix.
    fn kill_offset_now(&self) -> usize {
        let observed = self.observe().unwrap_or_default();
        logic::kill_offset(&self.receipts(0), &self.tail(), &observed)
    }

    /// Firmware DONE/READ reached the newest published ends, channels consumed, engines idle.
    fn drained(&mut self) -> bool {
        let tail = self.tail();
        match self.observe() {
            Ok(observed) if logic::queues_drained(&tail, &observed) => {
                matches!(self.engine_state(), Ok(state) if !state.busy)
            }
            _ => false,
        }
    }

    fn globals_w32(&self, offset: usize, value: u32) -> Result {
        self.init.globals.as_ref().ok_or(ENODEV)?.w32(offset, value)
    }

    fn control_r32(&self, offset: usize) -> Result<u32> {
        self.init.control.as_ref().ok_or(ENODEV)?.r32(offset)
    }

    fn control_w32(&self, offset: usize, value: u32) -> Result {
        self.init.control.as_ref().ok_or(ENODEV)?.w32(offset, value)
    }

    /// Writes a Globals divisor and waits for its import.
    pub(crate) fn preempt_set(&mut self, offset: usize, divisor: u32) -> Result {
        let ret = self.globals_w32(offset, divisor).and_then(|()| {
            barrier();
            self.devctl(&DevCtl::op(devctl::IMPORT_GLOBALS), true)
        });
        dev_info!(
            self.dev(),
            "preemption divisor Globals+{:#x}={}: {}\n",
            offset,
            divisor,
            errno(&ret)
        );
        ret
    }

    /// The class-2 compute profile: the unchanged-work compute monitor off (Globals +0x9b0 = 0,
    /// waited import) before the compute queue's first exposure (the periodic request +0x1c is
    /// 0 in the image). Called once by the backend start after the graphs are initialized.
    pub(crate) fn compute_monitor_off(&mut self) -> Result {
        if tuning::COMPUTE_QUEUE_CLASS == 2 {
            self.preempt_set(logic::COMPUTE_MONITOR, 0)
        } else {
            Ok(())
        }
    }

    /// Arms the hang request: divisor 1 at Globals `offset`, DEVCTL 0x13 posted without
    /// waiting (the firmware may halt before it consumes it), publication frozen until the
    /// outcome, whose budget is 2 s (none if the post failed).
    fn kill_arm(&mut self, offset: usize) -> Result {
        let ret = self.globals_w32(offset, 1).and_then(|()| {
            barrier();
            self.devctl(&DevCtl::op(devctl::IMPORT_GLOBALS), false)
        });
        let now = now_ns();
        self.recovery.armed = Some((offset, now));
        self.recovery.kill_ended = false;
        self.engine.frozen = true;
        self.recovery.deadline_ns = logic::kill_deadline(now, ret.is_ok());
        dev_info!(
            self.dev(),
            "hang request Globals+{:#x}=1: {}\n",
            offset,
            errno(&ret)
        );
        ret
    }

    /// Restores the armed divisor to 0 and waits for the import before anything is published
    /// again (an earlier arming record is consumed first). On failure the request stays armed
    /// and publication frozen.
    fn preempt_disarm(&mut self) -> Result {
        let Some((offset, _)) = self.recovery.armed else {
            return Ok(());
        };
        self.preempt_set(offset, 0)?;
        self.recovery.armed = None;
        self.recovery.kill_ended = false;
        self.engine.frozen = false;
        Ok(())
    }

    /// Logs the recovery handshake state: Control halt count, halted, resume and flag;
    /// HardwareData fault information; up to 16 pending records.
    fn recover_snapshot(&self, phase: &str) {
        let ctl = |offset| self.control_r32(offset).unwrap_or(0);
        let fault_info = |offset| {
            self.init
                .hwdata
                .as_ref()
                .and_then(|hw| hw.r32(self.cfg.layout.hwdata_view + offset).ok())
                .unwrap_or(0)
        };
        dev_info!(
            self.dev(),
            "recovery {}: halt_count={:#x} halted={} resume={} flag={:#x} faultinfo status={} do_not_halt={} did_not_halt={} total={}\n",
            phase,
            ctl(control::HALT_COUNT),
            ctl(control::HALTED),
            ctl(control::RESUME),
            ctl(control::FLAG),
            fault_info(0),
            fault_info(4),
            fault_info(8),
            fault_info(0x18)
        );
        let mut shown = 0;
        for i in 0..128 {
            if shown == 16 {
                break;
            }
            let info = ctl(control::PENDING + 8 * i);
            if info & 1 == 0 {
                continue;
            }
            shown += 1;
            dev_info!(
                self.dev(),
                "  pending[{}]: info {:#x} slot {} wait {:#x}\n",
                i,
                info,
                (info >> 4) & 0x7f,
                ctl(control::PENDING + 8 * i + 4)
            );
        }
    }

    /// Acknowledges a firmware halt: halted = 0, then resume = 1, ordered; nothing else. Then
    /// waits up to 500 polls for the firmware to clear resume. Returns whether it did, and
    /// the polls waited.
    fn acknowledge_halt(&mut self) -> Result<(bool, u32)> {
        self.control_w32(control::HALTED, 0)?;
        barrier();
        self.control_w32(control::RESUME, 1)?;
        barrier();
        let mut t = 0;
        while t < logic::RESUME_POLLS && self.control_r32(control::RESUME)? != 0 {
            poll_sleep();
            t += 1;
        }
        Ok((self.control_r32(control::RESUME)? == 0, t))
    }

    /// The firmware recovery handshake, with a published prefix active (or a firmware error
    /// with nothing published) and a firmware error event seen (or a receipt fault). Returns
    /// the error for the failed prefixes and whether the firmware halted, consumed the host's
    /// resume and drained; otherwise the caller fail-stops (no halt, transport failure, or a
    /// queue the firmware did not drain).
    fn recover(&mut self) -> (Error, bool) {
        // Growth requests of the failed epoch are never answered.
        self.events.grow.clear();
        barrier();
        let fault = self.engine_state().map_or(0, |state| state.fault);
        let last = self.events.last_error;
        let error = match logic::failure(fault, last.map(|e| e.kind)) {
            Failure::Fault => EIO,
            Failure::Timeout => ETIMEDOUT,
        };
        let (kind, slot, raw) = last.map_or((0, -1, [0; 14]), |e| (e.kind, e.slot, e.raw));
        let what = if error == EIO { "fault" } else { "timeout" };
        dev_warn!(
            self.dev(),
            "GPU {}: event type {} slot {} raw {:08x?}; recovering\n",
            what,
            kind,
            slot,
            raw
        );
        self.recover_snapshot("error");
        let recovered = match self.handshake() {
            Ok(recovered) => recovered,
            Err(e) => {
                dev_err!(
                    self.dev(),
                    "recovery: firmware state unavailable: {}\n",
                    e.to_errno()
                );
                false
            }
        };
        (error, recovered)
    }

    /// The halt, resume and drain steps of [`Self::recover`].
    fn handshake(&mut self) -> Result<bool> {
        // The event precedes the halt publication: wait for the real halt.
        let mut t = 0;
        while t < logic::HALT_POLLS && self.control_r32(control::HALTED)? != 1 {
            let _ = self.drain_rings();
            poll_sleep();
            t += 1;
        }
        if self.control_r32(control::HALTED)? != 1 || self.transport_error().is_some() {
            dev_err!(self.dev(), "recovery: firmware did not halt ({} ms)\n", t);
            return Ok(false);
        }
        self.recover_snapshot("halted");
        let (resumed, t) = self.acknowledge_halt()?;
        if !resumed || self.transport_error().is_some() {
            dev_err!(self.dev(), "recovery: resume not consumed ({} ms)\n", t);
            return Ok(false);
        }
        // The firmware clears resume before finishing: wait for the queues to drain. Work of
        // the failed prefixes that runs after the resume can fail in turn (a compute that
        // waited on the aborted FS hitting a firmware compute timeout): its own halt is
        // answered the same way, a bounded number of times, and the drain continues.
        let mut halts = 0;
        let mut t = 0;
        while t < logic::DRAIN_POLLS && !self.drained() {
            let _ = self.drain_rings();
            if self.control_r32(control::HALTED)? == 1
                && self.transport_error().is_none()
                && halts < logic::FURTHER_HALTS
            {
                halts += 1;
                self.recover_snapshot("halted again");
                if !self.acknowledge_halt()?.0 {
                    dev_err!(
                        self.dev(),
                        "recovery: resume after a further halt not consumed\n"
                    );
                    return Ok(false);
                }
                // Restart the drain wait, counting this poll.
                t = 1;
                continue;
            }
            poll_sleep();
            t += 1;
        }
        self.recover_snapshot("resumed");
        self.report("recovery");
        if !self.drained()
            || self.control_r32(control::HALTED)? != 0
            || self.transport_error().is_some()
        {
            dev_err!(
                self.dev(),
                "recovery: failed prefix not drained ({} ms)\n",
                t
            );
            return Ok(false);
        }
        Ok(true)
    }
}
