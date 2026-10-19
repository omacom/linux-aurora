// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! The poll work: drain, retire, growth, hang checks, publication; and the repoll timers.
//!
//! Order of one pass, under the device lock: drain the receive rings; retire flights (stamps,
//! queue DONE/READ, channel consumption, stage-event counts, idle bits, fault banks; renew the
//! progress deadline), with up to four trailing 5 us polls after an EP 0x20 event; answer PB
//! growth ([`Firmware::answer_growth`]); check hangs and recover ([`Firmware::check_hangs`],
//! which calls [`Firmware::progress`]); publish ([`Firmware::publish`]). Then drop the lock
//! and complete the retired jobs ([`super::job::complete_all`]). Wakeups: the EP 0x20 event
//! kick, then an hrtimer re-poll every `g15_resnap_us` (100) while anything is published, or
//! `g15_repoll_us` (15) while a ready prefix waits for record consumption, with a 1-jiffy
//! delayed-work backup.

use core::sync::atomic::Ordering;

use kernel::{
    bindings, c_str, debugfs, fmt, impl_has_hr_timer,
    prelude::*,
    sync::{Arc, ArcBorrow},
    time::{
        delay,
        hrtimer::{
            HrTimerCallback, HrTimerCallbackContext, HrTimerPointer, HrTimerRestart, RelativeMode,
        },
        Delta, Jiffies, Monotonic,
    },
    workqueue::RawWorkItem,
};

use super::channel::Channel;
use super::engine::{complete, now_ns, rmb, Flight, Lap};
use super::fw::graph::qstate;
use super::hw::sizes;
use super::job::{self, Completions, Job};
use super::regs::EngineState;
use super::{params, Firmware, Shared, POLL_WORK};
use crate::module_parameters;

impl_has_hr_timer! {
    impl HasHrTimer<Self> for Shared {
        mode: RelativeMode<Monotonic>, field: self.resnap
    }
}

impl HrTimerCallback for Shared {
    type Pointer<'a> = Arc<Self>;

    /// Runs the poll now.
    fn run(this: ArcBorrow<'_, Self>, _ctx: HrTimerCallbackContext<'_, Self>) -> HrTimerRestart {
        let this: Arc<Self> = this.into();
        this.poll_now();
        HrTimerRestart::NoRestart
    }
}

/// The `asahi-g15/stats` debugfs file contents.
fn stats_show(shared: &Arc<Shared>, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    let state = shared.state.lock();
    match state.as_deref() {
        Some(firmware) => firmware.write_stats(f),
        None => Ok(()),
    }
}

impl Shared {
    /// One poll pass (the poll work item).
    pub(crate) fn run_poll(self: Arc<Self>) {
        let mut done = Completions::new();
        {
            let mut state = self.state.lock();
            if let Some(firmware) = state.as_deref_mut() {
                if firmware.engine.shared.is_none() {
                    firmware.engine.shared = Some(self.clone());
                }
                let evented = self.evented.swap(false, Ordering::AcqRel);
                match firmware.poll(evented, &mut done) {
                    Ok(true) => self.rearm(firmware.engine.chan_wait),
                    Ok(false) => {}
                    Err(error) => self.fail_stop(error),
                }
            }
        }
        job::complete_all(done);
    }

    /// Hands a job to the engine and publishes what can be published. Err: the job was not
    /// accepted (the engine failed: EIO) and the caller fails it; once accepted, its fence is
    /// signalled by the engine, also on failure.
    pub(crate) fn submit(self: &Arc<Self>, job: Arc<Job>) -> Result {
        let mut done = Completions::new();
        let accepted = {
            let mut state = self.state.lock();
            let firmware = state.as_deref_mut().ok_or(ENODEV)?;
            if firmware.engine.shared.is_none() {
                firmware.engine.shared = Some(self.clone());
            }
            let accepted = firmware.enqueue(job);
            if accepted.is_ok() {
                if let Err(error) = firmware.publish(&mut done) {
                    self.fail_stop(error);
                }
                if !firmware.engine.flights.is_empty() {
                    self.rearm(firmware.engine.chan_wait);
                }
            }
            accepted
        };
        job::complete_all(done);
        accepted
    }

    /// Queues the poll work now, or brings a pending one forward. Callable from any context.
    pub(crate) fn poll_now(self: &Arc<Self>) {
        self.mod_poll(0);
    }

    /// Queues the poll on `system_highpri_wq` after `delay` jiffies, or moves an already
    /// pending one to that expiry.
    fn mod_poll(self: &Arc<Self>, delay: Jiffies) {
        // SAFETY: The closure returns false only when `mod_delayed_work_on` found the work
        // already pending (it then only modified its timer), so the queue keeps the reference
        // it took when it was queued; otherwise the work is queued and owns this reference.
        let _ = unsafe {
            <Arc<Shared> as RawWorkItem<POLL_WORK>>::__enqueue(self.clone(), |work| {
                let dwork = kernel::container_of!(work, bindings::delayed_work, work);
                !bindings::mod_delayed_work_on(
                    bindings::wq_misc_consts_WORK_CPU_UNBOUND as kernel::ffi::c_int,
                    bindings::system_highpri_wq,
                    dwork,
                    delay,
                )
            })
        };
    }

    /// Re-checks a published command soon: hrtimer, with a one-jiffy backup; sooner while a
    /// ready prefix waits for a channel record's consumption.
    pub(crate) fn rearm(self: &Arc<Self>, chan_wait: bool) {
        let mut us = params::resnap_us();
        if chan_wait && *module_parameters::g15_repoll_us.value() != 0 {
            us = params::repoll_us();
        }
        {
            let mut handle = self.resnap_handle.lock();
            // Dropping the previous handle cancels a still pending expiry before the timer
            // is started again.
            drop(handle.take());
            *handle = Some(self.clone().start(Delta::from_micros(us as i64)));
        }
        self.mod_poll(1);
    }

    /// Starts what the engine needs besides the firmware state: the `asahi-g15/stats` file.
    pub(crate) fn start_engine(self: &Arc<Self>) -> Result {
        let dir = debugfs::Dir::new(c_str!("asahi-g15"));
        let file = KBox::pin_init(
            dir.read_callback_file(c_str!("stats"), self.clone(), &stats_show),
            GFP_KERNEL,
        )?;
        let mut state = self.state.lock();
        let firmware = state.as_deref_mut().ok_or(ENODEV)?;
        firmware.engine.shared = Some(self.clone());
        firmware.engine.debugfs = Some(file);
        Ok(())
    }
}

/// Stamp cell `obs` reached its own target, bounded by the newest published target `tail`.
pub(crate) fn stamp_reached(obs: u32, own: u32, tail: u32) -> bool {
    obs != 0 && obs & 0xff == 0 && obs.wrapping_sub(own) <= tail.wrapping_sub(own)
}

/// Cursor `c` of a ring of `m` entries passed end `h` of entries from `b`, within tail `t`.
pub(crate) fn ring_passed(c: u32, b: u32, h: u32, t: u32, m: u32) -> bool {
    let d = |x: u32| x.wrapping_add(m).wrapping_sub(b) % m;
    let (dc, dh, dt) = (d(c), d(h), d(t));
    dh <= dc && dc <= dt
}

impl Firmware {
    /// One poll pass under the device lock. Returns whether to re-arm the poll timers.
    pub(crate) fn poll(&mut self, evented: bool, done: &mut Completions) -> Result<bool> {
        if self.recovery.failed().is_some() {
            return Ok(false);
        }
        // With nothing published only an armed hang request whose prefix ended or a firmware
        // error event needs this pass: one that arrived after its prefix's receipts is answered
        // here (the firmware halted and waits for the resume) instead of being taken for a
        // failure of the next prefix. Drain the receive rings before deciding, published or
        // not, so the error count is current and idle log/trace/stats traffic is consumed. With
        // prefixes published the retirement loop below drains first.
        if self.engine.flights.is_empty() {
            self.drain_rings()?;
            if self.recovery_idle() {
                return Ok(false);
            }
        }
        let mut lap = Lap::start();
        let start = lap.on().then(now_ns);
        self.retire(evented, done)?;
        lap.lap(&mut self.engine.prof.retire_ns);
        self.answer_growth()?;
        self.check_hangs(done)?;
        let published = self.publish(done);
        let s = &mut self.engine.stats;
        if self.engine.flights.is_empty() && s.idle_from_ns != 0 {
            // Idle with nothing ready is not a gap: the clients are late.
            s.starve_from_ns = s.idle_from_ns;
            s.idle_from_ns = 0;
        }
        if let Some(start) = start {
            self.engine.prof.polls += 1;
            self.engine.prof.poll_ns += now_ns().wrapping_sub(start);
        }
        published?;
        Ok(!self.engine.flights.is_empty() || self.recovery.needs_repoll() || self.engine.frozen)
    }

    /// Drains the rings and retires finished flights, with up to four trailing 5 us polls
    /// after an event. A receipt fault is left in `Recovery::receipt_fault` for the hang
    /// checks.
    pub(crate) fn retire(&mut self, evented: bool, done: &mut Completions) -> Result {
        // After an event, a few short trailing polls catch the final receipts.
        let mut tail = if evented { 4 } else { 0 };
        let mut ret;

        loop {
            let mut lap = Lap::start();
            self.drain_rings()?;
            lap.lap(&mut self.engine.prof.drain_ns);
            ret = self.retired();
            while let Ok(true) = ret {
                // An armed kill that ended it, or a doomed flight.
                let outcome = self.retire_outcome();
                self.retire_oldest(outcome, done);
                ret = self.retired();
            }
            if ret.is_err() || tail == 0 || self.engine.flights.is_empty() {
                break;
            }
            tail -= 1;
            delay::udelay(Delta::from_micros(5));
        }
        self.recovery.receipt_fault = ret.is_err();
        Ok(())
    }

    fn r32(o: &super::object::FwObject, offset: usize) -> Result<u32> {
        o.r32(offset)
    }

    /// Retirement of the oldest published prefix: Ok(true) retired, Ok(false) not yet,
    /// Err(EIO) fault. The last published one needs every receipt exactly: final stamps,
    /// DONE/READ at the tail heads, consumed pipes, new firing bits and idle engines.
    pub(crate) fn retired(&self) -> Result<bool> {
        let Some(f) = self.engine.oldest() else {
            return Ok(false);
        };
        rmb();
        if self.engine.flights.len() > 1 {
            return self.flight_retired(f);
        }
        let (g, r, e) = (&self.graph.compute, &self.graph.render, &self.engine);
        let ch = &self.channels;
        // A render: both stamp pairs, both queues, both pipes.
        if f.renders != 0 {
            let pending = Self::r32(&r.user_stamp[0], 0)? != e.ta_target
                || Self::r32(&r.fw_stamp[0], 0)? != e.ta_target
                || Self::r32(&r.user_stamp[1], 0)? != e.fs_target
                || Self::r32(&r.fw_stamp[1], 0)? != e.fs_target
                || Self::r32(&r.qstate[0], qstate::DONE)? != e.ta_head
                || Self::r32(&r.qstate[0], qstate::READ)? != e.ta_head
                || Self::r32(&r.qstate[1], qstate::DONE)? != e.fs_head
                || Self::r32(&r.qstate[1], qstate::READ)? != e.fs_head
                || ch.read(Channel::Ta)? != ch.producer(Channel::Ta)
                || ch.read(Channel::Fs)? != ch.producer(Channel::Fs);
            if pending {
                return Ok(false);
            }
        }
        if f.computes != 0 {
            let pending = Self::r32(&g.user_stamp, 0)? != e.target
                || Self::r32(&g.fw_stamp, 0)? != e.target
                || Self::r32(&g.qstate, qstate::DONE)? != e.head
                || Self::r32(&g.qstate, qstate::READ)? != e.head
                || ch.read(Channel::Compute)? != ch.producer(Channel::Compute)
                || self.events.compute_events == e.events;
            if pending {
                return Ok(false);
            }
        }
        // New firing bits: at least one per participating stage (1, 2 or 3).
        let stages = if f.computes != 0 { 3 } else { 2 };
        if f.renders != 0 && self.events.stage_events.wrapping_sub(e.stage_seen) < stages {
            return Ok(false);
        }
        let state = self.session.regs.engine_state()?;
        if state.fault() != 0 {
            self.log_fault(&state);
            return Err(EIO);
        }
        Ok(state.busy == 0)
    }

    /// The oldest published prefix while later ones may run: its own stamp targets reached
    /// within the published interval, DONE past its saved ring ends, its channel records
    /// consumed. Engine idle is not required; faults still are checked.
    fn flight_retired(&self, f: &Flight) -> Result<bool> {
        let (g, r, e) = (&self.graph.compute, &self.graph.render, &self.engine);
        let ch = &self.channels;
        let cap = sizes::QUEUE_CAP;
        let rec_passed = |c: Channel, rec: u32| -> Result<bool> {
            Ok(ring_passed(
                ch.read(c)?,
                (rec + 255) % 256,
                rec,
                ch.producer(c),
                256,
            ))
        };

        if f.computes != 0 {
            let pending = !stamp_reached(Self::r32(&g.user_stamp, 0)?, f.target, e.target)
                || !stamp_reached(Self::r32(&g.fw_stamp, 0)?, f.target, e.target)
                || !ring_passed(
                    Self::r32(&g.qstate, qstate::DONE)?,
                    f.c_start,
                    f.head,
                    e.head,
                    cap,
                )
                || !rec_passed(Channel::Compute, f.rec[2])?;
            if pending {
                return Ok(false);
            }
        }
        if f.renders != 0 {
            let pending = !stamp_reached(Self::r32(&r.user_stamp[0], 0)?, f.ta_target, e.ta_target)
                || !stamp_reached(Self::r32(&r.fw_stamp[0], 0)?, f.ta_target, e.ta_target)
                || !stamp_reached(Self::r32(&r.user_stamp[1], 0)?, f.fs_target, e.fs_target)
                || !stamp_reached(Self::r32(&r.fw_stamp[1], 0)?, f.fs_target, e.fs_target)
                || !ring_passed(
                    Self::r32(&r.qstate[0], qstate::DONE)?,
                    f.ta_start,
                    f.ta_head,
                    e.ta_head,
                    cap,
                )
                || !ring_passed(
                    Self::r32(&r.qstate[1], qstate::DONE)?,
                    f.fs_start,
                    f.fs_head,
                    e.fs_head,
                    cap,
                )
                || !rec_passed(Channel::Ta, f.rec[0])?
                || !rec_passed(Channel::Fs, f.rec[1])?;
            if pending {
                return Ok(false);
            }
        }
        let state = self.session.regs.engine_state()?;
        if state.fault() != 0 {
            self.log_fault(&state);
            return Err(EIO);
        }
        Ok(true)
    }

    /// Logs the non-zero fault banks of `state`.
    pub(crate) fn log_fault(&self, state: &EngineState) {
        for bank in 0..2 {
            let (f, va) = (state.status[bank], state.address[bank]);
            if f == 0 {
                continue;
            }
            let access = if (f >> 4) & 1 != 0 { "read" } else { "write" };
            let reason = if (f >> 1) & 7 != 0 {
                "reason!=unmapped"
            } else {
                "unmapped"
            };
            dev_info!(
                self.graph.dev.as_ref(),
                "G15: fault bank {}: status {:#x} va {:#x} ({} {} level {} unit {:#x} ctx {})\n",
                bank,
                f,
                va,
                access,
                reason,
                (f >> 7) & 3,
                (f >> 9) & 0xff,
                (f >> 17) & 0x3f
            );
        }
    }

    /// The oldest published prefix retired: its storage is released, its job completes when
    /// this was its last published prefix and nothing of it is left to publish. A failed
    /// (hang-killed, doomed, recovered) prefix fails its job, which drops its unpublished
    /// commands. Recovery calls this for every published prefix after a recovered failure.
    pub(crate) fn retire_oldest(&mut self, result: Result, done: &mut Completions) {
        if self.engine.flights.is_empty() {
            return;
        }
        let Ok(mut f) = self.engine.flights.remove(0) else {
            return;
        };
        if result.is_ok() {
            self.stat_retire(&f);
        }
        self.slots.release(&mut f.claims); // after the stats read its timestamp cells
        if f.rebuild {
            f.rebuild = false;
            self.engine.rebuild_live = false;
        }
        if self.engine.flights.is_empty() {
            // Drained: a gap until the next publication.
            self.engine.stats.idle_from_ns = now_ns();
        }
        // The next prefix starts executing about now.
        self.renew_deadlines();
        let job = f.job.clone();
        let p = &job.progress;
        p.next.store(p.next() + f.count as u32, Ordering::Relaxed);
        if let Err(error) = result {
            complete(done, &job, Err(error));
            if p.is_pending() {
                // Drop its unpublished commands.
                p.pending.store(false, Ordering::Relaxed);
                self.engine.pending.retain(|j| !Arc::ptr_eq(j, &job));
                p.count.store(p.sent(), Ordering::Relaxed);
            }
        }
        if p.next() < p.sent() || p.sent() < p.count() {
            return; // more of it published or still to publish
        }
        let s = &mut self.engine.stats;
        if result.is_ok() {
            s.jobs += 1;
            s.job_cmds += p.count() as u64;
            s.job_max = s.job_max.max(p.count() as u64);
        }
        complete(done, &job, Ok(()));
        // The rest of a compute horizon, after the first job (fence order).
        for j2 in f.group.iter() {
            let p2 = &j2.progress;
            p2.next.store(p2.count(), Ordering::Relaxed);
            match result {
                Err(error) => complete(done, j2, Err(error)),
                Ok(()) => {
                    s.jobs += 1;
                    s.job_cmds += p2.count() as u64;
                    s.job_max = s.job_max.max(p2.count() as u64);
                }
            }
            complete(done, j2, Ok(()));
        }
    }

    /// Progress of the oldest prefix: a FW stamp of an engine on which it still has unreached
    /// work moved since the last poll (later prefixes' work on other engines does not count).
    /// Renews the deadline; the first call for a prefix only records its position.
    /// [`Self::check_hangs`] calls it after restoring an armed request, while no request is
    /// armed.
    pub(crate) fn progress(&mut self) {
        let Some(f) = self.engine.oldest() else {
            return;
        };
        let (g, r, e) = (&self.graph.compute, &self.graph.render, &self.engine);
        let (mut c, mut ta, mut fs) = (0, 0, 0);

        if f.computes != 0 {
            c = g.fw_stamp.r32(0).unwrap_or(0);
            if stamp_reached(c, f.target, e.target) {
                c = 0; // its computes are done: later work does not count
            }
        }
        if f.renders != 0 {
            ta = r.fw_stamp[0].r32(0).unwrap_or(0);
            fs = r.fw_stamp[1].r32(0).unwrap_or(0);
            if stamp_reached(ta, f.ta_target, e.ta_target) {
                ta = 0;
            }
            if stamp_reached(fs, f.fs_target, e.fs_target) {
                fs = 0;
            }
        }
        let moved = e.progress_flight == f.published_ns && [c, ta, fs] != e.progress;
        let published = f.published_ns;
        if moved {
            self.renew_deadlines();
        }
        self.engine.progress_flight = published;
        self.engine.progress = [c, ta, fs];
    }

    /// A retired prefix: publication to retirement (including any wait behind its
    /// predecessor), event times, internal stage times of its members.
    fn stat_retire(&mut self, f: &Flight) {
        let now = now_ns();
        let d = now.wrapping_sub(f.published_ns);
        let kind = if f.computes != 0 && f.renders != 0 {
            2
        } else if f.renders != 0 {
            1
        } else {
            0
        };
        let (mut lo, mut hi) = (u64::MAX, 0u64);
        let (mut first, mut last) = (u64::MAX, 0u64);
        let scrub = self.slots.scrub.clone();
        let s = &mut self.engine.stats;

        // Add one stage interval read from internal timestamp cells (start, end).
        let mut stage = |o: &super::object::FwObject, off: usize, sum: &mut u64| {
            let rd = |x: usize| o.r32(x).unwrap_or(0) as u64;
            let start = rd(off + 4) << 32 | rd(off);
            let end = rd(off + 12) << 32 | rd(off + 8);
            if start == 0 || end < start {
                return;
            }
            *sum += end - start;
            lo = lo.min(start);
            hi = hi.max(end);
        };
        for k in 0..f.renders as usize {
            let p = &scrub.passes[f.claims.pass[k] as usize];
            stage(&p.its, 0, &mut s.ta_ticks);
            stage(&p.its, 0x10, &mut s.fs_ticks);
        }
        for k in 0..(f.computes as usize).min(f.claims.slot_count as usize) {
            let c = &scrub.compute[f.claims.slot[k] as usize];
            stage(&c.its, 0, &mut s.cdm_ticks);
        }
        if hi > lo {
            s.span_ticks += hi - lo;
        }
        for b in 0..3 {
            let t = self.events.event_ns[b];
            let skip = if b == 2 {
                f.computes == 0
            } else {
                f.renders == 0
            };
            if skip {
                continue;
            }
            first = first.min(t);
            last = last.max(t);
        }
        if first > f.published_ns && last <= now {
            s.first_ns[kind] += first - f.published_ns;
            s.last_ns[kind] += last - f.published_ns;
        }
        s.count[0] += f.computes as u64;
        s.count[1] += f.renders as u64;
        if kind == 2 {
            s.count[2] += f.count as u64;
        }
        s.prefixes[kind] += 1;
        s.busy_ns[kind] += d;
        s.max_ns[kind] = s.max_ns[kind].max(d);
    }

    /// Engine state report for failures and hang requests (without the initdata Runtime
    /// readiness and Flags words).
    pub(crate) fn report(&self, what: &str) {
        let (g, r, e) = (&self.graph.compute, &self.graph.render, &self.engine);
        let ch = &self.channels;
        let rd = |o: &super::object::FwObject, off| o.r32(off).unwrap_or(0xdeadbeef);
        let state = self.session.regs.engine_state().unwrap_or_default();
        let f = e.oldest();
        let ms = f.map_or(0, |f| now_ns().wrapping_sub(f.published_ns) / 1_000_000);

        dev_info!(
            self.graph.dev.as_ref(),
            "G15: {}: ctx={} prefixes={} n={} target={:#x} user={:#x} fw={:#x} done={} read={} write={} head={} ch={}/{} events={}/{} errors={} busy={} fault={:#x} {}ms\n",
            what,
            f.map_or(0, |f| f.job.vm.slot),
            e.flights.len(),
            e.ordinal,
            e.target,
            rd(&g.user_stamp, 0),
            rd(&g.fw_stamp, 0),
            rd(&g.qstate, qstate::DONE),
            rd(&g.qstate, qstate::READ),
            rd(&g.qstate, qstate::WRITE),
            e.head,
            ch.read(Channel::Compute).unwrap_or(0xdeadbeef),
            ch.write_cell(Channel::Compute).unwrap_or(0xdeadbeef),
            self.events.compute_events,
            e.events,
            self.events.errors,
            state.busy,
            state.fault(),
            ms
        );
        if f.is_some_and(|f| f.renders != 0) {
            dev_info!(
                self.graph.dev.as_ref(),
                "G15:   render: draw={} ta {:#x}/{:#x}/{:#x} done={} read={} write={} head={} ch0={}/{}; fs {:#x}/{:#x}/{:#x} done={} read={} write={} head={} ch1={}/{}; stage events={}/{}\n",
                e.draw,
                e.ta_target,
                rd(&r.user_stamp[0], 0),
                rd(&r.fw_stamp[0], 0),
                rd(&r.qstate[0], qstate::DONE),
                rd(&r.qstate[0], qstate::READ),
                rd(&r.qstate[0], qstate::WRITE),
                e.ta_head,
                ch.read(Channel::Ta).unwrap_or(0xdeadbeef),
                ch.write_cell(Channel::Ta).unwrap_or(0xdeadbeef),
                e.fs_target,
                rd(&r.user_stamp[1], 0),
                rd(&r.fw_stamp[1], 0),
                rd(&r.qstate[1], qstate::DONE),
                rd(&r.qstate[1], qstate::READ),
                rd(&r.qstate[1], qstate::WRITE),
                e.fs_head,
                ch.read(Channel::Fs).unwrap_or(0xdeadbeef),
                ch.write_cell(Channel::Fs).unwrap_or(0xdeadbeef),
                self.events.stage_events,
                e.stage_seen
            );
        }
    }

    /// The `asahi-g15/stats` contents. Scripts parse the keys: keep them stable.
    pub(crate) fn write_stats(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s = &self.engine.stats;
        let slots = &self.slots.stats;
        let (recoveries, hang_kills) = self.recovery_counts();
        let us = |ns: u64| ns / 1000;

        for (e, name) in ["compute", "render", "mixed"].iter().enumerate() {
            writeln!(
                f,
                "{}: count {} prefixes {} busy_us {} max_us {} first_us {} last_us {}",
                name,
                s.count[e],
                s.prefixes[e],
                us(s.busy_ns[e]),
                us(s.max_ns[e]),
                us(s.first_ns[e]),
                us(s.last_ns[e])
            )?;
        }
        writeln!(
            f,
            "gaps {} gap_us {} yields {} overlaps {}",
            s.gaps,
            us(s.gap_ns),
            s.yields,
            s.overlaps
        )?;
        writeln!(
            f,
            "jobs {} job_cmds {} job_max {} early {} recoveries {} hang_kills {} clustered {}",
            s.jobs, s.job_cmds, s.job_max, s.early, recoveries, hang_kills, s.clustered
        )?;
        writeln!(
            f,
            "ct_sync {} slot_sync {} starved_us {} prep_us {} grouped {} pass_sync {}",
            slots.ct_sync,
            slots.slot_sync,
            us(s.starved_ns),
            us(s.prep_ns),
            s.grouped,
            slots.pass_sync
        )?;
        writeln!(f, "type0: fs2c {} c2ta {}", s.fs2c0, s.c2ta0)?;
        write!(f, "horizon:")?;
        for n in s.hz.iter() {
            write!(f, " {}", n)?;
        }
        writeln!(
            f,
            " stop {}/{}/{}/{} behind {} hold {}/{}/{} ahead {}",
            s.hz_stop[0],
            s.hz_stop[1],
            s.hz_stop[2],
            s.hz_stop[3],
            s.hz_behind,
            s.hold[0],
            s.hold[1],
            s.hold[2],
            s.hz_ahead
        )?;
        self.pools.write_stats(f)?;
        writeln!(
            f,
            "stage: ta_us {} fs_us {} cdm_us {} span_us {}",
            s.ta_ticks / 24,
            s.fs_ticks / 24,
            s.cdm_ticks / 24,
            s.span_ticks / 24
        )?;
        // The phase timers of debug_flags bit 49, and the scrub flush waits.
        let p = &self.engine.prof;
        writeln!(
            f,
            "prof: prefixes {} select_us {} encode_us {} claim_us {} write_us {} post_us {} polls {} drain_us {} retire_us {} poll_us {} flushes {} flush_us {}",
            p.prefixes,
            us(p.select_ns),
            us(p.encode_ns),
            us(p.claim_ns),
            us(p.write_ns),
            us(p.post_ns),
            p.polls,
            us(p.drain_ns),
            us(p.retire_ns),
            us(p.poll_ns),
            slots.flushes,
            us(slots.flush_ns)
        )
    }
}
