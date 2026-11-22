// SPDX-License-Identifier: GPL-2.0-only OR MIT
//! GPU fault and hang recovery on the retained firmware session.
//!
//! The 26A428 GFX firmware resets the GPU itself (recovery routine at b000
//! 0x6ef0) and hands the host two edges of a status-B handshake: it writes
//! state 1 and spins until the host writes 2, resets, then writes 3 and spins
//! until the host writes 0. It begins one on its own after a fault or its
//! watchdog, posting a type-4 event first, or when the host asks with EP 0x21
//! type 6 (`AGXArmFirmware::stopFirmwareForRecoveryInspection`), posting no
//! event. This is the halted/resume protocol upstream Asahi drives on
//! G13/G14, in its newer shape. Servicing it makes a fault or hang end one job
//! instead of the boot.
use super::*;
use crate::g18::initdata::{G18RecoveryCause, G18RecoveryWord};

/// Recoveries serviced this boot. There is one firmware session per boot.
static RECOVERIES: AtomicU32 = AtomicU32::new(0);

/// EP 0x21 type 6: b000 0x255b0 requests a host recovery when the low 44
/// bits equal the status-B recovery epoch.
const HOST_RECOVERY_TAG: u64 = 0x0086_0000_0000_0000;
const HOST_RECOVERY_EPOCH_MASK: u64 = (1 << 44) - 1;
/// The firmware's per-QID queue records, which its restart walk (b000
/// 0x10760-0x10800) reads: +0x18 an activity gate it refreshes from the
/// scheduler, +0x40 its stamp position, +0x48 the count of stamps to flush.
/// They live in the GFX role's __DATA reservation, whose first byte is VA
/// 0xfffffc0000050000; the host reads them only while the firmware is halted.
const FW_DATA_VA: u64 = 0xffff_fc00_0005_0000;
const FW_QUEUE_RECORDS_VA: u64 = 0xffff_fc00_000b_b530;
const FW_QUEUE_RECORD_STRIDE: usize = 0x58;
/// Queues the application paths submit to: both graphics stages, compute.
const APPLICATION_QIDS: [usize; 3] = [9, 10, 11];

/// The firmware halts right after it posts its event or takes the request.
const RECOVERY_HALT_POLLS: u32 = 2000; // x 1 ms
/// Between the host's 2 and the firmware's 3 the GPU is reset.
const RECOVERY_RESET_POLLS: u32 = 5000; // x 1 ms
/// A job that has neither retired nor been recovered by the firmware after
/// this long is hung; the host then requests the recovery itself.
const JOB_WATCHDOG_MS: u32 = 10_000;
/// After a recovery, how long the ended job's queues may take to show that
/// the firmware consumed it.
const RECOVERED_RETIRE_MS: u32 = 2000;

/// Poll interval for a wait that has lasted `us`: fine while a job is
/// likely to be about to retire, coarse once it has clearly not.
pub(super) const fn poll_interval_us(us: i64) -> i64 {
    if us < 2_000 { 20 } else if us < 50_000 { 200 } else { 1_000 }
}

/// Upper bounds, in microseconds, of the latency buckets; the last bucket is
/// open-ended.
const LATENCY_BOUNDS_US: [i64; 6] = [100, 300, 1_000, 3_000, 10_000, 100_000];
const LATENCY_BUCKETS: usize = LATENCY_BOUNDS_US.len() + 1;

/// A cumulative latency histogram for this boot.
pub(super) struct Latency {
    buckets: [AtomicU32; LATENCY_BUCKETS],
    max_us: AtomicU32,
}

impl Latency {
    const fn new() -> Self {
        #[allow(clippy::declare_interior_mutable_const)]
        const EMPTY: AtomicU32 = AtomicU32::new(0);
        Self { buckets: [EMPTY; LATENCY_BUCKETS], max_us: EMPTY }
    }

    /// Count one sample; returns the samples counted so far.
    pub(super) fn record(&self, us: i64) -> u32 {
        let index = LATENCY_BOUNDS_US.iter().position(|&bound| us <= bound).unwrap_or(LATENCY_BOUNDS_US.len());
        self.buckets[index].fetch_add(1, Ordering::Relaxed);
        self.max_us.fetch_max(us.clamp(0, u32::MAX as i64) as u32, Ordering::Relaxed);
        self.buckets.iter().map(|bucket| bucket.load(Ordering::Relaxed)).sum()
    }

    fn snapshot(&self) -> ([u32; LATENCY_BUCKETS], u32) {
        let mut buckets = [0; LATENCY_BUCKETS];
        for (count, bucket) in buckets.iter_mut().zip(&self.buckets) {
            *count = bucket.load(Ordering::Relaxed);
        }
        (buckets, self.max_us.load(Ordering::Relaxed))
    }
}

/// Kick to retirement of jobs that retired normally, as the session worker
/// sees it.
static JOB_LATENCY: Latency = Latency::new();
/// A submitting thread's whole wait on the session worker: queueing,
/// preparation, kick, completion and retirement.
pub(super) static SUBMIT_LATENCY: Latency = Latency::new();

/// Microseconds after `since` at which each condition was first seen true
/// (0: not yet). Diagnostic: shows which firmware-owned condition a job's
/// retirement waits on.
pub(super) fn note_phases(first: &mut [i64], since: &Instant<Monotonic>, conditions: &[bool]) {
    let us = since.elapsed().as_micros_ceil().max(1);
    for (seen, &holds) in first.iter_mut().zip(conditions) {
        if holds && *seen == 0 {
            *seen = us;
        }
    }
}

/// Whether the `n`th job of a path is sampled for phase timing.
pub(super) fn phases_due(n: u32) -> bool {
    n <= 3 || n % 1024 == 0
}

/// Whether the `jobs`th retired job is one whose latency summary is logged.
fn latency_report_due(jobs: u32) -> bool {
    (jobs >= 64 && jobs.is_power_of_two()) || jobs % 4096 == 0
}

/// One serviced firmware recovery.
#[derive(Debug, Copy, Clone)]
pub(super) struct G18Recovery {
    pub(super) epoch: u64,
    pub(super) host: bool,
    pub(super) cause: G18RecoveryCause,
    pub(super) us: u64,
}

/// How one job's bounded wait ended.
pub(super) enum JobEnd {
    /// The job retired normally.
    Retired,
    /// A recovery ended the job; its queues then showed it consumed.
    Recovered(G18Recovery),
    /// A recovery ended the job; its queues never showed it consumed, so the
    /// firmware may still reference its memory.
    Stranded(G18Recovery),
}

/// The two application job paths; each can be quarantined alone.
#[derive(Debug, Copy, Clone)]
pub(super) enum JobPath {
    Render,
    Compute,
}

impl JobPath {
    pub(super) const fn bit(self) -> u32 {
        match self {
            JobPath::Render => 1,
            JobPath::Compute => 2,
        }
    }
}

/// A job failure that leaves the session running.
pub(super) enum JobFailure {
    /// Ended by a recovery; the queues were retired and stay usable.
    Recovered(G18Recovery),
    /// Ended by a recovery; its owners stay retained and its path refuses
    /// further jobs.
    Stranded(G18Recovery, JobPath),
    /// Ended by a recovery; its queues retired and its memory was released,
    /// but the path's firmware bindings did not survive the recovery, so the
    /// path refuses further jobs. The client is not poisoned.
    Unbound(G18Recovery, JobPath, &'static str),
    /// [`Self::Recovered`] as reported back to the submitting thread, which
    /// needs only whether the job's memory was released.
    Released,
    /// [`Self::Stranded`] as reported back to the submitting thread.
    Retained,
}

/// How far a failed task reaches.
pub(super) enum Failure {
    /// The session can no longer be trusted and is stopped.
    Fatal(Error),
    /// Only this job failed; the session keeps serving.
    Job(JobFailure),
}

impl From<Error> for Failure {
    fn from(error: Error) -> Self {
        Failure::Fatal(error)
    }
}

impl From<kernel::alloc::AllocError> for Failure {
    fn from(error: kernel::alloc::AllocError) -> Self {
        Failure::Fatal(error.into())
    }
}

pub(super) fn recoveries() -> u32 {
    RECOVERIES.load(Ordering::Relaxed)
}

impl G18Session {
    /// Drain the report rings and service any recovery the firmware has
    /// begun. The state word, not the event, is authoritative: a host
    /// recovery posts no event.
    pub(super) fn poll_faults(&mut self, label: &str) -> Result<Option<G18Recovery>> {
        let event = self.drain()?;
        if self.any_crashed() {
            dev_err!(self.device(), "G18 {}: firmware crashed\n", label);
            return Err(EIO);
        }
        let halted = match self.memory.as_mut() {
            Some(memory) => memory.recovery_word()?.state != 0,
            None => false,
        };
        if !event && !halted {
            return Ok(None);
        }
        if event {
            dev_err!(self.device(), "G18 {}: firmware posted a recovery event\n", label);
        }
        self.recover(label, false).map(Some)
    }

    /// Poll before a kick. A recovery with no job in flight blames nothing
    /// this job owns, so the job proceeds.
    pub(super) fn poll_idle_faults(&mut self, label: &str) -> Result {
        if let Some(recovery) = self.poll_faults(label)? {
            dev_warn!(self.device(), "G18_RECOVERY_IDLE {} epoch={} host={}: no job was in flight\n",
                label, recovery.epoch, recovery.host);
        }
        Ok(())
    }

    /// Complete one recovery handshake. With `request`, first ask the idle
    /// firmware for one. Resumes from whichever edge the firmware is at, so
    /// an interrupted handshake can be finished.
    pub(super) fn recover(&mut self, label: &str, request: bool) -> Result<G18Recovery> {
        let started = Instant::<Monotonic>::now();
        let word = self.recovery_word()?;
        if request && word.state == 0 {
            let message = HOST_RECOVERY_TAG | (word.epoch & HOST_RECOVERY_EPOCH_MASK);
            dev_err!(self.device(), "G18_RECOVERY_REQUEST {} epoch={} message={:#018x}\n",
                label, word.epoch, message);
            Pin::new(&mut self.roles[G18Role::Gfx.index()].rtkit)
                .send_message(EP_GFX_INTERRUPTS, message)?;
        }
        let mut word = word;
        if word.state == 0 {
            word = self.await_recovery_state(label, "halt", 0, 1, RECOVERY_HALT_POLLS)?;
        }
        let mut cause = G18RecoveryCause::default();
        if word.state == 1 {
            cause = self.memory.as_mut().ok_or(EIO)?.recovery_cause()?;
            dev_err!(self.device(),
                "G18_RECOVERY_HALT {} epoch={} host={} source-count={} command={:#x} reason={:#x} detail={:#x} slot-mask={:#x} request={:x?}\n",
                label, word.epoch, word.host, cause.source_count, cause.command, cause.reason,
                cause.detail, cause.slot_mask, cause.request);
            let memory = self.memory.as_mut().ok_or(EIO)?;
            let progress = [memory.queue_progress(9)?, memory.queue_progress(10)?, memory.queue_progress(11)?];
            dev_err!(self.device(), "G18_RECOVERY_PROGRESS {} qid9={:x?} qid10={:x?} qid11={:x?}\n",
                label, progress[0], progress[1], progress[2]);
            self.log_firmware_queue_records(label, "halted");
            self.log_parameter_buffer(label, "halted");
            self.publish_queue_progress(label)?;
            self.memory.as_mut().ok_or(EIO)?.acknowledge_recovery(1, 2)?;
            word.state = 2;
        }
        if word.state == 2 {
            word = self.await_recovery_state(label, "reset", 2, 3, RECOVERY_RESET_POLLS)?;
        }
        if word.state != 3 {
            dev_err!(self.device(), "G18_RECOVERY_FAIL {} phase=clear state={} epoch={}\n",
                label, word.state, word.epoch);
            return Err(EIO);
        }
        let info = self.memory.as_mut().ok_or(EIO)?.recovery_cause()?;
        self.log_firmware_queue_records(label, "restarted");
        self.log_parameter_buffer(label, "restarted");
        self.memory.as_mut().ok_or(EIO)?.acknowledge_recovery(3, 0)?;
        self.drain()?;
        // The restart walk retired every stamp up to the last-submitted table
        // published before 1->2, so each queue's next kick names its last
        // kicked stamp as its parent, as native encodeDependencyList does
        // (queue +0xa0 survives restartWorkQueue unchanged).
        let serviced = RECOVERIES.fetch_add(1, Ordering::Relaxed) + 1;
        let recovery = G18Recovery {
            epoch: word.epoch,
            host: word.host != 0,
            cause: G18RecoveryCause { valid_entries: info.valid_entries, first_entry: info.first_entry,
                pending: info.pending, ..cause },
            us: started.elapsed().as_micros_ceil() as u64,
        };
        dev_err!(self.device(),
            "G18_RECOVERY_DONE {} epoch={} host={} us={} recoveries={} info-valid={} info-first={:x?}\n",
            label, recovery.epoch, recovery.host, recovery.us, serviced,
            info.valid_entries, info.first_entry);
        let logged = (info.valid_entries as usize).min(info.pending.len());
        for &(index, raw) in &info.pending[..logged] {
            let entry = raw as u32;
            dev_err!(self.device(), "G18_RECOVERY_PENDING {} index={} slot={} flags={:#x} value={:#x} raw={:#018x}\n",
                label, index, (entry >> 4) & 0x7f, entry & 0xf, (raw >> 32) as u32, raw);
        }
        Ok(recovery)
    }

    /// Diagnostic: the render PB's firmware-visible words and scene record.
    fn log_parameter_buffer(&mut self, label: &str, phase: &str) {
        let Some(job) = self.application_job.as_mut() else { return };
        match job.job.parameter_buffer().recovery_words() {
            Ok((state, scene_words, counter, scene)) => dev_err!(self.device(),
                "G18_RECOVERY_PB {} {} counter={} scene={} state={:x?} scene_record={:x?}\n",
                label, phase, counter, scene, state, scene_words),
            Err(error) => dev_err!(self.device(), "G18_RECOVERY_PB {} {} unreadable={:?}\n", label, phase, error),
        }
    }

    /// What native `restartWorkQueue` does before resuming a halted
    /// firmware: publish each queue's last submitted kick stamp. The restart
    /// walk flushes every stamp from the queue's own position up to this one
    /// (count = stamp - position + 1), which is what retires a stamp the
    /// recovery killed; without it the queue's next kick waits on that stamp
    /// forever. Queues that never kicked keep their zero entry.
    fn publish_queue_progress(&mut self, label: &str) -> Result {
        let render = self.application_job.as_ref().map_or(0, |job| job.submitted_generation());
        let memory = self.memory.as_mut().ok_or(EIO)?;
        let compute = memory.application_compute_generation();
        for (qid, stamp) in [(9, render), (10, render), (11, compute)] {
            if stamp != 0 {
                memory.publish_queue_progress(qid, 1, u64::from(stamp))?;
            }
        }
        let progress = [memory.queue_progress(9)?, memory.queue_progress(10)?, memory.queue_progress(11)?];
        dev_err!(self.device(), "G18_RECOVERY_PUBLISH {} qid9={:x?} qid10={:x?} qid11={:x?}\n",
            label, progress[0], progress[1], progress[2]);
        Ok(())
    }

    /// Log the firmware's own records for the application queues. Read-only
    /// and diagnostic: a failure to map is logged, never fatal.
    fn log_firmware_queue_records(&self, label: &str, phase: &str) {
        let (base, size) = self.states[G18Role::Gfx.index()].data;
        let map = match usize::try_from(size).map_err(|_| EINVAL).and_then(|size| PhysMap::new(base, size)) {
            Ok(map) => map,
            Err(error) => {
                dev_warn!(self.device(), "G18_RECOVERY_FWQ {} {}: __DATA map failed ({:?})\n", label, phase, error);
                return;
            }
        };
        for qid in APPLICATION_QIDS {
            let offset = (FW_QUEUE_RECORDS_VA - FW_DATA_VA) as usize + qid * FW_QUEUE_RECORD_STRIDE;
            let mut raw = [0u8; FW_QUEUE_RECORD_STRIDE];
            if let Err(error) = map.copy(offset, &mut raw) {
                dev_warn!(self.device(), "G18_RECOVERY_FWQ {} {} qid={}: read failed ({:?})\n", label, phase, qid, error);
                continue;
            }
            let qword = |at: usize| u64::from_le_bytes(raw[at..at + 8].try_into().unwrap_or([0; 8]));
            let words: [u64; FW_QUEUE_RECORD_STRIDE / 8] = core::array::from_fn(|index| qword(index * 8));
            dev_err!(self.device(),
                "G18_RECOVERY_FWQ {} {} qid={} gate={:#x} position={:#x} count={:#x} raw={:x?}\n",
                label, phase, qid, words[3] as u32, words[8], words[9], words);
        }
    }

    fn recovery_word(&mut self) -> Result<G18RecoveryWord> {
        self.memory.as_mut().ok_or(EIO)?.recovery_word()
    }

    fn await_recovery_state(&mut self, label: &str, phase: &str, from: u32, to: u32, polls: u32)
        -> Result<G18RecoveryWord> {
        let mut word = self.recovery_word()?;
        for _ in 0..polls {
            if word.state == to || word.state != from || self.any_crashed() {
                break;
            }
            // Keep the report rings moving while the firmware works.
            self.drain()?;
            fsleep(Delta::from_millis(1));
            word = self.recovery_word()?;
        }
        if word.state == to {
            return Ok(word);
        }
        dev_err!(self.device(), "G18_RECOVERY_FAIL {} phase={} state={} epoch={} host={} crashed={}\n",
            label, phase, word.state, word.epoch, word.host, self.any_crashed());
        Err(ETIMEDOUT)
    }

    /// Wait for one kicked job. `retired(session, ms, recovered, report)`
    /// reports whether the job's queues have reached their retired state;
    /// once a recovery has ended the job it checks only what the firmware
    /// owns, since no stage will complete. `report` is set once per second
    /// of waiting (every 250 ms after a recovery) for progress logging. The
    /// host watchdog requests a recovery for a job the firmware never flags,
    /// and so does a refused parameter buffer growth: the tiler that asked
    /// stays stalled, and only a recovery ends that render.
    /// Timing is wall clock; most jobs retire within a few hundred
    /// microseconds, so polling starts fine and backs off.
    pub(super) fn wait_job<F>(&mut self, label: &str, mut retired: F) -> Result<JobEnd>
    where
        F: FnMut(&mut Self, u32, bool, bool) -> Result<bool>,
    {
        let mut recovery: Option<G18Recovery> = None;
        let mut since = Instant::<Monotonic>::now();
        let mut reported = 0u32;
        self.growth_refused = false;
        loop {
            if let Some(serviced) = self.poll_faults(label)? {
                if recovery.is_none() {
                    since = Instant::<Monotonic>::now();
                    reported = 0;
                    recovery = Some(serviced);
                }
            }
            let us = since.elapsed().as_micros_ceil();
            let ms = (us / 1000) as u32;
            let period = if recovery.is_some() { 250 } else { 1000 };
            let report = ms / period > reported;
            if report {
                reported = ms / period;
            }
            if retired(self, ms, recovery.is_some(), report)? {
                if recovery.is_none() && latency_report_due(JOB_LATENCY.record(us)) {
                    let (job, job_max) = JOB_LATENCY.snapshot();
                    let (submit, submit_max) = SUBMIT_LATENCY.snapshot();
                    dev_info!(self.device(), "G18_JOB_LATENCY bounds_us={:?} job={:?} job_max_us={} submit={:?} submit_max_us={}\n",
                        LATENCY_BOUNDS_US, job, job_max, submit, submit_max);
                }
                return Ok(recovery.map_or(JobEnd::Retired, JobEnd::Recovered));
            }
            if let Some(ended) = recovery {
                if ms >= RECOVERED_RETIRE_MS {
                    return Ok(JobEnd::Stranded(ended));
                }
            } else if ms >= JOB_WATCHDOG_MS {
                dev_err!(self.device(), "G18_JOB_WATCHDOG {} ms={}: requesting recovery\n", label, ms);
                recovery = Some(self.recover(label, true)?);
                since = Instant::<Monotonic>::now();
                reported = 0;
            } else if core::mem::take(&mut self.growth_refused) {
                dev_err!(self.device(), "G18_GROW_REFUSED {} ms={}: requesting recovery\n", label, ms);
                recovery = Some(self.recover(label, true)?);
                since = Instant::<Monotonic>::now();
                reported = 0;
            }
            fsleep(Delta::from_micros(poll_interval_us(us)));
        }
    }
}
