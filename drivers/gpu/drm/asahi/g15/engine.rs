// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! The prefix/horizon/flight engine.
//!
//! One device-global compute queue and one TA+FS queue pair serve every logical queue and VM.
//! Jobs (one per SUBMIT) wait in `pending`; pick order: priority, then the least recently
//! served queue, then FIFO. A *prefix* is a run of consecutive commands of one job (up to 16
//! computes and [`super::hw::tuning::PREFIX_RENDERS`] renders) published together: images,
//! then thresholds/counters once per prefix, then one queue WRITE per queue, then channel
//! records, then doorbells (compute, FS, TA). A compute-only job that ends its prefix pulls
//! further ready compute-only jobs of the same queue and VM into a *horizon* (up to 16, posted
//! ahead at the observed depth). At most [`super::hw::sizes::FLIGHTS`] prefixes are in flight,
//! the second only of the same VM, posted behind the first's unconsumed records
//! (submit-ahead). Ordering is a total order, which satisfies the header barriers and
//! EXACT_PRIOR ordinals validated at submission. Priming: after a recovered abort and for the
//! first render prefix after boot, an exclusive one-render prefix rebuilds the render
//! allocators and blocks publication behind it until it retires.
//!
//! Barrier types per edge (type 0 = firmware stamp only, 1 = also the predecessor job hold):
//! FS -> own TA 0; TA -> previous FS 0 only inside a prefix after a render with no
//! InitBM/resync, else 1; compute -> TA 1 on class 2; FS -> compute 1 (`CROSS_ENGINE_HOLD`
//! bit 0); compute -> compute 1 on class 2; early tiling (off) TA -> TA and FS -> FS 1.
//!
//! Completions: rather than signalling a fence under the device lock, the engine pushes the job
//! to [`Completions`] and the caller signals after dropping the lock; [`Job::complete`] runs
//! once per job, so a job may be pushed again when its last engine reference is released.

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use kernel::{prelude::*, sync::Arc, uapi};

use super::channel::Channel;
use super::command::{Command, ComputeCommand, RenderCommand};
use super::encode::{self, ClusterLayout, ComputeRefs, Geometry, RenderRefs};
use super::fw::channels::SubmitRecord;
use super::fw::graph::qstate;
use super::fw::work::{self, barrier::RENDER_WAIT_UUID, BarrierType};
use super::hw::{engine as ids, sizes, tuning};
use super::job::{Completions, Job};
use super::slots::Claims;
use super::{params, Firmware, Shared};

/// `wmb()`: `dsb st` (a full fence on other architectures).
#[inline(always)]
pub(crate) fn wmb() {
    #[cfg(target_arch = "aarch64")]
    // SAFETY: A barrier instruction with no other effects.
    unsafe {
        core::arch::asm!("dsb st", options(nostack, preserves_flags))
    };
    #[cfg(not(target_arch = "aarch64"))]
    core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);
}

/// `rmb()`: `dsb ld` (a full fence on other architectures).
#[inline(always)]
pub(crate) fn rmb() {
    #[cfg(target_arch = "aarch64")]
    // SAFETY: A barrier instruction with no other effects.
    unsafe {
        core::arch::asm!("dsb ld", options(nostack, preserves_flags))
    };
    #[cfg(not(target_arch = "aarch64"))]
    core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);
}

/// `ktime_get_ns()`.
pub(crate) fn now_ns() -> u64 {
    <kernel::time::Monotonic as kernel::time::ClockSource>::ktime_get() as u64
}

/// The engine's bookkeeping of one job, embedded in [`Job`]. Only the engine changes it, always
/// under the device lock (the atomics only make the shared `Arc<Job>` mutable).
pub(crate) struct Progress {
    /// Commands still to run: the job's command count, cut to `sent` when a failure drops its
    /// unpublished commands.
    pub(crate) count: AtomicU32,
    /// First command not yet published.
    pub(crate) sent: AtomicU32,
    /// First command not yet retired.
    pub(crate) next: AtomicU32,
    /// Waiting in `Engine::pending`.
    pub(crate) pending: AtomicBool,
    /// The fence was signalled: [`Job::complete`] runs once.
    pub(crate) signaled: AtomicBool,
}

impl Progress {
    pub(crate) fn new(commands: usize) -> Self {
        Self {
            count: AtomicU32::new(commands as u32),
            sent: AtomicU32::new(0),
            next: AtomicU32::new(0),
            pending: AtomicBool::new(false),
            signaled: AtomicBool::new(false),
        }
    }

    pub(crate) fn count(&self) -> u32 {
        self.count.load(Ordering::Relaxed)
    }

    pub(crate) fn sent(&self) -> u32 {
        self.sent.load(Ordering::Relaxed)
    }

    pub(crate) fn next(&self) -> u32 {
        self.next.load(Ordering::Relaxed)
    }

    pub(crate) fn is_pending(&self) -> bool {
        self.pending.load(Ordering::Relaxed)
    }

    fn set_sent(&self, v: u32) {
        self.sent.store(v, Ordering::Relaxed)
    }

    fn set_pending(&self, v: bool) {
        self.pending.store(v, Ordering::Relaxed)
    }
}

/// Engine of the previous member.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub(crate) enum Prev {
    #[default]
    None,
    Compute,
    Render,
}

/// A published prefix: its job and commands, its own receipts (final
/// targets, exclusive ring ends and the ring positions before it, channel producers after its
/// records) and the storage it holds until its own retirement.
pub(crate) struct Flight {
    /// The job whose commands the prefix holds, and horizon jobs folded into it (in order).
    pub(crate) job: Arc<Job>,
    pub(crate) group: KVec<Arc<Job>>,
    /// Index of the first command of the job in this prefix, and the number of commands.
    pub(crate) first: usize,
    pub(crate) count: usize,
    pub(crate) computes: u32,
    pub(crate) renders: u32,
    /// Compute: final FW stamp, ring end (exclusive) and ring position before the prefix.
    pub(crate) target: u32,
    pub(crate) head: u32,
    pub(crate) c_start: u32,
    /// Render: final TA/FS stamps, ring ends and ring positions before the prefix.
    pub(crate) ta_target: u32,
    pub(crate) fs_target: u32,
    pub(crate) ta_head: u32,
    pub(crate) fs_head: u32,
    pub(crate) ta_start: u32,
    pub(crate) fs_start: u32,
    /// Channel producers (TA, FS, compute) after the prefix's records.
    pub(crate) rec: [u32; 3],
    /// Compute member j's FW stamp target, and the FS target it waits on directly (0: none):
    /// its predecessors in the prefix (for the compute-hang check).
    pub(crate) cstamp: [u32; sizes::COMPUTE_SLOTS],
    pub(crate) cwait: [u32; sizes::COMPUTE_SLOTS],
    /// Compute slots, pass slots and clustering buffers it holds.
    pub(crate) claims: Claims,
    /// The prefix rebuilds the render allocators (priming); nothing is published behind it.
    pub(crate) rebuild: bool,
    /// A kill or recovery ended this prefix's VM: later prefixes of it fail with EIO.
    pub(crate) doomed: bool,
    /// Publication time, ns.
    pub(crate) published_ns: u64,
}

impl Flight {
    fn new(job: Arc<Job>) -> Self {
        Self {
            job,
            group: KVec::new(),
            first: 0,
            count: 0,
            computes: 0,
            renders: 0,
            target: 0,
            head: 0,
            c_start: 0,
            ta_target: 0,
            fs_target: 0,
            ta_head: 0,
            fs_head: 0,
            ta_start: 0,
            fs_start: 0,
            rec: [0; 3],
            cstamp: [0; sizes::COMPUTE_SLOTS],
            cwait: [0; sizes::COMPUTE_SLOTS],
            claims: Claims::new(),
            rebuild: false,
            doomed: false,
            published_ns: 0,
        }
    }
}

/// A prefix under preparation: members in API order, each in a claimed
/// compute or pass slot. `prev` is the engine of the previous member, of this prefix or else
/// the last member of the newest published one; `last_ct`/`last_fs` its compute/FS target.
#[derive(Default)]
struct Prefix {
    computes: u32,
    renders: u32,
    prev: Prev,
    /// This prefix (re)initializes the PB with InitBM.
    initbm: bool,
    c_head: u32,
    ta_head: u32,
    fs_head: u32,
    /// Last compute ordinal / last draw ordinal.
    n: u64,
    draw: u64,
    last_ct: u32,
    last_fs: u32,
    /// The last compute / render member has API timestamps.
    ct_ts: bool,
    fs_ts: bool,
}

/// Engine statistics, printed by `asahi-g15/stats`.
#[derive(Default)]
pub(crate) struct Stats {
    pub(crate) pub_ns: u64,
    pub(crate) idle_from_ns: u64,
    pub(crate) starve_from_ns: u64,
    pub(crate) prep_from_ns: u64,
    /// Idle with nothing ready; prefix preparation.
    pub(crate) starved_ns: u64,
    pub(crate) prep_ns: u64,
    /// Internal stage durations of retired prefixes (24 MHz ticks).
    pub(crate) ta_ticks: u64,
    pub(crate) fs_ticks: u64,
    pub(crate) cdm_ticks: u64,
    pub(crate) span_ticks: u64,
    /// [0] compute, [1] render, [2] mixed.
    pub(crate) count: [u64; 3],
    pub(crate) prefixes: [u64; 3],
    pub(crate) busy_ns: [u64; 3],
    pub(crate) max_ns: [u64; 3],
    pub(crate) first_ns: [u64; 3],
    pub(crate) last_ns: [u64; 3],
    pub(crate) gap_ns: u64,
    pub(crate) gaps: u64,
    pub(crate) yields: u64,
    /// Prefixes published while another ran.
    pub(crate) overlaps: u64,
    pub(crate) jobs: u64,
    pub(crate) job_cmds: u64,
    pub(crate) job_max: u64,
    pub(crate) early: u64,
    /// Type-0 FS->compute / compute->TA waits.
    pub(crate) fs2c0: u64,
    pub(crate) c2ta0: u64,
    pub(crate) clustered: u64,
    /// Jobs published behind another in a compute horizon.
    pub(crate) grouped: u64,
    /// Compute-only prefixes by jobs; why grouping stopped (no further ready job, the cap,
    /// compute slots, a job of the queue that does not qualify); behind a running prefix; of
    /// those behind its unconsumed record.
    pub(crate) hz: [u64; sizes::HORIZON_JOBS as usize],
    pub(crate) hz_stop: [u64; 4],
    pub(crate) hz_behind: u64,
    pub(crate) hz_ahead: u64,
    /// Why ready jobs stayed unpublished: both flights published, a channel record unconsumed,
    /// no free slot.
    pub(crate) hold: [u64; 3],
}

/// Host-side phase timers of publication and polling, on with `debug_flags` bit 49
/// (`Debug1`); printed as the `prof:` line of `asahi-g15/stats`. A measurement aid: nothing is
/// timed while the flag is off.
#[derive(Default)]
pub(crate) struct Prof {
    /// Published prefixes timed.
    pub(crate) prefixes: u64,
    /// Member selection, channel checks, allocator and ring-head setup.
    pub(crate) select_ns: u64,
    /// Encoders (images into the scratch buffers).
    pub(crate) encode_ns: u64,
    /// Slot, pass and clustering-buffer claims, with synchronous scrubs and flush waits.
    pub(crate) claim_ns: u64,
    /// Object writes: images, scene, barriers, InitBM, ring entries.
    pub(crate) write_ns: u64,
    /// Thresholds, queue WRITEs, channel records and doorbells.
    pub(crate) post_ns: u64,
    /// Poll passes, and the time of their drain, retirement and whole pass.
    pub(crate) polls: u64,
    pub(crate) drain_ns: u64,
    pub(crate) retire_ns: u64,
    pub(crate) poll_ns: u64,
}

/// A phase timer: reads the clock only while profiling is on.
pub(crate) struct Lap(Option<u64>);

impl Lap {
    pub(crate) fn start() -> Self {
        Lap(crate::debug::debug_enabled(crate::debug::DebugFlags::Debug1).then(now_ns))
    }

    /// Adds the time since the start or the last lap to `acc`.
    pub(crate) fn lap(&mut self, acc: &mut u64) {
        if let Some(t) = self.0 {
            let now = now_ns();
            *acc += now.wrapping_sub(t);
            self.0 = Some(now);
        }
    }

    pub(crate) fn on(&self) -> bool {
        self.0.is_some()
    }
}

/// Encoder output buffers, allocated once rather than per publication.
struct Scratch {
    cw: KVec<u8>,
    cq: KVec<u8>,
    ta: KVec<u8>,
    tas: KVec<u8>,
    fs: KVec<u8>,
    fss: KVec<u8>,
}

impl Scratch {
    fn new() -> Result<Self> {
        let z = |n| KVec::from_elem(0u8, n, GFP_KERNEL);
        Ok(Self {
            cw: z(work::compute::WORK_SIZE)?,
            cq: z(work::compute::SEQ_SIZE)?,
            ta: z(work::ta::WORK_SIZE)?,
            tas: z(work::ta::SEQ_SIZE)?,
            fs: z(work::fs::WORK_SIZE)?,
            fss: z(work::fs::SEQ_SIZE)?,
        })
    }
}

/// `&mut KVec<u8>` as a fixed-size image buffer.
fn image<const N: usize>(v: &mut KVec<u8>) -> Result<&mut [u8; N]> {
    <&mut [u8; N]>::try_from(&mut v[..]).map_err(|_| EINVAL)
}

/// Engine state.
pub(crate) struct Engine {
    /// Runnable jobs of every queue, in arrival order.
    pub(crate) pending: KVec<Arc<Job>>,
    /// Published prefixes, oldest first.
    pub(crate) flights: KVec<Flight>,
    /// Jobs whose storage a failed prefix may still reference, retained until reboot.
    pub(crate) retained: KVec<Arc<Job>>,
    /// Publication frozen by a kill request (recovery may use this or `Recovery::armed`; both
    /// freeze publication).
    pub(crate) frozen: bool,
    /// Service turn counter for the least-recently-served pick.
    pub(crate) turn: u64,
    /// The device's shared state, for the transport-error latch; set by the first poll or
    /// submission.
    pub(crate) shared: Option<Arc<Shared>>,
    /// Host compute ordinal and draw ordinal of the last publication.
    pub(crate) ordinal: u64,
    pub(crate) draw: u64,
    /// Newest published receipts, the tail.
    pub(crate) target: u32,
    pub(crate) head: u32,
    pub(crate) ta_target: u32,
    pub(crate) fs_target: u32,
    pub(crate) ta_head: u32,
    pub(crate) fs_head: u32,
    /// Event counts at the tail's publication.
    pub(crate) events: u32,
    pub(crate) stage_seen: u32,
    /// Last member of the newest prefix.
    pub(crate) tail_prev: Prev,
    pub(crate) tail_ct: u32,
    pub(crate) tail_fs: u32,
    /// The compute queue / render queues were registered by a first record (`new_queue`).
    pub(crate) queue_registered: bool,
    pub(crate) render_registered: bool,
    /// The rebuild (priming) prefix is published and not yet retired.
    pub(crate) rebuild_live: bool,
    /// A ready prefix waits only for a record's consumption: repoll sooner.
    pub(crate) chan_wait: bool,
    /// Progress tracking of the oldest prefix.
    pub(crate) progress_flight: u64,
    pub(crate) progress: [u32; 3],
    pub(crate) stats: Stats,
    /// Phase timers (debug_flags bit 49).
    pub(crate) prof: Prof,
    /// The `asahi-g15/stats` file.
    pub(crate) debugfs: Option<Pin<KBox<kernel::debugfs::File<Arc<Shared>>>>>,
    scratch: Option<KBox<Scratch>>,
}

impl Engine {
    pub(crate) fn new() -> Self {
        Self {
            pending: KVec::new(),
            flights: KVec::new(),
            retained: KVec::new(),
            frozen: false,
            turn: 0,
            shared: None,
            ordinal: 0,
            draw: 0,
            target: 0,
            head: 0,
            ta_target: 0,
            fs_target: 0,
            ta_head: 0,
            fs_head: 0,
            events: 0,
            stage_seen: 0,
            tail_prev: Prev::None,
            tail_ct: 0,
            tail_fs: 0,
            queue_registered: false,
            render_registered: false,
            rebuild_live: false,
            chan_wait: false,
            progress_flight: 0,
            progress: [0; 3],
            stats: Stats::default(),
            prof: Prof::default(),
            debugfs: None,
            scratch: None,
        }
    }

    /// The oldest published prefix.
    pub(crate) fn oldest(&self) -> Option<&Flight> {
        self.flights.first()
    }

    /// Jobs of the queue in published prefixes.
    fn queue_published(&self, job: &Job) -> u32 {
        self.flights
            .iter()
            .filter(|f| Arc::ptr_eq(&f.job.queue, &job.queue))
            .map(|f| 1 + f.group.len() as u32)
            .sum()
    }

    /// Index in `pending` of the next ready job: highest queue priority, then the queue served
    /// least recently, then submission order. `QueueState::priority` is
    /// `DRM_ASAHI_PRIORITY_REALTIME - priority` as `file.rs` passes it, so a smaller value
    /// outranks.
    fn pick(&self) -> Option<usize> {
        let mut best: Option<usize> = None;
        for (i, job) in self.pending.iter().enumerate() {
            let better = match best {
                None => true,
                Some(b) => {
                    let (q, bq) = (&job.queue, &self.pending[b].queue);
                    let (t, bt) = (
                        q.turn.load(Ordering::Relaxed),
                        bq.turn.load(Ordering::Relaxed),
                    );
                    q.priority < bq.priority || (q.priority == bq.priority && t < bt)
                }
            };
            if better {
                best = Some(i);
            }
        }
        best
    }

    /// Removes `job` from `pending`.
    fn unpend(&mut self, job: &Arc<Job>) {
        job.progress.set_pending(false);
        self.pending.retain(|j| !Arc::ptr_eq(j, job));
    }
}

/// The job's command `k`.
fn command(job: &Job, k: u32) -> Option<&Command> {
    job.commands.get(k as usize)
}

fn is_render(job: &Job, k: u32) -> bool {
    matches!(command(job, k), Some(Command::Render(_)))
}

/// Whole compute-only job.
fn compute_only(job: &Job) -> bool {
    (0..job.progress.count()).all(|k| !is_render(job, k))
}

/// Pushes a job's completion; [`Job::complete`] only acts the first time.
pub(crate) fn complete(done: &mut Completions, job: &Arc<Job>, result: Result) {
    if done.push((job.clone(), result), GFP_KERNEL).is_err() {
        // Signalling must not be lost: complete in place, under the caller's device lock.
        job.complete(result);
    }
}

impl Firmware {
    /// Accepts a job from the scheduler; it is published by [`Self::publish`]. Records how many
    /// jobs the queue owns at this arrival for the compute horizon.
    pub(crate) fn enqueue(&mut self, job: Arc<Job>) -> Result {
        if self.recovery.failed().is_some() {
            return Err(EIO);
        }
        if job.progress.count() == 0 || job.commands.is_empty() {
            return Err(EINVAL);
        }
        if self.engine.flights.capacity() < sizes::FLIGHTS {
            self.engine.flights.reserve(sizes::FLIGHTS, GFP_KERNEL)?;
        }
        self.engine.pending.push(job.clone(), GFP_KERNEL)?;
        job.progress.set_pending(true);
        let q = &job.queue;
        let owned = self.engine.queue_published(&job)
            + self
                .engine
                .pending
                .iter()
                .filter(|j| Arc::ptr_eq(&j.queue, q))
                .count() as u32;
        let i = q.owned_index.fetch_add(1, Ordering::Relaxed) as usize % q.owned.len();
        q.owned[i].store(owned.min(255), Ordering::Relaxed);
        Ok(())
    }

    /// Publishes further prefixes: with nothing published any ready job; behind published
    /// prefixes only the next job of the same VM, up to `max_inflight`, and not while recovery
    /// state is pending: neither the prefix that rebuilds the render allocators after an abort
    /// is published behind another one, nor anything behind it until it has retired. Jobs
    /// whose first publication failed before any visible record complete with an error.
    /// Returns Err only when this call latched the engine failed (a publication error after a
    /// queue WRITE advanced, or a partly sent job): the caller latches the fail-stop.
    pub(crate) fn publish(&mut self, done: &mut Completions) -> Result {
        let max_inflight = tuning::MAX_INFLIGHT.clamp(1, sizes::FLIGHTS as u32) as usize;
        let mut result = Ok(());

        self.engine.chan_wait = false;
        // Nothing is published while a hang request is armed.
        while self.recovery.failed().is_none()
            && !self.engine.frozen
            && self.recovery.armed.is_none()
            && !self.engine.pending.is_empty()
        {
            let Some(index) = self.engine.pick() else {
                break;
            };
            let job = self.engine.pending[index].clone();
            if let Some(tail) = self.engine.flights.last() {
                if self.engine.flights.len() >= max_inflight
                    || !Arc::ptr_eq(&job.vm, &tail.job.vm)
                    || self.recovery.reinit
                    || self.recovery.fs_resync
                    || self.engine.rebuild_live
                {
                    self.engine.stats.hold[0] += (self.engine.flights.len() >= max_inflight) as u64;
                    break;
                }
                if !Arc::ptr_eq(&tail.job, &job) && tail.job.progress.is_pending() {
                    self.engine.stats.yields += 1; // another queue's turn at a prefix boundary
                }
            }
            self.engine.turn += 1;
            job.queue.turn.store(self.engine.turn, Ordering::Relaxed);
            match self.publish_prefix(&job, done) {
                Ok(()) => {
                    if job.progress.sent() == job.progress.count() {
                        self.engine.unpend(&job); // fully published
                    }
                    continue;
                }
                Err(e) if e == EAGAIN => {
                    let i = if self.engine.chan_wait { 1 } else { 2 };
                    self.engine.stats.hold[i] += 1;
                    break; // storage or channel still held: on a later retirement
                }
                Err(e) => {
                    self.engine.unpend(&job);
                    if e == EIO || job.progress.sent() != 0 {
                        // Uncertain, or earlier commands ran: retain, stop the engine. No
                        // recovery: every VM is lost, also those with no failed job.
                        dev_err!(
                            self.graph.dev.as_ref(),
                            "G15: publication of ctx {} failed after a queue WRITE ({:?})\n",
                            job.vm.slot,
                            e
                        );
                        complete(done, &job, Err(EIO));
                        self.retain(&job);
                        self.fail_stop(EIO, done);
                        result = Err(EIO);
                        break;
                    }
                    complete(done, &job, Err(e));
                }
            }
        }
        result
    }

    /// Whether some of `job`'s commands were published: such a job is retained after a
    /// fail-stop.
    pub(crate) fn job_partly_published(&self, job: &Job) -> bool {
        job.progress.sent() != 0
    }

    /// Ready jobs the compute horizon of `job` could take now, besides `job` itself: the
    /// grouping rules of [`Self::publish_prefix`], at most `max - 1`.
    fn horizon_ready(&self, job: &Arc<Job>, mut computes: u32, free_c: u32, max: u32) -> u32 {
        let mut n = 0;
        for j2 in self.engine.pending.iter() {
            if 1 + n >= max {
                break;
            }
            if Arc::ptr_eq(j2, job) || !Arc::ptr_eq(&j2.queue, &job.queue) {
                continue;
            }
            let count = j2.progress.count();
            if !compute_only(j2)
                || !Arc::ptr_eq(&j2.vm, &job.vm)
                || j2.progress.sent() != 0
                || count == 0
                || computes + count > free_c
            {
                break;
            }
            computes += count;
            n += 1;
        }
        n
    }

    /// The allocator and graph addresses the render encoders name.
    fn render_refs(&self) -> RenderRefs {
        let r = &self.graph.render;
        let a = self.pools.allocator_refs();
        RenderRefs {
            hwdata: self.cfg.layout.hwdata_va,
            notifier: r.notifier.va,
            legacy_empty: r.legacy_empty.va,
            qinfo: [r.qinfo[0].va, r.qinfo[1].va],
            user_stamp: [r.user_stamp[0].va, r.user_stamp[1].va],
            fw_stamp: [r.fw_stamp[0].va, r.fw_stamp[1].va],
            uma_mgr: a.uma_mgr,
            pb_mgr: a.pb_mgr,
            scene_list: a.scene_list,
            mgr_misc: a.mgr_misc,
            pb_init_blocks: a.init_blocks,
        }
    }

    /// Publishes the next prefix of `job`, from its first unpublished command. Members:
    /// consecutive commands of one engine, or with `MIXED_PREFIX` of both engines in API order,
    /// each in a free compute or pass slot. All images and ring entries precede one WRITE per
    /// participating queue and the kicks (compute, FS, TA); host thresholds and counters name
    /// the final ordinals.
    ///
    /// With prefixes already published (same VM only), the new one is appended behind them on
    /// the running queues: it takes only slots they do not hold, its first member waits on
    /// their last member like any member on its predecessor, and its SubmitQueue records
    /// extend the queues' horizons (`new_queue` 0). Returns EAGAIN when it cannot be appended
    /// yet (no free slot, or a channel record still unconsumed); EIO once a queue WRITE
    /// advanced; any other error means nothing was published.
    fn publish_prefix(&mut self, job: &Arc<Job>, done: &mut Completions) -> Result {
        let mut f = Flight::new(job.clone());
        let result = self.publish_prefix_into(job, &mut f);
        match result {
            Err(e) if e != EIO => {
                // Never published: its storage is free again, and horizon jobs taken into it
                // fail with it.
                self.slots.release(&mut f.claims);
                for j2 in f.group.iter() {
                    complete(done, j2, Err(e));
                }
            }
            _ => {}
        }
        result
    }

    fn publish_prefix_into(&mut self, job: &Arc<Job>, f: &mut Flight) -> Result {
        let overlap = !self.engine.flights.is_empty();
        let mut mixed = tuning::MIXED_PREFIX;
        let mut pf = Prefix {
            n: self.engine.ordinal,
            draw: self.engine.draw,
            ..Default::default()
        };
        let i = job.progress.sent();
        let count = job.progress.count();
        let render0 = is_render(job, i);
        let (mut computes, mut renders, mut end) = (0u32, 0u32, i);
        let mut last_render = None;
        let mut hz_stop: Option<usize> = None;
        let mut rebuild = false;
        let prefix_renders = tuning::PREFIX_RENDERS.clamp(1, sizes::RENDER_SLOTS as u32);

        self.engine.stats.prep_from_ns = now_ns();
        let mut lap = Lap::start();
        let free_c = self.slots.free_compute();
        // After an abort the prefix that rebuilds the render allocators holds renders only (see
        // `publish`): a compute published with it, or behind it before it retired, crashed the
        // 2419 firmware in its progress path (F+0x35568, FAR 0x58) after a recovered hang whose
        // prefix had a compute waiting on the hung FS.
        if self.recovery.reinit || self.recovery.fs_resync {
            mixed = false;
        }
        while end < count {
            let cmd = command(job, end).ok_or(EINVAL)?;
            let render = matches!(cmd, Command::Render(_));
            if render != render0 && !mixed {
                break;
            }
            match cmd {
                Command::Compute(_) => {
                    if computes == free_c {
                        break;
                    }
                    computes += 1;
                }
                Command::Render(r) => {
                    if renders >= prefix_renders {
                        break;
                    }
                    // The prefix that primes freshly initialized render allocators (the first
                    // after boot, or after an abort's re-init) holds one render: their
                    // submitted cell is 1 and their completed count 0, so the first completed
                    // TA (FS) catches its allocator up and clears its registration. A second
                    // render of the prefix then holds a position of that allocator with no
                    // registered manager until it completes, and the firmware's
                    // progress/preemption path dereferences the missing manager: RTKit crash
                    // at F+0x35568, FAR 0x58.
                    if renders != 0 && (self.recovery.reinit || self.engine.draw == 0) {
                        break;
                    }
                    let geo = Geometry::new(&r.cmd, 0);
                    let Some(k) = self.slots.pick_pass(&geo, f.claims.pass_mask) else {
                        break;
                    };
                    f.claims.pass_mask |= 1 << k;
                    f.claims.pass[renders as usize] = k as u32;
                    renders += 1;
                }
            }
            end += 1;
        }
        if end == i {
            // Idle: every slot is free.
            return Err(if overlap { EAGAIN } else { E2BIG });
        }
        // The picked pass slots are not claimed yet: an early return below must not release
        // them.
        let picked = f.claims.pass_mask;
        f.claims.pass_mask = 0;
        // Participating channels: every earlier record consumed, or (a prefix with renders
        // behind a running one, SUBMIT_AHEAD) room behind the unconsumed ones. Behind a running
        // prefix the firmware takes its record within microseconds: retry soon rather than at
        // that prefix's retirement.
        let mut ahead = overlap && renders != 0 && tuning::SUBMIT_AHEAD;
        if overlap && renders == 0 && end == count && tuning::SUBMIT_AHEAD {
            let max = tuning::COMPUTE_HORIZON.clamp(1, sizes::HORIZON_JOBS);
            let ready = 1 + self.horizon_ready(job, computes, free_c, max);
            let depth = job
                .queue
                .owned
                .iter()
                .map(|o| o.load(Ordering::Relaxed))
                .max()
                .unwrap_or(0);
            ahead = ready >= max || ready + self.engine.queue_published(job) >= depth;
        }
        for ch in [Channel::Ta, Channel::Fs, Channel::Compute] {
            let used = if ch == Channel::Compute {
                computes != 0
            } else {
                renders != 0
            };
            if !used {
                continue;
            }
            let consumer = self.channels.read(ch)?;
            let producer = self.channels.producer(ch);
            let busy = if ahead {
                producer.wrapping_sub(consumer) % 256 >= sizes::AHEAD_RECORDS
            } else {
                consumer != producer
            };
            if busy || self.channels.write_cell(ch)? != producer {
                self.engine.chan_wait = overlap;
                return Err(if overlap { EAGAIN } else { EBUSY });
            }
        }
        f.claims.pass_mask = picked;
        self.slots.pass_busy |= f.claims.pass_mask;
        if renders != 0 {
            let cursor = match f.claims.pass[renders as usize - 1] {
                0 => self.slots.pass_cursor,
                k => k,
            };
            self.slots.pass_cursor = cursor % (sizes::RENDER_SLOTS as u32 - 1).max(1);
            // After a recovered render abort the allocator state is not trusted. The priming
            // prefix (also the first after boot) retires before anything is published behind
            // it.
            rebuild = self.recovery.reinit || self.engine.draw == 0;
            if self.recovery.reinit {
                self.init_render_pools(true)?;
                dev_info!(
                    self.graph.dev.as_ref(),
                    "G15: render allocators re-initialized; InitBM at draw {}\n",
                    self.engine.draw + 1
                );
            }
        }
        if self.engine.scratch.is_none() {
            self.engine.scratch = Some(KBox::new(Scratch::new()?, GFP_KERNEL)?);
        }
        if overlap {
            pf.prev = self.engine.tail_prev;
            pf.last_ct = self.engine.tail_ct;
            pf.last_fs = self.engine.tail_fs;
        }
        let cap = sizes::QUEUE_CAP;
        pf.c_head = self.graph.compute.qstate.r32(qstate::WRITE)? % cap;
        pf.ta_head = self.graph.render.qstate[0].r32(qstate::WRITE)? % cap;
        pf.fs_head = self.graph.render.qstate[1].r32(qstate::WRITE)? % cap;
        f.c_start = pf.c_head;
        f.ta_start = pf.ta_head;
        f.fs_start = pf.fs_head;
        for k in i..end {
            if is_render(job, k) {
                last_render = Some(k);
            }
        }
        let refs = self.render_refs();
        lap.lap(&mut self.engine.prof.select_ns);
        for k in i..end {
            match command(job, k).ok_or(EINVAL)? {
                Command::Render(r) => self.prefix_render(job, r, &refs, &mut pf, f)?,
                Command::Compute(c) => {
                    let later_render = last_render.is_some_and(|l| k < l);
                    self.prefix_compute(job, c, &mut pf, f, later_render)?
                }
            }
        }
        // Compute horizon: whole compute-only jobs of the same queue and VM that already wait
        // ready (their inputs resolved before the scheduler ran them), in queue order, into
        // slots still free. A job of that queue that does not qualify ends the group: no later
        // job overtakes it.
        if renders == 0 && end == count {
            let max = tuning::COMPUTE_HORIZON.clamp(1, sizes::HORIZON_JOBS);
            let mut idx = 0;
            hz_stop = Some(0);
            while idx < self.engine.pending.len() {
                if 1 + f.group.len() as u32 >= max {
                    hz_stop = Some(1);
                    break;
                }
                let j2 = self.engine.pending[idx].clone();
                if Arc::ptr_eq(&j2, job) || !Arc::ptr_eq(&j2.queue, &job.queue) {
                    idx += 1;
                    continue;
                }
                let n2 = j2.progress.count();
                if !compute_only(&j2)
                    || !Arc::ptr_eq(&j2.vm, &job.vm)
                    || j2.progress.sent() != 0
                    || n2 == 0
                {
                    hz_stop = Some(3);
                    break;
                }
                if computes + n2 > free_c {
                    hz_stop = Some(2);
                    break;
                }
                // Without room for another group entry the horizon ends here: the base job
                // is published already, and nothing of `j2` has been touched or sent, so a
                // transient allocation failure must not fail the prefix (which, after a queue
                // WRITE, would be taken for an uncertain post-WRITE state and stop the engine).
                if f.group.reserve(1, GFP_KERNEL).is_err() {
                    hz_stop = Some(1);
                    break;
                }
                for k in 0..n2 {
                    match command(&j2, k).ok_or(EINVAL)? {
                        Command::Compute(c) => self.prefix_compute(&j2, c, &mut pf, f, false)?,
                        Command::Render(_) => return Err(EINVAL),
                    }
                }
                computes += n2;
                j2.progress.set_sent(n2);
                self.engine.turn += 1;
                j2.queue.turn.store(self.engine.turn, Ordering::Relaxed);
                j2.progress.set_pending(false);
                let _ = self.engine.pending.remove(idx);
                // Reserved above.
                let _ = f.group.push(j2, GFP_KERNEL);
                self.engine.stats.grouped += 1;
            }
        }
        // Host thresholds/counters after the whole prefix. The notifiers' context (+0x24) is
        // not ours to write: every completing work stores its own context there before the
        // service reads it.
        let mut lap = Lap::start();
        if computes != 0 {
            self.graph.compute.threshold.w32(0, pf.n as u32)?;
            self.graph.compute.counter.w32(0, pf.n as u32)?;
        }
        if renders != 0 {
            self.graph
                .render
                .threshold
                .w32(0, 2u32.wrapping_mul(pf.draw as u32))?;
            // pb_counter = draw, then C1 once per (re)initialization.
            self.publish_pb(pf.draw as u32)?;
        }
        wmb();
        // The queue WRITEs: from here on a failure is uncertain (EIO).
        let writes = (|| -> Result {
            if computes != 0 {
                self.graph.compute.qstate.w32(qstate::WRITE, pf.c_head)?;
            }
            if renders != 0 {
                self.graph.render.qstate[0].w32(qstate::WRITE, pf.ta_head)?;
                self.graph.render.qstate[1].w32(qstate::WRITE, pf.fs_head)?;
            }
            Ok(())
        })();
        wmb();

        f.first = i as usize;
        f.count = (end - i) as usize;
        f.computes = computes;
        f.renders = renders;
        if computes != 0 {
            self.engine.ordinal = pf.n;
            f.target = pf.last_ct;
            self.engine.target = pf.last_ct;
            f.head = pf.c_head;
            self.engine.head = pf.c_head;
        }
        if renders != 0 {
            f.ta_target = encode::stamp_seed(ids::TA_SEED, pf.draw);
            self.engine.ta_target = f.ta_target;
            f.fs_target = pf.last_fs;
            self.engine.fs_target = pf.last_fs;
            f.ta_head = pf.ta_head;
            self.engine.ta_head = pf.ta_head;
            f.fs_head = pf.fs_head;
            self.engine.fs_head = pf.fs_head;
            self.engine.draw = pf.draw;
            self.recovery.fs_resync = false;
            self.recovery.reinit = false;
        }
        self.engine.tail_prev = pf.prev;
        self.engine.tail_ct = pf.last_ct;
        self.engine.tail_fs = pf.last_fs;
        self.engine.events = self.events.compute_events;
        self.engine.stage_seen = self.events.stage_events;
        self.stat_publish(f);
        if overlap {
            self.engine.stats.overlaps += 1;
        } else {
            self.renew_deadlines();
        }
        job.progress.set_sent(end);
        if let Some(stop) = hz_stop {
            let s = &mut self.engine.stats;
            s.hz[f.group.len()] += 1;
            s.hz_stop[stop] += 1;
            s.hz_behind += overlap as u64;
            s.hz_ahead += ahead as u64;
        }
        if rebuild {
            f.rebuild = true;
            self.engine.rebuild_live = true;
        }
        // Compute on channel 2, then FS on channel 1, then TA on channel 0.
        let mut posted = writes;
        if posted.is_ok() && computes != 0 {
            let rec = SubmitRecord {
                qinfo: self.graph.compute.qinfo.va,
                kind: 2,
                head: pf.c_head as u16,
                event: ids::EVENT_SLOT as u8,
                new_queue: !self.engine.queue_registered as u8,
                ..Default::default()
            };
            posted = self.post(Channel::Compute, &rec, ahead);
            if posted.is_ok() {
                self.engine.queue_registered = true;
                posted = self.session.ring(Channel::Compute);
            }
        }
        if posted.is_ok() && renders != 0 {
            let new_queue = !self.engine.render_registered as u8;
            let fs_rec = SubmitRecord {
                qinfo: self.graph.render.qinfo[1].va,
                kind: 1,
                head: pf.fs_head as u16,
                event: 1,
                new_queue,
                ..Default::default()
            };
            let ta_rec = SubmitRecord {
                qinfo: self.graph.render.qinfo[0].va,
                kind: 0,
                head: pf.ta_head as u16,
                event: 0,
                new_queue,
                ..Default::default()
            };
            posted = self.post(Channel::Fs, &fs_rec, ahead);
            if posted.is_ok() {
                posted = self.session.ring(Channel::Fs);
            }
            if posted.is_ok() {
                posted = self.post(Channel::Ta, &ta_rec, ahead);
            }
            if posted.is_ok() {
                posted = self.session.ring(Channel::Ta);
            }
            self.engine.render_registered = true;
        }
        f.rec = [
            self.channels.producer(Channel::Ta),
            self.channels.producer(Channel::Fs),
            self.channels.producer(Channel::Compute),
        ];
        if lap.on() {
            lap.lap(&mut self.engine.prof.post_ns);
            self.engine.prof.prefixes += 1;
        }
        // The flight is published: move it into the engine (capacity reserved in `enqueue`).
        let mut flight = Flight::new(job.clone());
        core::mem::swap(&mut flight, f);
        let _ = self.engine.flights.push(flight, GFP_KERNEL);
        crate::cls_dev_dbg!(
            Queue,
            self.graph.dev,
            "G15: prefix ctx={} computes {} (n={} head {}) renders {} (draw {} ta/fs head {}/{}) behind {}: {:?}\n",
            job.vm.slot,
            computes,
            pf.n,
            pf.c_head,
            renders,
            pf.draw,
            pf.ta_head,
            pf.fs_head,
            self.engine.flights.len() - 1,
            posted
        );
        // A queue WRITE already advanced: any later failure is uncertain.
        posted.map_err(|_| EIO)
    }

    /// Posts a submit record, behind unconsumed ones with `ahead`.
    fn post(&mut self, channel: Channel, record: &SubmitRecord, ahead: bool) -> Result {
        if ahead {
            self.channels.post_ahead(channel, record)?;
        } else {
            self.channels.post(channel, record)?;
        }
        Ok(())
    }

    /// Compute member: encode into its slot and append to the CDM ring. After a render of the
    /// same prefix it first waits on that render's FS: a distinct barrier entry ahead of the
    /// work.
    fn prefix_compute(
        &mut self,
        job: &Arc<Job>,
        cmd: &ComputeCommand,
        pf: &mut Prefix,
        f: &mut Flight,
        later_render: bool,
    ) -> Result {
        let n = pf.n + 1;
        let t = encode::stamp(n);
        let ctx = job.vm.slot;
        let ts = &cmd.timestamps;
        let mut lap = Lap::start();
        // Nothing published yet on failure.
        let index = self
            .slots
            .claim_compute(&mut f.claims, pf.computes)?
            .ok_or(EBUSY)?;
        lap.lap(&mut self.engine.prof.claim_ns);
        let scrub = self.slots.scrub.clone();
        let s = &scrub.compute[index];
        let g = &self.graph.compute;
        let refs = ComputeRefs {
            hwdata: self.cfg.layout.hwdata_va,
            notifier: g.notifier.va,
            qinfo: g.qinfo.va,
            user_stamp: g.user_stamp.va,
            fw_stamp: g.fw_stamp.va,
            manager: g.manager.va,
            work: s.work.va,
            work_gva: s.work.gva.unwrap_or(0),
            seq: s.seq.va,
            preempt_gva: s.preempt.gva.unwrap_or(0),
            its: s.its.va,
            tail: s.tail.va,
            scratch: [
                s.scratch[0].va,
                s.scratch[1].va,
                s.scratch[2].va,
                s.scratch[3].va,
                s.scratch[4].va,
            ],
        };
        let scratch = self.engine.scratch.as_deref_mut().ok_or(ENOMEM)?;
        let (w, q) = (&mut scratch.cw, &mut scratch.cq);
        encode::encode_compute(
            &refs,
            &self.cfg.compute,
            &cmd.cmd,
            &cmd.attachments,
            ts,
            ctx,
            n,
            tuning::COMPUTE_FAULT_MASK,
            image(w)?,
            image(q)?,
        );
        lap.lap(&mut self.engine.prof.encode_ns);
        // Rewrite the slot's images (its runtime storage is already scrubbed).
        s.work.fill(0, 0, work::compute::WORK_SIZE)?;
        s.seq.fill(0, 0, work::compute::SEQ_OBJECT)?;
        s.work.write(0, &w[..])?;
        s.seq.write(0, &q[..])?;

        let g = &self.graph.compute;
        if pf.prev == Prev::Render {
            // Type 0 when the FS is of this prefix and a later render of the prefix follows
            // it: its X names a later scene, so this FS takes no PB catch-up at this point.
            let type0 = pf.renders != 0
                && later_render
                && !pf.initbm
                && !self.recovery.fs_resync
                && !pf.fs_ts
                && ts[0] == 0
                && ts[1] == 0
                && tuning::CROSS_ENGINE_HOLD & 1 == 0;
            let b = encode::barrier(
                self.graph.render.fw_stamp[1].va,
                pf.last_fs,
                1,
                t,
                ids::COMPUTE_UUID,
                BarrierType::from_stamp_only(type0),
            );
            s.barrier.write(0, &b)?;
            self.engine.stats.fs2c0 += type0 as u64;
            g.ring.w64(8 * pf.c_head as usize, s.barrier.va)?;
            pf.c_head = (pf.c_head + 1) % sizes::QUEUE_CAP;
        } else if pf.prev == Prev::Compute && tuning::COMPUTE_QUEUE_CLASS != 0 {
            // Class 2 runs computes of the queue concurrently: keep the ring order with a wait
            // on the previous compute's FW stamp. Type 1: a successor admitted with only the
            // stamp wait makes the firmware decline the compute request that ends a hang.
            let b = encode::barrier(
                g.fw_stamp.va,
                pf.last_ct,
                ids::EVENT_SLOT,
                t,
                ids::COMPUTE_UUID,
                BarrierType::Hold,
            );
            s.barrier.write(0, &b)?;
            g.ring.w64(8 * pf.c_head as usize, s.barrier.va)?;
            pf.c_head = (pf.c_head + 1) % sizes::QUEUE_CAP;
        }
        let j = pf.computes as usize;
        f.cstamp[j] = t;
        f.cwait[j] = if pf.prev == Prev::Render {
            pf.last_fs
        } else {
            0
        };
        g.ring.w64(8 * pf.c_head as usize, s.work.va)?;
        pf.c_head = (pf.c_head + 1) % sizes::QUEUE_CAP;
        lap.lap(&mut self.engine.prof.write_ns);
        if n <= 4 {
            crate::cls_dev_dbg!(
                Compute,
                self.graph.dev,
                "G15: compute n={} slot {} ctx={} cdm {:#x}..{:#x} att={} ts={:#x}/{:#x}\n",
                n,
                index,
                ctx,
                cmd.cmd.cdm_ctrl_stream_base,
                cmd.cmd.cdm_ctrl_stream_end,
                cmd.attachments.len(),
                ts[0],
                ts[1]
            );
        }
        pf.computes += 1;
        pf.n = n;
        pf.last_ct = t;
        pf.ct_ts = ts[0] != 0 || ts[1] != 0;
        pf.prev = Prev::Compute;
        Ok(())
    }

    /// Render member: TA and FS work, sequences, scene and barriers in its pass slot, appended
    /// to the TA and FS rings. Its TA waits on the previous FS (full order), on the previous
    /// TA (early tiling: a flagged later render after a render, its FS then also waiting on the
    /// previous FS), or on the previous compute of the prefix; its FS always waits on its own
    /// TA.
    fn prefix_render(
        &mut self,
        job: &Arc<Job>,
        cmd: &RenderCommand,
        refs: &RenderRefs,
        pf: &mut Prefix,
        f: &mut Flight,
    ) -> Result {
        let u = &cmd.cmd;
        let ts = &cmd.timestamps;
        let k = f.claims.pass[pf.renders as usize] as usize;
        let n = pf.draw + 1;
        let usc = job.queue.usc_base;
        let ctx = job.vm.slot;
        let tta = encode::stamp_seed(ids::TA_SEED, n);
        let tfs = encode::stamp_seed(ids::FS_SEED, n);
        // Early tiling: a later render member after a render of the same prefix whose render
        // dependencies are fragment-only (FRAGMENT_BARRIERS) and that has no start timestamps
        // lets its TA wait for the previous TA; its FS still waits for the previous FS, then
        // its own TA. Compute dependencies are already ahead of the previous render's TA.
        let early = pf.computes + pf.renders != 0
            && pf.prev == Prev::Render
            && params::early_tiling()
            && u.flags & uapi::drm_asahi_render_flags_DRM_ASAHI_RENDER_FRAGMENT_BARRIERS != 0
            && ts[0] == 0
            && ts[2] == 0;
        let scrub = self.slots.scrub.clone();
        let p = &scrub.passes[k];
        let pass = p.refs();
        let mut lap = Lap::start();
        let geo = Geometry::new(u, pass.heap_gva);
        // Clustered tiling: one layer, one sample, 32x32 utiles, unless the submission asked
        // for single-cluster vertex work, and a zeroed buffer holds this geometry.
        let mut cl = None;
        if encode::cluster_eligible(u, &geo) {
            let bytes = ClusterLayout::new(&geo, 0).bytes;
            if let Some(c) = self.slots.claim_ct(&mut f.claims, bytes)? {
                let base = scrub.cts[c].gva.unwrap_or(0);
                cl = Some(ClusterLayout::new(&geo, base));
            }
        }
        lap.lap(&mut self.engine.prof.claim_ns);
        let scratch = self.engine.scratch.as_deref_mut().ok_or(ENOMEM)?;
        encode::encode_ta(
            refs,
            &pass,
            u,
            ts,
            ctx,
            tta,
            &geo,
            cl.as_ref(),
            image(&mut scratch.ta)?,
            image(&mut scratch.tas)?,
        );
        encode::encode_fs(
            refs,
            &pass,
            u,
            ts,
            ctx,
            usc,
            n,
            tfs,
            &geo,
            image(&mut scratch.fs)?,
            image(&mut scratch.fss)?,
        );

        lap.lap(&mut self.engine.prof.encode_ns);
        // The slot is unpublished: its previous prefix fully retired. Its storage was reset in
        // the background since then (flight release), or is reset here.
        self.slots.claim_pass(k)?;
        self.slots
            .set_pass_dirty(k, geo.tpc_bytes, geo.tilemap_bytes);
        lap.lap(&mut self.engine.prof.claim_ns);
        if cl.is_some() {
            self.engine.stats.clustered += 1; // its buffer is zero: cleared since its last use
        }
        let scratch = self.engine.scratch.as_deref().ok_or(ENOMEM)?;
        p.ta_work.write(0, &scratch.ta[..])?;
        p.fs_work.write(0, &scratch.fs[..])?;
        p.ta_seq.write(0, &scratch.tas[..])?;
        p.fs_seq.write(0, &scratch.fss[..])?;
        p.scene.write(0, &encode::scene(refs, &pass, n))?;

        let r = &self.graph.render;
        if pf.prev == Prev::Compute {
            // Compute -> render: the TA waits on the previous compute; type 0 when that compute
            // is of this prefix and this render starts no InitBM, on the class-0 compute queue
            // only (on class 2 a TA admitted with only the stamp wait makes the firmware
            // decline the request that ends a hung compute).
            let type0 = pf.computes != 0
                && !pf.initbm
                && !self.recovery.fs_resync
                && n != 1
                && !(pf.renders == 0 && self.recovery.reinit)
                && !pf.ct_ts
                && ts.iter().all(|t| *t == 0)
                && tuning::CROSS_ENGINE_HOLD & 2 == 0
                && tuning::COMPUTE_QUEUE_CLASS == 0;
            let b = encode::barrier(
                self.graph.compute.fw_stamp.va,
                pf.last_ct,
                ids::EVENT_SLOT,
                tta,
                RENDER_WAIT_UUID,
                BarrierType::from_stamp_only(type0),
            );
            p.prev_barrier.write(0, &b)?;
            self.engine.stats.c2ta0 += type0 as u64;
        } else if early {
            let b = encode::barrier(
                r.fw_stamp[0].va,
                encode::stamp_seed(ids::TA_SEED, n - 1),
                0,
                tta,
                RENDER_WAIT_UUID,
                BarrierType::Hold,
            );
            p.prev_barrier.write(0, &b)?;
            let b = encode::barrier(
                r.fw_stamp[1].va,
                pf.last_fs,
                1,
                tfs,
                RENDER_WAIT_UUID,
                BarrierType::Hold,
            );
            p.fs_dep.write(0, &b)?;
            self.engine.stats.early += 1;
        } else {
            let mut prev_fs = if pf.prev != Prev::None {
                pf.last_fs
            } else if n == 1 {
                ids::FS_SEED
            } else {
                encode::stamp_seed(ids::FS_SEED, n - 1)
            };
            // Restricted type 0: predecessor render in this prefix, no InitBM/resync.
            let internal = pf.prev == Prev::Render
                && pf.renders != 0
                && !pf.initbm
                && !self.recovery.fs_resync
                && tuning::RENDER_INTERNAL_HOLD == 0;
            // After a recovered failure the previous FS may never have written.
            if pf.prev == Prev::None && self.recovery.fs_resync {
                prev_fs = r.fw_stamp[1].r32(0)?;
            }
            let b = encode::barrier(
                r.fw_stamp[1].va,
                prev_fs,
                1,
                tta,
                RENDER_WAIT_UUID,
                BarrierType::from_stamp_only(internal),
            );
            p.prev_barrier.write(0, &b)?;
        }
        let b = encode::barrier(
            r.fw_stamp[0].va,
            tta,
            0,
            tfs,
            ids::FS_UUID,
            BarrierType::Stamp,
        );
        p.own_barrier.write(0, &b)?;
        let cap = sizes::QUEUE_CAP;
        if n == 1 || (pf.renders == 0 && self.recovery.reinit) {
            pf.initbm = true;
            let bm = encode::initbm(ctx, refs.pb_init_blocks, refs.pb_mgr, tta);
            r.initbm.write(0, &bm)?;
            r.ring[0].w64(8 * pf.ta_head as usize, r.initbm.va)?;
            pf.ta_head = (pf.ta_head + 1) % cap;
        }
        // Rings: TA [InitBM,] predecessor wait, TA; FS [prior-FS wait if early,] own-TA wait,
        // FS.
        r.ring[0].w64(8 * pf.ta_head as usize, p.prev_barrier.va)?;
        pf.ta_head = (pf.ta_head + 1) % cap;
        r.ring[0].w64(8 * pf.ta_head as usize, p.ta_work.va)?;
        pf.ta_head = (pf.ta_head + 1) % cap;
        if early {
            r.ring[1].w64(8 * pf.fs_head as usize, p.fs_dep.va)?;
            pf.fs_head = (pf.fs_head + 1) % cap;
        }
        r.ring[1].w64(8 * pf.fs_head as usize, p.own_barrier.va)?;
        pf.fs_head = (pf.fs_head + 1) % cap;
        r.ring[1].w64(8 * pf.fs_head as usize, p.fs_work.va)?;
        pf.fs_head = (pf.fs_head + 1) % cap;
        lap.lap(&mut self.engine.prof.write_ns);
        if n <= 4 {
            crate::cls_dev_dbg!(
                Render,
                self.graph.dev,
                "G15: render n={} slot {} ctx={} {}x{} L{} s{} utile {}x{} ss{} flags={:#x} vdm {:#x} ta/fs head {}/{} tpc {:#x} tilemap {:#x}\n",
                n,
                k,
                ctx,
                u.width_px,
                u.height_px,
                u.layers,
                u.samples,
                u.utile_width_px,
                u.utile_height_px,
                u.sample_size_B,
                u.flags,
                u.vdm_ctrl_stream_base,
                pf.ta_head,
                pf.fs_head,
                geo.tpc_bytes,
                geo.tilemap_bytes
            );
        }
        pf.renders += 1;
        pf.draw = n;
        pf.last_fs = tfs;
        pf.fs_ts = ts.iter().any(|t| *t != 0);
        pf.prev = Prev::Render;
        Ok(())
    }

    /// Records the publication time of `f` and the preparation, gap and starvation times.
    fn stat_publish(&mut self, f: &mut Flight) {
        let s = &mut self.engine.stats;
        s.pub_ns = now_ns();
        f.published_ns = s.pub_ns;
        s.prep_ns += s.pub_ns.wrapping_sub(s.prep_from_ns);
        if s.idle_from_ns != 0 {
            s.gap_ns += s.pub_ns.wrapping_sub(s.idle_from_ns);
            s.gaps += 1;
            s.idle_from_ns = 0;
        } else if s.starve_from_ns != 0 {
            s.starved_ns += s.pub_ns.wrapping_sub(s.starve_from_ns);
        }
        s.starve_from_ns = 0;
    }
}
