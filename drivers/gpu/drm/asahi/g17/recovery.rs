// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Firmware restart handshake and per-VM failure classification.
//!
//! The device mutex serializes this state with publication. Preparations are closed and joined
//! without that mutex before `service`; no queue callback may signal a fence here. Queue
//! owners retain failed work until its retirement witness, a complete pair-drain witness, or
//! processor stop. A failed fence alone never releases mappings.

mod attribution;
mod slot;
mod safety;
pub(crate) use safety::{note_retired, since_retired};
pub(crate) use attribution::{Sources, PAGE_FAULT};
use attribution::Attribution;

pub(super) fn slot_key(slot: usize, slot_keys: Option<u64>) -> Option<u64> {
    slot::key(slot, slot_keys?).ok()
}

use super::status::VmStatus;
use kernel::{prelude::*, sync::Arc};

const QIDS: u8 = 128;
const GUILTY: u8 = 1 << 1;
const CANCELLED: u8 = 1 << 2;
const MAX_GUILTY_VMS: usize = 8;
/// One-millisecond ticks per handshake phase. The firmware finishes a restart only after the
/// work it interrupted has drained; with several clients running long ray-traversal jobs that
/// takes seconds, and giving up loses every client's device. Ten seconds matches the longest
/// execution allowance (`timeout::POLL_MS` plus renewals).
const RECOVERY_POLLS: usize = 10_000;

/// Records in the firmware recovery table.
pub(super) const RECOVERY_RECORDS: usize = 256;

/// Per-QID classifications from the firmware recovery table.
/// Multiple records for a queue contribute all of their classification bits.
#[derive(Default)]
pub(super) struct Entries {
    guilty: u128,
    cancelled: u128,
    /// Valid records, each naming one command that was in flight at the restart.
    pub(super) listed: u32,
    /// Listed commands found by their exact stamp, and by the oldest-work fallback.
    pub(super) exact: u32,
    pub(super) fallback: u32,
    /// QIDs with a listed command found by its exact stamp: that command was dropped.
    exact_qids: u128,
}

impl Entries {
    /// Counts a valid record. Returns false for an invalid one, which the walk skips: the
    /// table need not be packed (see `read_entries`).
    pub(super) fn include(&mut self, flags: u32) -> bool {
        if flags & 1 == 0 {
            return false;
        }
        let qid = (flags >> 4) & 0x7f;
        let bit = 1u128 << qid;
        if flags & u32::from(GUILTY) != 0 {
            self.guilty |= bit;
        }
        if flags & u32::from(CANCELLED) != 0 {
            self.cancelled |= bit;
        }
        self.listed += 1;
        true
    }
}

/// How a listed recovery record was matched to retained work.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(super) enum Interrupted {
    /// No retained command of the queue.
    None,
    /// The command whose completion stamp equals the record's.
    Exact,
    /// No stamp matched; the queue's oldest unretired command was marked instead.
    Oldest,
}

/// How an error completion affects its VM's sticky status and other jobs.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(super) enum Class {
    /// Only this work's fence fails; VM status and unrelated work remain usable.
    Spared,
    /// This VM owns guilty work.
    Guilty,
}

/// Queue operations under the device mutex. Completion signalling belongs to the caller after
/// that mutex is dropped. `oldest_spared` returns None if the QID has no in-flight owner.
pub(super) trait Queues {
    fn render_scheduler_active(&self) -> bool;
    fn render_pass_started(&self, qid: u8, stamp: u64) -> Option<bool>;
    /// Read while firmware is halted, like `render_pass_started`. The QIDs of every pair of a VM
    /// an earlier recovery found guilty with a render that started and did not finish. Without
    /// one, and only when the oldest published render of the pair holding `blamed` had not
    /// started, those of the one VM with such a render. None if no such VM, several, or any
    /// other VM holds compute work, which these records cannot see.
    fn render_culprit(&self, _blamed: u8) -> Option<u128> { None }
    /// Copy every retained queue's live producer to its recovery target, then
    /// issue a full barrier. A failed graph must not skip subsequent queues.
    fn publish_recovery_targets(&self) -> Option<Error>;
    fn oldest_spared(&self, qid: u8) -> Option<bool>;
    fn visit_owners(&self, qid: u8, visit: &mut dyn FnMut(&Arc<VmStatus>));
    /// Classify every in-flight completion; return QIDs of render slots with spared work.
    fn classify_work(&mut self, class: &dyn Fn(&Arc<VmStatus>) -> Class) -> u128;
    /// Mark the command a recovery record lists as interrupted. It may have
    /// partly executed, so it must never report success; see
    /// `Completion::note_interrupted`.
    fn note_interrupted(&mut self, qid: u8, stamp: u32) -> Interrupted;
    /// Optional diagnostics only; absence cannot alter attribution or release.
    fn report_blamed_render(&self, _qid: u8) {}
    fn settle_blamed_render(&mut self, qid: u8, error: Error);
    fn settle_cancelled_renders(&mut self, qids: u128);
    fn rescan_compute(&mut self) -> Result;
    fn release_retired_compute(&mut self);
}

/// Live shared-memory operations. The handshake word uses aligned 32-bit accesses; transitions
/// check the expected value, issue a full barrier, store the next value, then another barrier.
pub(super) trait Memory {
    fn recovery_state(&self) -> Result<u32>;
    fn host_recovery(&self) -> Result<u32>;
    fn fault_sources(&self) -> Result<Sources>;
    fn transition(&self, expected: u32, next: u32) -> Result;
    fn clear_timestamps(&self) -> Result;
    /// One recovery record's flags and completion stamp, read while firmware
    /// is halted in state 3.
    fn entry(&self, index: usize) -> Result<(u32, u32)>;
    /// One millisecond wait; this is polled because the firmware sends no state-3 notification.
    fn wait_tick(&self);
    /// The M5 firmware's recovery table, whose stamp form is not the G17P one.
    fn is_m5(&self) -> bool;
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
struct Request {
    generation: u64,
    blamed: Option<u8>,
}

/// Generation accepted by the host and the coalesced request from either coprocessor.
pub(super) struct State {
    accepted: u64,
    pending: Option<Request>,
    timeouts: safety::Storm,
    restarts: safety::Storm,
}

/// Non-fatal post-recovery failures. The firmware handshake has already closed successfully.
pub(super) struct ServiceResult {
    pub(super) scan_error: Option<Error>,
    pub(super) target_error: Option<Error>,
    /// Every VM found guilty had failed before, and the restart dropped one of its commands.
    pub(super) leftover: bool,
}

impl State {
    pub(super) const fn new() -> Self {
        Self {
            accepted: 0,
            pending: None,
            timeouts: safety::Storm::new(),
            restarts: safety::Storm::new(),
        }
    }
    pub(super) fn timeout_limit(&mut self, now: u64) -> bool {
        self.timeouts.note_unless_progress(now, safety::retired())
    }
    pub(super) fn restart_limit(&mut self, now: u64) -> bool { self.restarts.note(now) }
    /// A restart that only dropped leftover work of failed VMs (`ServiceResult::leftover`)
    /// does not count toward the storm limit. Returns whether it was forgiven.
    pub(super) fn forgive_restart(&mut self) -> bool { self.restarts.forgive() }
    pub(super) fn generation(&self) -> u64 {
        self.accepted
    }
    pub(super) fn pending(&self) -> bool {
        self.pending.is_some()
    }

    /// Returns false for a consumed duplicate from an older generation, true when the event
    /// drain must stop until the current request is serviced. The first blamed QID wins.
    pub(super) fn request(&mut self, generation: u64, stamp: i32) -> Result<bool> {
        if generation < self.accepted {
            return Ok(false);
        }
        if generation != self.accepted {
            return Err(EIO);
        }
        if stamp != -1 && !(0..i32::from(QIDS)).contains(&stamp) {
            return Err(EIO);
        }
        let blamed = if (0..i32::from(QIDS)).contains(&stamp) {
            Some(stamp as u8)
        } else {
            None
        };
        if let Some(pending) = self.pending.as_mut() {
            if pending.generation != generation {
                return Err(EBUSY);
            }
            pending.blamed = pending.blamed.or(blamed);
        } else {
            self.pending = Some(Request { generation, blamed });
        }
        Ok(true)
    }
}

/// The firmware owner implements memory and queue access without splitting its
/// mutable borrow or duplicating the authoritative restart generation.
pub(super) trait Host: Memory + Queues {
    fn recovery(&self) -> &State;
    fn recovery_mut(&mut self) -> &mut State;
}

/// Performs one pending restart. The caller has closed preparation admission and joined all
/// builders off-lock. Work remains mapped throughout every callback, including on error.
pub(super) fn service<H: Host>(
    host: &mut H,
    crashed: impl Fn() -> bool,
) -> Result<ServiceResult> {
    let request = host.recovery().pending.ok_or(EINVAL)?;
    if request.generation != host.recovery().accepted || host.host_recovery()? != 0 {
        return Err(EIO);
    }
    // The restart record is published before firmware announces state 1.
    // Its ring cursor is already consumed; wait only for that readiness word,
    // without advancing the host generation or acknowledging an early event.
    let mut ready = false;
    for _ in 0..RECOVERY_POLLS {
        if crashed() {
            return Err(EIO);
        }
        match host.recovery_state()? {
            1 => {
                ready = true;
                break;
            }
            0 => host.wait_tick(),
            _ => return Err(EIO),
        }
    }
    if !ready {
        pr_warn!("G17: recovery gen {} did not reach state 1 (state {:?})\n",
            request.generation, host.recovery_state());
        return Err(ETIMEDOUT);
    }
    // Recheck ownership after the off-CPU waits, before any handshake mutation.
    if host.host_recovery()? != 0 {
        return Err(EIO);
    }
    host.recovery_mut().accepted = host.recovery().accepted.wrapping_add(1);
    if request.generation == 0 && !host.render_scheduler_active() {
        host.clear_timestamps()?;
    }
    let mut sources = host.fault_sources()?;
    sources.report(request.generation, super::queue::compute::retirement::stampless_total());
    if let Some(qid) = request.blamed {
        host.report_blamed_render(qid);
    }
    // The keys, monitor and pass-start evidence must describe the same halted
    // instant. Firmware may update pass timestamps as soon as state 3 is
    // acknowledged; sampling them during classification can blame a bystander.
    let attribution = sources.attribute(request.blamed,
        |qid, stamp| host.render_pass_started(qid, stamp));
    let attribution = sources.host_fallback(attribution,
        || host.render_culprit(request.blamed?));
    if let Attribution::Queues(qids) = attribution {
        pr_warn!("G17: recovery gen {} blames queues {:#x}\n", request.generation, qids);
    }
    let target_error = host.publish_recovery_targets();
    host.transition(1, 2)?;
    let mut recovered = false;
    for _ in 0..RECOVERY_POLLS {
        if host.recovery_state()? == 3 {
            recovered = true;
            break;
        }
        if crashed() {
            return Err(EIO);
        }
        host.wait_tick();
    }
    if !recovered {
        pr_warn!("G17: recovery gen {} did not reach state 3 (state {:?})\n",
            request.generation, host.recovery_state());
        return Err(ETIMEDOUT);
    }
    // A failed table read leaves only the explicit blamed QID as evidence of guilt.
    // Every listed command was in flight when firmware stopped. Firmware may
    // still retire it with stamps and records that look like success, so it is
    // marked before firmware resumes and never reports success.
    let entries = read_entries(host);
    host.transition(3, 0)?;
    if host.recovery_state()? != 0 {
        return Err(EIO);
    }
    host.recovery_mut().pending = None;

    let (listed, exact, fallback) = (entries.listed, entries.exact, entries.fallback);
    let (cancelled, blame, leftover) = classify(host, entries, request.blamed, attribution);
    if listed != 0 {
        pr_warn!(
            "G17: GPU recovery: {} commands were in flight ({} matched by stamp, {} by queue); they fail instead of completing\n",
            listed,
            exact,
            fallback
        );
    }
    if let Some(qid) = request.blamed {
        pr_warn!(
            "G17: GPU recovery (reason {:?}): firmware blamed queue {}, {}\n",
            sources.reason,
            qid,
            if blame { "failing its oldest render" } else { "attribution cleared it; spared" }
        );
        if blame {
            host.settle_blamed_render(qid, EIO);
        }
    }
    if cancelled != 0 {
        host.settle_cancelled_renders(cancelled);
    }
    let scan_error = host.rescan_compute().err();
    host.release_retired_compute();
    Ok(ServiceResult { scan_error, target_error, leftover })
}

/// Walks every record of the recovery table: the table need not be packed, so an invalid
/// record is skipped rather than ending the walk. Every listed command is
/// marked interrupted where the stamp form is known (not on M5, whose table only classifies).
/// A failed read keeps what was read so far.
fn read_entries<H: Host>(host: &mut H) -> Entries {
    let mut entries = Entries::default();
    let mark = !host.is_m5();
    for index in 0..RECOVERY_RECORDS {
        let Ok((flags, stamp)) = host.entry(index) else {
            break;
        };
        if !entries.include(flags) || !mark {
            continue;
        }
        let qid = ((flags >> 4) & 0x7f) as u8;
        match host.note_interrupted(qid, stamp) {
            Interrupted::Exact => {
                entries.exact += 1;
                entries.exact_qids |= 1u128 << qid;
            }
            Interrupted::Oldest => entries.fallback += 1,
            Interrupted::None => {}
        }
    }
    entries
}

/// Resolves guilty QIDs before changing any completion's classification. Unknown ownership or
/// too many distinct guilty VMs leaves all prior classifications intact. The oldest spared work
/// of a QID makes later blame collateral to its previous recovery, rather than new VM guilt.
/// Also returns whether the firmware's blamed QID is still guilty after attribution: a blamed
/// render that had not started while others had (or with no culprit at all) is innocent, and its
/// work is spared like that of every other innocent VM instead of failing with EIO.
/// Last, whether every guilty VM had failed before this restart and it dropped one of their
/// commands: a failed VM publishes nothing new, so such restarts end with its queued work.
fn classify<Q: Queues>(queues: &mut Q, entries: Entries, blamed: Option<u8>, attribution: Attribution) -> (u128, bool, bool) {
    let mut guilty_qids = entries.guilty | blamed.map_or(0, |qid| 1u128 << qid);
    let cancelled = entries.cancelled;
    match attribution {
        Attribution::Firmware => {}
        Attribution::Queues(qids) => guilty_qids = qids,
        Attribution::NoCulprit => guilty_qids = 0,
    }
    let no_culprit = attribution == Attribution::NoCulprit;
    let blame = blamed.is_some_and(|qid| guilty_qids & (1u128 << qid) != 0);
    if guilty_qids == 0 && !no_culprit {
        return (0, blame, false);
    }
    let mut guilty: [Option<Arc<VmStatus>>; MAX_GUILTY_VMS] = Default::default();
    let mut count = 0;
    let mut collateral = 0u128;
    let mut unresolved = false;
    for qid in 0..QIDS {
        if guilty_qids & (1u128 << qid) == 0 {
            continue;
        }
        match queues.oldest_spared(qid) {
            None => unresolved = true,
            Some(true) => collateral |= 1u128 << qid,
            Some(false) => {
                let mut found = false;
                queues.visit_owners(qid, &mut |status| {
                    found = true;
                    if guilty[..count]
                        .iter()
                        .flatten()
                        .any(|known| Arc::ptr_eq(known, status))
                    {
                        return;
                    }
                    if let Some(slot) = guilty.get_mut(count) {
                        *slot = Some(status.clone());
                        count += 1;
                    } else {
                        unresolved = true;
                    }
                });
                unresolved |= !found;
            }
        }
    }
    if unresolved || (count == 0 && collateral == 0 && !no_culprit) {
        return (0, blame, false);
    }
    let known = |status: &Arc<VmStatus>| guilty[..count].iter().flatten().any(|known| Arc::ptr_eq(known, status));
    let mut dropped = false;
    for qid in (0..QIDS).filter(|qid| entries.exact_qids & (1u128 << qid) != 0) {
        queues.visit_owners(qid, &mut |status| dropped |= known(status));
    }
    let leftover = count != 0 && dropped && guilty[..count].iter().flatten().all(|status| status.get() != 0);
    guilty[..count].iter().flatten().for_each(|status| status.note_recovery_guilty());
    let spared = queues.classify_work(&|status| {
        guilty[..count]
            .iter()
            .find_map(|known| {
                known
                    .as_ref()
                    .filter(|known| Arc::ptr_eq(known, status))
                    .map(|_| Class::Guilty)
            })
            // A VM an earlier recovery found guilty stays guilty. Spared, its leftover work would
            // make the next blame on it collateral, and that restart would count as a storm.
            .unwrap_or(if status.recovery_guilty() { Class::Guilty } else { Class::Spared })
    });
    (cancelled & !(guilty_qids & !collateral) & spared, blame, leftover)
}


#[kernel::macros::kunit_tests(asahi_g17_recovery_snapshot)]
mod tests {
    use super::*;
    use core::cell::Cell;

    struct Halted {
        recovery: State,
        state: Cell<u32>,
        reads: Cell<usize>,
        known_keys: bool,
        blamed: bool,
    }

    impl Halted {
        fn new(known_keys: bool) -> Self {
            let mut recovery = State::new();
            assert!(recovery.request(0, 38).unwrap());
            Self { recovery, state: Cell::new(1), reads: Cell::new(0),
                known_keys, blamed: false }
        }
    }

    impl Memory for Halted {
        fn is_m5(&self) -> bool { false }
        fn recovery_state(&self) -> Result<u32> { Ok(self.state.get()) }
        fn host_recovery(&self) -> Result<u32> { Ok(0) }
        fn fault_sources(&self) -> Result<Sources> {
            assert_eq!(self.state.get(), 1);
            let mut sources = Sources::sample(3, |slot| {
                Some((if self.known_keys { ((38 + slot as u64) << 40) | 1 } else { 0 }, 0))
            });
            sources.reason = Some(2);
            Ok(sources)
        }
        fn transition(&self, expected: u32, next: u32) -> Result {
            assert_eq!(self.state.get(), expected);
            self.state.set(next);
            Ok(())
        }
        fn clear_timestamps(&self) -> Result { panic!("active render scheduler") }
        fn entry(&self, _index: usize) -> Result<(u32, u32)> {
            assert_eq!(self.state.get(), 3);
            Ok((0, 0))
        }
        fn wait_tick(&self) {
            assert_eq!(self.state.get(), 2);
            self.state.set(3);
        }
    }

    impl Queues for Halted {
        fn render_scheduler_active(&self) -> bool { true }
        fn render_pass_started(&self, qid: u8, _stamp: u64) -> Option<bool> {
            // This is the regression oracle: state 0/2/3 is too late. A
            // resumed peer may have started after the fault we are attributing.
            assert_eq!(self.state.get(), 1);
            self.reads.set(self.reads.get() + 1);
            Some(qid == 39)
        }
        fn publish_recovery_targets(&self) -> Option<Error> { None }
        fn oldest_spared(&self, _qid: u8) -> Option<bool> { None }
        fn visit_owners(&self, _qid: u8, _visit: &mut dyn FnMut(&Arc<VmStatus>)) {}
        fn classify_work(&mut self, _class: &dyn Fn(&Arc<VmStatus>) -> Class) -> u128 { 0 }
        fn note_interrupted(&mut self, _qid: u8, _stamp: u32) -> Interrupted { Interrupted::None }
        fn settle_blamed_render(&mut self, qid: u8, _error: Error) {
            assert_eq!(qid, 38);
            self.blamed = true;
        }
        fn settle_cancelled_renders(&mut self, _qids: u128) {}
        fn rescan_compute(&mut self) -> Result { Ok(()) }
        fn release_retired_compute(&mut self) {}
        fn announce_withheld(&mut self) {}
    }

    impl Host for Halted {
        fn recovery(&self) -> &State { &self.recovery }
        fn recovery_mut(&mut self) -> &mut State { &mut self.recovery }
    }

    #[test]
    fn pass_evidence_is_frozen_before_the_first_acknowledgement() {
        let mut host = Halted::new(true);
        service(&mut host, || false).unwrap();
        assert_eq!(host.reads.get(), 2);
        assert_eq!(host.state.get(), 0);
        assert_eq!(host.recovery.generation(), 1);
        assert!(!host.blamed);
    }

    /// Two VMs, each owning one render pair: `a` queues 40/41, `b` queues 60/61.
    struct Owners([(u8, Arc<VmStatus>); 4], [Option<Class>; 4]);

    impl Queues for Owners {
        fn render_scheduler_active(&self) -> bool { true }
        fn render_pass_started(&self, _qid: u8, _stamp: u64) -> Option<bool> { None }
        fn publish_recovery_targets(&self) -> Option<Error> { None }
        fn oldest_spared(&self, qid: u8) -> Option<bool> {
            self.0.iter().any(|(own, _)| *own == qid).then_some(false)
        }
        fn visit_owners(&self, qid: u8, visit: &mut dyn FnMut(&Arc<VmStatus>)) {
            self.0.iter().filter(|(own, _)| *own == qid).for_each(|(_, status)| visit(status));
        }
        fn classify_work(&mut self, class: &dyn Fn(&Arc<VmStatus>) -> Class) -> u128 {
            for (index, (_, status)) in self.0.iter().enumerate() {
                self.1[index] = Some(class(status));
            }
            0
        }
        fn note_interrupted(&mut self, _qid: u8, _stamp: u32) -> Interrupted { Interrupted::None }
        fn settle_blamed_render(&mut self, _qid: u8, _error: Error) {}
        fn settle_cancelled_renders(&mut self, _qids: u128) {}
        fn rescan_compute(&mut self) -> Result { Ok(()) }
        fn release_retired_compute(&mut self) {}
    }

    fn owners() -> Owners {
        let a = Arc::pin_init(VmStatus::new(), GFP_KERNEL).unwrap();
        let b = Arc::pin_init(VmStatus::new(), GFP_KERNEL).unwrap();
        Owners([(40, a.clone()), (41, a), (60, b.clone()), (61, b)], [None; 4])
    }

    fn leftover(queues: &mut Owners, exact_qid: u8) -> bool {
        let entries = Entries { exact_qids: 1u128 << exact_qid, ..Entries::default() };
        classify(queues, entries, Some(40), Attribution::Queues(3u128 << 60)).2
    }

    #[test]
    fn only_dropped_work_of_an_already_failed_vm_is_leftover() {
        let mut queues = owners();
        // A first failure counts toward the storm.
        assert!(!leftover(&mut queues, 60));
        queues.0[2].1.record(EIO);
        assert!(leftover(&mut queues, 60));
        // The restart dropped nothing of the failed VM.
        assert!(!leftover(&mut queues, 40));
    }

    #[test]
    fn a_guilty_vm_is_not_spared_by_another_vms_recovery() {
        let mut queues = owners();
        let entries = || Entries { exact_qids: 1u128 << 40, ..Entries::default() };
        // b failing for its own client does not make it guilty in a's recovery.
        queues.0[2].1.record(EIO);
        classify(&mut queues, entries(), Some(40), Attribution::Queues(3u128 << 40));
        assert_eq!(queues.1, [Some(Class::Guilty), Some(Class::Guilty), Some(Class::Spared), Some(Class::Spared)]);
        // Once a recovery finds b guilty, a later recovery that blames a keeps b guilty.
        classify(&mut queues, entries(), Some(60), Attribution::Queues(3u128 << 60));
        classify(&mut queues, entries(), Some(40), Attribution::Queues(3u128 << 40));
        assert_eq!(queues.1, [Some(Class::Guilty); 4]);
    }

    #[test]
    fn unknown_25g83_keys_still_preserve_firmware_blame() {
        let mut host = Halted::new(false);
        service(&mut host, || false).unwrap();
        assert_eq!(host.reads.get(), 0);
        assert!(host.blamed);
        assert_eq!(host.state.get(), 0);
    }
}
