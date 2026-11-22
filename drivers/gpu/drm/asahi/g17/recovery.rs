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
pub(crate) use attribution::{Record, Sources, PAGE_FAULT};
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
    /// QIDs with a valid record.
    queues: u128,
}

impl Entries {
    /// Counts a valid record. Returns false for an invalid one, which ends the walk on G17P
    /// and is skipped on M5 (see `read_entries`).
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
        self.queues |= bit;
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
    /// Optional diagnostics: logs the MMU fault banks while the firmware is halted.
    fn report_mmu_fault(&self) {}
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
    /// The restart found guilty VMs and is charged to them ([`VmStatus::charge_restart`]): each
    /// was found guilty for the first time, which costs its client the device, or still held
    /// work from then. It does not count toward the device's storm limit.
    pub(super) charged: bool,
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
    /// Like timeouts, restarts with work retired between them start a new window: each one
    /// still costs the guilty client its device, but only restarts without progress stop the GPU.
    /// Restarts charged to their guilty clients (`ServiceResult::charged`) are not counted.
    pub(super) fn restart_limit(&mut self, now: u64) -> bool {
        self.restarts.note_unless_progress(now, safety::retired())
    }
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
    sources.report(request.generation, super::queue::compute::retirement::stampless_total(),
        request.blamed);
    host.report_mmu_fault();
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
    report_clients(host, request.generation, &entries, request.blamed);

    let (listed, exact, fallback) = (entries.listed, entries.exact, entries.fallback);
    let (cancelled, blame, leftover, charged) =
        classify(host, entries, request.blamed, attribution);
    if listed != 0 && host.is_m5() {
        // The M5 table only classifies (read_entries): each command is retired from its own
        // queue's records, replayed when it never ran.
        pr_warn!(
            "G17: GPU recovery: {} commands were in flight; each is retired from its own queue's records\n",
            listed
        );
    } else if listed != 0 {
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
            sources.firmware_reason(),
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
    Ok(ServiceResult { scan_error, target_error, leftover, charged })
}

/// Names the client of each queue the recovery table lists, and of the blamed queue, before any
/// classification changes: every recovery then says whose work was in flight. Diagnostics only.
fn report_clients<Q: Queues>(queues: &Q, generation: u64, entries: &Entries, blamed: Option<u8>) {
    let named = entries.queues | blamed.map_or(0, |qid| 1u128 << qid);
    for qid in (0..QIDS).filter(|qid| named & (1u128 << qid) != 0) {
        let bit = 1u128 << qid;
        let mut first: Option<Arc<VmStatus>> = None;
        let (mut commands, mut others) = (0u32, 0u32);
        queues.visit_owners(qid, &mut |status| {
            commands += 1;
            match first.as_ref().map(|known| Arc::ptr_eq(known, status)) {
                None => first = Some(status.clone()),
                Some(false) => others += 1,
                Some(true) => {}
            }
        });
        let flags = (
            if blamed == Some(qid) { " blamed" } else { "" },
            if entries.guilty & bit != 0 { " guilty" } else { "" },
            if entries.cancelled & bit != 0 { " cancelled" } else { "" },
        );
        match first {
            Some(status) => {
                let (name, tgid) = status.client();
                let shared = if others != 0 { " and other clients" } else { "" };
                pr_warn!(
                    "G17: recovery gen {} queue {}{}{}{}: {} retained commands, client {}[{}]{}\n",
                    generation, qid, flags.0, flags.1, flags.2, commands, name, tgid, shared
                );
            }
            None => pr_warn!(
                "G17: recovery gen {} queue {}{}{}{}: no retained commands\n",
                generation, qid, flags.0, flags.1, flags.2
            ),
        }
    }
}

/// Walks the recovery table. G17P's is a packed list: records after its first invalid one are
/// left over from an earlier restart, so the walk ends there. M5's need not be packed, so all
/// of its records are walked and invalid ones skipped; they only classify the recovery, while
/// on G17P every listed command is marked interrupted. A failed read keeps what was read so far.
fn read_entries<H: Host>(host: &mut H) -> Entries {
    let mut entries = Entries::default();
    let m5 = host.is_m5();
    for index in 0..RECOVERY_RECORDS {
        let Ok((flags, stamp)) = host.entry(index) else {
            break;
        };
        if !entries.include(flags) {
            if m5 {
                continue;
            }
            break;
        }
        if m5 {
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
/// Then whether every guilty VM had failed before this restart and it dropped one of their
/// commands: a failed VM publishes nothing new, so such restarts end with its queued work.
/// Last, whether the restart is charged to the guilty VMs ([`VmStatus::charge_restart`], with
/// the commands each holds): one client's faults or hangs then cost only that client, however
/// many VMs it opens. A restart without a guilty VM, or one that keeps blaming a VM for more
/// than the work it held, is the device's.
fn classify<Q: Queues>(queues: &mut Q, entries: Entries, blamed: Option<u8>, attribution: Attribution) -> (u128, bool, bool, bool) {
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
        return (0, blame, false, false);
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
        return (0, blame, false, false);
    }
    let known = |status: &Arc<VmStatus>| guilty[..count].iter().flatten().any(|known| Arc::ptr_eq(known, status));
    let mut dropped = false;
    for qid in (0..QIDS).filter(|qid| entries.exact_qids & (1u128 << qid) != 0) {
        queues.visit_owners(qid, &mut |status| dropped |= known(status));
    }
    let leftover = count != 0 && dropped && guilty[..count].iter().flatten().all(|status| status.get() != 0);
    guilty[..count].iter().flatten().for_each(|status| status.note_recovery_guilty());
    // Commands each guilty VM holds, counted as they are classified.
    let held: [core::cell::Cell<u32>; MAX_GUILTY_VMS] = Default::default();
    let spared = queues.classify_work(&|status| {
        let known = guilty[..count].iter()
            .position(|known| known.as_ref().is_some_and(|known| Arc::ptr_eq(known, status)));
        match known {
            Some(index) => {
                held[index].set(held[index].get().saturating_add(1));
                Class::Guilty
            }
            // A VM an earlier recovery found guilty stays guilty. Spared, its leftover work would
            // make the next blame on it collateral, and that restart would count as a storm.
            None if status.recovery_guilty() => Class::Guilty,
            None => Class::Spared,
        }
    });
    // Every guilty VM is charged, even once one of them is over its count.
    let charged = count != 0 && guilty[..count].iter().zip(&held).fold(true, |charged, (status, held)| {
        status.as_ref().is_some_and(|status| status.charge_restart(held.get())) && charged
    });
    (cancelled & !(guilty_qids & !collateral) & spared, blame, leftover, charged)
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
    }

    impl Host for Halted {
        fn recovery(&self) -> &State { &self.recovery }
        fn recovery_mut(&mut self) -> &mut State { &mut self.recovery }
    }

    /// A halted recovery table holding `records` (index, flags); every other record is invalid.
    /// Records the QIDs whose commands the walk marks interrupted.
    struct Table {
        recovery: State,
        m5: bool,
        records: [(usize, u32); 5],
        marked: u128,
    }

    impl Table {
        fn new(m5: bool) -> Self {
            const VALID: u32 = 1;
            Self {
                recovery: State::new(),
                m5,
                records: [
                    (0, VALID | (12 << 4)),
                    (1, VALID | u32::from(GUILTY) | (17 << 4)),
                    // Index 2 is invalid: G17P's list ends here, and what follows is left over.
                    (3, VALID | u32::from(GUILTY | CANCELLED) | (40 << 4)),
                    (RECOVERY_RECORDS - 1, VALID | u32::from(GUILTY) | (41 << 4)),
                    (RECOVERY_RECORDS, VALID | u32::from(GUILTY) | (42 << 4)),
                ],
                marked: 0,
            }
        }
    }

    impl Memory for Table {
        fn is_m5(&self) -> bool { self.m5 }
        fn recovery_state(&self) -> Result<u32> { Ok(3) }
        fn host_recovery(&self) -> Result<u32> { Ok(0) }
        fn fault_sources(&self) -> Result<Sources> { Err(EIO) }
        fn transition(&self, _expected: u32, _next: u32) -> Result { Err(EIO) }
        fn clear_timestamps(&self) -> Result { Err(EIO) }
        fn entry(&self, index: usize) -> Result<(u32, u32)> {
            assert!(index < RECOVERY_RECORDS);
            let flags = self.records.iter().find(|(at, _)| *at == index).map_or(0, |(_, flags)| *flags);
            Ok((flags, index as u32))
        }
        fn wait_tick(&self) {}
    }

    impl Queues for Table {
        fn render_scheduler_active(&self) -> bool { true }
        fn render_pass_started(&self, _qid: u8, _stamp: u64) -> Option<bool> { None }
        fn publish_recovery_targets(&self) -> Option<Error> { None }
        fn oldest_spared(&self, _qid: u8) -> Option<bool> { None }
        fn visit_owners(&self, _qid: u8, _visit: &mut dyn FnMut(&Arc<VmStatus>)) {}
        fn classify_work(&mut self, _class: &dyn Fn(&Arc<VmStatus>) -> Class) -> u128 { 0 }
        fn note_interrupted(&mut self, qid: u8, _stamp: u32) -> Interrupted {
            self.marked |= 1u128 << qid;
            Interrupted::Exact
        }
        fn settle_blamed_render(&mut self, _qid: u8, _error: Error) {}
        fn settle_cancelled_renders(&mut self, _qids: u128) {}
        fn rescan_compute(&mut self) -> Result { Ok(()) }
        fn release_retired_compute(&mut self) {}
    }

    impl Host for Table {
        fn recovery(&self) -> &State { &self.recovery }
        fn recovery_mut(&mut self) -> &mut State { &mut self.recovery }
    }

    #[test]
    fn recovery_table_ends_at_its_first_invalid_record() {
        let mut host = Table::new(false);
        let entries = read_entries(&mut host);
        assert_eq!(entries.listed, 2);
        assert_eq!(entries.guilty, 1u128 << 17);
        assert_eq!(entries.cancelled, 0);
        assert_eq!(entries.exact, 2);
        assert_eq!(entries.exact_qids, (1u128 << 12) | (1u128 << 17));
        // Leftover records blame and cancel nothing and mark no bystander's command.
        assert_eq!(host.marked, (1u128 << 12) | (1u128 << 17));
    }

    #[test]
    fn m5_recovery_table_is_walked_past_invalid_records() {
        let mut host = Table::new(true);
        let entries = read_entries(&mut host);
        assert_eq!(entries.listed, 4);
        assert_eq!(entries.guilty, (1u128 << 17) | (1u128 << 40) | (1u128 << 41));
        assert_eq!(entries.cancelled, 1u128 << 40);
        // The M5 table only classifies.
        assert_eq!((entries.exact, entries.fallback, entries.exact_qids), (0, 0, 0));
        assert_eq!(host.marked, 0);
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
        // A VM's first failure is not leftover work.
        assert!(!leftover(&mut queues, 60));
        queues.0[2].1.record(EIO);
        assert!(leftover(&mut queues, 60));
        // The restart dropped nothing of the failed VM.
        assert!(!leftover(&mut queues, 40));
    }

    #[test]
    fn restarts_are_charged_to_their_guilty_clients_and_only_unblamed_ones_to_the_device() {
        let mut queues = owners();
        let charged = |queues: &mut Owners, attribution| {
            classify(queues, Entries::default(), None, attribution).3
        };
        // Nobody to blame: the device's.
        assert!(!charged(&mut queues, Attribution::NoCulprit));
        assert!(!charged(&mut queues, Attribution::Firmware));
        // Each VM pays for its first restart, then for one more per command it held then (two
        // each here).
        assert!(charged(&mut queues, Attribution::Queues(1u128 << 60)));
        for _ in 0..3 {
            assert!(charged(&mut queues, Attribution::Queues(1u128 << 40)));
        }
        // A firmware that keeps blaming a for work it no longer holds stops the GPU as before,
        // even with b, still within its count, blamed beside it.
        assert!(!charged(&mut queues, Attribution::Queues(1u128 << 40)));
        assert!(!charged(&mut queues, Attribution::Queues((1u128 << 40) | (1u128 << 60))));
        assert!(charged(&mut queues, Attribution::Queues(1u128 << 60)));
        assert!(!charged(&mut queues, Attribution::Queues(1u128 << 60)));
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
