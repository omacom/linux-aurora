// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Process-exit controls for closed contexts whose compute commands remain owned.

use super::{now_ns, DeferredBatch};
use crate::g17::{
    channel::Rings,
    context::Context,
    fw::channels::{ContextKill, ControlRecord},
    queue::compute::exit,
    recovery::Memory as _,
    timeout::LIVENESS_NS,
    Firmware, MSG_CONTROL_NOTIFY,
};
use core::sync::atomic::{fence, AtomicU32, AtomicU64, Ordering};
use kernel::{prelude::*, sync::Arc};

/// Waits of failed contexts' kills for a moment without other contexts' compute in flight,
/// by bucket (under 10 ms, 100 ms, 1 s, 5 s, and longer) and the longest.
static KILL_WAITS: [AtomicU32; 5] = [const { AtomicU32::new(0) }; 5];
static KILL_WAIT_MAX_NS: AtomicU64 = AtomicU64::new(0);
const KILL_WAIT_BOUNDS_NS: [u64; 4] = [10_000_000, 100_000_000, 1_000_000_000, 5_000_000_000];
/// A failed context's kill waiting this long is reported, once per kill and at most once per
/// this interval across kills.
const SLOW_KILL_WAIT_NS: u64 = 5_000_000_000;
static SLOW_KILL_WARNED_NS: AtomicU64 = AtomicU64::new(0);
/// Failed contexts' kills published beside other contexts' compute under GPU memory pressure,
/// and when the last one was: at most one per [`PRESSED_KILL_INTERVAL_NS`], so the memory of
/// one is freed before another is considered.
static PRESSED_KILLS: AtomicU32 = AtomicU32::new(0);
static PRESSED_KILL_NS: AtomicU64 = AtomicU64::new(0);
const PRESSED_KILL_INTERVAL_NS: u64 = 250_000_000;

/// Render-only kills (see `Firmware::publish_render_kill`): published, acknowledged, and
/// contexts refused (each counted once, by its first reason) because the scheduler page names
/// a queue outside the context, because the held pairs have no single owner, or because
/// admission could not hold them.
static RENDER_KILLS: [AtomicU32; 5] = [const { AtomicU32::new(0) }; 5];
const RENDER_KILL_PUBLISHED: usize = 0;
const RENDER_KILL_ACKNOWLEDGED: usize = 1;
const RENDER_KILL_SCOPE: usize = 2;
const RENDER_KILL_OWNER: usize = 3;
const RENDER_KILL_HOLD: usize = 4;
/// A render-only candidate is left this long after close, as a quarantined compute queue is,
/// so that ordinary retirement goes first.
const RENDER_KILL_DELAY_NS: u64 = 250_000_000;

/// The render queue a render-only kill names: one of the context's own render queues that
/// its scheduler page lists, never the reserved queues 0 and 1.
fn render_kill_target(listed: u128, own: u128) -> Option<u8> {
    let candidates = listed & own & !0b11;
    (candidates != 0).then(|| candidates.trailing_zeros() as u8)
}

/// A closed context may be killed on its render pairs' behalf once it has been closed for
/// [`RENDER_KILL_DELAY_NS`], it was never killed, a pair of it holds executed but unstamped
/// commands and no compute queue still runs it, switches away from it or killed it. A second kill
/// of a killed context made the firmware fault on the killed pair's own tiling queue within a
/// millisecond of acknowledging it: with the pair's release held back, a context killed through
/// its compute queue still looked like a render-only candidate.
fn render_kill_eligible(age: u64, killed: bool, executed: bool, compute_clear: bool) -> bool {
    age >= RENDER_KILL_DELAY_NS && !killed && executed && compute_clear
}

/// The one admission owner of every pair a kill holds, if there is one.
fn hold_owner(owners: impl Iterator<Item = Option<u64>>) -> Option<u64> {
    let mut owner = None;
    for slot_owner in owners {
        match (owner, slot_owner) {
            (_, None) | (_, Some(0)) => return None,
            (None, Some(next)) => owner = Some(next),
            (Some(current), Some(next)) if current != next => return None,
            _ => {}
        }
    }
    owner
}

fn refuse_render_kill(teardown: &mut crate::g17::teardown::Pending, index: usize, reason: usize) {
    if teardown.refuse_render_kill(index) {
        RENDER_KILLS[reason].fetch_add(1, Ordering::Relaxed);
    }
}

/// GPU memory is within an eighth of the device limit. A failed context holds its memory
/// until its kill is acknowledged, so kills waiting for other contexts' compute to pause can
/// push the device to its limit, and then every client's allocations fail: on Neo,
/// gnome-shell crashed on such an ENOMEM. A kill beside foreign compute risks one GPU
/// recovery instead.
fn memory_pressed() -> bool {
    let ledger = &crate::memacct::LEDGER;
    let limit = ledger.limit.load(Ordering::Relaxed);
    limit != 0
        && ledger
            .user_committed
            .load(Ordering::Relaxed)
            .saturating_add(ledger.kernel_resident.load(Ordering::Relaxed))
            > limit - limit / 8
}

fn record_kill_wait(waited: u64) {
    let bucket = KILL_WAIT_BOUNDS_NS.iter().position(|bound| waited < *bound).unwrap_or(4);
    KILL_WAITS[bucket].fetch_add(1, Ordering::Relaxed);
    KILL_WAIT_MAX_NS.fetch_max(waited, Ordering::Relaxed);
}

pub(in crate::g17) fn write_kill_waits(f: &mut kernel::fmt::Formatter<'_>, waiting: usize, longest: u64)
    -> kernel::fmt::Result
{
    let count = |bucket: usize| KILL_WAITS[bucket].load(Ordering::Relaxed);
    writeln!(f, "failed_kill_waits under_10ms={} under_100ms={} under_1s={} under_5s={} longer={} max_ms={} memory_pressed={}",
             count(0), count(1), count(2), count(3), count(4),
             KILL_WAIT_MAX_NS.load(Ordering::Relaxed) / 1_000_000, PRESSED_KILLS.load(Ordering::Relaxed))?;
    writeln!(f, "failed_kills waiting={} longest_wait_ms={}", waiting, longest / 1_000_000)
}

pub(in crate::g17) fn write_render_kills(f: &mut kernel::fmt::Formatter<'_>, waiting: usize, longest: u64)
    -> kernel::fmt::Result
{
    let count = |index: usize| RENDER_KILLS[index].load(Ordering::Relaxed);
    writeln!(f, "render_kills published={} acknowledged={} refused_scope={} refused_owner={} refused_hold={} waiting={} longest_wait_ms={}",
             count(RENDER_KILL_PUBLISHED), count(RENDER_KILL_ACKNOWLEDGED), count(RENDER_KILL_SCOPE),
             count(RENDER_KILL_OWNER), count(RENDER_KILL_HOLD), waiting, longest / 1_000_000)
}

impl super::Registry {
    /// ACK storage belongs to the already-retained transaction, so a full event
    /// batch cannot overflow a separate cache. No allocation or wait is needed.
    pub(super) fn acknowledge_compute_exit(&mut self, cookie: u64) {
        for entry in self.compute.iter_mut().flatten() {
            if let Some(mut queue) = entry.queue.as_deref_mut() {
                if queue.acknowledge_exit(cookie) {
                    return;
                }
            }
        }
        if self.teardown.acknowledge_render_kill(cookie) {
            RENDER_KILLS[RENDER_KILL_ACKNOWLEDGED].fetch_add(1, Ordering::Relaxed);
            crate::g17::trace::teardown(4, cookie, 0x100, 0);
        }
    }
}

impl Firmware {
    pub(in crate::g17) fn service_compute_exit(&mut self, deferred: &mut DeferredBatch) -> Result {
        let recovery_closed = self.init.recovery_state()? == 0;
        let port = &self.port;
        for entry in self.queues.compute.iter_mut().flatten() {
            if let Some(mut queue) = entry.queue.as_deref_mut() {
                queue.settle_exit(
                    recovery_closed,
                    &mut |owner| port.qos_complete(&[owner]),
                    &mut |event| deferred.push(event),
                )?;
            }
        }
        if self.recovery.pending() || !recovery_closed {
            return Ok(());
        }
        let now = now_ns();
        // Publication has already retained each kill. Retry only the doorbell;
        // failed attempts do not extend its bounded polling window.
        for entry in self.queues.compute.iter_mut().flatten() {
            if let Some(mut queue) = entry.queue.as_deref_mut() {
                let _ = queue.announce_exit(now, || self.primary.notify(MSG_CONTROL_NOTIFY));
            }
        }
        let primary = &mut self.primary;
        self.queues.teardown.announce_render_kills(|| primary.notify(MSG_CONTROL_NOTIFY));
        // Under memory pressure one kill skips the wait, at most one per interval.
        let last = PRESSED_KILL_NS.load(Ordering::Relaxed);
        let mut pressed =
            (last == 0 || now.saturating_sub(last) >= PRESSED_KILL_INTERVAL_NS) && memory_pressed();
        for index in 0..self.queues.teardown.len() {
            let Some((context, age)) = self.queues.teardown.closed(index, now) else {
                continue;
            };
            if !self.queues.render.teardown_idle(&context)? {
                self.queues.teardown.clear_kill_wait(index);
                continue;
            }
            // A failed context's executed pairs keep their pool leases until the kill is
            // acknowledged (`Pair::firmware_drained`), so its kill cannot wait for them.
            let result = if self.compute_failed(&context) || self.queues.render.teardown_pools_released(&context) {
                // Only a render-only kill's wait lives on the entry; a compute kill keeps its
                // own on the queue.
                self.queues.teardown.clear_kill_wait(index);
                self.publish_compute_exit(&context, age, now, &mut pressed)
            } else if self.render_kill_candidate(index, &context, age) {
                self.publish_render_kill(index, &context, now, &mut pressed)
            } else {
                self.queues.teardown.clear_kill_wait(index);
                Ok(())
            };
            if let Err(error) = result {
                dev_err!(
                    self.primary.state.shared.dev.as_ref(),
                    "Scheduler {} exit request failed: {:?}\n",
                    context.id(),
                    error
                );
            }
        }
        Ok(())
    }

    fn publish_compute_exit(&mut self, context: &Arc<Context>, age: u64, now: u64, pressed: &mut bool) -> Result {
        let Some(index) = self.queues.compute.iter().position(|entry| {
            entry
                .as_ref()
                .and_then(|entry| entry.queue.as_deref())
                .is_some_and(|queue| queue.exit_candidate(context, age))
        }) else {
            return Ok(());
        };
        let failed = self.compute_failed(context);
        if self.render_control_backpressured() {
            return Ok(());
        }
        let beside = failed && self.foreign_compute_running(context, now);
        if failed {
            if beside && !*pressed {
                let queue = self.queues.compute[index]
                    .as_mut()
                    .and_then(|entry| entry.queue.as_deref_mut());
                if let Some(mut queue) = queue {
                    let (waited, crossed) = queue.note_kill_wait(now, SLOW_KILL_WAIT_NS);
                    let last = SLOW_KILL_WARNED_NS.load(Ordering::Relaxed);
                    if crossed
                        && (last == 0 || now.saturating_sub(last) >= SLOW_KILL_WAIT_NS)
                        && SLOW_KILL_WARNED_NS
                            .compare_exchange(last, now.max(1), Ordering::Relaxed, Ordering::Relaxed)
                            .is_ok()
                    {
                        dev_warn!(
                            self.primary.state.shared.dev.as_ref(),
                            "Scheduler {} kill has waited {} ms for other contexts' compute to pause\n",
                            context.id(),
                            waited / 1_000_000
                        );
                    }
                }
                return Ok(());
            }
        } else if self.kill_unsafe(context, now) {
            return Ok(());
        }
        let (owner, target) = {
            let queue = self.queues.compute[index]
                .as_ref()
                .and_then(|entry| entry.queue.as_deref())
                .ok_or(EIO)?;
            let Some(owner) = queue.exit_owner(context) else {
                return Ok(());
            };
            (owner, queue.qid())
        };
        let (mut own, mut foreign, slots) = self.queues.render.exit_scope(context);
        for entry in self.queues.compute.iter().flatten() {
            if let Some(queue) = entry.queue.as_deref() {
                let bit = 1u128 << queue.qid();
                if queue.context().is_some_and(|bound| Arc::ptr_eq(bound, context)) {
                    own |= bit;
                } else {
                    foreign |= bit;
                }
            }
        }
        let state = exit::State::read(context)?;
        let Some(qids) = state.scope(target, own, foreign) else {
            return Ok(());
        };
        let mut hold = 0u64;
        for slot in 0..crate::hw::t8140::queues::RENDER_SLOTS as u8 {
            if slots & (1u64 << slot) != 0 && self.queues.render.qids_for_slot(slot) & qids != 0 {
                hold |= 1u64 << slot;
            }
        }
        let cookie = exit::next_cookie()?;
        if hold != 0
            && !self
                .queues
                .admission
                .hold_render_slots_for_exit_kill(owner, hold, self.queues.render.executed_slots(context) & hold)
        {
            return Ok(());
        }
        let record = ControlRecord::ContextKill(ContextKill::new(context.scheduler_va(), cookie));
        if let Err(error) = Rings::publish_control(&self.init, &record) {
            self.queues.admission.release_exit_kill_hold(hold);
            return if error == EAGAIN { Ok(()) } else { Err(error) };
        }
        // Publication is irrevocable even if its doorbell fails, so mark the context
        // killed before anything below can fail. Keep the killed graph until processor
        // stop, and the held render identities until this context's scheduler release
        // is consumed or processors stop.
        context.note_kill_published();
        let mut queue = self.queues.compute[index]
            .as_mut()
            .and_then(|entry| entry.queue.as_deref_mut())
            .ok_or(EIO)?;
        if failed {
            record_kill_wait(queue.take_kill_wait(now));
        }
        if beside {
            *pressed = false;
            PRESSED_KILLS.fetch_add(1, Ordering::Relaxed);
            PRESSED_KILL_NS.store(now.max(1), Ordering::Relaxed);
        }
        queue.record_exit(cookie, now);
        crate::g17::trace::teardown(3, cookie, u64::from(context.id()), hold | u64::from(beside));
        fence(Ordering::SeqCst);
        if let Err(error) = queue.announce_exit(now, || self.primary.notify(MSG_CONTROL_NOTIFY)) {
            dev_warn!(
                self.primary.state.shared.dev.as_ref(),
                "Scheduler {} exit announcement failed: {:?}\n",
                context.id(),
                error
            );
        }
        Ok(())
    }

    /// A closed context whose compute side is clean, with render pairs whose commands all ran
    /// but were never stamped (`Pair::executed_for_kill`). Its failed compute command was
    /// retired on the firmware's fault record, so no compute queue is left to carry a kill,
    /// yet those pairs are released only after one: without this the context, its pairs and
    /// its memory stay until the processors stop.
    fn render_kill_candidate(&self, index: usize, context: &Arc<Context>, age: u64) -> bool {
        render_kill_eligible(
            age,
            self.queues.teardown.render_kill_published(index) || context.kill_published(),
            self.queues.render.executed_slots(context) != 0,
            self.queues.compute.iter().flatten().filter_map(|entry| entry.queue.as_deref()).all(|queue| {
                queue.idle_for_context(context) && !queue.switching_from(context) && !queue.exit_published(context)
            }),
        )
    }

    /// Kill a render-only candidate (`render_kill_candidate`). The rules of a failed
    /// context's kill apply: the pairs are held for the kill and their memory is released only
    /// once the scheduler page reports it, the kill waits for a moment without other contexts'
    /// compute unless GPU memory is pressed, and it names only queues of this context.
    fn publish_render_kill(&mut self, index: usize, context: &Arc<Context>, now: u64, pressed: &mut bool) -> Result {
        if self.render_control_backpressured() {
            return Ok(());
        }
        let beside = self.foreign_compute_running(context, now);
        if beside && !*pressed {
            let (waited, crossed) = self.queues.teardown.note_kill_wait(index, now, SLOW_KILL_WAIT_NS);
            let last = SLOW_KILL_WARNED_NS.load(Ordering::Relaxed);
            if crossed
                && (last == 0 || now.saturating_sub(last) >= SLOW_KILL_WAIT_NS)
                && SLOW_KILL_WARNED_NS
                    .compare_exchange(last, now.max(1), Ordering::Relaxed, Ordering::Relaxed)
                    .is_ok()
            {
                dev_warn!(
                    self.primary.state.shared.dev.as_ref(),
                    "Scheduler {} kill has waited {} ms for other contexts' compute to pause\n",
                    context.id(),
                    waited / 1_000_000
                );
            }
            return Ok(());
        }
        let (mut own, mut foreign, slots) = self.queues.render.exit_scope(context);
        for entry in self.queues.compute.iter().flatten() {
            if let Some(queue) = entry.queue.as_deref() {
                let bit = 1u128 << queue.qid();
                if queue.context().is_some_and(|bound| Arc::ptr_eq(bound, context)) {
                    own |= bit;
                } else {
                    foreign |= bit;
                }
            }
        }
        let state = exit::State::read(context)?;
        let Some(qids) = render_kill_target(state.qids, own).and_then(|target| state.scope(target, own, foreign)) else {
            refuse_render_kill(&mut self.queues.teardown, index, RENDER_KILL_SCOPE);
            return Ok(());
        };
        let held = |slot: u8| slots & (1u64 << slot) != 0 && self.queues.render.qids_for_slot(slot) & qids != 0;
        let range = 0..crate::hw::t8140::queues::RENDER_SLOTS as u8;
        let hold = range.clone().filter(|slot| held(*slot)).fold(0u64, |hold, slot| hold | 1u64 << slot);
        let owner = hold_owner(range.filter(|slot| held(*slot)).map(|slot| self.queues.render.slot_owner(slot)));
        let Some(owner) = owner.filter(|_| hold != 0) else {
            refuse_render_kill(&mut self.queues.teardown, index, RENDER_KILL_OWNER);
            return Ok(());
        };
        // As for a failed context's kill: the cookie first, so that running out of cookies
        // leaves no admission hold behind.
        let cookie = exit::next_cookie()?;
        if !self
            .queues
            .admission
            .hold_render_slots_for_exit_kill(owner, hold, self.queues.render.executed_slots(context) & hold)
        {
            refuse_render_kill(&mut self.queues.teardown, index, RENDER_KILL_HOLD);
            return Ok(());
        }
        let record = ControlRecord::ContextKill(ContextKill::new(context.scheduler_va(), cookie));
        if let Err(error) = Rings::publish_control(&self.init, &record) {
            self.queues.admission.release_exit_kill_hold(hold);
            return if error == EAGAIN { Ok(()) } else { Err(error) };
        }
        // Publication is irrevocable, so mark the context killed before anything below
        // can fail. The held pairs stay held until this context's scheduler release is
        // consumed (`release_killed_slots` at detach).
        context.note_kill_published();
        self.queues.teardown.record_render_kill(index, cookie, now)?;
        record_kill_wait(self.queues.teardown.take_kill_wait(index, now));
        if beside {
            *pressed = false;
            PRESSED_KILLS.fetch_add(1, Ordering::Relaxed);
            PRESSED_KILL_NS.store(now.max(1), Ordering::Relaxed);
        }
        RENDER_KILLS[RENDER_KILL_PUBLISHED].fetch_add(1, Ordering::Relaxed);
        crate::g17::trace::teardown(7, cookie, u64::from(context.id()), hold | u64::from(beside));
        fence(Ordering::SeqCst);
        let primary = &mut self.primary;
        self.queues.teardown.announce_render_kills(|| primary.notify(MSG_CONTROL_NOTIFY));
        Ok(())
    }

    /// Another context's compute command is in flight. On Neo, kills of failed contexts beside
    /// other clients' compute occasionally made the firmware recover the GPU (twice in ten
    /// minutes of a five-lane load with 67 faults), even with nothing freed before the kill's
    /// acknowledgement; such a kill takes the next moment without foreign compute instead.
    /// Render work is not waited for. A GPU that retired nothing for [`LIVENESS_NS`] has
    /// stopped.
    fn foreign_compute_running(&self, context: &Arc<Context>, now: u64) -> bool {
        let stalled = crate::g17::recovery::since_retired(now).is_none_or(|since| since >= LIVENESS_NS);
        !stalled
            && self.queues.compute.iter().flatten().filter_map(|entry| entry.queue.as_deref()).any(|queue| {
                queue.in_flight() && !queue.context().is_some_and(|bound| Arc::ptr_eq(bound, context))
            })
    }

    /// A compute command of `context` failed and the firmware holds commands behind it that it
    /// never runs. `publish_compute_exit` is only reached once `teardown_idle` found all of
    /// the context's render pairs idle; the pairs behind such a failure become idle when
    /// their commands have executed (`Pair::firmware_drained`), so by then nothing of the
    /// context runs any more.
    fn compute_failed(&self, context: &Arc<Context>) -> bool {
        self.queues.compute.iter().flatten().filter_map(|entry| entry.queue.as_deref()).any(|queue| {
            queue.context().is_some_and(|bound| Arc::ptr_eq(bound, context)) && queue.fault_pending()
        })
    }

    /// A kill while other contexts' commands run makes the firmware restart the
    /// GPU, and repeated ones crash it; on an otherwise idle GPU it is safe. A
    /// closed context's commands left alone retire like any others once the GPU
    /// reaches them, so its queues stay quarantined and retained until then. Only
    /// a GPU that retired nothing for [`LIVENESS_NS`] has stopped, and then the
    /// kill proceeds. A context whose compute failed ([`Self::compute_failed`])
    /// is not left alone, since its held commands never retire: its kill waits
    /// only for other contexts' compute ([`Self::foreign_compute_running`]).
    fn kill_unsafe(&self, context: &Arc<Context>, now: u64) -> bool {
        let stalled = crate::g17::recovery::since_retired(now).is_none_or(|since| since >= LIVENESS_NS);
        if stalled {
            return false;
        }
        self.queues.render.foreign_in_flight(context)
            || self
                .queues
                .compute
                .iter()
                .flatten()
                .filter_map(|entry| entry.queue.as_deref())
                .any(|queue| {
                    queue.in_flight() && !queue.context().is_some_and(|bound| Arc::ptr_eq(bound, context))
                })
    }

    pub(in crate::g17) fn compute_exit_polling(&self, now: u64) -> bool {
        if self
            .queues
            .compute
            .iter()
            .flatten()
            .filter_map(|entry| entry.queue.as_deref())
            .any(|queue| queue.exit_polling(now))
        {
            return true;
        }
        if self.queues.teardown.render_kill_polling(now, exit::POLL_NS) {
            return true;
        }
        (0..self.queues.teardown.len()).any(|index| {
            self.queues
                .teardown
                .closed(index, now)
                .is_some_and(|(context, age)| {
                    age < exit::POLL_NS
                        && self
                            .queues
                            .compute
                            .iter()
                            .flatten()
                            .filter_map(|entry| entry.queue.as_deref())
                            .any(|queue| queue.exit_candidate(&context, u64::MAX))
                })
        })
    }
}

#[kernel::macros::kunit_tests(asahi_g17_render_kill_target)]
mod render_kill_target_tests {
    use super::{hold_owner, render_kill_eligible, render_kill_target, RENDER_KILL_DELAY_NS};

    #[test]
    fn a_killed_context_is_never_killed_again() {
        assert!(render_kill_eligible(RENDER_KILL_DELAY_NS, false, true, true));
        assert!(!render_kill_eligible(RENDER_KILL_DELAY_NS, true, true, true));
        assert!(!render_kill_eligible(RENDER_KILL_DELAY_NS - 1, false, true, true));
        assert!(!render_kill_eligible(RENDER_KILL_DELAY_NS, false, false, true));
        assert!(!render_kill_eligible(RENDER_KILL_DELAY_NS, false, true, false));
    }

    #[test]
    fn a_render_kill_holds_pairs_of_one_owner_only() {
        assert_eq!(hold_owner([Some(5), Some(5)].into_iter()), Some(5));
        assert_eq!(hold_owner([Some(5), Some(6)].into_iter()), None);
        assert_eq!(hold_owner([Some(5), None].into_iter()), None);
        assert_eq!(hold_owner([Some(0)].into_iter()), None);
        assert_eq!(hold_owner(core::iter::empty()), None);
    }

    #[test]
    fn a_render_kill_names_an_own_listed_queue_never_0_or_1() {
        // Listed by the scheduler page but not the context's own: no target.
        assert_eq!(render_kill_target(1 << 40, 1 << 41), None);
        // The reserved queues are never named.
        assert_eq!(render_kill_target(0b11, 0b11), None);
        assert_eq!(render_kill_target(0b111 | 1 << 40, 0b111 | 1 << 40), Some(2));
        assert_eq!(render_kill_target(1 << 40 | 1 << 90, 1 << 90), Some(90));
    }
}
