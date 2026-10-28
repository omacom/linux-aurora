// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Paired render retirement evidence and bounded ownership of in-flight work.
//! Queue progress and execution timestamps precede the completion stamps which
//! authorize releasing command memory. A signalled error is not that witness.

use crate::hw::t8140::queues;
use core::sync::atomic::{fence, Ordering};
use kernel::prelude::*;

const DEPTH: usize = queues::RENDER_DEPTH as usize;
const MAX_GROUP_ITEMS: u32 = 4;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct Cursors {
    pub(crate) done: u32,
    pub(crate) read: u32,
    pub(crate) write: u32,
}

impl Cursors {
    fn valid(self) -> bool {
        self.done <= self.write && self.read <= self.write
    }

    fn idle(self) -> bool {
        self.done == self.read && self.read == self.write
    }
}

/// Each engine may omit its configuration update independently. Preserve both
/// exact windows; neither target can be reconstructed from a render ordinal.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct Window {
    base: u32,
    count: u32,
    capacity: u32,
}

impl Window {
    pub(crate) fn new(base: u32, count: u32, capacity: u32) -> Result<Self> {
        if base >= capacity || !(2..=MAX_GROUP_ITEMS).contains(&count) || count >= capacity {
            return Err(EINVAL);
        }
        Ok(Self {
            base,
            count,
            capacity,
        })
    }

    pub(crate) fn target(self) -> u32 {
        ((u64::from(self.base) + u64::from(self.count)) % u64::from(self.capacity)) as u32
    }

    fn normalize(self, cursors: Cursors) -> Result<Cursors> {
        // Earlier and later groups can coexist in a snapshot. Anchor behind
        // this group's base so a backwards observation cannot look like wrap.
        let back = MAX_GROUP_ITEMS * (DEPTH as u32 - 1);
        let limit = MAX_GROUP_ITEMS * DEPTH as u32 + back;
        if limit >= self.capacity {
            return Err(EINVAL);
        }
        let base = (u64::from(self.base) + u64::from(self.capacity) - u64::from(back))
            % u64::from(self.capacity);
        let distance = |value: u32| -> Result<u32> {
            if value >= self.capacity {
                return Err(EIO);
            }
            let distance =
                (u64::from(value) + u64::from(self.capacity) - base) % u64::from(self.capacity);
            if distance > u64::from(limit) {
                return Err(EIO);
            }
            Ok(distance as u32)
        };
        Ok(Cursors {
            done: distance(cursors.done)?,
            read: distance(cursors.read)?,
            write: distance(cursors.write)?,
        })
    }

    fn progress(self, previous: Cursors, current: Cursors) -> Result<(bool, bool, bool)> {
        let previous = self.normalize(previous)?;
        let current = self.normalize(current)?;
        let prefix = MAX_GROUP_ITEMS * (DEPTH as u32 - 1) + self.count;
        if !previous.valid()
            || !current.valid()
            || current.done < previous.done
            || current.read < previous.read
            || current.write < previous.write
            || prefix > current.write
        {
            return Err(EIO);
        }
        Ok((
            current.read >= prefix,
            current.done >= prefix,
            current.idle(),
        ))
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct Ticket {
    /// Full physical-pair ordinal, retained through owner switches and slot wrap.
    pub(crate) ordinal: u64,
    pub(crate) windows: [Window; 2],
    pub(crate) previous: [Cursors; 2],
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct Snapshot {
    pub(crate) cursors: [Cursors; 2],
    pub(crate) job_list_empty: bool,
    /// The tiler writes the start and fragment processing writes the end.
    pub(crate) pass: [u64; 2],
}

impl Snapshot {
    fn executed(self) -> bool {
        self.pass[0] != 0 && self.pass[1] != 0
    }

    /// Neither the tiler start nor the fragment end was written: nothing ran.
    pub(crate) fn untouched(self) -> bool {
        self.pass == [0, 0]
    }

    /// The command ran (tiler start and fragment end written) and the firmware consumed both
    /// queues' ring entries.
    pub(crate) fn executed_and_consumed(self) -> bool {
        self.executed() && self.cursors.iter().all(|cursors| cursors.done == cursors.write)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct Observation {
    pub(crate) accepted: bool,
    pub(crate) retired: bool,
    pub(crate) complete: bool,
    pub(crate) idle: bool,
}

pub(crate) fn observe(ticket: &Ticket, snapshot: Snapshot) -> Result<Observation> {
    let ta = ticket.windows[0].progress(ticket.previous[0], snapshot.cursors[0])?;
    let fragment = ticket.windows[1].progress(ticket.previous[1], snapshot.cursors[1])?;
    let retired = ta.1 && fragment.1;
    Ok(Observation {
        accepted: ta.0 && fragment.0,
        retired,
        complete: retired && snapshot.executed(),
        idle: ta.2 && fragment.2 && snapshot.job_list_empty,
    })
}

/// How long a front group the firmware executed, and whose ring entries it consumed, waits for
/// its completion stamp before it retires on its pass record.
///
/// The firmware stores both queues' stamps after the cache maintenance that follows a pass, so
/// the stamps, not the pass record, normally prove completion. Occasionally the stamps of a pair
/// stop one kick short while the pass record shows the group (and later groups) executed, and
/// no later event moves them. Without a bound the client waits for the scheduler watchdog and
/// its renewals, then loses its device. Compute applies the same bound to its records.
pub(crate) const STAMP_FALLBACK_NS: u64 = 50_000_000;
const _: () = assert!(STAMP_FALLBACK_NS * 4 <= crate::g17::job::TIMEOUT_MS as u64 * 1_000_000);

static STAMPLESS: core::sync::atomic::AtomicU64 = core::sync::atomic::AtomicU64::new(0);

/// Counts a render group that retired on its pass record because its completion stamps never
/// covered it; logs the first and every 64th.
pub(crate) fn note_stampless(slot: u16, ordinal: u64) {
    let count = STAMPLESS.fetch_add(1, Ordering::Relaxed) + 1;
    if count == 1 || count % 64 == 0 {
        pr_info!(
            "G17: lifetime total {} render groups retired on their pass record after {} ms without completion stamps (latest: slot {} ordinal {})\n",
            count,
            STAMP_FALLBACK_NS / 1_000_000,
            slot,
            ordinal
        );
    }
}

/// Retire an executed, consumed front group without its stamps, once [`STAMP_FALLBACK_NS`]
/// has passed. The caller decides that bound; this re-checks execution and consumption.
pub(crate) fn poll_stampless(ticket: &mut Ticket, memory: &mut impl Memory) -> Result<Poll> {
    fence(Ordering::Acquire);
    let snapshot = memory.snapshot(ticket.ordinal)?;
    let Ok(observation) = observe(ticket, snapshot) else {
        return Ok(Poll::Retry);
    };
    if !observation.complete {
        return Ok(Poll::Pending);
    }
    let [start, end] = memory.pass(ticket.ordinal)?;
    Ok(Poll::Complete([start, start, start, end]))
}

/// Outstanding publications span less than half the 24-bit stamp sequence.
/// A zero stamp is valid when that sequence wraps.
pub(crate) const fn stamp_covers(stamp: u32, published: u64) -> bool {
    let delta = stamp.wrapping_sub((published as u32).wrapping_shl(8));
    delta & 0xff == 0 && delta < 0x8000_0000
}

fn visible(ticket: &Ticket, stamps: [u32; 2]) -> Result<bool> {
    let published = ticket.ordinal.checked_add(1).ok_or(EOVERFLOW)?;
    Ok(stamps
        .into_iter()
        .all(|stamp| stamp_covers(stamp, published)))
}

/// Reads the graph owned by one physical pair. The owner validates the full
/// ordinal before reading reused descriptor/timestamp slots. Word accessors
/// perform aligned atomic loads; the observer supplies the acquire barriers.
pub(crate) trait Memory {
    fn snapshot(&mut self, ordinal: u64) -> Result<Snapshot>;
    fn stamps(&mut self) -> Result<[u32; 2]>;
    fn pass(&mut self, ordinal: u64) -> Result<[u64; 2]>;
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Poll {
    Retry,
    Pending,
    Complete([u64; 4]),
}

/// Observe the ordered front. Individually sampled cursor words can straddle
/// a firmware update, so an invalid snapshot receives one immediate reread.
/// Shared work-state membership can include other engines and cannot gate a
/// successful individual member's completion.
pub(crate) fn poll(ticket: &mut Ticket, memory: &mut impl Memory) -> Result<Poll> {
    fence(Ordering::Acquire);
    let mut snapshot = memory.snapshot(ticket.ordinal)?;
    let observation = match observe(ticket, snapshot) {
        Ok(observation) => observation,
        Err(_) => {
            fence(Ordering::Acquire);
            snapshot = memory.snapshot(ticket.ordinal)?;
            match observe(ticket, snapshot) {
                Ok(observation) => observation,
                Err(_) => return Ok(Poll::Retry),
            }
        }
    };
    complete_poll(ticket, memory, snapshot, observation)
}

/// Finish an observation sampled while its exact pair was exclusively borrowed.
/// The caller restores that pair and validates its epoch and front ticket first.
/// Completion stamps and the final timestamp refresh remain commit-time reads.
pub(crate) fn poll_observed(
    ticket: &mut Ticket,
    memory: &mut impl Memory,
    snapshot: Snapshot,
) -> Result<Poll> {
    match observe(ticket, snapshot) {
        Ok(observation) => complete_poll(ticket, memory, snapshot, observation),
        // The borrowed snapshot can straddle a firmware cursor update like any other sample:
        // take the ordinary poll's fresh snapshot and reread instead of failing the client.
        Err(_) => poll(ticket, memory),
    }
}

fn complete_poll(
    ticket: &mut Ticket,
    memory: &mut impl Memory,
    snapshot: Snapshot,
    observation: Observation,
) -> Result<Poll> {
    if observation.complete {
        if !visible(ticket, memory.stamps()?)? {
            return Ok(Poll::Pending);
        }
        // Refresh this generation's pass after observing the post-maintenance
        // stamps. The pre-stamp snapshot alone cannot authorize export.
        fence(Ordering::Acquire);
        let [start, end] = memory.pass(ticket.ordinal)?;
        return Ok(Poll::Complete([start, start, start, end]));
    }
    ticket.previous = snapshot.cursors;
    Ok(Poll::Pending)
}

/// Strict timeout settlement accepts only a nonexecuted, retired and unlinked
/// group. Drain reclamation may also settle executed groups, but retains both
/// ownership proofs and the exact post-maintenance stamps in either case.
pub(crate) fn abandoned(
    ticket: &Ticket,
    snapshot: Snapshot,
    stamps: [u32; 2],
    recovery_state: u32,
    accept_executed: bool,
) -> Result<bool> {
    let completion = observe(ticket, snapshot)?;
    if !completion.retired || !snapshot.job_list_empty || (snapshot.executed() && !accept_executed)
    {
        return Ok(false);
    }
    Ok(visible(ticket, stamps)? && recovery_state == 0)
}

/// Pair-wide proof, checked before and after servicing free-list events and
/// again after retiring all trackers. A proof from before event service must
/// not authorize releasing guards after a new recovery began.
pub(crate) fn drained(
    published: u64,
    stamps: [u32; 2],
    free_list: [u64; 2],
    recovery_state: u32,
) -> bool {
    published == 0
        || (stamps
            .into_iter()
            .all(|stamp| stamp_covers(stamp, published))
            && free_list[0] == free_list[1]
            && recovery_state == 0)
}

/// Fixed storage keeps admission, rollback and retirement allocation-free.
/// Each element retains its packet and resource leases independently of fences.
pub(crate) struct Tracker<T> {
    entries: [Option<T>; DEPTH],
    head: usize,
    len: usize,
}

impl<T> Tracker<T> {
    pub(crate) const fn new() -> Self {
        Self {
            entries: [const { None }; DEPTH],
            head: 0,
            len: 0,
        }
    }

    pub(crate) fn len(&self) -> usize {
        self.len
    }
    pub(crate) fn room(&self) -> bool {
        self.len < DEPTH
    }
    pub(crate) fn front(&self) -> Option<&T> {
        if self.len == 0 {
            None
        } else {
            self.entries[self.head].as_ref()
        }
    }
    pub(crate) fn front_mut(&mut self) -> Option<&mut T> {
        if self.len == 0 {
            None
        } else {
            self.entries[self.head].as_mut()
        }
    }
    pub(crate) fn back(&self) -> Option<&T> {
        if self.len == 0 {
            None
        } else {
            self.entries[(self.head + self.len - 1) % DEPTH].as_ref()
        }
    }
    pub(crate) fn back_mut(&mut self) -> Option<&mut T> {
        if self.len == 0 {
            None
        } else {
            self.entries[(self.head + self.len - 1) % DEPTH].as_mut()
        }
    }
    pub(crate) fn push(&mut self, value: T) -> core::result::Result<(), T> {
        if !self.room() {
            return Err(value);
        }
        self.entries[(self.head + self.len) % DEPTH] = Some(value);
        self.len += 1;
        Ok(())
    }
    pub(crate) fn pop(&mut self) -> Option<T> {
        if self.len == 0 {
            return None;
        }
        let value = self.entries[self.head].take();
        self.head = (self.head + 1) % DEPTH;
        self.len -= 1;
        value
    }

    pub(crate) fn iter(&self) -> impl Iterator<Item = &T> {
        Iterator::chain(
            self.entries[self.head..].iter(),
            self.entries[..self.head].iter(),
        )
        .take(self.len)
        .filter_map(Option::as_ref)
    }
    pub(crate) fn iter_mut(&mut self) -> impl Iterator<Item = &mut T> {
        let (before, after) = self.entries.split_at_mut(self.head);
        Iterator::chain(after.iter_mut(), before.iter_mut())
            .take(self.len)
            .filter_map(Option::as_mut)
    }
}

#[kunit_tests(asahi_g17_render_retirement)]
mod executed_tests {
    use super::*;

    fn snapshot(done: u32, write: u32, pass: [u64; 2]) -> Snapshot {
        Snapshot {
            cursors: [Cursors { done, read: done, write }; 2],
            job_list_empty: false,
            pass,
        }
    }

    #[test]
    fn executed_and_consumed_needs_both_passes_and_consumed_rings() {
        assert!(snapshot(3, 3, [1, 2]).executed_and_consumed());
        assert!(!snapshot(3, 3, [0, 0]).executed_and_consumed());
        assert!(!snapshot(3, 3, [1, 0]).executed_and_consumed());
        assert!(!snapshot(3, 3, [0, 2]).executed_and_consumed());
        assert!(!snapshot(2, 3, [1, 2]).executed_and_consumed());
    }

    /// The firmware state of a stalled pair as read on hardware: the front group executed and
    /// its two ring entries per queue were consumed, but both stamps hold the previous kick.
    struct Stalled {
        snapshot: Snapshot,
        stamps: [u32; 2],
    }
    impl Memory for Stalled {
        fn snapshot(&mut self, _ordinal: u64) -> Result<Snapshot> {
            Ok(self.snapshot)
        }
        fn stamps(&mut self) -> Result<[u32; 2]> {
            Ok(self.stamps)
        }
        fn pass(&mut self, _ordinal: u64) -> Result<[u64; 2]> {
            Ok(self.snapshot.pass)
        }
    }

    fn stalled_ticket() -> Ticket {
        let window = Window::new(1097, 2, 1280).unwrap();
        let previous = Cursors { done: 1097, read: 1097, write: 1097 };
        Ticket { ordinal: 406, windows: [window; 2], previous: [previous; 2] }
    }

    fn stalled(pass: [u64; 2]) -> Stalled {
        let cursors = Cursors { done: 1099, read: 1099, write: 1099 };
        Stalled {
            snapshot: Snapshot { cursors: [cursors; 2], job_list_empty: false, pass },
            stamps: [406 << 8; 2],
        }
    }

    #[test]
    fn lagging_stamps_keep_an_executed_group_pending() {
        let mut ticket = stalled_ticket();
        let mut memory = stalled([10, 20]);
        assert!(matches!(poll(&mut ticket, &mut memory), Ok(Poll::Pending)));
    }

    #[test]
    fn stampless_poll_retires_an_executed_consumed_group() {
        let mut ticket = stalled_ticket();
        let mut memory = stalled([10, 20]);
        assert!(matches!(poll_stampless(&mut ticket, &mut memory), Ok(Poll::Complete([10, 10, 10, 20]))));
    }

    #[test]
    fn stampless_poll_waits_for_the_fragment_end() {
        let mut ticket = stalled_ticket();
        let mut memory = stalled([10, 0]);
        assert!(matches!(poll_stampless(&mut ticket, &mut memory), Ok(Poll::Pending)));
    }
}
