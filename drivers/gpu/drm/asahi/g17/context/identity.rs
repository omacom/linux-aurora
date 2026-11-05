// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Context-local work-state identities. The low six bits address a node;
//! the complete sequence distinguishes its owner until exact retirement.

use core::sync::atomic::{AtomicU32, Ordering};
use kernel::prelude::*;

pub(super) const FREE: u32 = u32::MAX;
pub(super) const STATE_COUNT: usize = 63;
pub(super) const NODE_STRIDE: usize = 0x100;

/// Engines whose admissions take work-state nodes: render (0) and compute (1).
pub(super) const ENGINES: usize = 2;

/// Whether an engine that holds `held` nodes may take one of `free` nodes.
///
/// All engines of one ioctl share its node, and the node returns only once every command of
/// the ioctl has retired. So render admission can keep taking nodes for newer ioctls while
/// their compute commands still wait in the compute queue behind an older ioctl that has no
/// node yet; once render held every node, nothing could retire and both queues stalled. An
/// engine therefore never takes the last free node while it already holds one. The other
/// engine can then always take a node, and an engine refused here waits only for nodes it
/// took itself, for ioctls ahead of its waiting command. Each engine admits a context's
/// ioctls in submission order, so those waits cannot form a cycle.
pub(super) const fn may_take(free: usize, held: u32) -> bool {
    free > 1 || (free == 1 && held == 0)
}

pub(super) struct WorkStates {
    next: AtomicU32,
    live: [AtomicU32; 64],
    /// Nodes taken by each engine's admissions and not yet released.
    held: [AtomicU32; ENGINES],
}
impl WorkStates {
    pub(super) const fn new() -> Self {
        Self {
            next: AtomicU32::new(0),
            live: [const { AtomicU32::new(FREE) }; 64],
            held: [const { AtomicU32::new(0) }; ENGINES],
        }
    }
    /// Takes a node for `engine`, or none when [`may_take`] refuses. Callers serialize
    /// allocations of one context; releases may run concurrently.
    pub(super) fn allocate(&self, engine: usize) -> Option<u32> {
        let held = self.held.get(engine)?;
        if !may_take(STATE_COUNT.saturating_sub(self.used()), held.load(Ordering::Acquire)) {
            return None;
        }
        for _ in 0..64 {
            let word = self.next.fetch_add(1, Ordering::Relaxed);
            let index = (word & 63) as usize;
            if index != STATE_COUNT
                && self.live[index]
                    .compare_exchange(FREE, word, Ordering::AcqRel, Ordering::Acquire)
                    .is_ok()
            {
                held.fetch_add(1, Ordering::AcqRel);
                return Some(word);
            }
        }
        None
    }
    pub(super) fn release(&self, word: u32, engine: usize) -> bool {
        let index = (word & 63) as usize;
        let released = index != STATE_COUNT
            && self.live[index]
                .compare_exchange(word, FREE, Ordering::AcqRel, Ordering::Acquire)
                .is_ok();
        if released {
            if let Some(held) = self.held.get(engine) {
                held.fetch_sub(1, Ordering::AcqRel);
            }
        }
        released
    }
}

/// Ordinals past the turn remembered for ioctls that took a node or left early.
pub(super) const ORDER_WINDOW: u64 = 1024;

/// How far past the oldest waiting ioctl an ioctl may take its node.
pub(super) const RUN_AHEAD: u64 = 8;

/// The order in which a context's ioctls take work-state nodes.
///
/// Each engine admits a context's ioctls in submission order, but the engines advance
/// independently: render admission can take nodes for ioctls far ahead of an older ioctl still
/// waiting for its turn on the compute queue. Under deep mixed load (128 compute and 64 render
/// ioctls in flight on one queue) the firmware then stops completing all of the context's work
/// within seconds. Native macOS schedules a queue's command buffers in commit order across
/// engines, at most 36 between scheduling and completion, and the same load runs there without
/// failure.
///
/// So an ioctl takes its node only while at most [`RUN_AHEAD`] older ioctls of its context have
/// neither taken one nor left without one. Native scheduling happens at commit, on one thread;
/// here each engine admits from its own scheduler, and a strictly ordered turn costs a scheduler
/// round trip whenever the other engine's ioctl is ready first. A short run-ahead avoids that
/// without letting one engine get far ahead of the other.
pub(super) struct Order {
    /// The next ordinal to hand out.
    next: u64,
    /// The oldest ordinal that has neither taken a node nor left.
    turn: u64,
    /// Ordinals past the turn that took a node or left, as bits at ordinal % ORDER_WINDOW.
    done: [u64; (ORDER_WINDOW / 64) as usize],
    /// Set once an ioctl left too far past the turn to remember; admission is unordered after.
    unordered: bool,
    /// An ioctl was refused since the turn last moved: its publisher waits for the next move.
    refused: bool,
}
impl Order {
    pub(super) const fn new() -> Self {
        Self {
            next: 0,
            turn: 0,
            done: [0; (ORDER_WINDOW / 64) as usize],
            unordered: false,
            refused: false,
        }
    }
    /// The ordinal of a new ioctl, in submission order.
    pub(super) fn assign(&mut self) -> u64 {
        let ordinal = self.next;
        self.next += 1;
        ordinal
    }
    /// Whether the ioctl `ordinal` may take its node now. A refusal is remembered until the
    /// turn next moves, so that move wakes the refused publisher.
    pub(super) fn admits(&mut self, ordinal: u64) -> bool {
        let admits = self.unordered || ordinal <= self.turn + RUN_AHEAD;
        self.refused |= !admits;
        admits
    }
    /// The ioctl `ordinal` took its node. Returns whether a refused ioctl may now be admitted.
    pub(super) fn admit(&mut self, ordinal: u64) -> bool {
        self.finish(ordinal)
    }
    /// The ioctl `ordinal` left without taking a node (it failed or was cancelled first).
    /// Returns whether a refused ioctl may now be admitted.
    pub(super) fn leave(&mut self, ordinal: u64) -> bool {
        self.finish(ordinal)
    }
    fn finish(&mut self, ordinal: u64) -> bool {
        if self.unordered || ordinal < self.turn {
            return false;
        }
        if ordinal == self.turn {
            self.pass();
            return core::mem::take(&mut self.refused);
        }
        if ordinal - self.turn >= ORDER_WINDOW {
            self.unordered = true;
            return core::mem::take(&mut self.refused);
        }
        let bit = ordinal % ORDER_WINDOW;
        self.done[(bit / 64) as usize] |= 1 << (bit % 64);
        false
    }
    /// Moves the turn past the current ordinal and every following one already done.
    fn pass(&mut self) {
        loop {
            self.turn += 1;
            let bit = self.turn % ORDER_WINDOW;
            let (word, mask) = ((bit / 64) as usize, 1u64 << (bit % 64));
            if self.done[word] & mask == 0 {
                break;
            }
            self.done[word] &= !mask;
        }
    }
}

/// Two stable logical ordinals (render channels), independent of physical pair numbering. A
/// context's render work can move between physical pairs (sibling pairs, lending, relocation,
/// QID recycling), so a channel is rebound to a new pair once every publication this context
/// made through it has retired. A context's channel counts its passes up for the whole
/// process, so a rebound channel continues its pass sequence.
pub(super) struct RenderSlots {
    pairs: [Option<u8>; 2],
    origins: [Option<u64>; 2],
    /// Pass of the channel's last publication on its previous pair.
    bases: [u8; 2],
    /// QIDs and visibility target (ordinal + 1) of the last publication made through each
    /// channel.
    last: [Option<([u8; 2], u64)>; 2],
    /// Recency of each channel's last publication.
    used: [u64; 2],
    epoch: u64,
}
impl RenderSlots {
    pub(super) const fn new() -> Self {
        Self {
            pairs: [None; 2],
            origins: [None; 2],
            bases: [0; 2],
            last: [None; 2],
            used: [0; 2],
            epoch: 0,
        }
    }
    fn get(&mut self, pair: u8) -> Option<u32> {
        if let Some(index) = self.pairs.iter().position(|value| *value == Some(pair)) {
            return Some(index as u32);
        }
        let index = self.pairs.iter().position(Option::is_none)?;
        self.pairs[index] = Some(pair);
        Some(index as u32)
    }
    /// The channel of `pair`, rebinding the least recently used channel whose publications
    /// have all retired (`retired(qids, ordinal)`) when both are bound to other pairs. `None`:
    /// no channel can move yet.
    pub(super) fn bind(
        &mut self,
        pair: u8,
        retired: impl Fn([u8; 2], u64) -> bool,
    ) -> Option<(u32, Option<u8>)> {
        if let Some(index) = self.get(pair) {
            return Some((index, None));
        }
        let index = (0..2)
            .filter(|index| self.last[*index].is_none_or(|(qids, ordinal)| retired(qids, ordinal)))
            .min_by_key(|index| self.used[*index])?;
        let previous = self.pairs[index];
        // `last` holds the ordinal after the last publication (its visibility target), so that
        // publication's pass is base + (next - origin).
        if let (Some(origin), Some((_, next))) = (self.origins[index], self.last[index]) {
            self.bases[index] = self.bases[index].wrapping_add(next.wrapping_sub(origin) as u8);
        }
        self.pairs[index] = Some(pair);
        self.origins[index] = None;
        self.last[index] = None;
        Some((index as u32, previous))
    }
    pub(super) fn payload(&mut self, pair: u8, ordinal: u64) -> Option<u8> {
        let index = self.pairs.iter().position(|value| *value == Some(pair))?;
        let origin = *self.origins[index].get_or_insert(ordinal);
        Some(self.bases[index].wrapping_add(ordinal.checked_sub(origin)?.wrapping_add(1) as u8))
    }
    /// Records a publication that reached firmware through `pair`'s channel.
    pub(super) fn published(&mut self, pair: u8, qids: [u8; 2], ordinal: u64) -> bool {
        let Some(index) = self.pairs.iter().position(|value| *value == Some(pair)) else {
            return false;
        };
        self.epoch = self.epoch.wrapping_add(1);
        self.used[index] = self.epoch;
        self.last[index] = Some((qids, ordinal));
        true
    }
}

impl WorkStates {
    /// The oldest node in use that the firmware lists ahead of `word`: a live word older than
    /// it by the signed 32-bit difference the firmware sorts by. Its ioctl was submitted
    /// earlier ([`Order`]), and until that node drains the firmware withholds every stamp of
    /// `word`'s node behind it.
    pub(super) fn older_live(&self, word: u32) -> Option<u32> {
        self.live
            .iter()
            .map(|live| live.load(Ordering::Acquire))
            .filter(|&live| live != FREE && (live.wrapping_sub(word) as i32) < 0)
            .min_by_key(|&live| live.wrapping_sub(word) as i32)
    }

    pub(super) fn used(&self) -> usize {
        self.live.iter().filter(|word| word.load(Ordering::Acquire) != FREE).count()
    }
}
impl RenderSlots {
    pub(super) fn used(&self) -> usize { self.pairs.iter().flatten().count() }
}

#[kunit_tests(asahi_g17_work_states)]
mod tests {
    use super::*;

    #[test]
    fn last_node_stays_with_an_engine_holding_none() {
        assert!(!may_take(0, 0));
        assert!(may_take(1, 0));
        assert!(!may_take(1, 3));
        assert!(may_take(2, 3));
    }

    #[test]
    fn render_cannot_starve_compute_of_a_node() {
        let states = WorkStates::new();
        let mut render = KVec::new();
        while let Some(word) = states.allocate(0) {
            render.push(word, GFP_KERNEL).unwrap();
        }
        // Render stops one short; compute takes the last node, then nothing is left.
        assert_eq!(render.len(), STATE_COUNT - 1);
        let compute = states.allocate(1).unwrap();
        assert_eq!(states.used(), STATE_COUNT);
        assert!(states.allocate(0).is_none());
        assert!(states.allocate(1).is_none());
        // A lone free node goes to the engine holding none.
        assert!(states.release(compute, 1));
        assert!(states.allocate(0).is_none());
        let compute = states.allocate(1).unwrap();
        // With both engines holding nodes, a lone free node waits for one of their returns.
        assert!(states.release(render.pop().unwrap(), 0));
        assert!(states.allocate(0).is_none());
        assert!(states.allocate(1).is_none());
        // Two free nodes: render may take one again, never the last.
        assert!(states.release(render.pop().unwrap(), 0));
        assert!(states.allocate(0).is_some());
        assert!(states.allocate(0).is_none());
        assert!(states.release(compute, 1));
        assert!(states.allocate(1).is_some());
    }

    #[test]
    fn ioctls_take_nodes_at_most_run_ahead_past_the_oldest_waiting_one() {
        let mut order = Order::new();
        let ordinals: [u64; RUN_AHEAD as usize + 3] = core::array::from_fn(|_| order.assign());
        let (oldest, last) = (ordinals[0], ordinals[RUN_AHEAD as usize + 2]);
        // Every ioctl up to RUN_AHEAD past the oldest waiting one may take its node.
        for &ordinal in &ordinals[..=RUN_AHEAD as usize] {
            assert!(order.admits(ordinal));
        }
        assert!(!order.admits(ordinals[RUN_AHEAD as usize + 1]));
        // Ioctls ahead of the turn taking their nodes do not move it.
        for &ordinal in &ordinals[1..=RUN_AHEAD as usize] {
            assert!(!order.admit(ordinal));
        }
        assert!(!order.admits(ordinals[RUN_AHEAD as usize + 1]));
        // The oldest one moves the turn past everything already admitted, and wakes the refused.
        assert!(order.admit(oldest));
        assert!(order.admits(last));
        let next = order.assign();
        assert!(order.admits(next));
        assert!(!order.admits(next + RUN_AHEAD + 1));
    }

    #[test]
    fn ioctls_leaving_without_a_node_pass_the_turn() {
        let mut order = Order::new();
        let (a, b, c) = (order.assign(), order.assign(), order.assign());
        let later: [u64; RUN_AHEAD as usize] = core::array::from_fn(|_| order.assign());
        let beyond = later[RUN_AHEAD as usize - 1];
        // b and c fail before their turn; while a waits, ioctls past a + RUN_AHEAD wait too.
        assert!(!order.leave(b));
        assert!(!order.leave(c));
        assert!(!order.admits(beyond));
        // Once a takes its node, the turn passes b and c as well, and the refused one may retry.
        assert!(order.admit(a));
        assert!(order.admits(beyond));
        // With nobody refused since, moving the turn wakes nobody; an ordinal already passed
        // leaves without effect.
        assert!(!order.leave(later[0]));
        assert!(!order.leave(a));
    }

    #[test]
    fn an_ioctl_leaving_too_far_ahead_unorders_admission() {
        let mut order = Order::new();
        let first = order.assign();
        let mut last = first;
        for _ in 0..ORDER_WINDOW {
            last = order.assign();
        }
        assert!(!order.admits(last));
        assert!(order.leave(last));
        assert!(order.admits(last - 1));
    }

    #[test]
    fn single_engine_admission_wakes_nobody() {
        let mut order = Order::new();
        for _ in 0..3 * RUN_AHEAD {
            let ordinal = order.assign();
            assert!(order.admits(ordinal));
            assert!(!order.admit(ordinal));
        }
    }

    #[test]
    fn releases_return_nodes_to_their_engine() {
        let states = WorkStates::new();
        let word = states.allocate(1).unwrap();
        assert!(states.release(word, 1));
        assert!(!states.release(word, 1));
        assert_eq!(states.used(), 0);
        assert_eq!(states.held[1].load(Ordering::Acquire), 0);
        // An engine outside the table takes nothing.
        assert!(states.allocate(ENGINES).is_none());
    }
}
