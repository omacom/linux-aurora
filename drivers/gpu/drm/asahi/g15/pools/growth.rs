// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! The PB growth ledger and the growth-request checks.
//!
//! The firmware asks for more PB with an event of type 7 naming PB slot 0, the context of a
//! published render, its request counter (P+0x60 before its increment), the TA position and the
//! halt epoch; the host appends at most ten blocks of the mapped heap to the block list and
//! advances the block_ctl+4 producer (modulo K), then replies with DEVCTL 7. At the maximum the
//! reply refuses, and the firmware finishes the scene with partial renders. A request from
//! before a halt (epoch) or not matching a published render prefix is left unanswered.
//!
//! Core-only (no `kernel` imports): this file is compiled by the host test harness as well.

use crate::g15::fw::buffer::{grow_request, BLOCK_LIST_RECORDS, GROW_STEP, MAX_BLOCKS, MIN_BLOCKS};

/// The PB ledger since the last (re)initialization.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) struct Ledger {
    /// Initial blocks (the C1 pages, InitBM blocks and TA register #33 name them).
    pub(crate) initial: u32,
    /// Maximum blocks (P+0x70/+0x74 = 4 * max pages).
    pub(crate) max: u32,
    /// Blocks posted to the block list, cumulative (the block_ctl+4 producer is this modulo K).
    pub(crate) posted: u32,
    /// The next expected request counter.
    pub(crate) next_request: u32,
    /// A refusal at the maximum was logged since the last initialization.
    pub(crate) refusal_logged: bool,
}

impl Ledger {
    /// A fresh ledger. With growth, the initial size is clamped to 64..=2048 blocks and the
    /// maximum to initial..=2048; without, both are the fixed 2048-block PB.
    pub(crate) const fn new(grow: bool, blocks: u32, max_blocks: u32) -> Self {
        let (initial, max) = if grow {
            let nb = clamp(blocks, MIN_BLOCKS, MAX_BLOCKS);
            (nb, clamp(max_blocks, nb, MAX_BLOCKS))
        } else {
            (MAX_BLOCKS, MAX_BLOCKS)
        };
        Self {
            initial,
            max,
            posted: initial,
            next_request: 0,
            refusal_logged: false,
        }
    }

    /// The live block_ctl+4 producer.
    pub(crate) const fn producer(&self) -> u32 {
        self.posted % BLOCK_LIST_RECORDS
    }

    /// Whether `request` may be answered: growth on, an event of type 7 for PB slot 0 from a
    /// context with a published render prefix (`live`), a counter not below the next expected
    /// one and the current halt epoch.
    pub(crate) fn admits(&self, request: &[u32; 14], grow: bool, live: bool, epoch: u64) -> bool {
        grow && request[grow_request::TYPE] == 7
            && request[grow_request::SLOT] == 0
            && live
            && request[grow_request::COUNTER] >= self.next_request
            && request_epoch(request) == epoch
    }

    /// Accepts an admitted request: the next expected counter follows it, and the returned
    /// blocks `[first, first + count)` are to be written to the block list, `count` 0 meaning a
    /// refusal at the maximum.
    pub(crate) fn accept(&mut self, request: &[u32; 14]) -> (u32, u32) {
        self.next_request = request[grow_request::COUNTER].wrapping_add(1);
        (self.posted, GROW_STEP.min(self.max - self.posted))
    }

    /// Records `count` blocks as posted after their records were written and returns the new
    /// producer.
    pub(crate) fn post(&mut self, count: u32) -> u32 {
        self.posted += count;
        self.producer()
    }

    /// Whether the answer is logged at info level: a failed reply, the first two requests, or
    /// the first refusal at the maximum since the initialization. Records the refusal.
    pub(crate) fn log_answer(
        &mut self,
        request: &[u32; 14],
        count: u32,
        reply_failed: bool,
    ) -> bool {
        let info = reply_failed
            || request[grow_request::COUNTER] < 2
            || (count == 0 && self.posted == self.max && !self.refusal_logged);
        self.refusal_logged |= count == 0;
        info
    }
}

/// The halt epoch a request names (u64 at words 5..6).
pub(crate) const fn request_epoch(request: &[u32; 14]) -> u64 {
    request[grow_request::EPOCH] as u64 | (request[grow_request::EPOCH + 1] as u64) << 32
}

const fn clamp(v: u32, lo: u32, hi: u32) -> u32 {
    if v < lo {
        lo
    } else if v > hi {
        hi
    } else {
        v
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn request(ctx: u32, counter: u32, epoch: u64) -> [u32; 14] {
        let mut q = [0u32; 14];
        q[0] = 7;
        q[1] = ctx;
        q[2] = 0;
        q[3] = counter;
        q[4] = 0x100;
        q[5] = epoch as u32;
        q[6] = (epoch >> 32) as u32;
        q
    }

    #[test]
    fn clamps() {
        let l = Ledger::new(true, 1024, 2048);
        assert_eq!(
            (l.initial, l.max, l.posted, l.next_request),
            (1024, 2048, 1024, 0)
        );
        assert_eq!(l.producer(), 1024);
        assert_eq!(Ledger::new(true, 10, 20).initial, 64);
        assert_eq!(Ledger::new(true, 10, 20).max, 64);
        assert_eq!(Ledger::new(true, 4000, 100).initial, 2048);
        assert_eq!(Ledger::new(true, 512, 1024).max, 1024);
        assert_eq!(Ledger::new(true, 512, 100).max, 512);
        let fixed = Ledger::new(false, 512, 1024);
        assert_eq!((fixed.initial, fixed.max, fixed.posted), (2048, 2048, 2048));
    }

    #[test]
    fn admission() {
        let l = Ledger::new(true, 1024, 2048);
        let q = request(5, 0, 3);
        assert!(l.admits(&q, true, true, 3));
        assert!(!l.admits(&q, false, true, 3));
        assert!(!l.admits(&q, true, false, 3));
        // Request of an earlier halt epoch (dropped at recovery, never answered).
        assert!(!l.admits(&q, true, true, 4));
        let mut bad = q;
        bad[0] = 6;
        assert!(!l.admits(&bad, true, true, 3));
        let mut bad = q;
        bad[2] = 1;
        assert!(!l.admits(&bad, true, true, 3));
        // 64-bit epochs.
        let q = request(5, 0, 0x1_0000_0002);
        assert!(l.admits(&q, true, true, 0x1_0000_0002));
        assert!(!l.admits(&q, true, true, 2));
    }

    #[test]
    fn growth_to_the_maximum() {
        // 512 initial and 1024 maximum blocks.
        let mut l = Ledger::new(true, 512, 1024);
        let mut grown = 0;
        let mut counter = 0;
        loop {
            let q = request(5, counter, 1);
            assert!(l.admits(&q, true, true, 1));
            let (first, n) = l.accept(&q);
            assert_eq!(first, 512 + grown);
            if n == 0 {
                break;
            }
            assert!(n <= 10);
            assert_eq!(l.post(n), (512 + grown + n) % 4096);
            grown += n;
            counter += 1;
        }
        assert_eq!(l.posted, 1024);
        assert_eq!(grown, 512);
        // 51 requests of 10 blocks and one of 2.
        assert_eq!(counter, 52);
        // A stale (repeated) counter is refused, a later one accepted.
        assert!(!l.admits(&request(5, counter - 1, 1), true, true, 1));
        assert!(l.admits(&request(5, counter + 3, 1), true, true, 1));
    }

    #[test]
    fn producer_wraps_at_k() {
        let mut l = Ledger::new(true, 2048, 2048);
        assert_eq!(l.producer(), 2048);
        l.posted = 4095;
        assert_eq!(l.post(1), 0);
        assert_eq!(l.post(10), 10);
    }

    #[test]
    fn logging() {
        let mut l = Ledger::new(true, 64, 64);
        // The first two requests are logged.
        assert!(l.log_answer(&request(1, 0, 0), 10, false));
        assert!(l.log_answer(&request(1, 1, 0), 10, false));
        assert!(!l.log_answer(&request(1, 2, 0), 10, false));
        // A failed reply is logged.
        assert!(l.log_answer(&request(1, 5, 0), 10, true));
        // The first refusal at the maximum is logged once.
        assert!(l.log_answer(&request(1, 6, 0), 0, false));
        assert!(!l.log_answer(&request(1, 7, 0), 0, false));
        assert!(l.refusal_logged);
    }
}
