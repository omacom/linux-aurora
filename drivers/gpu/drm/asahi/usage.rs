// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Per-file engine time, shared by all firmware backends.
//!
//! Queues retain their file's ledger until their last completion. Only authenticated,
//! successful completion timestamps are charged, before signalling the completion fence.
//! These are command timer ticks, not shader clock cycles. Never export them as drm-cycles.

use core::sync::atomic::{AtomicU64, Ordering};

#[derive(Clone, Copy)]
pub(crate) enum Engine {
    Vertex,
    Fragment,
    Compute,
}

/// One ledger per DRM file, shared by all of its queues and VMs.
pub(crate) struct Client {
    ns: [AtomicU64; 3],
}

impl Client {
    pub(crate) const fn new() -> Self {
        Self {
            ns: [const { AtomicU64::new(0) }; 3],
        }
    }

    /// Called exactly once per retired stage, while its internal timestamps are retained.
    /// Missing, reversed or reset timestamps contribute nothing. Sub-nanosecond fractions
    /// are rounded down per stage. Saturation keeps fdinfo monotonic even on overflow.
    pub(crate) fn add(&self, engine: Engine, start: u64, end: u64, frequency_hz: u32) {
        if start == 0 || end <= start || frequency_hz == 0 {
            return;
        }
        let ns = ticks_to_ns(end - start, frequency_hz);
        let counter = &self.ns[engine as usize];
        let mut old = counter.load(Ordering::Relaxed);
        loop {
            match counter.compare_exchange_weak(
                old,
                old.saturating_add(ns),
                Ordering::Relaxed,
                Ordering::Relaxed,
            ) {
                Ok(_) => break,
                Err(value) => old = value,
            }
        }
    }

    /// Vertex, fragment and compute, in ns. No cross-engine atomic snapshot is promised.
    pub(crate) fn snapshot(&self) -> [u64; 3] {
        self.ns.each_ref().map(|ns| ns.load(Ordering::Relaxed))
    }
}

fn ticks_to_ns(ticks: u64, frequency_hz: u32) -> u64 {
    const NSEC_PER_SEC: u64 = 1_000_000_000;
    let hz = u64::from(frequency_hz);
    // The remainder is smaller than a u32 frequency, so its product fits in u64.
    (ticks / hz)
        .saturating_mul(NSEC_PER_SEC)
        .saturating_add(ticks % hz * NSEC_PER_SEC / hz)
}

#[cfg(test)]
mod tests {
    use super::*;

    const HZ: u32 = 24_000_000;

    #[test]
    fn idle_and_invalid_intervals() {
        let client = Client::new();
        assert_eq!(client.snapshot(), [0; 3]);
        for (start, end, hz) in [
            (0, 100, HZ),
            (100, 0, HZ),
            (20, 10, HZ),
            (1, 1, HZ),
            (1, 2, 0),
        ] {
            client.add(Engine::Compute, start, end, hz);
        }
        assert_eq!(client.snapshot(), [0; 3]);
    }

    #[test]
    fn engine_and_client_isolation() {
        let a = Client::new();
        let b = Client::new();
        a.add(Engine::Vertex, 10, 24_010, HZ);
        a.add(Engine::Fragment, 20, 48_020, HZ);
        b.add(Engine::Compute, 30, 72_030, HZ);
        assert_eq!(a.snapshot(), [1_000_000, 2_000_000, 0]);
        assert_eq!(b.snapshot(), [0, 0, 3_000_000]);
    }

    #[test]
    fn overlapping_stages_are_independent() {
        let client = Client::new();
        client.add(Engine::Vertex, 1, 25, HZ);
        client.add(Engine::Fragment, 13, 37, HZ);
        assert_eq!(client.snapshot(), [1_000, 1_000, 0]);
    }

    #[test]
    fn conversion_matches_wide_reference() {
        for hz in [1, HZ, 1_000_000_000, u32::MAX] {
            for ticks in [0, 1, 23, 24, 25, u32::MAX as u64, u64::MAX / 2, u64::MAX] {
                let expected = (u128::from(ticks) * 1_000_000_000 / u128::from(hz))
                    .min(u128::from(u64::MAX)) as u64;
                assert_eq!(ticks_to_ns(ticks, hz), expected);
            }
        }
    }

    #[test]
    fn fractional_ticks_round_down() {
        assert_eq!(ticks_to_ns(1, HZ), 41);
        assert_eq!(ticks_to_ns(3, HZ), 125);
    }

    #[test]
    fn out_of_order_completions_accumulate() {
        let client = Client::new();
        client.add(Engine::Compute, 100, 124, HZ);
        client.add(Engine::Compute, 1, 25, HZ);
        assert_eq!(client.snapshot(), [0, 0, 2_000]);
    }

    #[test]
    fn saturation_is_monotonic() {
        let client = Client::new();
        client.add(Engine::Compute, 1, u64::MAX, 1);
        assert_eq!(client.snapshot()[2], u64::MAX);
        client.add(Engine::Compute, 1, 25, HZ);
        assert_eq!(client.snapshot()[2], u64::MAX);
    }

    #[test]
    fn concurrent_queues_share_one_file() {
        let client = std::sync::Arc::new(Client::new());
        let threads: std::vec::Vec<_> = (0..8)
            .map(|_| {
                let client = client.clone();
                std::thread::spawn(move || {
                    for _ in 0..10_000 {
                        client.add(Engine::Compute, 1, 25, HZ);
                    }
                })
            })
            .collect();
        for thread in threads {
            thread.join().unwrap();
        }
        assert_eq!(client.snapshot(), [0, 0, 80_000_000]);
    }
}
