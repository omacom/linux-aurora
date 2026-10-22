// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Device-wide firmware utilization. This is not per-client execution time.
//!
//! The primary utilization record contains four independently reported percentages;
//! the aggregate is not the sum of the engine values. Power state comes from the
//! secondary's timestamped power record, never its coprocessor busy flag. Only
//! explicitly written fields are read: unused record tails retain old ring contents.

use core::sync::atomic::{fence, AtomicBool, AtomicU32, AtomicU64, Ordering};

pub(crate) const CLOCK_HZ: u64 = 24_000_000;
const UTILIZATION: u32 = 0x13;
const POWER_STATE: u32 = 0x19;
const POWER_ON: u32 = 0x11;
const POWER_OFF: u32 = 0x12;

pub(crate) struct Load {
    invalidated: AtomicBool,
    sequence: AtomicU64,
    samples: AtomicU64,
    received_ns: AtomicU64,
    ticks: AtomicU64,
    utilization: [AtomicU32; 4],
    power_known: AtomicU32,
    powered: AtomicU32,
    power_ticks: AtomicU64,
    on_ticks: AtomicU64,
    powered_ticks: AtomicU64,
    unpowered_ticks: AtomicU64,
    invalid: AtomicU64,
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct Report {
    pub(crate) invalidated: bool,
    pub(crate) samples: u64,
    pub(crate) received_ns: u64,
    pub(crate) ticks: u64,
    pub(crate) utilization: [u32; 4],
    pub(crate) power_known: bool,
    pub(crate) powered: bool,
    pub(crate) power_ticks: u64,
    pub(crate) on_ticks: u64,
    pub(crate) powered_ticks: u64,
    pub(crate) unpowered_ticks: u64,
    pub(crate) invalid: u64,
}

impl Report {
    /// An off witness at or after the sample overrides its retained percentages.
    /// After wake, an old sample is unavailable until the new interval is sampled.
    pub(crate) fn current(&self) -> Option<[u32; 4]> {
        if self.invalidated {
            return None;
        }
        if self.power_known && !self.powered && self.power_ticks >= self.ticks {
            return Some([0; 4]);
        }
        if self.samples == 0 || self.ticks < self.on_ticks {
            return None;
        }
        Some(self.utilization)
    }
}

fn ticks(words: &[u32; 18]) -> u64 {
    u64::from(words[1]) | (u64::from(words[2]) << 32)
}

impl Load {
    pub(crate) const fn new() -> Self {
        Self {
            invalidated: AtomicBool::new(false),
            sequence: AtomicU64::new(0),
            samples: AtomicU64::new(0),
            received_ns: AtomicU64::new(0),
            ticks: AtomicU64::new(0),
            utilization: [const { AtomicU32::new(0) }; 4],
            power_known: AtomicU32::new(0),
            powered: AtomicU32::new(0),
            power_ticks: AtomicU64::new(0),
            on_ticks: AtomicU64::new(0),
            powered_ticks: AtomicU64::new(0),
            unpowered_ticks: AtomicU64::new(0),
            invalid: AtomicU64::new(0),
        }
    }

    /// The firmware receive worker is the sole writer, serialized by the device lock.
    pub(crate) fn record(&self, role: usize, now_ns: u64, words: &[u32; 18]) {
        let tag = words[0];
        if !matches!(
            (role, tag),
            (0, UTILIZATION) | (1, POWER_STATE | POWER_ON | POWER_OFF)
        ) {
            return;
        }
        if role == 1 && tag == POWER_STATE && words[6] <= 1 {
            let timestamp = ticks(words);
            let powered = u32::from(words[6] == 0);
            if timestamp != 0
                && self.power_known.load(Ordering::Relaxed) != 0
                && self.powered.load(Ordering::Relaxed) == powered
            {
                // The state and wake epoch are unchanged. Only this atomic timestamp
                // advances; a concurrent transition still changes the sequence below.
                // Avoid rewriting the complete snapshot for every repeated report.
                if timestamp > self.power_ticks.load(Ordering::Relaxed) {
                    self.power_ticks.store(timestamp, Ordering::Relaxed);
                }
                return;
            }
        }
        self.sequence.fetch_add(1, Ordering::AcqRel);
        match (role, tag) {
            (0, UTILIZATION) => {
                let timestamp = ticks(words);
                if timestamp == 0 || words[3..7].iter().any(|&percent| percent > 100) {
                    self.invalid.fetch_add(1, Ordering::Relaxed);
                } else if timestamp > self.ticks.load(Ordering::Relaxed) {
                    for (out, value) in self.utilization.iter().zip(&words[3..7]) {
                        out.store(*value, Ordering::Relaxed);
                    }
                    self.received_ns.store(now_ns, Ordering::Relaxed);
                    self.ticks.store(timestamp, Ordering::Relaxed);
                    self.samples.fetch_add(1, Ordering::Relaxed);
                }
            }
            (1, POWER_STATE) => {
                let timestamp = ticks(words);
                if timestamp == 0 || words[6] > 1 {
                    self.invalid.fetch_add(1, Ordering::Relaxed);
                } else if timestamp > self.power_ticks.load(Ordering::Relaxed) {
                    let powered = u32::from(words[6] == 0);
                    if powered != 0
                        && (self.power_known.load(Ordering::Relaxed) == 0
                            || self.powered.load(Ordering::Relaxed) == 0)
                    {
                        self.on_ticks.store(timestamp, Ordering::Relaxed);
                    }
                    self.powered.store(powered, Ordering::Relaxed);
                    self.power_known.store(1, Ordering::Relaxed);
                    self.power_ticks.store(timestamp, Ordering::Relaxed);
                }
            }
            (1, POWER_ON | POWER_OFF) => {
                let counter = if tag == POWER_OFF {
                    &self.powered_ticks
                } else {
                    &self.unpowered_ticks
                };
                counter.store(
                    counter.load(Ordering::Relaxed).saturating_add(ticks(words)),
                    Ordering::Relaxed,
                );
            }
            _ => {}
        }
        self.sequence.fetch_add(1, Ordering::Release);
    }

    /// Fatal device loss and shutdown cannot leave the last busy value current.
    pub(crate) fn invalidate(&self) {
        self.invalidated.store(true, Ordering::Release);
    }

    pub(crate) fn snapshot(&self) -> Option<Report> {
        for _ in 0..8 {
            let sequence = self.sequence.load(Ordering::Acquire);
            if sequence & 1 != 0 {
                continue;
            }
            let report = Report {
                invalidated: self.invalidated.load(Ordering::Acquire),
                samples: self.samples.load(Ordering::Relaxed),
                received_ns: self.received_ns.load(Ordering::Relaxed),
                ticks: self.ticks.load(Ordering::Relaxed),
                utilization: core::array::from_fn(|i| self.utilization[i].load(Ordering::Relaxed)),
                power_known: self.power_known.load(Ordering::Relaxed) != 0,
                powered: self.powered.load(Ordering::Relaxed) != 0,
                power_ticks: self.power_ticks.load(Ordering::Relaxed),
                on_ticks: self.on_ticks.load(Ordering::Relaxed),
                powered_ticks: self.powered_ticks.load(Ordering::Relaxed),
                unpowered_ticks: self.unpowered_ticks.load(Ordering::Relaxed),
                invalid: self.invalid.load(Ordering::Relaxed),
            };
            fence(Ordering::Acquire);
            if sequence == self.sequence.load(Ordering::Relaxed) {
                return Some(report);
            }
        }
        None
    }
}
