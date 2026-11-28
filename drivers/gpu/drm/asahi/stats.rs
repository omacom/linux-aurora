// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Snapshot of AGX firmware statistics.
//!
//! [`StatsSnapshot`] accumulates the values decoded by `StatsChannel::poll`
//! (see `fw::channels.rs::StatsMsg`) and exposes them to the sysfs file
//! `agx_stats` (registered from `sysfs.c`).
//!
//! The snapshot belongs to one GPU manager. The firmware-stat
//! decoder that feeds it lives in `StatsChannel::poll` (`channel.rs`), inside
//! the versioned context where `StatsMsg` is nameable; `note_job()` is called
//! from the `recv_message` rtkit callback (so the firmware mailbox IRQ
//! context); the sysfs `show` callback runs in arbitrary process context. All
//! legacy fields use `AtomicU*`. G17 instead uses the optional reader under
//! its device mutex and does not populate these legacy atomic fields.
//!
//! Field semantics (firmware units, late-validated):
//! - `util1..util4`: raw `Utilization` u32s from the firmware. Meaning
//!   (firmware tick or %?) is validated against a controlled MLX load; we
//!   publish them unscaled per the producer contract.
//! - `pstate`: current performance state index from `PowerState`.
//! - `avg_power_mw`: average power in milliwatts from `AvgPower` (firmware
//!   field is u32 — units not validated for every chip rev; treat as raw).
//! - `temperature`: raw `Temperature` value, scale, tmin, tmax.
//! - `busy_ns`: cumulative utilization-weighted busy time in nanoseconds,
//!   derived from successive `Utilization` windows (busiest subqueue
//!   percentage x window duration; the firmware timestamps are base-clock
//!   ticks, 24 MHz, converted with `HwConfig::base_clock_hz`). Monotonic and
//!   bounded by elapsed time. Saturates at u64::MAX. A window only counts
//!   while the GPU stayed powered: `PowerOff`/`PowerOn` drop the baseline,
//!   so the idle gap before the first window after power-up is not credited
//!   at that window's (busy) percentage. See
//!   [`crate::stats_busy::BusyIntegrator`].
//! - `jobs`: completed submissions since boot, counted at submission
//!   completion (`JobFence::command_complete`).
//!
//! This module is read-only with respect to scheduling or power behaviour:
//! the firmware already sends the messages, the channel already polls them,
//! we only retain the latest values.

use core::sync::atomic::{AtomicU32, AtomicU64, Ordering};
use kernel::prelude::*;

/// Backend-owned qualified statistics reader for G17, G15 and M3 GPUs.
pub(crate) enum QualifiedReader {
    G17(crate::g17::stats::Reader),
    #[cfg(CONFIG_DRM_ASAHI_G15)]
    G15(crate::g15::stats::Reader),
    #[cfg(CONFIG_DRM_ASAHI_M3_T8122)]
    M3(crate::m3::m3_stats::Reader),
}

impl QualifiedReader {
    pub(crate) fn read(&self) -> crate::g17::telemetry::Readout {
        match self {
            Self::G17(reader) => reader.read(),
            #[cfg(CONFIG_DRM_ASAHI_G15)]
            Self::G15(reader) => reader.read(),
            #[cfg(CONFIG_DRM_ASAHI_M3_T8122)]
            Self::M3(reader) => reader.read(),
        }
    }

    pub(crate) fn suspend(&self, suspended: bool) {
        match self {
            Self::G17(reader) => reader.suspend(suspended),
            #[cfg(CONFIG_DRM_ASAHI_G15)]
            Self::G15(reader) => reader.suspend(suspended),
            #[cfg(CONFIG_DRM_ASAHI_M3_T8122)]
            Self::M3(reader) => reader.suspend(suspended),
        }
    }

    pub(crate) fn frequency(&self) -> Option<(Option<u64>, u64)> {
        match self {
            Self::G17(reader) => reader.frequency(),
            #[cfg(CONFIG_DRM_ASAHI_G15)]
            Self::G15(reader) => reader.frequency(),
            #[cfg(CONFIG_DRM_ASAHI_M3_T8122)]
            Self::M3(reader) => reader.frequency(),
        }
    }
}

/// Per-device snapshot of AGX firmware stats.
///
/// The manager and bound driver retain Arcs. The driver's Arc outlives the
/// per-device C sysfs registration; repr(C) establishes the shared layout.
#[derive(Default)]
#[repr(C)]
pub(crate) struct StatsSnapshot {
    pub(crate) util1: AtomicU32,
    pub(crate) util2: AtomicU32,
    pub(crate) util3: AtomicU32,
    pub(crate) util4: AtomicU32,
    pub(crate) pstate: AtomicU32,
    pub(crate) avg_power_mw: AtomicU32,
    pub(crate) temperature_raw: AtomicU32,
    pub(crate) temperature_scale: AtomicU32,
    pub(crate) temperature_tmin: AtomicU32,
    pub(crate) temperature_tmax: AtomicU32,
    /// Cumulative utilization-weighted busy time.
    pub(crate) busy_ns: AtomicU64,
    /// Completed submissions since boot, counted at fence signal in
    /// `JobFence::command_complete` when the last command of a submission
    /// completes.
    pub(crate) jobs: AtomicU64,
    /// Optional qualified reader. The legacy C layout above remains a prefix;
    /// C accesses this extension only through the registered Rust callbacks.
    pub(crate) qualified_reader: Option<QualifiedReader>,
    /// Frequencies in the same DT order used to build the legacy firmware table.
    pub(crate) frequencies_hz: KVec<u64>,
}

// Keep the Rust half of the C sysfs layout contract checked as well.
const _: () = {
    assert!(core::mem::offset_of!(StatsSnapshot, pstate) == 16);
    assert!(core::mem::offset_of!(StatsSnapshot, busy_ns) == 40);
    assert!(core::mem::offset_of!(StatsSnapshot, jobs) == 48);
    assert!(core::mem::offset_of!(StatsSnapshot, qualified_reader) == 56);
};

impl StatsSnapshot {
    pub(crate) fn requested_frequency(&self, pstate: u32) -> Option<u64> {
        self.frequencies_hz.get(pstate as usize).copied()
    }

    /// The frequency the GPU runs at now, if the firmware reports it, and its highest, in
    /// Hz, for fdinfo (`drm-curfreq-*`, `drm-maxfreq-*`): qualified backends (G17, G15, M3).
    pub(crate) fn engine_frequency(&self) -> Option<(Option<u64>, u64)> {
        self.qualified_reader.as_ref()?.frequency()
    }

    /// Bump the completed-submission counter (called from the queue
    /// completion path in `JobFence::command_complete`, NOT from the stats
    /// channel).
    pub(crate) fn note_job(&self) {
        self.jobs.fetch_add(1, Ordering::Relaxed);
    }
}
