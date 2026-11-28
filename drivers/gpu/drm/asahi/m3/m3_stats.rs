// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Sysfs qualified statistics reader for the M3 (`t8122`) backend.
//!
//! Owns the shared runtime handle and DVFS frequency table. Reads acquire the
//! runtime mutex, drain the statistics ring without waking the GPU, and return
//! an `UNAVAILABLE` readout once `Backend::stop()` detaches the runtime.

use crate::{
    g17::telemetry::Readout,
    m3::m3_drm,
    stats_m3::{self, FreqTable},
};

pub(crate) struct Reader(
    pub(crate) m3_drm::Shared,
    pub(crate) FreqTable,
    pub(crate) u32,
);

impl Reader {
    pub(crate) fn read(&self) -> Readout {
        let mut guard = self.0.lock();
        let Some(runtime) = Option::as_mut(&mut *guard) else {
            return Readout {
                util_source: self.2,
                ..Readout::default()
            };
        };
        runtime.read_qualified_stats()
    }

    pub(crate) fn suspend(&self, suspended: bool) {
        if let Some(runtime) = Option::as_mut(&mut *self.0.lock()) {
            runtime.suspend_stats(suspended);
        }
    }

    pub(crate) fn frequency(&self) -> Option<(Option<u64>, u64)> {
        let view = self.read();
        stats_m3::frequency_pair(&view, &self.1)
    }
}
