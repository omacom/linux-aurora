// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Page-table operations the M5 paths use beyond the common ones.

use super::{sparse_tile::SparseTile, Prot, UatPageTable};
use core::ops::Range;
use kernel::{prelude::*, sync::Arc};

impl UatPageTable {
    pub(super) fn visit_runs(
        &self,
        range: Range<u64>,
        visit: impl FnMut(u64, u64, u8) -> Result,
    ) -> Result<bool> {
        let _ = (range, visit);
        Err(EOPNOTSUPP)
    }

    pub(super) fn map_tiled(&mut self, range: Range<u64>, tile: &Arc<SparseTile>, prot: Prot) -> Result {
        let _ = (range, tile, prot);
        Err(EOPNOTSUPP)
    }
}
