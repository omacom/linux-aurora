// SPDX-License-Identifier: GPL-2.0-only OR MIT
//! STUB(G): the selected board's topology. Our #28/#30 port (group G) provides these as
//! `super::compute::{select_topology, clusters, g17d_forms, dispatch_control}`; this file and
//! its probe call go when that lands.

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use super::initdata::{Board, J714, J775D};

static CLUSTERS: AtomicU32 = AtomicU32::new(J714.clusters);
static G17D_FORMS: AtomicBool = AtomicBool::new(false);

/// Selects the board's topology at probe, before any command is encoded.
pub(crate) fn select_topology(board: &Board) {
    CLUSTERS.store(board.clusters, Ordering::Relaxed);
    G17D_FORMS.store(core::ptr::eq(board, &J775D), Ordering::Relaxed);
}

/// The selected board's cluster count.
pub(crate) fn clusters() -> u32 {
    CLUSTERS.load(Ordering::Relaxed)
}

/// Whether the board takes the G17D host encodings measured on the M5 Ultra.
pub(crate) fn g17d_forms() -> bool {
    G17D_FORMS.load(Ordering::Relaxed)
}

/// A dispatch control word for the selected board's cluster count.
pub(crate) fn dispatch_control(base: u64) -> u64 {
    if clusters() > 4 { base | 0x9c } else { base }
}
