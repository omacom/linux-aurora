// SPDX-License-Identifier: GPL-2.0-only

//! Fixed GPU VA arenas of the M5 lower address space.

use core::ops::Range;

/// Canonical queue backing has its own sparse arena. It must not overlap
/// the client-private arena or retain the A18 Pro's sub-2-GiB limit.
pub(super) const CANONICAL_RANGE: Range<u64> = 0x80_0000_0000..0x90_0000_0000;
