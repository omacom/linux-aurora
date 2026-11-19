// SPDX-License-Identifier: GPL-2.0-only

//! Fixed GPU VA arenas of the M5 lower address space.

use core::ops::Range;

/// Canonical queue backing has its own sparse arena. It must not overlap
/// the client-private arena or retain the A18 Pro's sub-2-GiB limit.
pub(super) const CANONICAL_RANGE: Range<u64> = 0x80_0000_0000..0x90_0000_0000;
/// Client-private aliases of M5 queues, placed per client VM. Like the
/// canonical arena it lies above Mesa's M5 windows (driver reservation, data
/// heap 0x40.., its read-only shadow 0x60..0x80, which Mesa binds for every
/// allocation, and the USC window 0x100..), so no user mapping can need it.
pub(super) const CLIENT_PRIVATE_RANGE: Range<u64> = 0x90_0000_0000..0xa0_0000_0000;
