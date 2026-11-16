// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! G17 firmware structures.
//!
//! Layouts of the records and descriptors the host shares with the G17
//! firmware, and the pure builders that fill them. Nothing here allocates,
//! locks or publishes: the owners of the rings and queues do that.
//!
//! This module also holds the few rules shared by the render and compute
//! work descriptors.

use kernel::prelude::*;

pub(crate) mod buffer;
pub(crate) mod channels;
pub(crate) mod compute;
pub(crate) mod compute_pool;
pub(crate) mod context;
pub(crate) mod freelist;
pub(crate) mod initdata;
pub(crate) mod kick;
pub(crate) mod queue;
pub(crate) mod render;
pub(crate) mod render_rt;
pub(crate) mod qos;
pub(crate) mod variant;

pub(crate) use variant::Variant;

/// Clears a firmware structure in place.
///
/// Work descriptors are several KiB large and must not be built on the stack.
pub(crate) fn clear<T: Zeroable>(object: &mut T) {
    // SAFETY: `T: Zeroable` guarantees that all-zero bytes are a valid `T`,
    // and `object` is a valid, exclusive reference to one `T`.
    unsafe { core::ptr::write_bytes(core::ptr::from_mut(object), 0, 1) };
}

/// Returns `va + offset`, or `EINVAL` if the sum overflows.
pub(crate) fn offset_va(va: u64, offset: u64) -> Result<u64> {
    va.checked_add(offset).ok_or(EINVAL)
}

/// Size of one slot of a start/end timestamp pair.
const TIMESTAMP_SIZE: u64 = 8;

/// Returns the address of the end slot of the timestamp pair at `start_va`.
///
/// A work item's start and end timestamps are two consecutive 64-bit slots.
pub(crate) fn timestamp_end_va(start_va: u64) -> Result<u64> {
    offset_va(start_va, TIMESTAMP_SIZE)
}

/// Returns the work key registers take: the UAT context in bits 15:8 and the
/// low six bits of the work-state word below.
pub(crate) const fn work_key(context_id: u16, state_word: u32) -> u64 {
    (context_id as u64) << 8 | (state_word & 0x3f) as u64
}

/// Returns the sampler bound descriptors store next to the sampler count: the
/// count plus one, or zero without samplers.
pub(crate) fn sampler_max(count: u32) -> Result<u32> {
    match count {
        0 => Ok(0),
        count => count.checked_add(1).ok_or(EINVAL),
    }
}

/// Returns the completion stamp value of sequence number `sequence`: the
/// value the firmware stores to a queue's completion stamp word when the work
/// completes.
pub(crate) const fn stamp_value(sequence: u32) -> u32 {
    sequence.wrapping_shl(8)
}

/// Object IDs of a work item, written to three registers of its engine and
/// to its descriptor.
///
/// A render pass takes `predecessor` from its queue pair and a newly
/// allocated `current`; compute work uses fixed IDs.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) struct ObjectIds {
    /// Predecessor object ID.
    pub(crate) predecessor: u32,
    /// Current object ID.
    pub(crate) current: u32,
}

impl ObjectIds {
    /// Returns the register value: `predecessor` in bits 63:32 and `current`
    /// in bits 31:0.
    pub(crate) const fn word(self) -> u64 {
        (self.predecessor as u64) << 32 | self.current as u64
    }
}

/// Required value of the per-engine registers 0x0a599 (compute), 0x0a5a1
/// (tiling) and 0x0a5a9 (fragment).
pub(crate) const UNK_0A599: u64 = 0x0000_0060_0040_0020;
/// Required value of the per-engine registers 0x0d411 (compute), 0x0d419
/// (tiling) and 0x0d429 (fragment).
pub(crate) const UNK_0D411: u64 = 0x0000_0002_0000_0001;
/// Required value of the per-engine registers 0x101d9 (compute), 0x101e1
/// (tiling) and 0x101e9 (fragment).
pub(crate) const UNK_101D9: u64 = 0x1c;
/// Required value of the per-engine registers 0x1a0e9 (compute), 0x1a0f1
/// (tiling) and 0x1a0f9 (fragment).
pub(crate) const UNK_1A0E9: u64 = 8;
/// Required value of the per-engine registers 0x107a1 (compute) and 0x10799
/// (tiling). The fragment register 0x10791 also sets bit 9.
pub(crate) const UNK_10791: u64 = 0xff_0000;
/// Required value of the tiling register 0x1c8f8 and the fragment register
/// 0x100b8; the compute register 0x1a458 holds it in its low bits.
pub(crate) const UNK_1C8F8: u64 = 0x8860;
