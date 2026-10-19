// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Queue-info rows, queue state, scheduling contexts, notifiers and the compute UMA manager.
//!
//! Queue-info scheduling row 2 (class 2, used for render and compute): +0x30 = 2, +0x34 = 2,
//! +0x38 = 0xffff000000000000, +0x40 = 0, +0x44 = 0, +0x48 = 2. Notifiers are initialized
//! once before first exposure and never written by the host again; the threshold is a
//! separate host-written uncached cell.

/// Queue info (0xb4 bytes, firmware-only, ASC-cached).
pub(crate) mod qinfo {
    pub(crate) const SIZE: usize = 0xb4;
    /// u64 VAs of the queue state, the work ring, the job list and the queue scratch.
    pub(crate) const STATE: usize = 0x00;
    pub(crate) const RING: usize = 0x08;
    pub(crate) const JOB_LIST: usize = 0x10;
    pub(crate) const SCRATCH: usize = 0x18;
    pub(crate) const UNK_2C: usize = 0x2c;
    pub(crate) const UNK_4C: usize = 0x4c;
    pub(crate) const UUID: usize = 0x50;
    /// u64 scheduling context VA, then u64 4.
    pub(crate) const SCHED_CTX: usize = 0xa4;
    pub(crate) const UNK_AC: usize = 0xac;
}

/// Queue-info scheduling rows (the firmware's recognized policy rows).
pub(crate) mod qinfo_row2 {
    pub(crate) const CLASS: usize = 0x30;
    pub(crate) const CLASS_B: usize = 0x34;
    pub(crate) const MASK: usize = 0x38;
    pub(crate) const MASK_VALUE: u64 = 0xffff000000000000;
    pub(crate) const UNK_40: usize = 0x40;
    pub(crate) const UNK_44: usize = 0x44;
    pub(crate) const UNK_48: usize = 0x48;
    /// Row 0 (class 0): +0x38 mask, +0x40 = 1, +0x48 = 1.
    pub(crate) const ROW0_MASK_VALUE: u64 = 0xffffffffffff0000;
}

/// Queue state (0x60 bytes, firmware-only, never cached).
pub(crate) mod qstate {
    pub(crate) const SIZE: usize = 0x60;
    /// Firmware DONE cursor.
    pub(crate) const DONE: usize = 0x00;
    /// Firmware READ cursor.
    pub(crate) const READ: usize = 0x30;
    /// Host WRITE cursor.
    pub(crate) const WRITE: usize = 0x40;
    /// Ring capacity in entries.
    pub(crate) const CAP: usize = 0x50;
}

/// Scheduling context (0x40 bytes): 0xff at +0..+1, 1 at +5, 0xff at +0x1e, 2 at +0x23.
pub(crate) mod sched_ctx {
    pub(crate) const SIZE: usize = 0x40;
}

/// Notifier (0x108 bytes): u64 threshold VA at +0, 0x50 at +0x10, 8 x 0xff at +0xa8; the
/// completing works store their context at +0x24 (firmware-owned once exposed).
pub(crate) mod notifier {
    pub(crate) const SIZE: usize = 0x108;
    pub(crate) const THRESHOLD: usize = 0x00;
    pub(crate) const UNK_10: usize = 0x10;
    pub(crate) const UNK_10_VALUE: u32 = 0x50;
    /// The 8-byte mask the microsequences name (notifier + 0xa8).
    pub(crate) const MASK: usize = 0xa8;
}

/// Job list (0x18 bytes): u64 self VA at +8.
pub(crate) mod job_list {
    pub(crate) const SIZE: usize = 0x18;
    pub(crate) const SELF: usize = 0x08;
}

/// The compute UMA allocator manager (0x84 bytes). The render managers have the same layout
/// (`fw::buffer`).
pub(crate) mod manager {
    pub(crate) const SIZE: usize = 0x84;
    pub(crate) const ID: usize = 0x00;
    pub(crate) const INDEX: usize = 0x08;
    pub(crate) const ONE: usize = 0x0c;
    pub(crate) const FREELIST: usize = 0x14;
    pub(crate) const UNK_1C: usize = 0x1c;
    pub(crate) const UNK_1C_VALUE: u32 = 0x80000;
    pub(crate) const SEEDS_A: usize = 0x24;
    pub(crate) const SEEDS_B: usize = 0x2c;
    pub(crate) const POOL_TABLE: usize = 0x34;
    pub(crate) const UNK_44: usize = 0x44;
    pub(crate) const UNK_44_VALUE: u32 = 0x40;
    pub(crate) const COUNTER: usize = 0x48;
    /// Compute manager identity: id 3, index 2.
    pub(crate) const COMPUTE_ID: u32 = 3;
    pub(crate) const COMPUTE_INDEX: u32 = 2;
}

/// Compute pool table rows (0x40 bytes each, `gva | 1 << 62`) and freelist seeds (u64 VAs of
/// 0x1000-byte units of the first eight pools).
pub(crate) mod pools {
    pub(crate) const TABLE_STRIDE: usize = 0x40;
    pub(crate) const TABLE_VALID: u64 = 1 << 62;
    pub(crate) const SEED_BYTES: u64 = 0x1000;
    pub(crate) const SEEDED_POOLS: usize = 8;
}
