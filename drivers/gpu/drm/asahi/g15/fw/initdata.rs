// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Initdata layout of the 2419 build.
//!
//! The driver writes sparse words into large objects and board images, so this module holds the
//! offsets that the initdata build and recovery use, not full structure definitions. Several
//! canonical objects (Runtime, HardwareData, Globals, Power, Control) start at unaligned VAs, so
//! their words are written as bytes.

/// Root object (0xc0 bytes).
pub(crate) mod root {
    pub(crate) const SIZE: usize = 0xc0;
    pub(crate) const MAGIC: u64 = 0x0c08e21e83800490;
    pub(crate) const REGION_A: usize = 0x08;
    pub(crate) const RUNTIME: usize = 0x18;
    pub(crate) const GLOBALS: usize = 0x20;
    /// u32 1.
    pub(crate) const ENABLE: usize = 0x2c;
    /// u16 page size, u8 page shift, u8 levels.
    pub(crate) const GEOMETRY: usize = 0x30;
    /// Three 0x20-byte level descriptors.
    pub(crate) const LEVELS: usize = 0x34;
    pub(crate) const LEVEL_STRIDE: usize = 0x20;
    pub(crate) const FLAGS: usize = 0xa8;
    pub(crate) const CONTROL: usize = 0xb0;
    pub(crate) const POWER: usize = 0xb8;
}

/// Region A (one page).
pub(crate) mod region_a {
    pub(crate) const SIZE: usize = 0x4000;
}

/// Runtime object (0x4b3 bytes).
pub(crate) mod runtime {
    pub(crate) const SIZE: usize = 0x4b3;
    pub(crate) const HWDATA: usize = 0x00;
    pub(crate) const PAIR: usize = 0x08;
    pub(crate) const PAIR2: usize = 0x10;
    /// 13 x {state, completion cell, write cell, ring}: the 12 submit channels, then DEVCTL.
    pub(crate) const CHANNELS: usize = 0x18;
    pub(crate) const CHANNEL_STRIDE: usize = 0x20;
    /// 4 x {state, ring}: event, log, trace, stats.
    pub(crate) const RX: usize = 0x1b8;
    pub(crate) const RX_STRIDE: usize = 0x10;
    pub(crate) const SMALL: usize = 0x1f8;
    pub(crate) const TA_ACTIVITY: usize = 0x234;
    pub(crate) const FS_ACTIVITY: usize = 0x23c;
    pub(crate) const COMPUTE_ACTIVITY: usize = 0x244;
    pub(crate) const FAULT_INFO: usize = 0x24c;
    pub(crate) const C0: usize = 0x2a8;
    pub(crate) const C1: usize = 0x2b0;
    pub(crate) const C1_2: usize = 0x2b8;
    pub(crate) const C3: usize = 0x2c0;
    pub(crate) const C3_2: usize = 0x2c8;
    /// u32 4.
    pub(crate) const UNK_2D0: usize = 0x2d0;
    /// Readiness magic written by the firmware.
    pub(crate) const READY: usize = 0x314;
    pub(crate) const READY_MAGIC: u32 = 0xabcdabcd;
    /// u32 0xff.
    pub(crate) const UNK_3B0: usize = 0x3b0;
    pub(crate) const HWDATA_A: usize = 0x441;
}

/// Pair (0x100 bytes) and Small (0x10 bytes).
pub(crate) mod pair {
    pub(crate) const SIZE: usize = 0x100;
}
pub(crate) mod small {
    pub(crate) const SIZE: usize = 0x10;
}

/// C0 (0x1000 bytes), C1 and C3 (one page each): firmware + GPU shared.
pub(crate) mod cbuf {
    pub(crate) const C0_SIZE: usize = 0x1000;
    pub(crate) const C1_SIZE: usize = 0x4000;
    pub(crate) const C3_SIZE: usize = 0x4000;
}

/// Control object (0xc3d0 bytes).
pub(crate) mod control {
    pub(crate) const SIZE: usize = 0xc3d0;
    /// u32 1 at initdata.
    pub(crate) const ENABLE: usize = 0x4;
    pub(crate) const PENDING: usize = 0x40a0;
    pub(crate) const FWCTL_STATE: usize = 0x4568;
    pub(crate) const FWCTL_RING: usize = 0x4570;
    pub(crate) const HALT_COUNT: usize = 0x4580;
    /// u32: firmware halted for recovery (it sets 1, the host clears).
    pub(crate) const HALTED: usize = 0x4590;
    pub(crate) const RESUME: usize = 0x45a0;
    pub(crate) const FLAG: usize = 0x45b0;
    /// u32 1 at initdata.
    pub(crate) const ENABLE2: usize = 0x45c4;
}

/// Globals (15,868 bytes) words used after READY.
pub(crate) mod globals {
    pub(crate) const SIZE: usize = 15868;
    pub(crate) const COMPUTE_KILL: usize = 0x1c;
    pub(crate) const FRAGMENT_KILL: usize = 0x24;
    pub(crate) const IDLE_ENABLE: usize = 0x2c;
    pub(crate) const IDLE_PREDICTIVE: usize = 0x30;
    pub(crate) const IDLE_STANDBY_US: usize = 0x7a8;
    pub(crate) const IDLE_MIN_CONF: usize = 0x7c4;
    /// Template 10, written 100.
    pub(crate) const UNK_980: usize = 0x980;
    pub(crate) const IDLE_DELAY: usize = 0x9a0;
    pub(crate) const COMPUTE_MONITOR: usize = 0x9b0;
}

/// Power (2,292 bytes).
pub(crate) mod power {
    pub(crate) const SIZE: usize = 2292;
}

/// HardwareData (35,332 bytes).
pub(crate) mod hwdata {
    pub(crate) const SIZE: usize = 35332;
    /// Bytes of the first pages mapped ASC-cached (HardwareDataB and the activity records):
    /// up to the end of the page after the one holding byte 0.
    pub(crate) const CACHED: usize = 0x4a04;
    pub(crate) const TIMESTAMPS: usize = 0x28;
    /// IOMapping records: 32 bytes per slot, the firmware VA at +8.
    pub(crate) const IOMAPS: usize = 0x640;
    pub(crate) const IOMAP_STRIDE: usize = 32;
    pub(crate) const IOMAP_VA: usize = 8;
    /// Template checks: u32 1, and the number of DVFS states minus one.
    pub(crate) const CHECK_A38: usize = 0xa38;
    pub(crate) const DVFS_COUNT: usize = 0xb58;
    pub(crate) const TTBAT: usize = 0xb44;
    pub(crate) const TA_ACTIVITY: usize = 0x1880;
    pub(crate) const FS_ACTIVITY: usize = 0x24c0;
    pub(crate) const COMPUTE_ACTIVITY: usize = 0x3740;
    /// HardwareDataA: the maintained S8 request words.
    pub(crate) const HWDATA_A: usize = 0x4580;
    pub(crate) const FAULT_INFO: usize = 0x8900;
}

/// Flags (0x40 bytes).
pub(crate) mod flags {
    pub(crate) const SIZE: usize = 0x40;
    pub(crate) const READY: usize = 4;
}
