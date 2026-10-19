// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Compute, TA and FS work images and microsequence records of the 2419 firmware.
//!
//! Compute work: 0x893 bytes at client +0x3760, sequence 0x2b8. TA work: 0x92b bytes, 72
//! register records, sequence 0x2d8. FS work: 0xc73 bytes, 79 register records and the
//! parameter block, sequence 0x310. Register records are 12-byte `(u32 selector, u64 value)`
//! pairs; timestamp records are 0x3c bytes (0x80000003/3). Barrier records are 0x3e bytes: +4
//! wait stamp VA, +0x14 value, +0x18 producing engine, +0x1c own stamp, +0x20 UUID, +0x2a type
//! (0 = firmware-stamp wait only, 1 = also the predecessor software-job hold).
//!
//! Microsequence opcodes: compute start 0x0b / finalize 0x0c, TA 5 / 6, FS 7 / 8, barrier 4,
//! InitBM 6, END 0x40000002. Work type word: compute 3, TA 0, FS 1. HardwareData references:
//! compute +0x3740, TA +0x1884, FS +0x24c8.
//!
//! The encoders ([`super::super::encode`]) write the sparse words of these images at literal
//! offsets (kept inline there); this module holds the sizes, the fixed records and the register
//! selector tables.

/// Microsequence opcodes.
pub(crate) mod op {
    pub(crate) const COMPUTE_START: u32 = 0x0b;
    pub(crate) const COMPUTE_FINALIZE: u32 = 0x0c;
    pub(crate) const TA_START: u32 = 5;
    pub(crate) const TA_FINALIZE: u32 = 6;
    pub(crate) const FS_START: u32 = 7;
    pub(crate) const FS_FINALIZE: u32 = 8;
    pub(crate) const BARRIER: u32 = 4;
    pub(crate) const INIT_BM: u32 = 6;
    pub(crate) const END: u32 = 0x40000002;
    /// Start timestamp record; the end record is `TIMESTAMP & 0xff` (3).
    pub(crate) const TIMESTAMP: u32 = 0x80000003;
    pub(crate) const TIMESTAMP_END: u32 = 3;
}

/// Barrier wait types (+0x2a of a barrier record).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u8)]
pub(crate) enum BarrierType {
    /// Wait for the producer's firmware stamp only.
    Stamp = 0,
    /// Also hold for the predecessor software job (0x537/0x538).
    Hold = 1,
}

impl BarrierType {
    /// Type 0 if `stamp_only`, else type 1.
    pub(crate) const fn from_stamp_only(stamp_only: bool) -> Self {
        if stamp_only {
            BarrierType::Stamp
        } else {
            BarrierType::Hold
        }
    }
}

/// Work type words.
pub(crate) mod work_type {
    pub(crate) const TA: u32 = 0;
    pub(crate) const FS: u32 = 1;
    pub(crate) const COMPUTE: u32 = 3;
}

/// One `(u32 selector, u64 value)` register record.
pub(crate) const REG_RECORD: usize = 12;

/// Compute work image and microsequence.
pub(crate) mod compute {
    /// Work image bytes.
    pub(crate) const WORK_SIZE: usize = 0x893;
    /// Microsequence bytes; the sequence object is 0x1000 bytes.
    pub(crate) const SEQ_SIZE: usize = 0x2b8;
    pub(crate) const SEQ_OBJECT: usize = 0x1000;
    /// Register records at +0x20.
    pub(crate) const REGS: usize = 0x20;
    /// HardwareData compute activity record.
    pub(crate) const HWDATA: u64 = 0x3740;
    /// Attachment records in the start record (+0x54, 16 bytes each).
    pub(crate) const ATTACHMENTS: usize = 0x54;
    pub(crate) const ATTACHMENT_STRIDE: usize = 16;
    /// Start/end timestamp records and the base of their cells in the work image.
    pub(crate) const TS_START: usize = 0x1bc;
    pub(crate) const TS_END: usize = 0x1fc;
    pub(crate) const TS_CELLS: u64 = 0x810;
    /// Finalize record.
    pub(crate) const FINALIZE: usize = 0x238;
    pub(crate) const END: usize = 0x2b4;

    /// Register selectors of the compute work image.
    pub(crate) const SEL: [u32; 22] = [
        0x1a510, 0x1a420, 0x1a4d0, 0x1a4d8, 0x1a4e0, 0x1a4e8, 0x1a440, 0x1a458, 0x12091, 0x101d9,
        0x1a089, 0x1a091, 0x1a059, 0x1a061, 0x1a0b9, 0x1a0c1, 0x101d1, 0xd479, 0x1a0e9, 0x107a1,
        0xa599, 0xd411,
    ];
}

/// TA work image and microsequence.
pub(crate) mod ta {
    pub(crate) const WORK_SIZE: usize = 0x92b;
    pub(crate) const SEQ_SIZE: usize = 0x2d8;
    pub(crate) const SEQ_OBJECT: usize = 0x1000;
    /// Register records at +0x40.
    pub(crate) const REGS: usize = 0x40;
    pub(crate) const HWDATA: u64 = 0x1884;
    /// Work tail (shared layout, [`super::tail`]).
    pub(crate) const TAIL: usize = 0x8a8;
    pub(crate) const TS_START: usize = 0x1cc;
    pub(crate) const TS_END: usize = 0x20c;
    pub(crate) const FINALIZE: usize = 0x248;
    pub(crate) const END: usize = 0x2d4;

    /// Register selectors of the TA work image.
    pub(crate) const SEL: [u32; 72] = [
        0x1748, 0x10141, 0x1c039, 0x1c9c8, 0x1c041, 0x1c9d0, 0x1c0a1, 0x1c031, 0x1c9c0, 0x1c051,
        0x1c061, 0x10149, 0x10139, 0x10111, 0x1c9b0, 0x10119, 0x1c9b8, 0x1c958, 0x1c950, 0x1c930,
        0x1c880, 0x1c898, 0x1c948, 0x1c888, 0x1c890, 0x1c918, 0x1c079, 0x1c9d8, 0x1c089, 0x1c9e0,
        0x16c41, 0x1ca40, 0x1c9a8, 0x1c920, 0x10151, 0x1c199, 0x1c1a1, 0x1c1a9, 0x1c1b1, 0x1c1b9,
        0x1c8f8, 0x1c0b1, 0x1c850, 0x10131, 0x10121, 0x10129, 0x101b9, 0x1c069, 0x1c071, 0x1c081,
        0x1c0a9, 0x10171, 0x10169, 0x12099, 0x101e1, 0x1c9e8, 0x1a099, 0x1a0a1, 0x1a069, 0x1a071,
        0x1a0c9, 0x1a0d1, 0x101c9, 0xd471, 0x1a0f1, 0x10799, 0xa5a1, 0xd419, 0x1c830, 0x1ca30,
        0x16c39, 0x1c910,
    ];

    /// Index of register #33 (PBREQ: the initial PB size in pages, 0x3c when clustered).
    pub(crate) const REG_PBREQ: usize = 32;
}

/// FS work image and microsequence.
pub(crate) mod fs {
    pub(crate) const WORK_SIZE: usize = 0xc73;
    pub(crate) const SEQ_SIZE: usize = 0x310;
    pub(crate) const SEQ_OBJECT: usize = 0x1000;
    /// Register records at +0x80.
    pub(crate) const REGS: usize = 0x80;
    /// Parameter block / firmware mirrors.
    pub(crate) const PARAMS: usize = 0x790;
    pub(crate) const HWDATA: u64 = 0x24c8;
    pub(crate) const TAIL: usize = 0xbf0;
    pub(crate) const TS_START: usize = 0x1dc;
    pub(crate) const TS_END: usize = 0x21c;
    pub(crate) const FINALIZE: usize = 0x258;
    pub(crate) const END: usize = 0x30c;

    /// Register selectors of the FS work image.
    pub(crate) const SEL: [u32; 79] = [
        0x1739, 0x10009, 0x15379, 0x15381, 0x15369, 0x15371, 0x15131, 0x15139, 0x100a1, 0x15069,
        0x15071, 0x16058, 0x10019, 0x100b1, 0x16030, 0x100d9, 0x10791, 0x16098, 0x15109, 0x15101,
        0x15021, 0x15211, 0x15049, 0x10051, 0x15321, 0x15301, 0x15309, 0x15311, 0x15319, 0x15349,
        0x15351, 0x15329, 0x15331, 0x15339, 0x15341, 0x15231, 0x15221, 0x15239, 0x15229, 0x15401,
        0x15421, 0x15409, 0x15429, 0x153c1, 0x15411, 0x153c9, 0x15431, 0x153d1, 0x15419, 0x153d9,
        0x15439, 0x16429, 0x16060, 0x16431, 0x10039, 0x16020, 0x16451, 0x15359, 0x100b8, 0x16461,
        0x16090, 0x120a1, 0x101e9, 0x160a8, 0x16068, 0x160b8, 0x1a0a9, 0x1a0b1, 0x1a079, 0x1a081,
        0x1a0d9, 0x1a0e1, 0x101c1, 0xd469, 0x1a0f9, 0x1c838, 0x1ca28, 0xa5a9, 0xd429,
    ];
}

/// The 0x3c timestamp record: `+0` opcode, `+4/+0xc/+0x14` cells of the work image at `B`,
/// `+0x1c` queue info, `+0x24` user pair (0 unless requested), `+0x2c` `W + B + 0x58`, `+0x34`
/// UUID.
pub(crate) mod ts_record {
    pub(crate) const SIZE: usize = 0x3c;
}

/// The work tail shared by TA (+0x8a8) and FS (+0xbf0); the compute image has the same layout
/// at +0x810.
pub(crate) mod tail {
    pub(crate) const ITS_START: usize = 0x8;
    pub(crate) const ITS_END: usize = 0x10;
    pub(crate) const USER_START: usize = 0x18;
    pub(crate) const USER_END: usize = 0x20;
    pub(crate) const UMA_MANAGER: usize = 0x2e;
    pub(crate) const SCRATCH: usize = 0x47;
}

/// Barrier record.
pub(crate) mod barrier {
    pub(crate) const SIZE: usize = 0x3e;
    pub(crate) const STAMP_VA: usize = 0x4;
    pub(crate) const STAMP_VA2: usize = 0xc;
    pub(crate) const WAIT: usize = 0x14;
    pub(crate) const EVENT: usize = 0x18;
    pub(crate) const OWN: usize = 0x1c;
    pub(crate) const UUID: usize = 0x20;
    pub(crate) const TYPE: usize = 0x2a;
    /// UUID of render predecessor waits.
    pub(crate) const RENDER_WAIT_UUID: u32 = 0x3d420;
}

/// InitBM record (0x20 bytes).
pub(crate) mod initbm {
    pub(crate) const SIZE: usize = 0x20;
    pub(crate) const CTX: usize = 0x4;
    pub(crate) const BLOCKS: usize = 0x10;
    pub(crate) const PB_MANAGER: usize = 0x14;
    pub(crate) const STAMP: usize = 0x1c;
}

/// Per-pass scene record (0x80 bytes).
pub(crate) mod scene {
    pub(crate) const SIZE: usize = 0x80;
    /// Scene-list slots cycled by draw ordinal (`scene_list + 4 * (n % 48)`).
    pub(crate) const LIST_SLOTS: u64 = 48;
}
