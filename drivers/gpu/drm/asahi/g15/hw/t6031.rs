// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! T6031 (M3 Max, G15C: J514C, J516C) configuration.
//!
//! One die, four clusters of ten core slots. The firmware is the same RTKit 2419 generation as
//! the T6030's and the initdata graph keeps the T6030 canonical VAs; what differs is the SoC
//! identity, the 40-bit enabled-core mask (two words), the source of the GPU regions (the
//! device tree node the boot loader fills, not an ADT export), the GPU power (its power domain;
//! there is no PMP vote), the initdata root version, a longer HardwareData record (0xb480
//! bytes, its fourth view at +0xb400) placed aligned and ASC-cached, the IOMapping table (17
//! slots at the T6031 register blocks) and the board images, which come from a firmware file
//! rather than built-in templates. Every value was measured on a J516C: the SGX identification
//! registers, the loaded firmware segment, the device tree the boot loader hands over and the
//! initdata of that machine.

use super::super::Config;
use super::{FirmwareBuild, HwId, IoMap, Layout};

/// SGX register window (the `sgx` resource), physical.
const SGX: u64 = 0x4_0800_0000;
/// The die 0 interrupt controller.
const AIC: u64 = 0x2_9240_0000;

/// Base of the IOMapping firmware VAs and the gap left between consecutive mappings.
const IOMAP_BASE: u64 = 0xfffffc2068000000;
const IOMAP_GAP: u64 = 0x4000;
const PAGE: u64 = 0x4000;

/// Mappings in slot order: slot, physical address, mapped bytes (rounded up to pages below).
/// Slots 0 (Fender, 0x144000), 1 (AIC timer page), 2 (AIC software interrupts), 3 (RGX), 7,
/// 9 (metrology sensors), 11 (memory cache, eight 0xd8000 instances), 12 (AIC banked
/// registers), 18-20 (telemetry dashboard), 21, 24, 25 (ANE), 26 (PMS metrology), 28 and
/// 29 (GPU clock generator at `sgx + 0xe1c000`). The board image carries each slot's
/// physical address, sizes and access flag; the build checks them against this table.
const SLOTS: [(u8, u64, u64); 17] = [
    (0, SGX + 0xd0_0000, 0x14_4000),
    (1, 0x2_0e10_0000, 0x4000),
    (2, AIC + 0x1_4000, 0x4000),
    (3, SGX, 0x2_0000),
    (7, 0x2_9044_4000, 0x4000),
    (9, SGX + 0xe0_8000, 0x8000),
    (11, 0x2_2000_0000, 8 * 0xd_8000),
    (12, AIC + 0x4_c000, 0x4000),
    (18, 0x2_903d_0000, 0x4000),
    (19, 0x2_903c_0000, 0x4000),
    (20, 0x2_903d_8000, 0x4000),
    (21, SGX + 0xeb_4000, 0x4000),
    (24, SGX + 0x100_0000, 0x8000),
    (25, 0x3_c945_c000, 0x4000),
    (26, 0x2_9040_0000, 0x8000),
    (28, 0x2_a4dc_0000, 0x2_4000),
    (29, SGX + 0xe1_c000, 0x4000),
];

/// Packs the slots' firmware VAs: consecutive from [`IOMAP_BASE`], each mapping followed by one
/// unmapped page.
const fn iomaps() -> [IoMap; 17] {
    let mut out = [IoMap {
        slot: 0,
        pa: 0,
        map_bytes: 0,
        va: 0,
        delta: 0,
    }; 17];
    let mut va = IOMAP_BASE;
    let mut i = 0;
    while i < SLOTS.len() {
        let (slot, pa, bytes) = SLOTS[i];
        let delta = pa & (PAGE - 1);
        let map_bytes = (delta + bytes + PAGE - 1) & !(PAGE - 1);
        out[i] = IoMap {
            slot,
            pa: pa & !(PAGE - 1),
            map_bytes,
            va,
            delta,
        };
        va += map_bytes + IOMAP_GAP;
        i += 1;
    }
    out
}

/// HardwareDataB IOMapping records of the J516C.
pub(crate) const IOMAPS: [IoMap; 17] = iomaps();

/// The J516C initdata layout: the T6030 graph with the root version this firmware's host
/// driver sends and the longer HardwareData record, placed 16-byte aligned (the firmware's MTR
/// alarm handler does a 64-bit atomic in HardwareDataA; at the T6030 placement the record is
/// 4-byte aligned, which faulted on T8122) and mapped ASC-cached as a whole.
pub(crate) const LAYOUT: Layout = Layout {
    root_magic: 0x0c08e21e84820490,
    hwdata_va: 0xfffffc20406fb580,
    hwdata_size: 0xb480,
    hwdata_cached: None,
    hwdata_view: 0xb400,
    iomaps: &IOMAPS,
};

/// The J516C board images (`asahi/t6031-j516c-board.bin`, built into the kernel): the
/// hardware record, Globals and the power block that this machine's firmware accepted, as the
/// earlier J516C driver uploaded them (its device-tree-built records with the per-board words
/// of this machine applied). Kept out of the tree; pinned by its SHA-256.
const BOARD_SHA256: [u8; 32] = [
    0x4e, 0x03, 0xf2, 0x9f, 0xa4, 0x47, 0xdc, 0xeb, 0xfd, 0x4d, 0x1c, 0x22, 0x5a, 0x64, 0x7b,
    0xfa, 0x31, 0xb3, 0x63, 0x45, 0x8f, 0x28, 0x33, 0x82, 0x3a, 0x5c, 0xa8, 0xa1, 0x0e, 0xdf,
    0xf3, 0xfc,
];

pub(crate) const CONFIG: Config = Config {
    name: "T6031 G15C",
    chip_id: 0x6031,
    gpu_variant: crate::hw::GpuVariant::C,
    gpu_revision: 0x12,
    uat_ias: 42,
    uat_oas: 42,
    num_clusters: 4,
    num_cores_per_cluster: 10,
    hw_ids: &[HwId {
        version: 0x07041100,
        counts: 0x0021040a,
    }],
    // The stubs the boot chains of this J516C leave loaded (RTKit-2419.140.12): the macOS 14.7
    // chain's and the macOS 14.8 boot volume's. Same segment sizes and marker; the layout, board
    // images and initdata below do not depend on which one is loaded. A further build can be
    // admitted for one boot with `g15_accept_text_prefix`, which logs its digest.
    firmware: &[
        FirmwareBuild {
            name: "macOS 14.7 stub firmware (iBoot-10151.140.19.700.2), J516C",
            text_size: 0x64000,
            data_size: 0x140000,
            marker: 0x5cf28,
            text_sha256: [
                0xc2, 0x6e, 0xc8, 0x9c, 0x11, 0x50, 0xeb, 0x65, 0x49, 0xcf, 0x60, 0x2e, 0x01,
                0x2d, 0x0c, 0x5f, 0x49, 0xeb, 0x75, 0x6e, 0x89, 0x6d, 0x6b, 0xb8, 0x34, 0x2c,
                0xb2, 0xc5, 0xd9, 0x8b, 0x33, 0x2c,
            ],
        },
        FirmwareBuild {
            name: "macOS 14.8 stub firmware, J516C",
            text_size: 0x64000,
            data_size: 0x140000,
            marker: 0x5cf28,
            text_sha256: [
                0x1b, 0xf2, 0xfd, 0x34, 0x76, 0xbe, 0xfe, 0x73, 0x6b, 0xc2, 0xc0, 0x70, 0xba,
                0xd4, 0x5f, 0x8b, 0xe3, 0x6b, 0x68, 0xcd, 0x11, 0xbc, 0xff, 0x16, 0xc9, 0x02,
                0x9d, 0x6b, 0xcd, 0xa0, 0x26, 0xcf,
            ],
        },
    ],
    machine: kernel::c_str!("apple,t6031"),
    platform: super::Platform::DeviceTree,
    layout: &LAYOUT,
    board: super::Board::Blob {
        name: kernel::c_str!("asahi/t6031-j516c-board.bin"),
        sha256: BOARD_SHA256,
    },
    // Four clusters: the saved-state pointers at +0x2980 of the preemption buffer (0xd80 plus
    // 0x700 per cluster), UMA PTC 0x6000400020, DUPM 0x3c000000140 (32 N | 96 N << 32 for
    // N = 10 cores per cluster) and 0x1a0e9 = 0, with the saved-state pointers and the
    // 0x12091/0x101d9 words zero in every command, as this machine's driver encodes a queue's
    // first command. Read from 38 compute commands on a J516C; the two-cluster values fault
    // the first job on this GPU.
    compute: super::ComputeRegs {
        tail: 0x2980,
        ptc: 0x6000400020,
        dupm: 0x3c000000140,
        reg_1a0e9: 0,
        fresh: true,
    },
};
