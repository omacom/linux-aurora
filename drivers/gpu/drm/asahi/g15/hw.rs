// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! G15 hardware and firmware-ABI constants (layout values are shared by several modules; change
//! one only together with its users). The qualified T6030 values are the module-level
//! constants; what differs per SoC is a [`super::Config`] row ([`t6030`], [`t6031`]). The tuned
//! policy values are in [`tuning`].

use core::ops::Range;

pub(crate) mod t6031;

/// Identification register values of a qualified G15S.
pub(crate) struct HwId {
    /// SGX identity word (`sgx + 0xd04000`).
    pub(crate) version: u32,
    /// SGX layout word (`sgx + 0xd04010`).
    pub(crate) counts: u32,
}

/// A qualified GPU firmware build, identified by its loaded TEXT.
pub(crate) struct FirmwareBuild {
    pub(crate) name: &'static str,
    pub(crate) text_size: u64,
    pub(crate) data_size: u64,
    /// Offset of the GKTS marker in TEXT; the 8 entropy bytes after it are zeroed before hashing.
    pub(crate) marker: u32,
    pub(crate) text_sha256: [u8; 32],
}

/// Where a SoC's GPU regions, firmware segments and performance states come from, and how its
/// GPU is powered.
#[derive(Copy, Clone, PartialEq, Eq)]
pub(crate) enum Platform {
    /// T6030: the boot loader's ADT export ([`super::adt`]), a GPU node synthesized by the PMP
    /// handoff, and the PMP AGX vote through the J514S supplier.
    AdtWithPmp,
    /// The GPU node of the kernel device tree, filled by the boot loader (regions, firmware
    /// segments and operating points); the GPU is powered by its power domain alone.
    DeviceTree,
}

/// The source of the board images (HardwareData, Globals, Power) of a SoC.
#[derive(Copy, Clone, PartialEq, Eq)]
pub(crate) enum Board {
    /// The hash-checked T6030 templates ([`super::board`]).
    Templates,
    /// The three images as one firmware file ([`super::board::blob`]), pinned by its SHA-256.
    Blob {
        name: &'static kernel::str::CStr,
        sha256: [u8; 32],
    },
    /// No board data: admission refuses the SoC before anything firmware-visible.
    None,
}

/// Per-SoC placement and size of the initdata objects that differ between G15 firmware
/// builds; everything else keeps the canonical [`va`] values.
pub(crate) struct Layout {
    /// Root object version word.
    pub(crate) root_magic: u64,
    /// HardwareData VA, size, ASC-cached prefix (`None`: the whole object), and the offset of
    /// the fourth HardwareDataB view (fault information).
    pub(crate) hwdata_va: u64,
    pub(crate) hwdata_size: usize,
    pub(crate) hwdata_cached: Option<usize>,
    pub(crate) hwdata_view: usize,
    /// HardwareDataB IOMapping records.
    pub(crate) iomaps: &'static [IoMap],
}

/// Compute command register values that scale with the cluster count.
pub(crate) struct ComputeRegs {
    /// Offset of the four saved-state pointers in the preemption buffer.
    pub(crate) tail: u64,
    /// UMA PTC (0xa599) and DUPM (0xd411).
    pub(crate) ptc: u64,
    pub(crate) dupm: u64,
    /// The 0x1a0e9 word.
    pub(crate) reg_1a0e9: u64,
    /// Encode every command the way the first command of a fresh queue is encoded: zero
    /// saved-state pointers and zero 0x12091/0x101d9.
    pub(crate) fresh: bool,
}

/// The T6030 configuration.
pub(crate) mod t6030 {
    use super::super::Config;
    use super::{ComputeRegs, FirmwareBuild, HwId, Layout};

    /// The qualified graph: the canonical VAs and sizes of [`super::va`] and
    /// [`super::super::fw::initdata`].
    pub(crate) const LAYOUT: Layout = Layout {
        root_magic: super::super::fw::initdata::root::MAGIC,
        hwdata_va: super::va::HWDATA,
        hwdata_size: super::super::fw::initdata::hwdata::SIZE,
        hwdata_cached: Some(super::super::fw::initdata::hwdata::CACHED),
        hwdata_view: super::super::fw::initdata::hwdata::FAULT_INFO,
        iomaps: &super::IOMAPS,
    };

    /// The two-cluster compute register values.
    pub(crate) const COMPUTE: ComputeRegs = ComputeRegs {
        tail: 0x1480,
        ptc: 0x13200400020,
        dupm: 0x200000001,
        reg_1a0e9: 8,
        fresh: false,
    };

    pub(crate) const CONFIG: Config = Config {
        name: "T6030 G15S",
        chip_id: 0x6030,
        gpu_variant: crate::hw::GpuVariant::S,
        gpu_revision: 0x11,
        uat_ias: 42,
        uat_oas: 42,
        num_clusters: 2,
        num_cores_per_cluster: 10,
        hw_ids: &[HwId {
            version: 0x07031100,
            counts: 0x00110209,
        }],
        firmware: &[FirmwareBuild {
            name: "macOS 14.7-14.8.x stub firmware (iBoot-10151.140.19.700.2)",
            text_size: 0x5c000,
            data_size: 0x114000,
            marker: 0x58e88,
            text_sha256: [
                0x11, 0xe4, 0x9f, 0x75, 0x0b, 0x67, 0x1a, 0x2b, 0x13, 0xd0, 0x92, 0xdb, 0x1c, 0xbc,
                0xa3, 0xa8, 0x77, 0x55, 0x86, 0xd6, 0xb1, 0xe3, 0xaa, 0xe8, 0xa6, 0xa4, 0x64, 0xa7,
                0x80, 0x02, 0x93, 0xd5,
            ],
        }],
        machine: kernel::c_str!("apple,t6030"),
        platform: super::Platform::AdtWithPmp,
        layout: &LAYOUT,
        board: super::Board::Templates,
        compute: COMPUTE,
    };
}

/// User VA range reported as `vm_start..vm_end`. Not the full 42-bit range: the userspace VA
/// layout on G15S was qualified with this end.
pub(crate) const USER_RANGE: Range<u64> = 0x4000..0x2ffffff8000;

/// Bits per cluster in the enabled-core mask (`sgx + 0xe01500`).
pub(crate) const CLUSTER_BITS: u32 = 10;

/// UAT page size.
pub(crate) const PAGE: u64 = 0x4000;
/// Physical address bits of UAT leaves.
pub(crate) const PA_MASK: u64 = 0x3ffffffc000;

/// Firmware upper half: root[0..1] are firmware-owned, root[2] is the host window.
pub(crate) const UPPER: u64 = 0xfffffc0000000000;
pub(crate) const HOST: u64 = 0xfffffc2000000000;
pub(crate) const HOST_SIZE: u64 = 0x1000000000;

/// Firmware UAT leaf words. Shared-layer `Prot` values for them are built with
/// `mmu::Prot::from_pte` and must round-trip to these exact words.
pub(crate) mod pte {
    /// Firmware-only shared RW data, AttrIndex 2 (inner non-cacheable for the ASC).
    pub(crate) const FW: u64 = 0x00c000000000044b;
    /// Firmware + GPU shared, AttrIndex 2.
    pub(crate) const SHARED: u64 = 0x00e000000000040b;
    /// The same two classes with AttrIndex 0 (inner+outer write-back for the ASC).
    pub(crate) const FW_CACHED: u64 = 0x00c0000000000443;
    pub(crate) const SHARED_CACHED: u64 = 0x00e0000000000403;
    /// Firmware device RW.
    pub(crate) const MMIO: u64 = 0x00c0000000000447;
    /// Table descriptor bits.
    pub(crate) const TABLE: u64 = 3;
    /// Context-0 lower work alias: GPU RW NC, non-global.
    pub(crate) const LOWER: u64 = 0x00c0000000000c8b;
    /// Client (user VM) driver leaves: NC, cached, PB list.
    pub(crate) const LEAF_NC: u64 = 0x00c0000000000c8b;
    pub(crate) const LEAF_CACHED: u64 = 0x00c0000000000c83;
    pub(crate) const LEAF_PB_LIST: u64 = 0x00e0000000000c03;
}

/// PPL handoff magic.
pub(crate) const PPL_MAGIC: u64 = 0x4b1d000000000002;

/// UAT handoff region sizes of this firmware ABI (TTBAT page, TTBAT tables (the firmware kernel
/// root), PPL handoff page, shared L2 page); the addresses come from the ADT.
pub(crate) const TTBS_SIZE: u64 = 0x4000;
pub(crate) const TABLES_SIZE: u64 = 0x40000;
pub(crate) const HANDOFF_SIZE: u64 = 0x4000;
pub(crate) const SHARED_L2_SIZE: u64 = 0x4000;

/// Canonical initdata object VAs of the qualified build.
pub(crate) mod va {
    pub(crate) const ROOT: u64 = 0xfffffc204062ff40;
    pub(crate) const REGION_A: u64 = 0xfffffc2040670000;
    pub(crate) const RUNTIME: u64 = 0xfffffc20406b7b4d;
    pub(crate) const HWDATA: u64 = 0xfffffc20406fb5fc;
    pub(crate) const PAIR: u64 = 0xfffffc2040747f00;
    pub(crate) const SMALL: u64 = 0xfffffc204078bff0;
    pub(crate) const C0: u64 = 0xfffffc2070003000;
    pub(crate) const C1: u64 = 0xfffffc2070008000;
    pub(crate) const C3: u64 = 0xfffffc2070010000;
    pub(crate) const GLOBALS: u64 = 0xfffffc20407cc204;
    pub(crate) const POWER: u64 = 0xfffffc204081370c;
    pub(crate) const CONTROL: u64 = 0xfffffc20003bbc30;
    pub(crate) const FLAGS: u64 = 0xfffffc2040857fc0;
    pub(crate) const DEVCTL_STATE: u64 = 0xfffffc2040333fd0;
    pub(crate) const DEVCTL_RING: u64 = 0xfffffc2000330800;
    pub(crate) const EVENT_STATE: u64 = 0xfffffc2040377fd0;
    pub(crate) const EVENT_RING: u64 = 0xfffffc20403b8800;
    pub(crate) const LOG_STATE: u64 = 0xfffffc20403ffee0;
    pub(crate) const LOG_RING: u64 = 0xfffffc2040443000;
    pub(crate) const TRACE_STATE: u64 = 0xfffffc20404d7fd0;
    pub(crate) const TRACE_RING: u64 = 0xfffffc2040519000;
    pub(crate) const STATS_STATE: u64 = 0xfffffc2040563fd0;
    pub(crate) const STATS_RING: u64 = 0xfffffc20405a4000;
    pub(crate) const FWCTL_STATE: u64 = 0xfffffc20405ebfd0;
    pub(crate) const FWCTL_RING: u64 = 0xfffffc2000376c00;
    /// Firmware timestamp arena (64 MiB).
    pub(crate) const TIMESTAMPS: u64 = 0xfffffc2ffc000000;
    pub(crate) const TIMESTAMPS_SIZE: u64 = 0x4000000;
    /// Firmware VA allocator window for backend objects.
    pub(crate) const FW_ALLOC: core::ops::Range<u64> = 0xfffffc2d00000000..0xfffffc2e00000000;

    /// State block of submit channel `i` (12 channels).
    pub(crate) const fn ch_state(i: u64) -> u64 {
        0xfffffc2040003fd0 + i * 0x44000
    }
    /// Ring of submit channel `i`.
    pub(crate) const fn ch_ring(i: u64) -> u64 {
        0xfffffc2000002800 + i * 0x44000
    }

    /// Scratch page of the boot-time PTE class self-check (host window; never published).
    pub(crate) const SELF_CHECK: u64 = 0xfffffc2ff8000000;
    /// Scratch page of the PTE self-check in the context-0 lower half (never published).
    pub(crate) const SELF_CHECK_LOWER: u64 = 0x7ff0000000;
}

/// Firmware channels and rings of the initdata graph.
pub(crate) mod rings {
    /// Submit channels allocated (TA, FS and compute are used).
    pub(crate) const CHANNELS: u64 = 12;
    /// Bytes of a channel state block.
    pub(crate) const STATE_SIZE: usize = 0x30;
    /// Submit channel ring: 256 x 0x18.
    pub(crate) const CHANNEL_RING_SIZE: usize = 0x1800;
    /// Device-control and event rings: 256 x 0x38.
    pub(crate) const DEVCTL_RING_SIZE: usize = 0x3800;
    pub(crate) const EVENT_RING_SIZE: usize = 0x3800;
    /// Six log sub-rings: their state blocks and their rings.
    pub(crate) const LOG_RINGS: usize = 6;
    pub(crate) const LOG_STATE_SIZE: usize = 0x120;
    pub(crate) const LOG_RING_SIZE: usize = 0x51000;
    pub(crate) const TRACE_RING_SIZE: usize = 0x7000;
    pub(crate) const STATS_RING_SIZE: usize = 0x4000;
    /// FwCtl ring: 256 x 0x18 (the record stride of this firmware). Never used by the host: no
    /// firmware cache flush path exists.
    pub(crate) const FWCTL_RING_SIZE: usize = 0x1800;
    /// Device-control and event record sizes.
    pub(crate) const DEVCTL_RECORD: usize = 0x38;
    pub(crate) const EVENT_RECORD: usize = 0x38;
    /// Submit channel record size.
    pub(crate) const CHANNEL_RECORD: usize = 0x18;
}

/// One HardwareDataB IOMapping record: firmware device alias of an MMIO range.
#[derive(Copy, Clone)]
pub(crate) struct IoMap {
    pub(crate) slot: u8,
    pub(crate) pa: u64,
    pub(crate) map_bytes: u64,
    pub(crate) va: u64,
    /// Added to `va` in the HardwareData record.
    pub(crate) delta: u64,
}

/// HardwareDataB IOMapping records of the qualified build.
pub(crate) const IOMAPS: [IoMap; 15] = [
    IoMap {
        slot: 0,
        pa: 0x290d00000,
        map_bytes: 0x144000,
        va: 0xfffffc2068000000,
        delta: 0,
    },
    IoMap {
        slot: 1,
        pa: 0x20e100000,
        map_bytes: 0x4000,
        va: 0xfffffc2068148000,
        delta: 0x1000,
    },
    IoMap {
        slot: 2,
        pa: 0x351014000,
        map_bytes: 0x4000,
        va: 0xfffffc2068150000,
        delta: 0,
    },
    IoMap {
        slot: 3,
        pa: 0x290000000,
        map_bytes: 0x20000,
        va: 0xfffffc2068158000,
        delta: 0,
    },
    IoMap {
        slot: 7,
        pa: 0x3502bc000,
        map_bytes: 0x4000,
        va: 0xfffffc206817c000,
        delta: 0,
    },
    IoMap {
        slot: 9,
        pa: 0x290e08000,
        map_bytes: 0x8000,
        va: 0xfffffc2068184000,
        delta: 0,
    },
    IoMap {
        slot: 11,
        pa: 0x220000000,
        map_bytes: 0x12c000,
        va: 0xfffffc2068190000,
        delta: 0,
    },
    IoMap {
        slot: 12,
        pa: 0x35104c000,
        map_bytes: 0x4000,
        va: 0xfffffc20682c0000,
        delta: 0,
    },
    IoMap {
        slot: 18,
        pa: 0x3503d0000,
        map_bytes: 0x4000,
        va: 0xfffffc20682c8000,
        delta: 0,
    },
    IoMap {
        slot: 19,
        pa: 0x3503c0000,
        map_bytes: 0x4000,
        va: 0xfffffc20682d0000,
        delta: 0,
    },
    IoMap {
        slot: 20,
        pa: 0x3503d8000,
        map_bytes: 0x4000,
        va: 0xfffffc20682d8000,
        delta: 0,
    },
    IoMap {
        slot: 23,
        pa: 0x293000000,
        map_bytes: 0x400000,
        va: 0xfffffc20682e0000,
        delta: 0,
    },
    IoMap {
        slot: 25,
        pa: 0x30945c000,
        map_bytes: 0x4000,
        va: 0xfffffc20686e4000,
        delta: 0,
    },
    IoMap {
        slot: 26,
        pa: 0x350280000,
        map_bytes: 0x8000,
        va: 0xfffffc20686ec000,
        delta: 0,
    },
    IoMap {
        slot: 29,
        pa: 0x290e5c000,
        map_bytes: 0x4000,
        va: 0xfffffc20686f8000,
        delta: 0,
    },
];

/// RTKit endpoints and messages.
pub(crate) mod ep {
    /// Firmware endpoint: the initdata doorbell and ring-service requests.
    pub(crate) const FIRMWARE: u8 = 0x20;
    /// Doorbell endpoint: channel and device-control notifications.
    pub(crate) const DOORBELL: u8 = 0x21;
    /// Host message handing the firmware its initdata root.
    pub(crate) const INITDATA: u64 = 0x0081 << 48;
    /// Address bits of the initdata root carried by [`INITDATA`].
    pub(crate) const INITDATA_ADDR_MASK: u64 = 0xfffffffffff;
    /// Firmware message requesting that the receive rings be drained.
    pub(crate) const RING_SERVICE: u64 = 0x0042 << 48;
}

/// Fixed per-VM client placement, identical in every VM, inside [`CLIENT_APERTURE`], plus the
/// pass slots and PB page list in the compact window. User binds into any of these fail.
pub(crate) mod client {
    use core::ops::Range;

    /// Per-VM kernel objects (the hole the userspace VA layout leaves for the runtime graph).
    pub(crate) const CLIENT_APERTURE: Range<u64> = 0x7000000000..0x7100000000;
    /// Compact-pointer window that VDM stream bases must lie in.
    pub(crate) const COMPACT_WINDOW: Range<u64> = 0x1000000000..0x9000000000;

    pub(crate) const fn pool(k: u64) -> u64 {
        0x7000000000 + k * 0x404000
    }
    pub(crate) const FREELIST: u64 = 0x7005858000;
    pub(crate) const POOL_TABLE_PAGE: u64 = 0x7005c5c000;
    pub(crate) const POOL_TABLE_OFF: u64 = 0x3000;
    pub(crate) const PREEMPT: u64 = 0x7005c64000;
    pub(crate) const PREEMPT_STRIDE: u64 = 0x10000;
    pub(crate) const WORK_PAGE: u64 = 0x7021000000;
    pub(crate) const WORK_OFF: u64 = 0x3760;
    pub(crate) const CWORK_STRIDE: u64 = 0x4000;
    pub(crate) const FS_WORK_PAGE: u64 = 0x7020000000;
    pub(crate) const FS_WORK_OFF: u64 = 0x3380;
    pub(crate) const TA_WORK_PAGE: u64 = 0x7020010000;
    pub(crate) const TA_WORK_OFF: u64 = 0x36c0;
    pub(crate) const WORK_STRIDE: u64 = 0x20000;
    pub(crate) const fn uma(engine: u64, k: u64) -> u64 {
        0x7010000000 + engine * 0x2450000 + k * 0x408000
    }
    pub(crate) const fn uma_freelist(engine: u64) -> u64 {
        0x7012040000 + engine * 0x2450000
    }
    pub(crate) const fn uma_table_page(engine: u64) -> u64 {
        0x7012448000 + engine * 0x2450000
    }
    pub(crate) const AUX: u64 = 0x70148a0000;
    pub(crate) const AUX_STRIDE: u64 = 0x98000;
    pub(crate) const TPC: u64 = 0x7040000000;
    pub(crate) const TPC_SIZE: u64 = 0x1000000;
    pub(crate) const TILEMAP: u64 = 0x7042000000;
    pub(crate) const TILEMAP_SIZE: u64 = 0x800000;
    pub(crate) const TPC_SLOT_SIZE: u64 = 0x200000;
    pub(crate) const TILEMAP_SLOT_SIZE: u64 = 0x100000;
    pub(crate) const TPC_SLOT_STRIDE: u64 = 0x4000000;
    pub(crate) const CT: u64 = 0x7080000000;
    pub(crate) const CT_STRIDE: u64 = 0x1000000;
    pub(crate) const CT_SIZE: u64 = 0x400000;
    pub(crate) const PB_HEAP: u64 = 0x70c0000000;
    pub(crate) const PB_HEAP_SIZE: u64 = 0x10000000;
    pub(crate) const PB_BLOCK_LIST: u64 = 0x70e0000000;
    /// Compact-window objects (32-bit compact pointers: `gva - 0x1000000000`).
    pub(crate) const PRE0_PAGE: u64 = 0x1006080000;
    pub(crate) const PRE1_PAGE: u64 = 0x1006098000;
    pub(crate) const PRE2_PAGE: u64 = 0x10060b0000;
    pub(crate) const HEAP_META_PAGE: u64 = 0x1006140000;
    pub(crate) const SCENE_USER: u64 = 0x1006158000;
    pub(crate) const FS_SCRATCH: u64 = 0x1006178000;
    pub(crate) const PASS_STRIDE: u64 = 0x118000;
    pub(crate) const PB_PAGE_LIST: u64 = 0x1074000000;
    pub(crate) const PB_PAGE_LIST_SIZE: u64 = 0x28000;
}

/// Sizes and counts of the engine's objects.
pub(crate) mod sizes {
    pub(crate) const POOLS: usize = 22;
    pub(crate) const POOL_SIZE: u64 = 0x400000;
    pub(crate) const FREELIST_SEED: u32 = 8192;
    pub(crate) const QUEUE_CAP: u32 = 0x500;
    pub(crate) const COMPUTE_SLOTS: usize = 16;
    pub(crate) const RENDER_SLOTS: usize = 16;
    pub(crate) const CT_BUFS: usize = 2 * RENDER_SLOTS;
    pub(crate) const CLUSTER_PB_PAGES: u32 = 0x3c;
    pub(crate) const MAX_ATTACHMENTS: usize = 16;
    pub(crate) const MAX_CLIENT: usize = 320;
    pub(crate) const PB_PAGES: u32 = 8192;
    pub(crate) const PB_BLOCKS: u32 = 2048;
    pub(crate) const PB_LIST_RECORDS: u32 = 4096;
    pub(crate) const PB_GROW_STEP: u32 = 10;
    /// Flights (published prefixes) in flight at most; the second must be of the same VM.
    pub(crate) const FLIGHTS: usize = 2;
    /// Jobs per compute horizon, and scheduler credits per queue.
    pub(crate) const HORIZON_JOBS: u32 = 16;
    /// Unconsumed records per 256-entry channel ring a prefix may be posted behind.
    pub(crate) const AHEAD_RECORDS: u32 = 64;
    /// Policy bound on logical queues reported in QUEUE_LIMITS (not a firmware limit).
    pub(crate) const MAX_QUEUES: u32 = 58;
    /// TTBAT user slots: user VMs hold one each for their lifetime (ASIDs 1..63).
    pub(crate) const MAX_VMS: u32 = 63;
}

/// Firmware channels, doorbells, seeds and identifiers.
pub(crate) mod engine {
    pub(crate) const DOORBELL: u64 = 0x0083000000000000;
    pub(crate) const DEVCTL_DOORBELL: u64 = 0x0083000000000011;
    pub(crate) const EVENT_SLOT: u32 = 2;
    pub(crate) const COMPUTE_SEED: u32 = 0xc1000000;
    pub(crate) const TA_SEED: u32 = 0x7a000000;
    pub(crate) const FS_SEED: u32 = 0x3d000000;
    pub(crate) const COMPUTE_UUID: u32 = 0xc10000;
    pub(crate) const TA_UUID: u32 = 0x7a0000;
    pub(crate) const FS_UUID: u32 = 0x3d0000;
}

/// Tuned policy. Every value was chosen by measurement on hardware; only the ones in
/// `params.rs` are tunable.
pub(crate) mod tuning {
    /// Command objects (work images, microsequences, barriers, queue info, scheduling contexts,
    /// job lists, FW-only queue scratch) map ASC-cached (AttrIndex 0).
    pub(crate) const FW_CACHED: bool = true;
    /// The first HardwareData pages map ASC-cached.
    pub(crate) const HWDATA_CACHED: bool = true;
    /// Render notifier ASC-cached (bit 1), compute notifier not (bit 0).
    pub(crate) const NOTIFIER_CACHED: u32 = 2;
    /// UMA/PB managers, block_ctl and misc ASC-cached, with an uncached spare set.
    pub(crate) const MANAGER_CACHED: bool = true;
    /// Scheduling class of the render and compute queue-info rows (2: killable compute).
    pub(crate) const RENDER_QUEUE_CLASS: u32 = 2;
    pub(crate) const COMPUTE_QUEUE_CLASS: u32 = 2;
    /// Intra-prefix FS->TA barrier type.
    pub(crate) const RENDER_INTERNAL_HOLD: u32 = 0;
    /// Bit 0: FS->compute type 1; bit 1: compute->TA type 1. Never both type 0.
    pub(crate) const CROSS_ENGINE_HOLD: u32 = 1;
    /// Compute-only jobs folded into one prefix.
    pub(crate) const COMPUTE_HORIZON: u32 = 16;
    /// Renders per prefix (half of the pass slots), with submit-ahead.
    pub(crate) const PREFIX_RENDERS: u32 = 8;
    pub(crate) const SUBMIT_AHEAD: bool = true;
    /// Computes and renders in one prefix.
    pub(crate) const MIXED_PREFIX: bool = true;
    /// Two-cluster vertex clustering.
    pub(crate) const VERTEX_CLUSTERING: bool = true;
    /// Background zeroing of retired storage; also zero aux, heap and scene-user.
    pub(crate) const CT_ASYNC: bool = true;
    pub(crate) const PASS_FULL_RESET: bool = true;
    /// Rotating compute-slot cursor.
    pub(crate) const SLOT_ROTATE: bool = true;
    /// Flights per VM.
    pub(crate) const MAX_INFLIGHT: u32 = 2;
    /// PB growth through event 7 / DEVCTL 7, initial and maximum blocks.
    pub(crate) const PB_GROW: bool = true;
    pub(crate) const PB_BLOCKS: u32 = super::sizes::PB_BLOCKS / 2;
    pub(crate) const PB_MAX_BLOCKS: u32 = super::sizes::PB_BLOCKS;
    /// At the host deadline, arm the firmware kill.
    pub(crate) const HANG_PREEMPT: bool = true;
    /// Compute start-sequence MMU fault mask (unmapped stores absorbed).
    pub(crate) const COMPUTE_FAULT_MASK: u32 = 1;
    /// Firmware idle decision: finite delay, not predictive.
    pub(crate) const IDLE_PREDICTIVE: bool = false;
    /// Handle EP 0x20 drain requests in the mailbox interrupt.
    pub(crate) const EARLY_WAKE: bool = true;
    /// Scheduler timeout per job, ms; the engine's deadlines own failure (always "no hang").
    pub(crate) const SCHED_TIMEOUT_MS: u32 = 3000;
}
