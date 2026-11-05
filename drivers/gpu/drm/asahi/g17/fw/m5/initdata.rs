// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! M5 firmware initialization encodings and allocation layout.
//! Opaque initialized words have no inferred meaning. Runtime ownership is
//! provided by the common G17 KernelObject and GPUVM paths.

#![allow(dead_code)]

use core::ops::Range;
use core::result::Result;

pub(crate) const ROOT_SIZE_PRIMARY: usize = 0x0b8;

pub(crate) const ROOT_SIZE_SECONDARY: usize = 0x0c8;

pub(crate) const MAIN_CONFIG_SIZE: usize = 0x600;

pub(crate) const G17P_QOS_RESOURCE_SIZE: usize = 0xe40;

pub(crate) const REGION_C_SIZE: usize = 0x1000;

pub(crate) const STATUS_BLOCK_SIZE: usize = 0x80;

pub(crate) const HW_DATA_BUNDLE_MIN_ALLOC: usize = 0x10000;

pub(crate) const INITDATA_VERSION_WORD: u64 = 0x0c8a_a322_0396_04c0;

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) enum RootProfile {
    J700G17P,

    J813G17G25G76,

    J873G18G26A428,

    J714G17S26A428,
}

impl RootProfile {
    pub(crate) const fn version_word(self) -> u64 {
        match self {
            Self::J700G17P => INITDATA_VERSION_WORD,
            Self::J813G17G25G76 => 0x0c8a_a322_039e_04c0,
            Self::J873G18G26A428 => COMMON26_INITDATA_VERSION_WORD,
            Self::J714G17S26A428 => M5_INITDATA_VERSION_WORD,
        }
    }

    pub(crate) const fn root_size(self, role: InstanceRole) -> usize {
        match self {
            Self::J873G18G26A428 => COMMON26_ROOT_SIZE,
            Self::J714G17S26A428 => M5_ROOT_SIZE,
            Self::J700G17P | Self::J813G17G25G76 => root_size(role),
        }
    }

    pub(crate) const fn control_opening_opcode(self, role: InstanceRole) -> u32 {
        match (self, role) {
            (Self::J873G18G26A428, InstanceRole::Primary) => {
                COMMON26_CONTROL_OPENING_OPCODE_PRIMARY
            }
            (Self::J873G18G26A428, InstanceRole::Secondary) => {
                COMMON26_CONTROL_OPENING_OPCODE_SECONDARY
            }
            (Self::J714G17S26A428, InstanceRole::Primary) => M5_CONTROL_OPENING_OPCODE_PRIMARY,
            (Self::J714G17S26A428, InstanceRole::Secondary) => {
                M5_CONTROL_OPENING_OPCODE_SECONDARY
            }
            (_, InstanceRole::Primary) => CONTROL_OPENING_OPCODE_PRIMARY,
            (_, InstanceRole::Secondary) => CONTROL_OPENING_OPCODE_SECONDARY,
        }
    }

    pub(crate) const fn control_register_opcode(self) -> Option<u32> {
        match self {
            Self::J873G18G26A428 | Self::J714G17S26A428 => None,
            Self::J700G17P | Self::J813G17G25G76 => Some(CONTROL_OPENING_PRIMARY_REGISTER_OPCODE),
        }
    }

    pub(crate) const fn bundle_view_offsets(self) -> [usize; 5] {
        match self {
            Self::J873G18G26A428 => COMMON26_BUNDLE_VIEW_OFFSETS,
            Self::J714G17S26A428 => M5_BUNDLE_VIEW_OFFSETS,
            Self::J700G17P | Self::J813G17G25G76 => BUNDLE_VIEW_OFFSETS,
        }
    }

    pub(crate) const fn bundle_view_extents(self) -> [usize; 5] {
        match self {
            Self::J873G18G26A428 => COMMON26_BUNDLE_VIEW_EXTENTS,
            Self::J714G17S26A428 => M5_BUNDLE_VIEW_EXTENTS,
            Self::J700G17P | Self::J813G17G25G76 => BUNDLE_VIEW_EXTENTS,
        }
    }

    pub(crate) const fn repeated_region_bundle_offset(self) -> usize {
        match self {
            Self::J873G18G26A428 => COMMON26_SHARED_LAYOUT.block_88,
            Self::J714G17S26A428 => M5_SHARED_LAYOUT.block_88,
            Self::J700G17P | Self::J813G17G25G76 => MAIN_REPEATED_REGION_BUNDLE_OFFSET,
        }
    }

    pub(crate) const fn work_ring_bundle_offsets(self) -> [usize; WORK_CHANNEL_COUNT] {
        match self {
            Self::J873G18G26A428 => COMMON26_WORK_RING_BUNDLE_OFFSETS,
            Self::J714G17S26A428 => M5_WORK_RING_BUNDLE_OFFSETS,
            Self::J700G17P | Self::J813G17G25G76 => WORK_RING_BUNDLE_OFFSETS,
        }
    }
}

const INITDATA_ADDR_MASK: u64 = 0x0000_0fff_ffff_ffff;

const INITDATA_ADDR_PREFIX: u64 = 0xffff_f000_0000_0000;

const ROOT_VERSION: usize = 0x00;

const ROOT_REGION_A: usize = 0x08;

const ROOT_MAIN_CONFIG: usize = 0x18;

const ROOT_REGION_C: usize = 0x20;

const ROOT_INSTANCE_KIND: usize = 0x28;

const ROOT_FIXED_WORD: usize = 0x2c;

const ROOT_PAGE_SIZE: usize = 0x30;

const ROOT_PAGE_BITS: usize = 0x32;

const ROOT_LEVEL_COUNT: usize = 0x33;

const ROOT_LEVELS: usize = 0x34;

const ROOT_STATUS_A: usize = 0xa8;

const ROOT_STATUS_B: usize = 0xb0;

const ROOT_SECONDARY_EXTRA_0: usize = 0xb8;

const ROOT_SECONDARY_EXTRA_1: usize = 0xc0;

pub(crate) const UAT_PAGE_SIZE: u16 = 0x4000;

pub(crate) const UAT_PAGE_BITS: u8 = 14;

pub(crate) const UAT_LEVEL_COUNT: usize = 3;

pub(crate) const UAT_LEVELS: [(u8, u16); UAT_LEVEL_COUNT] = [(36, 64), (25, 2048), (14, 2048)];

pub(crate) const UAT_LEVEL_DESC_SIZE: usize = 0x20;

pub(crate) const UAT_LEVEL_PHYS_MASK: u64 = 0x0000_03ff_ffff_c000;

const MAIN_HW_DATA: usize = 0x00;

const MAIN_REPEATED_0: usize = 0x08;

const MAIN_REPEATED_1: usize = 0x10;

pub(crate) const MAIN_REPEATED_REGION_BUNDLE_OFFSET: usize = 0xc500;

const MAIN_CHANNEL_TABLE: usize = 0x20;

const MAIN_BUNDLE_VIEWS: usize = 0x254;

const MAIN_REGION_VIEWS: usize = 0x2d0;

const MAIN_SECONDARY_KIND: usize = 0x300;

const MAIN_SECONDARY_KIND_VALUE: u32 = 4;

const MAIN_PRIMARY_MASK: usize = 0x3e0;

const MAIN_PRIMARY_MASK_VALUE: u32 = 0xff;

const MAIN_PRIMARY_SENTINEL_A: usize = 0x32c;

const MAIN_PRIMARY_SENTINEL_B: usize = 0x344;

const MAIN_SECONDARY_EXTRA_PTR: usize = 0x471;

pub(crate) const SECONDARY_EXTRA_VIEW_INDEX: usize = 2;

pub(crate) const SECONDARY_EXTRA_VIEW_OFFSET: usize = 0xe40;

const MAIN_CONTROL_RING: usize = 0x4c0;

pub(crate) const CONTROL_OPENING_OPCODE_PRIMARY: u32 = 0x16;

pub(crate) const CONTROL_OPENING_OPCODE_SECONDARY: u32 = 0x2a;

pub(crate) const CONTROL_RECORD_SIZE: usize = 0x40;

pub(crate) const CONTROL_OPENING_PRIMARY_PRODUCER: u32 = 1;

pub(crate) const CONTROL_OPENING_SECONDARY_PRODUCER: u32 = 1;

pub(crate) const CONTROL_OPENING_PRIMARY_REGISTER_OPCODE: u32 = 0x20;

pub(crate) const BUNDLE_VIEW_OFFSETS: [usize; 5] = [0x2740, 0x3380, 0x4400, 0xbc80, 0xbd00];

pub(crate) const BUNDLE_VIEW_EXTENTS: [usize; 5] = [0x18c0, 0x0c80, 0x3c00, 0x3380, 0x3300];

pub(crate) const REGION_VIEW_LOW_ADDRS: [u64; 2] = [0x70_0183_8000, 0x70_0184_0000];

pub(crate) const CHANNEL_TABLE_ENTRIES: usize = 17;

pub(crate) const CHANNEL_ENTRY_SIZE: usize = 0x20;

pub(crate) const CHANNEL_ENTRY_STATE_COUNT: usize = 3;

pub(crate) const CHANNEL_STATE_SPACING: usize = 0x10;

pub(crate) const CHANNEL_STATE_PRODUCER: usize = 2;

pub(crate) const WORK_CHANNEL_COUNT: usize = 12;

pub(crate) const CONTROL_CHANNEL_INDEX: usize = 12;

pub(crate) const REPORT_CHANNEL_INDICES: [usize; 2] = [13, 14];

pub(crate) const PARTIAL_CHANNEL_INDEX: usize = 15;

pub(crate) const WORK_STATE_GRID_OFFSETS: [usize; 13] = [
    0x0c0, 0x1c0, 0x2c0,
    0x080, 0x180, 0x280,
    0x040, 0x140, 0x240,
    0x000, 0x100, 0x200,
    0x300,
];

pub(crate) const REPORT_STATE_STATUS_A_OFFSETS: [[usize; 3]; 2] =
    [[0x00040, 0x002c0, 0x00080], [0x00240, 0xa6ac0, 0x00280]];

pub(crate) const REPORT_RING_STATUS_A_OFFSETS: [usize; 2] = [0x04ac0, 0xafac0];

pub(crate) const PARTIAL_STATE_STATUS_A_OFFSET: usize = 0x2d2c0;

pub(crate) const WORK_RING_BUNDLE_OFFSETS: [usize; 12] = [
    0x10dc0, 0x16dc0, 0x1cdc0, 0x0f5c0, 0x155c0, 0x1b5c0, 0x0ddc0, 0x13dc0, 0x19dc0, 0x0c5c0,
    0x125c0, 0x185c0,
];

pub(crate) const HW_REGISTER_MAP_OFFSET: usize = 0x640;

pub(crate) const HW_REGISTER_MAP_ENTRY_SIZE: usize = 0x28;

pub(crate) const HW_REGISTER_APERTURE: Range<u64> = 0xffff_fc21_8000_0000..0xffff_fc21_9000_0000;

pub(crate) const G17P_KSM_COMPLETION_DESCRIPTOR_STRIDE: usize = 0x20;

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) struct G17PKsmCompletionAliases {
    pub(crate) low: u64,
    pub(crate) high: u64,
}

pub(crate) const M5_25G76_COMPLETION_QUEUE_COUNT: usize = 4;

pub(crate) const M5_25G76_COMPLETION_BACKING_SIZE: usize = 0x800 * 0x40;

pub(crate) fn encode_m5_25g76_completion_table(
    aliases: [G17PKsmCompletionAliases; M5_25G76_COMPLETION_QUEUE_COUNT],
    out: &mut [u8],
) -> Result<(), InitdataError> {
    if out.len() != M5_25G76_COMPLETION_QUEUE_COUNT * 0x20 {
        return Err(InitdataError::BufferSize);
    }
    let extent = M5_25G76_COMPLETION_BACKING_SIZE as u64;

    for (i, alias) in aliases.iter().enumerate() {
        for address in [alias.low, alias.high] {
            required_address(address)?;
            let end = address.checked_add(extent).ok_or(InitdataError::AddressOutOfRange)?;
            required_address(end - 1)?;
        }
        for prior in &aliases[..i] {
            if (alias.low < prior.low + extent && prior.low < alias.low + extent)
                || (alias.high < prior.high + extent && prior.high < alias.high + extent)
            {
                return Err(InitdataError::PairAliasedObject);
            }
        }
    }
    out.fill(0);
    for (ordinal, alias) in aliases.iter().enumerate() {
        let offset = ordinal * 0x20;
        put_u64(out, offset, alias.low);
        put_u64(out, offset + 8, alias.high);
        put_u32(out, offset + 0x10, 0x800);
        put_u32(out, offset + 0x14, 0x40);
        put_u32(out, offset + 0x18, ordinal as u32);
    }
    Ok(())
}

pub(crate) const STATUS_BLOCK_ONE_OFFSET: usize = 0x04;

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[repr(u32)]
pub(crate) enum InstanceRole {

    Primary = 0,

    Secondary = 1,
}

pub(crate) const fn root_size(role: InstanceRole) -> usize {
    match role {
        InstanceRole::Primary => ROOT_SIZE_PRIMARY,
        InstanceRole::Secondary => ROOT_SIZE_SECONDARY,
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) struct SharedAddresses {

    pub(crate) region_a: u64,

    pub(crate) region_c: u64,

    pub(crate) hw_data_bundle: u64,

    pub(crate) hw_data_bundle_alloc: usize,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) struct InstanceAddresses {

    pub(crate) root: u64,
    pub(crate) main_config: u64,
    pub(crate) status_a: u64,

    pub(crate) status_b: u64,

    pub(crate) secondary_extras: [u64; 2],
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) struct ChannelEntry {
    pub(crate) states: [u64; 3],
    pub(crate) ring: u64,
}

impl ChannelEntry {
    pub(crate) const EMPTY: ChannelEntry = ChannelEntry {
        states: [0; 3],
        ring: 0,
    };
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) struct PrimaryRegionViews {
    pub(crate) sentinel_high: u64,
    pub(crate) alias_high: [u64; 2],
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) struct RegisterWindow {
    pub(crate) slot: usize,
    pub(crate) phys: u64,
    pub(crate) device_va: u64,

    pub(crate) size: u32,

    pub(crate) unk_18: u64,
    pub(crate) flag: u32,
}

pub(crate) const M5_25G76_HW_DATA_SIZE: usize = 0x2708;

pub(crate) const M5_25G76_AON_PTD_SIZE: usize = 0xe4;

pub(crate) fn encode_m5_25g76_aon_ptd(out: &mut [u8]) -> Result<(), InitdataError> {
    if out.len() != M5_25G76_AON_PTD_SIZE {
        return Err(InitdataError::BufferSize);
    }
    out.fill(0);
    put_u32(out, 0, 0x6730_5452);
    put_u32(out, 4, 0x001d_0001);
    put_u32(out, 8, 0x0000_1401);
    Ok(())
}

const M5_25G76_STATUS_CONFIG_WORDS: &[(usize, u32)] = &[
    (0x010, 0x0000001f), (0x014, 0x3e0a3d71), (0x018, 0x401ae148),
    (0x01c, 0x00000028), (0x020, 0x00000001), (0x0c8, 0x00002af8),
    (0x0cc, 0x00001f40), (0x0d8, 0x41980000), (0x0dc, 0x3d0f5c28),
    (0x0f0, 0x000026ac), (0x0f4, 0x000000c8), (0x108, 0x42c80000),
    (0x10c, 0x43c80000), (0x110, 0x000000c8), (0x128, 0x00000001),
    (0x12c, 0x00005eb0), (0x130, 0x3e800000), (0x134, 0x3f99999a),
    (0x158, 0x0000000e), (0x1c0, 0x00000001), (0x1c4, 0x00000001),
    (0x1c8, 0x00000002), (0x1cc, 0x00000001), (0x1d0, 0x00000001),
    (0x1d4, 0x00000001), (0x1d8, 0x00000001), (0x588, 0x00000001),
    (0x58c, 0x0000000a), (0x590, 0x00000dac), (0x594, 0x000003e8),
    (0x598, 0x00000bb8), (0x59c, 0x000002bc), (0x5cc, 0x00000001),
    (0x5d0, 0x00000004), (0x5dc, 0x00001f40), (0x5e0, 0x000000c8),
    (0x5e4, 0x00000fa0), (0x5e8, 0x0000012c), (0x5ec, 0x000007d0),
    (0x5f0, 0x0000012c), (0x5f4, 0x000003e8), (0x5f8, 0x0000012c),
    (0x600, 0x00000001), (0x604, 0x41700000), (0x608, 0x40a00000),
    (0x60c, 0x00000020), (0x610, 0x000004b0), (0x620, 0x0000044c),
];

#[derive(Clone, Copy)]
pub(crate) struct M5PolicyBindings {
    pub state: u64,
    pub aon_ptd: u64,
    pub boot_timestamp: u64,
}

const M5_25G76_POLICY_WORDS: &[(usize, u32)] = &[
    (0x3d98, 0xffffffff), (0x3db0, 0xffffffff), (0x5244, 0x00030d40),
    (0x5248, 0x00030d40), (0x5250, 0x00000004), (0x5254, 0x3f800000),
    (0x526c, 0x00000001), (0x5270, 0x00000001), (0x5280, 0x00000064),
    (0x5284, 0x00000001), (0x5288, 0x000004b0), (0x5290, 0x00000064),
    (0x52a0, 0x00000004), (0x52a4, 0x3f800000), (0x52ac, 0x00000001),
    (0x52b0, 0x00000001), (0x52c0, 0x00000064), (0x52c4, 0x00000001),
    (0x52c8, 0x000004b0), (0x52d0, 0x00000064), (0x5300, 0x3f828f5c),
    (0x5304, 0x3f828f5c), (0x5308, 0x3f828f5c), (0x530c, 0x3f828f5c),
    (0x5310, 0x3f828f5c), (0x5314, 0x3f828f5c), (0x5318, 0x3f828f5c),
    (0x531c, 0x3f828f5c), (0x5320, 0x3f828f5c), (0x5324, 0x3f828f5c),
    (0x5328, 0x3f828f5c), (0x532c, 0x3f828f5c), (0x5330, 0x3f828f5c),
    (0x5bf8, 0x00000271), (0x5c04, 0x3f7f2e9f), (0x5c0c, 0x3b516154),
    (0x5c14, 0x3ca59586), (0x5c20, 0x46bd6000), (0x5c24, 0x40a90fe4),
    (0x5c2c, 0xbd4ac5b4), (0x5c30, 0x00000028), (0x5c34, 0x000004b0),
    (0x5c38, 0x000004b0), (0x5c44, 0x00005eb0), (0x5c78, 0x000004b0),
    (0x5cb0, 0x00000008), (0x5cbc, 0x3f000000), (0x5cc4, 0x3f000000),
    (0x5ccc, 0x3f99999a), (0x5cd8, 0x47800000), (0x5cdc, 0x3e800000),
    (0x5ce8, 0x00000028), (0x5cec, 0x000004b0), (0x5cfc, 0x00005eb0),
    (0x5d08, 0x00000010), (0x5d10, 0x0005dc00), (0x5d68, 0x0000004a),
    (0x5d70, 0x0000005a), (0x5d74, 0x00000032), (0x5d78, 0x00000006),
    (0x5d80, 0x00000006), (0x5d84, 0x00000001), (0x5d8c, 0x3f4ccccd),
    (0x5d90, 0x3f7eb852), (0x5d94, 0x3e4ccccd), (0x5d98, 0x3ba3d70a),
    (0x5d9c, 0x3f144b3c), (0x5da0, 0x3f144b3c), (0x5da8, 0x42be0000),
    (0x5dac, 0x4029eb85), (0x5db0, 0x4029eb85), (0x5db4, 0x4139435e),
    (0x5db8, 0x00000064), (0x5dbc, 0x000004b0), (0x5dc0, 0x00000064),
    (0x5dcc, 0x0000004a), (0x5dd4, 0x00000001), (0x5e00, 0x00000064),
    (0x5e20, 0x00000401), (0x5e24, 0x00006400), (0x5e2c, 0x001f4000),
    (0x5e30, 0x0000c800), (0x5e34, 0x000fa000), (0x5e38, 0x00012c00),
    (0x5e3c, 0x0007d000), (0x5e40, 0x00012c00), (0x5e44, 0x0003e800),
    (0x5e48, 0x00012c00), (0x5e4c, 0x00000100), (0x5ea4, 0x00000001),
    (0x5ed0, 0x00000001), (0x74d8, 0x47800000), (0x74dc, 0x42200000),
    (0x74e0, 0x44960000), (0x74e4, 0x0000062a), (0x74f8, 0x00000028),
    (0x7500, 0x42c80000), (0x7504, 0x000004b0), (0x750c, 0x3f4ccccd),
    (0x7510, 0x3e4ccccd), (0x7550, 0xdcdcfbfa), (0x7558, 0x00001f40),
    (0x7560, 0x00000004), (0x7574, 0x3f800000), (0x757c, 0x3d0f5c28),
    (0x7588, 0x47800000), (0x758c, 0x41980000), (0x7598, 0x00000028),
    (0x759c, 0x000004b0), (0x75a0, 0x000004b0), (0x75ac, 0x00002af8),
    (0x75e0, 0x000004b0), (0x77f8, 0x00000002), (0x77fc, 0x42000006),
    (0x7a10, 0xb8520000), (0x7a14, 0x00003f7e), (0x7a18, 0xd70a0000),
    (0x7a1c, 0x00003ba3), (0x7a24, 0x000043c8), (0x7a30, 0x00004780),
    (0x7a34, 0x000042c8), (0x7a3c, 0x0384bac8), (0x7a40, 0x03e80000),
    (0x7a44, 0x03e80000), (0x7a4c, 0x80000000), (0x7a50, 0x26ac453b),
    (0x7a84, 0x03e80000), (0x7aa0, 0x00c80000), (0x7b58, 0x000004b0),
    (0x7b6c, 0x00000001), (0x7ba0, 0x00000101), (0x7ba4, 0x00000200),
    (0x7bbc, 0x00023f00), (0x7bc0, 0x00000100), (0x7bc4, 0x00000100),
    (0x7bdc, 0x00024000), (0x7be0, 0x00000100), (0x7be4, 0x00000100),
    (0x7bfc, 0x00024100), (0x7c48, 0x00000001), (0x7c50, 0x00005eb0),
    (0x7c54, 0x00005eb0), (0x7c58, 0x00005eb0), (0x7c68, 0x3f800000),
    (0x7c6c, 0x00000004), (0x7c70, 0x00000008), (0x7c74, 0x0002ee00),
    (0x7d44, 0x0000007c), (0x7d50, 0x3f77bdef), (0x7d58, 0x3d042108),
    (0x7d60, 0x3e0a3d71), (0x7d6c, 0x47800000), (0x7d70, 0x401ae148),
    (0x7d7c, 0x00000028), (0x7d80, 0x000004b0), (0x7d84, 0x000004b0),
    (0x7d8c, 0x46bd6000), (0x7d90, 0x00005eb0), (0x7d9c, 0x000000f8),
    (0x7da4, 0x005ad200), (0x7dc4, 0x000004b0), (0x7e04, 0x00000001),
    (0x7ed4, 0x00000032), (0x7ed8, 0x00000001), (0x7ee0, 0x3f638e39),
    (0x7ee4, 0x3f2aaaab), (0x7ee8, 0x3de38e39), (0x7eec, 0x3eaaaaab),
    (0x7ef0, 0xbecccccd), (0x7ef4, 0xbf4ccccd), (0x7efc, 0x47800000),
    (0x7f00, 0xc0a00000), (0x7f04, 0xc0a00000), (0x7f0c, 0x00000064),
    (0x7f10, 0x000004b0), (0x7f14, 0x00000064), (0x7f1c, 0x45fa0000),
    (0x7f20, 0x00000578), (0x7f28, 0x0000000e), (0x7f2c, 0x00000048),
    (0x7f30, 0x00000018), (0x7f34, 0x001a5e00), (0x7f3c, 0x0008ca00),
    (0x7f48, 0x47800000), (0x7f70, 0x00001f40), (0x7f74, 0x00000004),
    (0x7f78, 0x000009c4), (0x7f7c, 0x00000258), (0x7f80, 0x00000002),
    (0x7f84, 0x00000004), (0x7f88, 0x00000032), (0x7fb8, 0x47800000),
    (0x7fc8, 0x00000028), (0x7fcc, 0x000004b0), (0x8098, 0x00000001),
    (0x80ac, 0x0000000a), (0x80b8, 0x00000001), (0x8174, 0xac000000),
    (0x8178, 0xe800000d), (0x817c, 0xb8000003), (0x8180, 0xb000000b),
    (0x8184, 0xb0000004), (0x8188, 0xbc000004), (0x818c, 0x20000002),
    (0x8198, 0x00000001), (0x81b4, 0x00000004), (0x81c0, 0x3f780000),
    (0x81c8, 0x3d000000), (0x81d0, 0x40a00000), (0x81dc, 0x47800000),
    (0x81e0, 0x41700000), (0x81ec, 0x0000044c), (0x81f0, 0x000004b0),
    (0x81f4, 0x000004b0), (0x8200, 0x000004b0), (0x8234, 0x000004b0),
    (0x827c, 0x00000064), (0x8290, 0xdcdcfbfa), (0x8298, 0x0000007d),
    (0x829c, 0x00000001), (0x8ee0, 0xdcdcfbfb), (0x8ee8, 0x00000001),
    (0xaeec, 0x41c80000), (0xaef0, 0x41c80000), (0xb6a8, 0x67305452),
    (0xb6e8, 0x00000001), (0xb728, 0x5243464c), (0xb72c, 0x4758464c),
    (0xb738, 0x47584c50), (0xb748, 0x00000001), (0xb7a8, 0x00000001),
    (0xb7b4, 0x42b73333), (0xb7f4, 0x449b8000), (0xb834, 0x42bc0000),
    (0xb8e0, 0x0000186a),
];

const M5_25G76_HW_PLATFORM_WORDS: &[(usize, u32)] = &[
    (0x0004, 0x0000006f), (0x0008, 0xffc00000), (0x0014, 0x00000010),
    (0x001c, 0x00000010), (0x0020, 0xffff8000), (0x0024, 0x000002ff),
    (0x0028, 0x81400000), (0x002c, 0xfffffc21), (0x00e0, 0x00002008),
    (0x00e8, 0x20080000), (0x00f4, 0x00002008), (0x00f8, 0x000024cb),
    (0x00fc, 0xc98a2cfa), (0x0100, 0xf4f624cb), (0x0104, 0x1877e917),
    (0x0108, 0x38d924cb), (0x010c, 0xbdab0000), (0x02d8, 0xfff82000),
    (0x02dc, 0xd32d2cdb), (0x02e0, 0xf5002000), (0x02e4, 0x21dae926),
    (0x02e8, 0x38b62000), (0x02ec, 0xc7420008), (0x03e0, 0x00007fe0),
    (0x03e8, 0x7fe00000), (0x03f4, 0x00007fe0), (0x03f8, 0x00008000),
    (0x0400, 0x80000000), (0x040c, 0x00008000), (0x05d8, 0x4b232645),
    (0x05dc, 0x00000e98), (0x05e0, 0xd5a2ea5f), (0x05e4, 0x40003fff),
    (0x05e8, 0xca5e4000), (0x05ec, 0x4000f5a2), (0x0e90, 0x00008142),
    (0x0e94, 0x00000002), (0x0ea0, 0x00000001), (0x0ea4, 0x00000001),
    (0x0eb8, 0x00000001), (0x0ec0, 0x00000001), (0x0ec8, 0x00000001),
    (0x0ed0, 0x00005dc0), (0x0ed4, 0x00000001), (0x0ed8, 0x00000008),
    (0x0ee0, 0x00000001), (0x0ee4, 0x00000001), (0x0ee8, 0x00000001),
    (0x0f04, 0x0000001f), (0x0f24, 0x07800004), (0x0f28, 0x0000000c),
    (0x0f34, 0x00000001), (0x0f38, 0x00000001), (0x0f4c, 0x00000031),
    (0x0f6c, 0x00000001), (0x0f88, 0x0000000a), (0x0f8c, 0x0000000f),
    (0x0fac, 0x00000001), (0x0fb8, 0x00000020), (0x0fbc, 0x00000005),
    (0x0fc0, 0x0000000a), (0x0fc4, 0x0000000c), (0x2544, 0x00000004),
    (0x2548, 0x00000006), (0x254c, 0x00000003), (0x2550, 0x00000007),
    (0x2554, 0x00000007), (0x2568, 0x0000000a), (0x2570, 0x00000001),
    (0x258c, 0x00180000), (0x2594, 0x00000001), (0x2598, 0x00000001),
    (0x259c, 0x00000001), (0x25a4, 0x00000001), (0x25a8, 0x00000001),
    (0x25ac, 0x00000001), (0x25b4, 0xffffffff), (0x25b8, 0xffffffff),
    (0x25bc, 0xffffffff), (0x25c0, 0xffffffff), (0x25c4, 0xffffffff),
    (0x25c8, 0xffffffff), (0x25cc, 0xffffffff), (0x25d0, 0xffffffff),
    (0x25d4, 0xffffffff), (0x25d8, 0xffffffff), (0x25dc, 0xffffffff),
    (0x25e0, 0xffffffff), (0x25f4, 0x00000001), (0x2600, 0x00000001),
    (0x2608, 0x00000001), (0x2610, 0x00000100), (0x26b4, 0x00000001),
    (0x26b8, 0x00000001), (0x26e0, 0x000003e8), (0x26e8, 0x00000001),
];

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) enum InitdataError {
    BufferSize,
    AddressOutOfRange,
    NullRequiredAddress,
    RoleSpecificAddress,
    BundleAllocationTooSmall,
    ChannelTableShape,
    RegisterWindow,
    PairRootSeparation,
    PairAliasedObject,
    SharedRegionSeparation,
    InterfaceMismatch,
    HostMappedAllocationsDisabled,
}

fn address_representable(address: u64) -> bool {
    let high = address & !INITDATA_ADDR_MASK;
    high == 0 || high == INITDATA_ADDR_PREFIX
}

fn required_address(address: u64) -> Result<(), InitdataError> {
    if address == 0 {
        Err(InitdataError::NullRequiredAddress)
    } else if !address_representable(address) {
        Err(InitdataError::AddressOutOfRange)
    } else {
        Ok(())
    }
}

fn put_u16(bytes: &mut [u8], offset: usize, value: u16) {
    bytes[offset..offset + 2].copy_from_slice(&value.to_le_bytes());
}

fn put_u32(bytes: &mut [u8], offset: usize, value: u32) {
    bytes[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
}

fn put_u64(bytes: &mut [u8], offset: usize, value: u64) {
    bytes[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
}

fn write_runs(bytes: &mut [u8], runs: &[(usize, &[u8])]) {
    for (offset, data) in runs {
        bytes[*offset..*offset + data.len()].copy_from_slice(data);
    }
}

fn validate_shared(shared: &SharedAddresses) -> Result<(), InitdataError> {
    required_address(shared.region_a)?;
    required_address(shared.region_c)?;
    required_address(shared.hw_data_bundle)?;
    if shared.hw_data_bundle_alloc < HW_DATA_BUNDLE_MIN_ALLOC {
        return Err(InitdataError::BundleAllocationTooSmall);
    }
    Ok(())
}

fn validate_instance(role: InstanceRole, inst: &InstanceAddresses) -> Result<(), InitdataError> {
    required_address(inst.root)?;
    required_address(inst.main_config)?;
    required_address(inst.status_a)?;
    match role {
        InstanceRole::Primary => {

            if inst.secondary_extras != [0, 0] {
                return Err(InitdataError::RoleSpecificAddress);
            }
            required_address(inst.status_b)?;
        }
        InstanceRole::Secondary => {

            if inst.status_b != 0 {
                return Err(InitdataError::RoleSpecificAddress);
            }
            for extra in inst.secondary_extras {
                required_address(extra)?;
            }
        }
    }
    Ok(())
}

fn encode_uat_level_descriptor(bytes: &mut [u8], offset: usize, shift: u8, entries: u16) {
    bytes[offset] = 8;
    bytes[offset + 1] = UAT_PAGE_BITS;
    bytes[offset + 2] = UAT_PAGE_BITS;
    bytes[offset + 3] = shift;
    put_u16(bytes, offset + 0x04, entries);
    put_u16(bytes, offset + 0x06, UAT_PAGE_SIZE);
    put_u64(bytes, offset + 0x08, 1);
    put_u64(bytes, offset + 0x10, UAT_LEVEL_PHYS_MASK);
    put_u64(bytes, offset + 0x18, (entries as u64 - 1) << shift);
}

pub(crate) fn encode_root_for_profile(
    profile: RootProfile,
    role: InstanceRole,
    shared: &SharedAddresses,
    inst: &InstanceAddresses,
    out: &mut [u8],
) -> Result<(), InitdataError> {

    if out.len() != profile.root_size(role) {
        return Err(InitdataError::BufferSize);
    }
    validate_shared(shared)?;
    validate_instance(role, inst)?;

    out.fill(0);
    put_u64(out, ROOT_VERSION, profile.version_word());
    put_u64(out, ROOT_REGION_A, shared.region_a);

    put_u64(out, ROOT_MAIN_CONFIG, inst.main_config);
    put_u64(out, ROOT_REGION_C, shared.region_c);
    put_u32(out, ROOT_INSTANCE_KIND, role as u32);
    put_u32(out, ROOT_FIXED_WORD, 1);
    put_u16(out, ROOT_PAGE_SIZE, UAT_PAGE_SIZE);
    out[ROOT_PAGE_BITS] = UAT_PAGE_BITS;
    out[ROOT_LEVEL_COUNT] = UAT_LEVEL_COUNT as u8;
    for (index, (shift, entries)) in UAT_LEVELS.iter().enumerate() {
        encode_uat_level_descriptor(
            out,
            ROOT_LEVELS + index * UAT_LEVEL_DESC_SIZE,
            *shift,
            *entries,
        );
    }
    put_u64(out, ROOT_STATUS_A, inst.status_a);
    put_u64(out, ROOT_STATUS_B, inst.status_b);
    if role == InstanceRole::Secondary {
        put_u64(out, ROOT_SECONDARY_EXTRA_0, inst.secondary_extras[0]);
        put_u64(out, ROOT_SECONDARY_EXTRA_1, inst.secondary_extras[1]);
    }
    Ok(())
}

pub(crate) fn derive_channel_table(
    role: InstanceRole,
    state_grid: u64,
    status_a: u64,
    main_config: u64,
    bundle: u64,
) -> [ChannelEntry; CHANNEL_TABLE_ENTRIES] {
    let mut table = [ChannelEntry::EMPTY; CHANNEL_TABLE_ENTRIES];
    let grid_states = |index: usize| {
        let base = state_grid + WORK_STATE_GRID_OFFSETS[index] as u64;
        [
            base,
            base + CHANNEL_STATE_SPACING as u64,
            base + 2 * CHANNEL_STATE_SPACING as u64,
        ]
    };
    if role == InstanceRole::Primary {
        for index in 0..WORK_CHANNEL_COUNT {
            table[index] = ChannelEntry {
                states: grid_states(index),
                ring: bundle + WORK_RING_BUNDLE_OFFSETS[index] as u64,
            };
        }
    }
    table[CONTROL_CHANNEL_INDEX] = ChannelEntry {
        states: grid_states(CONTROL_CHANNEL_INDEX),
        ring: main_config + MAIN_CONTROL_RING as u64,
    };
    for (slot, index) in REPORT_CHANNEL_INDICES.iter().enumerate() {
        let mut states = [0u64; 3];
        for (word, offset) in states.iter_mut().zip(REPORT_STATE_STATUS_A_OFFSETS[slot]) {
            *word = status_a + offset as u64;
        }
        table[*index] = ChannelEntry {
            states,
            ring: status_a + REPORT_RING_STATUS_A_OFFSETS[slot] as u64,
        };
    }
    table[PARTIAL_CHANNEL_INDEX] = ChannelEntry {
        states: [status_a + PARTIAL_STATE_STATUS_A_OFFSET as u64, 0, 0],
        ring: 0,
    };
    table
}

pub(crate) fn derive_channel_table_for_profile(
    profile: RootProfile,
    role: InstanceRole,
    state_grid: u64,
    status_a: u64,
    main_config: u64,
    bundle: u64,
) -> [ChannelEntry; CHANNEL_TABLE_ENTRIES] {
    let mut table = derive_channel_table(role, state_grid, status_a, main_config, bundle);
    if role == InstanceRole::Primary {
        let rings = profile.work_ring_bundle_offsets();
        for (index, entry) in table.iter_mut().take(WORK_CHANNEL_COUNT).enumerate() {
            entry.ring = bundle + rings[index] as u64;
        }
    }
    table
}

fn validate_populated_entry(entry: &ChannelEntry) -> Result<(), InitdataError> {

    for state in entry.states {
        if state == 0 {
            return Err(InitdataError::ChannelTableShape);
        }
        if !address_representable(state) {
            return Err(InitdataError::AddressOutOfRange);
        }
    }
    required_address(entry.ring)
}

fn validate_channel_table(
    role: InstanceRole,
    main_config: u64,
    channels: &[ChannelEntry; CHANNEL_TABLE_ENTRIES],
) -> Result<(), InitdataError> {
    for (index, entry) in channels.iter().enumerate() {
        match index {
            0..=11 => match role {
                InstanceRole::Primary => validate_populated_entry(entry)?,

                InstanceRole::Secondary => {
                    if *entry != ChannelEntry::EMPTY {
                        return Err(InitdataError::ChannelTableShape);
                    }
                }
            },
            12 => {
                validate_populated_entry(entry)?;

                if entry.ring != main_config + MAIN_CONTROL_RING as u64 {
                    return Err(InitdataError::ChannelTableShape);
                }
            }
            13 | 14 => validate_populated_entry(entry)?,
            15 => {

                required_address(entry.states[0])?;
                if entry.states[1] != 0 || entry.states[2] != 0 || entry.ring != 0 {
                    return Err(InitdataError::ChannelTableShape);
                }
            }
            _ => {
                if *entry != ChannelEntry::EMPTY {
                    return Err(InitdataError::ChannelTableShape);
                }
            }
        }
    }
    Ok(())
}

pub(crate) fn encode_main_config_for_profile(
    profile: RootProfile,
    role: InstanceRole,
    shared: &SharedAddresses,
    inst: &InstanceAddresses,
    channels: &[ChannelEntry; CHANNEL_TABLE_ENTRIES],
    region_views: Option<&PrimaryRegionViews>,
    out: &mut [u8],
) -> Result<(), InitdataError> {
    if out.len() != MAIN_CONFIG_SIZE {
        return Err(InitdataError::BufferSize);
    }
    validate_shared(shared)?;
    validate_instance(role, inst)?;
    validate_channel_table(role, inst.main_config, channels)?;
    match (role, region_views) {
        (InstanceRole::Primary, Some(views)) => {
            required_address(views.sentinel_high)?;
            for high in views.alias_high {
                required_address(high)?;
            }
        }
        (InstanceRole::Secondary, None) => {}
        _ => return Err(InitdataError::RoleSpecificAddress),
    }

    let view_offsets = profile.bundle_view_offsets();
    for (offset, extent) in view_offsets.iter().zip(profile.bundle_view_extents()) {
        if offset + extent > shared.hw_data_bundle_alloc {
            return Err(InitdataError::BundleAllocationTooSmall);
        }
    }

    out.fill(0);

    let bundle = shared.hw_data_bundle;
    put_u64(out, MAIN_HW_DATA, bundle);
    let repeated = bundle + profile.repeated_region_bundle_offset() as u64;
    put_u64(out, MAIN_REPEATED_0, repeated);
    put_u64(out, MAIN_REPEATED_1, repeated);

    for (index, entry) in channels.iter().enumerate() {
        let base = MAIN_CHANNEL_TABLE + index * CHANNEL_ENTRY_SIZE;
        for (word, state) in entry.states.iter().enumerate() {
            put_u64(out, base + word * 8, *state);
        }
        put_u64(out, base + CHANNEL_ENTRY_STATE_COUNT * 8, entry.ring);
    }

    match (role, region_views) {
        (InstanceRole::Primary, Some(views)) => {

            for (index, offset) in view_offsets.iter().enumerate() {
                put_u64(out, MAIN_BUNDLE_VIEWS + index * 8, bundle + *offset as u64);
            }

            put_u64(out, MAIN_REGION_VIEWS, views.sentinel_high);
            put_u64(out, MAIN_REGION_VIEWS + 0x08, REGION_VIEW_LOW_ADDRS[0]);
            put_u64(out, MAIN_REGION_VIEWS + 0x10, views.alias_high[0]);
            put_u64(out, MAIN_REGION_VIEWS + 0x18, REGION_VIEW_LOW_ADDRS[1]);
            put_u64(out, MAIN_REGION_VIEWS + 0x20, views.alias_high[1]);
            // J714's firmware takes both primary sentinels too; they are not
            // exclusive to the older J700 ABI.
            if matches!(profile, RootProfile::J700G17P | RootProfile::J714G17S26A428) {
                put_u32(out, MAIN_PRIMARY_SENTINEL_A, 1);
                put_u32(out, MAIN_PRIMARY_SENTINEL_B, 0xabcd_abcd);
            }

            put_u32(out, MAIN_PRIMARY_MASK, MAIN_PRIMARY_MASK_VALUE);
            put_u32(out, MAIN_CONTROL_RING, profile.control_opening_opcode(role));
        }
        (InstanceRole::Secondary, _) => {

            put_u64(out, MAIN_REGION_VIEWS + 0x08, REGION_VIEW_LOW_ADDRS[0]);
            put_u64(out, MAIN_REGION_VIEWS + 0x18, REGION_VIEW_LOW_ADDRS[1]);
            put_u32(out, MAIN_SECONDARY_KIND, match profile {
                RootProfile::J700G17P => MAIN_SECONDARY_KIND_VALUE,
                RootProfile::J813G17G25G76 => 0xc,

                RootProfile::J873G18G26A428 | RootProfile::J714G17S26A428 => {
                    COMMON26_MAIN_SECONDARY_KIND
                }
            });
            if profile == RootProfile::J714G17S26A428 {

                put_u64(out, COMMON26_MAIN_SECONDARY_POLICY_PTR,
                        bundle + M5_SHARED_LAYOUT.policy as u64);
            } else if profile == RootProfile::J873G18G26A428 {

                put_u64(out, COMMON26_MAIN_SECONDARY_POLICY_PTR,
                        bundle + COMMON26_SHARED_LAYOUT.policy as u64);
            } else {

                let extra = bundle
                    + BUNDLE_VIEW_OFFSETS[SECONDARY_EXTRA_VIEW_INDEX] as u64
                    + SECONDARY_EXTRA_VIEW_OFFSET as u64;
                put_u64(out, MAIN_SECONDARY_EXTRA_PTR, extra);
            }
            put_u32(out, MAIN_CONTROL_RING, profile.control_opening_opcode(role));
        }
        _ => unreachable!(),
    }
    Ok(())
}

pub(crate) fn encode_region_c_for_profile(
    profile: RootProfile,
    out: &mut [u8],
) -> Result<(), InitdataError> {
    if out.len() != REGION_C_SIZE {
        return Err(InitdataError::BufferSize);
    }
    if profile == RootProfile::J873G18G26A428 {

        return encode_common26_region_c(out);
    }
    if profile == RootProfile::J714G17S26A428 {

        return Err(InitdataError::BufferSize);
    }
    out.fill(0);
    write_runs(out, REGION_C_RUNS);
    if profile == RootProfile::J813G17G25G76 {

        for (offset, value) in [
            (0x084, 0x0000_5eb0),
            (0x088, 0x0000_04b0),
            (0x08c, 0x0000_04b0),
            (0x098, 0x0000_04b0),
            (0x7c4, 0x0000_04e2),
            (0x7e0, 0x3f33_3333),
            (0x9a8, 0x0000_0333),
            (0xe38, 0),
            (0xe3c, 0),
            (0xe58, 0x0000_0300),
            (0xe5c, 0x0000_0100),
        ] {
            put_u32(out, offset, value);
        }
        return Ok(());
    }

    if 1 == 0 {
        put_u32(out, 0x9cc, 0);
    }

    if 0 != 0 {
        out[0xe21] = 1;
    }
    for (offset, value) in [
        (0x7c, 1),
        (0x80, 1),
        (0x9c, 0x3000_001d),
        (0xa0, 0x3100_0002),
    ] {
        put_u32(out, offset, value);
    }
    Ok(())
}

pub(crate) fn encode_status_block(out: &mut [u8]) -> Result<(), InitdataError> {
    if out.len() != STATUS_BLOCK_SIZE {
        return Err(InitdataError::BufferSize);
    }
    out.fill(0);
    put_u32(out, STATUS_BLOCK_ONE_OFFSET, 1);
    Ok(())
}

pub(crate) fn encode_secondary_root_extra_1_for_profile(
    profile: RootProfile,
    out: &mut [u8],
) -> Result<(), InitdataError> {
    if out.len() != STATUS_BLOCK_SIZE {
        return Err(InitdataError::BufferSize);
    }
    out.fill(0);
    write_runs(out, SECONDARY_ROOT_EXTRA_1_RUNS);

    if matches!(profile, RootProfile::J813G17G25G76 | RootProfile::J873G18G26A428) {
        put_u32(out, 0x10, 2);
    }
    Ok(())
}

pub(crate) const REGION_C_RUNS: &[(usize, &[u8])] = &[
    (0x0024, &[0xb8, 0x0b, 0x00, 0x00]),
    (
        0x0030,
        &[
            0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0x00,
            0x00, 0x00,
        ],
    ),
    (
        0x0054,
        &[
            0xff, 0xff, 0x28, 0x00, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
            0x01, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x01, 0x00,
        ],
    ),
    (0x0078, &[0x01, 0x00, 0x00, 0x00]),
    (
        0x0084,
        &[
            0x4e, 0x25, 0x00, 0x00, 0xe8, 0x03, 0x00, 0x00, 0xe8, 0x03, 0x00, 0x00,
        ],
    ),
    (0x0098, &[0xe8, 0x03, 0x00, 0x00]),
    (
        0x07c4,
        &[
            0xd0, 0x07, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3f, 0xcd, 0xcc, 0x4c, 0x3f, 0xcd, 0xcc,
            0x4c, 0x3e, 0x66, 0x66, 0x66, 0x3f, 0xcd, 0xcc, 0xcc, 0x3d, 0x00, 0x00, 0x80, 0x3e,
            0x9a, 0x99, 0x19, 0x3f, 0x66, 0x66, 0x66, 0x3f, 0x06, 0x00, 0x00, 0x00, 0x01, 0x00,
            0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00,
        ],
    ),
    (
        0x0998,
        &[
            0x28, 0x00, 0x00, 0x00, 0x0a, 0x00, 0x00, 0x00, 0xfa, 0x00, 0x00, 0x00, 0x01, 0x00,
            0x00, 0x00,
        ],
    ),
    (
        0x09b8,
        &[
            0x02, 0x00, 0x00, 0x00, 0x28, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x28, 0x00, 0x00, 0x00, 0x32, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
        ],
    ),
    (0x0e1c, &[0x00, 0x01, 0x00, 0x00]),
    (0x0e38, &[0x00, 0x03, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00]),
    (0x0e50, &[0x00, 0x00, 0x00, 0x00]),
];

pub(crate) const SECONDARY_ROOT_EXTRA_1_RUNS: &[(usize, &[u8])] = &[
    (0x14, &[0x01]),
    (0x2c, &[0x01]),
    (0x30, &[0x01]),
    (0x40, &[0x01]),
    (0x4c, &[0x01]),
    (0x50, &[0x6a, 0x18]),
];

pub(crate) const COMMON26_INITDATA_VERSION_WORD: u64 = 0x0c8b_4339_83a0_04c0;

pub(crate) const COMMON26_ROOT_SIZE: usize = 0xc8;

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) enum Evidence {
    FirmwareProven,
    ProducerChecked,
    AdtProven,
    Inferred,
    Guessed,
}

pub(crate) const ABI_ALLOCATION_SIZES_25G76: [usize; 24] = [
    0x2708, 0x0c18, 0x1048, 0x0e10, 0x6a30, 0x0068, 0x0800, 0x0088, 0x1800, 0x04c0, 0x4000, 0x0030,
    0xe440, 0x06a8, 0x0020, 0x01b0, 0x4800, 0x28800, 0x79800, 0x9000, 0x0060, 0x0008, 0x0e78, 0x00e4,
];

pub(crate) const ABI_ALLOCATION_SIZES_26A428: [usize; 24] = [
    0x2848, 0x0c18, 0x1048, 0x0e10, 0x7800, 0x0068, 0x0800, 0x0088, 0x1800, 0x04c0, 0x4000, 0x0030,
    0xe730, 0x06d0, 0x0020, 0x01b0, 0x4800, 0x28800, 0x79800, 0x9000, 0x0060, 0x0008, 0x0e80, 0x00e4,
];

pub(crate) const ALLOC_HW_DATA: usize = 0;

pub(crate) const ALLOC_BLOCK_C18: usize = 1;

pub(crate) const ALLOC_BLOCK_1048: usize = 2;

pub(crate) const ALLOC_BLOCK_E10: usize = 3;

pub(crate) const ALLOC_POLICY: usize = 4;

pub(crate) const ALLOC_BLOCK_68: usize = 5;

pub(crate) const ALLOC_BLOCK_800: usize = 6;

pub(crate) const ALLOC_BLOCK_88: usize = 7;

pub(crate) const ALLOC_DM_RING: usize = 8;

pub(crate) const ALLOC_MAIN_CONFIG: usize = 9;

pub(crate) const ALLOC_CONTROL_RING: usize = 10;

pub(crate) const ALLOC_RING_STATE: usize = 11;

pub(crate) const ALLOC_STATUS_B: usize = 12;

pub(crate) const ALLOC_STATUS_CONFIG: usize = 13;

pub(crate) const ALLOC_STATUS_A: usize = 14;

pub(crate) const ALLOC_LOG_STATE: usize = 15;

pub(crate) const ALLOC_RING_4800: usize = 16;

pub(crate) const ALLOC_BUFFER_28800: usize = 17;

pub(crate) const ALLOC_BUFFER_79800: usize = 18;

pub(crate) const ALLOC_RING_9000: usize = 19;

pub(crate) const ALLOC_SECONDARY_EXTRA_1: usize = 20;

pub(crate) const ALLOC_POLICY_STATE: usize = 21;

pub(crate) const ALLOC_REGION_C: usize = 22;

pub(crate) const ALLOC_AON_PTD: usize = 23;

pub(crate) const ABI_BRN_TABLE_SIZE: usize = 0;

pub(crate) const ABI_ALLOCATION_PAGE: usize = 0x4000;

pub(crate) const SHARED_ALLOCATION_ENTRIES: usize = 25;

pub(crate) const PRIVATE_ALLOCATION_ENTRIES: usize = 38;

const ROLE_SET_ENTRIES: usize = 10;

const fn round_up_usize(value: usize, alignment: usize) -> usize {
    (value + alignment - 1) & !(alignment - 1)
}

pub(crate) const fn next_packed_allocation(offset: usize, size: usize) -> usize {
    round_up_usize(offset + size, 0x40)
}

pub(crate) const fn packed_allocation_offsets<const N: usize>(sizes: &[usize; N]) -> [usize; N] {
    let mut out = [0usize; N];
    let mut index = 1;
    while index < N {
        out[index] = next_packed_allocation(out[index - 1], sizes[index - 1]);
        index += 1;
    }
    out
}

pub(crate) const fn shared_allocation_sizes(table: &[usize; 24]) -> [usize; SHARED_ALLOCATION_ENTRIES] {
    let mut sizes = [0usize; SHARED_ALLOCATION_ENTRIES];
    sizes[0] = table[ALLOC_HW_DATA];
    sizes[1] = table[ALLOC_BLOCK_C18];
    sizes[2] = table[ALLOC_BLOCK_1048];
    sizes[3] = table[ALLOC_BLOCK_E10];
    sizes[4] = table[ALLOC_POLICY];
    sizes[5] = table[ALLOC_BLOCK_68];
    sizes[6] = table[ALLOC_BLOCK_800];
    sizes[7] = ABI_BRN_TABLE_SIZE;
    sizes[8] = table[ALLOC_BLOCK_88];
    let mut ring = 0;
    while ring < WORK_CHANNEL_COUNT {
        sizes[9 + ring] = table[ALLOC_DM_RING];
        ring += 1;
    }
    sizes[21] = table[ALLOC_MAIN_CONFIG];
    sizes[22] = table[ALLOC_CONTROL_RING];
    sizes[23] = table[ALLOC_MAIN_CONFIG];
    sizes[24] = table[ALLOC_CONTROL_RING];
    sizes
}

pub(crate) const fn private_allocation_sizes(table: &[usize; 24]) -> [usize; PRIVATE_ALLOCATION_ENTRIES] {
    let mut sizes = [0usize; PRIVATE_ALLOCATION_ENTRIES];
    let mut state = 0;
    while state <= WORK_CHANNEL_COUNT {
        sizes[state] = table[ALLOC_RING_STATE];
        state += 1;
    }
    sizes[13] = table[ALLOC_STATUS_B];
    sizes[14] = table[ALLOC_STATUS_CONFIG];
    let role_set = [
        table[ALLOC_STATUS_A], table[ALLOC_RING_STATE], table[ALLOC_LOG_STATE], table[ALLOC_RING_STATE],
        table[ALLOC_RING_STATE], table[ALLOC_RING_4800], table[ALLOC_BUFFER_28800],
        table[ALLOC_BUFFER_79800], table[ALLOC_RING_9000], table[ALLOC_RING_4800],
    ];
    let mut role = 0;
    while role < 2 {
        let mut entry = 0;
        while entry < ROLE_SET_ENTRIES {
            sizes[15 + role * ROLE_SET_ENTRIES + entry] = role_set[entry];
            entry += 1;
        }
        role += 1;
    }
    sizes[35] = table[ALLOC_RING_STATE];
    sizes[36] = table[ALLOC_SECONDARY_EXTRA_1];
    sizes[37] = table[ALLOC_POLICY_STATE];
    sizes
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) struct SharedClusterLayout {
    pub(crate) hw_data: usize,
    pub(crate) block_c18: usize,
    pub(crate) block_1048: usize,
    pub(crate) block_e10: usize,
    pub(crate) policy: usize,
    pub(crate) block_68: usize,
    pub(crate) block_800: usize,
    pub(crate) brn_table: usize,
    pub(crate) block_88: usize,

    pub(crate) dm_rings: [usize; WORK_CHANNEL_COUNT],
    pub(crate) main_config: [usize; 2],
    pub(crate) control_ring: [usize; 2],
    pub(crate) end: usize,
    pub(crate) total: usize,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) struct RoleSetLayout {

    pub(crate) status_a: usize,

    pub(crate) event_state: usize,

    pub(crate) log_state: usize,

    pub(crate) trace_state: usize,
    pub(crate) state_30: usize,

    pub(crate) event_ring: usize,
    pub(crate) buffer_28800: usize,

    pub(crate) buffer_79800: usize,

    pub(crate) trace_ring: usize,
    pub(crate) buffer_4800: usize,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) struct PrivateClusterLayout {
    pub(crate) dm_ring_states: [usize; WORK_CHANNEL_COUNT],

    pub(crate) control_state: usize,

    pub(crate) status_b: usize,

    pub(crate) status_config: usize,
    pub(crate) roles: [RoleSetLayout; 2],

    pub(crate) secondary_control_state: usize,

    pub(crate) secondary_extra_1: usize,

    pub(crate) policy_state: usize,
    pub(crate) end: usize,
    pub(crate) total: usize,
}

pub(crate) const fn shared_cluster_layout(table: &[usize; 24]) -> SharedClusterLayout {
    let sizes = shared_allocation_sizes(table);
    let offsets = packed_allocation_offsets(&sizes);
    let mut dm_rings = [0usize; WORK_CHANNEL_COUNT];
    let mut ring = 0;
    while ring < WORK_CHANNEL_COUNT {
        dm_rings[ring] = offsets[9 + ring];
        ring += 1;
    }
    let end = offsets[24] + sizes[24];
    SharedClusterLayout {
        hw_data: offsets[0],
        block_c18: offsets[1],
        block_1048: offsets[2],
        block_e10: offsets[3],
        policy: offsets[4],
        block_68: offsets[5],
        block_800: offsets[6],
        brn_table: offsets[7],
        block_88: offsets[8],
        dm_rings,
        main_config: [offsets[21], offsets[23]],
        control_ring: [offsets[22], offsets[24]],
        end,
        total: round_up_usize(end, ABI_ALLOCATION_PAGE),
    }
}

const fn role_set_layout(offsets: &[usize; PRIVATE_ALLOCATION_ENTRIES], first: usize) -> RoleSetLayout {
    RoleSetLayout {
        status_a: offsets[first],
        event_state: offsets[first + 1],
        log_state: offsets[first + 2],
        trace_state: offsets[first + 3],
        state_30: offsets[first + 4],
        event_ring: offsets[first + 5],
        buffer_28800: offsets[first + 6],
        buffer_79800: offsets[first + 7],
        trace_ring: offsets[first + 8],
        buffer_4800: offsets[first + 9],
    }
}

pub(crate) const fn private_cluster_layout(table: &[usize; 24]) -> PrivateClusterLayout {
    let sizes = private_allocation_sizes(table);
    let offsets = packed_allocation_offsets(&sizes);
    let mut dm_ring_states = [0usize; WORK_CHANNEL_COUNT];
    let mut state = 0;
    while state < WORK_CHANNEL_COUNT {
        dm_ring_states[state] = offsets[state];
        state += 1;
    }
    let end = offsets[37] + sizes[37];
    PrivateClusterLayout {
        dm_ring_states,
        control_state: offsets[12],
        status_b: offsets[13],
        status_config: offsets[14],
        roles: [role_set_layout(&offsets, 15), role_set_layout(&offsets, 25)],
        secondary_control_state: offsets[35],
        secondary_extra_1: offsets[36],
        policy_state: offsets[37],
        end,
        total: round_up_usize(end, ABI_ALLOCATION_PAGE),
    }
}

pub(crate) const WORK_CHANNEL_DM_INDEX: [usize; WORK_CHANNEL_COUNT] =
    [3, 7, 11, 2, 6, 10, 1, 5, 9, 0, 4, 8];

pub(crate) const fn work_ring_offsets(layout: &SharedClusterLayout) -> [usize; WORK_CHANNEL_COUNT] {
    let mut rings = [0usize; WORK_CHANNEL_COUNT];
    let mut channel = 0;
    while channel < WORK_CHANNEL_COUNT {
        rings[channel] = layout.dm_rings[WORK_CHANNEL_DM_INDEX[channel]];
        channel += 1;
    }
    rings
}

pub(crate) const M5_25G76_SHARED_LAYOUT: SharedClusterLayout =
    shared_cluster_layout(&ABI_ALLOCATION_SIZES_25G76);

pub(crate) const COMMON26_SHARED_LAYOUT: SharedClusterLayout =
    shared_cluster_layout(&ABI_ALLOCATION_SIZES_26A428);

pub(crate) const COMMON26_HW_DATA_SIZE: usize = ABI_ALLOCATION_SIZES_26A428[ALLOC_HW_DATA];

pub(crate) const COMMON26_BUNDLE_VIEW_OFFSETS: [usize; 5] = [
    COMMON26_SHARED_LAYOUT.block_c18,
    COMMON26_SHARED_LAYOUT.block_1048,
    COMMON26_SHARED_LAYOUT.block_e10,
    COMMON26_SHARED_LAYOUT.block_68,
    COMMON26_SHARED_LAYOUT.block_800,
];

pub(crate) const COMMON26_BUNDLE_VIEW_EXTENTS: [usize; 5] = [
    ABI_ALLOCATION_SIZES_26A428[ALLOC_BLOCK_C18],
    ABI_ALLOCATION_SIZES_26A428[ALLOC_BLOCK_1048],
    ABI_ALLOCATION_SIZES_26A428[ALLOC_BLOCK_E10],
    ABI_ALLOCATION_SIZES_26A428[ALLOC_BLOCK_68],
    ABI_ALLOCATION_SIZES_26A428[ALLOC_BLOCK_800],
];

pub(crate) const COMMON26_WORK_RING_BUNDLE_OFFSETS: [usize; WORK_CHANNEL_COUNT] =
    work_ring_offsets(&COMMON26_SHARED_LAYOUT);

pub(crate) const COMMON26_MAIN_SECONDARY_POLICY_PTR: usize = 0x481;

pub(crate) const COMMON26_MAIN_SECONDARY_KIND: u32 = 0xc;

pub(crate) const COMMON26_FWCTL_RING_OFFSET: usize = 0x40;

pub(crate) const COMMON26_STATUS_B_FWCTL_STATE: usize = 0x4ac0;

pub(crate) const COMMON26_STATUS_B_FWCTL_RING: usize = 0x4ac8;

pub(crate) const COMMON26_STATUS_B_CONFIG_HEADER: usize = 0xe714;

pub(crate) const COMMON26_STATUS_B_LINK_WORD: usize = 0xe71c;

pub(crate) const COMMON26_STATUS_B_LINK_PTR: usize = 0xe720;

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) struct LayoutBand {
    pub(crate) start: usize,
    pub(crate) end: usize,
    pub(crate) delta: isize,
    pub(crate) evidence: Evidence,
}

const fn band(start: usize, end: usize, delta: isize, evidence: Evidence) -> LayoutBand {
    LayoutBand { start, end, delta, evidence }
}

pub(crate) const fn relocate_by_bands(bands: &[LayoutBand], offset: usize) -> Option<usize> {
    let mut index = 0;
    while index < bands.len() {
        let band = bands[index];
        if band.start <= offset && offset < band.end {
            return Some((offset as isize + band.delta) as usize);
        }
        index += 1;
    }
    None
}

pub(crate) const COMMON26_HW_DATA_BANDS: &[LayoutBand] = &[

    band(0x000, 0x640, 0, Evidence::Inferred),

    band(0xe88, 0xe90, 0x118, Evidence::ProducerChecked),

    band(0xe90, 0xebc, 0x120, Evidence::FirmwareProven),

    band(0xec0, 0xf5c, 0x11c, Evidence::FirmwareProven),
    band(0xf5c, 0xf60, 0x124, Evidence::ProducerChecked),

    band(0xf68, 0xf98, 0x11c, Evidence::FirmwareProven),
    band(0xf98, 0xfac, 0x120, Evidence::FirmwareProven),

    band(0xfb0, 0x25f8, 0x11c, Evidence::FirmwareProven),
    band(0x25f8, 0x26b4, 0x120, Evidence::FirmwareProven),
    band(0x26b4, 0x26f0, 0x128, Evidence::FirmwareProven),
    band(0x26f0, M5_25G76_HW_DATA_SIZE, 0x12c, Evidence::FirmwareProven),
];

pub(crate) const COMMON26_STATUS_CONFIG_BANDS: &[LayoutBand] = &[
    band(0x000, 0x1dc, 0, Evidence::ProducerChecked),
    band(0x4e0, 0x518, 0xc, Evidence::ProducerChecked),
    band(0x518, 0x528, 0x14, Evidence::ProducerChecked),
    band(0x530, 0x534, 0xc, Evidence::ProducerChecked),
    band(0x554, 0x584, 0xc, Evidence::ProducerChecked),
    band(0x584, 0x5a8, 0x1c, Evidence::ProducerChecked),
    band(0x5cc, 0x628, 0x18, Evidence::ProducerChecked),
];

fn write_relocated_words(
    words: &[(usize, u32)],
    source_base: usize,
    bands: &[LayoutBand],
    out: &mut [u8],
) -> Result<(), InitdataError> {
    for &(offset, value) in words {
        for (index, byte) in value.to_le_bytes().into_iter().enumerate() {
            if byte == 0 {
                continue;
            }
            let source = offset
                .checked_sub(source_base)
                .ok_or(InitdataError::BufferSize)?
                + index;
            if let Some(target) = relocate_by_bands(bands, source) {
                *out.get_mut(target).ok_or(InitdataError::BufferSize)? = byte;
            }
        }
    }
    Ok(())
}

pub(crate) const COMMON26_HW_REGISTER_MAP_SLOTS: usize = 60;

pub(crate) const COMMON26_HW_REGISTER_MAP_END: usize =
    HW_REGISTER_MAP_OFFSET + COMMON26_HW_REGISTER_MAP_SLOTS * HW_REGISTER_MAP_ENTRY_SIZE;

pub(crate) const COMMON26_HW_CHIP_ID_OFFSET: usize = 0xfb0;

pub(crate) const COMMON26_HW_PERF_STATE_COUNT: usize = 0x10e0;

pub(crate) const COMMON26_HW_PERF_FREQUENCY: usize = 0x10e4;

pub(crate) const COMMON26_HW_PERF_CORE_VOLTAGE: usize = 0x1124;

pub(crate) const COMMON26_HW_PERF_SRAM_VOLTAGE: usize = 0x1524;

pub(crate) const COMMON26_HW_PERF_ROWS: usize = 16;

pub(crate) const COMMON26_HW_PERF_ROW_STRIDE: usize = 0x40;

pub(crate) const COMMON26_HW_PERF_LANES: usize = 16;

pub(crate) const COMMON26_HW_PERF_SCALE: usize = 0x1964;

pub(crate) const COMMON26_HW_PERF_RELATIVE_A: usize = 0x19e4;

pub(crate) const COMMON26_HW_PERF_RELATIVE_B: usize = 0x1a24;

const M5_HW_AUX_RELATIVE_FREQUENCY: usize = 0x1aa4;
const M5_HW_AUX_PERF_STATE: usize = 0x1cac;
const M5_HW_AUX_PERF_STATE_SIZE: usize = 0x148;

// Clock-domain 2: MHz, core mV, SRAM mV. The second voltage lane is unused.
const J714_AUX_PERF_STATES: [M5AuxPerfState; 14] = one_lane([
    (24, 125, 775), (338, 590, 775), (486, 620, 775), (636, 645, 775),
    (796, 685, 775), (888, 700, 775), (988, 745, 775), (1084, 770, 775),
    (1182, 795, 795), (1278, 835, 835), (1374, 860, 860),
    (1470, 900, 900), (1578, 960, 960), (1620, 990, 990),
]);

// J716c ADT afr-perf-states (identical to cs-perf-states). SRAM is
// max(775, core), the rule the J714 column follows.
const J716C_AUX_PERF_STATES: [M5AuxPerfState; 14] = one_lane([
    (24, 125, 775), (338, 590, 775), (486, 625, 775), (636, 640, 775),
    (796, 690, 775), (888, 700, 775), (988, 740, 775), (1084, 765, 775),
    (1182, 780, 780), (1278, 830, 830), (1374, 875, 875),
    (1470, 910, 910), (1578, 975, 975), (1620, 1020, 1020),
]);

/// An auxiliary-domain performance state: MHz, then core and SRAM mV of each of its two voltage
/// lanes. A one-lane board leaves the second lane 0.
pub(crate) type M5AuxPerfState = (u32, [u32; 2], [u32; 2]);

// J775D ADT afr-perf-states / cs-perf-states: one lane per die, mV; SRAM floor 775 mV.
const J775D_AUX_PERF_STATES: [M5AuxPerfState; 14] = [
    (24, [125, 125], [775, 775]), (338, [600, 600], [775, 775]),
    (486, [660, 655], [775, 775]), (636, [670, 665], [775, 775]),
    (796, [710, 705], [775, 775]), (888, [720, 720], [775, 775]),
    (988, [755, 765], [775, 775]), (1084, [800, 795], [800, 795]),
    (1182, [810, 820], [810, 820]), (1278, [850, 860], [850, 860]),
    (1374, [905, 915], [905, 915]), (1470, [945, 955], [945, 955]),
    (1578, [1030, 1040], [1030, 1040]), (1620, [1075, 1085], [1075, 1085]),
];

/// One-lane auxiliary tables.
const fn one_lane(states: [(u32, u32, u32); 14]) -> [M5AuxPerfState; 14] {
    let mut out = [(0, [0; 2], [0; 2]); 14];
    let mut state = 0;
    while state < 14 {
        let (frequency, core, sram) = states[state];
        out[state] = (frequency, [core, 0], [sram, 0]);
        state += 1;
    }
    out
}

// Evaluate the frequency-only curve at compile time; no kernel FPU use.
// Conversion uses Hz before the single-precision multiply and divide. The curve is based at
// `floor`, the lowest state the performance controllers run the GPU at: states at and below it
// read 0.
const fn relative_frequency(states: &[M5PerfState; 14], floor: usize) -> [u32; 16] {
    let mut values = [0; 16];
    let base = states[floor].0 as u64 * 1_000_000;
    let maximum = states[13].0 as u64 * 1_000_000;
    let denominator = (maximum - base) as f32;
    let mut state = floor + 1;
    while state < 14 {
        let frequency = states[state].0 as u64 * 1_000_000;
        values[state] = (((frequency - base) as f32 * 100.0) / denominator).to_bits();
        state += 1;
    }
    values
}

fn encode_m5_aux_perf_states(board: &Board, out: &mut [u8]) {
    let table = &mut out[M5_HW_AUX_PERF_STATE..
        M5_HW_AUX_PERF_STATE + M5_HW_AUX_PERF_STATE_SIZE];
    table.fill(0);
    put_u32(table, 0, 13);
    put_u32(table, 4, 14);
    for (state, &(frequency, core, sram)) in board.aux_perf_states.iter().enumerate() {
        put_u32(table, 8 + state * 4, frequency);
        for lane in 0..2 {
            put_u32(table, 0x48 + state * 8 + lane * 4, core[lane]);
            put_u32(table, 0xc8 + state * 8 + lane * 4, sram[lane]);
        }
    }
}

pub(crate) const COMMON26_HW_PERF_INDEX_MAP: usize = 0x1ae4;

pub(crate) const COMMON26_KSM_COMPLETION_TABLE_OFFSET: usize = 0x2734;

pub(crate) const COMMON26_QOS_POINTER_OFFSET: usize = 0x27cc;

pub(crate) const COMMON26_HW_FIXED_WORD_OFFSET: usize = 0x2714;

pub(crate) const COMMON26_HW_FIXED_WORD_VALUE: u32 = 0x96;

pub(crate) const J873_GPU_NUM_PERF_STATES: u32 = 13;

const COMMON26_PERF_SCALE_VALUE: u32 = 0x3f82_8f5c;

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) struct RegisterWindowSpec {
    pub(crate) slot: usize,
    pub(crate) phys: u64,

    pub(crate) size_10: u32,

    pub(crate) size_14: u32,

    pub(crate) unk_18: u64,

    pub(crate) flag: u32,
}

const fn spec(slot: usize, phys: u64, size: u32, unk_18: u64, flag: u32) -> RegisterWindowSpec {
    RegisterWindowSpec { slot, phys, size_10: size, size_14: size, unk_18, flag }
}

pub(crate) const COMMON26_REGION_C_ADT_WORDS: [(usize, u32); 3] = [
    (0x7c8, 1500),
    (0x7e4, 0x3f4c_cccd),
    (0x09c, J873_GPU_NUM_PERF_STATES * 100),
];

pub(crate) fn encode_common26_region_c(out: &mut [u8]) -> Result<(), InitdataError> {
    if out.len() != REGION_C_SIZE {
        return Err(InitdataError::BufferSize);
    }
    encode_region_c_for_profile(RootProfile::J813G17G25G76, out)?;

    let shifted = ABI_ALLOCATION_SIZES_25G76[ALLOC_REGION_C];
    if out[shifted..].iter().any(|byte| *byte != 0) {
        return Err(InitdataError::BufferSize);
    }
    out.copy_within(0x38..shifted, 0x3c);
    out[0x38..0x3c].fill(0);
    for (offset, value) in COMMON26_REGION_C_ADT_WORDS {
        put_u32(out, offset, value);
    }
    Ok(())
}

// J714 descriptors carry device-specific AON table indices.
const J714_AON_RECORDS: [[u32; 3]; 3] = [
    [0x6730_5452, 0x0079_0001, 0x0000_1401],
    [0x6731_5452, 0x00bc_0001, 0x0000_1401],
    [0x7230_5452, 0x00ce_0001, 0x0000_1401],
];

// J716c's rows tagged RT0g, RT1g and RT0r.
const J716C_AON_RECORDS: [[u32; 3]; 3] = [
    [0x6730_5452, 0x0058_0001, 0x0000_1401],
    [0x6731_5452, 0x00a3_0001, 0x0000_1401],
    [0x7230_5452, 0x00b8_0001, 0x0000_1401],
];

// J775D: both dies' records, g0TR-g3TR and r0TR-r1TR.
const J775D_AON_RECORDS: [[u32; 3]; 6] = [
    [0x6730_5452, 0x019f_0001, 0x0000_1401],
    [0x6731_5452, 0x01bb_0001, 0x0000_1401],
    [0x6732_5452, 0x00a7_0001, 0x0000_1401],
    [0x6733_5452, 0x00c3_0001, 0x0000_1401],
    [0x7230_5452, 0x0113_0001, 0x0000_1401],
    [0x7231_5452, 0x001b_0001, 0x0000_1401],
];

pub(crate) fn encode_common26_aon_ptd(board: &Board, out: &mut [u8]) -> Result<(), InitdataError> {
    if out.len() != M5_AON_PTD_SIZE || board.aon_records.len() * 12 > out.len() {
        return Err(InitdataError::BufferSize);
    }
    out.fill(0);
    for (index, record) in board.aon_records.iter().enumerate() {
        for (word, value) in record.iter().enumerate() {
            put_u32(out, index * 12 + word * 4, *value);
        }
    }
    Ok(())
}

pub(crate) const COMMON26_CONTROL_OPENING_OPCODE_PRIMARY: u32 = 0x15;

pub(crate) const COMMON26_CONTROL_OPENING_OPCODE_SECONDARY: u32 = 0x2a;

pub(crate) const M5_CONTROL_OPENING_OPCODE_PRIMARY: u32 = COMMON26_CONTROL_OPENING_OPCODE_PRIMARY;

pub(crate) const M5_CONTROL_OPENING_OPCODE_SECONDARY: u32 =
    COMMON26_CONTROL_OPENING_OPCODE_SECONDARY;

pub(crate) const M5_INITDATA_VERSION_WORD: u64 = 0x0c8e_0339_8728_04c0;

pub(crate) const M5_ROOT_SIZE: usize = 0xc8;

pub(crate) const ABI_ALLOCATION_SIZES_26A428_M5: [usize; 24] = [
    0x2850, 0x0c18, 0x1048, 0x0e10, 0x126b0, 0x0068, 0x0800, 0x0088, 0x1800, 0x04c0, 0x4000, 0x0030,
    0xe730, 0x0780, 0x0020, 0x01b0, 0x4800, 0x28800, 0x79800, 0x9000, 0x00a8, 0x0008, 0x1ca0, 0x00e4,
];

pub(crate) const M5_SHARED_LAYOUT: SharedClusterLayout =
    shared_cluster_layout(&ABI_ALLOCATION_SIZES_26A428_M5);

pub(crate) const M5_PRIVATE_LAYOUT: PrivateClusterLayout =
    private_cluster_layout(&ABI_ALLOCATION_SIZES_26A428_M5);

pub(crate) const M5_SHARED_CLUSTER_SIZE: usize = M5_SHARED_LAYOUT.total;

pub(crate) const M5_PRIVATE_CLUSTER_SIZE: usize = M5_PRIVATE_LAYOUT.total;

pub(crate) const M5_HW_DATA_SIZE: usize = ABI_ALLOCATION_SIZES_26A428_M5[ALLOC_HW_DATA];

pub(crate) const M5_KICK_SHARED_SIZE: usize = 0x2000;
pub(crate) const M5_KICK_SHARED_POINTER: usize = 0x27dc;

pub(crate) fn set_m5_kick_shared(out: &mut [u8], address: u64) -> Result<(), InitdataError> {
    if out.len() != M5_HW_DATA_SIZE {
        return Err(InitdataError::BufferSize);
    }
    required_address(address)?;
    let end = address.checked_add(M5_KICK_SHARED_SIZE as u64 - 1)
        .ok_or(InitdataError::AddressOutOfRange)?;
    required_address(end)?;
    if address & (UAT_PAGE_SIZE as u64 - 1) != 0 {
        return Err(InitdataError::AddressOutOfRange);
    }
    put_u64(out, M5_KICK_SHARED_POINTER, address);
    Ok(())
}

pub(crate) const M5_POLICY_SIZE: usize = ABI_ALLOCATION_SIZES_26A428_M5[ALLOC_POLICY];

pub(crate) const M5_STATUS_CONFIG_SIZE: usize =
    ABI_ALLOCATION_SIZES_26A428_M5[ALLOC_STATUS_CONFIG];

pub(crate) const M5_REGION_C_ALLOCATION: usize =
    ABI_ALLOCATION_SIZES_26A428_M5[ALLOC_REGION_C];

pub(crate) const M5_AON_PTD_SIZE: usize = ABI_ALLOCATION_SIZES_26A428_M5[ALLOC_AON_PTD];

pub(crate) const M5_SHARED_PREFIX_SIZE: usize = M5_SHARED_LAYOUT.brn_table;

pub(crate) const M5_BUNDLE_VIEW_OFFSETS: [usize; 5] = [
    M5_SHARED_LAYOUT.block_c18,
    M5_SHARED_LAYOUT.block_1048,
    M5_SHARED_LAYOUT.block_e10,
    M5_SHARED_LAYOUT.block_68,
    M5_SHARED_LAYOUT.block_800,
];

pub(crate) const M5_BUNDLE_VIEW_EXTENTS: [usize; 5] = COMMON26_BUNDLE_VIEW_EXTENTS;

pub(crate) const M5_WORK_RING_BUNDLE_OFFSETS: [usize; WORK_CHANNEL_COUNT] =
    work_ring_offsets(&M5_SHARED_LAYOUT);

pub(crate) const M5_MAIN_BUNDLE_OFFSETS: [usize; 2] = M5_SHARED_LAYOUT.main_config;

pub(crate) const M5_CONTROL_RING_BUNDLE_OFFSETS: [usize; 2] =
    M5_SHARED_LAYOUT.control_ring;

pub(crate) const M5_PRIVATE_STATUS_A: [usize; 2] = [
    M5_PRIVATE_LAYOUT.roles[0].status_a,
    M5_PRIVATE_LAYOUT.roles[1].status_a,
];

pub(crate) const M5_PRIVATE_STATE_GRIDS: [usize; 2] = [
    M5_PRIVATE_LAYOUT.dm_ring_states[0],
    M5_PRIVATE_LAYOUT.secondary_control_state - WORK_STATE_GRID_OFFSETS[CONTROL_CHANNEL_INDEX],
];

pub(crate) const M5_STATUS_CONFIG_IN_STATUS_B: usize =
    M5_PRIVATE_LAYOUT.status_config - M5_PRIVATE_LAYOUT.status_b;

pub(crate) const M5_STATUS_B_SPAN: usize =
    M5_PRIVATE_LAYOUT.roles[0].status_a - M5_PRIVATE_LAYOUT.status_b;

pub(crate) const M5_RECOVERY_STATE: usize = 0x4ae0;
/// The firmware's progress word in the primary status B, beside the recovery state: the
/// firmware moves it as it works, and a reader that finds it changed since its previous read
/// counts the GPU as progressing.
pub(crate) const M5_PROGRESS: usize = 0x4b00;
pub(crate) const M5_HOST_RECOVERY: usize = 0x4af0;
pub(crate) const M5_RECOVERY_INFO: usize = 0x4298;
pub(crate) const M5_RECOVERY_SLOT_MASK: usize = 0x4ab8;

pub(crate) const M5_STATUS_CONFIG_LINK_WORD: usize = 0x774;

pub(crate) const M5_STATUS_CONFIG_LINK_PTR: usize = 0x778;

pub(crate) const M5_HW_DATA_INSERT: usize = 0x267c;

pub(crate) const M5_HW_DATA_INSERT_LEN: usize = 8;

/// What the M5 graph's HW data and power policy depend on per board. Every board is a T6050
/// running the same firmware build; [`crate::g17::Config::m5`] names the board's.
pub(crate) struct Board {
    /// GPU clusters.
    pub(crate) clusters: u32,
    /// Cores of all clusters.
    pub(crate) gpu_cores: u32,
    /// The core type and firmware revision the HW data names.
    core_revision: (u32, u32),
    /// Main-domain performance states.
    pub(crate) perf_states: &'static [M5PerfState; 14],
    /// Voltage tables in [`Board::perf_states`].
    perf_tables: usize,
    /// Auxiliary-domain performance states.
    aux_perf_states: &'static [M5AuxPerfState; 14],
    /// The relative-frequency curve of [`Board::perf_states`].
    relative_frequency: [u32; 16],
    /// Rows of the board's AON table the firmware reads.
    aon_records: &'static [[u32; 3]],
    /// Register windows the firmware maps, in device-VA order.
    register_windows: &'static [M5RegisterWindowSpec],
    /// The smallest parameter buffer, bytes. Renders carry it in pages too
    /// (`target::min_parameter_buffer_pages()`).
    min_parameter_buffer: u32,
    /// Render block size and private-memory address inventory of the board's
    /// render pools, bytes.
    pub(crate) render_pool: (usize, u64),
    /// The leakage calibration words [`Board::controller_defaults`] are valid for, or `None`
    /// where they hold for every part of the board.
    pub(crate) calibration: Option<[u32; 3]>,
    /// The policy controllers' configuration, where the board has one.
    pub(crate) controller_defaults: Option<fn(&mut [u8]) -> Result<(), InitdataError>>,
    /// Region C's idle standby timer where the board's differs from the image's 1500 us, us.
    pub(crate) idle_standby_us: Option<u32>,
    /// The active memory cache controller mask, where it differs from the inherited 0xf.
    mcc_mask: Option<u64>,
    /// The hotspot at which the thermal governor's fan curve starts, millidegrees C.
    pub(crate) fan_curve_start_mc: i32,
    /// Whether the host encodings are the G17D ones measured on the M5 Ultra
    /// ([`Board::g17d_forms`]).
    g17d_forms: bool,
}

/// M5 Pro: two clusters of ten cores.
pub(crate) const J714: Board = Board {
    clusters: 2,
    gpu_cores: 20,
    core_revision: (0x18, 4),
    perf_states: &J714_GPU_PERF_STATES,
    perf_tables: 2,
    aux_perf_states: &J714_AUX_PERF_STATES,
    relative_frequency: relative_frequency(&J714_GPU_PERF_STATES, 1),
    aon_records: &J714_AON_RECORDS,
    register_windows: &J714_REGISTER_WINDOWS,
    min_parameter_buffer: 0x0020_0000,
    render_pool: (0x40_0000, 1 << 32),
    calibration: Some([0xd2035701, 0xac3a1280, 0x0a281d81]),
    controller_defaults: Some(encode_m5_policy_controller_defaults),
    idle_standby_us: None,
    mcc_mask: None,
    fan_curve_start_mc: 84_000,
    g17d_forms: false,
};

/// M5 Max: four clusters of ten cores.
pub(crate) const J716C: Board = Board {
    clusters: 4,
    gpu_cores: 40,
    core_revision: (0x19, 4),
    perf_states: &J716C_GPU_PERF_STATES,
    perf_tables: 4,
    aux_perf_states: &J716C_AUX_PERF_STATES,
    relative_frequency: relative_frequency(&J716C_GPU_PERF_STATES, 1),
    aon_records: &J716C_AON_RECORDS,
    register_windows: &J716C_REGISTER_WINDOWS,
    min_parameter_buffer: 0x0030_0000,
    render_pool: (0x80_0000, 1 << 33),
    // J714's controller configuration, for a part with J714's calibration only.
    calibration: Some([0xd2035701, 0xac3a1280, 0x0a281d81]),
    controller_defaults: Some(encode_m5_policy_controller_defaults),
    idle_standby_us: None,
    mcc_mask: None,
    fan_curve_start_mc: 84_000,
    g17d_forms: false,
};

/// J775D's performance controllers run from P3.
const J775D_PERF_FLOOR: usize = 3;

/// M5 Ultra: two dies of four clusters of ten cores (G17D). Values from the J775D ADT.
pub(crate) const J775D: Board = Board {
    clusters: 8,
    gpu_cores: 80,
    // Firmware revision 2 for raw revision 0x11.
    core_revision: (0x1a, 2),
    perf_states: &J775D_GPU_PERF_STATES,
    // One voltage table per cluster.
    perf_tables: 8,
    aux_perf_states: &J775D_AUX_PERF_STATES,
    relative_frequency: relative_frequency(&J775D_GPU_PERF_STATES, J775D_PERF_FLOOR),
    aon_records: &J775D_AON_RECORDS,
    register_windows: &J775D_REGISTER_WINDOWS,
    // The minimum parameter buffer for eight clusters.
    min_parameter_buffer: 0x0050_0000,
    // 16 MiB render blocks in a 16 GiB inventory.
    render_pool: (0x100_0000, 1 << 34),
    // The power model's synthetic leakage inputs hold for every J775D: no calibration words.
    calibration: None,
    controller_defaults: Some(encode_m5_policy_controller_defaults_j775d),
    idle_standby_us: Some(3500),
    // The ADT GPU node's active-mcc-bit-vector: eight memory cache controllers.
    mcc_mask: Some(0xff),
    // The SMC raises the fans in its automatic mode from a 97 C hotspot under a sustained
    // full-chip load, with the CPU and GPU still at their top states: the curve starts there,
    // above the SMC's own start rather than below it.
    fan_curve_start_mc: 97_000,
    g17d_forms: true,
};

const _: () = {
    use super::target::min_parameter_buffer_pages;
    assert!(J714.min_parameter_buffer as u64 == min_parameter_buffer_pages(J714.clusters) << 15);
    assert!(J716C.min_parameter_buffer as u64 == min_parameter_buffer_pages(J716C.clusters) << 15);
    assert!(J775D.min_parameter_buffer as u64 == min_parameter_buffer_pages(J775D.clusters) << 15);
};

/// Populate the enabled-USC count from the device's sampled core masks.
/// This count covers the whole device, not one cluster.
pub(crate) fn set_m5_enabled_cores(out: &mut [u8], enabled: u32) -> Result<(), InitdataError> {
    if out.len() != M5_HW_DATA_SIZE {
        return Err(InitdataError::BufferSize);
    }
    if enabled == 0 || enabled > 128 {
        return Err(InitdataError::InterfaceMismatch);
    }
    put_u32(out, 0x10a4, enabled);
    put_u32(out, 0x268c, enabled);
    Ok(())
}

pub(crate) const M5_POLICY_STATE_PTR: usize = 0x8254;

pub(crate) const M5_POLICY_AON_PTD_PTR: usize = 0x11a1c;

pub(crate) const J714_GPU_POWER_SAMPLE_PERIOD_AIC_CLOCKS: u32 = 200_000;

pub(crate) const J714_GPU_PERF_BASE_PSTATE: u32 = 1;

const J714_POWER_POLICY_MINIMUM_TABLE: usize = 0x1cab;

const J714_POWER_POLICY_MINIMUM_ENTRIES: usize = 16 * 32;

pub(crate) const J714_GPU_NUM_PERF_STATES: u32 = 13;

/// A main-domain performance state: MHz, then core and SRAM mV for each voltage table. HW perf
/// lane n carries table n; lanes past the board's table count carry table 0.
pub(crate) type M5PerfState = (u32, [u32; M5_PERF_LANES], [u32; M5_PERF_LANES]);

/// Voltage tables an [`M5PerfState`] can hold.
pub(crate) const M5_PERF_LANES: usize = 8;

/// Two-table states (MHz, two core-voltage tables, two SRAM-voltage tables), the rest 0.
const fn two_tables(states: [(u32, u32, u32, u32, u32); 14]) -> [M5PerfState; 14] {
    let mut out = [(0, [0; M5_PERF_LANES], [0; M5_PERF_LANES]); 14];
    let mut state = 0;
    while state < 14 {
        let (frequency, core0, core1, sram0, sram1) = states[state];
        out[state].0 = frequency;
        out[state].1[0] = core0;
        out[state].1[1] = core1;
        out[state].2[0] = sram0;
        out[state].2[1] = sram1;
        state += 1;
    }
    out
}

// Frequency in MHz, two core-voltage tables, two SRAM-voltage tables (mV).
pub(crate) const J714_GPU_PERF_STATES: [M5PerfState; 14] = two_tables([
    (0, 125, 125, 775, 775), (338, 605, 605, 775, 775),
    (486, 620, 635, 775, 775), (636, 655, 655, 775, 775),
    (796, 705, 705, 775, 775), (888, 715, 715, 775, 775),
    (988, 745, 745, 775, 775), (1084, 770, 770, 775, 775),
    (1182, 800, 800, 800, 800), (1278, 840, 840, 840, 840),
    (1374, 880, 880, 880, 880), (1470, 925, 925, 925, 925),
    (1578, 1000, 1000, 1000, 1000), (1620, 1025, 1040, 1025, 1040),
]);

/// Four-table states (MHz, four core-voltage tables, four SRAM-voltage tables), the rest 0.
const fn four_tables(states: [(u32, [u32; 4], [u32; 4]); 14]) -> [M5PerfState; 14] {
    let mut out = [(0, [0; M5_PERF_LANES], [0; M5_PERF_LANES]); 14];
    let mut state = 0;
    while state < 14 {
        let (frequency, core, sram) = states[state];
        out[state].0 = frequency;
        let mut table = 0;
        while table < 4 {
            out[state].1[table] = core[table];
            out[state].2[table] = sram[table];
            table += 1;
        }
        state += 1;
    }
    out
}

// J716c ADT perf-states / perf-states-sram: four tables (0 == 3, 1 == 2).
pub(crate) const J716C_GPU_PERF_STATES: [M5PerfState; 14] = four_tables([
    (0, [125, 125, 125, 125], [775, 775, 775, 775]),
    (338, [635, 635, 635, 635], [775, 775, 775, 775]),
    (486, [655, 640, 640, 655], [775, 775, 775, 775]),
    (636, [680, 665, 665, 680], [775, 775, 775, 775]),
    (796, [730, 715, 715, 730], [775, 775, 775, 775]),
    (888, [740, 725, 725, 740], [775, 775, 775, 775]),
    (988, [775, 760, 760, 775], [775, 775, 775, 775]),
    (1084, [785, 785, 785, 785], [785, 785, 785, 785]),
    (1182, [805, 810, 810, 805], [805, 810, 810, 805]),
    (1278, [840, 840, 840, 840], [840, 840, 840, 840]),
    (1374, [875, 865, 865, 875], [875, 865, 865, 875]),
    (1470, [920, 905, 905, 920], [920, 905, 905, 920]),
    (1578, [995, 980, 980, 995], [995, 980, 980, 995]),
    (1620, [1015, 1015, 1015, 1015], [1015, 1015, 1015, 1015]),
]);

// J775D ADT sgx perf-states (one table per cluster) and perf-states-sram, mV.
pub(crate) const J775D_GPU_PERF_STATES: [M5PerfState; 14] = [
    (0, [125, 125, 125, 125, 125, 125, 125, 125], [775, 775, 775, 775, 775, 775, 775, 775]),
    (338, [615, 615, 615, 615, 615, 610, 610, 615], [775, 775, 775, 775, 775, 775, 775, 775]),
    (486, [670, 655, 655, 670, 650, 665, 665, 650], [775, 775, 775, 775, 775, 775, 775, 775]),
    (636, [695, 690, 690, 695, 690, 690, 690, 690], [775, 775, 775, 775, 775, 775, 775, 775]),
    (796, [760, 745, 745, 760, 745, 740, 740, 745], [775, 775, 775, 775, 775, 775, 775, 775]),
    (888, [770, 755, 755, 770, 750, 750, 750, 750], [775, 775, 775, 775, 775, 775, 775, 775]),
    (988, [795, 780, 780, 795, 775, 775, 775, 775], [795, 780, 780, 795, 775, 775, 775, 775]),
    (1084, [805, 805, 805, 805, 815, 800, 800, 815], [805, 805, 805, 805, 815, 800, 800, 815]),
    (1182, [815, 810, 810, 815, 825, 820, 820, 825], [815, 810, 810, 815, 825, 820, 820, 825]),
    (1278, [860, 845, 845, 860, 870, 855, 855, 870], [860, 845, 845, 860, 870, 855, 855, 870]),
    (1374, [885, 885, 885, 885, 895, 895, 895, 895], [885, 885, 885, 885, 895, 895, 895, 895]),
    (1470, [930, 930, 930, 930, 940, 935, 935, 940], [930, 930, 930, 930, 940, 935, 935, 940]),
    (1578, [1020, 1005, 1005, 1020, 1030, 1015, 1015, 1030],
        [1020, 1005, 1005, 1020, 1030, 1015, 1015, 1030]),
    (1620, [1045, 1040, 1040, 1045, 1065, 1050, 1050, 1065],
        [1045, 1040, 1040, 1045, 1065, 1050, 1050, 1065]),
];

pub(crate) const J714_GPU_BASE: u64 = 0x23_0000_0000;

#[derive(Copy, Clone)]
pub(crate) struct M5RegisterWindowSpec {
    pub(crate) slot: usize,

    pub(crate) instances: &'static [u64],
    pub(crate) size: u32,

    pub(crate) unk_18: u64,
    pub(crate) flag: u32,
}

const fn jw(slot: usize, instances: &'static [u64], size: u32, unk_18: u64, flag: u32)
    -> M5RegisterWindowSpec {
    M5RegisterWindowSpec { slot, instances, size, unk_18, flag }
}

const G: u64 = J714_GPU_BASE;

const J714_REGISTER_WINDOW_SPECS: [M5RegisterWindowSpec; 29] = [
    jw(0, &[0x2_8041_4000], 0x4000, 0, 2),
    jw(3, &[0x22_2010_4000, 0x22_2210_4000, 0x22_2410_4000, 0x22_2610_4000], 0x18000, 0, 2),
    jw(4, &[0x2_8046_8000], 0x4000, 0, 0),
    jw(9, &[0x2_8425_0000], 0x4000, 0, 2),
    jw(10, &[0x2_8424_0000], 0x4000, 0, 0),
    jw(11, &[0x2_8425_8000], 0x4000, 0, 2),
    jw(12, &[0x5_0965_c000], 0x4000, 0, 2),
    jw(14, &[0x23_8030_0000], 0x8000, 0, 0),
    jw(15, &[0x2_8cdc_0000], 0x24000, 0, 2),
    jw(17, &[G], 0x21500, 0, 2),
    jw(18, &[G + 0xeb_4000], 0x4000, 0, 2),
    jw(20, &[0x23_0300_0000, 0x23_0320_0000], 0x40000, 0, 2),
    jw(22, &[G + 0x100_0000], 0x8000, 0, 2),
    jw(23, &[G + 0xf0_0000], 0x10000, 0, 2),
    jw(24, &[G + 0xec_0000, G + 0xed_0000], 0x10000, 0, 2),
    jw(31, &[G + 0xd0_4000], 0x8000, 0xd0_4000, 2),
    jw(33, &[G + 0xd5_0000], 0x10000, 0xd5_0000, 2),
    jw(34, &[G + 0xd1_0000], 0x4000, 0xd1_0000, 2),
    jw(36, &[G + 0xd4_0000], 0x4000, 0xd4_0000, 2),
    jw(37, &[G + 0xd6_0000], 0x20000, 0xd6_0000, 2),
    jw(38, &[G + 0xd4_4000], 0x4000, 0xd4_4000, 2),
    jw(39, &[G + 0xd4_c000], 0x200, 0xd4_c000, 2),
    jw(40, &[G + 0xe0_0000], 0x4000, 0xe0_0000, 2),
    jw(42, &[G + 0xe4_0000], 0x4000, 0xe4_0000, 2),
    jw(44, &[G + 0xe0_8000, G + 0xe1_0000], 0x8000, 0, 2),
    jw(46, &[G + 0xe3_3800], 0x4000, 0, 2),
    jw(48, &[G + 0xe6_0000], 0x58, 0xe6_0000, 2),
    jw(52, &[G + 0xe4_8000], 0x8000, 0, 2),
    jw(53, &[G + 0x2_3d00], 0x200, 0x2_3d00, 2),
];

const J714_REGISTER_WINDOW_58: M5RegisterWindowSpec =
    jw(58, &[G + 0xe7_3800], 0x4000, 0, 2);

/// J714's windows, ending with slot 58.
const J714_REGISTER_WINDOWS: [M5RegisterWindowSpec; 30] = {
    let mut windows = [J714_REGISTER_WINDOW_58; 30];
    let mut index = 0;
    while index < J714_REGISTER_WINDOW_SPECS.len() {
        windows[index] = J714_REGISTER_WINDOW_SPECS[index];
        index += 1;
    }
    windows
};

/// J714 with all eight AMCCs in slot 3 (J716c amcc-enable-mask 0xff, MCC
/// reg[14..21] + 0x104000) and four instances, one per cluster, in slots 20 and
/// 24, where J714 has two at the same stride. Slot 44 is a fixed pair on every M5
/// part.
const J716C_REGISTER_WINDOW_SPECS: [M5RegisterWindowSpec; 29] = {
    let mut specs = J714_REGISTER_WINDOW_SPECS;
    assert!(specs[1].slot == 3 && specs[11].slot == 20 && specs[14].slot == 24);
    specs[1] = jw(3, &[0x22_2010_4000, 0x22_2210_4000, 0x22_2410_4000, 0x22_2610_4000,
                       0x22_2810_4000, 0x22_2a10_4000, 0x22_2c10_4000, 0x22_2e10_4000],
                  0x18000, 0, 2);
    specs[11] = jw(20, &[0x23_0300_0000, 0x23_0320_0000, 0x23_0340_0000, 0x23_0360_0000],
                   0x40000, 0, 2);
    specs[14] = jw(24, &[G + 0xec_0000, G + 0xed_0000, G + 0xee_0000, G + 0xef_0000],
                   0x10000, 0, 2);
    specs
};

/// J716c's windows, ending with slot 58.
const J716C_REGISTER_WINDOWS: [M5RegisterWindowSpec; 30] = {
    let mut windows = [J714_REGISTER_WINDOW_58; 30];
    let mut index = 0;
    while index < J716C_REGISTER_WINDOW_SPECS.len() {
        windows[index] = J716C_REGISTER_WINDOW_SPECS[index];
        index += 1;
    }
    windows
};

/// Second die of J775D.
const D: u64 = 0x40_0000_0000;

/// J775D: the explicit instances of both dies (ADT), in device-VA order.
const J775D_REGISTER_WINDOWS: [M5RegisterWindowSpec; 32] = [
    jw(0, &[0x2_8041_4000], 0x4000, 0, 2),
    // One per active memory cache controller.
    jw(3, &[0x22_2010_4000, 0x22_2210_4000, 0x22_2410_4000, 0x22_2610_4000,
            0x22_2810_4000, 0x22_2a10_4000, 0x22_2c10_4000, 0x22_2e10_4000], 0x18000, 0, 2),
    jw(4, &[0x2_8046_8000], 0x4000, 0, 0),
    jw(9, &[0x2_8425_0000], 0x4000, 0, 2),
    jw(10, &[0x2_8424_0000], 0x4000, 0, 0),
    jw(11, &[0x2_8425_8000], 0x4000, 0, 2),
    jw(12, &[0x5_0965_c000], 0x4000, 0, 2),
    jw(13, &[0x45_0965_c000], 0x4000, 0, 2),
    jw(14, &[0x23_8030_0000], 0x8000, 0, 0),
    jw(15, &[0x2_8cdc_0000], 0x24000, 0, 2),
    jw(17, &[G], 0x21500, 0, 2),
    jw(18, &[G + 0xeb_4000, G + 0xeb_4000 + D], 0x4000, 0, 2),
    jw(20, &[G + 0x300_0000, G + 0x320_0000, G + 0x340_0000, G + 0x360_0000,
             G + 0x380_0000, G + 0x3a0_0000, G + 0x3c0_0000, G + 0x3e0_0000,
             0x23_035a_8000, 0x23_037a_8000, 0x23_03da_8000, 0x23_03fa_8000], 0x40000, 0, 2),
    jw(22, &[G + 0x100_0000], 0x8000, 0, 2),
    jw(23, &[G + 0xf0_0000, G + 0xf0_0000 + D], 0x10000, 0, 2),
    jw(24, &[G + 0xec_0000, G + 0xed_0000, G + 0xee_0000, G + 0xef_0000,
             G + 0xec_0000 + D, G + 0xed_0000 + D, G + 0xee_0000 + D, G + 0xef_0000 + D],
       0x10000, 0, 2),
    jw(31, &[G + 0xd0_4000], 0x8000, 0xd0_4000, 2),
    jw(33, &[G + 0xd5_0000], 0x10000, 0xd5_0000, 2),
    jw(34, &[G + 0xd1_0000], 0x4000, 0xd1_0000, 2),
    jw(36, &[G + 0xd4_0000], 0x4000, 0xd4_0000, 2),
    jw(37, &[G + 0xd6_0000], 0x20000, 0xd6_0000, 2),
    jw(38, &[G + 0xd4_4000], 0x4000, 0xd4_4000, 2),
    jw(39, &[G + 0xd4_c000], 0x200, 0xd4_c000, 2),
    jw(40, &[G + 0xe0_0000], 0x4000, 0xe0_0000, 2),
    jw(42, &[G + 0xe4_0000], 0x4000, 0xe4_0000, 2),
    jw(44, &[G + 0xe0_8000, G + 0xe1_0000], 0x8000, 0, 2),
    jw(46, &[G + 0xe3_3800], 0x4000, 0, 2),
    jw(48, &[G + 0xe6_0000], 0x58, 0xe6_0000, 2),
    jw(52, &[G + 0xe4_8000], 0x8000, 0, 2),
    jw(53, &[G + 0x2_3d00], 0x200, 0x2_3d00, 2),
    jw(58, &[G + 0xe7_3800], 0x4000, 0, 2),
    jw(59, &[0x23_0358_0000, 0x23_0378_0000, 0x23_03d8_0000, 0x23_03f8_0000], 0x60000, 0, 2),
];

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) struct M5RegisterWindow {
    pub(crate) slot: usize,
    pub(crate) device_va: u64,
    pub(crate) count: usize,
    pub(crate) size: u32,
    pub(crate) unk_18: u64,
    pub(crate) flag: u32,
}

/// The most register windows of any board.
pub(crate) const M5_MAX_REGISTER_WINDOWS: usize = J775D_REGISTER_WINDOWS.len();

impl Board {
    /// Whether the board takes the host encodings measured on the M5 Ultra (G17D) where they
    /// differ from J714's, as `super::compute` and the render registers read them.
    pub(crate) const fn g17d_forms(&self) -> bool {
        self.g17d_forms
    }

    /// The board's `index`th register window.
    pub(crate) fn register_window_spec(&self, index: usize) -> Option<&'static M5RegisterWindowSpec> {
        self.register_windows.get(index)
    }

    /// The board's register windows: the first [`Board::register_window_count`] entries.
    pub(crate) fn register_windows(&self) -> [M5RegisterWindow; M5_MAX_REGISTER_WINDOWS] {
        m5_register_windows(self.register_windows)
    }

    /// Register windows of the board.
    pub(crate) fn register_window_count(&self) -> usize {
        self.register_windows.len()
    }
}

fn m5_register_windows(specs: &'static [M5RegisterWindowSpec])
    -> [M5RegisterWindow; M5_MAX_REGISTER_WINDOWS]
{
    let mask = UAT_PAGE_SIZE as u64 - 1;
    let mut out = [M5RegisterWindow { slot: 0, device_va: 0, count: 0, size: 0, unk_18: 0, flag: 0 };
                   M5_MAX_REGISTER_WINDOWS];
    let mut next: u64 = 0;
    for (window, spec) in out.iter_mut().zip(specs) {
        let va = next + (spec.instances[0] & mask);
        let span = spec.instances.len() as u64 * ((spec.size as u64 + mask) & !mask);
        *window = M5RegisterWindow {
            slot: spec.slot,
            device_va: HW_REGISTER_APERTURE.start + va,
            count: spec.instances.len(),
            size: spec.size,
            unk_18: spec.unk_18,
            flag: spec.flag,
        };
        let end = (va + span + mask) & !mask;
        next = (end + UAT_PAGE_SIZE as u64 + 0x7fff) & !0x7fff;
    }
    out
}

pub(crate) fn encode_m5_register_map(
    board: &Board,
    windows: &[M5RegisterWindow],
    out: &mut [u8],
) -> Result<(), InitdataError> {
    if out.len() != COMMON26_HW_REGISTER_MAP_SLOTS * HW_REGISTER_MAP_ENTRY_SIZE {
        return Err(InitdataError::BufferSize);
    }
    out.fill(0);
    for (index, window) in windows.iter().enumerate() {
        let spec = board.register_windows.get(index).ok_or(InitdataError::RegisterWindow)?;
        if window.slot >= COMMON26_HW_REGISTER_MAP_SLOTS
            || !HW_REGISTER_APERTURE.contains(&window.device_va)
        {
            return Err(InitdataError::RegisterWindow);
        }
        let base = window.slot * HW_REGISTER_MAP_ENTRY_SIZE;
        put_u64(out, base, spec.instances[0]);
        put_u64(out, base + 0x08, window.device_va);
        put_u32(out, base + 0x10, window.count as u32 * window.size);
        put_u32(out, base + 0x14, window.size);
        put_u64(out, base + 0x18, window.unk_18);
        put_u32(out, base + 0x20, window.flag);
    }
    Ok(())
}

pub(crate) fn encode_m5_hw_data(
    board: &Board,
    aliases: [G17PKsmCompletionAliases; M5_25G76_COMPLETION_QUEUE_COUNT],
    qos_high: u64,
    windows: &[M5RegisterWindow],
    out: &mut [u8],
) -> Result<(), InitdataError> {
    if out.len() != M5_HW_DATA_SIZE {
        return Err(InitdataError::BufferSize);
    }
    required_address(qos_high)?;
    let qos_end = qos_high
        .checked_add(G17P_QOS_RESOURCE_SIZE as u64)
        .ok_or(InitdataError::AddressOutOfRange)?;
    required_address(qos_end - 1)?;
    let mut completion = [0; M5_25G76_COMPLETION_QUEUE_COUNT * G17P_KSM_COMPLETION_DESCRIPTOR_STRIDE];
    encode_m5_25g76_completion_table(aliases, &mut completion)?;
    for alias in aliases {
        if qos_high < alias.high + M5_25G76_COMPLETION_BACKING_SIZE as u64 && alias.high < qos_end {
            return Err(InitdataError::PairAliasedObject);
        }
    }

    let g18 = &mut out[..COMMON26_HW_DATA_SIZE];
    g18.fill(0);
    write_relocated_words(M5_25G76_HW_PLATFORM_WORDS, 0, COMMON26_HW_DATA_BANDS, g18)?;
    encode_m5_register_map(board, windows,
        &mut g18[HW_REGISTER_MAP_OFFSET..COMMON26_HW_REGISTER_MAP_END])?;
    put_u32(g18, COMMON26_HW_CHIP_ID_OFFSET, 0x6050);
    put_u32(g18, COMMON26_HW_PERF_STATE_COUNT, J714_GPU_NUM_PERF_STATES);
    put_u32(g18, COMMON26_HW_FIXED_WORD_OFFSET, COMMON26_HW_FIXED_WORD_VALUE);

    put_u32(g18, 0x0fb4, 1);
    // J714's 26A428 HW data carries 1 here. The firmware copies this
    // header at initialization.
    put_u32(g18, 0x0fb8, 1);
    let (core_type, revision) = board.core_revision;
    put_u32(g18, 0x10d4, core_type);
    put_u32(g18, 0x10d8, revision);
    put_u32(g18, 0x10dc, board.gpu_cores);
    put_u32(g18, 0x283a, 1);
    put_u32(g18, 0x283e, 1);
    let states = board.perf_states;
    for (state, &(frequency, core, sram)) in states.iter().enumerate() {
        put_u32(g18, COMMON26_HW_PERF_FREQUENCY + state * 4, frequency);
        put_u32(g18, COMMON26_HW_PERF_SCALE + state * 4, COMMON26_PERF_SCALE_VALUE);

        let relative = frequency * 100 / states[13].0;
        put_u32(g18, COMMON26_HW_PERF_RELATIVE_A + state * 4, relative);
        for lane in 0..COMMON26_HW_PERF_LANES {
            let row = state * COMMON26_HW_PERF_ROW_STRIDE + lane * 4;
            let table = if lane < board.perf_tables { lane } else { 0 };
            put_u32(g18, COMMON26_HW_PERF_CORE_VOLTAGE + row, core[table]);
            put_u32(g18, COMMON26_HW_PERF_SRAM_VOLTAGE + row, sram[table]);
        }
    }
    for index in 0..COMMON26_HW_PERF_ROWS {
        put_u32(g18, COMMON26_HW_PERF_INDEX_MAP + index * 4, index as u32);
        put_u32(g18, COMMON26_HW_PERF_RELATIVE_B + index * 4, board.relative_frequency[index]);
        put_u32(g18, M5_HW_AUX_RELATIVE_FREQUENCY + index * 4, board.relative_frequency[index]);
    }
    encode_m5_aux_perf_states(board, g18);
    let table = COMMON26_KSM_COMPLETION_TABLE_OFFSET;
    g18[table..table + completion.len()].copy_from_slice(&completion);
    put_u64(g18, COMMON26_QOS_POINTER_OFFSET, qos_high);

    let insert = M5_HW_DATA_INSERT;
    let len = M5_HW_DATA_INSERT_LEN;
    out.copy_within(insert..COMMON26_HW_DATA_SIZE, insert + len);
    out[insert..insert + len].fill(0);
    put_u32(out, 0x2660, 0);
    put_u32(out, 0x2728, 0);
    put_u32(out, 0x272c, 1);
    if let Some(mask) = board.mcc_mask {
        put_u64(out, 0x10a8, mask);
    }
    // The inherited J813 minimum parameter-buffer size is replaced per board.
    put_u32(out, 0x26b0, board.min_parameter_buffer);
    Ok(())
}

pub(crate) const M5_POLICY_MAXIMUM_POWER: usize = 0xa18;

/// Region C's idle standby timer: how long the idle GPU waits before it goes to standby, us.
pub(crate) fn set_m5_region_idle_standby(out: &mut [u8], microseconds: u32)
    -> Result<(), InitdataError> {
    if out.len() < M5_REGION_C_ALLOCATION { return Err(InitdataError::BufferSize); }
    if microseconds == 0 { return Err(InitdataError::InterfaceMismatch); }
    put_u32(out, 0x7c8, microseconds);
    Ok(())
}

/// Apply the calibrated maximum shared by the policy and host power input.
pub(crate) fn set_m5_region_maximum_power(out: &mut [u8], maximum: u32)
    -> Result<(), InitdataError> {
    if out.len() < M5_REGION_C_ALLOCATION { return Err(InitdataError::BufferSize); }
    if maximum == 0 { return Err(InitdataError::InterfaceMismatch); }
    put_u32(out, 0x88, maximum);
    Ok(())
}

pub(crate) fn encode_m5_region_c(out: &mut [u8]) -> Result<(), InitdataError> {
    if out.len() < M5_REGION_C_ALLOCATION {
        return Err(InitdataError::BufferSize);
    }
    out.fill(0);
    encode_common26_region_c(&mut out[..REGION_C_SIZE])?;

    out.copy_within(0xe10..0xe78, 0x1c34);
    out[0x9e4..0x1c34].fill(0);
    out.copy_within(0x7f4..0x9e4, 0x7f8);
    put_u32(out, 0x7f4, 1);

    put_u32(out, 0x9e0, 50);

    put_u32(out, 0x08c, J714_GPU_NUM_PERF_STATES * 100);
    put_u32(out, 0x090, J714_GPU_NUM_PERF_STATES * 100);
    put_u32(out, 0x09c, (J714_GPU_NUM_PERF_STATES + 1) * 100);
    put_u32(out, 0x034, 1);
    put_u32(out, 0x038, 1);
    put_u32(out, 0x7e4, 0x3f19_999a);
    put_u32(out, 0x1c30, 250);
    const FW_UTIL_TUNABLES: [u8; 32] = [
        1, 0, 0, 0, 0, 2, 2, 1, 1, 40, 10, 1, 1, 1, 2, 40,
        10, 1, 1, 1, 2, 40, 10, 1, 1, 1, 1, 40, 10, 1, 1, 0,
    ];
    out[0x9e8..0xa08].copy_from_slice(&FW_UTIL_TUNABLES);
    Ok(())
}

pub(crate) fn encode_m5_primary_status_b(
    fwctl_state: u64,
    fwctl_ring: u64,
    status_b: u64,
    out: &mut [u8],
) -> Result<(), InitdataError> {
    if out.len() != M5_STATUS_B_SPAN {
        return Err(InitdataError::BufferSize);
    }
    required_address(fwctl_state)?;
    required_address(fwctl_ring)?;
    required_address(status_b)?;
    let config = M5_STATUS_CONFIG_IN_STATUS_B;
    out.fill(0);
    encode_status_block(&mut out[..STATUS_BLOCK_SIZE])?;
    put_u64(out, COMMON26_STATUS_B_FWCTL_STATE, fwctl_state);
    put_u64(out, COMMON26_STATUS_B_FWCTL_RING, fwctl_ring);
    put_u32(out, COMMON26_STATUS_B_CONFIG_HEADER, 1);
    put_u32(out, COMMON26_STATUS_B_LINK_WORD, 0);
    put_u64(out, COMMON26_STATUS_B_LINK_PTR,
            status_b + (config + M5_STATUS_CONFIG_LINK_WORD) as u64);
    write_relocated_words(M5_25G76_STATUS_CONFIG_WORDS, 0, COMMON26_STATUS_CONFIG_BANDS,
                          &mut out[config..config + M5_STATUS_CONFIG_SIZE])?;
    put_u32(out, config + M5_STATUS_CONFIG_LINK_WORD, 1);
    put_u64(out, config + M5_STATUS_CONFIG_LINK_PTR,
            status_b + COMMON26_STATUS_B_LINK_WORD as u64);
    Ok(())
}

pub(crate) fn encode_m5_private_cluster(
    private_base: u64,
    fwctl_state: u64,
    out: &mut [u8],
) -> Result<(), InitdataError> {
    let layout = M5_PRIVATE_LAYOUT;
    if out.len() != layout.total {
        return Err(InitdataError::BufferSize);
    }
    required_address(private_base)?;
    let private_end = private_base
        .checked_add(layout.total as u64)
        .ok_or(InitdataError::AddressOutOfRange)?;
    required_address(private_end - 1)?;
    required_address(fwctl_state)?;
    let fwctl_ring = fwctl_state
        .checked_add(COMMON26_FWCTL_RING_OFFSET as u64)
        .ok_or(InitdataError::AddressOutOfRange)?;
    required_address(fwctl_ring)?;
    if fwctl_state < private_end && private_base < fwctl_ring + CONTROL_RECORD_SIZE as u64 {
        return Err(InitdataError::PairAliasedObject);
    }
    out.fill(0);
    let status_b = layout.status_b;
    let status_a = layout.roles[0].status_a;
    encode_m5_primary_status_b(fwctl_state, fwctl_ring, private_base + status_b as u64,
                                        &mut out[status_b..status_a])?;
    for role in layout.roles {
        encode_status_block(&mut out[role.status_a..role.status_a + STATUS_BLOCK_SIZE])?;
    }

    let extra = layout.secondary_extra_1;
    encode_secondary_root_extra_1_for_profile(RootProfile::J873G18G26A428,
        &mut out[extra..extra + STATUS_BLOCK_SIZE])?;
    put_u32(out, layout.control_state + CHANNEL_STATE_PRODUCER * CHANNEL_STATE_SPACING,
            CONTROL_OPENING_PRIMARY_PRODUCER);
    put_u32(out, layout.secondary_control_state + CHANNEL_STATE_PRODUCER * CHANNEL_STATE_SPACING,
            CONTROL_OPENING_SECONDARY_PRODUCER);
    Ok(())
}

pub(crate) fn encode_m5_bundle_policy(
    bindings: M5PolicyBindings,
    out: &mut [u8],
) -> Result<(), InitdataError> {
    let layout = M5_SHARED_LAYOUT;
    if out.len() < M5_SHARED_PREFIX_SIZE {
        return Err(InitdataError::BufferSize);
    }
    required_address(bindings.state)?;
    required_address(bindings.aon_ptd)?;
    out[layout.block_c18..M5_SHARED_PREFIX_SIZE].fill(0);
    let m5 = M5_25G76_SHARED_LAYOUT;
    for &(offset, value) in M5_25G76_POLICY_WORDS {
        if (m5.block_1048..m5.block_e10).contains(&offset) {
            put_u32(out, layout.block_1048 + (offset - m5.block_1048), value);
        }
    }
    let policy = &mut out[layout.policy..layout.policy + M5_POLICY_SIZE];
    put_u32(policy, 0x04, J714_GPU_POWER_SAMPLE_PERIOD_AIC_CLOCKS);
    put_u32(policy, 0x08, J714_GPU_POWER_SAMPLE_PERIOD_AIC_CLOCKS);

    for (offset, value) in [
        (0x10, 4), (0x14, 0x3f80_0000), (0x2c, 1), (0x30, 1),
        (0x40, J714_GPU_PERF_BASE_PSTATE * 100), (0x44, 1),
        (0x48, J714_GPU_NUM_PERF_STATES * 100), (0x50, 100),
        (0x60, 4), (0x64, 0x3f80_0000), (0x6c, 1), (0x70, 1),
        (0x80, J714_GPU_PERF_BASE_PSTATE * 100), (0x84, 1),
        (0x88, J714_GPU_NUM_PERF_STATES * 100), (0x90, 100),

        (0x9c, 1), (0xa0, 1), (0xa4, 1), (0xa8, 1),
    ] {
        put_u32(policy, offset, value);
    }

    for index in 0..J714_POWER_POLICY_MINIMUM_ENTRIES {
        put_u16(policy, J714_POWER_POLICY_MINIMUM_TABLE + index * 2, i16::MAX as u16);
    }

    for (offset, value) in [
        (0xb58, 1),
        (0xb60, 0x3f4c_cccd),
        (0xb64, 0x3f7e_b852),
        (0xb68, 0x3e4c_cccd),
        (0xb6c, 0x3ba3_d70a),
        (0xb70, 0x3f85_0e56),
        (0xb74, 0x3f85_0e56),
        (0xb80, 0x4060_0000),
        (0xb84, 0x4060_0000),
    ] {
        put_u32(policy, offset, value);
    }
    put_u64(policy, M5_POLICY_STATE_PTR, bindings.state);
    put_u64(policy, M5_POLICY_AON_PTD_PTR, bindings.aon_ptd);
    Ok(())
}

/// Initial controller inputs for J714 with the matched calibration.
pub(crate) fn encode_m5_policy_controller_defaults(out: &mut [u8]) -> Result<(), InitdataError> {
    if out.len() < M5_SHARED_PREFIX_SIZE { return Err(InitdataError::BufferSize); }
    // Both calibrated hardware curves, as J714's firmware expects them.
    const RELATIVE_POWER_MAIN: [u32; 16] = [
        0x00000000, 0x4106f651, 0x413ea3b9, 0x4180c1c6,
        0x41b433fc, 0x41ccaf2d, 0x41f7a8ee, 0x42104b59,
        0x4228d2f7, 0x424956c5, 0x426d29a9, 0x428da278,
        0x42b4e595, 0x42c80000, 0x00000000, 0x00000000,
    ];
    for (index, &value) in RELATIVE_POWER_MAIN.iter().enumerate() { put_u32(out, 0x19e4 + index * 4, value); }
    const RELATIVE_POWER_AUX: [u32; 16] = [
        0x00000000, 0x4105a0ed, 0x414a36f5, 0x418b74a8,
        0x41c1510d, 0x41df97fb, 0x420be387, 0x4223193c,
        0x423cc4e6, 0x42604354, 0x427ef922, 0x42957243,
        0x42b6ec22, 0x42c80000, 0x00000000, 0x00000000,
    ];
    for (index, &value) in RELATIVE_POWER_AUX.iter().enumerate() { put_u32(out, 0x1a64 + index * 4, value); }
    let policy = &mut out[M5_SHARED_LAYOUT.policy..M5_SHARED_LAYOUT.policy + M5_POLICY_SIZE];
    const INITIAL: &[(usize, u32)] = &[
        (0xd4, 0x3f828f5c),
        (0xd8, 0x3f828f5c),
        (0xdc, 0x3f828f5c),
        (0xe0, 0x3f828f5c),
        (0xe4, 0x3f828f5c),
        (0xe8, 0x3f828f5c),
        (0xec, 0x3f828f5c),
        (0xf0, 0x3f828f5c),
        (0xf4, 0x3f828f5c),
        (0xf8, 0x3f828f5c),
        (0xfc, 0x3f828f5c),
        (0x100, 0x3f828f5c),
        (0x104, 0x3f828f5c),
        (0x108, 0x3f828f5c),
        (0x9cc, 0x271),
        (0x9d8, 0x3f7f2e9f),
        (0x9e0, 0x3b516154),
        (0x9e8, 0x3ca59586),
        (0x9f4, 0x47c12200),
        (0x9f8, 0x40a90fdb),
        (0xa00, 0xbc576554),
        (0xa04, 0x28),
        (0xa08, J714_GPU_NUM_PERF_STATES * 100),
        (0xa0c, J714_GPU_NUM_PERF_STATES * 100),
        (M5_POLICY_MAXIMUM_POWER, 0x18244),
        (0xa4c, J714_GPU_NUM_PERF_STATES * 100),
        (0xa84, 0x8),
        (0xa90, 0x3f000000),
        (0xa98, 0x3f000000),
        (0xaa0, 0x3e0b4396),
        (0xaac, 0x47800000),
        (0xab0, 0x3e99999a),
        (0xabc, 0x28),
        (0xac0, J714_GPU_NUM_PERF_STATES * 100),
        (0xad0, 0x18244),
        (0xadc, 0x10),
        (0xae4, 0x5dc00),
        (0xb3c, 0x4b),
        (0xb44, 0x55),
        (0xb48, 0x32),
        (0xb4c, 0x6),
        (0xb50, 0x1),
        (0xb54, 0x6),
        (0xb7c, 0x42be0000),
        (0xb88, 0x414a1af3),
        (0xb8c, 0x64),
        (0xb90, J714_GPU_NUM_PERF_STATES * 100),
        (0xb94, 0x64),
        (0xba0, 0x4a),
        (0xba8, 0x1),
        (0xbd4, 0x64),
        (0xbf4, 0x401),
        (0xbf8, 0x6400),
        (0xbfc, 0xa00),
        (0xc00, 0x1f4000),
        (0xc04, 0x12c00),
        (0xc08, 0xfa000),
        (0xc0c, 0x12c00),
        (0xc10, 0x7d000),
        (0xc14, 0x12c00),
        (0xc18, 0x3e800),
        (0xc1c, 0x19000),
        (0xc20, 0x100),
        (0xc78, 0x1),
        (0x22ac, 0x47800000),
        (0x22b0, 0x42c80000),
        (0x22b4, 0x44a28000),
        (0x22b8, 0x654),
        (0x22cc, 0x64),
        (0x22d4, 0x42c80000),
        (0x22d8, J714_GPU_NUM_PERF_STATES * 100),
        (0x22e0, 0x3f4ccccd),
        (0x22e4, 0x3e4ccccd),
        (0x2328, 0x6fcc3fcc),
        (0x232c, 0x6fcc3fcc),
        (0x234c, 0x1f40),
        (0x2354, 0x4),
        (0x2368, 0x3f800000),
        (0x2370, 0x40ad5555),
        (0x237c, 0x47800000),
        (0x2380, 0x42080000),
        (0x238c, 0x28),
        (0x2390, J714_GPU_NUM_PERF_STATES * 100),
        (0x2394, J714_GPU_NUM_PERF_STATES * 100),
        (0x23a0, 0x2af8),
        (0x23d4, J714_GPU_NUM_PERF_STATES * 100),
        (0x2cec, 0x20),
        (0x2cf0, 0x42000006),
        (0x2cfc, 0x3),
        (0x2d24, 0x1f40),
        (0x2d2c, 0x4),
        (0x2d40, 0x3f800000),
        (0x2d48, 0x40855555),
        (0x2d54, 0x47800000),
        (0x2d58, 0x42080000),
        (0x2d64, 0x64),
        (0x2d68, J714_GPU_NUM_PERF_STATES * 100),
        (0x2d6c, J714_GPU_NUM_PERF_STATES * 100),
        (0x2d78, 0x2bc0),
        (0x2dac, J714_GPU_NUM_PERF_STATES * 100),
        (0x36c4, 0x20),
        (0x36c8, 0x42000006),
        (0x3fdc, 0xb8520000),
        (0x3fe0, 0x3f7e),
        (0x3fe4, 0xd70a0000),
        (0x3fe8, 0x3ba3),
        (0x3ff0, 0x43c8),
        (0x3ffc, 0x4780),
        (0x4000, 0x42c8),
        (0x4008, 0x384bac8),
        (0x400c, 0x5140000),
        (0x4010, 0x3e80000),
        (0x4018, 0x80000000),
        (0x401c, 0x26ac453b),
        (0x4050, 0x5140000),
        (0x406c, 0xc80000),
        (0x4124, J714_GPU_NUM_PERF_STATES * 100),
        (0x4138, 0x1),
        (0x41cc, 0x10001),
        (0x41d0, 0x40000),
        (0x41e8, 0x23f0000),
        (0x41ec, 0x10000),
        (0x41f0, 0x10000),
        (0x4208, 0x2400000),
        (0x420c, 0x10000),
        (0x4210, 0x10000),
        (0x4228, 0x2410000),
        (0x42d4, 0x640000),
        (0x42f8, 0x1),
        (0x438c, 0x10001),
        (0x4390, 0x40000),
        (0x43a8, 0x25b0000),
        (0x43ac, 0x10000),
        (0x43b0, 0x10000),
        (0x43c8, 0x25c0000),
        (0x43cc, 0x10000),
        (0x43d0, 0x10000),
        (0x43e8, 0x25d0000),
        (0x4494, 0x640000),
        (0x449c, 0x1),
        (0x44a4, 0x18244),
        (0x44a8, 0x18244),
        (0x44ac, 0x18244),
        (0x44bc, 0x3f800000),
        (0x44c0, 0x4),
        (0x44c4, 0x8),
        (0x44c8, 0x2ee00),
        (0x4598, 0x7c),
        (0x45a4, 0x3f77bdef),
        (0x45ac, 0x3d042108),
        (0x45b4, 0x3cf5c290),
        (0x45c0, 0x47800000),
        (0x45c4, 0x3f19999a),
        (0x45d0, 0x28),
        (0x45d4, J714_GPU_NUM_PERF_STATES * 100),
        (0x45d8, J714_GPU_NUM_PERF_STATES * 100),
        (0x45e0, 0x47c12200),
        (0x45e4, 0x18244),
        (0x45f0, 0xf8),
        (0x45f8, 0x5ad200),
        (0x4618, J714_GPU_NUM_PERF_STATES * 100),
        (0x4658, 0x1),
        (0x4728, 0x32),
        (0x472c, 0x1),
        (0x4734, 0x3f638e39),
        (0x4738, 0x3f2aaaab),
        (0x473c, 0x3de38e39),
        (0x4740, 0x3eaaaaab),
        (0x4744, 0xbecccccd),
        (0x4748, 0xbecccccd),
        (0x4750, 0x47800000),
        (0x4754, 0xc0a00000),
        (0x4758, 0xc0a00000),
        (0x4760, 0x64),
        (0x4764, J714_GPU_NUM_PERF_STATES * 100),
        (0x4768, 0x64),
        (0x4770, 0x45fa0000),
        (0x4774, 0x578),
        (0x477c, 0xe),
        (0x4780, 0x48),
        (0x4784, 0x18),
        (0x4788, 0x1a5e00),
        (0x4790, 0x8ca00),
        (0x479c, 0x47800000),
        (0x47c4, 0x1f40),
        (0x47c8, 0x4),
        (0x47cc, 0x9c4),
        (0x47d0, 0x258),
        (0x47d4, 0x2),
        (0x47d8, 0x4),
        (0x47dc, 0x32),
        (0x480c, 0x47800000),
        (0x481c, 0x28),
        (0x4820, J714_GPU_NUM_PERF_STATES * 100),
        (0x48ec, 0x1),
        (0x48f0, 0x1),
        (0x4910, 0xdac),
        (0x4914, 0x3e8),
        (0x4918, 0xbb8),
        (0x49b8, J714_GPU_NUM_PERF_STATES * 100),
        (0x49bc, J714_GPU_NUM_PERF_STATES * 100),
        (0x49c0, 0x64),
        (0x49c4, 0x20),
        (0x49cc, 0x1),
        (0x49d0, 0x1),
        (0x49fc, 0x1f4),
        (0x4a98, J714_GPU_NUM_PERF_STATES * 100),
        (0x4a9c, J714_GPU_NUM_PERF_STATES * 100),
        (0x4aa0, 0x64),
        (0x4aa4, 0x20),
        (0x4ac8, 0x4),
        (0x4ad4, 0x3f780000),
        (0x4adc, 0x3d000000),
        (0x4ae4, 0x40a00000),
        (0x4af0, 0x47800000),
        (0x4af4, 0x41700000),
        (0x4b00, 0x4b0),
        (0x4b04, J714_GPU_NUM_PERF_STATES * 100),
        (0x4b08, J714_GPU_NUM_PERF_STATES * 100),
        (0x4b14, J714_GPU_NUM_PERF_STATES * 100),
        (0x4b48, J714_GPU_NUM_PERF_STATES * 100),
        (0x4b94, 0x42c80000),
        (0x4ba4, 0x3),
        (0x4ba8, 0x6fcc3fcc),
        (0x4bac, 0x6fcc3fcc),
        (0x4bcc, 0x7d),
        (0x4bd0, 0x1),
        (0x11a24, 0x67305452),
        (0x11a28, 0x67315452),
        (0x11a6c, 0x72305452),
        (0x121a4, 0x1),
        (0x122e0, 0x186a),
        (0x121b0, 0x44d5c000),
        (0x121b4, 0x44d20000),
        (0x12230, 0x43140000),
        (0x12234, 0x43140000),
        (0x12280, 0x43d60000),
    ];
    for &(offset, value) in INITIAL { put_u32(policy, offset, value); }
    Ok(())
}

/// Controller inputs for J775D: the J714 defaults, with the Ultra's power model over them. The
/// model covers the main and auxiliary operating points of both dies (64 cores, 110 C) with
/// synthetic leakage inputs (1350 per cluster core and per die auxiliary domain, 51 per cluster
/// SRAM), which hold for every J775D: P13 sums to 345262 mW. Then the relative power curves of
/// that model, the controllers' gains and utilization thresholds, and the temperature-sensor
/// channel masks of both dies (160 GPU and 8 auxiliary-domain channels; the hotspot set is both).
pub(crate) fn encode_m5_policy_controller_defaults_j775d(out: &mut [u8]) -> Result<(), InitdataError> {
    encode_m5_policy_controller_defaults(out)?;
    const RELATIVE_POWER_MAIN: [u32; 14] = [
        0x0000_0000, 0x4107_2435, 0x414b_7ecd, 0x4187_dfa6, 0x41be_759c, 0x41d6_f4a2, 0x41ff_340b,
        0x4215_4139, 0x4226_cc22, 0x4247_2db8, 0x426b_3fa4, 0x428b_aed3, 0x42b6_c65b, 0x42c8_0000,
    ];
    for (index, &value) in RELATIVE_POWER_MAIN.iter().enumerate() { put_u32(out, 0x19e4 + index * 4, value); }
    const RELATIVE_POWER_AUX: [u32; 14] = [
        0x0000_0000, 0x4106_2d7e, 0x4154_96b9, 0x4189_3d11, 0x41ba_e22e, 0x41d5_33ed, 0x4202_7bcd,
        0x421c_12dd, 0x4230_6915, 0x4250_6a36, 0x427d_e40c, 0x4294_0cc0, 0x42bd_bd8e, 0x42c8_0000,
    ];
    for (index, &value) in RELATIVE_POWER_AUX.iter().enumerate() { put_u32(out, 0x1a64 + index * 4, value); }
    let policy = &mut out[M5_SHARED_LAYOUT.policy..M5_SHARED_LAYOUT.policy + M5_POLICY_SIZE];
    const MAXIMUM_POWER: u32 = 345_262;
    const MAXIMUM_POWER_F32: u32 = 0x48a8_95c0;
    for offset in [M5_POLICY_MAXIMUM_POWER, 0xad0, 0x44a4, 0x44a8, 0x44ac, 0x45e4] {
        put_u32(policy, offset, MAXIMUM_POWER);
    }
    for offset in [0x9f4, 0x45e0] {
        put_u32(policy, offset, MAXIMUM_POWER_F32);
    }
    // Leakage: eight clusters' cores (1350.0) and SRAM (51.0), then each die's auxiliary domain.
    for index in 0..8 {
        put_u32(policy, 0x121b0 + index * 4, 0x44a8_c000);
        put_u32(policy, 0x12230 + index * 4, 0x424c_0000);
    }
    for index in 0..2 {
        put_u32(policy, 0x12280 + index * 4, 0x44a8_c000);
    }
    // The power controller's integral and proportional gains (0.08, 0.005), the second
    // controller's (0.01, 0.28), and the utilization thresholds 75 and 95.
    for (offset, value) in [(0xaa0, 0x3da3_d70b), (0xab0, 0x3ba3_d70a), (0x45b4, 0x3c23_d70b),
        (0x45c4, 0x3e8f_5c29), (0xb44, 75), (0xb48, 95)] {
        put_u32(policy, offset, value);
    }
    // The performance controllers' floor, P3, as state x 100 in the three floor words (the last
    // byte-packed in the boost block); HwData's relative-frequency tables are based at it too.
    for offset in [0xb94, 0xbd4, 0xbf9] {
        put_u32(policy, offset, J775D_PERF_FLOOR as u32 * 100);
    }
    // The fast-active boost may continue while the GPU idles for 20 percent of a boost window
    // (byte-packed in the boost block) on the Ultra's core type; the default is 10.
    policy[0xbfd] = 20;
    // The AON temperature records the GPU's and the auxiliary domain's controllers read, with
    // each list's found mask: both dies' g0TR-g3TR and r0TR-r1TR, the six records of J775D's
    // AON table (encode_common26_aon_ptd).
    for (index, tag) in [0x6730_5452, 0x6731_5452, 0x6732_5452, 0x6733_5452].into_iter().enumerate() {
        put_u32(policy, 0x11a24 + index * 4, tag);
    }
    put_u32(policy, 0x11a64, 0xf);
    for (index, tag) in [0x7230_5452, 0x7231_5452].into_iter().enumerate() {
        put_u32(policy, 0x11a6c + index * 4, tag);
    }
    put_u32(policy, 0x11a74, 0x3);
    const DIE_CHANNELS: u64 = 0x6fcc_3fcc_6fcc_3fcc;
    for (offset, words) in [
        (0x2324, [0x6fcc_3fcc_0000_0000, DIE_CHANNELS, DIE_CHANNELS, DIE_CHANNELS, 0x6fcc_3fcc]),
        (0x2cfc, [0x0303_0303, 0, 0, 0, 0]),
        (0x4ba4, [0x6fcc_3fcc_0303_0303, DIE_CHANNELS, DIE_CHANNELS, DIE_CHANNELS, 0x6fcc_3fcc]),
    ] {
        for (index, word) in words.into_iter().enumerate() {
            put_u64(policy, offset + index * 8, word);
        }
    }
    Ok(())
}
