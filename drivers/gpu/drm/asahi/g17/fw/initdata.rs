// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! G17 firmware initialization data.
//!
//! Each of the two firmware instances (primary and secondary) is handed one
//! root at boot. The roots name the objects the firmware needs before any
//! work is submitted:
//!
//! - region A and region C, shared by both instances;
//! - one main configuration per instance, holding its channel table;
//! - the hardware-data bundle, one allocation holding the hardware data, the
//!   power configuration, the work rings and both main configurations with
//!   their device-control rings;
//! - per-instance status blocks inside the private cluster, the fixed-address
//!   arena the firmware also writes at fixed offsets.
//!
//! This module defines those layouts and the pure functions that fill them.
//! Allocation, mapping and publication belong to the builder. Initialization
//! requires exclusive access before either firmware instance can access the
//! object. These constructors must not be used to update a published object:
//! its live fields require individual accesses of the widths documented below.
//!
//! All objects start out zeroed. An `unk_<offset>` field documents the value
//! the firmware requires there; an `unk_` field without a doc comment is zero.

use kernel::prelude::*;

use super::clear;

pub(crate) mod v25g83;

/// Interface version at root `+0x00`; the firmware rejects any other.
pub(crate) const INTERFACE_VERSION: [u16; 4] = [0x04c0, 0x0396, 0xa322, 0x0c8a];

/// UAT page size, as a shift.
pub(crate) const PAGE_BITS: u32 = 14;
/// UAT page size.
pub(crate) const PAGE_SIZE: usize = 1 << PAGE_BITS;
/// Page-table levels below the root.
const UAT_LEVELS: usize = 3;
/// Address bits resolved by one page-table level.
const BITS_PER_LEVEL: u32 = PAGE_BITS - 3;

/// Firmware instance, encoded in its root.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u32)]
pub(crate) enum Role {
    /// Runs the scheduler, the device-control ring and all work channels.
    Primary = 0,
    /// Control-only peer of the primary.
    Secondary = 1,
}

// ---------------------------------------------------------------------------
// Roots
// ---------------------------------------------------------------------------

/// One UAT page-table level, as the firmware walks it.
#[repr(C, packed)]
#[derive(Copy, Clone, Debug)]
pub(crate) struct LevelDescriptor {
    /// Required: 8.
    pub(crate) unk_0: u8,
    /// Page size shift of the tables at this level.
    pub(crate) table_page_bits: u8,
    /// Page size shift of the pages they map.
    pub(crate) page_bits: u8,
    /// Lowest input-address bit indexing this level.
    pub(crate) index_shift: u8,
    /// Entries per table.
    pub(crate) entries: u16,
    /// Table size in bytes.
    pub(crate) table_size: u16,
    /// Required: 1.
    pub(crate) unk_8: u64,
    /// Output-address bits of an entry.
    pub(crate) phys_mask: u64,
    /// Input-address bits indexing this level.
    pub(crate) index_mask: u64,
}

// SAFETY: `LevelDescriptor` consists of integers and arrays of integers only.
unsafe impl Zeroable for LevelDescriptor {}

static_assert!(size_of::<LevelDescriptor>() == 0x20);

impl LevelDescriptor {
    /// Returns the descriptor of `level` (0 = top) for a UAT with `ias`
    /// input-address and `oas` output-address bits.
    const fn new(level: u32, ias: u32, oas: u32) -> Self {
        let index_shift = PAGE_BITS + (UAT_LEVELS as u32 - 1 - level) * BITS_PER_LEVEL;
        let index_bits = if level == 0 {
            ias - index_shift
        } else {
            BITS_PER_LEVEL
        };
        let entries = 1u64 << index_bits;
        Self {
            unk_0: 8,
            table_page_bits: PAGE_BITS as u8,
            page_bits: PAGE_BITS as u8,
            index_shift: index_shift as u8,
            entries: entries as u16,
            table_size: PAGE_SIZE as u16,
            unk_8: 1,
            phys_mask: ((1u64 << oas) - 1) & !(PAGE_SIZE as u64 - 1),
            index_mask: (entries - 1) << index_shift,
        }
    }
}

/// Addresses and geometry common to both roots.
pub(crate) struct RootArgs {
    /// Region A.
    pub(crate) region_a_va: u64,
    /// Region C.
    pub(crate) region_c_va: u64,
    /// This instance's main configuration.
    pub(crate) main_config_va: u64,
    /// This instance's status A block.
    pub(crate) status_a_va: u64,
    /// UAT input-address bits of the firmware's context.
    pub(crate) ias: u32,
    /// UAT output-address bits.
    pub(crate) oas: u32,
}

/// Root of the primary instance; the secondary extends it ([`SecondaryRoot`]).
#[repr(C, packed)]
#[derive(Copy, Clone, Debug)]
pub(crate) struct Root {
    /// [`INTERFACE_VERSION`].
    pub(crate) version: [u16; 4],
    /// Region A.
    pub(crate) region_a_va: u64,
    pub(crate) unk_10: u64,
    /// Main configuration of this instance.
    pub(crate) main_config_va: u64,
    /// Region C.
    pub(crate) region_c_va: u64,
    /// [`Role`] of this instance.
    pub(crate) role: u32,
    /// Required: 1.
    pub(crate) unk_2c: u32,
    /// UAT page size.
    pub(crate) page_size: u16,
    /// UAT page size shift.
    pub(crate) page_bits: u8,
    /// Number of page-table levels.
    pub(crate) levels: u8,
    /// Page-table levels, top first.
    pub(crate) level: [LevelDescriptor; UAT_LEVELS],
    pub(crate) unk_94: [u8; 0x14],
    /// Status A block of this instance.
    pub(crate) status_a_va: u64,
    /// Primary status B ([`PrimaryStatusB`]); zero for the secondary.
    pub(crate) status_b_va: u64,
}

// SAFETY: `Root` consists of integers and arrays of integers only.
unsafe impl Zeroable for Root {}

static_assert!(core::mem::offset_of!(Root, role) == 0x28);
static_assert!(core::mem::offset_of!(Root, level) == 0x34);
static_assert!(core::mem::offset_of!(Root, status_a_va) == 0xa8);
static_assert!(size_of::<Root>() == 0xb8);

impl Root {
    fn new(role: Role, args: &RootArgs, status_b_va: u64) -> Self {
        Self {
            version: INTERFACE_VERSION,
            region_a_va: args.region_a_va,
            unk_10: 0,
            main_config_va: args.main_config_va,
            region_c_va: args.region_c_va,
            role: role as u32,
            unk_2c: 1,
            page_size: PAGE_SIZE as u16,
            page_bits: PAGE_BITS as u8,
            levels: UAT_LEVELS as u8,
            level: [
                LevelDescriptor::new(0, args.ias, args.oas),
                LevelDescriptor::new(1, args.ias, args.oas),
                LevelDescriptor::new(2, args.ias, args.oas),
            ],
            unk_94: [0; 0x14],
            status_a_va: args.status_a_va,
            status_b_va,
        }
    }

    /// Returns the primary root.
    pub(crate) fn primary(args: &RootArgs, status_b_va: u64) -> Self {
        Self::new(Role::Primary, args, status_b_va)
    }
}

/// Root of the secondary instance.
#[repr(C, packed)]
#[derive(Copy, Clone, Debug)]
pub(crate) struct SecondaryRoot {
    /// Common part; the secondary has no status B.
    pub(crate) root: Root,
    /// The primary's [`PowerState`].
    pub(crate) power_state_va: u64,
    /// The secondary's [`SecondaryConfig`].
    pub(crate) config_va: u64,
}

// SAFETY: `SecondaryRoot` consists of integers and arrays of integers only.
unsafe impl Zeroable for SecondaryRoot {}

static_assert!(size_of::<SecondaryRoot>() == 0xc8);

impl SecondaryRoot {
    /// Returns the secondary root.
    pub(crate) fn new(args: &RootArgs, power_state_va: u64, config_va: u64) -> Self {
        Self {
            root: Root::new(Role::Secondary, args, 0),
            power_state_va,
            config_va,
        }
    }

    /// Returns the root of a single instance that carries both roles: the primary root, with
    /// status B, extended by the power-state and configuration pointers.
    pub(crate) fn merged(
        args: &RootArgs,
        status_b_va: u64,
        power_state_va: u64,
        config_va: u64,
    ) -> Self {
        Self {
            root: Root::new(Role::Primary, args, status_b_va),
            power_state_va,
            config_va,
        }
    }
}

/// Offset of the secondary root from the primary root. Both live in one
/// allocation, and the firmware relies on this distance.
pub(crate) const SECONDARY_ROOT_OFFSET: usize = 0x8000;
/// Size of the allocation holding both roots.
pub(crate) const ROOTS_SIZE: usize = 0xc000;

// ---------------------------------------------------------------------------
// Channels
// ---------------------------------------------------------------------------

/// Work channels: TA, 3D and compute at each of four priorities.
pub(crate) const WORK_CHANNELS: usize = 12;
/// Engines with a work channel at each priority (TA, 3D, compute).
const WORK_ENGINES: usize = 3;
/// Work priorities.
const WORK_PRIORITIES: usize = WORK_CHANNELS / WORK_ENGINES;
/// Channel-table index of the device-control channel.
pub(crate) const CONTROL_CHANNEL: usize = 12;
/// Channel-table index of the event and firmware-log rings.
pub(crate) const EVENT_CHANNEL: usize = 13;
/// Channel-table index of the trace and statistics rings.
pub(crate) const TRACE_CHANNEL: usize = 14;
/// Channel-table index of the channel with a single state address.
const CHANNEL_15: usize = 15;
/// Entries in the channel table.
pub(crate) const CHANNELS: usize = 17;

/// One channel-table entry: four GPU addresses whose meaning depends on the
/// channel. Work and control channels name three counters of their
/// [`ChannelState`] and their ring; the event and trace channels name two
/// state/ring pairs.
#[repr(C)]
pub(crate) struct ChannelEntry(pub(crate) [u64; 4]);

// SAFETY: `ChannelEntry` is an array of integers.
unsafe impl Zeroable for ChannelEntry {}

static_assert!(size_of::<ChannelEntry>() == 0x20);

impl ChannelEntry {
    /// Returns the entry of a work or control channel.
    const fn ring(state_va: u64, ring_va: u64) -> Self {
        Self([
            state_va + ChannelState::CONSUMER as u64,
            state_va + ChannelState::UNK_10 as u64,
            state_va + ChannelState::PRODUCER as u64,
            ring_va,
        ])
    }
}

/// State of a work or control channel in an instance's state grid.
///
/// All three counters are live words. The firmware advances `consumer` and
/// `unk_10`; the host writes `producer` with single aligned stores, after the
/// records it publishes are complete. At boot the host stores
/// zero to the control channel's consumer, then zero to `unk_10`, then
/// [`OPENING_PRODUCER`] to its producer, primary instance before secondary,
/// after the rest of the initialization data is written. A full barrier follows
/// both instances' stores. These creation-time writes occur before publication; the firmware moves
/// all three to that value once it has drained the opening record.
#[repr(C)]
pub(crate) struct ChannelState {
    /// Records the firmware has consumed.
    pub(crate) consumer: u32,
    pub(crate) unk_4: [u8; 0xc],
    /// Firmware-owned.
    pub(crate) unk_10: u32,
    pub(crate) unk_14: [u8; 0xc],
    /// Records the host has published.
    pub(crate) producer: u32,
    pub(crate) unk_24: [u8; 0x1c],
}

// SAFETY: `ChannelState` consists of integers and arrays of integers only.
unsafe impl Zeroable for ChannelState {}

static_assert!(core::mem::offset_of!(ChannelState, consumer) == 0);
static_assert!(core::mem::offset_of!(ChannelState, unk_10) == 0x10);
static_assert!(core::mem::offset_of!(ChannelState, producer) == 0x20);
static_assert!(size_of::<ChannelState>() == 0x40);

impl ChannelState {
    pub(crate) const CONSUMER: usize = core::mem::offset_of!(Self, consumer);
    pub(crate) const UNK_10: usize = core::mem::offset_of!(Self, unk_10);
    /// Offset of [`Self::producer`] in the block.
    pub(crate) const PRODUCER: usize = core::mem::offset_of!(Self, producer);
}

/// Offset of the device-control channel's state in an instance's state grid.
pub(crate) const CONTROL_STATE_OFFSET: usize = WORK_CHANNELS * size_of::<ChannelState>();

/// Producer value of the device-control ring of either instance at boot: one
/// opening record.
pub(crate) const OPENING_PRODUCER: u32 = 1;

/// Position of work channel `channel` in the state grid and in the work-ring
/// array: grouped by engine, priorities in descending order.
const fn work_slot(channel: usize) -> usize {
    let engine = channel % WORK_ENGINES;
    let priority = channel / WORK_ENGINES;
    engine * WORK_PRIORITIES + (WORK_PRIORITIES - 1 - priority)
}

/// Offset of work channel `channel`'s state in the state grid.
pub(crate) const fn work_state_offset(channel: usize) -> usize {
    work_slot(channel) * size_of::<ChannelState>()
}

/// Bundle offset of work channel `channel`'s ring.
pub(crate) const fn work_ring_offset(channel: usize) -> usize {
    bundle::WORK_RINGS + work_slot(channel) * bundle::WORK_RING_SIZE
}

/// Offsets in a status area, the region starting at an instance's status A
/// block, of the rings the firmware fills.
pub(crate) mod status {
    /// Event ring state.
    pub(crate) const EVENT_STATE: u64 = 0x40;
    /// Firmware-log state.
    pub(crate) const LOG_STATE: u64 = 0x80;
    /// Trace ring state.
    pub(crate) const TRACE_STATE: u64 = 0x240;
    /// Statistics ring state.
    pub(crate) const STATS_STATE: u64 = 0x280;
    /// Event ring.
    pub(crate) const EVENT_RING: u64 = 0x2c0;
    /// Firmware-log ring.
    pub(crate) const LOG_RING: u64 = 0x4ac0;
    /// The single address in channel 15.
    pub(crate) const UNK_2D2C0: u64 = 0x2d2c0;
    /// Trace ring.
    pub(crate) const TRACE_RING: u64 = 0xa6ac0;
    /// Statistics ring.
    pub(crate) const STATS_RING: u64 = 0xafac0;
}

// ---------------------------------------------------------------------------
// Hardware-data bundle layout
// ---------------------------------------------------------------------------

/// Offsets in the hardware-data bundle.
pub(crate) mod bundle {
    /// Size of the bundle allocation.
    pub(crate) const SIZE: usize = 0x28000;
    /// The [`super::HwData`].
    pub(crate) const HW_DATA: usize = 0;
    /// The five views named by the primary main configuration.
    pub(crate) const VIEWS: [usize; 5] = [0x2740, 0x3380, 0x4400, 0xbc80, 0xbd00];
    /// The [`super::PowerConfig`], inside view 2.
    pub(crate) const POWER_CONFIG: usize = 0x5240;
    /// The [`super::HwDataAux`].
    pub(crate) const AUX: usize = 0x8000;
    /// A second copy of the [`super::PowerConfigHead`], inside views 3 and 4.
    pub(crate) const POWER_CONFIG_COPY: usize = 0xd240;
    /// Region the firmware writes, named twice by each main configuration.
    pub(crate) const FIRMWARE_AREA: usize = 0xc500;
    /// The work rings, see [`super::work_ring_offset`].
    pub(crate) const WORK_RINGS: usize = 0xc5c0;
    /// Size of one work ring (256 slots of 0x18 bytes).
    pub(crate) const WORK_RING_SIZE: usize = 0x1800;
    /// Main configuration of each instance, each followed by its
    /// device-control ring.
    pub(crate) const MAIN_CONFIG: [usize; 2] = [0x1e5c0, 0x22a80];
    /// Separately allocated pages the firmware finds at fixed offsets from
    /// the bundle: a zeroed page named by the primary main configuration.
    pub(crate) const ZERO_PAGE: usize = 0x40000;
    /// The two completion rings ([`super::CompletionRing`]).
    pub(crate) const COMPLETION_RINGS: [usize; 2] = [0x48000, 0x70000];
}

// ---------------------------------------------------------------------------
// Main configuration
// ---------------------------------------------------------------------------

/// Record the host stages in slot 0 of each instance's device-control ring
/// before boot.
#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub(crate) struct OpeningRecord {
    /// Opcode: [`Self::PRIMARY`] or [`Self::SECONDARY`].
    pub(crate) opcode: u32,
    pub(crate) unk_4: [u8; 0x3c],
}

// SAFETY: `OpeningRecord` consists of integers and arrays of integers only.
unsafe impl Zeroable for OpeningRecord {}

static_assert!(size_of::<OpeningRecord>() == 0x40);

impl OpeningRecord {
    const PRIMARY: u32 = 0x16;
    const SECONDARY: u32 = 0x2a;

    /// Returns the opening record of `role`.
    pub(crate) const fn new(role: Role) -> Self {
        Self {
            opcode: match role {
                Role::Primary => Self::PRIMARY,
                Role::Secondary => Self::SECONDARY,
            },
            unk_4: [0; 0x3c],
        }
    }
}

/// Addresses both main configurations need.
pub(crate) struct MainConfigArgs {
    /// Hardware-data bundle.
    pub(crate) bundle_va: u64,
    /// This main configuration.
    pub(crate) main_config_va: u64,
    /// This instance's state grid.
    pub(crate) state_grid_va: u64,
    /// This instance's status A block.
    pub(crate) status_a_va: u64,
    /// Context-0 alias of the parameter-buffer descriptor table.
    pub(crate) pb_descriptors_low_va: u64,
    /// Context-0 alias of the page-pool descriptor table.
    pub(crate) page_pool_descriptors_low_va: u64,
}

/// Addresses only the primary main configuration names.
pub(crate) struct PrimaryViews {
    /// The zeroed page at [`bundle::ZERO_PAGE`].
    pub(crate) zero_page_va: u64,
    /// Parameter-buffer descriptor table.
    pub(crate) pb_descriptors_va: u64,
    /// page-pool descriptor table.
    pub(crate) page_pool_descriptors_va: u64,
}

/// Main configuration of one instance. Its device-control ring
/// (256 records) follows it directly.
#[repr(C, packed)]
pub(crate) struct MainConfig {
    /// Hardware-data bundle ([`HwData`]).
    pub(crate) hw_data_va: u64,
    /// [`bundle::FIRMWARE_AREA`].
    pub(crate) firmware_area_va: [u64; 2],
    pub(crate) unk_18: [u8; 8],
    /// Channel table.
    pub(crate) channels: [ChannelEntry; CHANNELS],
    pub(crate) unk_240: [u8; 0x10],
    /// Non-zero enables firmware log output; the driver leaves it off.
    pub(crate) log_enable: u32,
    /// The bundle views ([`bundle::VIEWS`]); primary only.
    pub(crate) view_va: [u64; 5],
    pub(crate) unk_27c: [u8; 0x54],
    /// [`PrimaryViews::zero_page_va`]; primary only.
    pub(crate) zero_page_va: u64,
    /// Context-0 alias of the parameter-buffer descriptor table.
    pub(crate) pb_descriptors_low_va: u64,
    /// Parameter-buffer descriptor table; primary only.
    pub(crate) pb_descriptors_va: u64,
    /// Context-0 alias of the page-pool descriptor table.
    pub(crate) page_pool_descriptors_low_va: u64,
    /// page-pool descriptor table; primary only.
    pub(crate) page_pool_descriptors_va: u64,
    pub(crate) unk_2f8: [u8; 8],
    /// Required: 4 for the secondary.
    pub(crate) unk_300: u32,
    pub(crate) unk_304: [u8; 0x28],
    /// Required: 1 for the primary.
    pub(crate) unk_32c: u32,
    pub(crate) unk_330: [u8; 0x14],
    /// Required: 0xabcd_abcd for the primary.
    pub(crate) unk_344: u32,
    pub(crate) unk_348: [u8; 0x98],
    /// Required: 0xff for the primary.
    pub(crate) unk_3e0: u32,
    pub(crate) unk_3e4: [u8; 0x8d],
    /// [`PowerConfig`]; secondary only.
    pub(crate) power_config_va: u64,
    pub(crate) unk_479: [u8; 0x47],
}

static_assert!(core::mem::offset_of!(MainConfig, channels) == 0x20);
static_assert!(core::mem::offset_of!(MainConfig, log_enable) == 0x250);
static_assert!(core::mem::offset_of!(MainConfig, view_va) == 0x254);
static_assert!(core::mem::offset_of!(MainConfig, zero_page_va) == 0x2d0);
static_assert!(core::mem::offset_of!(MainConfig, unk_300) == 0x300);
static_assert!(core::mem::offset_of!(MainConfig, unk_3e0) == 0x3e0);
static_assert!(core::mem::offset_of!(MainConfig, power_config_va) == 0x471);
static_assert!(size_of::<MainConfig>() == MainConfig::CONTROL_RING);
static_assert!(
    bundle::MAIN_CONFIG[0] + MainConfig::CONTROL_RING + 256 * size_of::<OpeningRecord>()
        == bundle::MAIN_CONFIG[1]
);

// SAFETY: `MainConfig` consists of integers and arrays of integers only.
unsafe impl Zeroable for MainConfig {}

impl MainConfig {
    /// Offset of the device-control ring from the main configuration.
    pub(crate) const CONTROL_RING: usize = 0x4c0;

    fn init(&mut self, role: Role, args: &MainConfigArgs) {
        clear(self);
        let bundle = args.bundle_va;
        let grid = args.state_grid_va;
        let status_a = args.status_a_va;
        self.hw_data_va = bundle + bundle::HW_DATA as u64;
        self.firmware_area_va = [bundle + bundle::FIRMWARE_AREA as u64; 2];
        if role == Role::Primary {
            for channel in 0..WORK_CHANNELS {
                self.channels[channel] = ChannelEntry::ring(
                    grid + work_state_offset(channel) as u64,
                    bundle + work_ring_offset(channel) as u64,
                );
            }
        }
        self.channels[CONTROL_CHANNEL] = ChannelEntry::ring(
            grid + CONTROL_STATE_OFFSET as u64,
            args.main_config_va + Self::CONTROL_RING as u64,
        );
        self.channels[EVENT_CHANNEL] = ChannelEntry([
            status_a + status::EVENT_STATE,
            status_a + status::EVENT_RING,
            status_a + status::LOG_STATE,
            status_a + status::LOG_RING,
        ]);
        self.channels[TRACE_CHANNEL] = ChannelEntry([
            status_a + status::TRACE_STATE,
            status_a + status::TRACE_RING,
            status_a + status::STATS_STATE,
            status_a + status::STATS_RING,
        ]);
        self.channels[CHANNEL_15] = ChannelEntry([status_a + status::UNK_2D2C0, 0, 0, 0]);
        self.pb_descriptors_low_va = args.pb_descriptors_low_va;
        self.page_pool_descriptors_low_va = args.page_pool_descriptors_low_va;
    }

    /// Initializes the primary main configuration.
    pub(crate) fn init_primary(&mut self, args: &MainConfigArgs, views: &PrimaryViews) {
        self.init(Role::Primary, args);
        self.view_va = bundle::VIEWS.map(|offset| args.bundle_va + offset as u64);
        self.zero_page_va = views.zero_page_va;
        self.pb_descriptors_va = views.pb_descriptors_va;
        self.page_pool_descriptors_va = views.page_pool_descriptors_va;
        self.unk_32c = 1;
        self.unk_344 = 0xabcd_abcd;
        self.unk_3e0 = 0xff;
    }

    /// Initializes the secondary main configuration.
    pub(crate) fn init_secondary(&mut self, args: &MainConfigArgs) {
        self.init(Role::Secondary, args);
        self.unk_300 = 4;
        self.power_config_va = args.bundle_va + bundle::POWER_CONFIG as u64;
    }
}

// ---------------------------------------------------------------------------
// Hardware data
// ---------------------------------------------------------------------------

/// Slots in the hardware-data register map.
pub(crate) const REGISTER_MAP_SLOTS: usize = 53;

/// One register window the firmware maps through the hardware data
/// (per-SoC configuration).
#[derive(Copy, Clone, Debug)]
pub(crate) struct RegisterWindow {
    /// Register-map slot.
    pub(crate) slot: usize,
    /// Physical address of the window.
    pub(crate) phys: u64,
    /// GPU address of the window in the firmware MMIO aperture.
    pub(crate) va: u64,
    /// Size in bytes of one instance; need not be a multiple of the page size.
    pub(crate) size: u32,
    /// Value of [`RegisterMapEntry::unk_18`].
    pub(crate) unk_18: u64,
    /// Value of [`RegisterMapEntry::unk_20`].
    pub(crate) unk_20: u32,
    /// Number of instances of the block. The firmware finds instance `n` at
    /// `va + n * size`; it lives at `phys + n * phys_stride`.
    pub(crate) instances: u32,
    /// Distance between the physical addresses of consecutive instances.
    pub(crate) phys_stride: u64,
}

impl RegisterWindow {
    /// Returns the page-aligned physical address, GPU address and length that
    /// instance `instance` of the window must be mapped with, or `None` if its
    /// physical and GPU addresses differ within a page or the mapped span
    /// overflows.
    pub(crate) fn mapping(&self, instance: u32) -> Option<(u64, u64, u64)> {
        let mask = PAGE_SIZE as u64 - 1;
        let phys = self
            .phys
            .checked_add(self.phys_stride.checked_mul(u64::from(instance))?)?;
        let va = self
            .va
            .checked_add(u64::from(self.size).checked_mul(u64::from(instance))?)?;
        if phys & mask != va & mask {
            return None;
        }
        let start = va & !mask;
        let span = va
            .checked_add(u64::from(self.size))?
            .checked_sub(start)?
            .checked_add(mask)?
            & !mask;
        Some((phys & !mask, start, span))
    }
}

/// Value of [`RegisterMapEntry::unk_20`] in a reserved register-map slot.
const RESERVED_SLOT_UNK_20: u32 = 2;

/// One register-map slot of the hardware data.
#[repr(C)]
pub(crate) struct RegisterMapEntry {
    /// Physical address of the window.
    pub(crate) phys: u64,
    /// GPU address of the window.
    pub(crate) va: u64,
    /// Size in bytes of all instances of the window.
    pub(crate) total_size: u32,
    /// Size in bytes of one instance.
    pub(crate) element_size: u32,
    /// Per-window value, see [`RegisterWindow::unk_18`].
    pub(crate) unk_18: u64,
    /// Per-window value; [`RESERVED_SLOT_UNK_20`] in a reserved slot.
    pub(crate) unk_20: u32,
    pub(crate) unk_24: u32,
}

// SAFETY: `RegisterMapEntry` consists of integers and arrays of integers only.
unsafe impl Zeroable for RegisterMapEntry {}

static_assert!(size_of::<RegisterMapEntry>() == 0x28);

impl RegisterMapEntry {
    const fn window(window: &RegisterWindow) -> Self {
        Self {
            phys: window.phys,
            va: window.va,
            total_size: window.size * window.instances,
            element_size: window.size,
            unk_18: window.unk_18,
            unk_20: window.unk_20,
            unk_24: 0,
        }
    }

    const fn reserved() -> Self {
        Self {
            phys: 0,
            va: 0,
            total_size: 0,
            element_size: 0,
            unk_18: 0,
            unk_20: RESERVED_SLOT_UNK_20,
            unk_24: 0,
        }
    }
}

/// Performance states in each performance table.
pub(crate) const PERF_STATES: usize = 11;
/// Voltage lanes of a performance table.
const PERF_LANES: usize = 16;

/// Most voltage cells of an operating point: one per GPU cluster, at most one per lane.
pub(crate) const OPP_CELLS: usize = PERF_LANES;

/// One GPU operating point, from the device tree, in performance-state
/// order.
#[derive(Copy, Clone, Debug)]
pub(crate) struct Opp {
    /// Frequency in Hz.
    pub(crate) freq_hz: u64,
    /// GPU core voltage in µV: one cell for the whole GPU, or one per cluster.
    pub(crate) volt_uv: [u32; OPP_CELLS],
    /// Number of valid cells in [`Self::volt_uv`].
    pub(crate) cells: usize,
}

/// Per-SoC parts of the performance tables that the operating points do not
/// give.
pub(crate) struct PerfConfig {
    /// Operating point of each state of table A.
    pub(crate) states_a: [u8; PERF_STATES],
    /// Operating point of each state of table B.
    pub(crate) states_b: [u8; PERF_STATES],
    /// Relative performance of each state of table A, in percent.
    pub(crate) relative_a: [u32; PERF_STATES],
    /// Relative performance of each state of table B, in percent.
    pub(crate) relative_b: [u32; PERF_STATES],
    /// Lowest SRAM voltage in mV; a state's SRAM voltage is its core voltage
    /// raised to this floor.
    pub(crate) sram_min_mv: u32,
}

/// One performance table of the hardware data.
#[repr(C, packed)]
pub(crate) struct PerfGroup {
    /// Required: 10.
    pub(crate) unk_0: u32,
    /// Frequency of each state in MHz.
    pub(crate) freq_mhz: [u32; PERF_STATES],
    pub(crate) unk_30: [u8; 0x14],
    /// GPU core voltage of each state in mV, per lane.
    pub(crate) core_mv: [[u32; PERF_LANES]; PERF_STATES],
    pub(crate) unk_304: [u8; 0x140],
    /// SRAM voltage of each state in mV, per lane.
    pub(crate) sram_mv: [[u32; PERF_LANES]; PERF_STATES],
}

// SAFETY: `PerfGroup` consists of integers and arrays of integers only.
unsafe impl Zeroable for PerfGroup {}

static_assert!(core::mem::offset_of!(PerfGroup, core_mv) == 0x44);
static_assert!(core::mem::offset_of!(PerfGroup, sram_mv) == 0x444);
static_assert!(size_of::<PerfGroup>() == 0x704);

/// Frequency and voltages of one performance state.
#[derive(Copy, Clone)]
struct PerfState {
    freq_mhz: u32,
    core_mv: [u32; OPP_CELLS],
    sram_mv: [u32; OPP_CELLS],
    cells: usize,
}

impl PerfGroup {
    /// Fills the table. With one voltage cell, `lanes` is the number of voltage lanes
    /// holding each state's voltages; with one cell per cluster, lane `n` holds cluster
    /// `n`'s. The remaining lanes stay zero.
    fn fill(&mut self, states: &[PerfState; PERF_STATES], lanes: usize) {
        self.unk_0 = 10;
        for (index, state) in states.iter().enumerate() {
            self.freq_mhz[index] = state.freq_mhz;
            for lane in 0..if state.cells == 1 { lanes } else { state.cells } {
                let cell = if state.cells == 1 { 0 } else { lane };
                self.core_mv[index][lane] = state.core_mv[cell];
                self.sram_mv[index][lane] = state.sram_mv[cell];
            }
        }
    }
}

/// Returns the performance states selected by `map` from `opps`.
fn perf_states(
    opps: &[Opp],
    map: &[u8; PERF_STATES],
    sram_min_mv: u32,
) -> Result<[PerfState; PERF_STATES]> {
    let mut states = [PerfState {
        freq_mhz: 0,
        core_mv: [0; OPP_CELLS],
        sram_mv: [0; OPP_CELLS],
        cells: 0,
    }; PERF_STATES];
    for (state, &index) in states.iter_mut().zip(map) {
        let opp = opps.get(index as usize).ok_or(EINVAL)?;
        let core_mv = opp.volt_uv.map(|uv| uv / 1000);
        *state = PerfState {
            freq_mhz: u32::try_from(opp.freq_hz / 1_000_000)?,
            core_mv,
            sram_mv: core_mv.map(|mv| mv.max(sram_min_mv)),
            cells: opp.cells,
        };
    }
    Ok(states)
}

/// One completion ring the firmware fills.
#[repr(C)]
pub(crate) struct CompletionRingDescriptor {
    /// Context-0 alias of the ring.
    pub(crate) low_va: u64,
    /// Firmware alias of the ring.
    pub(crate) high_va: u64,
    /// Entries in the ring.
    pub(crate) entries: u32,
    /// Size of one entry.
    pub(crate) entry_size: u32,
    /// Index of this descriptor in the table.
    pub(crate) index: u32,
    /// Firmware-owned.
    pub(crate) unk_1c: u32,
}

// SAFETY: `CompletionRingDescriptor` consists of integers and arrays of integers only.
unsafe impl Zeroable for CompletionRingDescriptor {}

static_assert!(size_of::<CompletionRingDescriptor>() == 0x20);

/// Completion-ring descriptors in the hardware data; only
/// [`COMPLETION_RING_INDEX`] are used.
const COMPLETION_RING_DESCRIPTORS: usize = 4;
/// Descriptor index of each completion ring.
pub(crate) const COMPLETION_RING_INDEX: [u32; 2] = [0, 2];
/// Entries in a completion ring.
const COMPLETION_RING_ENTRIES: u32 = 0x800;
/// Size of a completion-ring entry.
const COMPLETION_RING_ENTRY_SIZE: u32 = 0x40;
/// Size of a completion ring.
pub(crate) const COMPLETION_RING_SIZE: usize =
    (COMPLETION_RING_ENTRIES * COMPLETION_RING_ENTRY_SIZE) as usize;

/// The two aliases of one completion ring.
#[derive(Copy, Clone, Debug)]
pub(crate) struct CompletionRing {
    /// Context-0 alias.
    pub(crate) low_va: u64,
    /// Firmware alias.
    pub(crate) high_va: u64,
}

/// Inputs of the hardware data.
pub(crate) struct HwDataArgs<'a> {
    /// Chip identifier.
    pub(crate) chip_id: u32,
    /// Mapped register windows.
    pub(crate) windows: &'a [RegisterWindow],
    /// Register-map slots that are reserved but unmapped.
    pub(crate) reserved_slots: &'a [usize],
    /// GPU operating points, in performance-state order.
    pub(crate) opps: &'a [Opp],
    /// Per-SoC parts of the performance tables.
    pub(crate) perf: &'a PerfConfig,
    /// The completion rings, in [`COMPLETION_RING_INDEX`] order.
    pub(crate) completion_rings: [CompletionRing; 2],
    /// The zeroed QoS object ([`QOS_SIZE`] bytes).
    pub(crate) qos_va: u64,
}

/// Size of the QoS object named by the hardware data.
pub(crate) const QOS_SIZE: usize = 0xe40;

/// The hardware data, at the base of the bundle and shared by both
/// instances.
#[repr(C, packed)]
pub(crate) struct HwData {
    /// Required: 0x6f_0000_0000.
    pub(crate) unk_0: u64,
    /// Required: 0xffc0_0000.
    pub(crate) unk_8: u64,
    /// Required: 0x10_0000_0000.
    pub(crate) unk_10: u64,
    /// Required: 0x10_0000_0000.
    pub(crate) unk_18: u64,
    /// Required: 0x2ff_ffff_8000.
    pub(crate) unk_20: u64,
    /// Required: 0xffff_fc21_8140_0000, a GPU address in the firmware MMIO
    /// aperture.
    pub(crate) unk_28: u64,
    pub(crate) unk_30: [u8; 0xb0],
    /// Required: see [`HwData::UNK_E0`].
    pub(crate) unk_e0: [u16; 24],
    pub(crate) unk_110: [u8; 0x1c8],
    /// Required: see [`HwData::UNK_2D8`].
    pub(crate) unk_2d8: [u16; 12],
    pub(crate) unk_2f0: [u8; 0xf0],
    /// Required: see [`HwData::UNK_3E0`].
    pub(crate) unk_3e0: [u16; 24],
    pub(crate) unk_410: [u8; 0x1c8],
    /// Required: see [`HwData::UNK_5D8`].
    pub(crate) unk_5d8: [u16; 12],
    pub(crate) unk_5f0: [u8; 0x50],
    /// Register windows the firmware maps.
    pub(crate) register_map: [RegisterMapEntry; REGISTER_MAP_SLOTS],
    pub(crate) unk_e88: [u8; 8],
    /// Chip identifier.
    pub(crate) chip_id: u32,
    /// Required: 1.
    pub(crate) unk_e94: u32,
    /// Required: 1.
    pub(crate) unk_e98: u32,
    pub(crate) unk_e9c: [u8; 4],
    /// Required: 1.
    pub(crate) unk_ea0: u32,
    /// Required: 1.
    pub(crate) unk_ea4: u32,
    pub(crate) unk_ea8: [u8; 0x10],
    /// Required: 1.
    pub(crate) unk_eb8: u32,
    pub(crate) unk_ebc: [u8; 4],
    /// Required: 1.
    pub(crate) unk_ec0: u32,
    pub(crate) unk_ec4: [u8; 4],
    /// Required: 1.
    pub(crate) unk_ec8: u32,
    pub(crate) unk_ecc: [u8; 4],
    /// Required: 0x5dc0 (24000).
    pub(crate) unk_ed0: u32,
    /// Required: 1.
    pub(crate) unk_ed4: u32,
    /// Required: 0x10 (16).
    pub(crate) unk_ed8: u32,
    pub(crate) unk_edc: [u8; 4],
    /// Required: 1 in every entry.
    pub(crate) unk_ee0: [u32; 3],
    pub(crate) unk_eec: [u8; 0x18],
    /// Required: 0x1f (31).
    pub(crate) unk_f04: u32,
    pub(crate) unk_f08: [u8; 0x1c],
    /// Required: 4.
    pub(crate) unk_f24: u32,
    pub(crate) unk_f28: [u8; 0xc],
    /// Required: 1.
    pub(crate) unk_f34: u32,
    /// Required: 1.
    pub(crate) unk_f38: u32,
    pub(crate) unk_f3c: [u8; 0x10],
    /// Required: 0x31 (49).
    pub(crate) unk_f4c: u32,
    pub(crate) unk_f50: [u8; 0x1c],
    /// Required: 1.
    pub(crate) unk_f6c: u32,
    pub(crate) unk_f70: [u8; 0x18],
    /// Required: 6.
    pub(crate) unk_f88: u32,
    /// Required: 1.
    pub(crate) unk_f8c: u32,
    pub(crate) unk_f90: [u8; 0x1c],
    /// Required: 1.
    pub(crate) unk_fac: u32,
    pub(crate) unk_fb0: [u8; 8],
    /// Required: 0x1e (30).
    pub(crate) unk_fb8: u32,
    /// Required: 4.
    pub(crate) unk_fbc: u32,
    /// Required: 6.
    pub(crate) unk_fc0: u32,
    /// Performance table A.
    pub(crate) perf_a: PerfGroup,
    pub(crate) unk_16c8: [u8; 0x140],
    /// Frequency of each state of table B in MHz.
    pub(crate) perf_b_freq_mhz: [u32; PERF_STATES],
    pub(crate) unk_1834: [u8; 0x14],
    /// Required: 0x3f82_8f5c (1.02 as an IEEE-754 single) in every entry.
    pub(crate) unk_1848: [u32; 11],
    pub(crate) unk_1874: [u8; 0x54],
    /// Relative performance of each state of table A, in percent.
    pub(crate) perf_a_relative: [u32; PERF_STATES],
    pub(crate) unk_18f4: [u8; 0x14],
    /// Relative performance of each state of table B, in percent.
    pub(crate) perf_b_relative: [u32; PERF_STATES],
    pub(crate) unk_1934: [u8; 0x94],
    /// Operating point of each state of table A.
    pub(crate) perf_a_states: [u32; PERF_STATES],
    pub(crate) unk_19f4: [u8; 0x14],
    /// Operating point of each state of table B.
    pub(crate) perf_b_states: [u32; PERF_STATES],
    pub(crate) unk_1a34: [u8; 0x2a4],
    /// Performance table B. Each voltage is stored in lane 0 only.
    pub(crate) perf_b: PerfGroup,
    pub(crate) unk_23dc: [u8; 0x168],
    /// Required: 4.
    pub(crate) unk_2544: u32,
    /// Required: 6.
    pub(crate) unk_2548: u32,
    /// Required: 3.
    pub(crate) unk_254c: u32,
    /// Required: 7.
    pub(crate) unk_2550: u32,
    /// Required: 7.
    pub(crate) unk_2554: u32,
    pub(crate) unk_2558: [u8; 0x10],
    /// Required: 5.
    pub(crate) unk_2568: u32,
    pub(crate) unk_256c: [u8; 4],
    /// Required: 1.
    pub(crate) unk_2570: u32,
    pub(crate) unk_2574: [u8; 0x1a],
    /// Required: [`WORK_CHANNELS`].
    pub(crate) unk_258e: u16,
    /// Required: 0, 1, 1, 1.
    pub(crate) unk_2590: [u32; 4],
    /// Required: 0, 1, 1, 1.
    pub(crate) unk_25a0: [u32; 4],
    pub(crate) unk_25b0: [u8; 4],
    /// Required: 0xffff_ffff in every entry, one per work channel.
    pub(crate) unk_25b4: [u32; WORK_CHANNELS],
    pub(crate) unk_25e4: [u8; 0x10],
    /// Required: 1.
    pub(crate) unk_25f4: u32,
    pub(crate) unk_25f8: [u8; 8],
    /// Required: 1.
    pub(crate) unk_2600: u32,
    pub(crate) unk_2604: [u8; 4],
    /// Required: 1.
    pub(crate) unk_2608: u32,
    pub(crate) unk_260c: [u8; 4],
    /// Required: 0x100.
    pub(crate) unk_2610: u32,
    /// Completion-ring descriptors.
    pub(crate) completion_rings: [CompletionRingDescriptor; COMPLETION_RING_DESCRIPTORS],
    pub(crate) unk_2694: [u8; 0x18],
    /// QoS object ([`QOS_SIZE`] bytes, zeroed).
    pub(crate) qos_va: u64,
    /// Required: 1.
    pub(crate) unk_26b4: u32,
    /// Required: 1.
    pub(crate) unk_26b8: u32,
    pub(crate) unk_26bc: [u8; 0x2c],
    /// Required: 1.
    pub(crate) unk_26e8: u32,
    pub(crate) unk_26ec: [u8; 0x16ac],
    /// Required: 0xffff_ffff.
    pub(crate) unk_3d98: u32,
    pub(crate) unk_3d9c: [u8; 0x14],
    /// Required: 0xffff_ffff.
    pub(crate) unk_3db0: u32,
}

static_assert!(core::mem::offset_of!(HwData, register_map) == 0x640);
static_assert!(core::mem::offset_of!(HwData, chip_id) == 0xe90);
static_assert!(core::mem::offset_of!(HwData, perf_a) == 0xfc4);
static_assert!(core::mem::offset_of!(HwData, perf_b_freq_mhz) == 0x1808);
static_assert!(core::mem::offset_of!(HwData, perf_b) == 0x1cd8);
static_assert!(core::mem::offset_of!(HwData, unk_258e) == 0x258e);
static_assert!(core::mem::offset_of!(HwData, completion_rings) == 0x2614);
static_assert!(core::mem::offset_of!(HwData, qos_va) == 0x26ac);
static_assert!(size_of::<HwData>() == 0x3db4);
static_assert!(bundle::HW_DATA + size_of::<HwData>() <= bundle::VIEWS[2]);

// SAFETY: `HwData` consists of integers and arrays of integers only.
unsafe impl Zeroable for HwData {}

impl HwData {
    const UNK_E0: [u16; 24] = [
        0x2008, 0, 0, 0, 0, 0x2008, 0, 0, 0, 0, 0x2008, 0, 0x24cb, 0, 0x2cfa, 0xc98a, 0x24cb,
        0xf4f6, 0xe917, 0x1877, 0x24cb, 0x38d9, 0, 0xbdab,
    ];
    const UNK_2D8: [u16; 12] = [
        0x2000, 0xfff8, 0x2cdb, 0xd32d, 0x2000, 0xf500, 0xe926, 0x21da, 0x2000, 0x38b6, 8, 0xc742,
    ];
    const UNK_3E0: [u16; 24] = [
        0x7fe0, 0, 0, 0, 0, 0x7fe0, 0, 0, 0, 0, 0x7fe0, 0, 0x8000, 0, 0, 0, 0, 0x8000, 0, 0, 0, 0,
        0x8000, 0,
    ];
    const UNK_5D8: [u16; 12] = [
        0x2645, 0x4b23, 0xe98, 0, 0xea5f, 0xd5a2, 0x3fff, 0x4000, 0x4000, 0xca5e, 0xf5a2, 0x4000,
    ];

    /// Initializes the hardware data.
    pub(crate) fn init(&mut self, args: &HwDataArgs<'_>) -> Result {
        clear(self);
        self.unk_0 = 0x6f_0000_0000;
        self.unk_8 = 0xffc0_0000;
        self.unk_10 = 0x10_0000_0000;
        self.unk_18 = 0x10_0000_0000;
        self.unk_20 = 0x2ff_ffff_8000;
        self.unk_28 = 0xffff_fc21_8140_0000;
        self.unk_e0 = Self::UNK_E0;
        self.unk_2d8 = Self::UNK_2D8;
        self.unk_3e0 = Self::UNK_3E0;
        self.unk_5d8 = Self::UNK_5D8;

        let mut used = [false; REGISTER_MAP_SLOTS];
        let mut claim = |slot: usize| -> Result {
            match used.get_mut(slot) {
                Some(used) if !*used => {
                    *used = true;
                    Ok(())
                }
                _ => Err(EINVAL),
            }
        };
        for window in args.windows {
            claim(window.slot)?;
            self.register_map[window.slot] = RegisterMapEntry::window(window);
        }
        for &slot in args.reserved_slots {
            claim(slot)?;
            self.register_map[slot] = RegisterMapEntry::reserved();
        }

        self.chip_id = args.chip_id;
        self.unk_e94 = 1;
        self.unk_e98 = 1;
        self.unk_ea0 = 1;
        self.unk_ea4 = 1;
        self.unk_eb8 = 1;
        self.unk_ec0 = 1;
        self.unk_ec8 = 1;
        self.unk_ed0 = 0x5dc0;
        self.unk_ed4 = 1;
        self.unk_ed8 = 0x10;
        self.unk_ee0 = [1; 3];
        self.unk_f04 = 0x1f;
        self.unk_f24 = 4;
        self.unk_f34 = 1;
        self.unk_f38 = 1;
        self.unk_f4c = 0x31;
        self.unk_f6c = 1;
        self.unk_f88 = 6;
        self.unk_f8c = 1;
        self.unk_fac = 1;
        self.unk_fb8 = 0x1e;
        self.unk_fbc = 4;
        self.unk_fc0 = 6;

        let perf = args.perf;
        let states_a = perf_states(args.opps, &perf.states_a, perf.sram_min_mv)?;
        let states_b = perf_states(args.opps, &perf.states_b, perf.sram_min_mv)?;
        self.perf_a.fill(&states_a, PERF_LANES);
        self.perf_b.fill(&states_b, 1);
        for (index, state) in states_b.iter().enumerate() {
            self.perf_b_freq_mhz[index] = state.freq_mhz;
            self.perf_a_states[index] = perf.states_a[index].into();
            self.perf_b_states[index] = perf.states_b[index].into();
        }
        self.perf_a_relative = perf.relative_a;
        self.perf_b_relative = perf.relative_b;
        self.unk_1848 = [0x3f82_8f5c; 11];
        self.unk_2544 = 4;
        self.unk_2548 = 6;
        self.unk_254c = 3;
        self.unk_2550 = 7;
        self.unk_2554 = 7;
        self.unk_2568 = 5;
        self.unk_2570 = 1;

        self.unk_258e = WORK_CHANNELS as u16;
        self.unk_2590 = [0, 1, 1, 1];
        self.unk_25a0 = [0, 1, 1, 1];
        self.unk_25b4 = [u32::MAX; WORK_CHANNELS];
        self.unk_25f4 = 1;
        self.unk_2600 = 1;
        self.unk_2608 = 1;
        self.unk_2610 = 0x100;
        for (ring, index) in args.completion_rings.iter().zip(COMPLETION_RING_INDEX) {
            self.completion_rings[index as usize] = CompletionRingDescriptor {
                low_va: ring.low_va,
                high_va: ring.high_va,
                entries: COMPLETION_RING_ENTRIES,
                entry_size: COMPLETION_RING_ENTRY_SIZE,
                index,
                unk_1c: 0,
            };
        }
        self.qos_va = args.qos_va;
        self.unk_26b4 = 1;
        self.unk_26b8 = 1;
        self.unk_26e8 = 1;
        self.unk_3d98 = 0xffff_ffff;
        self.unk_3db0 = 0xffff_ffff;
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Regions A and C
// ---------------------------------------------------------------------------

/// Region A: a zeroed page both instances read.
#[repr(C)]
pub(crate) struct RegionA {
    pub(crate) unk_0: [u8; 0x4000],
}

// SAFETY: `RegionA` is an array of integers.
unsafe impl Zeroable for RegionA {}

/// Engines whose work the firmware's context-switch timer may interrupt
/// (bit 0 TA, bit 1 3D, bit 2 compute). No engine is periodically resumed.
pub(crate) const CONTEXT_SWITCH_TIMER_MASK: u32 = 0;
/// Period of the context-switch timer, in units of the firmware's 1 ms
/// scheduling pass.
pub(crate) const CONTEXT_SWITCH_TIMER_MULTIPLIER: u8 = 1;

/// The values of region C that differ between SoCs and firmware builds.
pub(crate) struct RegionCConfig {
    /// A18 Pro: 0xffff.
    pub(crate) unk_54: u16,
    /// A18 Pro: 1 in every entry.
    pub(crate) unk_78: [u32; 3],
    /// Highest GPU power, in mW (A18 Pro: 9550).
    pub(crate) max_power_mw: u32,
    /// A18 Pro: 0x3000_001d.
    pub(crate) unk_9c: u32,
    /// A18 Pro: 0x3100_0002.
    pub(crate) unk_a0: u32,
    /// A18 Pro: 0x7d0 (2000).
    pub(crate) unk_7c4: u32,
    /// A18 Pro: zero.
    pub(crate) unk_9e0: [u32; 8],
}

/// Region C of the A18 Pro firmware (26A428).
pub(crate) const REGION_C_26A428: RegionCConfig = RegionCConfig {
    unk_54: 0xffff,
    unk_78: [1; 3],
    max_power_mw: 9550,
    unk_9c: 0x3000_001d,
    unk_a0: 0x3100_0002,
    unk_7c4: 0x7d0,
    unk_9e0: [0; 8],
};

/// Region C: firmware policy settings shared by both instances.
///
/// Three fields are live. The host updates [`Self::effort`] while it submits
/// work: it stores zero to [`Self::unk_e8`], then the effort (0x10000, 1.0
/// in 16.16 fixed point, while work is streaming; zero once the GPU has been
/// idle for 16 ms), each with one aligned 32-bit store, and then cleans the
/// cache line to the point of coherency. [`Self::power_assert_tally`] is
/// unaligned: for every device-control record whose opcode asserts GPU
/// power, the host adds one to it after writing the record and before
/// advancing the ring producer (with a barrier before the producer store).
/// The firmware powers the GPU cores up while the tally differs from its
/// own count of drained records.
#[repr(C, packed)]
pub(crate) struct RegionC {
    /// Engines the context-switch timer applies to ([`CONTEXT_SWITCH_TIMER_MASK`]).
    pub(crate) context_switch_timer_mask: u32,
    pub(crate) unk_4: [u8; 0x20],
    /// Required: 0xbb8 (3000).
    pub(crate) unk_24: u32,
    pub(crate) unk_28: [u8; 8],
    /// Non-zero lets the firmware power the GPU cores off when idle.
    pub(crate) idle_power_off: u32,
    /// Required: 1.
    pub(crate) unk_34: u32,
    pub(crate) unk_38: [u8; 4],
    /// Required: 0x78 (120).
    pub(crate) unk_3c: u32,
    pub(crate) unk_40: [u8; 0x14],
    /// See [`RegionCConfig::unk_54`].
    pub(crate) unk_54: u16,
    /// Required: 0x28 (40).
    pub(crate) unk_56: u16,
    /// Required: 0xffff.
    pub(crate) unk_58: u16,
    pub(crate) unk_5a: [u8; 4],
    /// Required: 1.
    pub(crate) unk_5e: u32,
    /// Required: 1.
    pub(crate) unk_62: u32,
    /// Required: 3.
    pub(crate) unk_66: u32,
    /// Period of the context-switch timer ([`CONTEXT_SWITCH_TIMER_MULTIPLIER`]).
    pub(crate) context_switch_timer_multiplier: u8,
    pub(crate) unk_6b: [u8; 0xd],
    /// See [`RegionCConfig::unk_78`].
    pub(crate) unk_78: [u32; 3],
    /// Highest GPU power, in mW.
    pub(crate) max_power_mw: u32,
    /// Required: 0x3e8 (1000).
    pub(crate) unk_88: u32,
    /// Required: 0x3e8 (1000).
    pub(crate) unk_8c: u32,
    pub(crate) unk_90: [u8; 8],
    /// Required: 0x3e8 (1000).
    pub(crate) unk_98: u32,
    /// See [`RegionCConfig::unk_9c`].
    pub(crate) unk_9c: u32,
    /// See [`RegionCConfig::unk_a0`].
    pub(crate) unk_a0: u32,
    pub(crate) unk_a4: [u8; 0x40],
    /// Performance-floor request (live, see the type documentation).
    pub(crate) effort: u32,
    /// Live; the host stores zero here whenever it stores [`Self::effort`].
    pub(crate) unk_e8: u32,
    pub(crate) unk_ec: [u8; 0x6d8],
    /// See [`RegionCConfig::unk_7c4`].
    pub(crate) unk_7c4: u32,
    /// Required: 0x3f80_0000 (1.0 as an IEEE-754 single).
    pub(crate) unk_7c8: u32,
    /// Required: 0x3f4c_cccd (0.8 as an IEEE-754 single).
    pub(crate) unk_7cc: u32,
    /// Required: 0x3e4c_cccd (0.2 as an IEEE-754 single).
    pub(crate) unk_7d0: u32,
    /// Required: 0x3f66_6666 (0.9 as an IEEE-754 single).
    pub(crate) unk_7d4: u32,
    /// Required: 0x3dcc_cccd (0.1 as an IEEE-754 single).
    pub(crate) unk_7d8: u32,
    /// Required: 0x3e80_0000 (0.25 as an IEEE-754 single).
    pub(crate) unk_7dc: u32,
    /// Required: 0x3f19_999a (0.6 as an IEEE-754 single).
    pub(crate) unk_7e0: u32,
    /// Required: 0x3f66_6666 (0.9 as an IEEE-754 single).
    pub(crate) unk_7e4: u32,
    /// Required: 6.
    pub(crate) unk_7e8: u32,
    /// Required: 1.
    pub(crate) unk_7ec: u32,
    /// Required: 1.
    pub(crate) unk_7f0: u32,
    /// Required: 0x64 (100).
    pub(crate) unk_7f4: u32,
    /// Required: 0x64 (100).
    pub(crate) unk_7f8: u32,
    pub(crate) unk_7fc: [u8; 0x19c],
    /// Required: 0x28 (40).
    pub(crate) unk_998: u32,
    /// Required: 0xa (10).
    pub(crate) unk_99c: u32,
    /// Required: 0xfa (250).
    pub(crate) unk_9a0: u32,
    /// Required: 1.
    pub(crate) unk_9a4: u32,
    pub(crate) unk_9a8: [u8; 0x10],
    /// Required: 2.
    pub(crate) unk_9b8: u32,
    /// Idle time before the firmware powers the GPU cores off.
    pub(crate) idle_power_off_delay: u32,
    /// Required: 5.
    pub(crate) unk_9c0: u32,
    pub(crate) unk_9c4: [u8; 4],
    /// Required: 0x28 (40).
    pub(crate) unk_9c8: u32,
    /// Compute progress watchdog threshold (zero disables the watchdog).
    pub(crate) compute_watchdog: u32,
    /// Required: 1.
    pub(crate) unk_9d0: u32,
    pub(crate) unk_9d4: [u8; 0xc],
    /// See [`RegionCConfig::unk_9e0`].
    pub(crate) unk_9e0: [u32; 8],
    pub(crate) unk_a00: [u8; 0x41d],
    /// Required: 1.
    pub(crate) unk_e1d: u32,
    pub(crate) unk_e21: [u8; 0x18],
    /// Required: 3.
    pub(crate) unk_e39: u32,
    /// Required: 1.
    pub(crate) unk_e3d: u32,
    pub(crate) unk_e41: [u8; 0x10],
    /// Host power-assert tally (live, unaligned; see the type documentation).
    pub(crate) power_assert_tally: u32,
    pub(crate) unk_e55: [u8; 0x1ab],
}

static_assert!(core::mem::offset_of!(RegionC, idle_power_off) == 0x30);
static_assert!(core::mem::offset_of!(RegionC, context_switch_timer_multiplier) == 0x6a);
static_assert!(core::mem::offset_of!(RegionC, effort) == 0xe4);
static_assert!(core::mem::offset_of!(RegionC, idle_power_off_delay) == 0x9bc);
static_assert!(core::mem::offset_of!(RegionC, compute_watchdog) == 0x9cc);
static_assert!(core::mem::offset_of!(RegionC, power_assert_tally) == 0xe51);
static_assert!(size_of::<RegionC>() == 0x1000);

// SAFETY: `RegionC` consists of integers and arrays of integers only.
unsafe impl Zeroable for RegionC {}

impl RegionC {
    /// Offset of [`Self::effort`].
    pub(crate) const EFFORT: usize = core::mem::offset_of!(Self, effort);
    /// Offset of [`Self::unk_e8`].
    pub(crate) const UNK_E8: usize = core::mem::offset_of!(Self, unk_e8);
    /// Offset of [`Self::power_assert_tally`].
    pub(crate) const POWER_ASSERT_TALLY: usize = core::mem::offset_of!(Self, power_assert_tally);

    /// Initializes region C.
    pub(crate) fn init(&mut self, cfg: &RegionCConfig) {
        clear(self);
        self.context_switch_timer_mask = CONTEXT_SWITCH_TIMER_MASK;
        self.unk_24 = 0xbb8;
        self.idle_power_off = 1;
        self.unk_34 = 1;
        self.unk_3c = 0x78;
        self.unk_54 = cfg.unk_54;
        self.unk_56 = 0x28;
        self.unk_58 = 0xffff;
        self.unk_5e = 1;
        self.unk_62 = 1;
        self.unk_66 = 3;
        self.context_switch_timer_multiplier = CONTEXT_SWITCH_TIMER_MULTIPLIER;
        self.unk_78 = cfg.unk_78;
        self.max_power_mw = cfg.max_power_mw;
        self.unk_88 = 0x3e8;
        self.unk_8c = 0x3e8;
        self.unk_98 = 0x3e8;
        self.unk_9c = cfg.unk_9c;
        self.unk_a0 = cfg.unk_a0;
        self.unk_7c4 = cfg.unk_7c4;
        self.unk_7c8 = 0x3f80_0000;
        self.unk_7cc = 0x3f4c_cccd;
        self.unk_7d0 = 0x3e4c_cccd;
        self.unk_7d4 = 0x3f66_6666;
        self.unk_7d8 = 0x3dcc_cccd;
        self.unk_7dc = 0x3e80_0000;
        self.unk_7e0 = 0x3f19_999a;
        self.unk_7e4 = 0x3f66_6666;
        self.unk_7e8 = 6;
        self.unk_7ec = 1;
        self.unk_7f0 = 1;
        self.unk_7f4 = 0x64;
        self.unk_7f8 = 0x64;
        self.unk_998 = 0x28;
        self.unk_99c = 0xa;
        self.unk_9a0 = 0xfa;
        self.unk_9a4 = 1;
        self.unk_9b8 = 2;
        self.idle_power_off_delay = 40;
        self.unk_9c0 = 5;
        self.unk_9c8 = 0x28;
        self.compute_watchdog = 50;
        self.unk_9d0 = 1;
        self.unk_9e0 = cfg.unk_9e0;
        self.unk_e1d = 1;
        self.unk_e39 = 3;
        self.unk_e3d = 1;
    }
}

// ---------------------------------------------------------------------------
// Status objects
// ---------------------------------------------------------------------------

/// Status block of an instance (its status A, and the head of the primary
/// status B).
///
/// At creation only `+0x04` is one. Firmware later owns the status words at
/// `+0x10` and `+0x14`; the host observes each with an aligned 32-bit read.
/// The event-ring state begins at status A `+0x40`, inside this boot view:
/// clearing or copying the whole status block after publication is forbidden.
#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub(crate) struct StatusBlock {
    pub(crate) unk_0: u32,
    /// Required: 1.
    pub(crate) unk_4: u32,
    pub(crate) unk_8: [u8; 0x78],
}

// SAFETY: `StatusBlock` consists of integers and arrays of integers only.
unsafe impl Zeroable for StatusBlock {}

static_assert!(size_of::<StatusBlock>() == 0x80);

impl StatusBlock {
    /// Returns a status block in its boot state.
    pub(crate) const fn new() -> Self {
        Self {
            unk_0: 0,
            unk_4: 1,
            unk_8: [0; 0x78],
        }
    }
}



/// Size of the firmware-control page named by the primary status B.
pub(crate) const FWCTL_SIZE: usize = 0x4000;
/// Offset of the firmware-control ring in its page.
pub(crate) const FWCTL_RING: u64 = 0x40;

/// One firmware-written recovery-info entry. The host reads each word separately after the
/// recovery handshake reaches state 3; it never copies the live table as a structure.
#[repr(C)]
pub(crate) struct RecoveryInfoEntry {
    pub(crate) flags: u32,
    pub(crate) stamp: u32,
}
// SAFETY: Both fields are integers.
unsafe impl Zeroable for RecoveryInfoEntry {}
static_assert!(size_of::<RecoveryInfoEntry>() == 8);

/// Status B of the primary instance, directly after its state grid.
#[repr(C, packed)]
pub(crate) struct PrimaryStatusB {
    /// Status block.
    pub(crate) status: StatusBlock,
    pub(crate) unk_80: [u8; 0x4038],
    pub(crate) recovery_info: [RecoveryInfoEntry; 256],
    pub(crate) unk_48b8: [u8; 0x20],
    pub(crate) recovery_slot_mask: u64,
    /// Firmware-control state, at the start of the firmware-control page.
    pub(crate) fwctl_state_va: u64,
    /// Firmware-control ring, in the firmware-control page.
    pub(crate) fwctl_ring_va: u64,
    /// Firmware-written epoch, read using an aligned 64-bit load.
    pub(crate) recovery_epoch: u64,
    pub(crate) unk_48f8: [u8; 8],
    /// Firmware writes 1 and 3; host writes 2 and 0 with full barriers around each 32-bit store.
    pub(crate) recovery_state: u32,
    pub(crate) unk_4904: [u8; 0xc],
    /// Must be zero for an event-backed restart; read using an aligned 32-bit load.
    pub(crate) host_recovery: u32,
    pub(crate) unk_4914: [u8; 0x9b20],
    /// Required: 1.
    pub(crate) unk_e434: u32,
    pub(crate) unk_e438: [u8; 8],
    /// Power state, see [`PowerState`].
    pub(crate) power: PowerState,
}

static_assert!(core::mem::offset_of!(PrimaryStatusB, recovery_info) == 0x40b8);
static_assert!(core::mem::offset_of!(PrimaryStatusB, recovery_slot_mask) == 0x48d8);
static_assert!(core::mem::offset_of!(PrimaryStatusB, recovery_epoch) == 0x48f0);
static_assert!(core::mem::offset_of!(PrimaryStatusB, recovery_state) == 0x4900);
static_assert!(core::mem::offset_of!(PrimaryStatusB, host_recovery) == 0x4910);
static_assert!(core::mem::offset_of!(PrimaryStatusB, fwctl_state_va) == 0x48e0);
static_assert!(core::mem::offset_of!(PrimaryStatusB, unk_e434) == 0xe434);
static_assert!(core::mem::offset_of!(PrimaryStatusB, power) == PrimaryStatusB::POWER_STATE);
static_assert!(size_of::<PrimaryStatusB>() == 0xeb00);

// SAFETY: `PrimaryStatusB` consists of integers and arrays of integers only.
unsafe impl Zeroable for PrimaryStatusB {}

impl PrimaryStatusB {
    pub(crate) const RECOVERY_SLOT_MASK: usize = core::mem::offset_of!(Self, recovery_slot_mask);
    pub(crate) const RECOVERY_INFO: usize = core::mem::offset_of!(Self, recovery_info);
    pub(crate) const RECOVERY_STATE: usize = core::mem::offset_of!(Self, recovery_state);
    pub(crate) const HOST_RECOVERY: usize = core::mem::offset_of!(Self, host_recovery);

    /// Offset of the [`PowerState`].
    pub(crate) const POWER_STATE: usize = 0xe440;

    /// Initializes the primary status B; `fwctl_va` is the zeroed
    /// firmware-control page ([`FWCTL_SIZE`] bytes).
    pub(crate) fn init(&mut self, fwctl_va: u64) {
        clear(self);
        self.status = StatusBlock::new();
        self.fwctl_state_va = fwctl_va;
        self.fwctl_ring_va = fwctl_va + FWCTL_RING;
        self.unk_e434 = 1;
        self.power.init();
    }
}



/// Offsets in the private cluster.
pub(crate) mod private {
    /// Size of the private cluster.
    pub(crate) const SIZE: usize = 0x178000;
    /// The primary state grid.
    pub(crate) const PRIMARY_STATE: usize = 0;
    /// The primary status A block.
    pub(crate) const PRIMARY_STATUS_A: usize = 0xee40;
    /// The secondary status A block.
    pub(crate) const SECONDARY_STATUS_A: usize = 0xc3100;
    /// The secondary state grid.
    pub(crate) const SECONDARY_STATE: usize = 0x1770c0;
    /// [`super::PrimaryStatusB`] or [`super::SecondaryConfig`], from an
    /// instance's state grid.
    pub(crate) const STATUS_B: usize = 0x340;
    /// Secondary state named by [`super::HwDataAux`], from the secondary
    /// state grid.
    pub(crate) const SECONDARY_STATUS: usize = 0x3c0;
}

static_assert!(private::STATUS_B == CONTROL_STATE_OFFSET + size_of::<ChannelState>());
static_assert!(
    private::PRIMARY_STATE + private::STATUS_B + size_of::<PrimaryStatusB>()
        == private::PRIMARY_STATUS_A
);
static_assert!(private::STATUS_B + size_of::<SecondaryConfig>() == private::SECONDARY_STATUS);

// ---------------------------------------------------------------------------
// Power configuration and auxiliary hardware data (bundle)
// ---------------------------------------------------------------------------

/// Die temperature sensors read by the die temperature limiter.
const TEMP_SENSOR_MASK: u64 = 0x82a;
/// Die temperature the limiter holds the GPU below, in 0.01 °C.
const TEMP_TARGET: u32 = 11_000;
/// Die temperature below which the limiter releases, in 0.01 °C.
const TEMP_RELEASE: u32 = 8_000;
/// Proportional gain of the limiter: 20.34 as an IEEE-754 single.
const TEMP_KP: u32 = 0x41a2_b852;
/// Integral gain of the limiter, per control period: 9.376 as an IEEE-754
/// single.
const TEMP_KI: u32 = 0x4116_0419;

/// Nonzero 32-bit words of a firmware object the host builds from constants: (byte offset, value, repeat).
/// Every other word is zero. The table is the sole definition of the object's constant contents.
type Words = &'static [(usize, u32, usize)];

fn fill_words(bytes: &mut [u8], words: Words) {
    bytes.fill(0);
    for &(offset, value, repeat) in words {
        for index in 0..repeat {
            let at = offset + 4 * index;
            bytes[at..at + 4].copy_from_slice(&value.to_le_bytes());
        }
    }
}

const POWER_CONFIG_HEAD_WORDS: Words = &[
    (0x4, 0x5dc00, 2),
    (0x10, 4, 1),
    (0x14, 0x3f800000, 1),
    (0x2c, 1, 2),
    (0x40, 0x64, 1),
    (0x44, 1, 1),
    (0x48, 0x3e8, 1),
    (0x50, 0x64, 1),
    (0x60, 4, 1),
    (0x64, 0x3f800000, 1),
    (0x6c, 1, 2),
    (0x80, 0x64, 1),
    (0x84, 1, 1),
    (0x88, 0x3e8, 1),
    (0x90, 0x64, 1),
    (0xc0, 0x3f828f5c, 11),
    (0x9b8, 0x271, 1),
    (0x9c4, 0x3f7f2e9f, 1),
    (0x9cc, 0x3b516154, 1),
    (0x9d4, 0x3ca59586, 1),
    (0x9e0, 0x46153800, 1),
    (0x9e4, 0x40a90fdb, 1),
    (0x9ec, 0xbdd67344, 1),
    (0x9f0, 0x28, 1),
    (0x9f4, 0x3e8, 2),
    (0xa04, 0x254e, 1),
    (0xa38, 0x3e8, 1),
    (0xa70, 4, 1),
    (0xa84, 0x3f800000, 1),
    (0xa8c, 0x40cccccd, 1),
    (0xa98, 0x47800000, 1),
    (0xa9c, 0x3f000000, 1),
    (0xaa8, 0x28, 1),
    (0xaac, 0x3e8, 1),
    (0xabc, 0x254e, 1),
    (0xac8, 0x10, 1),
    (0xad0, 0x5dc00, 1),
    (0xb28, 0x5c, 1),
    (0xb30, 0x64, 1),
    (0xb34, 0x22, 1),
    (0xb38, 6, 1),
    (0xb40, 6, 1),
    (0xb44, 1, 1),
    (0xb4c, 0x3f4ccccd, 1),
    (0xb50, 0x3f7df3b6, 1),
    (0xb54, 0x3e4ccccd, 1),
    (0xb58, 0x3c03126f, 1),
    (0xb5c, 0x3f69d4d8, 2),
    (0xb68, 0x42be0000, 1),
    (0xb6c, 0x4064f5c3, 2),
    (0xb74, 0x41179436, 1),
    (0xb78, 0x64, 1),
    (0xb7c, 0x3e8, 1),
    (0xb80, 0x64, 1),
    (0xb8c, 0x5c, 1),
    (0xbc0, 0x64, 1),
    (0xbe0, 0x401, 1),
    (0xbe4, 0x6400, 1),
    (0xbec, 0x1f4000, 1),
    (0xbf0, 0xc800, 1),
    (0xbf4, 0xfa000, 1),
    (0xbf8, 0xc800, 1),
    (0xbfc, 0x7d000, 1),
    (0xc00, 0xc800, 1),
    (0xc04, 0x3e800, 1),
    (0xc08, 0xc800, 1),
    (0xc0c, 0x100, 1),
    (0xc64, 1, 1),
    (0xc90, 1, 1),
    (0x1c94, 0xff000000, 1),
    (0x1c98, 0xff7fff7f, 74),
];

const POWER_CONFIG_TAIL_WORDS: Words = &[
    (0x0, 0xff7fff7f, 181),
    (0x2d4, 0x7fff7f, 1),
    (0x4d8, 0x47800000, 1),
    (0x4dc, 0x42200000, 1),
    (0x4e0, 0x447a0000, 1),
    (0x4e4, 0x5be, 1),
    (0x4f8, 0x28, 1),
    (0x500, 0x42c80000, 1),
    (0x504, 0x3e8, 1),
    (0x50c, 0x3f4ccccd, 1),
    (0x510, 0x3e4ccccd, 1),
    (0x550, TEMP_SENSOR_MASK as u32, 1),
    (0x558, TEMP_RELEASE, 1),
    (0x560, 4, 1),
    (0x574, 0x3f800000, 1),
    (0x57c, TEMP_KI, 1),
    (0x588, 0x47800000, 1),
    (0x58c, TEMP_KP, 1),
    (0x598, 0x28, 1),
    (0x59c, 0x3e8, 2),
    (0x5ac, TEMP_TARGET, 1),
    (0x5e0, 0x3e8, 1),
    (0x7fc, 0x42000006, 1),
    (0xa10, 0xb8520000, 1),
    (0xa14, 0x3f7e, 1),
    (0xa18, 0xd70a0000, 1),
    (0xa1c, 0x3ba3, 1),
    (0xa24, 0x43c8, 1),
    (0xa30, 0x4780, 1),
    (0xa34, 0x42c8, 1),
    (0xa3c, 0x384bac8, 1),
    (0xa40, 0x3e80000, 2),
    (0xa4c, 0x80000000, 1),
    (0xa50, 0x26ac453b, 1),
    (0xa84, 0x3e80000, 1),
    (0xaa0, 0xc80000, 1),
    (0xb58, 0x3e8, 1),
    (0xb6c, 1, 1),
    (0xba0, 0x101, 1),
    (0xba4, 0x400, 1),
    (0xbbc, 0x23f00, 1),
    (0xbc0, 0x100, 2),
    (0xbdc, 0x24000, 1),
    (0xbe0, 0x100, 2),
    (0xbfc, 0x24100, 1),
    (0xc48, 1, 1),
    (0xc50, 0x254e, 3),
    (0xc68, 0x3f800000, 1),
    (0xc6c, 4, 1),
    (0xc70, 0x10, 1),
    (0xc74, 0x5dc00, 1),
    (0xd44, 0x3c, 1),
    (0xd50, 0x3f6eeeef, 1),
    (0xd58, 0x3d888889, 1),
    (0xd60, 0x3f000000, 1),
    (0xd6c, 0x47800000, 1),
    (0xd70, 0x40880000, 1),
    (0xd7c, 0x28, 1),
    (0xd80, 0x3e8, 2),
    (0xd8c, 0x46153800, 1),
    (0xd90, 0x254e, 1),
    (0xd9c, 0xf0, 1),
    (0xda4, 0x57e400, 1),
    (0xdc4, 0x3e8, 1),
    (0xe04, 1, 1),
    (0xed4, 0x32, 1),
    (0xed8, 1, 1),
    (0xee0, 0x3f638e39, 1),
    (0xee4, 0x3f2aaaab, 1),
    (0xee8, 0x3de38e39, 1),
    (0xeec, 0x3eaaaaab, 1),
    (0xef0, 0xbf4ccccd, 2),
    (0xefc, 0x47800000, 1),
    (0xf00, 0xc0a00000, 2),
    (0xf0c, 0x64, 1),
    (0xf10, 0x3e8, 1),
    (0xf14, 0x64, 1),
    (0xf1c, 0x467a0000, 1),
    (0xf20, 0x578, 1),
    (0xf2c, 0x90, 1),
    (0xf30, 0x30, 1),
    (0xf34, 0x34bc00, 1),
    (0xf3c, 0x119400, 1),
    (0xf48, 0x47800000, 1),
    (0xf70, 0x3e80, 1),
    (0xf74, 2, 1),
    (0xf78, 0x9c4, 1),
    (0xf7c, 0x20d, 1),
    (0xf80, 2, 1),
    (0xf84, 4, 1),
    (0xf88, 0x32, 1),
    (0xfb8, 0x47800000, 1),
    (0xfc8, 0x28, 1),
    (0xfcc, 0x3e8, 1),
];

const POWER_STATE_WORDS: Words = &[
    (0x10, 0xf, 1),
    (0x14, 0x3f000000, 1),
    (0x18, 0x40880000, 1),
    (0x1c, 0x28, 1),
    (0x20, 1, 1),
    (0xc8, TEMP_TARGET, 1),
    (0xcc, TEMP_RELEASE, 1),
    (0xd8, TEMP_KP, 1),
    (0xdc, TEMP_KI, 1),
    (0xf0, 0x26ac, 1),
    (0xf4, 0xc8, 1),
    (0x108, 0x42c80000, 1),
    (0x10c, 0x43c80000, 1),
    (0x110, 0xc8, 1),
    (0x128, 1, 1),
    (0x12c, 0x2616, 1),
    (0x130, 0x3f000000, 1),
    (0x134, 0x40cccccd, 1),
    (0x14c, 1, 1),
    (0x1c0, 1, 2),
    (0x1c8, 4, 1),
    (0x1cc, 1, 4),
    (0x4e4, 1, 1),
    (0x4e8, 0x1f4, 1),
    (0x4ec, 6, 1),
    (0x4f0, 0x30d4, 6),
    (0x51c, 0x30d4, 5),
    (0x530, 1, 1),
    (0x58c, 6, 1),
    (0x590, 0xdac, 1),
    (0x594, 0x3e8, 1),
    (0x598, 0xbb8, 1),
    (0x59c, 0x64, 1),
    (0x5cc, 1, 1),
    (0x5d0, 4, 1),
    (0x5dc, 0x1f40, 1),
    (0x5e0, 0xc8, 1),
    (0x5e4, 0xfa0, 1),
    (0x5e8, 0xc8, 1),
    (0x5ec, 0x7d0, 1),
    (0x5f0, 0xc8, 1),
    (0x5f4, 0x3e8, 1),
    (0x5f8, 0xc8, 1),
    (0x604, 0x41700000, 1),
    (0x608, 0x40a00000, 1),
    (0x60c, 0x20, 1),
    (0x610, 0x3e8, 1),
    (0x620, 0x384, 1),
];

const HW_DATA_AUX_WORDS: Words = &[
    (0xac, 6, 1),
    (0xb8, 1, 1),
    (0x174, 0xac000000, 1),
    (0x178, 0xe800000d, 1),
    (0x17c, 0xb8000003, 1),
    (0x180, 0xe800000b, 1),
    (0x184, 0xe8000003, 1),
    (0x188, 0x64000003, 1),
    (0x18c, 0x20000000, 1),
    (0x1b4, 4, 1),
    (0x1c0, 0x3f780000, 1),
    (0x1c8, 0x3d000000, 1),
    (0x1d0, 0x40a00000, 1),
    (0x1dc, 0x47800000, 1),
    (0x1e0, 0x41700000, 1),
    (0x1ec, 0x384, 1),
    (0x1f0, 0x3e8, 2),
    (0x200, 0x3e8, 1),
    (0x234, 0x3e8, 1),
    (0x274, 0x64, 1),
    (0x290, 0x82a, 1),
    (0x298, 0x7d, 1),
    (0x29c, 1, 1),
    (0xee0, 0x18ae, 1),
    (0xee8, 1, 1),
    (0x2eec, 0x41c8cccd, 2),
    (0x35e4, 1, 1),
    (0x35e8, 0x1f4, 1),
    (0x35ec, 6, 1),
    (0x35f0, 0x30d4, 6),
    (0x3608, 0xfa0, 1),
    (0x361c, 0x30d4, 5),
    (0x3630, 1, 1),
    (0x37a8, 1, 1),
    (0x37b4, 0x424ccccd, 1),
    (0x37f4, 0x44068000, 1),
    (0x3834, 0x41c9999a, 1),
    (0x38e0, 0x186a, 1),
];

const SECONDARY_CONFIG_WORDS: Words = &[
    (0x14, 1, 1),
    (0x2c, 1, 2),
    (0x40, 1, 1),
    (0x4c, 1, 1),
    (0x50, 0x186a, 1),
];

/// Power state the primary status B carries for the secondary instance,
/// named by [`SecondaryRoot::power_state_va`].
#[repr(C)]
pub(crate) struct PowerState(pub(crate) [u8; 0x6c0]);

// SAFETY: `PowerState` is a byte array.
unsafe impl Zeroable for PowerState {}

impl PowerState {
    fn init(&mut self) {
        fill_words(&mut self.0, POWER_STATE_WORDS);
    }
}

/// Configuration of the secondary instance, named by
/// [`SecondaryRoot::config_va`].
#[repr(C)]
pub(crate) struct SecondaryConfig(pub(crate) [u8; 0x80]);

// SAFETY: `SecondaryConfig` is a byte array.
unsafe impl Zeroable for SecondaryConfig {}

impl SecondaryConfig {
    pub(crate) fn init(&mut self) {
        fill_words(&mut self.0, SECONDARY_CONFIG_WORDS);
    }
}

/// Leading part of the [`PowerConfig`]. The host writes it twice: in the
/// power configuration and at [`bundle::POWER_CONFIG_COPY`], where the copy
/// ends with the views it lies in.
#[repr(C)]
pub(crate) struct PowerConfigHead(pub(crate) [u8; 0x1dc0]);

// SAFETY: `PowerConfigHead` is a byte array.
unsafe impl Zeroable for PowerConfigHead {}

impl PowerConfigHead {
    pub(crate) fn init(&mut self) {
        fill_words(&mut self.0, POWER_CONFIG_HEAD_WORDS);
    }
}

/// Trailing part of the [`PowerConfig`]. Field names give the offset in the
/// whole power configuration. It carries the die temperature limiter: target
/// and release temperatures in 0.01 °C, proportional and integral gains as
/// IEEE-754 singles.
#[repr(C)]
pub(crate) struct PowerConfigTail(pub(crate) [u8; 0x1000]);

// SAFETY: `PowerConfigTail` is a byte array.
unsafe impl Zeroable for PowerConfigTail {}

impl PowerConfigTail {
    fn init(&mut self) {
        fill_words(&mut self.0, POWER_CONFIG_TAIL_WORDS);
    }
}

/// Auxiliary hardware data the firmware reads at [`bundle::AUX`], up to the
/// start of view 3. No pointer names it.
#[repr(C)]
pub(crate) struct HwDataAux(pub(crate) [u8; 0x3c80]);

// SAFETY: `HwDataAux` is a byte array.
unsafe impl Zeroable for HwDataAux {}

impl HwDataAux {
    pub(crate) fn init(&mut self, secondary_status_va: u64) {
        fill_words(&mut self.0, HW_DATA_AUX_WORDS);
        self.0[0xed8..0xee0].copy_from_slice(&secondary_status_va.to_le_bytes());
    }
}





/// Power configuration, in view 2 of the bundle. The secondary instance
/// finds it through [`MainConfig::power_config_va`].
#[repr(C, packed)]
pub(crate) struct PowerConfig {
    /// Leading part.
    pub(crate) head: PowerConfigHead,
    /// Trailing part.
    pub(crate) tail: PowerConfigTail,
}

static_assert!(bundle::POWER_CONFIG + size_of::<PowerConfig>() == bundle::AUX);
static_assert!(bundle::VIEWS[2] + 0xe40 == bundle::POWER_CONFIG);

// SAFETY: `PowerConfig` consists of integers and arrays of integers only.
unsafe impl Zeroable for PowerConfig {}

impl PowerConfig {
    /// Initializes the power configuration.
    pub(crate) fn init(&mut self) {
        self.head.init();
        self.tail.init();
    }
}


// ---------------------------------------------------------------------------
// Free lists
// ---------------------------------------------------------------------------

/// Size of a free-list page.
const FREE_LIST_PAGE_SIZE: u64 = 0x1000;
/// Pages in one free-list block.
pub(crate) const FREE_LIST_BLOCK_PAGES: u32 = 0x100;
/// Size of one free-list block.
pub(crate) const FREE_LIST_BLOCK_SIZE: usize =
    FREE_LIST_BLOCK_PAGES as usize * FREE_LIST_PAGE_SIZE as usize;
/// Run-list qwords reserved for each block.
const RUN_SLOT_QWORDS: u32 = 8;
/// Bit position of the page count in a run descriptor.
const RUN_PAGES_SHIFT: u32 = 52;
/// Address bits of a run descriptor.
const RUN_ADDRESS_MASK: u64 = 0x0000_ffff_ffff_f000;

/// Class of a free list, selecting [`FreeListControl::unk_10`] and
/// [`FreeListControl::unk_60`].
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u32)]
pub(crate) enum FreeListClass {
    /// `unk_10` 1, `unk_60` 2: the boot render and compute free lists.
    One = 1,
    /// `unk_10` 2, `unk_60` 3: the auxiliary boot and runtime render pools.
    Two = 2,
}

impl FreeListClass {
    const fn unk_60(self) -> u32 {
        match self {
            Self::One => 2,
            Self::Two => 3,
        }
    }
}

/// Description of a free list whose blocks are all populated.
pub(crate) struct FreeListArgs {
    /// Buffer slot of the free list.
    pub(crate) buffer_slot: u32,
    /// Class of the free list.
    pub(crate) class: FreeListClass,
    /// Page list: one 64-bit address per page of the blocks.
    pub(crate) page_list_va: u64,
    /// Run list: one [`FreeListRunSlot`] per block.
    pub(crate) run_list_va: u64,
    /// Blocks in the free list.
    pub(crate) blocks: u32,
    /// [`FreeListState`].
    pub(crate) state_va: u64,
}

/// Control object of a USC private-memory free list, at the start of its
/// page.
///
/// This constructor describes the boot objects. Once a pool is published, its
/// page counters (`+0x24`, `+0x2c`) and run cursor (`+0x48`) are shared state.
/// The firmware-completed operation counter occupies the eight bytes at
/// `+0x54`; it is unaligned and the host reads its bytes individually.
/// A released render pool can be repopulated only after its release is
/// consumed: write page-list/run-list words first, then the complete control
/// page in ascending aligned 64-bit words, then a full barrier. The completed
/// count in that image must equal the retained submitted count, not zero.
/// Publication and reuse belong to the host free-list implementation.
#[repr(C, packed)]
#[derive(Copy, Clone, Debug)]
pub(crate) struct FreeListControl {
    /// Boot value 1, compute value 2, or the render submitted-counter address.
    pub(crate) unk_0: u64,
    /// Buffer slot of the free list.
    pub(crate) buffer_slot: u32,
    pub(crate) unk_c: u32,
    /// Required: see [`FreeListClass`].
    pub(crate) unk_10: u32,
    /// Page list.
    pub(crate) page_list_va: u64,
    /// Required: 0x40000.
    pub(crate) unk_1c: u32,
    pub(crate) unk_20: u32,
    /// Required: [`Self::page_count`].
    pub(crate) unk_24: u32,
    pub(crate) unk_28: u32,
    /// Pages in the free list's blocks.
    pub(crate) page_count: u32,
    /// Run list.
    pub(crate) run_list_va: u64,
    pub(crate) unk_38: [u8; 8],
    /// Required: 4.
    pub(crate) unk_40: u32,
    pub(crate) unk_44: u32,
    /// Run-list qwords in use.
    pub(crate) run_cursor: u32,
    /// [`FreeListState`].
    pub(crate) state_va: u64,
    pub(crate) unk_54: [u8; 0xc],
    /// Required: see [`FreeListClass`].
    pub(crate) unk_60: u32,
    pub(crate) unk_64: u32,
}

// SAFETY: `FreeListControl` consists of integers and arrays of integers only.
unsafe impl Zeroable for FreeListControl {}

static_assert!(core::mem::offset_of!(FreeListControl, page_list_va) == 0x14);
static_assert!(core::mem::offset_of!(FreeListControl, run_cursor) == 0x48);
static_assert!(core::mem::offset_of!(FreeListControl, state_va) == 0x4c);
static_assert!(size_of::<FreeListControl>() == 0x68);

impl FreeListControl {
    /// Runtime render control; submitted and completed work are retained
    /// across release/repopulation of the same pool.
    pub(crate) fn render(args: &FreeListArgs, completed: u64) -> Self {
        let mut control = Self::new(args);
        control.unk_0 = args.state_va;
        control.unk_54[..8].copy_from_slice(&completed.to_le_bytes());
        control
    }

    /// Runtime compute control, with a compact run list and shared counters.
    pub(crate) fn compute(args: &FreeListArgs) -> Self {
        let mut control = Self::new(args);
        control.unk_0 = 2;
        control.unk_10 = 2;
        control.unk_54[8..12].copy_from_slice(&1u32.to_le_bytes());
        control
    }

    /// Offset of the firmware-completed counter. It must be read bytewise.
    pub(crate) const COMPLETED: usize = 0x54;

    /// Returns the control object of the free list `args` describes.
    pub(crate) const fn new(args: &FreeListArgs) -> Self {
        let pages = args.blocks * FREE_LIST_BLOCK_PAGES;
        Self {
            unk_0: 1,
            buffer_slot: args.buffer_slot,
            unk_c: 0,
            unk_10: args.class as u32,
            page_list_va: args.page_list_va,
            unk_1c: 0x4_0000,
            unk_20: 0,
            unk_24: pages,
            unk_28: 0,
            page_count: pages,
            run_list_va: args.run_list_va,
            unk_38: [0; 8],
            unk_40: 4,
            unk_44: 0,
            run_cursor: args.blocks * RUN_SLOT_QWORDS,
            state_va: args.state_va,
            unk_54: [0; 0xc],
            unk_60: args.class.unk_60(),
            unk_64: 0,
        }
    }
}

/// Shared state of a free list, at the start of its page.
///
/// Compute creation writes a 32-bit one at `+0x00` in a zeroed page. The
/// render boot object starts with a 64-bit two and the boot pool with zero.
/// In a live render pool this becomes the submitted-operation counter: the
/// host uses one aligned 64-bit store before making work visible. It is
/// retained across release/repopulation, never reset by a struct copy.
#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub(crate) struct FreeListState {
    /// [`Self::RENDER`], [`Self::BOOT`] or [`Self::COMPUTE`] at boot.
    pub(crate) unk_0: u64,
}

// SAFETY: `FreeListState` consists of integers and arrays of integers only.
unsafe impl Zeroable for FreeListState {}

impl FreeListState {
    /// Boot state of the render free list.
    pub(crate) const RENDER: Self = Self { unk_0: 2 };
    /// Boot state of the boot free list.
    pub(crate) const BOOT: Self = Self { unk_0: 0 };
    /// Boot state of a compute free list.
    pub(crate) const COMPUTE: Self = Self { unk_0: 1 };
}

/// One run-list slot: the run descriptor of one block.
#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub(crate) struct FreeListRunSlot {
    /// Block address, with its page count in the top bits.
    pub(crate) run: u64,
    pub(crate) unk_8: [u8; 0x38],
}

// SAFETY: `FreeListRunSlot` consists of integers and arrays of integers only.
unsafe impl Zeroable for FreeListRunSlot {}

static_assert!(size_of::<FreeListRunSlot>() == RUN_SLOT_QWORDS as usize * size_of::<u64>());

impl FreeListRunSlot {
    /// Returns the slot describing the block at `block_va`.
    pub(crate) const fn new(block_va: u64) -> Self {
        Self {
            run: (block_va & RUN_ADDRESS_MASK)
                | ((FREE_LIST_BLOCK_PAGES as u64) << RUN_PAGES_SHIFT),
            unk_8: [0; 0x38],
        }
    }
}

/// Writes the run descriptors of `blocks` to the start of `slots`.
pub(crate) fn fill_run_list(
    slots: &mut [FreeListRunSlot],
    blocks: impl IntoIterator<Item = u64>,
) -> Result {
    let mut slots = slots.iter_mut();
    for block in blocks {
        *slots.next().ok_or(EINVAL)? = FreeListRunSlot::new(block);
    }
    Ok(())
}

/// Writes the address of every page of `blocks` to the start of `list`.
pub(crate) fn fill_page_list(list: &mut [u64], blocks: impl IntoIterator<Item = u64>) -> Result {
    let mut entries = list.iter_mut();
    for block in blocks {
        for page in 0..u64::from(FREE_LIST_BLOCK_PAGES) {
            *entries.next().ok_or(EINVAL)? = block
                .checked_add(page * FREE_LIST_PAGE_SIZE)
                .ok_or(EOVERFLOW)?;
        }
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// Parameter-buffer and page-pool descriptor tables
// ---------------------------------------------------------------------------

/// One parameter-buffer descriptor, indexed by buffer slot.
#[repr(C)]
pub(crate) struct PbDescriptor {
    pub(crate) unk_0: u32,
    pub(crate) unk_4: u32,
    pub(crate) unk_8: u32,
    pub(crate) unk_c: u32,
}

// SAFETY: `PbDescriptor` consists of integers and arrays of integers only.
unsafe impl Zeroable for PbDescriptor {}

static_assert!(size_of::<PbDescriptor>() == 0x10);

/// Parameter-buffer descriptor table, one page.
#[repr(C)]
pub(crate) struct PbDescriptorTable {
    /// Descriptors.
    pub(crate) entries: [PbDescriptor; PAGE_SIZE / size_of::<PbDescriptor>()],
}

// SAFETY: `PbDescriptorTable` consists of integers only.
unsafe impl Zeroable for PbDescriptorTable {}

impl PbDescriptorTable {
    /// Required boot contents of slots 0 and 1.
    const BOOT: [PbDescriptor; 2] = [
        PbDescriptor {
            unk_0: 0x1_9000,
            unk_4: 0x20,
            unk_8: 0,
            unk_c: 0,
        },
        PbDescriptor {
            unk_0: 0x7_7000,
            unk_4: 0x80,
            unk_8: 0x17,
            unk_c: 0x17,
        },
    ];

    /// Initializes the table.
    pub(crate) fn init(&mut self) {
        clear(self);
        let [slot0, slot1] = Self::BOOT;
        self.entries[0] = slot0;
        self.entries[1] = slot1;
    }
}

/// Bit position of the unit size in [`PagePoolDescriptor::page_list`].
const PAGE_POOL_UNIT_SHIFT: u32 = 41;
/// Required unit size of a page-pool descriptor.
const PAGE_POOL_UNIT: u64 = 0x4_0000;
/// Alignment shift of the page-list address in [`PagePoolDescriptor::page_list`].
const PAGE_POOL_LIST_SHIFT: u32 = 7;
/// Bit position of the free-page count in [`PagePoolDescriptor::counters`].
const PAGE_POOL_FREE_PAGES_SHIFT: u32 = 33;

/// One Page-pool descriptor, indexed by buffer slot.
#[repr(C)]
pub(crate) struct PagePoolDescriptor {
    /// Unit size and page-list address.
    pub(crate) page_list: u64,
    /// Pool counters.
    pub(crate) counters: u64,
    /// Pages in the pool.
    pub(crate) page_count: u64,
    /// Firmware-owned.
    pub(crate) unk_18: u64,
}

// SAFETY: `PagePoolDescriptor` consists of integers and arrays of integers only.
unsafe impl Zeroable for PagePoolDescriptor {}

static_assert!(size_of::<PagePoolDescriptor>() == 0x20);

impl PagePoolDescriptor {
    /// Complete row for a physical compute queue's independent 21-block pool.
    pub(crate) const fn compute(page_list_va: u64) -> Self {
        Self::compute_blocks(page_list_va, 21)
    }

    pub(crate) const fn compute_blocks(page_list_va: u64, blocks: usize) -> Self {
        let pages = blocks as u64 * FREE_LIST_BLOCK_PAGES as u64;
        Self::new(page_list_va, pages << PAGE_POOL_FREE_PAGES_SHIFT, pages)
    }

    const fn new(page_list_va: u64, counters: u64, page_count: u64) -> Self {
        Self {
            page_list: (PAGE_POOL_UNIT << PAGE_POOL_UNIT_SHIFT)
                | (page_list_va >> PAGE_POOL_LIST_SHIFT),
            counters,
            page_count,
            unk_18: 0,
        }
    }
}

/// page-pool descriptor table, one page.
#[repr(C)]
pub(crate) struct PagePoolDescriptorTable {
    /// Descriptors.
    pub(crate) entries: [PagePoolDescriptor; PAGE_SIZE / size_of::<PagePoolDescriptor>()],
}

// SAFETY: `PagePoolDescriptorTable` consists of integers only.
unsafe impl Zeroable for PagePoolDescriptorTable {}

impl PagePoolDescriptorTable {
    /// Required counters and page count of slot 0 at boot.
    const RENDER_COUNTERS: u64 = 0x3d40_0000_3400;
    const RENDER_PAGES: u64 = 0x1d00;

    /// Initializes the table: slot 0 describes the render free list's page
    /// list, slot 1 the compute page list.
    pub(crate) fn init(&mut self, render_page_list_va: u64, compute_page_list_va: u64) {
        clear(self);
        self.entries[0] = PagePoolDescriptor::new(
            render_page_list_va,
            Self::RENDER_COUNTERS,
            Self::RENDER_PAGES,
        );
        self.entries[1] = PagePoolDescriptor::compute(compute_page_list_va);
    }
}

// ---------------------------------------------------------------------------
// Index table
// ---------------------------------------------------------------------------

/// Groups of the index table: first index and number of groups.
const INDEX_GROUPS: [(u32, usize); 2] = [(0x11, 6), (0x3c, 2)];
/// Indices per group.
const INDEX_GROUP_SIZE: usize = 4;
/// Distance between the first indices of consecutive groups.
const INDEX_GROUP_STRIDE: u32 = 5;
/// Entries of the index table.
const INDEX_ENTRIES: usize = (INDEX_GROUPS[0].1 + INDEX_GROUPS[1].1) * INDEX_GROUP_SIZE;

/// Index table at the start of the index object.
#[repr(C)]
pub(crate) struct IndexTable {
    /// Groups of consecutive indices.
    pub(crate) index: [u32; INDEX_ENTRIES],
    pub(crate) unk_80: [u8; PAGE_SIZE - INDEX_ENTRIES * size_of::<u32>()],
}

static_assert!(size_of::<IndexTable>() == PAGE_SIZE);

// SAFETY: `IndexTable` consists of integers only.
unsafe impl Zeroable for IndexTable {}

impl IndexTable {
    /// Initializes the table.
    pub(crate) fn init(&mut self) {
        clear(self);
        let mut entries = self.index.iter_mut();
        for (first, groups) in INDEX_GROUPS {
            for group in 0..groups as u32 {
                for member in 0..INDEX_GROUP_SIZE as u32 {
                    if let Some(entry) = entries.next() {
                        *entry = first + group * INDEX_GROUP_STRIDE + member;
                    }
                }
            }
        }
    }
}

/// Device-global compute state. Activation writes only the named fields, once, before the
/// first compute queue is exposed. The padding may contain firmware-owned state.
#[repr(C)]
pub(crate) struct ComputeGlobalState {
    pub(crate) unk_0: u8,
    unk_1: [u8; 7],
    pub(crate) unk_8: u8,
    unk_9: [u8; 7],
    pub(crate) unk_10: u16,
    unk_12: [u8; 6],
    pub(crate) unk_18: u16,
    unk_1a: [u8; 0x3ee],
    pub(crate) unk_408: u32,
    pub(crate) unk_40c: u32,
    unk_410: [u8; 0x1f4],
    pub(crate) unk_604: u32,
    unk_608: [u8; 0x200],
    pub(crate) scheduler_va: u64,
    unk_810: [u8; 0x3f4],
    pub(crate) unk_c04: u32,
    unk_c08: [u8; 0x1f8],
    pub(crate) unk_e00: u64,
    pub(crate) unk_e08: u64,
    unk_e10: [u8; 0x10],
    pub(crate) unk_e20: u64,
    pub(crate) unk_e28: u64,
}
static_assert!(core::mem::offset_of!(ComputeGlobalState, unk_408) == 0x408);
static_assert!(core::mem::offset_of!(ComputeGlobalState, scheduler_va) == 0x808);
static_assert!(core::mem::offset_of!(ComputeGlobalState, unk_c04) == 0xc04);
static_assert!(core::mem::offset_of!(ComputeGlobalState, unk_e00) == 0xe00);
static_assert!(size_of::<ComputeGlobalState>() == 0xe30);

/// Host-owned recovery report and progress records within the bundle views.
pub(crate) mod recovery {
    pub(crate) const REPORT_VIEW: usize = 3;
    pub(crate) const REASON: usize = 0x4c;
    pub(crate) const PROGRESS_VIEW: usize = 1;
    pub(crate) const PROGRESS_START: usize = 0xa40;
    pub(crate) const PROGRESS_STRIDE: usize = 0x18;
    pub(crate) const VIEW_EXTENTS: [usize; 5] = [0x18c0, 0x0c80, 0x3c00, 0x3380, 0x3300];
}
