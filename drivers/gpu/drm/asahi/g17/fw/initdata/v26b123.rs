// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! T6041 26B123 initialization wire layouts.
//!
//! The merged root keeps its geometry with a new interface version. The main
//! configuration keeps its control ring at `+0x4c0` while moving the unaligned
//! power pointer by 16 bytes. Register descriptors keep their encoding, with
//! 60 slots and a reordered resource table.

use super::{
    MainConfig as LegacyMainConfig, MainConfigArgs, PrimaryViews, RegionC, RegisterMapEntry,
    RegisterWindow, RootArgs, SecondaryRoot,
};
use kernel::prelude::*;

pub(crate) const INTERFACE_VERSION: [u16; 4] = [0x04c0, 0x8398, 0x6339, 0x0c8b];
pub(crate) const REGISTER_MAP_SLOTS: usize = 60;
/// Separate GPTBAT MMIO resource, outside the 60-entry register table.
pub(crate) const GPTBAT_PHYS: u64 = 0x2_88d6_0000;
pub(crate) const GPTBAT_SIZE: u32 = 0x8000;

/// Captured cold-start power state, including its limiter tuning.
#[repr(C)]
pub(crate) struct PowerState(pub(crate) [u8; 0x6d8]);

// SAFETY: A byte array has no invalid representations or padding.
unsafe impl Zeroable for PowerState {}

impl PowerState {
    pub(crate) fn init(&mut self, max_power_mw: u32) {
        self.0.fill(0);
        for (offset, value) in [
            (0x10, 31),
            (0x14, 0x3cf5c290),
            (0x18, 0x3f19999a),
            (0x1c, 40),
            (0x20, 1),
            (0xc8, 11000),
            (0xcc, 8000),
            (0xd8, 0x42080000),
            (0xdc, 0x40baaaaa),
            (0xf0, 9900),
            (0xf4, 200),
            (0x108, 0x42c80000),
            (0x10c, 0x43c80000),
            (0x110, 200),
            (0x128, 1),
            (0x12c, max_power_mw),
            (0x130, 0x3e99999a),
            (0x134, 0x3e0b4396),
            (0x624, 0x41700000),
            (0x628, 0x40a00000),
            (0x62c, 32),
            (0x630, 1000),
            (0x640, 900),
        ] {
            self.0[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
        }
    }
}

/// The first page of region C. Fields following `+0x38` shift by four bytes.
#[repr(C)]
pub(crate) struct RegionCPrefix(pub(crate) [u8; 0x1000]);

// SAFETY: A byte array has no invalid representations or padding.
unsafe impl Zeroable for RegionCPrefix {}

impl RegionCPrefix {
    pub(crate) fn init(&mut self, legacy: &RegionC) {
        // SAFETY: RegionC is packed and all fields are initialized integers.
        let old =
            unsafe { core::slice::from_raw_parts((legacy as *const RegionC).cast::<u8>(), 0x1000) };
        self.0[..0x38].copy_from_slice(&old[..0x38]);
        self.0[0x38..0x3c].copy_from_slice(&1u32.to_le_bytes());
        self.0[0x3c..].copy_from_slice(&old[0x38..0xffc]);
        // Cold-start policy words in previously reserved fields.
        self.0[0x9dc..0x9e0].copy_from_slice(&50u32.to_le_bytes());
        self.0[0xe0c..0xe10].copy_from_slice(&250u32.to_le_bytes());
    }
}

pub(crate) struct PowerArgs {
    pub(crate) config_va: u64,
    pub(crate) aon_va: u64,
    /// Caller value of the build's variable startup word.
    pub(crate) startup_word: u32,
}

#[repr(C)]
pub(crate) struct PowerConfig(pub(crate) [u8; 0x6c48]);

// SAFETY: Byte arrays have no invalid representations or padding.
unsafe impl Zeroable for PowerConfig {}

static_assert!(size_of::<PowerConfig>() == 0x6c48);

impl PowerConfig {
    pub(crate) fn init(
        &mut self,
        legacy: &super::v25g83::PowerConfig25G83,
        args: &PowerArgs,
    ) -> Result {
        let status = args.config_va.checked_add(0x80).ok_or(EINVAL)?;
        if args.config_va == 0 || args.aon_va == 0 {
            return Err(EINVAL);
        }
        let old = &legacy.0;
        self.0.fill(0);
        self.0[4..0x9c].copy_from_slice(&old[4..0x9c]);
        self.0[0xb0..0xfc].copy_from_slice(&old[0xa0..0xec]);
        self.0[0x934..0x2888].copy_from_slice(&old[0x924..0x2878]);
        self.0[0x28a0..0x2cb0].copy_from_slice(&old[0x2878..0x2c88]);
        self.0[0x2d20..0x3160].copy_from_slice(&old[0x2d00..0x3140]);
        self.0[0x3b98..0x630c].copy_from_slice(&old[0x3b50..0x62c4]);
        self.0[0x6348..0x63f0].copy_from_slice(&old[0x6320..0x63c8]);
        // Cold native startup has no legacy reporter/timestamp state. Only the
        // chip's leakage coefficients survive in the relocated tail.
        self.0[0x6ad4..0x6bb4].copy_from_slice(&old[0x642c..0x650c]);
        for (start, end) in [
            (0xac8, 0xacc),
            (0xad0, 0xad4),
            (0xaf0, 0xb34),
            (0xb3c, 0xb40),
            (0xb98, 0xb9c),
            (0xba0, 0xba4),
            (0xbd8, 0xbe8),
            (0x2274, 0x2278),
            (0x22d8, 0x22dc),
            (0x2878, 0x287c),
            (0x2884, 0x2888),
            (0x2900, 0x2904),
            (0x2a34, 0x2a38),
            (0x2a54, 0x2a58),
            (0x2a6c, 0x2b70),
            (0x2bc4, 0x2bc8),
            (0x2bec, 0x2bf8),
        ] {
            self.0[start..end].fill(0);
        }
        // T6041 cold-start policy observed on native 26B123 boots. Some legacy
        // controller tunings revert to the shared defaults in this build.
        for (offset, value) in [
            (0xa80, 8),
            (0xa8c, 0x3f000000),
            (0xa94, 0x3f000000),
            (0xae0, 0x5dc00),
            (0x2204, 0x42c80000),
            (0x2220, 100),
            (0x29e4, 124),
            (0x29f0, 0x3f77bdef),
            (0x29f8, 0x3d042108),
            (0x2bbc, 0x45fa0000),
            (0x2c10, 8000),
            (0x2f0c, 0x42c80000),
        ] {
            self.word32(offset, value);
        }
        self.0[0x2a2c..0x2a30].copy_from_slice(&old[0x9e0..0x9e4]);
        for offset in [0x44, 0x9c, 0xa0, 0xa4, 0xa8, 0x2f34, 0x6ac8] {
            self.word32(offset, 1);
        }
        self.0[0x3b98..0x3ba0].copy_from_slice(&status.to_le_bytes());
        self.0[0x6340..0x6348].copy_from_slice(&args.aon_va.to_le_bytes());
        self.word32(0x6a7c, args.startup_word);
        Ok(())
    }

    fn word32(&mut self, offset: usize, value: u32) {
        self.0[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
    }
}

pub(crate) fn merged_root(
    args: &RootArgs,
    status_b_va: u64,
    power_state_va: u64,
    config_va: u64,
) -> SecondaryRoot {
    let mut root = SecondaryRoot::merged(args, status_b_va, power_state_va, config_va);
    root.root.version = INTERFACE_VERSION;
    root
}

/// The common main-config fields followed by the 26B123 power pointer.
#[repr(C, packed)]
pub(crate) struct MainConfig {
    common: [u8; 0x471],
    reserved_471: [u8; 0x10],
    pub(crate) power_config_va: u64,
    reserved_489: [u8; 0x37],
}

/// Addresses of independently owned channel, descriptor and view objects.
pub(crate) struct MainArgs<'a> {
    pub(crate) common: &'a MainConfigArgs,
    pub(crate) primary_views: &'a PrimaryViews,
    pub(crate) view_va: [u64; 5],
    pub(crate) firmware_area_va: [u64; 2],
    pub(crate) work_rings_va: u64,
    pub(crate) power_config_va: u64,
}

// SAFETY: This is a packed sequence of integers and byte arrays.
unsafe impl Zeroable for MainConfig {}

static_assert!(core::mem::offset_of!(MainConfig, power_config_va) == 0x481);
static_assert!(size_of::<MainConfig>() == 0x4c0);

impl MainConfig {
    /// Builds all fields, including the merged-role defaults, from host inputs.
    /// Work rings may be placed independently of the hardware-data bundle.
    pub(crate) fn init_merged(&mut self, args: &MainArgs<'_>) -> Result {
        // SAFETY: LegacyMainConfig contains integers and byte arrays only.
        let mut common: LegacyMainConfig = unsafe { core::mem::zeroed() };
        common.init_primary(args.common, args.primary_views);
        common.view_va = args.view_va;
        common.firmware_area_va = args.firmware_area_va;
        common.unk_300 = 4;
        common.unk_32c = 0;
        common.unk_344 = 0;
        for channel in 0..super::WORK_CHANNELS {
            let offset = super::work_ring_offset(channel) - super::bundle::WORK_RINGS;
            common.channels[channel].0[3] = args
                .work_rings_va
                .checked_add(offset as u64)
                .ok_or(EINVAL)?;
        }
        self.init(&common, args.power_config_va);
        Ok(())
    }

    /// Reuses the common channel/view/descriptor fields. The old power slot and
    /// both reserved tails are zero, independently of the legacy pointer value.
    pub(crate) fn init(&mut self, common: &LegacyMainConfig, power_config_va: u64) {
        // SAFETY: LegacyMainConfig is packed and contains initialized integers
        // only. The common prefix ends before its old power pointer.
        let bytes = unsafe {
            core::slice::from_raw_parts((common as *const LegacyMainConfig).cast::<u8>(), 0x471)
        };
        self.common.copy_from_slice(bytes);
        self.reserved_471.fill(0);
        self.power_config_va = power_config_va;
        self.reserved_489.fill(0);
    }
}

/// Slots with the reserved/shared bit, including descriptors with no mapping.
const SHARED_SLOTS: &[usize] = &[
    0, 2, 3, 5, 6, 7, 8, 9, 11, 12, 17, 20, 22, 25, 26, 27, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40,
    42, 43, 44, 45, 46, 47, 48, 52, 53, 54, 55, 58,
];

/// The 25 physical T6041 windows keep their tuples, but 13 change slots.
pub(crate) fn register_slot(old: usize) -> Result<usize> {
    Ok(match old {
        0 | 3 | 4 | 9 | 10 | 11 | 12 | 14 | 15 | 17 | 20 | 22 => old,
        26 => 31,
        28 => 33,
        29 => 34,
        31 => 36,
        32 => 37,
        33 => 38,
        34 => 39,
        35 => 40,
        37 => 42,
        39 => 44,
        40 => 45,
        41 => 46,
        43 => 48,
        _ => return Err(EINVAL),
    })
}

/// Register descriptors at hardware-data `+0x640`.
#[repr(C)]
pub(crate) struct RegisterTable {
    pub(crate) entries: [RegisterMapEntry; REGISTER_MAP_SLOTS],
}

// SAFETY: RegisterMapEntry consists of integers; so does an array of it.
unsafe impl Zeroable for RegisterTable {}

static_assert!(size_of::<RegisterTable>() == 0x960);

impl RegisterTable {
    /// Builds the table from host-owned mappings; no native GPUVAs are copied.
    pub(crate) fn init(&mut self, windows: &[RegisterWindow]) -> Result {
        if windows.len() != 25 {
            return Err(EINVAL);
        }
        super::clear(self);
        for &slot in SHARED_SLOTS {
            self.entries[slot] = RegisterMapEntry::reserved();
        }
        let mut used = [false; REGISTER_MAP_SLOTS];
        for window in windows {
            let slot = register_slot(window.slot)?;
            if window.phys == 0 || window.va == 0 || window.size == 0 || window.instances == 0 {
                return Err(EINVAL);
            }
            window.size.checked_mul(window.instances).ok_or(EINVAL)?;
            if used[slot] {
                return Err(EINVAL);
            }
            used[slot] = true;
            self.entries[slot] = RegisterMapEntry::window(window);
        }
        Ok(())
    }
}

/// Addresses of separately owned objects and optional native calibration policy words.
pub(crate) struct HardwareArgs {
    pub(crate) gptbat_va: u64,
    pub(crate) gptbat_size: u32,
    pub(crate) completion_low: [u64; 2],
    pub(crate) completion_high: [u64; 2],
    pub(crate) qos_va: u64,
    pub(crate) auxiliary_va: u64,
    /// Native relative-performance calibration words when preserving exact fractions;
    /// `None` keeps the integer relative ladders encoded from `cfg.perf` in `legacy`.
    pub(crate) relative_words: Option<([u32; 11], [u32; 11])>,
}

/// The logical hardware-data object, excluding the aggregate allocation's peers.
#[repr(C)]
pub(crate) struct HardwareData(pub(crate) [u8; 0x2850]);

// SAFETY: A byte array has no invalid representations or padding.
unsafe impl Zeroable for HardwareData {}

impl HardwareData {
    /// Encodes the shared 25G83 fields in the 26B123 layout. Distinct insertions
    /// surround the register table, completion descriptors and auxiliary pointer.
    /// The supplied legacy object is an intermediate host value, not firmware data.
    pub(crate) fn init(
        &mut self,
        legacy: &super::v25g83::HwData25G83,
        registers: &RegisterTable,
        pointers: &HardwareArgs,
    ) -> Result {
        if pointers.gptbat_size != GPTBAT_SIZE
            || pointers.gptbat_va == 0
            || pointers.qos_va == 0
            || pointers.auxiliary_va == 0
        {
            return Err(EINVAL);
        }
        // SAFETY: These packed integer objects contain initialized bytes only.
        let old = unsafe {
            core::slice::from_raw_parts(
                (legacy as *const super::v25g83::HwData25G83).cast::<u8>(),
                size_of::<super::v25g83::HwData25G83>(),
            )
        };
        let table = unsafe {
            core::slice::from_raw_parts(
                (registers as *const RegisterTable).cast::<u8>(),
                size_of::<RegisterTable>(),
            )
        };
        self.0.fill(0);
        self.0[..0x640].copy_from_slice(&old[..0x640]);
        self.0[0x640..0xfa0].copy_from_slice(table);
        self.0[0xfb0..0xfdc].copy_from_slice(&old[0xe90..0xebc]);
        self.0[0xfdc..0x2734].copy_from_slice(&old[0xec0..0x2618]);
        self.0[0x2738..0x27e0].copy_from_slice(&old[0x2618..0x26c0]);
        self.0[0x27e8..0x2850].copy_from_slice(&old[0x26c0..0x2728]);
        self.word64(0xfa0, pointers.gptbat_va);
        self.word32(0xfa8, pointers.gptbat_size);
        // T6041-specific native startup policy words.
        for (offset, value) in [
            (0x1044, 12),
            (0x10c8, 0),
            (0x10d4, 20),
            (0x271c, 150),
            (0x272c, 0),
            (0x2734, 1),
            (0x2844, 0x10000),
            (0x2848, 0x10000),
        ] {
            self.word32(offset, value);
        }
        for (index, offset) in [0x2740, 0x2780].into_iter().enumerate() {
            self.word64(offset, pointers.completion_low[index]);
            self.word64(offset + 8, pointers.completion_high[index]);
        }
        self.word64(0x27d8, pointers.qos_va);
        self.word64(0x27e0, pointers.auxiliary_va);
        if let Some((relative_a, relative_b)) = pointers.relative_words {
            for index in 0..11 {
                self.word32(0x19e4 + index * 4, relative_a[index]);
                self.word32(0x1a24 + index * 4, relative_b[index]);
            }
        }
        Ok(())
    }

    fn word32(&mut self, offset: usize, value: u32) {
        self.0[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
    }

    fn word64(&mut self, offset: usize, value: u64) {
        self.0[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
    }
}

pub(crate) struct AonSensor {
    pub(crate) key: [u8; 4],
    pub(crate) selector: u32,
    pub(crate) config: u32,
}

/// Nineteen 12-byte native sensor rows, with unused rows zeroed.
#[repr(C)]
pub(crate) struct AonTable(pub(crate) [[u32; 3]; 19]);

// SAFETY: Integer arrays only.
unsafe impl Zeroable for AonTable {}

static_assert!(size_of::<AonTable>() == 0xe4);

impl AonTable {
    pub(crate) fn init(&mut self, sensors: &[AonSensor]) -> Result {
        if sensors.len() > 19 {
            return Err(EINVAL);
        }
        super::clear(self);
        for (index, sensor) in sensors.iter().enumerate() {
            if sensor.key == [0; 4] || sensors[..index].iter().any(|row| row.key == sensor.key) {
                return Err(EINVAL);
            }
            self.0[index] = [
                u32::from_be_bytes(sensor.key),
                sensor.selector,
                sensor.config,
            ];
        }
        Ok(())
    }
}
