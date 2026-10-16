// SPDX-License-Identifier: GPL-2.0-only OR MIT


use core::mem::{offset_of, size_of};

use kernel::{
    c_str, device,
    device::Core,
    platform,
    prelude::*,
    time::{delay::fsleep, Delta},
};

use crate::{
    float::F32,
    fw::initdata::raw,
    hw, initdata,
    initdata::{G15Layout, G15Options},
    m3_config::Records,
    m3_firmware::Firmware,
    m3_init_layout as init,
    m3_init_storage as storage,
    m3_memory::Buffer,
    m3_params::{self, G15Debug, InitDataParam, ThermalMode},
    m3_soc::{IoMapping, PstateTable, Soc},
    t8122_hwdata,
    t8122_start::Experiment,
};

// The T8122 HwDataA +0x418c block (`t8122_hwdata`) covers exactly the G13-era leakage
// coefficient fields from core_leak_coef[4] to the end of aux_leak_coef, which the G15 builder
// leaves zero, and nothing the builder fills.
const _: () = {
    type A = raw::HwDataAG15V14_8_3;
    assert!(offset_of!(A, core_leak_coef) == 0x417c);
    assert!(offset_of!(A, sram_leak_coef) == 0x419c);
    assert!(offset_of!(A, aux_leak_coef) == 0x41bc);
    assert!(t8122_hwdata::OFFSET == offset_of!(A, core_leak_coef) + 4 * size_of::<F32>());
    assert!(
        t8122_hwdata::OFFSET + t8122_hwdata::SIZE
            == offset_of!(A, aux_leak_coef) + size_of::<raw::AuxLeakCoef>()
    );
    assert!(t8122_hwdata::OFFSET + t8122_hwdata::SIZE <= size_of::<A>());
};

/// Owner indices (`m3_init_storage`) of the objects the typed builders generate.
pub(crate) const INITDATA: usize = storage::ROOT;
pub(crate) const RUNTIME_POINTERS: usize = storage::RUNTIME_POINTERS;
pub(crate) const HWDATA: usize = storage::HARDWARE_DATA;
pub(crate) const GLOBALS: usize = storage::GLOBALS;
pub(crate) const GLOBALS_POWER: usize = storage::GLOBALS_POWER;
pub(crate) const STATUS: usize = storage::CONTROL_REGION;
pub(crate) const DEBUG: usize = storage::RUNTIME_FLAGS;
/// HwDataA follows HwDataB inside the HwData object.
pub(crate) const HWDATA_A: usize = 0x4580;

/// Offset of HwDataB's IO-mapping table and the size of one entry.
const IO_MAPPINGS: usize = offset_of!(raw::HwDataBG15V14_8_3, io_mappings);
const IO_MAPPING_SIZE: usize = size_of::<raw::IOMapping>();
/// Number of IO-mapping slots of HwDataB.
const IO_MAPPING_SLOTS: usize = 31;

/// The T6030 firmware IO mappings of the runtime backend (HwDataB slot, physical address, total
/// size, element size, writable). The runtime maps IO with its own IO-mapping table
/// (`m3_init_storage::T6030_IOMAPS`), which must cover every entry (checked when the contents
/// are built). The physical addresses are T6030 SoC register blocks, the same ones the G15 manager
/// maps (`hw::t6030`).
pub(crate) const T6030_IO_MAPPINGS: [IoMapping; 15] = [
    (0, 0x2_90d0_0000, 0x14_4000, 0x14_4000, true), // Fender
    (1, 0x2_0e10_1000, 1, 1, false),                // AIC timer
    (2, 0x3_5101_4000, 0x4000, 0x4000, true),       // AIC software interrupts
    (3, 0x2_9000_0000, 0x2_0000, 0x2_0000, true),   // RGX
    (7, 0x3_502b_c000, 0x1000, 0x1000, false),      // analog temperature sensors
    (9, 0x2_90e0_8000, 0x8000, 0x8000, true),       // metrology sensors
    (11, 0x2_2000_0000, 0x12_a000, 0x9_5000, true), // memory cache, two instances
    (12, 0x3_5104_c000, 0x4000, 0x4000, false),     // AIC banked registers
    (18, 0x3_503d_0000, 0x4000, 0x4000, true),      // telemetry dashboard
    (19, 0x3_503c_0000, 0x4000, 0x4000, false),     // telemetry dashboard (read)
    (20, 0x3_503d_8000, 0x4000, 0x4000, true),      // telemetry dashboard (config)
    (23, 0x2_9300_0000, 0x40_0000, 0x40_0000, true), // AFR registers
    (25, 0x3_0945_c000, 1, 1, true),                // ANE doorbell
    (26, 0x3_5028_0000, 0x8000, 0x8000, false),     // PMS metrology sensors
    (29, 0x2_90e5_c000, 0x4000, 0x4000, false),     // GFX clock generator
];

/// The T8122 firmware IO mappings of the runtime backend, in the same form: the blocks of
/// `hw::t8122`, with the exact register, total size and element size the G15G firmware takes in
/// each slot. The AIC timer entry is the T6030 one, the same register on every SoC.
/// `fill_io_mappings` checks every entry against the runtime's IO maps of the SoC
/// ([`T8122_IOMAPS`]) before writing it.
pub(crate) const T8122_IO_MAPPINGS: [IoMapping; 12] = [
    (0, 0x2_90d0_0000, 0x10_4000, 0x10_4000, true), // Fender
    (1, 0x2_0e10_1000, 1, 1, false),                // AIC timer
    (2, 0x2_d101_4048, 1, 1, true),                 // AIC software interrupt register
    (3, 0x2_9000_0000, 0x2_0000, 0x2_0000, true),   // RGX
    (9, 0x2_90e0_8000, 0x8000, 0x8000, true),       // metrology sensors
    (10, 0x2_90d0_d000, 0x1000, 0x1000, true),      // GM GIFAF registers
    (11, 0x2_2000_0000, 0xa_a000, 0x5_5000, true),  // memory cache, two instances
    (18, 0x2_d03d_0000, 0x1000, 0x1000, true),      // telemetry dashboard
    (19, 0x2_d03c_0000, 0x2000, 0x2000, false),     // telemetry dashboard (read)
    (25, 0x3_1145_c000, 1, 1, true),                // ANE doorbell
    (26, 0x2_d028_0000, 0x8000, 0x8000, false),     // PMS metrology sensors
    (29, 0x2_90e1_c000, 0x4000, 0x4000, false),     // GPU clock generator
];

/// The T8122 runtime's IO maps (`m3_soc::T8122.iomaps`): its IO mappings, laid out as the T6030
/// ones are.
pub(crate) static T8122_IOMAPS: [storage::IoMap; 12] =
    storage::pack_iomaps(&T8122_IO_MAPPINGS, storage::IOMAP_BASE);

/// Whether every entry of `mappings` is a whole number of elements in the HwConfig block of the
/// same slot, with the same writability: the check `fill_io_mappings` makes when it builds the
/// InitData.
const fn same_blocks(mappings: &[IoMapping], cfg: &hw::HwConfig) -> bool {
    let mut i = 0;
    while i < mappings.len() {
        let (slot, phys, total, element, writable) = mappings[i];
        if slot >= cfg.io_mappings.len() || element == 0 || total % element != 0 {
            return false;
        }
        match cfg.io_mappings[slot] {
            Some(block)
                if block.base as u64 & !0x3fff == phys & !0x3fff && block.writable == writable => {}
            _ => return false,
        }
        i += 1;
    }
    true
}

const _: () = assert!(same_blocks(&T8122_IO_MAPPINGS, &hw::t8122::HWCONFIG_T8122));

// The T6030 IO maps are the packed IO maps of its IO mappings, the layout the T8122 start
// experiment derives its own from (`storage::pack_iomaps`).
const _: () = assert!(storage::same_iomaps(
    &storage::pack_iomaps(&T6030_IO_MAPPINGS, storage::IOMAP_BASE),
    &storage::T6030_IOMAPS
));

/// Highest performance state device-tree InitData may use without the driver's thermal limit
/// (`asahi.m3_thermal=off`), and the default thermal fallback for other SoCs. The T8122
/// experiment uses its lower default ceiling as the thermal fallback. The InitData's
/// fast-die temperature controller words stay 0, so the firmware does not limit the die
/// temperature by itself.
pub(crate) const ADT_MAX_PSTATE_LIMIT: u32 = 5;

/// The performance states the runtime backend lets the firmware use.
#[derive(Copy, Clone, Debug)]
pub(crate) struct PstatePolicy {
    /// Highest entry of the published table.
    pub(crate) table_max: u32,
    /// Frequency of the highest entry of the published table, in MHz.
    pub(crate) table_max_mhz: u32,
    /// The ceiling: the highest entry the firmware is given (the InitData's highest-state
    /// words). A reported state above it marks the GPU failed.
    pub(crate) max: u32,
    /// The runtime cap at boot (the Globals cap words): the highest entry the firmware may use
    /// until the thermal limit changes it. Equal to `max` unless the thermal limit is on or
    /// holding, which start here and never go below it.
    pub(crate) safe: u32,
    /// The entry requested at boot, at most `safe`.
    pub(crate) boot: u32,
    /// Frequency of the ceiling entry, in MHz.
    pub(crate) max_mhz: u32,
    /// Frequency of the `safe` entry, in MHz.
    pub(crate) safe_mhz: u32,
    /// Frequency of the boot entry, in MHz.
    pub(crate) boot_mhz: u32,
    /// Frequency of every entry of the published table, in MHz (entry 0 is the off state).
    pub(crate) freqs: [u32; 16],
    /// `asahi.m3_thermal`.
    pub(crate) thermal: ThermalMode,
}

impl PstatePolicy {
    /// The highest frequency the GPU may run at this boot, for userspace: the ceiling's when
    /// the thermal limit is on, the runtime cap's otherwise.
    pub(crate) fn reported_max_mhz(&self) -> u32 {
        match self.thermal {
            ThermalMode::On => self.max_mhz,
            ThermalMode::Off | ThermalMode::Hold => self.safe_mhz,
        }
    }

    /// Clamp the `asahi.m3_max_pstate` and `asahi.m3_boot_pstate` settings to the published table
    /// of `hwdata` (the generated HwData object image), and to `ceiling` when given (the T8122
    /// start experiment's `asahi.t8122_pstate_cap`).
    fn new(dev: &device::Device, hwdata: &[u8], ceiling: Option<u32>) -> Result<Self> {
        let hwb_max = offset_of!(raw::HwDataBG15V14_8_3, max_pstate);
        let hwb_freq = offset_of!(raw::HwDataBG15V14_8_3, frequencies);
        let table_max = read_u32(hwdata, hwb_max)?;
        if table_max == 0 || table_max >= 16 {
            return Err(EINVAL);
        }
        let freq = |i: u32| read_u32(hwdata, hwb_freq + 4 * i as usize);
        // Unset: the thermal limit is on.
        let thermal = m3_params::thermal_param().unwrap_or(ThermalMode::On);
        // Unset: the whole table when the thermal limit bounds the runtime cap, the default cap
        // otherwise.
        let requested = m3_params::max_pstate_param().unwrap_or(match thermal {
            ThermalMode::Hold | ThermalMode::On => u64::from(table_max),
            ThermalMode::Off => m3_params::MAX_PSTATE_DEFAULT,
        });
        let mut max = requested.clamp(1, u64::from(table_max)) as u32;
        if u64::from(max) != requested {
            dev_info!(
                dev,
                "M3: asahi.m3_max_pstate={} clamped to {} (the table has states 1..={})\n",
                requested,
                max,
                table_max
            );
        }
        if thermal == ThermalMode::Off && max > ADT_MAX_PSTATE_LIMIT {
            dev_info!(
                dev,
                "M3: performance cap {} lowered to {}: device-tree InitData leaves the fast-die temperature controller off and the thermal limit is off (asahi.m3_thermal=off)\n",
                max,
                ADT_MAX_PSTATE_LIMIT
            );
            max = ADT_MAX_PSTATE_LIMIT;
        }
        if let Some(ceiling) = ceiling {
            if max > ceiling {
                dev_info!(
                    dev,
                    "M3 G15G start: performance cap {} lowered to {} (asahi.t8122_pstate_cap)\n",
                    max,
                    ceiling
                );
                max = ceiling;
            }
        }
        // Without the thermal limit the runtime cap is the ceiling; with it, the runtime cap
        // starts at the board-specific no-feedback cap. The T8122 experiment may
        // raise its ceiling, but must retain state 2 as its thermal fallback.
        let safe = match thermal {
            ThermalMode::Off => max,
            ThermalMode::Hold | ThermalMode::On => crate::m3_thermal_policy::safe_cap(
                max,
                ceiling.is_some(),
                ADT_MAX_PSTATE_LIMIT,
            ),
        };
        let boot = m3_params::boot_pstate_param()
            .unwrap_or(u64::from(safe))
            .clamp(1, u64::from(safe)) as u32;
        let mut freqs = [0u32; 16];
        for (i, f) in freqs.iter_mut().enumerate().take(table_max as usize + 1) {
            *f = freq(i as u32)?;
        }
        Ok(PstatePolicy {
            table_max,
            table_max_mhz: freqs[table_max as usize],
            max,
            safe,
            boot,
            max_mhz: freqs[max as usize],
            safe_mhz: freqs[safe as usize],
            boot_mhz: freqs[boot as usize],
            freqs,
            thermal,
        })
    }
}

/// The runtime backend's InitData contents: the generated object images and the
/// performance-state policy.
pub(crate) struct Contents {
    /// Generated images, per owner; None for owners that stay zero. Only the three image
    /// owners' images are uploaded; the records' are checked.
    images: KVec<Option<KVVec<u8>>>,
    /// The performance states the firmware may use.
    pub(crate) pstates: PstatePolicy,
    /// The runtime's IO maps of the SoC (`Soc::iomaps`), which cover the InitData IO mappings.
    pub(crate) iomaps: &'static [storage::IoMap],
    /// The accepted performance-state table (`Soc::pstates`): states above the off state, and
    /// the frequency of the highest one in MHz.
    pub(crate) table: (u32, u32),
    /// The HwDataB slots whose IO maps the firmware gets read-only, as a bit mask. 0 (every IO
    /// map read-write) except in the T8122 start experiment.
    pub(crate) read_only_slots: u32,
    /// The HwData object's placement and mapping (`Soc::hwdata_object`).
    pub(crate) hwdata_object: Option<crate::m3_soc::HwDataObject>,
}

fn read_u32(bytes: &[u8], offset: usize) -> Result<u32> {
    let word = bytes.get(offset..offset + 4).ok_or(EINVAL)?;
    Ok(u32::from_le_bytes(word.try_into().map_err(|_| EINVAL)?))
}

fn write(bytes: &mut [u8], offset: usize, data: &[u8]) -> Result {
    bytes
        .get_mut(offset..offset + data.len())
        .ok_or(EINVAL)?
        .copy_from_slice(data);
    Ok(())
}

impl Contents {
    /// Build the InitData contents from this board's device tree (`asahi.m3_initdata`), before
    /// any GPU register is touched.
    ///
    /// `experiment` is the T8122 start experiment's values (`t8122_start`), on an armed T8122
    /// only; it then stands in for the SoC table's missing ones. `t6031` is the same for an
    /// armed T6031 (`t6031_start`).
    pub(crate) fn select(
        pdev: &platform::Device<Core>,
        firmware: &Firmware,
        soc: &Soc,
        experiment: Option<&Experiment>,
        t6031: Option<&crate::t6031_start::Experiment>,
    ) -> Result<Self> {
        let dev = pdev.as_ref();
        let param = m3_params::initdata_param();
        check_version(dev, firmware, experiment, t6031)?;
        let iomaps = match experiment {
            Some(e) => e.iomaps(),
            None => match t6031 {
                Some(e) => e.iomaps(),
                None => soc.iomaps.ok_or(ENODEV)?,
            },
        };
        storage::validate_iomaps(iomaps).map_err(|_| EINVAL)?;
        let images = build_images(dev, firmware, soc, experiment, t6031, iomaps)?;
        let hwdata = images.get(HWDATA).and_then(|i| i.as_deref()).ok_or(EINVAL)?;
        let pstates = PstatePolicy::new(
            dev,
            hwdata,
            experiment.map(|e| e.pstate_cap()).or_else(|| t6031.map(|e| e.pstate_cap())),
        )?;
        let table = match soc.pstates {
            PstateTable::Fixed { states, top_mhz } => (states, top_mhz),
            PstateTable::DeviceTree => crate::m3_board::opp_table_shape(pdev, soc).ok_or_else(|| {
                dev_err!(
                    dev,
                    "M3: the device tree's operating points give no performance-state table\n"
                );
                EINVAL
            })?,
        };
        if (pstates.table_max, pstates.table_max_mhz) != table {
            dev_err!(
                dev,
                "M3: performance-state table 1..={} up to {} MHz, the {} table is 1..={} up to {} MHz\n",
                pstates.table_max,
                pstates.table_max_mhz,
                soc.name,
                table.0,
                table.1
            );
            return Err(EINVAL);
        }
        // The published states never go above the device tree's highest operating point.
        if let Some(dt_khz) = crate::m3_board::max_frequency_khz(pdev) {
            if 1000 * pstates.max_mhz > dt_khz {
                dev_err!(
                    dev,
                    "M3: performance cap {} MHz is above the device tree's highest operating point ({} kHz)\n",
                    pstates.max_mhz,
                    dt_khz
                );
                return Err(EINVAL);
            }
        }
        dev_info!(
            dev,
            "M3: InitData contents: generated from the device tree (asahi.m3_initdata={}); performance states 1..={}, cap {} ({} MHz), boot request {} ({} MHz); thermal limit {}, firmware highest state {} ({} MHz)\n",
            match param {
                InitDataParam::Auto => "auto",
                InitDataParam::Adt => "adt",
            },
            pstates.table_max,
            pstates.safe,
            pstates.safe_mhz,
            pstates.boot,
            pstates.boot_mhz,
            match pstates.thermal {
                ThermalMode::Off => "off",
                ThermalMode::Hold => "holding",
                ThermalMode::On => "on",
            },
            pstates.max,
            pstates.max_mhz
        );
        let read_only_slots = experiment
            .map(|e| e.read_only_slots())
            .or_else(|| t6031.map(|e| e.read_only_slots()))
            .unwrap_or(0);
        let hwdata_object = match t6031 {
            Some(e) => e.hwdata_object(),
            None => soc.hwdata_object,
        };
        Ok(Contents { images, pstates, iomaps, table, read_only_slots, hwdata_object })
    }

    /// Check the contents before they are uploaded.
    pub(crate) fn check_for_upload(&self) -> Result {
        if self.images.len() == storage::COUNT {
            Ok(())
        } else {
            Err(EINVAL)
        }
    }

    /// The initial contents of owner `index`, allocated as `a`: its generated image, or nothing
    /// (zeroed) for owners without one.
    pub(crate) fn initial<'a>(&'a self, index: usize, a: &storage::Allocation) -> Result<&'a [u8]> {
        if a.initial == storage::Initial::Zero {
            return Ok(&[]);
        }
        let image = self.images.get(index).and_then(|i| i.as_deref()).ok_or(EINVAL)?;
        if image.len() != a.size {
            return Err(EINVAL);
        }
        Ok(image)
    }
}

/// The fixed allocation of owner `index`, as a firmware region moved by `delta`.
fn allocation_region(index: usize, delta: u64) -> Result<init::Region> {
    let a = storage::allocation(index).map_err(|_| EINVAL)?;
    init::Region::new(a.address.checked_add(delta).ok_or(EINVAL)?, a.size).map_err(|_| EINVAL)
}

/// The records the runtime constructs, encoded for its fixed allocation table moved by `delta`.
pub(crate) fn allocation_records(delta: u64) -> Result<Records> {
    Records::encode(|index| allocation_region(index, delta))
}

/// The version word of the constructed InitData root.
pub(crate) fn constructed_version() -> Result<u64> {
    let records = allocation_records(0)?;
    Ok(u64::from_le_bytes(records.root[..8].try_into().map_err(|_| EINVAL)?))
}

/// Refuse to go on when the InitData root the runtime constructs carries another version than
/// the loaded firmware expects. The T8122 start experiment gives the firmware the version of
/// `asahi.t8122_initdata_version` instead (`m3_config::Config::new` writes it into the root).
fn check_version(
    dev: &device::Device,
    firmware: &Firmware,
    experiment: Option<&Experiment>,
    t6031: Option<&crate::t6031_start::Experiment>,
) -> Result {
    let version = constructed_version()?;
    if version != firmware.initdata_magic && experiment.is_some() {
        dev_warn!(
            dev,
            "M3 G15G start: InitData version {:#x} (asahi.t8122_initdata_version) replaces the constructed {:#x}\n",
            firmware.initdata_magic,
            version
        );
        return Ok(());
    }
    if version != firmware.initdata_magic && t6031.is_some() {
        dev_warn!(
            dev,
            "M3 G15C start: InitData version {:#x} replaces the constructed {:#x}\n",
            firmware.initdata_magic,
            version
        );
        return Ok(());
    }
    if version != firmware.initdata_magic {
        dev_err!(
            dev,
            "M3: the constructed InitData version {:#x} is not the one the loaded firmware expects ({:#x})\n",
            version,
            firmware.initdata_magic
        );
        return Err(ENODEV);
    }
    Ok(())
}

/// Whether the runtime probe stops before the GPU coprocessor starts (`asahi.g15_debug` bit
/// 53). Logs the stop.
pub(crate) fn stop_before_asc(dev: &device::Device) -> bool {
    let stop = m3_params::g15_debug(G15Debug::StopBeforeAsc);
    if stop {
        dev_info!(
            dev,
            "M3: asahi.g15_debug bit 53: stopping before the GPU coprocessor starts; the firmware was not started\n"
        );
    }
    stop
}

/// The GPU identity the runtime backend admits for `soc` (its SGX ID words, checked in
/// `m3_device` before the firmware starts): on T6030, G15, variant S, revision B1, one die with
/// two clusters of ten core slots.
fn soc_identity(
    soc: &Soc,
    cfg: &'static hw::HwConfig,
    rev_id: Option<hw::GpuRevisionID>,
) -> hw::GpuIdConfig {
    hw::GpuIdConfig {
        gpu_gen: hw::GpuGen::G15,
        gpu_variant: soc.gpu_variant,
        usc_generation: 3,
        gpu_hal_generation: hw::GpuHalGeneration::Legacy,
        gpu_rev: soc.gpu_revision,
        gpu_rev_id: rev_id.unwrap_or(soc.gpu_revision_id),
        num_dies: cfg.num_dies,
        num_clusters: cfg.max_num_clusters,
        num_cores: cfg.max_num_cores,
        num_frags: cfg.max_num_frags,
        num_gps: cfg.max_num_gps,
        total_active_cores: 0,
        core_masks: KVec::new(),
        core_masks_packed: KVec::new(),
    }
}

pub(crate) const BOARD_DATA: [usize; 3] = [HWDATA, GLOBALS, GLOBALS_POWER];

/// The pointers of the constructed records that the typed firmware structures place: record
/// owner, offset, target owner, offset into the target.
const RECORD_POINTERS: [(usize, usize, usize, usize); 7] = {
    type InitData = raw::InitDataG15V14_8_3<'static>;
    type RuntimePointers = raw::RuntimePointersG15V14_8_3<'static>;
    [
        (INITDATA, offset_of!(InitData, runtime_pointers), RUNTIME_POINTERS, 0),
        (INITDATA, offset_of!(InitData, globals), GLOBALS, 0),
        (INITDATA, offset_of!(InitData, debug_block), DEBUG, 0),
        (INITDATA, offset_of!(InitData, status_block), STATUS, 0),
        (INITDATA, offset_of!(InitData, power_ctl_block), GLOBALS_POWER, 0),
        (RUNTIME_POINTERS, offset_of!(RuntimePointers, hwdata_b), HWDATA, 0),
        (RUNTIME_POINTERS, offset_of!(RuntimePointers, hwdata_a), HWDATA, HWDATA_A),
    ]
};

/// The bytes of a constructed record, by owner.
pub(crate) fn record_bytes(records: &Records, owner: usize) -> Option<&[u8]> {
    match owner {
        INITDATA => Some(&records.root[..]),
        RUNTIME_POINTERS => Some(&records.runtime[..]),
        STATUS => Some(&records.control[..]),
        _ => None,
    }
}

fn check_layout() -> Result {
    type InitData = raw::InitDataG15V14_8_3<'static>;
    type RuntimePointers = raw::RuntimePointersG15V14_8_3<'static>;
    let records = allocation_records(0)?;
    let pointer = |owner: usize, offset: usize| -> Option<u64> {
        let bytes = record_bytes(&records, owner)?.get(offset..offset + 8)?;
        Some(u64::from_le_bytes(bytes.try_into().ok()?))
    };
    let mut ok = true;
    for (owner, offset, target, at) in RECORD_POINTERS {
        let a = storage::allocation(target).map_err(|_| EINVAL)?;
        ok &= at < a.size && pointer(owner, offset) == Some(a.address + at as u64);
    }
    for index in 0..storage::COUNT {
        let a = storage::allocation(index).map_err(|_| EINVAL)?;
        let board = BOARD_DATA.contains(&index);
        ok &= board == (a.initial != storage::Initial::Zero);
    }
    let size = |index: usize| storage::allocation(index).map_or(0, |a| a.size);
    let ok = ok
        && size(INITDATA) >= size_of::<InitData>()
        && size(RUNTIME_POINTERS) >= offset_of!(RuntimePointers, hwdata_a) + 8
        && HWDATA_A >= size_of::<raw::HwDataBG15V14_8_3>()
        && size(HWDATA) >= HWDATA_A + size_of::<raw::HwDataAG15V14_8_3>()
        && size(GLOBALS) >= size_of::<raw::GlobalsG15V14_8_3>()
        && size(GLOBALS_POWER) >= size_of::<raw::PowerCtlBlock>()
        && size(STATUS) >= size_of::<raw::StatusBlock>()
        && size(DEBUG) >= size_of::<raw::DebugBlock>();
    if ok { Ok(()) } else { Err(EINVAL) }
}

/// Fill HwDataB's IO-mapping table (the runtime fills the virtual addresses when it maps them),
/// checking every entry against the runtime's IO mapping of the same slot (`iomaps`).
fn fill_io_mappings(
    dev: &device::Device,
    cfg: &'static hw::HwConfig,
    experiment: Option<&Experiment>,
    t6031: Option<&crate::t6031_start::Experiment>,
    mappings: &[IoMapping],
    iomaps: &[storage::IoMap],
    hwdata: &mut [u8],
) -> Result {
    for (i, &(slot, phys, total, element, writable)) in mappings.iter().enumerate() {
        if slot >= 31 || mappings[..i].iter().any(|&(other, ..)| other == slot) {
            return Err(EINVAL);
        }
        let entry = IO_MAPPINGS + slot * IO_MAPPING_SIZE;
        let virt = entry + offset_of!(raw::IOMapping, virt_addr);
        // The same register block as the manager's table.
        let same_block = cfg.io_mappings.get(slot).and_then(|m| m.as_ref()).is_some_and(|m| {
            let base = match (experiment, t6031) {
                (Some(e), _) => e.io_block(slot, m.base as u64),
                (None, Some(e)) => e.io_block(slot, m.base as u64),
                (None, None) => m.base as u64,
            };
            base & !0x3fff == phys & !0x3fff && m.writable == writable
        });
        let covered = iomaps.iter().any(|io| {
            io.slot == slot
                && io.pointer_field() == Ok(virt)
                && io.covers(phys, total)
        });
        if !same_block || !covered || element == 0 || total % element != 0 {
            dev_err!(dev, "M3: IO mapping slot {} ({:#x}) does not match the object layout\n", slot, phys);
            return Err(EINVAL);
        }
        write(hwdata, entry + offset_of!(raw::IOMapping, phys_addr), &phys.to_le_bytes())?;
        write(hwdata, entry + offset_of!(raw::IOMapping, total_size), &total.to_le_bytes())?;
        write(hwdata, entry + offset_of!(raw::IOMapping, element_size), &element.to_le_bytes())?;
        write(
            hwdata,
            entry + offset_of!(raw::IOMapping, readwrite),
            &u64::from(writable).to_le_bytes(),
        )?;
    }
    // Every IO mapping the runtime makes must belong to one of the entries above.
    for io in iomaps.iter() {
        if !mappings.iter().any(|&(slot, ..)| slot == io.slot) {
            dev_err!(dev, "M3: IO mapping slot {} ({:#x}) has no device-tree InitData entry\n", io.slot, io.physical);
            return Err(EINVAL);
        }
    }
    Ok(())
}

pub(crate) fn check_upload(
    dev: &device::Device,
    firmware: &Firmware,
    iomaps: &[storage::IoMap],
    objects: &mut [Buffer],
) -> Result {
    let mut bad = 0u32;
    let version = objects.get_mut(INITDATA).ok_or(EINVAL)?.read_u64(0)?;
    if version != firmware.initdata_magic {
        dev_err!(
            dev,
            "M3: InitData self-check: version {:#x}, the loaded firmware expects {:#x}\n",
            version,
            firmware.initdata_magic
        );
        bad += 1;
    }
    for (owner, offset, target, at) in RECORD_POINTERS {
        let expected = objects.get(target).ok_or(EINVAL)?.va() + at as u64;
        let value = objects.get_mut(owner).ok_or(EINVAL)?.read_u64(offset)?;
        if value != expected {
            dev_err!(
                dev,
                "M3: InitData self-check: owner {} +{:#x} holds {:#x}, not owner {} +{:#x} ({:#x})\n",
                owner,
                offset,
                value,
                target,
                at,
                expected
            );
            bad += 1;
        }
    }
    let hwdata = objects.get_mut(HWDATA).ok_or(EINVAL)?;
    for slot in 0..IO_MAPPING_SLOTS {
        let entry = IO_MAPPINGS + slot * IO_MAPPING_SIZE;
        let phys = hwdata.read_u64(entry + offset_of!(raw::IOMapping, phys_addr))?;
        let virt = hwdata.read_u64(entry + offset_of!(raw::IOMapping, virt_addr))?;
        let total = u64::from(hwdata.read_u32(entry + offset_of!(raw::IOMapping, total_size))?);
        let ok = match iomaps.iter().find(|io| io.slot == slot) {
            Some(io) => {
                phys == io.physical + io.offset as u64
                    && virt == io.address + io.offset as u64
                    && total != 0
                    && phys + total <= io.physical + io.size as u64
            }
            None => phys == 0 && virt == 0,
        };
        if !ok {
            dev_err!(
                dev,
                "M3: InitData self-check: IO mapping slot {} describes {:#x}+{:#x} at {:#x}, which the runtime does not map there\n",
                slot,
                phys,
                total,
                virt
            );
            bad += 1;
        }
    }
    let region = hwdata.read_u64(storage::GPU_REGION_PHYSICAL)?;
    if region != firmware.resources.regions[0].base {
        dev_err!(
            dev,
            "M3: InitData self-check: GPU region {:#x}, the resources admitted {:#x}\n",
            region,
            firmware.resources.regions[0].base
        );
        bad += 1;
    }
    if bad != 0 {
        dev_err!(dev, "M3: InitData self-check failed ({} findings); not publishing the InitData\n", bad);
        return Err(EINVAL);
    }
    dev_info!(
        dev,
        "M3: InitData self-check passed: version {:#x}, {} record pointers, {} IO mappings, GPU region {:#x}\n",
        version,
        RECORD_POINTERS.len(),
        iomaps.len(),
        region
    );
    Ok(())
}

/// The whole part of a positive `F32`, for logs.
fn whole(v: F32) -> u32 {
    let bits = v.to_bits();
    match ((bits >> 23) & 0xff) as i32 - 127 {
        e if e < 0 => 0,
        e if e > 31 => u32::MAX,
        e if e > 23 => ((bits & 0x7f_ffff) | 0x80_0000) << (e - 23),
        e => ((bits & 0x7f_ffff) | 0x80_0000) >> (23 - e),
    }
}

/// The boot loader's decoded GPU leakage fuse values (`Soc::leak_fuse`), logged when it exported
/// them. Returns them when the boot loader's switch is set. With the switch set and no two
/// valid values, refuses: the boot loader then used neither the fuse nor the stand-in.
fn leak_fuse(dev: &device::Device, lf: &crate::m3_soc::LeakFuse) -> Result<Option<[F32; 2]>> {
    let chosen = kernel::of::chosen();
    let switch = chosen
        .as_ref()
        .and_then(|c| c.get_property::<KVec<u8>>(lf.switch).ok())
        .is_some_and(|v| v.as_slice() == b"1\0");
    let values: Option<KVec<F32>> = chosen.as_ref().and_then(|c| c.get_property(lf.values).ok());
    // Positive, finite, nonzero.
    let valid = |v: &F32| {
        let b = v.to_bits();
        b != 0 && b >> 31 == 0 && (b >> 23) & 0xff != 0xff
    };
    let pair = values
        .as_ref()
        .filter(|v| v.len() == 2 && v.iter().all(valid))
        .map(|v| [v[0], v[1]]);
    let names = (lf.values.to_str().unwrap_or("?"), lf.switch.to_str().unwrap_or("?"));
    match (pair, switch) {
        (Some([a, b]), true) => {
            dev_info!(
                dev,
                "M3: GPU leakage from the boot loader's fuse values: {} and {} (/chosen/{}, {} set)\n",
                whole(a),
                whole(b),
                names.0,
                names.1
            );
            Ok(Some([a, b]))
        }
        (Some([a, b]), false) => {
            dev_info!(
                dev,
                "M3: the boot loader's GPU leakage fuse values are {} and {} (/chosen/{}); not used: {} is not set\n",
                whole(a),
                whole(b),
                names.0,
                names.1
            );
            Ok(None)
        }
        (None, true) => {
            dev_err!(
                dev,
                "M3: {} is set, but /chosen/{} does not hold two valid values; GPU startup disabled\n",
                names.1,
                names.0
            );
            Err(EINVAL)
        }
        (None, false) => {
            if values.is_some() {
                dev_warn!(dev, "M3: /chosen/{} is malformed; ignored\n", names.0);
            }
            Ok(None)
        }
    }
}

/// A zeroed image of owner `index`.
fn zeroed(index: usize) -> Result<KVVec<u8>> {
    let size = storage::allocation(index).map_err(|_| EINVAL)?.size;
    Ok(KVVec::from_elem(0u8, size, GFP_KERNEL)?)
}

/// Build the images of the generated owners from the device tree.
fn build_images(
    dev: &device::Device,
    firmware: &Firmware,
    soc: &Soc,
    experiment: Option<&Experiment>,
    t6031: Option<&crate::t6031_start::Experiment>,
    iomaps: &[storage::IoMap],
) -> Result<KVec<Option<KVVec<u8>>>> {
    check_layout().inspect_err(|_| {
        dev_err!(dev, "M3: the constructed InitData records do not match the G15 InitData structures\n")
    })?;
    let cfg: &'static hw::HwConfig = match t6031 {
        Some(e) => e.hwcfg(),
        None => soc.hwcfg.ok_or(ENODEV)?,
    };
    let io_mappings = match experiment {
        Some(e) => e.io_mappings(),
        None => match t6031 {
            Some(e) => e.io_mappings(),
            None => soc.io_mappings.ok_or(ENODEV)?,
        },
    };
    let runtime_hwdata_b = match experiment {
        Some(e) => e.hwdata_b(),
        None => match t6031 {
            Some(e) => e.hwdata_b(),
            None => soc.hwdata_b.ok_or(ENODEV)?,
        },
    };
    let mut pwr = hw::PwrConfig::load(dev, cfg).inspect_err(|e| {
        dev_err!(dev, "M3: cannot read the GPU power configuration from the device tree ({:?})\n", e)
    })?;
    // The boot loader's decoded leakage fuse, when its switch makes it this boot's leakage: value
    // 1 is the core leakage coefficient (the boot loader then writes it as apple,core-leak-coef
    // too), value 2 goes to HwDataA's third leakage table below.
    let fuse = match soc.leak_fuse {
        Some(lf) => leak_fuse(dev, &lf)?,
        None => None,
    };
    if let Some([core, _]) = fuse {
        match pwr.core_leak_coef.first_mut() {
            Some(c) if c.to_bits() != core.to_bits() => {
                dev_info!(
                    dev,
                    "M3: core leakage {} replaces the device tree's {}\n",
                    whole(core),
                    whole(*c)
                );
                *c = core;
            }
            Some(_) => {}
            None => return Err(EINVAL),
        }
    }
    // A SoC whose power model is still a stand-in caps the power target: every operating point's
    // power is scaled by the same factor, so the relative powers stay the boot loader's.
    let power_cap_mw = match t6031 {
        Some(e) => Some(e.power_cap_mw()),
        None => soc.power_target_cap_mw,
    };
    if let Some(cap) = power_cap_mw {
        let max = pwr.max_power_mw;
        if max > cap {
            for ps in pwr.perf_states.iter_mut() {
                ps.pwr_mw = (u64::from(ps.pwr_mw) * u64::from(cap) / u64::from(max)) as u32;
            }
            pwr.max_power_mw = pwr.perf_states.iter().map(|ps| ps.pwr_mw).max().unwrap_or(cap);
            dev_info!(
                dev,
                "M3: GPU power target capped at {} mW (the device tree's operating points reach {} mW)\n",
                pwr.max_power_mw,
                max
            );
        }
    }
    let node = dev.of_node().ok_or(ENODEV)?;
    let dyncfg = hw::DynConfig {
        uat_ttb_base: firmware.resources.regions[0].base,
        id: soc_identity(soc, cfg, t6031.and_then(|e| e.revision_id())),
        pwr,
        firmware_version: node
            .get_property::<KVec<u32>>(c_str!("apple,firmware-version"))
            .unwrap_or_default(),
        hw_data_a: KVVec::new(),
        hw_data_b: KVVec::new(),
        hw_globals: KVVec::new(),
    };
    let g15 = G15Options {
        layout: G15Layout::Runtime,
        split_pstates: true,
        // The cap is applied to the uploaded objects, for either source (`publish_pstates`).
        cap: None,
        timestamp_base: Some(crate::agx_memory::TIMESTAMP_RANGE.start),
        reference_ppm: true,
        runtime_hwdata_b: Some(runtime_hwdata_b),
        per_cluster_voltages: soc.per_cluster_voltages,
    };
    let c = initdata::InitDataBuilderG15V14_8_3::g15_contents(cfg, &dyncfg, &g15).inspect_err(|e| {
        dev_err!(dev, "M3: cannot build InitData from the device tree ({:?})\n", e)
    })?;

    let mut images = KVec::new();
    for _ in 0..storage::COUNT {
        images.push(None, GFP_KERNEL)?;
    }

    // InitData: the magic of the identified firmware, the host-mapped allocations flag and the
    // UAT description; the pointers are the constructed record's.
    type InitData = raw::InitDataG15V14_8_3<'static>;
    let mut root = zeroed(INITDATA)?;
    write(&mut root, offset_of!(InitData, ver_info), &firmware.initdata_magic.to_le_bytes())?;
    write(&mut root, offset_of!(InitData, unk_2c), &1u32.to_le_bytes())?;
    write(&mut root, offset_of!(InitData, uat_page_size), &0x4000u16.to_le_bytes())?;
    write(&mut root, offset_of!(InitData, uat_page_bits), &[14, 3])?;
    for (i, level) in c.uat_levels.iter().enumerate() {
        let at = offset_of!(InitData, uat_level_info) + i * size_of::<raw::UatLevelInfo>();
        write(&mut root, at, initdata::raw_bytes(level))?;
    }
    images[INITDATA] = Some(root);

    // RuntimePointers: only its configuration words; everything else is a pointer.
    type RuntimePointers = raw::RuntimePointersG15V14_8_3<'static>;
    let mut rt = zeroed(RUNTIME_POINTERS)?;
    write(&mut rt, offset_of!(RuntimePointers, unk_2d0), &g15.runtime_pointers_2d0().to_le_bytes())?;
    write(&mut rt, offset_of!(RuntimePointers, cswitch_hist_ctl), &[0xff])?;
    images[RUNTIME_POINTERS] = Some(rt);

    // HwData: HwDataB, then HwDataA.
    let mut hwdata = zeroed(HWDATA)?;
    write(&mut hwdata, 0, initdata::raw_bytes(&*c.hwdata_b))?;
    // The T8122 start experiment's unit masks (`asahi.t8122_unit_mask_a`/`_b`).
    if let Some(e) = experiment {
        type B = raw::HwDataBG15V14_8_3;
        let (a, b) = e.unit_masks();
        write(&mut hwdata, offset_of!(B, unit_mask_a), &a.to_le_bytes())?;
        write(&mut hwdata, offset_of!(B, unit_mask_b), &b.to_le_bytes())?;
        dev_info!(dev, "M3 G15G start: HwDataB unit masks +0x17c0 {:#x}, +0x17c8 {:#x}\n", a, b);
    } else if let Some(e) = t6031 {
        type B = raw::HwDataBG15V14_8_3;
        let (a, b) = e.unit_masks();
        write(&mut hwdata, offset_of!(B, unit_mask_a), &a.to_le_bytes())?;
        write(&mut hwdata, offset_of!(B, unit_mask_b), &b.to_le_bytes())?;
        dev_info!(dev, "M3 G15C start: HwDataB unit masks +0x17c0 {:#x}, +0x17c8 {:#x}\n", a, b);
    }
    write(&mut hwdata, HWDATA_A, initdata::raw_bytes(&*c.hwdata_a))?;
    // The system counter value at InitData creation, the base of the firmware's first power and
    // energy interval.
    let now: u64;
    // SAFETY: reading the architectural counter has no side effects and is allowed at EL1/EL2.
    unsafe { core::arch::asm!("mrs {}, cntpct_el0", out(reg) now, options(nomem, nostack, preserves_flags)) };
    write(
        &mut hwdata,
        HWDATA_A + offset_of!(raw::HwDataAG15V14_8_3, init_timestamp),
        &now.to_le_bytes(),
    )?;
    // The SoC's MTR sensor masks, where its table gives them.
    if let Some(m) = soc.mtr_masks {
        type A = raw::HwDataAG15V14_8_3;
        for at in [offset_of!(A, fast_die0_sensor_mask), offset_of!(A, fast_die0_sensor_mask_2)] {
            write(&mut hwdata, HWDATA_A + at, &m.fast_die.to_le_bytes())?;
        }
        write(&mut hwdata, HWDATA_A + offset_of!(A, fast_die0_sensor_mask_alt), &m.alarm.to_le_bytes())?;
        dev_info!(
            dev,
            "M3: MTR sensor masks: fast-die {:#x}, alarm {:#x}\n",
            m.fast_die,
            m.alarm
        );
    } else if let Some(e) = t6031 {
        let (fast, alarm) = e.mtr_masks(dev)?;
        type A = raw::HwDataAG15V14_8_3;
        for at in [offset_of!(A, fast_die0_sensor_mask), offset_of!(A, fast_die0_sensor_mask_2)] {
            write(&mut hwdata, HWDATA_A + at, &fast.to_le_bytes())?;
        }
        write(&mut hwdata, HWDATA_A + offset_of!(A, fast_die0_sensor_mask_alt), &alarm.to_le_bytes())?;
        dev_info!(dev, "M3: MTR sensor masks: fast-die {:#x}, alarm {:#x}\n", fast, alarm);
    }
    // The second leakage fuse value: the first entry of HwDataA's third leakage table (the first
    // table holds the core coefficient, from the builder).
    if let Some([_, second]) = fuse {
        type A = raw::HwDataAG15V14_8_3;
        let at = HWDATA_A + offset_of!(A, cluster_tables) + 2 * size_of::<[u32; 8]>();
        write(&mut hwdata, at, &second.to_bits().to_le_bytes())?;
    }
    // The SoC's own values for words the shared builder writes otherwise. An armed T6031 copies
    // the T8122 words only when asahi.t6031_fw_words=t8122.
    let (hwdata_words, globals_words): (&[(usize, u32)], &[(usize, u32)]) = match t6031 {
        Some(e) if e.fw_words() => (
            &crate::m3_soc::T8122_HWDATA_WORDS,
            &crate::m3_soc::T8122_GLOBALS_WORDS,
        ),
        Some(_) => (&[], &[]),
        None => (soc.hwdata_words, soc.globals_words),
    };
    for &(at, value) in hwdata_words {
        write(&mut hwdata, at, &value.to_le_bytes())?;
    }
    // T8122 with the G15 14.8.3 firmware (the only image the runtime accepts there): the 20
    // words of crate::t8122_hwdata at HwDataA +0x418c, unless asahi.t8122_hwdata_418c=0.
    if crate::t8122_start::is_t8122(soc)
        && core::ptr::eq(firmware.layout, &crate::m3_firmware::T8122_LAYOUT)
        && cfg.gpu_gen == hw::GpuGen::G15
    {
        if m3_params::t8122_hwdata_418c() {
            let a = hwdata
                .get_mut(HWDATA_A..HWDATA_A + size_of::<raw::HwDataAG15V14_8_3>())
                .ok_or(EINVAL)?;
            t8122_hwdata::write(a).map_err(|()| EINVAL)?;
            dev_info!(
                dev,
                "M3 G15G: HwDataA +0x418c..+0x41db: wrote the 20 words (asahi.t8122_hwdata_418c=0 leaves them 0)\n"
            );
        } else {
            dev_info!(
                dev,
                "M3 G15G: HwDataA +0x418c..+0x41db left 0 (asahi.t8122_hwdata_418c=0)\n"
            );
        }
    }
    // On T8122, log and require the two HwDataB words the firmware's power management depends
    // on: it powers the GPU cores up for a job only when +0xa38 and +0xa40 are both nonzero.
    if core::ptr::eq(soc, &crate::m3_soc::T8122) {
        type B = raw::HwDataBG15V14_8_3;
        let a38 = read_u32(&hwdata, offset_of!(B, unk_460))?;
        let a40 = read_u32(&hwdata, offset_of!(B, unk_468))?;
        dev_info!(dev, "M3: HwDataB +0xa38 {:#x}, +0xa40 {:#x}\n", a38, a40);
        if a38 == 0 || a40 == 0 {
            dev_err!(
                dev,
                "M3: HwDataB +0xa38 or +0xa40 is 0; the firmware would not power the GPU cores\n"
            );
            return Err(EINVAL);
        }
    }
    fill_io_mappings(dev, cfg, experiment, t6031, io_mappings, iomaps, &mut hwdata)?;
    images[HWDATA] = Some(hwdata);

    let mut globals = zeroed(GLOBALS)?;
    write(&mut globals, 0, initdata::raw_bytes(&*c.globals))?;
    // The gate of the firmware's frequency-feedback cap, per SoC.
    let (ut, given) = m3_params::ut_engagement(soc);
    write(&mut globals, offset_of!(raw::GlobalsG15V14_8_3, ut_engagement), &ut.to_le_bytes())?;
    for &(at, value) in globals_words {
        write(&mut globals, at, &value.to_le_bytes())?;
    }
    if !hwdata_words.is_empty() || !globals_words.is_empty() {
        dev_info!(
            dev,
            "M3: {} HwData and {} Globals words set from the {} table\n",
            hwdata_words.len(),
            globals_words.len(),
            soc.name
        );
    }
    if ut != 1 || given {
        dev_info!(
            dev,
            "M3: Globals ut_engagement={}{}\n",
            ut,
            if given { " (asahi.m3_ut_engagement)" } else { "" }
        );
    }
    if crate::m3_globals_debug::apply(soc.chip_id,m3_params::globals_debug_off(),&mut globals)
        .map_err(|_|EINVAL)? {
        dev_info!(dev,"M3: T6030 Globals debug word cleared at +0x2c\n");
    }
    images[GLOBALS] = Some(globals);

    let mut power = zeroed(GLOBALS_POWER)?;
    write(&mut power, 0, initdata::raw_bytes(&*c.power_ctl_block))?;
    images[GLOBALS_POWER] = Some(power);

    let mut status = zeroed(STATUS)?;
    write(&mut status, 0, initdata::raw_bytes(&*c.status_block))?;
    images[STATUS] = Some(status);

    let top = c.tables.primary_state(&dyncfg.pwr, c.tables.len - 1);
    dev_info!(
        dev,
        "M3: device-tree InitData: {} performance states in two voltage-sorted tables from {} device-tree states, top {} MHz at {} mV, SRAM floor {} mV, maximum power {} mW\n",
        c.tables.len - 1,
        dyncfg.pwr.perf_states.len() - 1,
        top.freq_hz / 1_000_000,
        top.max_volt_mv(),
        dyncfg.pwr.min_sram_microvolt.div_ceil(1000),
        dyncfg.pwr.max_power_mw
    );
    log_tables(dev, &c.hwdata_b, c.tables.len);
    Ok(images)
}

/// Log the published performance-state tables, one line per entry: the primary table (fastest
/// state per voltage) and the secondary one (slowest), each with the device-tree index of the
/// entry, and the third table.
fn log_tables(dev: &device::Device, b: &raw::HwDataBG15V14_8_3, len: usize) {
    let uniform = |row: &[u32; 8]| row.iter().all(|v| *v == row[0]);
    for i in 1..len.min(16) {
        let aux = &b.aux_ps_3;
        let ok = uniform(&b.voltages[i]) && uniform(&b.voltages_sram[i]);
        dev_info!(
            dev,
            "M3: performance state {}: primary dt {} {} MHz core {} mV SRAM {} mV; secondary dt {} {} MHz; third {} MHz core {} mV SRAM {} mV{}\n",
            i,
            b.unk_arr_0[i],
            b.frequencies[i],
            b.voltages[i][0],
            b.voltages_sram[i][0],
            b.unk_arr_0[16 + i],
            b.frequencies_2[i],
            aux.frequencies[i],
            aux.voltages[i][0],
            aux.voltages_sram[i][0],
            if ok { "" } else { " (columns differ)" }
        );
    }
}

/// HwDataA words that hold the highest usable performance state scaled by 100.
const HWA_MAX_SCALED: [usize; 15] = {
    type A = raw::HwDataAG15V14_8_3;
    let se = offset_of!(A, unk_e10_0);
    [
        offset_of!(A, max_pstate_scaled),
        offset_of!(A, max_pstate_scaled_2),
        offset_of!(A, max_pstate_scaled_3),
        offset_of!(A, max_pstate_scaled_4),
        offset_of!(A, max_pstate_scaled_5),
        offset_of!(A, max_pstate_scaled_7),
        offset_of!(A, max_pstate_scaled_8),
        offset_of!(A, max_pstate_scaled_9),
        offset_of!(A, max_pstate_scaled_10),
        offset_of!(A, max_pstate_scaled_11),
        offset_of!(A, max_pstate_scaled_12),
        offset_of!(A, max_pstate_scaled_13),
        offset_of!(A, max_pstate_scaled_14),
        se + offset_of!(raw::HwDataA140Extra, max_pstate_scaled_1),
        se + offset_of!(raw::HwDataA140Extra, max_pstate_scaled_2),
    ]
};

/// HwDataA words that hold the base (lowest settled) performance state scaled by 100.
const HWA_BASE_SCALED: [usize; 4] = {
    type A = raw::HwDataAG15V14_8_3;
    [
        offset_of!(A, base_pstate_scaled),
        offset_of!(A, base_pstate_scaled_2),
        offset_of!(A, base_pstate_scaled_3),
        offset_of!(A, base_pstate_scaled_4),
    ]
};

/// Offsets in Globals of the runtime cap words: the second and third power-interface targets and
/// the performance-state cap, each the highest usable state scaled by 100.
const GLOBALS_RUNTIME_CAP: [usize; 3] = {
    type G = raw::GlobalsG15V14_8_3;
    let power_if_target = offset_of!(G, power_if_target);
    [power_if_target + 4, power_if_target + 8, offset_of!(G, perf_state_cap)]
};

pub(crate) fn publish_pstates(
    dev: &device::Device,
    hwdata: &mut Buffer,
    globals: &mut Buffer,
    p: &PstatePolicy,
) -> Result {
    use crate::float::F32;
    type A = raw::HwDataAG15V14_8_3;
    let a = |offset: usize| HWDATA_A + offset;
    let full = 100 * p.table_max;
    let base_scaled = hwdata.read_u32(a(offset_of!(A, base_pstate_scaled)))?;
    let globals_max = GLOBALS_RUNTIME_CAP;

    let mut expected = true;
    for off in HWA_MAX_SCALED {
        expected &= hwdata.read_u32(a(off))? == full;
    }
    for off in HWA_BASE_SCALED {
        expected &= hwdata.read_u32(a(off))? == base_scaled;
    }
    for off in globals_max {
        expected &= globals.read_u32(off)? == full;
    }
    expected &= hwdata.read_u32(a(offset_of!(A, max_pstate_scaled_6)))? == F32::from(full).to_bits();
    let base = base_scaled / 100;
    if !expected
        || base_scaled % 100 != 0
        || base == 0
        || base > p.safe
        || p.boot > p.safe
        || p.safe > p.max
    {
        dev_err!(dev, "M3: unexpected performance-state words in the InitData; not starting the firmware\n");
        return Err(EINVAL);
    }

    let ceiling = 100 * p.max;
    let max_power = F32::from_bits(hwdata.read_u32(a(offset_of!(A, max_power_1)))?);
    let min_duty = hwdata.read_u32(a(offset_of!(A, pwr_min_duty_cycle)))?;
    for off in HWA_MAX_SCALED {
        hwdata.u32(a(off), ceiling)?;
    }
    hwdata.u32(a(offset_of!(A, max_pstate_scaled_6)), F32::from(ceiling).to_bits())?;
    hwdata.u32(a(offset_of!(A, pwr_pstate_related_k)), (-F32::from(ceiling) / max_power).to_bits())?;
    hwdata.u32(
        a(offset_of!(A, pwr_pstate_max_dc_offset)),
        (min_duty as i32 - ceiling as i32) as u32,
    )?;
    hwdata.u32(
        a(offset_of!(A, boost_state_unk_k)),
        (F32::from(p.max - base) / crate::f32!(0.95)).to_bits(),
    )?;
    for off in globals_max {
        globals.u32(off, 100 * p.safe)?;
    }

    for off in [offset_of!(A, actual_pstate), offset_of!(A, tgt_pstate)] {
        hwdata.u32(a(off), p.boot)?;
    }
    let settle = base;
    dev_info!(
        dev,
        "M3: firmware performance request: state {} ({} MHz), base state {}, cap state {} ({} MHz) of 1..={}\n",
        p.boot,
        p.boot_mhz,
        settle,
        p.safe,
        p.safe_mhz,
        p.table_max
    );
    if p.safe != p.max {
        dev_info!(
            dev,
            "M3: firmware highest state {} ({} MHz); the runtime cap starts at {} ({} MHz)\n",
            p.max,
            p.max_mhz,
            p.safe,
            p.safe_mhz
        );
    }
    Ok(())
}

/// Set the runtime cap: the Globals runtime cap words, which the firmware keeps reading while it
/// runs, so the store alone moves the cap, with no message. `cap` must be a state of the
/// published table.
pub(crate) fn publish_runtime_cap(globals: &mut Buffer, cap: u32) -> Result {
    if cap == 0 || cap >= 16 {
        return Err(EINVAL);
    }
    for off in GLOBALS_RUNTIME_CAP {
        globals.u32(off, 100 * cap)?;
    }
    crate::agx_memory::publish();
    Ok(())
}

/// The integer part of a non-negative, finite F32 given by its bits; None otherwise.
fn f32_bits_to_u32(bits: u32) -> Option<u32> {
    let exp = (bits >> 23) & 0xff;
    if bits >> 31 != 0 || exp == 0xff {
        return None;
    }
    if exp < 127 {
        return Some(0);
    }
    let mant = u64::from((bits & 0x7f_ffff) | 0x80_0000);
    let shift = exp - 127;
    let value = if shift >= 23 {
        mant.checked_shl(shift - 23)?
    } else {
        mant >> (23 - shift)
    };
    u32::try_from(value).ok()
}

/// One reading of the performance state: SGX+0xe01000, and the HwDataA actual and current
/// state and frequency (MHz, F32 bits) the firmware publishes.
#[derive(Copy, Clone, PartialEq, Eq)]
struct PstateSample {
    register: u32,
    actual: u32,
    current: u32,
    freq_bits: u32,
}

/// Number of lines logging changes of the performance state per boot.
const PSTATE_LOG_LINES: u32 = 64;

/// Number of lines warning about a reported state above the runtime cap per boot.
const RUNTIME_CAP_WARN_LINES: u32 = 16;

/// Checks the performance state the firmware reports against the ceiling and the runtime cap,
/// after boot and after every job.
///
/// The firmware publishes its actual and current state and its frequency in HwDataA. A
/// frequency above the ceiling entry's, or, when no running frequency is reported, a state in
/// the published table above the ceiling, that is still there when read again 1 ms later fails
/// the check. The same above the runtime cap (when it is below the ceiling) is reported to the
/// caller, which decides whether it persists too long (the thermal limit). Values outside the
/// published table are logged and not enforced: they are not states of this table.
/// SGX+0xe01000 is logged next to them but not enforced, as the index space of that register is
/// not established on every board. The published tables list device-tree operating points only,
/// so the check guards the cap, not the voltages.
pub(crate) struct PstateWatch {
    policy: PstatePolicy,
    last: Option<PstateSample>,
    lines: u32,
    warn_lines: u32,
    register_noted: bool,
    unknown_noted: bool,
    excused_noted: bool,
}

impl PstateWatch {
    pub(crate) fn new(policy: PstatePolicy) -> Self {
        PstateWatch {
            policy,
            last: None,
            lines: 0,
            warn_lines: 0,
            register_noted: false,
            unknown_noted: false,
            excused_noted: false,
        }
    }

    fn sample(hwdata: &mut Buffer, register: u32) -> Result<PstateSample> {
        type A = raw::HwDataAG15V14_8_3;
        Ok(PstateSample {
            register,
            actual: hwdata.read_u32(HWDATA_A + offset_of!(A, actual_pstate))?,
            current: hwdata.read_u32(HWDATA_A + offset_of!(A, cur_pstate))?,
            freq_bits: hwdata.read_u32(HWDATA_A + offset_of!(A, freq_mhz))?,
        })
    }

    /// Frequency of entry `state` of the published table, in MHz.
    fn mhz(&self, state: u32) -> u32 {
        self.policy.freqs.get(state as usize).copied().unwrap_or(0)
    }

    /// The reported frequency in MHz, if it is a running frequency of the published table.
    fn freq_mhz(&self, s: &PstateSample) -> Option<u32> {
        f32_bits_to_u32(s.freq_bits).filter(|mhz| *mhz > 0 && *mhz <= self.policy.table_max_mhz)
    }

    /// Whether a reported state lies within the published table but above `cap`.
    fn state_above(&self, s: &PstateSample, cap: u32) -> Option<(&'static str, u32)> {
        let above = |v: u32| v > cap && v <= self.policy.table_max;
        if above(s.actual) {
            Some(("actual state", s.actual))
        } else if above(s.current) {
            Some(("current state", s.current))
        } else {
            None
        }
    }

    /// The reported value above `cap` that lies within the published table, if any. The
    /// frequency is the physical quantity: a state above the cap is excused when the reported
    /// frequency is a running frequency within the cap (the state words then use another
    /// index space), and counts otherwise.
    fn above(&self, s: &PstateSample, cap: u32) -> Option<(&'static str, u32)> {
        match self.freq_mhz(s) {
            Some(mhz) if mhz > self.mhz(cap) => Some(("frequency (MHz)", mhz)),
            Some(_) => None,
            None => self.state_above(s, cap),
        }
    }

    /// Whether a reported value is not a state or frequency of the published table.
    fn unknown(&self, s: &PstateSample) -> bool {
        let p = &self.policy;
        s.actual > p.table_max
            || s.current > p.table_max
            || f32_bits_to_u32(s.freq_bits).is_none_or(|mhz| mhz > p.table_max_mhz)
    }

    fn log(&self, dev: &device::Device, what: &str, s: &PstateSample, cap: u32) {
        dev_info!(
            dev,
            "M3: performance state {}: firmware actual {} current {} frequency {} MHz, sgx+0xe01000 {:#x}; cap state {} ({} MHz)\n",
            what,
            s.actual,
            s.current,
            f32_bits_to_u32(s.freq_bits).unwrap_or(u32::MAX),
            s.register,
            cap,
            self.mhz(cap)
        );
    }

    /// Read the existing firmware state words and SGX state register without
    /// requesting a state, publishing a cap, or enforcing a sample.
    pub(crate) fn log_active(&self, dev: &device::Device, hwdata: &mut Buffer,
                            register: u32, cap: u32, sequence: u32) -> Result {
        let s = Self::sample(hwdata, register)?;
        dev_info!(dev,
            "M3_ACTIVE_POWER sample={} actual={} current={} freq_bits={:#x} sgx={:#x} cap={} cap_mhz={}\n",
            sequence, s.actual, s.current, s.freq_bits, s.register, cap, self.mhz(cap));
        Ok(())
    }

    /// Check the reported performance state. `register` reads SGX+0xe01000; `what` names the
    /// point of the check for the log; `cap` is the runtime cap the firmware may use now
    /// (clamped to the ceiling). Returns ERANGE when the firmware stays above the ceiling;
    /// otherwise whether it stays above `cap`, and the highest state it reports (0 if neither
    /// state word is a state of the published table).
    pub(crate) fn check(
        &mut self,
        dev: &device::Device,
        hwdata: &mut Buffer,
        register: impl Fn() -> Result<u32>,
        what: &str,
        cap: u32,
    ) -> Result<(bool, u32)> {
        let ceiling = self.policy.max;
        let cap = cap.clamp(1, ceiling);
        let mut s = Self::sample(hwdata, register()?)?;
        let mut over = false;
        if let Some((field, value)) = self.above(&s, cap) {
            if cap == ceiling || self.warn_lines < RUNTIME_CAP_WARN_LINES {
                self.warn_lines = self.warn_lines.saturating_add(u32::from(cap != ceiling));
                dev_warn!(
                    dev,
                    "M3: firmware reports {} {} above the cap {} ({} MHz) {}; reading again\n",
                    field,
                    value,
                    cap,
                    self.mhz(cap),
                    what
                );
            }
            fsleep(Delta::from_millis(1));
            s = Self::sample(hwdata, register()?)?;
            if let Some((field, value)) = self.above(&s, ceiling) {
                self.log(dev, what, &s, cap);
                dev_err!(
                    dev,
                    "M3: firmware {} {} stays above the cap {} ({} MHz); marking the GPU failed\n",
                    field,
                    value,
                    ceiling,
                    self.mhz(ceiling)
                );
                return Err(ERANGE);
            }
            over = self.above(&s, cap).is_some();
        }
        if !self.excused_noted {
            if let Some((field, value)) = self.state_above(&s, cap) {
                if !over {
                    self.excused_noted = true;
                    dev_info!(
                        dev,
                        "M3: firmware {} {} is above the cap {} but its frequency is within it {}; not enforced\n",
                        field,
                        value,
                        cap,
                        what
                    );
                }
            }
        }
        if !self.unknown_noted && self.unknown(&s) {
            self.unknown_noted = true;
            dev_info!(
                dev,
                "M3: firmware performance words outside the published table (states 1..={}, up to {} MHz) {}; not enforced\n",
                self.policy.table_max,
                self.policy.table_max_mhz,
                what
            );
            self.log(dev, what, &s, cap);
        }
        if !self.register_noted && s.register & 0xf > cap {
            self.register_noted = true;
            dev_info!(
                dev,
                "M3: sgx+0xe01000 reads {:#x}, above the cap as a table index; logged only\n",
                s.register
            );
        }
        let key = PstateSample {
            register: s.register & 0xf,
            ..s
        };
        if self.last != Some(key) {
            if self.lines < PSTATE_LOG_LINES {
                self.log(dev, what, &s, cap);
            } else if self.lines == PSTATE_LOG_LINES {
                dev_info!(dev, "M3: further performance state changes are not logged\n");
            }
            self.lines = self.lines.saturating_add(1);
            self.last = Some(key);
        }
        let peak = [s.actual, s.current]
            .into_iter()
            .filter(|v| *v <= self.policy.table_max)
            .max()
            .unwrap_or(0);
        Ok((over, peak))
    }
}
