// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! The initdata graph (49 objects at canonical VAs) and its live words.
//!
//! The build allocates the objects, channels and IOMappings, writes the board images and the
//! root, runtime and control pointers, and patches HardwareData and Globals (TTBAT PA at
//! HardwareData +0xb44, IOMapping VAs, the timestamp arena at +0x28, the tuned idle policy,
//! Globals +0x980 = 100, HardwareDataA +0x4580 words). The object classes follow
//! [`super::hw::tuning`]: HardwareData pages 0-1 ASC-cached, the fault-information page and
//! everything else uncached; channel rings NEVER cached (caching them hangs the firmware).
//!
//! Live words used after READY: Globals (+0x1c/+0x24 kill divisors, +0x2c/+0x30/+0x7a8/
//! +0x7c4/+0x9a0 idle policy, +0x9b0 compute monitor), Control (+0x40a0 pending, +0x4580 halt
//! count / PB epoch, +0x4590 halted, +0x45a0 resume), HardwareData +0x8900 fault information.
//! Globals changes are re-imported with DEVCTL 0x13. All objects here are firmware-only
//! (AttrIndex 2) unless noted; offsets are relative to each object's canonical VA.

use kernel::{device, prelude::*, sync::Arc};

use super::adt::GpuAdt;
use super::board::{self, data};
use super::boot::glue;
use super::channel::Channels;
use super::fw::initdata::{
    cbuf, control, flags, globals, hwdata, pair, power, region_a, root, runtime, small,
};
use super::hw::{self, va};
use super::identity::Hex;
use super::object::{Class, FwObject, Objects, Placement, Request};
use super::Config;

/// The built initdata graph.
pub(crate) struct InitData {
    /// Globals, Control and HardwareData: the live words of recovery, power and faults.
    pub(crate) globals: Option<Arc<FwObject>>,
    pub(crate) control: Option<Arc<FwObject>>,
    pub(crate) hwdata: Option<Arc<FwObject>>,
    /// Runtime (readiness) and Flags.
    pub(crate) runtime: Option<Arc<FwObject>>,
    pub(crate) flags: Option<Arc<FwObject>>,
    /// C1 (firmware + GPU shared), written by the PB (re)initialization.
    pub(crate) c1: Option<Arc<FwObject>>,
    /// The rest of the graph, retained (root, region A, pair, small, C0, C1, C3, power).
    others: KVec<Arc<FwObject>>,
}

impl InitData {
    pub(crate) fn new() -> Self {
        Self {
            globals: None,
            control: None,
            hwdata: None,
            runtime: None,
            flags: None,
            c1: None,
            others: KVec::new(),
        }
    }

    /// Control +0x4590: the firmware halted for recovery.
    pub(crate) fn halted(&self) -> Result<bool> {
        Ok(self.control.as_ref().ok_or(ENODEV)?.r32(control::HALTED)? == 1)
    }

    /// The readiness words: Runtime +0x314 and Flags +4.
    pub(crate) fn readiness(&self) -> Result<(u32, u32)> {
        Ok((
            self.runtime.as_ref().ok_or(ENODEV)?.r32(runtime::READY)?,
            self.flags.as_ref().ok_or(ENODEV)?.r32(flags::READY)?,
        ))
    }
}

impl super::Firmware {
    /// Writes the 16-byte C1 record into the initdata C1 object.
    pub(crate) fn write_c1(&mut self, record: &[u8; 16]) -> Result {
        self.init.c1.as_ref().ok_or(ENODEV)?.write(0, record)
    }
}

/// A canonical firmware-only object at `va`.
fn canonical(objects: &mut Objects, at: u64, size: usize, class: Class) -> Result<Arc<FwObject>> {
    objects.alloc(Request {
        size,
        class,
        placement: Placement::At(at),
        offset: 0,
        client: None,
        lower: None,
        cpu: true,
    })
}

fn get32(b: &[u8], offset: usize) -> Result<u32> {
    Ok(u32::from_le_bytes(
        b.get(offset..offset + 4)
            .ok_or(EINVAL)?
            .try_into()
            .map_err(|_| EINVAL)?,
    ))
}

fn put32(b: &mut [u8], offset: usize, value: u32) -> Result {
    b.get_mut(offset..offset + 4)
        .ok_or(EINVAL)?
        .copy_from_slice(&value.to_le_bytes());
    Ok(())
}

fn put64(b: &mut [u8], offset: usize, value: u64) -> Result {
    b.get_mut(offset..offset + 8)
        .ok_or(EINVAL)?
        .copy_from_slice(&value.to_le_bytes());
    Ok(())
}


/// The T6030 board images: the verified templates with DVFS from the ADT. Returns the images
/// and the top DVFS frequency in kHz.
fn templates(
    dev: &device::Device,
    adt: &GpuAdt,
) -> Result<(KVVec<u8>, KVVec<u8>, KVVec<u8>, u32)> {
    let mut hw_img = KVVec::from_elem(0u8, hwdata::SIZE, GFP_KERNEL)?;
    let mut glob = KVVec::from_elem(0u8, globals::SIZE, GFP_KERNEL)?;
    let mut pwr = KVVec::from_elem(0u8, power::SIZE, GFP_KERNEL)?;
    if board::expand(&data::HARDWARE, &mut hw_img).is_err()
        || board::expand(&data::GLOBALS, &mut glob).is_err()
        || board::expand(&data::POWER, &mut pwr).is_err()
    {
        dev_err!(dev, "board data template hash mismatch\n");
        return Err(EINVAL);
    }
    if get32(&hw_img, hwdata::CHECK_A38)? != 1
        || get32(&hw_img, hwdata::DVFS_COUNT)? != board::DVFS_STATES as u32 - 1
        || get32(&glob, globals::UNK_980)? != 10
        || get32(&glob, globals::IDLE_STANDBY_US)? != 700
        || get32(&glob, globals::IDLE_MIN_CONF)? != 0x3f333333
    {
        return Err(EINVAL);
    }
    let max_khz = board::from_adt(dev, adt, &mut hw_img)?;
    let reference = glue::sha256(&hw_img) == data::HARDWARE.sha256;
    dev_info!(
        dev,
        "board data: ADT performance states, {} MHz top{}\n",
        max_khz / 1000,
        if reference {
            " (reference machine)"
        } else {
            " (differs from the reference machine)"
        }
    );
    if !reference {
        dev_info!(dev, "board data sha256 {}\n", Hex(&glue::sha256(&hw_img)));
    }
    Ok((hw_img, glob, pwr, max_khz))
}

/// HardwareDataA word holding the system counter at initdata creation.
const HWDATA_A_INIT_TIMESTAMP: usize = 0x424c;

/// Board images of a device-tree SoC: the hardware record ([`board::hardware_record`]) from the
/// GPU node, the ADT's sensor rows and the SoC's `reference` words, Globals and Power
/// ([`board::device_tree`]) from the GPU node and that record. Returns the images and the top
/// operating-point frequency in kHz.
fn device_tree_board(
    dev: &device::Device,
    cfg: &Config,
    adt: &GpuAdt,
    reference: &board::record::Reference,
) -> Result<(KVVec<u8>, KVVec<u8>, KVVec<u8>, u32)> {
    let hw_img = board::hardware_record(dev, cfg, adt, reference)?;
    let (glob, pwr, max_power_mw) = board::device_tree(dev, &hw_img)?;
    let max_mhz = adt.perf_mhz.iter().copied().max().unwrap_or(0);
    dev_info!(
        dev,
        "board data: {} images from the device tree and the ADT sensor rows, reference maximum power {} mW and MTR masks, {} MHz top operating point\n",
        cfg.name,
        max_power_mw,
        max_mhz
    );
    dev_info!(dev, "board data sha256 {}\n", Hex(&glue::sha256(&hw_img)));
    let max_khz = max_mhz.checked_mul(1000).ok_or_else(|| {
        dev_err!(dev, "board data: top operating point {} MHz does not fit in kHz\n", max_mhz);
        EINVAL
    })?;
    Ok((hw_img, glob, pwr, max_khz))
}

/// Verified board bytes, prepared before any GPU firmware is started.
pub(crate) type BoardImages = (KVVec<u8>, KVVec<u8>, KVVec<u8>, u32);

/// Build and check the board images before any GPU firmware is started: the templates against
/// their digests and layout, or the device-tree record from the GPU node, the ADT and the SoC's
/// reference words.
pub(crate) fn prepare_board(
    dev: &device::Device,
    cfg: &Config,
    adt: &GpuAdt,
) -> Result<BoardImages> {
    match cfg.board {
        hw::Board::Templates => templates(dev, adt),
        hw::Board::DeviceTree(reference) => device_tree_board(dev, cfg, adt, reference),
        hw::Board::None => Err(ENODEV),
    }
}

/// Builds the initdata graph. Returns it with its channels and the top DVFS frequency in kHz.
/// Nothing is published here.
pub(crate) fn build(
    dev: &device::Device,
    cfg: &Config,
    adt: &GpuAdt,
    objects: &mut Objects,
    board_images: BoardImages,
) -> Result<(InitData, Channels, u32)> {
    let mut init = InitData::new();
    let fw = Class::Fw;
    let layout = cfg.layout;
    let hwdata_va = layout.hwdata_va;

    // Owned objects and channels.
    let root_obj = canonical(objects, va::ROOT, root::SIZE, fw)?;
    let region_a_obj = canonical(objects, va::REGION_A, region_a::SIZE, fw)?;
    let runtime_obj = canonical(objects, va::RUNTIME, runtime::SIZE, fw)?;
    // Activity records cached; fault info and HardwareDataA timestamps not (T6030), or the
    // whole record cached (a layout without a cached prefix).
    let cached = if hw::tuning::HWDATA_CACHED {
        Class::FwCached
    } else {
        fw
    };
    let hwdata_obj = match layout.hwdata_cached {
        Some(prefix) => objects.alloc_split(
            Request {
                size: layout.hwdata_size,
                class: fw,
                placement: Placement::At(hwdata_va),
                offset: 0,
                client: None,
                lower: None,
                cpu: true,
            },
            Some(((hwdata_va as usize & (hw::PAGE as usize - 1)) + prefix, cached)),
        )?,
        None => canonical(objects, hwdata_va, layout.hwdata_size, cached)?,
    };
    let pair_obj = canonical(objects, va::PAIR, pair::SIZE, fw)?;
    let small_obj = canonical(objects, va::SMALL, small::SIZE, fw)?;
    let c0 = canonical(objects, va::C0, cbuf::C0_SIZE, Class::Shared)?;
    let c1 = canonical(objects, va::C1, cbuf::C1_SIZE, Class::Shared)?;
    let c3 = canonical(objects, va::C3, cbuf::C3_SIZE, Class::Shared)?;
    let globals_obj = canonical(objects, va::GLOBALS, globals::SIZE, fw)?;
    let power_obj = canonical(objects, va::POWER, power::SIZE, fw)?;
    let control_obj = canonical(objects, va::CONTROL, control::SIZE, fw)?;
    let flags_obj = canonical(objects, va::FLAGS, flags::SIZE, fw)?;
    let channels = Channels::alloc(objects)?;
    // Firmware Device RW aliases for the IOMapping table.
    for map in layout.iomaps.iter() {
        objects.map_mmio(map.va, map.pa, map.map_bytes)?;
    }

    // Board images, verified, then the owned replacements.
    let (mut hw_img, mut glob, pwr, max_khz) = board_images;
    if matches!(cfg.board, hw::Board::DeviceTree(_)) {
        // This timestamp belongs to initdata creation, not board-data preflight.
        let now: u64;
        // SAFETY: Reading the architectural counter has no side effects.
        unsafe {
            core::arch::asm!("mrs {}, cntpct_el0", out(reg) now, options(nomem, nostack, preserves_flags))
        };
        put64(&mut hw_img, hwdata::HWDATA_A + HWDATA_A_INIT_TIMESTAMP, now)?;
    }
    if hw_img.len() != layout.hwdata_size {
        return Err(EINVAL);
    }
    put64(&mut hw_img, hwdata::TTBAT, adt.ttbs.start)?;
    for map in layout.iomaps.iter() {
        put64(
            &mut hw_img,
            hwdata::IOMAPS + hwdata::IOMAP_STRIDE * map.slot as usize + hwdata::IOMAP_VA,
            map.va + map.delta,
        )?;
    }
    put64(&mut hw_img, hwdata::TIMESTAMPS, va::TIMESTAMPS)?;
    put32(&mut glob, globals::UNK_980, 100)?;
    // The device tree's standby time (a device-tree SoC; the templates' machine
    // has none).
    let dt_standby = match cfg.board {
        hw::Board::DeviceTree(_) => {
            let prop = board::words::PROPS[board::words::STANDBY_TIMER];
            match dev
                .of_node()
                .ok_or(ENODEV)
                .and_then(|n| n.get_property::<u32>(prop))
            {
                Ok(v) => Some(v),
                Err(_) => {
                    dev_warn!(
                        dev,
                        "GPU node has no u32 {}, using the tuned idle standby\n",
                        prop
                    );
                    None
                }
            }
        }
        _ => None,
    };
    let (standby_us, standby_src, dt_rejected) = super::power::standby(
        super::params::idle_standby_override_us(),
        dt_standby,
        hw::tuning::IDLE_STANDBY_US,
    );
    if dt_rejected {
        dev_warn!(
            dev,
            "apple,idleoff-standby-timer is outside 100..100000 us, ignored\n"
        );
    }
    dev_info!(
        dev,
        "idle standby {} us from {}\n",
        standby_us,
        standby_src.name()
    );
    put32(&mut glob, globals::IDLE_STANDBY_US, standby_us)?;
    put32(
        &mut glob,
        globals::IDLE_MIN_CONF,
        super::power::permille_f32(super::params::idle_min_conf()),
    )?;
    put32(
        &mut glob,
        globals::IDLE_PREDICTIVE,
        hw::tuning::IDLE_PREDICTIVE as u32,
    )?;
    // HardwareDataA at +0x4580: maintained S8 request (the templates' machine; a record built
    // from the device tree carries its own state words).
    if cfg.board == hw::Board::Templates {
        for (offset, value) in [
            (0x2c, 8),
            (0x30, 8),
            (0x44, 800),
            (0x7d8, 800),
            (0x7e0, 800),
            (0x81c, 800),
        ] {
            put32(&mut hw_img, hwdata::HWDATA_A + offset, value)?;
        }
    }
    hwdata_obj.write(0, &hw_img)?;
    globals_obj.write(0, &glob)?;
    power_obj.write(0, &pwr)?;

    // Root.
    let r = &root_obj;
    r.w64(0x00, layout.root_magic)?;
    r.w64(root::REGION_A, va::REGION_A)?;
    r.w64(root::RUNTIME, va::RUNTIME)?;
    r.w64(root::GLOBALS, va::GLOBALS)?;
    r.w32(root::ENABLE, 1)?;
    r.write(root::GEOMETRY, &[0x00, 0x40, 14, 3])?;
    const SHIFTS: [u8; 3] = [36, 25, 14];
    const ENTRIES: [u16; 3] = [64, 2048, 2048];
    for i in 0..3 {
        let at = root::LEVELS + root::LEVEL_STRIDE * i;
        let e = ENTRIES[i].to_le_bytes();
        r.write(at, &[8, 14, 14, SHIFTS[i], e[0], e[1], 0x00, 0x40])?;
        r.w64(at + 8, 1)?;
        r.w64(at + 0x10, 0x3ffffffc000)?;
        r.w64(at + 0x18, ((ENTRIES[i] - 1) as u64) << SHIFTS[i])?;
    }
    r.w64(root::FLAGS, va::FLAGS)?;
    r.w64(root::CONTROL, va::CONTROL)?;
    r.w64(root::POWER, va::POWER)?;

    // Runtime pointers.
    let rt = &runtime_obj;
    rt.w64(runtime::HWDATA, hwdata_va)?;
    rt.w64(runtime::PAIR, va::PAIR)?;
    rt.w64(runtime::PAIR2, va::PAIR)?;
    for i in 0..13u64 {
        let (state, ring) = if i < 12 {
            (va::ch_state(i), va::ch_ring(i))
        } else {
            (va::DEVCTL_STATE, va::DEVCTL_RING)
        };
        let at = runtime::CHANNELS + runtime::CHANNEL_STRIDE * i as usize;
        rt.w64(at, state)?;
        rt.w64(at + 8, state + 0x10)?;
        rt.w64(at + 0x10, state + 0x20)?;
        rt.w64(at + 0x18, ring)?;
    }
    let rx = [
        (va::EVENT_STATE, va::EVENT_RING),
        (va::LOG_STATE, va::LOG_RING),
        (va::TRACE_STATE, va::TRACE_RING),
        (va::STATS_STATE, va::STATS_RING),
    ];
    for (i, (state, ring)) in rx.iter().enumerate() {
        rt.w64(runtime::RX + runtime::RX_STRIDE * i, *state)?;
        rt.w64(runtime::RX + 8 + runtime::RX_STRIDE * i, *ring)?;
    }
    rt.w64(runtime::SMALL, va::SMALL)?;
    rt.w64(
        runtime::TA_ACTIVITY,
        hwdata_va + hwdata::TA_ACTIVITY as u64,
    )?;
    rt.w64(
        runtime::FS_ACTIVITY,
        hwdata_va + hwdata::FS_ACTIVITY as u64,
    )?;
    rt.w64(
        runtime::COMPUTE_ACTIVITY,
        hwdata_va + hwdata::COMPUTE_ACTIVITY as u64,
    )?;
    rt.w64(runtime::FAULT_INFO, hwdata_va + layout.hwdata_view as u64)?;
    rt.w64(runtime::C0, va::C0)?;
    rt.w64(runtime::C1, va::C1)?;
    rt.w64(runtime::C1_2, va::C1)?;
    rt.w64(runtime::C3, va::C3)?;
    rt.w64(runtime::C3_2, va::C3)?;
    rt.w32(runtime::UNK_2D0, 4)?;
    rt.w32(runtime::UNK_3B0, 0xff)?;
    rt.w64(runtime::HWDATA_A, hwdata_va + hwdata::HWDATA_A as u64)?;

    // Control and flags.
    let c = &control_obj;
    c.w32(control::ENABLE, 1)?;
    c.w64(control::FWCTL_STATE, va::FWCTL_STATE)?;
    c.w64(control::FWCTL_RING, va::FWCTL_RING)?;
    c.w32(control::ENABLE2, 1)?;

    init.c1 = Some(c1);
    for o in [
        root_obj,
        region_a_obj,
        pair_obj,
        small_obj,
        c0,
        c3,
        power_obj,
    ] {
        init.others.push(o, GFP_KERNEL)?;
    }
    init.globals = Some(globals_obj);
    init.control = Some(control_obj);
    init.hwdata = Some(hwdata_obj);
    init.runtime = Some(runtime_obj);
    init.flags = Some(flags_obj);
    Ok((init, channels, max_khz))
}
