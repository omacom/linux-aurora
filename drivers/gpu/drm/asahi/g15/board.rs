// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Board-data templates of the supported firmware build.
//!
//! Sparse 16-byte-line images (HardwareData 35,332 B, Globals 15,868 B, Power 2,292 B), each
//! with its SHA-256: [`expand`] expands and verifies them, [`from_adt`] rebuilds DVFS tables A/B
//! from the ADT performance states (`DVFS_A` {0,1,2,3,5,7,9,11,13}, `DVFS_B`
//! {0,1,2,3,4,6,8,10,12}; MHz at +0xb5c, +0xf9c, +0x1350, core mV +0xb9c, SRAM mV +0xd9c, top
//! MHz +0x4dc4). The tables in [`data`] are generated, and every expanded image is checked
//! against its hash at boot.
//!
//! A SoC whose board data comes from its device tree ([`super::hw::Platform::DeviceTree`]) gets
//! its hardware record from [`hardware_record`] ([`record`]: the GPU node, the ADT's sensor
//! rows and the SoC's reference words) and its Globals and Power from [`device_tree`]: the G15S
//! templates with the GPU node's power words and the hardware record's maximum power
//! ([`words`]).

pub(crate) mod data;
pub(crate) mod record;
pub(crate) mod template;
pub(crate) mod words;

use kernel::{c_str, device, prelude::*};

use super::adt::{self, GpuAdt};
use super::boot::glue;
use super::fw::initdata::{globals, hwdata, power};
use super::hw::{self, va};
pub(crate) use template::{Line, Template};

static_assert!(words::GLOBALS_SIZE == globals::SIZE);
static_assert!(words::POWER_SIZE == power::SIZE);
static_assert!(words::HWDATA_A == hwdata::HWDATA_A);
static_assert!(words::HWDATA_A_MAX_POWER == hwdata::MAX_POWER_A);
static_assert!(words::TOP_STATE as usize == DVFS_STATES - 1);
static_assert!(record::HWDATA_A == hwdata::HWDATA_A);
static_assert!(record::MAX_POWER == hwdata::HWDATA_A + hwdata::MAX_POWER_A);
static_assert!(record::IOMAPS == hwdata::IOMAPS);
static_assert!(record::PERF_STATES == adt::PERF_STATES);
static_assert!(record::CLUSTERS == adt::CLUSTERS);

/// Expands a template into `out` and verifies its hash.
pub(crate) fn expand(template: &Template, out: &mut [u8]) -> Result {
    template::fill(template, out).map_err(|_| EINVAL)?;
    if glue::sha256(&out[..template.size]) != template.sha256 {
        return Err(EINVAL);
    }
    Ok(())
}

/// The GPU node's power words ([`words::PROPS`] and [`words::ZONES`]).
fn power_words(dev: &device::Device) -> Result<words::PowerWords> {
    let node = dev.of_node().ok_or(ENODEV)?;
    let mut values = [0u32; words::PROPS.len()];
    for (value, name) in values.iter_mut().zip(words::PROPS.iter()) {
        *value = node
            .get_property(name)
            .inspect_err(|_| dev_err!(dev, "GPU node has no u32 {}\n", name))?;
    }
    let zones: KVec<u32> = node.get_opt_property(words::ZONES)?.unwrap_or_default();
    words::PowerWords::new(values, &zones).map_err(|why| {
        dev_err!(dev, "GPU node power words: {}\n", why);
        EINVAL
    })
}

/// A CS or AFR operating-point table of the GPU node (`name`): MHz and one voltage cell per
/// state, at most `max` states.
fn aux_opps(
    dev: &device::Device,
    node: &kernel::of::Node,
    name: &CStr,
    out: &mut [record::Opp],
) -> Result<usize> {
    let table = node
        .parse_phandle(name, 0)
        .ok_or(EINVAL)
        .inspect_err(|_| dev_err!(dev, "GPU node has no {}\n", name))?;
    let mut n = 0;
    for opp in table.children() {
        let hz: u64 = opp.get_property(c_str!("opp-hz"))?;
        let uv: KVec<u32> = opp.get_property(c_str!("opp-microvolt"))?;
        if uv.len() != 1 || n >= out.len() {
            dev_err!(
                dev,
                "GPU {} table: state {} has {} cells\n",
                name,
                n,
                uv.len()
            );
            return Err(EINVAL);
        }
        let mhz = u32::try_from(hz / 1_000_000).map_err(|_| {
            dev_err!(dev, "GPU {} table: state {} is {} Hz\n", name, n, hz);
            EINVAL
        })?;
        out[n] = record::Opp {
            mhz,
            mv: [uv[0].div_ceil(1000), 0, 0, 0],
            cells: 1,
        };
        n += 1;
    }
    if n == 0 {
        dev_err!(dev, "GPU {} table is empty\n", name);
        return Err(EINVAL);
    }
    Ok(n)
}

/// The hardware record of a device-tree SoC ([`record`]): built from the GPU node's properties
/// and operating-point tables, the layout's IOMapping table, the ADT regions and sensor rows,
/// and the SoC's `reference` words, which neither carries.
pub(crate) fn hardware_record(
    dev: &device::Device,
    cfg: &super::Config,
    adt: &GpuAdt,
    reference: &record::Reference,
) -> Result<KVVec<u8>> {
    let layout = cfg.layout;
    let node = dev.of_node().ok_or(ENODEV)?;
    let mut props = [0u32; record::PROPS.len()];
    for (value, (name, default)) in props.iter_mut().zip(record::PROPS.iter()) {
        *value = match node.get_opt_property(name)? {
            Some(v) => v,
            None => default
                .ok_or(EINVAL)
                .inspect_err(|_| dev_err!(dev, "GPU node has no u32 {}\n", name))?,
        };
    }
    let zones: KVec<u32> = node.get_opt_property(words::ZONES)?.unwrap_or_default();
    let mut main = [record::Opp::default(); record::PERF_STATES];
    for (i, o) in main.iter_mut().enumerate() {
        *o = record::Opp {
            mhz: adt.perf_mhz[i],
            mv: adt.cluster_mv[i],
            cells: adt.cluster_cells,
        };
    }
    let mut cs = [record::Opp::default(); 16];
    let mut afr = [record::Opp::default(); 8];
    let ncs = aux_opps(dev, &node, c_str!("apple,cs-opp"), &mut cs)?;
    let nafr = aux_opps(dev, &node, c_str!("apple,afr-opp"), &mut afr)?;
    if layout.record_slots.len() != layout.iomaps.len() {
        return Err(EINVAL);
    }
    let mut slots = [record::IoSlot::default(); record::IOMAP_SLOTS];
    for (k, (map, &(total, element, writable))) in
        layout.iomaps.iter().zip(layout.record_slots).enumerate()
    {
        slots[k] = record::IoSlot {
            slot: map.slot,
            pa: map.pa + map.delta,
            va: map.va + map.delta,
            total,
            element,
            writable,
        };
    }
    let inputs = record::Inputs {
        chip_id: cfg.chip_id,
        props,
        zones: &zones,
        main: &main,
        cs: &cs[..ncs],
        afr: &afr[..nafr],
        slots: &slots[..layout.iomaps.len()],
        empty_writable: layout.record_empty_writable,
        ttbat: adt.ttbs.start,
        usc_base: hw::client::COMPACT_WINDOW.start,
        user_end: hw::USER_RANGE.end,
        timestamps: va::TIMESTAMPS,
        sensor_rows: adt.sensor_rows,
        reference,
    };
    let mut out = KVVec::from_elem(0u8, record::SIZE, GFP_KERNEL)?;
    record::build(&mut out, &inputs).map_err(|why| {
        dev_err!(dev, "hardware record from the device tree: {}\n", why);
        EINVAL
    })?;
    Ok(out)
}

/// Globals and Power of a device-tree SoC: the verified G15S templates with the GPU node's
/// power words and the maximum power of `hardware`, this machine's hardware record. Returns the
/// two images and that maximum power in mW.
pub(crate) fn device_tree(
    dev: &device::Device,
    hardware: &[u8],
) -> Result<(KVVec<u8>, KVVec<u8>, u32)> {
    let w = power_words(dev)?;
    let at = hwdata::HWDATA_A + hwdata::MAX_POWER_A;
    let max_power_mw = u32::from_le_bytes(
        hardware
            .get(at..at + 4)
            .ok_or(EINVAL)?
            .try_into()
            .map_err(|_| EINVAL)?,
    );
    let mut glob = KVVec::from_elem(0u8, globals::SIZE, GFP_KERNEL)?;
    let mut pwr = KVVec::from_elem(0u8, power::SIZE, GFP_KERNEL)?;
    if expand(&data::GLOBALS, &mut glob).is_err() || expand(&data::POWER, &mut pwr).is_err() {
        dev_err!(dev, "board data template hash mismatch\n");
        return Err(EINVAL);
    }
    words::globals(&mut glob, max_power_mw, &w)
        .and_then(|()| words::power(&mut pwr, max_power_mw, &w))
        .map_err(|why| {
            dev_err!(dev, "board data from the device tree: {}\n", why);
            EINVAL
        })?;
    Ok((glob, pwr, max_power_mw))
}

/// DVFS states of the firmware tables.
pub(crate) const DVFS_STATES: usize = 9;
const DVFS_A: [usize; DVFS_STATES] = [0, 1, 2, 3, 5, 7, 9, 11, 13];
const DVFS_B: [usize; DVFS_STATES] = [0, 1, 2, 3, 4, 6, 8, 10, 12];

/// Table A's frequency of each firmware DVFS state, MHz: the ADT (or device-tree) operating
/// points the firmware's state indices name.
pub(crate) fn table_a_mhz(adt: &GpuAdt) -> [u32; DVFS_STATES] {
    DVFS_A.map(|i| adt.perf_mhz[i])
}

fn put32(hw: &mut [u8], offset: usize, value: u32) -> Result {
    hw.get_mut(offset..offset + 4)
        .ok_or(EINVAL)?
        .copy_from_slice(&value.to_le_bytes());
    Ok(())
}

/// The machine-specific part of HardwareData, rebuilt from this machine's ADT performance
/// states over the template: the firmware's two DVFS tables take the three shared low states,
/// then per voltage level the higher (table A) and the lower (table B) frequency of the ADT
/// pair. Table A carries the core and SRAM voltages (eight identical lanes per state) and its top
/// frequency. With the reference machine's ADT this reproduces the template exactly. Returns the
/// top frequency in kHz.
pub(crate) fn from_adt(dev: &device::Device, adt: &GpuAdt, hw: &mut [u8]) -> Result<u32> {
    for s in 1..DVFS_STATES {
        let (a, pa) = (DVFS_A[s], DVFS_A[s - 1]);
        let (b, pb) = (DVFS_B[s], DVFS_B[s - 1]);
        if adt.perf_mhz[a] <= adt.perf_mhz[pa]
            || adt.perf_mhz[b] <= adt.perf_mhz[pb]
            || adt.perf_mv[a] < adt.perf_mv[pa]
            || adt.sram_mv[a] < adt.sram_mv[pa]
        {
            dev_err!(dev, "ADT perf-states do not form ordered DVFS tables\n");
            return Err(EINVAL);
        }
    }
    for s in 0..DVFS_STATES {
        let (a, b) = (DVFS_A[s], DVFS_B[s]);
        put32(hw, 0xb5c + 4 * s, adt.perf_mhz[a])?;
        put32(hw, 0xf9c + 4 * s, adt.perf_mhz[b])?;
        put32(hw, 0x1350 + 4 * s, adt.perf_mhz[b])?;
        for k in 0..8 {
            put32(hw, 0xb9c + 0x20 * s + 4 * k, adt.perf_mv[a])?;
            put32(hw, 0xd9c + 0x20 * s + 4 * k, adt.sram_mv[a])?;
        }
    }
    let top = adt.perf_mhz[DVFS_A[DVFS_STATES - 1]];
    put32(hw, 0x4dc4, top)?;
    Ok(top * 1000)
}
