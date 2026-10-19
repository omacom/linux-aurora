// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(clippy::unusual_byte_groupings)]

//! GPU initialization data builder.
//!
//! The root of all interaction between the GPU firmware and the host driver is a complex set of
//! nested structures that we call InitData. This includes both GPU hardware/firmware configuration
//! and the pointers to the ring buffers and global data fields that are used for communication at
//! runtime.
//!
//! Many of these structures are poorly understood, so there are lots of hardcoded unknown values
//! derived from observing the InitData structures that macOS generates.

use crate::f32;
use crate::fw::initdata::*;
use crate::fw::types::*;
use crate::module_parameters;
use crate::{
    driver::AsahiDevice,
    gem,
    gpu,
    hw,
    mmu, //
};
use kernel::error::{
    Error,
    Result, //
};
use kernel::macros::versions;
use kernel::prelude::*;
use kernel::try_init;

use ::pin_init;
use ::pin_init::Init;

/// Builder helper for the global GPU InitData.

/// G15 idle-off delay (ms, Globals +0x9a0/+0x9a4) used as "never idle off" while the rings live
/// in Fender scratch SRAM without a backup/restore path.
/// TODO: the firmware's encoding of "never" is unknown. i32::MAX stays positive if the
/// firmware treats the value as signed, and is never smaller than u32::MAX would be if it
/// converts ms to 24 MHz ticks in 32-bit arithmetic (both wrap to ~179 s); with 64-bit
/// arithmetic it is ~24 days.
#[allow(dead_code)]
pub(crate) const G15_KEEP_POWERED_MS: u32 = i32::MAX as u32;

/// Which backend a G15 InitData is built for. The two differ where the GpuManager layout keeps
/// the values the former bring-up backend was validated with on J516S.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub(crate) enum G15Layout {
    /// The GpuManager's layout: rings in Fender SRAM, the GPU never idles off.
    Manager,
    /// The serialized M3 runtime backend: rings in DRAM with finite idle-off, its own GPU and
    /// firmware VA layout, and the values its InitData carries for the power, idle and
    /// shader-engine controllers.
    Runtime,
}

/// G15 InitData choices that are not part of the hardware or device-tree configuration.
#[derive(Copy, Clone, Debug)]
pub(crate) struct G15Options {
    /// The backend the InitData is built for.
    pub(crate) layout: G15Layout,
    /// Publish the performance states as two voltage-sorted tables (see [`PStateTables`]).
    pub(crate) split_pstates: bool,
    /// Highest performance state the firmware may use, as an index into the voltage-sorted
    /// primary table (1 = lowest). None publishes the whole table.
    pub(crate) cap: Option<u64>,
    /// Firmware VA of the timestamp arena (HwDataB+0x28); None uses the manager's kernel range.
    pub(crate) timestamp_base: Option<u64>,
    /// Manager layout: write the PPM words and the Globals power targets as the runtime layout
    /// does. The runtime layout always does.
    pub(crate) reference_ppm: bool,
    /// Runtime layout: the SoC's HwDataB configuration words and unit masks
    /// (`m3_soc::Soc::hwdata_b`). None for the manager layout, which does not write them.
    pub(crate) runtime_hwdata_b: Option<&'static G15RuntimeHwDataB>,
    /// Accept operating points whose clusters have different voltages
    /// (`m3_soc::Soc::per_cluster_voltages`); see [`PStateTables`].
    pub(crate) per_cluster_voltages: bool,
}

/// The HwDataB words of the runtime backend's InitData that are neither its GPU VA layout nor
/// device-tree values: configuration words where the manager layout keeps 0 or its validated
/// values (including the A/B pair +0xa98/+0xb40 at 0), and the two unit masks. They are values
/// of one SoC's runtime InitData (`m3_soc::Soc::hwdata_b`), not derived from the device tree.
#[derive(Debug)]
pub(crate) struct G15RuntimeHwDataB {
    pub(crate) unk_454: u32,
    pub(crate) unk_464: u32,
    pub(crate) unk_a7c: u64,
    pub(crate) unk_a98: u32,
    pub(crate) unk_abc: u64,
    pub(crate) unk_ae4: u32,
    pub(crate) unk_b20: u32,
    pub(crate) unk_b24: u64,
    /// +0xb40, the "UAT enabled" word of the manager layout.
    pub(crate) unk_554: u32,
    pub(crate) unk_17b8: u32,
    /// Unit enable masks, +0x17c0/+0x17c8.
    pub(crate) unit_mask_a: u64,
    pub(crate) unit_mask_b: u32,
    pub(crate) unk_1808: u32,
    pub(crate) unk_1818: u32,
}

impl G15Options {
    /// The GpuManager's options: the validated defaults and the performance-state cap
    /// (`asahi.m3_max_pstate`).
    fn manager(cfg: &'static hw::HwConfig) -> G15Options {
        let g15 = cfg.gpu_gen == hw::GpuGen::G15;
        G15Options {
            layout: G15Layout::Manager,
            split_pstates: false,
            cap: g15.then(|| {
                crate::m3_params::max_pstate_param().unwrap_or(crate::m3_params::MAX_PSTATE_DEFAULT)
            }),
            timestamp_base: None,
            reference_ppm: false,
            runtime_hwdata_b: None,
            per_cluster_voltages: false,
        }
    }

    /// The published performance-state tables for `pwr`.
    pub(crate) fn tables(&self, pwr: &hw::PwrConfig) -> Result<PStateTables> {
        PStateTables::new(pwr, self.split_pstates, self.cap, self.per_cluster_voltages)
    }

    /// RuntimePointers+0x2d0, a configuration word: 4 in the runtime backend's InitData, 0 (its
    /// validated value) for the manager.
    pub(crate) fn runtime_pointers_2d0(&self) -> u32 {
        match self.layout {
            G15Layout::Manager => 0,
            G15Layout::Runtime => 4,
        }
    }
}

/// Start of the USC (shader code) window in the runtime backend's GPU VA layout.
const RUNTIME_USC_BASE: u64 = 0x10_0000_0000;
/// The runtime backend's unknown-page address, the end of its 42-bit user VA range.
const RUNTIME_UNKNOWN_PAGE: u64 = 0x2ff_ffff_8000;

/// Maximum number of entries of a firmware performance-state table, including the off state.
const PSTATE_TABLE_ENTRIES: usize = 16;

/// The performance-state tables as published to the G15 firmware.
///
/// The device tree lists the performance states in the bootloader's order: several states can
/// share one voltage, and the frequencies are not sorted. The firmware takes two tables sorted
/// by voltage: the primary one with the highest frequency of each voltage, the secondary one with
/// the lowest, plus the device-tree index of every entry. Entry 0 is the off state in both.
/// Every frequency and voltage comes from the device tree; nothing is interpolated.
///
/// With per-cluster voltages (T6031, whose clusters have their own binned voltage tables), the
/// states are grouped by their whole voltage list instead: states that share their highest
/// cluster voltage must have the same voltage on every cluster, the groups are ordered by that
/// highest voltage, and every cluster's voltage must rise from one entry to the next.
#[derive(Copy, Clone, Debug)]
pub(crate) struct PStateTables {
    /// Number of entries, including the off state.
    pub(crate) len: usize,
    /// Device-tree index of each primary-table entry.
    pub(crate) primary: [u8; PSTATE_TABLE_ENTRIES],
    /// Device-tree index of each secondary-table entry (split tables only).
    pub(crate) secondary: Option<[u8; PSTATE_TABLE_ENTRIES]>,
    /// Highest primary-table entry the firmware may select.
    pub(crate) max: u32,
}

impl PStateTables {
    /// The primary/secondary split of `states`: one entry per distinct voltage, ascending.
    fn split(
        states: &[hw::PState],
    ) -> Result<([u8; PSTATE_TABLE_ENTRIES], [u8; PSTATE_TABLE_ENTRIES], usize)> {
        Self::split_for(states, false)
    }

    /// [`Self::split`], accepting per-cluster voltages when `per_cluster` is set.
    fn split_for(
        states: &[hw::PState],
        per_cluster: bool,
    ) -> Result<([u8; PSTATE_TABLE_ENTRIES], [u8; PSTATE_TABLE_ENTRIES], usize)> {
        let mut primary = [0u8; PSTATE_TABLE_ENTRIES];
        let mut secondary = [0u8; PSTATE_TABLE_ENTRIES];
        if states.len() < 2 || states.len() > u8::MAX as usize || states[0].freq_hz != 0 {
            return Err(EINVAL);
        }
        let mut len = 1;
        let mut last_mv = 0;
        loop {
            // The next voltage above `last_mv`, and its fastest and slowest states.
            let mut next: Option<(u32, usize, usize)> = None;
            for (i, ps) in states.iter().enumerate().skip(1) {
                let mv = ps.max_volt_mv();
                // Every state after the off state runs, with one voltage for all clusters
                // unless the SoC gives each cluster its own.
                if ps.freq_hz == 0 || (!per_cluster && ps.volt_mv.iter().any(|v| *v != mv)) {
                    return Err(EINVAL);
                }
                if mv <= last_mv {
                    continue;
                }
                // States of one entry run every cluster at the same voltages.
                if let (true, Some((best, hi, _))) = (per_cluster, next) {
                    if mv == best && states[hi].volt_mv[..] != ps.volt_mv[..] {
                        return Err(EINVAL);
                    }
                }
                next = match next {
                    Some((best, hi, lo)) if mv == best => Some((
                        mv,
                        if ps.freq_hz > states[hi].freq_hz { i } else { hi },
                        if ps.freq_hz < states[lo].freq_hz { i } else { lo },
                    )),
                    Some((best, _, _)) if mv > best => next,
                    _ => Some((mv, i, i)),
                };
            }
            let Some((mv, hi, lo)) = next else {
                break;
            };
            if len >= PSTATE_TABLE_ENTRIES
                || states[hi].freq_hz <= states[primary[len - 1] as usize].freq_hz
                || states[lo].freq_hz <= states[secondary[len - 1] as usize].freq_hz
            {
                // Faster states must need a higher voltage in both tables.
                return Err(EINVAL);
            }
            // With per-cluster voltages, every cluster's voltage rises with the entry.
            if per_cluster && len > 1 {
                let previous = &states[primary[len - 1] as usize].volt_mv;
                let current = &states[hi].volt_mv;
                if previous.len() != current.len()
                    || previous.iter().zip(current.iter()).any(|(p, c)| c <= p)
                {
                    return Err(EINVAL);
                }
            }
            primary[len] = hi as u8;
            secondary[len] = lo as u8;
            len += 1;
            last_mv = mv;
        }
        // At least one running state must have a nonzero voltage; callers
        // index the table up to len - 1 and clamp caps to 1..=len - 1.
        if len < 2 {
            return Err(EINVAL);
        }
        Ok((primary, secondary, len))
    }

    /// Build the published tables. `cap` is an index into the voltage-sorted primary table,
    /// also when the states are published in device-tree order: the firmware may then use any
    /// device-tree state up to the capped frequency, and the cap is translated to the highest
    /// device-tree index whose states all stay at or below it.
    /// `per_cluster` accepts per-cluster voltages (see [`PStateTables`]).
    pub(crate) fn new(
        pwr: &hw::PwrConfig,
        split: bool,
        cap: Option<u64>,
        per_cluster: bool,
    ) -> Result<Self> {
        let states = &pwr.perf_states;
        if split {
            let (primary, secondary, len) = Self::split_for(states, per_cluster)?;
            let top = len as u64 - 1;
            let max = cap.map_or(top, |c| c.clamp(1, top)) as u32;
            return Ok(PStateTables {
                len,
                primary,
                secondary: Some(secondary),
                max,
            });
        }
        if states.len() > PSTATE_TABLE_ENTRIES {
            return Err(EINVAL);
        }
        let mut primary = [0u8; PSTATE_TABLE_ENTRIES];
        for (i, slot) in primary.iter_mut().enumerate().take(states.len()) {
            *slot = i as u8;
        }
        let top = states.len() as u32 - 1;
        let max = match cap {
            // No voltage-sorted table can have more entries than the device tree has states,
            // so such a cap leaves the whole table.
            None => top,
            Some(c) if c >= u64::from(top) => top,
            Some(c) => {
                let (sorted, _, sorted_len) = Self::split(states)?;
                let limit_hz = states[sorted[c.clamp(1, sorted_len as u64 - 1) as usize] as usize].freq_hz;
                // The longest device-tree prefix that stays at or below the capped frequency.
                let mut max = 1;
                while max < top && states[max as usize + 1].freq_hz <= limit_hz {
                    max += 1;
                }
                max
            }
        };
        Ok(PStateTables {
            len: states.len(),
            primary,
            secondary: None,
            max,
        })
    }

    /// The device-tree state of primary-table entry `i`.
    pub(crate) fn primary_state<'s>(&self, pwr: &'s hw::PwrConfig, i: usize) -> &'s hw::PState {
        &pwr.perf_states[self.primary[i] as usize]
    }
}

#[versions(AGX)]
pub(crate) struct InitDataBuilder<'a> {
    dev: &'a AsahiDevice,
    alloc: &'a mut gpu::KernelAllocators,
    cfg: &'static hw::HwConfig,
    dyncfg: &'a hw::DynConfig,
    g15: G15Options,
}

#[versions(AGX)]
impl<'a> InitDataBuilder::ver<'a> {
    /// Create a new InitData builder
    pub(crate) fn new(
        dev: &'a AsahiDevice,
        alloc: &'a mut gpu::KernelAllocators,
        cfg: &'static hw::HwConfig,
        dyncfg: &'a hw::DynConfig,
    ) -> InitDataBuilder::ver<'a> {
        InitDataBuilder::ver {
            dev,
            alloc,
            cfg,
            dyncfg,
            g15: G15Options::manager(cfg),
        }
    }

    // Not used on G15: the host does not write hws1/2/3 there.
    /// Create the HwDataShared1 structure, which is used in two places in InitData.
    #[ver(V < V14_8_3)]
    fn hw_shared1(cfg: &'static hw::HwConfig) -> impl Init<raw::HwDataShared1> {
        init!(raw::HwDataShared1 {
            unk_a4: cfg.shared1_a4,
            ..Zeroable::init_zeroed()
        })
        .chain(|ret| {
            for (i, val) in cfg.shared1_tab.iter().enumerate() {
                ret.table[i] = *val;
            }
            Ok(())
        })
    }

    // Not used on G15: the host does not write hws1/2/3 there.
    #[ver(V < V14_8_3)]
    fn init_curve(
        curve: &mut raw::HwDataShared2Curve,
        unk_0: u32,
        unk_4: u32,
        t1: &[u16],
        t2: &[i16],
        t3: &[KVec<i32>],
    ) {
        curve.unk_0 = unk_0;
        curve.unk_4 = unk_4;
        (*curve.t1)[..t1.len()].copy_from_slice(t1);
        (*curve.t1)[t1.len()..].fill(t1[0]);
        (*curve.t2)[..t2.len()].copy_from_slice(t2);
        (*curve.t2)[t2.len()..].fill(t2[0]);
        for (i, a) in curve.t3.iter_mut().enumerate() {
            a.fill(0x3ffffff);
            if i < t3.len() {
                let b = &t3[i];
                (**a)[..b.len()].copy_from_slice(b);
            }
        }
    }

    // Not used on G15: the host does not write hws1/2/3 there.
    /// Create the HwDataShared2 structure, which is used in two places in InitData.
    #[ver(V < V14_8_3)]
    fn hw_shared2(
        cfg: &'static hw::HwConfig,
        dyncfg: &'a hw::DynConfig,
    ) -> impl Init<raw::HwDataShared2, Error> + 'a {
        try_init!(raw::HwDataShared2 {
            unk_28: Array::new([0xff; 16]),
            g14: Default::default(),
            unk_508: cfg.shared2_unk_508,
            ..Zeroable::init_zeroed()
        })
        .chain(|ret| {
            for (i, val) in cfg.shared2_tab.iter().enumerate() {
                ret.table[i] = *val;
            }

            let curve_cfg = match cfg.shared2_curves.as_ref() {
                None => return Ok(()),
                Some(a) => a,
            };

            let mut t1 = KVec::new();
            let mut t3 = KVec::new();

            for _ in 0..curve_cfg.t3_scales.len() {
                t3.push(KVec::new(), GFP_KERNEL)?;
            }

            for (i, ps) in dyncfg.pwr.perf_states.iter().enumerate() {
                let t3_coef = curve_cfg.t3_coefs[i];
                if t3_coef == 0 {
                    t1.push(0xffff, GFP_KERNEL)?;
                    for j in t3.iter_mut() {
                        j.push(0x3ffffff, GFP_KERNEL)?;
                    }
                    continue;
                }

                let f_khz = (ps.freq_hz / 1000) as u64;
                let v_max = ps.max_volt_mv() as u64;

                t1.push(
                    (1000000000 * (curve_cfg.t1_coef as u64) / (f_khz * v_max))
                        .try_into()
                        .unwrap(),
                    GFP_KERNEL,
                )?;

                for (j, scale) in curve_cfg.t3_scales.iter().enumerate() {
                    t3[j].push(
                        (t3_coef as u64 * 1000000100 * *scale as u64 / (f_khz * v_max * 6))
                            .try_into()
                            .unwrap(),
                        GFP_KERNEL,
                    )?;
                }
            }

            ret.g14.unk_14 = 0x6000000;
            Self::init_curve(
                &mut ret.g14.curve1,
                0,
                0x20000000,
                &[0xffff],
                &[0x0f07],
                &[],
            );
            Self::init_curve(&mut ret.g14.curve2, 7, 0x80000000, &t1, curve_cfg.t2, &t3);

            Ok(())
        })
    }

    // Not used on G15: the host does not write hws1/2/3 there.
    /// Create the HwDataShared3 structure, which is used in two places in InitData.
    #[ver(V < V14_8_3)]
    fn hw_shared3(cfg: &'static hw::HwConfig) -> impl Init<raw::HwDataShared3> {
        pin_init::init_zeroed::<raw::HwDataShared3>().chain(|ret| {
            if !cfg.shared3_tab.is_empty() {
                ret.unk_0 = 1;
                ret.unk_4 = 500;
                ret.unk_8 = cfg.shared3_unk;
                ret.table.copy_from_slice(cfg.shared3_tab);
                ret.unk_4c = 1;
            }
            Ok(())
        })
    }

    /// Create an unknown T81xx-specific data structure.
    fn t81xx_data(
        cfg: &'static hw::HwConfig,
        dyncfg: &'a hw::DynConfig,
    ) -> impl Init<raw::T81xxData> {
        let _perf_max_pstate = dyncfg.pwr.perf_max_pstate;

        pin_init::init_zeroed::<raw::T81xxData>().chain(move |_ret| {
            match cfg.chip_id {
                0x8103 | 0x8112 => {
                    #[ver(V < V13_3)]
                    {
                        _ret.unk_d8c = 0x80000000;
                        _ret.unk_d90 = 4;
                        _ret.unk_d9c = f32!(0.6);
                        _ret.unk_da4 = f32!(0.4);
                        _ret.unk_dac = f32!(0.38552);
                        _ret.unk_db8 = f32!(65536.0);
                        _ret.unk_dbc = f32!(13.56);
                        _ret.max_pstate_scaled = 100 * _perf_max_pstate;
                    }
                }
                _ => (),
            }
            Ok(())
        })
    }

    /// Build the HwDataA contents (mostly power-related configuration) without allocating the
    /// object, so that they can fill a firmware object or a plain buffer alike.
    pub(crate) fn hwdata_a_init(
        cfg: &'static hw::HwConfig,
        dyncfg: &'a hw::DynConfig,
        g15: &G15Options,
    ) -> Result<impl Init<raw::HwDataA::ver, Error> + 'a> {
        let pwr = &dyncfg.pwr;
        let period_ms = pwr.power_sample_period;
        let period_s = F32::from(period_ms) / f32!(1000.0);
        let ppm_filter_tc_periods = pwr.ppm_filter_time_constant_ms / period_ms;
        #[ver(V >= V13_0B4)]
        let ppm_filter_tc_ms_rounded = ppm_filter_tc_periods * period_ms;
        let ppm_filter_a = f32!(1.0) / ppm_filter_tc_periods.into();
        let perf_filter_a = f32!(1.0) / pwr.perf_filter_time_constant.into();
        let perf_filter_a2 = f32!(1.0) / pwr.perf_filter_time_constant2.into();
        let avg_power_target_filter_a = f32!(1.0) / pwr.avg_power_target_filter_tc.into();
        let avg_power_filter_tc_periods = pwr.avg_power_filter_tc_ms / period_ms;
        #[ver(V >= V13_0B4)]
        let avg_power_filter_tc_ms_rounded = avg_power_filter_tc_periods * period_ms;
        let avg_power_filter_a = f32!(1.0) / avg_power_filter_tc_periods.into();
        let pwr_filter_a = f32!(1.0) / pwr.pwr_filter_time_constant.into();

        let base_ps = pwr.perf_base_pstate;
        let base_ps_scaled = 100 * base_ps;
        // G15 publishes its own performance-state tables (split and capped).
        #[ver(V < V14_8_3)]
        let max_ps = pwr.perf_max_pstate;
        #[ver(V < V14_8_3)]
        let _ = g15;
        #[ver(V >= V14_8_3)]
        let tables = g15.tables(pwr)?;
        #[ver(V >= V14_8_3)]
        let max_ps = tables.max;
        #[ver(V >= V14_8_3)]
        let runtime = g15.layout == G15Layout::Runtime;
        let max_ps_scaled = 100 * max_ps;
        let boost_ps_count = if cfg.gpu_gen == hw::GpuGen::G15 {
            max_ps.checked_sub(base_ps).ok_or(EINVAL)?
        } else { max_ps - base_ps };

        #[allow(unused_variables)]
        let base_clock_khz = cfg.base_clock_hz / 1000;
        let v_clocks_per_period = pwr.pwr_sample_period_aic_clks;

        #[allow(unused_variables)]
        let clocks_per_period_coarse = cfg.base_clock_hz / 1000 * pwr.power_sample_period;

        Ok(
                try_init!(raw::HwDataA::ver {
                    clocks_per_period: v_clocks_per_period,
                    #[ver(V >= V13_0B4)]
                    clocks_per_period_2: v_clocks_per_period,
                    pwr_status: AtomicU32::new(4),
                    unk_10: f32!(1.0),
                    actual_pstate: 1,
                    tgt_pstate: 1,
                    base_pstate_scaled: base_ps_scaled,
                    unk_40: 1,
                    max_pstate_scaled: max_ps_scaled,
                    min_pstate_scaled: 100,
                    unk_64c: 625,
                    pwr_filter_a_neg: f32!(1.0) - pwr_filter_a,
                    pwr_filter_a: pwr_filter_a,
                    pwr_integral_gain: pwr.pwr_integral_gain,
                    pwr_integral_min_clamp: pwr.pwr_integral_min_clamp.into(),
                    max_power_1: pwr.max_power_mw.into(),
                    pwr_proportional_gain: pwr.pwr_proportional_gain,
                    pwr_pstate_related_k: -F32::from(max_ps_scaled) / pwr.max_power_mw.into(),
                    pwr_pstate_max_dc_offset: pwr.pwr_min_duty_cycle as i32 - max_ps_scaled as i32,
                    max_pstate_scaled_2: max_ps_scaled,
                    max_power_2: pwr.max_power_mw,
                    max_pstate_scaled_3: max_ps_scaled,
                    ppm_filter_tc_periods_x4: ppm_filter_tc_periods * 4,
                    ppm_filter_a_neg: f32!(1.0) - ppm_filter_a,
                    ppm_filter_a: ppm_filter_a,
                    ppm_ki_dt: pwr.ppm_ki * period_s,
                    unk_6fc: f32!(65536.0),
                    ppm_kp: pwr.ppm_kp,
                    pwr_min_duty_cycle: pwr.pwr_min_duty_cycle,
                    max_pstate_scaled_4: max_ps_scaled,
                    unk_71c: f32!(0.0),
                    max_power_3: pwr.max_power_mw,
                    cur_power_mw_2: 0x0,
                    ppm_filter_tc_ms: pwr.ppm_filter_time_constant_ms,
                    #[ver(V >= V13_0B4)]
                    ppm_filter_tc_clks: ppm_filter_tc_ms_rounded * base_clock_khz,
                    perf_tgt_utilization: pwr.perf_tgt_utilization,
                    perf_boost_min_util: pwr.perf_boost_min_util,
                    perf_boost_ce_step: pwr.perf_boost_ce_step,
                    perf_reset_iters: pwr.perf_reset_iters,
                    unk_774: 6,
                    unk_778: 1,
                    perf_filter_drop_threshold: pwr.perf_filter_drop_threshold,
                    perf_filter_a_neg: f32!(1.0) - perf_filter_a,
                    perf_filter_a2_neg: f32!(1.0) - perf_filter_a2,
                    perf_filter_a: perf_filter_a,
                    perf_filter_a2: perf_filter_a2,
                    perf_ki: pwr.perf_integral_gain,
                    perf_ki2: pwr.perf_integral_gain2,
                    perf_integral_min_clamp: pwr.perf_integral_min_clamp.into(),
                    unk_79c: f32!(95.0),
                    perf_kp: pwr.perf_proportional_gain,
                    perf_kp2: pwr.perf_proportional_gain2,
                    boost_state_unk_k: F32::from(boost_ps_count) / f32!(0.95),
                    base_pstate_scaled_2: base_ps_scaled,
                    max_pstate_scaled_5: max_ps_scaled,
                    base_pstate_scaled_3: base_ps_scaled,
                    perf_tgt_utilization_2: pwr.perf_tgt_utilization,
                    base_pstate_scaled_4: base_ps_scaled,
                    unk_7fc: f32!(65536.0),
                    pwr_min_duty_cycle_2: pwr.pwr_min_duty_cycle.into(),
                    max_pstate_scaled_6: max_ps_scaled.into(),
                    max_freq_mhz: pwr.max_freq_mhz,
                    pwr_min_duty_cycle_3: pwr.pwr_min_duty_cycle,
                    min_pstate_scaled_4: f32!(100.0),
                    max_pstate_scaled_7: max_ps_scaled,
                    unk_alpha_neg: f32!(0.8),
                    unk_alpha: f32!(0.2),
                    fast_die0_sensor_mask: U64(cfg.fast_sensor_mask[0]),
                    #[ver(G >= G14X && G < G15)]
                    fast_die1_sensor_mask: U64(cfg.fast_sensor_mask[1]),
                    fast_die0_release_temp_cc: 100 * pwr.fast_die0_release_temp,
                    unk_87c: cfg.da.unk_87c,
                    unk_880: 0x4,
                    unk_894: f32!(1.0),

                    fast_die0_ki_dt: pwr.fast_die0_integral_gain * period_s,
                    unk_8a8: f32!(65536.0),
                    fast_die0_kp: pwr.fast_die0_proportional_gain,
                    pwr_min_duty_cycle_4: pwr.pwr_min_duty_cycle,
                    max_pstate_scaled_8: max_ps_scaled,
                    max_pstate_scaled_9: max_ps_scaled,
                    fast_die0_prop_tgt_delta: 100 * pwr.fast_die0_prop_tgt_delta,
                    unk_8cc: cfg.da.unk_8cc,
                    max_pstate_scaled_10: max_ps_scaled,
                    max_pstate_scaled_11: max_ps_scaled,
                    unk_c2c: 1,
                    power_zone_count: pwr.power_zones.len() as u32,
                    max_power_4: pwr.max_power_mw,
                    max_power_5: pwr.max_power_mw,
                    max_power_6: pwr.max_power_mw,
                    avg_power_target_filter_a_neg: f32!(1.0) - avg_power_target_filter_a,
                    avg_power_target_filter_a: avg_power_target_filter_a,
                    avg_power_target_filter_tc_x4: 4 * pwr.avg_power_target_filter_tc,
                    avg_power_target_filter_tc_xperiod: period_ms * pwr.avg_power_target_filter_tc,
                    #[ver(V >= V13_0B4)]
                    avg_power_target_filter_tc_clks: period_ms
                        * pwr.avg_power_target_filter_tc
                        * base_clock_khz,
                    avg_power_filter_tc_periods_x4: 4 * avg_power_filter_tc_periods,
                    avg_power_filter_a_neg: f32!(1.0) - avg_power_filter_a,
                    avg_power_filter_a: avg_power_filter_a,
                    avg_power_ki_dt: pwr.avg_power_ki_only * period_s,
                    unk_d20: f32!(65536.0),
                    avg_power_kp: pwr.avg_power_kp,
                    avg_power_min_duty_cycle: pwr.avg_power_min_duty_cycle,
                    max_pstate_scaled_12: max_ps_scaled,
                    max_pstate_scaled_13: max_ps_scaled,
                    max_power_7: pwr.max_power_mw.into(),
                    max_power_8: pwr.max_power_mw,
                    avg_power_filter_tc_ms: pwr.avg_power_filter_tc_ms,
                    #[ver(V >= V13_0B4)]
                    avg_power_filter_tc_clks: avg_power_filter_tc_ms_rounded * base_clock_khz,
                    max_pstate_scaled_14: max_ps_scaled,
                    t81xx_data <- Self::t81xx_data(cfg, dyncfg),
                    // G15 (14.x) uses a different layout for HwDataA 0x10b4..0x126b: the words
                    // where the G13 values below would land must be zero, and board-dependent
                    // power/performance values live elsewhere in the block (0x10e8..0x11e4).
                    // Leave the block zeroed on V >= V14_8_3; the two known constants are written
                    // in chain() below.
                    #[ver(V >= V13_0B4 && V < V14_8_3)]
                    unk_e10_0 <- {
                        let filter_a = f32!(1.0) / pwr.se_filter_time_constant.into();
                        let filter_1_a = f32!(1.0) / pwr.se_filter_time_constant_1.into();
                        try_init!(raw::HwDataA130Extra {
                            unk_38: 4,
                            unk_3c: 8000,
                            gpu_se_inactive_threshold: pwr.se_inactive_threshold,
                            gpu_se_engagement_criteria: pwr.se_engagement_criteria,
                            gpu_se_reset_criteria: pwr.se_reset_criteria,
                            unk_54: 50,
                            unk_58: 0x1,
                            gpu_se_filter_a_neg: f32!(1.0) - filter_a,
                            gpu_se_filter_1_a_neg: f32!(1.0) - filter_1_a,
                            gpu_se_filter_a: filter_a,
                            gpu_se_filter_1_a: filter_1_a,
                            gpu_se_ki_dt: pwr.se_ki * period_s,
                            gpu_se_ki_1_dt: pwr.se_ki_1 * period_s,
                            unk_7c: f32!(65536.0),
                            gpu_se_kp: pwr.se_kp,
                            gpu_se_kp_1: pwr.se_kp_1,

                            #[ver(V >= V13_3)]
                            unk_8c: 100,
                            #[ver(V < V13_3)]
                            unk_8c: 40,

                            max_pstate_scaled_1: max_ps_scaled,
                            unk_9c: f32!(8000.0),
                            unk_a0: 1400,
                            gpu_se_filter_time_constant_ms: pwr.se_filter_time_constant * period_ms,
                            gpu_se_filter_time_constant_1_ms: pwr.se_filter_time_constant_1
                                * period_ms,
                            gpu_se_filter_time_constant_clks: U64((pwr.se_filter_time_constant
                                * clocks_per_period_coarse)
                                .into()),
                            gpu_se_filter_time_constant_1_clks: U64((pwr
                                .se_filter_time_constant_1
                                * clocks_per_period_coarse)
                                .into()),
                            unk_c4: f32!(65536.0),
                            unk_114: f32!(65536.0),
                            unk_124: 40,
                            max_pstate_scaled_2: max_ps_scaled,
                            ..Zeroable::init_zeroed()
                        })
                    },
                    fast_die0_sensor_mask_2: U64(cfg.fast_sensor_mask[0]),
                    #[ver(G >= G14X && G < G15)]
                    fast_die1_sensor_mask_2: U64(cfg.fast_sensor_mask[1]),
                    unk_e24: cfg.da.unk_e24,
                    unk_e28: 1,
                    fast_die0_sensor_mask_alt: U64(cfg.fast_sensor_mask_alt[0]),
                    #[ver(G >= G14X && G < G15)]
                    fast_die1_sensor_mask_alt: U64(cfg.fast_sensor_mask_alt[1]),
                    #[ver(V < V13_0B4)]
                    fast_die0_sensor_present: U64(cfg.fast_die0_sensor_present as u64),
                    unk_163c: 1,
                    unk_3644: 0,
                    // On G15 the host writes neither hws1/2/3 nor 0x3ce8; the only host-written
                    // words in HwDataA 0x3aac..0x41bb are +0x3aa4 and the DPE leakage config at
                    // +0x4188..+0x41dc.
                    #[ver(V < V14_8_3)]
                    hws1 <- Self::hw_shared1(cfg),
                    #[ver(V < V14_8_3)]
                    hws2 <- Self::hw_shared2(cfg, dyncfg),
                    #[ver(V < V14_8_3)]
                    hws3 <- Self::hw_shared3(cfg),
                    #[ver(V < V14_8_3)]
                    unk_3ce8: 1,
                    ..Zeroable::init_zeroed()
                })
                .chain(move |raw| {
                    #[ver(V < V14_8_3)]
                    for i in 0..dyncfg.pwr.perf_states.len() {
                        raw.sram_k[i] = cfg.sram_k;
                    }
                    #[ver(V >= V14_8_3)]
                    for i in 0..tables.len {
                        raw.sram_k[i] = cfg.sram_k;
                    }

                    // On G15 the G13 leak-coefficient arrays overlap the DPE leakage config
                    // (HwDataA +0x4188), so they stay zero.
                    // TODO: write leakage data in the 0x4188 layout (mirrored at P1
                    // +0x158..0x1ab) once its source is understood.
                    #[ver(V < V14_8_3)]
                    for (i, coef) in pwr.core_leak_coef.iter().enumerate() {
                        raw.core_leak_coef[i] = *coef;
                    }

                    #[ver(V < V14_8_3)]
                    for (i, coef) in pwr.sram_leak_coef.iter().enumerate() {
                        raw.sram_leak_coef[i] = *coef;
                    }

                    // The only constants the G15 firmware expects in the HwDataA 0x10b4 block
                    // (+0x10f4 = 0x32, +0x11cc = {0, 65536.0}).
                    // unk_e10_0 sits at 0x10b4 (asserted in fw/initdata.rs), so +0x40 is 0x10f4
                    // and +0x11c is 0x11d0. The runtime layout fills the whole block below.
                    #[ver(V >= V14_8_3)]
                    {
                        raw.unk_e10_0.unk_40 = 0x32;
                        raw.unk_e10_0.unk_11c = f32!(65536.0);
                    }

                    #[ver(V >= V13_0B4)]
                    if let Some(csafr) = pwr.csafr.as_ref() {
                        for (i, coef) in csafr.leak_coef_afr.iter().enumerate() {
                            raw.aux_leak_coef.cs_1[i] = *coef;
                            raw.aux_leak_coef.cs_2[i] = *coef;
                        }

                        for (i, coef) in csafr.leak_coef_cs.iter().enumerate() {
                            raw.aux_leak_coef.afr_1[i] = *coef;
                            raw.aux_leak_coef.afr_2[i] = *coef;
                        }
                    }

                    for i in 0..dyncfg.id.num_clusters as usize {
                        if let Some(coef_a) = cfg.unk_coef_a.get(i) {
                            (*raw.unk_coef_a1[i])[..coef_a.len()].copy_from_slice(coef_a);
                            (*raw.unk_coef_a2[i])[..coef_a.len()].copy_from_slice(coef_a);
                        }
                        if let Some(coef_b) = cfg.unk_coef_b.get(i) {
                            (*raw.unk_coef_b1[i])[..coef_b.len()].copy_from_slice(coef_b);
                            (*raw.unk_coef_b2[i])[..coef_b.len()].copy_from_slice(coef_b);
                        }
                    }

                    for (i, pz) in pwr.power_zones.iter().enumerate() {
                        raw.power_zones[i].target = pz.target;
                        raw.power_zones[i].target_off = pz.target - pz.target_offset;
                        raw.power_zones[i].filter_tc_x4 = 4 * pz.filter_tc;
                        raw.power_zones[i].filter_tc_xperiod = period_ms * pz.filter_tc;
                        let filter_a = f32!(1.0) / pz.filter_tc.into();
                        raw.power_zones[i].filter_a = filter_a;
                        raw.power_zones[i].filter_a_neg = f32!(1.0) - filter_a;
                        #[ver(V >= V13_0B4)]
                        raw.power_zones[i].unk_10 = 1320000000;
                    }

                    #[ver(V >= V13_0B4 && G >= G14X)]
                    for (i, j) in raw.hws2.g14.curve2.t1.iter().enumerate() {
                        raw.unk_hws2[i] = if *j == 0xffff { 0 } else { j / 2 };
                    }

                    #[ver(V >= V14_8_3)]
                    if runtime {
                        Self::hwdata_a_runtime(raw, pwr, max_ps_scaled, clocks_per_period_coarse);
                    }

                    Ok(())
                }))
    }

    /// Create the HwDataA structure. This mostly contains power-related configuration.
    fn hwdata_a(&mut self) -> Result<GpuObject<HwDataA::ver>> {
        let init = Self::hwdata_a_init(self.cfg, self.dyncfg, &self.g15)?;
        let dev = self.dev;
        let dyncfg = self.dyncfg;
        self.alloc
            .private
            .new_init(pin_init::init_zeroed(), move |_inner, _ptr| {
                init.chain(move |raw| {
                    if !dyncfg.hw_data_b.is_empty() {
                        unsafe {
                            let mut matches: bool = true;
                            let sla = core::slice::from_raw_parts(
                                raw as *const raw::HwDataA::ver as *const u8,
                                core::mem::size_of::<raw::HwDataA::ver>(),
                            );
                            if sla.len() != dyncfg.hw_data_a.len() {
                                matches = false;
                                dev_err!(
                                    dev.as_ref(),
                                    "!!! Hwdata A size mismatch: {} {}",
                                    sla.len(),
                                    dyncfg.hw_data_a.len(),
                                );
                            }
                            for i in 0..core::cmp::min(sla.len(), dyncfg.hw_data_a.len()) {
                                if sla[i] != dyncfg.hw_data_a[i] {
                                    matches = false;
                                    dev_err!(dev.as_ref(), "!!! Hwdata A first mismatch: {i}");
                                    break;
                                }
                            }
                            if matches {
                                dev_info!(dev.as_ref(), "!!! Hwdata A match");
                            }
                        }
                    }
                    Ok(())
                })
            })
    }

    /// The runtime backend's HwDataA words that the manager layout leaves at their defaults: the
    /// shader-engine controller block at +0x10b4 in its G15 order (all inputs from the device
    /// tree), and the leakage coefficient and flag words of the G15 tail. Words that the
    /// runtime's InitData keeps at zero are cleared.
    #[ver(V >= V14_8_3)]
    fn hwdata_a_runtime(
        raw: &mut raw::HwDataA::ver,
        pwr: &hw::PwrConfig,
        max_ps_scaled: u32,
        clocks_per_period_coarse: u32,
    ) {
        let period_ms = pwr.power_sample_period;
        let period_s = F32::from(period_ms) / f32!(1000.0);
        let filter_a = f32!(1.0) / pwr.se_filter_time_constant.into();
        let filter_1_a = f32!(1.0) / pwr.se_filter_time_constant_1.into();
        let se = &mut raw.unk_e10_0;
        se.unk_40 = 50;
        se.unk_44 = 1;
        se.gpu_se_filter_a_neg = f32!(1.0) - filter_a;
        se.gpu_se_filter_1_a_neg = f32!(1.0) - filter_1_a;
        se.gpu_se_filter_a = filter_a;
        se.gpu_se_filter_1_a = filter_1_a;
        se.gpu_se_ki_dt = pwr.se_ki * period_s;
        se.gpu_se_ki_1_dt = pwr.se_ki_1 * period_s;
        se.unk_68 = f32!(65536.0);
        se.gpu_se_kp = pwr.se_kp;
        se.gpu_se_kp_1 = pwr.se_kp_1;
        se.unk_78 = 100;
        se.max_pstate_scaled_1 = max_ps_scaled;
        se.min_pstate_scaled = 100;
        se.unk_88 = f32!(8000.0);
        // The shader-engine target (ADT gpu-se-tgt, 1400 on J514S and J516S; the device tree
        // may override it).
        se.se_target = pwr.se_target;
        se.gpu_se_filter_time_constant_ms = pwr.se_filter_time_constant * period_ms;
        se.gpu_se_filter_time_constant_1_ms = pwr.se_filter_time_constant_1 * period_ms;
        se.gpu_se_filter_time_constant_clks =
            U64((pwr.se_filter_time_constant * clocks_per_period_coarse).into());
        se.gpu_se_filter_time_constant_1_clks =
            U64((pwr.se_filter_time_constant_1 * clocks_per_period_coarse).into());
        se.unk_b0 = f32!(65536.0);
        se.unk_d8 = 8000;
        se.unk_dc = 4;
        se.gpu_se_inactive_threshold = pwr.se_inactive_threshold;
        se.gpu_se_engagement_criteria = pwr.se_engagement_criteria;
        se.unk_e8 = 2;
        se.unk_ec = 4;
        se.gpu_se_reset_criteria = pwr.se_reset_criteria;
        se.unk_11c = f32!(65536.0);
        se.unk_12c = 40;
        se.max_pstate_scaled_2 = max_ps_scaled;

        // No fast-die release temperature, no +0x8b8 offset and no alternate sensor mask: the
        // T6030 device tree has no source for them.
        raw.fast_die0_release_temp_cc = 0;
        raw.unk_87c = 0;
        raw.fast_die0_sensor_mask_alt = U64(0);
        raw.unk_3640 = f32!(5.0).to_bits();
        // The fused core leakage coefficient of this machine (device tree), first cluster table.
        if let Some(coef) = pwr.core_leak_coef.first() {
            raw.cluster_tables[0][0] = coef.to_bits();
        }
        raw.unk_4298 = 1;
        raw.unk_429c[0] = 1;
    }

    /// Create the HwDataB structure. This mostly contains GPU-related configuration.
    #[ver(V < V14_8_3)]
    fn hwdata_b(&mut self) -> Result<GpuObject<HwDataB::ver>> {
        self.alloc
            .private
            .new_init(pin_init::init_zeroed(), |_inner, _ptr| {
                let cfg = &self.cfg;
                let dyncfg = &self.dyncfg;
                try_init!(raw::HwDataB::ver {
                    // Userspace VA map related
                    #[ver(V < V13_0B4)]
                    unk_0: U64(0x13_00000000),
                    unk_8: U64(0x14_00000000),
                    #[ver(V < V13_0B4)]
                    unk_10: U64(0x1_00000000),
                    unk_18: U64(0xffc00000),
                    // USC start
                    unk_20: U64(0), // U64(0x11_00000000),
                    unk_28: U64(0), // U64(0x11_00000000),
                    // Unknown page
                    //unk_30: U64(0x6f_ffff8000),
                    unk_30: U64(mmu::IOVA_UNK_PAGE),
                    timestamp_area_base: U64(mmu::kern_iova(
                        cfg,
                        gpu::IOVA_KERN_TIMESTAMP_RANGE.start
                    )),
                    // TODO: yuv matrices
                    chip_id: cfg.chip_id,
                    unk_454: cfg.db.unk_454,
                    unk_458: 0x1,
                    unk_460: 0x1,
                    unk_464: 0x1,
                    unk_468: 0x1,
                    unk_47c: 0x1,
                    unk_484: 0x1,
                    unk_48c: 0x1,
                    base_clock_khz: cfg.base_clock_hz / 1000,
                    power_sample_period: dyncfg.pwr.power_sample_period,
                    unk_49c: 0x1,
                    unk_4a0: 0x1,
                    unk_4a4: 0x1,
                    unk_4c0: 0x1f,
                    unk_4e0: U64(cfg.db.unk_4e0),
                    unk_4f0: 0x1,
                    unk_4f4: 0x1,
                    unk_504: 0x31,
                    unk_524: 0x1, // use_secure_cache_flush
                    unk_534: cfg.db.unk_534,
                    num_frags: dyncfg.id.num_frags * dyncfg.id.num_clusters,
                    unk_554: 0x1,
                    uat_ttb_base: U64(dyncfg.uat_ttb_base),
                    // Firmware-ABI core id; absent (AGX3) configs must never
                    // reach this builder, and fail closed here if they do.
                    gpu_core_id: cfg.gpu_core.ok_or(ENODEV)? as u32,
                    gpu_rev_id: dyncfg.id.gpu_rev_id as u32,
                    num_cores: dyncfg.id.num_cores * dyncfg.id.num_clusters,
                    max_pstate: dyncfg.pwr.perf_states.len() as u32 - 1,
                    #[ver(V < V13_0B4)]
                    num_pstates: dyncfg.pwr.perf_states.len() as u32,
                    #[ver(V < V13_0B4)]
                    min_sram_volt: dyncfg.pwr.min_sram_microvolt / 1000,
                    #[ver(V < V13_0B4)]
                    unk_ab8: cfg.db.unk_ab8,
                    #[ver(V < V13_0B4)]
                    unk_abc: cfg.db.unk_abc,
                    #[ver(V < V13_0B4)]
                    unk_ac0: 0x1020,

                    #[ver(V >= V13_0B4)]
                    unk_ae4: Array::new([0x0, 0x3, 0x7, 0x7]),
                    #[ver(V < V13_0B4)]
                    unk_ae4: Array::new([0x0, 0xf, 0x3f, 0x3f]),
                    unk_b10: 0x1,
                    timer_offset: U64(0),
                    unk_b24: 0x1,
                    unk_b28: 0x1,
                    unk_b2c: 0x1,
                    unk_b30: cfg.db.unk_b30,
                    #[ver(V >= V13_0B4)]
                    unk_b38_0: 1,
                    #[ver(V >= V13_0B4)]
                    unk_b38_4: 1,
                    unk_b38: Array::new([0xffffffff; 12]),
                    #[ver(V >= V13_0B4 && V < V13_3)]
                    unk_c3c: 0x19,
                    #[ver(V >= V13_3)]
                    unk_c3c: 0x1a,
                    ..Zeroable::init_zeroed()
                })
                .chain(|raw| {
                    #[ver(V >= V13_3)]
                    for i in 0..16 {
                        raw.unk_arr_0[i] = i as u32;
                    }

                    let base_ps = self.dyncfg.pwr.perf_base_pstate as usize;
                    let max_ps = self.dyncfg.pwr.perf_max_pstate as usize;
                    let base_freq = self.dyncfg.pwr.perf_states[base_ps].freq_hz;
                    let max_freq = self.dyncfg.pwr.perf_states[max_ps].freq_hz;

                    for (i, ps) in self.dyncfg.pwr.perf_states.iter().enumerate() {
                        raw.frequencies[i] = ps.freq_hz / 1000000;
                        for (j, mv) in ps.volt_mv.iter().enumerate() {
                            let sram_mv = (*mv).max(self.dyncfg.pwr.min_sram_microvolt / 1000);
                            raw.voltages[i][j] = *mv;
                            raw.voltages_sram[i][j] = sram_mv;
                        }
                        for j in ps.volt_mv.len()..raw.voltages[i].len() {
                            raw.voltages[i][j] = raw.voltages[i][0];
                            raw.voltages_sram[i][j] = raw.voltages_sram[i][0];
                        }
                        raw.sram_k[i] = self.cfg.sram_k;
                        raw.rel_max_powers[i] = ps.pwr_mw * 100 / self.dyncfg.pwr.max_power_mw;
                        raw.rel_boost_freqs[i] = if i > base_ps {
                            (ps.freq_hz - base_freq) / ((max_freq - base_freq) / 100)
                        } else {
                            0
                        };
                    }

                    #[ver(V >= V13_0B4)]
                    if let Some(csafr) = self.dyncfg.pwr.csafr.as_ref() {
                        let aux = &mut raw.aux_ps;
                        aux.cs_max_pstate = (csafr.perf_states_cs.len() - 1).try_into()?;
                        aux.afr_max_pstate = (csafr.perf_states_afr.len() - 1).try_into()?;

                        for (i, ps) in csafr.perf_states_cs.iter().enumerate() {
                            aux.cs_frequencies[i] = ps.freq_hz / 1000000;
                            for (j, mv) in ps.volt_mv.iter().enumerate() {
                                let sram_mv = (*mv).max(csafr.min_sram_microvolt / 1000);
                                aux.cs_voltages[i][j] = *mv;
                                aux.cs_voltages_sram[i][j] = sram_mv;
                            }
                        }

                        for (i, ps) in csafr.perf_states_afr.iter().enumerate() {
                            aux.afr_frequencies[i] = ps.freq_hz / 1000000;
                            for (j, mv) in ps.volt_mv.iter().enumerate() {
                                let sram_mv = (*mv).max(csafr.min_sram_microvolt / 1000);
                                aux.afr_voltages[i][j] = *mv;
                                aux.afr_voltages_sram[i][j] = sram_mv;
                            }
                        }
                    }

                    // Special case override for T602x
                    #[ver(G == G14X)]
                    if dyncfg.id.gpu_rev_id == hw::GpuRevisionID::B1 {
                        raw.gpu_rev_id = hw::GpuRevisionID::B0 as u32;
                    }

                    if !dyncfg.hw_data_b.is_empty() {
                        unsafe {
                            let mut matches: bool = true;
                            let sla = core::slice::from_raw_parts(
                                raw as *const raw::HwDataB::ver as *const u8,
                                core::mem::size_of::<raw::HwDataB::ver>(),
                            );
                            if sla.len() != dyncfg.hw_data_b.len() {
                                matches = false;
                                dev_err!(
                                    self.dev.as_ref(),
                                    "!!! Hwdata B size mismatch: {} {}",
                                    sla.len(),
                                    dyncfg.hw_data_b.len(),
                                );
                            }
                            for i in 0..core::cmp::min(sla.len(), dyncfg.hw_data_b.len()) {
                                if sla[i] != dyncfg.hw_data_b[i] {
                                    matches = false;
                                    dev_err!(self.dev.as_ref(), "!!! Hwdata B first mismatch: {i}");
                                    break;
                                }
                            }
                            if matches {
                                dev_info!(self.dev.as_ref(), "!!! Hwdata B match");
                            }
                        }
                    }

                    Ok(())
                })
            })
    }

    /// Create the Globals structure, which contains global firmware config including more power
    /// configuration data and globals used to exchange state between the firmware and driver.
    #[ver(V < V14_8_3)]
    fn globals(&mut self) -> Result<GpuObject<Globals::ver>> {
        self.alloc
            .private
            .new_init(pin_init::init_zeroed(), |_inner, _ptr| {
                let cfg = &self.cfg;
                let dyncfg = &self.dyncfg;
                let pwr = &dyncfg.pwr;
                let period_ms = pwr.power_sample_period;
                let period_s = F32::from(period_ms) / f32!(1000.0);
                let avg_power_filter_tc_periods = pwr.avg_power_filter_tc_ms / period_ms;

                let max_ps = pwr.perf_max_pstate;
                let max_ps_scaled = 100 * max_ps;

                try_init!(raw::Globals::ver {
                    //ktrace_enable: 0xffffffff,
                    ktrace_enable: 0,
                    #[ver(V >= V13_2)]
                    unk_24_0: 3000,
                    unk_24: 0,
                    #[ver(V >= V13_0B4)]
                    debug: 0,
                    unk_28: 1,
                    #[ver(G >= G14X)]
                    unk_2c_0: 1,
                    #[ver(V >= V13_0B4 && G < G14X)]
                    unk_2c_0: 0,
                    unk_2c: 1,
                    unk_30: 0,
                    unk_34: 120,
                    // sub <- try_init!(raw::GlobalsSub::ver {
                        unk_54: cfg.global_unk_54,
                        unk_56: 40,
                        unk_58: 0xffff,
                        unk_5e: U32(1),
                        unk_66: U32(1),
                    //     ..Zeroable::init_zeroed()
                    // }),
                    unk_8900: 1,
                    pending_submissions: AtomicU32::new(0),
                    max_power: pwr.max_power_mw,
                    max_pstate_scaled: max_ps_scaled,
                    max_pstate_scaled_2: max_ps_scaled,
                    max_pstate_scaled_3: max_ps_scaled,
                    power_zone_count: pwr.power_zones.len() as u32,
                    avg_power_filter_tc_periods: avg_power_filter_tc_periods,
                    avg_power_ki_dt: pwr.avg_power_ki_only * period_s,
                    avg_power_kp: pwr.avg_power_kp,
                    avg_power_min_duty_cycle: pwr.avg_power_min_duty_cycle,
                    avg_power_target_filter_tc: pwr.avg_power_target_filter_tc,
                    unk_89bc: cfg.da.unk_8cc,
                    fast_die0_release_temp: 100 * pwr.fast_die0_release_temp,
                    unk_89c4: cfg.da.unk_87c,
                    fast_die0_prop_tgt_delta: 100 * pwr.fast_die0_prop_tgt_delta,
                    fast_die0_kp: pwr.fast_die0_proportional_gain,
                    fast_die0_ki_dt: pwr.fast_die0_integral_gain * period_s,
                    unk_89e0: 1,
                    max_power_2: pwr.max_power_mw,
                    ppm_kp: pwr.ppm_kp,
                    ppm_ki_dt: pwr.ppm_ki * period_s,
                    #[ver(V >= V13_0B4)]
                    unk_89f4_8: 1,
                    unk_89f4: 0,
                    hws1 <- Self::hw_shared1(cfg),
                    hws2 <- Self::hw_shared2(cfg, dyncfg),
                    hws3 <- Self::hw_shared3(cfg),
                    #[ver(V >= V13_0B4)]
                    idle_off_standby_timer: pwr.idle_off_standby_timer,
                    #[ver(V >= V13_0B4)]
                    unk_hws2_4: cfg.unk_hws2_4.map(Array::new).unwrap_or_default(),
                    #[ver(V >= V13_0B4)]
                    unk_hws2_24: cfg.unk_hws2_24,
                    unk_900c: 1,
                    #[ver(V >= V13_0B4)]
                    unk_9010_0: 1,
                    #[ver(V >= V13_0B4)]
                    unk_903c: 1,
                    #[ver(V < V13_0B4)]
                    unk_903c: 0,
                    fault_control: *module_parameters::fault_control.value(),
                    do_init: 1,
                    progress_check_interval_3d: 40,
                    progress_check_interval_ta: 10,
                    progress_check_interval_cl: 250,
                    #[ver(V >= V13_0B4)]
                    unk_1102c_0: 1,
                    #[ver(V >= V13_0B4)]
                    unk_1102c_4: 1,
                    #[ver(V >= V13_0B4)]
                    unk_1102c_8: 100,
                    #[ver(V >= V13_0B4)]
                    unk_1102c_c: 1,
                    idle_off_delay_ms: AtomicU32::new(pwr.idle_off_delay_ms),
                    fender_idle_off_delay_ms: pwr.fender_idle_off_delay_ms,
                    fw_early_wake_timeout_ms: pwr.fw_early_wake_timeout_ms,
                    cl_context_switch_timeout_ms: 40,
                    #[ver(V >= V13_0B4)]
                    cl_kill_timeout_ms: 50,
                    #[ver(V >= V13_0B4)]
                    unk_11edc: 0,
                    #[ver(V >= V13_0B4)]
                    unk_11efc: 0,
                    ..Zeroable::init_zeroed()
                })
                .chain(|raw| {
                    for (i, pz) in self.dyncfg.pwr.power_zones.iter().enumerate() {
                        raw.power_zones[i].target = pz.target;
                        raw.power_zones[i].target_off = pz.target - pz.target_offset;
                        raw.power_zones[i].filter_tc = pz.filter_tc;
                    }

                    if let Some(tab) = self.cfg.global_tab.as_ref() {
                        for (i, x) in tab.iter().enumerate() {
                            raw.unk_118ec[i] = *x;
                        }
                        raw.unk_118e8 = 1;
                    }

                    if !dyncfg.hw_globals.is_empty() {
                        unsafe {
                            let mut matches: bool = true;
                            let sla = core::slice::from_raw_parts(
                                raw as *const raw::Globals::ver as *const u8,
                                core::mem::size_of::<raw::Globals::ver>(),
                            );
                            if sla.len() != dyncfg.hw_globals.len() {
                                matches = false;
                                dev_err!(
                                    self.dev.as_ref(),
                                    "!!! Globals size mismatch: {} {}",
                                    sla.len(),
                                    dyncfg.hw_globals.len(),
                                );
                            }
                            for i in 0..core::cmp::min(sla.len(), dyncfg.hw_globals.len()) {
                                if sla[i] != dyncfg.hw_globals[i] {
                                    matches = false;
                                    dev_err!(self.dev.as_ref(), "!!! Globals first mismatch: {i}");
                                    break;
                                }
                            }
                            if matches {
                                dev_info!(self.dev.as_ref(), "!!! Globals match");
                            }
                        }
                    }

                    Ok(())
                })
            })
    }

    /// Create the RuntimePointers structure, which contains pointers to most of the other
    /// structures including the ring buffer channels, statistics structures, and HwDataA/HwDataB.
    #[ver(V < V14_8_3)]
    fn runtime_pointers(&mut self) -> Result<GpuObject<RuntimePointers::ver>> {
        let hwa = self.hwdata_a()?;
        let hwb = self.hwdata_b()?;
        let bufmgr_fw_addr = mmu::kern_iova(self.cfg, gpu::IOVA_KERN_GPU_BUFMGR_HIGH);

        let mut buffer_mgr_ctl = gem::new_kernel_object(self.dev, 0x4000)?;
        buffer_mgr_ctl.vmap()?.memset(0);

        GpuObject::new_init_prealloc(
            self.alloc.private.alloc_object()?,
            |_ptr| {
                let alloc = &mut *self.alloc;
                try_init!(RuntimePointers::ver {
                    stats <- {
                        let alloc = &mut *alloc;
                        try_init!(Stats::ver {
                            vtx: alloc.private.new_default::<GpuGlobalStatsVtx>()?,
                            frag: alloc.private.new_init(
                                pin_init::init_zeroed::<GpuGlobalStatsFrag::ver>(),
                                |_inner, _ptr| {
                                    try_init!(raw::GpuGlobalStatsFrag::ver {
                                        total_cmds: 0,
                                        unk_4: 0,
                                        stats: Default::default(),
                                    })
                                }
                            )?,
                            comp: alloc.private.new_default::<GpuStatsComp>()?,
                        })
                    },

                    hwdata_a: hwa,
                    unkptr_190: alloc.private.array_empty_tagged(0x80, b"I190")?,
                    unkptr_198: alloc.private.array_empty_tagged(0xc0, b"I198")?,
                    hwdata_b: hwb,

                    unkptr_1b8: alloc.private.array_empty_tagged(0x1000, b"I1B8")?,
                    unkptr_1c0: alloc.private.array_empty_tagged(0x300, b"I1C0")?,
                    unkptr_1c8: alloc.private.array_empty_tagged(0x1000, b"I1C8")?,

                    buffer_mgr_ctl,
                    buffer_mgr_ctl_low_mapping: None,
                    buffer_mgr_ctl_high_mapping: None,
                })
            },
            |inner, _ptr| {
                try_init!(raw::RuntimePointers::ver {
                    pipes: Default::default(),
                    device_control: Default::default(),
                    event: Default::default(),
                    fw_log: Default::default(),
                    ktrace: Default::default(),
                    stats: Default::default(),

                    stats_vtx: inner.stats.vtx.gpu_pointer(),
                    stats_frag: inner.stats.frag.gpu_pointer(),
                    stats_comp: inner.stats.comp.gpu_pointer(),

                    hwdata_a: inner.hwdata_a.gpu_pointer(),
                    unkptr_190: inner.unkptr_190.gpu_pointer(),
                    unkptr_198: inner.unkptr_198.gpu_pointer(),
                    hwdata_b: inner.hwdata_b.gpu_pointer(),
                    hwdata_b_2: inner.hwdata_b.gpu_pointer(),

                    fwlog_buf: None,

                    unkptr_1b8: inner.unkptr_1b8.gpu_pointer(),

                    #[ver(G < G14X)]
                    unkptr_1c0: inner.unkptr_1c0.gpu_pointer(),
                    #[ver(G < G14X)]
                    unkptr_1c8: inner.unkptr_1c8.gpu_pointer(),

                    buffer_mgr_ctl_gpu_addr: U64(gpu::IOVA_KERN_GPU_BUFMGR_LOW),
                    buffer_mgr_ctl_fw_addr: U64(bufmgr_fw_addr),

                    __pad0: Default::default(),
                    unk_160: U64(0),
                    unk_168: U64(0),
                    unk_1d0: 0,
                    unk_1d4: 0,
                    unk_1d8: Default::default(),

                    __pad1: Default::default(),
                    gpu_scratch: raw::RuntimeScratch::ver {
                        unk_6b38: 0xff,
                        ..Default::default()
                    },
                })
            },
        )
    }

    /// Create the FwStatus structure, which is used to coordinate the firmware halt state between
    /// the firmware and the driver.
    #[ver(V < V14_8_3)]
    fn fw_status(&mut self) -> Result<GpuObject<FwStatus>> {
        self.alloc
            .shared
            .new_object(Default::default(), |_inner| Default::default())
    }

    // Compare a structure against the copy m1n1 left in the reserved-memory blob, if any
    // (debug aid, same as the G13/G14 builders do inline).
    #[ver(V >= V14_8_3)]
    fn compare_blob<T>(dev: &AsahiDevice, what: &str, raw: &T, blob: &[u8]) {
        if blob.is_empty() {
            return;
        }
        // SAFETY: `raw` is a fully initialized firmware structure; viewing it as bytes is fine.
        let sla = unsafe {
            core::slice::from_raw_parts(raw as *const T as *const u8, core::mem::size_of::<T>())
        };
        if sla.len() != blob.len() {
            dev_err!(
                dev.as_ref(),
                "!!! {} size mismatch: {} {}",
                what,
                sla.len(),
                blob.len()
            );
        }
        match sla.iter().zip(blob.iter()).position(|(a, b)| a != b) {
            Some(i) => dev_err!(dev.as_ref(), "!!! {} first mismatch: {i}", what),
            None => dev_info!(dev.as_ref(), "!!! {} match", what),
        }
    }

    // Build the G15 (14.8.3) HwDataB contents without allocating the object. Board values whose
    // source is not known yet are left 0 and marked TODO.
    // (Plain comments: the versions macro needs `fn` right after a gating #[ver].)
    #[ver(V >= V14_8_3)]
    fn hwdata_b_init(
        cfg: &'static hw::HwConfig,
        dyncfg: &'a hw::DynConfig,
        g15: &G15Options,
    ) -> Result<impl Init<raw::HwDataB::ver, Error> + 'a> {
        let pwr = &dyncfg.pwr;
        let tables = g15.tables(pwr)?;
        let runtime = g15.layout == G15Layout::Runtime;
        let runtime_hwdata_b = g15.runtime_hwdata_b;
        let timestamp_base = match g15.timestamp_base {
            Some(base) => base,
            None => mmu::kern_iova(cfg, gpu::IOVA_KERN_TIMESTAMP_RANGE.start),
        };
        Ok(try_init!(raw::HwDataB::ver {
            // VA-layout words, +0x0/+0x8
            unk_8: U64(0x6f_00000000),
            unk_18: U64(0xffc00000),
            // USC start: the G15 firmware takes 0x10_00000000 at +0x10/+0x18. Like the G13
            // builder, the manager leaves 0: drm/asahi passes the USC base per queue.
            unk_20: U64(0),
            unk_28: U64(0),
            // "Unknown page": the G15 firmware layout uses 0x2ff_ffff8000 (+0x20) in its
            // 42-bit user layout; the manager keeps a 39-bit user range and maps the page
            // at IOVA_UNK_PAGE, so it passes that.
            unk_30: U64(mmu::IOVA_UNK_PAGE),
            // FW VA of the timestamp area, +0x28
            timestamp_area_base: U64(timestamp_base),
            // TODO: CSC matrices (+0x38) and border-colour pointer (+0x638)
            // are left 0, as on G13 ("TODO: yuv matrices").
            // +0xa28..+0xa34: chip id + 3 words (G13 unk_454..45c)
            chip_id: cfg.chip_id,
            unk_454: cfg.db.unk_454,
            unk_458: 0x1,
            // +0xa38..+0xa64 (G13 unk_460..unk_48c). +0xa48/+0xa54/+0xa64 are 0.
            // +0xa38 (GPC present, 1 on t6030) and +0xa40 (PIO supported) are 1: the
            // firmware runs its power management, and so powers up the GPU cores for
            // a kicked job, only if both are non-zero. +0xa50/+0xa58/+0xa60 are 1.
            // Tested on J516S: an empty compute job completes with these values.
            unk_460: 1,
            unk_468: 1,
            unk_478: 1,
            unk_480: 1,
            unk_488: 1,
            // +0xa68: 0x5dc0 = 24000
            base_clock_khz: cfg.base_clock_hz / 1000,
            // +0xa6c: 200 on slow simulation models, otherwise 1, so 1 on t6030. The
            // HwDataA filter constants follow +0xa70 (the sample period in ms), which
            // stays the DT value.
            power_sample_period: 1,
            // +0xa70: the value HwDataA +0xea8/+0xeac multiply the filter time
            // constants by, i.e. the sample period in ms (inferred)
            unk_a70: pwr.power_sample_period,
            // TODO: +0xa74, +0xa88, +0xa90, +0xaa0, +0xac4 are board/config values whose
            // source is not known yet; left 0.
            unk_a78: 1,
            unk_a7c: U64(1),
            unk_a98: 1,
            unk_a9c: 0x1f,
            unk_ac8: Array::new([0, 1, 1, 0]),
            // TODO: +0xaa8 and +0xb04 are condition flags; left 0.
            // "UAT enabled" flag, G13 unk_554 = 1. +0xb40
            unk_554: 0x1,
            // +0xb44..+0xb58
            uat_ttb_base: U64(dyncfg.uat_ttb_base),
            gpu_core_id: cfg.gpu_core.ok_or(ENODEV)? as u32,
            gpu_rev_id: dyncfg.id.gpu_rev_id as u32,
            num_cores: dyncfg.id.num_cores * dyncfg.id.num_clusters,
            max_pstate: tables.len as u32 - 1,
            // +0x17e8/+0x1800/+0x1804/+0x1810;
            // +0x1814 = 1 as well.
            unk_17e8: 1,
            timer_offset: U64(0),
            unk_1800: 1,
            unk_1804: 1,
            unk_1810: 1,
            unk_1814: 1,
            // +0x17fc = 1 on t6030.
            unk_17fc: 1,
            // +0x1820: 48 x 0xff
            all_ones_masks: Array::new([0xff; 0x30]),
            // +0x1860 = 1 & ~(debug global bit 0), so 1 without debug. (inferred)
            unk_1860: 1,
            ..Zeroable::init_zeroed()
        })
        .chain(move |raw| {
            // Rounded up, like the operating-point voltages.
            let min_sram_mv = pwr.min_sram_microvolt.div_ceil(1000);
            // Core and SRAM voltage rows of one state: every column holds the state's
            // device-tree voltage (columns beyond the device tree's repeat the first one), and
            // the SRAM rail never goes below the device tree's floor.
            let rows = |ps: &hw::PState| -> ([u32; 8], [u32; 8]) {
                let mut volts = [0u32; 8];
                for (j, v) in volts.iter_mut().enumerate() {
                    *v = *ps.volt_mv.get(j).unwrap_or(&ps.volt_mv[0]);
                }
                (volts, volts.map(|mv| mv.max(min_sram_mv)))
            };

            // p-state tables at +0xb58..+0xf9b, +0xfdc, same formulas as G13. Every
            // frequency and voltage comes from the device tree.
            for i in 0..tables.len {
                let ps = tables.primary_state(pwr, i);
                let (volts, sram) = rows(ps);
                raw.frequencies[i] = ps.freq_hz / 1000000;
                raw.voltages[i] = volts;
                raw.voltages_sram[i] = sram;
                raw.sram_k[i] = cfg.sram_k;
            }

            match tables.secondary {
                None => {
                    // One table in device-tree order: the second frequency table repeats it,
                    // and the index map is the identity.
                    for i in 0..tables.len {
                        raw.frequencies_2[i] = raw.frequencies[i];
                    }
                    for i in 0..16 {
                        raw.unk_arr_0[i] = i as u32;
                    }
                }
                Some(secondary) => {
                    // The secondary table (+0xf9c), the device-tree index of every entry of
                    // both tables (+0x10dc, +0x111c) and the third table (+0x134c, the
                    // secondary table with one voltage column).
                    // The base state is a device-tree index, and the HwDataA base words use it as
                    // an index into the published table: the two must name the same state.
                    if pwr.perf_base_pstate as usize >= tables.len
                        || u32::from(tables.primary[pwr.perf_base_pstate as usize])
                            != pwr.perf_base_pstate
                    {
                        return Err(EINVAL);
                    }
                    let top = tables.primary_state(pwr, tables.len - 1);
                    let base = tables.primary_state(pwr, pwr.perf_base_pstate as usize);
                    if top.pwr_mw == 0 || top.freq_hz <= base.freq_hz {
                        return Err(EINVAL);
                    }
                    let aux = &mut raw.aux_ps_3;
                    aux.max_pstate = tables.len as u32 - 1;
                    for i in 0..tables.len {
                        let ps = &pwr.perf_states[secondary[i] as usize];
                        let (volts, sram) = rows(ps);
                        raw.frequencies_2[i] = ps.freq_hz / 1000000;
                        raw.unk_arr_0[i] = tables.primary[i] as u32;
                        raw.unk_arr_0[16 + i] = secondary[i] as u32;
                        aux.frequencies[i] = ps.freq_hz / 1000000;
                        aux.voltages[i][0] = volts[0];
                        aux.voltages_sram[i][0] = sram[0];

                        // Maximum power and boost frequency of each primary entry, in percent
                        // of the top entry (boost relative to the base state).
                        let ps = tables.primary_state(pwr, i);
                        if i > 0 {
                            raw.rel_max_powers[i] =
                                (100 * u64::from(ps.pwr_mw) / u64::from(top.pwr_mw)) as u32;
                        }
                        if ps.freq_hz > base.freq_hz {
                            let f = u64::from(ps.freq_hz / 1000000 - base.freq_hz / 1000000);
                            let span = u64::from(top.freq_hz / 1000000 - base.freq_hz / 1000000);
                            raw.rel_boost_freqs[i] = (100 * f / span) as u32;
                        }
                    }
                }
            }

            if let Some(csafr) = pwr.csafr.as_ref() {
                let aux = &mut raw.aux_ps;
                aux.cs_max_pstate = (csafr.perf_states_cs.len() - 1).try_into()?;
                aux.afr_max_pstate = (csafr.perf_states_afr.len() - 1).try_into()?;

                for (i, ps) in csafr.perf_states_cs.iter().enumerate() {
                    aux.cs_frequencies[i] = ps.freq_hz / 1000000;
                    for (j, mv) in ps.volt_mv.iter().enumerate() {
                        let sram_mv = (*mv).max(csafr.min_sram_microvolt / 1000);
                        aux.cs_voltages[i][j] = *mv;
                        aux.cs_voltages_sram[i][j] = sram_mv;
                    }
                }

                for (i, ps) in csafr.perf_states_afr.iter().enumerate() {
                    aux.afr_frequencies[i] = ps.freq_hz / 1000000;
                    for (j, mv) in ps.volt_mv.iter().enumerate() {
                        let sram_mv = (*mv).max(csafr.min_sram_microvolt / 1000);
                        aux.afr_voltages[i][j] = *mv;
                        aux.afr_voltages_sram[i][j] = sram_mv;
                    }
                }
            }

            // +0x17c0 (u64) / +0x17c8 (u32): masks with the low N bits set
            // (per-core / per-cluster enable masks, inferred).
            // TODO: which counts these are; default total cores / clusters.
            let low_bits = |n: u32| -> u64 {
                if n >= 64 {
                    u64::MAX
                } else {
                    (1u64 << n) - 1
                }
            };
            raw.unit_mask_a = U64(low_bits(dyncfg.id.num_cores * dyncfg.id.num_clusters));
            raw.unit_mask_b = low_bits(dyncfg.id.num_clusters.min(32)) as u32;

            if runtime {
                Self::hwdata_b_runtime(raw, runtime_hwdata_b.ok_or(EINVAL)?);
            }
            Ok(())
        }))
    }

    // The runtime backend's HwDataB words: its GPU VA layout (USC base, unknown page), and the
    // SoC's configuration words and unit masks (`words`, see `G15RuntimeHwDataB`). Those are a
    // runtime InitData's values, not derived from the device tree: the T6030 ones hold for the
    // boards that InitData was checked on (J514S, J516S), not necessarily for other
    // configurations.
    #[ver(V >= V14_8_3)]
    fn hwdata_b_runtime(raw: &mut raw::HwDataB::ver, words: &G15RuntimeHwDataB) {
        raw.unk_20 = U64(RUNTIME_USC_BASE);
        raw.unk_28 = U64(RUNTIME_USC_BASE);
        raw.unk_30 = U64(RUNTIME_UNKNOWN_PAGE);
        raw.unk_454 = words.unk_454;
        raw.unk_464 = words.unk_464;
        raw.unk_a7c = U64(words.unk_a7c);
        raw.unk_a98 = words.unk_a98;
        raw.unk_abc = U64(words.unk_abc);
        raw.unk_ae4 = words.unk_ae4;
        raw.unk_b20 = words.unk_b20;
        raw.unk_b24 = U64(words.unk_b24);
        raw.unk_554 = words.unk_554;
        raw.unk_17b8 = words.unk_17b8;
        raw.unit_mask_a = U64(words.unit_mask_a);
        raw.unit_mask_b = words.unit_mask_b;
        raw.unk_1808 = words.unk_1808;
        raw.unk_1818 = words.unk_1818;
    }

    // Create the G15 (14.8.3) HwDataB structure.
    #[ver(V >= V14_8_3)]
    fn hwdata_b(&mut self) -> Result<GpuObject<HwDataB::ver>> {
        let init = Self::hwdata_b_init(self.cfg, self.dyncfg, &self.g15)?;
        let dev = self.dev;
        let dyncfg = self.dyncfg;
        self.alloc
            .private
            .new_init(pin_init::init_zeroed(), move |_inner, _ptr| {
                init.chain(move |raw| {
                    Self::compare_blob(dev, "Hwdata B", &*raw, &dyncfg.hw_data_b);
                    Ok(())
                })
            })
    }

    // Build the G15 (14.8.3) Globals contents without allocating the object. Only the fields
    // with a known init value are set; the rest start at 0.
    #[ver(V >= V14_8_3)]
    fn globals_init(
        cfg: &'static hw::HwConfig,
        dyncfg: &'a hw::DynConfig,
        g15: &G15Options,
    ) -> Result<impl Init<raw::Globals::ver, Error> + 'a> {
        let pwr = &dyncfg.pwr;
        let tables = g15.tables(pwr)?;
        let runtime = g15.layout == G15Layout::Runtime;
        let reference_ppm = g15.reference_ppm;
        Ok(try_init!(raw::Globals::ver {
            // TODO: 0 or 7 depending on a board condition (+0x0); 0 = timer
            // context switch off.
            timer_cswitch: 0,
            // G13 unk_24 = 0, debug = 0 (inferred)
            unk_28: 0,
            debug: 0,
            // +0x38: constant 0x78
            unk_38: 0x78,
            // G13 unk_54/56/58 moved to +0x50..+0x54 (inferred)
            // TODO: these are board configuration values; G13 values for now.
            unk_50: cfg.global_unk_54,
            unk_52: 40,
            unk_54: 0xffff,
            // G13 unk_5e = 1 moved to +0x5a (a flag on G15) (inferred)
            unk_5a: U32(1),
            // +0x5e = max(HwDataB+0xa6c, 1) = 1.
            cswitch_timer_multiplier: U32(1),
            // Must be non-zero before work is submitted.
            // Non-zero at boot has been accepted by the firmware on J516S.
            command_submission_enabled: 1,
            pending_submissions: AtomicU32::new(0),
            // G13 values. TODO: the G15 defaults for +0x97c..+0x984 are not
            // known.
            progress_check_interval_3d: 40,
            progress_check_interval_ta: 10,
            progress_check_interval_cl: 250,
            // G13 unk_1102c_0.._c values; same group at +0x988 (inferred)
            unk_1102c_0: 1,
            unk_1102c_4: 1,
            unk_1102c_8: 100,
            unk_1102c_c: 1,
            // +0x9a0..+0x9a8
            // TODO: with the rings in Fender SRAM the GPU and Fender must stay
            // powered from firmware boot on, until an SRAM backup/restore exists.
            // gpu.rs update_globals() keeps idle_off_delay_ms at the same value on
            // every submit.
            idle_off_delay_ms: AtomicU32::new(G15_KEEP_POWERED_MS),
            fender_idle_off_delay_ms: G15_KEEP_POWERED_MS,
            fw_early_wake_timeout_ms: pwr.fw_early_wake_timeout_ms,
            // G13 values (+0x9b0/+0x9b4, set by setters)
            cl_context_switch_timeout_ms: 40,
            cl_kill_timeout_ms: 50,
            // The manager leaves the FwUtil perf tunables (cfg.global_tab) and
            // `soft_fault_settings` (+0xdf9) unwritten; the runtime layout sets them below.
            ..Zeroable::init_zeroed()
        })
        .chain(move |raw| {
            if runtime {
                Self::globals_runtime(raw, cfg, pwr, &tables)?;
            } else if reference_ppm {
                Self::globals_power_targets(raw, pwr, &tables);
            }
            Ok(())
        }))
    }

    // The Globals power-interface targets (maximum power from the device tree, then the highest
    // usable performance state scaled by 100, twice) and the performance-state cap.
    #[ver(V >= V14_8_3)]
    fn globals_power_targets(raw: &mut raw::Globals::ver, pwr: &hw::PwrConfig, tables: &PStateTables) {
        let max_ps_scaled = 100 * tables.max;
        raw.power_if_target = Array::new([pwr.max_power_mw, max_ps_scaled, max_ps_scaled]);
        raw.perf_state_cap = max_ps_scaled;
    }

    // The runtime backend's Globals: DRAM rings with finite idle-off, so the idle policy
    // (imported by DeviceControl 0x13) and smart idle-off are on and the device-tree
    // idle-off delays apply; the power interface targets and the performance-state cap
    // (maximum power from the device tree, pstates scaled by 100); the smart idle-off
    // configuration (standby timer from the device tree); and the FwUtil tunables.
    #[ver(V >= V14_8_3)]
    fn globals_runtime(
        raw: &mut raw::Globals::ver,
        cfg: &'static hw::HwConfig,
        pwr: &hw::PwrConfig,
        tables: &PStateTables,
    ) -> Result {
        raw.relaxed_cl_kill_timeout = 3000;
        raw.debug = 1;
        raw.smart_idle_off_enable = 1;
        raw.unk_50 = 0xffff;
        // The runtime enables command submission through its own queue protocol.
        raw.command_submission_enabled = 0;
        Self::globals_power_targets(raw, pwr, tables);

        let mut idle = [0u32; 10];
        idle[0] = pwr.idle_off_standby_timer;
        if let Some(curve) = cfg.unk_hws2_4 {
            for (slot, k) in idle[1..9].iter_mut().zip(curve.iter()) {
                *slot = k.to_bits();
            }
        }
        idle[9] = cfg.unk_hws2_24;
        raw.smart_idle_off_cfg = Array::new(idle);
        raw.ut_engagement = 1;
        raw.clvr_engagement = 1;
        raw.keepalive_perf_threshold_rd = 100;
        raw.keepalive_off_threshold_rd = 100;

        raw.idle_off_delay_ms = AtomicU32::new(pwr.idle_off_delay_ms);
        raw.fender_idle_off_delay_ms = pwr.fender_idle_off_delay_ms;
        raw.fw_early_wake_timeout_ms = pwr.fw_early_wake_timeout_ms;

        // CDM backoff 4, then a u32 1 and the FwUtil perf tunables from +0x9c1.
        raw.cdm_backoff_timeout = 4;
        raw.unk_9bd = Array::new([1, 0, 0, 0]);
        if let Some(tab) = cfg.global_tab {
            let mut bytes = [0u8; 0x1c];
            bytes.get_mut(..tab.len()).ok_or(EINVAL)?.copy_from_slice(tab);
            raw.fwutil_default_fab_pstate = Array::new([bytes[0], bytes[1]]);
            raw.fwutil_timer_period = bytes[2];
            (*raw.unk_9c4)[..0x19].copy_from_slice(&bytes[3..0x1c]);
        }
        raw.soft_fault_settings = U32(3);
        Ok(())
    }

    // Create the G15 (14.8.3) Globals structure.
    #[ver(V >= V14_8_3)]
    fn globals(&mut self) -> Result<GpuObject<Globals::ver>> {
        let init = Self::globals_init(self.cfg, self.dyncfg, &self.g15)?;
        let dev = self.dev;
        let dyncfg = self.dyncfg;
        self.alloc
            .private
            .new_init(pin_init::init_zeroed(), move |_inner, _ptr| {
                init.chain(move |raw| {
                    Self::compare_blob(dev, "Globals", &*raw, &dyncfg.hw_globals);
                    Ok(())
                })
            })
    }

    // Create the G15 (14.8.3) RuntimePointers structure. The ring descriptors are filled in by
    // the GPU manager.
    #[ver(V >= V14_8_3)]
    fn runtime_pointers(&mut self) -> Result<GpuObject<RuntimePointers::ver>> {
        let hwa = self.hwdata_a()?;
        let hwb = self.hwdata_b()?;
        let bufmgr_fw_addr = mmu::kern_iova(self.cfg, gpu::IOVA_KERN_GPU_BUFMGR_HIGH);
        let rp_2d0 = self.g15.runtime_pointers_2d0();

        let mut buffer_mgr_ctl = gem::new_kernel_object(self.dev, 0x4000)?;
        buffer_mgr_ctl.vmap()?.memset(0);

        GpuObject::new_init_prealloc(
            self.alloc.private.alloc_object()?,
            |_ptr| {
                let alloc = &mut *self.alloc;
                try_init!(RuntimePointers::ver {
                    // G15 E1/E2/E3 are 0xc10/0x1248/0xe10 bytes; the G13
                    // GpuGlobalStatsVtx/GpuStatsComp buffers are larger, which is harmless.
                    stats <- {
                        let alloc = &mut *alloc;
                        try_init!(Stats::ver {
                            vtx: alloc.private.new_default::<GpuGlobalStatsVtx>()?,
                            frag: alloc.private.new_init(
                                pin_init::init_zeroed::<GpuGlobalStatsFrag::ver>(),
                                |_inner, _ptr| {
                                    try_init!(raw::GpuGlobalStatsFrag::ver {
                                        total_cmds: 0,
                                        unk_4: 0,
                                        stats: Default::default(),
                                    })
                                }
                            )?,
                            comp: alloc.private.new_default::<GpuStatsComp>()?,
                        })
                    },

                    hwdata_a: hwa,
                    hwdata_b: hwb,

                    // BRN (erratum) table.
                    // TODO: size and contents unknown; one zeroed page (no errata).
                    brn_table: alloc.private.array_empty_tagged(0x4000, b"IBRN")?,
                    // E7: 0x88 bytes, zeroed
                    unkptr_e7: alloc.private.array_empty_tagged(0x88, b"IE7_")?,
                    // E5: 0x60 bytes, zeroed
                    unkptr_e5: alloc.private.array_empty_tagged(0x60, b"IE5_")?,
                    // UMA page-pool descriptor table (RuntimePointers +0x2c0/+0x2c8).
                    // TODO: size/format unknown; one zeroed page. The firmware takes a
                    // GPU VA and a FW VA; both point at this kernel mapping for now.
                    uma_pool_desc: alloc.shared.array_empty_tagged(0x4000, b"IUMA")?,

                    buffer_mgr_ctl,
                    buffer_mgr_ctl_low_mapping: None,
                    buffer_mgr_ctl_high_mapping: None,
                })
            },
            |inner, _ptr| {
                try_init!(raw::RuntimePointers::ver {
                    hwdata_b: inner.hwdata_b.gpu_pointer(),
                    brn_table: inner.brn_table.gpu_pointer(),
                    unkptr_e7: inner.unkptr_e7.gpu_pointer(),

                    pipes: Default::default(),
                    device_control: Default::default(),
                    event: Default::default(),
                    fw_log: Default::default(),
                    ktrace: Default::default(),
                    stats: Default::default(),
                    fwlog_buf: None,
                    __pad_200: Default::default(),
                    // FWLog enable: the firmware writes no log records while this is 0.
                    unk_230: 0,

                    stats_vtx: inner.stats.vtx.gpu_pointer(),
                    stats_frag: inner.stats.frag.gpu_pointer(),
                    stats_comp: inner.stats.comp.gpu_pointer(),
                    unkptr_e5: inner.unkptr_e5.gpu_pointer(),
                    __pad_254: Default::default(),
                    // Optional pointer, left 0
                    unkptr_2a8: U64(0),
                    buffer_mgr_ctl_gpu_addr: U64(gpu::IOVA_KERN_GPU_BUFMGR_LOW),
                    buffer_mgr_ctl_fw_addr: U64(bufmgr_fw_addr),
                    uma_pool_desc_gpu_addr: U64(inner.uma_pool_desc.gpu_va().get()),
                    uma_pool_desc_fw_addr: U64(inner.uma_pool_desc.gpu_va().get()),
                    // TODO: board configuration words; +0x2d4 left 0.
                    unk_2d0: rp_2d0,
                    unk_2d4: 0,
                    __pad_2d8: Default::default(),
                    // +0x3b0: 0xff, then 0x90 zero bytes
                    cswitch_hist_ctl: 0xff,
                    unk_3b1: Default::default(),
                    hwdata_a: inner.hwdata_a.gpu_pointer(),
                    // TODO: 16 board configuration bytes; left 0.
                    unk_449: Default::default(),
                    __pad_459: Default::default(),
                    unk_46d: Default::default(),
                    __pad_48d: Default::default(),
                })
            },
        )
    }

    // Fill the G15 P0 status block. Host-initialised words are 0 except +0x45c4, which arms the
    // firmware work-scheduler callbacks, and in the runtime layout the noise-suppression idle
    // power-off state (+0x4), which the runtime's InitData starts at 1.
    #[ver(V >= V14_8_3)]
    fn status_block_fill(raw: &mut raw::StatusBlock, g15: &G15Options) {
        raw.flags.sched_callbacks = 1;
        if g15.layout == G15Layout::Runtime {
            raw.noise_suppression_idle_pwroff = 1;
        }
    }

    // Create the G15 P0 status block.
    #[ver(V >= V14_8_3)]
    fn status_block(&mut self) -> Result<GpuObject<StatusBlock>> {
        let g15 = self.g15;
        let mut block = self.alloc.shared.new_default::<StatusBlock>()?;
        block.with_mut(|raw, _inner| Self::status_block_fill(raw, &g15));
        Ok(block)
    }

    // Build the G15 P1 power-controller block without allocating it. The fields keep the order
    // and the formulas of the G13 13.5 Globals power fields (inferred).
    #[ver(V >= V14_8_3)]
    fn power_ctl_block_init(
        cfg: &'static hw::HwConfig,
        dyncfg: &'a hw::DynConfig,
        g15: &G15Options,
    ) -> impl Init<raw::PowerCtlBlock, Error> + 'a {
        let pwr = &dyncfg.pwr;
        let period_ms = pwr.power_sample_period;
        let period_s = F32::from(period_ms) / f32!(1000.0);
        let runtime = g15.layout == G15Layout::Runtime;
        let reference_ppm = runtime || g15.reference_ppm;

        try_init!(raw::PowerCtlBlock {
            power_zone_count: pwr.power_zones.len() as u32,
            avg_power_filter_tc_periods: pwr.avg_power_filter_tc_ms / period_ms,
            avg_power_ki_dt: pwr.avg_power_ki_only * period_s,
            avg_power_kp: pwr.avg_power_kp,
            avg_power_min_duty_cycle: pwr.avg_power_min_duty_cycle,
            avg_power_target_filter_tc: pwr.avg_power_target_filter_tc,
            // G13 Globals unk_89bc..unk_89e0 order (inferred)
            fast_die_temp_target: cfg.da.unk_8cc,
            fast_die0_release_temp: 100 * pwr.fast_die0_release_temp,
            unk_b0: cfg.da.unk_87c,
            fast_die0_prop_tgt_delta: 100 * pwr.fast_die0_prop_tgt_delta,
            fast_die0_kp: pwr.fast_die0_proportional_gain,
            fast_die0_ki_dt: pwr.fast_die0_integral_gain * period_s,
            // +0xc0/+0xc4 are board values at init (also mirrored at HwDataA
            // +0x934/+0x93c, G13 unk_8e8/unk_8f0, which the G13 path leaves 0).
            // TODO: real values unknown; default 0 as in the G13 HwDataA mirror.
            unk_c0: 0,
            unk_c4: 0,
            // +0xc8..+0xd8, manager layout: the G13 unk_89e0 = 1 flag (on G15 a
            // condition flag), then the PPM target and the PPM kp. The PPM controller
            // normally receives them at runtime; the G13 Globals values keep it from
            // starting with a zero target.
            ppm_words: Array::new([1, pwr.max_power_mw, pwr.ppm_kp.to_bits(), 0, 0]),
            // +0xec: 1 at init
            se_engagement: 1,
            ..Zeroable::init_zeroed()
        })
        .chain(move |raw| {
            for (i, pz) in pwr.power_zones.iter().enumerate() {
                raw.power_zones[i].target = pz.target;
                raw.power_zones[i].target_off = pz.target - pz.target_offset;
                raw.power_zones[i].filter_tc = pz.filter_tc;
            }
            if runtime {
                // Runtime layout: no fast-die release temperature or +0xb0 offset (the
                // T6030 device tree has no source for them).
                raw.fast_die0_release_temp = 0;
                raw.unk_b0 = 0;
            }
            if reference_ppm {
                // 0 at +0xc8, then the enable flag, the PPM target and the PPM kp and ki*dt.
                let ppm_ki_dt = pwr.ppm_ki * period_s;
                raw.ppm_words = Array::new([
                    0,
                    1,
                    pwr.max_power_mw,
                    pwr.ppm_kp.to_bits(),
                    ppm_ki_dt.to_bits(),
                ]);
            }
            Ok(())
        })
    }

    // Create the G15 P1 power-controller block.
    #[ver(V >= V14_8_3)]
    fn power_ctl_block(&mut self) -> Result<GpuObject<PowerCtlBlock>> {
        let init = Self::power_ctl_block_init(self.cfg, self.dyncfg, &self.g15);
        self.alloc
            .shared
            .new_init(pin_init::init_zeroed(), move |_inner, _ptr| init)
    }

    // Create the G15 P2 debug block. ktrace stays disabled, as on G13.
    #[ver(V >= V14_8_3)]
    fn debug_block(&mut self) -> Result<GpuObject<DebugBlock>> {
        self.alloc.shared.new_default::<DebugBlock>()
    }

    /// Create one UatLevelInfo structure, which describes one level of translation for the UAT MMU.
    fn uat_level_info(
        cfg: &'static hw::HwConfig,
        index_shift: usize,
        num_entries: usize,
    ) -> raw::UatLevelInfo {
        raw::UatLevelInfo {
            index_shift: index_shift as _,
            unk_1: 14,
            unk_2: 14,
            unk_3: 8,
            unk_4: 0x4000,
            num_entries: num_entries as _,
            unk_8: U64(1),
            unk_10: U64(((1u64 << cfg.uat_oas) - 1) & !(mmu::UAT_PGMSK as u64)),
            index_mask: U64(((num_entries - 1) << index_shift) as u64),
        }
    }

    /// The three UatLevelInfo records of InitData: L1 ("page catalogue") entries are 8 with a
    /// 39-bit IAS and 64 with the 42-bit G15 IAS (index mask 0x3f0_0000_0000).
    pub(crate) fn uat_levels(cfg: &'static hw::HwConfig) -> [raw::UatLevelInfo; 3] {
        let uat_l1_entries = 1usize << (cfg.uat_ias as usize - 36);
        [
            Self::uat_level_info(cfg, 36, uat_l1_entries),
            Self::uat_level_info(cfg, 25, 2048),
            Self::uat_level_info(cfg, 14, 2048),
        ]
    }

    /// Build the top-level InitData object.
    #[inline(never)]
    pub(crate) fn build(&mut self) -> Result<KBox<GpuObject<InitData::ver>>> {
        #[ver(V >= V14_8_3)]
        {
            let pwr = &self.dyncfg.pwr;
            let tables = self.g15.tables(pwr)?;
            dev_info!(
                self.dev.as_ref(),
                "G15: performance cap {} -> {} state {} of 1..={} ({} MHz)\n",
                self.g15.cap.unwrap_or(0),
                if tables.secondary.is_some() { "voltage-sorted" } else { "device-tree" },
                tables.max,
                tables.len - 1,
                tables.primary_state(pwr, tables.max as usize).freq_hz / 1_000_000
            );
        }
        let runtime_pointers = self.runtime_pointers()?;
        let globals = self.globals()?;
        #[ver(V < V14_8_3)]
        let fw_status = self.fw_status()?;
        // 14.x: FwStatus lives inside the P0 status block
        #[ver(V >= V14_8_3)]
        let fw_status = self.status_block()?;
        #[ver(V >= V14_8_3)]
        let power_ctl_block = self.power_ctl_block()?;
        #[ver(V >= V14_8_3)]
        let debug_block = self.debug_block()?;
        let shared_ro = &mut self.alloc.shared_ro;

        let obj = self.alloc.private.new_init(
            try_init!(InitData::ver {
                unk_buf: shared_ro.array_empty_tagged(0x4000, b"IDTA")?,
                runtime_pointers,
                globals,
                fw_status,
                #[ver(V >= V14_8_3)]
                power_ctl_block,
                #[ver(V >= V14_8_3)]
                debug_block,
            }),
            |inner, _ptr| {
                let cfg = &self.cfg;
                try_init!(raw::InitData::ver {
                    #[ver(V == V13_5 && G != G14X)]
                    ver_info: Array::new([0x6ba0, 0x1f28, 0x601, 0xb0]),
                    #[ver(V == V13_5 && G == G14X)]
                    ver_info: Array::new([0xb390, 0x70f8, 0x601, 0xb0]),
                    #[ver(V == V14_8_3 && G == G15)]
                    ver_info: Array::new([0x0490, 0x8380, 0xe21e, 0x0c08]),
                    unk_buf: inner.unk_buf.gpu_pointer(),
                    unk_8: 0,
                    unk_c: 0,
                    runtime_pointers: inner.runtime_pointers.gpu_pointer(),
                    globals: inner.globals.gpu_pointer(),
                    #[ver(V < V14_8_3)]
                    fw_status: inner.fw_status.gpu_pointer(),
                    // InitData +0x28/+0x2c: u32 0, u32 1
                    #[ver(V >= V14_8_3)]
                    unk_28: 0,
                    #[ver(V >= V14_8_3)]
                    unk_2c: 1,
                    uat_page_size: 0x4000,
                    uat_page_bits: 14,
                    uat_num_levels: 3,
                    uat_level_info: Array::new(Self::uat_levels(cfg)),
                    #[ver(V < V14_8_3)]
                    __pad0: Default::default(),
                    #[ver(V < V14_8_3)]
                    host_mapped_fw_allocations: 1,
                    #[ver(V < V14_8_3)]
                    unk_ac: 0,
                    #[ver(V < V14_8_3)]
                    unk_b0: 0,
                    #[ver(V < V14_8_3)]
                    unk_b4: 0,
                    #[ver(V < V14_8_3)]
                    unk_b8: 0,
                    // TODO: pmap descriptor +0x7c, meaning unknown; left 0.
                    #[ver(V >= V14_8_3)]
                    unk_94: 0,
                    #[ver(V >= V14_8_3)]
                    __pad_98: Default::default(),
                    #[ver(V >= V14_8_3)]
                    debug_block: inner.debug_block.gpu_pointer(),
                    #[ver(V >= V14_8_3)]
                    status_block: inner.fw_status.gpu_pointer(),
                    #[ver(V >= V14_8_3)]
                    power_ctl_block: inner.power_ctl_block.gpu_pointer(),
                })
            },
        )?;
        Ok(KBox::new(obj, GFP_KERNEL)?)
    }
}

/// G15 InitData contents built into plain memory, for a backend that places the firmware objects
/// itself (the M3 runtime backend, whose object layout is fixed).
pub(crate) struct G15Contents {
    pub(crate) hwdata_a: KVBox<raw::HwDataAG15V14_8_3>,
    pub(crate) hwdata_b: KVBox<raw::HwDataBG15V14_8_3>,
    pub(crate) globals: KVBox<raw::GlobalsG15V14_8_3>,
    pub(crate) power_ctl_block: KVBox<raw::PowerCtlBlock>,
    pub(crate) status_block: KVBox<raw::StatusBlock>,
    pub(crate) uat_levels: [raw::UatLevelInfo; 3],
    /// The published performance-state tables.
    pub(crate) tables: PStateTables,
}

impl<'a> InitDataBuilderG15V14_8_3<'a> {
    /// Build the G15 InitData contents with the same builders the manager backend uses, into
    /// plain memory. Pointer fields stay zero; the caller relocates them.
    pub(crate) fn g15_contents(
        cfg: &'static hw::HwConfig,
        dyncfg: &'a hw::DynConfig,
        g15: &G15Options,
    ) -> Result<G15Contents> {
        let mut status_block = KVBox::init(pin_init::init_zeroed(), GFP_KERNEL)?;
        Self::status_block_fill(&mut status_block, g15);
        Ok(G15Contents {
            hwdata_a: KVBox::init(Self::hwdata_a_init(cfg, dyncfg, g15)?, GFP_KERNEL)?,
            hwdata_b: KVBox::init(Self::hwdata_b_init(cfg, dyncfg, g15)?, GFP_KERNEL)?,
            globals: KVBox::init(Self::globals_init(cfg, dyncfg, g15)?, GFP_KERNEL)?,
            power_ctl_block: KVBox::init(Self::power_ctl_block_init(cfg, dyncfg, g15), GFP_KERNEL)?,
            status_block,
            uat_levels: Self::uat_levels(cfg),
            tables: g15.tables(&dyncfg.pwr)?,
        })
    }
}

/// The bytes of a firmware structure built by this module.
///
/// The structures are plain `repr(C)` data, and every builder starts from a zeroed object, so
/// no byte is uninitialized.
pub(crate) fn raw_bytes<T>(value: &T) -> &[u8] {
    // SAFETY: `value` is a live, initialized `T` (see above); the slice covers exactly it and
    // borrows it for its lifetime.
    unsafe { core::slice::from_raw_parts((value as *const T).cast::<u8>(), size_of::<T>()) }
}
