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
use crate::m3::g15_raw::*;
use crate::fw::types::*;
use crate::{
    m3::hw,
    mmu, //
};
use kernel::error::{
    Error,
    Result, //
};
use kernel::prelude::*;
use kernel::try_init;

use ::pin_init;
use ::pin_init::Init;

/// Builder helper for the global GPU InitData.

/// The "unknown page" of the 39-bit user layout (7.1.12 `mmu::IOVA_UNK_PAGE`): two pages below
/// the top of the user range.
const IOVA_UNK_PAGE: u64 = (1 << 39) - 2 * 0x4000;

/// G15 idle-off delay (ms, Globals +0x9a0/+0x9a4) used as "never idle off" while the rings live
/// in Fender scratch SRAM without a backup/restore path.
/// TODO: the firmware's encoding of "never" is unknown. i32::MAX stays positive if the
/// firmware treats the value as signed, and is never smaller than u32::MAX would be if it
/// converts ms to 24 MHz ticks in 32-bit arithmetic (both wrap to ~179 s); with 64-bit
/// arithmetic it is ~24 days.
#[allow(dead_code)]
pub(crate) const G15_KEEP_POWERED_MS: u32 = i32::MAX as u32;

/// Which T6030 backend a G15 InitData is built for. The two differ where the manager backend
/// keeps the values it was validated with on J516S.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub(crate) enum G15Layout {
    /// The G15 GpuManager backend: rings in Fender SRAM, the GPU never idles off.
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
    /// does (`aurora.g15_debug` bit 55). The runtime layout always does.
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
pub(crate) const RUNTIME_UNKNOWN_PAGE: u64 = 0x2ff_ffff_8000;

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

    /// The frequency of the highest entry the firmware may select, in kHz.
    pub(crate) fn max_frequency_khz(&self, pwr: &hw::PwrConfig) -> u32 {
        self.primary_state(pwr, self.max as usize).freq_hz / 1000
    }
}


/// The G15 InitData builders. The M3 runtime places the firmware objects itself, so only the
/// builders into plain memory are kept ([`Self::g15_contents`]).
pub(crate) struct InitDataBuilderG15V14_8_3<'a>(core::marker::PhantomData<&'a ()>);

impl<'a> InitDataBuilderG15V14_8_3<'a> {

    /// Create an unknown T81xx-specific data structure.
    fn t81xx_data(
        cfg: &'static hw::HwConfig,
        dyncfg: &'a hw::DynConfig,
    ) -> impl Init<raw::T81xxData> {
        let _perf_max_pstate = dyncfg.pwr.perf_max_pstate;

        pin_init::init_zeroed::<raw::T81xxData>().chain(move |_ret| {
            match cfg.chip_id {
                0x8103 | 0x8112 => {
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
    ) -> Result<impl Init<raw::HwDataAG15V14_8_3, Error> + 'a> {
        let pwr = &dyncfg.pwr;
        let period_ms = pwr.power_sample_period;
        let period_s = F32::from(period_ms) / f32!(1000.0);
        let ppm_filter_tc_periods = pwr.ppm_filter_time_constant_ms / period_ms;
        let ppm_filter_tc_ms_rounded = ppm_filter_tc_periods * period_ms;
        let ppm_filter_a = f32!(1.0) / ppm_filter_tc_periods.into();
        let perf_filter_a = f32!(1.0) / pwr.perf_filter_time_constant.into();
        let perf_filter_a2 = f32!(1.0) / pwr.perf_filter_time_constant2.into();
        let avg_power_target_filter_a = f32!(1.0) / pwr.avg_power_target_filter_tc.into();
        let avg_power_filter_tc_periods = pwr.avg_power_filter_tc_ms / period_ms;
        let avg_power_filter_tc_ms_rounded = avg_power_filter_tc_periods * period_ms;
        let avg_power_filter_a = f32!(1.0) / avg_power_filter_tc_periods.into();
        let pwr_filter_a = f32!(1.0) / pwr.pwr_filter_time_constant.into();

        let base_ps = pwr.perf_base_pstate;
        let base_ps_scaled = 100 * base_ps;
        let tables = g15.tables(pwr)?;
        let max_ps = tables.max;
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
                try_init!(raw::HwDataAG15V14_8_3 {
                    clocks_per_period: v_clocks_per_period,
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
                    avg_power_filter_tc_clks: avg_power_filter_tc_ms_rounded * base_clock_khz,
                    max_pstate_scaled_14: max_ps_scaled,
                    t81xx_data <- Self::t81xx_data(cfg, dyncfg),
                    fast_die0_sensor_mask_2: U64(cfg.fast_sensor_mask[0]),
                    unk_e24: cfg.da.unk_e24,
                    unk_e28: 1,
                    fast_die0_sensor_mask_alt: U64(cfg.fast_sensor_mask_alt[0]),
                    unk_163c: 1,
                    unk_3644: 0,
                    ..Zeroable::init_zeroed()
                })
                .chain(move |raw| {
                    for i in 0..tables.len {
                        raw.sram_k[i] = cfg.sram_k;
                    }

                    // The only constants the G15 firmware expects in the HwDataA 0x10b4 block
                    // (+0x10f4 = 0x32, +0x11cc = {0, 65536.0}).
                    // unk_e10_0 sits at 0x10b4 (asserted in fw/initdata.rs), so +0x40 is 0x10f4
                    // and +0x11c is 0x11d0. The runtime layout fills the whole block below.
                    {
                        raw.unk_e10_0.unk_40 = 0x32;
                        raw.unk_e10_0.unk_11c = f32!(65536.0);
                    }

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
                        raw.power_zones[i].unk_10 = 1320000000;
                    }

                    for (i, j) in raw.hws2.g14.curve2.t1.iter().enumerate() {
                        raw.unk_hws2[i] = if *j == 0xffff { 0 } else { j / 2 };
                    }

                    if runtime {
                        Self::hwdata_a_runtime(raw, pwr, max_ps_scaled, clocks_per_period_coarse);
                    }

                    Ok(())
                }))
    }


    /// The runtime backend's HwDataA words that the manager layout leaves at their defaults: the
    /// shader-engine controller block at +0x10b4 in its G15 order (all inputs from the device
    /// tree), and the leakage coefficient and flag words of the G15 tail. Words that the
    /// runtime's InitData keeps at zero are cleared.
    fn hwdata_a_runtime(
        raw: &mut raw::HwDataAG15V14_8_3,
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


    // Build the G15 (14.8.3) HwDataB contents without allocating the object. Board values whose
    // source is not known yet are left 0 and marked TODO.
    // (Plain comments: the versions macro needs `fn` right after a gating #[ver].)
    fn hwdata_b_init(
        cfg: &'static hw::HwConfig,
        dyncfg: &'a hw::DynConfig,
        g15: &G15Options,
    ) -> Result<impl Init<raw::HwDataBG15V14_8_3, Error> + 'a> {
        let pwr = &dyncfg.pwr;
        let tables = g15.tables(pwr)?;
        let runtime = g15.layout == G15Layout::Runtime;
        let runtime_hwdata_b = g15.runtime_hwdata_b;
        let timestamp_base = match g15.timestamp_base {
            Some(base) => base,
            // The runtime always names its timestamp arena.
            None => return Err(EINVAL),
        };
        Ok(try_init!(raw::HwDataBG15V14_8_3 {
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
            unk_30: U64(IOVA_UNK_PAGE),
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
    fn hwdata_b_runtime(raw: &mut raw::HwDataBG15V14_8_3, words: &G15RuntimeHwDataB) {
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


    // Build the G15 (14.8.3) Globals contents without allocating the object. Only the fields
    // with a known init value are set; the rest start at 0.
    fn globals_init(
        cfg: &'static hw::HwConfig,
        dyncfg: &'a hw::DynConfig,
        g15: &G15Options,
    ) -> Result<impl Init<raw::GlobalsG15V14_8_3, Error> + 'a> {
        let pwr = &dyncfg.pwr;
        let tables = g15.tables(pwr)?;
        let runtime = g15.layout == G15Layout::Runtime;
        let reference_ppm = g15.reference_ppm;
        Ok(try_init!(raw::GlobalsG15V14_8_3 {
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
    fn globals_power_targets(raw: &mut raw::GlobalsG15V14_8_3, pwr: &hw::PwrConfig, tables: &PStateTables) {
        let max_ps_scaled = 100 * tables.max;
        raw.power_if_target = Array::new([pwr.max_power_mw, max_ps_scaled, max_ps_scaled]);
        raw.perf_state_cap = max_ps_scaled;
    }

    // The runtime backend's Globals: DRAM rings with finite idle-off, so the idle policy
    // (imported by DeviceControl 0x13) and smart idle-off are on and the device-tree
    // idle-off delays apply; the power interface targets and the performance-state cap
    // (maximum power from the device tree, pstates scaled by 100); the smart idle-off
    // configuration (standby timer from the device tree); and the FwUtil tunables.
    fn globals_runtime(
        raw: &mut raw::GlobalsG15V14_8_3,
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



    // Fill the G15 P0 status block. Host-initialised words are 0 except +0x45c4, which arms the
    // firmware work-scheduler callbacks, and in the runtime layout the noise-suppression idle
    // power-off state (+0x4), which the runtime's InitData starts at 1.
    fn status_block_fill(raw: &mut raw::StatusBlock, g15: &G15Options) {
        raw.flags.sched_callbacks = 1;
        if g15.layout == G15Layout::Runtime {
            raw.noise_suppression_idle_pwroff = 1;
        }
    }


    // Build the G15 P1 power-controller block without allocating it. The fields keep the order
    // and the formulas of the G13 13.5 Globals power fields (inferred).
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
