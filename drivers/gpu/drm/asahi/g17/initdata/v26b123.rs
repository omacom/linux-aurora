// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! T6041 26B123 initialization graph. Legacy constructors supply common values
//! in unpublished storage; the 26B123 views are built into distinct allocations
//! before the root is published, and the session retains every allocation until
//! ASC stop.

use super::{abi, low, BuildEnv, BundleObjects, CpuMap, InitData};
use crate::{
    g17::{Config, Role},
    mmu,
};
use abi::{v25g83 as old, v26b123 as v};
use core::sync::atomic::Ordering;
use kernel::prelude::*;

// Relative-performance calibration observed on native 40-core T6041 26B123 boots.
const RELATIVE_A: [u32; 11] = [
    0, 0x41219d11, 0x4194a46c, 0x41cd5835, 0x41f9b2d3, 0x42220d2d, 0x424ab13c, 0x4279cf40,
    0x428cb17b, 0x429da759, 0x42c80000,
];
const RELATIVE_B: [u32; 11] = [
    0, 0, 0x41b4a529, 0x4213bdf0, 0x423d0842, 0x42679ce6, 0x42882108, 0x429d18c6, 0x42a81085,
    0x42b694a5, 0x42c80000,
];

impl InitData {
    #[expect(clippy::too_many_arguments, reason = "Explicit retained graph objects")]
    pub(super) fn upgrade_26b123(
        &mut self,
        env: &BuildEnv<'_>,
        cfg: &Config,
        active_cores: u32,
        bundle: &BundleObjects,
        aon: usize,
        power_state: usize,
        firmware_area: usize,
        views: [usize; 5],
    ) -> Result {
        use mmu::PROT_FW_PRIV_RW as FP;
        use mmu::PROT_FW_SHARED_RW as FS;
        use CpuMap::WriteCombined as WC;

        // Both 32-core and 40-core T6041 parts have four GPU clusters (g0TR..g3TR);
        // core counts, AMCC windows, OPPs, and power limits come from `cfg` via
        // `initialize_25g83()` and `Gpu::new()`'s core-layout check.
        if cfg.chip_id != crate::hw::t6041::CHIP_ID || env.clusters != 4 {
            return Err(EINVAL);
        }
        let chip = cfg.chip.ok_or(EINVAL)?;
        let bundle_va = self.object(self.bundle)?.gpu_va();
        let cluster_va = self.object(self.cluster)?.gpu_va();
        let config_va = cluster_va + old::private::CONFIG as u64;
        let aon_va = self.object(aon)?.gpu_va();
        // The existing register window already maps the complete GPTBAT span
        // with firmware MMIO permissions; reuse its first 0x8000 bytes.
        let gptbat = cfg
            .windows
            .iter()
            .find(|window| window.phys == v::GPTBAT_PHYS && window.size >= v::GPTBAT_SIZE)
            .ok_or(EINVAL)?;
        let auxiliary = self.add(env.dynamic(mmu::UAT_PGSZ, mmu::UAT_PGSZ as u64, FP, WC)?)?;
        let hardware = self.add(env.dynamic(mmu::UAT_PGSZ, mmu::UAT_PGSZ as u64, FP, WC)?)?;
        let power = self.add(env.dynamic(0x8000, mmu::UAT_PGSZ as u64, FP, WC)?)?;
        let region = self.add(env.dynamic(mmu::UAT_PGSZ, mmu::UAT_PGSZ as u64, FS, WC)?)?;

        // SAFETY: RegisterTable consists of initialized integer descriptors only.
        let mut registers: v::RegisterTable = unsafe { core::mem::zeroed() };
        registers.init(cfg.windows)?;
        let args = v::HardwareArgs {
            gptbat_va: gptbat.va,
            gptbat_size: v::GPTBAT_SIZE,
            completion_low: bundle.completion_rings.map(|ring| ring.low_va),
            completion_high: bundle.completion_rings.map(|ring| ring.high_va),
            qos_va: self.object(bundle.qos)?.gpu_va(),
            auxiliary_va: self.object(auxiliary)?.gpu_va(),
            relative_words: (active_cores == 40).then_some((RELATIVE_A, RELATIVE_B)),
        };
        let legacy_hw = self
            .object(self.bundle)?
            .pointer(0, size_of::<old::HwData25G83>())?
            .cast::<old::HwData25G83>();
        self.object_mut(hardware)?
            .initialize::<v::HardwareData>(0, |hw| {
                // SAFETY: This checked, initialized legacy view remains in a distinct
                // retained allocation. No graph has been published to the firmware.
                hw.init(unsafe { &*legacy_hw }, &registers, &args)
            })?;
        let startup_word = self
            .object(self.bundle)?
            .word(old::POWER_CONFIG + 0x63d4)?
            .load(Ordering::Relaxed);
        let legacy_power = self
            .object(self.bundle)?
            .pointer(old::POWER_CONFIG, size_of::<old::PowerConfig25G83>())?
            .cast::<old::PowerConfig25G83>();
        self.object_mut(power)?
            .initialize::<v::PowerConfig>(0, |power| {
                // SAFETY: The initialized legacy power view is in the distinct bundle.
                power.init(
                    unsafe { &*legacy_power },
                    &v::PowerArgs {
                        config_va,
                        aon_va,
                        startup_word,
                    },
                )
            })?;
        let legacy_region = self
            .object(self.region_c)?
            .pointer(0, size_of::<abi::RegionC>())?
            .cast::<abi::RegionC>();
        self.object_mut(region)?
            .initialize::<v::RegionCPrefix>(0, |region| {
                // SAFETY: The old region remains retained and is distinct from `region`.
                region.init(unsafe { &*legacy_region });
                Ok(())
            })?;
        self.region_c = region;
        self.region_c_shift = 4;
        self.object_mut(power_state)?
            .initialize::<v::PowerState>(0, |state| {
                state.init(chip.max_power_mw);
                Ok(())
            })?;
        self.object_mut(aon)?
            .initialize::<v::AonTable>(0, |table| {
                table.init(&[
                    v::AonSensor {
                        key: *b"g0TR",
                        selector: 0x00c20001,
                        config: 0x1401,
                    },
                    v::AonSensor {
                        key: *b"g1TR",
                        selector: 0x00970001,
                        config: 0x1401,
                    },
                    v::AonSensor {
                        key: *b"g2TR",
                        selector: 0x00920001,
                        config: 0x1401,
                    },
                    v::AonSensor {
                        key: *b"g3TR",
                        selector: 0x00c70001,
                        config: 0x1401,
                    },
                    v::AonSensor {
                        key: *b"r0TR",
                        selector: 0x012c0001,
                        config: 0x1401,
                    },
                ])
            })?;

        let main_offset = abi::bundle::MAIN_CONFIG[Role::Primary as usize];
        let main_va = bundle_va + main_offset as u64;
        let common = abi::MainConfigArgs {
            bundle_va: self.object(hardware)?.gpu_va(),
            main_config_va: main_va,
            state_grid_va: cluster_va + self.state_offset(Role::Primary) as u64,
            status_a_va: cluster_va + old::private::STATUS_A as u64,
            pb_descriptors_low_va: low::PB_DESCRIPTORS,
            page_pool_descriptors_low_va: low::PAGE_POOL_DESCRIPTORS,
        };
        // Preserve the already constructed host-owned descriptor addresses.
        let old_main = self
            .object(self.bundle)?
            .pointer(main_offset, size_of::<abi::MainConfig>())?
            .cast::<abi::MainConfig>();
        // SAFETY: The bundle view is initialized and not yet published. Copy only
        // integer values before initialize clears that same view below.
        let primary_views = unsafe {
            abi::PrimaryViews {
                zero_page_va: (*old_main).zero_page_va,
                pb_descriptors_va: (*old_main).pb_descriptors_va,
                page_pool_descriptors_va: (*old_main).page_pool_descriptors_va,
            }
        };
        let mut view_va = [0; 5];
        for (va, index) in view_va.iter_mut().zip(views) {
            *va = self.object(index)?.gpu_va();
        }
        let firmware_area_va = self.object(firmware_area)?.gpu_va();
        let main_args = v::MainArgs {
            common: &common,
            primary_views: &primary_views,
            view_va,
            firmware_area_va: [firmware_area_va; 2],
            work_rings_va: bundle_va + abi::bundle::WORK_RINGS as u64,
            power_config_va: self.object(power)?.gpu_va(),
        };
        self.object_mut(self.bundle)?
            .initialize::<v::MainConfig>(main_offset, |main| main.init_merged(&main_args))?;
        let root = abi::RootArgs {
            region_a_va: self.object(bundle.region_a)?.gpu_va(),
            region_c_va: self.object(self.region_c)?.gpu_va(),
            main_config_va: main_va,
            status_a_va: common.status_a_va,
            ias: cfg.uat_ias,
            oas: cfg.uat_oas,
        };
        let merged = v::merged_root(
            &root,
            cluster_va + old::private::STATUS_B as u64,
            self.object(power_state)?.gpu_va(),
            config_va,
        );
        self.object_mut(self.roots)?.write(0, merged)?;
        Ok(())
    }
}
