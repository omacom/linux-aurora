// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! The initialization graph of the 25G83 firmware build.
//!
//! One coprocessor carries both roles: there is one main configuration, with the
//! secondary-only words, and one merged root. The cluster, the hardware data, the power
//! configuration, region C, the views and the firmware area follow the 25G83 layouts
//! ([`abi::v25g83`]); free lists, pools, descriptor tables, rings and regions are the A18 Pro's.

use super::{abi, low, BuildEnv, BundleObjects, CpuMap, DescriptorTables, InitData, KernelObject};
use crate::g17::{Config, Role};
use crate::mmu;
use abi::v25g83 as v;
use core::sync::atomic::{fence, Ordering};
use kernel::{c_str, prelude::*};

/// Object indices of the 25G83-specific objects.
struct Objects {
    views: [usize; 5],
    firmware_area: usize,
    power_state: usize,
    aon: usize,
}

impl InitData {
    /// Constructs the 25G83 graph after the firmware has published its root entries.
    pub(super) fn build_25g83(
        &mut self,
        env: &BuildEnv<'_>,
        cfg: &Config,
        metrics: &KernelObject,
        active_cores: u32,
    ) -> Result {
        self.status_a = v::private::STATUS_A;
        self.status_b = v::private::STATUS_B;
        self.map_register_windows(env, cfg.windows)?;
        let tables = self.build_descriptor_tables(env, v::private::SIZE)?;
        self.build_render_free_lists(env)?;
        let first_block = self.build_render_pool(env)?;
        self.build_compute_pools(env, first_block)?;
        let bundle = self.build_bundle(env, cfg, metrics)?;
        let objects = self.build_25g83_objects(env)?;
        self.views = Some(objects.views);
        self.initialize_25g83(env, cfg, active_cores, &tables, &bundle, &objects)?;
        if cfg.uat_quirks.clean_tables {
            // Write back every host-built byte before the firmware may read it.
            for object in self.objects.iter_mut() {
                let bytes = object.bytes_mut();
                crate::pgtable::clean_poc(bytes.as_ptr(), bytes.len());
            }
            crate::pgtable::poc_barrier();
        }
        Ok(())
    }

    /// The views, the firmware area, the power-state object and the AON table.
    fn build_25g83_objects(&mut self, env: &BuildEnv<'_>) -> Result<Objects> {
        use mmu::{PROT_FW_PRIV_RW as FP, PROT_FW_SHARED_RW as FS};
        use CpuMap::WriteCombined as WC;
        let mut views = [0; 5];
        for (view, offset) in views.iter_mut().zip(v::kernel_window::VIEWS) {
            *view = self.add(env.fixed(offset, mmu::UAT_PGSZ, FP, WC)?)?;
        }
        let firmware_area =
            self.add(env.fixed(v::kernel_window::FIRMWARE_AREA, mmu::UAT_PGSZ, FP, WC)?)?;
        let power_state =
            self.add(env.fixed(v::kernel_window::POWER_STATE, mmu::UAT_PGSZ, FS, WC)?)?;
        let aon = self.add(env.fixed(v::kernel_window::AON_TABLE, mmu::UAT_PGSZ, FP, WC)?)?;
        Ok(Objects {
            views,
            firmware_area,
            power_state,
            aon,
        })
    }

    /// Writes the 25G83 firmware objects, checks them against the expected contents, then
    /// stages the opening control-ring state.
    fn initialize_25g83(
        &mut self,
        env: &BuildEnv<'_>,
        cfg: &Config,
        active_cores: u32,
        tables: &DescriptorTables,
        bundle: &BundleObjects,
        objects: &Objects,
    ) -> Result {
        let bundle_va = self.object(self.bundle)?.gpu_va();
        let cluster_va = self.object(self.cluster)?.gpu_va();
        let config_va = cluster_va + v::private::CONFIG as u64;
        let status_a_va = cluster_va + v::private::STATUS_A as u64;
        let status_b_va = cluster_va + v::private::STATUS_B as u64;
        let aon_va = self.object(objects.aon)?.gpu_va();
        let power_state_va = self.object(objects.power_state)?.gpu_va();
        let firmware_area_va = self.object(objects.firmware_area)?.gpu_va();
        let mut view_va = [0; 5];
        for (va, index) in view_va.iter_mut().zip(objects.views) {
            *va = self.object(index)?.gpu_va();
        }

        let opps = Self::opps(env.dev, cfg.opps, env.clusters)?;
        let cs_opps = Self::opp_table(env.dev, c_str!("apple,cs-opp"))?;
        let afr_opps = Self::opp_table(env.dev, c_str!("apple,afr-opp"))?;
        let hw_args = abi::HwDataArgs {
            chip_id: cfg.chip_id,
            windows: cfg.windows,
            reserved_slots: cfg.reserved_slots,
            opps: &opps,
            perf: cfg.perf,
            completion_rings: bundle.completion_rings,
            qos_va: self.object(bundle.qos)?.gpu_va(),
        };
        let hw_args_25g83 = v::HwDataArgs25G83 {
            active_cores,
            cs_opps: &cs_opps,
            afr_opps: &afr_opps,
        };
        // SAFETY: Plain C entropy source.
        let entropy = unsafe { kernel::bindings::get_random_u32() };
        let mut hashes = [0; 3];
        self.object_mut(self.bundle)?
            .initialize::<v::HwData25G83>(0, |hw| {
                hw.init(&hw_args, &hw_args_25g83)?;
                hashes[0] = hw.hash();
                Ok(())
            })?;
        self.object_mut(self.bundle)?
            .initialize::<v::PowerConfig25G83>(v::POWER_CONFIG, |power| {
                power.init(config_va, aon_va, entropy);
                hashes[1] = power.hash();
                Ok(())
            })?;
        self.object_mut(self.region_c)?
            .initialize::<abi::RegionC>(0, |region| {
                region.init(cfg.region_c);
                Ok(())
            })?;
        hashes[2] = v::hash_without(self.object_mut(self.region_c)?.bytes_mut(), &[]);
        // The constant contents must equal what the Gravity Linux encoders produce on the proven
        // setup; the operating points and the core count come from the device and are not hashed.
        let expected = [
            v::expected::HW_DATA,
            v::expected::POWER_CONFIG,
            v::expected::REGION_C,
        ];
        if hashes != expected {
            dev_err!(
                env.dev.as_ref(),
                "25G83 initialization data differs: {:#x?}, expected {:#x?}\n",
                hashes,
                expected
            );
            return Err(EINVAL);
        }
        for (index, word) in v::AON_TABLE.iter().enumerate() {
            self.object_mut(objects.aon)?.write(index * 4, *word)?;
        }

        // Status A and B: only the boot words and the firmware-control pointers.
        self.object_mut(self.cluster)?
            .write(v::private::STATUS_A, abi::StatusBlock::new())?;
        self.object_mut(self.cluster)?
            .write(v::private::STATUS_B, abi::StatusBlock::new())?;
        // Status B keeps the A18 Pro's field offsets up to its recovery words.
        let fwctl_va = self.object(tables.fwctl)?.gpu_va();
        let fwctl =
            v::private::STATUS_B + core::mem::offset_of!(abi::PrimaryStatusB, fwctl_state_va);
        let cluster = self.object(self.cluster)?;
        cluster.dword(fwctl)?.store(fwctl_va, Ordering::Relaxed);
        cluster.dword(fwctl + 8)?.store(fwctl_va + abi::FWCTL_RING, Ordering::Relaxed);

        // One main configuration carrying both roles.
        let main_offset = abi::bundle::MAIN_CONFIG[Role::Primary as usize];
        let main_va = bundle_va + main_offset as u64;
        let args = abi::MainConfigArgs {
            bundle_va,
            main_config_va: main_va,
            state_grid_va: cluster_va + self.state_offset(Role::Primary) as u64,
            status_a_va,
            pb_descriptors_low_va: low::PB_DESCRIPTORS,
            page_pool_descriptors_low_va: low::PAGE_POOL_DESCRIPTORS,
        };
        let views = abi::PrimaryViews {
            zero_page_va: self.object(bundle.zero)?.gpu_va(),
            pb_descriptors_va: self.object(tables.pb)?.gpu_va(),
            page_pool_descriptors_va: self.object(tables.page_pool)?.gpu_va(),
        };
        self.object_mut(self.bundle)?
            .initialize::<abi::MainConfig>(main_offset, |main| {
                main.init_primary(&args, &views);
                main.view_va = view_va;
                main.firmware_area_va = [firmware_area_va; 2];
                main.unk_18 = (cluster_va + v::private::MAIN_HEADER as u64).to_le_bytes();
                // The secondary role's words, on the single instance.
                main.unk_300 = 4;
                main.power_config_va = bundle_va + v::POWER_CONFIG as u64;
                main.unk_32c = 0;
                main.unk_344 = 0;
                Ok(())
            })?;
        self.object_mut(self.bundle)?.write(
            main_offset + abi::MainConfig::CONTROL_RING,
            abi::OpeningRecord::new(abi::Role::Primary),
        )?;

        // The merged root.
        let root = abi::RootArgs {
            region_a_va: self.object(bundle.region_a)?.gpu_va(),
            region_c_va: self.object(self.region_c)?.gpu_va(),
            main_config_va: main_va,
            status_a_va,
            ias: cfg.uat_ias,
            oas: cfg.uat_oas,
        };
        self.object_mut(self.roots)?.write(
            0,
            abi::SecondaryRoot::merged(&root, status_b_va, power_state_va, config_va),
        )?;
        for &(offset, value, repeat) in v::POWER_STATE_WORDS {
            for index in 0..repeat {
                self.object_mut(objects.power_state)?.write(offset + 4 * index, value)?;
            }
        }

        // The opening control record is staged in slot 0.
        for (offset, value) in [
            (0, 0),
            (0x10, 0),
            (abi::ChannelState::PRODUCER, abi::OPENING_PRODUCER),
        ] {
            self.object(self.cluster)?
                .word(self.state_offset(Role::Primary) + abi::CONTROL_STATE_OFFSET + offset)?
                .store(value, Ordering::Relaxed);
        }
        fence(Ordering::SeqCst);
        Ok(())
    }
}
