// SPDX-License-Identifier: GPL-2.0-only OR MIT

#![allow(dead_code)]

//! Retained RAM for the J873 / T8152 / G18G / 26A428 initialization graph
//! (plan M1, steps 1.1-1.7).
//!
//! Construction only. Nothing here starts a processor, publishes on a
//! mailbox, reads or writes MMIO, requests PMGR/PSD power, touches the ASC
//! wrappers or the post-office bank, or submits work: every write lands in
//! host-allocated GEM memory or in UAT page tables. Register windows are
//! mapped for the firmware (page-table writes only); their contents are never
//! accessed from the host.
//!
//! The G18 live-boot session owns this object. It must build the UAT with
//! `mmu::Uat::new_t8152` (plan step 0.6) before calling
//! [`G18FirmwareMemory::new_with_uat`], and must retain both the UAT and this
//! owner until BOTH firmware processors are stopped, including on failure.
//!
//! Derived from `m5_resources.rs` with the M2/M3 compute and graphics owners
//! removed (they depend on the unported HAL400 submission ABI). The firmware
//! VA plan is the J813 one: the addresses are host-chosen and both shared
//! allocations keep their 25G76 page totals (0x28000 / 0x178000) in 26A428.
//! Every object inside the allocations follows the 26A428 kext layout from
//! `g17_initdata` (`G18G_26A428_*`). Values that are not proven for 26A428
//! are listed in `g17_initdata::G18G_26A428_UNPROVEN`.
//!
//! Integration (outside this file): include it from `g17_resources.rs` next
//! to the M5 module, so it can use the private `MappedObject`:
//!
//! ```ignore
//! #[cfg(not(test))]
//! #[path = "g18_resources.rs"]
//! pub(crate) mod g18;
//! ```

#[path = "application.rs"]
mod application;
#[path = "application_compute.rs"]
mod application_compute;
use application_compute::ApplicationCompute;

use crate::g18::buffer::object::MappedObject;
use crate::g18::buffer::pool::{G18UmaPoolBinding, G18UmaPoolMemory};
use crate::g18::queue::scheduler::G18SchedulerMemory;
use crate::g18::runtime::compute::{G18ComputeMemory, G18PreparedComputeCommand};
use crate::g18::runtime::compute_payload::G18ComputePayloadMemory;
use crate::{driver::AsahiDrmDevice, g18::fw::initdata as init, mmu};
use core::sync::atomic::{fence, Ordering};
use kernel::prelude::*;
use kernel::sync::Arc;


const KERNEL_BASE: u64 = 0xffff_fc20_0000_0000;
/// Shared allocation #2 (the private cluster, 0x178000 bytes).
const PRIVATE_BASE: u64 = KERNEL_BASE + 0x2_0000;
const REGION_C_BASE: u64 = KERNEL_BASE + 0x1a_0000;
/// The secondary root sits [`init::SECONDARY_ROOT_DELTA`] above the primary.
const ROOTS_BASE: u64 = KERNEL_BASE + 0x1a_8000;
const REGION_A_BASE: u64 = KERNEL_BASE + 0x1b_8000;
/// ArmFirmware fwctl allocation: 0x20-byte state, then the ring at +0x40.
const FWCTL_BASE: u64 = KERNEL_BASE + 0x1c_0000;
const FWCTL_SIZE: usize = 0x4000;
/// Shared allocation #1 (hw_data, policy blocks, rings, main configs).
const SHARED_BASE: u64 = KERNEL_BASE + 0xc078_8000;
/// Main `+0x2d0` blank sentinel; unmapped at handoff, as on J813.
const SENTINEL_BASE: u64 = KERNEL_BASE + 0xc07e_0000;
/// PB descriptor and UMA pool tables, aliased at [`init::REGION_VIEW_LOW_ADDRS`].
const PBDESC_BASE: u64 = KERNEL_BASE + 0x15e_0000;
const UMA_BASE: u64 = KERNEL_BASE + 0x15e_8000;
const PBDESC_TABLE_SIZE: usize = 0x4000;
/// 26A428 createHardwareMappings at 0xfffffe0008c41978 requests 0x8000
/// bytes: 256 native 0x80-byte UMA descriptors, rather than the M5 page.
const UMA_TABLE_SIZE: usize = 0x8000;
/// The four KSM completion queues (low context-0 alias / firmware-high).
const COMPLETION_LOW_BASE: u64 = 0x70_0184_8000;
const COMPLETION_HIGH_BASE: u64 = KERNEL_BASE + 0xc07e_8000;
const COMPLETION_STRIDE: u64 = 0x2_8000;
/// Allocator-selected objects (QoS block, AON PTD) stay between the PB/UMA
/// tables and the fixed high graph.
const DYNAMIC_START: u64 = KERNEL_BASE + 0x168_0000;
const DYNAMIC_END: u64 = KERNEL_BASE + 0x1000_0000;

const _: () = {
    assert!(PBDESC_BASE + PBDESC_TABLE_SIZE as u64 <= UMA_BASE);
    assert!(UMA_BASE + UMA_TABLE_SIZE as u64 <= DYNAMIC_START);
    assert!(init::REGION_VIEW_LOW_ADDRS[0] + PBDESC_TABLE_SIZE as u64
        <= init::REGION_VIEW_LOW_ADDRS[1]);
    assert!(init::REGION_VIEW_LOW_ADDRS[1] + UMA_TABLE_SIZE as u64 <= COMPLETION_LOW_BASE);
};

/// Report rings keep their 25G76 shape (FW_PROVEN: event producer b000
/// 0x7f14, 256 x 0x48; trace producer b000 0x19984, 512 x 0x48; host owns the
/// cursor at +0, firmware the producer at +0x20).
const REPORT_RECORD_SIZE: usize = 0x48;
const REPORT_PRODUCER_OFFSET: u64 = 0x20;
const REPORT_CAPACITY: [u32; 2] = [256, 512];
const REPORT_DRAIN_BUDGET: usize = 128;
const TRACE_PRINT_BUDGET: [u32; 2] = [512, 16];
/// Host-only storage; do not change the native firmware ring geometry.
/// Roughly 5 MiB for both roles, allocated before publication.
const TRACE_JOURNAL_CAPACITY: usize = 32768;
/// b000 0x7de8 constructs type4 with counter at +4 and stamp index at +12.
const EVENT_TYPE_RECOVERY: u32 = 4;
/// Parameter-buffer (TVB) grow request in the 25G76 TvbGrowEventRecord
/// layout (g17::fw::channels). The tiler waits for the reply.
const EVENT_TYPE_TVB_GROW: u32 = 6;
/// Device-control TvbGrowReply (g17 DeviceControlOp::TvbGrowReply).
const DEVICE_CONTROL_OPCODE_TVB_GROW_REPLY: u32 = 0x08;
/// Grow requests held between a drain and the session answering them. More
/// stay in the event ring until a later drain.
const TVB_GROW_QUEUE: usize = 4;

/// `halLoadUMADesc` device words (accelerator+0x11af8..0x11b10) as native
/// `halInitSMAEnableMask` 0xfffffe0008c41bdc derives them on J873.
/// getProbeScore stores readChipInfo's 0xe01500 core-enable word as the
/// accelerator+0x480 fallback (0xfffffe0008c3d6c4, 0xfffffe0008c002d8); the
/// +0x470 override stays zero without gpu-fstp-downgrade properties, which the
/// J873 ADT lacks. Width +0x4ec is ID_COUNTS_1[7:0] (0xfffffe0008c3d554) and
/// +0x4d4 the cluster count; generateRegisterList reads the same two fields,
/// so they must equal the command topology. getPropertyCoreConfigImpl splits
/// the same selected mask into the native registry core_mask_list, so every
/// 16-bit lane must equal that list (zero beyond it).
fn j873_sma_device_words(clusters: u32, width: u32) -> Result<[u64; 4]> {
    let facts = crate::g18::hw::T8152_REGISTRY;
    if clusters != facts.num_mgpus || facts.core_mask_list.len() != clusters as usize
        || !(1..=16).contains(&width) || width.checked_mul(clusters) != Some(facts.num_cores) {
        return Err(EINVAL);
    }
    let mut mask = 0u64;
    for (index, &cores) in facts.core_mask_list.iter().enumerate() {
        if u64::from(cores) >> width != 0 { return Err(EINVAL); }
        mask |= u64::from(cores) << (index as u32 * width);
    }
    let words = crate::g18::fw::uma::pack_g18_sma_masks([0; 2], [mask, 0], width).ok_or(EINVAL)?;
    for group in 0..16 {
        let lane = (words[group / 4] >> ((group % 4) * 16)) & 0xffff;
        if lane != facts.core_mask_list.get(group).map_or(0, |&cores| u64::from(cores)) {
            return Err(EINVAL);
        }
    }
    Ok(words)
}

struct G18TraceRecord {
    ordinal: u32,
    raw: [u8; REPORT_RECORD_SIZE],
}

/// Owning mappings are declared before the UAT so they drop first. Aliases
/// retain the SAME GEM backing; no firmware pointer names a copied snapshot.
pub(crate) struct G18FirmwareMemory {
    primary_root: MappedObject,
    secondary_root: MappedObject,
    shared: MappedObject,
    private: MappedObject,
    region_a: MappedObject,
    region_c: MappedObject,
    fwctl: MappedObject,
    aon: MappedObject,
    qos: MappedObject,
    pbdesc: MappedObject,
    uma: MappedObject,
    uma_pool: G18UmaPoolMemory,
    scheduler: G18SchedulerMemory,
    compute: G18ComputeMemory,
    compute_payload: Option<G18ComputePayloadMemory>,
    application_compute: Option<KBox<ApplicationCompute>>,
    completions: KVec<MappedObject>,
    aliases: KVec<mmu::KernelMapping>,
    registers: KVec<mmu::KernelMapping>,
    report_counts: [[u32; 2]; 2],
    completion_masks: [u64; 2],
    trace_printed: [u32; 2],
    trace_journal: [KVVec<G18TraceRecord>; 2],
    trace_dropped: [u32; 2],
    /// TVB grow requests drained but not yet answered, oldest first.
    tvb_grow: KVec<G18TvbGrowRequest>,
    /// Primary device-control records published after the idle wake.
    control_published: u32,
    uat: Arc<mmu::Uat>,
    /// InitData layouts of the firmware build the loader admitted.
    layouts: &'static init::G18GLayouts,
}

/// Read-back of the new 26A428 status cross-links (plan step 1.3 validation):
/// status B +0xe714 header, +0xe71c word and +0xe720 pointer; status config
/// +0x6c4 word and +0x6c8 pointer.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) struct G18StatusLinks {
    pub(crate) status_b_header: u32,
    pub(crate) status_b_word: u32,
    pub(crate) status_b_pointer: u64,
    pub(crate) status_config_word: u32,
    pub(crate) status_config_pointer: u64,
}

/// A firmware TVB grow request (event 6): vm_slot +4, buffer_slot +8,
/// counter +0xc, subpipe +0x10, u64 halt count (the recovery epoch) +0x14.
/// The reply echoes every field.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) struct G18TvbGrowRequest {
    pub(crate) vm_slot: u32,
    pub(crate) buffer_slot: u32,
    pub(crate) counter: u32,
    pub(crate) subpipe: u32,
    pub(crate) halt_count: u64,
}

/// The status-B recovery handshake words.
#[derive(Debug, Copy, Clone, Default)]
pub(crate) struct G18RecoveryWord {
    pub(crate) epoch: u64,
    pub(crate) state: u32,
    pub(crate) host: u32,
}

/// What the firmware published about one recovery: the status-B cause words,
/// the valid recovery-info entries, and the shared recovery-request block.
#[derive(Debug, Copy, Clone, Default)]
pub(crate) struct G18RecoveryCause {
    pub(crate) emitted: u32,
    pub(crate) source_count: u32,
    pub(crate) command: u32,
    pub(crate) reason: u32,
    pub(crate) detail: u64,
    pub(crate) slot_mask: u64,
    pub(crate) valid_entries: u32,
    pub(crate) first_entry: Option<(u32, u64)>,
    /// The first valid entries as (index, raw). Upstream's 13.5 layout for
    /// the same 256 x 8 table (`pending_stamps`) is info u32 (bit 0 valid,
    /// flags 3:0, stamp slot 10:4) then the awaited stamp value u32.
    pub(crate) pending: [(u16, u64); G18_RECOVERY_PENDING_LOGGED],
    pub(crate) request: [u32; init::G18G_26A428_RECOVERY_REQUEST_WORDS],
}

pub(crate) const G18_RECOVERY_PENDING_LOGGED: usize = 8;

impl G18FirmwareMemory {
    /// Allocate, map and encode the complete G18G init-data graph while both
    /// firmware processors are stopped. All fallible work happens before the
    /// caller publishes anything. The caller keeps its own `Arc` of the UAT
    /// until both processors stop, including when this construction fails.
    #[inline(never)]
    pub(crate) fn new_with_uat(
        dev: &AsahiDrmDevice,
        uat: Arc<mmu::Uat>,
        boot_timestamp: u64,
        render: bool,
        profile: init::RootProfile,
    ) -> Result<KBox<Self>> {
        if !uat.is_t8152_geometry() {
            return Err(ENODEV);
        }
        let layouts = profile.g18g_layouts().ok_or(EINVAL)?;
        match crate::g18::params::g18_compute_owner_test.value() {
            0 => {},
            1 => G18UmaPoolMemory::verify_shared_compute_owner(dev, &uat)?,
            _ => return Err(EINVAL),
        }
        let trace_journal = [
            KVVec::with_capacity(TRACE_JOURNAL_CAPACITY, GFP_KERNEL)?,
            KVVec::with_capacity(TRACE_JOURNAL_CAPACITY, GFP_KERNEL)?,
        ];
        let tvb_grow = KVec::with_capacity(TVB_GROW_QUEUE, GFP_KERNEL)?;
        let mut registers = KVec::new();
        for window in init::G18G_26A428_REGISTER_WINDOWS {
            let (physical, virtual_address, span) = window.mapped_pages().map_err(|_| EINVAL)?;
            // Flag 2 windows are firmware read-write; flag 0 windows are
            // mapped read-only, the J813 rule (INFERRED for 26A428).
            let protection = match window.flag {
                0 => mmu::PROT_FW_MMIO_RO,
                2 => mmu::PROT_FW_MMIO_RW,
                _ => return Err(EINVAL),
            };
            // Page-table writes only. No register contents are accessed.
            registers.push(
                uat.kernel_vm().map_io(dev.as_ref().as_ref(), virtual_address, physical.try_into()?, span.try_into()?,
                                       protection)?,
                GFP_KERNEL,
            )?;
        }

        let fixed = |address, size, protection| {
            MappedObject::new_wc_at(dev, uat.kernel_vm(), address, size, protection)
        };
        // Install every fixed address before allocator-selected objects.
        let mut private = fixed(PRIVATE_BASE, layouts.private.total,
                                mmu::PROT_FW_SHARED_RW)?;
        // Both 26A428 roots are 0xc8 bytes. Separate mappings keep the empty
        // page between them.
        let mut primary_root = fixed(ROOTS_BASE, init::G18G_26A428_ROOT_SIZE, mmu::PROT_FW_SHARED_RO)?;
        let mut secondary_root = fixed(ROOTS_BASE + init::SECONDARY_ROOT_DELTA,
                                       init::G18G_26A428_ROOT_SIZE, mmu::PROT_FW_SHARED_RO)?;
        let mut shared = fixed(SHARED_BASE, layouts.shared.total, mmu::PROT_FW_PRIV_RW)?;
        let region_a = fixed(REGION_A_BASE, init::REGION_A_SIZE, mmu::PROT_FW_SHARED_RO)?;
        let mut region_c = fixed(REGION_C_BASE, init::REGION_C_SIZE, mmu::PROT_FW_SHARED_RO)?;
        let fwctl = fixed(FWCTL_BASE, FWCTL_SIZE, mmu::PROT_FW_SHARED_RW)?;
        // Zero-backed at handoff; owned by later (M2/M3) publication.
        let mut pbdesc = fixed(PBDESC_BASE, PBDESC_TABLE_SIZE, mmu::PROT_FW_SHARED_RW)?;
        let mut uma = fixed(UMA_BASE, UMA_TABLE_SIZE, mmu::PROT_FW_SHARED_RW)?;
        let mut aliases = KVec::new();
        for (object, address, size) in [
            (&mut pbdesc, init::REGION_VIEW_LOW_ADDRS[0], PBDESC_TABLE_SIZE),
            (&mut uma, init::REGION_VIEW_LOW_ADDRS[1], UMA_TABLE_SIZE),
        ] {
            aliases.push(object.map_alias_at(uat.kernel_lower_vm(), address, mmu::PROT_GPU_SHARED_RW)?,
                         GFP_KERNEL)?;
            for offset in (0..size).step_by(mmu::UAT_PGSZ) {
                if uat.kernel_vm().translate_iova(object.iova() + offset as u64)?
                    != uat.kernel_lower_vm().translate_iova(address + offset as u64)?
                {
                    return Err(EFAULT);
                }
            }
        }
        dev_info!(dev.as_ref(), "G18 UMA descriptor table: bytes={:#x} slots=256 stride=0x80 alias_pages={} verified\n",
            UMA_TABLE_SIZE, UMA_TABLE_SIZE / mmu::UAT_PGSZ);

        let mut completions = KVec::new();
        let mut completion_bindings =
            [init::G17PKsmCompletionAliases { low: 0, high: 0 }; init::M5_25G76_COMPLETION_QUEUE_COUNT];
        for (ordinal, binding) in completion_bindings.iter_mut().enumerate() {
            // Host-chosen pairs, the J813 placement (INFERRED for 26A428).
            let low = COMPLETION_LOW_BASE + ordinal as u64 * COMPLETION_STRIDE;
            let high = COMPLETION_HIGH_BASE + ordinal as u64 * COMPLETION_STRIDE;
            let mut backing = MappedObject::new_wc_at(dev, uat.kernel_vm(), high,
                init::M5_25G76_COMPLETION_BACKING_SIZE, mmu::PROT_FW_PRIV_RW)?;
            aliases.push(backing.map_alias_at(uat.kernel_lower_vm(), low, mmu::PROT_GPU_SHARED_RW)?,
                         GFP_KERNEL)?;
            for offset in (0..init::M5_25G76_COMPLETION_BACKING_SIZE).step_by(mmu::UAT_PGSZ) {
                if uat.kernel_vm().translate_iova(high + offset as u64)?
                    != uat.kernel_lower_vm().translate_iova(low + offset as u64)?
                {
                    return Err(EFAULT);
                }
            }
            *binding = init::G17PKsmCompletionAliases { low, high };
            completions.push(backing, GFP_KERNEL)?;
        }
        let dynamic = DYNAMIC_START..DYNAMIC_END;
        let qos = MappedObject::new_wc_in_range(dev, uat.kernel_vm(), dynamic.clone(),
            init::G17P_QOS_RESOURCE_SIZE, mmu::UAT_PGSZ as u64, mmu::PROT_FW_SHARED_RW)?;
        let mut aon = MappedObject::new_wc_in_range(dev, uat.kernel_vm(), dynamic,
            init::G18G_26A428_AON_PTD_SIZE, mmu::UAT_PGSZ as u64, mmu::PROT_FW_PRIV_RW)?;
        let uma_pool = if render { G18UmaPoolMemory::new_render(dev, &uat)? } else {
            G18UmaPoolMemory::new(dev, &uat, DYNAMIC_START..DYNAMIC_END,
                crate::g18::hw::T8152_REGISTRY.num_cores)?
        };
        let scheduler = G18SchedulerMemory::new(dev, &uat, DYNAMIC_START..DYNAMIC_END)?;
        let facts = crate::g18::hw::T8152_REGISTRY;
        let compute = G18ComputeMemory::new(dev, &uat, DYNAMIC_START..DYNAMIC_END,
            facts.num_mgpus, facts.num_gps)?;

        aon.with_bytes_mut(|out| init::encode_g18g_26a428_aon_ptd(out).map_err(|_| EINVAL))?;
        region_c.with_bytes_mut(|out| {
            init::encode_region_c_for_profile(layouts.profile, out).map_err(|_| EINVAL)
        })?;
        private.with_bytes_mut(|out| {
            init::encode_g18g_26a428_private_cluster(layouts, PRIVATE_BASE, FWCTL_BASE, out)
                .map_err(|_| EINVAL)
        })?;

        let layout = layouts.private;
        let addresses = init::SharedAddresses {
            region_a: region_a.iova(),
            region_c: region_c.iova(),
            hw_data_bundle: SHARED_BASE,
            hw_data_bundle_alloc: layouts.shared.total,
        };
        let primary = init::InstanceAddresses {
            root: ROOTS_BASE,
            main_config: SHARED_BASE + layouts.shared.main_config[0] as u64,
            status_a: PRIVATE_BASE + init::G18G_26A428_PRIVATE_STATUS_A[0] as u64,
            status_b: PRIVATE_BASE + layout.status_b as u64,
            secondary_extras: [0, 0],
        };
        let secondary = init::InstanceAddresses {
            root: ROOTS_BASE + init::SECONDARY_ROOT_DELTA,
            main_config: SHARED_BASE + layouts.shared.main_config[1] as u64,
            status_a: PRIVATE_BASE + init::G18G_26A428_PRIVATE_STATUS_A[1] as u64,
            status_b: 0,
            // Secondary root +0xb8 = status config, +0xc0 = the 0x60 object
            // (KEXT_PROVEN, g18g_key.c:3748-3812).
            secondary_extras: [
                PRIVATE_BASE + layout.status_config as u64,
                PRIVATE_BASE + layout.secondary_extra_1 as u64,
            ],
        };
        init::validate_pair(&primary, &secondary).map_err(|_| EINVAL)?;
        let views = init::PrimaryRegionViews {
            sentinel_high: SENTINEL_BASE,
            alias_high: [PBDESC_BASE, UMA_BASE],
        };
        // GFX traces remain disabled here. Stage8 explicitly enables only
        // re-derived b000 command/scheduler classes before root publication.
        shared.with_bytes_mut(|out| {
            init::encode_g18g_26a428_hw_data(completion_bindings, qos.iova(),
                &mut out[..init::G18G_26A428_HW_DATA_SIZE]).map_err(|_| EINVAL)?;
            init::encode_g18g_26a428_bundle_policy(layouts, init::M5PolicyBindings {
                state: PRIVATE_BASE + layout.policy_state as u64,
                aon_ptd: aon.iova(),
                boot_timestamp,
            }, out).map_err(|_| EINVAL)?;
            for (role, instance, views) in [
                (init::InstanceRole::Primary, &primary, Some(&views)),
                (init::InstanceRole::Secondary, &secondary, None),
            ] {
                let grid = PRIVATE_BASE + init::G18G_26A428_PRIVATE_STATE_GRIDS[role as usize] as u64;
                let channels = init::derive_channel_table_for_profile(layouts.profile, role, grid,
                    instance.status_a, instance.main_config, SHARED_BASE);
                let offset = (instance.main_config - SHARED_BASE) as usize;
                init::encode_main_config_for_profile(layouts.profile, role, &addresses, instance,
                    &channels, views, &mut out[offset..offset + init::MAIN_CONFIG_SIZE])
                    .map_err(|_| EINVAL)?;
            }
            Ok(())
        })?;
        for (root, role, instance) in [
            (&mut primary_root, init::InstanceRole::Primary, &primary),
            (&mut secondary_root, init::InstanceRole::Secondary, &secondary),
        ] {
            root.with_bytes_mut(|out| {
                init::encode_root_for_profile(layouts.profile, role, &addresses, instance, out)
                    .map_err(|_| EINVAL)?;
                // The two firmware gates (version word, root+0x2c) must pass
                // before anything is published.
                init::check_firmware_root_gates_for_profile(layouts.profile, out)
                    .map_err(|_| EINVAL)
            })?;
        }
        // Unlike PB/UMA, the blank sentinel is unmapped at native handoff.
        if uat.kernel_vm().translate_iova(SENTINEL_BASE).is_ok() {
            return Err(EINVAL);
        }
        // Writes to all CPU WC mappings precede any future handoff. Only the
        // two opening control records are staged; work rings stay empty.
        fence(Ordering::SeqCst);
        // Keep the retained graph off the kernel stack.
        KBox::init(try_init!(Self {
            primary_root, secondary_root, shared, private, region_a, region_c, fwctl, aon, qos,
            pbdesc, uma, uma_pool, scheduler, compute, compute_payload: None,
            application_compute: None,
            completions, aliases, registers,
            report_counts: [[0; 2]; 2], completion_masks: [0; 2], trace_printed: [0; 2], uat,
            trace_journal, trace_dropped: [0; 2], tvb_grow, control_published: 0, layouts,
        }), GFP_KERNEL)
    }

    pub(crate) fn uma_pool(&self) -> &G18UmaPoolMemory { &self.uma_pool }

    pub(crate) fn prepare_render(&mut self, dev: &AsahiDrmDevice,
        job: &mut crate::g18::runtime::graphics_job::G18GraphicsJob) -> Result<G18UmaPoolBinding> {
        self.uma_pool.prepare_render_backing(dev, &self.uat, job.vm())?;
        let pool = self.uma_pool.prepare_binding()?;
        self.uma_pool.load_hardware_descriptor(dev, &mut self.uma, pool, j873_sma_device_words(2, 6)?)?;
        job.parameter_buffer().load_row(&mut self.pbdesc)?;
        // Without a capture (stage 10 only) the first application frame encodes.
        if !job.layout_only_payload() { job.encode_h1(dev)?; }
        Ok(pool)
    }

    pub(crate) fn audit_render(&mut self, dev: &AsahiDrmDevice,
        job: &mut crate::g18::runtime::graphics_job::G18GraphicsJob) -> Result {
        let pb = job.parameter_buffer().verify_before_kick(&mut self.pbdesc)?;
        if pb.counter != 1 { return Err(EIO); }
        let uma = self.uma_pool.verify_render_descriptor(&mut self.uma, j873_sma_device_words(2, 6)?)?;
        let pages = self.uma_pool.audit_render_mappings(&self.uat, job.vm())?;
        job.audit_h1(dev, &self.uat, &pb, &uma)?;
        let state = self.uma_pool.render_state()?;
        dev_info!(dev.as_ref(), "G18_RENDER_DIGEST name=uma-state fnv={:#018x}\n",
            crate::g18::fw::graphics_payload::fnv64(&state));
        dev_info!(dev.as_ref(), "G18_RENDER_AUDIT_UMA pages={} row=match state-slot=0\n", pages);
        Ok(())
    }

    /// Batched CL_2 publication. Both queue records become visible through one
    /// producer write, after every byte and every counter has been validated.
    /// Used only by stage 9; the existing compute publication stays unchanged.
    pub(crate) fn publish_outer_records(&mut self, records: &[[u8; 24]]) -> Result<u32> {
        if records.len() != 2 || self.control_counters()? != [[2; 3], [1; 3]] { return Err(EINVAL); }
        let before = self.compute_outer_counters()?;
        let head = before[0];
        if before != [head; 3] || head > 6 || head & 1 != 0 { return Err(EBUSY); }
        for (i, record) in records.iter().enumerate() {
            let qid = record[22];
            let address = u64::from_le_bytes(record[8..16].try_into().map_err(|_| EINVAL)?);
            if qid != if i == 0 {10} else {9}
                || address != crate::g18::address_map::GRAPHICS_QUEUE_HIGH + if qid == 9 {0} else {0x100}
                || record[16..20] != 2u32.to_le_bytes() || record[23] != 1 { return Err(EINVAL); }
        }
        self.shared.with_bytes_mut(|out| {
            let start = self.layouts.work_rings[8] + head as usize * 24;
            let slots = out.get_mut(start..start + 48).ok_or(EINVAL)?;
            if slots.iter().any(|&b| b != 0) { return Err(EBUSY); }
            for (out, record) in slots.chunks_exact_mut(24).zip(records) { out.copy_from_slice(record); }
            Ok(())
        })?;
        // No producer moved while the batch was being stored.
        if self.compute_outer_counters()? != before { return Err(EBUSY); }
        fence(Ordering::SeqCst);
        let address = PRIVATE_BASE + (init::G18G_26A428_PRIVATE_STATE_GRIDS[0]
            + init::WORK_STATE_GRID_OFFSETS[8] + 0x20) as u64;
        self.private.write_u32(address, head + 2)?;
        fence(Ordering::SeqCst);
        Ok(head + 2)
    }

    pub(crate) fn scheduler(&mut self) -> &mut G18SchedulerMemory { &mut self.scheduler }

    pub(crate) fn compute(&mut self) -> &mut G18ComputeMemory { &mut self.compute }

    /// Select the firmware-owned KSMFE queue path before root publication.
    /// b000 0x216f8..0x21700 forms x10=global+0x1093e0, then 0x21884
    /// copies hwdata+0x2724 to [x10+0xb7c] (global+0x109f5c);
    /// 0x12cdc..0x12de8 skips tag15 hardware queue setup when it is nonzero.
    /// Native macOS uses host queue programming. This first Linux submission
    /// sends tag15/tag14 and performs no host KSMFE MMIO, so choose one owner.
    pub(crate) fn use_firmware_queue_programming(&mut self, dev: &AsahiDrmDevice) -> Result {
        let policy = SHARED_BASE + 0x2724;
        let trace = PRIVATE_BASE + init::G18G_26A428_PRIVATE_STATUS_A[0] as u64;
        let progress = REGION_C_BASE + 0x9ac;
        if self.shared.read_u32(policy)? != 1 || self.private.read_u32(trace)? != 0
            || self.region_c.read_u32(progress)? != 0x333 {
            return Err(EINVAL);
        }
        self.shared.write_u32(policy, 0)?;
        // b000 0x216b8 obtains GFX status-A; 0x3508 loads its first word
        // into global 0x1093e4. Bit0 gates commands/completion; bit1 gates
        // scheduling/recovery. audit-g18-trace-mask-native.py executes these
        // exact branches. Opt-in graphics diagnosis additionally enables
        // bit9 (global 0x1093e5 bit1), the native stall diagnostic category.
        // audit-g18-graphics-stall.py executes that original predicate.
        let graphics_stall_trace = crate::g18::params::g18_graphics_stall_trace.value() == 1;
        let trace_mask = if graphics_stall_trace { 0x203 } else { 3 };
        self.private.write_u32(trace, trace_mask)?;
        // b000 refresh 0x352c/0x3688 copies region-C+0x9ac into
        // global 0x109488. Its compute nibble (bits8..11) selects native
        // progress sampling: mode3 only checks busy; mode2 takes the
        // detailed sampler. TA (nibble 0) also takes mode2: b000 0x890c
        // then enters the original sampler at 0x8924 instead of the
        // busy-bit-only path. The busy-only path recovers any TA busy for
        // region-C+0x9a0 (=0xa) one-tick scans, ~12.8 ms after the kick,
        // which legitimate single-invocation vertex/geometry work reaches
        // (image-op-rootcause-1113). Mode2 still recovered a real TA hang
        // (linux-workloads-123, ~32 ms). Keep 3D and all timeout bounds.
        let progress_policy = 0x232;
        self.region_c.write_u32(progress, progress_policy)?;
        fence(Ordering::SeqCst);
        dev_info!(dev.as_ref(),
            "G18_QUEUE_OWNER firmware: hwdata+0x2724=0 before root publication; no host KSMFE writes\n");
        dev_info!(dev.as_ref(), "G18_TRACE_MASK GFX status-A={:#x} before root publication\n", trace_mask);
        dev_info!(dev.as_ref(), "G18_PROGRESS_POLICY region-C+0x9ac={:#x} before publication; timeout bounds unchanged\n", progress_policy);
        Ok(())
    }

    /// Retain a private shader/input/output VM. This does not select a process
    /// context or make its roots visible to firmware, and never queues work.
    pub(crate) fn prepare_compute_payload(&mut self, dev: &AsahiDrmDevice)
        -> Result<&mut G18ComputePayloadMemory> {
        if self.compute_payload.is_some() { return Err(EBUSY); }
        self.compute_payload = Some(G18ComputePayloadMemory::load(dev, &self.uat)?);
        Ok(self.compute_payload.as_mut().unwrap())
    }

    /// Connect the retained payload VM to the command/channel owner. Context
    /// roots remain unpublished until the complete command has been prepared.
    pub(crate) fn bind_compute_process(&mut self, dev: &AsahiDrmDevice) -> Result<u32> {
        let payload = self.compute_payload.as_ref().ok_or(EINVAL)?;
        self.compute.bind_process(dev, &self.uat, payload.vm())
    }

    /// Allocate the captured job's scratch pool while its state is unpublished.
    /// The returned register inputs are valid only with this retained owner.
    pub(crate) fn prepare_compute_scratch(&mut self, dev: &AsahiDrmDevice)
        -> Result<crate::g18::fw::uma::G18ComputeScratchPlan> {
        let plan = self.compute_payload.as_ref().ok_or(EINVAL)?.scratch_plan()?;
        self.uma_pool.prepare_compute_backing(dev, &self.uat, DYNAMIC_START..DYNAMIC_END, plan)?;
        Ok(plan)
    }

    /// Assemble the single captured addition with retained Linux resources.
    /// Every step is still unpublished: the caller must subsequently bind the
    /// pool, publish the process roots, encode the owned SKSM entry and only
    /// then publish a kick. Errors leave any partial owners retained here.
    pub(crate) fn prepare_addition(&mut self, dev: &AsahiDrmDevice)
        -> Result<(G18PreparedComputeCommand, G18UmaPoolBinding)> {
        use crate::g18::fw::compute::G18ComputeCompletionState;
        use crate::g18::queue::probe::QUEUE_ID;

        let (mut command, mut registers) = self.prepare_compute_payload(dev)?.addition_inputs()?;
        let context_id = self.bind_compute_process(dev)?;
        let scratch = self.prepare_compute_scratch(dev)?;
        // The hardware reads the pool through the job's context (stage27
        // GMMU fault in vm_slot 3), so the pool must be mapped in that VM.
        let payload_vm = self.compute_payload.as_ref().ok_or(EINVAL)?.vm();
        self.uma_pool.alias_into_process(dev, &self.uat, payload_vm)?;
        let pool = self.prepare_uma_pool_binding()?;
        // Native prepareLocked loads the hardware descriptor for the same slot
        // before the submit's tag15 names the pool; bind_pool publishes later.
        let sma = j873_sma_device_words(registers.clusters, registers.cluster_width)?;
        self.uma_pool.load_hardware_descriptor(dev, &mut self.uma, pool, sma)?;
        let scheduler = self.scheduler.prepare(0, 1)?;

        // One fresh CL queue, first work ordinal 0, hardware stamp 1. Native
        // event-machine conversion with 256 entries and base 1 yields 1 for
        // event stamp 0x100; no previous completion is imported from macOS.
        command.sequence = 1;
        command.d100 = 1;
        command.d7b8 = pool.state;
        command.completion = G18ComputeCompletionState {
            stamp: 0x100, index: u32::from(QUEUE_ID), entries: 256,
            generation: 0, base: 1,
        };
        registers.context_id = context_id;
        registers.pool_slot = pool.slot;
        // The passing fixture ran with a zero SMA result (0x10241 = slot).
        // The pool binding generation is the separate FList field (tag15).
        registers.sma_units = 0;
        registers.d550 = scratch.register_pair[0];
        registers.d558 = scratch.register_pair[1];
        let prepared = self.compute.prepare(scheduler, &command, &registers)?;
        dev_info!(dev.as_ref(),
            "G18 addition prepared: command={:#x}/{:#x} context={} scheduler={:#x} stamp={:#x} pool={}/{} registers={:?}; unpublished\n",
            prepared.gpu_address, prepared.firmware_address, context_id,
            scheduler.record, prepared.expected_stamp, pool.slot, pool.generation,
            prepared.registers);
        Ok((prepared, pool))
    }

    pub(crate) fn addition_output(&mut self) -> Result<crate::g18::fixture::compute_payload::ComputeResult> {
        self.compute_payload.as_mut().ok_or(EINVAL)?.output_snapshot()
    }

    pub(crate) fn triangle_output(&mut self) -> Result<crate::g18::fixture::compute_triangle::TriangleResult> {
        self.compute_payload.as_mut().ok_or(EINVAL)?.triangle_output_snapshot()
    }

    pub(crate) fn prepare_uma_pool_binding(&mut self) -> Result<G18UmaPoolBinding> {
        self.uma_pool.prepare_binding()
    }

    /// FW VA of one role's root, for the EP 0x20 0x81 doorbell.
    pub(crate) fn root(&self, secondary: bool) -> u64 {
        if secondary { self.secondary_root.iova() } else { self.primary_root.iova() }
    }

    pub(crate) fn compute_outer_counters(&mut self) -> Result<[u32; 3]> {
        let base = PRIVATE_BASE + (init::G18G_26A428_PRIVATE_STATE_GRIDS[0]
            + init::WORK_STATE_GRID_OFFSETS[8]) as u64;
        Ok([self.private.read_u32(base)?, self.private.read_u32(base + 0x10)?,
            self.private.read_u32(base + 0x20)?])
    }

    /// G18 native 24-byte CL_2 record, limited to the registration and
    /// scheduler-bind records. Each new producer is published exactly once.
    pub(crate) fn publish_queue_probe(&mut self, address: u64, queue_id: u8, producer: u32) -> Result {
        if !(1..=3).contains(&producer) { return Err(EINVAL); }
        self.publish_compute_outer(address, queue_id, producer, producer)
    }

    /// After the three setup records, publish one outer item covering the
    /// command, context configuration and kick (inner producer 6).
    pub(crate) fn publish_compute_queue(&mut self, address: u64, queue_id: u8) -> Result {
        self.publish_compute_outer(address, queue_id, 4, 6)
    }

    fn publish_compute_outer(&mut self, address: u64, queue_id: u8,
                             producer: u32, inner_producer: u32) -> Result {
        if address < KERNEL_BASE || address & 0x3f != 0 || queue_id > 127
            || !(1..=4).contains(&producer) {
            return Err(EINVAL);
        }
        if self.compute_outer_counters()? != [producer - 1; 3]
            || self.control_counters()? != [[2; 3], [1; 3]] {
            return Err(EBUSY);
        }
        self.shared.with_bytes_mut(|out| {
            let start = self.layouts.work_rings[8] + (producer as usize - 1) * 24;
            let record = out.get_mut(start..start + 24).ok_or(EINVAL)?;
            if record.iter().any(|byte| *byte != 0) { return Err(EBUSY); }
            record[8..16].copy_from_slice(&address.to_le_bytes());
            record[16..20].copy_from_slice(&2u32.to_le_bytes());
            record[20..22].copy_from_slice(&(inner_producer as u16).to_le_bytes());
            record[22] = queue_id;
            record[23] = 1;
            Ok(())
        })?;
        fence(Ordering::SeqCst);
        let producer_address = PRIVATE_BASE + (init::G18G_26A428_PRIVATE_STATE_GRIDS[0]
            + init::WORK_STATE_GRID_OFFSETS[8] + 0x20) as u64;
        self.private.write_u32(producer_address, producer)?;
        fence(Ordering::SeqCst);
        Ok(())
    }

    /// The live device-control counters (consumer, CFI, producer) of both
    /// roles, read from the shared state the firmware consumes.
    pub(crate) fn control_counters(&mut self) -> Result<[[u32; 3]; 2]> {
        self.private.with_bytes_mut(|bytes| {
            let mut counters = [[0; 3]; 2];
            for (role, grid) in init::G18G_26A428_PRIVATE_STATE_GRIDS.into_iter().enumerate() {
                let base = grid + init::WORK_STATE_GRID_OFFSETS[init::CONTROL_CHANNEL_INDEX];
                for (index, value) in counters[role].iter_mut().enumerate() {
                    let offset = base + index * init::CHANNEL_STATE_SPACING;
                    let word = bytes.get(offset..offset + 4).ok_or(EINVAL)?;
                    // SAFETY: the grid offsets are u32 aligned inside this
                    // bounded WC mapping, which is retained while firmware runs.
                    *value = unsafe { core::ptr::read_volatile(word.as_ptr().cast::<u32>()) };
                }
            }
            fence(Ordering::Acquire);
            Ok(counters)
        })
    }

    /// Stage device-control record 1 of role 0, IdlePowerOff(0) (opcode 0x0a,
    /// FW_PROVEN unchanged: b000 0x4ef0), and publish it. Only valid after
    /// both opening records retired, i.e. every counter reads 1.
    pub(crate) fn publish_idle_wake(&mut self) -> Result {
        if self.control_counters()? != [[1; 3]; 2] {
            return Err(EBUSY);
        }
        let record = self.layouts.shared.control_ring[0] + init::CONTROL_RECORD_SIZE;
        self.shared.with_bytes_mut(|out| {
            let record = out.get_mut(record..record + init::CONTROL_RECORD_SIZE).ok_or(EINVAL)?;
            if record.iter().any(|byte| *byte != 0) {
                return Err(EBUSY);
            }
            init::encode_device_control_record(
                init::G18G_26A428_DEVICE_CONTROL_OPCODE_IDLE_POWER_OFF,
                init::DEVICE_CONTROL_IDLE_POWER_OFF_ARG_DISABLE_IDLE_AND_WAKE,
                record,
            )
            .map_err(|_| EINVAL)
        })?;
        fence(Ordering::SeqCst);
        let producer = init::G18G_26A428_PRIVATE_STATE_GRIDS[0]
            + init::WORK_STATE_GRID_OFFSETS[init::CONTROL_CHANNEL_INDEX]
            + init::CHANNEL_STATE_PRODUCER * init::CHANNEL_STATE_SPACING;
        self.private.write_u32(PRIVATE_BASE + producer as u64, init::CONTROL_OPENING_PRIMARY_PRODUCER + 1)?;
        fence(Ordering::SeqCst);
        Ok(())
    }

    /// The four firmware-owned KSM completion cursors (hw_data +0x2734 +
    /// ordinal * 0x20 + 0x1c; FW_PROVEN b000 0x1cbc0..0x1cc48).
    pub(crate) fn completion_cursors(&mut self) -> Result<[u32; init::M5_25G76_COMPLETION_QUEUE_COUNT]> {
        let mut cursors = [0; init::M5_25G76_COMPLETION_QUEUE_COUNT];
        for (ordinal, cursor) in cursors.iter_mut().enumerate() {
            *cursor = self.shared.read_u32(
                SHARED_BASE
                    + (init::G18G_26A428_KSM_COMPLETION_TABLE_OFFSET
                        + ordinal * init::G17P_KSM_COMPLETION_DESCRIPTOR_STRIDE
                        + init::G18G_26A428_KSM_COMPLETION_CURSOR) as u64,
            )?;
        }
        Ok(cursors)
    }

    /// Current recovery handshake words. Host DRAM only; no SGX MMIO.
    pub(crate) fn recovery_word(&mut self) -> Result<G18RecoveryWord> {
        let status_b = PRIVATE_BASE + init::G18G_26A428_PRIVATE_LAYOUT.status_b as u64;
        fence(Ordering::Acquire);
        Ok(G18RecoveryWord {
            epoch: self.private.read_u64(status_b + init::G18G_26A428_STATUS_B_RECOVERY_EPOCH as u64)?,
            state: self.private.read_u32(status_b + init::G18G_26A428_STATUS_B_RECOVERY_STATE as u64)?,
            host: self.private.read_u32(status_b + init::G18G_26A428_STATUS_B_RECOVERY_HOST as u64)?,
        })
    }

    /// The cause and info the firmware published for the current recovery.
    /// Like macOS, count entries by their valid bit rather than trusting the
    /// emitted count.
    pub(crate) fn recovery_cause(&mut self) -> Result<G18RecoveryCause> {
        let status_b = PRIVATE_BASE + init::G18G_26A428_PRIVATE_LAYOUT.status_b as u64;
        fence(Ordering::Acquire);
        let word = |memory: &mut Self, offset: usize| memory.private.read_u32(status_b + offset as u64);
        let mut cause = G18RecoveryCause {
            emitted: word(self, init::G18G_26A428_STATUS_B_RECOVERY_EMITTED)?,
            source_count: word(self, init::G18G_26A428_STATUS_B_RECOVERY_SOURCE_COUNT)?,
            command: word(self, init::G18G_26A428_STATUS_B_RECOVERY_COMMAND)?,
            reason: word(self, init::G18G_26A428_STATUS_B_RECOVERY_REASON)?,
            detail: self.private.read_u64(status_b + init::G18G_26A428_STATUS_B_RECOVERY_DETAIL as u64)?,
            slot_mask: self.private.read_u64(status_b + init::G18G_26A428_STATUS_B_RECOVERY_SLOT_MASK as u64)?,
            ..Default::default()
        };
        for index in 0..init::G18G_26A428_STATUS_B_RECOVERY_INFO_ENTRIES {
            let entry = self.private.read_u64(status_b
                + (init::G18G_26A428_STATUS_B_RECOVERY_INFO + index * 8) as u64)?;
            if entry & 1 != 0 {
                if let Some(slot) = cause.pending.get_mut(cause.valid_entries as usize) {
                    *slot = (index as u16, entry);
                }
                cause.valid_entries += 1;
                cause.first_entry.get_or_insert((index as u32, entry));
            }
        }
        let request = SHARED_BASE + self.layouts.shared.block_68 as u64;
        for (index, value) in cause.request.iter_mut().enumerate() {
            *value = self.shared.read_u32(request + (index * 4) as u64)?;
        }
        Ok(cause)
    }

    /// One QID's last-submitted entry: (valid, stamp).
    pub(crate) fn queue_progress(&mut self, qid: usize) -> Result<(u32, u64)> {
        if qid >= init::G18G_26A428_QUEUE_PROGRESS_ENTRIES {
            return Err(EINVAL);
        }
        let entry = SHARED_BASE + (self.layouts.shared.block_800
            + qid * init::G18G_26A428_QUEUE_PROGRESS_STRIDE) as u64;
        fence(Ordering::Acquire);
        Ok((self.shared.read_u32(entry)?, self.shared.read_u64(entry + 8)?))
    }

    /// Publish one QID's last-submitted entry for the firmware's restart.
    pub(crate) fn publish_queue_progress(&mut self, qid: usize, valid: u32, stamp: u64) -> Result {
        if qid >= init::G18G_26A428_QUEUE_PROGRESS_ENTRIES {
            return Err(EINVAL);
        }
        let entry = SHARED_BASE + (self.layouts.shared.block_800
            + qid * init::G18G_26A428_QUEUE_PROGRESS_STRIDE) as u64;
        self.shared.write_u32(entry, valid)?;
        self.shared.write_u32(entry + 4, 0)?;
        self.shared.write_u32(entry + 8, stamp as u32)?;
        self.shared.write_u32(entry + 12, (stamp >> 32) as u32)?;
        fence(Ordering::SeqCst);
        Ok(())
    }

    /// Take the host's half of one handshake edge: 1 -> 2 lets the halted
    /// firmware recover, 3 -> 0 is the final clear. Any other edge, or a
    /// state other than `expected`, is refused without writing.
    pub(crate) fn acknowledge_recovery(&mut self, expected: u32, next: u32) -> Result {
        if !matches!((expected, next), (1, 2) | (3, 0)) {
            return Err(EINVAL);
        }
        let state = PRIVATE_BASE + (init::G18G_26A428_PRIVATE_LAYOUT.status_b
            + init::G18G_26A428_STATUS_B_RECOVERY_STATE) as u64;
        fence(Ordering::Acquire);
        if self.private.read_u32(state)? != expected {
            return Err(EBUSY);
        }
        fence(Ordering::SeqCst);
        self.private.write_u32(state, next)?;
        fence(Ordering::SeqCst);
        Ok(())
    }

    /// Read back the new status cross-links from the host mapping (plan step
    /// 1.3 validation after the 0x81 ACKs).
    pub(crate) fn status_links(&mut self) -> Result<G18StatusLinks> {
        let layout = init::G18G_26A428_PRIVATE_LAYOUT;
        let status_b = PRIVATE_BASE + layout.status_b as u64;
        let config = PRIVATE_BASE + layout.status_config as u64;
        Ok(G18StatusLinks {
            status_b_header: self.private.read_u32(status_b + init::G18G_26A428_STATUS_B_CONFIG_HEADER as u64)?,
            status_b_word: self.private.read_u32(status_b + init::G18G_26A428_STATUS_B_LINK_WORD as u64)?,
            status_b_pointer: self.private.read_u64(status_b + init::G18G_26A428_STATUS_B_LINK_PTR as u64)?,
            status_config_word: self.private
                .read_u32(config + self.layouts.status_config_link_word as u64)?,
            status_config_pointer: self.private
                .read_u64(config + self.layouts.status_config_link_ptr as u64)?,
        })
    }

    /// Drain both roles' event (channel 13) and trace (channel 14) report
    /// rings. Records are copied before their slot is released; each drain is
    /// bounded. Returns whether an event record of the J813 recovery type was
    /// seen. This is the milestone-1 heartbeat (plan step 1.8, stage 4).
    pub(crate) fn drain_reports(&mut self, dev: &AsahiDrmDevice) -> Result<bool> {
        let mut recovery = false;
        for (role, status_a) in init::G18G_26A428_PRIVATE_STATUS_A.into_iter().enumerate() {
            for channel in 0..2 {
                let [cursor_offset, ring_offset, _] = init::REPORT_STATE_STATUS_A_OFFSETS[channel];
                let capacity = REPORT_CAPACITY[channel];
                let cursor = PRIVATE_BASE + (status_a + cursor_offset) as u64;
                let ring = status_a + ring_offset;
                for _ in 0..REPORT_DRAIN_BUDGET {
                    let consumer = self.private.read_u32(cursor)?;
                    let producer = self.private.read_u32(cursor + REPORT_PRODUCER_OFFSET)?;
                    if consumer >= capacity || producer >= capacity {
                        return Err(EIO);
                    }
                    if consumer == producer {
                        break;
                    }
                    fence(Ordering::Acquire);
                    let offset = ring + consumer as usize * REPORT_RECORD_SIZE;
                    let mut raw = [0u8; REPORT_RECORD_SIZE];
                    self.private.with_bytes_mut(|bytes| {
                        raw.copy_from_slice(bytes.get(offset..offset + REPORT_RECORD_SIZE).ok_or(EINVAL)?);
                        Ok(())
                    })?;
                    let word = |off: usize| u32::from_le_bytes([raw[off], raw[off + 1], raw[off + 2], raw[off + 3]]);
                    let qword = |off: usize| u64::from(word(off)) | (u64::from(word(off + 4)) << 32);
                    let count = self.report_counts[role][channel];
                    if channel == 1 {
                        // Preserve all bytes before firmware may reuse this slot.
                        // No allocation, MMIO, or extra logging in the hot path.
                        if self.trace_journal[role].len() >= TRACE_JOURNAL_CAPACITY
                            || self.trace_journal[role]
                                .push_within_capacity(G18TraceRecord { ordinal: count, raw })
                                .is_err()
                        {
                            self.trace_dropped[role] = self.trace_dropped[role].saturating_add(1);
                        }
                    }
                    if channel == 0 {
                        crate::g18_event_info!(role == 0 && word(0) == 1, dev.as_ref(), "G18_EVENT role={} ordinal={} type={} payload={:x?}\n",
                            role, count, word(0), [qword(4), qword(12), qword(20), qword(28), qword(36)]);
                        recovery |= word(0) == EVENT_TYPE_RECOVERY;
                        if word(0) == EVENT_TYPE_TVB_GROW {
                            // The session grows the PB and replies after the drain.
                            let request = G18TvbGrowRequest {
                                vm_slot: word(4), buffer_slot: word(8), counter: word(0xc),
                                subpipe: word(0x10), halt_count: qword(0x14),
                            };
                            // A repeated request gets one reply, as on G17.
                            if !self.tvb_grow.contains(&request) {
                                if self.tvb_grow.len() >= TVB_GROW_QUEUE {
                                    // Leave it in the ring: the session answers
                                    // the queued ones first, the next drain
                                    // takes this one. The session drains on
                                    // every poll of its waits (poll_faults(),
                                    // check_faults()), not on an interrupt, so
                                    // the next drain is one poll away.
                                    break;
                                }
                                self.tvb_grow.push_within_capacity(request).map_err(|_| EBUSY)?;
                            }
                        }
                        if role == 0 && word(0) == 1 {
                            // Original G18 b000 233a8..233d4 uses this exact
                            // type-1 prefix. Copy/validate before releasing the
                            // slot; the remaining bytes are not completion proof.
                            let event = crate::g18::fw::events::decode_g17p_firmware_stamp_event(&raw)
                                .map_err(|_| EIO)?;
                            self.completion_masks[0] |= event.masks[0];
                            self.completion_masks[1] |= event.masks[1];
                        }
                    } else if self.trace_printed[role] < TRACE_PRINT_BUDGET[role] {
                        dev_info!(dev.as_ref(),
                            "G18_TRACE role={} ordinal={} timestamp={} type={} id={:#x} args={:x?}\n",
                            role, count, qword(4), word(0), word(0x2c),
                            [qword(12), qword(20), qword(28), qword(36)]);
                        self.trace_printed[role] += 1;
                    }
                    self.report_counts[role][channel] = count.saturating_add(1);
                    fence(Ordering::SeqCst);
                    self.private.write_u32(cursor, (consumer + 1) % capacity)?;
                }
            }
        }
        Ok(recovery)
    }

    /// The oldest drained TVB grow request not yet answered.
    pub(crate) fn take_tvb_grow(&mut self) -> Option<G18TvbGrowRequest> {
        if self.tvb_grow.is_empty() { None } else { self.tvb_grow.remove(0).ok() }
    }

    /// Publish the TvbGrowReply for `request` on the primary device-control
    /// ring (g17 TvbGrowReply: opcode, grew, buffer_slot, vm_slot, counter,
    /// subpipe, halt_count). The caller rings the EP 0x21 control doorbell.
    pub(crate) fn reply_tvb_grow(&mut self, request: G18TvbGrowRequest, grew: bool) -> Result {
        let [consumer, _, producer] = self.control_counters()?[0];
        if consumer > init::CONTROL_RING_INDEX_MASK || producer > init::CONTROL_RING_INDEX_MASK {
            return Err(EIO);
        }
        let next = (producer + 1) & init::CONTROL_RING_INDEX_MASK;
        if next == consumer {
            return Err(EBUSY);
        }
        let record = self.layouts.shared.control_ring[0] + producer as usize * init::CONTROL_RECORD_SIZE;
        self.shared.with_bytes_mut(|out| {
            let record = out.get_mut(record..record + init::CONTROL_RECORD_SIZE).ok_or(EINVAL)?;
            init::encode_device_control_record(DEVICE_CONTROL_OPCODE_TVB_GROW_REPLY, u32::from(grew), record)
                .map_err(|_| EINVAL)?;
            let r = request;
            for (offset, value) in [(0x08, r.buffer_slot), (0x0c, r.vm_slot), (0x10, r.counter),
                                    (0x14, r.subpipe), (0x18, r.halt_count as u32),
                                    (0x1c, (r.halt_count >> 32) as u32)] {
                record[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
            }
            Ok(())
        })?;
        fence(Ordering::SeqCst);
        let producer_offset = init::G18G_26A428_PRIVATE_STATE_GRIDS[0]
            + init::WORK_STATE_GRID_OFFSETS[init::CONTROL_CHANNEL_INDEX]
            + init::CHANNEL_STATE_PRODUCER * init::CHANNEL_STATE_SPACING;
        self.private.write_u32(PRIVATE_BASE + producer_offset as u64, next)?;
        fence(Ordering::SeqCst);
        self.control_published = self.control_published.wrapping_add(1);
        Ok(())
    }

    /// Device-control counters of both roles once every record published
    /// after boot retired: the idle wake left the primary ring at 2.
    pub(crate) fn control_retired(&self) -> [[u32; 3]; 2] {
        [[(2 + self.control_published) & init::CONTROL_RING_INDEX_MASK; 3], [1; 3]]
    }

    /// Report-record totals per role and channel since construction.
    pub(crate) fn report_counts(&self) -> [[u32; 2]; 2] {
        self.report_counts
    }

    /// Only primary firmware type-1 records contribute to these fresh-boot bits.
    pub(crate) fn completion_masks(&self) -> [u64; 2] {
        self.completion_masks
    }

    /// Export after the transaction and stop requests, never while polling a
    /// GPU job. Completeness covers consumed records, not a possibly live producer.
    pub(crate) fn log_trace_journal(&self, dev: &AsahiDrmDevice) {
        for role in 0..2 {
            let journal = &self.trace_journal[role];
            let consumed = self.report_counts[role][1];
            dev_info!(dev.as_ref(),
                "G18_TRACE_JOURNAL_BEGIN role={} consumed={} stored={} dropped={} capacity={}\n",
                role, consumed, journal.len(), self.trace_dropped[role], TRACE_JOURNAL_CAPACITY);
            for record in journal.iter() {
                let words: [u64; 9] = core::array::from_fn(|index| {
                    let mut bytes = [0; 8];
                    bytes.copy_from_slice(&record.raw[index * 8..index * 8 + 8]);
                    u64::from_le_bytes(bytes)
                });
                dev_info!(dev.as_ref(), "G18_TRACE_JOURNAL role={} ordinal={} words={:016x?}\n",
                    role, record.ordinal, words);
            }
            dev_info!(dev.as_ref(),
                "G18_TRACE_JOURNAL_END role={} consumed={} stored={} dropped={}\n",
                role, consumed, journal.len(), self.trace_dropped[role]);
        }
    }

    /// One-line construction summary for the bring-up log.
    pub(crate) fn log_summary(&self, dev: &AsahiDrmDevice) {
        dev_info!(dev.as_ref(),
            "G18G initdata: roots {:#x}/{:#x} version {:#x}; shared {:#x}+{:#x}; private {:#x}+{:#x}; {} register windows; {} unproven fields\n",
            self.root(false), self.root(true), self.layouts.version_word,
            SHARED_BASE, self.layouts.shared.total,
            PRIVATE_BASE, self.layouts.private.total,
            self.registers.len(), init::G18G_26A428_UNPROVEN.len());
    }
}
