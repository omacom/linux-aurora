// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! GPU MMIO register abstraction
//!
//! Since the vast majority of the interactions with the GPU are brokered through the firmware,
//! there is very little need to interact directly with GPU MMIO register. This module abstracts
//! the few operations that require that, mainly reading the MMU fault status, reading GPU ID
//! information, and starting the GPU firmware coprocessor.
//!
//! On G17, the driver also enables the bridge between the GPU and the fabric before the
//! coprocessors start, and enables dynamic power gating before the firmware is initialized.

use crate::{hw, identity};
use core::mem::MaybeUninit;
use kernel::{
    bindings, c_str,
    device::Core,
    devres::Devres,
    error::{from_err_ptr, to_result},
    io::{
        mem::IoMem, //
        Io,
    },
    platform,
    prelude::*,
    sync::{aref::ARef, Arc}, //
};

/// Size of the ASC control MMIO region.
pub(crate) const ASC_CTL_SIZE: usize = 0x4000;

/// Size of the SGX MMIO region.
pub(crate) const SGX_SIZE: usize = 0x1000000;

const CPU_CONTROL: usize = 0x44;
const CPU_RUN: u32 = 0x1 << 4; // BIT(4)

const FAULT_INFO: usize = 0x17030;

/// A nonzero low nibble permits writes through the G17 queue configuration port.
const G17_HOST_IRQ_SUMMARY: usize = 0xe01000;
const G17_HOST_POWERED: u32 = 0xf;

const ID_VERSION: usize = 0xd04000;
const ID_UNK08: usize = 0xd04008;
const ID_COUNTS_1: usize = 0xd04010;
const ID_COUNTS_2: usize = 0xd04014;
const ID_UNK18: usize = 0xd04018;
const ID_CLUSTERS: usize = 0xd0401c;

const CORE_MASK_0: usize = 0xd01500;
const CORE_MASK_1: usize = 0xd01514;

const CORE_MASKS_G14X: usize = 0xe01500;
const FAULT_INFO_G14X: usize = 0xd8c0;
const FAULT_ADDR_G14X: usize = 0xd8c8;

/// G17 GPU-to-fabric bridge control words. Bit 0 enables the bridge; the other bits are status.
const G17_BRIDGE_CONTROL: [usize; 2] = [0x1000104, 0x1000108];
const G17_BRIDGE_ENABLE: u32 = 0x1; // BIT(0)

/// G17 dynamic power gating control. Clearing these bits enables gating of idle GPU cores.
const G17_GATING_CONTROL: usize = 0xd06030;
const G17_GATING_DISABLE: u32 = 0x6; // BIT(1) | BIT(2)

/// These performance selectors must be sampled before firmware can gate the core registers.
const G17_PERF_CONTROL: usize = 0xe0141c;
const G17_PERF_MAP: usize = 0xe01480;
const G17_CORE_POWER: usize = 0xe01000;

const FAULT_ADDR_MASK_G15: u64 = (1 << 42) - 1;
/// Enum representing the unit that caused an MMU fault.
#[allow(non_camel_case_types)]
#[allow(clippy::upper_case_acronyms)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub(crate) enum FaultUnit {
    /// Decompress / pixel fetch
    DCMP(u8),
    /// USC L1 Cache (device loads/stores)
    UL1C(u8),
    /// Compress / pixel store
    CMP(u8),
    GSL1(u8),
    IAP(u8),
    VCE(u8),
    /// Tiling Engine
    TE(u8),
    RAS(u8),
    /// Vertex Data Master
    VDM(u8),
    PPP(u8),
    /// ISP Parameter Fetch
    IPF(u8),
    IPF_CPF(u8),
    VF(u8),
    VF_CPF(u8),
    /// Depth/Stencil load/store
    ZLS(u8),

    /// Parameter Management
    dPM,
    /// Compute Data Master
    dCDM_KS(u8),
    dIPP,
    dIPP_CS,
    // Vertex Data Master
    dVDM_CSD,
    dVDM_SSD,
    dVDM_ILF,
    dVDM_ILD,
    dRDE(u8),
    FC,
    GSL2,

    /// Graphics L2 Cache Control?
    GL2CC_META(u8),
    GL2CC_MB,

    /// Parameter Management
    gPM_SP(u8),
    /// Vertex Data Master - CSD
    gVDM_CSD_SP(u8),
    gVDM_SSD_SP(u8),
    gVDM_ILF_SP(u8),
    gVDM_TFP_SP(u8),
    gVDM_MMB_SP(u8),
    /// Compute Data Master
    gCDM_CS_KS0_SP(u8),
    gCDM_CS_KS1_SP(u8),
    gCDM_CS_KS2_SP(u8),
    gCDM_KS0_SP(u8),
    gCDM_KS1_SP(u8),
    gCDM_KS2_SP(u8),
    gIPP_SP(u8),
    gIPP_CS_SP(u8),
    gRDE0_SP(u8),
    gRDE1_SP(u8),

    gCDM_CS,
    gCDM_ID,
    gCDM_CSR,
    gCDM_CSW,
    gCDM_CTXR,
    gCDM_CTXW,
    gIPP,
    gIPP_CS,
    gKSM_RCE,

    /// G15: Graphics L2 cache metadata, in the x4/xF slots of clusters 1, 2, 6, 7
    GL2CC_META_G15GX(u8),
    /// G15: UMA (Dynamic Caching) core
    UMA_CORE,
    /// G15: UMA parameter management?
    gUPM,

    Unknown(u8),
}

/// Reason for an MMU fault.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub(crate) enum FaultReason {
    Unmapped,
    AfFault,
    WriteOnly,
    ReadOnly,
    NoAccess,
    Unknown(u8),
}

/// Collection of information about an MMU fault.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub(crate) struct FaultInfo {
    pub(crate) address: u64,
    pub(crate) sideband: u8,
    pub(crate) vm_slot: u32,
    pub(crate) unit_code: u8,
    pub(crate) unit: FaultUnit,
    pub(crate) level: u8,
    pub(crate) unk_5: u8,
    pub(crate) read: bool,
    pub(crate) reason: FaultReason,
}

/// Identification of a G17 GPU.
pub(crate) struct G17Id {
    /// GPU family field of the ID register.
    pub(crate) family: u8,
    /// GPU variant field of the ID register.
    pub(crate) variant: u8,
    /// Silicon revision.
    pub(crate) gpu_rev: hw::GpuRevision,
    /// Number of dies.
    pub(crate) num_dies: u32,
    /// Number of clusters, over all dies.
    pub(crate) num_clusters: u32,
    /// Number of cores per cluster.
    pub(crate) num_cores: u32,
    /// Enabled cores of each cluster.
    pub(crate) core_masks: KVec<u32>,
    /// Performance selector control, sampled before core power gating starts.
    pub(crate) perf_control: u32,
    /// Performance selector table indices, low and high nibbles.
    pub(crate) perf_map: [u32; 2],
}

/// Provider-owned lifecycle of one G17 firmware coprocessor.
///
/// The mailbox provider owns the wrapper mappings and runtime-PM references.
/// The consumer device link keeps it alive until this GPU has stopped using it.
pub(crate) struct CpuControl {
    dev: ARef<platform::Device>,
    mbox: *mut bindings::apple_mbox,
}

// SAFETY: The live provider serializes CPU transitions with its lifecycle mutex;
// the device link orders provider removal after this consumer's teardown.
unsafe impl Send for CpuControl {}
// SAFETY: All accesses use the same provider-serialized lifecycle API.
unsafe impl Sync for CpuControl {}

impl CpuControl {
    /// Resolve the mailbox phandle and validate its firmware role before use.
    pub(crate) fn new(pdev: &platform::Device<Core>, index: usize) -> Result<Self> {
        let expected = match index {
            0 => bindings::apple_mbox_ascwrap_v6_role_APPLE_MBOX_ASCWRAP_V6_ROLE_GFX,
            1 => bindings::apple_mbox_ascwrap_v6_role_APPLE_MBOX_ASCWRAP_V6_ROLE_GFX1,
            _ => return Err(EINVAL),
        };
        // SAFETY: pdev is live; the provider establishes the consumer device link.
        let mbox = unsafe {
            from_err_ptr(bindings::apple_mbox_get(pdev.as_ref().as_raw(), index.try_into()?))?
        };
        let mut lifecycle = MaybeUninit::uninit();
        // SAFETY: mbox is live, and lifecycle is writable storage for the C result.
        to_result(unsafe {
            bindings::apple_mbox_ascwrap_v6_get_lifecycle(mbox, lifecycle.as_mut_ptr())
        })?;
        // SAFETY: A successful provider query initialized every field.
        if unsafe { lifecycle.assume_init() }.role != expected {
            return Err(EINVAL);
        }
        // SAFETY: This only checks the provider's matching start/stop capability.
        to_result(unsafe { bindings::apple_mbox_ascwrap_v6_require_safe_cpu_lifecycle(mbox) })?;
        Ok(Self { dev: pdev.into(), mbox })
    }

    /// Start through the owner of the CPU registers and runtime-PM reference.
    pub(crate) fn start(&self) -> Result {
        // SAFETY: The constructor established the live, role-checked provider.
        to_result(unsafe { bindings::apple_mbox_ascwrap_v6_start_cpu(self.mbox) })
    }

    /// Stop before releasing the transport or any firmware-visible memory.
    pub(crate) fn stop(&self) {
        // SAFETY: The provider remains live during consumer teardown. Removal
        // stops the CPU before setting the fence that can return ENODEV here.
        let result = to_result(unsafe { bindings::apple_mbox_ascwrap_v6_stop_cpu(self.mbox) });
        if let Err(error) = result {
            if error != ENODEV {
                dev_err!(self.dev.as_ref(), "Could not stop GPU coprocessor: {:?}\n", error);
            }
        }
    }
}

/// Device resources for this GPU instance.
#[derive(Clone)]
pub(crate) struct Resources {
    dev: ARef<platform::Device>,
    sgx: Arc<Devres<IoMem<SGX_SIZE>>>,
}

impl Resources {
    fn fault_unit_g15(unit_code: u8) -> FaultUnit {
        let hi = unit_code >> 4;
        // x4 -> GL2CC_META_G15GX_{0,2,4,6}, xF -> GL2CC_META_G15GX_{1,3,5,7}
        let gl2cc_meta = |odd: u8| match hi {
            1 => FaultUnit::GL2CC_META_G15GX(odd),
            2 => FaultUnit::GL2CC_META_G15GX(2 + odd),
            6 => FaultUnit::GL2CC_META_G15GX(4 + odd),
            7 => FaultUnit::GL2CC_META_G15GX(6 + odd),
            _ => FaultUnit::Unknown(unit_code),
        };

        match unit_code {
            0x00..=0x9f => match unit_code & 0xf {
                0x0 => FaultUnit::DCMP(hi),
                0x1 => FaultUnit::UL1C(hi),
                0x2 => FaultUnit::CMP(hi),
                0x3 => FaultUnit::GSL1(hi),
                0x4 => gl2cc_meta(0),
                0x5 => FaultUnit::VCE(hi),
                0x6 => FaultUnit::TE(hi),
                0x7 => FaultUnit::RAS(hi),
                0x8 => FaultUnit::VDM(hi),
                0x9 => FaultUnit::PPP(hi),
                0xa => FaultUnit::IPF(hi),
                0xb => FaultUnit::IPF_CPF(hi),
                0xc => FaultUnit::VF(hi),
                0xd => FaultUnit::VF_CPF(hi),
                0xe => FaultUnit::ZLS(hi),
                _ => gl2cc_meta(1),
            },
            0xa0 => FaultUnit::UMA_CORE,
            0xa1 => FaultUnit::dPM,
            0xa2 => FaultUnit::dCDM_KS(0),
            0xa3 => FaultUnit::dCDM_KS(1),
            0xa4 => FaultUnit::dCDM_KS(2),
            0xa5 => FaultUnit::dIPP,
            0xa6 => FaultUnit::dIPP_CS,
            0xa7 => FaultUnit::dVDM_CSD,
            0xa8 => FaultUnit::dVDM_SSD,
            0xa9 => FaultUnit::dVDM_ILF,
            0xaa => FaultUnit::dVDM_ILD,
            0xab => FaultUnit::dRDE(0),
            0xac => FaultUnit::dRDE(1),
            0xad => FaultUnit::FC,
            0xae => FaultUnit::GSL2,
            0xb8 => FaultUnit::GL2CC_MB,
            0xd0 => FaultUnit::gCDM_CS,
            0xd1 => FaultUnit::gCDM_ID,
            0xd2 => FaultUnit::gCDM_CSR,
            0xd3 => FaultUnit::gCDM_CSW,
            0xd4 => FaultUnit::gCDM_CTXR,
            0xd5 => FaultUnit::gCDM_CTXW,
            0xd6 => FaultUnit::gIPP,
            0xd7 => FaultUnit::gIPP_CS,
            0xd8 => FaultUnit::gKSM_RCE,
            0xd9 => FaultUnit::gUPM,
            0xe0..=0xff => {
                // Group n: bit 4 selects 0/1, bit 3 adds 2 (0xe0 -> 0, 0xf0 -> 1, 0xe8 -> 2,
                // 0xf8 -> 3). RDE exists only in groups 0 and 1.
                let n = ((unit_code >> 4) & 1) | ((unit_code >> 2) & 2);
                match (unit_code & 0x7, unit_code & 0x8 != 0) {
                    (0x0, _) => FaultUnit::gPM_SP(n),
                    (0x1, _) => FaultUnit::gVDM_CSD_SP(n),
                    (0x2, _) => FaultUnit::gVDM_SSD_SP(n),
                    (0x3, _) => FaultUnit::gVDM_ILF_SP(n),
                    (0x4, _) => FaultUnit::gVDM_TFP_SP(n),
                    (0x5, _) => FaultUnit::gVDM_MMB_SP(n),
                    (0x6, false) => FaultUnit::gRDE0_SP(n),
                    _ => FaultUnit::Unknown(unit_code),
                }
            }
            _ => FaultUnit::Unknown(unit_code),
        }
    }

    /// Map the required resources given our platform device.
    pub(crate) fn new(pdev: &platform::Device<Core>) -> Result<Resources> {
        let sgx_req = pdev.io_request_by_name(c_str!("sgx")).ok_or(EINVAL)?;
        let sgx_iomem = Arc::pin_init(sgx_req.iomap_sized::<SGX_SIZE>(), GFP_KERNEL)?;

        Ok(Resources {
            // SAFETY: This device does DMA via the UAT IOMMU.
            dev: pdev.into(),
            sgx: sgx_iomem,
        })
    }

    /// Installs both render queues while the caller holds the device mutex.
    /// No other port publisher may interleave between this power witness and
    /// the final validity strobe. False means no write was attempted.
    pub(crate) fn install_render_pair(
        &self,
        registration: &crate::g17::fw::kick::RenderRegistration,
    ) -> Result<bool> {
        let sgx = self.sgx.try_access().ok_or(ENODEV)?;
        if sgx.relaxed().read32(G17_HOST_IRQ_SUMMARY) & G17_HOST_POWERED == 0 {
            return Ok(false);
        }
        for &(offset, value) in registration.writes() {
            sgx.try_write64(value, offset)?;
        }
        Ok(true)
    }

    /* Not yet used
    fn sgx_write32<OFF: usize>(&self, val: u32) {
        if let Some(sgx) = self.sgx.try_access() {
            sgx.write32_relaxed(val, OFF)
        }
    }
    */

    fn sgx_read32<const OFF: usize>(&self) -> u32 {
        if let Some(sgx) = self.sgx.try_access() {
            sgx.relaxed().read32(OFF)
        } else {
            0
        }
    }

    fn sgx_read64<const OFF: usize>(&self) -> u64 {
        if let Some(sgx) = self.sgx.try_access() {
            sgx.relaxed().read64(OFF)
        } else {
            0
        }
    }

    /* Not yet used
    fn sgx_write64<OFF: usize>(&self, val: u64) {
        if let Some(sgx) = self.sgx.try_access() {
            sgx.write64_relaxed(val, OFF)
        }
    }
    */

    /// Reads a 32-bit SGX register at a runtime offset (G15 identification, power and fault
    /// windows).
    pub(crate) fn sgx_read32_at(&self, offset: usize) -> Result<u32> {
        let sgx = self.sgx.try_access().ok_or(ENODEV)?;
        sgx.try_read32(offset)
    }

    /// Writes a 32-bit SGX register at a runtime offset (G15).
    pub(crate) fn sgx_write32_at(&self, offset: usize, value: u32) -> Result {
        let sgx = self.sgx.try_access().ok_or(ENODEV)?;
        sgx.try_write32(value, offset)
    }

    /// Initialize the MMIO registers for the GPU.
    pub(crate) fn init_mmio(&self) -> Result {
        // Nothing to do for now...

        Ok(())
    }

    /// Start the ASC coprocessor CPU.
    pub(crate) fn start_cpu(pdev: &platform::Device<Core>) -> Result {
        let asc_req = pdev.io_request_by_name(c_str!("asc")).ok_or(EINVAL)?;
        let asc_iomem = KBox::pin_init(asc_req.iomap_sized::<ASC_CTL_SIZE>(), GFP_KERNEL)?;
        let res = asc_iomem.access(pdev.as_ref())?.relaxed();

        let val = res.read32(CPU_CONTROL);
        res.write32(val | CPU_RUN, CPU_CONTROL);
        Ok(())
    }

    /// Get the GPU identification info from registers.
    ///
    /// See [`hw::GpuIdConfig`] for the result.
    pub(crate) fn get_gpu_id(&self) -> Result<hw::GpuIdConfig> {
        let id_version = self.sgx_read32::<ID_VERSION>();
        let id_unk08 = self.sgx_read32::<ID_UNK08>();
        let id_counts_1 = self.sgx_read32::<ID_COUNTS_1>();
        let id_counts_2 = self.sgx_read32::<ID_COUNTS_2>();
        let id_unk18 = self.sgx_read32::<ID_UNK18>();
        let id_clusters = self.sgx_read32::<ID_CLUSTERS>();

        dev_info!(
            self.dev.as_ref(),
            "GPU ID registers: {:#x} {:#x} {:#x} {:#x} {:#x} {:#x}\n",
            id_version,
            id_unk08,
            id_counts_1,
            id_counts_2,
            id_unk18,
            id_clusters
        );

        let family = ((id_version >> 24) & 0xff) as u8;
        let variant = ((id_version >> 16) & 0xff) as u8;
        let num_dies = (id_counts_1 >> 16) & 0xf;
        let mut core_mask_regs = KVec::new();

        let num_clusters = match family {
            4 | 5 => {
                // G13 | G14G
                core_mask_regs.push(self.sgx_read32::<CORE_MASK_0>(), GFP_KERNEL)?;
                core_mask_regs.push(self.sgx_read32::<CORE_MASK_1>(), GFP_KERNEL)?;
                (id_clusters >> 12) & 0xff
            }
            6 | 0x7 => {
                core_mask_regs.push(self.sgx_read32::<CORE_MASKS_G14X>(), GFP_KERNEL)?;
                core_mask_regs.push(self.sgx_read32::<{ CORE_MASKS_G14X + 4 }>(), GFP_KERNEL)?;
                core_mask_regs.push(self.sgx_read32::<{ CORE_MASKS_G14X + 8 }>(), GFP_KERNEL)?;
                // Clusters per die * num dies
                ((id_counts_1 >> 8) & 0xff) * ((id_counts_1 >> 16) & 0xf)
            }
            a => {
                dev_err!(self.dev.as_ref(), "Unsupported GPU family {:#x}\n", a);
                return Err(ENODEV);
            }
        };

        let mut core_masks_packed = KVec::new();
        core_masks_packed.extend_from_slice(&core_mask_regs, GFP_KERNEL)?;

        dev_info!(self.dev.as_ref(), "Core masks: {:#x?}\n", core_masks_packed);

        let num_cores = match (family, variant) {
            // G15S (variant 3) and G15C (variant 4) have 10 cores per MGPU; ID_COUNTS_1[7:0]
            // does not report that on these variants.
            // TODO: read the real ID_COUNTS_1[7:0] on the M3 Pro to see why.
            (7, 3 | 4) => 10,
            _ => id_counts_1 & 0xff,
        };

        if num_cores > 32 {
            dev_err!(
                self.dev.as_ref(),
                "Too many cores per cluster ({} > 32)\n",
                num_cores
            );
            return Err(ENODEV);
        }

        if num_cores * num_clusters > (core_mask_regs.len() * 32) as u32 {
            dev_err!(
                self.dev.as_ref(),
                "Too many total cores ({} x {} > {})\n",
                num_clusters,
                num_cores,
                core_mask_regs.len() * 32
            );
            return Err(ENODEV);
        }

        let mut core_masks = KVec::new();
        let mut total_active_cores: u32 = 0;

        let max_core_mask = ((1u64 << num_cores) - 1) as u32;
        for _ in 0..num_clusters {
            let mask = core_mask_regs[0] & max_core_mask;
            core_masks.push(mask, GFP_KERNEL)?;
            for i in 0..core_mask_regs.len() {
                core_mask_regs[i] >>= num_cores;
                if i < (core_mask_regs.len() - 1) {
                    core_mask_regs[i] |= core_mask_regs[i + 1] << (32 - num_cores);
                }
            }
            total_active_cores += mask.count_ones();
        }

        if core_mask_regs.iter().any(|a| *a != 0) {
            dev_err!(
                self.dev.as_ref(),
                "Leftover core mask: {:#x?}\n",
                core_mask_regs
            );
            return Err(EIO);
        }

        let (gpu_rev, gpu_rev_id) = match (id_version >> 8) & 0xff {
            0x00 => (hw::GpuRevision::A0, hw::GpuRevisionID::A0),
            0x01 => (hw::GpuRevision::A1, hw::GpuRevisionID::A1),
            0x10 => (hw::GpuRevision::B0, hw::GpuRevisionID::B0),
            0x11 => (hw::GpuRevision::B1, hw::GpuRevisionID::B1),
            0x20 => (hw::GpuRevision::C0, hw::GpuRevisionID::C0),
            0x21 => (hw::GpuRevision::C1, hw::GpuRevisionID::C1),
            a => {
                dev_err!(self.dev.as_ref(), "Unknown GPU revision {}\n", a);
                return Err(ENODEV);
            }
        };

        // Preserve the public G13/G14 identity independently of the firmware
        // core identity: the legacy ABI distinguishes Max and Ultra by clusters.
        let (gpu_gen, gpu_variant, usc_generation, gpu_hal_generation) = if family <= 6 {
            let gpu_gen = match family {
                4 => hw::GpuGen::G13,
                5 | 6 => hw::GpuGen::G14,
                a => {
                    dev_err!(self.dev.as_ref(), "Unknown GPU generation {}\n", a);
                    return Err(ENODEV);
                }
            };
            let gpu_variant = match variant {
                1 => hw::GpuVariant::P,
                2 => hw::GpuVariant::G,
                3 => hw::GpuVariant::S,
                4 if num_clusters > 4 => hw::GpuVariant::D,
                4 => hw::GpuVariant::C,
                a => {
                    dev_err!(self.dev.as_ref(), "Unknown GPU variant {}\n", a);
                    return Err(ENODEV);
                }
            };
            (gpu_gen, gpu_variant, 2, identity::GpuHalGeneration::Legacy)
        } else {
            let id = identity::decode_gpu_identity(family, variant, num_dies as u8).ok_or(ENODEV)?;
            (id.gpu_gen, id.gpu_variant, id.usc_generation, id.gpu_hal_generation)
        };

        Ok(hw::GpuIdConfig {
            gpu_gen,
            gpu_variant,
            usc_generation,
            gpu_hal_generation,
            gpu_rev,
            gpu_rev_id,
            num_dies,
            num_clusters,
            num_cores,
            num_frags: num_cores, // Used to be id_counts_1[15:8] but does not work for G14X
            num_gps: (id_counts_2 >> 16) & 0xff,
            total_active_cores,
            core_masks,
            core_masks_packed,
        })
    }

    /// Get the identification of a G17 GPU from its registers.
    pub(crate) fn get_g17_id(&self) -> Result<G17Id> {
        let id_version = self.sgx_read32::<ID_VERSION>();
        let id_unk08 = self.sgx_read32::<ID_UNK08>();
        let id_counts_1 = self.sgx_read32::<ID_COUNTS_1>();
        let id_counts_2 = self.sgx_read32::<ID_COUNTS_2>();
        let id_unk18 = self.sgx_read32::<ID_UNK18>();
        let id_clusters = self.sgx_read32::<ID_CLUSTERS>();

        dev_info!(
            self.dev.as_ref(),
            "GPU ID registers: {:#x} {:#x} {:#x} {:#x} {:#x} {:#x}\n",
            id_version,
            id_unk08,
            id_counts_1,
            id_counts_2,
            id_unk18,
            id_clusters
        );

        // Later reads can fault while the firmware has gated the cores. Submission uses only
        // these saved values and never re-reads the registers.
        let perf_control = self.sgx_read32::<G17_PERF_CONTROL>() & 1;
        let perf_map = self.sgx_read32::<G17_PERF_MAP>();
        let mut core_mask_regs = [
            self.sgx_read32::<CORE_MASKS_G14X>(),
            self.sgx_read32::<{ CORE_MASKS_G14X + 4 }>(),
            self.sgx_read32::<{ CORE_MASKS_G14X + 8 }>(),
        ];
        dev_info!(self.dev.as_ref(), "Core masks: {:#x?}\n", core_mask_regs);

        let num_dies = (id_counts_1 >> 16) & 0xf;
        let num_clusters = ((id_counts_1 >> 8) & 0xff) * num_dies;
        let num_cores = id_counts_1 & 0xff;
        if num_cores == 0 || num_cores > 32 || num_cores * num_clusters > 3 * 32 {
            dev_err!(
                self.dev.as_ref(),
                "Invalid core counts ({} x {})\n",
                num_clusters,
                num_cores
            );
            return Err(ENODEV);
        }
        let (core_masks, _) =
            self.split_core_masks(&mut core_mask_regs, num_clusters, num_cores)?;
        let (gpu_rev, _) = self.gpu_revision(id_version)?;

        Ok(G17Id {
            family: ((id_version >> 24) & 0xff) as u8,
            variant: ((id_version >> 16) & 0xff) as u8,
            gpu_rev,
            num_dies,
            num_clusters,
            num_cores,
            core_masks,
            perf_control,
            perf_map: [perf_map & 0xf, (perf_map >> 16) & 0xf],
        })
    }

    /// Enable the G17 GPU-to-fabric bridge and check that it took effect.
    ///
    /// The GPU power domain must be on. The coprocessors cannot reach memory until this is done.
    pub(crate) fn enable_g17_bridge(&self) -> Result {
        let sgx = self.sgx.try_access().ok_or(ENODEV)?;
        for offset in G17_BRIDGE_CONTROL {
            let val = sgx.try_read32(offset)?;
            sgx.try_write32(val | G17_BRIDGE_ENABLE, offset)?;
            if sgx.try_read32(offset)? & G17_BRIDGE_ENABLE == 0 {
                dev_err!(
                    self.dev.as_ref(),
                    "GPU bridge {:#x} did not enable\n",
                    offset
                );
                return Err(EIO);
            }
        }
        Ok(())
    }

    /// Disable the G17 GPU-to-fabric bridge. Both coprocessors must be stopped.
    pub(crate) fn disable_g17_bridge(&self) {
        if let Some(sgx) = self.sgx.try_access() {
            for offset in G17_BRIDGE_CONTROL {
                if let Ok(val) = sgx.try_read32(offset) {
                    // Cannot fail: the offset was read just before.
                    let _ = sgx.try_write32(val & !G17_BRIDGE_ENABLE, offset);
                }
            }
        }
    }

    /// Power witness for the queue-configuration port, accessible while the cores are gated.
    pub(crate) fn g17_configuration_powered(&self) -> bool {
        self.sgx_read32::<G17_CORE_POWER>() & 0xf != 0
    }

    /// Enable dynamic power gating of idle G17 GPU cores. The firmware expects this before it is
    /// initialized.
    pub(crate) fn enable_g17_gating(&self) -> Result {
        let sgx = self.sgx.try_access().ok_or(ENODEV)?;
        let val = sgx.try_read32(G17_GATING_CONTROL)?;
        sgx.try_write32(val & !G17_GATING_DISABLE, G17_GATING_CONTROL)
    }

    /// Split the packed per-cluster core masks into one mask per cluster.
    ///
    /// Returns the masks and the total number of enabled cores.
    fn split_core_masks(
        &self,
        core_mask_regs: &mut [u32],
        num_clusters: u32,
        num_cores: u32,
    ) -> Result<(KVec<u32>, u32)> {
        let mut core_masks = KVec::new();
        let mut total_active_cores: u32 = 0;

        let max_core_mask = ((1u64 << num_cores) - 1) as u32;
        for _ in 0..num_clusters {
            let mask = core_mask_regs[0] & max_core_mask;
            core_masks.push(mask, GFP_KERNEL)?;
            for i in 0..core_mask_regs.len() {
                core_mask_regs[i] = core_mask_regs[i].checked_shr(num_cores).unwrap_or(0);
                if i < (core_mask_regs.len() - 1) {
                    core_mask_regs[i] |= core_mask_regs[i + 1]
                        .checked_shl(32 - num_cores)
                        .unwrap_or(0);
                }
            }
            total_active_cores += mask.count_ones();
        }

        if core_mask_regs.iter().any(|a| *a != 0) {
            dev_err!(
                self.dev.as_ref(),
                "Leftover core mask: {:#x?}\n",
                core_mask_regs
            );
            return Err(EIO);
        }

        Ok((core_masks, total_active_cores))
    }

    /// Decode the silicon revision from the ID version register.
    fn gpu_revision(&self, id_version: u32) -> Result<(hw::GpuRevision, hw::GpuRevisionID)> {
        Ok(match (id_version >> 8) & 0xff {
            0x00 => (hw::GpuRevision::A0, hw::GpuRevisionID::A0),
            0x01 => (hw::GpuRevision::A1, hw::GpuRevisionID::A1),
            0x10 => (hw::GpuRevision::B0, hw::GpuRevisionID::B0),
            0x11 => (hw::GpuRevision::B1, hw::GpuRevisionID::B1),
            0x20 => (hw::GpuRevision::C0, hw::GpuRevisionID::C0),
            0x21 => (hw::GpuRevision::C1, hw::GpuRevisionID::C1),
            a => {
                dev_err!(self.dev.as_ref(), "Unknown GPU revision {}\n", a);
                return Err(ENODEV);
            }
        })
    }

    /// Get the fault information from the MMU status register, if one occurred.
    pub(crate) fn get_fault_info(&self, cfg: &'static hw::HwConfig) -> Option<FaultInfo> {
        let g14x = cfg
            .gpu_core
            .map(|core| core as u32 >= hw::GpuCore::G14S as u32)
            .unwrap_or(false)
            || cfg.gpu_gen as u32 >= hw::GpuGen::G15 as u32;
        let g15 = cfg.gpu_gen == hw::GpuGen::G15;

        // SGX+0xd800 is a fault-instance selector, value ((s >> 2) & 0xf) | ((s & 3) << 18).
        // Writing it is optional and drm/asahi never writes it on G14X; a host write could race
        // with the firmware's own MMU fault handling and select the wrong instance, so G15 reads
        // the registers the same way.
        // TODO: which instance `s` to select, if any.

        let fault_info = if g14x {
            self.sgx_read64::<FAULT_INFO_G14X>()
        } else {
            self.sgx_read64::<FAULT_INFO>()
        };

        if fault_info & 1 == 0 {
            return None;
        }

        let fault_addr = if g15 {
            self.sgx_read64::<FAULT_ADDR_G14X>() & FAULT_ADDR_MASK_G15
        } else if g14x {
            self.sgx_read64::<FAULT_ADDR_G14X>()
        } else {
            fault_info >> 30
        };

        let unit_code = ((fault_info >> 9) & 0xff) as u8;
        let unit = match unit_code {
            _ if g15 => Self::fault_unit_g15(unit_code),
            0x00..=0x9f => match unit_code & 0xf {
                0x0 => FaultUnit::DCMP(unit_code >> 4),
                0x1 => FaultUnit::UL1C(unit_code >> 4),
                0x2 => FaultUnit::CMP(unit_code >> 4),
                0x3 => FaultUnit::GSL1(unit_code >> 4),
                0x4 => FaultUnit::IAP(unit_code >> 4),
                0x5 => FaultUnit::VCE(unit_code >> 4),
                0x6 => FaultUnit::TE(unit_code >> 4),
                0x7 => FaultUnit::RAS(unit_code >> 4),
                0x8 => FaultUnit::VDM(unit_code >> 4),
                0x9 => FaultUnit::PPP(unit_code >> 4),
                0xa => FaultUnit::IPF(unit_code >> 4),
                0xb => FaultUnit::IPF_CPF(unit_code >> 4),
                0xc => FaultUnit::VF(unit_code >> 4),
                0xd => FaultUnit::VF_CPF(unit_code >> 4),
                0xe => FaultUnit::ZLS(unit_code >> 4),
                _ => FaultUnit::Unknown(unit_code),
            },
            0xa1 => FaultUnit::dPM,
            0xa2 => FaultUnit::dCDM_KS(0),
            0xa3 => FaultUnit::dCDM_KS(1),
            0xa4 => FaultUnit::dCDM_KS(2),
            0xa5 => FaultUnit::dIPP,
            0xa6 => FaultUnit::dIPP_CS,
            0xa7 => FaultUnit::dVDM_CSD,
            0xa8 => FaultUnit::dVDM_SSD,
            0xa9 => FaultUnit::dVDM_ILF,
            0xaa => FaultUnit::dVDM_ILD,
            0xab => FaultUnit::dRDE(0),
            0xac => FaultUnit::dRDE(1),
            0xad => FaultUnit::FC,
            0xae => FaultUnit::GSL2,
            0xb0..=0xb7 => FaultUnit::GL2CC_META(unit_code & 0xf),
            0xb8 => FaultUnit::GL2CC_MB,
            0xd0..=0xdf if g14x => match unit_code & 0xf {
                0x0 => FaultUnit::gCDM_CS,
                0x1 => FaultUnit::gCDM_ID,
                0x2 => FaultUnit::gCDM_CSR,
                0x3 => FaultUnit::gCDM_CSW,
                0x4 => FaultUnit::gCDM_CTXR,
                0x5 => FaultUnit::gCDM_CTXW,
                0x6 => FaultUnit::gIPP,
                0x7 => FaultUnit::gIPP_CS,
                0x8 => FaultUnit::gKSM_RCE,
                _ => FaultUnit::Unknown(unit_code),
            },
            0xe0..=0xff if g14x => match unit_code & 0xf {
                0x0 => FaultUnit::gPM_SP((unit_code >> 4) & 1),
                0x1 => FaultUnit::gVDM_CSD_SP((unit_code >> 4) & 1),
                0x2 => FaultUnit::gVDM_SSD_SP((unit_code >> 4) & 1),
                0x3 => FaultUnit::gVDM_ILF_SP((unit_code >> 4) & 1),
                0x4 => FaultUnit::gVDM_TFP_SP((unit_code >> 4) & 1),
                0x5 => FaultUnit::gVDM_MMB_SP((unit_code >> 4) & 1),
                0x6 => FaultUnit::gRDE0_SP((unit_code >> 4) & 1),
                _ => FaultUnit::Unknown(unit_code),
            },
            0xe0..=0xff if !g14x => match unit_code & 0xf {
                0x0 => FaultUnit::gPM_SP((unit_code >> 4) & 1),
                0x1 => FaultUnit::gVDM_CSD_SP((unit_code >> 4) & 1),
                0x2 => FaultUnit::gVDM_SSD_SP((unit_code >> 4) & 1),
                0x3 => FaultUnit::gVDM_ILF_SP((unit_code >> 4) & 1),
                0x4 => FaultUnit::gVDM_TFP_SP((unit_code >> 4) & 1),
                0x5 => FaultUnit::gVDM_MMB_SP((unit_code >> 4) & 1),
                0x6 => FaultUnit::gCDM_CS_KS0_SP((unit_code >> 4) & 1),
                0x7 => FaultUnit::gCDM_CS_KS1_SP((unit_code >> 4) & 1),
                0x8 => FaultUnit::gCDM_CS_KS2_SP((unit_code >> 4) & 1),
                0x9 => FaultUnit::gCDM_KS0_SP((unit_code >> 4) & 1),
                0xa => FaultUnit::gCDM_KS1_SP((unit_code >> 4) & 1),
                0xb => FaultUnit::gCDM_KS2_SP((unit_code >> 4) & 1),
                0xc => FaultUnit::gIPP_SP((unit_code >> 4) & 1),
                0xd => FaultUnit::gIPP_CS_SP((unit_code >> 4) & 1),
                0xe => FaultUnit::gRDE0_SP((unit_code >> 4) & 1),
                0xf => FaultUnit::gRDE1_SP((unit_code >> 4) & 1),
                _ => FaultUnit::Unknown(unit_code),
            },
            _ => FaultUnit::Unknown(unit_code),
        };

        let reason = match (fault_info >> 1) & 0x7 {
            0 => FaultReason::Unmapped,
            1 => FaultReason::AfFault,
            2 => FaultReason::WriteOnly,
            3 => FaultReason::ReadOnly,
            4 => FaultReason::NoAccess,
            a => FaultReason::Unknown(a as u8),
        };

        Some(FaultInfo {
            address: fault_addr << 6,
            sideband: ((fault_info >> 23) & 0x7f) as u8,
            vm_slot: ((fault_info >> 17) & 0x3f) as u32,
            unit_code,
            unit,
            level: ((fault_info >> 7) & 3) as u8,
            unk_5: ((fault_info >> 5) & 3) as u8,
            read: (fault_info & (1 << 4)) != 0,
            reason,
        })
    }
}
