// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! T8152 probe: the G18 platform driver, its parameter gates, device
//! preparation and the stage-10 DRM path.
//!
//! Items copied from the J873 module-1134 driver.rs (provenance in
//! the port's extraction ledger), then adapted to this driver's core: G18 is
//! its own platform driver, like G16; the shared probe body
//! ([`AsahiPlatformDriver::probe_g18`]) owns the DRM device and its
//! registration, and the G18 adapter is a `gpu::Gpu`. The parameter gates and
//! every log line are unchanged.

use kernel::{
    device::Core,
    dma::{Device, DmaMask},
    of,
    platform,
    prelude::*,
    sync::{aref::ARef, Arc},
};

use crate::g18::boot as g18_live_boot;
use crate::g18::{hw, identity, regs};
use crate::{
    driver::{AsahiDrmDevice, AsahiPlatformDriver, AsahiPlatformDriverData},
    gpu,
};

/// Name of the G18 platform driver.
pub(crate) const DRIVER_NAME: &CStr = c"aurora-g18";

kernel::of_device_table!(
    OF_TABLE,
    <Driver as platform::Driver>::IdInfo,
    [(of::DeviceId::new(c"apple,agx-t8152"), &crate::hw::t8152::T8152)]
);

/// The G18 platform driver: the shared [`AsahiPlatformDriver`] probed through
/// [`AsahiPlatformDriver::probe_g18`].
pub(crate) struct Driver;

/// What a bound G18 device holds until unbind.
pub(crate) struct G18Bound<'bound> {
    /// The firmware owner, shut down on unbind before devres releases its resources.
    pub(crate) runtime: Arc<dyn G18Runtime>,
    /// The DMA binding of a diagnostic stage, which has no DRM device.
    pub(crate) binding: Option<crate::binding::Owner>,
    /// The shared driver data of the application stage's render node.
    pub(crate) driver: Option<AsahiPlatformDriverData<'bound>>,
}

impl platform::Driver for Driver {
    type IdInfo = &'static crate::hw::t8152::SocConfig;
    type Data<'bound> = G18Bound<'bound>;
    const OF_ID_TABLE: Option<of::IdTable<Self::IdInfo>> = Some(&OF_TABLE);

    /// The firmware session is retained until reboot once started.
    fn suppress_bind_attrs() -> bool {
        true
    }

    fn probe<'bound>(
        pdev: &'bound platform::Device<Core<'_>>,
        info: Option<&'bound Self::IdInfo>,
    ) -> impl PinInit<Self::Data<'bound>, Error> + 'bound {
        let soc = *info.ok_or(ENODEV)?;
        // The memory limits the parameter buffer's growth is admitted
        // against (memacct), as the other generations' probe sets them.
        crate::memacct::LEDGER.configure();
        AsahiPlatformDriver::probe_g18(pdev, soc)
    }

    fn unbind<'bound>(_pdev: &'bound platform::Device<Core<'_>>, this: Pin<&Self::Data<'bound>>) {
        this.runtime.shutdown();
        if let Some(binding) = &this.binding {
            binding.close();
        }
        if let Some(driver) = &this.driver {
            driver.close_binding();
        }
    }
}

/// A started G18 session: its owner and, for the application stage, the GPU
/// the render node is registered over.
pub(crate) struct G18Started {
    pub(crate) runtime: Arc<dyn G18Runtime>,
    pub(crate) gpu: Option<Arc<dyn gpu::Gpu>>,
}

/// Owner object returned by the G18 live-boot module.
///
/// The live-boot module implements this for its session type. The driver
/// keeps it for the device's lifetime and calls [`G18Runtime::shutdown`] on
/// unbind, before devres releases the SGX mapping and the mailbox links.
pub(crate) trait G18Runtime: Send + Sync {
    /// Quiesce the firmware, both ASC processors and every host mapping.
    fn shutdown(&self);
}

/// Everything the T8152 probe gate hands to the G18 live-boot entry point.
///
/// Built only by [`prepare_t8152_device`], and only when `g18_initdata_test`
/// is non-zero. Holding one means:
/// - the three GPU PSD services (sgx 3, gfx-asc 81, gfx1-asc 82) were read
///   once, read-only, and their samples are in [`T8152Probe::psd`];
/// - the SGX window is mapped in `registers`. Its ID block was read only if
///   the sgx PSD sample proved the block powered: `observed` is `Some`
///   exactly then;
/// - for the firmware stages 2..4 the loader's G18G firmware receipt matched
///   and the 42-bit DMA mask is set. Stages 1 and 90 require neither.
///
/// No firmware, ASC CPU, PSD request, DMA or UAT state has been touched.
#[allow(dead_code)]
pub(crate) struct T8152Probe {
    /// The matched SoC entry (`hw::agx3::T8152`).
    pub(crate) soc: &'static hw::agx3::SocConfig,
    /// Static identity decoded from the SoC entry (G18G, HAL400, SKSM).
    pub(crate) identity: identity::GpuIdentity,
    /// The `g18_initdata_test` stage, always non-zero here.
    pub(crate) stage: u32,
    /// Whether /chosen `aurora,g18g-gpu-firmware-profile` matched.
    pub(crate) firmware_profile_admitted: bool,
    /// The InitData profile of the admitted firmware build.
    pub(crate) firmware_build: Option<crate::g18::fw::initdata::RootProfile>,
    /// Read-only samples of PSD services 3, 81 and 82, in that order.
    pub(crate) psd: [regs::T8152PsdSample; 3],
    /// SGX mapping. Every further SGX read must again follow a powered
    /// `regs::t8152_prove_sgx_powered()` result.
    pub(crate) registers: regs::Resources,
    /// Hardware identity, only when the SGX PSD proved the block powered.
    pub(crate) observed: Option<hw::GpuIdConfig>,
    /// The DRM device, not yet registered. The live-boot module allocates
    /// its kernel objects on it; the shared probe registers it at stage 10.
    pub(crate) drm: ARef<AsahiDrmDevice>,
}

/// Loader receipts the G18G firmware stages require. The loader writes one
/// only after hashing both loaded TEXT images (b000 GFX 0x50000, b010 GFX1
/// 0x54000) against that build's images (plan step 0.2): 26A428 (macOS 27.0
/// and 27.0.1) or 26B123 (macOS 27.2).
const G18G_FIRMWARE_PROFILES: [(&[u8], crate::g18::fw::initdata::RootProfile); 2] = [
    (b"G18G-RTKit-3514.0.15.release\0", crate::g18::fw::initdata::RootProfile::J873G18G26A428),
    (b"G18G-RTKit-3514.41.1.release\0", crate::g18::fw::initdata::RootProfile::J873G18G26B123),
];

/// T8152 probe gate. With `g18_initdata_test=0` (the default) this is a
/// complete no-op: it logs and refuses with ENODEV before any MMIO, DMA
/// mask, reserved-memory, mailbox or processor access, just as M5 refuses by
/// default. Unlike the M5 default it also skips the read-only SGX identity
/// probe, because J873 cold-boots with the GPU PSD services idle, where an
/// SGX read can raise an SError.
pub(crate) fn admit_t8152(pdev: &platform::Device<Core<'_>>) -> Result {
    let stage = crate::g18::params::g18_initdata_test.value();
    if stage == 0 {
        dev_info!(
            pdev.as_ref(),
            "G18G: T8152 matched; g18_initdata_test=0 refuses before any MMIO, DMA, mailbox or processor access\n"
        );
        return Err(ENODEV);
    }
    if !g18_live_boot::stage_is_known(stage) {
        dev_err!(
            pdev.as_ref(),
            "G18G: g18_initdata_test={} is not a known stage (1-9, 90-93); refused before any MMIO\n",
            stage
        );
        return Err(EINVAL);
    }
    let variant = crate::g18::params::g18_compute_variant.value();
    if variant > 2 || (variant != 0 && stage != g18_live_boot::STAGE_COMPUTE_ADD) {
        dev_err!(
            pdev.as_ref(),
            "G18G: g18_compute_variant={} needs 0, or 1/2 with g18_initdata_test=8; refused before any MMIO\n",
            variant
        );
        return Err(EINVAL);
    }
    let render_variant = crate::g18::params::g18_render_variant.value();
    let render_profile = crate::g18::params::g18_render_profile.value();
    let sma_minimum = crate::g18::params::g18_sma_minimum.value();
    let sma_ideal = crate::g18::params::g18_sma_ideal.value();
    let native_barrier = crate::g18::params::g18_native_barrier.value();
    if native_barrier > 1 || (native_barrier != 0 && (sma_ideal != 1 || sma_minimum != 0 ||
        stage != crate::g18::boot::render_stage::STAGE_RENDER || render_variant != 0 || render_profile != 4)) {
        dev_err!(pdev.as_ref(), "G18G: native barrier probe requires ideal SMA stage9 variant0 profile4; refused before MMIO\n");
        return Err(EINVAL);
    }
    if sma_ideal > 1 || (sma_ideal != 0 && (sma_minimum != 0 ||
        stage != crate::g18::boot::render_stage::STAGE_RENDER || render_variant != 0 || render_profile != 4)) {
        dev_err!(pdev.as_ref(), "G18G: SMA ideal probe requires stage9 variant0 profile4 and minimum=0; refused before MMIO\n");
        return Err(EINVAL);
    }
    // Explicit fixed-minimum comparison for the retained first application
    // draw, as well as the original H1 witness. This is not dynamic sizing.
    let minimum_probe = render_variant == 0 &&
        ((stage == crate::g18::boot::render_stage::STAGE_RENDER && render_profile == 4) ||
         (stage == g18_live_boot::STAGE_DRM && render_profile == 1));
    let stall_trace = crate::g18::params::g18_graphics_stall_trace.value();
    if stall_trace > 1 || (stall_trace != 0 && !minimum_probe) {
        dev_err!(pdev.as_ref(), "G18G: graphics stall tracing requires stage9/profile4 or stage10/profile1, variant0; refused before MMIO\n");
        return Err(EINVAL);
    }
    let high_vdm = crate::g18::params::g18_vdm_high_alias.value();
    if high_vdm > 1 || (high_vdm != 0 &&
        (stage != crate::g18::boot::render_stage::STAGE_RENDER || render_variant != 0 ||
         render_profile != 4 || sma_minimum != 1 || sma_ideal != 0 || stall_trace != 1)) {
        dev_err!(pdev.as_ref(), "G18G: high VDM alias requires stage9/profile4/variant0, minimum SMA and stall tracing; refused before MMIO\n");
        return Err(EINVAL);
    }
    let high_ppp = crate::g18::params::g18_ppp_high_alias.value();
    if high_ppp > 1 || (high_ppp != 0 && high_vdm != 1) {
        dev_err!(pdev.as_ref(), "G18G: high PPP aliases require the bounded high VDM control; refused before MMIO\n");
        return Err(EINVAL);
    }
    if sma_minimum > 1 || (sma_minimum != 0 && !minimum_probe) {
        dev_err!(pdev.as_ref(), "G18G: SMA minimum probe requires stage9/profile4 or stage10/profile1, variant0; refused before MMIO\n");
        return Err(EINVAL);
    }
    let usc_statistics = crate::g18::params::g18_usc_statistics.value();
    if usc_statistics > 1 || (usc_statistics != 0 &&
        (stage != crate::g18::boot::render_stage::STAGE_RENDER || render_variant != 0)) {
        dev_err!(pdev.as_ref(), "G18G: USC statistics require the stage9 colour fixture; refused before MMIO\n");
        return Err(EINVAL);
    }
    if stage == crate::g18::boot::render_stage::STAGE_RENDER {
        crate::g18::boot::render_stage::admit(pdev.as_ref(), render_variant, render_profile)?;
    } else if render_variant != 0 || render_profile != 1 {
        return Err(EINVAL);
    }
    let admit_msaa4 = crate::g18::params::g18_admit_msaa4.value();
    if admit_msaa4 > 1 || (admit_msaa4 != 0 && stage != g18_live_boot::STAGE_DRM) {
        dev_err!(pdev.as_ref(), "G18G: g18_admit_msaa4={} needs 0, or 1 with g18_initdata_test=10; refused before MMIO\n", admit_msaa4);
        return Err(EINVAL);
    }
    if stage == g18_live_boot::STAGE_DRM && crate::g18::params::g18_admit_compute.value() > 1 {
        return Err(EINVAL);
    }
    Ok(())
}

/// Start the stage that [`admit_t8152`] admitted, on the unregistered `drm`.
pub(crate) fn probe_t8152(
    pdev: &platform::Device<Core<'_>>,
    drm: &AsahiDrmDevice,
    soc: &'static hw::agx3::SocConfig,
) -> Result<G18Started> {
    let stage = crate::g18::params::g18_initdata_test.value();
    if stage == g18_live_boot::STAGE_DRM {
        return probe_t8152_drm(pdev, drm, soc, stage);
    }
    let probe = prepare_t8152_device(pdev, drm, soc, stage)?;
    Ok(G18Started { runtime: g18_live_boot_start(pdev, probe)?, gpu: None })
}

/// Start the G18 session the render node is registered over.
fn probe_t8152_drm(
    pdev: &platform::Device<Core<'_>>,
    drm: &AsahiDrmDevice,
    soc: &'static hw::agx3::SocConfig,
    stage: u32,
) -> Result<G18Started> {
    let mut probe = prepare_t8152_device(pdev, drm, soc, stage)?;
    let observed = probe.observed.take();
    let (owner, runtime) = g18_live_boot::start_drm(pdev, probe)?;
    let config = crate::g18::drm::G18DrmConfig {
        admit_compute: crate::g18::params::g18_admit_compute.value() != 0,
        admit_msaa4: crate::g18::params::g18_admit_msaa4.value() != 0,
        admit_graphics_scratch:
            crate::g18::params::g18_admit_graphics_scratch.value() != 0,
    };
    let gpu: Arc<dyn gpu::Gpu> = Arc::new(
        crate::g18::drm::G18DrmGpu::new(runtime, observed.as_ref(), config)?,
        GFP_KERNEL,
    )?;
    gpu.init()?;
    Ok(G18Started { runtime: owner, gpu: Some(gpu) })
}

fn prepare_t8152_device(
    pdev: &platform::Device<Core<'_>>,
    drm: &AsahiDrmDevice,
    soc: &'static hw::agx3::SocConfig,
    stage: u32,
) -> Result<T8152Probe> {
    let identity =
        identity::decode_gpu_identity(soc.hw_family, soc.hw_variant, soc.num_dies as u8)
            .ok_or(EINVAL)?;
    if soc.chip_id != 0x8152
        || identity.gpu_gen != hw::GpuGen::G18
        || identity.gpu_variant != identity::GpuVariant::G
        || identity.uat_input_address_bits != soc.uat_ias
    {
        return Err(EINVAL);
    }
    dev_info!(
        pdev.as_ref(),
        "G18G: T8152 stage {}: {:?}{:?} USC gen {} HAL {:?} {:?} roles {:?} submission, {}-bit UAT roots\n",
        stage,
        identity.gpu_gen,
        identity.gpu_variant,
        identity.usc_generation,
        identity.gpu_hal_generation,
        identity.firmware_roles,
        identity.submission_transport,
        soc.uat_ias
    );

    let firmware_build = match of::chosen() {
        Some(chosen) => {
            let profile: Result<KVec<u8>> =
                chosen.get_property(c"aurora,g18g-gpu-firmware-profile");
            profile.ok().and_then(|profile| {
                G18G_FIRMWARE_PROFILES.iter()
                    .find(|(name, _)| profile.as_slice() == *name)
                    .map(|(_, build)| *build)
            })
        }
        None => None,
    };
    let firmware_profile_admitted = firmware_build.is_some();
    match firmware_build {
        Some(build) => dev_info!(pdev.as_ref(), "G18G: firmware receipt admitted {:?}\n", build),
        None => dev_info!(pdev.as_ref(), "G18G: firmware receipt absent or different\n"),
    }
    if g18_live_boot::stage_runs_firmware(stage) && !firmware_profile_admitted {
        dev_err!(
            pdev.as_ref(),
            "G18G: stage {} needs a G18G firmware receipt in /chosen aurora,g18g-gpu-firmware-profile\n",
            stage
        );
        return Err(ENODEV);
    }

    // PSD state first, read-only. Nothing below reads SGX unless the sgx
    // service proves the block active and acknowledged.
    let mut psd = [regs::T8152PsdSample {
        service: regs::T8152PsdService::Sgx,
        state: 0,
        ack: 0,
    }; 3];
    for (slot, service) in psd.iter_mut().zip(regs::T8152PsdService::ALL) {
        *slot = regs::t8152_read_psd(pdev, service)?;
        dev_info!(
            pdev.as_ref(),
            "G18G: PSD service {} ({:#x}) state={:#x} power state {} ack={:#x} -> {}\n",
            service.service_id(),
            service.physical_base(),
            slot.state,
            slot.power_state(),
            slot.ack,
            (if slot.powered() { "powered" } else { "not powered" })
        );
    }

    let registers = regs::Resources::new(pdev)?;
    let observed = match regs::t8152_prove_sgx_powered(pdev) {
        Ok(powered) => Some(registers.get_gpu_id_t8152(&powered, soc)?),
        Err(error) if error == regs::t8152_not_powered() => {
            dev_info!(
                pdev.as_ref(),
                "G18G: SGX identity not read (sgx PSD idle); identified by chip id {:#x} and compatible\n",
                soc.chip_id
            );
            None
        }
        Err(error) => return Err(error),
    };

    if g18_live_boot::stage_runs_firmware(stage) {
        // J873 DRAM ends at 0x106_0000_0000, inside a 42-bit output space.
        unsafe { pdev.dma_set_mask_and_coherent(DmaMask::try_new(soc.uat_oas)?)? };
    }

    // The device stays unregistered until the shared probe registers it at
    // stage 10; diagnostic stages never register it.
    let drm: ARef<AsahiDrmDevice> = drm.into();
    Ok(T8152Probe {
        soc,
        identity,
        stage,
        firmware_profile_admitted,
        firmware_build,
        psd,
        registers,
        observed,
        drm,
    })
}

/// G18 live-boot entry point: the staged bring-up in `g18_live_boot.rs`.
///
/// `probe.stage` is `g18_initdata_test`, already checked by
/// [`g18_live_boot::stage_is_known`]:
/// - 1: read-only probe (PSD, SGX ID, wrapper CPU state, post-office
///   status), then ENODEV;
/// - 2, 3, 4, 5, 6 (REQUIRE USER APPROVAL, plus the mailbox provider's own
///   `apple_mailbox.ascwrap_v8_cpu_control=1`): ASC CPU start and RTKit,
///   init-data ACKs, openings + idle wake + heartbeat. Stages 2 and 3 end in
///   ENODEV with every firmware owner retained; stages 4/5/6 return the owner.
///   Stage 5 registers one empty queue; stage 6 binds its scheduler. No GPU jobs;
/// - 90 (REQUIRES USER APPROVAL): PMGR2 PSD power-on only, then ENODEV.
///
/// On success the driver keeps the owner and calls
/// [`G18Runtime::shutdown`] on unbind.
fn g18_live_boot_start(
    pdev: &platform::Device<Core<'_>>,
    probe: T8152Probe,
) -> Result<Arc<dyn G18Runtime>> {
    g18_live_boot::start(pdev, probe)
}
