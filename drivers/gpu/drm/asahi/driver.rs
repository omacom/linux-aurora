// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Top-level GPU driver implementation.

use kernel::{
    c_str,
    device::Core,
    dma::{
        Device,
        DmaMask, //
    },
    drm,
    drm::ioctl,
    of,
    platform,
    workqueue::{self, impl_has_work, new_work, Work, WorkItem},
    prelude::*,
    sync::{
        aref::ARef,
        Arc, SetOnce, //
    },
};

use crate::{
    debug,
    drm_gpu,
    file,
    g15,
    g17,
    gem::AsahiObject,
    gpu,
    hw,
    identity,
    regs, //
    t8122_admission,
};

use kernel::macros::vtable;

/// Holds a reference to the top-level `GpuManager` object.
#[pin_data]
pub(crate) struct AsahiData {
    pub(crate) gpu: SetOnce<drm_gpu::Backend>,
    pub(crate) is_m3: bool,
    pub(crate) completion: SetOnce<Arc<crate::m3_completion::Completion>>,
    pub(crate) pdev: ARef<platform::Device>,
    pub(crate) resources: Option<regs::Resources>,
    /// Kernel GEM objects are marked held for the device (`gem::set_mapping_held`). Only the
    /// G17P runtime sets this.
    pub(crate) hold_kernel_pages: bool,
    #[pin]
    g16_completion_work: Work<AsahiDevice, 3>,

}

impl AsahiData {
    pub(crate) fn new(pdev: &platform::Device<Core>, resources: Option<regs::Resources>, is_m3: bool) -> impl PinInit<Self, Error> {
        let pdev: ARef<platform::Device> = pdev.into();
        try_pin_init!(Self {
            gpu: SetOnce::new(),
            is_m3,
            completion: SetOnce::new(),
            pdev,
            resources,
            hold_kernel_pages: false,
            g16_completion_work <- new_work!("AsahiData::g16_completion_work"),
        })
    }

    /// Driver data of a G17P device.
    fn new_g17(pdev: &platform::Device<Core>, resources: regs::Resources) -> impl PinInit<Self, Error> {
        let pdev: ARef<platform::Device> = pdev.into();
        try_pin_init!(Self {
            gpu: SetOnce::new(),
            is_m3: false,
            completion: SetOnce::new(),
            pdev,
            resources: Some(resources),
            hold_kernel_pages: true,
            g16_completion_work <- new_work!("AsahiData::g16_completion_work"),
        })
    }

    /// The G17P runtime, if this device runs it.
    pub(crate) fn g17(&self) -> Option<&Arc<g17::Gpu>> {
        match self.gpu.as_ref() {
            Some(drm_gpu::Backend::G17(gpu)) => Some(gpu),
            _ => None,
        }
    }

    /// The G15 generation, if this device runs it.
    pub(crate) fn g15(&self) -> Option<&Arc<g15::Gpu>> {
        match self.gpu.as_ref() {
            Some(drm_gpu::Backend::G15(gpu)) => Some(gpu),
            _ => None,
        }
    }

    /// Runtime construction can use GEM before the GPU backend exists. All
    /// DRM operations fail with ENODEV until probe publishes the backend once.
    pub(crate) fn gpu(&self) -> Result<&dyn drm_gpu::DrmGpu> {
        self.gpu.as_ref().map(|backend| backend.gpu()).ok_or(ENODEV)
    }
}

unsafe impl Send for AsahiData {}
unsafe impl Sync for AsahiData {}

impl_has_work! { impl HasWork<AsahiDevice, 3> for AsahiData { self.g16_completion_work } }
impl WorkItem<3> for AsahiData {
    type Pointer = ARef<AsahiDevice>;
    fn run(dev: ARef<AsahiDevice>) {
        if let Ok(gpu) = dev.gpu() { gpu.service_g16_jobs(); }
    }
}
pub(crate) fn queue_g16_completion_worker(dev: ARef<AsahiDevice>) {
    let _ = workqueue::system_highpri().enqueue::<ARef<AsahiDevice>, 3>(dev);
}

#[allow(dead_code)]
enum AsahiRuntime {
    Legacy(ARef<drm::Device<AsahiDriver>>),
    M3(crate::m3_drm::Registered),
    G16(crate::g16_drm::Registered),
    G17(ARef<drm::Device<AsahiDriver>>, Arc<g17::Gpu>),
    /// The G15 generation (`g15`): its DRM device and GPU, retained until reboot.
    G15(ARef<drm::Device<AsahiDriver>>, Arc<g15::Gpu>),
    /// A G15 probe that failed once the firmware had been touched: the device stays bound
    /// (unbinding would detach and power off the GFX domain under the running firmware), the
    /// firmware session is retained until reboot and no DRM device exists.
    G15Retained,
}

pub(crate) struct AsahiDriver {
    runtime: AsahiRuntime,
}

/// A matched device either has a complete legacy hardware configuration or is
/// an exact AGX3 identification target whose firmware boundary remains closed.
pub(crate) enum ProbeConfig {
    Supported(&'static hw::HwConfig),
    /// AGX3 (G15) target: chip identification and the read-only
    /// topology decode run at probe time, then the probe fails closed before
    /// DMA setup, ASC start, or any MMIO write.
    Agx3Diagnostic(&'static hw::agx3::SocConfig),
    /// G15 (T6030, T6031): the G15 generation, driving the firmware the boot loader left
    /// loaded.
    G15(&'static g15::Config),
    /// G17 (T8140): the G17P firmware runtime.
    G17(&'static g17::Config),
}

unsafe impl Send for AsahiDriver {}
unsafe impl Sync for AsahiDriver {}

/// Convenience type alias for the DRM device type for this driver.
pub(crate) type AsahiDevice = drm::device::Device<AsahiDriver>;
pub(crate) type AsahiDevRef = ARef<AsahiDevice>;

/// DRM Driver metadata
const INFO: drm::driver::DriverInfo = drm::driver::DriverInfo {
    major: 0,
    minor: 2,
    patchlevel: 0,
    name: c_str!("asahi"),
    desc: c_str!("Apple AGX Graphics"),
};

/// DRM Driver implementation for `AsahiDriver`.
#[vtable]
impl drm::driver::Driver for AsahiDriver {
    /// Our `DeviceData` type, reference-counted
    type Data = AsahiData;
    /// Our `File` type.
    type File = file::File;
    /// Our `Object` type.
    type Object = drm::gem::shmem::Object<AsahiObject>;

    const INFO: drm::driver::DriverInfo = INFO;
    /// Open files and exported buffers pin the module while G17P deferred cleanup can run.
    const MODULE: Option<&'static kernel::ThisModule> = Some(&crate::THIS_MODULE);
    const FEATURES: u32 = drm::driver::FEAT_GEM
        | drm::driver::FEAT_RENDER
        | drm::driver::FEAT_SYNCOBJ
        | drm::driver::FEAT_SYNCOBJ_TIMELINE
        | drm::driver::FEAT_GEM_GPUVA;

    kernel::declare_drm_ioctls! {
        (ASAHI_GET_PARAMS,      drm_asahi_get_params,
                          ioctl::RENDER_ALLOW, crate::file::File::get_params),
        (ASAHI_GET_TIME,        drm_asahi_get_time,
            ioctl::AUTH | ioctl::RENDER_ALLOW, crate::file::File::get_time),
        (ASAHI_VM_CREATE,       drm_asahi_vm_create,
            ioctl::AUTH | ioctl::RENDER_ALLOW, crate::file::File::vm_create),
        (ASAHI_VM_DESTROY,      drm_asahi_vm_destroy,
            ioctl::AUTH | ioctl::RENDER_ALLOW, crate::file::File::vm_destroy),
        (ASAHI_VM_BIND,         drm_asahi_vm_bind,
            ioctl::AUTH | ioctl::RENDER_ALLOW, crate::file::File::vm_bind),
        (ASAHI_GEM_CREATE,      drm_asahi_gem_create,
            ioctl::AUTH | ioctl::RENDER_ALLOW, crate::file::File::gem_create),
        (ASAHI_GEM_MMAP_OFFSET, drm_asahi_gem_mmap_offset,
            ioctl::AUTH | ioctl::RENDER_ALLOW, crate::file::File::gem_mmap_offset),
        (ASAHI_GEM_BIND_OBJECT, drm_asahi_gem_bind_object,
            ioctl::AUTH | ioctl::RENDER_ALLOW, crate::file::File::gem_bind_object),
        (ASAHI_QUEUE_CREATE,    drm_asahi_queue_create,
            ioctl::AUTH | ioctl::RENDER_ALLOW, crate::file::File::queue_create),
        (ASAHI_QUEUE_DESTROY,   drm_asahi_queue_destroy,
            ioctl::AUTH | ioctl::RENDER_ALLOW, crate::file::File::queue_destroy),
        (ASAHI_SUBMIT,          drm_asahi_submit,
            ioctl::AUTH | ioctl::RENDER_ALLOW, crate::file::File::submit),
        (ASAHI_GEM_MADVISE,     drm_asahi_gem_madvise,
            ioctl::AUTH | ioctl::RENDER_ALLOW, crate::file::File::gem_madvise),
    }
}

// OF Device ID table.s
kernel::of_device_table!(
    OF_TABLE,
    MODULE_OF_TABLE,
    <AsahiDriver as platform::Driver>::IdInfo,
    [
        (
            of::DeviceId::new(c_str!("apple,agx-t8103")),
            ProbeConfig::Supported(&hw::t8103::HWCONFIG)
        ),
        (
            of::DeviceId::new(c_str!("apple,agx-t8112")),
            ProbeConfig::Supported(&hw::t8112::HWCONFIG)
        ),
        (
            of::DeviceId::new(c_str!("apple,agx-t6000")),
            ProbeConfig::Supported(&hw::t600x::HWCONFIG_T6000)
        ),
        (
            of::DeviceId::new(c_str!("apple,agx-t6001")),
            ProbeConfig::Supported(&hw::t600x::HWCONFIG_T6001)
        ),
        (
            of::DeviceId::new(c_str!("apple,agx-t6002")),
            ProbeConfig::Supported(&hw::t600x::HWCONFIG_T6002)
        ),
        (
            of::DeviceId::new(c_str!("apple,agx-t6020")),
            ProbeConfig::Supported(&hw::t602x::HWCONFIG_T6020)
        ),
        (
            of::DeviceId::new(c_str!("apple,agx-t6021")),
            ProbeConfig::Supported(&hw::t602x::HWCONFIG_T6021)
        ),
        (
            of::DeviceId::new(c_str!("apple,agx-t6022")),
            ProbeConfig::Supported(&hw::t602x::HWCONFIG_T6022)
        ),
        // AGX3 (G15) identification targets. These intentionally select no
        // HwConfig: unproved power/MMIO/firmware-tuning fields must never be
        // represented by copied or zero-filled legacy values. A handed-over
        // T8122 is admitted by the M3 runtime and refused while its table is
        // incomplete; the others fail closed before any firmware handoff.
        // The M4 (G16) and M5 bring-up runtimes are not bound in this kernel.
        // The M3 Pro/Max (T6030/T6031, G15) generation and the A18 Pro
        // (T8140, G17P) runtime are bound below.
        (
            of::DeviceId::new(c_str!("apple,agx-t8122")),
            ProbeConfig::Agx3Diagnostic(&hw::agx3::T8122)
        ),
        (
            of::DeviceId::new(c_str!("apple,agx-t6030")),
            ProbeConfig::G15(&g15::hw::t6030::CONFIG)
        ),
        (
            of::DeviceId::new(c_str!("apple,agx-t6031")),
            ProbeConfig::G15(&g15::hw::t6031::CONFIG)
        ),
        (
            of::DeviceId::new(c_str!("apple,agx-t6032")),
            ProbeConfig::Agx3Diagnostic(&hw::agx3::T6032)
        ),
        (
            of::DeviceId::new(c_str!("apple,agx-t8140")),
            ProbeConfig::G17(&hw::t8140::CONFIG)
        ),
    ]
);

/// Start the M3 runtime on a T8122 (M3, G15G) whose GPU the boot loader handed over: only after
/// its identity gate passes, and only on a board the T8122 table allows. The runtime then admits
/// the boot loader's description and refuses the SoC while its table is incomplete
/// (`m3_soc::T8122`).
fn start_t8122(pdev: &platform::Device<Core>) -> Result<crate::m3_drm::Registered> {
    // This gate reads only DT properties.
    let identity = t8122_admission::read_identity(pdev).map_err(|error| {
        dev_info!(
            pdev.as_ref(),
            "G15G: T8122 identity or ABI rejected ({:?}); startup refused\n",
            error
        );
        ENODEV
    })?;
    dev_info!(
        pdev.as_ref(),
        "G15G: T8122 revision 2.0 identity admitted ({} active / {} slots)\n",
        identity.active_cores,
        identity.core_slots
    );
    if !crate::m3_params::t8122_backend(pdev) {
        return Err(ENODEV);
    }
    crate::m3_drm::Registered::start(pdev, &crate::m3_soc::T8122)
}

fn refuse_agx3_probe(pdev: &platform::Device<Core>, soc: &'static hw::agx3::SocConfig) -> Error {

    let Some(expected) =
        identity::decode_gpu_identity(soc.hw_family, soc.hw_variant, soc.num_dies as u8)
    else {
        dev_err!(
            pdev.as_ref(),
            "AGX3: static identity for chip {:#x} does not decode\n",
            soc.chip_id
        );
        return EINVAL;
    };

    if soc.uat_ias != 42 {
        dev_err!(
            pdev.as_ref(),
            "AGX3: static UAT width for chip {:#x} disagrees with the identity decode\n",
            soc.chip_id
        );
        return EINVAL;
    }

    dev_info!(
        pdev.as_ref(),
        "AGX3: matched {:?}{:?} (chip {:#x}, USC gen {}, HAL {:?}, {}-bit UAT roots, {}-bit OAS)\n",
        expected.gpu_gen,
        expected.gpu_variant,
        soc.chip_id,
        expected.usc_generation,
        expected.gpu_hal_generation,
        soc.uat_ias,
        soc.uat_oas
    );

    match regs::Resources::new(pdev) {
        Ok(res) => match res.get_gpu_id() {
            Ok(id) => {
                dev_info!(
                    pdev.as_ref(),
                    "AGX3: hardware reports {:?}{:?} rev {:?}, {} dies, {} clusters, {} cores/cluster, {} active cores\n",
                    id.gpu_gen,
                    id.gpu_variant,
                    id.gpu_rev,
                    id.num_dies,
                    id.num_clusters,
                    id.num_cores,
                    id.total_active_cores
                );
                if id.gpu_gen != soc.gpu_gen || id.gpu_variant != soc.gpu_variant {
                    dev_warn!(
                        pdev.as_ref(),
                        "AGX3: hardware identity {:?}{:?} does not match the matched compatible ({:?}{:?})\n",
                        id.gpu_gen,
                        id.gpu_variant,
                        soc.gpu_gen,
                        soc.gpu_variant
                    );
                }
            }
            Err(e) => dev_warn!(pdev.as_ref(), "AGX3: GPU ID decode failed: {:?}\n", e),
        },
        Err(e) => {
            dev_warn!(
                pdev.as_ref(),
                "AGX3: SGX MMIO region unavailable: {:?}\n",
                e
            );
        }
    }

    match expected.gpu_gen {
        hw::GpuGen::G15 => refuse_g15_probe(pdev),
        gen => {
            dev_err!(
                pdev.as_ref(),
                "AGX3 setup unavailable for {:?}: firmware tuning, initdata, and handoff are incomplete; no DMA or processor state changed\n",
                gen
            );
            ENODEV
        }
    }
}

/// Refuse a G15 SoC that no generation binds (the two-die T6032): nothing is known about its
/// firmware handoff, so the probe stops before any DMA, MMIO, UAT or processor state changes.
fn refuse_g15_probe(pdev: &platform::Device<Core>) -> Error {
    dev_err!(
        pdev.as_ref(),
        "G15 setup unavailable for this SoC: firmware tuning, initdata and handoff are not known; no DMA, MMIO, UAT or processor state changed\n"
    );
    ENODEV
}

/// Platform Driver implementation for `AsahiDriver`.
impl platform::Driver for AsahiDriver {
    type IdInfo = ProbeConfig;
    const OF_ID_TABLE: Option<of::IdTable<Self::IdInfo>> = Some(&OF_TABLE);

    fn unbind(_pdev: &platform::Device<Core>, this: Pin<&Self>) {

        match &this.runtime {
            AsahiRuntime::M3(runtime) => runtime.stop(),
            AsahiRuntime::G16(runtime) => runtime.stop(),
            // Driver data is dropped after devres, so CPU control must happen in unbind.
            AsahiRuntime::G17(_, gpu) => gpu.shutdown(),
            _ => {},
        }
    }

    /// Device probe function.
    fn probe(
        pdev: &platform::Device<Core>,
        info: Option<&Self::IdInfo>,
    ) -> impl PinInit<Self, Error> {
        debug::update_debug_flags();

        dev_info!(pdev.as_ref(), "Probing...\n");

        let cfg = match info.ok_or(ENODEV)? {
            ProbeConfig::Supported(cfg) => *cfg,
            ProbeConfig::Agx3Diagnostic(soc) if soc.chip_id == 0x8122 => {
                if crate::g16_profile::selected(pdev).inspect_err(|error| {
                    dev_err!(pdev.as_ref(), "G16G: firmware/profile admission failed: {:?}\n", error);
                })? {
                    if crate::m3_params::t8122_params().start != 1 { return Err(ENODEV); }
                    crate::g16_board::initialize(pdev).inspect_err(|error| {
                        dev_err!(pdev.as_ref(), "G16G: per-Mac calibration admission failed: {:?}\n", error);
                    })?;
                    let runtime = crate::g16_drm::Registered::start(pdev)?;
                    return Ok(Self { runtime: AsahiRuntime::G16(runtime) });
                }
                let runtime = start_t8122(pdev)?;
                return Ok(Self { runtime: AsahiRuntime::M3(runtime) });
            }
            ProbeConfig::Agx3Diagnostic(soc) => return Err(refuse_agx3_probe(pdev, soc)),
            ProbeConfig::G15(cfg) => return Self::probe_g15(pdev, cfg),
            ProbeConfig::G17(cfg) => return Self::probe_g17(pdev, cfg),
        };

        unsafe { pdev.dma_set_mask_and_coherent(DmaMask::try_new(cfg.uat_oas)?)? };

        let res = regs::Resources::new(pdev)?;

        // Initialize misc MMIO
        res.init_mmio()?;

        // Start the coprocessor CPU, so UAT can initialize the handoff
        regs::Resources::start_cpu(pdev)?;

        let fwnode = pdev.as_ref().fwnode().ok_or(EIO)?;
        let compat: KVec<u32> = fwnode
            .property_read_array_vec(c_str!("apple,firmware-compat"), 3)?
            .required_by(pdev.as_ref())?;

        let drm: ARef<AsahiDevice> = drm::device::Device::new(
            pdev.as_ref(), AsahiData::new(pdev, Some(res), false))?;
        let res = drm.resources.as_ref().ok_or(ENODEV)?;

        let legacy_gpu = match (cfg.gpu_gen, cfg.gpu_variant, compat.as_slice()) {
            (hw::GpuGen::G13, _, &[12, 3, 0]) => {
                gpu::GpuManagerG13V12_3::new(&drm.clone(), &res, cfg)? as Arc<dyn gpu::GpuManager>
            }
            (hw::GpuGen::G14, hw::GpuVariant::G, &[12, 4, 0]) => {
                gpu::GpuManagerG14V12_4::new(&drm.clone(), &res, cfg)? as Arc<dyn gpu::GpuManager>
            }
            (hw::GpuGen::G13, _, &[13, 5, 0]) => {
                gpu::GpuManagerG13V13_5::new(&drm.clone(), &res, cfg)? as Arc<dyn gpu::GpuManager>
            }
            (hw::GpuGen::G14, hw::GpuVariant::G, &[13, 5, 0]) => {
                gpu::GpuManagerG14V13_5::new(&drm.clone(), &res, cfg)? as Arc<dyn gpu::GpuManager>
            }
            (hw::GpuGen::G14, _, &[13, 5, 0]) => {
                gpu::GpuManagerG14XV13_5::new(&drm.clone(), &res, cfg)? as Arc<dyn gpu::GpuManager>
            }
            _ => {
                dev_info!(
                    pdev.as_ref(),
                    "Unsupported GPU/firmware combination ({:?}, {:?}, {:?})\n",
                    cfg.gpu_gen,
                    cfg.gpu_variant,
                    compat
                );
                return Err(ENODEV);
            }
        };
        let gpu = drm_gpu::Backend::Legacy(drm_gpu::LegacyDrmGpu::new(legacy_gpu));

        if !drm.gpu.populate(gpu) { return Err(EBUSY); }

        (*drm).gpu()?.init()?;

        drm::driver::Registration::new_foreign_owned(&drm, pdev.as_ref(), 0)?;

        Ok(Self {
            runtime: AsahiRuntime::Legacy(drm),
        })
    }
}

impl AsahiDriver {
    fn probe_g17(pdev: &platform::Device<Core>, cfg: &'static g17::Config) -> Result<Self> {
        // SAFETY: The GPU performs DMA through a UAT whose output width is part of the SoC config.
        unsafe { pdev.dma_set_mask_and_coherent(DmaMask::try_new(cfg.uat_oas)?)? };
        let res = regs::Resources::new(pdev)?;
        // No userspace or GPU backend is published before the runtime is constructed.
        let drm: ARef<AsahiDevice> =
            drm::device::Device::new(pdev.as_ref(), AsahiData::new_g17(pdev, res.clone()))?;
        let gpu = g17::Gpu::new(pdev, &drm, cfg, res)?;
        if !drm.gpu.populate(drm_gpu::Backend::G17(gpu.clone())) {
            gpu.shutdown();
            return Err(EBUSY);
        }
        if let Err(error) = drm::driver::Registration::new_foreign_owned(&drm, pdev.as_ref(), 0) {
            gpu.shutdown();
            return Err(error);
        }
        Ok(Self {
            runtime: AsahiRuntime::G17(drm, gpu),
        })
    }
}

impl AsahiDriver {
    /// Boots a G15 GPU and registers its DRM device.
    ///
    /// A G15 firmware session cannot be stopped once started. If the boot or the registration
    /// fails after the firmware has been touched ([`g15::retained`]), the device stays bound with
    /// no DRM device and [`g15::Gpu`] retains everything the firmware can reach until reboot.
    fn probe_g15(pdev: &platform::Device<Core>, cfg: &'static g15::Config) -> Result<Self> {
        dev_info!(pdev.as_ref(), "G15: probing\n");
        // SAFETY: The GPU performs DMA through a UAT whose output width is part of the SoC config.
        unsafe { pdev.dma_set_mask_and_coherent(DmaMask::try_new(cfg.uat_oas)?)? };
        let res = regs::Resources::new(pdev)?;
        // No userspace or GPU backend is published before the generation is constructed.
        let drm: ARef<AsahiDevice> = drm::device::Device::new(
            pdev.as_ref(),
            AsahiData::new(pdev, Some(res.clone()), false),
        )?;
        let retained = |error: Error| -> Result<Self> {
            dev_err!(
                pdev.as_ref(),
                "G15: probe failed after the firmware start ({:?}); device kept bound, firmware retained until reboot\n",
                error
            );
            Ok(Self {
                runtime: AsahiRuntime::G15Retained,
            })
        };
        let gpu = match g15::Gpu::new(pdev, &drm, cfg, res) {
            Ok(gpu) => gpu,
            Err(error) if g15::retained() => return retained(error),
            Err(error) => return Err(error),
        };
        if !drm.gpu.populate(drm_gpu::Backend::G15(gpu.clone())) {
            return retained(EBUSY);
        }
        if let Err(error) = drm::driver::Registration::new_foreign_owned(&drm, pdev.as_ref(), 0) {
            return retained(error);
        }
        Ok(Self {
            runtime: AsahiRuntime::G15(drm, gpu),
        })
    }
}

impl Drop for AsahiDriver {
    fn drop(&mut self) {
        if let AsahiRuntime::G17(_, gpu) = &self.runtime {
            // Also covers failure to allocate platform driver data after the probe body returned.
            gpu.shutdown();
        }
    }
}
