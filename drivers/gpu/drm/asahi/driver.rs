// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Top-level GPU driver implementation.

use crate::module_parameters;
use kernel::bindings;
use kernel::{
    device::{
        Core,
        DeviceContext, //
    },
    dma::{
        Device,
        DmaMask, //
    },
    drm,
    drm::ioctl,
    of,
    platform,
    prelude::*,
    sync::{
        aref::ARef,
        Arc, //
    }, //
};

use crate::{
    debug,
    file,
    g17,
    gem::AsahiObject,
    gpu,
    hw,
    regs, //
};

use kernel::macros::vtable;

pub(crate) struct AsahiDrmDriver;

/// Convenience type alias for the DRM device type for this driver.
pub(crate) type AsahiDrmDevice<Ctx = drm::Normal> = drm::Device<AsahiDrmDriver, Ctx>;

pub(crate) struct AsahiPlatformDriver;

// #[pin_data(PinnedDrop)]
pub(crate) struct AsahiPlatformDriverData<'bound> {
    binding: crate::binding::Owner,
    _device: ARef<AsahiDrmDevice>,
    _reg: drm::Registration<'bound, AsahiDrmDriver>,
    /// The G17 GPU, whose coprocessors must be stopped when the driver is unbound.
    g17: Option<Arc<g17::Gpu>>,
    #[cfg(CONFIG_DRM_ASAHI_G16)]
    pub(crate) g16: Option<Arc<crate::g16::Gpu>>,
    m3: Option<M3Backend>,
    /// Raw `struct device *` of the platform device. Stored once at probe
    /// time so the `Drop` impl can find the sysfs file on unregister.
    raw_dev: *mut bindings::device,
    raw_stats: *mut core::ffi::c_void,
    // Own the allocation until unregister has drained all sysfs readers.
    stats: Option<Arc<crate::stats::StatsSnapshot>>,
}

// SAFETY: `raw_dev` is only passed back to the sysfs shim, which serializes its own state.
unsafe impl Send for AsahiPlatformDriverData<'_> {}
// SAFETY: As above.
unsafe impl Sync for AsahiPlatformDriverData<'_> {}

#[cfg(CONFIG_DRM_ASAHI_M3_T8122)]
type M3Backend = Arc<crate::m3::m3_drm::Backend>;
#[cfg(not(CONFIG_DRM_ASAHI_M3_T8122))]
type M3Backend = core::convert::Infallible;

/// Holds a reference to the top-level GPU object.
#[pin_data]
pub(crate) struct AsahiDrmRegistrationData<'drm> {
    #[pin]
    pub(crate) gpu: Arc<dyn gpu::Gpu>,
    pub(crate) resources: regs::Resources<'drm>,
}

/// Convenience type alias for the DRM device type for this driver.
pub(crate) type AsahiDevice = ARef<AsahiDrmDevice>;
pub(crate) type AsahiDevRef = ARef<AsahiDrmDevice>;
pub(crate) type AsahiRegData<'drm> = AsahiDrmRegistrationData<'drm>;

/// DRM Driver metadata
///
/// The major version names the UAPI generation in include/uapi/drm/asahi_drm.h
/// and only changes if that ABI ever breaks. The minor version goes up with
/// each new ioctl, flag or feature bit, so userspace can tell what a kernel
/// accepts without probing. The patch level marks fixes userspace may need to
/// detect.
const INFO: drm::driver::DriverInfo = drm::driver::DriverInfo {
    major: 1,
    minor: 0,
    patchlevel: 0,
    name: c"aurora",
    desc: c"Aurora GPU for Apple AGX",
};

/// DRM Driver implementation for `AsahiDriver`.
#[vtable]
impl drm::driver::Driver for AsahiDrmDriver {
    type Data = ();
    type RegistrationData<'drm> = AsahiDrmRegistrationData<'drm>;
    /// Our `File` type.
    type File = file::File;
    /// Our `Object` type.
    type Object = drm::gem::shmem::Object<AsahiObject>;
    type ParentDevice<Ctx: DeviceContext> = platform::Device<Ctx>;

    const INFO: drm::driver::DriverInfo = INFO;
    const FEAT_RENDER: bool = true;
    const FEAT_SYNCOBJ: bool = true;
    const FEAT_SYNCOBJ_TIMELINE: bool = true;

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
        (ASAHI_GEM_USERPTR,     drm_asahi_gem_userptr,
            ioctl::AUTH | ioctl::RENDER_ALLOW, crate::file::File::gem_userptr),
        (ASAHI_GEM_RESIDENCY,   drm_asahi_gem_residency,
            ioctl::AUTH | ioctl::RENDER_ALLOW, crate::file::File::gem_residency),
    }
}

/// Firmware interface selected by the hardware compatible.
pub(crate) enum ProbeConfig {
    Legacy(&'static hw::HwConfig),
    G17(&'static g17::Config),
    /// M5 Pro, Max and Ultra: the ID registers select the configuration.
    G17M5,
    M3(crate::m3::Chip),
    /// The M5 (T8142) backend, refused unless it is built.
    M5,
}

// OF Device ID table. Linux 7.3's of_device_table!() takes no attributes on
// its entries, so the entries that the native G15 backend owns when it is
// built are passed in by configuration.
macro_rules! asahi_of_device_table {
    ($($g15_owned:tt)*) => {
        kernel::of_device_table!(
            OF_TABLE,
            <AsahiPlatformDriver as platform::Driver>::IdInfo,
            [
                (
                    of::DeviceId::new(c"apple,agx-t8140"),
                    ProbeConfig::G17(&hw::t8140::CONFIG)
                ),
                (
                    of::DeviceId::new(c"apple,agx-t6040"),
                    ProbeConfig::G17(&hw::t6040::CONFIG)
                ),
                (
                    of::DeviceId::new(c"apple,agx-t6041"),
                    ProbeConfig::G17(&hw::t6041::CONFIG)
                ),
                (
                    of::DeviceId::new(c"apple,agx-t8142"),
                    ProbeConfig::M5
                ),
                (
                    of::DeviceId::new(c"gpu,t6050"),
                    ProbeConfig::G17M5
                ),
                (
                    of::DeviceId::new(c"apple,agx-g17s"),
                    ProbeConfig::G17M5
                ),
                (
                    of::DeviceId::new(c"apple,agx-t8103"),
                    ProbeConfig::Legacy(&hw::t8103::HWCONFIG)
                ),
                (
                    of::DeviceId::new(c"apple,agx-t8112"),
                    ProbeConfig::Legacy(&hw::t8112::HWCONFIG)
                ),
                (
                    of::DeviceId::new(c"apple,agx-t6000"),
                    ProbeConfig::Legacy(&hw::t600x::HWCONFIG_T6000)
                ),
                (
                    of::DeviceId::new(c"apple,agx-t6001"),
                    ProbeConfig::Legacy(&hw::t600x::HWCONFIG_T6001)
                ),
                (
                    of::DeviceId::new(c"apple,agx-t6002"),
                    ProbeConfig::Legacy(&hw::t600x::HWCONFIG_T6002)
                ),
                (
                    of::DeviceId::new(c"apple,agx-t6020"),
                    ProbeConfig::Legacy(&hw::t602x::HWCONFIG_T6020)
                ),
                (
                    of::DeviceId::new(c"apple,agx-t6021"),
                    ProbeConfig::Legacy(&hw::t602x::HWCONFIG_T6021)
                ),
                (
                    of::DeviceId::new(c"apple,agx-t6022"),
                    ProbeConfig::Legacy(&hw::t602x::HWCONFIG_T6022)
                ),
                (
                    of::DeviceId::new(c"apple,agx-t8122"),
                    ProbeConfig::M3(crate::m3::Chip::T8122)
                ),
                $($g15_owned)*
                (
                    of::DeviceId::new(c"apple,agx-t6032"),
                    ProbeConfig::M3(crate::m3::Chip::T6032)
                ),
            ]
        );
    };
}

#[cfg(not(CONFIG_DRM_ASAHI_G15))]
asahi_of_device_table!(
    (
        of::DeviceId::new(c"apple,agx-t6030"),
        ProbeConfig::M3(crate::m3::Chip::T6030)
    ),
    (
        of::DeviceId::new(c"apple,agx-t6031"),
        ProbeConfig::M3(crate::m3::Chip::T6031)
    ),
);
#[cfg(CONFIG_DRM_ASAHI_G15)]
asahi_of_device_table!();

pub(crate) struct SleepOps(pub(crate) bindings::dev_pm_ops);

// SAFETY: Immutable: the callbacks are plain functions.
unsafe impl Sync for SleepOps {}

/// # Safety
///
/// `dev` must be a device this driver is bound to, as for a PM core callback.
unsafe fn sleep_data<'a>(dev: *mut bindings::device) -> &'a AsahiPlatformDriverData<'a> {
    // SAFETY: Probe stored a `Pin<KBox<AsahiPlatformDriverData>>` here, and the PM core keeps the
    // device bound for the duration of a callback.
    unsafe { &*bindings::dev_get_drvdata(dev).cast() }
}

/// Runs before any device suspends, so the GPU goes first and work still in flight can finish.
unsafe extern "C" fn sleep_prepare(dev: *mut bindings::device) -> core::ffi::c_int {
    // SAFETY: Reads a global flag word.
    if !unsafe { bindings::pm_suspend_via_firmware() } {
        return 0;
    }
    // SAFETY: Called by the PM core.
    let data = unsafe { sleep_data(dev) };
    match data.g17.as_ref() {
        Some(gpu) => kernel::error::from_result(|| gpu.sleep_prepare().map(|_| 0)),
        None => EBUSY.to_errno(),
    }
}

/// Runs after every device resumed. Does nothing unless [`sleep_prepare`] stopped the firmware.
unsafe extern "C" fn sleep_complete(dev: *mut bindings::device) {
    // SAFETY: Called by the PM core.
    if let Some(gpu) = unsafe { sleep_data(dev) }.g17.as_ref() {
        gpu.sleep_complete();
    }
}

static SLEEP_OPS: SleepOps = SleepOps(bindings::dev_pm_ops {
    prepare: Some(sleep_prepare),
    complete: Some(sleep_complete),
    // SAFETY: All-zero is the absence of every other callback.
    ..unsafe { core::mem::zeroed() }
});

/// For a driver with no firmware restart: a sleep that powers the SoC down would lose its firmware.
unsafe extern "C" fn sleep_refuse(_dev: *mut bindings::device) -> core::ffi::c_int {
    // SAFETY: Reads a global flag word.
    if unsafe { bindings::pm_suspend_via_firmware() } {
        EBUSY.to_errno()
    } else {
        0
    }
}

static SLEEP_REFUSE_OPS: SleepOps = SleepOps(bindings::dev_pm_ops {
    prepare: Some(sleep_refuse),
    // SAFETY: All-zero is the absence of every other callback.
    ..unsafe { core::mem::zeroed() }
});

/// The callbacks of a driver that refuses a sleep that powers the SoC down.
pub(crate) fn sleep_refuse_ops() -> Option<&'static bindings::dev_pm_ops> {
    Some(&SLEEP_REFUSE_OPS.0)
}

/// Platform Driver implementation for `AsahiDriver`.
impl platform::Driver for AsahiPlatformDriver {
    type IdInfo = ProbeConfig;
    const OF_ID_TABLE: Option<of::IdTable<Self::IdInfo>> = Some(&OF_TABLE);
    type Data<'bound> = AsahiPlatformDriverData<'bound>;

    fn pm_ops() -> Option<&'static bindings::dev_pm_ops> {
        Some(&SLEEP_OPS.0)
    }

    fn unbind<'bound>(_pdev: &'bound platform::Device<Core<'_>>, this: Pin<&Self::Data<'bound>>) {
        // SAFETY: Takes no arguments, process context. Clears the T6001 AGX request in the PMP report if the pmp_report.agx_vote
        // parameter set it; a no-op everywhere else.
        unsafe { bindings::apple_pmp_agx_activity_release() };
        if let Some(gpu) = this.g17.as_ref() {
            // Driver data is dropped after devres, so CPU control must happen in unbind.
            gpu.shutdown();
        }
        #[cfg(CONFIG_DRM_ASAHI_M3_T8122)]
        if let Some(m3) = this.m3.as_ref() {
            m3.stop();
        }
        this.close_binding();
    }

    /// Device probe function.
    fn probe<'bound>(
        pdev: &'bound platform::Device<Core<'_>>,
        info: Option<&'bound Self::IdInfo>,
    ) -> impl PinInit<Self::Data<'bound>, Error> + 'bound {
        debug::update_debug_flags();
        crate::memacct::LEDGER.configure();

        // SAFETY: Takes no arguments, process context. Lets the T6001 AGX vote run again after an earlier unbind released it; a no-op everywhere else.
        unsafe { bindings::apple_pmp_agx_activity_resume() };

        dev_info!(pdev.as_ref(), "Probing...\n");

        let cfg = match info.ok_or(ENODEV)? {
            ProbeConfig::Legacy(cfg) => *cfg,
            ProbeConfig::G17(cfg) => {
                return AsahiPlatformDriver::probe_g17(pdev, regs::G17Resources::new(pdev)?, cfg)
            }
            ProbeConfig::G17M5 => return AsahiPlatformDriver::probe_g17_m5(pdev),
            #[cfg(CONFIG_DRM_ASAHI_M3_T8122)]
            ProbeConfig::M3(crate::m3::Chip::T8122) => return AsahiPlatformDriver::probe_t8122(pdev),
            ProbeConfig::M3(chip) => return Err(crate::m3::refuse_probe(pdev, *chip)),
            #[cfg(CONFIG_DRM_ASAHI_M5)]
            ProbeConfig::M5 => return AsahiPlatformDriver::probe_m5(pdev),
            #[cfg(not(CONFIG_DRM_ASAHI_M5))]
            ProbeConfig::M5 => return Err(ENODEV),
        };

        // SAFETY: The selected SoC's UAT output width bounds all device DMA.
        unsafe { pdev.dma_set_mask_and_coherent(DmaMask::try_new(cfg.uat_oas)?)? };

        let resources = regs::Resources::new(pdev)?;

        // Initialize misc MMIO
        resources.init_mmio()?;

        // Start the coprocessor CPU, so UAT can initialize the handoff
        regs::Resources::start_cpu(pdev)?;

        let fwnode = pdev.as_ref().fwnode().ok_or(EIO)?;
        let compat: KVec<u32> = fwnode
            .property_read_array_vec(c"apple,firmware-compat", 3)?
            .required_by(pdev.as_ref())?;

        let drm = drm::UnregisteredDevice::<AsahiDrmDriver>::new(pdev, Ok(()))?;
        // SAFETY: Every adapter calls close_binding in unbind before devres;
        // probe errors drop this local owner while the bound callback is live.
        let binding = unsafe { crate::binding::Owner::new(pdev.as_ref(), &drm)? };

        let gpu = match (cfg.gpu_gen, cfg.gpu_variant, compat.as_slice()) {
            (hw::GpuGen::G13, _, &[12, 3, 0]) => {
                gpu::GpuManagerG13V12_3::new(&drm, &resources, cfg)? as Arc<dyn gpu::Gpu>
            }
            (hw::GpuGen::G14, hw::GpuVariant::G, &[12, 4, 0]) => {
                gpu::GpuManagerG14V12_4::new(&drm, &resources, cfg)? as Arc<dyn gpu::Gpu>
            }
            (hw::GpuGen::G13, _, &[13, 5, 0]) => {
                gpu::GpuManagerG13V13_5::new(&drm, &resources, cfg)? as Arc<dyn gpu::Gpu>
            }
            (hw::GpuGen::G14, hw::GpuVariant::G, &[13, 5, 0]) => {
                gpu::GpuManagerG14V13_5::new(&drm, &resources, cfg)? as Arc<dyn gpu::Gpu>
            }
            (hw::GpuGen::G14, _, &[13, 5, 0]) => {
                gpu::GpuManagerG14XV13_5::new(&drm, &resources, cfg)? as Arc<dyn gpu::Gpu>
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
        gpu.init()?;

        let stats = gpu.stats_snapshot();
        let debug_stats = gpu.raw_stats_snapshot();
        let data = try_pin_init!(AsahiDrmRegistrationData { gpu, resources });

        // SAFETY: `reg` is stored in `AsahiPlatformDriverData` and dropped when the driver is
        // unbound; it is never forgotten.
        let reg = unsafe { drm::Registration::new(pdev.as_ref(), drm, data, 0)? };

        // The DRM core created this card's debugfs directory when the device was registered;
        // publish the raw firmware statistics in it. drm_dev_unregister() removes the file, and
        // waits for its readers, before the registration data that owns the snapshot is dropped.
        if let Some(snapshot) = debug_stats.as_ref() {
            crate::stats_raw::register(reg.device(), snapshot);
        }

        // Register the sysfs file on the platform device. Must happen after
        // the DRM device is registered so the device is fully bound.
        let raw_dev = pdev.as_ref().as_raw();
        let snapshot = stats.as_ref().map_or(core::ptr::null(), Arc::as_ptr);
        let raw_stats = crate::sysfs_exports::register(
            raw_dev,
            snapshot,
            stats.is_some() && module_parameters::stats_export.value() != 0,
        )?;

        let driver = AsahiPlatformDriverData {
            binding,
            _device: reg.device().into(),
            _reg: reg,
            g17: None,
            m3: None,
            #[cfg(CONFIG_DRM_ASAHI_G16)]
            g16: None,
            raw_dev,
            raw_stats,
            stats,
        };

        Ok(driver)
    }
}

impl AsahiPlatformDriver {
    #[cfg(CONFIG_DRM_ASAHI_G15)]
    pub(crate) fn probe_g15<'bound>(
        pdev: &'bound platform::Device<Core<'_>>,
        cfg: &'static crate::g15::Config,
    ) -> Result<AsahiPlatformDriverData<'bound>> {
        // SAFETY: Admission checks the SoC and the UAT output width before DMA starts.
        unsafe { pdev.dma_set_mask_and_coherent(DmaMask::try_new(cfg.uat_oas)?)? };
        let resources = regs::Resources::new(pdev)?;
        let drm = drm::UnregisteredDevice::<AsahiDrmDriver>::new(pdev, Ok(()))?;
        // SAFETY: Every adapter calls close_binding in unbind before devres;
        // probe errors drop this local owner while the bound callback is live.
        let binding = unsafe { crate::binding::Owner::new(pdev.as_ref(), &drm)? };
        let gpu = crate::g15::Gpu::new(pdev, &drm, cfg)?;
        let data = try_pin_init!(AsahiDrmRegistrationData {
            gpu: gpu as Arc<dyn gpu::Gpu>,
            resources,
        });
        // SAFETY: The registration is stored and unregistered by platform data destruction.
        // G15 retains its firmware-visible state independently until reboot, even on error.
        let reg = unsafe { drm::Registration::new(pdev.as_ref(), drm, data, 0)? };
        Ok(AsahiPlatformDriverData {
            binding,
            _device: reg.device().into(),
            _reg: reg,
            g17: None,
            m3: None,
            #[cfg(CONFIG_DRM_ASAHI_G16)]
            g16: None,
            raw_dev: core::ptr::null_mut(),
            raw_stats: core::ptr::null_mut(),
            stats: None,
        })
    }

    /// The M3 MacBook Air (T8122): the M3 runtime, started only when its identity gate, the
    /// backend selection and the start experiment (`aurora.t8122_start=1`) admit it; otherwise
    /// the probe is refused as without the runtime.
    #[cfg(CONFIG_DRM_ASAHI_M3_T8122)]
    fn probe_t8122<'bound>(
        pdev: &'bound platform::Device<Core<'_>>,
    ) -> Result<AsahiPlatformDriverData<'bound>> {
        let soc = crate::m3::t8122_admitted(pdev)?;
        // SAFETY: The admitted M3 GPU's UAT outputs 42-bit physical addresses.
        unsafe { pdev.dma_set_mask_and_coherent(DmaMask::try_new(42)?)? };
        let resources = regs::Resources::new(pdev)?;
        let drm = drm::UnregisteredDevice::<AsahiDrmDriver>::new(pdev, Ok(()))?;
        let binding = unsafe { crate::binding::Owner::new(pdev.as_ref(), &drm)? };
        let backend = crate::m3::m3_drm::Backend::start(pdev, &drm, soc)?;
        let gpu = backend.clone() as Arc<dyn gpu::Gpu>;
        let data = try_pin_init!(AsahiDrmRegistrationData { gpu, resources });
        // SAFETY: The registration is stored and unregistered by platform data destruction,
        // which drops the runtime with it.
        let reg = match unsafe { drm::Registration::new(pdev.as_ref(), drm, data, 0) } {
            Ok(reg) => reg,
            Err(error) => {
                backend.stop();
                return Err(error);
            }
        };
        dev_info!(pdev.as_ref(), "M3: firmware ready; common DRM GEM/VM frontend registered\n");
        Ok(AsahiPlatformDriverData {
            binding,
            #[cfg(CONFIG_DRM_ASAHI_G16)]
            g16: None,
            _device: reg.device().into(),
            _reg: reg,
            g17: None,
            m3: Some(backend),
            raw_dev: core::ptr::null_mut(),
            raw_stats: core::ptr::null_mut(),
            stats: None,
        })
    }

    fn probe_g17_m5<'bound>(
        pdev: &'bound platform::Device<Core<'_>>,
    ) -> Result<AsahiPlatformDriverData<'bound>> {
        // Every T6050 board binds gpu,t6050. Let the ID registers choose the configuration.
        let res = regs::G17Resources::new(pdev)?;
        let id = res.get_g17_id()?;
        let cfg = hw::t6050::config(&id).ok_or_else(|| {
            dev_err!(pdev.as_ref(), "No M5 configuration for GPU variant {:#x} with {} dies\n",
                id.variant, id.num_dies);
            ENODEV
        })?;
        Self::probe_g17(pdev, res, cfg)
    }

    fn probe_g17<'bound>(
        pdev: &'bound platform::Device<Core<'_>>,
        res: regs::G17Resources,
        cfg: &'static g17::Config,
    ) -> Result<AsahiPlatformDriverData<'bound>> {
        // SAFETY: The GPU performs DMA through a UAT whose output width is part of the SoC config.
        unsafe { pdev.dma_set_mask_and_coherent(DmaMask::try_new(cfg.uat_oas)?)? };
        let resources = regs::Resources::new(pdev)?;
        let drm = drm::UnregisteredDevice::<AsahiDrmDriver>::new(pdev, Ok(()))?;
        // SAFETY: Every adapter calls close_binding in unbind before devres;
        // probe errors drop this local owner while the bound callback is live.
        let binding = unsafe { crate::binding::Owner::new(pdev.as_ref(), &drm)? };
        let gpu = g17::Gpu::new(pdev, &drm, cfg, res)?;
        let data_gpu = gpu.clone() as Arc<dyn gpu::Gpu>;
        let data = try_pin_init!(AsahiDrmRegistrationData {
            gpu: data_gpu,
            resources,
        });
        // SAFETY: `reg` is stored in `AsahiPlatformDriverData` and dropped when the driver is
        // unbound; it is never forgotten.
        let reg = match unsafe { drm::Registration::new(pdev.as_ref(), drm, data, 0) } {
            Ok(reg) => reg,
            Err(error) => {
                gpu.shutdown();
                return Err(error);
            }
        };
        let stats = gpu::Gpu::stats_snapshot(&*gpu);
        let raw_dev = pdev.as_ref().as_raw();
        let snapshot = stats.as_ref().map_or(core::ptr::null(), Arc::as_ptr);
        let raw_stats = match crate::sysfs_exports::register_g17(
            raw_dev,
            snapshot,
            module_parameters::stats_export.value() != 0,
        ) {
            Ok(handle) => handle,
            Err(error) => {
                gpu.shutdown();
                return Err(error);
            }
        };
        if let Err(error) =
            crate::sysfs_exports::register_idle(raw_dev, crate::g17::power::IDLE_HOLD_MS.as_ptr())
        {
            dev_warn!(
                pdev.as_ref(),
                "idle_off_delay_ms not registered: {:?}\n",
                error
            );
        }
        Ok(AsahiPlatformDriverData {
            binding,
            _device: reg.device().into(),
            _reg: reg,
            g17: Some(gpu),
            m3: None,
            #[cfg(CONFIG_DRM_ASAHI_G16)]
            g16: None,
            raw_dev,
            raw_stats,
            stats,
        })
    }
}

#[cfg(CONFIG_DRM_ASAHI_M5)]
impl AsahiPlatformDriver {
    fn probe_m5<'bound>(
        pdev: &'bound platform::Device<Core<'_>>,
    ) -> Result<AsahiPlatformDriverData<'bound>> {
        // Without aura's 25G76 profile (the macOS 27.0 stub's 26A428 firmware), take the M5
        // graph's G17 path (J813 bring-up).
        let has_25g76 = of::chosen().is_some_and(|chosen| {
            matches!(
                chosen.get_opt_property::<KVec<u8>>(c"aurora,m5-gpu-firmware-profile"),
                Ok(Some(_))
            )
        });
        if !has_25g76 {
            return Self::probe_g17(pdev, regs::G17Resources::new(pdev)?, &hw::t8142::CONFIG);
        }
        // SAFETY: The M5 UAT has 42 output bits.
        unsafe { pdev.dma_set_mask_and_coherent(DmaMask::try_new(42)?)? };
        let resources = regs::Resources::new(pdev)?;
        let res = regs::G17Resources::new(pdev)?;
        let drm = drm::UnregisteredDevice::<AsahiDrmDriver>::new(pdev, Ok(()))?;
        // SAFETY: Every adapter calls close_binding in unbind before devres;
        // probe errors drop this local owner while the bound callback is live.
        let binding = unsafe { crate::binding::Owner::new(pdev.as_ref(), &drm)? };
        let gpu = crate::m5::probe(pdev, &drm, res)?;
        let data = try_pin_init!(AsahiDrmRegistrationData {
            gpu: gpu as Arc<dyn gpu::Gpu>,
            resources,
        });
        // SAFETY: The registration is stored and unregistered by platform data destruction.
        // The M5 firmware session retains its firmware-visible state until reboot.
        let reg = unsafe { drm::Registration::new(pdev.as_ref(), drm, data, 0)? };
        Ok(AsahiPlatformDriverData {
            binding,
            _device: reg.device().into(),
            _reg: reg,
            g17: None,
            m3: None,
            #[cfg(CONFIG_DRM_ASAHI_G16)]
            g16: None,
            raw_dev: core::ptr::null_mut(),
            raw_stats: core::ptr::null_mut(),
            stats: None,
        })
    }
}

#[cfg(CONFIG_DRM_ASAHI_G16)]
impl AsahiPlatformDriver {
    pub(crate) fn probe_g16<'bound>(
        pdev: &'bound platform::Device<Core<'_>>,
        cfg: &'static crate::g16::Config,
    ) -> Result<AsahiPlatformDriverData<'bound>> {
        // SAFETY: the qualified UAT output width constrains this device's DMA.
        unsafe { pdev.dma_set_mask_and_coherent(DmaMask::try_new(cfg.uat_oas)?)? };
        let resources = regs::Resources::new(pdev)?;
        let res = regs::G16Resources::new(pdev)?;
        let drm = drm::UnregisteredDevice::<AsahiDrmDriver>::new(pdev, Ok(()))?;
        // SAFETY: Every adapter calls close_binding in unbind before devres;
        // probe errors drop this local owner while the bound callback is live.
        let binding = unsafe { crate::binding::Owner::new(pdev.as_ref(), &drm)? };
        let gpu = crate::g16::Gpu::new(pdev, &drm, cfg, res)?;
        let data_gpu = gpu.clone() as Arc<dyn gpu::Gpu>;
        let data = try_pin_init!(AsahiDrmRegistrationData {
            gpu: data_gpu,
            resources,
        });
        // SAFETY: registration is stored in bound platform data and never forgotten.
        let reg = unsafe { drm::Registration::new(pdev.as_ref(), drm, data, 0)? };
        let raw_dev = pdev.as_ref().as_raw();
        let raw_stats = crate::sysfs_exports::register(raw_dev, core::ptr::null(), false)?;
        Ok(AsahiPlatformDriverData {
            binding,
            _device: reg.device().into(),
            _reg: reg,
            g17: None,
            m3: None,
            g16: Some(gpu),
            raw_dev,
            raw_stats,
            stats: None,
        })
    }
}

#[cfg(CONFIG_DRM_ASAHI_G18)]
impl AsahiPlatformDriver {
    /// Probe the M6 GPU through the G18 backend.
    ///
    /// The parameter gate refuses before any MMIO, DMA, mailbox or processor access. A
    /// diagnostic bring-up stage retains its firmware owner without a DRM device; the
    /// application stage registers the render node over the started session.
    pub(crate) fn probe_g18<'bound>(
        pdev: &'bound platform::Device<Core<'_>>,
        soc: &'static hw::t8152::SocConfig,
    ) -> Result<crate::g18::probe::G18Bound<'bound>> {
        crate::g18::probe::admit_t8152(pdev)?;
        let resources = regs::Resources::new(pdev)?;
        let drm = drm::UnregisteredDevice::<AsahiDrmDriver>::new(pdev, Ok(()))?;
        // SAFETY: The G18 adapter closes the binding in unbind before devres; probe errors
        // drop this local owner while the bound callback is live.
        let binding = unsafe { crate::binding::Owner::new(pdev.as_ref(), &drm)? };
        let started = crate::g18::probe::probe_t8152(pdev, &drm, soc)?;
        let Some(gpu) = started.gpu else {
            return Ok(crate::g18::probe::G18Bound {
                runtime: started.runtime,
                binding: Some(binding),
                driver: None,
            });
        };
        let data = try_pin_init!(AsahiDrmRegistrationData { gpu, resources });
        // SAFETY: `reg` is stored in `AsahiPlatformDriverData` and dropped when the driver is
        // unbound; it is never forgotten.
        let reg = match unsafe { drm::Registration::new(pdev.as_ref(), drm, data, 0) } {
            Ok(reg) => reg,
            Err(error) => {
                started.runtime.shutdown();
                return Err(error);
            }
        };
        dev_info!(
            pdev.as_ref(),
            "G18_DRM registered render node (aurora, apple,agx-t8152)\n"
        );
        Ok(crate::g18::probe::G18Bound {
            runtime: started.runtime,
            binding: None,
            driver: Some(AsahiPlatformDriverData {
                binding,
                _device: reg.device().into(),
                _reg: reg,
                g17: None,
                m3: None,
                #[cfg(CONFIG_DRM_ASAHI_G16)]
                g16: None,
                raw_dev: core::ptr::null_mut(),
                raw_stats: core::ptr::null_mut(),
                stats: None,
            }),
        })
    }
}

impl AsahiPlatformDriverData<'_> {
    pub(crate) fn close_binding(&self) {
        self.binding.close();
    }
}

impl Drop for AsahiPlatformDriverData<'_> {
    fn drop(&mut self) {
        self.binding.close();
        // `self.raw_dev` was set in `probe` from the platform
        // device's `as_raw()`, valid for the lifetime of the bound driver.
        if !self.raw_dev.is_null() {
            crate::sysfs_exports::unregister(self.raw_dev, self.raw_stats);
        }
        drop(self.stats.take());
        if let Some(gpu) = self.g17.as_ref() {
            // Also covers failure to allocate platform driver data after the probe body returned.
            gpu.shutdown();
        }
        #[cfg(CONFIG_DRM_ASAHI_M3_T8122)]
        if let Some(m3) = self.m3.as_ref() {
            m3.stop();
        }
    }
}
