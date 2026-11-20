// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Apple AOP lid angle sensor driver
//!
//! Copyright (C) The Asahi Linux Contributors

use core::{
    ffi::{c_int, c_void},
    ptr,
};

use kernel::{
    bindings, c_str,
    device::{self, Core},
    iio::common::aop_sensors::{AopSensorData, IIORegistration, MessageProcessor, MICRO},
    module_platform_driver, of, platform,
    prelude::*,
    soc::apple::aop::{EPICService, FakehidRegistration, AOP},
    sync::{
        aref::ARef,
        atomic::{Atomic, Relaxed},
        Arc,
    },
    types::Opaque,
};

const SUBTYPE_SET_PROPERTY: u16 = 4;
const SUBTYPE_GET_PROPERTY: u16 = 10;
/// The report interval, microseconds; 0 stops the reports.
const PROP_INTERVAL: u32 = 0;

/// Writes one u32 property; returns the service's status word.
fn set_u32(aop: &dyn AOP, svc: &EPICService, prop: u32, value: u32) -> Result<u32> {
    let mut msg = KVec::with_capacity(12, GFP_KERNEL)?;
    msg.extend_from_slice(&0u32.to_le_bytes(), GFP_KERNEL)?;
    msg.extend_from_slice(&prop.to_le_bytes(), GFP_KERNEL)?;
    msg.extend_from_slice(&value.to_le_bytes(), GFP_KERNEL)?;
    aop.epic_call(svc, SUBTYPE_SET_PROPERTY, &msg)
}

/// Reads one u32 property; `None` when the service refuses or answers short.
fn get_u32(aop: &dyn AOP, svc: &EPICService, prop: u32) -> Result<Option<u32>> {
    let mut msg = KVec::new();
    msg.resize(8, 0, GFP_KERNEL)?;
    msg[4..8].copy_from_slice(&prop.to_le_bytes());
    let (status, value) = aop.epic_call_ret(svc, SUBTYPE_GET_PROPERTY, &msg, 4)?;
    if status != 0 || value.len() < 4 {
        return Ok(None);
    }
    Ok(Some(u32::from_le_bytes([value[0], value[1], value[2], value[3]])))
}

/// Stops the lid angle reports around system sleep and restores them after.
///
/// The firmware streams the angle at its own interval whether anyone reads
/// it or not, and the AOP mailbox interrupt stays enabled in
/// suspend-to-idle: every report ends the CPUs' idle period. On the J815 a
/// 4 h lid-closed s2idle left the freeze loop 14478 times, once every
/// 0.999 s, with as many AOP mailbox interrupts, after the light sensor
/// already stopped its reports for sleep. Lid open and close come from the
/// SMC, not from this sensor, so a sleeping system still wakes on the lid.
///
/// The interval the firmware runs at is read when the system sleeps and
/// written back when it wakes; a service that does not answer the read is
/// left alone. Only suspend is handled, as for the light sensor.
#[repr(C)]
struct PmHook {
    /// First, so that the callback finds the hook from it.
    nb: Opaque<bindings::notifier_block>,
    dev: ARef<device::Device>,
    aop: Arc<dyn AOP>,
    service: EPICService,
    /// The interval to restore after the sleep; 0 when nothing was stopped.
    saved: Atomic<u32>,
    registered: Atomic<bool>,
}

/// The PM notifier callback.
///
/// # Safety
///
/// `nb` is the notifier block of a registered [`PmHook`].
unsafe extern "C" fn pm_event(
    nb: *mut bindings::notifier_block,
    action: usize,
    _data: *mut c_void,
) -> c_int {
    // SAFETY: `nb` is the first field of a registered `PmHook`.
    let hook = unsafe { &*nb.cast::<PmHook>() };
    let aop = hook.aop.as_ref();
    match action as u32 {
        bindings::PM_SUSPEND_PREPARE => match get_u32(aop, &hook.service, PROP_INTERVAL) {
            Ok(Some(0)) => {}
            Ok(Some(interval)) => match set_u32(aop, &hook.service, PROP_INTERVAL, 0) {
                Ok(status) => {
                    hook.saved.store(interval, Relaxed);
                    dev_info!(
                        hook.dev,
                        "reports off for sleep (interval {} us): status {:#x}\n",
                        interval,
                        status
                    );
                }
                Err(e) => dev_warn!(hook.dev, "reports not stopped for sleep: {:?}\n", e),
            },
            Ok(None) => dev_info!(hook.dev, "interval not readable: reports left on for sleep\n"),
            Err(e) => dev_warn!(hook.dev, "interval read failed: {:?}\n", e),
        },
        bindings::PM_POST_SUSPEND => {
            let interval = hook.saved.xchg(0, Relaxed);
            if interval != 0 {
                match set_u32(aop, &hook.service, PROP_INTERVAL, interval) {
                    Ok(status) => dev_info!(
                        hook.dev,
                        "reports back after sleep (interval {} us): status {:#x}\n",
                        interval,
                        status
                    ),
                    Err(e) => dev_warn!(hook.dev, "reports not restarted after sleep: {:?}\n", e),
                }
            }
        }
        _ => return bindings::NOTIFY_DONE as c_int,
    }
    bindings::NOTIFY_OK as c_int
}

/// # Safety
///
/// `nb` stays valid and pinned until [`pm_unregister`] is called on it.
#[cfg(CONFIG_PM_SLEEP)]
unsafe fn pm_register(nb: *mut bindings::notifier_block) -> c_int {
    // SAFETY: By the caller's contract.
    unsafe { bindings::register_pm_notifier(nb) }
}

/// # Safety
///
/// `nb` was registered by [`pm_register`].
#[cfg(CONFIG_PM_SLEEP)]
unsafe fn pm_unregister(nb: *mut bindings::notifier_block) {
    // SAFETY: By the caller's contract.
    unsafe { bindings::unregister_pm_notifier(nb) };
}

/// Without system sleep there is nothing to switch.
///
/// # Safety
///
/// None; the signature matches the one with system sleep.
#[cfg(not(CONFIG_PM_SLEEP))]
unsafe fn pm_register(_nb: *mut bindings::notifier_block) -> c_int {
    0
}

/// # Safety
///
/// None; the signature matches the one with system sleep.
#[cfg(not(CONFIG_PM_SLEEP))]
unsafe fn pm_unregister(_nb: *mut bindings::notifier_block) {}

// SAFETY: The notifier block is only touched by the PM core, which
// serializes registration, removal and the callbacks itself.
unsafe impl Send for PmHook {}
// SAFETY: See above; the other fields are `Sync`.
unsafe impl Sync for PmHook {}

impl PmHook {
    fn new(
        dev: &device::Device,
        aop: Arc<dyn AOP>,
        service: EPICService,
    ) -> Result<Pin<KBox<Self>>> {
        let hook = KBox::pin(
            Self {
                nb: Opaque::new(bindings::notifier_block {
                    notifier_call: Some(pm_event),
                    next: ptr::null_mut(),
                    priority: 0,
                }),
                dev: dev.into(),
                aop,
                service,
                saved: Atomic::new(0),
                registered: Atomic::new(false),
            },
            GFP_KERNEL,
        )?;
        // SAFETY: The block is pinned inside the hook, which unregisters it
        // before it goes.
        let ret = unsafe { pm_register(hook.nb.get()) };
        if ret < 0 {
            return Err(Error::from_errno(ret));
        }
        hook.registered.store(true, Relaxed);
        Ok(hook)
    }

    /// Returns once no callback is running any more; repeated calls do
    /// nothing.
    fn unregister(&self) {
        if self.registered.xchg(false, Relaxed) {
            // SAFETY: `nb` was registered by `new()`.
            unsafe { pm_unregister(self.nb.get()) };
        }
    }
}

impl Drop for PmHook {
    fn drop(&mut self) {
        self.unregister();
    }
}

struct MsgProc;

impl MessageProcessor for MsgProc {
    fn process(&self, message: &[u8]) -> Option<u64> {
        // Report 1 contains a little-endian, 9-bit angle in degrees.
        // Other report IDs carry sensor properties, not angle samples.
        if *message.first()? != 1 {
            return None;
        }
        let angle = u16::from_le_bytes([*message.get(1)?, *message.get(2)?]) & 0x1ff;
        if angle > 360 {
            return None;
        }
        Some(u64::from(angle) * MICRO)
    }

    fn scale(&self) -> Option<(i32, i32)> {
        // IIO_ANGL uses radians: one raw count is pi / 180 radians.
        Some((0, 17_453_293))
    }
}

struct IIOAopLasDriver {
    /// Fields are declared in teardown order: sleep handling first, then the
    /// subscription, which ends before the IIO device goes.
    pm: Pin<KBox<PmHook>>,
    listener: FakehidRegistration,
    _iio: IIORegistration<MsgProc>,
}

kernel::of_device_table!(
    OF_TABLE,
    (),
    [(of::DeviceId::new(c_str!("apple,aop-las")), ())]
);

impl platform::Driver for IIOAopLasDriver {
    type IdInfo = ();
    type Data<'bound> = IIOAopLasDriver;

    const OF_ID_TABLE: Option<of::IdTable<Self::IdInfo>> = Some(&OF_TABLE);

    fn probe<'bound>(
        pdev: &'bound platform::Device<Core<'_>>,
        _info: Option<&'bound ()>,
    ) -> impl PinInit<Self::Data<'bound>, Error> + 'bound {
        let dev = pdev.as_ref();
        // SAFETY: This driver binds only to the service devices the AOP core
        // registers, and `pdev` is being probed.
        let adata = unsafe { <dyn AOP>::from_child(dev) }?;
        // SAFETY: `dev` is a valid device; the AOP core attached its service
        // record as the platform data of every service device it registers.
        let service = unsafe { (*dev.as_raw()).platform_data.cast::<EPICService>() };
        if service.is_null() {
            return Err(ENODEV);
        }
        // SAFETY: The record lives as long as the device it is attached to.
        let service = unsafe { *service };

        let ty = bindings::BINDINGS_IIO_ANGL;
        let data = AopSensorData::new(dev.into(), ty, MsgProc)?;
        let listener = FakehidRegistration::new(adata.clone(), service, data.clone())?;
        let info_mask = (1 << bindings::BINDINGS_IIO_CHAN_INFO_RAW)
            | (1 << bindings::BINDINGS_IIO_CHAN_INFO_SCALE);
        let iio = IIORegistration::<MsgProc>::new(
            data,
            c"aop-sensors-las",
            ty,
            info_mask,
            kernel::module::this_module::<crate::LocalModule>(),
        )?;
        let pm = PmHook::new(dev, adata, service)?;
        Ok(IIOAopLasDriver {
            pm,
            listener,
            _iio: iio,
        })
    }

    fn unbind(_dev: &platform::Device<Core<'_>>, this: Pin<&Self>) {
        this.pm.unregister();
        this.listener.unregister();
    }
}

module_platform_driver! {
    type: IIOAopLasDriver,
    name: "iio_aop_las",
    description: "AOP lid angle sensor driver",
    license: "Dual MIT/GPL",
    alias: ["platform:iio_aop_las"],
}
