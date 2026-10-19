// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Apple AOP ambient light sensor driver
//!
//! Copyright (C) The Asahi Linux Contributors

use kernel::{
    bindings, c_str,
    device::Core,
    firmware::Firmware,
    iio::common::aop_sensors::{AopSensorData, IIORegistration, MessageProcessor, MICRO},
    module_platform_driver, of, platform,
    prelude::*,
    soc::apple::aop::{EPICService, FakehidRegistration, AOP},
    sync::Arc,
};

const EPIC_SUBTYPE_GET_AOP_PROPERTY: u16 = 0xa;
const EPIC_SUBTYPE_SET_ALS_PROPERTY: u16 = 0x4;

/* ALS properties. */
const ALS_PROP_INTERVAL: u32 = 0x00;
const ALS_PROP_CALIBRATION: u32 = 0x0b;
const ALS_PROP_BATCH_INTERVAL: u32 = 0x16;
const ALS_PROP_RAW_INTERVAL: u32 = 0x5c;
const ALS_PROP_VERBOSITY: u32 = 0xe1;

/*
 * The CT817 starts reporting on the zero to non-zero transition of its
 * report interval, and it accepts that transition only once its batch and
 * raw interval properties have been written. Despite their names these two
 * are not intervals; they are the values the sensor is configured with on
 * the T8140. The sensor answers every one of these writes with a
 * not-permitted status and applies them regardless, so the status is only
 * logged.
 */
const ALS_BATCH_INTERVAL: u32 = 1;
const ALS_RAW_INTERVAL: u32 = 0;
const ALS_VERBOSITY: u32 = 6;
/// The reporting interval in microseconds.
const ALS_INTERVAL_US: u32 = 200000;
const LUX_OFFSET_CT720: usize = 0x1d;
const LUX_OFFSET_VD6286: usize = 0x28;
/*
 * The CT817 sends a 32-byte report: two raw channel floats at +0x08 and
 * +0x0c and the lux float at +0x1c, one byte below the CT720's, so the two
 * parts do not share a layout.
 */
const LUX_OFFSET_CT817: usize = 0x1c;

fn get_lux_offset(aop: &dyn AOP, dev: &platform::Device, svc: &EPICService) -> Result<usize> {
    let name = get_aop_property(aop, svc, 0xf, 16)?.1;
    match name.as_slice() {
        b"Redbird\0" => Ok(LUX_OFFSET_VD6286),
        b"FireFish2\0" => Ok(LUX_OFFSET_CT720),
        // The T8140 AOP reports the part number rather than a code name.
        b"CT817\0" => Ok(LUX_OFFSET_CT817),
        _ => {
            dev_warn!(
                dev.as_ref(),
                "Unknown sensor type {:?}",
                core::str::from_utf8(&name)
            );
            Err(EIO)
        }
    }
}

fn enable_als(
    aop: &dyn AOP,
    dev: &platform::Device,
    svc: &EPICService,
    offset: usize,
) -> Result<()> {
    if offset != LUX_OFFSET_CT817 {
        let fw = Firmware::request(c_str!("apple/aop-als-cal.bin"), dev.as_ref())?;
        set_als_property(aop, svc, ALS_PROP_CALIBRATION, fw.data())?;
        set_als_property(aop, svc, ALS_PROP_INTERVAL, &ALS_INTERVAL_US.to_le_bytes())?;
        return Ok(());
    }

    // The CT817 gets its calibration from the AOP driver over the setup port,
    // not as an ALS property. Only the transport can fail here; the sensor's
    // status words are logged, see the property comments above. Do not fetch
    // the HID report descriptor (subtype 0x01) on this endpoint: the AOP never
    // answers it, and do not write the MODE (0xd7) or 0xe4 properties: either
    // wedges the AOP for the rest of the boot.
    let set = |tag: u32, val: u32, what: &str| -> Result<()> {
        let status = set_als_property(aop, svc, tag, &val.to_le_bytes())?;
        dev_dbg!(
            dev.as_ref(),
            "{} = {:#x}: status {:#x}\n",
            what,
            val,
            status
        );
        Ok(())
    };
    set(ALS_PROP_VERBOSITY, ALS_VERBOSITY, "verbosity")?;
    set(
        ALS_PROP_BATCH_INTERVAL,
        ALS_BATCH_INTERVAL,
        "batch interval",
    )?;
    set(ALS_PROP_RAW_INTERVAL, ALS_RAW_INTERVAL, "raw interval")?;
    // The interval is zero at probe, so this is the transition that arms the
    // sensor.
    set(ALS_PROP_INTERVAL, ALS_INTERVAL_US, "interval")
}

fn get_aop_property(
    aop: &dyn AOP,
    svc: &EPICService,
    tag: u32,
    data_len: usize,
) -> Result<(u32, KVec<u8>)> {
    let mut buf = KVec::new();
    buf.resize(8, 0, GFP_KERNEL)?;
    buf[4..8].copy_from_slice(&tag.to_le_bytes());
    aop.epic_call_ret(svc, EPIC_SUBTYPE_GET_AOP_PROPERTY, &buf, data_len)
}

fn set_als_property(aop: &dyn AOP, svc: &EPICService, tag: u32, data: &[u8]) -> Result<u32> {
    let mut buf = KVec::new();
    buf.resize(data.len() + 8, 0, GFP_KERNEL)?;
    buf[8..].copy_from_slice(data);
    buf[4..8].copy_from_slice(&tag.to_le_bytes());
    aop.epic_call(svc, EPIC_SUBTYPE_SET_ALS_PROPERTY, &buf)
}

/// Converts the bits of an IEEE 754 single to microlux, rounding down.
/// Negative values clamp to zero; NaN and the infinities carry no reading.
/// Values beyond `MAX_MICRO` saturate when they are stored.
fn f32_to_micro(f: u32) -> Option<u64> {
    let exp = ((f >> 23) & 0xff) as i32;
    if exp == 0xff {
        return None;
    }
    if f & 0x8000_0000 != 0 || exp == 0 {
        return Some(0);
    }
    // The value is mant * 2^shift; scaled to microlux it is below 2^44
    // before the shift, so a left shift of up to 19 cannot overflow.
    let mant = u64::from(f & 0x7f_ffff | 0x80_0000) * MICRO;
    let shift = exp - 127 - 23;
    Some(if shift >= 20 {
        u64::MAX
    } else if shift >= 0 {
        mant << shift
    } else if shift <= -64 {
        0
    } else {
        mant >> -shift
    })
}

struct MsgProc(usize);

impl MessageProcessor for MsgProc {
    fn process(&self, message: &[u8]) -> Option<u64> {
        let raw = message.get(self.0..self.0 + 4)?;
        f32_to_micro(u32::from_le_bytes([raw[0], raw[1], raw[2], raw[3]]))
    }
}

struct IIOAopAlsDriver {
    /// Declared first: the subscription ends before the IIO device goes.
    listener: FakehidRegistration,
    _iio: IIORegistration<MsgProc>,
    aop: Arc<dyn AOP>,
    service: EPICService,
    j613_quiesce: bool,
}

impl IIOAopAlsDriver {
    fn set_report_interval(&self, interval: u32) -> Result {
        let status = set_als_property(
            self.aop.as_ref(),
            &self.service,
            ALS_PROP_INTERVAL,
            &interval.to_le_bytes(),
        )?;
        if status != 0 {
            return Err(EIO);
        }
        Ok(())
    }
}

kernel::of_device_table!(
    OF_TABLE,
    MODULE_OF_TABLE,
    (),
    [(of::DeviceId::new(c_str!("apple,aop-als")), ())]
);

impl platform::Driver for IIOAopAlsDriver {
    type IdInfo = ();

    const OF_ID_TABLE: Option<of::IdTable<Self::IdInfo>> = Some(&OF_TABLE);
    const HAS_PM: bool = true;

    fn suspend(dev: &kernel::device::Device<Core>, this: Pin<&Self>) -> Result {
        if this.j613_quiesce {
            if let Err(error) = this.set_report_interval(0) {
                // A failed suspend callback is not guaranteed a resume call.
                // Try to restore reporting before returning the failure.
                if let Err(restore_error) = this.set_report_interval(ALS_INTERVAL_US) {
                    dev_err!(dev, "ALS reporting rollback failed: {:?}\n", restore_error);
                }
                return Err(error);
            }
            dev_info!(dev, "J613 ALS reports paused for suspend\n");
        }
        Ok(())
    }

    fn resume(dev: &kernel::device::Device<Core>, this: Pin<&Self>) -> Result {
        if this.j613_quiesce {
            this.set_report_interval(ALS_INTERVAL_US)?;
            dev_info!(dev, "J613 ALS reports restored after resume\n");
        }
        Ok(())
    }

    fn probe(pdev: &platform::Device<Core>, _info: Option<&()>) -> impl PinInit<Self, Error> {
        let dev = pdev.as_ref();
        // SAFETY: The boot device-tree root is either NULL or a live node;
        // of_device_is_compatible only reads its compatibility property.
        let j613_quiesce = *module_parameters::j613_suspend_quiesce.value() == 1
            && unsafe {
                !bindings::of_root.is_null()
                    && bindings::of_device_is_compatible(
                        bindings::of_root,
                        c_str!("apple,j613").as_char_ptr(),
                    ) != 0
            };
        if j613_quiesce {
            dev_info!(dev, "J613 ALS suspend quiescing enabled\n");
        }
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
        let ty = bindings::BINDINGS_IIO_LIGHT;
        let offset = get_lux_offset(adata.as_ref(), pdev, &service)?;
        let data = AopSensorData::new(dev.into(), ty, MsgProc(offset))?;
        let listener = FakehidRegistration::new(adata.clone(), service, data.clone())?;
        enable_als(adata.as_ref(), pdev, &service, offset)?;
        let info_mask = 1 << bindings::BINDINGS_IIO_CHAN_INFO_PROCESSED;
        let iio =
            IIORegistration::<MsgProc>::new(data, c"aop-sensors-als", ty, info_mask, &THIS_MODULE)?;
        Ok(IIOAopAlsDriver {
            listener,
            _iio: iio,
            aop: adata,
            service,
            j613_quiesce,
        })
    }

    fn unbind(_dev: &platform::Device<Core>, this: Pin<&Self>) {
        this.listener.unregister();
    }
}

module_platform_driver! {
    type: IIOAopAlsDriver,
    name: "iio_aop_als",
    description: "AOP ambient light sensor driver",
    license: "Dual MIT/GPL",
    alias: ["platform:iio_aop_als"],
    firmware: ["apple/aop-als-cal.bin"],
    params: {
        j613_suspend_quiesce: u32 {
            default: 0,
            description: "J613 experiment: stop ALS reports during ordinary suspend and restore on resume",
        },
    },
}
