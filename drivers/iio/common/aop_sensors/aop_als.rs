// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Apple AOP ambient light sensor driver
//!
//! Copyright (C) The Asahi Linux Contributors

use kernel::{
    bindings, c_str,
    device::Core,
    firmware::Firmware,
    iio::common::aop_sensors::{AopSensorData, IIORegistration, MessageProcessor, MICRO, MAX_CHANS},
    module_platform_driver, of, platform,
    prelude::*,
    soc::apple::aop::{EPICService, FakehidRegistration, AOP},
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

/// Slots in a CT720 decode. Slot 0 is the lux channel the driver already published.
const CH_LUX: usize = 0;
const CH_RED: usize = 1;
const CH_GREEN: usize = 2;
const CH_BLUE: usize = 3;
const CH_CLEAR: usize = 4;
const CH_CCT: usize = 5;
const CT720_CHANS: usize = 6;

// FireFish2 / CT720 report, 55 bytes, lux float at +0x1d. The four counts are
// red, green, blue, clear. Clear is the wideband count (largest in room light;
// the calibration gains are 8, 8, 8, 4). Under the ~5460 K light captured on
// this J313 the first three ranked green, red, blue. Byte +0x2a is non-zero
// when the color temperature at +0x2b is valid. The integration time is stored
// twice at +0x2f (the AOP logs 197380 µs when it arms this sensor); disagreeing
// copies mean this is not that layout, and color is left unpublished.
const CT720_LEN: usize = 55;
const CT720_COUNT_OFF: [usize; 4] = [0x0d, 0x11, 0x15, 0x19];
const CT720_CCT_FLAG: usize = 0x2a;
const CT720_CCT_OFF: usize = 0x2b;
const CT720_INTEG_OFF: usize = 0x2f;
const CT720_INTEG_MAX: u32 = 10_000_000;
/// Clear counts at or below 11 on a covered J313 still had the firmware CCT
/// byte set, and the published temperatures were 788 K, 1288 K, 2904 K and
/// 6781 K. Publish CCT only when clear is at least this. Not hardware-tested.
const CT720_CCT_MIN_CLEAR: u32 = 12;

const _: () = assert!(CT720_CHANS <= MAX_CHANS);

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

fn u32_at(message: &[u8], off: usize) -> Option<u32> {
    let b = message.get(off..off + 4)?;
    Some(u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
}

/// Color slots of a CT720 report. Lux is filled by [`MsgProc::process`].
fn ct720_color(message: &[u8], out: &mut [u64]) -> u32 {
    if message.len() < CT720_LEN {
        return 0;
    }
    let Some(integ) = u32_at(message, CT720_INTEG_OFF) else {
        return 0;
    };
    let Some(again) = u32_at(message, CT720_INTEG_OFF + 4) else {
        return 0;
    };
    if integ == 0 || integ != again || integ > CT720_INTEG_MAX {
        return 0;
    }
    let mut mask = 0u32;
    let mut clear = 0u32;
    let mut slot = CH_RED;
    for off in CT720_COUNT_OFF {
        if let Some(count) = u32_at(message, off) {
            if slot == CH_CLEAR {
                clear = count;
            }
            if let Some(dest) = out.get_mut(slot) {
                *dest = u64::from(count).saturating_mul(MICRO);
                mask |= 1u32 << slot;
            }
        }
        slot += 1;
    }
    if message.get(CT720_CCT_FLAG).copied() == Some(0) || clear < CT720_CCT_MIN_CLEAR {
        return mask;
    }
    if let Some(bits) = u32_at(message, CT720_CCT_OFF) {
        if bits & 0x8000_0000 == 0 {
            if let Some(cct) = f32_to_micro(bits) {
                if cct > 0 {
                    if let Some(dest) = out.get_mut(CH_CCT) {
                        *dest = cct;
                        mask |= 1u32 << CH_CCT;
                    }
                }
            }
        }
    }
    mask
}

fn chan_spec(
    ty: u32,
    channel2: i32,
    modified: u32,
    info_mask: usize,
    index: usize,
) -> bindings::iio_chan_spec {
    let mut spec = bindings::iio_chan_spec {
        type_: ty,
        channel2,
        address: index,
        // Polled sysfs channels. A negative scan index keeps them out of a buffer.
        scan_index: -1,
        __bindgen_anon_1: bindings::iio_chan_spec__bindgen_ty_1 {
            scan_type: bindings::iio_scan_type {
                __bindgen_anon_1: bindings::iio_scan_type__bindgen_ty_1 {
                    format: b'u' as _,
                },
                realbits: 32,
                storagebits: 32,
                ..Default::default()
            },
        },
        info_mask_separate: info_mask,
        ..Default::default()
    };
    spec.set_modified(modified);
    spec
}

fn ct720_channels() -> [bindings::iio_chan_spec; CT720_CHANS] {
    let raw: usize = 1 << bindings::BINDINGS_IIO_CHAN_INFO_RAW;
    let processed: usize = 1 << bindings::BINDINGS_IIO_CHAN_INFO_PROCESSED;
    [
        chan_spec(
            bindings::iio_chan_type_IIO_LIGHT,
            0,
            0,
            processed,
            CH_LUX,
        ),
        chan_spec(
            bindings::iio_chan_type_IIO_INTENSITY,
            bindings::iio_modifier_IIO_MOD_LIGHT_RED as i32,
            1,
            raw,
            CH_RED,
        ),
        chan_spec(
            bindings::iio_chan_type_IIO_INTENSITY,
            bindings::iio_modifier_IIO_MOD_LIGHT_GREEN as i32,
            1,
            raw,
            CH_GREEN,
        ),
        chan_spec(
            bindings::iio_chan_type_IIO_INTENSITY,
            bindings::iio_modifier_IIO_MOD_LIGHT_BLUE as i32,
            1,
            raw,
            CH_BLUE,
        ),
        chan_spec(
            bindings::iio_chan_type_IIO_INTENSITY,
            bindings::iio_modifier_IIO_MOD_LIGHT_CLEAR as i32,
            1,
            raw,
            CH_CLEAR,
        ),
        chan_spec(
            bindings::iio_chan_type_IIO_COLORTEMP,
            0,
            0,
            processed,
            CH_CCT,
        ),
    ]
}

struct MsgProc(usize);

impl MessageProcessor for MsgProc {
    fn process(&self, message: &[u8]) -> Option<u64> {
        let raw = message.get(self.0..self.0 + 4)?;
        f32_to_micro(u32::from_le_bytes([raw[0], raw[1], raw[2], raw[3]]))
    }

    fn decode(&self, message: &[u8], out: &mut [u64]) -> u32 {
        let mut mask = 0u32;
        if let Some(lux) = self.process(message) {
            if let Some(dest) = out.get_mut(CH_LUX) {
                *dest = lux;
                mask |= 1u32 << CH_LUX;
            }
        }
        if self.0 == LUX_OFFSET_CT720 {
            mask |= ct720_color(message, out);
        }
        mask
    }
}

struct IIOAopAlsDriver {
    /// Declared first: the subscription ends before the IIO device goes.
    listener: FakehidRegistration,
    _iio: IIORegistration<MsgProc>,
}

kernel::of_device_table!(
    OF_TABLE,
    MODULE_OF_TABLE,
    (),
    [(of::DeviceId::new(c_str!("apple,aop-als")), ())]
);

impl platform::Driver for IIOAopAlsDriver {
    type IdInfo = ();
    type Data<'bound> = IIOAopAlsDriver;

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
        let ty = bindings::BINDINGS_IIO_LIGHT;
        let offset = get_lux_offset(adata.as_ref(), pdev, &service)?;
        let data = AopSensorData::new(dev.into(), ty, MsgProc(offset))?;
        let listener = FakehidRegistration::new(adata.clone(), service, data.clone())?;
        enable_als(adata.as_ref(), pdev, &service, offset)?;
        let info_mask = 1 << bindings::BINDINGS_IIO_CHAN_INFO_PROCESSED;
        let iio = if offset == LUX_OFFSET_CT720 {
            let channels = ct720_channels();
            IIORegistration::<MsgProc>::with_channels(
                data,
                c"aop-sensors-als",
                &channels,
                &THIS_MODULE,
            )?
        } else {
            IIORegistration::<MsgProc>::new(
                data,
                c"aop-sensors-als",
                ty,
                info_mask,
                &THIS_MODULE,
            )?
        };
        Ok(IIOAopAlsDriver {
            listener,
            _iio: iio,
        })
    }

    fn unbind(_dev: &platform::Device<Core<'_>>, this: Pin<&Self>) {
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
}
