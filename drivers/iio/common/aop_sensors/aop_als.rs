// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Apple AOP ambient light sensor driver
//!
//! Copyright (C) The Asahi Linux Contributors

use kernel::{
    bindings, c_str,
    device::Core,
    firmware::Firmware,
    iio::common::aop_sensors::{
        AopSensorData, ChannelSpec, IIORegistration, MessageProcessor, MICRO,
    },
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
// These offsets address the fake-HID EPIC payload, not a host HID event.
const LUX_OFFSET_VD6286: usize = 0x28;
// The documented 122-byte Redbird report carries firmware-computed Kelvin.
// Other report layouts must be decoded separately, even for the same sensor.
const VD6286_COLOR_REPORT_LEN: usize = 122;
const CCT_OFFSET_VD6286: usize = 0x45;
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
        // The installer collects the calibration blob from macOS, so it is
        // not present on every system. Without it the VD6286 of the M4 Pro
        // never reports and its channel reads ENODATA. The warning below
        // reports the missing file once, so the firmware loader is asked not
        // to report it too.
        match Firmware::request_nowarn(c_str!("apple/aop-als-cal.bin"), dev.as_ref()) {
            Ok(fw) => {
                set_als_property(aop, svc, ALS_PROP_CALIBRATION, fw.data())?;
            }
            Err(e) => dev_warn!(
                dev.as_ref(),
                "no apple/aop-als-cal.bin ({:?}); the sensor may not report\n",
                e
            ),
        }
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

// FireFish2 / CT720 report, 55 bytes, lux float at +0x1d. The four counts are
// red, green, blue, clear. Clear is the wideband count (largest in room light;
// the calibration gains are 8, 8, 8, 4). Under the ~5460 K light captured on
// a J313 the first three ranked green, red, blue. Byte +0x2a is non-zero
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

/// Channels of a CT720. Channel 0 is the lux channel every part has.
const CT720_RED: usize = 1;
const CT720_CLEAR: usize = 4;
const CT720_CCT: usize = 5;
const CT720_CHANS: usize = 6;

fn u32_at(message: &[u8], off: usize) -> Option<u32> {
    let b = message.get(off..off + 4)?;
    Some(u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
}

/// Whether a CT720 report has the layout the color fields are read from.
fn ct720_layout(message: &[u8]) -> bool {
    if message.len() < CT720_LEN {
        return false;
    }
    match (
        u32_at(message, CT720_INTEG_OFF),
        u32_at(message, CT720_INTEG_OFF + 4),
    ) {
        (Some(integ), Some(again)) => integ != 0 && integ == again && integ <= CT720_INTEG_MAX,
        _ => false,
    }
}

/// A color channel of a CT720 report: a count in whole units, or the color temperature.
fn ct720_color(message: &[u8], channel: usize) -> Option<u64> {
    if !ct720_layout(message) {
        return None;
    }
    if (CT720_RED..=CT720_CLEAR).contains(&channel) {
        let count = u32_at(message, CT720_COUNT_OFF[channel - CT720_RED])?;
        return Some(u64::from(count).saturating_mul(MICRO));
    }
    if channel != CT720_CCT || message.get(CT720_CCT_FLAG).copied() == Some(0) {
        return None;
    }
    if u32_at(message, CT720_COUNT_OFF[CT720_CLEAR - CT720_RED])? < CT720_CCT_MIN_CLEAR {
        return None;
    }
    let bits = u32_at(message, CT720_CCT_OFF)?;
    if bits & 0x8000_0000 != 0 {
        return None;
    }
    // Zero does not describe a colour temperature.
    f32_to_micro(bits).filter(|value| *value != 0)
}

struct MsgProc(usize);

fn report_float(message: &[u8], offset: usize) -> Option<u64> {
    let raw = message.get(offset..offset + 4)?;
    f32_to_micro(u32::from_le_bytes([raw[0], raw[1], raw[2], raw[3]]))
}

impl MessageProcessor for MsgProc {
    fn process(&self, message: &[u8]) -> Option<u64> {
        report_float(message, self.0)
    }

    fn process_channel(&self, message: &[u8], channel: usize) -> Option<u64> {
        match channel {
            0 => self.process(message),
            1 if self.0 == LUX_OFFSET_VD6286 && message.len() == VD6286_COLOR_REPORT_LEN => {
                // Zero and negative CCT do not describe a colour temperature.
                report_float(message, CCT_OFFSET_VD6286).filter(|value| *value != 0)
            }
            _ if self.0 == LUX_OFFSET_CT720 => ct720_color(message, channel),
            _ => None,
        }
    }

    /// A CT720 report that carries lux but not a color channel (another layout, no valid
    /// color temperature, too little light for one) withdraws that channel's last reading:
    /// it reads `ENODATA` rather than a stale color. A report with no lux reading leaves
    /// every channel as it was.
    fn channel_withdrawn(&self, message: &[u8], channel: usize) -> bool {
        self.0 == LUX_OFFSET_CT720 && channel != 0 && self.process(message).is_some()
    }
}

enum AlsIio {
    Light {
        _registration: IIORegistration<MsgProc>,
    },
    Color {
        _registration: IIORegistration<MsgProc, 2>,
    },
    Ct720 {
        _registration: IIORegistration<MsgProc, CT720_CHANS>,
    },
}

/// Lux and color temperature are processed values; the four counts are raw.
fn ct720_channels() -> [ChannelSpec; CT720_CHANS] {
    let raw = 1 << bindings::BINDINGS_IIO_CHAN_INFO_RAW;
    let processed = 1 << bindings::BINDINGS_IIO_CHAN_INFO_PROCESSED;
    let count = |modifier| ChannelSpec {
        ty: bindings::BINDINGS_IIO_INTENSITY,
        modifier: Some(modifier),
        info_mask: raw,
    };
    [
        ChannelSpec {
            ty: bindings::BINDINGS_IIO_LIGHT,
            modifier: None,
            info_mask: processed,
        },
        count(bindings::BINDINGS_IIO_MOD_LIGHT_RED),
        count(bindings::BINDINGS_IIO_MOD_LIGHT_GREEN),
        count(bindings::BINDINGS_IIO_MOD_LIGHT_BLUE),
        count(bindings::BINDINGS_IIO_MOD_LIGHT_CLEAR),
        ChannelSpec {
            ty: bindings::BINDINGS_IIO_COLORTEMP,
            modifier: None,
            info_mask: processed,
        },
    ]
}

fn register_ct720(
    pdev: &platform::Device,
    aop: &Arc<dyn AOP>,
    service: EPICService,
    offset: usize,
) -> Result<(FakehidRegistration, IIORegistration<MsgProc, CT720_CHANS>)> {
    let channels = ct720_channels();
    let data = AopSensorData::new_channels(
        pdev.as_ref().into(),
        channels.map(|channel| channel.ty),
        MsgProc(offset),
    )?;
    let listener = FakehidRegistration::new(aop.clone(), service, data.clone())?;
    enable_als(aop.as_ref(), pdev, &service, offset)?;
    let iio = IIORegistration::new_specs(
        data,
        c"aop-sensors-als",
        channels,
        kernel::module::this_module::<crate::LocalModule>(),
    )?;
    Ok((listener, iio))
}

struct IIOAopAlsDriver {
    /// Declared first: the subscription ends before the IIO device goes.
    listener: FakehidRegistration,
    _iio: AlsIio,
}

fn register_channels<const N: usize>(
    pdev: &platform::Device,
    aop: &Arc<dyn AOP>,
    service: EPICService,
    offset: usize,
    types: [u32; N],
) -> Result<(FakehidRegistration, IIORegistration<MsgProc, N>)> {
    let data = AopSensorData::new_channels(pdev.as_ref().into(), types, MsgProc(offset))?;
    let listener = FakehidRegistration::new(aop.clone(), service, data.clone())?;
    enable_als(aop.as_ref(), pdev, &service, offset)?;
    let info_mask = 1 << bindings::BINDINGS_IIO_CHAN_INFO_PROCESSED;
    let iio = IIORegistration::new_channels(
        data,
        c"aop-sensors-als",
        types,
        info_mask,
        kernel::module::this_module::<crate::LocalModule>(),
    )?;
    Ok((listener, iio))
}

kernel::of_device_table!(
    OF_TABLE,
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
        let (listener, iio) = if offset == LUX_OFFSET_VD6286 {
            let (listener, registration) = register_channels(
                pdev,
                &adata,
                service,
                offset,
                [ty, bindings::BINDINGS_IIO_COLORTEMP],
            )?;
            (
                listener,
                AlsIio::Color {
                    _registration: registration,
                },
            )
        } else if offset == LUX_OFFSET_CT720 {
            let (listener, registration) = register_ct720(pdev, &adata, service, offset)?;
            (
                listener,
                AlsIio::Ct720 {
                    _registration: registration,
                },
            )
        } else {
            let (listener, registration) = register_channels(pdev, &adata, service, offset, [ty])?;
            (
                listener,
                AlsIio::Light {
                    _registration: registration,
                },
            )
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
