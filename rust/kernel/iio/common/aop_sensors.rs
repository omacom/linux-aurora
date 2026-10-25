// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Apple AOP sensors common code
//!
//! Copyright (C) The Asahi Linux Contributors

use core::marker::{PhantomData, PhantomPinned};
use core::ptr;

use kernel::{
    bindings,
    device,
    prelude::*,
    soc::apple::aop::FakehidListener,
    sync::{
        aref::ARef,
        atomic::{
            Acquire,
            Atomic,
            Relaxed,
            Release, //
        },
        Arc, //
    },
    types::ForeignOwnable,
    ThisModule, //
};

/// Micro-units per unit of a channel value.
pub const MICRO: u64 = 1_000_000;

/// The largest reading that can be stored: both the integer part and the micro part have to fit
/// the `i32` values IIO reports. Larger readings saturate.
pub const MAX_MICRO: u64 = i32::MAX as u64 * MICRO + (MICRO - 1);

/// Decodes one fake-HID report of a sensor into the value the IIO channel exposes.
///
/// It is called from the AOP driver's receive worker while the IIO device may be read from
/// another thread, hence `Send + Sync`.
pub trait MessageProcessor: Send + Sync {
    /// Returns the reading carried by `message` in micro-units of the channel (millionths of a
    /// lux, of a degree, ...), or `None` when the message carries no reading.
    fn process(&self, message: &[u8]) -> Option<u64>;

    /// Returns the fixed scale of a raw channel as an integer and a fractional part in
    /// billionths, or `None` for a channel without a scale attribute.
    fn scale(&self) -> Option<(i32, i32)> {
        None
    }

    /// Decodes a channel of a multi-channel sensor. Single-channel processors need only
    /// implement [`Self::process`]. A missing channel leaves its previous reading in place.
    fn process_channel(&self, message: &[u8], channel: usize) -> Option<u64> {
        if channel == 0 {
            self.process(message)
        } else {
            None
        }
    }

    /// Whether `message` says that `channel` has no reading any more, so that the reading it
    /// last had must stop being reported: reads return `ENODATA` until the next one arrives.
    /// It is asked only when [`Self::process_channel`] returned `None` for the same message.
    fn channel_withdrawn(&self, _message: &[u8], _channel: usize) -> bool {
        false
    }
}

/// One IIO channel of a sensor: its type, an optional modifier (`IIO_MOD_*`) and the
/// `info_mask_separate` bits it answers.
#[derive(Clone, Copy)]
pub struct ChannelSpec {
    /// The `IIO_*` channel type.
    pub ty: u32,
    /// The `IIO_MOD_*` modifier of a modified channel.
    pub modifier: Option<u32>,
    /// The `IIO_CHAN_INFO_*` bits of the channel.
    pub info_mask: usize,
}

/// The state shared between a sensor's report listener and its IIO device: the latest reading.
pub struct AopSensorData<T: MessageProcessor, const N: usize = 1> {
    dev: ARef<device::Device>,
    ty: [u32; N],
    /// The latest reading in micro-units, at most [`MAX_MICRO`].
    value: [Atomic<u64>; N],
    /// Whether a reading has arrived at all; until then reads return `ENODATA`.
    valid: [Atomic<bool>; N],
    msg_proc: T,
}

impl<T: MessageProcessor> AopSensorData<T> {
    /// Creates the state for one channel of type `ty` whose reports `msg_proc` decodes.
    pub fn new(dev: ARef<device::Device>, ty: u32, msg_proc: T) -> Result<Arc<AopSensorData<T>>> {
        Self::new_channels(dev, [ty], msg_proc)
    }
}

impl<T: MessageProcessor, const N: usize> AopSensorData<T, N> {
    /// Creates state for the channels in `ty`, in the order decoded by the processor.
    pub fn new_channels(dev: ARef<device::Device>, ty: [u32; N], msg_proc: T) -> Result<Arc<Self>> {
        if N == 0 {
            return Err(EINVAL);
        }
        Ok(Arc::new(
            AopSensorData {
                dev,
                ty,
                value: [const { Atomic::new(0) }; N],
                valid: [const { Atomic::new(false) }; N],
                msg_proc,
            },
            GFP_KERNEL,
        )?)
    }
}

impl<T: MessageProcessor, const N: usize> FakehidListener for AopSensorData<T, N> {
    fn process_fakehid_report(&self, data: &[u8]) -> Result<()> {
        // A report without a reading leaves the last reading in place.
        for channel in 0..N {
            if let Some(value) = self.msg_proc.process_channel(data, channel) {
                self.value[channel].store(value.min(MAX_MICRO), Relaxed);
                // Publishes the reading above to readers that see `valid`.
                self.valid[channel].store(true, Release);
            } else if self.msg_proc.channel_withdrawn(data, channel) {
                self.valid[channel].store(false, Release);
            }
        }
        Ok(())
    }
}

/// The `read_raw` callback: processed values are reported with their micro part, raw values as
/// integers.
unsafe extern "C" fn aop_read_raw<T: MessageProcessor + 'static, const N: usize>(
    dev: *mut bindings::iio_dev,
    chan: *const bindings::iio_chan_spec,
    val: *mut i32,
    val2: *mut i32,
    mask: isize,
) -> i32 {
    // SAFETY: The IIO core calls this with the device and channel that `IIORegistration::new()`
    // registered; `priv_` is the `Arc` it stored there.
    let data = unsafe { Arc::<AopSensorData<T, N>>::borrow((*dev).priv_.cast()) };
    // SAFETY: `chan` is one of the registered channel specs.
    let ty = unsafe { (*chan).type_ };
    // SAFETY: the registration assigns each channel its index as its address.
    let channel = unsafe { (*chan).address as usize };
    if data.ty.get(channel) != Some(&ty) {
        return EINVAL.to_errno();
    }
    // A fixed scale is available even before the first sensor report.
    if mask == bindings::BINDINGS_IIO_CHAN_INFO_SCALE as isize {
        let Some((int, frac)) = data.msg_proc.scale() else {
            return EINVAL.to_errno();
        };
        // SAFETY: `val` and `val2` point to the caller's result variables.
        unsafe {
            *val = int;
            *val2 = frac;
        }
        return bindings::IIO_VAL_INT_PLUS_NANO as i32;
    }
    // A sensor that has not reported yet, for instance because it lacks its
    // calibration, has no reading rather than a reading of zero.
    if !data.valid[channel].load(Acquire) {
        return ENODATA.to_errno();
    }
    let micro = data.value[channel].load(Relaxed);
    // Both parts fit an `i32` because stored readings never exceed `MAX_MICRO`.
    let (int, frac) = ((micro / MICRO) as i32, (micro % MICRO) as i32);
    // SAFETY: `val` and `val2` point to the caller's result variables.
    unsafe {
        *val = int;
        *val2 = frac;
    }
    if mask == bindings::BINDINGS_IIO_CHAN_INFO_PROCESSED as isize {
        bindings::IIO_VAL_INT_PLUS_MICRO as i32
    } else if mask == bindings::BINDINGS_IIO_CHAN_INFO_RAW as isize {
        bindings::IIO_VAL_INT as i32
    } else {
        EINVAL.to_errno()
    }
}

struct IIOSpec<const N: usize> {
    spec: [bindings::iio_chan_spec; N],
    vtable: bindings::iio_info,
    _p: PhantomPinned,
}

/// TODO: add documentation
pub struct IIORegistration<T: MessageProcessor + 'static, const N: usize = 1> {
    dev: *mut bindings::iio_dev,
    spec: Pin<KBox<IIOSpec<N>>>,
    registered: bool,
    _p: PhantomData<AopSensorData<T, N>>,
}

impl<T: MessageProcessor + 'static> IIORegistration<T> {
    /// TODO: add documentation
    pub fn new(
        data: Arc<AopSensorData<T>>,
        name: &'static CStr,
        ty: u32,
        info_mask: usize,
        module: &ThisModule,
    ) -> Result<Self> {
        Self::new_channels(data, name, [ty], info_mask, module)
    }
}

impl<T: MessageProcessor + 'static, const N: usize> IIORegistration<T, N> {
    /// Registers the sensor's channels. The types must match those of `data`.
    pub fn new_channels(
        data: Arc<AopSensorData<T, N>>,
        name: &'static CStr,
        ty: [u32; N],
        info_mask: usize,
        module: &ThisModule,
    ) -> Result<Self> {
        Self::new_specs(
            data,
            name,
            ty.map(|ty| ChannelSpec {
                ty,
                modifier: None,
                info_mask,
            }),
            module,
        )
    }

    /// Registers the sensor's channels, each with its own modifier and info mask. The types
    /// must match those of `data`.
    pub fn new_specs(
        data: Arc<AopSensorData<T, N>>,
        name: &'static CStr,
        channels: [ChannelSpec; N],
        module: &ThisModule,
    ) -> Result<Self> {
        if channels.map(|channel| channel.ty) != data.ty {
            return Err(EINVAL);
        }
        let spec = KBox::pin(
            IIOSpec {
                spec: core::array::from_fn(|channel| {
                    let mut spec = bindings::iio_chan_spec {
                        type_: channels[channel].ty,
                        channel2: channels[channel].modifier.unwrap_or(0) as _,
                        address: channel as _,
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
                        info_mask_separate: channels[channel].info_mask,
                        ..Default::default()
                    };
                    spec.set_modified(channels[channel].modifier.is_some() as _);
                    spec
                }),
                vtable: bindings::iio_info {
                    read_raw: Some(aop_read_raw::<T, N>),
                    ..Default::default()
                },
                _p: PhantomPinned,
            },
            GFP_KERNEL,
        )?;
        let mut this = IIORegistration {
            dev: ptr::null_mut(),
            spec,
            registered: false,
            _p: PhantomData,
        };
        this.dev = unsafe { bindings::iio_device_alloc(data.dev.as_raw(), 0) };
        if this.dev.is_null() {
            return Err(ENOMEM);
        }
        unsafe {
            (*this.dev).priv_ = data.clone().into_foreign().cast();
            (*this.dev).name = name.as_ptr() as _;
            // spec is now pinned
            (*this.dev).channels = this.spec.spec.as_ptr();
            (*this.dev).num_channels = this.spec.spec.len() as i32;
            (*this.dev).info = &this.spec.vtable;
        }
        let ret = unsafe { bindings::__iio_device_register(this.dev, module.as_ptr()) };
        if ret < 0 {
            dev_err!(data.dev, "Unable to register iio sensor");
            return Err(Error::from_errno(ret));
        }
        this.registered = true;
        Ok(this)
    }
}

impl<T: MessageProcessor + 'static, const N: usize> Drop for IIORegistration<T, N> {
    fn drop(&mut self) {
        if self.dev != ptr::null_mut() {
            unsafe {
                if self.registered {
                    bindings::iio_device_unregister(self.dev);
                }
                Arc::<AopSensorData<T, N>>::from_foreign((*self.dev).priv_.cast());
                bindings::iio_device_free(self.dev);
            }
        }
    }
}

unsafe impl<T: MessageProcessor, const N: usize> Send for IIORegistration<T, N> {}
unsafe impl<T: MessageProcessor, const N: usize> Sync for IIORegistration<T, N> {}
