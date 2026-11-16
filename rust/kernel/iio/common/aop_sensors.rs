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
            Atomic,
            Relaxed, //
        },
        barrier::{
            smp_rmb,
            smp_wmb, //
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

/// How many readings one sensor can publish from a single report.
pub const MAX_CHANS: usize = 6;

/// Decodes one fake-HID report of a sensor into the value the IIO channel exposes.
///
/// It is called from the AOP driver's receive worker while the IIO device may be read from
/// another thread, hence `Send + Sync`.
pub trait MessageProcessor: Send + Sync {
    /// Returns the reading carried by `message` in micro-units of channel 0 (millionths of a
    /// lux, of a degree, ...), or `None` when the message carries no reading.
    fn process(&self, message: &[u8]) -> Option<u64>;

    /// Fills `out` with this report's channel values, in micro-units, and returns a bit mask of
    /// the slots it wrote. Bit 0 is the channel [`IIORegistration::new`] registers. A zero mask
    /// leaves the previous readings in place.
    fn decode(&self, message: &[u8], out: &mut [u64]) -> u32 {
        match self.process(message) {
            Some(value) if !out.is_empty() => {
                out[0] = value;
                1
            }
            _ => 0,
        }
    }
}

/// The state shared between a sensor's report listener and its IIO device.
pub struct AopSensorData<T: MessageProcessor> {
    dev: ARef<device::Device>,
    /// Type of channel 0. Extra channels are selected by `iio_chan_spec.address`.
    ty: u32,
    /// Even while the slots are stable, odd while a report is being published.
    seq: Atomic<u64>,
    /// Latest readings in micro-units, at most [`MAX_MICRO`].
    values: [Atomic<u64>; MAX_CHANS],
    /// Bit per channel: set once that slot holds a reading.
    valid: Atomic<u64>,
    msg_proc: T,
}

impl<T: MessageProcessor> AopSensorData<T> {
    /// Creates the state for a sensor whose channel 0 has type `ty` and whose reports `msg_proc`
    /// decodes.
    pub fn new(dev: ARef<device::Device>, ty: u32, msg_proc: T) -> Result<Arc<AopSensorData<T>>> {
        Ok(Arc::new(
            AopSensorData {
                dev,
                ty,
                seq: Atomic::new(0),
                values: [
                    Atomic::new(0),
                    Atomic::new(0),
                    Atomic::new(0),
                    Atomic::new(0),
                    Atomic::new(0),
                    Atomic::new(0),
                ],
                valid: Atomic::new(0),
                msg_proc,
            },
            GFP_KERNEL,
        )?)
    }

    /// One stable reading, or an errno when none has arrived or a publish is in progress.
    fn load(&self, idx: usize) -> Result<u64, i32> {
        // The writer holds the sequence odd only across a few stores. A reader that keeps
        // spinning can deadlock if it preempted that writer on this CPU, so give up.
        let mut spins = 0;
        while spins < 8 {
            let seq = self.seq.load(Relaxed);
            smp_rmb();
            if seq & 1 != 0 {
                spins += 1;
                continue;
            }
            let valid = self.valid.load(Relaxed);
            let value = self.values[idx].load(Relaxed);
            smp_rmb();
            if self.seq.load(Relaxed) != seq {
                spins += 1;
                continue;
            }
            if valid & (1u64 << idx) == 0 {
                return Err(ENODATA.to_errno());
            }
            return Ok(value);
        }
        Err(EAGAIN.to_errno())
    }
}

impl<T: MessageProcessor> FakehidListener for AopSensorData<T> {
    fn process_fakehid_report(&self, data: &[u8]) -> Result<()> {
        let mut raw = [0u64; MAX_CHANS];
        let mask = self.msg_proc.decode(data, &mut raw);
        // A report without a reading leaves the last reading in place.
        if mask == 0 {
            return Ok(());
        }
        let seq = self.seq.load(Relaxed);
        self.seq.store(seq.wrapping_add(1), Relaxed);
        smp_wmb();
        for i in 0..MAX_CHANS {
            if (mask & (1u32 << i)) != 0 {
                self.values[i].store(raw[i].min(MAX_MICRO), Relaxed);
            }
        }
        // Replace, do not OR. A CT720 report that fails the layout check
        // contributes only the lux bit, and the previous color sample must
        // read back as ENODATA rather than stay published. A zero mask never
        // reaches here, so a report with no reading keeps the last sample.
        self.valid.store(u64::from(mask), Relaxed);
        smp_wmb();
        self.seq.store(seq.wrapping_add(2), Relaxed);
        Ok(())
    }
}

/// The `read_raw` callback: processed values are reported with their micro part, raw values as
/// integers.
unsafe extern "C" fn aop_read_raw<T: MessageProcessor + 'static>(
    dev: *mut bindings::iio_dev,
    chan: *const bindings::iio_chan_spec,
    val: *mut i32,
    val2: *mut i32,
    mask: isize,
) -> i32 {
    // SAFETY: The IIO core calls this with the device and channel that `IIORegistration`
    // registered; `priv_` is the `Arc` it stored there.
    let data = unsafe { Arc::<AopSensorData<T>>::borrow((*dev).priv_.cast()) };
    // SAFETY: `chan` is one of the registered channel specs.
    let (idx, ty) = unsafe { ((*chan).address, (*chan).type_) };
    if idx >= MAX_CHANS {
        return EINVAL.to_errno();
    }
    // Channel 0 is the original sensor type. Later channels carry their own type.
    if idx == 0 && data.ty != ty {
        return EINVAL.to_errno();
    }
    let micro = match data.load(idx) {
        Ok(micro) => micro,
        Err(err) => return err,
    };
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

struct IIOSpec {
    spec: [bindings::iio_chan_spec; MAX_CHANS],
    n: usize,
    vtable: bindings::iio_info,
    _p: PhantomPinned,
}

/// TODO: add documentation
pub struct IIORegistration<T: MessageProcessor + 'static> {
    dev: *mut bindings::iio_dev,
    spec: Pin<KBox<IIOSpec>>,
    registered: bool,
    _p: PhantomData<AopSensorData<T>>,
}

fn one_channel(ty: u32, info_mask: usize) -> bindings::iio_chan_spec {
    bindings::iio_chan_spec {
        type_: ty,
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
    }
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
        Self::with_channels(data, name, &[one_channel(ty, info_mask)], module)
    }

    /// Registers `channels` on one IIO device. Each spec's `address` is the slot `decode` fills.
    /// `channels` must be non-empty and no longer than [`MAX_CHANS`].
    pub fn with_channels(
        data: Arc<AopSensorData<T>>,
        name: &'static CStr,
        channels: &[bindings::iio_chan_spec],
        module: &ThisModule,
    ) -> Result<Self> {
        if channels.is_empty() || channels.len() > MAX_CHANS {
            return Err(EINVAL);
        }
        let mut spec = [one_channel(0, 0); MAX_CHANS];
        spec[..channels.len()].copy_from_slice(channels);
        let spec = KBox::pin(
            IIOSpec {
                spec,
                n: channels.len(),
                vtable: bindings::iio_info {
                    read_raw: Some(aop_read_raw::<T>),
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
            (*this.dev).num_channels = this.spec.n as i32;
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

impl<T: MessageProcessor + 'static> Drop for IIORegistration<T> {
    fn drop(&mut self) {
        if self.dev != ptr::null_mut() {
            unsafe {
                if self.registered {
                    bindings::iio_device_unregister(self.dev);
                }
                Arc::<AopSensorData<T>>::from_foreign((*self.dev).priv_.cast());
                bindings::iio_device_free(self.dev);
            }
        }
    }
}

unsafe impl<T: MessageProcessor> Send for IIORegistration<T> {}
unsafe impl<T: MessageProcessor> Sync for IIORegistration<T> {}
