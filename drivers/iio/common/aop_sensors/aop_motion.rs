// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Apple AOP accelerometer and gyroscope driver
//!
//! The Always-On Processor reads the machine's inertial sensor and reports
//! acceleration and angular rate as two services, one fake-HID report per
//! sample. A service starts reporting once it has been sent the machine's
//! three calibration packets and a report interval.
//!
//! The reports run only while someone reads them: while a buffer is enabled,
//! or for the length of one direct read. With nobody reading they stay off,
//! as macOS keeps them (its accel and gyro drivers sit at report interval 0 on
//! an idle desktop), so the AOP does not wake the application processor 50
//! times a second per sensor.

use core::{
    ffi::{c_int, c_void},
    marker::PhantomPinned,
    ptr,
};

use kernel::{
    bindings, c_str,
    device::{self, Core},
    module_platform_driver, new_mutex, new_spinlock, of, platform,
    prelude::*,
    soc::apple::aop::{EPICService, FakehidListener, FakehidRegistration, AOP},
    sync::{
        aref::ARef,
        atomic::{Atomic, Relaxed},
        Arc, Mutex, SpinLock,
    },
    time::{delay::fsleep, Delta},
    types::{ForeignOwnable, Opaque},
};

/// The EPIC notification that writes a property of a sensor service.
const SUBTYPE_SET_PROPERTY: u16 = 4;
/// The report interval property, in microseconds. Zero stops the reports.
const PROP_INTERVAL: u32 = 0;
/// 50 Hz.
const INTERVAL_US: u32 = 20_000;
/// How long a direct read waits for the first report of a fresh start.
const FIRST_SAMPLE_MS: i64 = 200;
const FIRST_SAMPLE_POLL_MS: i64 = 5;
/// The calibration properties of a service and the device tree properties the
/// bootloader fills them from. They are written in this order, before the
/// report interval.
const CALIBRATION: [(u32, &CStr); 3] = [
    (75, c_str!("apple,range-intrinsic-cal")),
    (76, c_str!("apple,nominal-extrinsic-cal")),
    (77, c_str!("apple,range-extrinsic-cal")),
];
/// A sample report is 22 bytes; X, Y and Z are signed little-endian 32-bit
/// values in 1/65536 of the sensor's unit (g or degrees per second), starting
/// at byte 6. The other bytes carry no sample.
const REPORT_LEN: usize = 22;
const XYZ_OFFSET: usize = 6;
/// The status a calibration write is answered with when the service already
/// holds a calibration from an earlier start in the same AOP boot, as after a
/// rebind or a resume. The firmware keeps that calibration, so the reports can
/// be started without it.
const STATUS_UNSUPPORTED: u32 = 0xe000_02c7;

/// What the status of a calibration write means for the start of the reports.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum CalStatus {
    /// The firmware took the packet.
    Taken,
    /// The firmware holds a calibration from an earlier start.
    Held,
    /// Anything else: the reports are not started.
    Refused,
}

fn cal_status(status: u32) -> CalStatus {
    match status {
        0 => CalStatus::Taken,
        STATUS_UNSUPPORTED => CalStatus::Held,
        _ => CalStatus::Refused,
    }
}

/// What one of the two services measures.
struct Kind {
    name: &'static CStr,
    chan_type: u32,
    /// The size of one raw count in m/s^2 or rad/s, in billionths.
    scale_nano: i32,
}

/// 9.80665 / 65536 m/s^2 per count.
static ACCEL: Kind = Kind {
    name: c"aop-sensors-accel",
    chan_type: bindings::BINDINGS_IIO_ACCEL,
    scale_nano: 149_638,
};

/// pi / (180 * 65536) rad/s per count.
static GYRO: Kind = Kind {
    name: c"aop-sensors-gyro",
    chan_type: bindings::BINDINGS_IIO_ANGL_VEL,
    scale_nano: 266,
};

/// Decodes a sample report into its X, Y and Z counts.
fn decode(report: &[u8]) -> Option<[i32; 3]> {
    if report.len() != REPORT_LEN {
        return None;
    }
    let axis = |i: usize| {
        let at = XYZ_OFFSET + 4 * i;
        i32::from_le_bytes([report[at], report[at + 1], report[at + 2], report[at + 3]])
    };
    Some([axis(0), axis(1), axis(2)])
}

fn le_u32(bytes: &[u8], at: usize) -> Option<u32> {
    Some(u32::from_le_bytes(bytes.get(at..at + 4)?.try_into().ok()?))
}

/// Returns the packet for location 0 of a calibration property.
///
/// The property is a container: byte 3 is 1 and byte 2 counts the entries
/// that follow the 4-byte header. Each entry is a little-endian 32-bit
/// location and length, then that many bytes of packet.
fn location0_packet(prop: &[u8]) -> Option<&[u8]> {
    if prop.len() < 4 || prop[3] != 1 {
        return None;
    }
    let mut at = 4;
    for _ in 0..prop[2] {
        let location = le_u32(prop, at)?;
        let len = usize::try_from(le_u32(prop, at + 4)?).ok()?;
        let start = at + 8;
        let packet = prop.get(start..start.checked_add(len)?)?;
        if location == 0 {
            return Some(packet);
        }
        at = start + len;
    }
    None
}

/// Who wants the reports of a service.
#[derive(Default)]
struct Stream {
    /// A buffer is enabled.
    buffer_on: bool,
    /// Set across system sleep and after unbind: no reports.
    parked: bool,
    /// The reports were started for the buffer.
    active: bool,
}

/// Starts and stops the reports of one service.
#[pin_data]
struct Control {
    dev: ARef<device::Device>,
    aop: Arc<dyn AOP>,
    service: EPICService,
    packets: [KVec<u8>; 3],
    /// Serializes starting and stopping the reports.
    #[pin]
    stream: Mutex<Stream>,
}

impl Control {
    fn new(dev: &device::Device, aop: Arc<dyn AOP>, service: EPICService) -> Result<Arc<Self>> {
        let node = dev.fwnode().ok_or(ENODEV)?;
        let mut packets = [KVec::new(), KVec::new(), KVec::new()];
        for ((_, name), packet) in CALIBRATION.iter().zip(packets.iter_mut()) {
            if !node.property_present(name) {
                dev_info!(dev, "no {:?} from the bootloader, sensor left off\n", name);
                return Err(ENODEV);
            }
            let len = node.property_count_elem::<u8>(name)?;
            let prop = node
                .property_read_array_vec::<u8>(name, len)?
                .required_by(dev)?;
            let Some(data) = location0_packet(&prop) else {
                dev_err!(dev, "{:?} holds no calibration for location 0\n", name);
                return Err(EINVAL);
            };
            packet.extend_from_slice(data, GFP_KERNEL)?;
        }
        Arc::pin_init(
            pin_init!(Self {
                dev: dev.into(),
                aop,
                service,
                packets,
                stream <- new_mutex!(Stream::default()),
            }),
            GFP_KERNEL,
        )
    }

    /// Writes a property and returns the firmware's status.
    fn write(&self, prop: u32, value: &[u8]) -> Result<u32> {
        let mut msg = KVec::with_capacity(8 + value.len(), GFP_KERNEL)?;
        msg.extend_from_slice(&0u32.to_le_bytes(), GFP_KERNEL)?;
        msg.extend_from_slice(&prop.to_le_bytes(), GFP_KERNEL)?;
        msg.extend_from_slice(value, GFP_KERNEL)?;
        self.aop
            .epic_call(&self.service, SUBTYPE_SET_PROPERTY, &msg)
    }

    fn refused(&self, prop: u32, status: u32) -> Error {
        dev_err!(
            self.dev,
            "property {} refused: status {:#x}\n",
            prop,
            status
        );
        EIO
    }

    fn set(&self, prop: u32, value: &[u8]) -> Result {
        match self.write(prop, value)? {
            0 => Ok(()),
            status => Err(self.refused(prop, status)),
        }
    }

    /// Sends the calibration, then the interval that starts the reports.
    fn start(&self) -> Result {
        let mut held = 0;
        for ((prop, _), packet) in CALIBRATION.iter().zip(self.packets.iter()) {
            let status = self.write(*prop, packet)?;
            match cal_status(status) {
                CalStatus::Taken => {}
                CalStatus::Held => held += 1,
                CalStatus::Refused => return Err(self.refused(*prop, status)),
            }
        }
        if held > 0 {
            dev_info!(
                self.dev,
                "{} of {} calibration packets kept from an earlier start\n",
                held,
                CALIBRATION.len()
            );
        }
        self.set(PROP_INTERVAL, &INTERVAL_US.to_le_bytes())
    }

    fn stop(&self) -> Result {
        self.set(PROP_INTERVAL, &0u32.to_le_bytes())
    }

    /// Starts or stops the buffer's reports to match their users. The caller
    /// holds the stream lock.
    fn sync_locked(&self, s: &mut Stream) -> Result {
        match (s.buffer_on && !s.parked, s.active) {
            (true, false) => {
                self.start()?;
                s.active = true;
                Ok(())
            }
            (false, true) => {
                s.active = false;
                self.stop()
            }
            _ => Ok(()),
        }
    }

    fn set_buffer(&self, on: bool) -> Result {
        let mut s = self.stream.lock();
        s.buffer_on = on;
        let ret = self.sync_locked(&mut s);
        if on && ret.is_err() {
            // The IIO core does not call predisable after a failed enable.
            s.buffer_on = false;
        }
        ret
    }

    fn set_parked(&self, parked: bool) -> Result {
        let mut s = self.stream.lock();
        s.parked = parked;
        self.sync_locked(&mut s)
    }
}

/// The latest sample, read by the IIO callbacks.
#[derive(Clone, Copy, Default)]
struct Sample {
    xyz: [i32; 3],
    valid: bool,
}

/// One buffer scan: X, Y and Z, then the timestamp at the next 8-byte
/// boundary, where the IIO core expects it.
#[repr(C, align(8))]
struct Scan {
    xyz: [i32; 3],
    pad: u32,
    timestamp: i64,
}

/// The state shared by the report listener and the IIO device.
#[pin_data]
struct State {
    kind: &'static Kind,
    /// The IIO device, which outlives the report listener.
    iio: IioPtr,
    control: Arc<Control>,
    #[pin]
    sample: SpinLock<Sample>,
}

impl State {
    /// Returns the latest sample. Without a buffer the reports are started
    /// only for this read and stopped again after it.
    fn read_now(&self) -> Result<[i32; 3]> {
        let s = self.control.stream.lock();
        if !s.active {
            if s.parked {
                return Err(EBUSY);
            }
            // A sample from an earlier start may be stale.
            self.sample.lock().valid = false;
            self.control.start()?;
            let mut waited = 0;
            while waited < FIRST_SAMPLE_MS && !self.sample.lock().valid {
                fsleep(Delta::from_millis(FIRST_SAMPLE_POLL_MS));
                waited += FIRST_SAMPLE_POLL_MS;
            }
            self.control.stop()?;
        }
        let sample = *self.sample.lock();
        if sample.valid {
            Ok(sample.xyz)
        } else {
            Err(ENODATA)
        }
    }
}

struct IioPtr(*mut bindings::iio_dev);

// SAFETY: The IIO device may be used from any thread; the functions this
// driver calls on it from the listener are safe from any context.
unsafe impl Send for IioPtr {}
// SAFETY: See above.
unsafe impl Sync for IioPtr {}

impl FakehidListener for State {
    fn process_fakehid_report(&self, data: &[u8]) -> Result<()> {
        let xyz = decode(data).ok_or(EMSGSIZE)?;
        *self.sample.lock() = Sample { xyz, valid: true };
        // SAFETY: The listener is removed before the IIO device is freed.
        let timestamp = unsafe { bindings::iio_get_time_ns(self.iio.0) };
        let scan = Scan {
            xyz,
            pad: 0,
            timestamp,
        };
        // SAFETY: As above; `scan` holds the full scan of all three axes,
        // which is the only scan mask the device offers, followed by the
        // timestamp. The core copies what the enabled buffers take.
        unsafe { bindings::iio_push_to_buffers(self.iio.0, ptr::from_ref(&scan).cast()) };
        Ok(())
    }
}

/// The threaded half of the poll function.
///
/// The trigger never fires, so this never runs; it exists because the IRQ
/// core rejects a oneshot poll function without a threaded handler.
///
/// # Safety
///
/// The IIO core calls it with the device's `iio_poll_func`.
unsafe extern "C" fn motion_trigger_handler(
    _irq: core::ffi::c_int,
    p: *mut core::ffi::c_void,
) -> bindings::irqreturn_t {
    let pf = p.cast::<bindings::iio_poll_func>();
    // SAFETY: `p` is the poll function of a registered device that has a
    // current trigger while its buffer runs.
    unsafe { bindings::iio_trigger_notify_done((*(*pf).indio_dev).trig) };
    bindings::irqreturn_IRQ_HANDLED as bindings::irqreturn_t
}

/// The only scan the device offers: X, Y and Z.
static SCAN_MASKS: [usize; 2] = [0b111, 0];

/// The `read_raw` callback of both devices.
///
/// # Safety
///
/// The IIO core calls it with a device registered by [`MotionIio`], one of its
/// channels and valid result pointers.
unsafe extern "C" fn motion_read_raw(
    indio_dev: *mut bindings::iio_dev,
    chan: *const bindings::iio_chan_spec,
    val: *mut i32,
    val2: *mut i32,
    mask: isize,
) -> i32 {
    // SAFETY: The IIO core calls this with the device `MotionIio::new()`
    // registered, whose private data is the `Arc` stored there.
    let state = unsafe { Arc::<State>::borrow((*indio_dev).priv_.cast()) };
    // SAFETY: `chan` is one of the registered channels.
    let axis = unsafe { (*chan).address };
    let (int, frac, ret) = if mask == bindings::BINDINGS_IIO_CHAN_INFO_RAW as isize {
        let xyz = match state.read_now() {
            Ok(xyz) => xyz,
            Err(e) => return e.to_errno(),
        };
        let Some(&count) = xyz.get(axis) else {
            return EINVAL.to_errno();
        };
        (count, 0, bindings::IIO_VAL_INT)
    } else if mask == bindings::BINDINGS_IIO_CHAN_INFO_SCALE as isize {
        (0, state.kind.scale_nano, bindings::IIO_VAL_INT_PLUS_NANO)
    } else if mask == bindings::BINDINGS_IIO_CHAN_INFO_SAMP_FREQ as isize {
        ((1_000_000 / INTERVAL_US) as i32, 0, bindings::IIO_VAL_INT)
    } else {
        return EINVAL.to_errno();
    };
    // SAFETY: `val` and `val2` point to the caller's result variables.
    unsafe {
        *val = int;
        *val2 = frac;
    }
    ret as i32
}

/// Starts the reports for an enabled buffer.
///
/// # Safety
///
/// The IIO core calls it with a device registered by [`MotionIio`].
unsafe extern "C" fn buffer_postenable(indio_dev: *mut bindings::iio_dev) -> c_int {
    // SAFETY: The private data is the `Arc<State>` stored by `MotionIio::new()`.
    let state = unsafe { Arc::<State>::borrow((*indio_dev).priv_.cast()) };
    match state.control.set_buffer(true) {
        Ok(()) => 0,
        Err(e) => e.to_errno(),
    }
}

/// Stops the reports when the buffer is disabled.
///
/// # Safety
///
/// As [`buffer_postenable`].
unsafe extern "C" fn buffer_predisable(indio_dev: *mut bindings::iio_dev) -> c_int {
    // SAFETY: As in `buffer_postenable()`.
    let state = unsafe { Arc::<State>::borrow((*indio_dev).priv_.cast()) };
    match state.control.set_buffer(false) {
        Ok(()) => 0,
        Err(e) => e.to_errno(),
    }
}

static BUFFER_OPS: bindings::iio_buffer_setup_ops = bindings::iio_buffer_setup_ops {
    preenable: None,
    postenable: Some(buffer_postenable),
    predisable: Some(buffer_predisable),
    postdisable: None,
    validate_scan_mask: None,
};

fn axis_channel(chan_type: u32, modifier: u32, index: i32) -> bindings::iio_chan_spec {
    let mut chan = bindings::iio_chan_spec {
        type_: chan_type,
        channel2: modifier as i32,
        address: index as usize,
        scan_index: index,
        __bindgen_anon_1: bindings::iio_chan_spec__bindgen_ty_1 {
            scan_type: bindings::iio_scan_type {
                __bindgen_anon_1: bindings::iio_scan_type__bindgen_ty_1 { sign: b's' },
                realbits: 32,
                storagebits: 32,
                ..Default::default()
            },
        },
        info_mask_separate: 1usize << bindings::BINDINGS_IIO_CHAN_INFO_RAW,
        info_mask_shared_by_type: (1usize << bindings::BINDINGS_IIO_CHAN_INFO_SCALE)
            | (1usize << bindings::BINDINGS_IIO_CHAN_INFO_SAMP_FREQ),
        ..Default::default()
    };
    chan.set_modified(1);
    chan
}

fn timestamp_channel(index: i32) -> bindings::iio_chan_spec {
    bindings::iio_chan_spec {
        type_: bindings::BINDINGS_IIO_TIMESTAMP,
        channel: -1,
        scan_index: index,
        __bindgen_anon_1: bindings::iio_chan_spec__bindgen_ty_1 {
            scan_type: bindings::iio_scan_type {
                __bindgen_anon_1: bindings::iio_scan_type__bindgen_ty_1 { sign: b's' },
                realbits: 64,
                storagebits: 64,
                ..Default::default()
            },
        },
        ..Default::default()
    }
}

struct Spec {
    channels: [bindings::iio_chan_spec; 4],
    info: bindings::iio_info,
    _p: PhantomPinned,
}

/// The IIO device of a service, with its buffer.
struct MotionIio {
    dev: *mut bindings::iio_dev,
    _spec: Pin<KBox<Spec>>,
    registered: bool,
}

impl MotionIio {
    /// Allocates the device and its buffer; [`MotionIio::register`] makes it
    /// visible.
    fn new(
        pdev: &platform::Device<Core<'_>>,
        kind: &'static Kind,
        control: Arc<Control>,
    ) -> Result<(Self, Arc<State>)> {
        let dev = pdev.as_ref();
        let spec = KBox::pin(
            Spec {
                channels: [
                    axis_channel(kind.chan_type, bindings::BINDINGS_IIO_MOD_X, 0),
                    axis_channel(kind.chan_type, bindings::BINDINGS_IIO_MOD_Y, 1),
                    axis_channel(kind.chan_type, bindings::BINDINGS_IIO_MOD_Z, 2),
                    timestamp_channel(3),
                ],
                info: bindings::iio_info {
                    read_raw: Some(motion_read_raw),
                    // The buffer fills from the AOP's reports, not from a
                    // trigger, so only the device's own trigger fits it.
                    validate_trigger: Some(bindings::iio_validate_own_trigger),
                    ..Default::default()
                },
                _p: PhantomPinned,
            },
            GFP_KERNEL,
        )?;
        // SAFETY: `dev` is a valid device.
        let indio_dev = unsafe { bindings::iio_device_alloc(dev.as_raw(), 0) };
        if indio_dev.is_null() {
            return Err(ENOMEM);
        }
        let state = match Arc::pin_init(
            pin_init!(State {
                kind,
                iio: IioPtr(indio_dev),
                control,
                sample <- new_spinlock!(Sample::default()),
            }),
            GFP_KERNEL,
        ) {
            Ok(state) => state,
            Err(e) => {
                // SAFETY: Allocated above and not registered.
                unsafe { bindings::iio_device_free(indio_dev) };
                return Err(e);
            }
        };
        // SAFETY: `indio_dev` was allocated above and is not registered yet;
        // `spec` is pinned and lives as long as the device.
        unsafe {
            (*indio_dev).priv_ = state.clone().into_foreign().cast();
            (*indio_dev).name = kind.name.as_char_ptr();
            (*indio_dev).channels = spec.channels.as_ptr();
            (*indio_dev).num_channels = spec.channels.len() as i32;
            (*indio_dev).info = &spec.info;
            (*indio_dev).modes = bindings::INDIO_DIRECT_MODE as i32;
            (*indio_dev).available_scan_masks = SCAN_MASKS.as_ptr();
        }
        let this = Self {
            dev: indio_dev,
            _spec: spec,
            registered: false,
        };
        // The buffer fills as the AOP reports, so the trigger never fires.
        // It exists because userspace finds a buffered sensor through its
        // trigger: iio-sensor-proxy looks for "<name>-dev<id>" and does not
        // use a buffer without one.
        // SAFETY: `dev` is valid and `indio_dev` is allocated; the format
        // takes the device name and its id, both valid here.
        let module = kernel::module::this_module::<crate::LocalModule>();
        let trig = unsafe {
            bindings::__devm_iio_trigger_alloc(
                dev.as_raw(),
                module.as_ptr(),
                c"%s-dev%d".as_char_ptr(),
                kind.name.as_char_ptr(),
                bindings::iio_device_id(indio_dev),
            )
        };
        if trig.is_null() {
            return Err(ENOMEM);
        }
        // SAFETY: `trig` was allocated above; it is unregistered and freed
        // with the platform device's resources, after the IIO device that
        // holds a reference to it.
        let ret = unsafe { bindings::devm_iio_trigger_register(dev.as_raw(), trig) };
        if ret < 0 {
            return Err(Error::from_errno(ret));
        }
        // SAFETY: `indio_dev` is not registered yet; the reference taken
        // here is dropped when the IIO device is released.
        unsafe { (*indio_dev).trig = bindings::iio_trigger_get(trig) };
        // SAFETY: Both devices are valid. The buffer is released with the
        // platform device's resources; the IIO device holds its own
        // reference to it until it is freed. The poll function only stamps
        // the time and completes the trigger; the report listener pushes the
        // scans. The setup ops start the reports for the buffer.
        let ret = unsafe {
            bindings::devm_iio_triggered_buffer_setup_ext(
                dev.as_raw(),
                indio_dev,
                Some(bindings::iio_pollfunc_store_time),
                Some(motion_trigger_handler),
                bindings::iio_buffer_direction_IIO_BUFFER_DIRECTION_IN,
                &BUFFER_OPS,
                ptr::null_mut(),
            )
        };
        if ret < 0 {
            return Err(Error::from_errno(ret));
        }
        Ok((this, state))
    }

    fn register(&mut self) -> Result {
        let module = kernel::module::this_module::<crate::LocalModule>();
        // SAFETY: `self.dev` is a fully set up, unregistered IIO device.
        let ret = unsafe { bindings::__iio_device_register(self.dev, module.as_ptr()) };
        if ret < 0 {
            return Err(Error::from_errno(ret));
        }
        self.registered = true;
        Ok(())
    }
}

impl Drop for MotionIio {
    fn drop(&mut self) {
        // SAFETY: `self.dev` was allocated in `new()` with an `Arc<State>`
        // as its private data, and nothing uses it after this.
        unsafe {
            if self.registered {
                bindings::iio_device_unregister(self.dev);
            }
            drop(Arc::<State>::from_foreign((*self.dev).priv_.cast()));
            bindings::iio_device_free(self.dev);
        }
    }
}

// SAFETY: The IIO core serializes registration and removal internally.
unsafe impl Send for MotionIio {}
// SAFETY: See above; `MotionIio` has no methods taking `&self`.
unsafe impl Sync for MotionIio {}

/// Stops the buffer's reports around system sleep, so that the AOP does not
/// wake the application processor 50 times a second, and starts them again
/// after if a buffer still wants them.
#[repr(C)]
struct PmHook {
    /// First, so that the callback finds the hook from it.
    nb: Opaque<bindings::notifier_block>,
    control: Arc<Control>,
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
    let ret = match action as u32 {
        bindings::PM_SUSPEND_PREPARE | bindings::PM_HIBERNATION_PREPARE => {
            hook.control.set_parked(true)
        }
        bindings::PM_POST_SUSPEND | bindings::PM_POST_HIBERNATION => hook.control.set_parked(false),
        _ => return bindings::NOTIFY_DONE as c_int,
    };
    if let Err(e) = ret {
        dev_warn!(
            hook.control.dev,
            "reports not switched for sleep: {:?}\n",
            e
        );
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
    fn new(control: Arc<Control>) -> Result<Pin<KBox<Self>>> {
        let hook = KBox::pin(
            Self {
                nb: Opaque::new(bindings::notifier_block {
                    notifier_call: Some(pm_event),
                    next: ptr::null_mut(),
                    priority: 0,
                }),
                control,
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

struct IIOAopMotionDriver {
    /// Fields are declared in teardown order: sleep handling first, then the
    /// subscription, which ends before the IIO device goes.
    pm: Pin<KBox<PmHook>>,
    listener: FakehidRegistration,
    control: Arc<Control>,
    _iio: MotionIio,
}

kernel::of_device_table!(
    OF_TABLE,
    &'static Kind,
    [
        (of::DeviceId::new(c_str!("apple,aop-accel")), &ACCEL),
        (of::DeviceId::new(c_str!("apple,aop-gyro")), &GYRO),
    ]
);

impl platform::Driver for IIOAopMotionDriver {
    type IdInfo = &'static Kind;
    type Data<'bound> = IIOAopMotionDriver;

    const OF_ID_TABLE: Option<of::IdTable<Self::IdInfo>> = Some(&OF_TABLE);

    fn probe<'bound>(
        pdev: &'bound platform::Device<Core<'_>>,
        info: Option<&'bound Self::IdInfo>,
    ) -> impl PinInit<Self::Data<'bound>, Error> + 'bound {
        let kind = *info.ok_or(ENODEV)?;
        let dev = pdev.as_ref();
        // SAFETY: This driver binds only to the service devices the AOP core
        // registers, and `pdev` is being probed.
        let aop = unsafe { <dyn AOP>::from_child(dev) }?;
        // SAFETY: `dev` is a valid device; the AOP core attached its service
        // record as the platform data of every service device it registers.
        let service = unsafe { (*dev.as_raw()).platform_data.cast::<EPICService>() };
        if service.is_null() {
            return Err(ENODEV);
        }
        // SAFETY: The record lives as long as the device it is attached to.
        let service = unsafe { *service };

        let control = Control::new(dev, aop.clone(), service)?;
        let (mut iio, state) = MotionIio::new(pdev, kind, control.clone())?;
        let listener = FakehidRegistration::new(aop, service, state)?;
        let pm = PmHook::new(control.clone())?;
        // Check once that the service takes the calibration and the interval,
        // then leave the reports off until someone reads them.
        control.start()?;
        control.stop()?;
        iio.register()?;
        Ok(IIOAopMotionDriver {
            pm,
            listener,
            control,
            _iio: iio,
        })
    }

    fn unbind(_dev: &platform::Device<Core<'_>>, this: Pin<&Self>) {
        this.pm.unregister();
        let _ = this.control.set_parked(true);
        this.listener.unregister();
    }
}

module_platform_driver! {
    type: IIOAopMotionDriver,
    name: "iio_aop_motion",
    description: "AOP accelerometer and gyroscope driver",
    license: "Dual MIT/GPL",
    alias: ["platform:iio_aop_motion"],
}
