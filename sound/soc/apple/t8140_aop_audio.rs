// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![recursion_limit = "2048"]

//! Apple T8140 (J700, MacBook Neo) AOP audio
//!
//! The AOP firmware owns the audio fabric on this SoC, so the paths the AP
//! sees are its services:
//!
//! * `lpai`, the low-power microphone: 16 kHz stereo S32 frames (24
//!   meaningful bits) produced into a 512 KiB host ring mapped through
//!   dart-aop stream 9, with a cursor report every 3,200 frames.  There is
//!   no DMA engine on the host side: the ring is bound to the firmware over
//!   the AOP setup port (the AOP driver owns it, the firmware binds it once
//!   per boot, and hands out reads of it) and the driver copies each
//!   reported span into the ALSA buffer of the first PCM of its own card.
//! * `hpai`, the high-quality microphone array: a PDM part behind the AOP,
//!   read back as float32 over leap-s ADMAC RX0.  It has no codec and no
//!   serializer of ours, so it is the second PCM of the driver's own card,
//!   as the other AOP microphones are on the machines with an `aop_audio`
//!   node.
//! * `cout`/`cin` (the CS42L83 jack on MCA1, base-ns ADMAC TX2/RX2), `spkr`
//!   (the MAX98360A speakers on LEAP TX0, leap-ns ADMAC TX0) and `tap `
//!   (the speaker tap, base-ns RX1): ASoC back-end DAIs that attach the
//!   service, request its power states (`pw0 ` idle, `pwrd` running, the
//!   firmware then runs the serializer clocks) and runtime-power the
//!   AP-side PMGR leaves, with DPCM front-end PCMs on the ADMAC channels.
//!   The macaudio J700 variant links them to the codecs.
//!
//! The speaker front-end is copy-only: the LEAP consumes float32 and the
//! driver converts each S32 sample through the amplifier's full scale and
//! the "Speaker Playback Volume" control, which the card's volume lock holds
//! 20 dB down whenever no speaker-protection daemon owns it.
//!
//! Closing the lid disconnects both internal microphones in hardware.  The
//! array then reads back as digital silence, but the low-power microphone's
//! decimator, fed a PDM line held low, reads back as a constant close to
//! negative full scale.  The driver follows the lid switch through the input
//! core: while a lid is closed its card's "Internal Mic" jack reads
//! unplugged, so userspace stops offering the microphones.
//!
//! Copyright (C) The Asahi Linux Contributors
//! Copyright (C) 2026 The Aurora Silicon Contributors

use core::{
    mem,
    ptr,
    slice, //
};

use kernel::{
    bindings,
    c_str,
    device,
    device::property::FwNode,
    device::Core,
    dma::{
        Device as DmaDevice,
        DmaMask, //
    },
    error::from_err_ptr,
    module_platform_driver,
    new_mutex,
    new_spinlock,
    of,
    platform,
    prelude::*,
    soc::apple::aop::{
        from_fourcc,
        EPICService,
        ReportListener,
        SourceRing,
        AOP, //
    },
    str::CString,
    sync::{
        aref::ARef,
        atomic::{
            Acquire,
            Atomic,
            AtomicFlag,
            Relaxed,
            Release, //
        },
        Arc,
        ArcBorrow,
        Mutex,
        SpinLock, //
    },
    types::{ForeignOwnable, Opaque},
    workqueue::{
        self,
        impl_has_work,
        new_work,
        Work,
        WorkItem, //
    }, //
};

const EPIC_SUBTYPE_WRAPPED_CALL: u16 = 0x20;
const EPIC_SUBTYPE_PRODUCER_REPORT: u16 = 0x20;
const CALLTYPE_AUDIO_ATTACH_DEVICE: u32 = 0xc3000002;
const CALLTYPE_AUDIO_GET_PROP: u32 = 0xc3000004;
const CALLTYPE_AUDIO_SET_PROP: u32 = 0xc3000005;
const AUDIO_DEV_LPAI: u32 = from_fourcc(b"lpai");
const PROP_POWER_STATE: u32 = 200;
const PROP_POWER_REQUEST: u32 = 202;
const PROP_CHANNEL_CONTROL: u32 = 300;
const PROP_STREAM_FORMAT: u32 = 301;
const PROP_RING_GEOMETRY: u32 = 302;
/// Output services: 4-byte boolean, 1 mutes (with the firmware's ramp).
const PROP_OUTPUT_MUTE: u32 = 700;
const POWER_STATE_RUN: u32 = from_fourcc(b"runn");
const POWER_STATE_IDLE: u32 = from_fourcc(b"idle");

const AUDIO_DEV_SPKR: u32 = from_fourcc(b"spkr");
const AUDIO_DEV_TAP: u32 = from_fourcc(b"tap ");
const AUDIO_DEV_COUT: u32 = from_fourcc(b"cout");
const AUDIO_DEV_CIN: u32 = from_fourcc(b"cin ");
/// The high-quality microphone array (ADT /arm-io/iop-audio-controller/
/// audio-hp-mic, identifier 'iaph' -- ADT FourCCs are byte-reversed).  Unlike
/// `lpai` it is the machine's only `iop-audio-s` stream: its data comes back
/// over leap-s ADMAC RX0, not the AOP source ring.
const AUDIO_DEV_HPAI: u32 = from_fourcc(b"hpai");
/// kIOReturnBusy from the firmware: the device is already attached.
const AUDIO_RET_BUSY: u32 = 0xe00002d5;
const SPKR_POWER_STATE_PW0: u32 = from_fourcc(b"pw0 ");
const SPKR_POWER_STATE_PWRD: u32 = from_fourcc(b"pwrd");

/// Leaf power-up retries while the firmware raises the fabric (see `PowerDomain::get`).
/// Blind attempts before the fabric is raised, and attempts while it rises.
const PD_GET_ATTEMPTS: u32 = 3;
const PD_GET_RAISED_ATTEMPTS: u32 = 60;
const PD_GET_RETRY_MS: i64 = 10;
/// The fabric comes up some tens to hundreds of milliseconds after the
/// low-power microphone is asked to run (45–200 ms measured at boot); its
/// first report proves the stream, and so the fabric, is up. Poll for it
/// before the first leaf request so that request does not time out, and
/// log, in the PMGR driver.
const FABRIC_RAISE_POLL_MS: i64 = 5;
const FABRIC_RAISE_POLLS: u32 = 100;

/// The low-power microphone stream: 16 kHz stereo S32 in a 512 KiB ring
/// reported every 3,200 frames.
const LPAI_RATE: u32 = 16000;
const LPAI_CHANNELS: u32 = 2;
const LPAI_FRAME_BYTES: usize = 8;
const RING_BYTES: usize = 0x80000;
const LPAI_PERIOD_FRAMES: usize = 3200;
const LPAI_PERIOD_BYTES: usize = LPAI_PERIOD_FRAMES * LPAI_FRAME_BYTES;
const LPAI_PERIODS_MAX: u32 = 16;
const REPORT_MIN_LEN: usize = 0x68;

/// The high-quality microphone array: 48 kHz stereo float32 over the LEAP
/// ADMAC.
const HPAI_RATE: u32 = 48000;
const HPAI_CHANNELS: u32 = 2;

/// Firmware power requests carry a timestamp in the 24 MHz always-on
/// timebase; the architected counter runs at a different rate (1 GHz on
/// T8140), so its reading is converted.
const AOP_TIMEBASE_HZ: u64 = 24_000_000;

fn aop_counter_ticks(counter: u64, frequency: u64) -> Result<u64> {
    if frequency == 0 || frequency > u64::from(u32::MAX) {
        return Err(EINVAL);
    }
    // Split whole seconds from the remainder to avoid overflowing a u64
    // multiplication on an otherwise ordinary long-running system.
    let seconds = counter / frequency;
    let remainder = (counter % frequency) * AOP_TIMEBASE_HZ / frequency;
    seconds
        .checked_mul(AOP_TIMEBASE_HZ)
        .and_then(|ticks| ticks.checked_add(remainder))
        .ok_or(EOVERFLOW)
}

fn aop_timestamp() -> Result<u64> {
    // SAFETY: both helpers read the architected timer through the arch timer
    // layer and take no arguments.
    let (counter, rate) = unsafe { (bindings::arch_timer_counter(), bindings::arch_timer_rate()) };
    aop_counter_ticks(counter, u64::from(rate))
}

/// Little-endian field readers for firmware replies and reports.
fn le_u32(buf: &[u8], off: usize) -> Result<u32> {
    let b = buf.get(off..off + 4).ok_or(EINVAL)?;
    Ok(u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
}

fn le_u64(buf: &[u8], off: usize) -> Result<u64> {
    let b = buf.get(off..off + 8).ok_or(EINVAL)?;
    Ok(u64::from_le_bytes([
        b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7],
    ]))
}

#[repr(C, packed(4))]
#[derive(Clone, Copy, Default)]
struct AudioAttachDevice {
    _zero0: u32,
    unk0: u32,
    calltype: u32,
    _zero1: [u8; 16],
    cookie: u32,
    len: u64,
    dev_id: u32,
    _zero2: u32,
}

impl AudioAttachDevice {
    fn new(dev_id: u32) -> AudioAttachDevice {
        AudioAttachDevice {
            unk0: 0xffffffff,
            calltype: CALLTYPE_AUDIO_ATTACH_DEVICE,
            len: 0x2c,
            dev_id,
            ..AudioAttachDevice::default()
        }
    }
}

#[repr(C, packed(4))]
#[derive(Clone, Copy, Default)]
struct AudioGetDeviceProp {
    _zero0: u32,
    unk0: u32,
    calltype: u32,
    _zero1: [u8; 16],
    _pad0: u32,
    len: u64,
    dev_id: u32,
    modifier: u32,
    unk6: u32,
}

impl AudioGetDeviceProp {
    fn new(dev_id: u32, modifier: u32) -> AudioGetDeviceProp {
        AudioGetDeviceProp {
            unk0: 0xffffffff,
            calltype: CALLTYPE_AUDIO_GET_PROP,
            len: 0x30,
            dev_id,
            modifier,
            unk6: 1,
            ..AudioGetDeviceProp::default()
        }
    }
}

#[repr(C, packed(4))]
#[derive(Clone, Copy, Default)]
struct AudioSetDeviceProp<T: Copy + Default> {
    _zero0: u32,
    unk0: u32,
    calltype: u32,
    _zero1: [u8; 16],
    _pad0: u32,
    len: u64,
    dev_id: u32,
    modifier: u32,
    len2: u32,
    data: T,
}

impl<T: Copy + Default> AudioSetDeviceProp<T> {
    fn new(dev_id: u32, modifier: u32, data: T) -> AudioSetDeviceProp<T> {
        AudioSetDeviceProp {
            unk0: 0xffffffff,
            calltype: CALLTYPE_AUDIO_SET_PROP,
            len: mem::size_of::<T>() as u64 + 0x30,
            dev_id,
            modifier,
            len2: mem::size_of::<T>() as u32,
            data,
            ..AudioSetDeviceProp::default()
        }
    }
}

/// Property 300: supported / enabled / history channel masks.
#[repr(C, packed(4))]
#[derive(Clone, Copy, Default)]
struct ChannelControl {
    supported: u32,
    enabled: u32,
    history: u32,
}

/// Property 202, the power request: the device (the FourCC for the output
/// services, 0 for lpai), a per-device request sequence, the request time
/// in the firmware's 24 MHz timebase, the target state FourCC and a
/// parameter (1 for lpai, 0 for the output services).
#[repr(C, packed(4))]
#[derive(Clone, Copy, Default)]
struct PowerRequest {
    dev_id: u32,
    sequence: u32,
    timestamp: u64,
    _zero0: u32,
    state: u32,
    param: u32,
    _zero1: [u8; 20],
}

impl PowerRequest {
    fn new(dev_id: u32, sequence: u32, state: u32, param: u32) -> Result<PowerRequest> {
        Ok(PowerRequest {
            dev_id,
            sequence,
            timestamp: aop_timestamp()?,
            state,
            param,
            ..PowerRequest::default()
        })
    }
}

/// A named power domain of this device (`dev_pm_domain_attach_by_name()`):
/// the virtual device the PM core hands out for one of several domains,
/// runtime-powered around the streams that need it and detached when the
/// driver data goes away.
struct PowerDomain {
    dev: ptr::NonNull<bindings::device>,
    name: &'static str,
}

// SAFETY: the virtual domain device belongs to the PM core, which serializes
// its runtime-PM state itself; this handle only passes the pointer to the
// runtime-PM calls, which may be made from any thread, until `Drop` detaches
// it after the last user is gone.
unsafe impl Send for PowerDomain {}
// SAFETY: as for `Send`; a shared `PowerDomain` exposes nothing but those
// calls.
unsafe impl Sync for PowerDomain {}

impl PowerDomain {
    /// Attaches the domain named `name`; `Ok(None)` when the device has no
    /// domain of that name.
    fn attach(
        dev: &device::Device,
        name: &'static str,
        dt_name: &'static CStr,
    ) -> Result<Option<Self>> {
        // SAFETY: `dev` is live for the call and `dt_name` is NUL-terminated.
        let pd = from_err_ptr(unsafe {
            bindings::dev_pm_domain_attach_by_name(dev.as_raw(), dt_name.as_char_ptr())
        })?;
        Ok(ptr::NonNull::new(pd).map(|dev| PowerDomain { dev, name }))
    }

    /// Takes a runtime-PM reference and waits for the domain to be on.
    ///
    /// The firmware brings the audio fabric back asynchronously after the
    /// service's `pw0 ` request, so the first leaf request under it can miss
    /// the PMGR driver's poll window with the parent still coming up. A
    /// failed resume leaves the virtual domain device in the runtime-PM error
    /// state, where every later request fails with EINVAL; clear that state
    /// and retry a bounded number of times before giving up.
    fn get(&self, attempts: u32) -> Result<()> {
        let mut rc = 0;
        for attempt in 0..attempts {
            // SAFETY: the domain device stays attached for the lifetime of `self`.
            rc = unsafe { bindings::pm_runtime_get_sync(self.dev.as_ptr()) };
            if rc >= 0 {
                return Ok(());
            }
            // SAFETY: as above; a failed resume still took the usage count.
            unsafe { bindings::pm_runtime_put_noidle(self.dev.as_ptr()) };
            // SAFETY: as above; the device is suspended from runtime PM's point
            // of view once the failed resume has been undone. Clearing the
            // error state also lets a later open try again after a final
            // failure here.
            unsafe {
                bindings::__pm_runtime_set_status(
                    self.dev.as_ptr(),
                    bindings::rpm_status_RPM_SUSPENDED as u32,
                )
            };
            if Error::from_errno(rc) != ETIMEDOUT || attempt + 1 == attempts {
                break;
            }
            kernel::time::delay::fsleep(kernel::time::Delta::from_millis(PD_GET_RETRY_MS));
        }
        Err(Error::from_errno(rc))
    }

    /// Drops a reference taken by [`Self::get`].
    fn put(&self) {
        // SAFETY: the domain device stays attached for the lifetime of `self`.
        unsafe { bindings::pm_runtime_put_sync(self.dev.as_ptr()) };
    }
}

impl Drop for PowerDomain {
    fn drop(&mut self) {
        // SAFETY: attached in `attach`; the driver data that owns this handle
        // is dropped only after every stream, and with it every `get`, has
        // been balanced by a `put`.
        unsafe { bindings::dev_pm_domain_detach(self.dev.as_ptr(), true) };
    }
}

/// One firmware audio service driven as an ASoC back-end (or, for the
/// microphone array, by its own PCM): attached once, requested into `pw0 `
/// (idle) at startup and `pwrd` (running) when the stream starts, back to
/// `pw0 ` at shutdown, with its serializer's PMGR leaves runtime-powered for
/// the stream's lifetime.
struct Service {
    dev_id: u32,
    name: &'static str,
    /// Request sequence of the property 202 messages.
    sequence: Atomic<u32>,
    /// In `pwrd`.
    running: AtomicFlag,
}

impl Service {
    const fn new(dev_id: u32, name: &'static str) -> Service {
        Service {
            dev_id,
            name,
            sequence: Atomic::new(0),
            running: AtomicFlag::new(false),
        }
    }
}

/// The low-power microphone stream, guarded by a spinlock that is never taken
/// with the ALSA stream lock held (trigger only touches the atomic run
/// flag), so the report path may call snd_pcm_period_elapsed() while holding
/// it and close cannot free the substream under a report.
struct StreamState {
    substream: *mut bindings::snd_pcm_substream,
    /// absolute producer byte count already copied into the ALSA buffer
    copied: Option<u64>,
    hw_ptr_bytes: usize,
    period_acc: usize,
    buffer_bytes: usize,
}

// SAFETY: the raw substream pointer is only dereferenced under the spinlock
// that holds this state, and only while ALSA keeps the substream alive: open
// sets it and close clears it under the same lock.
unsafe impl Send for StreamState {}

/// The holders of the low-power microphone's `runn` state: its own PCM
/// while prepared, and every service startup that raises the fabric.
struct LpaiRun {
    users: u32,
    /// `runn` is the last state the firmware confirmed.
    running: bool,
}

/// The lid switches seen through the input core.  The internal microphones
/// are disconnected while any of them reads closed.
///
/// Each switch's flag (in its [`LidHandle`]) and the count change together:
/// every writer of one switch runs under that input device's event lock (the
/// event callback, and the resynchronizations in `lid_resync`), or after the
/// device stopped delivering events to it (`lid_disconnect`).  The count is
/// the only state shared between switches, and it is a single atomic word.
struct LidState {
    /// Lid switches currently closed.
    closed: Atomic<u32>,
}

impl LidState {
    const fn new() -> Self {
        LidState {
            closed: Atomic::new(0),
        }
    }

    /// Records that the switch whose state is `switch` now reads `closed`.
    /// Returns whether that changed anything; a repeated state does not.
    fn set(&self, switch: &AtomicFlag, closed: bool) -> bool {
        if switch.xchg(closed, Relaxed) == closed {
            return false;
        }
        if closed {
            self.closed.add(1, Relaxed);
        } else {
            let _ = self.closed.fetch_sub(1, Relaxed);
        }
        true
    }

    fn mic_cut(&self) -> bool {
        self.closed.load(Relaxed) != 0
    }
}

/// The "Internal Mic" jack status for the lid state.
fn mic_jack_status(mic_cut: bool) -> i32 {
    if mic_cut {
        0
    } else {
        bindings::snd_jack_types_SND_JACK_MICROPHONE as i32
    }
}

/// The input handler that follows the lid switches.  Only its address is
/// shared: the input core owns its contents once it is registered.
struct LidHandler(Opaque<bindings::input_handler>);

// SAFETY: the handler is filled in before registration and only the input
// core, which serializes its own access, touches it after that.
unsafe impl Send for LidHandler {}
// SAFETY: as for `Send`.
unsafe impl Sync for LidHandler {}

/// One lid switch the handler is connected to.  `handle` comes first so the
/// input core's handle pointer is also a pointer to this.
#[repr(C)]
struct LidHandle {
    handle: Opaque<bindings::input_handle>,
    /// The switch reads closed, as recorded in [`LidState`].
    closed: AtomicFlag,
    /// Readings are recorded: the handle is open.
    open: AtomicFlag,
}

impl LidHandle {
    /// Records a reading of this switch, once the handle is open.
    ///
    /// `input_register_handle()` calls `lid_start()` before `lid_connect()`
    /// knows whether opening the handle succeeds, and a handle that fails to
    /// open is freed without `lid_disconnect()`: a reading recorded then
    /// could never be taken back.  `lid_connect()` reads the switch again
    /// once the handle is open.
    fn record(&self, lid: &LidState, closed: bool) -> bool {
        self.open.load(Relaxed) && lid.set(&self.closed, closed)
    }

    /// Records the switch as read under its device's event lock.  With
    /// `activate`, readings are enabled first: `lid_connect()` enables and
    /// reads the switch in one step after opening the handle, so an event
    /// delivered between the open and this reading, which `record()` ignored,
    /// is caught up with.
    fn sync(&self, lid: &LidState, closed: bool, activate: bool) -> bool {
        if activate {
            self.open.store(true, Relaxed);
        }
        self.record(lid, closed)
    }

    /// The switch goes away: it no longer disconnects the microphones.
    fn forget(&self, lid: &LidState) -> bool {
        self.open.store(false, Relaxed);
        lid.set(&self.closed, false)
    }
}

#[pin_data]
struct SndSocT8140AopData {
    dev: ARef<device::Device>,
    adata: Arc<dyn AOP>,
    service: EPICService,
    /// The low-power microphone's source ring; `None` on machines without lpai.
    ring: Option<Arc<SourceRing>>,
    /// The machine has the lpai microphone, whose run state also raises the
    /// gated audio fabric, and PMGR serializer leaves the AP must power
    /// (T8140). On T8152 neither exists: the firmware powers its own fabric
    /// and the serializers, and lpai, hpai and tap attach with
    /// kIOReturnNoDevice.
    has_lpai: bool,
    ring_bytes: Atomic<u32>,
    sequence: Atomic<u32>,
    /// The low-power microphone PCM holds a reference on `lpai_run`.
    powered: AtomicFlag,
    /// A low-power microphone report arrived since the last run request.
    reported: AtomicFlag,
    /// The low-power microphone stream is triggered (set and cleared under
    /// the ALSA stream lock, read by the report path).
    running: AtomicFlag,
    /// The low-power microphone's hardware pointer.  Stored with release
    /// after the frames before it are in the ALSA buffer, loaded with acquire
    /// by the .pointer callback, which ALSA calls without the stream state's
    /// lock (a status or HWSYNC on another CPU).
    hw_ptr_frames: Atomic<u32>,
    /// One warning per producer-report fault class and driver instance.
    report_warnings: [AtomicFlag; 5],
    /// The back-end services: the jack's output and headset microphone,
    /// the speakers and their tap, and the microphone array.
    cout: Service,
    cin: Service,
    spkr: Service,
    tap: Service,
    hpai: Service,
    /// The serializer leaves, runtime-powered around streams: audio_mca1_m
    /// (jack), audio_mca0_m (tap and the LEAP's serializer), audio_leap_tx0
    /// and audio_leap_mca (speakers), audio_leap_rx0 (microphone array).
    pd_mca1: Option<PowerDomain>,
    pd_mca0: Option<PowerDomain>,
    pd_tx0: Option<PowerDomain>,
    pd_leap_mca: Option<PowerDomain>,
    pd_rx0: Option<PowerDomain>,
    /// A speaker stream has been triggered and not yet stopped.
    spkr_want_run: AtomicFlag,
    /// A microphone-array stream has been triggered and not yet stopped.
    hpai_want_run: AtomicFlag,
    /// "Speaker Playback Volume" (REG_SPKR_VOLUME), 0..=SPKR_VOLUME_MAX.
    spkr_volume: Atomic<u32>,
    /// The sleeping firmware starts queued from the atomic triggers.
    #[pin]
    spkr_start_work: Work<Self, SPKR_START_WORK>,
    #[pin]
    hpai_start_work: Work<Self, HPAI_START_WORK>,
    /// The lid switches, and the card's "Internal Mic" jack that follows
    /// them (null outside the card's lifetime), reported from `jack_work`
    /// because the input core delivers switch events in atomic context.
    lid: LidState,
    lid_handler: LidHandler,
    mic_jack: Atomic<*mut bindings::snd_jack>,
    #[pin]
    jack_work: Work<Self, JACK_WORK>,
    #[pin]
    stream: SpinLock<StreamState>,
    /// Every lpai power request goes through here, see
    /// [`SndSocT8140AopData::lpai_run_get`].
    #[pin]
    lpai_run: Mutex<LpaiRun>,
    /// The T8152 speaker's thermal limiter, see [`spk_thermal`].
    #[pin]
    spkr_thermal: SpinLock<SpkThermal>,
}

const SPKR_START_WORK: u64 = 0;
const HPAI_START_WORK: u64 = 1;
const JACK_WORK: u64 = 2;

impl_has_work! {
    impl HasWork<Self, SPKR_START_WORK> for SndSocT8140AopData { self.spkr_start_work }
    impl HasWork<Self, HPAI_START_WORK> for SndSocT8140AopData { self.hpai_start_work }
    impl HasWork<Self, JACK_WORK> for SndSocT8140AopData { self.jack_work }
}

/// The "Internal Mic" jack report after a lid change: it takes mutexes.
impl WorkItem<JACK_WORK> for SndSocT8140AopData {
    type Pointer = Arc<Self>;

    fn run(this: Arc<Self>) {
        this.report_mic_jack();
    }
}

/// The speaker stream start (pwrd + unmute), queued by the back-end DAI's
/// trigger START: those are sleeping firmware calls, and the DMA must already
/// be running when the LEAP starts consuming.
impl WorkItem<SPKR_START_WORK> for SndSocT8140AopData {
    type Pointer = Arc<Self>;

    fn run(this: Arc<Self>) {
        if let Err(e) = this.spkr_go() {
            dev_err!(this.dev, "spkr: start failed: {:?}\n", e);
        }
    }
}

/// The microphone-array stream start (`pwrd`), queued by its PCM trigger
/// START for the same reason as the speaker's: the ADMAC must already be
/// running before the LEAP starts producing.
impl WorkItem<HPAI_START_WORK> for SndSocT8140AopData {
    type Pointer = Arc<Self>;

    fn run(this: Arc<Self>) {
        if let Err(e) = this.hpai_go() {
            dev_err!(this.dev, "hpai: start failed: {:?}\n", e);
        }
    }
}

impl SndSocT8140AopData {
    fn epic_wrapped_call<T>(&self, data: &T) -> Result<u32> {
        // SAFETY: T is a plain packed firmware record.
        let msg_bytes =
            unsafe { slice::from_raw_parts(ptr::from_ref(data).cast::<u8>(), mem::size_of::<T>()) };
        self.adata
            .epic_call(&self.service, EPIC_SUBTYPE_WRAPPED_CALL, msg_bytes)
    }

    fn epic_wrapped_call_ret<T>(&self, data: &T, ret_len: usize) -> Result<KVec<u8>> {
        // SAFETY: T is a plain packed firmware record.
        let msg_bytes =
            unsafe { slice::from_raw_parts(ptr::from_ref(data).cast::<u8>(), mem::size_of::<T>()) };
        let (retcode, ret) = self.adata.epic_call_ret(
            &self.service,
            EPIC_SUBTYPE_WRAPPED_CALL,
            msg_bytes,
            ret_len,
        )?;
        if retcode != 0 {
            dev_err!(
                self.dev,
                "audio wrapped call failed, return code {:#x}\n",
                retcode
            );
            return Err(EIO);
        }
        Ok(ret)
    }

    /// Attach an audio device.  The firmware keeps attachments for its
    /// lifetime (there is no detach in the protocol we know), so a reload of
    /// this module sees kIOReturnBusy for a device it attached earlier.
    fn attach_device(&self, dev_id: u32, name: &str) -> Result<()> {
        let ret = self.epic_wrapped_call(&AudioAttachDevice::new(dev_id))?;
        match ret {
            0 | AUDIO_RET_BUSY => Ok(()),
            _ => {
                dev_err!(
                    self.dev,
                    "unable to attach {}, return code {:#x}\n",
                    name,
                    ret
                );
                Err(EIO)
            }
        }
    }

    fn set_channel_control(&self) -> Result<()> {
        let ctl = ChannelControl {
            supported: 3,
            enabled: 3,
            history: 3,
        };
        let ret = self.epic_wrapped_call(&AudioSetDeviceProp::new(
            AUDIO_DEV_LPAI,
            PROP_CHANNEL_CONTROL,
            ctl,
        ))?;
        if ret != 0 {
            dev_err!(
                self.dev,
                "unable to set lpai channels, return code {:#x}\n",
                ret
            );
            return Err(EIO);
        }
        Ok(())
    }

    /// Reply layout after the retcode: u32 length, then the property bytes.
    fn get_prop(&self, modifier: u32, len: usize) -> Result<KVec<u8>> {
        let ret = self
            .epic_wrapped_call_ret(&AudioGetDeviceProp::new(AUDIO_DEV_LPAI, modifier), 4 + len)?;
        if ret.len() < 4 + len {
            dev_err!(
                self.dev,
                "lpai property {} reply too short ({})\n",
                modifier,
                ret.len()
            );
            return Err(EIO);
        }
        let reported = le_u32(&ret, 0)? as usize;
        if reported != len {
            dev_err!(
                self.dev,
                "lpai property {} length {} (expected {})\n",
                modifier,
                reported,
                len
            );
            return Err(EIO);
        }
        Ok(ret)
    }

    fn read_geometry(&self) -> Result<()> {
        // 302: FourCC/pad, ring frames u64, report frames u64
        let geo = self.get_prop(PROP_RING_GEOMETRY, 24)?;
        let frames = le_u64(&geo, 12)?;
        let quantum = le_u64(&geo, 20)?;
        if frames == 0
            || frames > (RING_BYTES / LPAI_FRAME_BYTES) as u64
            || quantum == 0
            || quantum > frames
        {
            dev_err!(
                self.dev,
                "invalid lpai ring geometry: {} frames, {} per report\n",
                frames,
                quantum
            );
            return Err(EIO);
        }
        self.ring_bytes
            .store((frames as usize * LPAI_FRAME_BYTES) as u32, Relaxed);
        // 301: stream format, sample rate at +4
        let fmt = self.get_prop(PROP_STREAM_FORMAT, 16)?;
        let rate = le_u32(&fmt, 8)?;
        if rate != LPAI_RATE {
            dev_err!(self.dev, "unexpected lpai sample rate {}\n", rate);
            return Err(EIO);
        }
        dev_dbg!(
            self.dev,
            "lpai: {} Hz, ring {} frames, {} frames per report\n",
            rate,
            frames,
            quantum
        );
        Ok(())
    }

    fn set_power(&self, state: u32) -> Result<()> {
        let sequence = self.sequence.fetch_add(1, Relaxed).wrapping_add(1);
        let req = PowerRequest::new(0, sequence, state, 1)?;
        let ret = self.epic_wrapped_call(&AudioSetDeviceProp::new(
            AUDIO_DEV_LPAI,
            PROP_POWER_REQUEST,
            req,
        ))?;
        if ret != 0 {
            dev_err!(
                self.dev,
                "lpai power request {:#x} rejected: {:#x}\n",
                state,
                ret
            );
            return Err(EIO);
        }
        let st = self.get_prop(PROP_POWER_STATE, 4)?;
        let now = le_u32(&st, 4)?;
        if now != state {
            dev_err!(
                self.dev,
                "lpai power state {:#x} after requesting {:#x}\n",
                now,
                state
            );
            return Err(EIO);
        }
        Ok(())
    }

    /// Takes a reference on the low-power microphone's `runn` state,
    /// requesting it for the first holder.
    ///
    /// The PCM and the fabric raises share the stream, and every power
    /// request is checked against the state the firmware reports back: a
    /// raise that returns lpai to `idle` while another service starts up
    /// makes that one read `idle` after requesting `runn`, and idles a
    /// capture the PCM just started.  So they count their holds here and
    /// only the first and last holder change the state.
    fn lpai_run_get(&self) -> Result<()> {
        let mut run = self.lpai_run.lock();
        if !run.running {
            self.reported.store(false, Relaxed);
            self.set_power(POWER_STATE_RUN)?;
            run.running = true;
        }
        run.users += 1;
        Ok(())
    }

    /// Drops a reference taken by [`Self::lpai_run_get`], returning lpai to
    /// `idle` after the last holder.  If that fails, the next holder finds
    /// it still running.
    fn lpai_run_put(&self) -> Result<()> {
        let mut run = self.lpai_run.lock();
        run.users -= 1;
        if run.users == 0 && run.running {
            self.set_power(POWER_STATE_IDLE)?;
            run.running = false;
        }
        Ok(())
    }

    // ---- the back-end services ----

    /// Property 202 with the service's sequence and a 24 MHz timestamp;
    /// `pw0 ` is confirmed through property 200 like the native host does.
    fn service_set_power(&self, svc: &Service, state: u32) -> Result<()> {
        let sequence = svc.sequence.fetch_add(1, Relaxed).wrapping_add(1);
        let req = PowerRequest::new(svc.dev_id, sequence, state, 0)?;
        let ret = self.epic_wrapped_call(&AudioSetDeviceProp::new(
            svc.dev_id,
            PROP_POWER_REQUEST,
            req,
        ))?;
        if ret != 0 {
            dev_err!(
                self.dev,
                "{} power request {:#x} rejected: {:#x}\n",
                svc.name,
                state,
                ret
            );
            return Err(EIO);
        }
        if state == SPKR_POWER_STATE_PW0 {
            let ret = self
                .epic_wrapped_call_ret(&AudioGetDeviceProp::new(svc.dev_id, PROP_POWER_STATE), 8)?;
            let now = le_u32(&ret, 4)?;
            if now != state {
                dev_err!(
                    self.dev,
                    "{} power state {:#x} after requesting {:#x}\n",
                    svc.name,
                    now,
                    state
                );
                return Err(EIO);
            }
        }
        Ok(())
    }

    /// Runtime-power serializer leaves, parents first.  A leaf that fails to
    /// power up fails the stream; recovering a domain from that state is the
    /// power-domain driver's business, not a consumer's.
    fn power_up(&self, name: &str, pds: &[&Option<PowerDomain>], attempts: u32) -> Result<()> {
        if !self.has_lpai {
            return Ok(());
        }
        for (i, slot) in pds.iter().enumerate() {
            let Some(pd) = slot else {
                dev_err!(self.dev, "{}: power domain not attached\n", name);
                self.power_down(&pds[..i]);
                return Err(ENODEV);
            };
            if let Err(e) = pd.get(attempts) {
                if e != ETIMEDOUT || attempts > PD_GET_ATTEMPTS {
                    dev_err!(
                        self.dev,
                        "{}: power domain {} did not power up: {:?}\n",
                        name,
                        pd.name,
                        e
                    );
                }
                self.power_down(&pds[..i]);
                return Err(e);
            }
        }
        Ok(())
    }

    /// Release leaves powered by [`Self::power_up`], children first.
    fn power_down(&self, pds: &[&Option<PowerDomain>]) {
        for pd in pds.iter().rev().filter_map(|pd| pd.as_ref()) {
            pd.put();
        }
    }

    /// Back-end startup: attach the service, `pw0 `, then power its leaves.
    ///
    /// The firmware owns the audio fabric and power-gates it (`audio_p` and
    /// the LEAP domains under it) whenever none of its services is held in
    /// `pw0 ` or `pwrd`, for instance after a low-power microphone capture.
    /// Requesting `pw0 ` first brings the fabric back; only then can the
    /// serializer leaves under it be powered from the AP side, otherwise the
    /// PMGR reports the parent off and the leaf request never completes.
    fn service_startup(&self, svc: &Service, pds: &[&Option<PowerDomain>]) -> Result<()> {
        self.attach_device(svc.dev_id, svc.name)?;
        self.service_set_power(svc, SPKR_POWER_STATE_PW0)?;
        if !self.has_lpai {
            self.log_service_props(svc, "pw0");
        }
        self.power_up_raising(svc.name, pds)?;
        svc.running.store(false, Relaxed);
        Ok(())
    }

    /// T8152 bring-up diagnostics: log what the firmware reports for a
    /// service (properties 200, 300, 301 and 302) so that the stream format
    /// and the serializer state can be read from the log. Read-only.
    fn log_service_props(&self, svc: &Service, when: &str) {
        for modifier in [
            PROP_POWER_STATE,
            PROP_CHANNEL_CONTROL,
            PROP_STREAM_FORMAT,
            PROP_RING_GEOMETRY,
        ] {
            let msg = AudioGetDeviceProp::new(svc.dev_id, modifier);
            // SAFETY: the message is a plain packed firmware record.
            let bytes = unsafe {
                slice::from_raw_parts(
                    ptr::from_ref(&msg).cast::<u8>(),
                    mem::size_of::<AudioGetDeviceProp>(),
                )
            };
            match self
                .adata
                .epic_call_ret(&self.service, EPIC_SUBTYPE_WRAPPED_CALL, bytes, 64)
            {
                Ok((ret, data)) => dev_dbg!(
                    self.dev,
                    "{} {}: property {} retcode {:#x} data {:02x?}\n",
                    svc.name,
                    when,
                    modifier,
                    ret,
                    &data[..]
                ),
                Err(e) => dev_dbg!(
                    self.dev,
                    "{} {}: property {} call failed: {:?}\n",
                    svc.name,
                    when,
                    modifier,
                    e
                ),
            }
        }
    }

    /// [`Self::power_up`] with the fabric raised first.
    ///
    /// The fabric may be gated at any open (a low-power capture is enough,
    /// and a service left in `pw0 ` after its stream does not keep it up),
    /// and a blind leaf request against it fails noisily in the PMGR
    /// driver, so always go through the raised path; with the fabric up it
    /// costs one brief low-power microphone run.
    fn power_up_raising(&self, name: &str, pds: &[&Option<PowerDomain>]) -> Result<()> {
        self.with_fabric_raised(|| self.power_up(name, pds, PD_GET_RAISED_ATTEMPTS))
    }

    /// Raise the audio fabric the firmware gated. A service's `pw0 ` request
    /// does not do that (measured: `audio_p` stays gated for the whole leaf
    /// retry window), only a running stream does, after which a service in
    /// `pw0 ` keeps it up only while its stream runs. The low-power
    /// microphone is attached for the firmware's lifetime, so hold it in
    /// `runn` around `f`, once its first report has arrived.
    fn with_fabric_raised<R>(&self, f: impl FnOnce() -> Result<R>) -> Result<R> {
        if !self.has_lpai {
            return f();
        }
        self.lpai_run_get()?;
        let mut polls = 0;
        while !self.reported.load(Relaxed) && polls < FABRIC_RAISE_POLLS {
            kernel::time::delay::fsleep(kernel::time::Delta::from_millis(FABRIC_RAISE_POLL_MS));
            polls += 1;
        }
        if polls == FABRIC_RAISE_POLLS {
            dev_warn!(self.dev, "lpai: no report while raising the fabric\n");
        }
        let ret = f();
        if let Err(e) = self.lpai_run_put() {
            dev_warn!(
                self.dev,
                "lpai: unable to idle after raising the fabric: {:?}\n",
                e
            );
        }
        ret
    }

    /// `pwrd`: the firmware starts the serializer clocks and the stream.
    fn service_run(&self, svc: &Service) -> Result<()> {
        if svc.running.load(Relaxed) {
            return Ok(());
        }
        self.service_set_power(svc, SPKR_POWER_STATE_PWRD)?;
        svc.running.store(true, Relaxed);
        if !self.has_lpai {
            self.log_service_props(svc, "pwrd");
        }
        Ok(())
    }

    /// Back-end shutdown: `pw0 ` if running, then the leaves.
    fn service_shutdown(&self, svc: &Service, pds: &[&Option<PowerDomain>]) {
        if svc.running.load(Relaxed) {
            if let Err(e) = self.service_set_power(svc, SPKR_POWER_STATE_PW0) {
                dev_err!(self.dev, "{}: pw0 failed: {:?}\n", svc.name, e);
            }
            svc.running.store(false, Relaxed);
        }
        self.power_down(pds);
    }

    // ---- 3.5 mm jack: cout (MCA1, base-ns TX2) and cin (the headset
    // microphone, base-ns RX2), one MCA1 power reference per open direction ----

    fn jack_service(&self, playback: bool) -> &Service {
        if playback {
            &self.cout
        } else {
            &self.cin
        }
    }

    fn jack_startup(&self, playback: bool) -> Result<()> {
        self.service_startup(self.jack_service(playback), &[&self.pd_mca1])
    }

    /// pwrd starts the MCA1 clocks; the codec's PLL locks to them when ASoC
    /// unmutes it right after this.
    fn jack_prepare(&self, playback: bool) -> Result<()> {
        self.service_run(self.jack_service(playback))
    }

    fn jack_shutdown(&self, playback: bool) {
        self.service_shutdown(self.jack_service(playback), &[&self.pd_mca1])
    }

    // ---- speaker tap: the native SpeakerTap, base-ns RX1 under the tap
    // service on MCA0.  The words the amplifier receives come back on this
    // stream (the card's "Speaker Sense" capture): it observes the serial
    // data after the copy-time gain, it senses no current or voltage. ----

    fn sense_startup(&self) -> Result<()> {
        if !self.has_lpai {
            return Ok(());
        }
        self.service_startup(&self.tap, &[&self.pd_mca0])
    }

    fn sense_prepare(&self) -> Result<()> {
        if !self.has_lpai {
            return Ok(());
        }
        self.service_run(&self.tap)
    }

    fn sense_shutdown(&self) {
        if !self.has_lpai {
            return;
        }
        self.service_shutdown(&self.tap, &[&self.pd_mca0])
    }

    // ---- the high-quality microphone array: the hpai service feeding
    // leap-s ADMAC RX0.  The host holds its RX0 leaf and the shared LEAP MCA
    // domain while the firmware service owns the microphone fabric. ----

    /// The array's LEAP leaves: its own RX0 channel leaf, plus the LEAP MCA
    /// the speaker path also holds.
    fn hpai_domains(&self) -> [&Option<PowerDomain>; 2] {
        [&self.pd_rx0, &self.pd_leap_mca]
    }

    fn hpai_startup(&self) -> Result<()> {
        self.service_startup(&self.hpai, &self.hpai_domains())
    }

    /// The array's start (`pwrd`), from the system workqueue right after
    /// trigger START.  Started earlier, from prepare, the LEAP produces into
    /// a ring the ADMAC is not servicing yet and the capture is noise from
    /// then on; started here, the DMA is already running.
    fn hpai_go(&self) -> Result<()> {
        if !self.hpai_want_run.load(Relaxed) || self.hpai.running.load(Relaxed) {
            return Ok(());
        }
        self.service_run(&self.hpai)
    }

    /// Called from sleeping close paths: a queued start must finish before
    /// the service is retired.
    fn flush_hpai_start(&self) {
        self.hpai_want_run.store(false, Relaxed);
        // SAFETY: the work item lives in `self` for the driver's lifetime;
        // flushing waits for a queued `hpai_go` to finish.
        unsafe { bindings::flush_work(Work::raw_get(&self.hpai_start_work)) };
    }

    fn hpai_shutdown(&self) {
        self.flush_hpai_start();
        self.service_shutdown(&self.hpai, &self.hpai_domains())
    }

    // ---- the lid: both microphones are disconnected in hardware while it
    // is closed ----

    /// A lid switch now reads `closed`: queue the jack report on a change.
    fn lid_set(self: ArcBorrow<'_, Self>, switch: &LidHandle, closed: bool) {
        if switch.record(&self.lid, closed) {
            self.queue_jack_report();
        }
    }

    /// A lid switch went away.
    fn lid_forget(self: ArcBorrow<'_, Self>, switch: &LidHandle) {
        if switch.forget(&self.lid) {
            self.queue_jack_report();
        }
    }

    /// The jack work is the only reporter, so reports reach ALSA in the order
    /// of the states they read: a queued or running report that read an
    /// older state is followed by another run.
    fn queue_jack_report(self: ArcBorrow<'_, Self>) {
        // Err = already queued: the pending report reads the new state.
        let _ = workqueue::system_dfl().enqueue::<_, JACK_WORK>(Arc::from(self));
    }

    fn report_mic_jack(&self) {
        let jack = self.mic_jack.load(Relaxed);
        if !jack.is_null() {
            // SAFETY: the jack belongs to the card, which teardown frees only
            // after clearing `mic_jack` and flushing this work.
            unsafe { bindings::snd_jack_report(jack, mic_jack_status(self.lid.mic_cut())) };
        }
    }

    fn flush_jack_report(&self) {
        // SAFETY: the work item lives in `self` for the driver's lifetime;
        // flushing waits for a queued report to finish.
        unsafe { bindings::flush_work(Work::raw_get(&self.jack_work)) };
    }

    // ---- internal speakers: the spkr service on LEAP TX0 (leap-ns ADMAC
    // channel 0); the leaves are tx0 (-> leap_c -> leap_a -> audio_fr ->
    // audio_a), leap_mca (-> audio_p) and the MCA0 serializer the LEAP
    // drives the amplifiers through -- shared with the tap capture, so
    // both hold it and neither can power it off under the other ----

    /// Output mute (property 700) is independent of the amplifier's SD_MODE;
    /// the amplifier stays off across a failure here.
    fn spkr_set_mute(&self, muted: bool) -> Result<()> {
        if !self.has_lpai {
            // T8152: the firmware answers every property above 302 with
            // 0xe00002f0 (unsupported), the mute request included (measured on
            // an M6 Mac mini); the amplifier is muted through its own codec.
            dev_dbg!(
                self.dev,
                "speaker mute {} not requested on this firmware\n",
                muted
            );
            return Ok(());
        }
        let ret = self.epic_wrapped_call(&AudioSetDeviceProp::new(
            AUDIO_DEV_SPKR,
            PROP_OUTPUT_MUTE,
            u32::from(muted),
        ))?;
        if ret != 0 {
            dev_err!(self.dev, "speaker mute request rejected: {:#x}\n", ret);
            return Err(EIO);
        }
        Ok(())
    }

    fn spkr_domains(&self) -> [&Option<PowerDomain>; 3] {
        [&self.pd_tx0, &self.pd_leap_mca, &self.pd_mca0]
    }

    fn spkr_startup(&self) -> Result<()> {
        self.service_startup(&self.spkr, &self.spkr_domains())
    }

    /// A PCM underrun stops the DMA without closing the back-end, so the
    /// service may still be in `pwrd` when the stream is prepared again.
    /// Retire that run here so the next trigger queues a fresh `pwrd` and
    /// unmute once the DMA has restarted.
    fn spkr_prepare(&self) -> Result<()> {
        self.flush_spkr_start();
        if !self.spkr.running.load(Relaxed) {
            return Ok(());
        }
        self.spkr_set_mute(true)?;
        self.service_set_power(&self.spkr, SPKR_POWER_STATE_PW0)?;
        self.spkr.running.store(false, Relaxed);
        Ok(())
    }

    /// Speaker stream start, from the system workqueue right after trigger
    /// START: the DMA must be running before pwrd starts the LEAP consuming
    /// (issued from prepare, before the DMA, the wire stays silent), then the
    /// firmware unmute, the value observed on the bus once the amplifier
    /// output is enabled.
    fn spkr_go(&self) -> Result<()> {
        if !self.spkr_want_run.load(Relaxed) || self.spkr.running.load(Relaxed) {
            return Ok(());
        }
        self.service_run(&self.spkr)?;
        if let Err(e) = self.spkr_set_mute(false) {
            let _ = self.service_set_power(&self.spkr, SPKR_POWER_STATE_PW0);
            self.spkr.running.store(false, Relaxed);
            return Err(e);
        }
        Ok(())
    }

    /// Called from sleeping prepare/shutdown paths while ALSA serializes
    /// this stream: a queued start must finish before the mute and pw0.
    fn flush_spkr_start(&self) {
        self.spkr_want_run.store(false, Relaxed);
        // SAFETY: the work item lives in `self` for the driver's lifetime;
        // flushing waits for a queued `spkr_go` to finish.
        unsafe { bindings::flush_work(Work::raw_get(&self.spkr_start_work)) };
    }

    /// Speaker back-end shutdown: mute, then the service; the amplifier is
    /// already off (codec trigger STOP) and a queued start was flushed.
    fn spkr_shutdown(&self) {
        self.flush_spkr_start();
        if self.spkr.running.load(Relaxed) {
            if let Err(e) = self.spkr_set_mute(true) {
                dev_err!(self.dev, "spkr: mute failed: {:?}\n", e);
            }
        }
        self.service_shutdown(&self.spkr, &self.spkr_domains());
    }
}

/// Copy `len` bytes from the source ring (absolute producer offset `from`)
/// into the ALSA buffer at `hw_ptr`, both wrapping.  Only spans the firmware
/// has announced through a producer report are copied, so the producer is
/// past them; `ring_bytes` is the geometry the firmware reported and never
/// exceeds the ring, so `read()` cannot fail on range.
fn copy_span(
    ring: &SourceRing,
    dma_area: *mut u8,
    buffer_bytes: usize,
    ring_bytes: usize,
    from: u64,
    len: usize,
    hw_ptr: usize,
) -> Result<usize> {
    let mut src = (from % ring_bytes as u64) as usize;
    let mut dst = hw_ptr;
    let mut left = len;
    while left > 0 {
        let n = left.min(ring_bytes - src).min(buffer_bytes - dst);
        // SAFETY: dst + n <= buffer_bytes, and the ALSA buffer stays mapped
        // while the substream is set in the stream state, whose lock the
        // caller holds.
        let span = unsafe { slice::from_raw_parts_mut(dma_area.add(dst), n) };
        ring.read(src, span)?;
        src = (src + n) % ring_bytes;
        dst = (dst + n) % buffer_bytes;
        left -= n;
    }
    Ok(dst)
}

#[derive(Debug, PartialEq, Eq)]
enum ProducerReportError {
    Short,
    Overflow,
}

fn producer_position(report: &[u8]) -> core::result::Result<(u64, u64), ProducerReportError> {
    if report.len() < REPORT_MIN_LEN {
        return Err(ProducerReportError::Short);
    }
    let count = le_u64(report, 0x40).map_err(|_| ProducerReportError::Short)?;
    let absolute = count
        .checked_add(1)
        .and_then(|frames| frames.checked_mul(LPAI_FRAME_BYTES as u64))
        .ok_or(ProducerReportError::Overflow)?;
    let cursor = le_u64(report, 0x60).map_err(|_| ProducerReportError::Short)?;
    Ok((absolute, cursor))
}

fn producer_cursor_matches(absolute: u64, cursor: u64, ring_bytes: usize) -> bool {
    ring_bytes != 0 && cursor == absolute % ring_bytes as u64
}

impl ReportListener for SndSocT8140AopData {
    fn process_report(&self, _subtype: u16, report: &[u8]) -> Result<()> {
        let (absolute, cursor) = match producer_position(report) {
            Ok(position) => position,
            Err(ProducerReportError::Short) => {
                if !self.report_warnings[0].xchg(true, Relaxed) {
                    dev_warn!(
                        self.dev,
                        "short lpai producer report ({} bytes)\n",
                        report.len()
                    );
                }
                return Ok(());
            }
            Err(ProducerReportError::Overflow) => {
                if !self.report_warnings[1].xchg(true, Relaxed) {
                    dev_warn!(self.dev, "overflowing lpai producer counter\n");
                }
                return Ok(());
            }
        };
        let frames = absolute / LPAI_FRAME_BYTES as u64;
        self.reported.store(true, Relaxed);
        let ring_bytes = self.ring_bytes.load(Relaxed) as usize;
        if !producer_cursor_matches(absolute, cursor, ring_bytes) {
            if !self.report_warnings[2].xchg(true, Relaxed) {
                dev_warn!(
                    self.dev,
                    "inconsistent lpai report: frames {} cursor {:#x}\n",
                    frames,
                    cursor
                );
            }
            return Ok(());
        }

        let mut st = self.stream.lock();
        if !self.running.load(Relaxed) || st.substream.is_null() {
            return Ok(());
        }
        let copied = match st.copied {
            Some(c) => c,
            None => {
                // start at a completed producer position, not at ring zero
                st.copied = Some(absolute);
                return Ok(());
            }
        };
        if absolute < copied {
            if !self.report_warnings[3].xchg(true, Relaxed) {
                dev_warn!(self.dev, "lpai producer moved backwards\n");
            }
            st.copied = Some(absolute);
            return Ok(());
        }
        let delta = (absolute - copied) as usize;
        if delta >= ring_bytes {
            st.copied = Some(absolute);
            if !self.report_warnings[4].xchg(true, Relaxed) {
                dev_warn!(self.dev, "lpai source overrun ({} bytes)\n", delta);
            }
            return Ok(());
        }
        if delta == 0 {
            return Ok(());
        }
        // SAFETY: the substream is registered in the stream state, which
        // close clears under this lock, so it is live; its runtime and DMA
        // buffer exist between hw_params and hw_free, which bracket prepare
        // (where `buffer_bytes` was taken) and every triggered state.
        let dma_area = unsafe { (*(*st.substream).runtime).dma_area };
        let buffer_bytes = st.buffer_bytes;
        let new_ptr = copy_span(
            self.ring.as_ref().ok_or(ENODEV)?,
            dma_area,
            buffer_bytes,
            ring_bytes,
            copied,
            delta,
            st.hw_ptr_bytes,
        )?;
        st.hw_ptr_bytes = new_ptr;
        st.copied = Some(absolute);
        st.period_acc += delta;
        self.hw_ptr_frames
            .store((new_ptr / LPAI_FRAME_BYTES) as u32, Release);
        if st.period_acc >= LPAI_PERIOD_BYTES {
            st.period_acc %= LPAI_PERIOD_BYTES;
            // SAFETY: the substream is valid while it is registered in
            // the stream state, which close clears under this lock.
            unsafe { bindings::snd_pcm_period_elapsed(st.substream) };
        }
        Ok(())
    }
}

// ---- the lid switch handler: an input handler on every device with
// SW_LID, observing the events of the lid switch userspace already
// listens to ----

/// Devices with a lid switch, and the terminating entry.
static LID_IDS: [bindings::input_device_id; 2] = {
    // SAFETY: the ID is plain integers, for which zero is valid.
    let mut lid: bindings::input_device_id = unsafe { mem::zeroed() };
    lid.flags = (bindings::INPUT_DEVICE_ID_MATCH_EVBIT | bindings::INPUT_DEVICE_ID_MATCH_SWBIT)
        as bindings::kernel_ulong_t;
    lid.evbit[0] = 1 << bindings::EV_SW;
    lid.swbit[0] = 1 << bindings::SW_LID;
    // SAFETY: as above.
    [lid, unsafe { mem::zeroed() }]
};

/// The driver data behind an input handle of the lid handler.
///
/// # Safety
///
/// `handle` must be a handle `lid_connect` registered and the handler must
/// still be registered, so the `Arc` leaked into its private data is live.
unsafe fn lid_data<'a>(handle: *mut bindings::input_handle) -> ArcBorrow<'a, SndSocT8140AopData> {
    // SAFETY: by the caller's contract.
    unsafe { Arc::<SndSocT8140AopData>::borrow((*(*handle).handler).private.cast()) }
}

/// Reads the switch from the device's state and records it, under the
/// device's event lock: events are delivered under that lock after the input
/// core updated the state, so no event can be overtaken by an older reading.
/// `activate` enables readings first (see [`LidHandle::sync`]).
///
/// # Safety
///
/// `handle` must be a handle `lid_connect` registered, with the handler
/// still registered.
unsafe fn lid_resync(handle: *mut bindings::input_handle, activate: bool) {
    // SAFETY: the handle and its device are live while it is registered.
    let dev = unsafe { (*handle).dev };
    // SAFETY: `dev` is live; its event lock is a spinlock taken with
    // interrupts saved, as the input core takes it.
    let lock = unsafe { &raw mut (*dev).event_lock };
    // SAFETY: as above.
    let flags = unsafe { bindings::spin_lock_irqsave(lock) };
    // SAFETY: the switch bitmap is written under the event lock, held here.
    let closed = unsafe { (*dev).sw[0] } & (1 << bindings::SW_LID) != 0;
    // SAFETY: the handle is the first field of a `LidHandle`, live while it
    // is registered, and the handler is registered.
    let (lid, data) = unsafe { (&*handle.cast::<LidHandle>(), lid_data(handle)) };
    if lid.sync(&data.lid, closed, activate) {
        data.queue_jack_report();
    }
    // SAFETY: taken above with these flags.
    unsafe { bindings::spin_unlock_irqrestore(lock, flags) };
}

/// Called after the handle is registered, and again when a client that
/// grabbed the device releases it: events delivered only to the grabbing
/// client in between never reached `lid_event`.
///
/// # Safety
///
/// Called by the input core with a handle `lid_connect` registered.
unsafe extern "C" fn lid_start(handle: *mut bindings::input_handle) {
    // SAFETY: by the caller's contract; the input core calls this without
    // the event lock.
    unsafe { lid_resync(handle, false) };
}

/// # Safety
///
/// Called by the input core, in atomic context with the device's event lock
/// held, with a handle `lid_connect` registered.
unsafe extern "C" fn lid_event(
    handle: *mut bindings::input_handle,
    type_: u32,
    code: u32,
    value: i32,
) {
    if type_ != bindings::EV_SW || code != bindings::SW_LID {
        return;
    }
    // SAFETY: the handle is the first field of a `LidHandle`, live until
    // `lid_disconnect`, and the handler is registered.
    let (lid, data) = unsafe { (&*handle.cast::<LidHandle>(), lid_data(handle)) };
    data.lid_set(lid, value != 0);
}

/// # Safety
///
/// Called by the input core with the registered lid handler and a device
/// that matched `LID_IDS`.
unsafe extern "C" fn lid_connect(
    handler: *mut bindings::input_handler,
    dev: *mut bindings::input_dev,
    _id: *const bindings::input_device_id,
) -> i32 {
    let lid = match KBox::new(
        LidHandle {
            handle: Opaque::zeroed(),
            closed: AtomicFlag::new(false),
            open: AtomicFlag::new(false),
        },
        GFP_KERNEL,
    ) {
        Ok(lid) => KBox::into_raw(lid),
        Err(_) => return ENOMEM.to_errno(),
    };
    // SAFETY: `lid` was allocated above; `dev` and `handler` are live for
    // the handle's lifetime, which ends in `lid_disconnect`.
    let handle = unsafe { (*lid).handle.get() };
    // SAFETY: as above; the handle is not registered yet.
    unsafe {
        (*handle).dev = dev;
        (*handle).handler = handler;
        (*handle).name = c"t8140-aop-audio".as_char_ptr();
    }
    // SAFETY: the handle is filled in and stays allocated until it is
    // unregistered.
    let ret = unsafe { bindings::input_register_handle(handle) };
    if ret < 0 {
        // SAFETY: allocated above and never registered.
        drop(unsafe { KBox::from_raw(lid) });
        return ret;
    }
    // SAFETY: the handle is registered.
    let ret = unsafe { bindings::input_open_device(handle) };
    if ret < 0 {
        // Nothing was recorded for the handle (`LidHandle::record`).
        // SAFETY: registered above, not open; nothing else refers to it.
        unsafe {
            bindings::input_unregister_handle(handle);
            drop(KBox::from_raw(lid));
        }
        return ret;
    }
    // Events reach the handle from here on: enable readings and read the
    // switch as it is now, in one step under the event lock.
    // SAFETY: the handle is registered and open, and so is the handler.
    unsafe { lid_resync(handle, true) };
    0
}

/// # Safety
///
/// Called by the input core with a handle `lid_connect` registered, while
/// the handler is still registered.
unsafe extern "C" fn lid_disconnect(handle: *mut bindings::input_handle) {
    // SAFETY: the handle is registered and open; after this no event
    // reaches it.
    unsafe {
        bindings::input_close_device(handle);
        bindings::input_unregister_handle(handle);
    }
    // SAFETY: the handle is the first field of the `LidHandle` allocated by
    // `lid_connect`, and the handler is still registered.
    let (lid, data) = unsafe { (KBox::from_raw(handle.cast::<LidHandle>()), lid_data(handle)) };
    data.lid_forget(&lid);
}

// ---- the driver's own card: the low-power microphone (PCM 0) and the
// microphone array (PCM 1) ----

/// The driver data behind one of this card's PCMs.
///
/// # Safety
///
/// `substream` must be a live substream of a PCM whose private data is the
/// `Arc` handed to ALSA at probe (released in `pcm_free_private` after the
/// last op).
unsafe fn pcm_data<'a>(
    substream: *mut bindings::snd_pcm_substream,
) -> ArcBorrow<'a, SndSocT8140AopData> {
    // SAFETY: by the caller's contract.
    unsafe { Arc::<SndSocT8140AopData>::borrow((*substream).private_data.cast()) }
}

/// # Safety
///
/// Called by ALSA with a live substream of this card's PCM, whose private
/// data is the driver's `Arc`.
unsafe extern "C" fn lpai_pcm_open(substream: *mut bindings::snd_pcm_substream) -> i32 {
    // SAFETY: ALSA calls this op with a live substream of the LPAI PCM.
    let data = unsafe { pcm_data(substream) };
    let hw = bindings::snd_pcm_hardware {
        info: bindings::SNDRV_PCM_INFO_MMAP
            | bindings::SNDRV_PCM_INFO_MMAP_VALID
            | bindings::SNDRV_PCM_INFO_INTERLEAVED,
        formats: bindings::BINDINGS_SNDRV_PCM_FMTBIT_S32_LE,
        subformats: 0,
        rates: bindings::SNDRV_PCM_RATE_16000,
        rate_min: LPAI_RATE,
        rate_max: LPAI_RATE,
        channels_min: LPAI_CHANNELS,
        channels_max: LPAI_CHANNELS,
        buffer_bytes_max: LPAI_PERIOD_BYTES * LPAI_PERIODS_MAX as usize,
        period_bytes_min: LPAI_PERIOD_BYTES,
        period_bytes_max: LPAI_PERIOD_BYTES,
        periods_min: 2,
        periods_max: LPAI_PERIODS_MAX,
        fifo_size: 0,
    };
    // SAFETY: ALSA calls this op with a live substream whose runtime exists
    // for the op's duration.
    unsafe {
        (*(*substream).runtime).hw = hw;
    }
    data.running.store(false, Relaxed);
    let mut st = data.stream.lock();
    st.substream = substream;
    st.copied = None;
    st.hw_ptr_bytes = 0;
    st.period_acc = 0;
    data.hw_ptr_frames.store(0, Release);
    0
}

/// # Safety
///
/// Called by ALSA with a live substream of this card's PCM, whose private
/// data is the driver's `Arc`.
unsafe extern "C" fn lpai_pcm_close(substream: *mut bindings::snd_pcm_substream) -> i32 {
    // SAFETY: ALSA calls this op with a live substream of the LPAI PCM.
    let data = unsafe { pcm_data(substream) };
    data.running.store(false, Relaxed);
    {
        let mut st = data.stream.lock();
        st.substream = ptr::null_mut();
    }
    if data.powered.xchg(false, Relaxed) {
        if let Err(e) = data.lpai_run_put() {
            dev_err!(data.dev, "unable to return lpai to idle\n");
            return e.to_errno();
        }
    }
    0
}

/// # Safety
///
/// Called by ALSA with a live substream of this card's PCM, whose private
/// data is the driver's `Arc`.
unsafe extern "C" fn lpai_pcm_prepare(substream: *mut bindings::snd_pcm_substream) -> i32 {
    // SAFETY: ALSA calls this op with a live substream of the LPAI PCM.
    let data = unsafe { pcm_data(substream) };
    {
        let mut st = data.stream.lock();
        // SAFETY: ALSA calls this op with a live substream; its runtime and
        // DMA buffer exist between hw_params and hw_free, which bracket
        // prepare.
        st.buffer_bytes = unsafe { (*(*substream).runtime).dma_bytes };
        st.hw_ptr_bytes = 0;
        st.period_acc = 0;
        st.copied = None;
        data.hw_ptr_frames.store(0, Release);
    }
    if !data.powered.load(Relaxed) {
        if let Err(e) = data.lpai_run_get() {
            dev_err!(data.dev, "unable to run lpai\n");
            return e.to_errno();
        }
        data.powered.store(true, Relaxed);
    }
    0
}

/// Called with the ALSA stream lock held: only the run flag changes here,
/// the counters were reset by prepare.
///
/// # Safety
///
/// Called by ALSA with a live substream of this card's PCM, whose private
/// data is the driver's `Arc`.
unsafe extern "C" fn lpai_pcm_trigger(
    substream: *mut bindings::snd_pcm_substream,
    cmd: i32,
) -> i32 {
    // SAFETY: ALSA calls this op with a live substream of the LPAI PCM.
    let data = unsafe { pcm_data(substream) };
    match cmd as u32 {
        bindings::SNDRV_PCM_TRIGGER_START | bindings::SNDRV_PCM_TRIGGER_RESUME => {
            data.running.store(true, Relaxed);
            0
        }
        bindings::SNDRV_PCM_TRIGGER_STOP | bindings::SNDRV_PCM_TRIGGER_SUSPEND => {
            data.running.store(false, Relaxed);
            0
        }
        _ => EINVAL.to_errno(),
    }
}

/// Called after a stop, without the ALSA stream lock, before the buffer is
/// reallocated or freed (hw_params, hw_free) or the counters are reset
/// (prepare).  The trigger only clears the run flag, so a report that saw
/// the stream running may still be copying into the buffer: it holds the
/// stream lock for the whole copy, so taking that lock waits for it, and
/// every report after it sees the flag clear.
///
/// # Safety
///
/// Called by ALSA with a live substream of this card's PCM, whose private
/// data is the driver's `Arc`.
unsafe extern "C" fn lpai_pcm_sync_stop(substream: *mut bindings::snd_pcm_substream) -> i32 {
    // SAFETY: ALSA calls this op with a live substream of the LPAI PCM.
    let data = unsafe { pcm_data(substream) };
    drop(data.stream.lock());
    0
}

/// # Safety
///
/// Called by ALSA with a live substream of this card's PCM, whose private
/// data is the driver's `Arc`.
unsafe extern "C" fn lpai_pcm_pointer(
    substream: *mut bindings::snd_pcm_substream,
) -> bindings::snd_pcm_uframes_t {
    // SAFETY: ALSA calls this op with a live substream of the LPAI PCM.
    let data = unsafe { pcm_data(substream) };
    data.hw_ptr_frames.load(Acquire) as bindings::snd_pcm_uframes_t
}

/// The microphone array's capture: the LEAP produces float32 into the ADMAC
/// channel requested here; the channel's SRAM registers only answer while
/// its own LEAP leaf is powered, so the service (and with it the leaves)
/// comes up before the request and goes down after the release.
///
/// # Safety
///
/// Called by ALSA with a live substream of this card's PCM, whose private
/// data is the driver's `Arc`.
unsafe extern "C" fn hpai_pcm_open(substream: *mut bindings::snd_pcm_substream) -> i32 {
    // SAFETY: ALSA calls this op with a live substream of the HPAI PCM.
    let data = unsafe { pcm_data(substream) };
    if let Err(e) = data.hpai_startup() {
        return e.to_errno();
    }
    // SAFETY: the platform device is live and the channel name is
    // NUL-terminated.
    let chan = match from_err_ptr(unsafe {
        bindings::dma_request_chan(data.dev.as_raw(), c"hpai".as_char_ptr())
    }) {
        Ok(chan) => chan,
        Err(e) => {
            dev_err!(data.dev, "hpai: DMA channel unavailable: {:?}\n", e);
            data.hpai_shutdown();
            return e.to_errno();
        }
    };
    let mut hw = dmaengine_hw();
    hw.rates = bindings::SNDRV_PCM_RATE_48000;
    hw.rate_min = HPAI_RATE;
    hw.rate_max = HPAI_RATE;
    hw.channels_min = HPAI_CHANNELS;
    hw.channels_max = HPAI_CHANNELS;
    let ret = pcm_open_dmaengine(
        substream,
        chan,
        hw,
        bindings::BINDINGS_SNDRV_PCM_FMTBIT_FLOAT_LE,
    );
    if ret < 0 {
        // SAFETY: `chan` was requested above and is not attached to the
        // substream when the open failed.
        unsafe { bindings::dma_release_channel(chan) };
        data.hpai_shutdown();
    }
    ret
}

/// # Safety
///
/// Called by ALSA with a live substream of this card's PCM, whose private
/// data is the driver's `Arc`.
unsafe extern "C" fn hpai_pcm_close(substream: *mut bindings::snd_pcm_substream) -> i32 {
    // SAFETY: ALSA calls this op with a live substream of the HPAI PCM.
    let data = unsafe { pcm_data(substream) };
    // SAFETY: the substream is live and its dmaengine runtime was set up by
    // `snd_dmaengine_pcm_open` in `hpai_pcm_open`.
    let ret = unsafe { bindings::snd_dmaengine_pcm_close_release_chan(substream) };
    data.hpai_shutdown();
    ret
}

/// Nothing to arm before the trigger: the dmaengine runtime was set up at
/// open, and the DMA start followed by the queued firmware run belong to
/// trigger.  ALSA calls the prepare op unconditionally, so it must exist:
/// without it the first hpai prepare jumped to address 0.
///
/// # Safety
///
/// Called by ALSA with a live substream of this card's PCM.
unsafe extern "C" fn hpai_pcm_prepare(_substream: *mut bindings::snd_pcm_substream) -> i32 {
    0
}

/// Trigger, with the ALSA stream lock held: the DMA first, then the
/// sleeping firmware start queued behind it.
///
/// # Safety
///
/// Called by ALSA with a live substream of this card's PCM, whose private
/// data is the driver's `Arc`.
unsafe extern "C" fn hpai_pcm_trigger(
    substream: *mut bindings::snd_pcm_substream,
    cmd: i32,
) -> i32 {
    // SAFETY: ALSA calls this op with a live substream of the HPAI PCM.
    let data = unsafe { pcm_data(substream) };
    // SAFETY: the substream is live and its dmaengine runtime was set up by
    // `snd_dmaengine_pcm_open` in `hpai_pcm_open`.
    let ret = unsafe { bindings::snd_dmaengine_pcm_trigger(substream, cmd) };
    if ret < 0 {
        return ret;
    }
    match cmd as u32 {
        bindings::SNDRV_PCM_TRIGGER_START
        | bindings::SNDRV_PCM_TRIGGER_RESUME
        | bindings::SNDRV_PCM_TRIGGER_PAUSE_RELEASE => {
            data.hpai_want_run.store(true, Relaxed);
            // Err = already queued: the pending run will see want_run.
            let _ = workqueue::system_dfl().enqueue::<_, HPAI_START_WORK>(Arc::from(data));
        }
        _ => data.hpai_want_run.store(false, Relaxed),
    }
    0
}

/// The trigger stops the DMA asynchronously: wait for the channel to stop
/// and for its completion callbacks to finish before ALSA reuses or frees
/// the buffer.
///
/// # Safety
///
/// Called by ALSA with a live substream of this card's PCM.
unsafe extern "C" fn hpai_pcm_sync_stop(substream: *mut bindings::snd_pcm_substream) -> i32 {
    // SAFETY: the substream is live and its dmaengine runtime was set up by
    // `snd_dmaengine_pcm_open` in `hpai_pcm_open`.
    unsafe { bindings::snd_dmaengine_pcm_sync_stop(substream) }
}

/// # Safety
///
/// Called once by ALSA when the PCM goes away, with the `Arc` leaked into
/// its private data at probe.
unsafe extern "C" fn pcm_free_private(pcm: *mut bindings::snd_pcm) {
    // SAFETY: this is the `Arc` leaked into the PCM's private data at probe;
    // ALSA calls this once, when the PCM goes away.
    unsafe {
        Arc::<SndSocT8140AopData>::from_foreign((*pcm).private_data.cast());
    }
}

/// The dmaengine PCMs' common description: mmap-able, interleaved, a 1 MiB
/// buffer in periods from 256 bytes to the ADMAC's segment size.  Rates,
/// channels and formats are left to the caller (or, for a DPCM front-end,
/// to the DAI streams: ASoC fills in what is still zero here).
fn dmaengine_hw() -> bindings::snd_pcm_hardware {
    bindings::snd_pcm_hardware {
        info: bindings::SNDRV_PCM_INFO_MMAP
            | bindings::SNDRV_PCM_INFO_MMAP_VALID
            | bindings::SNDRV_PCM_INFO_INTERLEAVED,
        formats: 0,
        subformats: 0,
        rates: 0,
        rate_min: 0,
        rate_max: 0,
        channels_min: 0,
        channels_max: 0,
        buffer_bytes_max: PCM_BUFFER_MAX,
        period_bytes_min: PCM_PERIOD_BYTES_MIN,
        period_bytes_max: PCM_PERIOD_BYTES_MAX,
        periods_min: 2,
        periods_max: (PCM_BUFFER_MAX / PCM_PERIOD_BYTES_MIN) as u32,
        fifo_size: 16,
    }
}

/// Set up a dmaengine-backed substream on `chan`: a managed buffer on the
/// DMA device (once per substream), `hw` refined by the channel's
/// capabilities and its formats narrowed to `formats`, then the dmaengine
/// runtime.  On failure the channel is left with the caller.
fn pcm_open_dmaengine(
    substream: *mut bindings::snd_pcm_substream,
    chan: *mut bindings::dma_chan,
    mut hw: bindings::snd_pcm_hardware,
    formats: u64,
) -> i32 {
    // SAFETY: the callers hand over a live substream and a channel they
    // requested.
    let unallocated =
        unsafe { (*substream).dma_buffer.dev.type_ == bindings::SNDRV_DMA_TYPE_UNKNOWN as i32 };
    if unallocated {
        // SAFETY: as above; the channel's device is the DMA controller and
        // outlives the buffer, which ALSA frees with the PCM.
        let ret = unsafe {
            bindings::snd_pcm_set_managed_buffer(
                substream,
                bindings::SNDRV_DMA_TYPE_DEV as i32,
                (*(*chan).device).dev,
                0,
                0,
            )
        };
        if ret < 0 {
            return ret;
        }
    }
    let mut dma_data = bindings::snd_dmaengine_dai_dma_data::default();
    // SAFETY: the substream and the channel are live for the call.
    let ret = unsafe {
        bindings::snd_dmaengine_pcm_refine_runtime_hwparams(substream, &mut dma_data, &mut hw, chan)
    };
    if ret < 0 {
        return ret;
    }
    // The refinement describes the bus widths the channel can move, not the
    // sample representation; the LEAP produces and consumes one encoding.
    hw.formats &= formats;
    if hw.formats == 0 {
        return EINVAL.to_errno();
    }
    // SAFETY: the substream's runtime exists for the op's duration; the
    // dmaengine open attaches `chan` to it on success.
    unsafe {
        (*(*substream).runtime).hw = hw;
        bindings::snd_dmaengine_pcm_open(substream, chan)
    }
}

/// Buffer limits of the dmaengine PCMs.  A period is bounded by the ADMAC's
/// segment size (the channel's capabilities refine it further).
const PCM_BUFFER_MAX: usize = 1024 * 1024;
const PCM_PERIOD_BYTES_MIN: usize = 256;
const PCM_PERIOD_BYTES_MAX: usize = 0x10000;
/// A copy-time gain change must not sit behind a long queue of converted
/// speaker samples: at 48 kHz stereo S32 this bounds the queued audio to
/// 16384 frames (341 ms), while still allowing the largest PipeWire quantum.
const SPKR_BUFFER_MAX: usize = 128 * 1024;

/// Copies the NUL-terminated `source` into the fixed-size C string field
/// `target`; EINVAL if it does not fit.
fn copy_str(target: &mut [u8], source: &[u8]) -> Result {
    target
        .get_mut(..source.len())
        .ok_or(EINVAL)?
        .copy_from_slice(source);
    Ok(())
}

// ---- ASoC component: DPCM front-end PCMs on the ADMAC channels, back-end
// DAIs on the AOP output services (macaudio's J700 variant links them) ----

/// Front-end PCM DAIs ("j700-pcm-N"): 0 the jack (playback and the headset
/// microphone), 1 the speaker (playback only), 2 the speaker tap (capture).
const FE_COUNT: usize = 3;
/// DMA channel names per front-end and stream (dma-names on the audio
/// child): [playback, capture].  The primary front-end is the jack: cout on
/// base-ns TX2, cin (the headset microphone) on base-ns RX2.  The speaker
/// front-end's "spkr" channel is on the LEAP ADMAC: requesting it touches
/// the channel's SRAM registers, which only answer with its leaf powered.
const FE_DMA_NAMES: [[Option<&core::ffi::CStr>; 2]; FE_COUNT] = [
    [Some(c"cout"), Some(c"cin")],
    [Some(c"spkr"), None],
    [None, Some(c"spkr-tap")],
];
/// The front-end that plays the internal speaker (j700-pcm-1, the card's
/// "Secondary" PCM): its data goes through `spkr_copy` below.
const FE_SPKR: usize = 1;

/// Digital ceiling of the speaker path, as a Q31 gain applied to every
/// sample userspace writes: 0.5, the amplifier's full scale.
///
/// The words on the speaker wire are `2 × float × 2^31`, as read back on
/// the SpeakerTap: a float of 6 791 000/2^31 came back as a clean 1 kHz
/// sine with peak 13 582 000 and nothing else in the path.  A float of 0.5
/// is therefore the I2S full scale (0 dBFS on the wire, 7.3 V peak at the
/// MAX98360A's output on the J700) and anything above it would wrap.
/// Protection is the userspace daemon's job, exactly as on the other Apple
/// laptops: it owns "Speaker Playback Volume" and governs it with the
/// speaker's thermal model, fed by the tap; when no daemon holds the lock,
/// macaudio keeps the control 20 dB down.
const SPKR_CEILING_Q31: i64 = 1 << 30;

/// "Speaker Playback Volume": the software gain below the ceiling that the
/// speaker-protection daemon and the card's volume lock drive, 0.5 dB per
/// step from -63.0 dB (1) to 0 dB (127) relative to SPKR_CEILING_Q31, 0 =
/// mute; entry i = round(SPKR_CEILING_Q31 * 10^(-(127 - i) / 40)).
const SPKR_VOLUME_MAX: u32 = 127;
const _: () = assert!(SPKR_VOLUME_Q31[SPKR_VOLUME_MAX as usize] as i64 == SPKR_CEILING_Q31);
const SPKR_VOLUME_Q31: [u32; 128] = [
    0, 760151, 805193, 852903, 903441, 956973, 1013678, 1073742, 1137365, 1204758, 1276145,
    1351761, 1431858, 1516701, 1606571, 1701766, 1802602, 1909413, 2022553, 2142397, 2269342,
    2403809, 2546243, 2697118, 2856932, 3026216, 3205530, 3395470, 3596664, 3809780, 4035523,
    4274643, 4527932, 4796229, 5080423, 5381457, 5700328, 6038094, 6395874, 6774853, 7176288,
    7601510, 8051928, 8529034, 9034412, 9569734, 10136776, 10737418, 11373650, 12047581, 12761445,
    13517609, 14318577, 15167006, 16065708, 17017661, 18026021, 19094130, 20225528, 21423966,
    22693416, 24038085, 25462431, 26971175, 28569318, 30262156, 32055302, 33954698, 35966640,
    38097798, 40355234, 42746432, 45279317, 47962285, 50804230, 53814569, 57003283, 60380940,
    63958736, 67748529, 71762882, 76015100, 80519278, 85290345, 90344115, 95697341, 101367765,
    107374182, 113736503, 120475814, 127614455, 135176087, 143185773, 151670064, 160657080,
    170176611, 180260209, 190941298, 202255281, 214239660, 226934158, 240380852, 254624313,
    269711752, 285693178, 302621563, 320553018, 339546978, 359666402, 380977976, 403552340,
    427464319, 452793173, 479622855, 508042296, 538145694, 570032831, 603809400, 639587356,
    677485290, 717628817, 760150998, 805192776, 852903448, 903441154, 956973408, 1013677647,
    1073741824,
];
/// The component's only "register": the speaker volume.
const REG_SPKR_VOLUME: u32 = 0;
/// DECLARE_TLV_DB_SCALE(-6350, 50, 1): a dB scale from -63.50 dB in 0.50 dB
/// steps whose lowest value mutes.
static SPKR_VOLUME_TLV: [u32; 4] = [
    bindings::SNDRV_CTL_TLVT_DB_SCALE,
    2 * mem::size_of::<u32>() as u32,
    (-6350i32) as u32,
    50 | bindings::TLV_DB_SCALE_MUTE,
];

/// Audio data from a live ASoC component's separately owned context.
///
/// # Safety
///
/// `component` must belong to this driver's live per-instance descriptor.
/// The descriptor/context outlives synchronous registration and unregister.
unsafe fn component_data<'a>(
    component: *mut bindings::snd_soc_component,
) -> &'a SndSocT8140AopData {
    // SAFETY: by the caller's contract.
    unsafe { &component_context(component).data }
}

/// The stable context behind this component's descriptor.
///
/// # Safety
///
/// As for [`component_data`].
unsafe fn component_context<'a>(component: *mut bindings::snd_soc_component) -> &'a AsocContext {
    // SAFETY: by the caller's contract.
    let descriptor = unsafe { (*component).driver };
    // SAFETY: AsocContext is repr(C), with the descriptor as its first field.
    // C keeps the exact descriptor pointer passed to registration.
    unsafe { &*descriptor.cast::<AsocContext>() }
}

/// The ASoC runtime of a substream.
///
/// # Safety
///
/// `substream` must be live and belong to one of this component's links.
unsafe fn substream_rtd(
    substream: *mut bindings::snd_pcm_substream,
) -> *mut bindings::snd_soc_pcm_runtime {
    // SAFETY: by the caller's contract.
    unsafe {
        (*substream)
            .private_data
            .cast::<bindings::snd_soc_pcm_runtime>()
    }
}

/// Whether a runtime is a DPCM back-end (no PCM of its own).
///
/// # Safety
///
/// `rtd` must be a live ASoC runtime.
unsafe fn rtd_is_be(rtd: *mut bindings::snd_soc_pcm_runtime) -> bool {
    // SAFETY: by the caller's contract.
    unsafe { (*(*rtd).dai_link).no_pcm() != 0 }
}

/// The first CPU DAI of a runtime.
///
/// # Safety
///
/// `rtd` must be a live ASoC runtime.
unsafe fn rtd_cpu_dai(rtd: *mut bindings::snd_soc_pcm_runtime) -> *mut bindings::snd_soc_dai {
    // SAFETY: by the caller's contract.
    unsafe { *(*rtd).dais }
}

/// DAI ids: 0 means "assign one" to ASoC, so the front-ends carry 1..=3
/// (index + 1) and the back-ends start at 10.
const FE_DAI_ID_BASE: u32 = 1;
const BE_DAI_ID_COUT: u32 = 10;
const BE_DAI_ID_SPKR: u32 = 11;

/// The front-end index of a runtime, `None` for a back-end.
///
/// # Safety
///
/// `rtd` must be a live ASoC runtime of one of this component's links.
unsafe fn fe_index(rtd: *mut bindings::snd_soc_pcm_runtime) -> Option<usize> {
    // SAFETY: by the caller's contract.
    let id = unsafe { (*rtd_cpu_dai(rtd)).id } as u32;
    if id >= FE_DAI_ID_BASE && id < FE_DAI_ID_BASE + FE_COUNT as u32 {
        Some((id - FE_DAI_ID_BASE) as usize)
    } else {
        None
    }
}

/// # Safety
///
/// `substream` must be live.
unsafe fn substream_is_playback(substream: *mut bindings::snd_pcm_substream) -> bool {
    // SAFETY: by the caller's contract.
    let stream = unsafe { (*substream).stream };
    stream == bindings::SNDRV_PCM_STREAM_PLAYBACK as i32
}

/// Front-end open: request the stream's ADMAC channel and set the substream
/// up on it.  The speaker channel is on the LEAP ADMAC, whose channel
/// registers only answer with the LEAP leaves powered, so those are held
/// from here to close (the back-end holds its own references while the
/// service runs).
///
/// # Safety
///
/// Called by ASoC on this driver's registered component, with live
/// arguments belonging to one of its links.
unsafe extern "C" fn asoc_pcm_open(
    component: *mut bindings::snd_soc_component,
    substream: *mut bindings::snd_pcm_substream,
) -> i32 {
    // SAFETY: ASoC calls this op on a component whose descriptor/context stays live
    // throughout registration, use and unregister.
    let data = unsafe { component_data(component) };
    // SAFETY: ASoC hands its ops a live substream whose private data is the
    // `snd_soc_pcm_runtime` of one of this component's links.
    let rtd = unsafe { substream_rtd(substream) };
    // SAFETY: as above.
    if unsafe { rtd_is_be(rtd) } {
        return 0;
    }
    // SAFETY: as above.
    let Some(fe) = (unsafe { fe_index(rtd) }) else {
        return EINVAL.to_errno();
    };
    // SAFETY: the substream is live for the duration of the op.
    let playback = unsafe { substream_is_playback(substream) };
    let Some(name) = FE_DMA_NAMES[fe][usize::from(!playback)] else {
        return ENODEV.to_errno();
    };
    let leap = fe == FE_SPKR;
    if leap {
        if let Err(e) = data.power_up_raising(data.spkr.name, &data.spkr_domains()) {
            return e.to_errno();
        }
    }
    // SAFETY: the platform device is live and the channel name is
    // NUL-terminated.
    let chan = match from_err_ptr(unsafe {
        bindings::dma_request_chan(data.dev.as_raw(), name.as_char_ptr())
    }) {
        Ok(chan) => chan,
        Err(e) => {
            dev_err!(
                data.dev,
                "front-end {}: DMA channel {:?} unavailable: {:?}\n",
                fe,
                name,
                e
            );
            if leap {
                data.power_down(&data.spkr_domains());
            }
            return e.to_errno();
        }
    };
    let mut hw = dmaengine_hw();
    let mut formats = u64::MAX;
    if fe == FE_SPKR {
        // S32 in, converted by `asoc_pcm_copy` through the ceiling into the
        // float32 the LEAP consumes: no mmap, and a bounded queue.
        hw.info = bindings::SNDRV_PCM_INFO_INTERLEAVED;
        hw.buffer_bytes_max = SPKR_BUFFER_MAX;
        hw.periods_max = (SPKR_BUFFER_MAX / PCM_PERIOD_BYTES_MIN) as u32;
        formats = bindings::BINDINGS_SNDRV_PCM_FMTBIT_S32_LE;
    }
    let ret = pcm_open_dmaengine(substream, chan, hw, formats);
    if ret < 0 {
        // SAFETY: `chan` was requested above and is not attached to the
        // substream when the open failed.
        unsafe { bindings::dma_release_channel(chan) };
        if leap {
            data.power_down(&data.spkr_domains());
        }
    }
    ret
}

/// # Safety
///
/// Called by ASoC on this driver's registered component, with live
/// arguments belonging to one of its links.
unsafe extern "C" fn asoc_pcm_close(
    component: *mut bindings::snd_soc_component,
    substream: *mut bindings::snd_pcm_substream,
) -> i32 {
    // SAFETY: ASoC calls this op on a component whose descriptor/context stays live
    // throughout registration, use and unregister.
    let data = unsafe { component_data(component) };
    // SAFETY: ASoC hands its ops a live substream whose private data is the
    // `snd_soc_pcm_runtime` of one of this component's links.
    let rtd = unsafe { substream_rtd(substream) };
    // SAFETY: as above.
    if unsafe { rtd_is_be(rtd) } {
        return 0;
    }
    // SAFETY: the substream is live and its dmaengine runtime was set up by
    // `snd_dmaengine_pcm_open` in `asoc_pcm_open`.
    let ret = unsafe { bindings::snd_dmaengine_pcm_close_release_chan(substream) };
    // SAFETY: as for `rtd_is_be`.
    if unsafe { fe_index(rtd) } == Some(FE_SPKR) {
        data.power_down(&data.spkr_domains());
    }
    ret
}

/// # Safety
///
/// Called by ASoC on this driver's registered component, with live
/// arguments belonging to one of its links.
unsafe extern "C" fn asoc_pcm_hw_params(
    _component: *mut bindings::snd_soc_component,
    substream: *mut bindings::snd_pcm_substream,
    params: *mut bindings::snd_pcm_hw_params,
) -> i32 {
    // SAFETY: ASoC hands its ops a live substream whose private data is the
    // `snd_soc_pcm_runtime` of one of this component's links.
    if unsafe { rtd_is_be(substream_rtd(substream)) } {
        return 0;
    }
    // SAFETY: the substream is live and its dmaengine runtime was set up by
    // `snd_dmaengine_pcm_open` in `asoc_pcm_open`.
    let chan = unsafe { bindings::snd_dmaengine_pcm_get_chan(substream) };
    let mut cfg = bindings::dma_slave_config::default();
    // SAFETY: the substream and the hardware parameters are live for the op.
    let rc = unsafe { bindings::snd_hwparams_to_dma_slave_config(substream, params, &mut cfg) };
    if rc < 0 {
        return rc;
    }
    // Interval query for the channel count: bindgen has no inline helper.
    // SAFETY: ASoC calls hw_params with a live `snd_pcm_hw_params`.
    let channels = unsafe {
        let iv = &(*params).intervals[(bindings::SNDRV_PCM_HW_PARAM_CHANNELS
            - bindings::SNDRV_PCM_HW_PARAM_FIRST_INTERVAL)
            as usize];
        iv.min
    };
    // The ADMAC moves one frame per burst, up to four words wide.
    let window = core::cmp::min(channels, 4);
    // SAFETY: the substream is live for the duration of the op.
    if unsafe { substream_is_playback(substream) } {
        cfg.dst_port_window_size = window;
    } else {
        cfg.src_port_window_size = window;
    }
    // SAFETY: `chan` is the live channel behind this substream's dmaengine runtime.
    unsafe {
        match (*(*chan).device).device_config {
            Some(f) => f(chan, &mut cfg),
            None => ENOSYS.to_errno(),
        }
    }
}

/// Clear old words before a new speaker stream or recovery from a revoked
/// protection lease; the queued firmware start follows the DMA trigger.
///
/// # Safety
///
/// Called by ASoC on this driver's registered component, with live
/// arguments belonging to one of its links.
unsafe extern "C" fn asoc_pcm_prepare(
    component: *mut bindings::snd_soc_component,
    substream: *mut bindings::snd_pcm_substream,
) -> i32 {
    // SAFETY: ASoC hands its ops a live substream whose private data is the
    // `snd_soc_pcm_runtime` of one of this component's links.
    let rtd = unsafe { substream_rtd(substream) };
    // SAFETY: as above.
    if unsafe { rtd_is_be(rtd) } || unsafe { fe_index(rtd) } != Some(FE_SPKR) {
        return 0;
    }
    // SAFETY: ALSA calls this op with a live substream; its runtime and DMA
    // buffer exist between hw_params and hw_free, which bracket prepare.
    let (area, bytes) = unsafe {
        let runtime = (*substream).runtime;
        ((*runtime).dma_area, (*runtime).dma_bytes)
    };
    if !area.is_null() && bytes != 0 {
        // SAFETY: the managed buffer is dma_bytes long and no DMA runs before
        // trigger START.
        unsafe { ptr::write_bytes(area, 0, bytes) };
    }
    // SAFETY: ASoC calls this op on a component whose descriptor/context stays live.
    let data = unsafe { component_data(component) };
    if !data.has_lpai {
        // The speaker cooled while no stream ran.
        let mut thermal = data.spkr_thermal.lock();
        thermal.cool_idle();
        thermal.reset_highpass();
    }
    0
}

/// The float32 bit pattern of `v / 2^31`, built with integer arithmetic (the
/// kernel has no floating point).
fn f32_bits_of_q31(v: i32) -> u32 {
    if v == 0 {
        return 0;
    }
    let sign = if v < 0 { 1u32 << 31 } else { 0 };
    let m = v.unsigned_abs();
    // The position of the leading one, 0..=31: |v| / 2^31 = 1.f * 2^(p - 31).
    let p = 31 - m.leading_zeros();
    let exp = (p as i32 - 31 + 127) as u32;
    let mant = if p >= 23 {
        (m >> (p - 23)) & 0x7f_ffff
    } else {
        (m << (23 - p)) & 0x7f_ffff
    };
    sign | (exp << 23) | mant
}

/// One S32 sample -> the float32 word the LEAP consumes, through the volume
/// table (whose top entry is the ceiling).
fn spkr_f32_bits(sample: i32, gain_q31: u32) -> u32 {
    let scaled = (i64::from(sample) * i64::from(gain_q31)) >> 31; // |scaled| < SPKR_CEILING_Q31
    f32_bits_of_q31(scaled as i32)
}

/// Feed-forward thermal limiter of the T8152 speaker.
///
/// The J873 has no speaker tap: the AOP offers no sense stream, so there is
/// nothing a daemon could measure.  The vendor protects this speaker with a
/// thermal model driven by the signal itself, and so does this limiter, in the
/// copy path where no stream can bypass it.  The vendor's numbers for the
/// speaker (one type-A speaker for all channels): amplifier sensitivity
/// 11.9 V per full scale, voice-coil resistance 4.6 ohm, coil 40 C/W with a
/// time constant of 3.4 s, magnet 60 C/W with 210 s, ambient 50 C, limit
/// 140 C.  The thermal part is the two-filter model of the Linux speaker
/// daemon: the power of every word (x * 11.9 V)^2 / 4.6 ohm, with x the word
/// after the limiter gain in units of the I2S full scale, heats the magnet
/// towards ambient + 60 C/W * P and the coil towards the magnet + 40 C/W * P;
/// the gain falls from 0 dB to the sustained level (a full-scale square wave
/// at 98 % of (140 - 50) / (40 + 60) = 0.9 W, -15.43 dB) over the last 20 C
/// below the limit.  Beyond the vendor, the attack is immediate (one block of
/// 128 words) and the release follows the vendor's 20 s, and the amplifier is
/// assumed at the voltage of the full 11.9 V per full scale.  All of it is
/// fixed point: no floating point exists in the kernel.  `CEILING` is a hard
/// amplitude ceiling in the first audible tests (-20 dB below the sustained
/// level), taken out only by a rebuild.
mod spk_thermal {
    // Temperatures are Q40 degrees C, gains Q30, the power in units of
    // (I2S full scale)^2 * 2^32.
    const KCQ: i64 = 315236;
    const KMQ: i64 = 472854;
    const AC: i64 = 26317;
    const AM: i64 = 426;
    const T_AMB_Q: i64 = 54975581388800;
    const LO_Q: i64 = 131941395333120;
    const WIN_Q: i64 = 21990232555520;
    const HYST_Q: i64 = 5497558138880;
    const REL: i64 = 572586;
    /// 10^(-15.43 dB * k / 64 / 20) in Q30, rounded down.
    const GTAB: [i64; 65] = [
        1073741824, 1044350352, 1015763411, 987958979, 960915636, 934612548, 909029454, 884146644,
        859944950, 836405728, 813510843, 791242658, 769584020, 748518241, 728029096, 708100798,
        688717996, 669865759, 651529563, 633695283, 616349179, 599477889, 583068416, 567108119,
        551584701, 536486206, 521801000, 507517772, 493625518, 480113536, 466971416, 454189035,
        441756545, 429664369, 417903192, 406463952, 395337838, 384516279, 373990937, 363753705,
        353796697, 344112240, 334692876, 325531348, 316620597, 307953760, 299524160, 291325303,
        283350873, 275594726, 268050888, 260713546, 253577050, 246635900, 239884750, 233318399,
        226931788, 220719998, 214678242, 208801867, 203086346, 197527275, 192120373, 186861473,
        181746525,
    ];
    /// Start state: the coil just below the start of the limiting window (the
    /// driver cannot know how warm the speaker is).
    const T_START_COIL_Q: i64 = 130841883705344;
    const T_START_MAGNET_Q: i64 = 100495362778726;
    /// The hard amplitude ceiling of the first audible tests: -20 dB below the
    /// sustained level (a worst-case power of about 9 mW); kept for the unit
    /// tests of the model.
    #[allow(dead_code)]
    pub(super) const CEILING_M20_Q30: i64 = 18174652;
    /// The ceiling of the speaker path: 0.078 of full scale (0.93 V peak and
    /// about 94 mW for a sine at an amplifier gain of 15, by this model), 6 dB
    /// below the level at which the vendor's own driver plays a 440 Hz tone at
    /// its maximum volume, measured with one microphone at one place.
    pub(super) const CEILING_Q30: i64 = 83_800_000;
    /// No ceiling (unit tests, later steps).
    #[allow(dead_code)]
    pub(super) const CEILING_0DB_Q30: i64 = 1073741824;
    /// 10 ms steps of the idle cooling: 1 - exp(-0.01 s / tau) in Q32.
    const AC10: i64 = 12613698;
    const AM10: i64 = 204517;
    const ONE: i64 = 1 << 30;

    fn gain_for_reduction(r16: i64) -> i64 {
        let k = (r16 >> 10) as usize;
        let f = r16 & 1023;
        if k >= 64 {
            return GTAB[64];
        }
        GTAB[k] + (((GTAB[k + 1] - GTAB[k]) * f) >> 10)
    }

    // Fixed high-pass of the speaker path: 2nd-order Butterworth, 100 Hz at
    // 48 kHz, direct form I, Q28 coefficients, 64-bit state.  The speaker
    // cone moves far more at low frequencies for the same voltage and the
    // power model of the limiter has no excursion term; below 100 Hz the
    // speaker radiates almost nothing anyway.  The output is held to the
    // input's full scale (a step through a high-pass overshoots by up to
    // 2x), so the ceiling stays a true peak ceiling.  |x| <= 2^31 and the
    // coefficients are below 2^29: the sum of five products stays below
    // 2^63 (the state, below 2.5 full scales, keeps it under 2^61).
    const HP_B0: i64 = 265_962_279;
    const HP_B1: i64 = -531_924_558;
    const HP_B2: i64 = 265_962_279;
    const HP_A1: i64 = -531_901_772;
    const HP_A2: i64 = 263_511_889;
    const HP_SHIFT: u32 = 28;
    const HP_LIMIT: i64 = 1 << 30;

    #[derive(Default)]
    pub(super) struct Highpass {
        x1: i64,
        x2: i64,
        y1: i64,
        y2: i64,
    }

    impl Highpass {
        pub(super) fn step(&mut self, x: i32) -> i32 {
            let x = i64::from(x);
            let acc = HP_B0 * x + HP_B1 * self.x1 + HP_B2 * self.x2
                - HP_A1 * self.y1
                - HP_A2 * self.y2;
            // The feedback keeps the unclamped value: feeding the clamped one
            // back would latch a nearly full-scale sine into sustained
            // clipping.  The state stays below 2.5 full scales.
            let y = (acc + (1 << (HP_SHIFT - 1))) >> HP_SHIFT;
            self.x2 = self.x1;
            self.x1 = x;
            self.y2 = self.y1;
            self.y1 = y;
            y.clamp(-HP_LIMIT, HP_LIMIT) as i32
        }
    }

    pub(super) struct SpkThermal {
        t_coil: i64,
        t_magnet: i64,
        hyst_coil: i64,
        hyst_magnet: i64,
        gain: i64,
        ceiling: i64,
        hp: Highpass,
        last: Option<kernel::time::Instant<kernel::time::Monotonic>>,
    }

    impl SpkThermal {
        pub(super) fn new(ceiling: i64) -> Self {
            Self {
                t_coil: T_START_COIL_Q,
                t_magnet: T_START_MAGNET_Q,
                hyst_coil: T_START_COIL_Q,
                hyst_magnet: T_START_MAGNET_Q,
                gain: ONE,
                ceiling,
                hp: Highpass::default(),
                last: None,
            }
        }

        /// Limit one block of words (the I2S full scale is 2^30), in place, and
        /// advance the model by the block.
        pub(super) fn block(&mut self, words: &mut [i32]) {
            let g = (self.gain * self.ceiling) >> 30;
            let (mut tc, mut tm) = (self.t_coil, self.t_magnet);
            for w in words.iter_mut() {
                let y = (i64::from(*w) * g) >> 30;
                let yq = y >> 14;
                let pfs = yq * yq;
                let tgt_c = tm + pfs * KCQ;
                let tgt_m = T_AMB_Q + pfs * KMQ;
                tc += (((tgt_c - tc) >> 12) * AC) >> 20;
                tm += (((tgt_m - tm) >> 12) * AM) >> 20;
                *w = y as i32;
            }
            self.t_coil = tc;
            self.t_magnet = tm;
            self.hyst_coil = self.hyst_coil.max(tc).min(tc + HYST_Q);
            self.hyst_magnet = self.hyst_magnet.max(tm).min(tm + HYST_Q);
            let temp = self.hyst_coil.max(self.hyst_magnet);
            let r16 = if temp <= LO_Q {
                0
            } else {
                core::cmp::min(65536, ((temp - LO_Q) >> 20) * 65536 / (WIN_Q >> 20))
            };
            let target = gain_for_reduction(r16);
            if target < self.gain {
                self.gain = target;
            } else {
                self.gain += ((target - self.gain) * REL) >> 32;
            }
            self.last = Some(kernel::time::Instant::now());
        }

        /// Filter one block of words in place with the speaker path's high-pass
        /// (before `block`; the filter state carries over from block to block).
        pub(super) fn highpass(&mut self, words: &mut [i32]) {
            for w in words.iter_mut() {
                *w = self.hp.step(*w);
            }
        }

        /// Start the high-pass from rest (a new stream).
        pub(super) fn reset_highpass(&mut self) {
            self.hp = Highpass::default();
        }

        /// Let the speaker cool for the time since the last block: the same
        /// filters with no power, in steps of 10 ms (at most 20 minutes).
        pub(super) fn cool_idle(&mut self) {
            let Some(last) = self.last.take() else {
                return;
            };
            let ms = core::cmp::min(last.elapsed().as_millis(), 20 * 60 * 1000);
            self.cool(ms as u64);
        }

        pub(super) fn cool(&mut self, ms: u64) {
            let (mut tc, mut tm) = (self.t_coil, self.t_magnet);
            for _ in 0..ms / 10 {
                tc += (((tm - tc) >> 12) * AC10) >> 20;
                tm += (((T_AMB_Q - tm) >> 12) * AM10) >> 20;
            }
            self.t_coil = tc;
            self.t_magnet = tm;
            self.hyst_coil = tc;
            self.hyst_magnet = tm;
        }

        /// Unit tests: start from a given state.
        #[allow(dead_code)]
        pub(super) fn cool_from(&mut self, coil: i64, magnet: i64, ms: u64) {
            self.t_coil = coil;
            self.t_magnet = magnet;
            self.cool(ms);
        }

        #[allow(dead_code)]
        pub(super) fn state(&self) -> (i64, i64, i64, i64, i64) {
            (
                self.t_coil,
                self.t_magnet,
                self.hyst_coil,
                self.hyst_magnet,
                self.gain,
            )
        }
    }
}

use spk_thermal::SpkThermal;

/// Copy between userspace and the front-end buffers.  The speaker front-end
/// converts and limits; the jack front-ends copy verbatim (their buffers are
/// also mmap-able, this path serves read()/write()).
///
/// # Safety
///
/// Called by ASoC on this driver's registered component, with live
/// arguments belonging to one of its links.
unsafe extern "C" fn asoc_pcm_copy(
    component: *mut bindings::snd_soc_component,
    substream: *mut bindings::snd_pcm_substream,
    _channel: i32,
    pos: usize,
    iter: *mut bindings::iov_iter,
    bytes: usize,
) -> i32 {
    // SAFETY: ASoC hands its ops a live substream whose private data is the
    // `snd_soc_pcm_runtime` of one of this component's links.
    let rtd = unsafe { substream_rtd(substream) };
    // SAFETY: as above.
    if unsafe { rtd_is_be(rtd) } {
        return EINVAL.to_errno();
    }
    // SAFETY: ALSA calls this op with a live substream; its runtime and DMA
    // buffer exist between hw_params and hw_free, which bracket every copy.
    let (area, dma_bytes) = unsafe {
        let runtime = (*substream).runtime;
        ((*runtime).dma_area, (*runtime).dma_bytes)
    };
    if area.is_null() || pos.checked_add(bytes).is_none_or(|end| end > dma_bytes) {
        return EINVAL.to_errno();
    }
    // SAFETY: bounds checked against the managed buffer above; ALSA
    // serializes the copies of a substream.
    let dst = unsafe { slice::from_raw_parts_mut(area.add(pos), bytes) };
    // SAFETY: the substream is live for the duration of the op.
    let playback = unsafe { substream_is_playback(substream) };
    if !playback {
        // SAFETY: ALSA hands us a live iov_iter for the whole call.
        let iov = unsafe { kernel::iov::IovIterDest::from_raw(iter) };
        return if iov.copy_to_iter(dst) == bytes {
            0
        } else {
            EFAULT.to_errno()
        };
    }
    // SAFETY: as above.
    let iov = unsafe { kernel::iov::IovIterSource::from_raw(iter) };
    // SAFETY: as for `rtd_is_be`.
    if unsafe { fe_index(rtd) } != Some(FE_SPKR) {
        return if iov.copy_from_iter(dst) == bytes {
            0
        } else {
            EFAULT.to_errno()
        };
    }
    // S32_LE interleaved stereo in, float32 LE out, same byte count.
    // SAFETY: ASoC calls this op on a component whose descriptor/context stays live
    // throughout registration, use and unregister.
    let data = unsafe { component_data(component) };
    let volume = core::cmp::min(data.spkr_volume.load(Relaxed), SPKR_VOLUME_MAX) as usize;
    let gain = SPKR_VOLUME_Q31[volume];
    let mut chunk = [0u8; 512];
    let mut off = 0;
    while off < bytes {
        let n = core::cmp::min(chunk.len(), bytes - off) & !3;
        if n == 0 {
            return EINVAL.to_errno();
        }
        if iov.copy_from_iter(&mut chunk[..n]) != n {
            return EFAULT.to_errno();
        }
        if data.has_lpai {
            for (i, word) in chunk[..n].chunks_exact(4).enumerate() {
                let sample = i32::from_le_bytes([word[0], word[1], word[2], word[3]]);
                let bits = spkr_f32_bits(sample, gain);
                dst[off + i * 4..off + i * 4 + 4].copy_from_slice(&bits.to_le_bytes());
            }
        } else {
            // T8152: every word goes through the thermal limiter, the one
            // protection this speaker has.
            let mut words = [0i32; 128];
            let count = n / 4;
            for (w, word) in words.iter_mut().zip(chunk[..n].chunks_exact(4)) {
                let sample = i32::from_le_bytes([word[0], word[1], word[2], word[3]]);
                *w = ((i64::from(sample) * i64::from(gain)) >> 31) as i32;
            }
            {
                let mut thermal = data.spkr_thermal.lock();
                thermal.highpass(&mut words[..count]);
                thermal.block(&mut words[..count]);
            }
            // The amplifier takes plain PCM from the AOP link (there is no
            // LEAP on this machine to consume float32 words), but only the
            // low 24 bits of each word: measured, a word beyond 2^23 wraps.
            // The limiter's full scale, 2^30, is therefore 2^23 on the wire.
            for (i, w) in words[..count].iter().enumerate() {
                let w24 = *w >> 7;
                dst[off + i * 4..off + i * 4 + 4].copy_from_slice(&w24.to_le_bytes());
            }
        }
        off += n;
    }
    0
}

/// # Safety
///
/// Called by ASoC on this driver's registered component, with live
/// arguments belonging to one of its links.
unsafe extern "C" fn asoc_pcm_trigger(
    _component: *mut bindings::snd_soc_component,
    substream: *mut bindings::snd_pcm_substream,
    cmd: i32,
) -> i32 {
    // SAFETY: ASoC hands its ops a live substream whose private data is the
    // `snd_soc_pcm_runtime` of one of this component's links.
    if unsafe { rtd_is_be(substream_rtd(substream)) } {
        return 0;
    }
    // SAFETY: the substream is live and its dmaengine runtime was set up by
    // `snd_dmaengine_pcm_open` in `asoc_pcm_open`.
    unsafe { bindings::snd_dmaengine_pcm_trigger(substream, cmd) }
}

/// Front-end stop synchronization, as for the microphone array: the trigger
/// stops the DMA asynchronously, so wait for the channel to stop and for its
/// completion callbacks to finish before ALSA reuses or frees the buffer.
///
/// # Safety
///
/// Called by ASoC on this driver's registered component, with live
/// arguments belonging to one of its links.
unsafe extern "C" fn asoc_pcm_sync_stop(
    _component: *mut bindings::snd_soc_component,
    substream: *mut bindings::snd_pcm_substream,
) -> i32 {
    // SAFETY: ASoC hands its ops a live substream whose private data is the
    // `snd_soc_pcm_runtime` of one of this component's links.
    if unsafe { rtd_is_be(substream_rtd(substream)) } {
        return 0;
    }
    // SAFETY: the substream is live and its dmaengine runtime was set up by
    // `snd_dmaengine_pcm_open` in `asoc_pcm_open`.
    unsafe { bindings::snd_dmaengine_pcm_sync_stop(substream) }
}

/// # Safety
///
/// Called by ASoC on this driver's registered component, with live
/// arguments belonging to one of its links.
unsafe extern "C" fn asoc_pcm_pointer(
    _component: *mut bindings::snd_soc_component,
    substream: *mut bindings::snd_pcm_substream,
) -> bindings::snd_pcm_uframes_t {
    // SAFETY: ASoC hands its ops a live substream whose private data is the
    // `snd_soc_pcm_runtime` of one of this component's links.
    if unsafe { rtd_is_be(substream_rtd(substream)) } {
        // -ENOTSUPP as the pointer callback returns it (mca does the same).
        return (-i64::from(bindings::ENOTSUPP)) as bindings::snd_pcm_uframes_t;
    }
    // SAFETY: the substream is live and its dmaengine runtime was set up by
    // `snd_dmaengine_pcm_open` in `asoc_pcm_open`.
    unsafe { bindings::snd_dmaengine_pcm_pointer(substream) }
}

/// The component's register file: the speaker volume, read and written by
/// the standard volsw control helpers.
///
/// # Safety
///
/// Called by ASoC on this driver's registered component, with live
/// arguments belonging to one of its links.
unsafe extern "C" fn asoc_component_read(
    component: *mut bindings::snd_soc_component,
    reg: u32,
) -> u32 {
    // SAFETY: ASoC calls this op on a component whose descriptor/context stays live
    // throughout registration, use and unregister.
    let data = unsafe { component_data(component) };
    match reg {
        REG_SPKR_VOLUME => data.spkr_volume.load(Relaxed),
        _ => 0,
    }
}

/// # Safety
///
/// Called by ASoC on this driver's registered component, with live
/// arguments belonging to one of its links.
unsafe extern "C" fn asoc_component_write(
    component: *mut bindings::snd_soc_component,
    reg: u32,
    val: u32,
) -> i32 {
    // SAFETY: ASoC calls this op on a component whose descriptor/context stays live
    // throughout registration, use and unregister.
    let data = unsafe { component_data(component) };
    match reg {
        REG_SPKR_VOLUME => {
            data.spkr_volume
                .store(core::cmp::min(val, SPKR_VOLUME_MAX), Relaxed);
            0
        }
        _ => EINVAL.to_errno(),
    }
}

/// Driver data from one of this component's DAIs.
///
/// # Safety
///
/// `dai` must be a live DAI of this driver's registered component.
unsafe fn dai_data<'a>(dai: *mut bindings::snd_soc_dai) -> &'a SndSocT8140AopData {
    // SAFETY: by the caller's contract.
    unsafe { component_data((*dai).component) }
}

/// The ASoC context of one of this component's DAIs.
///
/// # Safety
///
/// As for [`dai_data`].
unsafe fn dai_context<'a>(dai: *mut bindings::snd_soc_dai) -> &'a AsocContext {
    // SAFETY: by the caller's contract.
    unsafe { component_context((*dai).component) }
}

/// # Safety
///
/// Called by ASoC with a live DAI of this driver's registered component
/// (and a live substream where one is passed).
unsafe extern "C" fn dai_accept_fmt(_dai: *mut bindings::snd_soc_dai, _fmt: u32) -> i32 {
    // The AOP runs the serializers: the format, slots and clocks are the
    // firmware's, whatever the machine driver asks for.
    0
}

/// # Safety
///
/// Called by ASoC with a live DAI of this driver's registered component
/// (and a live substream where one is passed).
unsafe extern "C" fn dai_accept_sysclk(
    _dai: *mut bindings::snd_soc_dai,
    _id: i32,
    _freq: u32,
    _dir: i32,
) -> i32 {
    0
}

/// # Safety
///
/// Called by ASoC with a live DAI of this driver's registered component
/// (and a live substream where one is passed).
unsafe extern "C" fn dai_accept_bclk_ratio(_dai: *mut bindings::snd_soc_dai, _ratio: u32) -> i32 {
    0
}

/// # Safety
///
/// Called by ASoC with a live DAI of this driver's registered component
/// (and a live substream where one is passed).
unsafe extern "C" fn cout_dai_startup(
    substream: *mut bindings::snd_pcm_substream,
    dai: *mut bindings::snd_soc_dai,
) -> i32 {
    // SAFETY: ASoC calls DAI ops with a DAI of our registered component.
    let data = unsafe { dai_data(dai) };
    // SAFETY: the substream is live for the duration of the op.
    match data.jack_startup(unsafe { substream_is_playback(substream) }) {
        Ok(()) => 0,
        Err(e) => e.to_errno(),
    }
}

/// # Safety
///
/// Called by ASoC with a live DAI of this driver's registered component
/// (and a live substream where one is passed).
unsafe extern "C" fn cout_dai_prepare(
    substream: *mut bindings::snd_pcm_substream,
    dai: *mut bindings::snd_soc_dai,
) -> i32 {
    // SAFETY: ASoC calls DAI ops with a DAI of our registered component.
    let data = unsafe { dai_data(dai) };
    // SAFETY: the substream is live for the duration of the op.
    match data.jack_prepare(unsafe { substream_is_playback(substream) }) {
        Ok(()) => 0,
        Err(e) => e.to_errno(),
    }
}

/// # Safety
///
/// Called by ASoC with a live DAI of this driver's registered component
/// (and a live substream where one is passed).
unsafe extern "C" fn cout_dai_shutdown(
    substream: *mut bindings::snd_pcm_substream,
    dai: *mut bindings::snd_soc_dai,
) {
    // SAFETY: ASoC calls DAI ops with a DAI of our registered component.
    let data = unsafe { dai_data(dai) };
    // SAFETY: the substream is live for the duration of the op.
    data.jack_shutdown(unsafe { substream_is_playback(substream) });
}

/// # Safety
///
/// Called by ASoC with a live DAI of this driver's registered component
/// (and a live substream where one is passed).
unsafe extern "C" fn spkr_dai_startup(
    substream: *mut bindings::snd_pcm_substream,
    dai: *mut bindings::snd_soc_dai,
) -> i32 {
    // SAFETY: ASoC calls DAI ops with a DAI of our registered component.
    let data = unsafe { dai_data(dai) };
    // SAFETY: the substream is live for the duration of the op.
    let ret = if unsafe { substream_is_playback(substream) } {
        data.spkr_startup()
    } else {
        data.sense_startup()
    };
    match ret {
        Ok(()) => 0,
        Err(e) => e.to_errno(),
    }
}

/// The tap (capture) side runs from prepare: the RX DMA is armed by then
/// and a capture may start before or after the speaker plays.
///
/// # Safety
///
/// Called by ASoC with a live DAI of this driver's registered component
/// (and a live substream where one is passed).
unsafe extern "C" fn spkr_dai_prepare(
    substream: *mut bindings::snd_pcm_substream,
    dai: *mut bindings::snd_soc_dai,
) -> i32 {
    // SAFETY: ASoC calls DAI ops with a DAI of our registered component.
    let data = unsafe { dai_data(dai) };
    // SAFETY: the substream is live for the duration of the op.
    let ret = if unsafe { substream_is_playback(substream) } {
        data.spkr_prepare()
    } else {
        data.sense_prepare()
    };
    match ret {
        Ok(()) => 0,
        Err(e) => e.to_errno(),
    }
}

/// Atomic context: queue the sleeping start, or note the stop (playback
/// only; the tap side needs nothing here).
///
/// # Safety
///
/// Called by ASoC with a live DAI of this driver's registered component
/// (and a live substream where one is passed).
unsafe extern "C" fn spkr_dai_trigger(
    substream: *mut bindings::snd_pcm_substream,
    cmd: i32,
    dai: *mut bindings::snd_soc_dai,
) -> i32 {
    // SAFETY: the substream is live for the duration of the op.
    if !unsafe { substream_is_playback(substream) } {
        return 0;
    }
    // SAFETY: ASoC calls DAI ops with a DAI of our registered component.
    let context = unsafe { dai_context(dai) };
    match cmd as u32 {
        bindings::SNDRV_PCM_TRIGGER_START
        | bindings::SNDRV_PCM_TRIGGER_RESUME
        | bindings::SNDRV_PCM_TRIGGER_PAUSE_RELEASE => {
            context.data.spkr_want_run.store(true, Relaxed);
            // Err = already queued: the pending run will see want_run.
            let _ = workqueue::system_dfl().enqueue::<_, SPKR_START_WORK>(context.data.clone());
            0
        }
        bindings::SNDRV_PCM_TRIGGER_STOP
        | bindings::SNDRV_PCM_TRIGGER_SUSPEND
        | bindings::SNDRV_PCM_TRIGGER_PAUSE_PUSH => {
            context.data.spkr_want_run.store(false, Relaxed);
            0
        }
        _ => EINVAL.to_errno(),
    }
}

/// # Safety
///
/// Called by ASoC with a live DAI of this driver's registered component
/// (and a live substream where one is passed).
unsafe extern "C" fn spkr_dai_shutdown(
    substream: *mut bindings::snd_pcm_substream,
    dai: *mut bindings::snd_soc_dai,
) {
    // SAFETY: ASoC calls DAI ops with a DAI of our registered component.
    let data = unsafe { dai_data(dai) };
    // SAFETY: the substream is live for the duration of the op.
    if unsafe { substream_is_playback(substream) } {
        data.spkr_shutdown();
    } else {
        data.sense_shutdown();
    }
}

/// The DAI operation tables, built once per instance (the C descriptors
/// hold function pointers, which have no constant zero form in Rust).
struct DaiOps {
    cout: bindings::snd_soc_dai_ops,
    spkr: bindings::snd_soc_dai_ops,
    fe: bindings::snd_soc_dai_ops,
}

impl DaiOps {
    fn new() -> Self {
        let fe = bindings::snd_soc_dai_ops {
            set_fmt: Some(dai_accept_fmt),
            set_sysclk: Some(dai_accept_sysclk),
            set_bclk_ratio: Some(dai_accept_bclk_ratio),
            ..Default::default()
        };
        DaiOps {
            cout: bindings::snd_soc_dai_ops {
                startup: Some(cout_dai_startup),
                prepare: Some(cout_dai_prepare),
                shutdown: Some(cout_dai_shutdown),
                ..fe
            },
            spkr: bindings::snd_soc_dai_ops {
                startup: Some(spkr_dai_startup),
                prepare: Some(spkr_dai_prepare),
                trigger: Some(spkr_dai_trigger),
                shutdown: Some(spkr_dai_shutdown),
                ..fe
            },
            fe,
        }
    }
}

fn pcm_stream(
    name: &'static core::ffi::CStr,
    formats: u64,
    channels: u32,
) -> bindings::snd_soc_pcm_stream {
    bindings::snd_soc_pcm_stream {
        stream_name: name.as_char_ptr(),
        formats,
        rates: bindings::SNDRV_PCM_RATE_48000,
        rate_min: 48000,
        rate_max: 48000,
        channels_min: channels,
        channels_max: channels,
        ..Default::default()
    }
}

fn pcm_stream_range(
    name: &'static core::ffi::CStr,
    formats: u64,
    channels_min: u32,
    channels_max: u32,
) -> bindings::snd_soc_pcm_stream {
    bindings::snd_soc_pcm_stream {
        stream_name: name.as_char_ptr(),
        formats,
        rates: bindings::SNDRV_PCM_RATE_48000,
        rate_min: 48000,
        rate_max: 48000,
        channels_min,
        channels_max,
        ..Default::default()
    }
}

/// The AOP-configured serializers take 24-bit samples from the low bits
/// of each 32-bit DMA word: S24_LE.
const OUT_FORMATS: u64 = bindings::BINDINGS_SNDRV_PCM_FMTBIT_S24_LE;
const S32_FORMATS: u64 = bindings::BINDINGS_SNDRV_PCM_FMTBIT_S32_LE;

/// The DAI table: [0] the cout back-end, [1..=3] the front-end PCMs, [4] the
/// speaker back-end.  The order is ABI: `sound-dai = <&aop_audio N>` in the
/// machine DTS indexes this array, so `<&aop_audio 0>` is cout and
/// `<&aop_audio 4>` is spkr.  New DAIs go at the end; inserting one
/// renumbers every DAI after it and silently repoints the existing links.
const DAI_COUNT: usize = 2 + FE_COUNT;

// `mono_speaker`: T8152, whose speaker serializer takes one 32-bit word per
// 48 kHz frame (measured: the DMA drains 192000 bytes per second whatever the
// stream), so the speaker front-end must also accept one channel.
fn dai_drivers(ops: &DaiOps, mono_speaker: bool) -> [bindings::snd_soc_dai_driver; DAI_COUNT] {
    [
        bindings::snd_soc_dai_driver {
            name: c"cout".as_char_ptr(),
            id: BE_DAI_ID_COUT,
            ops: &ops.cout,
            playback: pcm_stream(c"COUT TX", OUT_FORMATS, 2),
            // cin: the headset microphone, one 32-bit slot per frame
            capture: pcm_stream(c"CIN RX", OUT_FORMATS, 1),
            ..Default::default()
        },
        bindings::snd_soc_dai_driver {
            name: c"j700-pcm-0".as_char_ptr(),
            id: FE_DAI_ID_BASE,
            ops: &ops.fe,
            playback: pcm_stream(c"PCM0 TX", OUT_FORMATS, 2),
            // macaudio routes "Headset Capture" into PCM0 RX (mono cin)
            capture: pcm_stream(c"PCM0 RX", OUT_FORMATS, 1),
            ..Default::default()
        },
        bindings::snd_soc_dai_driver {
            name: c"j700-pcm-1".as_char_ptr(),
            id: FE_DAI_ID_BASE + 1,
            ops: &ops.fe,
            // the speaker front-end: S32 in, converted through the ceiling
            playback: pcm_stream_range(
                c"PCM1 TX",
                S32_FORMATS,
                if mono_speaker { 1 } else { 2 },
                2,
            ),
            ..Default::default()
        },
        bindings::snd_soc_dai_driver {
            name: c"j700-pcm-2".as_char_ptr(),
            id: FE_DAI_ID_BASE + 2,
            ops: &ops.fe,
            // the speaker tap front-end: the words on the speaker wire
            capture: pcm_stream(c"PCM2 RX", S32_FORMATS, 2),
            ..Default::default()
        },
        // The internal speaker back-end: LEAP TX0 under the AOP spkr service,
        // the MAX98360A's SD_MODE driven by its own codec driver; its capture
        // side is the SpeakerTap (base-ns RX1, the tap service).
        bindings::snd_soc_dai_driver {
            name: c"spkr".as_char_ptr(),
            id: BE_DAI_ID_SPKR,
            ops: &ops.spkr,
            playback: pcm_stream_range(
                c"SPKR TX",
                S32_FORMATS,
                if mono_speaker { 1 } else { 2 },
                2,
            ),
            capture: pcm_stream(c"SPKR RX", S32_FORMATS, 2),
            ..Default::default()
        },
    ]
}

/// The ASoC component and everything the core keeps pointers into.  C
/// retains the address of `component`, so this is allocated before
/// registration and kept until unregistration has finished all callbacks.
#[repr(C)]
struct AsocContext {
    component: bindings::snd_soc_component_driver,
    data: Arc<SndSocT8140AopData>,
    dais: KBox<Opaque<[bindings::snd_soc_dai_driver; DAI_COUNT]>>,
    _ops: KBox<DaiOps>,
    _controls: KBox<[bindings::snd_kcontrol_new; 1]>,
    // The C mixer helpers modify platform_max under their control locks.
    _volume: KBox<Opaque<bindings::soc_mixer_control>>,
}

const _: () = assert!(mem::offset_of!(AsocContext, component) == 0);

impl AsocContext {
    fn new(data: Arc<SndSocT8140AopData>) -> Result<KBox<Self>> {
        let ops = KBox::new(DaiOps::new(), GFP_KERNEL)?;
        let dais = KBox::new(Opaque::new(dai_drivers(&ops, !data.has_lpai)), GFP_KERNEL)?;
        let volume = KBox::new(
            Opaque::new(bindings::soc_mixer_control {
                min: 0,
                max: SPKR_VOLUME_MAX as i32,
                platform_max: SPKR_VOLUME_MAX as i32,
                reg: REG_SPKR_VOLUME as i32,
                rreg: REG_SPKR_VOLUME as i32,
                ..Default::default()
            }),
            GFP_KERNEL,
        )?;
        // SOC_SINGLE_TLV("Speaker Playback Volume", REG_SPKR_VOLUME, 0, 127, 0, tlv)
        let controls = KBox::new(
            [bindings::snd_kcontrol_new {
                iface: bindings::BINDINGS_SNDRV_CTL_ELEM_IFACE_MIXER,
                name: c"Speaker Playback Volume".as_char_ptr(),
                access: bindings::SNDRV_CTL_ELEM_ACCESS_TLV_READ
                    | bindings::SNDRV_CTL_ELEM_ACCESS_READWRITE,
                info: Some(bindings::snd_soc_info_volsw),
                get: Some(bindings::snd_soc_get_volsw),
                put: Some(bindings::snd_soc_put_volsw),
                tlv: bindings::snd_kcontrol_new__bindgen_ty_1 {
                    p: SPKR_VOLUME_TLV.as_ptr(),
                },
                private_value: volume.get() as usize,
                ..Default::default()
            }],
            GFP_KERNEL,
        )?;
        let component = bindings::snd_soc_component_driver {
            name: c"t8140-aop-audio".as_char_ptr(),
            read: Some(asoc_component_read),
            write: Some(asoc_component_write),
            controls: controls.as_ptr(),
            num_controls: 1,
            open: Some(asoc_pcm_open),
            close: Some(asoc_pcm_close),
            hw_params: Some(asoc_pcm_hw_params),
            prepare: Some(asoc_pcm_prepare),
            trigger: Some(asoc_pcm_trigger),
            sync_stop: Some(asoc_pcm_sync_stop),
            pointer: Some(asoc_pcm_pointer),
            copy: Some(asoc_pcm_copy),
            ..Default::default()
        };
        Ok(KBox::new(
            Self {
                component,
                data,
                dais,
                _ops: ops,
                _controls: controls,
                _volume: volume,
            },
            GFP_KERNEL,
        )?)
    }
}

struct SndSocT8140AopDriver {
    card: *mut bindings::snd_card,
    data: Arc<SndSocT8140AopData>,
    asoc: KBox<AsocContext>,
    asoc_registered: bool,
    lid_registered: bool,
    closed: AtomicFlag,
}

// SAFETY: the raw card pointer is only used from probe, unbind and drop,
// which the driver core serializes; the card's own state is guarded by
// ALSA.  Everything else is `Send + Sync` on its own.
unsafe impl Send for SndSocT8140AopDriver {}
// SAFETY: as for `Send`.
unsafe impl Sync for SndSocT8140AopDriver {}

impl SndSocT8140AopDriver {
    const LPAI_OPS: bindings::snd_pcm_ops = bindings::snd_pcm_ops {
        open: Some(lpai_pcm_open),
        close: Some(lpai_pcm_close),
        prepare: Some(lpai_pcm_prepare),
        trigger: Some(lpai_pcm_trigger),
        pointer: Some(lpai_pcm_pointer),
        ioctl: None,
        hw_params: None,
        hw_free: None,
        sync_stop: Some(lpai_pcm_sync_stop),
        get_time_info: None,
        fill_silence: None,
        copy: None,
        page: None,
        mmap: None,
        ack: None,
    };

    const HPAI_OPS: bindings::snd_pcm_ops = bindings::snd_pcm_ops {
        open: Some(hpai_pcm_open),
        close: Some(hpai_pcm_close),
        prepare: Some(hpai_pcm_prepare),
        trigger: Some(hpai_pcm_trigger),
        pointer: Some(bindings::snd_dmaengine_pcm_pointer),
        ioctl: None,
        hw_params: Some(hpai_pcm_hw_params),
        hw_free: None,
        sync_stop: Some(hpai_pcm_sync_stop),
        get_time_info: None,
        fill_silence: None,
        copy: None,
        page: None,
        mmap: None,
        ack: None,
    };

    fn new(data: Arc<SndSocT8140AopData>, fwnode: &FwNode) -> Result<Self> {
        // From here on a failure drops `this`, whose teardown undoes
        // whatever was set up.
        let mut this = SndSocT8140AopDriver {
            card: ptr::null_mut(),
            asoc: AsocContext::new(data.clone())?,
            data: data.clone(),
            asoc_registered: false,
            lid_registered: false,
            closed: AtomicFlag::new(false),
        };

        if data.has_lpai {
            // The low-power microphone: its ring is reported to us by the
            // firmware for the AOP's lifetime.
            data.attach_device(AUDIO_DEV_LPAI, "lpai")?;
            data.set_channel_control()?;
            data.read_geometry()?;
            let listener: Arc<dyn ReportListener> = data.clone();
            data.adata
                .add_report_listener(data.service, EPIC_SUBTYPE_PRODUCER_REPORT, listener)?;

            // SAFETY: FFI call with the live device and the module; `this.card` receives the new card.
            let ret = unsafe {
                bindings::snd_card_new(
                    data.dev.as_raw(),
                    -1,
                    ptr::null(),
                    THIS_MODULE.as_ptr(),
                    0,
                    &mut this.card,
                )
            };
            if ret < 0 {
                dev_err!(data.dev, "unable to allocate the sound card\n");
                return Err(Error::from_errno(ret));
            }
            // Named like the other AOP audio cards: Apple<chassis><interface>.
            let chassis = fwnode
                .property_read::<CString>(c_str!("apple,chassis-name"))
                .required_by(&data.dev)?;
            let machine_kind = fwnode
                .property_read::<CString>(c_str!("apple,machine-kind"))
                .required_by(&data.dev)?;
            let id_str = CString::try_from_fmt(fmt!("Apple{}AOP", *chassis))?;
            let shortname = CString::try_from_fmt(fmt!("{} {} AOP", *machine_kind, *chassis))?;
            let longname = CString::try_from_fmt(fmt!("{} {} AOP Audio", *machine_kind, *chassis))?;
            // SAFETY: `card` was created by `snd_card_new` above and is not
            // registered yet, so nothing else accesses it.
            let card = unsafe { &mut *this.card };
            let named = copy_str(&mut card.driver, c"t8140-aop-audio".to_bytes_with_nul())
                .and_then(|()| copy_str(&mut card.id, id_str.to_bytes_with_nul()))
                .and_then(|()| copy_str(&mut card.shortname, shortname.to_bytes_with_nul()))
                .and_then(|()| copy_str(&mut card.longname, longname.to_bytes_with_nul()));
            if let Err(e) = named {
                dev_err!(
                    data.dev,
                    "apple,chassis-name or apple,machine-kind too long for the card names\n"
                );
                return Err(e);
            }

            this.new_pcm(0, c"lpai", c"Low-Power Microphone", &Self::LPAI_OPS)?;
            this.new_pcm(1, c"hpai", c"High-Quality Microphone", &Self::HPAI_OPS)?;

            // Both microphones' jack: unplugged while a lid is closed.
            let mut jack = ptr::null_mut();
            // SAFETY: `card` was created by `snd_card_new` above and is not registered yet; the jack
            // becomes one of its devices and is freed with it.
            let ret = unsafe {
                bindings::snd_jack_new(
                    this.card,
                    c"Internal Mic".as_char_ptr(),
                    bindings::snd_jack_types_SND_JACK_MICROPHONE as i32,
                    &mut jack,
                    true,
                    // No input device of its own: the lid it follows already
                    // is one, and the control is what userspace reads.
                    true,
                )
            };
            if ret < 0 {
                dev_err!(data.dev, "unable to create the microphone jack\n");
                return Err(Error::from_errno(ret));
            }
            data.mic_jack.store(jack, Relaxed);
            this.register_lid_handler()?;
            // The state as the card is registered, from the only reporter.
            data.as_arc_borrow().queue_jack_report();
            data.flush_jack_report();

            // SAFETY: `card` was created by `snd_card_new` above and is not registered yet.
            let ret = unsafe { bindings::snd_card_register(this.card) };
            if ret < 0 {
                dev_err!(data.dev, "unable to register the sound card\n");
                return Err(Error::from_errno(ret));
            }
        }

        // The ASoC side: the jack (cout) and speaker back-ends, the
        // front-end PCMs and the speaker volume control.
        // SAFETY: the descriptor, the DAI table and the controls are heap
        // allocations owned by `this.asoc`, which outlives the component.
        let ret = unsafe {
            bindings::snd_soc_register_component(
                data.dev.as_raw(),
                &this.asoc.component,
                this.asoc.dais.get().cast(),
                DAI_COUNT as i32,
            )
        };
        if ret < 0 {
            dev_err!(data.dev, "unable to register the ASoC component: {}\n", ret);
            return Err(Error::from_errno(ret));
        }
        this.asoc_registered = true;
        Ok(this)
    }

    /// Follow the lid switches: the input core connects the handler to every
    /// device with one, now and as they come.  The handler's private data is
    /// an `Arc` of the driver data, reclaimed in `teardown`.
    fn register_lid_handler(&mut self) -> Result<()> {
        let handler = self.data.lid_handler.0.get();
        // SAFETY: the handler lives in the driver data, which this driver
        // keeps allocated until after unregistering it, and is not
        // registered yet.
        unsafe {
            (*handler).private = self.data.clone().into_foreign().cast();
            (*handler).event = Some(lid_event);
            (*handler).connect = Some(lid_connect);
            (*handler).disconnect = Some(lid_disconnect);
            (*handler).start = Some(lid_start);
            (*handler).passive_observer = true;
            (*handler).name = c"t8140-aop-audio".as_char_ptr();
            (*handler).id_table = LID_IDS.as_ptr();
        }
        // SAFETY: the handler is filled in above.
        let ret = unsafe { bindings::input_register_handler(handler) };
        if ret < 0 {
            // SAFETY: the `Arc` leaked above; the handler never registered.
            drop(unsafe { Arc::<SndSocT8140AopData>::from_foreign((*handler).private.cast()) });
            dev_err!(self.data.dev, "unable to follow the lid switch: {}\n", ret);
            return Err(Error::from_errno(ret));
        }
        self.lid_registered = true;
        Ok(())
    }

    /// One capture PCM of the card, with the driver data as its private
    /// data.
    fn new_pcm(
        &mut self,
        device: i32,
        id: &'static core::ffi::CStr,
        name: &'static core::ffi::CStr,
        ops: &'static bindings::snd_pcm_ops,
    ) -> Result<()> {
        let mut pcm = ptr::null_mut();
        // SAFETY: `card` was created by `snd_card_new`; `pcm` receives the new PCM.
        let ret =
            unsafe { bindings::snd_pcm_new(self.card, id.as_char_ptr(), device, 0, 1, &mut pcm) };
        if ret < 0 {
            dev_err!(self.data.dev, "unable to allocate PCM {}\n", device);
            return Err(Error::from_errno(ret));
        }
        // SAFETY: `pcm` was created by `snd_pcm_new` above; the ops are static and the private
        // data is an `Arc` released by `pcm_free_private`.
        unsafe {
            bindings::snd_pcm_set_ops(pcm, bindings::SNDRV_PCM_STREAM_CAPTURE as i32, ops);
            (*pcm).private_data = self.data.clone().into_foreign().cast();
            (*pcm).private_free = Some(pcm_free_private);
            (*pcm).info_flags = 0;
        }
        // SAFETY: as above.
        copy_str(unsafe { &mut (*pcm).name }, name.to_bytes_with_nul())?;
        if device == 0 {
            // The low-power microphone's buffer is filled by the driver from
            // the source ring; the array's is set up on its DMA device at
            // open.
            // SAFETY: `pcm` is live; a vmalloc buffer needs no device.
            unsafe {
                bindings::snd_pcm_set_managed_buffer_all(
                    pcm,
                    bindings::SNDRV_DMA_TYPE_VMALLOC as i32,
                    ptr::null_mut(),
                    LPAI_PERIOD_BYTES * 4,
                    LPAI_PERIOD_BYTES * LPAI_PERIODS_MAX as usize,
                );
            }
        }
        Ok(())
    }

    /// Undo probe: stop the reports, unregister the ASoC component and free
    /// the card (which closes every stream, releasing its channel and power
    /// references), then return lpai to idle.  Runs from unbind, and again
    /// (as a no-op) from Drop; also on a failed probe, with whatever had
    /// been set up.
    fn teardown(&self) {
        if self.closed.xchg(true, Relaxed) {
            return;
        }
        self.data.running.store(false, Relaxed);
        if self.lid_registered {
            let handler = self.data.lid_handler.0.get();
            // SAFETY: registered in `register_lid_handler`; unregistering
            // disconnects every handle, after which no callback runs.
            unsafe { bindings::input_unregister_handler(handler) };
            // SAFETY: the `Arc` leaked into the handler, unused from here on.
            drop(unsafe { Arc::<SndSocT8140AopData>::from_foreign((*handler).private.cast()) });
        }
        // No lid change can queue a jack report any more: let the last one
        // finish before the card, and the jack with it, goes away.
        self.data.flush_jack_report();
        self.data.mic_jack.store(ptr::null_mut(), Relaxed);
        // Stop reports before ALSA destroys any substream or module callback.
        self.data
            .adata
            .remove_report_listener(&self.data.service, EPIC_SUBTYPE_PRODUCER_REPORT);
        if self.asoc_registered {
            // SAFETY: the context remains owned until after unregister
            // completes PCM, DAI and control callbacks.
            unsafe {
                bindings::snd_soc_unregister_component_by_driver(
                    self.data.dev.as_raw(),
                    &self.asoc.component,
                )
            };
        }
        self.data.flush_spkr_start();
        self.data.flush_hpai_start();
        if !self.card.is_null() {
            // SAFETY: the card is freed once, while the binding remains live.
            unsafe { bindings::snd_card_free(self.card) };
        }
        // Every stream is closed now; retry an idle request that failed then.
        let mut run = self.data.lpai_run.lock();
        if run.running && self.data.set_power(POWER_STATE_IDLE).is_ok() {
            run.running = false;
        }
    }
}

impl Drop for SndSocT8140AopDriver {
    fn drop(&mut self) {
        self.teardown();
    }
}

/// # Safety
///
/// Called by ALSA with a live substream of this card's PCM, whose private
/// data is the driver's `Arc`.
unsafe extern "C" fn hpai_pcm_hw_params(
    substream: *mut bindings::snd_pcm_substream,
    params: *mut bindings::snd_pcm_hw_params,
) -> i32 {
    // SAFETY: the substream is live and its dmaengine runtime was set up by
    // `snd_dmaengine_pcm_open` in `hpai_pcm_open`.
    let chan = unsafe { bindings::snd_dmaengine_pcm_get_chan(substream) };
    let mut cfg = bindings::dma_slave_config::default();
    // SAFETY: the substream and the hardware parameters are live for the op.
    let rc = unsafe { bindings::snd_hwparams_to_dma_slave_config(substream, params, &mut cfg) };
    if rc < 0 {
        return rc;
    }
    // One stereo frame per burst.
    cfg.src_port_window_size = HPAI_CHANNELS;
    // SAFETY: `chan` is the live channel behind this substream's dmaengine runtime.
    unsafe {
        match (*(*chan).device).device_config {
            Some(f) => f(chan, &mut cfg),
            None => ENOSYS.to_errno(),
        }
    }
}

// The match data says whether the machine has the lpai microphone, its source
// ring and the AP-powered serializer leaves (T8140), or not (T8152).
kernel::of_device_table!(
    OF_TABLE,
    MODULE_OF_TABLE,
    <SndSocT8140AopDriver as platform::Driver>::IdInfo,
    [
        (of::DeviceId::new(c_str!("apple,t8140-aop-audio")), true),
        (of::DeviceId::new(c_str!("apple,t8152-aop-audio")), false),
    ]
);

impl platform::Driver for SndSocT8140AopDriver {
    type IdInfo = bool;
    type Data<'bound> = SndSocT8140AopDriver;

    const OF_ID_TABLE: Option<of::IdTable<Self::IdInfo>> = Some(&OF_TABLE);

    fn probe<'bound>(
        pdev: &'bound platform::Device<Core<'_>>,
        info: Option<&'bound Self::IdInfo>,
    ) -> impl PinInit<Self::Data<'bound>, Error> + 'bound {
        let has_lpai = *info.ok_or(ENODEV)?;
        let dev = ARef::<device::Device>::from(pdev.as_ref());
        // SAFETY: this is a service device the AOP core registered, being
        // probed here; the lookup defers the probe until the core has
        // published its driver data.
        let adata = unsafe { <dyn AOP>::from_child(pdev.as_ref()) }?;
        // SAFETY: the AOP core sets this child's platform data to the EPIC
        // service it created the child for; it lives as long as the child.
        let service = unsafe { (*dev.as_raw()).platform_data as *const EPICService };
        if service.is_null() {
            return Err(ENODEV);
        }
        // SAFETY: the pointer was checked above and stays valid with the child.
        let svc = unsafe { *service };

        // The source ring: 512 KiB through our own IOMMU stream (dart-aop 9),
        // owned by the AOP driver because the firmware binds it once per boot.
        // SAFETY: no DMA is in flight on this device yet.
        unsafe { pdev.dma_set_mask_and_coherent(DmaMask::new::<42>())? };
        let ring = if has_lpai {
            Some(adata.source_ring(pdev.as_ref(), RING_BYTES)?)
        } else {
            None
        };

        // The serializer leaves.  A missing one only disables the paths that
        // need it; those fail to open with ENODEV.  T8152 has none: the
        // firmware powers its own serializers.
        let mut pds: [Option<PowerDomain>; 5] = [None, None, None, None, None];
        let leaves: &[(&'static str, &'static core::ffi::CStr)] = if has_lpai {
            &[
                ("mca1", c"mca1"),
                ("mca0", c"mca0"),
                ("tx0", c"tx0"),
                ("leap mca", c"mca"),
                ("rx0", c"rx0"),
            ]
        } else {
            &[]
        };
        for (slot, (name, dt_name)) in pds.iter_mut().zip(leaves.iter().copied()) {
            match PowerDomain::attach(&dev, name, dt_name) {
                Ok(Some(pd)) => *slot = Some(pd),
                Ok(None) => dev_warn!(dev, "no {} power domain\n", name),
                Err(e) => dev_warn!(dev, "{} power domain attach failed: {:?}\n", name, e),
            }
        }
        let [pd_mca1, pd_mca0, pd_tx0, pd_leap_mca, pd_rx0] = pds;

        let fwnode = pdev.as_ref().fwnode().ok_or(ENOENT)?;
        let data = Arc::pin_init(
            pin_init!(SndSocT8140AopData {
                dev,
                adata,
                service: svc,
                ring,
                has_lpai,
                ring_bytes: Atomic::new(0),
                sequence: Atomic::new(0),
                powered: AtomicFlag::new(false),
                reported: AtomicFlag::new(false),
                running: AtomicFlag::new(false),
                hw_ptr_frames: Atomic::new(0),
                report_warnings: [const { AtomicFlag::new(false) }; 5],
                cout: Service::new(AUDIO_DEV_COUT, "cout"),
                cin: Service::new(AUDIO_DEV_CIN, "cin"),
                spkr: Service::new(AUDIO_DEV_SPKR, "spkr"),
                tap: Service::new(AUDIO_DEV_TAP, "tap"),
                hpai: Service::new(AUDIO_DEV_HPAI, "hpai"),
                pd_mca1,
                pd_mca0,
                pd_tx0,
                pd_leap_mca,
                pd_rx0,
                spkr_want_run: AtomicFlag::new(false),
                hpai_want_run: AtomicFlag::new(false),
                spkr_volume: Atomic::new(SPKR_VOLUME_MAX),
                spkr_start_work <- new_work!("SndSocT8140AopData::spkr_start_work"),
                hpai_start_work <- new_work!("SndSocT8140AopData::hpai_start_work"),
                lid: LidState::new(),
                lid_handler: LidHandler(Opaque::zeroed()),
                mic_jack: Atomic::new(ptr::null_mut()),
                jack_work <- new_work!("SndSocT8140AopData::jack_work"),
                stream <- new_spinlock!(StreamState {
                    substream: ptr::null_mut(),
                    copied: None,
                    hw_ptr_bytes: 0,
                    period_acc: 0,
                    buffer_bytes: 0,
                }),
                lpai_run <- new_mutex!(LpaiRun {
                    users: 0,
                    running: false,
                }),
                spkr_thermal <- new_spinlock!(SpkThermal::new(spk_thermal::CEILING_Q30)),
            }),
            GFP_KERNEL,
        )?;
        Self::new(data, fwnode)
    }

    fn unbind(_dev: &platform::Device<Core<'_>>, this: Pin<&Self>) {
        this.teardown();
    }
}

module_platform_driver! {
    type: SndSocT8140AopDriver,
    name: "snd_soc_t8140_aop_audio",
    description: "Apple T8140 AOP audio: microphones, headphone jack and speakers",
    license: "Dual MIT/GPL",
}

/// The speaker ceiling and the copy-path conversion are the whole digital
/// protection of the J700 speakers below the daemon: check them where they
/// are built, without the hardware.
#[kunit_tests(snd_soc_t8140_aop_audio_speaker)]
mod tests {
    use super::*;

    /// macaudio's level for the J700 while no daemon holds the lock:
    /// MAX98360A_DB(safe_vol = -20).
    const LOCKED_VOLUME: usize = 87;
    /// 10^(0.5 / 20) in parts per 10^9.
    const HALF_DB_PPB: u128 = 1_059_253_725;
    /// 0.5 as a float32: the amplifier's full scale on the wire.
    const F32_HALF: u32 = 0x3f00_0000;

    #[test]
    fn the_top_of_the_volume_range_is_the_amplifier_full_scale() {
        assert_eq!(
            i64::from(SPKR_VOLUME_Q31[SPKR_VOLUME_MAX as usize]),
            SPKR_CEILING_Q31
        );
        assert_eq!(SPKR_CEILING_Q31, 1 << 30);
        assert_eq!(f32_bits_of_q31(SPKR_CEILING_Q31 as i32), F32_HALF);
    }

    #[test]
    fn the_volume_table_falls_in_half_db_steps_to_mute() {
        assert_eq!(SPKR_VOLUME_Q31[0], 0);
        for i in 2..=SPKR_VOLUME_MAX as usize {
            let (lo, hi) = (
                u128::from(SPKR_VOLUME_Q31[i - 1]),
                u128::from(SPKR_VOLUME_Q31[i]),
            );
            assert!(hi > lo);
            // hi / lo == 10^(0.5/20) to within the table's rounding
            let expect = lo * HALF_DB_PPB / 1_000_000_000;
            assert!(hi.abs_diff(expect) <= 2);
        }
    }

    #[test]
    fn the_locked_level_is_20_db_below_full_scale() {
        let locked = i64::from(SPKR_VOLUME_Q31[LOCKED_VOLUME]);
        // 10 * locked == ceiling to within the table's rounding (0.1 ppm)
        assert!((10 * locked - SPKR_CEILING_Q31).abs() <= 10);
    }

    #[test]
    fn no_sample_at_any_volume_exceeds_the_full_scale() {
        for &gain in SPKR_VOLUME_Q31.iter() {
            for sample in [
                i32::MAX,
                i32::MIN,
                i32::MAX - 1,
                i32::MIN + 1,
                1 << 30,
                -(1 << 30),
                1,
                -1,
                0,
            ] {
                let bits = spkr_f32_bits(sample, gain);
                assert!(bits & 0x7fff_ffff <= F32_HALF);
            }
        }
        // the extremes at full volume: just under and exactly at full scale
        assert_eq!(
            spkr_f32_bits(i32::MAX, SPKR_CEILING_Q31 as u32),
            0x3eff_ffff
        );
        assert_eq!(
            spkr_f32_bits(i32::MIN, SPKR_CEILING_Q31 as u32),
            0x8000_0000 | F32_HALF
        );
        // and 20 dB down at the locked level
        let locked = spkr_f32_bits(i32::MIN, SPKR_VOLUME_Q31[LOCKED_VOLUME]);
        // 0.05 as a float32 is 0x3d4c_cccd; the conversion truncates
        assert!(locked & 0x7fff_ffff <= 0x3d4c_cccd && locked & 0x7fff_ffff > 0x3d4c_cccd - 64);
    }

    #[test]
    fn the_float_conversion_is_exact_on_powers_of_two() {
        assert_eq!(f32_bits_of_q31(0), 0);
        assert_eq!(f32_bits_of_q31(1 << 29), 0x3e80_0000); // 0.25
        assert_eq!(f32_bits_of_q31(-(1 << 30)), 0xbf00_0000); // -0.5
        assert_eq!(f32_bits_of_q31(1), 0x3000_0000); // 2^-31
        assert_eq!(f32_bits_of_q31(i32::MIN), 0xbf80_0000); // -1.0
    }
}

/// The card and PCM names are copied into ALSA's fixed-size fields from
/// device-tree strings.
#[kunit_tests(snd_soc_t8140_aop_audio_names)]
mod name_tests {
    use super::*;

    #[test]
    fn a_name_that_fits_is_copied_with_its_nul() {
        let mut field = [0xffu8; 16];
        assert!(copy_str(&mut field, c"AppleJ700AOP".to_bytes_with_nul()).is_ok());
        assert_eq!(&field[..13], b"AppleJ700AOP\0");
    }

    #[test]
    fn a_name_longer_than_its_field_is_refused() {
        let mut field = [0u8; 16];
        let long = c"AppleJ700LongerChassisAOP".to_bytes_with_nul();
        assert!(copy_str(&mut field, long).is_err());
        assert_eq!(field, [0u8; 16]);
    }
}

#[kunit_tests(snd_soc_t8140_aop_audio_parser)]
mod parser_tests {
    use super::*;

    #[test]
    fn short_reply_fields_are_rejected() {
        let data = [0u8; REPORT_MIN_LEN];
        for len in 0..4 {
            assert_eq!(le_u32(&data[..len], 0), Err(EINVAL));
        }
        for len in 0..8 {
            assert_eq!(le_u64(&data[..len], 0), Err(EINVAL));
        }
        for len in 0..REPORT_MIN_LEN {
            assert_eq!(le_u64(&data[..len], 0x60), Err(EINVAL));
        }
    }

    #[test]
    fn unaligned_fields_at_the_end_of_a_reply_are_decoded() {
        let data = [0xff, 1, 2, 3, 4, 5, 6, 7, 8];
        assert_eq!(le_u32(&data, 5), Ok(0x0807_0605));
        assert_eq!(le_u64(&data, 1), Ok(0x0807_0605_0403_0201));
        assert_eq!(le_u32(&data, 6), Err(EINVAL));
        assert_eq!(le_u64(&data, 2), Err(EINVAL));
    }

    fn producer_report(count: u64, cursor: u64) -> [u8; REPORT_MIN_LEN] {
        let mut report = [0u8; REPORT_MIN_LEN];
        report[0x40..0x48].copy_from_slice(&count.to_le_bytes());
        report[0x60..0x68].copy_from_slice(&cursor.to_le_bytes());
        report
    }

    #[test]
    fn short_producer_reports_are_rejected() {
        let report = producer_report(3199, LPAI_PERIOD_BYTES as u64);
        for len in 0..REPORT_MIN_LEN {
            assert_eq!(
                producer_position(&report[..len]),
                Err(ProducerReportError::Short)
            );
        }
        assert_eq!(
            producer_position(&report),
            Ok((LPAI_PERIOD_BYTES as u64, LPAI_PERIOD_BYTES as u64))
        );
    }

    #[test]
    fn overflowing_producer_positions_are_rejected() {
        for count in [u64::MAX, u64::MAX / LPAI_FRAME_BYTES as u64] {
            assert_eq!(
                producer_position(&producer_report(count, 0)),
                Err(ProducerReportError::Overflow)
            );
        }
        let count = u64::MAX / LPAI_FRAME_BYTES as u64 - 1;
        assert_eq!(
            producer_position(&producer_report(count, 0)),
            Ok(((count + 1) * LPAI_FRAME_BYTES as u64, 0))
        );
    }

    #[test]
    fn producer_cursors_must_match_the_owned_ring() {
        assert!(producer_cursor_matches(RING_BYTES as u64, 0, RING_BYTES));
        assert!(producer_cursor_matches(
            RING_BYTES as u64 + 8,
            8,
            RING_BYTES
        ));
        assert!(!producer_cursor_matches(8, 8, 0));
        assert!(!producer_cursor_matches(
            8,
            RING_BYTES as u64 + 8,
            RING_BYTES
        ));
        assert!(!producer_cursor_matches(8, u64::MAX, RING_BYTES));
        assert!(!producer_cursor_matches(8, 7, RING_BYTES));
    }
}

/// The lid bookkeeping behind the microphone jack.
#[kunit_tests(snd_soc_t8140_aop_audio_lid)]
mod lid_tests {
    use super::*;

    fn handle() -> LidHandle {
        LidHandle {
            handle: Opaque::zeroed(),
            closed: AtomicFlag::new(false),
            open: AtomicFlag::new(false),
        }
    }

    fn opened() -> LidHandle {
        let h = handle();
        h.open.store(true, Relaxed);
        h
    }

    #[test]
    fn a_closed_lid_unplugs_the_microphone_jack() {
        let lid = LidState::new();
        let switch = opened();
        assert!(!lid.mic_cut());
        assert_ne!(mic_jack_status(false), 0);

        assert!(switch.record(&lid, true));
        assert!(lid.mic_cut());
        assert_eq!(mic_jack_status(lid.mic_cut()), 0);

        assert!(switch.record(&lid, false));
        assert!(!lid.mic_cut());
    }

    #[test]
    fn a_repeated_switch_state_changes_nothing() {
        let lid = LidState::new();
        let switch = opened();
        // the reading at connect, then the same state again from an event
        assert!(!switch.record(&lid, false));
        assert!(switch.record(&lid, true));
        assert!(!switch.record(&lid, true));
        assert!(lid.mic_cut());
        assert!(switch.record(&lid, false));
        assert!(!switch.record(&lid, false));
        assert!(!lid.mic_cut());
    }

    #[test]
    fn any_closed_lid_switch_cuts_the_microphones() {
        let lid = LidState::new();
        let (a, b) = (opened(), opened());
        a.record(&lid, true);
        b.record(&lid, true);
        a.record(&lid, false);
        assert!(lid.mic_cut());
        b.record(&lid, false);
        assert!(!lid.mic_cut());
    }

    #[test]
    fn a_lid_switch_that_goes_away_closed_restores_the_microphones() {
        let lid = LidState::new();
        let switch = opened();
        switch.record(&lid, true);
        assert!(switch.forget(&lid));
        assert!(!lid.mic_cut());
        assert_eq!(lid.closed.load(Relaxed), 0);
        // nothing reaches a forgotten handle any more
        assert!(!switch.record(&lid, true));
        assert!(!lid.mic_cut());
    }

    /// Re-review finding 2: `lid_start()` runs from `input_register_handle()`
    /// before the open; a handle whose open then fails is freed without
    /// `lid_disconnect()`, so its closed reading must never have counted.  A
    /// later handle for the same device that opens counts normally.
    #[test]
    fn a_failed_open_leaves_no_closed_switch_behind() {
        let lid = LidState::new();
        let failed = handle();
        // lid_start() during input_register_handle(), switch closed
        assert!(!failed.record(&lid, true));
        // input_open_device() fails: unregistered and freed, no disconnect
        drop(failed);
        assert!(!lid.mic_cut());
        assert_eq!(lid.closed.load(Relaxed), 0);

        let attached = handle();
        // lid_start() during registration, before the open
        assert!(!attached.sync(&lid, true, false));
        // opened: lid_connect() enables and reads the switch in one step
        assert!(attached.sync(&lid, true, true));
        assert!(lid.mic_cut());
        assert!(attached.forget(&lid));
        assert!(!lid.mic_cut());
    }

    /// Review of v3, minor 3: an initially closed switch must count once the
    /// handle is open.  `lid_connect()` enables readings and reads the switch
    /// in the same step, in that order; reading first would drop the reading
    /// and leave the jack plugged until the next lid event.
    #[test]
    fn a_connect_enables_readings_before_reading_the_switch() {
        let lid = LidState::new();
        let switch = handle();
        assert!(switch.sync(&lid, true, true));
        assert!(lid.mic_cut());
        // the start() from a grab release reads without enabling anything
        let other = handle();
        assert!(!other.sync(&lid, true, false));
        assert_eq!(lid.closed.load(Relaxed), 1);
    }

    /// Events delivered only to a client that grabbed the device never reach
    /// the handler; the resynchronization on release reads the device's
    /// switch and repairs the state, in both directions.
    #[test]
    fn a_resync_after_missed_events_repairs_the_state() {
        let lid = LidState::new();
        let switch = opened();
        switch.record(&lid, true);
        // grabbed: the lid opens unseen; on release the bitmap reads open
        assert!(switch.record(&lid, false));
        assert!(!lid.mic_cut());
        // grabbed: the lid closes unseen; on release the bitmap reads closed
        assert!(switch.record(&lid, true));
        assert!(lid.mic_cut());
    }
}

#[kunit_tests(snd_soc_t8140_aop_audio_thermal)]
mod thermal_tests {
    use super::spk_thermal::{Highpass, SpkThermal, CEILING_0DB_Q30, CEILING_M20_Q30, CEILING_Q30};

    /// The 100 Hz square wave at the I2S full scale, 240 words per half period.
    fn square(i: usize) -> i32 {
        if (i / 240) % 2 == 0 {
            0x4000_0000
        } else {
            -0x4000_0000
        }
    }

    fn run_square(l: &mut SpkThermal, blocks: usize) -> i32 {
        let mut words = [0i32; 128];
        let mut peak = 0i32;
        for b in 0..blocks {
            for (k, w) in words.iter_mut().enumerate() {
                *w = square(b * 128 + k);
            }
            l.block(&mut words);
            for w in words.iter() {
                peak = peak.max(w.abs());
            }
        }
        peak
    }

    /// The same numbers as the host model of the J873 speaker limiter (ten
    /// seconds of a full-scale square wave, then two seconds of silence).
    #[test]
    fn ten_seconds_of_full_scale_square_match_the_host_model() {
        let mut l = SpkThermal::new(CEILING_0DB_Q30);
        let peak = run_square(&mut l, 3750);
        assert_eq!(peak, 1_073_741_824);
        assert_eq!(
            l.state(),
            (
                152358436004836,
                102381555896261,
                152358436004836,
                102381555896261,
                206396213
            )
        );
        let silence = [0i32; 128];
        for _ in 0..750 {
            let mut w = silence;
            l.block(&mut w);
        }
        assert_eq!(
            l.state(),
            (
                130024328395229,
                101932305864922,
                135521886534109,
                102381555896261,
                226457860
            )
        );
    }

    /// Peak of the high-pass output for a sine of peak 2^28 made by an integer
    /// oscillator (`next = 2 cos(w) cur - prev`, Q30): `c2` is 2 cos(w) and
    /// `s1` the first sample; the peak is taken over the last two periods
    /// after the start-up has died out.
    fn highpass_peak(c2: i64, s1: i64, period: usize) -> i64 {
        let mut hp = Highpass::default();
        let (mut prev, mut cur) = (0i64, s1);
        let mut peak = 0i64;
        for n in 0..8000usize {
            let y = i64::from(hp.step(prev as i32));
            if n >= 8000 - 2 * period {
                peak = peak.max(y.abs());
            }
            let next = ((c2 * cur) >> 30) - prev;
            prev = cur;
            cur = next;
        }
        peak
    }

    /// Amplitude relative to the input peak 2^28, in 1/10000: the ideal
    /// Butterworth magnitudes are 0.2425, 0.7071, 0.9701 and 0.9987.
    fn highpass_gain(c2: i64, s1: i64, period: usize) -> i64 {
        highpass_peak(c2, s1, period) * 10_000 / (1 << 28)
    }

    #[test]
    fn the_highpass_follows_a_100_hz_butterworth() {
        // 50, 100, 200 and 440 Hz at 48 kHz
        assert!((highpass_gain(2_147_437_652, 1_756_893, 960) - 2_425).abs() <= 100);
        assert!((highpass_gain(2_147_299_668, 3_513_712, 480) - 7_071).abs() <= 100);
        assert!((highpass_gain(2_146_747_759, 7_026_821, 240) - 9_701).abs() <= 100);
        assert!((highpass_gain(2_143_922_726, 15_452_226, 109) - 9_987).abs() <= 100);
    }

    #[test]
    fn the_highpass_blocks_a_constant_and_holds_full_scale() {
        let mut hp = Highpass::default();
        let mut last = 0i32;
        for _ in 0..300_000 {
            last = hp.step(1 << 20);
        }
        assert!(i64::from(last).abs() < 1 << 15);
        // a full-scale square wave at 120 Hz: the overshoot is held to full scale
        let mut hp = Highpass::default();
        let mut peak = 0i32;
        for i in 0..40_000usize {
            let x = if (i / 200) % 2 == 0 { 1 << 30 } else { -(1 << 30) };
            peak = peak.max(hp.step(x).abs());
        }
        assert_eq!(peak, 1 << 30);
    }

    /// A full-scale 440 Hz sine (peak 2^30, integer oscillator) through the
    /// high-pass: its linear peak is 0.9987 of full scale, so no sample may
    /// sit on the clamp (a clamped feedback latched it into clipping).
    #[test]
    fn a_full_scale_sine_does_not_latch_the_highpass_into_clipping() {
        let mut hp = Highpass::default();
        let (mut prev, mut cur) = (0i64, 61_808_904i64);
        let mut clamped = 0;
        for n in 0..48_000usize {
            let y = hp.step(prev as i32);
            if n >= 48_000 - 3_000 && y.abs() == 1 << 30 {
                clamped += 1;
            }
            let next = ((2_143_922_726i64 * cur) >> 30) - prev;
            prev = cur;
            cur = next;
        }
        assert_eq!(clamped, 0);
    }

    #[test]
    fn the_coil_stays_below_the_limit_with_a_full_scale_square() {
        let mut l = SpkThermal::new(CEILING_0DB_Q30);
        run_square(&mut l, 3750 * 6);
        // 140 C in Q40
        assert!(l.state().0 < 140i64 << 40);
    }

    /// At the speaker ceiling a full-scale square wave stays far below the
    /// sustained power (0.19 W against 0.9 W at 11.9 V per full scale): the
    /// thermal gain never moves, the ceiling is a pure peak limit.
    #[test]
    fn the_speaker_ceiling_is_a_peak_limit_below_the_sustained_level() {
        let mut l = SpkThermal::new(CEILING_Q30);
        let peak = run_square(&mut l, 3750 * 3);
        assert_eq!(i64::from(peak), CEILING_Q30);
        assert_eq!(l.state().4, 1 << 30);
    }

    #[test]
    fn idle_cooling_matches_the_host_model() {
        let mut l = SpkThermal::new(CEILING_0DB_Q30);
        l.cool_from(
            139i64 << 40,
            (50i64 << 40) + (((139i64 - 50) * 60 << 40) / 100),
            60_000,
        );
        assert_eq!(l.state().0, 99824966751432);
        assert_eq!(l.state().1, 99097783336276);
    }

    #[test]
    fn the_first_audible_ceiling_is_20_db_below_the_sustained_level() {
        let mut l = SpkThermal::new(CEILING_M20_Q30);
        let mut w = [0x4000_0000i32; 128];
        l.block(&mut w);
        assert_eq!(w[0], 18174652);
        assert_eq!(w[127], 18174652);
    }
}
