// SPDX-License-Identifier: GPL-2.0-only OR MIT
// Copyright 2026 Dj
// Copyright 2026 Ryan Murray
//! Driver for the Apple SEP (Secure Enclave Processor): attaches to a running
//! SEP over the AP mailbox, drives Touch ID enrol/verify via `/dev/sep-bio`,
//! and registers a hwrng and a SEP-backed `trusted` key source.
#![recursion_limit = "2048"]

#[cfg(not(CONFIG_OF_DYNAMIC))]
compile_error!("apple_sep requires CONFIG_OF_DYNAMIC: the SEP and DART nodes ship disabled and are enabled with a device-tree changeset");

#[macro_use]
mod ratelimit;
mod attach;
mod bio;
mod catacomb;
mod control;
mod der;
mod diag;
mod dt;
mod fv;
mod hibe;
mod hwrng;
mod image;
mod key;
mod keybag;
mod keybag_identity;
mod os_identity;
mod pm;
mod pnon;
mod profile;
mod proto;
mod refkey;
mod refkey_seal;
mod registration;
mod rxring;
mod sbio;
mod scrd;
mod sensor;
mod shim;
mod shmem;
mod skdl;
mod sks;
mod sse;
mod stac;
mod store;
mod transfer;
mod trusted;
mod userbag;
mod xarm;
mod xart_apfs;
mod xart_guard;
mod xart_store;

use kernel::{
    bindings,
    device,
    dma,
    dma::Device as _,
    driver,
    new_mutex,
    of,
    platform,
    prelude::*,
    soc::apple::mailbox::{
        MailCallback,
        Mailbox,
        Message, //
    },
    sync::{
        aref::ARef,
        atomic::{
            Acquire,
            Atomic,
            Relaxed,
            Release, //
        },
        barrier::{
            dma_mb,
            Read,
            Write, //
        },
        new_condvar,
        Arc,
        CondVar,
        CondVarTimeoutResult,
        Mutex,
        MutexGuard, //
    },
    time,
    types::ForeignOwnable,
    workqueue::{
        self,
        impl_has_delayed_work,
        impl_has_work,
        new_delayed_work,
        new_work,
        DelayedWork,
        Work,
        WorkItem, //
    }, //
};

const SETTLE_MS: time::Msecs = 200;
/// Messages on an endpoint without a handler are logged this many times per
/// boot; diag/unhandled_messages counts them all.
const UNHANDLED_LOG_LIMIT: u32 = 4;
/// Users whose last logged lock state is remembered (a Mac has the system
/// user 501 and one or two Linux users).
pub(crate) const LOCK_STATE_LOG_USERS: usize = 4;
const FIRST_RESPONSE_MS: time::Msecs = 3000;
const RNG_SURVEY_ATTEMPTS: u32 = 5;

const SETTLE_WORK_ID: u64 = 1;

const ENROL_WORK_ID: u64 = 2;
static_assert!(ENROL_WORK_ID != SETTLE_WORK_ID && ENROL_WORK_ID != 0);
const VERIFY_WORK_ID: u64 = 3;
static_assert!(VERIFY_WORK_ID != ENROL_WORK_ID && VERIFY_WORK_ID != SETTLE_WORK_ID);
static_assert!(VERIFY_WORK_ID != 0);
const LATE_ATTACH_WORK_ID: u64 = 4;
static_assert!(LATE_ATTACH_WORK_ID != VERIFY_WORK_ID && LATE_ATTACH_WORK_ID != ENROL_WORK_ID);
static_assert!(LATE_ATTACH_WORK_ID != SETTLE_WORK_ID && LATE_ATTACH_WORK_ID != 0);
/// The trusted-accessory power-state report; see `stac`.
const STAC_WORK_ID: u64 = 5;
static_assert!(STAC_WORK_ID != LATE_ATTACH_WORK_ID && STAC_WORK_ID != VERIFY_WORK_ID);
static_assert!(STAC_WORK_ID != ENROL_WORK_ID && STAC_WORK_ID != SETTLE_WORK_ID);
static_assert!(STAC_WORK_ID != 0);
/// The boot-policy digest read; see `pnon`.
const PNON_WORK_ID: u64 = 6;
static_assert!(PNON_WORK_ID != STAC_WORK_ID && PNON_WORK_ID != LATE_ATTACH_WORK_ID);
static_assert!(PNON_WORK_ID != VERIFY_WORK_ID && PNON_WORK_ID != ENROL_WORK_ID);
static_assert!(PNON_WORK_ID != SETTLE_WORK_ID && PNON_WORK_ID != 0);

// When an attach-time sensor bring-up falls short, one more round runs this
// long after Touch ID was activated. A round that cannot run yet, because the
// boot-time bring-up is still going or the system is going to sleep, is put
// off by the same delay, at most this many times.
const LATE_ATTACH_DELAY_MS: time::Msecs = 10_000;
const LATE_ATTACH_DEFERS: u32 = 3;

// The step in which a pause between bring-up attempts checks whether the
// result is still wanted.
const BRINGUP_PAUSE_STEP_MS: u32 = 100;

// J700 traces show the key-store endpoint arriving around 43 s after boot,
// well after the first infrastructure endpoints. The wait exits promptly on
// other machines when their key store becomes ready.
const EXCHANGE_TIMEOUT_MS: time::Msecs = 90000;

const PHASE_ATTACH: u32 = 0;
const PHASE_EXCHANGE: u32 = 1;
const PHASE_READY: u32 = 2;

const OOL_SIZE_XARM: usize = 0x8000;

const OOL_SIZE_SBIO: usize = 0x4000;

const OOL_SIZE_SCRD: usize = 0x4000;

const SBIO_TIMEOUT_MS: time::Msecs = 5000;

// How long going to sleep waits for an ended capture to stop: one enclave
// request in flight, plus the capture loop's interrupt-wait backstop.
const SLEEP_DRAIN_MS: u32 = 6000;
static_assert!(SLEEP_DRAIN_MS as u64 > SBIO_TIMEOUT_MS as u64);

// How long an open or a capture start waits for the last enrol or verify work
// item to stop once its operation has ended. It is stopped in the same way,
// so it gets the same bound as a capture draining before sleep.
const WORKER_EXIT_WAIT_MS: time::Msecs = SLEEP_DRAIN_MS;

const SKS_ALLOC: usize = 0x8000;

const SKS_SECRET_LEN: usize = 32;
const SKS_MAX_CAPTURE: usize = 8;

const SCRD_MAX_CAPTURE: usize = 8;

const SCRD_TIMEOUT_MS: time::Msecs = 2000;

const SKS_TIMEOUT_MS: time::Msecs = 2000;
/// The per-try wait for the 13.5 key store.
const SKS_TIMEOUT_13_MS: time::Msecs = 6000;

const SKS_TIMEOUT_PER_KIB_MS: time::Msecs = 6000;

const SKS_TIMEOUT_MAX_MS: time::Msecs = 30_000;

const SKS_MAX_ABANDONED: usize = 8;

const fn sks_timeout_for(image_len: usize) -> time::Msecs {
    let kib = image_len / 1024;
    let kib = if kib > u16::MAX as usize {
        u16::MAX as time::Msecs
    } else {
        kib as time::Msecs
    };
    let scaled = SKS_TIMEOUT_MS.saturating_add(SKS_TIMEOUT_PER_KIB_MS.saturating_mul(kib));
    if scaled > SKS_TIMEOUT_MAX_MS {
        SKS_TIMEOUT_MAX_MS
    } else {
        scaled
    }
}

static_assert!(sks_timeout_for(64) == SKS_TIMEOUT_MS);
static_assert!(sks_timeout_for(1508) > SKS_TIMEOUT_MS);
static_assert!(sks_timeout_for(usize::MAX) == SKS_TIMEOUT_MAX_MS);

const SKS_MAX_SET_ASIDE: u32 = 16;

const DMA_RING_SIZE: usize = 4 * (1 << 12);

const OOL_WRITE_POLL_MS: u32 = 1;
const OOL_WRITE_POLL_ATTEMPTS: u32 = 200;

const OOL_POISON_INBOUND: u8 = 0xA5;
const OOL_POISON_OUTBOUND: u8 = 0x5A;

// transfer::take_header() tells a landed SBIO header from the fill by its
// words, which a valid reply cannot hold as four equal nonzero bytes.
static_assert!(OOL_POISON_OUTBOUND != 0);

const PROTECTED_DATA_AVAILABLE: bool = true;

const RNG_MAX_WORDS_PER_READ: usize = 16;

// How long going to sleep waits for an entropy request in flight: the
// mailbox's 500 ms wait for room, then the wait for the reply. Only a request
// already being written can still go out once the sleep has begun; see
// `send_entropy_request()`.
const RNG_SLEEP_DRAIN_MS: u32 = proto::CONTROL_TIMEOUT_MS + 1000;

extern "C" {
    fn sep_cancel_work_sync(work: *mut c_void);
    fn sep_cancel_delayed_work_sync(work: *mut c_void);
}

const MAX_ENDPOINTS: usize = 64;

#[derive(Clone, Copy)]
struct Endpoint {
    id: u8,
}

impl Endpoint {
    fn new(id: u8) -> Self {
        Endpoint {
            id,
        }
    }
}

struct OolPair {
    endpoint: u8,
    allocated: usize,
    declared_in: usize,
    declared_out: usize,
    inbound: shmem::ShMem,
    outbound: shmem::ShMem,
    registered: bool,
}

fn sks_declared_sizes() -> (usize, usize) {
    (0x8000, 0x4000)
}

fn boottime_ms() -> u64 {
    shim::boottime_ns() / 1_000_000
}

struct Hex<'a>(&'a [u8]);

impl kernel::fmt::Display for Hex<'_> {
    fn fmt(&self, f: &mut kernel::fmt::Formatter<'_>) -> kernel::fmt::Result {
        use core::fmt::Write;
        const DIGITS: &[u8; 16] = b"0123456789abcdef";
        for (i, byte) in self.0.iter().enumerate() {
            if i > 0 {
                f.write_char(' ')?;
            }
            f.write_char(DIGITS[(*byte >> 4) as usize] as char)?;
            f.write_char(DIGITS[(*byte & 0x0f) as usize] as char)?;
        }
        Ok(())
    }
}

const SKS_LOCK_STATE_VARIANT: u32 = 1;

#[derive(Clone, Copy)]
enum LockState {
    Unlocked,
}

impl LockState {
    const fn wire(self) -> i32 {
        match self {
            LockState::Unlocked => 0,
        }
    }
}

static_assert!(LockState::Unlocked.wire() == 0);

const SKS_LOCK_STATE_FLAGS: u64 = 0;

enum SbioOutcome {
    Ok(KVec<u8>),
    // status 0x01
    PrerequisiteMissing,
    // status 0x16; NOT "malformed" — that mapping is another service's status space
    Status16,
    Other,
}

const BRINGUP_FRESH: u32 = registration::FRESH;
const BRINGUP_IDENTIFIED: u32 = registration::IDENTIFIED;
const BRINGUP_ESTABLISHED: u32 = registration::ESTABLISHED;

pub(crate) struct PatchLoaded(());

pub(crate) struct ParametersApplied(());

pub(crate) struct CalibrationBlob(kernel::firmware::Firmware);

impl CalibrationBlob {
    fn new(fw: kernel::firmware::Firmware) -> CalibrationBlob {
        CalibrationBlob(fw)
    }

    pub(crate) fn bytes(&self) -> &[u8] {
        self.0.data()
    }
}

struct EnrolMaterial {
    special: crate::sks::SpecialHandle,
    secret: Secret,
    snapshot_identity: Option<keybag::SnapshotIdentity>,
}

struct ImageContext<'a> {
    sep: &'a SepData,
}

impl Drop for ImageContext<'_> {
    fn drop(&mut self) {
        let op = crate::sbio::sbio_image_cleanup();
        let _ = self.sep.sbio_call(&op);
    }
}

struct OpenEnrolment<'a> {
    sep: &'a SepData,
    armed: bool,
}

impl OpenEnrolment<'_> {
    fn completed(mut self) {
        self.armed = false;
        self.sep.enrol_open.store(false, Relaxed);
    }
}

impl Drop for OpenEnrolment<'_> {
    fn drop(&mut self) {
        if !self.armed {
            return;
        }
        let _ = self.sep.sbio_call(&crate::sbio::sbio_cancel_operation());
        self.sep.enrol_open.store(false, Relaxed);
    }
}

enum ImageOutcome {
    // `complete` flag from 0xbfe
    Progress {
        stage: u32,
        percent: u32,
        complete: bool,
        has_template: bool,
    },
    // The capture did not count; the guidance says why.
    Retry(bio::Guidance),
    NoFinger,
    Failed(u32),
}

enum CaptureWait {
    Ready(u32),
    Timeout,
    Fault(u8),
    Abandon,
}

const ENROL_STATUS_SENSOR: u32 = 1;
const ENROL_STATUS_ENCLAVE: u32 = 2;
const ENROL_STATUS_TOO_MANY: u32 = 3;
const ENROL_STATUS_TIMEOUT: u32 = 4;
#[derive(Clone, Copy)]
enum LoadAnswer {
    StatusZero,
    ColdTransition,
    // status 0x101: SEP re-activated the component from its own xART records
    AlreadyActive,
}

impl LoadAnswer {
    fn describe(self) -> &'static CStr {
        match self {
            LoadAnswer::StatusZero => c"status 0",
            LoadAnswer::ColdTransition => c"the cold-transition status 0x8002",
            LoadAnswer::AlreadyActive => c"the already-active status 0x101",
        }
    }
}

#[derive(Debug)]
enum RestoreOutcome {
    Restored,
    AlreadyActive,
    NoStoredFile,
    Failed,
    // 0x8002 cold-transition; tolerated only for the lockout record, never a catacomb
    EmptyTolerated,
}

const PRIVATE_TYPE_CATACOMB_MASTER: u8 = 0xF2;
const PRIVATE_TYPE_CATACOMB_OWNER: u8 = 0xF3;
const PRIVATE_TYPE_CATACOMB_USER: u8 = 0xF4;
const PRIVATE_TYPE_LOCKOUT: u8 = 0xF5;
// 0xF1 is the identity index's type; a colliding type silently mis-restores
static_assert!(PRIVATE_TYPE_CATACOMB_MASTER != 0xF1);
static_assert!(PRIVATE_TYPE_CATACOMB_OWNER != PRIVATE_TYPE_CATACOMB_MASTER);
static_assert!(PRIVATE_TYPE_CATACOMB_USER != PRIVATE_TYPE_CATACOMB_OWNER);
static_assert!(PRIVATE_TYPE_CATACOMB_USER != PRIVATE_TYPE_CATACOMB_MASTER);
static_assert!(PRIVATE_TYPE_LOCKOUT != PRIVATE_TYPE_CATACOMB_USER);
static_assert!(PRIVATE_TYPE_LOCKOUT != PRIVATE_TYPE_CATACOMB_OWNER);
static_assert!(PRIVATE_TYPE_LOCKOUT != PRIVATE_TYPE_CATACOMB_MASTER);

const ENROL_STATUS_UNFILED: u32 = 6;
static_assert!(ENROL_STATUS_UNFILED != ENROL_STATUS_RETRY);
static_assert!(ENROL_STATUS_UNFILED != ENROL_STATUS_ENCLAVE);
static_assert!(ENROL_STATUS_UNFILED != ENROL_STATUS_SENSOR);
static_assert!(ENROL_STATUS_UNFILED != ENROL_STATUS_TIMEOUT);
static_assert!(ENROL_STATUS_UNFILED != ENROL_STATUS_TOO_MANY);

const ENROL_STATUS_RETRY: u32 = 5;
static_assert!(bio::ENROL_POLL_RETRY == ENROL_STATUS_RETRY);
// 7 is SEP_BIO_STATUS_REJECTED in libfprint's aurora driver.
// The enclave stopped matching after too many failed attempts. Only a
// password authentication, through /dev/sep-bio-rearm, makes it match again.
const ENROL_STATUS_LOCKOUT: u32 = 8;
static_assert!(ENROL_STATUS_LOCKOUT != ENROL_STATUS_UNFILED);
static_assert!(ENROL_STATUS_LOCKOUT != ENROL_STATUS_RETRY);
static_assert!(ENROL_STATUS_LOCKOUT != ENROL_STATUS_ENCLAVE);
static_assert!(ENROL_STATUS_RETRY != ENROL_STATUS_TIMEOUT);
static_assert!(ENROL_STATUS_RETRY != ENROL_STATUS_ENCLAVE);
static_assert!(ENROL_STATUS_RETRY != ENROL_STATUS_SENSOR);

// Captures per enrolment, including retries for partial contact and bad
// frames. One J414s has needed up to 22. bio::ENROL_STAGES is derived from
// this, so the stages cannot run out before the budget does.
const ENROL_MAX_CAPTURES: u32 = 36;

const ENROL_POLL_MS: u32 = 2;

// Interrupt-driven capture blocks on the data-ready line rather than polling
// SPI status. This is the maximum a single wait blocks before re-reading status
// as a backstop; the interrupt wakes it far sooner when a frame is ready.
const ENROL_IRQ_WAIT_MS: u32 = 250;

const ENROL_CAPTURE_TIMEOUT_MS: u32 = 60_000;
const ENROL_POLL_ATTEMPTS: u32 = ENROL_CAPTURE_TIMEOUT_MS / ENROL_POLL_MS;
static_assert!(ENROL_POLL_ATTEMPTS * ENROL_POLL_MS == ENROL_CAPTURE_TIMEOUT_MS);

// How long the next capture waits for the last touch's finger to be lifted
// before it arms anyway: as long as a capture waits for a finger.
const LIFT_WAIT_MS: u32 = ENROL_CAPTURE_TIMEOUT_MS;

// The backstop interval of that wait. The sensor's interrupt wakes it at the
// lift; this bounds a missed edge (macOS arms within 2 ms of the lift).
const LIFT_POLL_MS: u32 = 20;

const PATCH_POLL_MS: u32 = 20;
const PATCH_POLL_ATTEMPTS: u32 = 250;

const CALIBRATION_FIRMWARE: &CStr = c"apple/mesa_calibration.bin";
/// The signed root-key container on boards whose sensor node names only the
/// calibration (J700).
const ROOT_KEY_FIRMWARE: &CStr = c"apple/msrk-j700.bin";

const SBIO_PROBE_USER_ID: i32 = 1000;
static_assert!(SBIO_PROBE_USER_ID >= crate::sks::SKS_DESIGNATE_USER_MIN);
static_assert!(SBIO_PROBE_USER_ID == bio::ENROL_USER_ID);
static_assert!(SBIO_PROBE_USER_ID > 0);

const SKS_LOAD_REPLY_LEN: usize = 8;

#[derive(Clone, Copy)]

#[must_use]
struct Healthy(());

struct SksRequest {
    name: &'static CStr,
    msg: Message,
    img: image::RequestImage,
}

struct SksOutcome {
    reply: crate::sks::SksReply,
    response: Secret,
}

pub(crate) struct Secret(pub(crate) KVec<u8>);

impl Secret {
    pub(crate) fn empty() -> Secret {
        Secret(KVec::new())
    }
}

impl core::ops::Deref for Secret {
    type Target = [u8];

    fn deref(&self) -> &[u8] {
        &self.0
    }
}

impl Drop for Secret {
    fn drop(&mut self) {
        image::wipe(&mut self.0);
    }
}

#[derive(Clone, Copy)]
struct Abandoned {
    selector: u8,
    seq: u8,
    label: &'static CStr,
    waited_ms: u64,
}

struct SksProbe {
    active: bool,
    captured: KVec<Message>,
    abandoned: [Option<Abandoned>; SKS_MAX_ABANDONED],
    abandoned_next: usize,
}

impl SksProbe {
    fn new() -> Self {
        SksProbe {
            active: false,
            captured: KVec::new(),
            abandoned: [None; SKS_MAX_ABANDONED],
            abandoned_next: 0,
        }
    }
}

struct ScrdProbe {
    active: bool,
    captured: KVec<Message>,
}

impl ScrdProbe {
    fn new() -> Self {
        ScrdProbe {
            active: false,
            captured: KVec::new(),
        }
    }
}

struct XarmState {
    deferred_query: Option<u8>,
    os_uuid: Option<[u8; 16]>,
}

impl XarmState {
    fn new() -> Self {
        XarmState {
            deferred_query: None,
            os_uuid: None,
        }
    }
}

struct EndpointTable {
    eps: KVec<Endpoint>,
}

impl EndpointTable {
    fn new() -> Self {
        EndpointTable {
            eps: KVec::new(),
        }
    }

    fn slot(&mut self, id: u8) -> Result<usize> {
        if let Some(i) = self.eps.iter().position(|e| e.id == id) {
            return Ok(i);
        }
        if self.eps.len() >= MAX_ENDPOINTS {
            return Err(ENOSPC);
        }
        self.eps.push(Endpoint::new(id), GFP_KERNEL)?;
        Ok(self.eps.len() - 1)
    }
}


struct MachineRefKey {
    blob: KVec<u8>,
    pub_raw: KVec<u8>,
}

/// One mapped firmware reserved region, unmapped exactly once in `remove()`.
struct FirmwareMap {
    size: usize,
    iova: u64,
}

#[pin_data]
struct SepData {
    dev: ARef<device::Device>,

    profile: &'static profile::PlatformProfile,

    // Serialises completion, including concurrent sysfs writers and retries.
    #[pin]
    attach_phase: Mutex<attach::Phase>,
    keystore_only: Atomic<bool>,
    // keystore_only=2: /dev/sep-bio-rearm is published in the initrd phase
    // for the password-unlock subset of the per-user key bag requests.
    keystore_user_bags: bool,
    keystore_write_refusals: Atomic<u32>,
    #[pin]
    identity_keybag: Mutex<Option<keybag::StoredKeyBag>>,


    #[pin]
    fw_map: Mutex<Option<FirmwareMap>>,

    #[pin]
    mbox: Mutex<Option<Mailbox<SepData>>>,

    #[pin]
    shmem: Mutex<Option<shmem::ShMem>>,

    #[pin]
    endpoints: Mutex<EndpointTable>,
    #[pin]
    trusted_ready: CondVar,

    #[pin]
    control: Mutex<control::ControlState>,
    #[pin]
    control_wq: CondVar,

    sks_wedged: Atomic<u32>,

    sks_seq: Atomic<u32>,

    // IPC header version negotiated by the 13.5 endpoint init; 1 otherwise.
    sks_header_version: Atomic<u32>,

    // The 13.5 endpoint init (0x4d, set_env) completed.
    sks_initialized: Atomic<bool>,

    // u32, not u8: the kernel's Rust atomics have no u8 AtomicType
    phase: Atomic<u32>,

    // A `pm::SleepGate`.
    sleep_gate: Atomic<u32>,

    // The mailbox's receive interrupt is held off for the enclave's sleep.
    sleep_rx_held: Atomic<bool>,

    #[pin]
    ool_xarm: Mutex<Option<OolPair>>,

    #[pin]
    ool_sbio: Mutex<Option<OolPair>>,

    #[pin]
    dma_ring: Mutex<Option<shmem::ShMem>>,

    // One SBIO request/reply exchange at a time: the endpoint has a single
    // out-of-line buffer and `sbio_rx` tracks a single transfer.
    #[pin]
    sbio_exchange_lock: Mutex<()>,

    #[pin]
    sbio_rx: Mutex<transfer::Reassembly>,

    #[pin]
    sbio_wq: CondVar,

    sbio_ready: Atomic<bool>,

    templates_restored: Atomic<bool>,
    // Provisional cold restore; COMPLETE_INIT must still prove the device view.
    cold_restore_candidate: Atomic<bool>,

    sensor_calibrated: Atomic<bool>,
    sensor_key_loaded: Atomic<bool>,

    enrol_open: Atomic<bool>,

    // One warning each per boot: an xART write refused because writes are
    // off, and a Touch ID capture refused for the same reason.
    xart_refusal_logged: Atomic<bool>,
    capture_refusal_logged: Atomic<bool>,

    #[pin]
    enrol_material: Mutex<Option<EnrolMaterial>>,

    // The SCRD context whose handle every match carries, so the enclave
    // matches in the authenticated unlock scope of the Linux user. Replaced
    // by each successful SCRD verification of the designated bag (boot and
    // password re-arm); `match_context_generation` numbers them for the log.
    #[pin]
    match_context: Mutex<Option<[u8; crate::scrd::SCRD_ACM_HANDLE_LEN]>>,
    match_context_generation: Atomic<u32>,

    // Every live cold-match credential: the SCRD contexts that a flags-0
    // verification of the designated bag satisfied (two per boot: before and
    // after the Catacomb restore). Nothing deleted them, so a password re-arm
    // added a new one beside the old ones while the old ones stayed live, and
    // only a restart of the enclave dropped them. A re-arm now retires them
    // all before it sets up the new one, as a restart does.
    #[pin]
    cold_match_contexts: Mutex<KVec<[u8; crate::scrd::SCRD_ACM_HANDLE_LEN]>>,

    // The sensor stayed powered and registered after the last verify, so the
    // next verify captures without a new bring-up, as macOS does. Cleared by
    // anything that powers the sensor off or resets it.
    sensor_kept: Atomic<bool>,

    // The kept sensor reset itself under a verify capture (state 9, it asks
    // for its patch again). The verify reports a retry, ends the prompt's
    // match operation and drops the sensor, so the next try brings it up.
    sensor_reset_in_capture: Atomic<bool>,

    // The enclave announced (SBIO tag 0x03) that a background operation
    // result is ready; it waits for the host to collect it with 0x21.
    bg_ops_ready: Atomic<bool>,

    // A template update was announced and its user/master pair is not yet
    // persisted; matching stays off until it is.
    persistence_owed: Atomic<bool>,

    #[pin]
    enrol_identity_candidates: Mutex<KVec<[u8; bio::UUID_LEN]>>,

    // Boot-time clock when the sensor was paused after a touch whose finger
    // may still be down, or 0. The next capture is armed once the sensor
    // reports the finger lifted, as macOS does, instead of after a fixed pause.
    paused_at_ns: Atomic<u64>,

    // Boot-time clock at the start of the last sensor power cycle, for
    // reporting how far into a bring-up a step failed.
    sensor_power_ns: Atomic<u64>,

    device_view_synced: Atomic<bool>,

    keybag_designated: Atomic<bool>,

    bringup_started: Atomic<bool>,
    touchid_started: Atomic<bool>,
    touchid_failed: Atomic<bool>,

    bringup: Atomic<u32>,
    // The enclave accepted an identification of the sensor since it booted,
    // even if the registration it began then failed.
    sensor_held: Atomic<bool>,
    // The enclave may hold a serial registration whose session never opened;
    // see registration::authenticate.
    serial_unfinished: Atomic<bool>,
    sensor_test_fault_done: Atomic<bool>,

    // Activation left the sensor short of attached; the opener queues the
    // late attach. And how often that has been put off.
    late_attach_wanted: Atomic<bool>,
    late_attach_defers: Atomic<u32>,

    #[pin]
    ool_sks: Mutex<Option<OolPair>>,

    #[pin]
    sks_exchange_lock: Mutex<()>,
    // One SCRD request in flight at a time: the endpoint has one pair of
    // shared buffers and replies are told apart by request type only, so
    // two callers (a touch-gated /dev/sep-key call, the re-arm, a self-test)
    // would overwrite each other's payload or take each other's reply,
    // including a new credential context's handle.
    #[pin]
    scrd_exchange_lock: Mutex<()>,
    #[pin]
    sks_probe: Mutex<SksProbe>,
    #[pin]
    sks_wq: CondVar,

    #[pin]
    ool_scrd: Mutex<Option<OolPair>>,

    // The trusted-accessory endpoint (see `stac`): its buffers, its one
    // exchange, and what discovery said about it.
    #[pin]
    ool_stac: Mutex<Option<OolPair>>,
    #[pin]
    stac_rx: Mutex<stac::Exchange>,
    #[pin]
    stac_wq: CondVar,
    stac_named: Atomic<bool>,
    stac_layout: Atomic<u32>,
    stac_absent_logged: Atomic<bool>,

    // The boot-policy endpoint (see `pnon`): its buffers, its one
    // exchange, and what discovery said about it.
    #[pin]
    ool_pnon: Mutex<Option<OolPair>>,
    #[pin]
    pnon_rx: Mutex<pnon::Exchange>,
    #[pin]
    pnon_wq: CondVar,
    pnon_named: Atomic<bool>,
    pnon_layout: Atomic<u32>,
    pnon_done: Atomic<bool>,

    // What discovery said about the endpoints the driver checks and sends
    // nothing to (see `sse`, `hibe`, `skdl`).
    sse_named: Atomic<bool>,
    sse_layout: Atomic<u32>,
    hibe_named: Atomic<bool>,
    hibe_layout: Atomic<u32>,
    skdl_named: Atomic<bool>,
    skdl_layout: Atomic<u32>,
    #[pin]
    scrd_probe: Mutex<ScrdProbe>,
    #[pin]
    scrd_wq: CondVar,

    #[pin]
    bio_session: Mutex<bio::Session>,

    #[pin]
    bio_index: Mutex<bio::IdentityIndex>,

    #[pin]
    bio_dev: Mutex<Option<shim::BioChardev>>,

    // /dev/sep-bio-rearm: reports the Touch ID lockout and clears it after a
    // password authentication.
    #[pin]
    rearm_dev: Mutex<Option<shim::BioChardev>>,

    // /dev/sep-key: P-256 keys created inside the SEP (see `key`).
    #[pin]
    key_dev: Mutex<Option<shim::BioChardev>>,

    // /dev/sep-key: one identity-bag load, operation and unload at a time.
    // The device's ioctls run concurrently (a touch wait must not block
    // SEP_KEY_CANCEL), so this, not the chardev lock, keeps one caller from
    // unloading the bag under another's operation.
    #[pin]
    key_bag_lock: Mutex<()>,

    // /dev/sep-key: the signature a touch-gated SEP_KEY_SIGN is waiting for.
    // Set only for the length of that ioctl; while set, a verify run belongs
    // to it and not to /dev/sep-bio's session.
    #[pin]
    key_touch: Mutex<Option<crate::key::KeyTouch>>,
    key_touch_active: Atomic<bool>,
    /// `SEP_KEY_CANCEL`: ends the touch-gated operation that is waiting.
    key_touch_cancel: Atomic<bool>,
    /// An unconfirmed credential disposal disables private key operations.
    key_context_failed: Atomic<bool>,
    // The enclave status of the last /dev/sep-key signature (self-test).
    last_key_status: Atomic<u32>,

    // The enclave refused to match (MATCH_RESULT answered 0x1) after failed
    // attempts, or the lockout survived a restart. Every verify is refused
    // without a capture until a password authentication re-arms Touch ID.
    bio_lockout: Atomic<bool>,

    // Touch ID waits for the first password since this boot
    // (SEP_BIO_REQUIRE_PASSWORD): every verify is refused without a capture
    // until a password authentication re-arms it. Never recorded.
    bio_after_boot: Atomic<bool>,

    // The refusal of a verify during a lockout has been logged.
    bio_lockout_logged: Atomic<bool>,

    // Failed matches since the last match or re-arm, across prompts.
    bio_nomatch_streak: Atomic<u32>,

    // A password cleared the failed attempts while a capture held the sensor
    // lock; the enclave's own count is cleared at the next authentication.
    enclave_failures_owed: Atomic<bool>,

    // A saved enclave lockout record waits to be handed back (LOAD_LOCKOUT).
    // macOS loads it only after the user's first unlock, the sensor's
    // registration and the full Catacomb restore; sent earlier, the enclave
    // refuses it (0x8002). The first password authentication sends it.
    lockout_import_pending: Atomic<bool>,

    // Opens of the Touch ID device. One open is one prompt: fprintd claims
    // the device for one PAM authentication.
    bio_open_gen: Atomic<u64>,

    // The open whose prompt holds the live match operation and the unlock
    // evaluation in `match_context`, or 0 when none is open.
    match_scope_gen: Atomic<u64>,

    // Touches in that prompt so far.
    match_scope_touches: Atomic<u32>,

    // Failed matches in that prompt so far.
    match_scope_misses: Atomic<u32>,

    // The enclave's most recent biometric lock-state notification: the
    // state in bits 0-15 and the user in bits 16-47, or 0 before the first.
    // `bio_lock_state_seq` counts them, so a waiter can tell a fresh one
    // from a stale one.
    bio_lock_state: Atomic<u64>,
    bio_lock_state_seq: Atomic<u32>,
    // The last state logged for each of the most recent users, in the same
    // layout, so a repeated notification is not logged again. Notifications
    // are handled one at a time, on the receive path.
    bio_lock_logged: [Atomic<u64>; LOCK_STATE_LOG_USERS],

    #[pin]
    store: Mutex<Option<xart_store::Store>>,

    #[pin]
    host_store: Mutex<Option<store::Store>>,

    #[pin]
    xarm: Mutex<XarmState>,

    #[pin]
    rng: Mutex<Option<hwrng::HwRngHandle>>,

    #[pin]
    machine_refkey: Mutex<Option<MachineRefKey>>,

    #[pin]
    fv_volumes: Mutex<KVec<fv::VolumeMap>>,

    enrol_frames_accepted: Atomic<u32>,

    rng_shutdown: Atomic<bool>,

    rng_failures: Atomic<u64>,

    rng_survey_attempts: Atomic<u32>,

    rx: rxring::RxRing,

    rx_count: Atomic<u64>,
    settle_mark: Atomic<u64>,
    settle_idle_ticks: Atomic<u64>,

    registered: Atomic<bool>,

    shutting_down: Atomic<bool>,
    capture_epoch: Atomic<u64>,

    // The system is going to sleep: no capture may start. Changed under the
    // bio session lock; `sleep_over` is signalled when it clears.
    suspending: Atomic<bool>,

    #[pin]
    sleep_over: CondVar,

    // Enrol and verify work items, and deletions, between `capture_begin`
    // and `capture_end`.
    captures_running: Atomic<u32>,

    // Signalled when an enrol or verify work item has stopped.
    #[pin]
    worker_gone: CondVar,

    // Held by an enrol or verify work item for its whole run and by a
    // deletion while it removes identities and saves the Catacombs, so two
    // Catacomb save sequences never interleave.
    #[pin]
    catacomb_writer: Mutex<()>,

    // An identity has left the enclave but the Catacomb save that makes its
    // deletion durable failed; the next deletion retries the save. Changed
    // only under `catacomb_writer`.
    deletion_unsaved: Atomic<bool>,

    // PREPARE_ENROL selected the empty Linux context during this boot.
    empty_context_prepared: Atomic<bool>,

    #[pin]
    rx_work: Work<SepData>,

    #[pin]
    settle_work: DelayedWork<SepData, SETTLE_WORK_ID>,

    #[pin]
    enrol_work: Work<SepData, ENROL_WORK_ID>,

    #[pin]
    verify_work: Work<SepData, VERIFY_WORK_ID>,

    #[pin]
    late_attach_work: DelayedWork<SepData, LATE_ATTACH_WORK_ID>,

    #[pin]
    stac_work: Work<SepData, STAC_WORK_ID>,

    #[pin]
    pnon_work: Work<SepData, PNON_WORK_ID>,
}

impl_has_work! {
    impl HasWork<Self, 0> for SepData { self.rx_work }
    impl HasWork<Self, ENROL_WORK_ID> for SepData { self.enrol_work }
    impl HasWork<Self, VERIFY_WORK_ID> for SepData { self.verify_work }
    impl HasWork<Self, STAC_WORK_ID> for SepData { self.stac_work }
    impl HasWork<Self, PNON_WORK_ID> for SepData { self.pnon_work }
}

impl_has_delayed_work! {
    impl HasDelayedWork<Self, 1> for SepData { self.settle_work }
    impl HasDelayedWork<Self, LATE_ATTACH_WORK_ID> for SepData { self.late_attach_work }
}

// SAFETY: every field is either internally synchronised (the mutexes, the
// atomics, the lock-free ring) or immutable after construction. The DMA buffer
// is only touched in probe, before the SEP has been told it exists, and at
// unbind, after the mailbox has been stopped.
unsafe impl Send for SepData {}
// SAFETY: see above.
unsafe impl Sync for SepData {}

impl SepData {
    fn new(pdev: &platform::Device<device::Core<'_>>) -> Result<Arc<SepData>> {
        let profile = profile::detect()?;
        let built = shmem::build(pdev, profile.shmem_capacity, profile.shmem_first_item)?;
        let dev: &device::Device<device::Core<'_>> = pdev.as_ref();
        dev_info_ratelimited!(dev, "SEP platform profile: {}\n", profile.name);

        let buf = built.buf;

        let ool_xarm = Self::alloc_ool(
            dev,
            xarm::EP_XARM,
            OOL_SIZE_XARM,
            (OOL_SIZE_XARM, OOL_SIZE_XARM),
        )?;
        let ool_sbio = Self::alloc_ool(
            dev,
            proto::EP_SBIO,
            OOL_SIZE_SBIO,
            (OOL_SIZE_SBIO, OOL_SIZE_SBIO),
        )?;
        let sks_declared = sks_declared_sizes();
        let ool_sks = Self::alloc_ool(dev, proto::EP_SKS, SKS_ALLOC, sks_declared)?;
        let ool_scrd = Self::alloc_ool(
            dev,
            proto::EP_SCRD,
            OOL_SIZE_SCRD,
            (OOL_SIZE_SCRD, OOL_SIZE_SCRD),
        )?;
        let ool_stac = Self::alloc_ool(
            dev,
            stac::EP_STAC,
            stac::OOL_SIZE_STAC,
            (stac::OOL_SIZE_STAC, stac::OOL_SIZE_STAC),
        )?;
        // Sized from the endpoint's layout before it is registered.
        let ool_pnon = Self::alloc_ool(dev, pnon::EP_PNON, pnon::OOL_SIZE_PNON, (0, 0))?;
        let dma_ring = dma::Coherent::<u8>::zeroed_slice(dev, DMA_RING_SIZE, GFP_KERNEL)?;

        let keystore_only = attach::requested();
        let keystore_user_bags = keystore_only && attach::user_bags_requested();
        let xart_start = module_parameters::xart_start_sector.value();
        let xart_guard =
            xart_guard::level(module_parameters::xart_guard.value(), profile.xart_guard);
        // SEP writes are always on. Only the key-store-only initrd phase opens
        // the store read-only, until complete_attach reopens it writable.
        let xart_writes = if keystore_only {
            attach::initrd_xart_writes(keystore_user_bags, xart_guard)
        } else {
            true
        };
        if keystore_user_bags && !xart_writes {
            dev_info_ratelimited!(
                dev,
                "initrd: per-user key bags need xART writes, which the xART guard holds back until the real root (its list lives there); they wait for the real root\n"
            );
        }
        // The qualified 26.6.2 lane uses only the boot-forwarded identity.
        // Check before opening the shared xART store writable.
        if profile::j613_26_6_2() && xart_writes {
            if dt::preboot_uuid().is_none()
                || module_parameters::os_uuid_hi.value() != 0
                || module_parameters::os_uuid_lo.value() != 0
                || xart_guard.is_none()
            {
                dev_err_ratelimited!(dev, "J613 26.6.2 requires a DT identity and xART guard\n");
                return Err(EINVAL);
            }
        }
        let store = match xart_store::Store::open_owner(xart_writes, xart_start) {
            Ok(mut store) => {
                match xart_guard {
                    Some(level) if store.writes_enabled() => store.set_guard(level)?,
                    None if store.writes_enabled() => dev_info_ratelimited!(
                        dev,
                        "xART guard off: the enclave may rewrite any session record\n"
                    ),
                    _ => {}
                }
                let (slots, records, revision, malformed, duplicates, repaired, writable) =
                    store.summary();
                let base = store.base();
                dev_info_ratelimited!(
                    dev,
                    "xART: APFS-resolved .gl extent (base {:#x}); {} slots, {} live records, max revision {}, {} malformed, {} duplicate, {} repaired; writes {}\n",
                    base,
                    slots,
                    records,
                    revision,
                    malformed,
                    duplicates,
                    repaired,
                    if writable { "ENABLED" } else { "disabled" }
                );
                Some(store)
            }
            Err(e) => {
                dev_err_ratelimited!(
                    dev,
                    "xART gigalocker in the iBoot container is unavailable or invalid: {:?}\n",
                    e
                );
                return Err(e);
            }
        };

        let mut host_store = if keystore_only {
            None
        } else {
            match store::Store::open() {
                Ok(store) => Some(store),
                Err(e) => {
                    if profile::j613_26_6_2() {
                        dev_err_ratelimited!(dev, "J613 26.6.2 host-state store unavailable: {:?}\n", e);
                        return Err(e);
                    }
                    dev_warn_ratelimited!(dev, "Linux host-state store unavailable: {:?}\n", e);
                    None
                }
            }
        };

        let bio_index = match host_store.as_mut().map(bio::IdentityIndex::load) {
            Some(Ok(index)) => index,
            Some(Err(e)) if profile::j613_26_6_2() => return Err(e),
            Some(Err(_)) => bio::IdentityIndex::new(),
            None => bio::IdentityIndex::new(),
        };

        Arc::pin_init(
            try_pin_init!(SepData {
                dev: ARef::<device::Device>::from(dev),
                profile,
                attach_phase <- new_mutex!(attach::Phase::new(keystore_only)),
                keystore_only: Atomic::new(keystore_only),
                keystore_user_bags,
                keystore_write_refusals: Atomic::new(0),
                identity_keybag <- new_mutex!(None),
                fw_map <- new_mutex!(None),
                mbox <- new_mutex!(None),
                shmem <- new_mutex!(Some(buf)),
                endpoints <- new_mutex!(EndpointTable::new()),
                trusted_ready <- new_condvar!("SepData::trusted_ready"),
                control <- new_mutex!(control::ControlState::new()),
                control_wq <- new_condvar!("SepData::control_wq"),
                sks_seq: Atomic::new(0),
                sks_header_version: Atomic::new(1),
                sks_initialized: Atomic::new(false),
                sks_wedged: Atomic::new(0),
                bringup: Atomic::new(BRINGUP_FRESH),
                sensor_held: Atomic::new(false),
                serial_unfinished: Atomic::new(false),
                sensor_test_fault_done: Atomic::new(false),
                late_attach_wanted: Atomic::new(false),
                late_attach_defers: Atomic::new(0),
                keybag_designated: Atomic::new(false),
                bringup_started: Atomic::new(false),
                touchid_started: Atomic::new(false),
                touchid_failed: Atomic::new(false),
                enrol_open: Atomic::new(false),
                xart_refusal_logged: Atomic::new(false),
                capture_refusal_logged: Atomic::new(false),
                sensor_calibrated: Atomic::new(false),
                sensor_key_loaded: Atomic::new(false),
                templates_restored: Atomic::new(false),
                cold_restore_candidate: Atomic::new(false),
                enrol_material <- new_mutex!(None),
                match_context <- new_mutex!(None),
                match_context_generation: Atomic::new(0),
                cold_match_contexts <- new_mutex!(KVec::new()),
                sensor_kept: Atomic::new(false),
                sensor_reset_in_capture: Atomic::new(false),
                bg_ops_ready: Atomic::new(false),
                persistence_owed: Atomic::new(false),
                enrol_identity_candidates <- new_mutex!(KVec::new()),
                paused_at_ns: Atomic::new(0),
                sensor_power_ns: Atomic::new(0),
                device_view_synced: Atomic::new(false),
                phase: Atomic::new(PHASE_ATTACH),
                sleep_gate: Atomic::new(pm::SleepGate::Open as u32),
                sleep_rx_held: Atomic::new(false),
                ool_xarm <- new_mutex!(Some(ool_xarm)),
                ool_sbio <- new_mutex!(Some(ool_sbio)),
                ool_sks <- new_mutex!(Some(ool_sks)),
                sks_exchange_lock <- new_mutex!(()),
                scrd_exchange_lock <- new_mutex!(()),
                sks_probe <- new_mutex!(SksProbe::new()),
                sks_wq <- new_condvar!("SepData::sks_wq"),
                ool_scrd <- new_mutex!(Some(ool_scrd)),
                scrd_probe <- new_mutex!(ScrdProbe::new()),
                scrd_wq <- new_condvar!("SepData::scrd_wq"),
                ool_stac <- new_mutex!(Some(ool_stac)),
                stac_rx <- new_mutex!(stac::Exchange::new()),
                stac_wq <- new_condvar!("SepData::stac_wq"),
                stac_named: Atomic::new(false),
                stac_layout: Atomic::new(0),
                stac_absent_logged: Atomic::new(false),
                ool_pnon <- new_mutex!(Some(ool_pnon)),
                pnon_rx <- new_mutex!(pnon::Exchange::new()),
                pnon_wq <- new_condvar!("SepData::pnon_wq"),
                pnon_named: Atomic::new(false),
                pnon_layout: Atomic::new(0),
                pnon_done: Atomic::new(false),
                sse_named: Atomic::new(false),
                sse_layout: Atomic::new(0),
                hibe_named: Atomic::new(false),
                hibe_layout: Atomic::new(0),
                skdl_named: Atomic::new(false),
                skdl_layout: Atomic::new(0),
                dma_ring <- new_mutex!(Some(dma_ring)),
                sbio_exchange_lock <- new_mutex!(()),
                sbio_rx <- new_mutex!(transfer::Reassembly::new()),
                sbio_wq <- new_condvar!("SepData::sbio_wq"),
                sbio_ready: Atomic::new(false),
                bio_session <- new_mutex!(bio::Session::new()),
                bio_index <- new_mutex!(bio_index),
                bio_dev <- new_mutex!(None),
                rearm_dev <- new_mutex!(None),
                key_dev <- new_mutex!(None),
                key_bag_lock <- new_mutex!(()),
                key_touch <- new_mutex!(None),
                key_touch_active: Atomic::new(false),
                key_touch_cancel: Atomic::new(false),
                key_context_failed: Atomic::new(false),
                last_key_status: Atomic::new(0),
                bio_lockout: Atomic::new(false),
                bio_after_boot: Atomic::new(false),
                bio_lockout_logged: Atomic::new(false),
                bio_nomatch_streak: Atomic::new(0),
                enclave_failures_owed: Atomic::new(false),
                lockout_import_pending: Atomic::new(false),
                bio_open_gen: Atomic::new(0),
                match_scope_gen: Atomic::new(0),
                match_scope_touches: Atomic::new(0),
                match_scope_misses: Atomic::new(0),
                bio_lock_state: Atomic::new(0),
                bio_lock_state_seq: Atomic::new(0),
                bio_lock_logged: [Atomic::new(0), Atomic::new(0), Atomic::new(0), Atomic::new(0)],
                store <- new_mutex!(store),
                host_store <- new_mutex!(host_store),
                xarm <- new_mutex!(XarmState::new()),
                rng <- new_mutex!(None),
                machine_refkey <- new_mutex!(None),
                fv_volumes <- new_mutex!(KVec::new()),
                enrol_frames_accepted: Atomic::new(0),
                rng_shutdown: Atomic::new(false),
                rng_failures: Atomic::new(0),
                rng_survey_attempts: Atomic::new(0),
                rx: rxring::RxRing::new(),
                rx_count: Atomic::new(0),
                settle_mark: Atomic::new(0),
                settle_idle_ticks: Atomic::new(0),
                registered: Atomic::new(false),
                shutting_down: Atomic::new(false),
                capture_epoch: Atomic::new(0),
                suspending: Atomic::new(false),
                sleep_over <- new_condvar!("SepData::sleep_over"),
                captures_running: Atomic::new(0),
                worker_gone <- new_condvar!("SepData::worker_gone"),
                catacomb_writer <- new_mutex!(()),
                deletion_unsaved: Atomic::new(false),
                empty_context_prepared: Atomic::new(false),
                rx_work <- new_work!("SepData::rx_work"),
                enrol_work <- new_work!("SepData::enrol_work"),
                verify_work <- new_work!("SepData::verify_work"),
                settle_work <- new_delayed_work!("SepData::settle_work"),
                late_attach_work <- new_delayed_work!("SepData::late_attach_work"),
                stac_work <- new_work!("SepData::stac_work"),
                pnon_work <- new_work!("SepData::pnon_work"),
            }),
            GFP_KERNEL,
        )
    }

    /// Bring the shared-memory table to the SEP using the profile's bootstrap.
    fn attach(&self, sep_node: &dt::DtNode) -> Result<()> {
        match self.profile.bootstrap {
            profile::Bootstrap::WarmRegister => self.attach_warm(sep_node),
            profile::Bootstrap::Boot => self.attach_m1(sep_node),
        }
    }

    /// Warm attach: the firmware left a running SEP, so one registration message
    /// hands it the shared-memory table.
    fn attach_warm(&self, sep_node: &dt::DtNode) -> Result<()> {
        let (iova, size) = {
            let guard = self.shmem.lock();
            let buf = guard.as_ref().ok_or(EINVAL)?;
            (buf.dma_address(), buf.len())
        };

        let msg = proto::shmem_registration(iova, size)?;


        if dt::registration_already_sent(sep_node) {
            return Err(EBUSY);
        }

        if let Err(e) = dt::mark_registration_sent(sep_node, iova) {
            dev_err_ratelimited!(
                self.dev,
                "could not record the registration marker ({:?}); refusing to send, since without it an rmmod/insmod cycle could send it twice and park the SEP\n",
                e
            );
            return Err(e);
        }
        self.registered.store(true, Relaxed);

        Self::ring(self.mbox.lock().as_ref().ok_or(EINVAL)?, msg)?;

        // no ack on 0xFE; success is the discovery burst on 0xFD
        Ok(())
    }

    /// Cold boot: send TZ0 and let the boot-endpoint acknowledgements drive the
    /// firmware and shared-memory handoff.
    fn attach_m1(&self, sep_node: &dt::DtNode) -> Result<()> {
        // The SEP stays booted after rmmod and Linux cannot reset it, so a
        // reload's TZ0 would go unanswered. Record the boot first, as the warm
        // path does, so a reload stops at probe and says to reboot instead.
        let iova = self.shmem.lock().as_ref().map_or(0, |buf| buf.dma_address());
        if let Err(e) = dt::mark_registration_sent(sep_node, iova) {
            dev_err_ratelimited!(
                self.dev,
                "could not record the boot marker ({:?}); refusing to boot the SEP, since a reload could not tell it is already running\n",
                e
            );
            return Err(e);
        }
        let msg = Message {
            msg0: u64::from(proto::EP_BOOT) | (proto::MSG_BOOT_TZ0 << proto::MSG_TYPE_SHIFT),
            msg1: 0,
        };
        self.send(msg)
    }

    /// Handle a message from the boot endpoint during the cold-boot handshake.
    fn on_boot(&self, msg: Message) {
        let ty = (msg.msg0 >> proto::MSG_TYPE_SHIFT) & 0xff;
        match ty {
            proto::MSG_BOOT_TZ0_ACK1 => {
                dev_info_ratelimited!(self.dev, "boot: first TZ0 acknowledgement\n");
            }
            proto::MSG_BOOT_TZ0_ACK2 => {
                dev_info_ratelimited!(
                    self.dev,
                    "boot: TZ0 accepted; handing over firmware and shared memory\n"
                );
                if let Err(e) = self.load_firmware_and_shmem() {
                    dev_err_ratelimited!(
                        self.dev,
                        "boot: firmware/shared-memory handoff failed ({:?})\n",
                        e
                    );
                }
            }
            proto::MSG_BOOT_IMG4_ACK => {
                dev_info_ratelimited!(
                    self.dev,
                    "boot: IMG4 acknowledged; the SEP owns the shared-memory table\n"
                );
                self.registered.store(true, Relaxed);
            }
            _ => {
                dev_warn_ratelimited!(
                    self.dev,
                    "boot: unknown message type {} (msg0 {:#018x})\n",
                    ty,
                    msg.msg0
                );
            }
        }
    }

    /// Map the firmware reserved region once and hand the SEP the firmware
    /// address (IMG4) and the shared-memory table (SET_SHMEM).
    fn load_firmware_and_shmem(&self) -> Result<()> {
        if self.shutting_down.load(Relaxed) {
            return Err(ENODEV);
        }

        let (phys, size) = dt::reserved_region(&self.dev, self.profile.firmware_region)?;

        // Hold the mapping lock across the teardown re-check, the map and the
        // store: `remove()` takes the same lock, so a mapping either exists
        // before removal and is unmapped by it, or is refused here.
        let mut fw_map = self.fw_map.lock();
        if fw_map.is_some() {
            // One-shot per boot: a second acknowledgement cannot remap it.
            return Ok(());
        }
        if self.shutting_down.load(Relaxed) {
            return Err(ENODEV);
        }
        // SAFETY: `self.dev` is live; the reserved region is owned by the
        // firmware handoff and the mapping is retained until `remove()`.
        let iova = unsafe {
            let mapped = bindings::dma_map_resource(
                self.dev.as_raw(),
                phys,
                size,
                bindings::dma_data_direction_DMA_TO_DEVICE,
                0,
            );
            if bindings::dma_mapping_error(self.dev.as_raw(), mapped) != 0 {
                return Err(ENOMEM);
            }
            mapped
        };
        *fw_map = Some(FirmwareMap { size, iova });
        drop(fw_map);

        let msg = Message {
            msg0: u64::from(proto::EP_BOOT)
                | (proto::MSG_BOOT_IMG4 << proto::MSG_TYPE_SHIFT)
                | ((iova >> proto::IOVA_SHIFT) << proto::MSG_DATA_SHIFT),
            msg1: 0,
        };
        self.send(msg)?;

        let shm = {
            let guard = self.shmem.lock();
            let buf = guard.as_ref().ok_or(EINVAL)?;
            buf.dma_address()
        };
        let msg = Message {
            msg0: u64::from(proto::EP_SHMEM)
                | (proto::MSG_SET_SHMEM << proto::MSG_TYPE_SHIFT)
                | ((shm >> proto::IOVA_SHIFT) << proto::MSG_DATA_SHIFT),
            msg1: 0,
        };
        // The peer may know the address even if this send fails ambiguously, so
        // the buffer stays retained: `remove()` must never free shared memory
        // the SEP was handed.
        self.registered.store(true, Relaxed);
        self.send(msg)?;
        Ok(())
    }

    /// Release the firmware mapping if one is live. Called by `remove()` so a
    /// mapping created by a racing boot message cannot outlive the driver.
    fn release_firmware_mapping(&self) {
        if let Some(mapped) = self.fw_map.lock().take() {
            // SAFETY: created by `dma_map_resource` with these exact parameters
            // and unmapped exactly once.
            unsafe {
                bindings::dma_unmap_resource(
                    self.dev.as_raw(),
                    mapped.iova,
                    mapped.size,
                    bindings::dma_data_direction_DMA_TO_DEVICE,
                    0,
                );
            }
        }
    }

    fn send(&self, msg: Message) -> Result<()> {
        if self.shutting_down.load(Relaxed) {
            return Err(ENODEV);
        }
        // Checked with the mailbox lock held: a sender can wait for it for longer
        // than the system takes to tell the enclave it's going to sleep.
        let mbox = self.mbox.lock();
        let mbox = mbox.as_ref().ok_or(ENODEV)?;
        if !self.sleep_gate().may_request() {
            return Err(EAGAIN);
        }
        Self::ring(mbox, msg)
    }

    /// As [`Self::send`], for an answer to something the enclave asked: it
    /// waits for these before it can answer the sleep request.
    fn send_reply(&self, msg: Message) -> Result<()> {
        if self.shutting_down.load(Relaxed) {
            return Err(ENODEV);
        }
        let mbox = self.mbox.lock();
        let mbox = mbox.as_ref().ok_or(ENODEV)?;
        if !self.sleep_gate().may_reply() {
            return Err(EAGAIN);
        }
        Self::ring(mbox, msg)
    }

    pub(crate) fn sleep_gate(&self) -> pm::SleepGate {
        pm::SleepGate::from_u32(self.sleep_gate.load(Acquire))
    }

    /// Under the mailbox lock, so that a sender holding it finishes first.
    fn set_sleep_gate(&self, gate: pm::SleepGate) {
        let _mbox = self.mbox.lock();
        self.sleep_gate.store(gate as u32, Release);
    }

    /// Prepare a firmware-driven suspend without sending SLEEP. The platform
    /// owns the irreversible request, after every returnable refusal.
    pub(crate) fn sleep_enter(&self) -> Result<()> {
        match self.sleep_gate() {
            // A failed earlier wake is not repaired by another precommit attempt.
            pm::SleepGate::Lost => return Err(EIO),
            pm::SleepGate::Open => {}
            _ => return Err(EBUSY),
        }
        // Do not commit a running enclave whose sleep protocol is not ready.
        if self.phase.load(Relaxed) != PHASE_READY || !self.endpoint_present(proto::EP_CONTROL) {
            dev_warn_ratelimited!(
                self.dev,
                "sleep refused: the enclave did not finish coming up\n"
            );
            return Err(EBUSY);
        }

        self.set_sleep_gate(pm::SleepGate::Draining);
        let idle = control::wait_idle(
            proto::CONTROL_TIMEOUT_MS,
            10,
            || self.control.lock().requests_in_flight(),
            |ms| kernel::time::delay::fsleep(kernel::time::Delta::from_millis(i64::from(ms))),
        );
        if !idle {
            dev_err_ratelimited!(
                self.dev,
                "sleep refused: a request is still waiting for its reply\n"
            );
            self.set_sleep_gate(pm::SleepGate::Open);
            return Err(EBUSY);
        }

        if let Err(e) = self.sleep_leave_for_commit() {
            dev_err_ratelimited!(
                self.dev,
                "sleep refused: irreversible request could not be staged ({:?})\n",
                e
            );
            self.set_sleep_gate(pm::SleepGate::Open);
            return Err(e);
        }
        Ok(())
    }

    /// Stage without sending; the platform's commit owns SLEEP with IRQs off.
    fn sleep_leave_for_commit(&self) -> Result<()> {
        let request = proto::encode_control(&proto::op_sleep(), proto::TAG_SLEEP);
        match self.mbox.lock().as_ref() {
            Some(mbox) => pm::leave_for_commit(
                &self.dev,
                mbox.as_raw(),
                request,
                proto::SLEEP_REPLY,
                proto::SLEEP_REPLY_MASK,
            ),
            None => Err(ENODEV),
        }?;
        self.sleep_hold_rx();
        dev_dbg!(self.dev, "SEP SLEEP staged for the irreversible commit\n");
        Ok(())
    }

    /// Holds off the receive interrupt, whose handler reads the mailbox.
    fn sleep_hold_rx(&self) {
        if !self.sleep_rx_held.xchg(true, Relaxed) {
            if let Some(mbox) = self.mbox.lock().as_ref() {
                mbox.hold_rx();
            }
        }
    }

    fn sleep_release_rx(&self) {
        if self.sleep_rx_held.xchg(false, Relaxed) {
            if let Some(mbox) = self.mbox.lock().as_ref() {
                mbox.release_rx();
            }
        }
    }

    /// A refused precommit attempt was never sent; reopen normal admission.
    /// A sent request can be restarted only after actual physical sleep/wake.
    /// Returns whether the enclave came back from a sleep it was told of.
    pub(crate) fn sleep_exit(&self) -> bool {
        // Staged only: an unsent refusal releases RX without a restart.
        match pm::commit_finish(&self.dev) {
            Some(true) => self.set_sleep_gate(pm::SleepGate::Told),
            Some(false) => self.sleep_release_rx(),
            None => {}
        }
        let gate = self.sleep_gate();
        if gate == pm::SleepGate::Draining {
            self.set_sleep_gate(pm::SleepGate::Open);
            return false;
        }
        if !gate.needs_wake() {
            return false;
        }
        // SAFETY: only reads a flag.
        let real = unsafe { bindings::pm_resume_via_firmware() };
        if !real && gate == pm::SleepGate::Lost {
            return false;
        }
        if !real {
            // commit_finish already terminates a current sent-without-sleep case.
            // An old lost state must not cause mailbox or DART accesses here.
            self.set_sleep_gate(pm::SleepGate::Lost);
            return false;
        }
        let answered = self.sleep_kick();
        match answered {
            Ok(()) => {
                self.set_sleep_gate(pm::SleepGate::Open);
                // One line per sleep it was told of: nothing else says it came back.
                dev_info_ratelimited!(self.dev, "the enclave is back from its sleep request\n");
                true
            }
            Err(e) => {
                dev_err_ratelimited!(
                    self.dev,
                    "the enclave did not come back ({:?}); it stays unused until a reboot\n",
                    e
                );
                self.sleep_hold_rx();
                self.set_sleep_gate(pm::SleepGate::Lost);
                false
            }
        }
    }

    /// Starts a gated enclave and waits for the messages that say it's up.
    fn sleep_kick(&self) -> Result<()> {
        pm::clear_dev_disable(&self.dev)?;
        pm::dart_restore(&self.dev);
        // Arm completion before the held receive IRQ can deliver an early wake.
        self.control.lock().wake_reset();
        self.sleep_release_rx();
        match self.mbox.lock().as_ref() {
            Some(mbox) => {
                mbox.set_recv_enabled(true);
                Self::ring(mbox, proto::WAKE_KICK)?;
            }
            None => return Err(ENODEV),
        }
        let mut remaining = time::msecs_to_jiffies(proto::CONTROL_TIMEOUT_MS);
        let mut guard = self.control.lock();
        loop {
            if guard.wake_complete() {
                return Ok(());
            }
            if remaining == 0 {
                return Err(ETIMEDOUT);
            }
            match self
                .control_wq
                .wait_interruptible_timeout(&mut guard, remaining)
            {
                CondVarTimeoutResult::Woken { jiffies } => remaining = jiffies,
                CondVarTimeoutResult::Signal { jiffies } => remaining = jiffies,
                CondVarTimeoutResult::Timeout => remaining = 0,
            }
        }
    }

    /// Puts `msg` in the mailbox once the enclave can see what the CPU wrote
    /// to the buffers before it: a request payload, an SBIO header's poison,
    /// the shared memory a registration hands over. Those are coherent
    /// allocations, which have no sync call to order them, and the mailbox
    /// write is relaxed MMIO, which is not ordered after them either.
    /// Without the barrier the enclave could read old bytes, or have its
    /// reply's header overwritten by a refill that lands late. dma_wmb() is
    /// what writel() would put there, and what RTKit puts before its sends.
    fn ring(mbox: &Mailbox<SepData>, msg: Message) -> Result<()> {
        dma_mb(Write);
        mbox.send(msg, false)
    }


    fn control_request(&self, op: &proto::ControlOp) -> Result<Option<proto::ControlReply>> {
        let (idx, tag) = self.control.lock().alloc()?;
        let msg = proto::encode_control(op, tag);

        if let Err(e) = self.send(msg) {
            self.control.lock().abandon(idx);
            return Err(e);
        }

        self.control_wait(op, idx, tag)
    }

    /// Waits for the answer to the request sent under `idx` and `tag`.
    fn control_wait(
        &self,
        op: &proto::ControlOp,
        idx: usize,
        tag: u8,
    ) -> Result<Option<proto::ControlReply>> {
        let mut remaining = time::msecs_to_jiffies(op.timeout_ms());
        let mut guard = self.control.lock();
        loop {
            if self.shutting_down.load(Relaxed) {
                guard.abandon(idx);
                return Err(ENODEV);
            }
            if let Some(reply) = guard.take_reply(idx) {
                guard.release(idx);
                drop(guard);
                if reply.tag != tag {
                    return Err(EIO);
                }
                return Ok(Some(reply));
            }

            if remaining == 0 {
                let tag = guard.abandon(idx);
                let retired = guard.retired_count();
                drop(guard);
                if op.expects_reply() {
                    dev_warn_ratelimited!(
                        self.dev,
                        "control <- {}: no reply within {} ms; tag 0x{:02x} retired ({} retired in total)\n",
                        op.name(),
                        op.timeout_ms(),
                        tag,
                        retired
                    );
                }
                return Ok(None);
            }

            // A reply wait sleeps uninterruptibly: with a signal pending an
            // interruptible wait returns at once with the time left unchanged,
            // so this loop would spin until the reply came, or forever.
            match self
                .control_wq
                .wait_timeout(&mut guard, remaining)
            {
                CondVarTimeoutResult::Woken { jiffies } => remaining = jiffies,
                CondVarTimeoutResult::Signal { .. } => remaining = 0,
                CondVarTimeoutResult::Timeout => remaining = 0,
            }
        }
    }

    fn get_entropy_word(&self) -> Result<u32> {
        let op = proto::op_get_entropy();
        if !op.uses_reserved_tag() {
            return Err(EINVAL);
        }

        self.control.lock().entropy_begin(boottime_ms())?;

        let msg = proto::encode_control(&op, proto::TAG_ENTROPY);
        if let Err(e) = self.send_entropy_request(msg) {
            self.control
                .lock()
                .entropy_end(control::EntropyEnd::not_sent(e), boottime_ms());
            return Err(e);
        }

        let mut remaining = time::msecs_to_jiffies(op.timeout_ms());
        let mut guard = self.control.lock();
        let (result, end) = loop {
            if let Some(value) = guard.entropy_take() {
                break (Ok(value), control::EntropyEnd::Answered);
            }
            if self.rng_shutdown.load(Relaxed) {
                break (Err(ECANCELED), control::EntropyEnd::Abandoned);
            }
            if remaining == 0 {
                break (Err(ETIMEDOUT), control::EntropyEnd::TimedOut);
            }
            // A reply wait sleeps uninterruptibly: with a signal pending an
            // interruptible wait returns at once with the time left unchanged,
            // so this loop would spin until the reply came, or forever.
            match self
                .control_wq
                .wait_timeout(&mut guard, remaining)
            {
                CondVarTimeoutResult::Woken { jiffies } => remaining = jiffies,
                CondVarTimeoutResult::Signal { .. } => remaining = 0,
                CondVarTimeoutResult::Timeout => remaining = 0,
            }
        };
        guard.entropy_end(end, boottime_ms());
        result
    }

    /// As [`Self::get_entropy_word`], for a reader that blocks. While requests
    /// are held back, wait for that to end, for as long as a request waits
    /// for its reply, rather than fail at once. EAGAIN if it does not end
    /// in that time, ERESTARTSYS if a signal comes.
    fn get_entropy_word_blocking(&self) -> Result<u32> {
        let limit = u64::from(proto::CONTROL_TIMEOUT_MS);
        let deadline = boottime_ms().saturating_add(limit);
        loop {
            // As `hwrng_fill()` checks before each word: a removal wakes the
            // wait below, and no request may go out after it.
            if self.rng_shutdown.load(Relaxed) {
                return Err(ENODEV);
            }
            match self.get_entropy_word() {
                Err(e) if e == EAGAIN => {}
                done => return done,
            }
            let mut guard = self.control.lock();
            let now = boottime_ms();
            let left = deadline.saturating_sub(now);
            match guard.entropy_held_ms(now, left) {
                // It ended since; ask now.
                None if left > 0 => continue,
                None | Some(0) => return Err(EAGAIN),
                Some(ms) => {
                    if self.rng_shutdown.load(Relaxed) {
                        return Err(ENODEV);
                    }
                    let ms = u32::try_from(ms).unwrap_or(proto::CONTROL_TIMEOUT_MS);
                    if let CondVarTimeoutResult::Signal { .. } = self
                        .control_wq
                        .wait_interruptible_timeout(&mut guard, time::msecs_to_jiffies(ms))
                    {
                        return Err(ERESTARTSYS);
                    }
                }
            }
        }
    }

    /// Sends an entropy request claimed by `entropy_begin()`, or EAGAIN if
    /// the system has begun going to sleep since. The check is made with the
    /// mailbox lock held. A sender can wait for that lock behind others, such
    /// as a Touch ID capture still running, for longer than the drain on the
    /// way to sleep waits for it. It must not then write its request into
    /// the suspend. Lock order: mailbox, then control.
    fn send_entropy_request(&self, msg: Message) -> Result<()> {
        if self.shutting_down.load(Relaxed) {
            return Err(ENODEV);
        }
        let mbox = self.mbox.lock();
        let mbox = mbox.as_ref().ok_or(ENODEV)?;
        if !self.sleep_gate().may_request() || !self.control.lock().entropy_may_send() {
            return Err(EAGAIN);
        }
        Self::ring(mbox, msg)
    }

    /// The system is about to suspend: send no entropy request until it has
    /// resumed, and let one already sent be answered while the enclave can
    /// still answer it. On a T6000 a request sent while devices were
    /// suspended was never answered, and the enclave did not read its mailbox
    /// again after the resume.
    pub(crate) fn rng_sleep_prepare(&self) {
        if !self.control.lock().entropy_sleep() {
            return;
        }
        let idle = control::wait_idle(
            RNG_SLEEP_DRAIN_MS,
            10,
            || self.control.lock().entropy_busy(),
            |ms| kernel::time::delay::fsleep(kernel::time::Delta::from_millis(i64::from(ms))),
        );
        if !idle {
            dev_warn_ratelimited!(
                self.dev,
                "hwrng: a request was still in flight after {} ms; suspending anyway\n",
                RNG_SLEEP_DRAIN_MS
            );
        }
    }

    /// The system has resumed, or the suspend was aborted.
    pub(crate) fn rng_sleep_finished(&self) {
        self.control.lock().entropy_wake();
        // A blocking hwrng reader may be waiting for this.
        self.control_wq.notify_all();
    }

    fn control_survey(&self) {
        for param in [0x00u8, 0x01, 0xff] {
            let _ = self.control_request(&proto::op_nop(param));
        }

        let _ = self.control_request(&proto::op_security_mode());

        let _state = self.control.lock();
    }

    fn survey_hwrng(&self) -> Result<()> {
        // Successful transport is not proof of entropy: J313/13.5 has returned
        // four zero words here. Do not advertise that source to the RNG core.
        // This is a minimal sanity check, not a complete RNG health test.
        let mut any_bits = 0u32;
        for _ in 0..4 {
            any_bits |= self.get_entropy_word()?;
        }
        if any_bits == 0 {
            return Err(EIO);
        }
        self.register_hwrng()
    }

    fn tick_ready(this: &Arc<SepData>) {
        if this.rng.lock().is_some() {
            return;
        }
        let used = this.rng_survey_attempts.load(Relaxed);
        let Some((attempt, next)) =
            control::survey_step(used, RNG_SURVEY_ATTEMPTS, || this.survey_hwrng())
        else {
            return;
        };
        this.rng_survey_attempts.store(attempt, Relaxed);
        match next {
            control::Survey::Registered => {
                diag::hwrng(true);
                dev_info_ratelimited!(
                    this.dev,
                    "hwrng: registered after ready-phase survey (attempt {})\n",
                    attempt
                )
            }
            // Asleep, or holding back after a request that went unanswered:
            // this attempt did not ask the enclave anything.
            control::Survey::Held => Self::arm_settle(this),
            control::Survey::Retry(e) => {
                dev_warn_ratelimited!(this.dev, "hwrng: survey attempt {} failed ({:?}); retrying\n", attempt, e);
                Self::arm_settle(this);
            }
            control::Survey::GiveUp(e) => {
                diag::hwrng(false);
                dev_err_ratelimited!(
                    this.dev,
                    "hwrng: survey failed after {} attempts ({:?})\n",
                    attempt,
                    e
                )
            }
        }
    }


    fn alloc_ool(
        dev: &device::Device<device::Core<'_>>,
        endpoint: u8,
        allocated: usize,
        declared: (usize, usize),
    ) -> Result<OolPair> {
        if declared.0 > allocated || declared.1 > allocated {
            return Err(EINVAL);
        }

        let inbound = dma::Coherent::<u8>::zeroed_slice(dev, allocated, GFP_KERNEL)?;
        let outbound = dma::Coherent::<u8>::zeroed_slice(dev, allocated, GFP_KERNEL)?;

        // SAFETY: nothing is registered yet, so the SEP does not know these
        // buffers exist and cannot be touching them.
        unsafe {
            inbound.as_mut().fill(OOL_POISON_INBOUND);
            outbound.as_mut().fill(OOL_POISON_OUTBOUND);
        }

        Ok(OolPair {
            endpoint,
            allocated,
            declared_in: declared.0,
            declared_out: declared.1,
            inbound,
            outbound,
            registered: false,
        })
    }

    fn control_request_required(&self, op: &proto::ControlOp) -> Result<proto::ControlReply> {
        match self.control_request(op)? {
            Some(reply) => Ok(reply),
            None => Err(ETIMEDOUT),
        }
    }

    /// Sets the sizes an endpoint's buffers are registered with, for an
    /// endpoint sized from its discovery layout. Once registered, only the
    /// same sizes are accepted.
    fn size_ool(&self, slot: &Mutex<Option<OolPair>>, declared: (usize, usize)) -> Result<()> {
        let mut guard = slot.lock();
        let pair: &mut Option<OolPair> = &mut guard;
        let b = pair.as_mut().ok_or(ENODEV)?;
        if b.registered {
            return if (b.declared_in, b.declared_out) == declared {
                Ok(())
            } else {
                Err(EBUSY)
            };
        }
        if declared.0 == 0
            || declared.1 == 0
            || declared.0 > b.allocated
            || declared.1 > b.allocated
        {
            return Err(EINVAL);
        }
        b.declared_in = declared.0;
        b.declared_out = declared.1;
        Ok(())
    }

    fn register_ool(&self, slot: &Mutex<Option<OolPair>>) -> Result<()> {
        let guard = slot.lock();
        if guard.as_ref().is_some_and(|b| b.registered) {
            return Ok(());
        }

        let (endpoint, allocated, declared_in, declared_out, in_iova, out_iova) = {
            let b = guard.as_ref().ok_or(EINVAL)?;
            (
                b.endpoint,
                b.allocated,
                b.declared_in,
                b.declared_out,
                b.inbound.dma_address(),
                b.outbound.dma_address(),
            )
        };

        if declared_in > allocated || declared_out > allocated {
            return Err(EINVAL);
        }

        // drop the buffer lock before blocking control requests; the drain path takes it
        drop(guard);

        self.control_request_required(&proto::op_ool_inbound_size(endpoint, declared_in as u32))?;
        self.control_request_required(&proto::op_ool_inbound_addr(endpoint, in_iova))?;
        self.control_request_required(&proto::op_ool_outbound_size(endpoint, declared_out as u32))?;
        self.control_request_required(&proto::op_ool_outbound_addr(endpoint, out_iova))?;

        {
            // deref first, or the guard's inherent as_mut wins over Option::as_mut
            let mut guard = slot.lock();
            let pair: &mut Option<OolPair> = &mut guard;
            if let Some(b) = pair.as_mut() {
                b.registered = true;
            }
        }
        Ok(())
    }

    fn with_store<R>(&self, f: impl FnOnce(&mut xart_store::Store) -> R) -> Option<R> {
        let mut guard = self.store.lock();
        let store: &mut Option<xart_store::Store> = &mut guard;
        store.as_mut().map(f)
    }

    /// Whether the enclave's xART writes reach the store.
    pub(crate) fn xart_writable(&self) -> bool {
        self.with_store(|store| store.writes_enabled())
            .unwrap_or(false)
    }

    pub(crate) fn capture_refusal_logged(&self) -> &Atomic<bool> {
        &self.capture_refusal_logged
    }

    fn with_host_store<R>(&self, f: impl FnOnce(&mut store::Store) -> R) -> Option<R> {
        let mut guard = self.host_store.lock();
        let store: &mut Option<store::Store> = &mut guard;
        store.as_mut().map(f)
    }

    fn prepare_os_uuid(&self) {
        let hi = module_parameters::os_uuid_hi.value();
        let lo = module_parameters::os_uuid_lo.value();
        // Keep the existing lane on installed systems. The tested selector
        // never invents an identity or lets a profile override explicit params.
        match os_identity::select(hi, lo, dt::preboot_uuid) {
            os_identity::Selection::Explicit(uuid) => {
                self.xarm.lock().os_uuid = Some(uuid);
                diag::identity_source(diag::IdentitySource::Override);
                diag::os_identity(diag::OsIdentity::Explicit);
                dev_info_ratelimited!(self.dev, "xART: using explicit OS identity\n");
            }
            os_identity::Selection::Chosen(uuid) => {
                self.xarm.lock().os_uuid = Some(uuid);
                diag::identity_source(diag::IdentitySource::DeviceTree);
                diag::os_identity(diag::OsIdentity::Chosen);
                dev_info_ratelimited!(self.dev, "xART: using /chosen/apfs-preboot-uuid\n");
            }
            os_identity::Selection::Unavailable => {
                self.xarm.lock().os_uuid = None;
                diag::identity_source(diag::IdentitySource::Missing);
                diag::os_identity(diag::OsIdentity::Unavailable);
                dev_warn_ratelimited!(self.dev, "xART: /chosen/apfs-preboot-uuid is unavailable\n");
            }
        }
    }

    fn on_xarm(&self, msg: Message) {
        let req = crate::xarm::decode_xarm(&msg);

        if xarm::is_silent(req.opcode) {
            return;
        }

        let ready = self.ool_xarm.lock().as_ref().is_some_and(|b| b.registered);

        if !ready {
            if !xarm::needs_buffers(req.opcode) {
                self.xarm.lock().deferred_query = Some(req.tag);
                return;
            }

            self.fail_xarm(req.tag);
            return;
        }

        self.service_xarm(&req);
    }

    fn service_xarm(&self, req: &crate::xarm::XarmRequest) {
        let want = req.length as usize;
        if want > OOL_SIZE_XARM {
            self.fail_xarm(req.tag);
            return;
        }

        if want > 0 && !self.ool_await_written(&self.ool_xarm, 0, want) {
            self.fail_xarm(req.tag);
            return;
        }
        let payload = match self.ool_read(&self.ool_xarm, 0, want) {
            Ok(p) => p,
            Err(_) => {
                self.fail_xarm(req.tag);
                return;
            }
        };

        let mut staging = KVec::new();
        if staging.resize(OOL_SIZE_XARM, 0u8, GFP_KERNEL).is_err() {
            self.fail_xarm(req.tag);
            return;
        }

        let os_uuid = self.xarm.lock().os_uuid;
        let serviced = self.with_store(|store| {
            let done = xarm::service(
                req,
                &payload,
                &mut staging,
                store,
                PROTECTED_DATA_AVAILABLE,
                os_uuid,
            );
            (done, store.writes_enabled())
        });
        let Some((done, writable)) = serviced else {
            self.fail_xarm(req.tag);
            return;
        };

        if !writable
            && xarm::is_write(req.opcode)
            && done.reply.status != xarm::STATUS_OK
            && self.keystore_only.load(Relaxed)
        {
            self.keystore_write_refusals.fetch_add(1, Relaxed);
        }
        if !writable
            && xarm::is_write(req.opcode)
            && done.reply.status != xarm::STATUS_OK
            && !self.xart_refusal_logged.xchg(true, Relaxed)
        {
            dev_warn_ratelimited!(
                self.dev,
                "xART: refused the enclave's write (op 0x{:02x}): the store is open read-only (the key-store-only initrd phase, or a failed writable reopen)\n",
                req.opcode
            );
        }

        if module_parameters::xarm_trace.value() != 0 {
            dev_info!(
                self.dev,
                "xarm: op 0x{:02x} request_len {} reply_status 0x{:02x} reply_len {}\n",
                req.opcode,
                req.length,
                done.reply.status,
                done.reply.length
            );
        }

        if done.reply_bytes > 0 {
            if self
                .ool_write(&self.ool_xarm, 0, &staging[..done.reply_bytes])
                .is_err()
            {
                self.fail_xarm(req.tag);
                return;
            }
        }

        self.send_xarm_reply(&done.reply);
    }

    fn ool_read(&self, slot: &Mutex<Option<OolPair>>, off: usize, len: usize) -> Result<KVec<u8>> {
        let mut guard = slot.lock();
        let pair: &mut Option<OolPair> = &mut guard;
        let buffers = pair.as_mut().ok_or(ENODEV)?;
        if off + len > buffers.allocated {
            return Err(EINVAL);
        }

        let mut payload = KVec::new();
        if len > 0 {
            // SAFETY: the SEP fills this buffer while preparing a request and
            // then waits for the host's reply, so it is quiescent here, and the
            // driver is the only other accessor.
            let src = unsafe { &buffers.outbound.as_ref()[off..off + len] };
            payload.extend_from_slice(src, GFP_KERNEL)?;
        }

        Ok(payload)
    }

    /// Copies `out.len()` bytes at `off` of the outbound buffer into `out`;
    /// `false` once the buffers are gone or the range is outside them.
    fn ool_peek(&self, slot: &Mutex<Option<OolPair>>, off: usize, out: &mut [u8]) -> bool {
        let guard = slot.lock();
        let Some(buffers) = guard.as_ref() else {
            return false;
        };
        if off + out.len() > buffers.allocated {
            return false;
        }
        // SAFETY: the region is within the allocation as checked above, and
        // the driver is the only host-side accessor.
        out.copy_from_slice(unsafe { &buffers.outbound.as_ref()[off..off + out.len()] });
        true
    }

    /// Fills `len` bytes at `off` of the outbound buffer with the poison
    /// again, once the message they held has been read.
    fn ool_repoison(&self, slot: &Mutex<Option<OolPair>>, off: usize, len: usize) {
        let guard = slot.lock();
        let Some(buffers) = guard.as_ref() else {
            return;
        };
        if off + len > buffers.allocated {
            return;
        }
        // SAFETY: the region is within the allocation as checked above, and
        // the driver is the only host-side accessor. The enclave writes the
        // region only to deliver a message, and the one it held has been
        // looked at. If part of it was still landing, that part may land over
        // the fill afterwards, as a header that has not landed yet does.
        unsafe { buffers.outbound.as_mut()[off..off + len].fill(OOL_POISON_OUTBOUND) };
    }

    fn ool_await_written(&self, slot: &Mutex<Option<OolPair>>, off: usize, len: usize) -> bool {
        for _ in 0..OOL_WRITE_POLL_ATTEMPTS {
            {
                let guard = slot.lock();
                let Some(buffers) = guard.as_ref() else {
                    return false;
                };
                if off + len > buffers.allocated {
                    return true;
                }
                // SAFETY: the region is within the allocation as checked above,
                // and the driver is the only host-side accessor.
                let region = unsafe { &buffers.outbound.as_ref()[off..off + len] };
                if region.iter().any(|&b| b != OOL_POISON_OUTBOUND) {
                    return true;
                }
            }
            kernel::time::delay::fsleep(kernel::time::Delta::from_millis(i64::from(OOL_WRITE_POLL_MS)));
        }
        false
    }

    fn ool_write(&self, slot: &Mutex<Option<OolPair>>, off: usize, bytes: &[u8]) -> Result<()> {
        let guard = slot.lock();
        let buffers = guard.as_ref().ok_or(ENODEV)?;
        if off + bytes.len() > buffers.allocated {
            return Err(ENOSPC);
        }
        // SAFETY: the SEP reads this buffer only after the reply message is
        // sent, which happens after this returns; the driver is the only writer.
        unsafe { buffers.inbound.as_mut()[off..off + bytes.len()].copy_from_slice(bytes) };
        Ok(())
    }

    fn fail_xarm(&self, tag: u8) {
        self.send_xarm_reply(&crate::xarm::XarmReply {
            tag,
            status: xarm::STATUS_FAILED,
            length: 0,
            args: [0; 3],
        });
    }

    fn send_xarm_reply(&self, reply: &crate::xarm::XarmReply) {
        // Each of these used to fail the enclave's request without a trace.
        // Count them all and log the first, so a refusal is never silent.
        if reply.status == xarm::STATUS_FAILED && diag::xarm_failed() == 1 {
            dev_warn_ratelimited!(
                self.dev,
                "xART: answered a request (tag {:#04x}) with the failure status; diag/xarm_failures counts them\n",
                reply.tag
            );
        }
        let msg = crate::xarm::encode_xarm_reply(reply);
        let _ = self.send_reply(msg);
    }

    fn release_deferred_query(&self) {
        let Some(tag) = self.xarm.lock().deferred_query.take() else {
            return;
        };
        let mut reply = crate::xarm::XarmReply {
            tag,
            status: xarm::STATUS_OK,
            length: 0,
            args: [0; 3],
        };
        reply.args[0] = u8::from(PROTECTED_DATA_AVAILABLE);
        self.send_xarm_reply(&reply);
    }


    fn on_scrd(&self, msg: Message) {
        let mut probe = self.scrd_probe.lock();
        if probe.active {
            if probe.captured.len() < SCRD_MAX_CAPTURE {
                let _ = probe.captured.push(msg, GFP_KERNEL);
            }
            drop(probe);
            self.scrd_wq.notify_all();
        }
    }

    fn scrd_arm(&self) {
        let mut probe = self.scrd_probe.lock();
        probe.captured.clear();
        probe.active = true;
    }

    fn scrd_disarm(&self) -> KVec<Message> {
        let mut probe = self.scrd_probe.lock();
        probe.active = false;
        core::mem::take(&mut probe.captured)
    }

    fn scrd_wait(&self, ms: time::Msecs, until: usize) {
        let mut remaining = time::msecs_to_jiffies(ms);
        let mut guard = self.scrd_probe.lock();
        loop {
            if self.shutting_down.load(Relaxed)
                || guard.captured.len() >= until || remaining == 0 {
                return;
            }
            // A reply wait sleeps uninterruptibly: with a signal pending an
            // interruptible wait returns at once with the time left unchanged,
            // so this loop would spin until the reply came, or forever.
            match self
                .scrd_wq
                .wait_timeout(&mut guard, remaining)
            {
                CondVarTimeoutResult::Woken { jiffies } => remaining = jiffies,
                CondVarTimeoutResult::Signal { .. } => remaining = 0,
                CondVarTimeoutResult::Timeout => remaining = 0,
            }
        }
    }

    fn scrd_zero_buffers(&self) -> Result<()> {
        let mut guard = self.ool_scrd.lock();
        let pair: &mut Option<OolPair> = &mut guard;
        let buffers = pair.as_mut().ok_or(ENODEV)?;
        // SAFETY: nothing is outstanding on this endpoint here; the enclave
        // touches these only between a request going out and its reply, and the
        // driver is the only other accessor.
        unsafe {
            buffers.inbound.as_mut()[..buffers.allocated].fill(0);
            buffers.outbound.as_mut()[..buffers.allocated].fill(0);
        }
        Ok(())
    }

    fn scrd_transact(
        &self,
        _name: &'static CStr,
        request: u8,
        payload: &[u8],
        timeout_ms: time::Msecs,
    ) -> Option<(crate::scrd::ScrdReply, KVec<u8>)> {
        let _exchange = self.scrd_exchange_lock.lock();
        if !self.ool_registered(&self.ool_scrd) {
            return None;
        }
        if self.scrd_zero_buffers().is_err() {
            return None;
        }
        if !payload.is_empty() {
            if self.ool_write(&self.ool_scrd, 0, payload).is_err() {
                return None;
            }
        }

        let msg = crate::scrd::encode_scrd(request, payload.len());
        self.scrd_arm();
        if self.send(msg).is_err() {
            let _ = self.scrd_disarm();
            return None;
        }

        let mut examined = 0usize;
        let correlating = loop {
            self.scrd_wait(timeout_ms, examined + 1);
            let next = {
                let probe = self.scrd_probe.lock();
                let m = probe.captured.get(examined).copied();
                if m.is_some() {
                    examined += 1;
                }
                m
            };
            let Some(candidate) = next else { break None };
            let decoded = crate::scrd::decode_scrd_reply(&candidate);
            if decoded.request == request {
                break Some(candidate);
            }
            if examined >= SCRD_MAX_CAPTURE {
                break None;
            }
        };

        let leftover = self.scrd_disarm();
        let correlating = match correlating {
            Some(m) => Some(m),
            None => leftover
                .iter()
                .skip(examined)
                .copied()
                .find(|m| crate::scrd::decode_scrd_reply(m).request == request),
        };

        let raw = correlating?;

        let reply = crate::scrd::decode_scrd_reply(&raw);
        let mut response = KVec::new();
        if reply.response_size > 0 {
            if let Ok(bytes) = self.ool_read(&self.ool_scrd, 0, reply.response_size as usize) {
                response = bytes;
            }
        }
        Some((reply, response))
    }

    fn scrd_ready(&self) -> bool {
        if self.ool_registered(&self.ool_scrd) {
            return true;
        }
        if !self.endpoint_present(proto::EP_SCRD) {
            return false;
        }
        if self.register_ool(&self.ool_scrd).is_err() {
            return false;
        }
        true
    }

    fn scrd_command(&self, cmd: &crate::scrd::ScrdCommand) -> Option<(i32, KVec<u8>)> {
        let (reply, out) =
            self.scrd_transact(cmd.name(), cmd.request(), cmd.payload(), SCRD_TIMEOUT_MS)?;
        Some((reply.status, out))
    }

    fn establish_passcode_validated_context(
        &self,
        user: crate::sbio::UserId,
    ) -> Option<[u8; crate::scrd::SCRD_ACM_HANDLE_LEN]> {
        if !self.scrd_ready() {
            dev_err_ratelimited!(self.dev, "enrol: SCRD endpoint is not ready\n");
            return None;
        }

        let (special, secret) = {
            let guard = self.enrol_material.lock();
            let material = guard.as_ref()?;
            let mut copy = KVec::new();
            copy.extend_from_slice(&material.secret, GFP_KERNEL).ok()?;
            (material.special, Secret(copy))
        };

        let Some((status, _)) = self.scrd_command(&crate::scrd::scrd_initialize()) else {
            dev_err_ratelimited!(self.dev, "enrol: SCRD initialize did not answer\n");
            return None;
        };
        if status != 0 {
            dev_err_ratelimited!(self.dev, "enrol: SCRD initialize refused (status {})\n", status);
            return None;
        }

        let Some((status, out)) = self.scrd_command(&crate::scrd::scrd_context_create_tracked(user.value())) else {
            dev_err_ratelimited!(self.dev, "enrol: SCRD context create did not answer\n");
            return None;
        };
        if status != 0 || out.len() < crate::scrd::SCRD_ACM_HANDLE_LEN {
            dev_err_ratelimited!(self.dev, "enrol: SCRD context create failed (status {}, {} reply bytes)\n", status, out.len());
            return None;
        }
        let mut acm_handle = [0u8; crate::scrd::SCRD_ACM_HANDLE_LEN];
        acm_handle.copy_from_slice(&out[..crate::scrd::SCRD_ACM_HANDLE_LEN]);

        let Some((status, _)) = self.scrd_command(&crate::scrd::scrd_context_externalize(&acm_handle)) else {
            dev_err_ratelimited!(self.dev, "enrol: SCRD context externalize did not answer\n");
            return None;
        };
        if status != 0 {
            dev_err_ratelimited!(self.dev, "enrol: SCRD context externalize refused (status {})\n", status);
            return None;
        }

        let Some(out) = self.sks_send(self.sks_req_verify_secret(special, &secret, &acm_handle)) else {
            dev_err_ratelimited!(self.dev, "enrol: SKS verify-secret did not answer\n");
            return None;
        };
        if out.reply.status != 0 {
            dev_err_ratelimited!(self.dev, "enrol: SKS verify-secret refused (status {})\n", out.reply.status);
            return None;
        }

        let Some((status, out)) = self.scrd_command(&crate::scrd::scrd_verify_touchid_enrollment(&acm_handle)) else {
            dev_err_ratelimited!(self.dev, "enrol: SCRD Touch ID policy check did not answer\n");
            return None;
        };
        let satisfied = out.len() >= 4
            && u32::from_le_bytes([out[0], out[1], out[2], out[3]]) != 0;
        if status != 0 || !satisfied {
            dev_err_ratelimited!(self.dev, "enrol: SCRD Touch ID policy rejected (status {}, satisfied {})\n", status, satisfied);
            return None;
        }

        Some(acm_handle)
    }

    fn establish_scrd_match_context(&self, user: crate::sbio::UserId) -> bool {
        if !self.scrd_ready() {
            return false;
        }
        let Some(du) = crate::sks::DesignateUser::new(SBIO_PROBE_USER_ID) else {
            return false;
        };
        let special = du.special_handle();
        let stored = match keybag::read(keybag::Slot::Identity) {
            Ok(keybag::State::Present(s)) => s,
            _ => {
                return false;
            }
        };

        let established = (|| -> Option<[u8; crate::scrd::SCRD_ACM_HANDLE_LEN]> {
            let (status, _) = self.scrd_command(&crate::scrd::scrd_initialize())?;
            if status != 0 {
                return None;
            }
            let (status, out) =
                self.scrd_command(&crate::scrd::scrd_context_create_tracked(user.value()))?;
            if status != 0 || out.len() < crate::scrd::SCRD_ACM_HANDLE_LEN {
                return None;
            }
            let mut acm_handle = [0u8; crate::scrd::SCRD_ACM_HANDLE_LEN];
            acm_handle.copy_from_slice(&out[..crate::scrd::SCRD_ACM_HANDLE_LEN]);
            let (status, _) = self.scrd_command(&crate::scrd::scrd_context_externalize(&acm_handle))?;
            if status != 0 {
                return None;
            }
            let out = self.sks_send(self.sks_req_verify_secret(special, stored.secret(), &acm_handle))?;
            if out.reply.status != 0 {
                return None;
            }
            let (status, out) =
                self.scrd_command(&crate::scrd::scrd_verify_touchid_enrollment(&acm_handle))?;
            let satisfied = out.len() >= 4 && u32::from_le_bytes([out[0], out[1], out[2], out[3]]) != 0;
            if status != 0 || !satisfied {
                return None;
            }
            Some(acm_handle)
        })();
        let ok = established.is_some();
        let live = match established {
            Some(handle) => {
                let mut contexts = self.cold_match_contexts.lock();
                if contexts.push(handle, GFP_KERNEL).is_err() {
                    dev_warn_ratelimited!(self.dev, "scrd: could not track the new cold-match credential\n");
                }
                contexts.len()
            }
            None => self.cold_match_contexts.lock().len(),
        };
        dev_info_ratelimited!(self.dev, "scrd: cold-match credential established {} ({} live)\n", ok, live);
        ok
    }

    /// Deletes every cold-match credential this boot set up. Returns how many
    /// the enclave deleted and how many it refused. Logs counts only.
    pub(crate) fn retire_cold_match_contexts(&self) -> (usize, usize) {
        let old = core::mem::replace(&mut *self.cold_match_contexts.lock(), KVec::new());
        let (mut deleted, mut refused) = (0usize, 0usize);
        for handle in old.iter() {
            match self.scrd_command(&crate::scrd::scrd_context_delete(handle)) {
                Some((0, _)) => deleted += 1,
                other => {
                    refused += 1;
                    dev_warn_ratelimited!(
                        self.dev,
                        "scrd: deleting a cold-match credential answered {:?}\n",
                        other.map(|(status, _)| status)
                    );
                }
            }
        }
        dev_info_ratelimited!(
            self.dev,
            "scrd: retired {} cold-match credential(s), {} refused\n",
            deleted,
            refused
        );
        (deleted, refused)
    }

    /// Opens the evaluation one verify matches under: a fresh tracked
    /// context for `user`, externalised, whose unlock policy preflight leaves
    /// exactly a biometric match outstanding. The context is not satisfied in
    /// advance; the match satisfies it. Logs metadata only.
    pub(crate) fn open_unlock_evaluation(
        &self,
        user: crate::sbio::UserId,
    ) -> Option<[u8; crate::scrd::SCRD_ACM_HANDLE_LEN]> {
        if !self.scrd_ready() {
            dev_err_ratelimited!(self.dev, "scrd: unlock evaluation: endpoint not ready\n");
            return None;
        }
        let (status, _) = self.scrd_command(&crate::scrd::scrd_initialize())?;
        if status != 0 {
            dev_err_ratelimited!(self.dev, "scrd: unlock evaluation: initialize answered {}\n", status);
            return None;
        }
        let (status, out) =
            self.scrd_command(&crate::scrd::scrd_context_create_tracked(user.value()))?;
        let Some(handle) = crate::scrd::tracked_create_reference(&out).filter(|_| status == 0) else {
            dev_err_ratelimited!(
                self.dev,
                "scrd: unlock evaluation: tracked create answered {} with {} reply bytes\n",
                status,
                out.len()
            );
            return None;
        };
        let opened = (|| -> Option<()> {
            let (status, _) = self.scrd_command(&crate::scrd::scrd_context_externalize(&handle))?;
            if status != 0 {
                dev_err_ratelimited!(self.dev, "scrd: unlock evaluation: externalize answered {}\n", status);
                return None;
            }
            let cmd = crate::scrd::scrd_verify_unlock_policy(&handle, user.value(), true);
            let request_len = cmd.payload().len();
            let (status, out) = self.scrd_command(&cmd)?;
            let answer = crate::scrd::parse_policy_answer(&out);
            let generation = self.match_context_generation.load(Relaxed).wrapping_add(1);
            match answer {
                Some(a) => dev_info_ratelimited!(
                    self.dev,
                    "scrd: unlock preflight: policy device-owner-biometry, user {}, request {:#x}, parameters 1, status {}, satisfied {}, requirement {:?}, generation {}\n",
                    user.value(),
                    request_len,
                    status,
                    a.satisfied,
                    a.requirement,
                    generation
                ),
                None => dev_err_ratelimited!(
                    self.dev,
                    "scrd: unlock preflight: status {}, malformed answer of {} bytes\n",
                    status,
                    out.len()
                ),
            }
            let a = answer?;
            // A biometric match must be outstanding. Flags bit 0 is opaque
            // mechanism metadata the backend may set; nothing else may be.
            let admitted = matches!(
                a.requirement,
                Some((ty, 1, flags)) if ty == crate::scrd::SCRD_REQUIREMENT_BIOMETRY_MATCHED && flags & !1 == 0
            );
            if status != 0 || a.satisfied || !admitted {
                return None;
            }
            if a.requirement.is_some_and(|(_, _, flags)| flags & 1 != 0) {
                dev_info_ratelimited!(self.dev, "scrd: unlock preflight: requirement carries acm-mechanism-bit0\n");
            }
            Some(())
        })();
        if opened.is_none() {
            self.close_unlock_evaluation(&handle);
            return None;
        }
        self.match_context_generation.fetch_add(1, Relaxed);
        Some(handle)
    }

    /// The final, non-preflight unlock policy after an enrolled match. Only a
    /// satisfied answer completes the evaluation.
    pub(crate) fn complete_unlock_evaluation(
        &self,
        handle: &[u8; crate::scrd::SCRD_ACM_HANDLE_LEN],
        user: crate::sbio::UserId,
    ) -> bool {
        let answer = self
            .scrd_command(&crate::scrd::scrd_verify_unlock_policy(handle, user.value(), false))
            .map(|(status, out)| (status, crate::scrd::parse_policy_answer(&out)));
        let satisfied = matches!(answer, Some((0, Some(a))) if a.satisfied);
        dev_info_ratelimited!(
            self.dev,
            "scrd: unlock policy after the match: satisfied {}, generation {}\n",
            satisfied,
            self.match_context_generation.load(Relaxed)
        );
        satisfied
    }

    pub(crate) fn close_unlock_evaluation(&self, handle: &[u8; crate::scrd::SCRD_ACM_HANDLE_LEN]) {
        let _ = self.delete_unlock_evaluation(handle);
    }

    pub(crate) fn delete_unlock_evaluation(&self, handle: &[u8; crate::scrd::SCRD_ACM_HANDLE_LEN]) -> Result<()> {
        match self.scrd_command(&crate::scrd::scrd_context_delete(handle)) {
            Some((0, _)) => Ok(()),
            other => {
                self.key_context_failed.store(true, Relaxed);
                dev_warn_ratelimited!(
                    self.dev,
                    "scrd: deleting the unlock evaluation answered {:?}; private key operations disabled until reboot\n",
                    other.map(|(status, _)| status)
                );
                Err(EIO)
            }
        }
    }


    fn hwrng_fill(&self, buf: *mut u8, max: usize, wait: bool) -> c_int {
        if !wait {
            return 0;
        }

        let words = core::cmp::min(max / 4, RNG_MAX_WORDS_PER_READ);
        let mut written: usize = 0;
        let mut failure: Option<Error> = None;

        for i in 0..words {
            if self.rng_shutdown.load(Relaxed) {
                failure = Some(ENODEV);
                break;
            }
            // Only a read that has nothing yet waits while requests are held.
            let word = if written == 0 {
                self.get_entropy_word_blocking()
            } else {
                self.get_entropy_word()
            };
            match word {
                Ok(value) => {
                    // SAFETY: the core guarantees `buf` is valid for `max`
                    // bytes and aligned for any type, and `i < max / 4`.
                    unsafe { buf.add(i * 4).cast::<u32>().write_unaligned(value) };
                    written += 4;
                }
                Err(e) => {
                    failure = Some(e);
                    break;
                }
            }
        }

        let e = match control::read_end(written, failure) {
            control::ReadEnd::Bytes(n) => return n as c_int,
            // Held back for as long as the read could wait, or interrupted.
            control::ReadEnd::Held(e) => return e.to_errno(),
            control::ReadEnd::Failed(e) => e,
        };
        let n = self.rng_failures.load(Relaxed).wrapping_add(1);
        self.rng_failures.store(n, Relaxed);
        if n == 1 {
            dev_err_ratelimited!(
                self.dev,
                "hwrng: read produced nothing ({:?}); further failures will not be logged\n",
                e
            );
        }
        e.to_errno()
    }

    fn register_hwrng(&self) -> Result<()> {
        let mut guard = self.rng.lock();
        if guard.is_some() {
            return Ok(());
        }

        let mut handle = hwrng::HwRngHandle::new()?;
        let ctx = core::ptr::from_ref(self).cast_mut().cast::<c_void>();

        // SAFETY: `ctx` is this `SepData`, which is kept alive by the `Arc` held
        // in the driver's private data. `remove()` sets `rng_shutdown`, wakes
        // any blocked draw, and unregisters, and `hwrng_unregister()` blocks
        // until the core has finished with the device, all before that `Arc`
        // can be dropped. So the pointer cannot outlive the registration.
        unsafe { handle.register(ctx, hwrng_read_trampoline) }?;

        *guard = Some(handle);
        Ok(())
    }


    fn drain(&self) -> u32 {
        let mut n: u32 = 0;
        while let Some(msg) = self.rx.pop() {
            self.dispatch(msg);
            n += 1;
        }
        if n > 0 {
            // single writer, single instance: a load/store pair suffices
            let total = self.rx_count.load(Relaxed).wrapping_add(u64::from(n));
            self.rx_count.store(total, Relaxed);
        }
        n
    }


    fn arm_settle(this: &Arc<SepData>) {
        if this.shutting_down.load(Relaxed) {
            return;
        }
        let delay = time::msecs_to_jiffies(SETTLE_MS);
        let _ = workqueue::system_dfl()
            .enqueue_delayed::<Arc<SepData>, SETTLE_WORK_ID>(this.clone(), delay);
    }

    fn settle_tick(this: &Arc<SepData>) {
        match this.phase.load(Relaxed) {
            PHASE_ATTACH => Self::tick_attach(this),
            PHASE_EXCHANGE => Self::tick_exchange(this),
            PHASE_READY => Self::tick_ready(this),
            _ => {},
        }
    }

    fn tick_attach(this: &Arc<SepData>) {
        let now = this.rx_count.load(Relaxed);

        // HW: the SEP takes ~265 ms to answer a warm registration, and on the
        // cold-boot path J316s starts discovery ~370 ms after the IMG4 ack.
        // The boot acks count as traffic, so wait for the first endpoint, not
        // the first message, or a quiet gap after IMG4 ends the attach early.
        if this.endpoint_count() == 0 {
            let ticks = this.settle_idle_ticks.load(Relaxed).wrapping_add(1);
            this.settle_idle_ticks.store(ticks, Relaxed);
            if ticks.saturating_mul(u64::from(SETTLE_MS)) < u64::from(FIRST_RESPONSE_MS) {
                Self::arm_settle(this);
                return;
            }
        }

        if now != this.settle_mark.load(Relaxed) {
            this.settle_mark.store(now, Relaxed);
            Self::arm_settle(this);
            return;
        }

        if this.endpoint_count() == 0 {
            dev_err_ratelimited!(
                this.dev,
                "attach: {} messages received but no endpoint advertised; xART and key store cannot run\n",
                now
            );
            diag::attach(false);
            diag::bringup_ended();
            this.finish_bringup();
            return;
        }
        dev_info_ratelimited!(
            this.dev,
            "attach: {} endpoints advertised in {} messages\n",
            this.endpoint_count(),
            now
        );
        diag::attach(true);

        this.prepare_os_uuid();

        this.control_survey();

        if let Err(e) = this.register_ool(&this.ool_xarm) {
            dev_err_ratelimited!(
                this.dev,
                "xarm: out-of-line buffer registration failed ({:?}); the persistent-state exchange cannot run\n",
                e
            );
            diag::bringup_ended();
            this.finish_bringup();
            return;
        }

        this.phase.store(PHASE_EXCHANGE, Relaxed);
        this.settle_mark.store(this.rx_count.load(Relaxed), Relaxed);
        this.settle_idle_ticks.store(0, Relaxed);
        this.release_deferred_query();
        Self::arm_settle(this);
    }

    fn tick_exchange(this: &Arc<SepData>) {
        let now = this.rx_count.load(Relaxed);

        if now != this.settle_mark.load(Relaxed) {
            this.settle_mark.store(now, Relaxed);
            this.settle_idle_ticks.store(0, Relaxed);
            Self::arm_settle(this);
            return;
        }

        // The endpoint ladder climbs 7 -> 11 (sbio, scrd) -> 12 (the key
        // store, EP 0x12). The SEP can advertise the key store a beat after it
        // falls briefly quiet following scrd, so treating any count past the
        // pre-exchange baseline as "done" races that last step and strands the
        // boot at 11 endpoints with no key store. Wait through quiescence until
        // the key store itself is advertised, bounded by the same
        // EXCHANGE_TIMEOUT_MS. A boot that already has it exits immediately, so
        // healthy boots are unaffected; this also drops the hardcoded endpoint
        // count, which is not portable across SoCs.
        if !this.endpoint_present(proto::EP_SKS) {
            let ticks = this.settle_idle_ticks.load(Relaxed).wrapping_add(1);
            this.settle_idle_ticks.store(ticks, Relaxed);
            if ticks.saturating_mul(u64::from(SETTLE_MS)) < u64::from(EXCHANGE_TIMEOUT_MS) {
                Self::arm_settle(this);
                return;
            }
        }

        if this.keystore_only.load(Relaxed) {
            this.run_keystore_bringup();
            diag::keystore_ended();
        } else {
            this.run_bringup();
            diag::bringup_ended();
        }
        diag::log_summary(&this.dev);
        this.finish_bringup();
        // As macOS does once the enclave is up.
        Self::queue_stac_online(this.clone());
        // The enclave's account of the LocalPolicy that admitted this boot.
        Self::queue_pnon_digest(this.clone());
        // As macOS does at boot, these are checked and sent nothing.
        this.sse_attach();
        this.hibe_attach();
        this.skdl_attach();
        Self::arm_settle(this);
    }

    // Publish completion under the same lock that trusted consumers wait on.
    // PHASE_READY ends discovery/initialisation; it is not proof of success.
    fn finish_bringup(&self) {
        let _guard = self.endpoints.lock();
        self.phase.store(PHASE_READY, Relaxed);
        self.trusted_ready.notify_all();
    }

    /// Completes the original attach without sending a second registration.
    fn complete_attach(&self) -> Result<()> {
        self.attach_phase.lock().complete(|| self.finish_attach())
    }

    fn finish_attach(&self) -> Result<()> {
        self.wait_for_trusted_ready()?;

        // Wait out a cached-keybag consumer before enabling persistent writes.
        // All opens below resolve against PID 1's current (real) root.
        let mut cached = self.identity_keybag.lock();
        let mut host = store::Store::open()?;
        let index = bio::IdentityIndex::load(&mut host)?;
        let mut store = self.store.lock();
        // Drop the read-only block handle before reopening the owner writable.
        // Failure leaves a usable read-only store when it can be reopened.
        *store = None;
        let reopened = (|| {
            let mut new = xart_store::Store::open_owner(
                true,
                module_parameters::xart_start_sector.value(),
            )?;
            if new.writes_enabled() {
                if let Some(level) = xart_guard::level(
                    module_parameters::xart_guard.value(),
                    self.profile.xart_guard,
                ) {
                    new.set_guard(level)?;
                }
            }
            Ok(new)
        })();
        let new = match reopened {
            Ok(new) => new,
            Err(e) => {
                *store = xart_store::Store::open_owner(
                    false,
                    module_parameters::xart_start_sector.value(),
                )
                .ok();
                // diag/xart must not keep reading "enabled" (keystore_only=2
                // without a guard opened the store writable at probe).
                diag::xart_state(false, None);
                return Err(e);
            }
        };
        let writable = new.writes_enabled();
        let guard = new.guard_state();
        *store = Some(new);
        drop(store);
        *self.host_store.lock() = Some(host);
        *self.bio_index.lock() = index;
        diag::xart_state(writable, guard);
        self.keystore_only.store(false, Relaxed);
        *cached = None;
        drop(cached);

        // The default path's ordering stays in run_bringup. SKS has already
        // been initialised, so completion resumes immediately after that step.
        self.finish_keystore_bringup();
        diag::bringup_ended();
        Ok(())
    }

    /// Trusted-key callbacks can run as soon as probe publishes the source,
    /// while endpoint discovery and key-store initialisation are still running.
    /// Wait only in those consumers, leaving bringup free to establish the key.
    pub(crate) fn wait_for_trusted_ready(&self) -> Result<()> {
        let mut remaining = time::msecs_to_jiffies(EXCHANGE_TIMEOUT_MS);
        let mut guard = self.endpoints.lock();
        loop {
            if self.shutting_down.load(Relaxed) {
                return Err(ENODEV);
            }
            if self.phase.load(Relaxed) == PHASE_READY {
                // A completed but failed attach must not look ready.
                if !guard.eps.iter().any(|e| e.id == proto::EP_SKS) {
                    return Err(ENODEV);
                }
                if !self.sks_initialized.load(Relaxed) {
                    return Err(EIO);
                }
                return Ok(());
            }
            if remaining == 0 {
                dev_err_ratelimited!(self.dev, "trusted-keys: key-store bringup timed out\n");
                return Err(ETIMEDOUT);
            }
            match self.trusted_ready.wait_interruptible_timeout(&mut guard, remaining) {
                CondVarTimeoutResult::Woken { jiffies } => remaining = jiffies,
                CondVarTimeoutResult::Signal { .. } => return Err(ERESTARTSYS),
                CondVarTimeoutResult::Timeout => remaining = 0,
            }
        }
    }

    fn endpoint_present(&self, id: u8) -> bool {
        self.endpoints.lock().eps.iter().any(|e| e.id == id)
    }

    fn endpoint_count(&self) -> usize {
        self.endpoints.lock().eps.len()
    }

    fn dispatch(&self, msg: Message) {
        let f = proto::decode(&msg);

        match f.ep {
            proto::EP_DISCOVER => self.on_discovery(msg, f),

            proto::EP_SHMEM => {},

            proto::EP_CONTROL => self.on_control(msg, f),

            xarm::EP_XARM => self.on_xarm(msg),

            proto::EP_SBIO => self.on_sbio(msg),

            proto::EP_SKS => self.on_sks(msg),

            proto::EP_SCRD => self.on_scrd(msg),

            stac::EP_STAC => self.on_stac(msg),

            pnon::EP_PNON => self.on_pnon(msg),

            sse::EP_SSE => self.on_sse(msg),

            hibe::EP_HIBE => self.on_hibe(msg),

            skdl::EP_SKDL => self.on_skdl(msg),

            proto::EP_BOOT => self.on_boot(msg),

            ep => {
                // Counted in diag/unhandled_messages; logged the first few
                // times only, so a chatty endpoint cannot flood the log.
                if diag::unhandled() <= UNHANDLED_LOG_LIMIT {
                    dev_info_ratelimited!(self.dev, "rx: unhandled endpoint {:#04x} (msg0 {:#018x})\n", ep, msg.msg0);
                }
            }
        }
    }

    fn on_control(&self, msg: Message, f: proto::Fields) {
        if f.ty != proto::CONTROL_REPLY_TYPE {
            self.control.lock().wake_note(f.ty);
            self.control_wq.notify_all();
            return;
        }

        let reply = proto::ControlReply::from_message(&msg);
        let disposition = self.control.lock().deliver(reply);

        match disposition {
            control::Delivery::Entropy => {
                self.control_wq.notify_all();
            }
            control::Delivery::EntropyLate { recovered: true } => {
                dev_info_ratelimited!(
                    self.dev,
                    "hwrng: the enclave answered a request that had timed out; reads resume\n"
                );
                // A blocking hwrng reader may be waiting for this.
                self.control_wq.notify_all();
            }
            control::Delivery::EntropyLate { recovered: false } => {}
            control::Delivery::Matched => self.control_wq.notify_all(),
            control::Delivery::Unmatched => {}
        }
    }

    fn on_discovery(&self, msg: Message, f: proto::Fields) {
        let mut table = self.endpoints.lock();
        match f.ty {
            proto::DISCOVER_TYPE_DESCRIPTOR | proto::DISCOVER_TYPE_CONFIG => {
                let _ = table.slot(f.param);
                self.stac_discovered(&f);
                self.pnon_discovered(&f);
                self.sse_discovered(&f);
                self.hibe_discovered(&f);
                self.skdl_discovered(&f);
                diag::endpoints(table.eps.len());
                if f.ty == proto::DISCOVER_TYPE_DESCRIPTOR {
                    diag::endpoint(f.param, (msg.msg0 >> 32) as u32);
                }
                // One summary line follows at the end of the bring-up, and
                // diag/endpoint_names lists them; keep the per-message trace
                // at debug level.
                dev_dbg!(
                    self.dev,
                    "discover: endpoint {:#04x} (type {}, msg0 {:#018x}); {} known\n",
                    f.param,
                    f.ty,
                    msg.msg0,
                    table.eps.len()
                );
            }
            ty => dev_dbg!(self.dev, "discover: other type {} (msg0 {:#018x})\n", ty, msg.msg0),
        }
    }


    fn remove(&self) {
        self.shutting_down.store(true, Relaxed);
        {
            let _guard = self.endpoints.lock();
            self.trusted_ready.notify_all();
        }
        bio::release(&mut self.bio_session.lock());
        self.worker_gone.notify_all();
        self.rng_shutdown.store(true, Relaxed);
        self.control_wq.notify_all();
        self.sbio_wq.notify_all();
        self.sks_wq.notify_all();
        self.scrd_wq.notify_all();
        pm::unregister();
        // Release any capture start still held for a resume that will not be
        // reported now.
        self.sleep_finished();
        trusted::unregister();
        self.unregister_fv_kernel();

        let dev = self.bio_dev.lock().take();
        drop(dev);
        let dev = self.rearm_dev.lock().take();
        drop(dev);
        let dev = self.key_dev.lock().take();
        drop(dev);

        self.rng_shutdown.store(true, Relaxed);
        self.control_wq.notify_all();
        self.sbio_wq.notify_all();
        self.sks_wq.notify_all();
        self.scrd_wq.notify_all();
        self.stac_wq.notify_all();
        self.pnon_wq.notify_all();
        if let Some(mut handle) = self.rng.lock().take() {
            handle.unregister();
        }

        *self.mbox.lock() = None;

        // SAFETY: all seven pointers refer to pinned work fields in `self`.
        // Shutdown blocks requeueing, and the mailbox can no longer add work.
        unsafe {
            sep_cancel_work_sync(Work::raw_get(core::ptr::addr_of!(self.rx_work)).cast());
            sep_cancel_work_sync(Work::raw_get(core::ptr::addr_of!(self.enrol_work)).cast());
            sep_cancel_work_sync(Work::raw_get(core::ptr::addr_of!(self.verify_work)).cast());
            sep_cancel_work_sync(Work::raw_get(core::ptr::addr_of!(self.stac_work)).cast());
            sep_cancel_work_sync(Work::raw_get(core::ptr::addr_of!(self.pnon_work)).cast());
            sep_cancel_delayed_work_sync(
                DelayedWork::raw_as_work(core::ptr::addr_of!(self.settle_work)).cast(),
            );
            sep_cancel_delayed_work_sync(
                DelayedWork::raw_as_work(core::ptr::addr_of!(self.late_attach_work)).cast(),
            );
        }

        self.sensor_kept.store(false, Relaxed);
        sensor::power(false);
        sensor::unregister_driver();
        self.release_firmware_mapping();

        let _ = self.store.lock().take();
        let _ = self.host_store.lock().take();

        for slot in [
            &self.ool_xarm,
            &self.ool_sbio,
            &self.ool_sks,
            &self.ool_scrd,
            &self.ool_stac,
            &self.ool_pnon,
        ] {
            if let Some(buffers) = slot.lock().take() {
                if buffers.registered {
                    core::mem::forget(buffers);
                }
            }
        }

        if let Some(ring) = self.dma_ring.lock().take() {
            if self.sbio_ready.load(Relaxed) {
                core::mem::forget(ring);
            }
        }

        let mut guard = self.shmem.lock();
        if let Some(buf) = guard.take() {
            if self.registered.load(Relaxed) {
                core::mem::forget(buf);
            } else {
                drop(buf);
            }
        }
    }
}

/// # Safety
///
/// Each is called only by `bio_shim.c`, with the `*const SepData` handed to
/// `BioChardev::register`, which stays valid for the whole registered window.
unsafe extern "C" fn bio_open_trampoline(ctx: *mut c_void) -> c_int {
    // SAFETY: per the contract above.
    let this = unsafe { &*ctx.cast::<SepData>() };
    match this.bio_open() {
        Ok(()) => {
            if this.take_late_attach() {
                // SAFETY: as in `bio_ioctl_trampoline`: `ctx` is the pointer
                // the driver's own `Arc<SepData>` was made from, and that
                // `Arc` outlives every callback.
                let borrow =
                    unsafe { kernel::sync::ArcBorrow::<SepData>::from_raw(ctx.cast::<SepData>()) };
                SepData::queue_late_attach(borrow.into());
            }
            0
        }
        Err(e) => e.to_errno(),
    }
}

/// # Safety
unsafe extern "C" fn bio_release_trampoline(ctx: *mut c_void) {
    // SAFETY: per the contract above.
    let this = unsafe { &*ctx.cast::<SepData>() };
    this.bio_release();
}

/// # Safety
unsafe extern "C" fn bio_ioctl_trampoline(ctx: *mut c_void, cmd: c_uint, arg: c_ulong) -> c_long {
    // SAFETY: per the contract above.
    let this = unsafe { &*ctx.cast::<SepData>() };
    let handled = match this.bio_ioctl(cmd, arg) {
        Ok(h) => h,
        Err(e) => return e.to_errno() as c_long,
    };

    if handled.start_verify {
        // SAFETY: as for `start_enrol` below: same pointer, same guarantee.
        let borrow = unsafe { kernel::sync::ArcBorrow::<SepData>::from_raw(ctx.cast::<SepData>()) };
        SepData::queue_verify(borrow.into());
    }

    if handled.start_enrol {
        // SAFETY: `ctx` is the pointer the driver's own `Arc<SepData>` was made
        // from and that `Arc` outlives every callback; see the contract on the
        // registration. `ArcBorrow` is what turns that guarantee into an owned
        // reference with the refcount incremented rather than a second owner of
        // the same count.
        let borrow = unsafe { kernel::sync::ArcBorrow::<SepData>::from_raw(ctx.cast::<SepData>()) };
        SepData::queue_enrolment(borrow.into());
    }
    handled.ret as c_long
}

/// # Safety
unsafe extern "C" fn bio_ready_trampoline(ctx: *mut c_void) -> c_int {
    // SAFETY: per the contract above.
    let this = unsafe { &*ctx.cast::<SepData>() };
    c_int::from(this.bio_ready())
}

// /dev/sep-bio-rearm has no session: any number of root openers, no poll.

/// # Safety
unsafe extern "C" fn rearm_open_trampoline(ctx: *mut c_void) -> c_int {
    // SAFETY: per the contract above.
    let this = unsafe { &*ctx.cast::<SepData>() };
    if this.shutting_down.load(Relaxed) {
        return ENODEV.to_errno();
    }
    0
}

/// # Safety
unsafe extern "C" fn rearm_release_trampoline(_ctx: *mut c_void) {}

/// # Safety
unsafe extern "C" fn rearm_ioctl_trampoline(ctx: *mut c_void, cmd: c_uint, arg: c_ulong) -> c_long {
    // SAFETY: per the contract above.
    let this = unsafe { &*ctx.cast::<SepData>() };
    match this.rearm_ioctl(cmd, arg) {
        Ok(()) => 0,
        Err(e) => e.to_errno() as c_long,
    }
}

/// # Safety
unsafe extern "C" fn rearm_ready_trampoline(_ctx: *mut c_void) -> c_int {
    0
}

/// # Safety
///
/// Only ever called by the hwrng core, through `hwrng_shim.c`. `ctx` is the
/// `*const SepData` handed to `HwRngHandle::register`, which stays valid for
/// the whole registered window (see the ordering note in `remove()`), and
/// `data` is valid for `max` bytes.
unsafe extern "C" fn hwrng_read_trampoline(
    ctx: *mut c_void,
    data: *mut c_void,
    max: usize,
    wait: bool,
) -> c_int {
    // SAFETY: per the contract above.
    let this = unsafe { &*ctx.cast::<SepData>() };
    this.hwrng_fill(data.cast::<u8>(), max, wait)
}

impl MailCallback for SepData {
    type Data = Arc<SepData>;

    fn recv_message(data: <Self::Data as ForeignOwnable>::Borrowed<'_>, msg: Message) {
        if data.shutting_down.load(Relaxed) {
            return;
        }
        // The message was read from the mailbox with relaxed MMIO. Read the
        // buffers the enclave wrote before it only after that, as RTKit does.
        dma_mb(Read);
        if !data.rx.push(msg) && data.rx.dropped() == 1 {
            dev_err_ratelimited!(
                data.dev,
                "receive ring full; dropping messages (first was msg0 {:#018x})\n",
                msg.msg0
            );
        }

        let this: Arc<SepData> = data.into();
        let _ = workqueue::system_dfl().enqueue::<Arc<SepData>, 0>(this);
    }
}

impl WorkItem for SepData {
    type Pointer = Arc<SepData>;

    fn run(this: Arc<SepData>) {
        if this.shutting_down.load(Relaxed) {
            return;
        }
        if this.drain() > 0 {
            SepData::arm_settle(&this);
        }
    }
}

impl WorkItem<SETTLE_WORK_ID> for SepData {
    type Pointer = Arc<SepData>;

    fn run(this: Arc<SepData>) {
        if this.shutting_down.load(Relaxed) {
            return;
        }
        SepData::settle_tick(&this);
    }
}

impl WorkItem<LATE_ATTACH_WORK_ID> for SepData {
    type Pointer = Arc<SepData>;

    fn run(this: Arc<SepData>) {
        SepData::run_late_attach(&this);
    }
}

impl WorkItem<PNON_WORK_ID> for SepData {
    type Pointer = Arc<SepData>;

    fn run(this: Arc<SepData>) {
        this.run_pnon_digest();
    }
}

impl WorkItem<STAC_WORK_ID> for SepData {
    type Pointer = Arc<SepData>;

    fn run(this: Arc<SepData>) {
        this.run_stac_online();
    }
}

impl WorkItem<ENROL_WORK_ID> for SepData {
    type Pointer = Arc<SepData>;

    fn run(this: Arc<SepData>) {
        // Serialize saves before joining the suspend drain.
        let _writer = this.catacomb_writer.lock();
        let epoch = this.capture_epoch.load(Relaxed);
        if !this.shutting_down.load(Relaxed) && this.capture_begin() {
            this.run_enrolment();
            this.capture_end();
        }
        this.capture_worker_stopped(epoch);
    }
}

impl WorkItem<VERIFY_WORK_ID> for SepData {
    type Pointer = Arc<SepData>;

    fn run(this: Arc<SepData>) {
        // Serialize saves before joining the suspend drain.
        let _writer = this.catacomb_writer.lock();
        let epoch = this.capture_epoch.load(Relaxed);
        if !this.shutting_down.load(Relaxed) && this.capture_begin() {
            this.run_verify();
            this.capture_end();
        }
        this.capture_worker_stopped(epoch);
    }
}


struct SepDriver(Arc<SepData>, attach::Attribute);

// Deliberately not exported as a module alias: udev must not load the driver
// on its own. The loader service loads it once the iBoot system container's
// partition link exists, which the xART store needs at probe, and only after
// the owner has enabled it (or chosen read-only) for this Mac.
const OF_TABLE: kernel::device_id::IdArray<of::DeviceId, (), 1> =
    kernel::device_id::IdArray::new([(of::DeviceId::new(c"apple,sep"), &())]);

impl platform::Driver for SepDriver {
    type IdInfo = ();
    type Data<'bound> = SepDriver;

    const OF_ID_TABLE: Option<of::IdTable<()>> = Some(&OF_TABLE);

    fn pm_ops() -> Option<&'static bindings::dev_pm_ops> {
        // SAFETY: a static in `pm_shim.c`, never written.
        Some(unsafe { &*core::ptr::addr_of!(pm::sep_pm_ops) })
    }

    fn probe<'bound>(
        pdev: &'bound platform::Device<device::Core<'_>>,
        _info: Option<&'bound Self::IdInfo>,
    ) -> impl PinInit<Self::Data<'bound>, Error> + 'bound {
        let dev: &device::Device<device::Core<'_>> = pdev.as_ref();
        let sep_node = dt::DtNode::of_device(dev).ok_or(ENODEV)?;

        // J700's SEP DART window starts above 4 GiB. Leave the proven masks
        // on older platforms unchanged for the M1/M2 regression run.
        if profile::detect()?.wide_dma_mask {
            // SAFETY: probe has not allocated DMA memory for this device yet.
            unsafe { pdev.dma_set_mask_and_coherent(dma::DmaMask::new::<42>())? };
        }

        if dt::registration_already_sent(&sep_node) {
            dev_err_ratelimited!(
                dev,
                "the SEP was already booted or registered on this boot by an earlier load (marker property present). Linux cannot reset it and the handoff cannot be repeated, so reloading the driver is not supported: reboot to attach again. Not probing.\n"
            );
            return Err(EBUSY);
        }

        if let Err(e) = pm::keep_domains(dev) {
            dev_err_ratelimited!(dev, "could not keep the power domains on ({:?})\n", e);
            return Err(e);
        }

        let data = SepData::new(pdev)?;

        // As soon as the fixed facts are known, so a tool can watch the attach
        // from "pending" on. Diagnostics are not worth failing the probe for.
        let guard = data.with_store(|store| store.guard_state()).flatten();
        let policy = diag::Policy {
            trusted_keys: module_parameters::trusted_keys.value() != 0,
            filevault_api: module_parameters::filevault_api.value() != 0,
            xarm_trace: module_parameters::xarm_trace.value() != 0,
            refkey_v2: module_parameters::refkey_v2.value() != 0,
            identity_override: module_parameters::os_uuid_hi.value() != 0
                || module_parameters::os_uuid_lo.value() != 0,
        };
        if let Err(e) = diag::register(dev, data.profile, data.xart_writable(), guard, policy) {
            dev_warn_ratelimited!(dev, "diag: could not add the sysfs attributes ({:?})\n", e);
        }

        *data.mbox.lock() = Some(Mailbox::new_byname(dev, c"mbox", data.clone())?);

        data.attach(&sep_node)?;

        if module_parameters::filevault_api.value() != 0 {
            if let Err(e) = data.register_fv_kernel() {
                dev_err_ratelimited!(dev, "could not register the FileVault kernel API: {:?}\n", e);
            }
        }

        if module_parameters::trusted_keys.value() != 0 {
            if let Err(e) = trusted::register(data.clone()) {
                dev_warn_ratelimited!(data.dev, "trusted-keys: registration failed ({:?})\n", e);
            }
        }

        if let Err(e) = pm::register(data.clone()) {
            dev_warn_ratelimited!(
                data.dev,
                "sleep notifier registration failed ({:?}); a capture may run into a suspend\n",
                e
            );
        }

        if data.registered.load(Relaxed) {
            SepData::arm_settle(&data);
        }

        let attribute = match attach::Attribute::register(data.clone()) {
            Ok(attribute) => attribute,
            Err(e) => {
                data.remove();
                return Err(e);
            }
        };
        Ok(SepDriver(data, attribute))
    }
}

impl Drop for SepDriver {
    fn drop(&mut self) {
        self.1.unregister();
        self.0.remove();
    }
}


#[pin_data]
struct SepModule {
    // field-init order enables the DT nodes before the driver registers; a failure here aborts module init
    _dt: (),

    #[pin]
    _driver: driver::Registration<platform::Adapter<SepDriver>>,
}

impl kernel::InPlaceModule for SepModule {
    fn init(module: &'static ThisModule) -> impl PinInit<Self, Error> {
        try_pin_init!(Self {
            // The warm-attach target enables the SEP and its DART with a runtime
            // changeset; the cold-boot target describes them statically (its DART
            // is enabled before probe), so no changeset runs there.
            _dt: {
                match profile::detect() {
                    Ok(p) => {
                        if matches!(p.bootstrap, profile::Bootstrap::WarmRegister) {
                            dt::enable_sep_and_dart(p.dart_window)?;
                        }
                    }
                    // Without this the module loads, probes nothing and logs
                    // nothing on an unlisted SoC, which is indistinguishable
                    // from a driver that started correctly.
                    Err(_) => pr_info_ratelimited!(
                        "no platform profile for this SoC; the SEP stays disabled\n"
                    ),
                }
            },

            _driver <- driver::Registration::new(
                <Self as kernel::ModuleMetadata>::NAME,
                module,
            ),
        })
    }
}

module! {
    type: SepModule,
    name: "apple_sep",
    description: "Apple SEP coprocessor: warm attach, endpoint discovery and tag-correlated control endpoint",
    license: "Dual MIT/GPL",
    params: {
        trusted_keys: u8 {
            default: 1,
            description: "Register the SEP trusted-key source; 0 leaves it disabled",
        },
        filevault_api: u8 {
            default: 1,
            description: "Register the SEP FileVault kernel API; 0 leaves it disabled",
        },
        xart_start_sector: u64 {
            default: 0,
            description: "Expected 512-byte start sector of the APFS .gl extent in the iBoot container. Zero accepts the APFS result; nonzero must match it and cannot override it",
        },
        xart_guard: i8 {
            default: -1,
            description: "-1: the platform's level, 1 on M2 and M3 Macs and 0 on M1 Macs and the MacBook Neo; 0: off; 1: refuse the enclave's writes and deletes of xART session records this Linux install did not create; 2: also refuse writes to the shared root records",
        },
        os_uuid_hi: u64 {
            default: 0,
            description: "High 64 bits of an explicit xART OS UUID",
        },
        os_uuid_lo: u64 {
            default: 0,
            description: "Low 64 bits of an explicit xART OS UUID",
        },
        sensor_trace: u8 {
            default: 0,
            description: "Trace Touch ID sensor registration steps and timing, never sensor data",
        },
        sensor_test_fault: u8 {
            default: 0,
            description: "Test only: stop the first nonce-session sensor authentication of the boot after REGISTER_SERIAL, leaving the enclave an unfinished registration to recover from",
        },
        xarm_trace: u8 {
            default: 0,
            description: "Opt-in xART opcode/status trace without payloads or key material",
        },
        probe_owner_export: u8 {
            default: 0,
            description: "Opt-in J414s diagnostic: with zero live identities, select the system context and try saving the missing owner Catacomb",
        },
        key_selftest: u8 {
            default: 0,
            description: "Test only: let root run SEP_KEY_SELFTEST, which signs with an access-controlled /dev/sep-key key under deliberately unsuitable credential contexts to show the enclave refuses",
        },
        refkey_v2: u8 {
            default: 0,
            description: "Explicit recovery slot for a new identity-bag context; preserves the original machine ref-key",
        },
        j414s_persistent_enrol: u8 {
            default: 0,
            description: "Opt-in J414s enrollment test: export owner from the fresh context, then save user before master at completion",
        },
    },
}
