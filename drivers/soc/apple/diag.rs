// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Rust side of `diag_shim.c`: the passive diagnostics in sysfs.
//!
//! The driver reports each decision here as it makes it, and the shim caches
//! it for the `diag` attributes. Every call only stores a word, so none of them
//! can block a bring-up step, and a sysfs read never reaches back into the
//! driver or the SEP.

use kernel::{
    device,
    prelude::*,
    sync::atomic::{Atomic, Relaxed}, //
};

use crate::{
    profile,
    registration,
    xart_guard::MatchFacts, //
};

extern "C" {
    // `struct device *`, passed untyped: its lockdep map is a zero-sized
    // struct in some configurations, which Rust does not accept across FFI.
    fn sep_diag_register(
        dev: *mut c_void,
        profile: *const c_char,
        profile_len: usize,
        cold: bool,
        sepos13: bool,
        xart: bool,
    ) -> c_int;
    fn sep_diag_set_policy(
        trusted_keys: bool,
        filevault_api: bool,
        xarm_trace: bool,
        refkey_v2: bool,
        identity_override: bool,
    );
    fn sep_diag_set_identity_source(source: c_uint);
    fn sep_diag_set_attach(attached: bool);
    fn sep_diag_set_endpoints(count: c_uint);
    fn sep_diag_set_keystore_open();
    fn sep_diag_keystore_ended();
    fn sep_diag_set_xart(writable: bool);
    fn sep_diag_set_keybag(state: c_int);
    fn sep_diag_sensor_bind() -> c_uint;
    fn sep_diag_set_sensor_result(bind: c_uint, online: bool);
    fn sep_diag_set_bio_published();
    fn sep_diag_set_touchid(started: bool);
    fn sep_diag_set_sensor_attach(state: c_int);
    fn sep_diag_set_restore(state: c_int);
    fn sep_diag_set_xart_guard(level: c_uint, list_loaded: bool, matching: c_int);
    fn sep_diag_xart_guard_refused(matching: c_int);
    fn sep_diag_bringup_ended();
    fn sep_diag_set_endpoint(id: u8, name: u32);
    fn sep_diag_set_os_identity(source: c_int);
    fn sep_diag_set_hwrng(registered: bool);
    fn sep_diag_set_stac(state: c_int);
    fn sep_diag_set_pnon(state: c_int, digest: *const u8, len: usize);
    fn sep_diag_set_offer(endpoint: u8, state: c_int);
    fn sep_diag_sse_event() -> c_uint;
    fn sep_diag_xarm_failed() -> c_uint;
    fn sep_diag_unhandled() -> c_uint;
    fn sep_diag_log_summary(dev: *mut c_void);
}

/// The identity keybag, as `SEP_DIAG_KEYBAG_*` in `shim.h` spells it.
#[derive(Clone, Copy)]
pub(crate) enum Keybag {
    Present = 1,
    Missing = 2,
    Failed = 3,
}

/// Where the OS identity the driver answers xART's GET_OS_UUID with came from,
/// as `SEP_DIAG_OS_IDENTITY_*` in `shim.h` spells it.
#[derive(Clone, Copy)]
pub(crate) enum OsIdentity {
    /// The `os_uuid_hi`/`os_uuid_lo` parameters.
    Explicit = 1,
    /// `/chosen/apfs-preboot-uuid`.
    Chosen = 2,
    /// Neither: GET_OS_UUID is answered "unavailable".
    Unavailable = 3,
}

/// The enrolled fingerprints at attach, as `SEP_DIAG_RESTORE_*` in `shim.h`
/// spells it.
#[derive(Clone, Copy)]
pub(crate) enum Restore {
    /// The enclave listed them and the restore was proven: the driver logged
    /// "Touch ID: restored N".
    Restored = 1,
    /// The enclave lists no enrolled identity.
    Empty = 2,
    /// It lists some, but the restore could not be proven.
    Unproven = 3,
}

/// Boot-time policy, without identity values or key material.
pub(crate) struct Policy {
    pub trusted_keys: bool,
    pub filevault_api: bool,
    pub xarm_trace: bool,
    pub refkey_v2: bool,
    pub identity_override: bool,
}

/// The source selected by prepare_os_uuid, not the UUID itself.
pub(crate) enum IdentitySource {
    DeviceTree = 1,
    Override = 2,
    Missing = 3,
}

pub(crate) fn identity_source(source: IdentitySource) {
    // SAFETY: this only stores the source enum in the passive cache.
    unsafe { sep_diag_set_identity_source(source as c_uint) };
}

/// Adds the `diag` group to the SEP device, with the facts fixed at probe. The
/// group goes away when the driver unbinds.
///
/// `guard` is the xART guard as the store opened it: its effective level and
/// whether Linux's list loaded, or `None` when no guard runs.
pub(crate) fn register(
    dev: &device::Device<device::Bound>,
    profile: &profile::PlatformProfile,
    xart_writable: bool,
    guard: Option<(u8, bool)>,
    policy: Policy,
) -> Result {
    // Exhaustive, so a new bootstrap or key store fails to build here until
    // the ABI has a value for it.
    let cold = match profile.bootstrap {
        profile::Bootstrap::Boot => true,
        profile::Bootstrap::WarmRegister => false,
    };
    let sepos13 = match profile.key_store {
        profile::KeyStore::Sepos13 { .. } => true,
        profile::KeyStore::Variant5 => false,
    };
    // The facts `matching` is derived from, kept for the guard's refusals.
    // Probe runs before the guard can refuse anything.
    XART_WRITABLE.store(xart_writable, Relaxed);
    let (level, list_loaded) = guard.unwrap_or((0, false));
    GUARD_LEVEL.store(u32::from(level), Relaxed);
    GUARD_LIST.store(list_loaded, Relaxed);
    REFUSALS.store(0, Relaxed);
    // SAFETY: no preconditions; stores three words.
    unsafe { sep_diag_set_xart_guard(c_uint::from(level), list_loaded, matching() as c_int) };

    // SAFETY: the facts are fixed before the attribute group is published.
    unsafe {
        sep_diag_set_policy(
            policy.trusted_keys,
            policy.filevault_api,
            policy.xarm_trace,
            policy.refkey_v2,
            policy.identity_override,
        )
    };

    // SAFETY: `dev` is bound for the duration of the call, which is what devres
    // needs. `profile.name` is a `str`, valid for reads of `profile.name.len()`
    // bytes; the shim copies at most that many, needs no NUL terminator, and
    // keeps no pointer to it.
    let ret = unsafe {
        sep_diag_register(
            dev.as_raw().cast(),
            profile.name.as_ptr().cast::<c_char>(),
            profile.name.len(),
            cold,
            sepos13,
            xart_writable,
        )
    };
    kernel::error::to_result(ret)
}

pub(crate) fn xart_state(writable: bool, guard: Option<(u8, bool)>) {
    XART_WRITABLE.store(writable, Relaxed);
    let (level, loaded) = guard.unwrap_or((0, false));
    GUARD_LEVEL.store(u32::from(level), Relaxed);
    GUARD_LIST.store(loaded, Relaxed);
    REFUSALS.store(0, Relaxed);
    // SAFETY: each setter only publishes diagnostic state.
    unsafe {
        sep_diag_set_xart(writable);
        sep_diag_set_xart_guard(c_uint::from(level), loaded, matching() as c_int);
    }
}

pub(crate) fn keystore_ended() {
    // SAFETY: no preconditions; updates one diagnostic word.
    unsafe { sep_diag_keystore_ended() };
}

pub(crate) fn attach(attached: bool) {
    // SAFETY: no preconditions; stores one word.
    unsafe { sep_diag_set_attach(attached) };
}

pub(crate) fn endpoints(count: usize) {
    let count = c_uint::try_from(count).unwrap_or(c_uint::MAX);
    // SAFETY: no preconditions; stores one word.
    unsafe { sep_diag_set_endpoints(count) };
}

pub(crate) fn keystore_open() {
    // SAFETY: no preconditions; stores one word.
    unsafe { sep_diag_set_keystore_open() };
}

pub(crate) fn keybag(state: Keybag) {
    // SAFETY: no preconditions; stores one word.
    unsafe { sep_diag_set_keybag(state as c_int) };
}

/// Which bind of the sensor a bring-up ran against.
#[derive(Clone, Copy)]
pub(crate) struct SensorBind(c_uint);

/// Taken as a sensor bring-up begins, so that its result is dropped if the
/// sensor has been unbound or bound again by the time it finishes.
pub(crate) fn sensor_bind() -> SensorBind {
    // SAFETY: no preconditions; loads one word.
    SensorBind(unsafe { sep_diag_sensor_bind() })
}

/// The result of a sensor bring-up, attach-time or later.
pub(crate) fn sensor_brought_up(bind: SensorBind, online: bool) {
    // SAFETY: no preconditions; updates one word.
    unsafe { sep_diag_set_sensor_result(bind.0, online) };
}

pub(crate) fn bio_published() {
    // SAFETY: no preconditions; updates one word.
    unsafe { sep_diag_set_bio_published() };
}

/// Touch ID activation, run by the first open of `/dev/sep-bio`, finished.
pub(crate) fn touchid(started: bool) {
    // SAFETY: no preconditions; stores one word.
    unsafe { sep_diag_set_touchid(started) };
}

/// How far the attach of the sensor got, `SEP_DIAG_SENSOR_ATTACH_*`.
pub(crate) fn sensor_attach(state: registration::AttachState) {
    // SAFETY: no preconditions; stores one word.
    unsafe { sep_diag_set_sensor_attach(state as c_int) };
}

/// What became of the enrolled fingerprints at attach.
pub(crate) fn restore(state: Restore) {
    // SAFETY: no preconditions; stores one word.
    unsafe { sep_diag_set_restore(state as c_int) };
}

static XART_WRITABLE: Atomic<bool> = Atomic::new(false);
static GUARD_LEVEL: Atomic<u32> = Atomic::new(0);
static GUARD_LIST: Atomic<bool> = Atomic::new(false);
static REFUSALS: Atomic<u32> = Atomic::new(0);

/// What a fingerprint match can expect of the xART guard, from the facts
/// [`register`] stored and the refusals since.
fn matching() -> crate::xart_guard::Matching {
    let level = GUARD_LEVEL.load(Relaxed);
    MatchFacts {
        xart_writable: XART_WRITABLE.load(Relaxed),
        guard: (level != 0).then(|| {
            (
                u8::try_from(level).unwrap_or(u8::MAX),
                GUARD_LIST.load(Relaxed),
            )
        }),
        refusals: REFUSALS.load(Relaxed),
    }
    .matching()
}

/// The xART guard refused one of the enclave's writes or deletes.
pub(crate) fn xart_guard_refused() {
    REFUSALS.fetch_add(1, Relaxed);
    // SAFETY: no preconditions; updates two words.
    unsafe { sep_diag_xart_guard_refused(matching() as c_int) };
}

/// The boot-time bring-up is over, however far it got.
pub(crate) fn bringup_ended() {
    // SAFETY: no preconditions; updates two words.
    unsafe { sep_diag_bringup_ended() };
}

/// An endpoint the SEP advertised, with the four-character name its
/// descriptor carries in the upper half of `msg0`.
pub(crate) fn endpoint(id: u8, name: u32) {
    // SAFETY: no preconditions; stores one word and sets one bit.
    unsafe { sep_diag_set_endpoint(id, name) };
}

pub(crate) fn os_identity(source: OsIdentity) {
    // SAFETY: no preconditions; stores one word.
    unsafe { sep_diag_set_os_identity(source as c_int) };
}

/// The hwrng survey registered the enclave's entropy source, or gave up.
pub(crate) fn hwrng(registered: bool) {
    // SAFETY: no preconditions; stores one word.
    unsafe { sep_diag_set_hwrng(registered) };
}

/// The trusted-accessory power-state report, as `SEP_DIAG_STAC_*` in
/// `shim.h` spells it.
#[derive(Clone, Copy)]
pub(crate) enum Stac {
    /// The service took the online state.
    Online = 1,
    /// The enclave does not offer the endpoint as the driver knows it.
    Absent = 2,
    /// The last report was refused, malformed, unanswered or not sent.
    Failed = 3,
}

/// The outcome of a trusted-accessory power-state report; an accepted one is
/// also counted.
pub(crate) fn stac(state: Stac) {
    // SAFETY: no preconditions; updates two words.
    unsafe { sep_diag_set_stac(state as c_int) };
}

/// The boot-policy digest read, as `SEP_DIAG_PNON_*` in `shim.h` spells it.
#[derive(Clone, Copy)]
pub(crate) enum Pnon {
    /// The service returned the blessed LocalPolicy nonce digest.
    Read = 1,
    /// The enclave does not offer the endpoint as the driver knows it.
    Absent = 2,
    /// The read was refused, malformed, unanswered or not sent.
    Failed = 3,
}

/// The outcome of the boot-policy digest read, with the digest when read.
pub(crate) fn pnon(state: Pnon, digest: Option<&[u8; crate::pnon::DIGEST_LEN]>) {
    let (ptr, len) = digest.map_or((core::ptr::null(), 0), |d| (d.as_ptr(), d.len()));
    // SAFETY: `ptr` is null or points at `len` readable bytes for the call;
    // the shim copies them.
    unsafe { sep_diag_set_pnon(state as c_int, ptr, len) };
}

/// An endpoint the driver checks at bring-up and sends nothing to, as
/// `SEP_DIAG_OFFER_*` in `shim.h` spells the reported states.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Offer {
    /// Discovered with its name and the layout the driver knows.
    Idle,
    /// Discovered with another name or layout; left alone.
    Unknown,
    /// Not discovered.
    Absent,
}

impl Offer {
    pub(crate) fn of(present: bool, named: bool, layout_known: bool) -> Offer {
        match (present, named && layout_known) {
            (false, _) => Offer::Absent,
            (true, true) => Offer::Idle,
            (true, false) => Offer::Unknown,
        }
    }

    fn state(self) -> c_int {
        match self {
            Offer::Idle => 1,
            // Reported as absent: the driver does not use it.
            Offer::Unknown | Offer::Absent => 2,
        }
    }
}

/// The secure-element sharing endpoint at bring-up (see `sse`).
pub(crate) fn sse(offer: Offer) {
    // SAFETY: no preconditions; updates one word.
    unsafe { sep_diag_set_offer(crate::sse::EP_SSE, offer.state()) };
}

/// The hibernation endpoint at bring-up (see `hibe`).
pub(crate) fn hibe(offer: Offer) {
    // SAFETY: no preconditions; updates one word.
    unsafe { sep_diag_set_offer(crate::hibe::EP_HIBE, offer.state()) };
}

/// The kernel deny-list endpoint at bring-up (see `skdl`).
pub(crate) fn skdl(offer: Offer) {
    // SAFETY: no preconditions; updates one word.
    unsafe { sep_diag_set_offer(crate::skdl::EP_SKDL, offer.state()) };
}

/// The secure-element sharing service announced an event. Returns how many
/// it has announced since probe, this one included.
pub(crate) fn sse_event() -> u32 {
    // SAFETY: no preconditions; updates one word.
    unsafe { sep_diag_sse_event() }
}

/// The driver answered an xART request with the failure status. Returns how
/// many it has answered so since probe, this one included.
pub(crate) fn xarm_failed() -> u32 {
    // SAFETY: no preconditions; updates one word.
    unsafe { sep_diag_xarm_failed() }
}

/// A message arrived on an endpoint the driver has no handler for. Returns
/// how many have, this one included.
pub(crate) fn unhandled() -> u32 {
    // SAFETY: no preconditions; updates one word.
    unsafe { sep_diag_unhandled() }
}

/// Logs the one-line summary of the boot-time bring-up. Call it after
/// [`bringup_ended`].
pub(crate) fn log_summary(dev: &device::Device) {
    // SAFETY: `dev` is the bound SEP device for the duration of the call; the
    // shim only passes it to `dev_info()` and keeps no reference.
    unsafe { sep_diag_log_summary(dev.as_raw().cast()) };
}
