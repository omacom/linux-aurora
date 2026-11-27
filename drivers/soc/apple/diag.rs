// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Rust side of `diag_shim.c`: the passive diagnostics in sysfs.
//!
//! The driver reports each decision here as it makes it, and the shim caches
//! it for the `diag` attributes. Every call only stores a word, so none of them
//! can block a bring-up step, and a sysfs read never reaches back into the
//! driver or the SEP.

use kernel::{device, prelude::*};

use crate::{
    profile,
    registration,
    xart_guard::{KeybagEvidence, MatchFacts}, //
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
    fn sep_diag_set_attach(attached: bool);
    fn sep_diag_set_endpoints(count: c_uint);
    fn sep_diag_set_keystore_open();
    fn sep_diag_set_keybag(state: c_int);
    fn sep_diag_sensor_bind() -> c_uint;
    fn sep_diag_set_sensor_result(bind: c_uint, online: bool);
    fn sep_diag_set_bio_published();
    fn sep_diag_set_touchid(started: bool);
    fn sep_diag_set_sensor_attach(state: c_int);
    fn sep_diag_set_restore(state: c_int);
    fn sep_diag_set_xart_guard(level: c_uint, list_loaded: bool, pins: bool, matching: c_int);
    fn sep_diag_set_matching(matching: c_int);
    fn sep_diag_xart_guard_refused();
    fn sep_diag_verified_match();
    fn sep_diag_bringup_ended();
}

/// The identity keybag, as `SEP_DIAG_KEYBAG_*` in `shim.h` spells it.
#[derive(Clone, Copy)]
pub(crate) enum Keybag {
    Present = 1,
    Missing = 2,
    Failed = 3,
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

/// Adds the `diag` group to the SEP device, with the facts fixed at probe. The
/// group goes away when the driver unbinds.
///
/// `guard` is the xART guard as the store opened it: its effective level and
/// whether Linux's list loaded and whether its pin policy is effective, or
/// `None` when no guard runs. `keybag` describes the guard's evidence for
/// the validated, persisted identity bag.
pub(crate) fn register(
    dev: &device::Device<device::Bound>,
    profile: &profile::PlatformProfile,
    xart_writable: bool,
    guard: Option<(u8, bool, bool)>,
    keybag: KeybagEvidence,
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
    let (level, list_loaded, pins) = guard.unwrap_or((0, false, false));
    let matching = MatchFacts {
        xart_writable,
        cold,
        keybag,
        guard: guard.map(|(level, list_loaded, _)| (level, list_loaded)),
    }
    .matching();
    // SAFETY: no preconditions; caches the probe's fixed facts.
    unsafe { sep_diag_set_xart_guard(c_uint::from(level), list_loaded, pins, matching as c_int) };

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

/// The caller verified the guarded identity after the keybag commit.
pub(crate) fn xart_guard_keybag_committed() {
    // SAFETY: stores the base status. A refusal takes precedence at readout.
    unsafe { sep_diag_set_matching(crate::xart_guard::Matching::Unverified as c_int) };
}

/// The xART guard refused one of the enclave's writes or deletes.
pub(crate) fn xart_guard_refused() {
    // SAFETY: no preconditions; increments one counter.
    unsafe { sep_diag_xart_guard_refused() };
}

/// The current userspace verification completed with a durably saved match.
pub(crate) fn verified_match() {
    // SAFETY: promotes the base status only for a legacy pin list.
    unsafe { sep_diag_verified_match() };
}

/// The boot-time bring-up is over, however far it got.
pub(crate) fn bringup_ended() {
    // SAFETY: no preconditions; updates two words.
    unsafe { sep_diag_bringup_ended() };
}
