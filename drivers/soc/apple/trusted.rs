// SPDX-License-Identifier: GPL-2.0-only OR MIT
// Copyright 2026 Dj

//! A Linux trusted-key source backed by the SEP reference-key seal.
//!
//! Seal encrypts to the machine ref-key's public point and checks private-key
//! recovery before returning the blob. Unseal asks the enclave to ECIES-decrypt (op 0x22
//! `"oecd"`) with the in-enclave private, which needs the login key bag reloaded
//! and unlocked first. The ref-key lifecycle lives on [`SepData`]; this is the
//! keyring glue. The config-dependent payload ABI is in `trusted_shim.c`.

use core::sync::atomic::{AtomicPtr, Ordering};

use kernel::prelude::*;
use kernel::sync::Arc;
use kernel::types::ForeignOwnable;

use crate::{Secret, SepData};

/// One opaque answer for every unseal rejection: telling a wrong blob, a foreign
/// machine's blob and a tampered blob apart would be an oracle. No name in the
/// Rust error set, hence the literal.
const EBADMSG: c_int = -74;

/// The framework dispatches the ops with no context pointer, so `SepData` is
/// reached through this global. Holds the [`ForeignOwnable::into_foreign`]
/// pointer while registered; `null` otherwise, and the cmpxchg once-guard.
static SEP: AtomicPtr<c_void> = AtomicPtr::new(core::ptr::null_mut());

extern "C" {
    fn sep_tk_register(
        init: unsafe extern "C" fn() -> c_int,
        seal: unsafe extern "C" fn(*mut c_void, *mut c_char) -> c_int,
        unseal: unsafe extern "C" fn(*mut c_void, *mut c_char) -> c_int,
        random: unsafe extern "C" fn(*mut u8, usize) -> c_int,
        exit: unsafe extern "C" fn(),
    ) -> c_int;
    fn sep_tk_unregister();

    fn sep_tk_register_key_type() -> c_int;
    fn sep_tk_unregister_key_type();

    fn sep_tk_max_key_size() -> usize;
    fn sep_tk_max_blob_size() -> usize;

    fn sep_tk_key_ptr(p: *mut c_void) -> *mut u8;
    fn sep_tk_key_len(p: *const c_void) -> c_uint;
    fn sep_tk_set_key_len(p: *mut c_void, n: c_uint);

    fn sep_tk_blob_ptr(p: *mut c_void) -> *mut u8;
    fn sep_tk_blob_len(p: *const c_void) -> c_uint;
    fn sep_tk_set_blob_len(p: *mut c_void, n: c_uint);
}

/// Soundness rests on [`unregister`] resetting the framework's static calls
/// before it reclaims the `Arc`: no op runs after that, and while one runs the
/// pointer is live.
fn with_sep<T>(f: impl FnOnce(&SepData) -> T) -> Option<T> {
    let ptr = SEP.load(Ordering::Acquire);
    if ptr.is_null() {
        return None;
    }
    // SAFETY: produced by `into_foreign` in `register`, reclaimed only by
    // `unregister` after ops stop dispatching, so live for this borrow.
    let borrow = unsafe { <Arc<SepData> as ForeignOwnable>::borrow(ptr) };
    Some(f(&borrow))
}

/// # Safety
/// Called by the key-type core as the `init` op; takes no arguments.
unsafe extern "C" fn tk_init() -> c_int {
    let rc = with_sep(|sep| {
        dev_info!(
            sep.dev,
            "trusted-keys: registering \"trusted\" key type backed by the SEP ref-key seal\n"
        );

        // SAFETY: no preconditions; -EEXIST if already registered.
        let krc = unsafe { sep_tk_register_key_type() };
        if krc != 0 {
            dev_err!(
                sep.dev,
                "trusted-keys: register_key_type failed ({}); registration aborted\n",
                krc
            );
        }
        krc
    });
    rc.unwrap_or_else(|| ENODEV.to_errno())
}

/// `datablob` options (keyhandle/hash/policy) are ignored: no migration or
/// policy binding, so a key is bound only to the machine ref-key.
///
/// # Safety
/// Called by the framework with a live `struct trusted_key_payload *p`.
unsafe extern "C" fn tk_seal(p: *mut c_void, _datablob: *mut c_char) -> c_int {
    let rc = with_sep(|sep| {
        // SAFETY: `p` is a live payload; key_len <= MAX_KEY_SIZE bytes at p->key.
        let key_len = unsafe { sep_tk_key_len(p.cast_const()) } as usize;
        // SAFETY: `p` is a live payload.
        let key_ptr = unsafe { sep_tk_key_ptr(p) };
        // SAFETY: `key_ptr`/`key_len` describe `p->key`; read-only.
        let key = unsafe { core::slice::from_raw_parts(key_ptr.cast_const(), key_len) };

        let blob = match sep.refkey_seal_trusted(key) {
            Ok(blob) => blob,
            Err(e) => {
                // Callers retry until the ref-key is up: warn once, and keep
                // each failure for debugging.
                kernel::do_once_lite! {
                    dev_warn!(
                        sep.dev,
                        "trusted-keys: seal failed ({:?}); the machine ref-key may not be established yet\n",
                        e
                    );
                }
                dev_dbg!(sep.dev, "trusted-keys: seal failed ({:?})\n", e);
                return e.to_errno();
            }
        };

        // SAFETY: no preconditions.
        let max_blob = unsafe { sep_tk_max_blob_size() };
        if blob.len() > max_blob {
            dev_warn!(
                sep.dev,
                "trusted-keys: sealed blob {} byte(s) exceeds MAX_BLOB_SIZE {} byte(s); refusing\n",
                blob.len(),
                max_blob
            );
            return E2BIG.to_errno();
        }

        // SAFETY: `p` is a live payload.
        let dst = unsafe { sep_tk_blob_ptr(p) };
        // SAFETY: `dst` is `p->blob` (MAX_BLOB_SIZE bytes), blob.len() <= max_blob,
        // no overlap.
        unsafe { core::ptr::copy_nonoverlapping(blob.as_ptr(), dst, blob.len()) };
        // SAFETY: `p` is live.
        unsafe { sep_tk_set_blob_len(p, blob.len() as c_uint) };

        // Lengths only, never the key or blob bytes.
        dev_info!(
            sep.dev,
            "trusted-keys: sealed {}-byte key into {}-byte enclave blob\n",
            key_len,
            blob.len()
        );
        0
    });
    rc.unwrap_or_else(|| ENODEV.to_errno())
}

/// # Safety
/// Called by the framework with a live `struct trusted_key_payload *p`.
unsafe extern "C" fn tk_unseal(p: *mut c_void, _datablob: *mut c_char) -> c_int {
    let rc = with_sep(|sep| {
        // SAFETY: `p` is a live payload; blob_len <= MAX_BLOB_SIZE bytes at p->blob.
        let blob_len = unsafe { sep_tk_blob_len(p.cast_const()) } as usize;
        // SAFETY: `p` is a live payload.
        let blob_ptr = unsafe { sep_tk_blob_ptr(p) };
        // SAFETY: `blob_ptr`/`blob_len` describe `p->blob`; read-only.
        let blob = unsafe { core::slice::from_raw_parts(blob_ptr.cast_const(), blob_len) };

        // Every failure past here is one opaque EBADMSG.
        let Ok(plain) = sep.refkey_unseal_trusted(blob) else {
            return EBADMSG;
        };
        let plain = Secret(plain);

        // SAFETY: no preconditions.
        let max_key = unsafe { sep_tk_max_key_size() };
        if plain.is_empty() || plain.len() > max_key {
            return EBADMSG;
        }

        // SAFETY: `p` is a live payload.
        let dst = unsafe { sep_tk_key_ptr(p) };
        // SAFETY: `dst` is `p->key` (MAX_KEY_SIZE + 1 bytes), plain.len() <= max_key,
        // no overlap.
        unsafe { core::ptr::copy_nonoverlapping(plain.as_ptr(), dst, plain.len()) };
        // SAFETY: `p` is live.
        unsafe { sep_tk_set_key_len(p, plain.len() as c_uint) };

        dev_info!(
            sep.dev,
            "trusted-keys: unsealed {}-byte enclave blob to {}-byte key\n",
            blob_len,
            plain.len()
        );
        0
    });
    rc.unwrap_or_else(|| ENODEV.to_errno())
}

/// Returns the byte count on success (not 0) per the framework contract, so the
/// caller's `ret != key_len` check passes.
///
/// # Safety
/// Called by the framework with `key` writable for `key_len` bytes.
unsafe extern "C" fn tk_get_random(key: *mut u8, key_len: usize) -> c_int {
    let rc = with_sep(|sep| {
        if key_len == 0 {
            return 0;
        }
        if let Err(e) = sep.wait_for_trusted_ready() {
            return e.to_errno();
        }
        // SAFETY: the framework guarantees `key` is writable for `key_len` bytes.
        let buf = unsafe { core::slice::from_raw_parts_mut(key, key_len) };
        match sep.sep_random(buf) {
            Ok(()) => key_len as c_int,
            Err(e) => e.to_errno(),
        }
    });
    rc.unwrap_or_else(|| ENODEV.to_errno())
}

/// # Safety
/// Called by the framework at most once per successful `init()`.
unsafe extern "C" fn tk_exit() {
    // SAFETY: no preconditions; the key type was registered by `tk_init`.
    unsafe { sep_tk_unregister_key_type() };
}

/// Idempotent: the cmpxchg claims the global slot once. On any failure the `Arc`
/// is reclaimed and the slot left clear, so a later call can retry.
pub(crate) fn register(sep: Arc<SepData>) -> Result<()> {
    let ptr = sep.into_foreign();

    if SEP
        .compare_exchange(
            core::ptr::null_mut(),
            ptr,
            Ordering::AcqRel,
            Ordering::Acquire,
        )
        .is_err()
    {
        // SAFETY: `ptr` was just produced by `into_foreign` and never published.
        drop(unsafe { <Arc<SepData> as ForeignOwnable>::from_foreign(ptr) });
        return Err(EBUSY);
    }

    // SAFETY: the shim stores these callbacks and calls
    // `register_trusted_key_source`; valid for the module's lifetime.
    let rc = unsafe { sep_tk_register(tk_init, tk_seal, tk_unseal, tk_get_random, tk_exit) };
    if rc != 0 {
        // Nothing was wired, so no op can dispatch; clear the slot and reclaim.
        let raw = SEP.swap(core::ptr::null_mut(), Ordering::AcqRel);
        if !raw.is_null() {
            // SAFETY: `raw` is the pointer we published; the ops are not wired.
            drop(unsafe { <Arc<SepData> as ForeignOwnable>::from_foreign(raw) });
        }
        return Err(Error::from_errno(rc));
    }
    Ok(())
}

/// Safe to call even if [`register`] never ran. Resets the framework's static
/// calls and runs `exit()` before reclaiming the `Arc`, so no op is in flight
/// against freed data once this returns.
pub(crate) fn unregister() {
    if SEP.load(Ordering::Acquire).is_null() {
        return;
    }
    // SAFETY: no preconditions; matched with the `sep_tk_register` above.
    unsafe { sep_tk_unregister() };

    let ptr = SEP.swap(core::ptr::null_mut(), Ordering::AcqRel);
    if !ptr.is_null() {
        // SAFETY: `ptr` came from `into_foreign` in `register`, and no op runs now.
        drop(unsafe { <Arc<SepData> as ForeignOwnable>::from_foreign(ptr) });
    }
}
