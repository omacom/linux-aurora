// SPDX-License-Identifier: GPL-2.0-only OR MIT
// Copyright 2026 Ryan Murray
//! `/dev/sep-key`: P-256 keys created inside the SEP.
//!
//! Each key is an enclave ref-key bound to this SEP and to the Linux identity
//! key bag. Its private half never leaves the enclave; userspace holds only a
//! key handle, the wrapped ref-key record behind a small versioned header.
//! A handle works on the Mac that created it and nowhere else. Signing takes a
//! 32-byte digest and returns a DER `ECDSA-Sig-Value`.
//!
//! A key made with `CREATE_BIOMETRY_CURRENT_SET` carries an access-control
//! list in its record, the one macOS gives a Secure Enclave key made with
//! `.biometryCurrentSet`: signing (and key agreement) needs a credential
//! context in which the enclave itself saw a match of one of the fingers
//! enrolled now. The kernel opens that context inside the `SIGN` call, runs a
//! Touch ID match in it, signs, and deletes it before the call returns. No
//! handle to it ever reaches userspace, and every signature needs its own
//! touch. The enclave refuses the signature for any other context; the
//! header's flag only tells the kernel to ask for the touch.
//!
//! `CREATE_BIOMETRY_ANY` is macOS's `.biometryAny`: the same list without
//! the enrolled-set hash, so any finger enrolled in this catacomb matches,
//! including fingers enrolled after the key was made.
//!
//! Interface version 3 adds three calls. `CANCEL` ends a touch-gated call
//! that is waiting for a finger (it returns `ECANCELED`). `SEAL` encrypts up
//! to 64 bytes to a biometric key's public point (ECIES, done here: it needs
//! no private key and no touch). `UNSEAL` has the enclave do the key exchange
//! (`ock`) of such a blob's ephemeral point with the key, under the same
//! fresh, touch-satisfied credential context as a `SIGN`, and finishes the
//! AES-GCM decrypt in the kernel, as macOS does for Secure Enclave keys: the
//! key's access-control list constrains the key exchange as it does the
//! signature, so a sealed secret is released only after a match the enclave
//! saw. Userspace uses it for secrets that must not exist outside the
//! enclave without the user (FIDO hmac-secret).
//!
//! Interface version 4 adds keys under a user's own key bag (linux#921;
//! variant-5 key store, `INFO.features` `FEATURE_PASSWORD`). `CREATE_USER`
//! makes a `.userPresence` key under `uid`'s bag, or with flags 0 a key with
//! no list that works only while the bag is unlocked; the enclave refuses
//! either while the bag is locked. `SIGN` / `UNSEAL` on a user-presence key
//! wait for a Touch ID match as for a biometric key; `SIGN_PASSWORD` /
//! `UNSEAL_PASSWORD` hand the bag's password to the enclave instead
//! (`VERIFY_SECRET` into a fresh context), which checks it and counts a
//! wrong one before the one operation runs. Root or the bag's own user may
//! make these calls. `CREATE`'s own `CREATE_USER_PRESENCE` bit stays
//! reserved: under the identity bag the only passcode is the identity bag's
//! secret, which root holds.
//!
//! macOS's `.userPresence` (Touch ID or the password) and `.devicePasscode`
//! need a passcode credential the enclave checks against the user's own
//! secret. On Linux the only passcode the enclave knows today is the identity
//! key bag's secret, which root holds, so those policies would be
//! root-satisfiable. Their `CREATE` bits are reserved and refused with
//! `EOPNOTSUPP` until a per-user, password-keyed bag exists; they are never
//! emulated in software.

use crate::der::RefKeyValue as RV;
use crate::SepData;
use kernel::ioctl::{_IO, _IOR, _IOWR};
use kernel::prelude::*;
use kernel::sync::atomic::Relaxed;
use kernel::uaccess::{UserPtr, UserSlice};

pub(crate) const DEVICE_NAME: &CStr = c"sep-key";
/// Root only by default; distributions grant a group with a udev rule.
pub(crate) const DEVICE_MODE: u16 = 0o600;

pub(crate) const IFACE_VERSION: u32 = 4;

/// `INFO.features` (version 3): `CANCEL` is available.
pub(crate) const FEATURE_CANCEL: u32 = 1;
/// `INFO.features`: `SEAL` and `UNSEAL` are available.
pub(crate) const FEATURE_SEAL: u32 = 2;
const FEATURES: u32 = FEATURE_CANCEL | FEATURE_SEAL;
/// `INFO.features` (version 4): `CREATE_USER`, `SIGN_PASSWORD` and
/// `UNSEAL_PASSWORD` are available (per-user key bags: the variant-5 key
/// store only).
pub(crate) const FEATURE_PASSWORD: u32 = 4;
/// `INFO.features`: a user bag's failed-password count survives a restart
/// in the enclave. Never set yet: the enclave counts wrong passwords only
/// while the bag is loaded (the wrapped bag carries no count and the
/// enclave keeps no anti-replay record for it), so a FIDO PIN retry budget
/// built on it would not be root-proof. aurora-fido offers no PIN without it.
pub(crate) const FEATURE_PERSISTENT_ATTEMPTS: u32 = 8;
/// The longest password `SIGN_PASSWORD` / `UNSEAL_PASSWORD` take, as the
/// key-bag ioctls.
pub(crate) const PASSWORD_MAX: usize = crate::userbag::SECRET_MAX;

/// The largest secret `SEAL` takes and `UNSEAL` returns.
pub(crate) const SECRET_MAX: usize = 64;
/// The largest sealed blob: ephemeral point, ciphertext and tag fit easily.
pub(crate) const SEALED_MAX: usize = 256;

pub(crate) const HANDLE_MAX: usize = 2048;
pub(crate) const PUB_LEN: usize = 65;
pub(crate) const DIGEST_LEN: usize = 32;
pub(crate) const SIG_MAX: usize = 72;

/// `CREATE` flag: signing needs a Touch ID match of a currently enrolled
/// finger, enforced by the enclave.
pub(crate) const CREATE_BIOMETRY_CURRENT_SET: u32 = 1;
/// `CREATE` flag: signing needs a Touch ID match of any finger enrolled in
/// this catacomb, now or later, enforced by the enclave.
pub(crate) const CREATE_BIOMETRY_ANY: u32 = 2;
/// Reserved `CREATE` flags for macOS's `.userPresence` and `.devicePasscode`.
/// Refused with `EOPNOTSUPP` until the enclave can check the user's own
/// password (see the module comment).
pub(crate) const CREATE_USER_PRESENCE: u32 = 4;
pub(crate) const CREATE_DEVICE_PASSCODE: u32 = 8;
const CREATE_FLAGS: u32 = CREATE_BIOMETRY_CURRENT_SET | CREATE_BIOMETRY_ANY;
/// `CREATE_USER` flags (version 4): `.userPresence` under the user's bag;
/// 0 makes a key with no list there, usable exactly while the bag is unlocked.
const CREATE_USER_FLAGS: u32 = CREATE_USER_PRESENCE;
const CREATE_USER_PLAIN: u32 = 0;
const CREATE_RESERVED: u32 = CREATE_USER_PRESENCE | CREATE_DEVICE_PASSCODE;

/// The access policy a `CREATE` request asks for: at most one policy bit.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(crate) enum Policy {
    None,
    BiometryCurrentSet,
    BiometryAny,
}

pub(crate) fn create_policy(flags: u32) -> Result<Policy> {
    if flags & !(CREATE_FLAGS | CREATE_RESERVED) != 0 || flags.count_ones() > 1 {
        return Err(EINVAL);
    }
    match flags {
        0 => Ok(Policy::None),
        CREATE_BIOMETRY_CURRENT_SET => Ok(Policy::BiometryCurrentSet),
        CREATE_BIOMETRY_ANY => Ok(Policy::BiometryAny),
        _ => Err(EOPNOTSUPP),
    }
}

/// A touch-gated `SIGN` asks for a finger again after a capture that saw
/// none until this long has passed; one capture itself waits up to a minute.
const TOUCH_WAIT_MS: u64 = 60_000;

const MAGIC: u32 = 0xB1;

/// Handle header: `"SEPK"`, version, flags, reserved u16, blob length (LE u32).
const HANDLE_MAGIC: &[u8; 4] = b"SEPK";
const HANDLE_VERSION: u8 = 1;
/// Handle flag: the record carries a biometric access-control list.
pub(crate) const HANDLE_BIOMETRY: u8 = 0x01;
/// Handle flag, with `HANDLE_BIOMETRY` only: the list names no enrolled-set
/// hash (biometry-any).
pub(crate) const HANDLE_BIOMETRY_ANY: u8 = 0x02;
pub(crate) const HANDLE_HEADER: usize = 12;
/// Handle flag (handle version 2): the record lives under a user's key bag
/// with macOS's `.userPresence` list; the uid follows the length.
pub(crate) const HANDLE_USER_BAG: u8 = 0x04;
/// Handle flag, with `HANDLE_USER_BAG` only: the record has no list. It
/// works with no touch and no password, but only while the bag is unlocked
/// (protection class 9: the enclave refuses while it is locked).
pub(crate) const HANDLE_USER_PLAIN: u8 = 0x08;
const HANDLE_VERSION_USER: u8 = 2;
const HANDLE_HEADER_USER: usize = 16;
/// The largest enclave record a handle can carry.
pub(crate) const BLOB_MAX: usize = HANDLE_MAX - HANDLE_HEADER;

#[repr(C)]
pub(crate) struct Info {
    pub(crate) version: u32,
    /// `flags` bits `CREATE` accepts.
    pub(crate) flags_supported: u32,
    pub(crate) handle_max: u32,
    /// `FEATURE_*` (version 3; reserved and zero before).
    pub(crate) features: u32,
}

#[repr(C)]
pub(crate) struct Create {
    pub(crate) flags: u32,
    pub(crate) handle_len: u32,
    pub(crate) public_key: [u8; PUB_LEN],
    pub(crate) reserved: [u8; 3],
    pub(crate) handle: [u8; HANDLE_MAX],
}

#[repr(C)]
pub(crate) struct Sign {
    pub(crate) flags: u32,
    pub(crate) handle_len: u32,
    pub(crate) digest: [u8; DIGEST_LEN],
    pub(crate) sig_len: u32,
    pub(crate) signature: [u8; SIG_MAX],
    pub(crate) handle: [u8; HANDLE_MAX],
}

#[repr(C)]
pub(crate) struct Public {
    pub(crate) flags: u32,
    pub(crate) handle_len: u32,
    pub(crate) public_key: [u8; PUB_LEN],
    pub(crate) reserved: [u8; 3],
    pub(crate) handle: [u8; HANDLE_MAX],
}

/// `SEAL` (secret in, sealed out) and `UNSEAL` (sealed in, secret out).
#[repr(C)]
pub(crate) struct Seal {
    /// In: 0.
    pub(crate) flags: u32,
    pub(crate) handle_len: u32,
    pub(crate) secret_len: u32,
    pub(crate) sealed_len: u32,
    pub(crate) secret: [u8; SECRET_MAX],
    pub(crate) sealed: [u8; SEALED_MAX],
    pub(crate) handle: [u8; HANDLE_MAX],
}

/// `CREATE_USER` (version 4): a key under `uid`'s bag.
#[repr(C)]
pub(crate) struct CreateUser {
    pub(crate) flags: u32,
    pub(crate) uid: u32,
    pub(crate) handle_len: u32,
    pub(crate) public_key: [u8; PUB_LEN],
    pub(crate) reserved: [u8; 3],
    pub(crate) handle: [u8; HANDLE_MAX],
}

/// The password a `SIGN_PASSWORD` / `UNSEAL_PASSWORD` carries. The kernel
/// keeps no copy; the request's own copy is wiped when it drops.
#[repr(C)]
pub(crate) struct Password {
    pub(crate) len: u32,
    pub(crate) reserved: u32,
    pub(crate) bytes: [u8; PASSWORD_MAX],
}

impl Drop for Password {
    fn drop(&mut self) {
        crate::image::wipe(&mut self.bytes);
    }
}

#[repr(C)]
pub(crate) struct SignPassword {
    pub(crate) sign: Sign,
    pub(crate) password: Password,
}

#[repr(C)]
pub(crate) struct UnsealPassword {
    pub(crate) seal: Seal,
    pub(crate) password: Password,
}

/// Test only (module parameter `key_selftest`, root): sign with a
/// biometric key under a deliberately unsuitable context.
#[repr(C)]
pub(crate) struct Selftest {
    pub(crate) mode: u32,
    pub(crate) handle_len: u32,
    pub(crate) digest: [u8; DIGEST_LEN],
    /// 0 when the enclave signed, else the negative errno.
    pub(crate) result: i32,
    /// The enclave's status for that signature.
    pub(crate) enclave_status: i32,
    /// `SELFTEST_TOUCH_THEN_DELETED`: the second signature.
    pub(crate) result2: i32,
    pub(crate) enclave_status2: i32,
    pub(crate) handle: [u8; HANDLE_MAX],
}

/// No credential context at all.
pub(crate) const SELFTEST_NO_CONTEXT: u32 = 1;
/// A fresh context in which nothing has matched.
pub(crate) const SELFTEST_UNSATISFIED: u32 = 2;
/// A context holding only the key bag's secret (Linux's "passcode").
pub(crate) const SELFTEST_PASSCODE: u32 = 3;
/// A fresh context, deleted before the signature.
pub(crate) const SELFTEST_DELETED: u32 = 4;
/// One touch: sign in the matched context, delete it, sign again with it
/// (needs a finger).
pub(crate) const SELFTEST_TOUCH_THEN_DELETED: u32 = 5;
/// Research probe, no finger, ignores the handle: make a throwaway key with
/// macOS's `.userPresence` list (`cpo` "DeviceOwnerAuthentication") and sign
/// with a context holding only the key bag's secret (`result`) and with an
/// unsatisfied context (`result2`). Shows whether the enclave enforces that
/// policy and whether a root-held secret satisfies it; `CREATE` refuses the
/// policy either way.
pub(crate) const SELFTEST_PRESENCE_PROBE: u32 = 6;
/// Research probe, no finger: the `.userPresence` throwaway key (identity
/// bag) signed with a context whose passcode credential came from another
/// user's bag. In: `enclave_status` = that user's uid, the handle buffer =
/// that user's bag password (`handle_len` bytes). Out: `result2` = the
/// verify-secret status, `result`/`enclave_status` = the signature. Shows
/// whether a credential from one bag satisfies a key under another.
pub(crate) const SELFTEST_PRESENCE_OTHER_BAG: u32 = 7;
/// Research probe, no finger: a throwaway `.userPresence` key made under
/// the (unlocked) bag of the user in `enclave_status` (outer handle -uid).
/// `result`: signed with a context holding a passcode credential from that
/// same bag (the password in the handle buffer). `result2`: signed with a
/// fresh, unsatisfied context. Statuses in `enclave_status`/`2`.
pub(crate) const SELFTEST_USERBAG_KEY_SAME_BAG: u32 = 8;
/// As 8, but `result` signs with the Linux identity bag's secret as the
/// credential (what root holds) and `result2` with the handle buffer's
/// password used as given (pass a wrong one).
pub(crate) const SELFTEST_USERBAG_KEY_ROOT: u32 = 9;
/// Research probe, no finger (T2: a locked bag refuses without its
/// password): a throwaway `.userPresence` key under the unlocked bag of the
/// uid in `enclave_status`, password in the handle buffer. The kernel signs
/// (r0) with a context holding that bag's password credential, locks the bag
/// for good (screen lock, grace expired now), then signs with the same, now
/// stale, context (r1), with no credential (r2) and with a fresh password
/// credential (r3), unlocks the bag with the password and signs with a fresh
/// credential again (r4). The digest buffer returns, as LE i32: r0..r4 (0 =
/// signed, else the enclave status), the lock phase before the lock, after
/// it and after the unlock.
pub(crate) const SELFTEST_USERBAG_LOCKED: u32 = 10;
/// Research: create a throwaway `.userPresence` key under the unlocked bag
/// of the uid in `enclave_status` and return its record in the handle
/// buffer (`handle_len`), so a later boot can use it after the bag is
/// imported again.
pub(crate) const SELFTEST_USERBAG_KEY_MAKE: u32 = 11;
/// Research: sign the record in the handle buffer under the bag of the uid
/// in `enclave_status`, with a password credential from that bag; the
/// password is the NUL-padded `digest` (at most 32 bytes). `result` = the
/// signature, `result2` = the verify-secret status.
pub(crate) const SELFTEST_USERBAG_KEY_USE: u32 = 12;
/// One touch: sign in the matched context, then sign again in the same,
/// still open context (`result2`). Measures whether one match lets the
/// enclave do more than one operation (needs a finger).
pub(crate) const SELFTEST_TOUCH_TWO_SIGNATURES: u32 = 20;
/// One touch: sign, then unseal a secret sealed to the key in the same
/// context (`result2` is 0 when the secret came back unchanged; needs a finger).
pub(crate) const SELFTEST_TOUCH_SIGN_THEN_UNSEAL: u32 = 21;
/// No finger: unseal a secret sealed to the key with no context (`result`)
/// and with an unsatisfied one (`result2`). Both must be refused.
pub(crate) const SELFTEST_UNSEAL_UNSATISFIED: u32 = 22;
/// Research probe, no finger, ignores the handle: what the enclave does
/// for a decrypt (`oecd`) and a key exchange (`ock`) on keys with and without
/// a list. Five throwaway keys (no list; the list with `oecd` free; with no
/// decrypt entry; with `ock` free; macOS's plain list, `ock`/`odel`/`osgn`/
/// `oa`/`okd` free), each tried with `UNSEAL_CONTEXT_PROBE_ROWS`. Bit
/// `8 * key + row` of `result` (keys 0-3) or bit `row` of `result2` (key 4)
/// is set when that row worked (a key exchange only when it returned the
/// expected shared secret, an unseal only when the secret came back). The
/// kernel log has every enclave status.
pub(crate) const SELFTEST_UNSEAL_CONTEXT_PROBE: u32 = 24;

/// Whether this kernel has self-test `mode` at all. The modes that run a
/// second operation in a matched, still open context (20, 21) exist only
/// in research kernels built with `CONFIG_APPLE_SEP_KEY_RESEARCH`.
pub(crate) fn selftest_mode_built(mode: u32) -> bool {
    match mode {
        SELFTEST_TOUCH_TWO_SIGNATURES | SELFTEST_TOUCH_SIGN_THEN_UNSEAL => {
            cfg!(CONFIG_APPLE_SEP_KEY_RESEARCH)
        }
        _ => true,
    }
}

/// Rows of `SELFTEST_UNSEAL_CONTEXT_PROBE`: (name, operation, context).
/// Context 0 none, 1 a fresh unsatisfied one, 2 the key bag's secret.
/// Operation `unseal` is SEP_KEY_UNSEAL's own path.
pub(crate) const UNSEAL_CONTEXT_PROBE_ROWS: [(&str, &[u8], u8); 8] = [
    ("sign, no context", b"osgn".as_slice(), 0),
    ("sign, unsatisfied context", b"osgn", 1),
    ("decrypt, no context", b"oecd", 0),
    ("decrypt, unsatisfied context", b"oecd", 1),
    ("key exchange, no context", b"ock", 0),
    ("key exchange, unsatisfied context", b"ock", 1),
    ("key exchange, key-bag-secret context", b"ock", 2),
    ("unseal, no context", b"unseal", 0),
];

static_assert!(core::mem::size_of::<Info>() == 16);
static_assert!(core::mem::size_of::<Create>() == 2124);
static_assert!(core::mem::size_of::<Sign>() == 2164);
static_assert!(core::mem::size_of::<Public>() == 2124);
static_assert!(core::mem::size_of::<Selftest>() == 2104);
static_assert!(core::mem::size_of::<Seal>() == 2384);
static_assert!(core::mem::size_of::<CreateUser>() == 2128);
static_assert!(core::mem::size_of::<SignPassword>() == 2164 + 8 + PASSWORD_MAX);
static_assert!(core::mem::size_of::<UnsealPassword>() == 2384 + 8 + PASSWORD_MAX);

// SAFETY: `repr(C)` with no padding (the asserts above), so each value is a
// faithful byte image and every bit pattern is a valid value.
unsafe impl kernel::transmute::AsBytes for Info {}
// SAFETY: as above.
unsafe impl kernel::transmute::AsBytes for Create {}
// SAFETY: as above.
unsafe impl kernel::transmute::FromBytes for Create {}
// SAFETY: as above.
unsafe impl kernel::transmute::AsBytes for Sign {}
// SAFETY: as above.
unsafe impl kernel::transmute::FromBytes for Sign {}
// SAFETY: as above.
unsafe impl kernel::transmute::AsBytes for Public {}
// SAFETY: as above.
unsafe impl kernel::transmute::FromBytes for Public {}
// SAFETY: as above.
unsafe impl kernel::transmute::AsBytes for Seal {}
// SAFETY: as above.
unsafe impl kernel::transmute::FromBytes for Seal {}
// SAFETY: as above.
unsafe impl kernel::transmute::AsBytes for Selftest {}
// SAFETY: as above.
unsafe impl kernel::transmute::FromBytes for Selftest {}
// SAFETY: as above (`repr(C)`, no padding: the size asserts).
unsafe impl kernel::transmute::AsBytes for CreateUser {}
// SAFETY: as above.
unsafe impl kernel::transmute::FromBytes for CreateUser {}
// SAFETY: as above (`repr(C)`, no padding: the size asserts).
unsafe impl kernel::transmute::AsBytes for SignPassword {}
// SAFETY: as above.
unsafe impl kernel::transmute::FromBytes for SignPassword {}
// SAFETY: as above (`repr(C)`, no padding: the size asserts).
unsafe impl kernel::transmute::AsBytes for UnsealPassword {}
// SAFETY: as above.
unsafe impl kernel::transmute::FromBytes for UnsealPassword {}

pub(crate) const IOC_INFO: u32 = _IOR::<Info>(MAGIC, 0x20);
pub(crate) const IOC_CREATE: u32 = _IOWR::<Create>(MAGIC, 0x21);
pub(crate) const IOC_SIGN: u32 = _IOWR::<Sign>(MAGIC, 0x22);
pub(crate) const IOC_PUBLIC: u32 = _IOWR::<Public>(MAGIC, 0x23);
pub(crate) const IOC_SELFTEST: u32 = _IOWR::<Selftest>(MAGIC, 0x24);
pub(crate) const IOC_CANCEL: u32 = _IO(MAGIC, 0x25);
pub(crate) const IOC_SEAL: u32 = _IOWR::<Seal>(MAGIC, 0x26);
pub(crate) const IOC_UNSEAL: u32 = _IOWR::<Seal>(MAGIC, 0x27);
pub(crate) const IOC_CREATE_USER: u32 = _IOWR::<CreateUser>(MAGIC, 0x28);
pub(crate) const IOC_SIGN_PASSWORD: u32 = _IOWR::<SignPassword>(MAGIC, 0x29);
pub(crate) const IOC_UNSEAL_PASSWORD: u32 = _IOWR::<UnsealPassword>(MAGIC, 0x2a);

/// A sealed secret's plaintext never outlives the request that carries it:
/// `SEAL` brings it in, `UNSEAL` takes it out.
impl Seal {
    fn wipe_secret(&mut self) {
        crate::image::wipe(&mut self.secret);
    }
}

impl Drop for Seal {
    fn drop(&mut self) {
        self.wipe_secret();
    }
}

/// The research self-tests carry passwords in the handle and digest buffers.
impl Drop for Selftest {
    fn drop(&mut self) {
        crate::image::wipe(&mut self.handle);
        crate::image::wipe(&mut self.digest);
    }
}

/// Copies a `T` from userspace straight into a new kernel allocation, so no
/// copy of a request (some carry secrets) is left on the kernel stack. A
/// copy that faults part way is wiped before the error returns.
pub(crate) fn read_user_boxed<T: kernel::transmute::FromBytes>(user: UserPtr) -> Result<KBox<T>> {
    let size = core::mem::size_of::<T>();
    let mut b = KBox::<T>::new_uninit(GFP_KERNEL)?;
    // SAFETY: the allocation is `size` bytes, uniquely owned here, and any
    // byte value may be written to `MaybeUninit` storage.
    let bytes = unsafe {
        core::slice::from_raw_parts_mut((*b).as_mut_ptr().cast::<core::mem::MaybeUninit<u8>>(), size)
    };
    if let Err(e) = UserSlice::new(user, size).reader().read_raw(bytes) {
        for byte in bytes.iter_mut() {
            // SAFETY: a valid, uniquely borrowed byte of the allocation.
            unsafe { core::ptr::write_volatile(byte.as_mut_ptr(), 0) };
        }
        return Err(e);
    }
    // SAFETY: every byte was written above, and `T: FromBytes` makes any
    // bit pattern a valid `T`.
    Ok(unsafe { b.assume_init() })
}

fn write_user<T: kernel::transmute::AsBytes>(user: UserPtr, value: &T) -> Result {
    UserSlice::new(user, core::mem::size_of::<T>()).writer().write(value)
}

/// A `/dev/sep-key` request, copied in from userspace and checked. Nothing
/// in it has reached the enclave, and anything malformed was refused before
/// a touch is asked for.
pub(crate) enum KeyRequest {
    Info,
    Create(KBox<Create>, Policy),
    Sign(KBox<Sign>),
    Public(KBox<Public>),
    Cancel,
    Seal(KBox<Seal>),
    Unseal(KBox<Seal>),
    Selftest(KBox<Selftest>),
    CreateUser(KBox<CreateUser>),
    SignPassword(KBox<SignPassword>),
    UnsealPassword(KBox<UnsealPassword>),
}

/// The enclave record in a request's handle, refused unless it is a ref-key
/// record (it carries a public point).
fn checked_record(handle: &[u8; HANDLE_MAX], len: u32) -> Result<Handle<'_>> {
    let h = decode_any_handle(handle, len)?;
    SepData::user_key_public(h.blob)?;
    Ok(h)
}

pub(crate) fn check_create(req: &Create) -> Result<Policy> {
    if req.reserved != [0; 3] {
        return Err(EINVAL);
    }
    create_policy(req.flags)
}

pub(crate) fn check_sign(req: &Sign) -> Result<Handle<'_>> {
    if req.flags != 0 {
        return Err(EINVAL);
    }
    checked_record(&req.handle, req.handle_len)
}

pub(crate) fn check_public(req: &Public) -> Result<&[u8]> {
    if req.flags != 0 || req.reserved != [0; 3] {
        return Err(EINVAL);
    }
    Ok(checked_record(&req.handle, req.handle_len)?.blob)
}

/// `SEAL` and `UNSEAL` take only keys whose use the enclave gates on a
/// touch, the password or the user's unlocked bag.
fn check_seal_common(req: &Seal) -> Result<Handle<'_>> {
    if req.flags != 0 {
        return Err(EINVAL);
    }
    let h = checked_record(&req.handle, req.handle_len)?;
    if h.flags & (HANDLE_BIOMETRY | HANDLE_USER_BAG) == 0 {
        return Err(EINVAL);
    }
    Ok(h)
}

pub(crate) fn check_seal(req: &Seal) -> Result<(&[u8], u8, &[u8])> {
    let h = check_seal_common(req)?;
    let n = req.secret_len as usize;
    if n == 0 || n > SECRET_MAX {
        return Err(EINVAL);
    }
    Ok((h.blob, h.flags, &req.secret[..n]))
}

/// The sealed blob must have the shape `SEAL` makes before the caller is
/// asked to touch the sensor for it.
pub(crate) fn check_unseal(req: &Seal) -> Result<(Handle<'_>, &[u8])> {
    let h = check_seal_common(req)?;
    let n = req.sealed_len as usize;
    if n == 0 || n > SEALED_MAX {
        return Err(EINVAL);
    }
    let sealed = &req.sealed[..n];
    crate::refkey_seal::sealed_ephemeral(sealed)?;
    Ok((h, sealed))
}

pub(crate) fn check_create_user(req: &CreateUser) -> Result<bool> {
    if req.reserved != [0; 3] || req.flags & !CREATE_USER_FLAGS != 0 {
        return Err(EINVAL);
    }
    bag_user(req.uid)?;
    Ok(req.flags == CREATE_USER_PLAIN)
}

/// A password call takes only a user-presence key under a user's bag.
fn check_password_handle(h: &Handle<'_>) -> Result<u32> {
    let uid = h.uid.ok_or(EINVAL)?;
    if h.flags & HANDLE_USER_PLAIN != 0 {
        return Err(EINVAL);
    }
    Ok(uid)
}

pub(crate) fn check_password(pw: &Password) -> Result<&[u8]> {
    let n = pw.len as usize;
    if pw.reserved != 0 || n == 0 || n > PASSWORD_MAX {
        return Err(EINVAL);
    }
    Ok(&pw.bytes[..n])
}

/// Copies in and checks the request `cmd` at `arg`. This is the whole of
/// what `/dev/sep-key` does with user memory before it acts.
pub(crate) fn read_request(cmd: u32, arg: usize) -> Result<KeyRequest> {
    let user = UserPtr::from_addr(arg);
    Ok(match cmd {
        IOC_INFO => KeyRequest::Info,
        IOC_CANCEL => KeyRequest::Cancel,
        IOC_CREATE => {
            let req = read_user_boxed::<Create>(user)?;
            let policy = check_create(&req)?;
            KeyRequest::Create(req, policy)
        }
        IOC_SIGN => {
            let req = read_user_boxed::<Sign>(user)?;
            check_sign(&req)?;
            KeyRequest::Sign(req)
        }
        IOC_PUBLIC => {
            let req = read_user_boxed::<Public>(user)?;
            check_public(&req)?;
            KeyRequest::Public(req)
        }
        IOC_SEAL => {
            let req = read_user_boxed::<Seal>(user)?;
            check_seal(&req)?;
            KeyRequest::Seal(req)
        }
        IOC_UNSEAL => {
            let req = read_user_boxed::<Seal>(user)?;
            check_unseal(&req)?;
            KeyRequest::Unseal(req)
        }
        IOC_SELFTEST => KeyRequest::Selftest(read_user_boxed::<Selftest>(user)?),
        IOC_CREATE_USER => {
            let req = read_user_boxed::<CreateUser>(user)?;
            check_create_user(&req)?;
            KeyRequest::CreateUser(req)
        }
        IOC_SIGN_PASSWORD => {
            let req = read_user_boxed::<SignPassword>(user)?;
            check_password(&req.password)?;
            check_password_handle(&check_sign(&req.sign)?)?;
            KeyRequest::SignPassword(req)
        }
        IOC_UNSEAL_PASSWORD => {
            let req = read_user_boxed::<UnsealPassword>(user)?;
            check_password(&req.password)?;
            check_password_handle(&check_unseal(&req.seal)?.0)?;
            KeyRequest::UnsealPassword(req)
        }
        _ => return Err(ENOTTY),
    })
}

pub(crate) fn info() -> Info {
    info_for(false)
}

/// `INFO`; `user_bags` when the key store has per-user bags (variant 5).
pub(crate) fn info_for(user_bags: bool) -> Info {
    let mut i = info_base();
    if user_bags {
        i.features |= FEATURE_PASSWORD;
    }
    i
}

fn info_base() -> Info {
    Info {
        version: IFACE_VERSION,
        flags_supported: CREATE_FLAGS,
        handle_max: HANDLE_MAX as u32,
        features: FEATURES,
    }
}

/// No flags, biometry (current set), or biometry plus biometry-any.
fn handle_flags_valid(flags: u8) -> bool {
    matches!(flags, 0 | HANDLE_BIOMETRY | 0x03)
}

/// Writes the handle for `blob` with `flags` into `out`; returns its length.
pub(crate) fn encode_handle(blob: &[u8], flags: u8, out: &mut [u8; HANDLE_MAX]) -> Result<usize> {
    if blob.is_empty() || blob.len() > BLOB_MAX || !handle_flags_valid(flags) {
        return Err(EINVAL);
    }
    let len = u32::try_from(blob.len()).map_err(|_| EINVAL)?;
    out.fill(0);
    out[..4].copy_from_slice(HANDLE_MAGIC);
    out[4] = HANDLE_VERSION;
    out[5] = flags;
    out[8..12].copy_from_slice(&len.to_le_bytes());
    out[HANDLE_HEADER..HANDLE_HEADER + blob.len()].copy_from_slice(blob);
    Ok(HANDLE_HEADER + blob.len())
}

/// Writes a version-2 handle for a record under `uid`'s bag.
pub(crate) fn encode_user_handle(blob: &[u8], uid: u32, plain: bool, out: &mut [u8; HANDLE_MAX]) -> Result<usize> {
    if blob.is_empty() || blob.len() > HANDLE_MAX - HANDLE_HEADER_USER {
        return Err(EINVAL);
    }
    let len = u32::try_from(blob.len()).map_err(|_| EINVAL)?;
    out.fill(0);
    out[..4].copy_from_slice(HANDLE_MAGIC);
    out[4] = HANDLE_VERSION_USER;
    out[5] = HANDLE_USER_BAG | (if plain { HANDLE_USER_PLAIN } else { 0 });
    out[8..12].copy_from_slice(&len.to_le_bytes());
    out[12..16].copy_from_slice(&uid.to_le_bytes());
    out[HANDLE_HEADER_USER..HANDLE_HEADER_USER + blob.len()].copy_from_slice(blob);
    Ok(HANDLE_HEADER_USER + blob.len())
}

/// A decoded handle: the enclave record, its flags and, for a record under
/// a user's bag, the uid.
pub(crate) struct Handle<'a> {
    pub(crate) blob: &'a [u8],
    pub(crate) flags: u8,
    pub(crate) uid: Option<u32>,
}

/// Either handle version. Version 2 is only `HANDLE_USER_BAG`.
pub(crate) fn decode_any_handle(handle: &[u8; HANDLE_MAX], len: u32) -> Result<Handle<'_>> {
    if handle[4] != HANDLE_VERSION_USER {
        let (blob, flags) = decode_handle(handle, len)?;
        return Ok(Handle { blob, flags, uid: None });
    }
    let len = len as usize;
    if !(HANDLE_HEADER_USER + 1..=HANDLE_MAX).contains(&len)
        || &handle[..4] != HANDLE_MAGIC
        || (handle[5] != HANDLE_USER_BAG && handle[5] != HANDLE_USER_BAG | HANDLE_USER_PLAIN)
        || handle[6] != 0
        || handle[7] != 0
    {
        return Err(EINVAL);
    }
    let blob_len = u32::from_le_bytes([handle[8], handle[9], handle[10], handle[11]]) as usize;
    if blob_len != len - HANDLE_HEADER_USER {
        return Err(EINVAL);
    }
    let uid = u32::from_le_bytes([handle[12], handle[13], handle[14], handle[15]]);
    Ok(Handle { blob: &handle[HANDLE_HEADER_USER..len], flags: handle[5], uid: Some(uid) })
}

/// The key store's designated user for `uid`'s bag (see
/// `userbag::USER_BAG_BASE`: never the identity bag's designation).
pub(crate) fn bag_user(uid: u32) -> Result<crate::sks::DesignateUser> {
    if uid < crate::userbag::MIN_UID {
        return Err(EINVAL);
    }
    crate::userbag::bag_designation(uid)
}

/// The tracked-context user id for a designated user.
fn designated_id(user: crate::sks::DesignateUser) -> i32 {
    -user.special_handle().value()
}

/// Whether a caller may make a key under the Linux identity (machine) bag.
/// As on macOS, where only the system makes machine-identity keys, that is
/// root (CAP_SYS_ADMIN or uid 0); users make keys under their own bag
/// (`CREATE_USER`).
pub(crate) fn may_create_machine_key(admin: bool, euid: u32) -> Result<()> {
    if admin || euid == 0 {
        Ok(())
    } else {
        Err(EPERM)
    }
}

/// A user-bag call may come from root or from the bag's own user.
pub(crate) fn require_admin_or(uid: u32) -> Result<()> {
    if crate::shim::capable_admin() || crate::shim::current_euid() == uid {
        Ok(())
    } else {
        Err(EPERM)
    }
}

/// The enclave record inside the first `len` bytes of `handle`, and the
/// handle's flags. Refuses an unknown version or flag, a non-zero reserved
/// field or a length that does not match exactly. The flags are not
/// authenticated; the enclave enforces the record's own list either way.
pub(crate) fn decode_handle(handle: &[u8; HANDLE_MAX], len: u32) -> Result<(&[u8], u8)> {
    let len = len as usize;
    if !(HANDLE_HEADER + 1..=HANDLE_MAX).contains(&len) {
        return Err(EINVAL);
    }
    if &handle[..4] != HANDLE_MAGIC || handle[4] != HANDLE_VERSION || !handle_flags_valid(handle[5]) {
        return Err(EINVAL);
    }
    if handle[6] != 0 || handle[7] != 0 {
        return Err(EINVAL);
    }
    let blob_len = u32::from_le_bytes([handle[8], handle[9], handle[10], handle[11]]) as usize;
    if blob_len != len - HANDLE_HEADER {
        return Err(EINVAL);
    }
    Ok((&handle[HANDLE_HEADER..len], handle[5]))
}

/// Copies a DER signature into the reply; refuses one that does not fit.
pub(crate) fn put_signature(sig: &[u8], out: &mut Sign) -> Result<()> {
    if sig.is_empty() || sig.len() > SIG_MAX {
        return Err(EIO);
    }
    out.signature.fill(0);
    out.signature[..sig.len()].copy_from_slice(sig);
    out.sig_len = sig.len() as u32;
    Ok(())
}

/// Copies an unsealed secret into the reply; refuses one that does not fit.
pub(crate) fn put_secret(secret: &[u8], out: &mut Seal) -> Result<()> {
    if secret.is_empty() || secret.len() > SECRET_MAX {
        return Err(EIO);
    }
    out.secret.fill(0);
    out.secret[..secret.len()].copy_from_slice(secret);
    out.secret_len = secret.len() as u32;
    Ok(())
}

/// The ECIES blob for `secret` to the public point of the record `blob`.
pub(crate) fn seal_to(blob: &[u8], secret: &[u8]) -> Result<KVec<u8>> {
    if secret.is_empty() || secret.len() > SECRET_MAX {
        return Err(EINVAL);
    }
    let point = SepData::user_key_public(blob)?;
    let sealed = crate::refkey_seal::ecies_seal(&point, secret)?;
    if sealed.len() > SEALED_MAX {
        return Err(EIO);
    }
    Ok(sealed)
}

/// macOS's `.userPresence` list: every constrained operation names the
/// `DeviceOwnerAuthentication` policy (Touch ID or the password). Used only
/// by `SELFTEST_PRESENCE_PROBE`.
/// The constraint `presence_acl` puts on each private operation.
pub(crate) fn presence_constraint() -> Result<KVec<u8>> {
    crate::der::encode_refkey_set(&[(b"cpo".as_slice(), RV::Utf8(b"DeviceOwnerAuthentication"))])
}

pub(crate) fn presence_acl() -> Result<KVec<u8>> {
    let constraint =
        crate::der::encode_refkey_set(&[(b"cpo".as_slice(), RV::Utf8(b"DeviceOwnerAuthentication"))])?;
    let acl = crate::der::encode_refkey_set(&[
        (b"oa".as_slice(), RV::Bool(true)),
        (b"ock".as_slice(), RV::Der(&constraint)),
        (b"odel".as_slice(), RV::Bool(true)),
        (b"okd".as_slice(), RV::Der(&constraint)),
        (b"osgn".as_slice(), RV::Der(&constraint)),
    ])?;
    crate::der::encode_refkey_set(&[(b"acl".as_slice(), RV::Der(&acl))])
}

/// The access-control list of a biometry-current-set key, in the key
/// store's DER, and the constraint its signing operation carries.
///
/// As macOS writes it for `.privateKeyUsage | .biometryCurrentSet`: sign,
/// key agreement (`ock`) and derivation (`okd`) need a biometric credential
/// (`cbio`) for this catacomb (`pbioc`) and enrolled set (`pbioh`); delete
/// (`odel`) and attest (`oa`) are free. SEP_KEY_UNSEAL uses the key
/// exchange (`UNSEAL_ACL_OP`), so a secret sealed to the key comes back only
/// after a match too. With no `enrolled_hash` it is the
/// `.biometryAny` list: `cbio` names the catacomb only.
pub(crate) struct BioAcl {
    pub(crate) ed: KVec<u8>,
    pub(crate) constraint: KVec<u8>,
}

/// The list operation SEP_KEY_UNSEAL needs: the enclave's key exchange
/// (`ock`). The enclave refuses its own decrypt (`oecd`) for every key with
/// a list, even one that leaves it free (self-test 24, 11-11), so the
/// shared secret comes from `ock` and the AES-GCM decrypt runs in the kernel,
/// as macOS does it for Secure Enclave keys.
pub(crate) const UNSEAL_ACL_OP: &[u8] = b"ock";

pub(crate) fn bio_acl(catacomb: &[u8; 16], enrolled_hash: Option<&[u8; 32]>) -> Result<BioAcl> {
    let cbio = match enrolled_hash {
        Some(hash) => crate::der::encode_refkey_set(&[
            (b"pbioc".as_slice(), RV::Octets(catacomb)),
            (b"pbioh".as_slice(), RV::Octets(hash)),
        ])?,
        None => crate::der::encode_refkey_set(&[(b"pbioc".as_slice(), RV::Octets(catacomb))])?,
    };
    let constraint = crate::der::encode_refkey_set(&[(b"cbio".as_slice(), RV::Der(&cbio))])?;
    let acl = crate::der::encode_refkey_set(&[
        (b"oa".as_slice(), RV::Bool(true)),
        (b"ock".as_slice(), RV::Der(&constraint)),
        (b"odel".as_slice(), RV::Bool(true)),
        (b"okd".as_slice(), RV::Der(&constraint)),
        (b"osgn".as_slice(), RV::Der(&constraint)),
    ])?;
    let ed = crate::der::encode_refkey_set(&[(b"acl".as_slice(), RV::Der(&acl))])?;
    Ok(BioAcl { ed, constraint })
}

/// What one verify run of a touch-gated signature ended with.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(crate) enum TouchOutcome {
    Matched,
    NoMatch,
    Failed(u32),
}

/// What a touch-gated call has the enclave do in the matched context.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(crate) enum TouchOp {
    /// `osgn` over the job's digest.
    Sign,
    /// The key exchange (`ock`) for the job's sealed blob, then the decrypt.
    Unseal,
}

/// The one key operation a touch-gated `SIGN` or `UNSEAL` call may have the
/// enclave run in the credential context its Touch ID match satisfied.
///
/// The enclave does not limit what a matched context authorises: one match
/// lets any number of operations through until the context is deleted
/// (measured, self-test 20). The bound is this type. A grant is made only
/// by [`KeyTouch::new`], once per call, and [`KeyGrant::run`] takes it by
/// value: an operation that ran, or failed in any way that might have let
/// the enclave act, consumes it, so nothing is left to run a second
/// operation with. Only an outright refusal by the enclave (`EKEYREJECTED`:
/// it performed nothing) hands the grant back, once, for the one retry
/// the verify makes after it completes the evaluation.
pub(crate) struct KeyGrant {
    op: TouchOp,
    /// The user bag the record lives under (`HANDLE_USER_BAG`), if any.
    bag: Option<u32>,
    blob: KVec<u8>,
    digest: [u8; DIGEST_LEN],
    sealed: KVec<u8>,
    retry: bool,
}

/// How a [`KeyGrant`] ended.
pub(crate) enum GrantEnd {
    /// The operation's result. The grant is gone.
    Done(Result<crate::Secret>),
    /// The enclave refused before doing anything; one more attempt in the
    /// same context may use the grant that comes back.
    Refused(KeyGrant),
}

impl KeyGrant {
    pub(crate) fn op(&self) -> TouchOp {
        self.op
    }

    pub(crate) fn bag(&self) -> Option<u32> {
        self.bag
    }

    /// Runs the operation through `exec` (the enclave call: operation,
    /// record, digest, sealed blob). Consumes the grant.
    pub(crate) fn run(
        mut self,
        exec: impl FnOnce(TouchOp, &[u8], &[u8; DIGEST_LEN], &[u8]) -> Result<KVec<u8>>,
    ) -> GrantEnd {
        match exec(self.op, &self.blob, &self.digest, &self.sealed) {
            Err(e) if e == EKEYREJECTED && self.retry => {
                self.retry = false;
                GrantEnd::Refused(self)
            }
            r => GrantEnd::Done(r.map(crate::Secret)),
        }
    }
}

/// Deletes an enclave credential context.
pub(crate) trait ContextDelete {
    fn delete_context(&self, handle: &[u8; crate::scrd::SCRD_ACM_HANDLE_LEN]) -> Result<()>;
}

impl ContextDelete for SepData {
    fn delete_context(&self, handle: &[u8; crate::scrd::SCRD_ACM_HANDLE_LEN]) -> Result<()> {
        self.delete_unlock_evaluation(handle)
    }
}

/// A credential context the kernel opened for one operation: deleted when
/// this is dropped, on every path out of the code that owns it (success,
/// error, cancel, timeout, signal). Its handle never leaves the kernel.
pub(crate) struct OwnedContext<'a, D: ContextDelete + ?Sized> {
    owner: &'a D,
    handle: [u8; crate::scrd::SCRD_ACM_HANDLE_LEN],
    disposed: bool,
}

impl<'a, D: ContextDelete + ?Sized> OwnedContext<'a, D> {
    pub(crate) fn new(owner: &'a D, handle: [u8; crate::scrd::SCRD_ACM_HANDLE_LEN]) -> Self {
        OwnedContext { owner, handle, disposed: false }
    }

    pub(crate) fn handle(&self) -> &[u8; crate::scrd::SCRD_ACM_HANDLE_LEN] {
        &self.handle
    }

    fn dispose(&mut self) -> Result<()> {
        if self.disposed {
            return Ok(());
        }
        self.disposed = true;
        let result = self.owner.delete_context(&self.handle);
        crate::image::wipe(&mut self.handle);
        result
    }

    /// Dispose before publishing the result. Failure drops any owned output.
    pub(crate) fn finish<T>(mut self, result: Result<T>) -> Result<T> {
        self.dispose()?;
        result
    }
}

impl<D: ContextDelete + ?Sized> Drop for OwnedContext<'_, D> {
    fn drop(&mut self) {
        let _ = self.dispose();
    }
}

/// Self-test copies of the operation's inputs, for the measurement modes
/// that act again after the one operation (5 always; 20 and 21 only with
/// `CONFIG_APPLE_SEP_KEY_RESEARCH`).
struct Probe {
    blob: KVec<u8>,
    digest: [u8; DIGEST_LEN],
    #[cfg(CONFIG_APPLE_SEP_KEY_RESEARCH)]
    sealed: KVec<u8>,
    /// Mode 5: sign once more with the matched context's handle after it
    /// was deleted (the enclave must refuse).
    after_delete: bool,
    /// Modes 20 and 21: a second operation in the same, still open, matched
    /// context. Not built into production kernels.
    #[cfg(CONFIG_APPLE_SEP_KEY_RESEARCH)]
    then: Option<TouchOp>,
}

/// The touch-gated call that is waiting for a match. Lives only for that
/// call; the verify it runs fills it in.
pub(crate) struct KeyTouch {
    op: TouchOp,
    grant: Option<KeyGrant>,
    constraint: KVec<u8>,
    outcome: Option<TouchOutcome>,
    result: Option<Result<crate::Secret>>,
    status: i32,
    probe: Option<Probe>,
    again: Option<Result<crate::Secret>>,
    again_status: i32,
}

impl KeyTouch {
    pub(crate) fn new(op: TouchOp, blob: &[u8], digest: &[u8; DIGEST_LEN], sealed: &[u8], constraint: KVec<u8>) -> Result<KeyTouch> {
        Self::new_in(op, blob, digest, sealed, constraint, None)
    }

    /// As `new`, for a record under the user bag `bag`.
    pub(crate) fn new_in(
        op: TouchOp,
        blob: &[u8],
        digest: &[u8; DIGEST_LEN],
        sealed: &[u8],
        constraint: KVec<u8>,
        bag: Option<u32>,
    ) -> Result<KeyTouch> {
        let mut b = KVec::new();
        b.extend_from_slice(blob, GFP_KERNEL)?;
        let mut sl = KVec::new();
        sl.extend_from_slice(sealed, GFP_KERNEL)?;
        Ok(KeyTouch {
            op,
            grant: Some(KeyGrant { op, bag, blob: b, digest: *digest, sealed: sl, retry: true }),
            constraint,
            outcome: None,
            result: None,
            status: 0,
            probe: None,
            again: None,
            again_status: 0,
        })
    }

    pub(crate) fn set_outcome(&mut self, outcome: &crate::bio::VerifyOutcome) {
        self.outcome = Some(match outcome {
            crate::bio::VerifyOutcome::Matched(_) => TouchOutcome::Matched,
            crate::bio::VerifyOutcome::NoMatch => TouchOutcome::NoMatch,
            crate::bio::VerifyOutcome::Failed(s) => TouchOutcome::Failed(*s),
        });
    }

    /// The grant, if the call's one operation has not run yet.
    pub(crate) fn take_grant(&mut self) -> Option<KeyGrant> {
        self.grant.take()
    }

    /// Files what the grant ended with. Returns whether the operation is
    /// done (the grant is gone for good).
    pub(crate) fn settle(&mut self, end: GrantEnd, status: i32) -> bool {
        self.status = status;
        match end {
            GrantEnd::Done(r) => {
                self.result = Some(r);
                true
            }
            GrantEnd::Refused(g) => {
                self.grant = Some(g);
                self.result = Some(Err(EKEYREJECTED));
                false
            }
        }
    }

    /// The operation's output, once it ran.
    pub(crate) fn into_result(mut self) -> Result<crate::Secret> {
        self.result.take().ok_or(EIO)?
    }
}

fn errno_of<T>(r: &Option<Result<T>>) -> i32 {
    match r {
        Some(Ok(_)) => 0,
        Some(Err(e)) => e.to_errno(),
        None => EIO.to_errno(),
    }
}

#[::kernel::macros::kunit_tests(apple_sep_key_handle)]
mod tests {
    use super::*;

    fn blob() -> [u8; 100] {
        let mut b = [0u8; 100];
        for (i, v) in b.iter_mut().enumerate() {
            *v = i as u8 ^ 0x5a;
        }
        b
    }

    #[test]
    fn round_trip() {
        let mut h = [0u8; HANDLE_MAX];
        let n = encode_handle(&blob(), 0, &mut h).unwrap();
        assert_eq!(n, HANDLE_HEADER + 100);
        assert_eq!(&h[..4], b"SEPK");
        assert_eq!(decode_handle(&h, n as u32).unwrap(), (&blob()[..], 0));
    }

    #[test]
    fn length_must_match_exactly() {
        let mut h = [0u8; HANDLE_MAX];
        let n = encode_handle(&blob(), 0, &mut h).unwrap() as u32;
        assert!(decode_handle(&h, n - 1).is_err());
        assert!(decode_handle(&h, n + 1).is_err());
        assert!(decode_handle(&h, 0).is_err());
        assert!(decode_handle(&h, HANDLE_MAX as u32 + 1).is_err());
    }

    #[test]
    fn header_fields_are_checked() {
        let mut h = [0u8; HANDLE_MAX];
        let n = encode_handle(&blob(), 0, &mut h).unwrap() as u32;
        for at in [0usize, 4, 6, 7] {
            let mut bad = h;
            bad[at] ^= 0x01;
            assert!(decode_handle(&bad, n).is_err());
        }
    }

    #[test]
    fn empty_and_oversize_blobs_are_refused() {
        let mut h = [0u8; HANDLE_MAX];
        assert!(encode_handle(&[], 0, &mut h).is_err());
        let big = [1u8; BLOB_MAX + 1];
        assert!(encode_handle(&big, 0, &mut h).is_err());
        let max = [1u8; BLOB_MAX];
        assert_eq!(encode_handle(&max, 0, &mut h).unwrap(), HANDLE_MAX);
    }

    #[test]
    fn signature_bounds() {
        let mut s = Sign {
            flags: 0,
            handle_len: 0,
            digest: [0; DIGEST_LEN],
            sig_len: 0,
            signature: [0; SIG_MAX],
            handle: [0; HANDLE_MAX],
        };
        assert!(put_signature(&[], &mut s).is_err());
        assert!(put_signature(&[0x30; SIG_MAX + 1], &mut s).is_err());
        put_signature(&[0x30, 0x02, 0x01, 0x01], &mut s).unwrap();
        assert_eq!(s.sig_len, 4);
    }

    #[test]
    fn biometry_flag_round_trips_and_unknown_flags_are_refused() {
        let mut h = [0u8; HANDLE_MAX];
        let n = encode_handle(&blob(), HANDLE_BIOMETRY, &mut h).unwrap() as u32;
        assert_eq!(decode_handle(&h, n).unwrap().1, HANDLE_BIOMETRY);
        assert!(encode_handle(&blob(), 0x02, &mut h).is_err());
        assert!(encode_handle(&blob(), 0x04, &mut h).is_err());
        let mut bad = h;
        bad[5] = 0x02;
        assert!(decode_handle(&bad, n).is_err());
        let n = encode_handle(&blob(), HANDLE_BIOMETRY | HANDLE_BIOMETRY_ANY, &mut h).unwrap() as u32;
        assert_eq!(decode_handle(&h, n).unwrap().1, HANDLE_BIOMETRY | HANDLE_BIOMETRY_ANY);
    }

    #[test]
    fn version_four_lists_cancel_seal_and_password() {
        let i = info();
        assert_eq!(i.version, 4);
        assert_eq!(info_for(true).features, i.features | FEATURE_PASSWORD);
        assert_eq!(info_for(true).features & FEATURE_PERSISTENT_ATTEMPTS, 0);
        assert_eq!(i.features, FEATURE_CANCEL | FEATURE_SEAL);
        assert_eq!(IOC_CANCEL, 0xb125);
        assert_eq!(IOC_SEAL >> 16 & 0x3fff, 2384);
        assert_eq!(IOC_UNSEAL & 0xff, 0x27);
    }

    #[test]
    fn secret_bounds() {
        let mut s = Seal {
            flags: 0,
            handle_len: 0,
            secret_len: 0,
            sealed_len: 0,
            secret: [0xaa; SECRET_MAX],
            sealed: [0; SEALED_MAX],
            handle: [0; HANDLE_MAX],
        };
        assert!(put_secret(&[], &mut s).is_err());
        assert!(put_secret(&[1; SECRET_MAX + 1], &mut s).is_err());
        put_secret(&[7; 5], &mut s).unwrap();
        assert_eq!(s.secret_len, 5);
        assert_eq!(&s.secret[..6], &[7, 7, 7, 7, 7, 0]);
        assert!(seal_to(&[0u8; 10], &[]).is_err());
        assert!(seal_to(&[0u8; 10], &[1; SECRET_MAX + 1]).is_err());
    }

    #[test]
    fn create_takes_one_policy_and_refuses_the_password_ones() {
        assert_eq!(create_policy(0).unwrap(), Policy::None);
        assert_eq!(create_policy(CREATE_BIOMETRY_CURRENT_SET).unwrap(), Policy::BiometryCurrentSet);
        assert_eq!(create_policy(CREATE_BIOMETRY_ANY).unwrap(), Policy::BiometryAny);
        assert_eq!(create_policy(CREATE_USER_PRESENCE).unwrap_err(), EOPNOTSUPP);
        assert_eq!(create_policy(CREATE_DEVICE_PASSCODE).unwrap_err(), EOPNOTSUPP);
        assert_eq!(create_policy(CREATE_BIOMETRY_CURRENT_SET | CREATE_BIOMETRY_ANY).unwrap_err(), EINVAL);
        assert_eq!(create_policy(0x10).unwrap_err(), EINVAL);
        assert_eq!(info().flags_supported, CREATE_BIOMETRY_CURRENT_SET | CREATE_BIOMETRY_ANY);
    }

    #[test]
    fn biometry_any_list_names_the_catacomb_only() {
        let acl = bio_acl(&[0x11; 16], None).unwrap();
        // SET { SEQ { "cbio", SET { SEQ { "pbioc", OCTETS[16] } } } }
        assert_eq!(acl.constraint.len(), 2 + 2 + 6 + 2 + (2 + 7 + 18));
        assert!(!acl.ed.windows(5).any(|w| w == b"pbioh"));
        assert!(acl.ed.windows(5).any(|w| w == b"pbioc"));
    }

    #[test]
    fn presence_list_has_the_captured_shape() {
        let ed = presence_acl().unwrap();
        // macOS: each constrained op is SET(34) { SEQ { "cpo", "DeviceOwnerAuthentication" } }.
        assert_eq!(ed.windows(2).filter(|w| *w == [0x31u8, 34].as_slice()).count(), 3);
        assert_eq!(ed.windows(25).filter(|w| *w == b"DeviceOwnerAuthentication").count(), 3);
    }

    #[test]
    fn biometric_acl_has_the_key_store_shape() {
        let acl = bio_acl(&[0x11; 16], Some(&[0x22; 32])).unwrap();
        // SET { SEQ { "cbio", SET { SEQ { "pbioc", OCTETS[16] }, SEQ { "pbioh", OCTETS[32] } } } }
        assert_eq!(acl.constraint.len(), 2 + 2 + 6 + 2 + (2 + 7 + 18) + (2 + 7 + 34));
        assert_eq!(&acl.constraint[..2], &[0x31, 0x50]);
        assert_eq!(&acl.constraint[4..10], b"\x0c\x04cbio");
        let ed = &acl.ed;
        assert_eq!(ed[0], 0x31);
        // The list names all six operations; delete and attest are free.
        for op in [b"oa".as_slice(), b"ock", b"odel", b"okd", b"osgn"] {
            assert!(ed.windows(op.len() + 2).any(|w| w[0] == 0x0c && w[1] as usize == op.len() && &w[2..] == op));
        }
        assert!(ed.windows(3).any(|w| w == [0x01u8, 0x01, 0x01].as_slice()));
    }

    #[test]
    fn user_handle_round_trips_and_keeps_the_uid() {
        let b = blob();
        let mut h = [0u8; HANDLE_MAX];
        let n = encode_user_handle(&b, 60124, false, &mut h).unwrap();
        assert_eq!(n, HANDLE_HEADER_USER + b.len());
        let d = decode_any_handle(&h, n as u32).unwrap();
        assert_eq!(d.blob, &b[..]);
        assert_eq!(d.flags, HANDLE_USER_BAG);
        assert_eq!(d.uid, Some(60124));
        // The old decoder never takes a user-bag handle for an identity-bag one.
        assert!(decode_handle(&h, n as u32).is_err());
        let mut bad = h;
        bad[5] = HANDLE_BIOMETRY;
        assert!(decode_any_handle(&bad, n as u32).is_err());
        assert!(decode_any_handle(&h, n as u32 - 1).is_err());
        let mut p = [0u8; HANDLE_MAX];
        let np = encode_user_handle(&b, 60124, true, &mut p).unwrap();
        assert_eq!(decode_any_handle(&p, np as u32).unwrap().flags, HANDLE_USER_BAG | HANDLE_USER_PLAIN);
        let mut v1 = [0u8; HANDLE_MAX];
        let n1 = encode_handle(&b, HANDLE_BIOMETRY, &mut v1).unwrap();
        let d1 = decode_any_handle(&v1, n1 as u32).unwrap();
        assert_eq!((d1.flags, d1.uid), (HANDLE_BIOMETRY, None));
    }

    #[test]
    fn only_root_makes_machine_keys() {
        assert_eq!(may_create_machine_key(true, 1000), Ok(()));
        assert_eq!(may_create_machine_key(false, 0), Ok(()));
        assert_eq!(may_create_machine_key(false, 1000), Err(EPERM));
        assert_eq!(may_create_machine_key(false, 60127), Err(EPERM));
    }

    #[test]
    fn user_bags_never_share_the_identity_designation() {
        let ryan = bag_user(1000).unwrap();
        assert_ne!(designated_id(ryan), crate::SBIO_PROBE_USER_ID);
        assert_eq!(designated_id(ryan), (crate::userbag::USER_BAG_BASE + 1000) as i32);
        assert!(bag_user(5).is_err());
        assert!(bag_user(u32::MAX).is_err());
        assert!(bag_user(60124).is_ok());
    }

    #[test]
    fn password_requests_are_checked() {
        let mut pw = Password { len: 3, reserved: 0, bytes: [0; PASSWORD_MAX] };
        pw.bytes[..3].copy_from_slice(b"abc");
        assert_eq!(check_password(&pw).unwrap(), b"abc");
        pw.len = 0;
        assert!(check_password(&pw).is_err());
        pw.len = PASSWORD_MAX as u32 + 1;
        assert!(check_password(&pw).is_err());
        pw.len = 3;
        pw.reserved = 1;
        assert!(check_password(&pw).is_err());
    }

    #[test]
    fn version_4_ioctls_have_their_sizes() {
        assert_eq!(IOC_CREATE_USER & 0xff, 0x28);
        assert_eq!(IOC_SIGN_PASSWORD >> 16 & 0x3fff, (2164 + 8 + PASSWORD_MAX) as u32);
        assert_eq!(IOC_UNSEAL_PASSWORD >> 16 & 0x3fff, (2384 + 8 + PASSWORD_MAX) as u32);
    }

    #[test]
    fn acl_constraint_command_has_the_captured_layout() {
        let acl = bio_acl(&[0x11; 16], Some(&[0x22; 32])).unwrap();
        let cmd = crate::scrd::scrd_verify_acl_constraint(&[0x33; 16], &acl.constraint, b"ock").unwrap();
        // macOS: 138 bytes for a 94-byte constraint and "ock"; this one is 80 bytes.
        assert_eq!(cmd.len(), 8 + 16 + 4 + acl.constraint.len() + 4 + 3 + 1 + 8);
        assert_eq!(&cmd[..8], b"DRCS\x20\x00\x00\x02");
        assert_eq!(&cmd[24..28], &(acl.constraint.len() as u32).to_le_bytes());
    }
}

impl SepData {
    pub(crate) fn register_key(&self) -> Result<()> {
        let ctx = core::ptr::from_ref(self).cast_mut().cast::<c_void>();
        self.publish_chardev(&self.key_dev, || {
            // SAFETY: as in `register_bio`: `remove()` drops the registration
            // before the `Arc` that keeps `ctx` alive, and the shim's unregister
            // waits for running ioctls and clears the stored context before
            // any later callback. Ioctls run concurrently: a touch-gated SIGN or
            // UNSEAL sleeps up to a minute, and CANCEL must reach it meanwhile.
            // They serialise on `catacomb_writer`; CANCEL only sets a flag.
            unsafe {
                crate::shim::BioChardev::register_concurrent(
                    DEVICE_NAME,
                    DEVICE_MODE,
                    ctx,
                    key_open_trampoline,
                    key_release_trampoline,
                    key_ioctl_trampoline,
                    key_ready_trampoline,
                )
            }
        })?;
        Ok(())
    }

    /// Whether the capture a verify is waiting on is still wanted: the
    /// touch-gated signature that owns it, or /dev/sep-bio's live session.
    /// A touch-gated call runs its capture in the caller's own thread, so a
    /// signal to the caller (or its death) ends the capture at once.
    pub(crate) fn capture_live(&self) -> bool {
        (self.key_touch_active.load(Relaxed)
            && !self.key_touch_cancel.load(Relaxed)
            && !self.shutting_down.load(Relaxed)
            && !kernel::current!().signal_pending())
            || crate::bio::capture_is_live(&self.bio_session.lock(), self.capture_epoch.load(Relaxed))
    }

    fn key_bio_acl(&self, any: bool) -> Result<BioAcl> {
        if !self.touchid_started.load(Relaxed) {
            return Err(ENODEV);
        }
        let (catacomb, enrolled) = self.catacomb_binding()?;
        bio_acl(&catacomb, (!any).then_some(&enrolled))
    }

    /// Evaluates the signing constraint on the fresh context before the
    /// match, as LocalAuthentication does. Best effort: logged only.
    pub(crate) fn key_touch_preflight(&self, evaluation: &[u8; crate::scrd::SCRD_ACM_HANDLE_LEN]) {
        let (constraint, op) = {
            let guard = self.key_touch.lock();
            let Some(job) = guard.as_ref() else {
                return;
            };
            let mut c = KVec::new();
            if c.extend_from_slice(&job.constraint, GFP_KERNEL).is_err() {
                return;
            }
            let op: &[u8] = match job.op {
                TouchOp::Sign => b"osgn",
                TouchOp::Unseal => UNSEAL_ACL_OP,
            };
            (c, op)
        };
        let Ok(payload) = crate::scrd::scrd_verify_acl_constraint(evaluation, &constraint, op) else {
            return;
        };
        match self.scrd_transact(
            c"SCRD_VERIFY_ACL_CONSTRAINT",
            crate::scrd::SCRD_REQUEST_COMMAND,
            &payload,
            crate::SCRD_TIMEOUT_MS,
        ) {
            Some((reply, out)) => {
                let word = |at: usize| {
                    out.get(at..at + 4)
                        .map(|b| u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
                };
                dev_dbg!(
                    self.dev,
                    "sep-key: signing constraint on the new context: status {}, {} reply bytes, satisfied {:?}, first requirement {:?}\n",
                    reply.status,
                    out.len(),
                    word(0),
                    word(4)
                );
            }
            None => dev_dbg!(self.dev, "sep-key: signing constraint check got no answer\n"),
        }
    }

    /// One operation of the waiting job in `evaluation`.
    fn key_touch_op(
        &self,
        op: TouchOp,
        blob: &[u8],
        digest: &[u8; DIGEST_LEN],
        sealed: &[u8],
        evaluation: &[u8; crate::scrd::SCRD_ACM_HANDLE_LEN],
    ) -> Result<KVec<u8>> {
        self.key_op_in(None, op, blob, digest, sealed, Some(evaluation))
    }

    /// Runs `op` on the record in `blob`, under the identity bag or under
    /// the user bag `bag`, checked against `acm`.
    fn key_op_in(
        &self,
        bag: Option<u32>,
        op: TouchOp,
        blob: &[u8],
        digest: &[u8; DIGEST_LEN],
        sealed: &[u8],
        acm: Option<&[u8; crate::scrd::SCRD_ACM_HANDLE_LEN]>,
    ) -> Result<KVec<u8>> {
        match (bag, op) {
            (None, TouchOp::Sign) => self.user_key_sign(blob, digest, acm),
            (None, TouchOp::Unseal) => self.user_key_unseal(blob, sealed, acm),
            (Some(uid), TouchOp::Sign) => self.user_key_sign_in_bag(bag_user(uid)?, blob, digest, acm),
            (Some(uid), TouchOp::Unseal) => self.user_key_unseal_in_bag(bag_user(uid)?, blob, sealed, acm),
        }
    }

    /// Runs the waiting call's one operation in `evaluation`, the context
    /// the match just satisfied. The verify calls this after the match and,
    /// in case the enclave wanted the completed evaluation, once more after
    /// it: the second call finds no grant unless the first was refused
    /// outright, so at most one operation ever runs per match.
    pub(crate) fn key_touch_sign(&self, evaluation: &[u8; crate::scrd::SCRD_ACM_HANDLE_LEN], when: &CStr) {
        let grant = match &mut *self.key_touch.lock() {
            Some(job) => job.take_grant(),
            None => None,
        };
        let Some(grant) = grant else {
            return;
        };
        let op = grant.op();
        let bag = grant.bag();
        let end = grant.run(|op, blob, digest, sealed| self.key_op_in(bag, op, blob, digest, sealed, Some(evaluation)));
        let status = self.last_key_status.load(Relaxed) as i32;
        let ok = matches!(end, GrantEnd::Done(Ok(_)));
        dev_dbg!(
            self.dev,
            "sep-key: touch-gated {:?} {}: {} (enclave status {})\n",
            op,
            when,
            (if ok { "done" } else { "refused" }),
            status
        );
        if let Some(job) = &mut *self.key_touch.lock() {
            job.settle(end, status);
        }
        if ok {
            self.key_touch_research_second(evaluation);
        }
    }

    /// Production kernels: nothing runs after the one operation.
    #[cfg(not(CONFIG_APPLE_SEP_KEY_RESEARCH))]
    fn key_touch_research_second(&self, _evaluation: &[u8; crate::scrd::SCRD_ACM_HANDLE_LEN]) {}

    /// Research build only (modes 20 and 21): a second operation in the
    /// matched context that is still open, to measure what the enclave lets
    /// through. Production kernels do not contain this.
    #[cfg(CONFIG_APPLE_SEP_KEY_RESEARCH)]
    fn key_touch_research_second(&self, evaluation: &[u8; crate::scrd::SCRD_ACM_HANDLE_LEN]) {
        let (second, blob, digest, sealed) = {
            let guard = self.key_touch.lock();
            let Some(Probe { then: Some(second), blob, digest, sealed, .. }) = guard.as_ref().and_then(|j| j.probe.as_ref()) else {
                return;
            };
            let mut b = KVec::new();
            let mut sl = KVec::new();
            if b.extend_from_slice(blob, GFP_KERNEL).is_err() || sl.extend_from_slice(sealed, GFP_KERNEL).is_err() {
                return;
            }
            (*second, b, *digest, sl)
        };
        let r2 = self.key_touch_op(second, &blob, &digest, &sealed, evaluation).map(crate::Secret);
        let status2 = self.last_key_status.load(Relaxed) as i32;
        dev_info_ratelimited!(
            self.dev,
            "sep-key: self-test: second operation ({:?}) in the same matched context: {} (enclave status {})\n",
            second,
            (if r2.is_ok() { "DONE" } else { "refused" }),
            status2
        );
        if let Some(job) = &mut *self.key_touch.lock() {
            if job.again.is_none() {
                job.again = Some(r2);
                job.again_status = status2;
            }
        }
    }

    /// Self-test only (mode 5): after the matched context is deleted, sign
    /// once more with its old handle; the enclave must refuse.
    pub(crate) fn key_touch_after_close(&self, evaluation: &[u8; crate::scrd::SCRD_ACM_HANDLE_LEN]) {
        let (blob, digest) = {
            let guard = self.key_touch.lock();
            let Some(job) = guard.as_ref() else {
                return;
            };
            let Some(probe) = job.probe.as_ref().filter(|p| p.after_delete) else {
                return;
            };
            if !matches!(job.result, Some(Ok(_))) {
                return;
            }
            let mut b = KVec::new();
            if b.extend_from_slice(&probe.blob, GFP_KERNEL).is_err() {
                return;
            }
            (b, probe.digest)
        };
        let r = self.user_key_sign(&blob, &digest, Some(evaluation)).map(crate::Secret);
        let status = self.last_key_status.load(Relaxed) as i32;
        dev_info_ratelimited!(
            self.dev,
            "sep-key: self-test: signature with the deleted matched context: {} (enclave status {})\n",
            (if r.is_ok() { "SIGNED" } else { "refused" }),
            status
        );
        if let Some(job) = &mut *self.key_touch.lock() {
            job.again = Some(r);
            job.again_status = status;
        }
    }

    /// A touch-gated signature: one fresh credential context, one Touch ID
    /// match in it, the signature, and the context deleted, all inside this
    /// call. Waits about a minute for a finger.
    fn key_sign_with_touch(&self, blob: &[u8], flags: u8, digest: &[u8; DIGEST_LEN]) -> Result<KeyTouch> {
        self.key_touch_run(blob, flags, TouchOp::Sign, digest, &[], None, None)
    }

    /// A touch-gated operation (see `key_sign_with_touch`). `CANCEL` ends
    /// the wait with `ECANCELED`, a signal to the caller (including its
    /// death) with `EINTR`.
    fn key_touch_run(
        &self,
        blob: &[u8],
        flags: u8,
        op: TouchOp,
        digest: &[u8; DIGEST_LEN],
        sealed: &[u8],
        probe: Option<Probe>,
        bag: Option<u32>,
    ) -> Result<KeyTouch> {
        let constraint = if bag.is_some() {
            presence_constraint()?
        } else {
            self.key_bio_acl(flags & HANDLE_BIOMETRY_ANY != 0)?.constraint
        };
        let mut job = KeyTouch::new_in(op, blob, digest, sealed, constraint, bag)?;
        job.probe = probe;
        // Serialises with /dev/sep-bio's enrol and verify work, as they do
        // with each other, and with every other touch-gated call: one job,
        // one grant, one context at a time.
        let _writer = self.catacomb_writer.lock();
        *self.key_touch.lock() = Some(job);
        self.key_touch_cancel.store(false, Relaxed);
        self.key_touch_active.store(true, Relaxed);
        dev_dbg!(self.dev, "sep-key: waiting for a Touch ID match ({:?})\n", op);
        let deadline = crate::shim::boottime_ns().saturating_add(TOUCH_WAIT_MS * 1_000_000);
        let ended = loop {
            if self.shutting_down.load(Relaxed) {
                break Err(ENODEV);
            }
            if self.key_touch_cancel.load(Relaxed) {
                break Err(ECANCELED);
            }
            if kernel::current!().signal_pending() {
                break Err(EINTR);
            }
            if !self.capture_begin() {
                break Err(EAGAIN);
            }
            self.run_verify();
            self.capture_end();
            let outcome = match &mut *self.key_touch.lock() {
                Some(job) => job.outcome.take(),
                None => None,
            };
            if outcome != Some(TouchOutcome::Matched) {
                if self.key_touch_cancel.load(Relaxed) {
                    break Err(ECANCELED);
                }
                if kernel::current!().signal_pending() {
                    break Err(EINTR);
                }
            }
            match outcome {
                Some(TouchOutcome::Matched) => break Ok(()),
                Some(TouchOutcome::NoMatch) => break Err(EACCES),
                Some(TouchOutcome::Failed(s)) if s == crate::ENROL_STATUS_RETRY => {
                    if crate::shim::boottime_ns() >= deadline {
                        break Err(ETIMEDOUT);
                    }
                }
                Some(TouchOutcome::Failed(s)) if s == crate::ENROL_STATUS_LOCKOUT => break Err(EACCES),
                Some(TouchOutcome::Failed(_)) | None => break Err(EIO),
            }
        };
        self.key_touch_active.store(false, Relaxed);
        self.key_touch_cancel.store(false, Relaxed);
        // Whatever is left of the grant goes with the job.
        let job = self.key_touch.lock().take().ok_or(EIO)?;
        dev_dbg!(
            self.dev,
            "sep-key: touch-gated {:?} ended: {:?}, result {}\n",
            op,
            ended,
            errno_of(&job.result)
        );
        ended?;
        if self.key_context_failed.load(Relaxed) {
            return Err(EIO);
        }
        Ok(job)
    }

    /// A credential context holding only the identity key bag's secret, the
    /// Linux stand-in for a passcode (self-test).
    fn passcode_context(&self) -> Result<[u8; crate::scrd::SCRD_ACM_HANDLE_LEN]> {
        let du = crate::sks::DesignateUser::new(crate::SBIO_PROBE_USER_ID).ok_or(EINVAL)?;
        let crate::keybag::State::Present(stored) = crate::keybag::read(crate::keybag::Slot::Identity)? else {
            return Err(ENODEV);
        };
        let (status, _) = self.scrd_command(&crate::scrd::scrd_initialize()).ok_or(EIO)?;
        if status != 0 {
            return Err(EIO);
        }
        let (status, out) = self
            .scrd_command(&crate::scrd::scrd_context_create_tracked(crate::SBIO_PROBE_USER_ID))
            .ok_or(EIO)?;
        let handle = crate::scrd::tracked_create_reference(&out)
            .filter(|_| status == 0)
            .ok_or(EIO)?;
        let ok = (|| -> Option<()> {
            let (status, _) = self.scrd_command(&crate::scrd::scrd_context_externalize(&handle))?;
            if status != 0 {
                return None;
            }
            let out = self.sks_send(self.sks_req_verify_secret(du.special_handle(), stored.secret(), &handle))?;
            dev_info_ratelimited!(self.dev, "sep-key: self-test: passcode credential added, status {}\n", out.reply.status);
            (out.reply.status == 0).then_some(())
        })();
        if ok.is_none() {
            self.close_unlock_evaluation(&handle);
            return Err(EIO);
        }
        Ok(handle)
    }

    /// A fresh tracked context for `uid`; with `secret`, a passcode
    /// credential from `bag`'s verify-secret is added. Returns the context
    /// and the verify status (0 when none was asked).
    fn key_context_for(
        &self,
        uid: i32,
        cred: Option<(crate::sks::SpecialHandle, &[u8])>,
    ) -> Result<([u8; crate::scrd::SCRD_ACM_HANDLE_LEN], i32)> {
        let (status, _) = self.scrd_command(&crate::scrd::scrd_initialize()).ok_or(EIO)?;
        if status != 0 {
            return Err(EIO);
        }
        let (status, out) = self.scrd_command(&crate::scrd::scrd_context_create_tracked(uid)).ok_or(EIO)?;
        let ctx = crate::scrd::tracked_create_reference(&out).filter(|_| status == 0).ok_or(EIO)?;
        let (status, _) = self.scrd_command(&crate::scrd::scrd_context_externalize(&ctx)).ok_or(EIO)?;
        if status != 0 {
            self.close_unlock_evaluation(&ctx);
            return Err(EIO);
        }
        let mut verify = 0;
        if let Some((bag, secret)) = cred {
            verify = match self.sks_send(self.sks_req_verify_secret(bag, secret, &ctx)) {
                Some(out) => out.reply.status as i32,
                None => -(EIO.to_errno()),
            };
        }
        Ok((ctx, verify))
    }

    fn key_userbag_record(&self, req: &mut Selftest) -> Result<()> {
        let uid = req.enclave_status;
        if uid < crate::sks::SKS_DESIGNATE_USER_MIN {
            return Err(EINVAL);
        }
        let user = bag_user(u32::try_from(uid).map_err(|_| EINVAL)?)?;
        if req.mode == SELFTEST_USERBAG_KEY_MAKE {
            let ed = presence_acl()?;
            let blob = self.user_key_create_in_bag(user, Some(&ed))?;
            if blob.len() > HANDLE_MAX {
                return Err(E2BIG);
            }
            req.handle.fill(0);
            req.handle[..blob.len()].copy_from_slice(&blob);
            req.handle_len = blob.len() as u32;
            req.result = 0;
            dev_info_ratelimited!(self.dev, "sep-key: self-test 11: key under uid {}'s bag, record {} bytes\n", uid, blob.len());
            return Ok(());
        }
        let len = req.handle_len as usize;
        if len == 0 || len > HANDLE_MAX {
            return Err(EINVAL);
        }
        let mut pw = req.digest;
        req.digest.fill(0);
        let plen = pw.iter().position(|&b| b == 0).unwrap_or(pw.len());
        if plen == 0 {
            return Err(EINVAL);
        }
        let mut record = KVec::new();
        record.extend_from_slice(&req.handle[..len], GFP_KERNEL)?;
        let made = self.key_context_for(designated_id(user), Some((user.special_handle(), &pw[..plen])));
        pw.fill(0);
        let (ctx, v) = made?;
        req.result2 = v;
        self.last_key_status.store(0, Relaxed);
        let r = self.user_key_sign_in_bag(user, &record, &[0x5a; DIGEST_LEN], Some(&ctx));
        self.close_unlock_evaluation(&ctx);
        req.result = r.err().map(|e| e.to_errno()).unwrap_or(0);
        req.enclave_status = self.last_key_status.load(Relaxed) as i32;
        dev_info_ratelimited!(
            self.dev,
            "sep-key: self-test 12: saved key under uid {}'s bag: verify {} -> {} ({})\n",
            uid,
            v,
            (if req.result == 0 { "SIGNED" } else { "refused" }),
            req.enclave_status
        );
        Ok(())
    }

    fn key_userbag_locked(&self, req: &mut Selftest) -> Result<()> {
        let uid = req.enclave_status;
        let len = req.handle_len as usize;
        if len == 0 || len > 256 || uid < crate::sks::SKS_DESIGNATE_USER_MIN {
            return Err(EINVAL);
        }
        let mut pw = [0u8; 256];
        pw[..len].copy_from_slice(&req.handle[..len]);
        req.handle.fill(0);
        let user = bag_user(u32::try_from(uid).map_err(|_| EINVAL)?)?;
        let out = (|| -> Result<[i32; 8]> {
            let ed = presence_acl()?;
            let blob = self.user_key_create_in_bag(user, Some(&ed))?;
            let digest = [0x5au8; DIGEST_LEN];
            let sign = |ctx: Option<&[u8; crate::scrd::SCRD_ACM_HANDLE_LEN]>| -> i32 {
                self.last_key_status.store(0, Relaxed);
                match self.user_key_sign_in_bag(user, &blob, &digest, ctx) {
                    Ok(_) => 0,
                    Err(_) => match self.last_key_status.load(Relaxed) as i32 {
                        0 => -(EIO.to_errno()),
                        s => s,
                    },
                }
            };
            let mut r = [0i32; 8];
            r[5] = self.userbag_lock_phase(user).map(|p| p as i32).unwrap_or(-1);
            // Owned: an early return below still deletes it.
            let stale = OwnedContext::new(self, self.key_context_for(designated_id(user), Some((user.special_handle(), &pw[..len])))?.0);
            r[0] = sign(Some(stale.handle()));
            // Lock for good: the screen lock, then grace-timer fires (as the
            // Mac's kernel sends them) until the store locks the bag, for at
            // most LOCK_WAIT_S. The store sets the grace period itself.
            const LOCK_WAIT_S: u64 = 330;
            self.userbag_lock(user, true, false)?;
            let start = crate::shim::boottime_ns();
            let mut locked = false;
            while crate::shim::boottime_ns() - start < LOCK_WAIT_S * 1_000_000_000 {
                kernel::time::delay::fsleep(kernel::time::Delta::from_millis(5000));
                match self.userbag_timer_fire(user) {
                    Ok(true) => {
                        locked = true;
                        break;
                    }
                    Ok(false) => {}
                    Err(e) => {
                        dev_info_ratelimited!(self.dev, "sep-key: self-test 10: timer fire returned {:?}\n", e);
                        break;
                    }
                }
            }
            dev_info_ratelimited!(
                self.dev,
                "sep-key: self-test 10: bag locked by the store: {} after {} s\n",
                locked,
                (crate::shim::boottime_ns() - start) / 1_000_000_000
            );
            r[6] = self.userbag_lock_phase(user).map(|p| p as i32).unwrap_or(-1);
            r[1] = sign(Some(stale.handle()));
            drop(stale);
            r[2] = sign(None);
            let (fresh, v) = self.key_context_for(designated_id(user), Some((user.special_handle(), &pw[..len])))?;
            r[3] = if v == 0 { sign(Some(&fresh)) } else { v };
            self.close_unlock_evaluation(&fresh);
            self.userbag_unlock(user, &pw[..len])?;
            r[7] = self.userbag_lock_phase(user).map(|p| p as i32).unwrap_or(-1);
            let (fresh2, v2) = self.key_context_for(designated_id(user), Some((user.special_handle(), &pw[..len])))?;
            r[4] = if v2 == 0 { sign(Some(&fresh2)) } else { v2 };
            self.close_unlock_evaluation(&fresh2);
            Ok(r)
        })();
        pw.fill(0);
        let r = out?;
        for (i, v) in r.iter().enumerate() {
            req.digest[i * 4..i * 4 + 4].copy_from_slice(&v.to_le_bytes());
        }
        req.result = 0;
        dev_info_ratelimited!(
            self.dev,
            "sep-key: self-test 10, uid {}'s bag: phase {} -> sign {}; locked (phase {}): stale credential {}, none {}, fresh password credential {}; unlocked (phase {}): {}\n",
            uid, r[5], r[0], r[6], r[1], r[2], r[3], r[7], r[4]
        );
        Ok(())
    }

    fn key_userbag_probe(&self, req: &mut Selftest) -> Result<()> {
        let uid = req.enclave_status;
        let len = req.handle_len as usize;
        if len == 0 || len > 256 || uid < crate::sks::SKS_DESIGNATE_USER_MIN {
            return Err(EINVAL);
        }
        let mut pw = [0u8; 256];
        pw[..len].copy_from_slice(&req.handle[..len]);
        req.handle.fill(0);
        let user = bag_user(u32::try_from(uid).map_err(|_| EINVAL)?)?;
        let ed = presence_acl()?;
        let made = self.user_key_create_in_bag(user, Some(&ed));
        let blob = match made {
            Ok(b) => b,
            Err(e) => {
                pw.fill(0);
                dev_info_ratelimited!(self.dev, "sep-key: self-test: key under uid {}'s bag not created ({:?})\n", uid, e);
                return Err(e);
            }
        };
        let digest = req.digest;
        let sign = |ctx: &[u8; crate::scrd::SCRD_ACM_HANDLE_LEN]| -> (i32, i32) {
            self.last_key_status.store(0, Relaxed);
            let r = self.user_key_sign_in_bag(user, &blob, &digest, Some(ctx));
            (r.err().map(|e| e.to_errno()).unwrap_or(0), self.last_key_status.load(Relaxed) as i32)
        };
        let first = if req.mode == SELFTEST_USERBAG_KEY_SAME_BAG {
            self.key_context_for(designated_id(user), Some((user.special_handle(), &pw[..len])))
        } else {
            let du = crate::sks::DesignateUser::new(crate::SBIO_PROBE_USER_ID).ok_or(EINVAL)?;
            match crate::keybag::read(crate::keybag::Slot::Identity) {
                Ok(crate::keybag::State::Present(stored)) => {
                    self.key_context_for(crate::SBIO_PROBE_USER_ID, Some((du.special_handle(), stored.secret())))
                }
                _ => Err(ENODEV),
            }
        };
        let (ctx, v1) = match first {
            Ok(c) => c,
            Err(e) => {
                pw.fill(0);
                return Err(e);
            }
        };
        (req.result, req.enclave_status) = sign(&ctx);
        self.close_unlock_evaluation(&ctx);
        let second = if req.mode == SELFTEST_USERBAG_KEY_SAME_BAG {
            self.key_context_for(designated_id(user), None)
        } else {
            self.key_context_for(designated_id(user), Some((user.special_handle(), &pw[..len])))
        };
        pw.fill(0);
        let (ctx2, v2) = second?;
        (req.result2, req.enclave_status2) = sign(&ctx2);
        self.close_unlock_evaluation(&ctx2);
        dev_info_ratelimited!(
            self.dev,
            "sep-key: self-test {}: key under uid {}'s bag: first ({}) verify {} -> {} ({}); second ({}) verify {} -> {} ({})\n",
            req.mode,
            uid,
            (if req.mode == SELFTEST_USERBAG_KEY_SAME_BAG { "same-bag password" } else { "identity-bag secret" }),
            v1,
            (if req.result == 0 { "SIGNED" } else { "refused" }),
            req.enclave_status,
            (if req.mode == SELFTEST_USERBAG_KEY_SAME_BAG { "unsatisfied" } else { "given password" }),
            v2,
            (if req.result2 == 0 { "SIGNED" } else { "refused" }),
            req.enclave_status2
        );
        Ok(())
    }

    fn key_presence_other_bag(&self, req: &mut Selftest) -> Result<()> {
        let uid = req.enclave_status;
        let len = req.handle_len as usize;
        if len == 0 || len > 256 || uid < crate::sks::SKS_DESIGNATE_USER_MIN {
            return Err(EINVAL);
        }
        let du = bag_user(u32::try_from(uid).map_err(|_| EINVAL)?)?;
        let ed = presence_acl()?;
        let (blob, _) = self.user_key_create(Some(&ed))?;
        let digest = req.digest;
        let (status, _) = self.scrd_command(&crate::scrd::scrd_initialize()).ok_or(EIO)?;
        if status != 0 {
            return Err(EIO);
        }
        let (status, out) = self.scrd_command(&crate::scrd::scrd_context_create_tracked(uid)).ok_or(EIO)?;
        let ctx = crate::scrd::tracked_create_reference(&out).filter(|_| status == 0).ok_or(EIO)?;
        let verify = (|| -> Option<i32> {
            let (status, _) = self.scrd_command(&crate::scrd::scrd_context_externalize(&ctx))?;
            if status != 0 {
                return None;
            }
            let out = self.sks_send(self.sks_req_verify_secret(du.special_handle(), &req.handle[..len], &ctx))?;
            Some(out.reply.status as i32)
        })();
        req.handle.fill(0);
        req.result2 = verify.unwrap_or(-(EIO.to_errno()));
        self.last_key_status.store(0, Relaxed);
        let r = self.user_key_sign(&blob, &digest, Some(&ctx));
        req.result = r.err().map(|e| e.to_errno()).unwrap_or(0);
        req.enclave_status = self.last_key_status.load(Relaxed) as i32;
        self.close_unlock_evaluation(&ctx);
        dev_info_ratelimited!(
            self.dev,
            "sep-key: self-test presence key with a credential from uid {}'s bag: verify status {}, signature {} (enclave status {})\n",
            uid,
            req.result2,
            (if req.result == 0 { "SIGNED" } else { "refused" }),
            req.enclave_status
        );
        Ok(())
    }

    fn key_presence_probe(&self, req: &mut Selftest) -> Result<()> {
        let ed = presence_acl()?;
        let (blob, _) = self.user_key_create(Some(&ed))?;
        let digest = req.digest;
        let user = crate::sbio::UserId::new(crate::SBIO_PROBE_USER_ID).ok_or(EINVAL)?;
        let record = |r: Result<KVec<u8>>, s: &SepData| -> (i32, i32) {
            (r.err().map(|e| e.to_errno()).unwrap_or(0), s.last_key_status.load(Relaxed) as i32)
        };
        self.last_key_status.store(0, Relaxed);
        let ctx = self.passcode_context()?;
        (req.result, req.enclave_status) = record(self.user_key_sign(&blob, &digest, Some(&ctx)), self);
        self.close_unlock_evaluation(&ctx);
        self.last_key_status.store(0, Relaxed);
        let ctx = self.open_unlock_evaluation(user).ok_or(EIO)?;
        (req.result2, req.enclave_status2) = record(self.user_key_sign(&blob, &digest, Some(&ctx)), self);
        self.close_unlock_evaluation(&ctx);
        dev_info_ratelimited!(
            self.dev,
            "sep-key: self-test presence probe: bag-secret context {} (enclave status {}), unsatisfied context {} (enclave status {})\n",
            (if req.result == 0 { "SIGNED" } else { "refused" }),
            req.enclave_status,
            (if req.result2 == 0 { "SIGNED" } else { "refused" }),
            req.enclave_status2
        );
        Ok(())
    }

    /// One `SELFTEST_UNSEAL_CONTEXT_PROBE` key: the bits of the rows that
    /// worked on it.
    fn key_unseal_context_probe_key(&self, name: &str, free: Option<&[&[u8]]>, secret: &[u8]) -> Result<i32> {
        use crate::der::RefKeyValue as RV;
        let ed = match free {
            None => None,
            Some(ops) => {
                let mut items: KVec<(&[u8], RV<'_>)> = KVec::new();
                for op in ops {
                    items.push((op, RV::Bool(true)), GFP_KERNEL)?;
                }
                let acl = crate::der::encode_refkey_set(&items)?;
                Some(crate::der::encode_refkey_set(&[(b"acl".as_slice(), RV::Der(&acl))])?)
            }
        };
        let (blob, point) = self.user_key_create(ed.as_deref())?;
        let sealed = seal_to(&blob, secret)?;
        let digest = [0x5au8; DIGEST_LEN];
        let user = crate::sbio::UserId::new(crate::SBIO_PROBE_USER_ID).ok_or(EINVAL)?;
        let mut bits = 0i32;
        for (row, (what, op, ctx_kind)) in UNSEAL_CONTEXT_PROBE_ROWS.iter().enumerate() {
            let ctx = match ctx_kind {
                0 => None,
                1 => Some(OwnedContext::new(self, self.open_unlock_evaluation(user).ok_or(EIO)?)),
                _ => Some(OwnedContext::new(self, self.passcode_context()?)),
            };
            let (eph, z) = crate::refkey_seal::ecdh_probe_pair(&point)?;
            self.last_key_status.store(0, Relaxed);
            let acm = ctx.as_ref().map(|c| c.handle());
            let r = match *op {
                b"unseal" => self.user_key_unseal(&blob, &sealed, acm).map(|v| (0, v)),
                b"osgn" => self.user_key_raw_op(&blob, op, b"d", &digest, acm),
                b"oecd" => self.user_key_raw_op(&blob, op, b"d", &sealed, acm),
                _ => self.user_key_raw_op(&blob, op, b"pub", &eph, acm),
            };
            drop(ctx);
            let (status, out) = match r {
                Ok(v) => v,
                Err(_) => (self.last_key_status.load(Relaxed) as i32, KVec::new()),
            };
            let ok = status == 0
                && match *op {
                    b"oecd" | b"unseal" => out[..] == secret[..],
                    b"ock" => out.windows(32).any(|w| w == z),
                    _ => !out.is_empty(),
                };
            dev_info_ratelimited!(
                self.dev,
                "sep-key: self-test decrypt probe: {}: {}: {} (enclave status {}, {} reply bytes)\n",
                name,
                what,
                (if ok { "OK" } else { "refused" }),
                status,
                out.len()
            );
            if ok {
                bits |= 1 << row;
            }
        }
        Ok(bits)
    }

    fn key_unseal_context_probe(&self, req: &mut Selftest) -> Result<()> {
        const OECD_FREE: &[&[u8]] = &[b"oa", b"odel", b"osgn", b"oecd"];
        const NO_DECRYPT: &[&[u8]] = &[b"oa", b"odel", b"osgn"];
        const OCK_FREE: &[&[u8]] = &[b"oa", b"odel", b"osgn", b"ock"];
        const MACOS_PLAIN: &[&[u8]] = &[b"oa", b"ock", b"odel", b"okd", b"osgn"];
        let secret = req.digest;
        let keys: [(&str, Option<&[&[u8]]>); 5] = [
            ("no list", None),
            ("oecd free", Some(OECD_FREE)),
            ("no decrypt entry", Some(NO_DECRYPT)),
            ("ock free", Some(OCK_FREE)),
            ("macOS plain", Some(MACOS_PLAIN)),
        ];
        let (mut lo, mut hi) = (0i32, 0i32);
        for (i, (name, free)) in keys.iter().enumerate() {
            let bits = match self.key_unseal_context_probe_key(name, *free, &secret) {
                Ok(b) => b,
                Err(e) => {
                    dev_info_ratelimited!(self.dev, "sep-key: self-test decrypt probe: {}: key failed ({:?})\n", name, e);
                    0
                }
            };
            if i < 4 {
                lo |= bits << (8 * i);
            } else {
                hi = bits;
            }
        }
        req.result = lo;
        req.enclave_status = 0;
        req.result2 = hi;
        req.enclave_status2 = 0;
        Ok(())
    }

    /// `op` on a record under `uid`'s bag with that bag's password as the
    /// credential: a fresh tracked context, the enclave's VERIFY_SECRET of
    /// the password into it, the one operation, the context deleted (it is
    /// an `OwnedContext`). Nothing is kept. `EACCES` when the enclave refuses
    /// the password (it counts the attempt), `EKEYREJECTED` when it refuses
    /// the operation (a locked bag, or a record from another bag).
    fn key_op_with_password(
        &self,
        uid: u32,
        blob: &[u8],
        op: TouchOp,
        digest: &[u8; DIGEST_LEN],
        sealed: &[u8],
        pw: &[u8],
    ) -> Result<crate::Secret> {
        let bag = bag_user(uid)?;
        // Not under `catacomb_writer`: a touch wait holds it for up to a
        // minute, and this path needs no capture.
        let (handle, verify) = self.key_context_for(designated_id(bag), Some((bag.special_handle(), pw)))?;
        let ctx = OwnedContext::new(self, handle);
        let r = if verify != 0 {
            dev_info_refusal!(self.dev, "sep-key: uid {}'s password refused by the enclave (status {})\n", uid, verify);
            Err(EACCES)
        } else {
            self.key_op_in(Some(uid), op, blob, digest, sealed, Some(ctx.handle())).map(crate::Secret)
        };
        let r = ctx.finish(r);
        dev_dbg!(
            self.dev,
            "sep-key: password-gated {:?} under uid {}'s bag: {}\n",
            op,
            uid,
            (if r.is_ok() { "done" } else { "refused" })
        );
        r
    }

    fn key_selftest(&self, req: &mut Selftest) -> Result<()> {
        if crate::module_parameters::key_selftest.value() == 0 {
            return Err(EPERM);
        }
        crate::bio::require_admin()?;
        if !selftest_mode_built(req.mode) {
            return Err(EOPNOTSUPP);
        }
        if req.mode == SELFTEST_PRESENCE_PROBE {
            return self.key_presence_probe(req);
        }
        if req.mode == SELFTEST_PRESENCE_OTHER_BAG {
            return self.key_presence_other_bag(req);
        }
        if req.mode == SELFTEST_USERBAG_KEY_MAKE || req.mode == SELFTEST_USERBAG_KEY_USE {
            return self.key_userbag_record(req);
        }
        if req.mode == SELFTEST_USERBAG_LOCKED {
            return self.key_userbag_locked(req);
        }
        if req.mode == SELFTEST_USERBAG_KEY_SAME_BAG || req.mode == SELFTEST_USERBAG_KEY_ROOT {
            return self.key_userbag_probe(req);
        }
        if req.mode == SELFTEST_UNSEAL_CONTEXT_PROBE {
            return self.key_unseal_context_probe(req);
        }
        let (blob, flags) = decode_handle(&req.handle, req.handle_len)?;
        if flags & HANDLE_BIOMETRY == 0 {
            return Err(EINVAL);
        }
        let mut owned = KVec::new();
        owned.extend_from_slice(blob, GFP_KERNEL)?;
        let digest = req.digest;
        let user = crate::sbio::UserId::new(crate::SBIO_PROBE_USER_ID).ok_or(EINVAL)?;
        let record = |r: Result<KVec<u8>>, s: &SepData| -> (i32, i32) {
            (r.err().map(|e| e.to_errno()).unwrap_or(0), s.last_key_status.load(Relaxed) as i32)
        };
        self.last_key_status.store(0, Relaxed);
        let (result, status) = match req.mode {
            SELFTEST_NO_CONTEXT => record(self.user_key_sign(&owned, &digest, None), self),
            SELFTEST_UNSATISFIED => {
                let ctx = OwnedContext::new(self, self.open_unlock_evaluation(user).ok_or(EIO)?);
                record(self.user_key_sign(&owned, &digest, Some(ctx.handle())), self)
            }
            SELFTEST_PASSCODE => {
                let ctx = OwnedContext::new(self, self.passcode_context()?);
                record(self.user_key_sign(&owned, &digest, Some(ctx.handle())), self)
            }
            SELFTEST_DELETED => {
                let ctx = self.open_unlock_evaluation(user).ok_or(EIO)?;
                self.close_unlock_evaluation(&ctx);
                record(self.user_key_sign(&owned, &digest, Some(&ctx)), self)
            }
            SELFTEST_TOUCH_THEN_DELETED => {
                let mut b = KVec::new();
                b.extend_from_slice(&owned, GFP_KERNEL)?;
                let probe = Probe {
                    blob: b,
                    digest,
                    #[cfg(CONFIG_APPLE_SEP_KEY_RESEARCH)]
                    sealed: KVec::new(),
                    after_delete: true,
                    #[cfg(CONFIG_APPLE_SEP_KEY_RESEARCH)]
                    then: None,
                };
                let job = self.key_touch_run(&owned, flags, TouchOp::Sign, &digest, &[], Some(probe), None)?;
                req.result2 = errno_of(&job.again);
                req.enclave_status2 = job.again_status;
                (errno_of(&job.result), job.status)
            }
            #[cfg(CONFIG_APPLE_SEP_KEY_RESEARCH)]
            SELFTEST_TOUCH_TWO_SIGNATURES | SELFTEST_TOUCH_SIGN_THEN_UNSEAL => {
                let two = req.mode == SELFTEST_TOUCH_TWO_SIGNATURES;
                let sealed = if two { KVec::new() } else { seal_to(&owned, &digest)? };
                let mut b = KVec::new();
                b.extend_from_slice(&owned, GFP_KERNEL)?;
                let mut sl = KVec::new();
                sl.extend_from_slice(&sealed, GFP_KERNEL)?;
                let probe = Probe {
                    blob: b,
                    digest,
                    sealed: sl,
                    after_delete: false,
                    then: Some(if two { TouchOp::Sign } else { TouchOp::Unseal }),
                };
                let job = self.key_touch_run(&owned, flags, TouchOp::Sign, &digest, &sealed, Some(probe), None)?;
                req.result2 = match &job.again {
                    Some(Ok(secret)) if !two && secret[..] == digest[..] => 0,
                    Some(Ok(_)) if !two => EIO.to_errno(),
                    other => errno_of(other),
                };
                req.enclave_status2 = job.again_status;
                (errno_of(&job.result), job.status)
            }
            // The context-reuse measurements are built only into research
            // kernels (CONFIG_APPLE_SEP_KEY_RESEARCH).
            #[cfg(not(CONFIG_APPLE_SEP_KEY_RESEARCH))]
            SELFTEST_TOUCH_TWO_SIGNATURES | SELFTEST_TOUCH_SIGN_THEN_UNSEAL => return Err(EOPNOTSUPP),
            SELFTEST_UNSEAL_UNSATISFIED => {
                let sealed = seal_to(&owned, &digest)?;
                let r = record(self.user_key_unseal(&owned, &sealed, None), self);
                self.last_key_status.store(0, Relaxed);
                let ctx = OwnedContext::new(self, self.open_unlock_evaluation(user).ok_or(EIO)?);
                (req.result2, req.enclave_status2) =
                    record(self.user_key_unseal(&owned, &sealed, Some(ctx.handle())), self);
                r
            }
            _ => return Err(EINVAL),
        };
        req.result = result;
        req.enclave_status = status;
        dev_info_ratelimited!(
            self.dev,
            "sep-key: self-test mode {}: {} (enclave status {})\n",
            req.mode,
            (if result == 0 { "SIGNED" } else { "refused" }),
            status
        );
        Ok(())
    }

    fn key_ioctl(&self, cmd: u32, arg: usize) -> Result<()> {
        if self.shutting_down.load(Relaxed) {
            return Err(ENODEV);
        }
        if cmd != IOC_INFO && cmd != IOC_CANCEL && self.key_context_failed.load(Relaxed) {
            return Err(EIO);
        }
        let user = UserPtr::from_addr(arg);
        match read_request(cmd, arg)? {
            KeyRequest::Info => write_user(user, &info_for(self.profile.key_store == crate::profile::KeyStore::Variant5)),
            KeyRequest::Create(mut req, policy) => {
                may_create_machine_key(crate::shim::capable_admin(), crate::shim::current_euid())?;
                let (acl, flags) = match policy {
                    Policy::None => (None, 0),
                    Policy::BiometryCurrentSet => (Some(self.key_bio_acl(false)?), HANDLE_BIOMETRY),
                    Policy::BiometryAny => (Some(self.key_bio_acl(true)?), HANDLE_BIOMETRY | HANDLE_BIOMETRY_ANY),
                };
                let (blob, point) = self.user_key_create(acl.as_ref().map(|a| &a.ed[..]))?;
                req.handle_len = encode_handle(&blob, flags, &mut req.handle)? as u32;
                req.public_key = point;
                write_user(user, &*req)
            }
            KeyRequest::Sign(mut req) => {
                let sig = {
                    let h = check_sign(&req)?;
                    let digest = req.digest;
                    if let Some(uid) = h.uid {
                        require_admin_or(uid)?;
                    }
                    if h.flags & HANDLE_USER_PLAIN != 0 {
                        crate::Secret(self.key_op_in(h.uid, TouchOp::Sign, h.blob, &digest, &[], None)?)
                    } else if h.flags & (HANDLE_BIOMETRY | HANDLE_USER_BAG) != 0 {
                        self.key_touch_run(h.blob, h.flags, TouchOp::Sign, &digest, &[], None, h.uid)?
                            .into_result()?
                    } else {
                        crate::Secret(self.user_key_sign(h.blob, &digest, None)?)
                    }
                };
                put_signature(&sig, &mut req)?;
                write_user(user, &*req)
            }
            KeyRequest::Public(mut req) => {
                req.public_key = SepData::user_key_public(check_public(&req)?)?;
                write_user(user, &*req)
            }
            KeyRequest::Cancel => {
                // Ends the touch-gated call that is waiting, whoever made it:
                // like pulling a security key, it can only make a request fail.
                if self.key_touch_active.load(Relaxed) {
                    self.key_touch_cancel.store(true, Relaxed);
                    dev_dbg!(self.dev, "sep-key: waiting operation cancelled\n");
                }
                Ok(())
            }
            KeyRequest::Seal(mut req) => {
                let sealed = {
                    let (blob, _, secret) = check_seal(&req)?;
                    seal_to(blob, secret)?
                };
                req.sealed.fill(0);
                req.sealed[..sealed.len()].copy_from_slice(&sealed);
                req.sealed_len = sealed.len() as u32;
                // `req` wipes the plaintext when it drops.
                write_user(user, &*req)
            }
            KeyRequest::Unseal(mut req) => {
                let secret = {
                    let (h, sealed) = check_unseal(&req)?;
                    if let Some(uid) = h.uid {
                        require_admin_or(uid)?;
                    }
                    if h.flags & HANDLE_USER_PLAIN != 0 {
                        crate::Secret(self.key_op_in(h.uid, TouchOp::Unseal, h.blob, &[0; DIGEST_LEN], sealed, None)?)
                    } else {
                        self.key_touch_run(h.blob, h.flags, TouchOp::Unseal, &[0; DIGEST_LEN], sealed, None, h.uid)?
                            .into_result()?
                    }
                };
                put_secret(&secret, &mut req)?;
                write_user(user, &*req)
                // `secret` and `req` wipe the plaintext when they drop.
            }
            KeyRequest::Selftest(mut req) => {
                self.key_selftest(&mut req)?;
                write_user(user, &*req)
            }
            KeyRequest::CreateUser(mut req) => {
                require_admin_or(req.uid)?;
                let plain = check_create_user(&req)?;
                let bag = bag_user(req.uid)?;
                let ed = if plain { None } else { Some(presence_acl()?) };
                // The enclave refuses while the bag is locked or absent.
                let blob = self.user_key_create_in_bag(bag, ed.as_deref())?;
                req.public_key = SepData::user_key_public(&blob)?;
                req.handle_len = encode_user_handle(&blob, req.uid, plain, &mut req.handle)? as u32;
                dev_info_ratelimited!(
                    self.dev,
                    "sep-key: {} key under uid {}'s bag\n",
                    (if plain { "bag-unlocked-only" } else { "user-presence" }),
                    req.uid
                );
                write_user(user, &*req)
            }
            KeyRequest::SignPassword(mut req) => {
                let sig = {
                    let h = check_sign(&req.sign)?;
                    let uid = check_password_handle(&h)?;
                    require_admin_or(uid)?;
                    let pw = check_password(&req.password)?;
                    let digest = req.sign.digest;
                    self.key_op_with_password(uid, h.blob, TouchOp::Sign, &digest, &[], pw)?
                };
                put_signature(&sig, &mut req.sign)?;
                // `req` wipes the password when it drops.
                write_user(user, &*req)
            }
            KeyRequest::UnsealPassword(mut req) => {
                let secret = {
                    let (h, sealed) = check_unseal(&req.seal)?;
                    let uid = check_password_handle(&h)?;
                    require_admin_or(uid)?;
                    let pw = check_password(&req.password)?;
                    self.key_op_with_password(uid, h.blob, TouchOp::Unseal, &[0; DIGEST_LEN], sealed, pw)?
                };
                put_secret(&secret, &mut req.seal)?;
                // `secret` and `req` wipe the plaintext and password when they drop.
                write_user(user, &*req)
            }
        }
    }
}

// /dev/sep-key has no session: any number of openers, no poll.

/// # Safety
/// `ctx` is the `SepData` pointer given to `register_key`, valid while the
/// registration lives (see `register_key`).
unsafe extern "C" fn key_open_trampoline(ctx: *mut c_void) -> c_int {
    // SAFETY: per the contract above.
    let this = unsafe { &*ctx.cast::<SepData>() };
    if this.shutting_down.load(Relaxed) {
        return ENODEV.to_errno();
    }
    0
}

/// # Safety
/// As for `key_open_trampoline`.
unsafe extern "C" fn key_release_trampoline(_ctx: *mut c_void) {}

/// # Safety
/// As for `key_open_trampoline`.
unsafe extern "C" fn key_ioctl_trampoline(ctx: *mut c_void, cmd: c_uint, arg: c_ulong) -> c_long {
    // SAFETY: per the contract above.
    let this = unsafe { &*ctx.cast::<SepData>() };
    match this.key_ioctl(cmd, arg) {
        Ok(()) => 0,
        Err(e) => e.to_errno() as c_long,
    }
}

/// # Safety
/// As for `key_open_trampoline`.
unsafe extern "C" fn key_ready_trampoline(_ctx: *mut c_void) -> c_int {
    0
}

/// KUnit hook for `sep_ioctl_test.c`: [`read_request`] on user memory the
/// test maps, so the copy-in of every request is exercised from real user
/// addresses (unaligned, faulting, truncated) with no hardware.
#[cfg(CONFIG_APPLE_SEP_KUNIT_TEST)]
#[no_mangle]
pub extern "C" fn sep_kunit_key_read_request(cmd: c_uint, arg: c_ulong) -> c_int {
    match read_request(cmd, arg as usize) {
        Ok(_) => 0,
        Err(e) => e.to_errno(),
    }
}

#[cfg(CONFIG_APPLE_SEP_KUNIT_TEST)]
#[::kernel::macros::kunit_tests(apple_sep_key_one_op)]
mod one_op_tests {
    use super::*;
    use core::cell::Cell;

    fn job() -> KeyTouch {
        KeyTouch::new(TouchOp::Sign, &[1, 2, 3], &[9; DIGEST_LEN], &[], KVec::new()).unwrap()
    }

    /// What `key_touch_sign` does with the job: take the grant, run it,
    /// settle. Returns how many times the enclave was asked.
    fn step(job: &mut KeyTouch, calls: &Cell<u32>, answer: Result<()>) {
        let Some(grant) = job.take_grant() else {
            return;
        };
        let end = grant.run(|op, blob, digest, _| {
            // Count only calls that carry the job's own operation and inputs.
            if op == TouchOp::Sign && blob == [1, 2, 3] && digest == &[9; DIGEST_LEN] {
                calls.set(calls.get() + 1);
            }
            answer.map(|()| {
                let mut v = KVec::new();
                v.push(0x30, GFP_KERNEL).unwrap();
                v
            })
        });
        job.settle(end, 0);
    }

    // The verify offers the matched context twice (after the match, after
    // the completed evaluation); one success leaves nothing to run again.
    #[test]
    fn a_match_runs_one_operation() {
        let calls = Cell::new(0);
        let mut j = job();
        step(&mut j, &calls, Ok(()));
        step(&mut j, &calls, Ok(()));
        step(&mut j, &calls, Ok(()));
        assert_eq!(calls.get(), 1);
        assert!(j.take_grant().is_none());
        assert!(matches!(j.into_result(), Ok(ref s) if s[..] == [0x30]));
    }

    // An outright refusal (the enclave did nothing) leaves one retry.
    #[test]
    fn a_refusal_allows_exactly_one_retry() {
        let calls = Cell::new(0);
        let mut j = job();
        step(&mut j, &calls, Err(EKEYREJECTED));
        assert!(matches!(j.result, Some(Err(e)) if e == EKEYREJECTED));
        step(&mut j, &calls, Ok(()));
        step(&mut j, &calls, Ok(()));
        assert_eq!(calls.get(), 2);
        assert!(j.into_result().is_ok());

        let calls = Cell::new(0);
        let mut j = job();
        step(&mut j, &calls, Err(EKEYREJECTED));
        step(&mut j, &calls, Err(EKEYREJECTED));
        step(&mut j, &calls, Ok(()));
        assert_eq!(calls.get(), 2);
        assert!(matches!(j.into_result(), Err(e) if e == EKEYREJECTED));
    }

    // Any other failure may have let the enclave act (a lost reply): the
    // grant is gone and nothing is retried.
    #[test]
    fn other_failures_consume_the_grant() {
        for e in [EIO, EINVAL, ENOMEM, EBADMSG, ETIMEDOUT] {
            let calls = Cell::new(0);
            let mut j = job();
            step(&mut j, &calls, Err(e));
            step(&mut j, &calls, Ok(()));
            assert_eq!(calls.get(), 1);
            assert!(matches!(j.into_result(), Err(x) if x == e));
        }
    }

    // A job that never ran (no match, cancel, timeout, signal) reports no
    // result; its grant dies with it.
    #[test]
    fn an_unrun_job_has_no_result() {
        let j = job();
        assert!(matches!(j.into_result(), Err(e) if e == EIO));
    }

    struct Deleter {
        deleted: Cell<u32>,
        last: Cell<[u8; crate::scrd::SCRD_ACM_HANDLE_LEN]>,
    }

    impl ContextDelete for Deleter {
        fn delete_context(&self, handle: &[u8; crate::scrd::SCRD_ACM_HANDLE_LEN]) -> Result<()> {
            self.deleted.set(self.deleted.get() + 1);
            self.last.set(*handle);
            Ok(())
        }
    }

    fn deleter() -> Deleter {
        Deleter { deleted: Cell::new(0), last: Cell::new([0; crate::scrd::SCRD_ACM_HANDLE_LEN]) }
    }

    /// An operation owning a context that leaves at step `fail_at` (0: runs
    /// to the end).
    fn owning(d: &Deleter, fail_at: u32) -> Result<u32> {
        let ctx = OwnedContext::new(d, [0xa5; crate::scrd::SCRD_ACM_HANDLE_LEN]);
        for step in 1..=4 {
            if step == fail_at {
                return Err([EINTR, ECANCELED, ETIMEDOUT, ENODEV][step as usize - 1]);
            }
            if ctx.handle() != &[0xa5; crate::scrd::SCRD_ACM_HANDLE_LEN] {
                return Err(EIO);
            }
        }
        Ok(0)
    }

    // The context is deleted exactly once on success and on each way out
    // (signal, cancel, timeout, device gone).
    #[test]
    fn an_owned_context_is_deleted_on_every_path() {
        for fail_at in 0..=4 {
            let d = deleter();
            let r = owning(&d, fail_at);
            assert_eq!(r.is_ok(), fail_at == 0);
            assert_eq!(d.deleted.get(), 1);
            assert_eq!(d.last.get(), [0xa5; crate::scrd::SCRD_ACM_HANDLE_LEN]);
        }
        let d = deleter();
        let maybe = Some(OwnedContext::new(&d, [1; crate::scrd::SCRD_ACM_HANDLE_LEN]));
        assert_eq!(d.deleted.get(), 0);
        drop(maybe);
        assert_eq!(d.deleted.get(), 1);
    }

    struct RefusingDeleter(Cell<u32>);

    impl ContextDelete for RefusingDeleter {
        fn delete_context(&self, _handle: &[u8; crate::scrd::SCRD_ACM_HANDLE_LEN]) -> Result<()> {
            self.0.set(self.0.get() + 1);
            Err(EIO)
        }
    }

    struct PendingOutput<'a>(&'a Cell<bool>);

    impl Drop for PendingOutput<'_> {
        fn drop(&mut self) {
            self.0.set(true);
        }
    }

    #[test]
    fn cleanup_failure_discards_success_without_retrying_disposal() {
        let d = RefusingDeleter(Cell::new(0));
        let discarded = Cell::new(false);
        let ctx = OwnedContext::new(&d, [0x51; crate::scrd::SCRD_ACM_HANDLE_LEN]);
        let result = ctx.finish(Ok(PendingOutput(&discarded)));
        assert!(matches!(result, Err(e) if e == EIO));
        assert!(discarded.get());
        assert_eq!(d.0.get(), 1);
    }

    #[test]
    fn confirmed_disposal_preserves_the_operation_result() {
        let d = deleter();
        let ctx = OwnedContext::new(&d, [0x52; crate::scrd::SCRD_ACM_HANDLE_LEN]);
        assert_eq!(ctx.finish(Ok(37)), Ok(37));
        assert_eq!(d.deleted.get(), 1);
        let ctx = OwnedContext::new(&d, [0x53; crate::scrd::SCRD_ACM_HANDLE_LEN]);
        assert_eq!(ctx.finish::<()>(Err(ECANCELED)), Err(ECANCELED));
        assert_eq!(d.deleted.get(), 2);
    }

    // Production kernels do not build the reuse measurements.
    #[test]
    fn reuse_measurements_are_research_only() {
        let built = cfg!(CONFIG_APPLE_SEP_KEY_RESEARCH);
        assert_eq!(selftest_mode_built(SELFTEST_TOUCH_TWO_SIGNATURES), built);
        assert_eq!(selftest_mode_built(SELFTEST_TOUCH_SIGN_THEN_UNSEAL), built);
        for mode in [SELFTEST_NO_CONTEXT, SELFTEST_DELETED, SELFTEST_TOUCH_THEN_DELETED, SELFTEST_UNSEAL_UNSATISFIED] {
            assert!(selftest_mode_built(mode));
        }
    }
}

#[cfg(CONFIG_APPLE_SEP_KUNIT_TEST)]
#[::kernel::macros::kunit_tests(apple_sep_key_fuzz)]
mod fuzz_tests {
    use super::*;
    use kernel::transmute::AsBytes;

    /// A small deterministic generator, so a failure replays.
    struct Rng(u64);

    impl Rng {
        fn next(&mut self) -> u64 {
            let mut x = self.0;
            x ^= x << 13;
            x ^= x >> 7;
            x ^= x << 17;
            self.0 = x;
            x
        }
        fn below(&mut self, n: u64) -> u64 {
            self.next() % n
        }
        fn fill(&mut self, b: &mut [u8]) {
            for v in b.iter_mut() {
                *v = self.next() as u8;
            }
        }
    }

    fn zeroed<T: kernel::transmute::FromBytes>() -> KBox<T> {
        let b = KBox::<T>::new_uninit(GFP_KERNEL | kernel::alloc::flags::__GFP_ZERO).unwrap();
        // SAFETY: zero-filled, and any bit pattern is a valid `T`.
        unsafe { b.assume_init() }
    }

    /// A ref-key record carrying a public point, as the enclave makes them.
    fn record() -> KVec<u8> {
        let mut point = [0x11u8; crate::refkey_seal::POINT_LEN];
        point[0] = 0x04;
        let inner = crate::der::encode_refkey_set(&[(b"pub".as_slice(), RV::Octets(&point))]).unwrap();
        crate::der::encode_refkey_set(&[(b"rk".as_slice(), RV::Der(&inner))]).unwrap()
    }

    fn handle(flags: u8) -> ([u8; HANDLE_MAX], u32) {
        let mut h = [0u8; HANDLE_MAX];
        let n = encode_handle(&record(), flags, &mut h).unwrap();
        (h, n as u32)
    }

    fn sealed_blob() -> KVec<u8> {
        let mut v = KVec::new();
        v.push(0x04, GFP_KERNEL).unwrap();
        v.extend_from_slice(&[7; crate::refkey_seal::POINT_LEN - 1 + 16 + 8], GFP_KERNEL).unwrap();
        v
    }

    /// Whatever a check accepts must be a well-formed record and handle.
    fn accepted_handle_is_sound(blob: &[u8], flags: u8, len: u32) -> bool {
        // A version-2 (user-bag) handle carries the uid in a longer header.
        let (valid, header) = if flags & HANDLE_USER_BAG != 0 {
            (flags == HANDLE_USER_BAG || flags == HANDLE_USER_BAG | HANDLE_USER_PLAIN, HANDLE_HEADER_USER)
        } else {
            (handle_flags_valid(flags), HANDLE_HEADER)
        };
        valid
            && blob.len() + header == len as usize
            && !blob.is_empty()
            && blob.len() <= BLOB_MAX
            && SepData::user_key_public(blob).is_ok()
    }

    #[test]
    fn valid_requests_pass_the_checks() {
        assert!(SepData::user_key_public(&record()).is_ok());
        let mut s = zeroed::<Sign>();
        (s.handle, s.handle_len) = handle(0);
        assert!(check_sign(&s).is_ok());
        let mut p = zeroed::<Public>();
        (p.handle, p.handle_len) = handle(HANDLE_BIOMETRY);
        assert!(check_public(&p).is_ok());
        let mut q = zeroed::<Seal>();
        (q.handle, q.handle_len) = handle(HANDLE_BIOMETRY | HANDLE_BIOMETRY_ANY);
        q.secret_len = SECRET_MAX as u32;
        assert!(check_seal(&q).is_ok());
        let blob = sealed_blob();
        q.sealed[..blob.len()].copy_from_slice(&blob);
        q.sealed_len = blob.len() as u32;
        assert!(check_unseal(&q).is_ok());
        let c = zeroed::<Create>();
        assert_eq!(check_create(&c), Ok(Policy::None));
    }

    // Every length field at its edges and beyond: refused with EINVAL,
    // never a panic or an out-of-range slice.
    #[test]
    fn lengths_at_and_past_their_bounds() {
        let (h, n) = handle(HANDLE_BIOMETRY);
        let mut s = zeroed::<Sign>();
        s.handle = h;
        for len in [0, 1, HANDLE_HEADER as u32, n - 1, n + 1, HANDLE_MAX as u32, HANDLE_MAX as u32 + 1, u32::MAX / 2, u32::MAX] {
            s.handle_len = len;
            assert_eq!(check_sign(&s).err(), Some(EINVAL));
        }
        let mut q = zeroed::<Seal>();
        (q.handle, q.handle_len) = handle(HANDLE_BIOMETRY);
        for len in [0, SECRET_MAX as u32 + 1, SEALED_MAX as u32, u32::MAX] {
            q.secret_len = len;
            assert_eq!(check_seal(&q).err(), Some(EINVAL));
        }
        let blob = sealed_blob();
        q.sealed[..blob.len()].copy_from_slice(&blob);
        for len in [0, 1, crate::refkey_seal::POINT_LEN as u32, (crate::refkey_seal::POINT_LEN + 16) as u32, SEALED_MAX as u32 + 1, u32::MAX] {
            q.sealed_len = len;
            assert_eq!(check_unseal(&q).err(), Some(EINVAL));
        }
        // The blob's own length field disagreeing with the request's.
        let (mut h, n) = handle(0);
        h[8] ^= 1;
        s.handle = h;
        s.handle_len = n;
        assert_eq!(check_sign(&s).err(), Some(EINVAL));
    }

    // Every value of the enum-like fields: only the defined ones pass.
    #[test]
    fn out_of_range_flags_and_policies() {
        let mut c = zeroed::<Create>();
        for bit in 0..32 {
            c.flags = 1 << bit;
            let want = match c.flags {
                CREATE_BIOMETRY_CURRENT_SET => Ok(Policy::BiometryCurrentSet),
                CREATE_BIOMETRY_ANY => Ok(Policy::BiometryAny),
                CREATE_USER_PRESENCE | CREATE_DEVICE_PASSCODE => Err(EOPNOTSUPP),
                _ => Err(EINVAL),
            };
            assert_eq!(check_create(&c), want);
        }
        c.flags = CREATE_BIOMETRY_CURRENT_SET | CREATE_BIOMETRY_ANY;
        assert_eq!(check_create(&c), Err(EINVAL));
        c.flags = 0;
        c.reserved = [0, 0, 1];
        assert_eq!(check_create(&c), Err(EINVAL));
        let mut s = zeroed::<Sign>();
        for hf in 0..=255u8 {
            let (mut h, n) = handle(0);
            h[5] = hf;
            s.handle = h;
            s.handle_len = n;
            assert_eq!(check_sign(&s).is_ok(), matches!(hf, 0 | 1 | 3));
        }
        let (h, n) = handle(0);
        s.handle = h;
        s.handle_len = n;
        s.flags = 1;
        assert_eq!(check_sign(&s).err(), Some(EINVAL));
        // SEAL/UNSEAL refuse a key without a biometric list.
        let mut q = zeroed::<Seal>();
        (q.handle, q.handle_len) = handle(0);
        q.secret_len = 1;
        assert_eq!(check_seal(&q).err(), Some(EINVAL));
    }

    // Garbage and truncated DER where the record goes: refused, never a
    // panic. Bytes are mutated from a valid record as well as drawn whole.
    #[test]
    fn garbage_records_are_refused_cleanly() {
        let mut rng = Rng(0x5eb0_5eb0_d00d_f00d);
        let good = record();
        let mut s = zeroed::<Sign>();
        let mut accepted = 0u32;
        for i in 0..4000 {
            let mut blob = [0u8; 256];
            let len = 1 + rng.below(255) as usize;
            if i % 2 == 0 {
                rng.fill(&mut blob[..len]);
            } else {
                let len = core::cmp::min(len, good.len());
                blob[..len].copy_from_slice(&good[..len]);
                for _ in 0..1 + rng.below(4) {
                    let at = rng.below(len as u64) as usize;
                    blob[at] = rng.next() as u8;
                }
            }
            let flags = [0u8, 1, 3][rng.below(3) as usize];
            match encode_handle(&blob[..len], flags, &mut s.handle) {
                Ok(n) => s.handle_len = n as u32,
                Err(_) => continue,
            }
            if let Ok(h) = check_sign(&s) {
                accepted += 1;
                assert!(accepted_handle_is_sound(h.blob, h.flags, s.handle_len));
            }
            let _ = crate::refkey_seal::sealed_ephemeral(&blob[..len]);
        }
        // Mutation keeps some records valid; most are refused.
        assert!(accepted < 4000);
    }

    // Whole requests drawn at random: every check returns, and anything
    // accepted is sound.
    #[test]
    fn random_requests_never_misbehave() {
        let mut rng = Rng(0x0123_4567_89ab_cdef);
        for _ in 0..500 {
            let mut c = zeroed::<Create>();
            rng.fill((*c).as_bytes_mut());
            let _ = check_create(&c);
            let mut s = zeroed::<Sign>();
            rng.fill((*s).as_bytes_mut());
            if rng.below(2) == 0 {
                s.handle_len = rng.below(HANDLE_MAX as u64 + 64) as u32;
            }
            if let Ok(h) = check_sign(&s) {
                assert!(accepted_handle_is_sound(h.blob, h.flags, s.handle_len));
            }
            let mut p = zeroed::<Public>();
            rng.fill((*p).as_bytes_mut());
            let _ = check_public(&p);
            let mut q = zeroed::<Seal>();
            rng.fill((*q).as_bytes_mut());
            let _ = check_seal(&q);
            let _ = check_unseal(&q);
            // A real record with random everything else.
            (q.handle, q.handle_len) = handle(if rng.below(2) == 0 { 1 } else { 3 });
            q.secret_len = rng.below(SECRET_MAX as u64 + 4) as u32;
            q.sealed_len = rng.below(SEALED_MAX as u64 + 4) as u32;
            if let Ok((_, _, secret)) = check_seal(&q) {
                assert!((1..=SECRET_MAX).contains(&secret.len()));
            }
            if let Ok((_, sealed)) = check_unseal(&q) {
                assert!(sealed.len() > crate::refkey_seal::POINT_LEN + 16 && sealed[0] == 0x04);
            }
        }
    }

    // The plaintext a SEAL or UNSEAL request carries is wiped when the
    // request goes away (what `Drop for Seal` does).
    #[test]
    fn a_seal_request_wipes_its_secret() {
        let mut q = zeroed::<Seal>();
        q.secret = [0x77; SECRET_MAX];
        q.wipe_secret();
        assert!(q.secret.iter().all(|&b| b == 0));
    }
}
