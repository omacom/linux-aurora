// SPDX-License-Identifier: GPL-2.0-only OR MIT
//! Blocking application-key ABI; each open owns its requests and cancellation.

use crate::{key_request::Request, keys, SepData};
use kernel::prelude::*;
use kernel::sync::{atomic::Relaxed, Arc, ArcBorrow, Mutex};
use kernel::types::ForeignOwnable;
use kernel::uaccess::{UserPtr, UserSlice};

#[repr(C)]
struct Info {
    version: u32,
    capabilities: u32,
    reserved: [u32; 2],
}
#[repr(C)]
struct Create {
    flags: u32,
    blob_len: u32,
    blob_ptr: u64,
    public_key: [u8; 65],
    reserved: [u8; 7],
}
#[repr(C)]
struct Sign {
    request_id: u64,
    flags: u32,
    timeout_ms: u32,
    blob_len: u32,
    sig_len: u32,
    blob_ptr: u64,
    digest: [u8; 32],
    signature: [u8; 72],
}
#[repr(C)]
struct Decrypt {
    request_id: u64,
    flags: u32,
    timeout_ms: u32,
    blob_len: u32,
    sealed_len: u32,
    blob_ptr: u64,
    sealed_ptr: u64,
    plain_len: u32,
    reserved: u32,
    plain_ptr: u64,
}
#[repr(C)]
struct Cancel {
    request_id: u64,
}

static_assert!(core::mem::size_of::<Info>() == 16);
static_assert!(core::mem::size_of::<Create>() == 88);
static_assert!(core::mem::size_of::<Sign>() == 136);
static_assert!(core::mem::size_of::<Decrypt>() == 56);
static_assert!(core::mem::size_of::<Cancel>() == 8);
// SAFETY: all repr(C) layouts contain initialized integers/bytes without padding.
unsafe impl kernel::transmute::AsBytes for Info {}
// SAFETY: initialized integers/bytes, no padding.
unsafe impl kernel::transmute::AsBytes for Create {}
// SAFETY: every bit pattern is valid.
unsafe impl kernel::transmute::FromBytes for Create {}
// SAFETY: initialized integers/bytes, no padding.
unsafe impl kernel::transmute::AsBytes for Sign {}
// SAFETY: every bit pattern is valid.
unsafe impl kernel::transmute::FromBytes for Sign {}
// SAFETY: initialized integers, no padding.
unsafe impl kernel::transmute::AsBytes for Decrypt {}
// SAFETY: every bit pattern is valid.
unsafe impl kernel::transmute::FromBytes for Decrypt {}
// SAFETY: every bit pattern is valid.
unsafe impl kernel::transmute::FromBytes for Cancel {}

const IOC_INFO: u32 = kernel::ioctl::_IOR::<Info>(0xb2, 0);
const IOC_CREATE: u32 = kernel::ioctl::_IOWR::<Create>(0xb2, 1);
const IOC_SIGN: u32 = kernel::ioctl::_IOWR::<Sign>(0xb2, 2);
const IOC_DECRYPT: u32 = kernel::ioctl::_IOWR::<Decrypt>(0xb2, 3);
const IOC_CANCEL: u32 = kernel::ioctl::_IOW::<Cancel>(0xb2, 4);

enum Operation {
    Idle,
    Create,
    Private(Arc<Request>),
}

struct State {
    last_id: u64,
    cancelled_through: u64,
    operation: Operation,
}

#[pin_data]
struct File {
    sep: Arc<SepData>,
    #[pin]
    state: Mutex<State>,
}

/// The ioctl owns the file's active slot until it has finished copying out.
struct Active<'a>(&'a File);

impl Drop for Active<'_> {
    fn drop(&mut self) { self.0.state.lock().operation = Operation::Idle; }
}

enum Destination {
    Sign(Sign),
    Decrypt(Decrypt),
}

impl File {
    fn begin(&self, request: Option<Arc<Request>>) -> Result<Active<'_>> {
        let mut state = self.state.lock();
        if !matches!(state.operation, Operation::Idle) { return Err(EBUSY); }
        state.operation = if let Some(request) = request {
            if request.id == 0 { return Err(EINVAL); }
            if request.id <= state.cancelled_through { return Err(ECANCELED); }
            if request.id <= state.last_id { return Err(EINVAL); }
            state.last_id = request.id;
            Operation::Private(request)
        } else { Operation::Create };
        Ok(Active(self))
    }

    fn cancel(&self, id: u64) -> Result {
        if id == 0 { return Err(EINVAL); }
        let request = {
            let mut state = self.state.lock();
            match &state.operation {
                Operation::Private(request) if request.id == id => request.clone(),
                _ if id > state.last_id => {
                    // Cover cancellation while EXECUTE is still copying input,
                    // before an active request can be observed by this thread.
                    state.cancelled_through = state.cancelled_through.max(id);
                    return Ok(());
                }
                _ => return Err(ENOENT),
            }
        };
        request.cancel(ECANCELED)
    }

    fn check_api(&self) -> Result {
        if !self.sep.biometric_keys_supported() { return Err(EOPNOTSUPP); }
        if !self.sep.xart_writable() { return Err(EROFS); }
        Ok(())
    }

    fn ioctl(&self, cmd: u32, arg: usize) -> Result {
        if self.sep.shutting_down.load(Relaxed) { return Err(ENODEV); }
        let user = UserPtr::from_addr(arg);
        if cmd == IOC_INFO {
            let out = Info { version: 1,
                capabilities: if self.sep.biometric_keys_supported() { 3 } else { 0 },
                reserved: [0; 2] };
            return UserSlice::new(user, core::mem::size_of_val(&out)).writer().write(&out);
        }
        if cmd == IOC_CANCEL {
            let request: Cancel = UserSlice::new(user, core::mem::size_of::<Cancel>()).reader().read()?;
            return self.cancel(request.request_id);
        }
        if !matches!(cmd, IOC_CREATE | IOC_SIGN | IOC_DECRYPT) { return Err(ENOTTY); }
        self.check_api()?;
        if cmd == IOC_CREATE {
            let request: Create = UserSlice::new(user, core::mem::size_of::<Create>()).reader().read()?;
            if request.flags > 1 || request.reserved != [0; 7] || request.blob_len as usize != keys::MAX_BLOB {
                return Err(EINVAL);
            }
            let _active = self.begin(None)?;
            let operation = Request::new(0, keys::KeyRequest { blob: KVec::new(),
                input: keys::Input::Create(request.flags),
                deadline_ns: crate::shim::monotonic_ns().saturating_add(120_000_000_000) })?;
            SepData::queue_key(self.sep.clone(), operation.clone())?;
            let keys::Output::Created(blob, public_key) = operation.wait()? else { return Err(EIO); };
            UserSlice::new(UserPtr::from_addr(request.blob_ptr as usize), keys::MAX_BLOB).writer().write_slice(&blob)?;
            let out = Create { flags: 0, blob_len: blob.len() as u32, blob_ptr: request.blob_ptr,
                public_key, reserved: [0; 7] };
            return UserSlice::new(user, core::mem::size_of_val(&out)).writer().write(&out);
        }

        let (id, blob_len, blob_ptr, timeout_ms, input, destination) = match cmd {
            IOC_SIGN => {
                let request: Sign = UserSlice::new(user, core::mem::size_of::<Sign>()).reader().read()?;
                if request.flags != 0 || request.sig_len != 0 || request.signature != [0; 72] { return Err(EINVAL); }
                (request.request_id, request.blob_len, request.blob_ptr, request.timeout_ms,
                    keys::Input::Sign(request.digest), Destination::Sign(request))
            }
            IOC_DECRYPT => {
                let request: Decrypt = UserSlice::new(user, core::mem::size_of::<Decrypt>()).reader().read()?;
                if request.flags != 0 || request.reserved != 0 || request.plain_len as usize > keys::MAX_BLOB
                    || !(81..=keys::MAX_BLOB).contains(&(request.sealed_len as usize))
                {
                    return Err(EINVAL);
                }
                let mut sealed = KVec::new();
                sealed.resize(request.sealed_len as usize, 0, GFP_KERNEL)?;
                UserSlice::new(UserPtr::from_addr(request.sealed_ptr as usize), sealed.len()).reader().read_slice(&mut sealed)?;
                if sealed[0] != 4 { return Err(EINVAL); }
                (request.request_id, request.blob_len, request.blob_ptr, request.timeout_ms,
                    keys::Input::Decrypt(sealed), Destination::Decrypt(request))
            }
            _ => return Err(ENOTTY),
        };
        if blob_len == 0 || blob_len as usize > keys::MAX_BLOB || !(1..=120_000).contains(&timeout_ms) {
            return Err(EINVAL);
        }
        let mut blob = KVec::new();
        blob.resize(blob_len as usize, 0, GFP_KERNEL)?;
        UserSlice::new(UserPtr::from_addr(blob_ptr as usize), blob.len()).reader().read_slice(&mut blob)?;
        if SepData::refkey_pub(&blob).is_none() { return Err(EINVAL); }
        let deadline_ns = crate::shim::monotonic_ns().saturating_add(u64::from(timeout_ms) * 1_000_000);
        let request = Request::new(id, keys::KeyRequest { blob, input, deadline_ns })?;
        let _active = self.begin(Some(request.clone()))?;
        SepData::queue_key(self.sep.clone(), request.clone())?;
        let result = request.wait()?;
        match (result, destination) {
            (keys::Output::Signature(signature), Destination::Sign(mut out)) => {
                out.sig_len = signature.len;
                out.signature = signature.bytes;
                UserSlice::new(user, core::mem::size_of_val(&out)).writer().write(&out)
            }
            (keys::Output::Plaintext(plain), Destination::Decrypt(mut out)) => {
                let capacity = out.plain_len as usize;
                out.plain_len = plain.len() as u32;
                if capacity < plain.len() {
                    UserSlice::new(user, core::mem::size_of_val(&out)).writer().write(&out)?;
                    return Err(ENOSPC);
                }
                UserSlice::new(UserPtr::from_addr(out.plain_ptr as usize), capacity).writer().write_slice(&plain)?;
                UserSlice::new(user, core::mem::size_of_val(&out)).writer().write(&out)
            }
            _ => Err(EIO),
        }
    }
}

extern "C" {
    fn sep_key_register(ctx: *mut c_void,
        open: unsafe extern "C" fn(*mut c_void, *mut *mut c_void) -> c_int,
        release: unsafe extern "C" fn(*mut c_void),
        ioctl: unsafe extern "C" fn(*mut c_void, c_uint, c_ulong) -> c_long) -> *mut c_void;
    fn sep_key_unregister(device: *mut c_void);
}

pub(crate) struct Registration(*mut c_void);
// SAFETY: registration has no thread affinity; the C shim serializes open/detach.
unsafe impl Send for Registration {}

impl Registration {
    /// # Safety
    /// The driver's original Arc must keep sep alive until this is dropped.
    pub(crate) unsafe fn new(sep: &SepData) -> Result<Self> {
        // SAFETY: the caller supplies the registered-window guarantee above.
        let device = unsafe { sep_key_register(core::ptr::from_ref(sep).cast_mut().cast(), open, release, ioctl) };
        if device.is_null() { return Err(ENODEV); }
        Ok(Self(device))
    }
}

impl Drop for Registration {
    fn drop(&mut self) {
        // SAFETY: one live registration, never used after this call.
        unsafe { sep_key_unregister(self.0) };
    }
}

// SAFETY: the C shim holds its registration lock throughout open. ctx is the
// original SepData Arc's pointer, and out points to its per-file storage.
unsafe extern "C" fn open(ctx: *mut c_void, out: *mut *mut c_void) -> c_int {
    let sep = unsafe { ArcBorrow::<SepData>::from_raw(ctx.cast()) };
    if sep.shutting_down.load(Relaxed) { return ENODEV.to_errno(); }
    let file = Arc::pin_init(try_pin_init!(File {
        sep: sep.into(),
        state <- kernel::new_mutex!(State { last_id: 0, cancelled_through: 0, operation: Operation::Idle }),
    }), GFP_KERNEL);
    match file {
        Ok(file) => {
            // SAFETY: out is writable; ownership moves to the matching release.
            unsafe { out.write(file.into_foreign()) };
            0
        }
        Err(error) => error.to_errno(),
    }
}

// SAFETY: C passes the owned file pointer once, after all file callbacks stop.
unsafe extern "C" fn release(ctx: *mut c_void) {
    drop(unsafe { <Arc<File> as ForeignOwnable>::from_foreign(ctx) });
}

// SAFETY: every successful open stores an Arc<File>, retained until release;
// the file reference held by ioctl keeps it live even during concurrent close.
unsafe extern "C" fn ioctl(ctx: *mut c_void, cmd: c_uint, arg: c_ulong) -> c_long {
    let file = unsafe { <Arc<File> as ForeignOwnable>::borrow(ctx) };
    match file.ioctl(cmd, arg) {
        Ok(()) => 0,
        Err(error) => error.to_errno() as c_long,
    }
}

impl SepData {
    pub(crate) fn register_keys(&self) -> Result {
        let mut device = self.key_dev.lock();
        if device.is_none() {
            // SAFETY: the driver's Arc outlives this registration; remove drops
            // it before tearing down the driver. Opens acquire their own Arc.
            *device = Some(unsafe { Registration::new(self)? });
        }
        Ok(())
    }

    pub(crate) fn queue_key(this: Arc<Self>, request: Arc<Request>) -> Result {
        // remove sets shutting_down, then takes this same lock to revoke the
        // active request before cancel_work_sync. Keep enqueue in that window.
        let mut capture = this.capture.lock();
        if this.shutting_down.load(Relaxed) { return Err(ENODEV); }
        if this.suspending.load(Relaxed) { return Err(EBUSY); }
        capture.start_key(request.clone())?;
        if kernel::workqueue::system_dfl()
            .enqueue::<Arc<SepData>, { crate::KEY_WORK_ID }>(this.clone()).is_err()
        {
            request.complete(Err(EIO));
            capture.finish_key(&request);
            return Err(EIO);
        }
        Ok(())
    }

    pub(crate) fn cancel_key(&self, reason: Error) {
        let request = self.capture.lock().key();
        if let Some(request) = request { let _ = request.cancel(reason); }
    }

    pub(crate) fn capture_wanted(&self) -> bool {
        if self.shutting_down.load(Relaxed) || self.suspending.load(Relaxed) { return false; }
        let request = self.capture.lock().key();
        if let Some(request) = request { return request.is_live(); }
        crate::bio::capture_is_live(&self.bio_session.lock(), self.capture_epoch.load(Relaxed))
    }
}
