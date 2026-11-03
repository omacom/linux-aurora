// SPDX-License-Identifier: GPL-2.0-only OR MIT
// Copyright 2026 Dj

//! Rust side of `store_shim.c`: the backing-store file.

use kernel::prelude::*;

extern "C" {
    fn sep_store_open(path: *const c_char) -> *mut c_void;
    fn sep_store_open_host(path: *const c_char, created: *mut bool) -> *mut c_void;
    fn sep_store_open_trunc(path: *const c_char) -> *mut c_void;
    fn sep_store_open_ro(path: *const c_char) -> *mut c_void;
    fn sep_store_open_block(path: *const c_char, writable: c_int) -> *mut c_void;
    fn sep_store_close(handle: *mut c_void);
    fn sep_store_size(handle: *mut c_void) -> i64;
    fn sep_store_read(handle: *mut c_void, off: i64, buf: *mut c_void, len: usize) -> c_long;
    fn sep_store_write(handle: *mut c_void, off: i64, buf: *const c_void, len: usize) -> c_long;
    fn sep_store_sync(handle: *mut c_void) -> c_int;
    fn sep_store_sync_parent(handle: *mut c_void) -> c_int;
    fn sep_random_bytes(buf: *mut c_void, len: usize) -> c_int;
}

pub(crate) fn random_bytes(buf: &mut [u8]) -> Result<()> {
    // SAFETY: `buf.as_mut_ptr()` is valid for writes of `buf.len()` bytes for
    // the duration of the call, and `sep_random_bytes` writes exactly that many
    // bytes (via `get_random_bytes`) and retains no reference to the buffer.
    let ret = unsafe { sep_random_bytes(buf.as_mut_ptr().cast(), buf.len()) };
    kernel::error::to_result(ret)
}

/// Backing-store file handle.
///
/// # Invariants
/// `handle` is a live `struct file *` from one of the store-open shims.
pub(crate) struct StoreFile {
    handle: *mut c_void,
}

// SAFETY: a `struct file *` has no thread affinity; every access goes through
// the C shim, and the driver keeps this behind a mutex.
unsafe impl Send for StoreFile {}

fn result_of(ret: c_long) -> Result<usize> {
    if ret < 0 {
        Err(Error::from_errno(ret as c_int))
    } else {
        Ok(ret as usize)
    }
}

impl StoreFile {
    pub(crate) fn open(path: &CStr) -> Result<StoreFile> {
        // SAFETY: `path` is NUL-terminated; the shim returns a live file or
        // an error pointer, as required by `from_err_ptr`.
        let handle = unsafe { kernel::error::from_err_ptr(sep_store_open(path.as_char_ptr()))? };
        Ok(StoreFile { handle })
    }

    /// Opens the file like [`StoreFile::open`], and also reports whether this
    /// call created it.
    pub(crate) fn open_host(path: &CStr) -> Result<(StoreFile, bool)> {
        let mut created = false;
        // SAFETY: `path` is NUL-terminated and `created` is valid for writes
        // for the duration of the call. The shim returns a live file or an
        // error pointer.
        let handle = unsafe {
            kernel::error::from_err_ptr(sep_store_open_host(path.as_char_ptr(), &mut created))?
        };
        Ok((StoreFile { handle }, created))
    }

    pub(crate) fn open_trunc(path: &CStr) -> Result<StoreFile> {
        // SAFETY: `path` is NUL-terminated; the shim returns a live file or
        // an error pointer, as required by `from_err_ptr`.
        let handle =
            unsafe { kernel::error::from_err_ptr(sep_store_open_trunc(path.as_char_ptr()))? };
        Ok(StoreFile { handle })
    }

    pub(crate) fn open_readonly(path: &CStr) -> Result<StoreFile> {
        // SAFETY: `path` is NUL-terminated; the shim returns a live file or
        // an error pointer, as required by `from_err_ptr`.
        let handle = unsafe { kernel::error::from_err_ptr(sep_store_open_ro(path.as_char_ptr()))? };
        Ok(StoreFile { handle })
    }

    pub(crate) fn open_block(path: &CStr, writable: bool) -> Result<StoreFile> {
        // SAFETY: `path` is NUL-terminated; the shim opens read-only unless
        // explicitly asked to write, and rejects writes to a read-only device.
        let handle = unsafe { sep_store_open_block(path.as_char_ptr(), c_int::from(writable)) };
        if handle.is_null() {
            return Err(ENODEV);
        }
        Ok(StoreFile { handle })
    }

    pub(crate) fn size(&self) -> Result<u64> {
        // SAFETY: `handle` is live per the type invariant.
        let n = unsafe { sep_store_size(self.handle) };
        if n < 0 {
            return Err(EIO);
        }
        Ok(n as u64)
    }

    pub(crate) fn read_exact(&self, off: u64, buf: &mut [u8]) -> Result<()> {
        let mut done = 0usize;
        while done < buf.len() {
            // SAFETY: `handle` is live; the pointer and length describe the
            // remaining tail of `buf`, which we own mutably.
            let n = result_of(unsafe {
                sep_store_read(
                    self.handle,
                    (off + done as u64) as i64,
                    buf[done..].as_mut_ptr().cast::<c_void>(),
                    buf.len() - done,
                )
            })?;
            if n == 0 {
                return Err(EIO);
            }
            done += n;
        }
        Ok(())
    }

    pub(crate) fn read_block_exact(&self, off: u64, buf: &mut [u8]) -> Result<()> {
        // SAFETY: `handle` remains live and `buf` is writable for its length.
        let n = result_of(unsafe {
            sep_store_read(
                self.handle,
                off as i64,
                buf.as_mut_ptr().cast::<c_void>(),
                buf.len(),
            )
        })?;
        if n != buf.len() {
            return Err(EIO);
        }
        Ok(())
    }

    pub(crate) fn write_all(&self, off: u64, buf: &[u8]) -> Result<()> {
        let mut done = 0usize;
        while done < buf.len() {
            // SAFETY: as above, for a shared borrow.
            let n = result_of(unsafe {
                sep_store_write(
                    self.handle,
                    (off + done as u64) as i64,
                    buf[done..].as_ptr().cast::<c_void>(),
                    buf.len() - done,
                )
            })?;
            if n == 0 {
                return Err(EIO);
            }
            done += n;
        }
        Ok(())
    }

    pub(crate) fn write_block_exact(&self, off: u64, buf: &[u8]) -> Result<()> {
        // SAFETY: `handle` remains live and `buf` is readable for its length.
        let n = result_of(unsafe {
            sep_store_write(
                self.handle,
                off as i64,
                buf.as_ptr().cast::<c_void>(),
                buf.len(),
            )
        })?;
        if n != buf.len() {
            return Err(EIO);
        }
        Ok(())
    }

    pub(crate) fn sync(&self) -> Result<()> {
        // SAFETY: `handle` is live per the type invariant.
        kernel::error::to_result(unsafe { sep_store_sync(self.handle) })
    }

    /// Flushes the directory entry of this file, which [`StoreFile::sync`]
    /// alone does not make durable for a newly created file.
    pub(crate) fn sync_parent(&self) -> Result<()> {
        // SAFETY: `handle` is live per the type invariant.
        kernel::error::to_result(unsafe { sep_store_sync_parent(self.handle) })
    }
}

impl Drop for StoreFile {
    fn drop(&mut self) {
        // SAFETY: `handle` is live per the type invariant and is not used again.
        unsafe { sep_store_close(self.handle) };
    }
}

extern "C" {
    fn sep_bio_register(
        name: *const c_char,
        mode: u16,
        ctx: *mut c_void,
        f_open: Option<unsafe extern "C" fn(*mut c_void) -> c_int>,
        f_release: Option<unsafe extern "C" fn(*mut c_void)>,
        f_ioctl: Option<unsafe extern "C" fn(*mut c_void, c_uint, c_ulong) -> c_long>,
        f_ready: Option<unsafe extern "C" fn(*mut c_void) -> c_int>,
    ) -> *mut c_void;
    fn sep_bio_unregister(dev: *mut c_void);
    fn sep_bio_wake(dev: *mut c_void);
    fn sep_bio_capable_admin() -> c_int;
    fn sep_bio_monotonic_ns() -> u64;
    fn sep_bio_boottime_ns() -> u64;
}

pub(crate) fn capable_admin() -> bool {
    // SAFETY: no preconditions; reads the current task's credentials.
    unsafe { sep_bio_capable_admin() != 0 }
}

pub(crate) fn monotonic_ns() -> u64 {
    // SAFETY: no preconditions.
    unsafe { sep_bio_monotonic_ns() }
}

/// `CLOCK_BOOTTIME`; differs from monotonic by suspend time — how "tokens don't
/// survive a suspend" is detected.
pub(crate) fn boottime_ns() -> u64 {
    // SAFETY: no preconditions.
    unsafe { sep_bio_boottime_ns() }
}

/// The registered character device.
///
/// # Invariants
/// `dev` is a live registration from `sep_bio_register`.
pub(crate) struct BioChardev {
    dev: *mut c_void,
}

// SAFETY: the registration has no thread affinity; every access goes through
// the C shim, which locks internally.
unsafe impl Send for BioChardev {}

impl BioChardev {
    /// Registers `/dev/<name>`.
    ///
    /// # Safety
    /// `ctx` must stay valid and safe to pass to the four callbacks until this
    /// is dropped; the callbacks may run on any task at any time.
    pub(crate) unsafe fn register(
        name: &'static CStr,
        mode: u16,
        ctx: *mut c_void,
        f_open: unsafe extern "C" fn(*mut c_void) -> c_int,
        f_release: unsafe extern "C" fn(*mut c_void),
        f_ioctl: unsafe extern "C" fn(*mut c_void, c_uint, c_ulong) -> c_long,
        f_ready: unsafe extern "C" fn(*mut c_void) -> c_int,
    ) -> Result<BioChardev> {
        // SAFETY: `name` is a static NUL-terminated string that outlives the
        // registration, and the caller guarantees the same of `ctx`.
        let dev = unsafe {
            sep_bio_register(
                name.as_char_ptr(),
                mode,
                ctx,
                Some(f_open),
                Some(f_release),
                Some(f_ioctl),
                Some(f_ready),
            )
        };
        if dev.is_null() {
            return Err(ENODEV);
        }
        Ok(BioChardev { dev })
    }

    pub(crate) fn wake(&self) {
        // SAFETY: `dev` is live per the type invariant.
        unsafe { sep_bio_wake(self.dev) };
    }
}

impl Drop for BioChardev {
    fn drop(&mut self) {
        // SAFETY: `dev` is live per the type invariant and not used again.
        // Deregistration waits for open files, so no callback runs once this returns.
        unsafe { sep_bio_unregister(self.dev) };
    }
}
