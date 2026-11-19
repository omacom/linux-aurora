// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Support for Apple RTKit coprocessors.
//!
//! C header: [`include/linux/soc/apple/rtkit.h`](../../../../include/linux/gpio/driver.h)

use crate::{
    bindings,
    device,
    error::{
        from_err_ptr,
        from_result,
        to_result, //
    },
    io::{
        register,
        Io,
        IoSysMap,
        Region, //
    },
    prelude::*,
    str::CStrExt,
    types::{
        ForeignOwnable,
        ScopeGuard, //
    }, //
};

use core::marker::PhantomData;
use core::ptr;
use macros::vtable;

register! {
    /// ASC CPU control register
    pub ASC_CPU_CONTROL(u32) @ 0x44 {
        /// CPU run bit.
        4:4 cpu_run;
    }
}

/// Trait to represent allocatable buffers for the RTKit core.
///
/// Users must implement this trait for their own representation of those allocations.
pub trait Buffer {
    /// Returns the IOVA (virtual address) of the buffer from RTKit's point of view, or an error if
    /// unavailable.
    fn iova(&self) -> Result<usize>;

    /// Returns a mutable byte slice of the buffer contents, or an
    /// error if unavailable.
    fn buf(&mut self) -> Result<IoSysMap<'_, Region>>;
}

/// Callback operations for an RTKit client.
#[vtable]
pub trait Operations {
    /// Arbitrary user context type.
    type Data: ForeignOwnable + Send + Sync;

    /// Type representing an allocated buffer for RTKit.
    type Buffer: Buffer;

    /// Whether the coprocessor places its shared buffers itself.
    ///
    /// If set, buffer requests that carry an IOVA are served by [`Operations::shmem_map`] and
    /// acknowledged by sending the request back unchanged, and the crashlog endpoint is started
    /// before the endpoint map is acknowledged.
    const COPROC_PLACES_BUFFERS: bool = false;

    /// Called when RTKit crashes.
    fn crashed(_data: <Self::Data as ForeignOwnable>::Borrowed<'_>, _crashlog: Option<&[u8]>) {}

    /// Called when a message was received on a non-system endpoint. Called in non-IRQ context.
    fn recv_message(
        _data: <Self::Data as ForeignOwnable>::Borrowed<'_>,
        _endpoint: u8,
        _message: u64,
    ) {
    }

    /// Called in IRQ context when a message was received on a non-system endpoint.
    ///
    /// Must return `true` if the message is handled, or `false` to process it in
    /// the handling thread.
    fn recv_message_early(
        _data: <Self::Data as ForeignOwnable>::Borrowed<'_>,
        _endpoint: u8,
        _message: u64,
    ) -> bool {
        false
    }

    /// Allocate a buffer for use by RTKit.
    fn shmem_alloc(
        _data: <Self::Data as ForeignOwnable>::Borrowed<'_>,
        _size: usize,
    ) -> Result<Self::Buffer> {
        Err(EINVAL)
    }

    /// Map an existing buffer used by RTKit at a device-specified virtual address.
    fn shmem_map(
        _data: <Self::Data as ForeignOwnable>::Borrowed<'_>,
        _iova: usize,
        _size: usize,
    ) -> Result<Self::Buffer> {
        Err(EINVAL)
    }
}

/// Represents `struct apple_rtkit *`.
///
/// # Invariants
///
/// The rtk pointer is valid.
/// The data pointer is a valid pointer from T::Data::into_foreign().
pub struct RtKit<T: Operations> {
    rtk: *mut bindings::apple_rtkit,
    data: *mut core::ffi::c_void,
    retain_shared_buffers: bool,
    _p: PhantomData<T>,
}

// The wrapper owns opaque pointers. Neither the C allocation nor the foreign
// callback context points back to this Rust wrapper, so moving it is safe.
impl<T: Operations> Unpin for RtKit<T> {}

unsafe extern "C" fn crashed_callback<T: Operations>(
    cookie: *mut core::ffi::c_void,
    crashlog: *const core::ffi::c_void,
    crashlog_size: usize,
) {
    let crashlog = if !crashlog.is_null() && crashlog_size > 0 {
        // SAFETY: The crashlog is either missing or a byte buffer of the specified size
        Some(unsafe { core::slice::from_raw_parts(crashlog as *const u8, crashlog_size) })
    } else {
        None
    };
    // SAFETY: cookie is always a T::Data in this API
    T::crashed(unsafe { T::Data::borrow(cookie.cast()) }, crashlog);
}

unsafe extern "C" fn recv_message_callback<T: Operations>(
    cookie: *mut core::ffi::c_void,
    endpoint: u8,
    message: u64,
) {
    // SAFETY: cookie is always a T::Data in this API
    T::recv_message(unsafe { T::Data::borrow(cookie.cast()) }, endpoint, message);
}

unsafe extern "C" fn recv_message_early_callback<T: Operations>(
    cookie: *mut core::ffi::c_void,
    endpoint: u8,
    message: u64,
) -> bool {
    // SAFETY: cookie is always a T::Data in this API
    T::recv_message_early(unsafe { T::Data::borrow(cookie.cast()) }, endpoint, message)
}

unsafe extern "C" fn shmem_setup_callback<T: Operations>(
    cookie: *mut core::ffi::c_void,
    bfr: *mut bindings::apple_rtkit_shmem,
) -> core::ffi::c_int {
    // SAFETY: `bfr` is a valid buffer
    let bfr_mut = unsafe { &mut *bfr };

    from_result(|| {
        let mut buf = if bfr_mut.iova != 0 {
            bfr_mut.is_mapped = true;
            T::shmem_map(
                // SAFETY: `cookie` came from a previous call to `into_foreign`.
                unsafe { T::Data::borrow(cookie.cast()) },
                bfr_mut.iova as usize,
                bfr_mut.size,
            )?
        } else {
            bfr_mut.is_mapped = false;
            // SAFETY: `cookie` came from a previous call to `into_foreign`.
            T::shmem_alloc(unsafe { T::Data::borrow(cookie.cast()) }, bfr_mut.size)?
        };

        let iova = buf.iova()?;
        let IoSysMap::Sys(sysmem) = buf.buf()? else {
            return Err(EINVAL);
        };

        if sysmem.size() < bfr_mut.size {
            return Err(ENOMEM);
        }

        bfr_mut.iova = iova as u64;
        bfr_mut.buffer = sysmem.as_ptr() as *mut _;

        // Now box the returned buffer type and stash it in the private pointer of the
        // `apple_rtkit_shmem` struct for safekeeping.
        let boxed = KBox::new(buf, GFP_KERNEL)?;
        bfr_mut.private = KBox::into_raw(boxed) as *mut _;
        Ok(0)
    })
}

unsafe extern "C" fn shmem_destroy_callback<T: Operations>(
    _cookie: *mut core::ffi::c_void,
    bfr: *mut bindings::apple_rtkit_shmem,
) {
    // SAFETY: `bfr` is a valid buffer
    let bfr_mut = unsafe { &mut *bfr };
    if !bfr_mut.private.is_null() {
        // SAFETY: Per shmem_setup_callback, this has to be a pointer to a Buffer if it is set.
        unsafe {
            core::mem::drop(KBox::from_raw(bfr_mut.private as *mut T::Buffer));
        }
        bfr_mut.private = core::ptr::null_mut();
    }
}

impl<T: Operations> RtKit<T> {
    const VTABLE: bindings::apple_rtkit_ops = bindings::apple_rtkit_ops {
        flags: if T::COPROC_PLACES_BUFFERS {
            bindings::APPLE_RTKIT_COPROC_PLACES_BUFFERS
        } else {
            0
        },
        crashed: Some(crashed_callback::<T>),
        recv_message: Some(recv_message_callback::<T>),
        recv_message_early: Some(recv_message_early_callback::<T>),
        shmem_setup: if T::HAS_SHMEM_ALLOC || T::HAS_SHMEM_MAP {
            Some(shmem_setup_callback::<T>)
        } else {
            None
        },
        shmem_destroy: if T::HAS_SHMEM_ALLOC || T::HAS_SHMEM_MAP {
            Some(shmem_destroy_callback::<T>)
        } else {
            None
        },
    };

    /// Creates a new RTKit client for a given device and optional mailbox name or index.
    pub fn new(
        dev: &device::Device,
        mbox_name: Option<&'static CStr>,
        mbox_idx: usize,
        data: T::Data,
    ) -> Result<Self> {
        let ptr: *mut crate::ffi::c_void = data.into_foreign().cast();
        let guard = ScopeGuard::new(|| {
            // SAFETY: `ptr` came from a previous call to `into_foreign`.
            unsafe { T::Data::from_foreign(ptr.cast()) };
        });
        // SAFETY: `dev` is valid by its type invarants and otherwise his just
        //          calls the C init function.
        let rtk = unsafe {
            from_err_ptr(bindings::apple_rtkit_init(
                dev.as_raw(),
                ptr,
                match mbox_name {
                    Some(s) => s.as_char_ptr(),
                    None => ptr::null(),
                },
                mbox_idx.try_into()?,
                &Self::VTABLE,
            ))
        }?;

        guard.dismiss();
        // INVARIANT: `rtk` and `data` are valid here.
        Ok(Self {
            rtk,
            data: ptr,
            retain_shared_buffers: false,
            _p: PhantomData,
        })
    }

    /// Requests acknowledged RTKit shutdown using the C core's bounded waits.
    ///
    /// The caller must not hold a lock needed by its receive callbacks: the
    /// shutdown path also drains the RTKit workqueue.
    pub fn shutdown(self: Pin<&mut Self>) -> Result {
        // SAFETY: `rtk` is valid and the mutable borrow excludes other callers.
        to_result(unsafe { bindings::apple_rtkit_shutdown(self.rtk) })
    }

    /// Retains firmware-visible buffers when a shutdown cannot be confirmed.
    ///
    /// Drop will still close callbacks and drain the private RTKit queue, but
    /// shared mappings/allocation contexts remain retained until reboot. The
    /// owner must retain its own DMA buffers too. This does not permit reprobe.
    pub fn retain_shared_buffers_on_drop(&mut self) {
        self.retain_shared_buffers = true;
    }

    /// Boots (wakes up) the RTKit coprocessor.
    pub fn wake(self: Pin<&mut Self>) -> Result {
        // SAFETY: `rtk` is valid per the type invariant.
        to_result(unsafe { bindings::apple_rtkit_wake(self.rtk) })
    }

    /// Waits for the RTKit coprocessor to finish booting.
    pub fn boot(self: Pin<&mut Self>) -> Result {
        // SAFETY: `rtk` is valid per the type invariant.
        to_result(unsafe { bindings::apple_rtkit_boot(self.rtk) })
    }

    /// Starts a non-system endpoint.
    pub fn start_endpoint(self: Pin<&mut Self>, endpoint: u8) -> Result {
        // SAFETY: `rtk` is valid per the type invariant.
        to_result(unsafe { bindings::apple_rtkit_start_ep(self.rtk, endpoint) })
    }

    /// Sends a message to a given endpoint.
    pub fn send_message(self: Pin<&mut Self>, endpoint: u8, message: u64) -> Result {
        // SAFETY: `rtk` is valid per the type invariant.
        to_result(unsafe {
            bindings::apple_rtkit_send_message(self.rtk, endpoint, message, ptr::null_mut(), false)
        })
    }

    /// Returns whether the coprocessor is running (not crashed, IOP and AP power states on).
    pub fn is_running(&self) -> bool {
        // SAFETY: `rtk` is valid per the type invariant. The call only reads state.
        unsafe { bindings::apple_rtkit_is_running(self.rtk) }
    }

    /// Returns whether the coprocessor has reported a crash.
    pub fn is_crashed(&self) -> bool {
        // SAFETY: `rtk` is valid per the type invariant. The call only reads state.
        unsafe { bindings::apple_rtkit_is_crashed(self.rtk) }
    }

    /// Checks if an endpoint is present
    pub fn has_endpoint(self: Pin<&mut Self>, endpoint: u8) -> bool {
        unsafe { bindings::apple_rtkit_has_endpoint(self.rtk, endpoint) }
    }

    /// Sets the minimum utilization (0..=1024) the coprocessor's receive thread runs with, or
    /// restores the scheduler's default with `None`.
    ///
    /// The receive thread applies it to itself when it next runs. Returns `EOPNOTSUPP` for a
    /// mailbox without a threaded receive handler.
    pub fn set_rx_util_min(&self, util_min: Option<u32>) -> Result {
        // INVARIANT: `rtk` is valid per the type invariant.
        RxUtilMin { rtk: self.rtk }.set(util_min)
    }

    /// Returns a handle that sets the receive thread's minimum utilization (see
    /// [`RtKit::set_rx_util_min`]) without borrowing this [`RtKit`], from any context.
    ///
    /// # Safety
    ///
    /// The caller must not use the handle after this [`RtKit`] is dropped.
    pub unsafe fn rx_util_min_handle(&self) -> RxUtilMin {
        // INVARIANT: The caller keeps this `RtKit`, and so `rtk`, alive while using the handle.
        RxUtilMin { rtk: self.rtk }
    }
}

/// Sets the minimum utilization of an RTKit coprocessor's receive thread.
///
/// The request is a single store that the receive thread applies to itself when it next runs,
/// so it may be made from any context.
///
/// # Invariants
///
/// `rtk` is valid while the handle is used.
#[derive(Clone, Copy)]
pub struct RxUtilMin {
    rtk: *mut bindings::apple_rtkit,
}

// SAFETY: The request is one store into the mailbox, valid from any thread.
unsafe impl Send for RxUtilMin {}
// SAFETY: As above; the handle has no state of its own.
unsafe impl Sync for RxUtilMin {}

impl RxUtilMin {
    /// See [`RtKit::set_rx_util_min`].
    pub fn set(&self, util_min: Option<u32>) -> Result {
        let util_min = match util_min {
            Some(v) => i32::try_from(v).map_err(|_| EINVAL)?,
            None => -1,
        };
        // SAFETY: `rtk` is valid per the type invariant; the call only stores the request.
        to_result(unsafe { bindings::apple_rtkit_set_rx_util_min(self.rtk, util_min) })
    }
}

// SAFETY: `RtKit` operations require a mutable reference
unsafe impl<T: Operations> Sync for RtKit<T> {}

// SAFETY: `RtKit` operations require a mutable reference
unsafe impl<T: Operations> Send for RtKit<T> {}

impl<T: Operations> Drop for RtKit<T> {
    fn drop(&mut self) {
        // SAFETY: The pointer is valid by the type invariant.
        unsafe {
            if self.retain_shared_buffers {
                bindings::apple_rtkit_free_retaining_buffers(self.rtk);
            } else {
                bindings::apple_rtkit_free(self.rtk);
            }
        }

        // Free context data.
        //
        // SAFETY: This matches the call to `into_foreign` from `new` in the success case.
        unsafe { T::Data::from_foreign(self.data.cast()) };
    }
}
