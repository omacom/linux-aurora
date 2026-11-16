// SPDX-License-Identifier: GPL-2.0 OR MIT

//! DRM File objects.
//!
//! C header: [`include/drm/drm_file.h`](srctree/include/drm/drm_file.h)

use crate::{
    bindings,
    drm,
    prelude::*,
    types::Opaque, //
};
use core::marker::PhantomData;

/// Memory a client reports in its fdinfo, in bytes, as `drm-<key>-system` lines.
///
/// See `Documentation/gpu/drm-usage-stats.rst`. `None` leaves an optional key out.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct MemoryStats {
    /// Memory shared with other clients (`drm-shared-*`, also part of `drm-total-*`).
    pub shared: u64,
    /// Memory private to the client (part of `drm-total-*`).
    pub private: u64,
    /// Memory currently resident (`drm-resident-*`).
    pub resident: Option<u64>,
    /// Resident memory userspace marked purgeable (`drm-purgeable-*`).
    pub purgeable: Option<u64>,
    /// Memory in use by active submissions (`drm-active-*`).
    pub active: Option<u64>,
}

/// Trait that must be implemented by DRM drivers to represent a DRM File (a client instance).
pub trait DriverFile {
    /// The parent `Driver` implementation for this `DriverFile`.
    type Driver: drm::Driver;

    /// Open a new file (called when a client opens the DRM device).
    fn open(device: &drm::Device<Self::Driver, crate::drm::Registered>) -> Result<Pin<KBox<Self>>>;

    /// Get raw drm_file pointer
    fn as_raw(&self) -> *mut bindings::drm_file;

    /// Memory statistics for this client's fdinfo. The default, `None`, prints no memory keys.
    fn memory_stats(&self) -> Option<MemoryStats> {
        None
    }
}

/// An open DRM File.
///
/// # Invariants
///
/// `self.0` is a valid instance of a `struct drm_file`.
#[repr(transparent)]
pub struct File<T: DriverFile>(Opaque<bindings::drm_file>, PhantomData<T>);

impl<T: DriverFile> File<T> {
    #[doc(hidden)]
    /// Not intended to be called externally, except via declare_drm_ioctls!()
    ///
    /// # Safety
    ///
    /// `raw_file` must be a valid pointer to an open `struct drm_file`, opened through `T::open`.
    pub unsafe fn from_raw<'a>(ptr: *mut bindings::drm_file) -> &'a File<T> {
        // SAFETY: `raw_file` is valid by the safety requirements of this function.
        unsafe { &*ptr.cast() }
    }

    pub(super) fn as_raw(&self) -> *mut bindings::drm_file {
        self.0.get()
    }

    /// Whether this file holds the device's DRM master (the compositor or display server).
    pub fn is_current_master(&self) -> bool {
        // SAFETY: By the type invariants of `Self`, `self.as_raw()` is a valid open `drm_file`.
        unsafe { bindings::drm_is_current_master(self.as_raw()) }
    }

    /// Tag recent feeder work for temporary capacity hints during syncobj waits.
    /// User scheduling attributes and CPU affinity remain unchanged.
    pub fn set_syncobj_wait_hint(&self, util_min: u32, duration_ms: u32) {
        // SAFETY: this File keeps the opened drm_file alive; the C helper
        // bounds both arguments and serializes the per-file hint internally.
        unsafe { bindings::drm_syncobj_set_wait_hint(self.as_raw(), util_min, duration_ms) };
    }

    fn driver_priv(&self) -> *mut T {
        // SAFETY: By the type invariants of `Self`, `self.as_raw()` is always valid.
        unsafe { (*self.as_raw()).driver_priv }.cast()
    }

    /// Return a pinned reference to the driver file structure.
    pub fn inner(&self) -> Pin<&T> {
        // SAFETY: By the type invariant the pointer `self.as_raw()` points to a valid and opened
        // `struct drm_file`, hence `driver_priv` has been properly initialized by `open_callback`.
        unsafe { Pin::new_unchecked(&*(self.driver_priv())) }
    }

    /// The open callback of a `struct drm_file`.
    pub(crate) extern "C" fn open_callback(
        raw_dev: *mut bindings::drm_device,
        raw_file: *mut bindings::drm_file,
    ) -> core::ffi::c_int {
        // SAFETY: A callback from `struct drm_driver::open` guarantees that
        // - `raw_dev` is valid pointer to a `struct drm_device`,
        // - the corresponding `struct drm_device` has been registered.
        let drm: &drm::Device<T::Driver> = unsafe { drm::Device::from_raw(raw_dev) };
        let drm = unsafe { drm.assume_ctx::<crate::drm::Registered>() };

        // SAFETY: `raw_file` is a valid pointer to a `struct drm_file`.
        let file = unsafe { File::<T>::from_raw(raw_file) };

        let inner = match T::open(drm) {
            Err(e) => {
                return e.to_errno();
            }
            Ok(i) => i,
        };

        // SAFETY: This pointer is treated as pinned, and the Drop guarantee is upheld in
        // `postclose_callback()`.
        let driver_priv = KBox::into_raw(unsafe { Pin::into_inner_unchecked(inner) });

        // SAFETY: By the type invariants of `Self`, `self.as_raw()` is always valid.
        unsafe { (*file.as_raw()).driver_priv = driver_priv.cast() };

        0
    }

    /// The `show_fdinfo` callback of a `struct drm_driver`.
    pub(crate) extern "C" fn show_fdinfo_callback(
        printer: *mut bindings::drm_printer,
        raw_file: *mut bindings::drm_file,
    ) {
        // SAFETY: `drm_show_fdinfo()` passes an open `struct drm_file` of this driver, whose
        // `driver_priv` was set by `open_callback()`. The seq_file read holds a reference to the
        // file, so `postclose_callback()` cannot run concurrently.
        let file = unsafe { File::<T>::from_raw(raw_file) };
        let Some(stats) = file.inner().memory_stats() else {
            return;
        };

        let mut supported = 0;
        let mut raw = bindings::drm_memory_stats {
            shared: stats.shared,
            private: stats.private,
            resident: 0,
            purgeable: 0,
            active: 0,
        };
        if let Some(resident) = stats.resident {
            raw.resident = resident;
            supported |= bindings::drm_gem_object_status_DRM_GEM_OBJECT_RESIDENT;
        }
        if let Some(purgeable) = stats.purgeable {
            raw.purgeable = purgeable;
            supported |= bindings::drm_gem_object_status_DRM_GEM_OBJECT_PURGEABLE;
        }
        if let Some(active) = stats.active {
            raw.active = active;
            supported |= bindings::drm_gem_object_status_DRM_GEM_OBJECT_ACTIVE;
        }

        // SAFETY: `printer` is the printer `drm_show_fdinfo()` passes, valid for this call; `raw`
        // and the region name outlive the call.
        unsafe {
            bindings::drm_print_memory_stats(printer, &raw, supported, c"system".as_ptr())
        };
    }

    /// The postclose callback of a `struct drm_file`.
    pub(crate) extern "C" fn postclose_callback(
        _raw_dev: *mut bindings::drm_device,
        raw_file: *mut bindings::drm_file,
    ) {
        // SAFETY: This reference won't escape this function
        let file = unsafe { File::<T>::from_raw(raw_file) };

        // SAFETY: `file.driver_priv` has been created in `open_callback` through `KBox::into_raw`.
        let _ = unsafe { KBox::from_raw(file.driver_priv()) };
    }
}

impl<T: DriverFile> super::private::Sealed for File<T> {}
