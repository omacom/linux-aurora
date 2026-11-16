// SPDX-License-Identifier: GPL-2.0-only OR MIT
//! Atomic preparation of a queue completion scope and its exported signals.

use kernel::{bindings, dma_fence::{Fence, RawDmaFence}, prelude::*};

// Wire values also documented in include/uapi/drm/asahi_drm.h. Keeping these
// local permits building this module against an older kernel's Rust bindings.
pub(crate) const SYNC_CUMULATIVE: u32 = 2;
pub(crate) const FEATURE_CUMULATIVE_SIGNALS: u64 = 1 << 29;

unsafe extern "C" {
    fn asahi_g17_signals_new(syncs: *const SignalSpec, count: usize, anchor: usize,
        error: *mut i32) -> *mut core::ffi::c_void;
    fn asahi_g17_signals_free(bundle: *mut core::ffi::c_void);
    fn asahi_g17_signals_publish(bundle: *mut core::ffi::c_void,
        current: *mut bindings::dma_fence) -> i32;
    fn asahi_g17_signals_fail(bundle: *mut core::ffi::c_void,
        failure: *mut bindings::dma_fence);
}

#[repr(C)]
struct SignalSpec {
    syncobj: *mut core::ffi::c_void,
    point: u64,
    timeline: u32,
}

/// Owns syncobj references and preallocated chain nodes; never owns user pointers.
pub(crate) struct SignalBundle(*mut core::ffi::c_void);

impl SignalBundle {
    /// C retains references before the borrowed sync items can be dropped.
    pub(crate) fn new(syncs: &[crate::file::SyncItem], anchor: usize) -> Result<Self>
    {
        let mut specs = KVec::with_capacity(syncs.len(), GFP_KERNEL)?;
        for sync in syncs {
            specs.push(SignalSpec {
                syncobj: sync.syncobj.as_raw().cast(), point: sync.timeline_value,
                timeline: u32::from(sync.timeline),
            }, GFP_KERNEL)?;
        }
        let mut error = 0;
        // SAFETY: All entries borrow live sync objects; C copies the entries and
        // acquires its own references before returning.
        let raw = unsafe { asahi_g17_signals_new(
            specs.as_ptr(), specs.len(), anchor, &mut error) };
        if raw.is_null() { return Err(Error::from_errno(error)); }
        Ok(Self(raw))
    }

    pub(crate) fn publish(&mut self, current: &Fence) -> Result {
        // SAFETY: The bundle and current fence are owned for this call; C takes
        // its own references on publication and reports failure before any store.
        let error = unsafe { asahi_g17_signals_publish(self.0, current.raw()) };
        if error == 0 { Ok(()) } else { Err(Error::from_errno(error)) }
    }

    pub(crate) fn fail(&mut self, failure: &Fence) {
        // SAFETY: Live owned arguments; this only consumes uninstalled chain nodes.
        unsafe { asahi_g17_signals_fail(self.0, failure.raw()) };
    }
}

impl Drop for SignalBundle {
    fn drop(&mut self) {
        // SAFETY: Exactly one owner; published syncobjs retain their own fences.
        unsafe { asahi_g17_signals_free(self.0) };
    }
}
