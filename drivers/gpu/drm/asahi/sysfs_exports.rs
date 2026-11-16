// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Per-device sysfs registration. The caller retains the snapshot Arc until
//! unregister has drained active readers and freed the C attribute wrapper.

use crate::{memacct::Ledger, stats::StatsSnapshot};
use core::ffi::{c_int, c_void};

extern "C" {
    fn asahi_sysfs_register(
        dev: *mut c_void,
        snapshot: *const StatsSnapshot,
        export_enabled: c_int,
        memory: *const Ledger,
        handle: *mut *mut c_void,
    ) -> c_int;
    fn asahi_sysfs_unregister(dev: *mut c_void, handle: *mut c_void);
}

extern "C" {
    fn asahi_sysfs_register_idle(dev: *mut c_void, word: *mut u32) -> c_int;
}

/// Export the idle power-off delay as the read-write `idle_off_delay_ms` (G17).
pub(crate) fn register_idle(
    dev: *mut kernel::bindings::device,
    word: *mut u32,
) -> kernel::error::Result {
    // SAFETY: `word` is a static atomic that outlives the attribute; the C side reads and writes
    // it with READ_ONCE/WRITE_ONCE.
    let ret = unsafe { asahi_sysfs_register_idle(dev.cast(), word) };
    if ret < 0 { Err(kernel::error::Error::from_errno(ret)) } else { Ok(()) }
}

/// Register only after initialization succeeds, keeping the snapshot Arc alive.
///
/// Also registers `gpu_memory`, which shows the static GPU memory ledger.
pub(crate) fn register(
    dev: *mut kernel::bindings::device,
    snapshot: *const StatsSnapshot,
    export_enabled: bool,
) -> kernel::error::Result<*mut c_void> {
    let mut handle = core::ptr::null_mut();
    // SAFETY: The bound driver keeps the device and optional snapshot alive until
    // unregister drains readers. `handle` is writable for the duration of the call.
    let ret = unsafe {
        asahi_sysfs_register(
            dev.cast(),
            snapshot,
            c_int::from(export_enabled),
            &crate::memacct::LEDGER,
            &mut handle,
        )
    };
    if ret < 0 {
        Err(kernel::error::Error::from_errno(ret))
    } else {
        Ok(handle)
    }
}

/// Drain readers before the caller drops its snapshot Arc.
pub(crate) fn unregister(dev: *mut kernel::bindings::device, handle: *mut c_void) {
    // SAFETY: `handle` was returned by register for this device. The driver still
    // owns the device and optional snapshot while the C helper drains readers.
    unsafe { asahi_sysfs_unregister(dev.cast(), handle) };
}
