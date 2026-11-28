// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Per-device sysfs registration. The caller retains the snapshot Arc until
//! unregister has drained active readers and freed the C attribute wrapper.

use crate::{g17::telemetry::Readout, memacct::Ledger, stats::StatsSnapshot};
use core::ffi::{c_int, c_void};

type ReadStats = unsafe extern "C" fn(*const c_void, *mut Readout);
type SuspendStats = unsafe extern "C" fn(*const c_void, bool);
type ReadFrequency = unsafe extern "C" fn(*const c_void, u32, *mut u64) -> bool;

extern "C" {
    fn asahi_sysfs_register(
        dev: *mut c_void,
        snapshot: *const c_void,
        export_enabled: c_int,
        memory: *const Ledger,
        handle: *mut *mut c_void,
        read: Option<ReadStats>,
        suspend: Option<SuspendStats>,
        frequency: Option<ReadFrequency>,
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
    if ret < 0 {
        Err(kernel::error::Error::from_errno(ret))
    } else {
        Ok(())
    }
}

/// Register only after initialization succeeds, keeping the snapshot Arc alive.
///
/// Also registers `gpu_memory`, which shows the static GPU memory ledger.
pub(crate) fn register(
    dev: *mut kernel::bindings::device,
    snapshot: *const StatsSnapshot,
    export_enabled: bool,
) -> kernel::error::Result<*mut c_void> {
    register_with_callbacks(
        dev,
        snapshot,
        export_enabled,
        None,
        None,
        Some(read_frequency),
    )
}

/// The caller retains the qualified snapshot Arc until unregister joins both
/// sysfs readers and the PM notifier. The callbacks never power the GPU up.
pub(crate) fn register_qualified(
    dev: *mut kernel::bindings::device,
    snapshot: *const StatsSnapshot,
    export_enabled: bool,
) -> kernel::error::Result<*mut c_void> {
    register_with_callbacks(
        dev,
        snapshot,
        export_enabled,
        Some(read_qualified),
        Some(suspend_qualified),
        None,
    )
}

/// Look up the exact pstate printed by C in the retained firmware table.
///
/// # Safety
///
/// Registration retains `snapshot`; `out` is writable for one u64.
unsafe extern "C" fn read_frequency(snapshot: *const c_void, pstate: u32, out: *mut u64) -> bool {
    // SAFETY: The attribute is removed and readers joined before the Arc is dropped.
    let snapshot = unsafe { &*snapshot.cast::<StatsSnapshot>() };
    let Some(frequency) = snapshot.requested_frequency(pstate) else {
        return false;
    };
    // SAFETY: C provides writable, aligned storage for this call.
    unsafe { out.write(frequency) };
    true
}

/// Copy the locked qualified snapshot to the C formatter's local output.
///
/// # Safety
///
/// `snapshot` must name a live StatsSnapshot retained by the registration.
/// `out` must be aligned and writable for one Readout for this call.
unsafe extern "C" fn read_qualified(snapshot: *const c_void, out: *mut Readout) {
    // SAFETY: C calls only while registration retains the snapshot Arc, and
    // supplies a writable, aligned output for this call.
    let snapshot = unsafe { &*snapshot.cast::<StatsSnapshot>() };
    let view = snapshot
        .qualified_reader
        .as_ref()
        .map_or_else(Readout::default, |reader| reader.read());
    // SAFETY: As above; the C output is initialized in one locked read.
    unsafe { out.write(view) };
}

/// End the telemetry epoch at either side of system sleep.
///
/// # Safety
///
/// `snapshot` must name a live StatsSnapshot. The caller must join this
/// callback before releasing the registration's owning Arc.
unsafe extern "C" fn suspend_qualified(snapshot: *const c_void, suspended: bool) {
    // SAFETY: The notifier is unregistered and joined before the Arc is dropped.
    if let Some(reader) = unsafe { &*snapshot.cast::<StatsSnapshot>() }
        .qualified_reader
        .as_ref()
    {
        reader.suspend(suspended);
    }
}

fn register_with_callbacks(
    dev: *mut kernel::bindings::device,
    snapshot: *const StatsSnapshot,
    export_enabled: bool,
    read: Option<ReadStats>,
    suspend: Option<SuspendStats>,
    frequency: Option<ReadFrequency>,
) -> kernel::error::Result<*mut c_void> {
    let mut handle = core::ptr::null_mut();
    // SAFETY: The bound driver keeps the device and optional snapshot alive until
    // unregister drains readers. `handle` is writable for the duration of the call.
    let ret = unsafe {
        asahi_sysfs_register(
            dev.cast(),
            snapshot.cast(),
            c_int::from(export_enabled),
            &crate::memacct::LEDGER,
            &mut handle,
            read,
            suspend,
            frequency,
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
