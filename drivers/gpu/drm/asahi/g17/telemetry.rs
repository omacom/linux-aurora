// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Per-device sysfs lifetime. Readers retain only the atomic statistics allocation.

use super::statistics::Statistics;
use core::ffi::{c_int, c_void};
use kernel::{device::Device, prelude::*, sync::Arc};

/// Private callback contract shared with `asahi_telemetry.c`, not a userspace ABI.
#[repr(C)]
struct Export {
    valid: u32,
    power_known: u32,
    powered: u32,
    utilization: [u32; 4],
    clock_hz: u32,
    samples: u64,
    received_ns: u64,
    timestamp_ticks: u64,
    power_timestamp_ticks: u64,
    powered_ticks: u64,
    unpowered_ticks: u64,
    invalid: u64,
    age_ns: u64,
}

const _: () = {
    assert!(core::mem::size_of::<Export>() == 96);
    assert!(core::mem::offset_of!(Export, samples) == 32);
};

unsafe extern "C" {
    fn asahi_telemetry_register(
        dev: *mut c_void,
        data: *const c_void,
        snapshot: unsafe extern "C" fn(*const c_void, *mut c_void) -> c_int,
        handle: *mut *mut c_void,
    ) -> c_int;
    fn asahi_telemetry_unregister(dev: *mut c_void, handle: *mut c_void);
}

unsafe extern "C" fn snapshot(data: *const c_void, out: *mut c_void) -> c_int {
    // SAFETY: registration retains this exact allocation until removal drains readers.
    let stats = unsafe { &*data.cast::<Statistics>() };
    let Some(report) = stats.load.snapshot() else {
        return EAGAIN.to_errno();
    };
    let current = report.current();
    // SAFETY: the C callback provides exclusive, properly aligned stack output storage.
    unsafe {
        out.cast::<Export>().write(Export {
            valid: u32::from(current.is_some()),
            power_known: u32::from(report.power_known),
            powered: u32::from(report.powered),
            utilization: current.unwrap_or([0; 4]),
            clock_hz: super::load::CLOCK_HZ as u32,
            samples: report.samples,
            received_ns: report.received_ns,
            timestamp_ticks: report.ticks,
            power_timestamp_ticks: report.power_ticks,
            powered_ticks: report.powered_ticks,
            unpowered_ticks: report.unpowered_ticks,
            invalid: report.invalid,
            age_ns: super::now_ns().saturating_sub(report.received_ns),
        })
    };
    0
}

pub(super) struct Telemetry {
    dev: *mut c_void,
    handle: *mut c_void,
    _statistics: Arc<Statistics>,
}

// SAFETY: raw pointers are used only for attribute removal; readers access atomics.
unsafe impl Send for Telemetry {}
// SAFETY: registration state is immutable until its exclusive drop removes the attribute.
unsafe impl Sync for Telemetry {}

impl Telemetry {
    pub(super) fn new(dev: &Device, statistics: Arc<Statistics>) -> Result<Self> {
        let raw = dev.as_raw().cast();
        let mut handle = core::ptr::null_mut();
        // SAFETY: the bound platform device outlives this registration; the Arc owns data.
        let ret = unsafe {
            asahi_telemetry_register(
                raw,
                (&*statistics as *const Statistics).cast(),
                snapshot,
                &mut handle,
            )
        };
        kernel::error::to_result(ret)?;
        Ok(Self {
            dev: raw,
            handle,
            _statistics: statistics,
        })
    }
}

impl Drop for Telemetry {
    fn drop(&mut self) {
        // SAFETY: removal drains callback users before the Arc and its allocation are dropped.
        unsafe { asahi_telemetry_unregister(self.dev, self.handle) };
    }
}
