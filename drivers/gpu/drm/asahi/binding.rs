// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Scoped DMA-device access during probe and runtime mapping preparation.
//!
//! The platform owner closes admission and joins leases before unbind returns.
//! VMs retain the gate, so an old VM cannot acquire access after removal/rebind.

use crate::driver::AsahiDrmDevice;
use kernel::{
    device::{Bound, Device},
    new_condvar, new_mutex,
    prelude::*,
    sync::{aref::ARef, Arc, CondVar, Mutex},
};

kernel::sync::global_lock! {
    // SAFETY: Module initialization initializes this lock before registering adapters.
    unsafe(uninit) static DEVICES: Mutex<KVec<Arc<Gate>>> = KVec::new();
}

/// Initialize the registry before a platform adapter can probe.
///
/// # Safety
///
/// Called exactly once per module instance, before any other binding operation.
pub(crate) unsafe fn init() {
    // SAFETY: The module initializer establishes the single-call ordering above.
    unsafe { DEVICES.init() };
}

struct State {
    live: bool,
    users: usize,
}

#[pin_data]
pub(crate) struct Gate {
    key: usize,
    dev: ARef<Device>,
    #[pin]
    state: Mutex<State>,
    #[pin]
    changed: CondVar,
}

/// The platform binding's owner; Drop also covers errors during probe.
pub(crate) struct Owner(Arc<Gate>);

impl Owner {
    /// Register while probe supplies the parent bound-device proof.
    ///
    /// # Safety
    ///
    /// The platform adapter must call close before unbind returns, independently
    /// of this owner's Drop. On probe error the owner must be closed/dropped
    /// before the bound probe callback returns.
    pub(crate) unsafe fn new(dev: &Device<Bound>, drm: &AsahiDrmDevice) -> Result<Self> {
        let gate = Arc::pin_init(
            pin_init!(Gate {
                key: core::ptr::from_ref(drm) as usize,
                dev: dev.into(),
                state <- new_mutex!(State { live: true, users: 0 }, "asahi::binding::state"),
                changed <- new_condvar!("asahi::binding::changed"),
            }),
            GFP_KERNEL,
        )?;
        DEVICES.lock().push(gate.clone(), GFP_KERNEL)?;
        Ok(Self(gate))
    }

    /// Called from platform unbind before devres releases the parent's state.
    pub(crate) fn close(&self) {
        DEVICES.lock().retain(|gate| !Arc::ptr_eq(gate, &self.0));
        let mut state = self.0.state.lock();
        state.live = false;
        while state.users != 0 {
            self.0.changed.wait(&mut state);
        }
    }
}

impl Drop for Owner {
    fn drop(&mut self) {
        self.close();
    }
}

/// Look up the binding for this DRM instance, not just its reusable parent address.
pub(crate) fn lookup(drm: &AsahiDrmDevice) -> Result<Arc<Gate>> {
    let key = core::ptr::from_ref(drm) as usize;
    DEVICES
        .lock()
        .iter()
        .find(|gate| gate.key == key)
        .cloned()
        .ok_or(ENODEV)
}

impl Gate {
    /// Covers only SG-table acquisition/segment preparation, not firmware waits.
    pub(crate) fn enter(&self) -> Result<Lease<'_>> {
        let mut state = self.state.lock();
        if !state.live {
            return Err(ENODEV);
        }
        state.users = state.users.checked_add(1).ok_or(EOVERFLOW)?;
        Ok(Lease(self))
    }
}

pub(crate) struct Lease<'a>(&'a Gate);

impl Lease<'_> {
    pub(crate) fn device(&self) -> &Device<Bound> {
        // SAFETY: Owner starts during bound probe, rejects new leases on close,
        // and joins existing leases before unbind returns. This borrow cannot
        // outlive its lease; the ARef also retains the device allocation.
        unsafe { self.0.dev.as_bound() }
    }
}

impl Drop for Lease<'_> {
    fn drop(&mut self) {
        let mut state = self.0.state.lock();
        state.users -= 1;
        if state.users == 0 {
            self.0.changed.notify_all();
        }
    }
}
