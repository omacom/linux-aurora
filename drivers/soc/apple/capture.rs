// SPDX-License-Identifier: GPL-2.0-only OR MIT
//! Shared capture ownership; an idle biometric file does not own the sensor.

use crate::{key_request::Request, SepData};
use kernel::prelude::*;
use kernel::sync::{atomic::Relaxed, Arc};

enum Owner {
    Idle,
    Sync(u64),
    Bio(u64),
    Key(Arc<Request>),
}

pub(crate) struct State {
    generation: u64,
    owner: Owner,
}

impl State {
    pub(crate) fn new() -> Self {
        Self { generation: 0, owner: Owner::Idle }
    }

    fn acquire(&mut self) -> Result<u64> {
        if !matches!(self.owner, Owner::Idle) { return Err(EBUSY); }
        self.generation = self.generation.checked_add(1).ok_or(EOVERFLOW)?;
        self.owner = Owner::Sync(self.generation);
        Ok(self.generation)
    }

    pub(crate) fn start_key(&mut self, request: Arc<Request>) -> Result {
        if !matches!(self.owner, Owner::Idle) { return Err(EBUSY); }
        self.owner = Owner::Key(request);
        Ok(())
    }

    pub(crate) fn key(&self) -> Option<Arc<Request>> {
        match &self.owner {
            Owner::Key(request) => Some(request.clone()),
            _ => None,
        }
    }

    pub(crate) fn finish_key(&mut self, request: &Arc<Request>) {
        if matches!(&self.owner, Owner::Key(active) if Arc::ptr_eq(active, request)) {
            self.owner = Owner::Idle;
        }
    }

    pub(crate) fn finish_bio(&mut self, epoch: u64) {
        if matches!(self.owner, Owner::Bio(active) if active == epoch) {
            self.owner = Owner::Idle;
        }
    }
}

/// A synchronous firmware operation joins the suspend drain. Ownership can be
/// transferred to a queued bio worker, which releases it only after cleanup.
pub(crate) struct Lease<'a> {
    sep: &'a SepData,
    generation: u64,
    running: bool,
}

impl<'a> Lease<'a> {
    pub(crate) fn acquire(sep: &'a SepData) -> Result<Self> {
        let generation = {
            let mut state = sep.capture.lock();
            if sep.shutting_down.load(Relaxed) { return Err(ENODEV); }
            if sep.suspending.load(Relaxed) { return Err(EBUSY); }
            state.acquire()?
        };
        let mut lease = Self { sep, generation, running: false };
        if !sep.capture_begin() { return Err(EBUSY); }
        lease.running = true;
        Ok(lease)
    }

    pub(crate) fn into_bio(mut self, epoch: u64) {
        let mut state = self.sep.capture.lock();
        if matches!(state.owner, Owner::Sync(active) if active == self.generation) {
            state.owner = Owner::Bio(epoch);
        }
        // The queued worker will join the drain when it starts; a suspend
        // before then aborts its session, and capture_begin refuses hardware.
        self.sep.capture_end();
        self.running = false;
    }
}

impl Drop for Lease<'_> {
    fn drop(&mut self) {
        if self.running { self.sep.capture_end(); }
        let mut state = self.sep.capture.lock();
        if matches!(state.owner, Owner::Sync(active) if active == self.generation) {
            state.owner = Owner::Idle;
        }
    }
}
