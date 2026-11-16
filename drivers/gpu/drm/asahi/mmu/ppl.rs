// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! The M4 two-party PPL protocol. The transport owns the host mutex for the entire
//! guard lifetime: firmware exclusion alone does not serialize two AP threads.
//! This core has no kernel dependencies so the production guard can be exercised
//! against deterministic firmware and concurrent host transports.

pub(super) trait Memory {
    fn firmware_interested(&self) -> bool;
    fn firmware_turn(&self) -> bool;
    fn ap_interest(&self, interested: bool);
    fn give_firmware_turn(&self);
    fn barrier(&self);
    fn expired(&self) -> bool;
    fn relax(&self);
    /// Latch failure before the transport releases host exclusion on timeout.
    fn fail(&self);
}

/// Owns both the host-side lock (in `memory`) and successful firmware exclusion.
#[must_use = "retain the guard until every protected table mutation has completed"]
pub(super) struct Guard<T: Memory> {
    memory: T,
}

impl<T: Memory> Guard<T> {
    pub(super) fn acquire(memory: T) -> Result<Self, ()> {
        let timeout = || {
            memory.ap_interest(false);
            memory.barrier();
            memory.fail();
            Err(())
        };
        memory.ap_interest(true);
        memory.barrier();
        while memory.firmware_interested() {
            if memory.expired() {
                return timeout();
            }
            if memory.firmware_turn() {
                memory.ap_interest(false);
                memory.barrier();
                while memory.firmware_turn() {
                    if memory.expired() {
                        return timeout();
                    }
                    memory.relax();
                }
                memory.ap_interest(true);
                memory.barrier();
            }
            memory.relax();
        }
        memory.barrier();
        Ok(Self { memory })
    }

    pub(super) fn memory_mut(&mut self) -> &mut T {
        &mut self.memory
    }
}

impl<T: Memory> Drop for Guard<T> {
    fn drop(&mut self) {
        self.memory.give_firmware_turn();
        self.memory.barrier();
        self.memory.ap_interest(false);
        self.memory.barrier();
        // Only after these stores complete may `memory` release the host mutex.
    }
}
