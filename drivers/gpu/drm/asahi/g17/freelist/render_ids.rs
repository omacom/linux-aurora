// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Pure physical-ID lease rules. The caller serializes this state and supplies
//! a firmware release proof before a slot is reassigned to another pool.

pub(super) const FIRST: u16 = 129;
pub(super) const COUNT: usize = 127;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) struct Lease {
    pub(super) slot: u16,
    pub(super) owner: u64,
    pub(super) generation: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) struct Claim {
    pub(super) lease: Lease,
    pub(super) displaced: Option<Lease>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Error {
    BadOwner,
    Busy,
    PendingRelease,
    StaleLease,
    Overflow,
}

#[derive(Clone, Copy)]
struct Slot {
    owner: u64,
    generation: u64,
    pins: u32,
    last_use: u64,
}

impl Slot {
    const EMPTY: Self = Self { owner: 0, generation: 0, pins: 0, last_use: 0 };

    fn lease(self, index: usize) -> Lease {
        Lease { slot: FIRST + index as u16, owner: self.owner, generation: self.generation }
    }
}

pub(super) struct State {
    slots: [Slot; COUNT],
    clock: u64,
}

impl State {
    pub(super) const fn new() -> Self { Self { slots: [Slot::EMPTY; COUNT], clock: 0 } }

    /// Constant-time cached-owner path; a recycled generation cannot be pinned.
    pub(super) fn pin(&mut self, lease: Lease) -> Result<Lease, Error> {
        let index = lease.slot.checked_sub(FIRST).ok_or(Error::StaleLease)? as usize;
        let slot = self.slots.get_mut(index).ok_or(Error::StaleLease)?;
        if slot.owner != lease.owner || slot.generation != lease.generation || slot.owner == 0 {
            return Err(Error::StaleLease);
        }
        let clock = self.clock.checked_add(1).ok_or(Error::Overflow)?;
        let pins = slot.pins.checked_add(1).ok_or(Error::Overflow)?;
        slot.pins = pins;
        slot.last_use = clock;
        self.clock = clock;
        Ok(lease)
    }

    /// A successful claim is a preparation pin. It must remain pinned through
    /// any out-of-lock construction and every firmware reference, not just
    /// through publication. Existing-owner reacquisition does not reassign it.
    pub(super) fn claim(&mut self, owner: u64, mut released: impl FnMut(Lease) -> bool)
        -> Result<Claim, Error>
    {
        if owner == 0 { return Err(Error::BadOwner); }
        let next_clock = self.clock.checked_add(1).ok_or(Error::Overflow)?;
        if let Some(index) = self.slots.iter().position(|s| s.owner == owner) {
            let slot = &mut self.slots[index];
            let pins = slot.pins.checked_add(1).ok_or(Error::Overflow)?;
            slot.pins = pins;
            slot.last_use = next_clock;
            self.clock = next_clock;
            return Ok(Claim { lease: slot.lease(index), displaced: None });
        }
        let mut candidate = self.slots.iter().position(|s| s.owner == 0);
        let mut displaced = None;
        if candidate.is_none() {
            // One bounded scan, with no allocation and at most one release
            // query per idle slot. Normal cached-owner pinning is O(1).
            let mut idle = false;
            for (index, slot) in self.slots.iter().enumerate() {
                if slot.pins != 0 { continue; }
                idle = true;
                if candidate.is_some_and(|best| self.slots[best].last_use <= slot.last_use) { continue; }
                let old = slot.lease(index);
                if released(old) {
                    candidate = Some(index);
                    displaced = Some(old);
                }
            }
            if candidate.is_none() {
                return Err(if idle { Error::PendingRelease } else { Error::Busy });
            }
        }
        let index = candidate.ok_or(Error::Busy)?;
        let generation = self.slots[index].generation.checked_add(1).ok_or(Error::Overflow)?;
        self.slots[index] = Slot { owner, generation, pins: 1, last_use: next_clock };
        self.clock = next_clock;
        Ok(Claim { lease: self.slots[index].lease(index), displaced })
    }

    /// An old owner or old generation must never release the successor's pin.
    pub(super) fn unpin(&mut self, lease: Lease) -> Result<bool, Error> {
        let index = lease.slot.checked_sub(FIRST).ok_or(Error::StaleLease)? as usize;
        let slot = self.slots.get_mut(index).ok_or(Error::StaleLease)?;
        if slot.owner != lease.owner || slot.generation != lease.generation || slot.pins == 0 {
            return Err(Error::StaleLease);
        }
        slot.pins -= 1;
        Ok(slot.pins == 0)
    }

    /// Pool destruction may also need a pending release to finish. The caller
    /// retains its memory when this refuses; dropping it is not a release proof.
    pub(super) fn remove(&mut self, owner: u64, released: impl FnOnce(Lease) -> bool)
        -> Result<(), Error>
    {
        if owner == 0 { return Err(Error::BadOwner); }
        let Some(index) = self.slots.iter().position(|s| s.owner == owner) else { return Ok(()); };
        let slot = self.slots[index];
        if slot.pins != 0 { return Err(Error::Busy); }
        if !released(slot.lease(index)) { return Err(Error::PendingRelease); }
        self.slots[index].owner = 0;
        Ok(())
    }

    /// Return one cached, unpinned registration only after its release proof.
    pub(super) fn reclaim_one(&mut self, mut released: impl FnMut(Lease) -> bool) -> Option<Lease> {
        let index = (0..COUNT)
            .filter(|&i| self.slots[i].owner != 0 && self.slots[i].pins == 0)
            .filter(|&i| released(self.slots[i].lease(i)))
            .min_by_key(|&i| self.slots[i].last_use)?;
        let old = self.slots[index].lease(index);
        self.slots[index].owner = 0;
        Some(old)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn full() -> (State, Vec<Lease>) {
        let mut state = State::new();
        let leases = (1..=COUNT as u64).map(|owner|
            state.claim(owner, |_| panic!("free slot must not need release proof")).unwrap().lease
        ).collect();
        (state, leases)
    }

    #[test] fn bounded_physical_namespace() {
        let (state, leases) = full();
        assert_eq!(leases.iter().map(|l| l.slot).collect::<Vec<_>>(), (129..=255).collect::<Vec<_>>());
        assert_eq!(state.slots.len(), COUNT);
    }

    #[test] fn all_preparation_pins_prevent_reassignment() {
        let (mut state, _) = full();
        assert_eq!(state.claim(1000, |_| panic!("pinned slot must not be offered")), Err(Error::Busy));
    }

    #[test] fn unpinned_is_not_firmware_released() {
        let (mut state, leases) = full();
        for lease in &leases { state.unpin(*lease).unwrap(); }
        assert_eq!(state.claim(1000, |_| false), Err(Error::PendingRelease));
        for (i, lease) in leases.iter().enumerate() { assert_eq!(state.slots[i].lease(i), *lease); }
    }

    #[test] fn released_slot_can_serve_another_logical_owner() {
        let (mut state, leases) = full();
        state.unpin(leases[13]).unwrap();
        let claim = state.claim(1000, |old| old == leases[13]).unwrap();
        assert_eq!(claim.displaced, Some(leases[13]));
        assert_eq!(claim.lease.slot, leases[13].slot);
        assert!(claim.lease.generation > leases[13].generation);
        assert_eq!(state.unpin(leases[13]), Err(Error::StaleLease));
        assert!(state.unpin(claim.lease).unwrap());
    }

    #[test] fn same_owner_reuses_without_release_proof_or_generation_change() {
        let mut state = State::new();
        let a = state.claim(1, |_| false).unwrap().lease;
        let b = state.claim(1, |_| panic!("same owner is not reassigned")).unwrap().lease;
        assert_eq!(a, b);
        assert!(!state.unpin(a).unwrap());
        assert!(state.unpin(b).unwrap());
        assert_eq!(state.unpin(a), Err(Error::StaleLease));
    }

    #[test] fn stale_generation_cannot_unpin_returned_owner() {
        let (mut state, leases) = full();
        state.unpin(leases[0]).unwrap();
        let other = state.claim(1000, |_| true).unwrap().lease;
        state.unpin(other).unwrap();
        let returned = state.claim(leases[0].owner, |_| true).unwrap().lease;
        assert_eq!(returned.slot, leases[0].slot);
        assert_eq!(state.unpin(leases[0]), Err(Error::StaleLease));
        assert!(state.unpin(returned).unwrap());
    }

    #[test] fn thousands_of_logical_owners_do_not_expand_the_namespace() {
        let mut state = State::new();
        for owner in 1..=4096 {
            let lease = state.claim(owner, |_| true).unwrap().lease;
            assert!((129..=255).contains(&lease.slot));
            assert!(state.unpin(lease).unwrap());
        }
        assert_eq!(state.slots.iter().filter(|s| s.owner != 0).count(), COUNT);
    }

    #[test] fn removal_waits_for_both_pins_and_firmware() {
        let mut state = State::new();
        let lease = state.claim(1, |_| false).unwrap().lease;
        assert_eq!(state.remove(1, |_| true), Err(Error::Busy));
        state.unpin(lease).unwrap();
        assert_eq!(state.remove(1, |_| false), Err(Error::PendingRelease));
        state.remove(1, |old| old == lease).unwrap();
        assert_eq!(state.unpin(lease), Err(Error::StaleLease));
    }

    #[test] fn cached_pin_preserves_lease_and_blocks_reclaim() {
        let (mut state, leases) = full();
        state.unpin(leases[0]).unwrap();
        assert_eq!(state.pin(leases[0]), Ok(leases[0]));
        assert_eq!(state.claim(1000, |_| true), Err(Error::Busy));
    }

    #[test] fn stale_cached_pin_cannot_name_new_owner() {
        let (mut state, leases) = full();
        state.unpin(leases[0]).unwrap();
        let replacement = state.claim(1000, |_| true).unwrap().lease;
        assert_eq!(state.pin(leases[0]), Err(Error::StaleLease));
        assert!(state.unpin(replacement).unwrap());
    }
}
