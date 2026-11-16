// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Reuse address-coverage proofs only while the VM's page tables are unchanged.

use super::command::{Access, AddressSpace, Payload, UscWindow, Validated};
use crate::mmu;
use core::{cell::RefCell, ops::Range};
use kernel::prelude::*;

const MAX_RANGES: usize = 48;
// The persistent proof must retain a deep queue's rotating command buffers.
// Keep the transient parser request set small to bound its stack frame.
const MAX_CACHED_RANGES: usize = 128;
type Request = (u64, u64, u8);

/// Match the page footprint already checked by `Vm::covers_range_locked`.
fn pages((address, size, access): Request) -> Option<Request> {
    if size == 0 || access == 0 || access & !3 != 0 {
        return None;
    }
    let mask = mmu::UAT_PGSZ as u64 - 1;
    let end = address.checked_add(size)?;
    let end = end.checked_add(mask)? & !mask;
    let start = address & !mask;
    Some((start, end.checked_sub(start)?, access))
}

fn contains((start, size, access): Request, (other, length, needed): Request) -> bool {
    access & needed == needed
        && other >= start
        && other.checked_add(length).is_some_and(|end|
            start.checked_add(size).is_some_and(|limit| end <= limit))
}

#[derive(Clone, Copy)]
struct Requests<const CAPACITY: usize = MAX_RANGES> {
    ranges: [Request; CAPACITY],
    count: usize,
    overflow: bool,
}

impl<const CAPACITY: usize> Requests<CAPACITY> {
    const fn new() -> Self {
        Self {
            ranges: [(0, 0, 0); CAPACITY],
            count: 0,
            overflow: false,
        }
    }

    fn as_slice(&self) -> &[Request] {
        &self.ranges[..self.count]
    }

    fn covers(&self, request: Request) -> bool {
        self.as_slice().iter().any(|&range| contains(range, request))
    }

    fn insert(&mut self, request: Request) {
        let Some(mut request) = pages(request) else {
            self.overflow = true;
            return;
        };
        if self.covers(request) {
            return;
        }
        // Merge only equal permissions, so an adjacent writable page never
        // grants write access to a read-only page. Restart after each merge
        // because it can join a range visited earlier.
        let mut index = 0;
        while index < self.count {
            let range = self.ranges[index];
            let end = request.0 + request.1;
            let other_end = range.0 + range.1;
            if contains(request, range)
                || (request.2 == range.2 && request.0 <= other_end && range.0 <= end)
            {
                if request.2 == range.2 {
                    let start = request.0.min(range.0);
                    request = (start, end.max(other_end) - start, request.2);
                }
                self.count -= 1;
                self.ranges[index] = self.ranges[self.count];
                index = 0;
            } else {
                index += 1;
            }
        }
        if self.count == CAPACITY {
            self.overflow = true;
            return;
        }
        self.ranges[self.count] = request;
        self.count += 1;
    }

}

/// One logical queue's bounded cache of validated page ranges and permissions.
/// Mapping mutations make the epoch unavailable until their final page-table write.
pub(crate) struct Proof {
    epoch: Option<u64>,
    requests: Requests<MAX_CACHED_RANGES>,
}

impl Proof {
    pub(crate) const fn new() -> Self {
        Self {
            epoch: None,
            requests: Requests::new(),
        }
    }

    fn covers(&self, epoch: Option<u64>, requests: &Requests) -> bool {
        self.epoch.is_some()
            && self.epoch == epoch
            && requests
                .as_slice()
                .iter()
                .all(|&request| self.requests.covers(request))
    }

    fn record(&mut self, epoch: Option<u64>, requests: Requests) {
        // Alternating in-flight buffers must not evict each other's proofs.
        if epoch.is_some() && self.epoch == epoch && !requests.overflow {
            for &request in requests.as_slice() {
                self.requests.insert(request);
            }
            if !self.requests.overflow {
                return;
            }
        }
        // An incomplete union cannot be reused; retain the latest complete set.
        self.epoch = if requests.overflow { None } else { epoch };
        self.requests = Requests::new();
        for &request in requests.as_slice() {
            self.requests.insert(request);
        }
    }

    /// The VM-job guard must already pin the mappings for this command. Keep the
    /// range arrays off the caller's frame before descending into publication.
    #[inline(never)]
    pub(crate) fn validate(
        &mut self,
        vm: &mmu::Vm,
        range: Range<u64>,
        window: &UscWindow,
        variant: &crate::g17::fw::Variant,
        payload: &Payload,
    ) -> Result<Validated> {
        let requests = RefCell::new(Requests::new());
        let optimistic = payload.validate(window, variant, &Space::Record(&range, &requests));
        let requests = requests.into_inner();
        if optimistic.is_ok() && !requests.overflow {
            if self.covers(vm.mapping_validation_epoch(), &requests) {
                return optimistic;
            }
            let (covered, epoch) = vm.covers_ranges_batch(requests.as_slice());
            if covered {
                self.record(epoch, requests);
                return optimistic;
            }
            self.epoch = None;
        }
        // Preserve per-range validation's failure and concurrent-mutation behavior.
        payload.validate(window, variant, &Space::Checked(&range, vm))
    }
}

enum Space<'a> {
    Record(&'a Range<u64>, &'a RefCell<Requests>),
    Checked(&'a Range<u64>, &'a mmu::Vm),
}

impl AddressSpace for Space<'_> {
    fn covers(&self, address: u64, size: u64, access: Access) -> bool {
        let range = match self {
            Self::Record(range, _) | Self::Checked(range, _) => range,
        };
        let Some(end) = address.checked_add(size) else {
            return false;
        };
        if size == 0 || address < range.start || end > range.end {
            return false;
        }
        let permissions = match access {
            Access::Read => 1,
            Access::Write => 2,
            Access::ReadWrite => 3,
        };
        match self {
            Self::Record(_, requests) => {
                requests.borrow_mut().insert((address, size, permissions));
                true
            }
            Self::Checked(_, vm) => {
                vm.covers_range(address, size, permissions & 1 != 0, permissions & 2 != 0)
            }
        }
    }
}


#[kunit_tests(asahi_g17_validation_pages)]
mod tests {
    use super::*;
    const PAGE: u64 = mmu::UAT_PGSZ as u64;

    fn requests(range: Request) -> Requests {
        let mut result = Requests::new();
        result.insert(range);
        result
    }

    #[test]
    fn different_offsets_share_the_already_validated_page() {
        let mut proof = Proof::new();
        proof.record(Some(7), requests((PAGE + 4, 4, 3)));
        assert!(proof.covers(Some(7), &requests((PAGE + 0x100, 8, 1))));
        assert!(proof.covers(Some(7), &requests((2 * PAGE - 8, 8, 2))));
        assert!(!proof.covers(Some(7), &requests((2 * PAGE - 8, 16, 1))));
        assert!(!proof.covers(Some(8), &requests((PAGE + 4, 4, 1))));
        assert!(!proof.covers(None, &requests((PAGE + 4, 4, 1))));
    }

    #[test]
    fn adjacent_equal_permissions_merge_but_holes_do_not() {
        let mut ranges = requests((PAGE, 1, 1));
        ranges.insert((3 * PAGE, 1, 1));
        assert_eq!(ranges.count, 2);
        assert!(!ranges.covers((2 * PAGE, PAGE, 1)));
        ranges.insert((2 * PAGE, 1, 1));
        assert_eq!(ranges.count, 1);
        assert_eq!(ranges.as_slice(), &[(PAGE, 3 * PAGE, 1)]);
    }

    #[test]
    fn adjacent_permissions_never_grant_access_to_another_page() {
        let mut proof = Proof::new();
        let mut ranges = requests((PAGE, 1, 1));
        ranges.insert((2 * PAGE, 1, 3));
        proof.record(Some(1), ranges);
        assert!(!proof.covers(Some(1), &requests((PAGE, 1, 2))));
        assert!(proof.covers(Some(1), &requests((2 * PAGE, 1, 2))));
        assert!(!proof.covers(Some(1), &requests((PAGE, 2 * PAGE, 3))));
    }

    #[test]
    fn stronger_coverage_removes_only_fully_covered_ranges() {
        let mut ranges = requests((PAGE, PAGE, 1));
        ranges.insert((2 * PAGE, PAGE, 2));
        ranges.insert((PAGE, 2 * PAGE, 3));
        assert_eq!(ranges.as_slice(), &[(PAGE, 2 * PAGE, 3)]);
        assert!(!ranges.covers((0, PAGE, 1)));
    }

    #[test]
    fn persistent_cache_retains_a_deep_rotating_working_set() {
        let mut proof = Proof::new();
        for index in 0..64 {
            proof.record(Some(7), requests((index * 2 * PAGE, 8, 1)));
        }
        assert_eq!(proof.requests.count, 64);
        for index in 0..64 {
            assert!(proof.covers(Some(7), &requests((index * 2 * PAGE + 16, 8, 1))));
        }
        assert_eq!(core::mem::size_of::<Requests>(), core::mem::size_of::<Requests<48>>());
    }

    #[test]
    fn persistent_overflow_keeps_only_a_new_complete_proof() {
        let mut proof = Proof::new();
        for index in 0..=MAX_CACHED_RANGES {
            proof.record(Some(7), requests((index as u64 * 2 * PAGE, 8, 1)));
        }
        assert_eq!(proof.requests.count, 1);
        assert!(!proof.covers(Some(7), &requests((0, 8, 1))));
        assert!(proof.covers(Some(7),
            &requests((MAX_CACHED_RANGES as u64 * 2 * PAGE, 8, 1))));
    }

    #[test]
    fn invalid_rounding_and_full_cache_disable_reuse() {
        for range in [(0, 0, 1), (u64::MAX - 1, 1, 1), (0, 1, 0), (0, 1, 4)] {
            assert!(pages(range).is_none());
            let mut proof = Proof::new();
            proof.record(Some(1), requests(range));
            assert!(!proof.covers(Some(1), &requests((0, 1, 1))));
        }
        let mut ranges = Requests::new();
        for index in 0..=MAX_RANGES {
            ranges.insert(((2 * index) as u64 * PAGE, PAGE, 1));
        }
        assert!(ranges.overflow);
        let mut proof = Proof::new();
        proof.record(Some(1), ranges);
        assert!(!proof.covers(Some(1), &requests((0, PAGE, 1))));
    }
}
