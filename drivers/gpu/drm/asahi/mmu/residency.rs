// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Pressure-only backing residency for idle G17 user VMs.
//!
//! This gate protects backing/address preparation as well as accepted jobs.
//! Its writer never waits for active readers: reclaim skips the VM instead.

use super::*;
use kernel::drm::gem::BaseObject;
use kernel::{new_condvar, sync::CondVar};

/// Re-evicting backing that a job just had to restore only repeats the
/// restoration. After a restore, reclaim of this VM pauses for a backoff
/// proportional to what the restore cost, within these bounds.
const RESTORE_BACKOFF_MIN_NS: u64 = 1_000_000_000;
const RESTORE_BACKOFF_MAX_NS: u64 = 30_000_000_000;
const RESTORE_BACKOFF_FACTOR: u64 = 16;
/// A VM whose last job, binding or other residency user ended (or which was
/// created) more recently than this is in use: a submitting or loading client
/// would restore whatever reclaim released, synchronously in its next job.
const RECLAIM_QUIET_NS: u64 = 2000000000;

fn now_ns() -> u64 {
    <kernel::time::Monotonic as kernel::time::ClockSource>::ktime_get() as u64
}

#[derive(Default)]
struct State {
    users: usize,
    busy: bool,
    evicted: bool,
    closed: bool,
    /// Monotonic time the last restoration finished and its duration.
    restored_at: u64,
    restore_cost: u64,
    /// Monotonic time the last residency user (job, binding, retirement)
    /// released its lease; the gate's creation time before the first one.
    last_active: u64,
}

impl State {
    /// Reclaim skips a VM that is in use, was recently used or recently restored.
    fn reclaim_deferred(&self, now: u64) -> bool {
        if now.saturating_sub(self.last_active) < RECLAIM_QUIET_NS {
            return true;
        }
        if self.restored_at == 0 {
            return false;
        }
        let backoff = self
            .restore_cost
            .saturating_mul(RESTORE_BACKOFF_FACTOR)
            .clamp(RESTORE_BACKOFF_MIN_NS, RESTORE_BACKOFF_MAX_NS);
        now.saturating_sub(self.restored_at) < backoff
    }
}

#[pin_data]
pub(crate) struct Gate {
    #[pin]
    state: Mutex<State>,
    #[pin]
    ready: CondVar,
}

pub(crate) struct Lease {
    gate: Arc<Gate>,
}

pub(crate) struct Reclaim {
    gate: Arc<Gate>,
    changed: bool,
}

impl Gate {
    pub(crate) fn new() -> Result<Arc<Self>> {
        Arc::pin_init(
            pin_init!(Self {
                state <- new_mutex!(
                    State { last_active: now_ns(), ..State::default() },
                    "Asahi residency"
                ),
                ready <- new_condvar!("Asahi residency ready"),
            }),
            GFP_KERNEL,
        )
    }

    /// Only a previous pressure eviction can cause restoration or waiting.
    pub(crate) fn enter(gate: &Arc<Self>, mut restore: impl FnMut() -> Result) -> Result<Lease> {
        loop {
            let mut state = gate.state.lock();
            if state.closed {
                return Err(ENOENT);
            }
            while state.busy {
                if gate.ready.wait_interruptible(&mut state) {
                    return Err(ERESTARTSYS);
                }
            }
            if state.closed {
                return Err(ENOENT);
            }
            if !state.evicted {
                state.users = state.users.checked_add(1).ok_or(EOVERFLOW)?;
                return Ok(Lease { gate: gate.clone() });
            }
            // Binding edits can add mappings or retire invalidated GPUVAs
            // without restoring the rest of the VM. Wait for those mutations
            // before walking the authoritative logical bindings for restoration.
            if state.users != 0 {
                if gate.ready.wait_interruptible(&mut state) {
                    return Err(ERESTARTSYS);
                }
                continue;
            }
            // No new reader can enter until every retained mapping is restored.
            state.busy = true;
            drop(state);
            let started = now_ns();
            let result = restore();
            let mut state = gate.state.lock();
            state.busy = false;
            if result.is_ok() {
                state.evicted = false;
                let now = now_ns();
                state.restored_at = now;
                state.restore_cost = now.saturating_sub(started);
            }
            drop(state);
            gate.ready.notify_all();
            result?;
        }
    }

    pub(crate) fn enter_metadata(gate: &Arc<Self>) -> Result<Lease> {
        let mut state = gate.state.lock();
        while state.busy {
            if gate.ready.wait_interruptible(&mut state) {
                return Err(ERESTARTSYS);
            }
        }
        if state.closed {
            return Err(ENOENT);
        }
        state.users = state.users.checked_add(1).ok_or(EOVERFLOW)?;
        Ok(Lease { gate: gate.clone() })
    }

    /// Retirement of an already closed handle may outlive the file's VM.
    /// A closed gate excludes every new reclaimer and restorer, so no lease
    /// is necessary then. Otherwise serialize metadata removal with restoration.
    pub(super) fn enter_retirement(gate: &Arc<Self>) -> Option<Lease> {
        let mut state = gate.state.lock();
        while state.busy {
            gate.ready.wait(&mut state);
        }
        if state.closed {
            return None;
        }
        state.users += 1;
        Some(Lease { gate: gate.clone() })
    }

    /// Direct reclaim cannot block on a job, another reclaimer or restoration.
    /// A prior partial eviction does not exclude another scan: the VM may
    /// still retain eligible BOs after satisfying a previous scan target.
    /// The busy gate continues to exclude restoration and metadata changes.
    pub(crate) fn try_reclaim(gate: &Arc<Self>) -> Option<Reclaim> {
        let mut state = gate.state.try_lock()?;
        if state.users != 0 || state.busy || state.closed || state.reclaim_deferred(now_ns()) {
            return None;
        }
        state.busy = true;
        Some(Reclaim {
            gate: gate.clone(),
            changed: false,
        })
    }

    pub(crate) fn close(&self) {
        let mut state = self.state.lock();
        state.closed = true;
        while state.busy {
            self.ready.wait(&mut state);
        }
    }

    pub(crate) fn idle(&self) -> bool {
        self.state.try_lock().is_some_and(|state| {
            state.users == 0
                && !state.busy
                && !state.closed
                && !state.reclaim_deferred(now_ns())
        })
    }

    /// Pressure reclaim released this VM's backing and no user has entered
    /// since: its next job restores first. Never blocks; a contended gate
    /// reads as not evicted.
    pub(crate) fn evicted(&self) -> bool {
        self.state
            .try_lock()
            .is_some_and(|state| state.evicted && !state.busy && state.users == 0)
    }

    /// A page-table mutation happened outside a `Reclaim` (the purge path runs
    /// on busy VMs). The next `enter` restores every INVALIDATED mapping.
    pub(crate) fn note_evicted(&self) {
        self.state.lock().evicted = true;
    }
}

impl Reclaim {
    /// Call before the first page-table mutation, including fallible unmaps.
    pub(crate) fn mark_changed(&mut self) {
        self.changed = true;
    }
}

impl Drop for Reclaim {
    fn drop(&mut self) {
        let mut state = self.gate.state.lock();
        state.evicted |= self.changed;
        state.busy = false;
        drop(state);
        self.gate.ready.notify_all();
    }
}

impl Drop for Lease {
    fn drop(&mut self) {
        let mut state = self.gate.state.lock();
        debug_assert!(state.users != 0);
        state.users -= 1;
        state.last_active = now_ns();
        if state.users == 0 {
            drop(state);
            self.gate.ready.notify_all();
        }
    }
}

/// Registration lifetime belongs to the file's VM handle, not the VM clone
/// retained by the callback. Unregister synchronously before dropping the clone.
pub(crate) struct VmShrinker {
    vm: crate::mmu::Vm,
    raw: *mut kernel::bindings::shrinker,
}

// SAFETY: shrinker core serializes registration lifetime; callback VM operations
// use its residency/reservation locks. raw is never dereferenced after free.
unsafe impl Send for VmShrinker {}
// SAFETY: Shared callbacks use the VM's residency/reservation locks. The
// registration remains live until shrinker_free has drained all callbacks.
unsafe impl Sync for VmShrinker {}

impl VmShrinker {
    pub(crate) fn new(vm: &crate::mmu::Vm) -> Result<KBox<Self>> {
        let mut owner = KBox::new(
            Self {
                vm: vm.clone(),
                raw: core::ptr::null_mut(),
            },
            GFP_KERNEL,
        )?;
        // SAFETY: The format is static and has no format arguments.
        let raw =
            unsafe { kernel::bindings::shrinker_alloc(0, c_str!("asahi-idle-vm").as_char_ptr()) };
        if raw.is_null() {
            return Err(ENOMEM);
        }
        owner.raw = raw;
        // SAFETY: Newly allocated/unregistered shrinker is exclusively owned.
        unsafe {
            (*raw).count_objects = Some(Self::count);
            (*raw).scan_objects = Some(Self::scan);
            (*raw).private_data = core::ptr::from_ref(&*owner).cast_mut().cast();
            (*raw).seeks = 2;
            (*raw).batch = 256;
            kernel::bindings::shrinker_register(raw);
        }
        Ok(owner)
    }

    /// Count reclaimable pages.
    ///
    /// # Safety
    ///
    /// `raw` must be this owner's live registered shrinker. The core must
    /// retain its private data until the callback returns.
    unsafe extern "C" fn count(
        raw: *mut kernel::bindings::shrinker,
        _sc: *mut kernel::bindings::shrink_control,
    ) -> kernel::ffi::c_ulong {
        // SAFETY: The registration owns this stable allocation until callbacks drain.
        let owner = unsafe { &*((*raw).private_data as *const Self) };
        owner.vm.idle_reclaim_count()
    }

    /// Reclaim idle pages.
    ///
    /// # Safety
    ///
    /// `raw` must be this owner's live registered shrinker, and `sc` must
    /// point to the initialized control supplied by the shrinker core.
    unsafe extern "C" fn scan(
        raw: *mut kernel::bindings::shrinker,
        sc: *mut kernel::bindings::shrink_control,
    ) -> kernel::ffi::c_ulong {
        // SAFETY: Both pointers are supplied by the registered shrinker core.
        let owner = unsafe { &*((*raw).private_data as *const Self) };
        // SAFETY: The shrinker core supplies a live initialized control.
        let target = unsafe { (*sc).nr_to_scan };
        owner.vm.reclaim_idle_pages(target)
    }
}

impl Drop for VmShrinker {
    fn drop(&mut self) {
        if !self.raw.is_null() {
            // SAFETY: This owns the sole registration; free drains callbacks
            // before the embedded VM and callback private_data can disappear.
            unsafe { kernel::bindings::shrinker_free(self.raw) };
        }
    }
}

impl Vm {
    pub(crate) fn close_idle_reclaim(&self) {
        if let Some(gate) = self.residency.as_ref() {
            gate.close();
        }
    }

    pub(crate) fn residency_reclaim_enabled(&self) -> bool {
        self.residency.is_some()
    }

    /// The residency gate of a user VM, for observers that must not keep the VM.
    pub(crate) fn residency_gate(&self) -> Option<Arc<Gate>> {
        self.residency.clone()
    }

    pub(super) fn enter_residency_metadata(&self) -> Result<Option<Lease>> {
        self.residency
            .as_ref()
            .map(Gate::enter_metadata)
            .transpose()
    }

    pub(super) fn enter_residency(&self) -> Result<Option<Lease>> {
        self.residency
            .as_ref()
            .map(|gate| Gate::enter(gate, || self.restore_idle_pages()))
            .transpose()
    }

    pub(crate) fn idle_reclaim_count(&self) -> usize {
        let Some(gate) = self.residency.as_ref() else {
            return 0;
        };
        if self
            .status
            .as_ref()
            .is_none_or(|status| status.get() != 0)
        {
            return 0;
        }
        let idle = gate.idle();
        let Some(mut inner) = self.gpuvm.try_lock_private() else {
            return 0;
        };
        let mut pages = 0usize;
        // SAFETY: Counting changes no logical mapping/list. Private BOs share
        // this reservation; linked aliases retain their borrowed BO and GEM.
        let _ = unsafe { inner.for_each_private_bo(|_, bo| {
            let object = bo.obj();
            // Count retained backing on idle VMs, and purgeable contents even if idle
            // reclaim already released the table. Never wait on the BO mutex under resv.
            let Some(segments) = bo.data().segments.try_lock() else { return Ok(true); };
            let advised = object.madv_locked() > 0;
            let releasable = if segments.is_some() {
                object.sole_sg_lease_releasable_locked()
            } else {
                (advised || object.has_sg_table_locked())
                    && object.unleased_pages_releasable_locked()
            };
            if object.idle_reclaim_candidate() && (advised || idle) && releasable {
                pages = pages.saturating_add(object.size() / kernel::page::PAGE_SIZE);
            }
            Ok(true)
        }) };

        pages
    }

    /// Discard the backing of objects userspace marked DONTNEED. This needs
    /// only the VM reservation: binding edits and restoration hold it too,
    /// and the advice guarantees no pending or future GPU reference until
    /// the next WILLNEED. Purged mappings stay INVALIDATED until unbound.
    fn purge_advised_pages(&self, target: usize) -> usize {
        let Some(gate) = self.residency.as_ref() else {
            return 0;
        };
        if self
            .status
            .as_ref()
            .is_none_or(|status| status.get() != 0)
        {
            return 0;
        }
        let Some(mut inner) = self.gpuvm.try_lock_private() else {
            return 0;
        };
        let uat = inner.uat_inner.clone();
        let Some(shared) = uat.shared.try_lock() else {
            return 0;
        };
        let contexts = shared.contexts_naming_root(inner.ttb(), inner.uat_inner.m4_adopted);
        let mut purged = 0usize;
        // SAFETY: The private reservation excludes binding edits and restoration.
        // Callbacks change PTEs/flags only; all aliases stay linked.
        let _ = unsafe { inner.for_each_private_bo(|inner, bo| {
            if purged >= target { return Ok(false); }
            let object = bo.obj();
            if !object.idle_reclaim_candidate() || object.madv_locked() <= 0 {
                return Ok(true);
            }
            {
                let Some(segments) = bo.data().segments.try_lock() else { return Ok(true); };
                let releasable = if segments.is_some() {
                    object.sole_sg_lease_releasable_locked()
                } else {
                    object.unleased_pages_releasable_locked()
                };
                if !releasable { return Ok(true); }
            }
            // Any mutation that does not end in a completed purge leaves
            // INVALIDATED aliases with cleared PTEs while madv stays >= 0, so
            // the next WILLNEED would report the contents retained. Record the
            // eviction so the gate restores those aliases before the next job.
            let mut mutated = false;
            let invalidated = inner.invalidate_private_bo_mappings(bo, |inner, start, range| {
                mutated = true;
                let end = start.checked_add(range).ok_or(EOVERFLOW)?;
                let _change = inner.mapping_mutation();
                inner.page_table.discard_partial_map(start..end)?;
                inner.tlbi_context_mask(start, range as usize, contexts);
                Ok(())
            });
            if let Err(error) = invalidated {
                if mutated {
                    gate.note_evicted();
                }
                return Err(error);
            }
            {
                let Some(mut segments) = bo.data().segments.try_lock() else {
                    gate.note_evicted();
                    return Ok(true);
                };
                *segments = None;
            }
            if object.purge_locked().is_ok() {
                crate::gem::set_mapping_held(object, false);
                purged = purged.saturating_add(object.size() / kernel::page::PAGE_SIZE);
            } else {
                gate.note_evicted();
            }
            Ok(purged < target)
        }) };
        drop(shared);
        drop(inner);
        purged
    }

    pub(crate) fn reclaim_idle_pages(&self, target: usize) -> usize {
        let Some(gate) = self.residency.as_ref() else {
            return 0;
        };
        let purged = self.purge_advised_pages(target);
        if purged >= target {
            return purged;
        }
        let target = target - purged;
        let Some(mut reclaim) = Gate::try_reclaim(gate) else {
            return purged;
        };
        // Error fences cannot prove firmware ownership ended.
        if self
            .status
            .as_ref()
            .is_none_or(|status| status.get() != 0)
            || !self
                .lifetime
                .as_ref()
                .is_some_and(|lifetime| lifetime.can_reclaim())
        {
            return purged;
        }
        let Some(mut inner) = self.gpuvm.try_lock_private() else {
            return purged;
        };
        let uat = inner.uat_inner.clone();
        let Some(shared) = uat.shared.try_lock() else {
            return purged;
        };
        let contexts = shared.contexts_naming_root(inner.ttb(), inner.uat_inner.m4_adopted);
        let mut unpinned = 0usize;
        // SAFETY: Exclusive residency and the private reservation exclude jobs,
        // binding edits and retirement. Callbacks change PTEs/flags only; all
        // logical aliases stay linked and retain their borrowed BO/GEM.
        let _ = unsafe { inner.for_each_private_bo(|inner, bo| {
            if unpinned >= target { return Ok(false); }
            let object = bo.obj();
            if !object.idle_reclaim_candidate() { return Ok(true); }
            {
                // Avoid inversion against preparation (VmBoInner -> resv).
                let Some(segments) = bo.data().segments.try_lock() else { return Ok(true); };
                let releasable = if segments.is_some() {
                    object.sole_sg_lease_releasable_locked()
                } else {
                    // A competing lease may have made the previous release fail after we
                    // dropped our segments. Retry that table, but do not count empty backing.
                    object.has_sg_table_locked() && object.unleased_pages_releasable_locked()
                };
                if !releasable { return Ok(true); }
            }
            // The association retains SG backing until *all* aliases have
            // been invalidated and their device translations flushed. A
            // contended cursor or PTE failure retains backing and the touched
            // INVALIDATED prefix for restoration or a later reclaim attempt.
            inner.invalidate_private_bo_mappings(bo, |inner, start, range| {
                reclaim.mark_changed();
                let end = start.checked_add(range).ok_or(EOVERFLOW)?;
                let _change = inner.mapping_mutation();
                inner.page_table.discard_partial_map(start..end)?;
                inner.tlbi_context_mask(start, range as usize, contexts);
                Ok(())
            })?;
            {
                let Some(mut segments) = bo.data().segments.try_lock() else { return Ok(true); };
                *segments = None;
            }
            // Every private GPUVA alias was removed from the device page
            // tables above. Kernel mappings and other SG owners are excluded;
            // the C/Rust backing helper rechecks exclusion before release.
            if object.release_idle_pages_locked().is_ok() {
                crate::gem::set_mapping_held(object, false);
                unpinned = unpinned.saturating_add(object.size() / kernel::page::PAGE_SIZE);
            }
            Ok(unpinned < target)
        }) };
        drop(shared);
        drop(inner);

        purged + unpinned
    }

    fn restore_idle_pages(&self) -> Result {
        // Unbind/free can remove or split invalidated mappings while the VM
        // stays evicted. Rebuild solely from the authoritative logical GPUVAs,
        // never from stale pre-eviction records that could resurrect a BO.
        // Gate::enter holds exclusive residency ownership for this entire
        // walk: metadata edits, retirement and reclaim cannot change the tree.
        // Retain one invalidated mapping at a time, avoiding a VM-sized KVec
        // allocation precisely when memory pressure requires restoration.
        let mut cursor = 0u64;
        loop {
            let map = loop {
                if let Some(inner) = self.gpuvm.try_lock_private() {
                    let map = inner.next_private_mapping(
                        &mut cursor, GpuVaFlag::INVALIDATED.into(),
                    )?;
                    // Purged (DONTNEED) objects have no contents to restore;
                    // their mappings stay INVALIDATED until unbound.
                    // SAFETY: The private guard holds the VM reservation.
                    let purged = map.as_ref().is_some_and(|map| unsafe { map.object.madv_locked() < 0 });
                    break (map, purged);
                }
                // Wait for reservation ownership without retaining a mapping.
                drop(lock_vm(&self.gpuvm, None, true)?);
            };
            // The reservation guard has dropped before pinning backing, and
            // before any returned object reference can be released on failure.
            let (Some(map), purged) = map else { break; };
            if purged {
                continue;
            }
            let mut restored = 0u64;
            let bo_owner = lock_vm(&self.gpuvm, None, true)?
                .0
                .unique_gpuvm
                .find_bo(&map.object)
                .ok_or(EFAULT)?;
            {
                let needs_segments = bo_owner.data().segments.lock().is_none();
                if needs_segments {
                    // Preserve review1's reservation/BO lock order: the getter
                    // may acquire the object reservation; never hold BO here.
                    // SAFETY: As in `Vm::bind_object()`.
                    let dev = unsafe { self.dev.as_bound() };
                    let segments = prefetch_sgt_segments(dev, &map.object)?;
                    let mut bo = bo_owner.data().segments.lock();
                    if bo.is_none() { bo.replace(segments); }
                    drop(bo);
                }
            }
            let mut inner = lock_vm(&self.gpuvm, None, true)?;
            let bo = bo_owner.data().segments.lock();
            let sgt = bo.as_ref().ok_or(EFAULT)?;
            let result = (|| -> Result {
                let end = map.addr.checked_add(map.range).ok_or(EOVERFLOW)?;
                let _mutation = inner.mapping_mutation();
                inner.page_table.discard_partial_map(map.addr..end)?;
                inner.tlbi_contexts_naming_root(map.addr, map.range as usize);
                let prot = map.inner.prot.ok_or(EINVAL)?;
                let repeat = map.flags.contains(GpuVaFlag::REPEAT);
                let mut skip = map
                    .offset
                    .checked_add(if repeat { 0 } else { restored })
                    .ok_or(EOVERFLOW)? as usize;
                let mut left = map.range.checked_sub(restored).ok_or(EOVERFLOW)? as usize;
                for &(dma_address, dma_len) in sgt.iter() {
                    if left == 0 {
                        break;
                    }
                    let len = dma_len as usize;
                    if skip >= len {
                        skip -= len;
                        continue;
                    }
                    let physical = dma_address as usize + skip;
                    let bytes = if repeat { left } else { left.min(len - skip) };
                    let start = map.addr.checked_add(restored).ok_or(EOVERFLOW)?;
                    let end = start.checked_add(bytes as u64).ok_or(EOVERFLOW)?;
                    let _change = inner.mapping_mutation();
                    inner.page_table.map_pages(
                        start..end,
                        physical as PhysicalAddr,
                        prot,
                        repeat,
                    )?;
                    restored += bytes as u64;
                    left -= bytes;
                    skip = 0;
                }
                if left != 0 {
                    return Err(EFAULT);
                }
                // The exclusive residency gate excludes submissions until
                // every invalidated mapping is restored. Publish this logical
                // range once, instead of issuing a synchronized TLBI for each
                // physical SG segment. Keep the preceding break-before-make
                // invalidation; the error path clears and flushes the entire
                // range if any map operation (including a partial one) fails.
                inner.tlbi_contexts_naming_root(map.addr, map.range as usize);
                inner.set_invalidated(map.addr, map.range, false)?;

                Ok(())
            })();
            if let Err(error) = result {
                // Keep an invalidated GPUVA wholly unmapped even on partial
                // restoration failure, so later unbind/close cannot skip a
                // prefix that was already reinstalled. Backing stays retained.
                // map_pages may fail inside its current SG segment. Clear the
                // complete logical range, including that partially written segment.
                let end = map.addr.checked_add(map.range).ok_or(EOVERFLOW)?;
                let _mutation = inner.mapping_mutation();
                inner.page_table.discard_partial_map(map.addr..end)?;
                inner.tlbi_contexts_naming_root(map.addr, map.range as usize);
                return Err(error);
            }
        }
        Ok(())
    }

    pub(super) fn enter_retirement_residency(&self) -> Option<Lease> {
        self.residency.as_ref().and_then(Gate::enter_retirement)
    }
}
