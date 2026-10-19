// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Keeps user mappings alive until accepted work has relinquished firmware ownership.
//!
//! The state lock is taken after GPUVM's exec lock when committing user mappings. Waiting for
//! that lock's commits never holds a GPUVM reservation. Draining takes no state lock while it
//! unmaps memory. Driver aliases retain only the inner GPUVM, so they create no cycle with Vm.

use super::*;
use kernel::{
    new_condvar,
    sync::{CondVar, CondVarTimeoutResult},
    time::msecs_to_jiffies,
};

const ADMISSION_WAIT_MS: u32 = 1000;

pub(super) struct DriverMappings {
    mappings: KVec<KernelMapping>,
    reserved: KVec<Range<u64>>,
    compute: Option<PoolMappings>,
    render: [Option<PoolMappings>; crate::hw::t8140::queues::RENDER_SLOTS],
}

struct PoolMappings {
    id: u64,
    blocks: usize,
    mappings: KVec<KernelMapping>,
}

impl DriverMappings {
    fn ranges(&self) -> impl Iterator<Item = Range<u64>> + '_ {
        Iterator::chain(
            Iterator::chain(
                self.mappings.iter(),
                Iterator::chain(
                    self.compute.iter().flat_map(|pool| pool.mappings.iter()),
                    self.render
                        .iter()
                        .flatten()
                        .flat_map(|pool| pool.mappings.iter()),
                ),
            )
            .map(KernelMapping::iova_range),
            self.reserved.iter().cloned(),
        )
    }
}

struct State {
    active: usize,
    commits: usize,
    blocked: bool,
    quarantined: bool,
    draining: bool,
    closed: bool,
    cleanup_failed: bool,
    close_ranges: Option<(Range<u64>, Range<u64>)>,
    unmaps: KVec<super::bind::PreparedUserUnmap>,
    objects: KVec<ARef<gem::Object>>,
}

#[pin_data]
pub(super) struct VmLifetime {
    #[pin]
    state: Mutex<State>,
    #[pin]
    changed: CondVar,
    #[pin]
    driver: Mutex<Option<DriverMappings>>,
}

impl VmLifetime {
    pub(super) fn can_reclaim(&self) -> bool {
        self.state.try_lock().is_some_and(|state| {
            state.active == 0 && state.commits == 0
                && !state.blocked && !state.quarantined && !state.draining
                && !state.closed && !state.cleanup_failed
        })
    }

    pub(super) fn new() -> Result<Arc<Self>> {
        Arc::pin_init(
            pin_init!(Self {
                state <- new_mutex!(State { active:0, commits:0, blocked:false,
                    quarantined:false, draining:false, closed:false, cleanup_failed:false, close_ranges:None,
                    unmaps:KVec::new(), objects:KVec::new() }, "VM job lifetime"),
                changed <- new_condvar!("VM mapping admission"),
                driver <- new_mutex!(None, "VM driver mappings"),
            }),
            GFP_KERNEL,
        )
    }
}

/// Retains a VM's mappings through a command's exact retirement. An error fence is not
/// retirement: potentially visible work must retain its guard through quarantine or stop.
pub(crate) struct VmJobGuard {
    vm: Vm,
    _residency: Option<residency::Lease>,
}

impl VmJobGuard {
    /// Blocks mapping commits before the error fence can become visible to userspace.
    pub(crate) fn quarantine(&self) {
        if let Some(lifetime) = self.vm.lifetime.as_ref() {
            let mut state = lifetime.state.lock();
            state.blocked = true;
            while state.commits != 0 {
                lifetime.changed.wait(&mut state);
            }
            state.quarantined = true;
        }
    }
}

pub(super) struct MappingCommit {
    lifetime: Arc<VmLifetime>,
}
impl Drop for MappingCommit {
    fn drop(&mut self) {
        let mut state = self.lifetime.state.lock();
        state.commits -= 1;
        if state.commits == 0 {
            self.lifetime.changed.notify_all();
        }
    }
}

impl Vm {
    /// Attaches aliases while constructing this VM, before exposing it to userspace.
    pub(crate) fn install_driver_mappings(
        &self,
        mappings: KVec<KernelMapping>,
        reserved: KVec<Range<u64>>,
    ) -> Result {
        // A tracked VM of a handoff UAT (G15) keeps its aliases in the lifetime.
        if !self.published() && self.lifetime.is_none() {
            return self.m3_install_driver_mappings(mappings, reserved);
        }
        let lifetime = self.lifetime.as_ref().ok_or(EINVAL)?;
        let mut driver = lifetime.driver.lock();
        if driver.is_some() {
            return Err(EBUSY);
        }
        *driver = Some(DriverMappings {
            mappings,
            reserved,
            compute: None,
            render: core::array::from_fn(|_| None),
        });
        Ok(())
    }

    /// Reserves the complete append before moving aliases into VM ownership.
    pub(crate) fn append_driver_mappings(&self, mappings: KVec<KernelMapping>) -> Result {
        let lifetime = self.lifetime.as_ref().ok_or(EINVAL)?;
        let mut driver = lifetime.driver.lock();
        let driver = Option::as_mut(&mut *driver).ok_or(EINVAL)?;
        driver.mappings.reserve(mappings.len(), GFP_KERNEL)?;
        for mapping in mappings {
            driver.mappings.push(mapping, GFP_KERNEL)?;
        }
        Ok(())
    }

    pub(crate) fn render_pool_tvb_blocks(&self, slot: u8, id: u64) -> Option<usize> {
        if id == 0 {
            return None;
        }
        self.lifetime
            .as_ref()?
            .driver
            .lock()
            .as_ref()?
            .render
            .get(usize::from(slot))?
            .as_ref()
            .filter(|pool| pool.id == id)
            .map(|pool| pool.blocks)
    }

    /// Retains a new generation after all references to a replaced pool retire.
    pub(crate) fn install_render_pool_mappings(
        &self,
        slot: u8,
        id: u64,
        blocks: usize,
        mappings: KVec<KernelMapping>,
    ) -> Result {
        if id == 0 {
            return Err(EINVAL);
        }
        let lifetime = self.lifetime.as_ref().ok_or(EINVAL)?;
        let mut driver = lifetime.driver.lock();
        let driver = Option::as_mut(&mut *driver).ok_or(EINVAL)?;
        let pool = driver.render.get_mut(usize::from(slot)).ok_or(EINVAL)?;
        if pool.is_some() {
            return Err(EBUSY);
        }
        *pool = Some(PoolMappings {
            id,
            blocks,
            mappings,
        });
        Ok(())
    }

    /// Consumes aliases only after the exact cached owner and complete append
    /// capacity have been checked. Failed preparation retains its mappings for
    /// destruction after the caller releases the device mutex.
    pub(crate) fn append_render_pool_mappings_prepared(
        &self,
        slot: u8,
        id: u64,
        previous: usize,
        blocks: usize,
        mappings: &mut KVec<KernelMapping>,
    ) -> Result {
        if id == 0 || blocks <= previous || mappings.len() != blocks - previous {
            return Err(EINVAL);
        }
        let lifetime = self.lifetime.as_ref().ok_or(EINVAL)?;
        let mut driver = lifetime.driver.lock();
        let pool = Option::as_mut(&mut *driver)
            .and_then(|driver| driver.render.get_mut(usize::from(slot)))
            .and_then(Option::as_mut)
            .filter(|pool| pool.id == id)
            .ok_or(EINVAL)?;
        if pool.blocks != previous {
            return Err(EBUSY);
        }
        pool.mappings
            .len()
            .checked_add(mappings.len())
            .ok_or(EOVERFLOW)?;
        pool.mappings.reserve(mappings.len(), GFP_KERNEL)?;
        for mapping in mappings.drain_all() {
            pool.mappings.push(mapping, GFP_KERNEL)?;
        }
        pool.blocks = blocks;
        Ok(())
    }

    /// Moves the aliases of the last `blocks - kept` heap blocks of the exact
    /// cached owner into `out`, which the caller drops after its locks. The
    /// growth path appends one alias per added block, so these are the
    /// blocks a shrink removed. Nothing moves unless all of them fit in `out`.
    pub(crate) fn truncate_render_pool_mappings(
        &self,
        slot: u8,
        id: u64,
        blocks: usize,
        kept: usize,
        out: &mut KVec<KernelMapping>,
    ) -> Result {
        let count = blocks.checked_sub(kept).ok_or(EINVAL)?;
        if id == 0 || count == 0 || out.capacity() - out.len() < count {
            return Err(EINVAL);
        }
        let lifetime = self.lifetime.as_ref().ok_or(EINVAL)?;
        let mut driver = lifetime.driver.lock();
        let pool = Option::as_mut(&mut *driver)
            .and_then(|driver| driver.render.get_mut(usize::from(slot)))
            .and_then(Option::as_mut)
            .filter(|pool| pool.id == id)
            .ok_or(EINVAL)?;
        if pool.blocks != blocks || pool.mappings.len() < count {
            return Err(EBUSY);
        }
        for _ in 0..count {
            let mapping = pool.mappings.pop().ok_or(EIO)?;
            out.push_within_capacity(mapping).map_err(|_| EIO)?;
        }
        pool.blocks = kept;
        Ok(())
    }

    /// A delayed teardown must not remove a successor generation. Remove
    /// under the cache lock, then unmap after dropping that lock.
    pub(crate) fn clear_render_pool_mappings_if_owner(&self, slot: u8, id: u64) {
        if id == 0 {
            return;
        }
        let previous = self.lifetime.as_ref().and_then(|lifetime| {
            let mut driver = lifetime.driver.lock();
            Option::as_mut(&mut *driver)
                .and_then(|driver| driver.render.get_mut(usize::from(slot)))
                .filter(|pool| pool.as_ref().is_some_and(|pool| pool.id == id))
                .and_then(Option::take)
        });
        drop(previous);
    }

    pub(crate) fn compute_pool_mappings_match(&self, id: u64) -> bool {
        id != 0
            && self.lifetime.as_ref().is_some_and(|lifetime| {
                lifetime
                    .driver
                    .lock()
                    .as_ref()
                    .is_some_and(|driver| driver.compute.as_ref().is_some_and(|pool| pool.id == id))
            })
    }

    /// The device publication lock serializes first-use alias construction.
    /// Never replace a live generation; its jobs must retire before clearing.
    pub(crate) fn install_compute_pool_mappings(
        &self,
        id: u64,
        mappings: KVec<KernelMapping>,
    ) -> Result {
        if id == 0 {
            return Err(EINVAL);
        }
        let lifetime = self.lifetime.as_ref().ok_or(EINVAL)?;
        let mut driver = lifetime.driver.lock();
        let driver = Option::as_mut(&mut *driver).ok_or(EINVAL)?;
        if driver.compute.is_some() {
            return Err(EBUSY);
        }
        driver.compute = Some(PoolMappings {
            id,
            blocks: 0,
            mappings,
        });
        Ok(())
    }

    /// Called under the publication lock after all obsolete pool work retires.
    /// Drop outside the driver lock because mapping teardown acquires GPUVM locks.
    pub(crate) fn clear_compute_pool_mappings(&self) {
        let previous = self.lifetime.as_ref().and_then(|lifetime| {
            let mut driver = lifetime.driver.lock();
            Option::as_mut(&mut *driver).and_then(|driver| driver.compute.take())
        });
        drop(previous);
    }

    /// Whether the client's handle of this VM is gone ([`Self::unmap_user_ranges`] closed
    /// admission for good). G15 releases its per-VM state after that.
    pub(crate) fn is_closed(&self) -> bool {
        self.lifetime
            .as_ref()
            .is_some_and(|lifetime| lifetime.state.lock().closed)
    }

    /// Tests mappings and reservations which userspace must never replace.
    pub(crate) fn driver_range_overlaps(&self, range: Range<u64>) -> bool {
        // A tracked VM of a handoff UAT (G15) keeps its reservations in the lifetime.
        if !self.published() && self.lifetime.is_none() {
            return self.m3_driver_range_overlaps(range);
        }
        let Some(lifetime) = self.lifetime.as_ref() else {
            return false;
        };
        let driver = lifetime.driver.lock();
        driver.as_ref().is_some_and(|driver| {
            driver
                .ranges()
                .any(|reserved| range.start < reserved.end && reserved.start < range.end)
        })
    }

    /// Waits before reservation acquisition, allowing healthy retirement to drain.
    pub(crate) fn wait_for_user_map_admission(&self) -> Result {
        let Some(lifetime) = self.lifetime.as_ref() else {
            return Ok(());
        };
        let mut state = lifetime.state.lock();
        let mut remaining = msecs_to_jiffies(ADMISSION_WAIT_MS);
        loop {
            let failed = self.status.as_ref().is_some_and(|status| status.get() != 0);
            if !failed
                && !state.closed
                && (state.blocked || (state.active != 0 && state.quarantined))
                && remaining != 0
            {
                remaining = match lifetime
                    .changed
                    .wait_interruptible_timeout(&mut state, remaining)
                {
                    CondVarTimeoutResult::Signal { .. } => return Err(ERESTARTSYS),
                    CondVarTimeoutResult::Timeout => 0,
                    CondVarTimeoutResult::Woken { jiffies } => jiffies,
                };
                continue;
            }
            if state.closed {
                return Err(ENOENT);
            }
            if failed || state.blocked || (state.active != 0 && state.quarantined) {
                return Err(EIO);
            }
            if !state.draining {
                return Ok(());
            }
            if lifetime.changed.wait_interruptible(&mut state) {
                return Err(ERESTARTSYS);
            }
        }
    }

    /// Acquires a job reference without imposing a VM-wide limit on healthy work.
    pub(crate) fn retain_job(&self) -> Result<VmJobGuard> {
        let residency = self.enter_residency()?;
        let lifetime = self.lifetime.as_ref().ok_or(EINVAL)?;
        let mut state = lifetime.state.lock();
        loop {
            if state.closed {
                return Err(ENOENT);
            }
            if state.blocked {
                return Err(EIO);
            }
            if !state.draining {
                state.active = state.active.checked_add(1).ok_or(EOVERFLOW)?;
                return Ok(VmJobGuard { vm: self.clone(), _residency: residency });
            }
            lifetime.changed.wait(&mut state);
        }
    }

    /// The first job of a submission may wait for a clean VM's quarantine drain.
    pub(crate) fn retain_first_job(&self) -> Result<VmJobGuard> {
        match self.retain_job() {
            Err(error)
                if error == EIO && self.status.as_ref().is_some_and(|status| status.get() == 0) => {
            }
            result => return result,
        }
        let lifetime = self.lifetime.as_ref().ok_or(EINVAL)?;
        let mut remaining = msecs_to_jiffies(ADMISSION_WAIT_MS);
        let mut state = lifetime.state.lock();
        while state.blocked
            && remaining != 0
            && self.status.as_ref().is_some_and(|status| status.get() == 0)
        {
            remaining = match lifetime
                .changed
                .wait_interruptible_timeout(&mut state, remaining)
            {
                CondVarTimeoutResult::Signal { .. } => return Err(ERESTARTSYS),
                CondVarTimeoutResult::Timeout => 0,
                CondVarTimeoutResult::Woken { jiffies } => jiffies,
            };
        }
        drop(state);
        self.retain_job()
    }

    /// Called only after the GPUVM exec/reservation locks have been acquired.
    pub(super) fn mapping_commit(&self) -> Result<Option<MappingCommit>> {
        let Some(lifetime) = self.lifetime.as_ref() else {
            return Ok(None);
        };
        let mut state = lifetime.state.lock();
        if state.closed {
            return Err(ENOENT);
        }
        if state.blocked {
            return Err(EIO);
        }
        state.commits = state.commits.checked_add(1).ok_or(EOVERFLOW)?;
        Ok(Some(MappingCommit {
            lifetime: lifetime.clone(),
        }))
    }

    /// Reserves cleanup bookkeeping before a batch starts committing leaves.
    pub(crate) fn reserve_deferred_user_unmaps(&self, count: usize) -> Result {
        if let Some(lifetime) = self.lifetime.as_ref() {
            lifetime.state.lock().unmaps.reserve(count, GFP_KERNEL)?;
        }
        Ok(())
    }

    pub(super) fn defer_bind_batch(&self, batch: &mut PreparedUserBindBatch, start: usize) -> Result<bool> {
        use super::bind::PreparedUserBindOp;
        let Some(lifetime) = self.lifetime.as_ref() else {
            return Ok(false);
        };
        let mut state = lifetime.state.lock();
        if !state.blocked {
            return Ok(false);
        }
        if state.closed {
            return Err(ENOENT);
        }
        if batch
            .ops
            .iter().skip(start)
            .any(|op| matches!(op, PreparedUserBindOp::Map(_)))
        {
            return Err(EIO);
        }
        if !state.quarantined {
            return Err(EBUSY);
        }
        // Reserve before accepting any range; a partial deferred batch must
        // not leave canonical metadata ahead of the accepted cleanup.
        state.unmaps.reserve(batch.ops.len() - start, GFP_KERNEL)?;
        let retired = self.untrack_context_ranges(batch.ops.iter().skip(start).filter_map(|op| match op {
            PreparedUserBindOp::Unmap(unmap) => Some(unmap.iova..unmap.iova + unmap.size),
            PreparedUserBindOp::Map(_) => None,
        }));
        // Transfer the preallocated split nodes along with each accepted range.
        // Already-committed prefix operations must not be replayed: their nodes
        // may have been consumed and another binding may now occupy the range.
        for op in batch.ops.drain(start..) {
            if let PreparedUserBindOp::Unmap(unmap) = op {
                if !state.unmaps.iter().any(|old| old.iova == unmap.iova && old.size == unmap.size) {
                    state.unmaps.push(unmap, GFP_KERNEL)?;
                }
            }
        }
        drop(state);
        drop(retired);
        Ok(true)
    }

    /// Returns true if a quarantined owner requires this unmap to be deferred.
    pub(super) fn defer_unmap(&self, iova: u64, size: u64) -> Result<bool> {
        let Some(lifetime) = self.lifetime.as_ref() else {
            return Ok(false);
        };
        let state = lifetime.state.lock();
        if state.closed {
            return Err(ENOENT);
        }
        if !state.blocked {
            return Ok(false);
        }
        if !state.quarantined {
            return Err(EBUSY);
        }
        if state.unmaps.iter().any(|old| old.iova == iova && old.size == size) {
            return Ok(true);
        }
        drop(state);
        let unmap = self.prepare_user_unmap(iova, size)?;
        let mut state = lifetime.state.lock();
        if state.closed { return Err(ENOENT); }
        if !state.blocked { return Ok(false); }
        if !state.quarantined { return Err(EBUSY); }
        if !state.unmaps.iter().any(|old| old.iova == iova && old.size == size) {
            state.unmaps.push(unmap, GFP_KERNEL)?;
        }
        Ok(true)
    }

    /// Defers handle-close cleanup while any accepted job can still use the BO.
    pub(crate) fn drop_mappings(&self, gem: &gem::Object) -> Result {
        let Some(lifetime) = self.lifetime.as_ref() else {
            return self.drop_mappings_now(gem);
        };
        let mut state = lifetime.state.lock();
        if !state
            .objects
            .iter()
            .any(|object| core::ptr::eq(&**object, gem))
        {
            state.objects.push(gem.into(), GFP_KERNEL)?;
        }
        if state.active != 0 || state.draining {
            // Stop new jobs from selecting this accepted cleanup candidate.
            // The queued object retains its GEM while the candidate is removed.
            // Actual unmap repeats this under exec to order any intervening bind.
            self.untrack_context_object(gem);
            return Ok(());
        }
        state.draining = true;
        drop(state);
        self.drain_mappings(lifetime);
        Ok(())
    }

    /// Closes admission permanently and preserves mappings referenced by accepted jobs.
    pub(crate) fn unmap_user_ranges(&self, user: Range<u64>, kernel: Range<u64>) -> Result {
        // A tracked VM of a handoff UAT (G15) closes through its lifetime.
        if !self.published() && self.lifetime.is_none() {
            return self.m3_unmap_user_ranges(user, kernel);
        }
        if let Some(lifetime) = self.lifetime.as_ref() {
            let mut state = lifetime.state.lock();
            state.blocked = true;
            while state.commits != 0 {
                lifetime.changed.wait(&mut state);
            }
            state.closed = true;
            if state.active != 0 {
                state.close_ranges = Some((user, kernel));
                return Ok(());
            }
        }
        self.unmap_user_ranges_now(user, kernel)
    }

    fn unmap_user_ranges_now(&self, user: Range<u64>, _kernel: Range<u64>) -> Result {
        let _residency = self.enter_retirement_residency();
        // User mappings alone have GPUVA nodes. Kernel and driver aliases use
        // mm::Node and are untouched; GPUVM's embedded kernel cutout is skipped.
        // Closed admission and zero accepted jobs make whole-node teardown safe.
        let mut inner = self.inner.lock_inner();
        let result = inner.unmap_all(VmInner::unmap_gpuva);
        if result.is_ok() {
            self.untrack_shared_range(user.start, user.end - user.start);
        }
        drop(inner);
        // Final GEM/SG-table destruction may acquire reservations itself.
        self.bo_deferred_cleanup();
        result
    }
}

impl Drop for VmJobGuard {
    fn drop(&mut self) {
        let Some(lifetime) = self.vm.lifetime.as_ref() else {
            return;
        };
        let mut state = lifetime.state.lock();
        state.active -= 1;
        if state.active != 0 {
            return;
        }
        state.draining = true;
        drop(state);
        self.vm.drain_mappings(lifetime);
    }
}

impl Vm {
    /// Admission stays closed to new jobs while the exec lock is acquired for cleanup.
    /// Recheck queued cleanup and reopen under the same lock used to append it.
    fn drain_mappings(&self, lifetime: &VmLifetime) {
        let _residency = self.enter_retirement_residency();
        let mut state = lifetime.state.lock();
        let mut failure = None;
        loop {
            let close = state.close_ranges.take();
            let unmaps = core::mem::replace(&mut state.unmaps, KVec::new());
            let objects = core::mem::replace(&mut state.objects, KVec::new());
            drop(state);
            if let Some((user, kernel)) = close {
                if let Err(error) = self.unmap_user_ranges_now(user, kernel) {
                    failure.get_or_insert(error);
                    pr_err!("MMU: deferred VM close failed\n");
                }
            } else {
                for mut unmap in unmaps {
                    if let Err(error) = self.unmap_prepared_cleanup(&mut unmap) {
                        failure.get_or_insert(error);
                        pr_err!("MMU: deferred user unmap failed\n");
                    }
                }
            }
            for object in objects {
                if let Err(error) = self.drop_mappings_now(&object) {
                    failure.get_or_insert(error);
                    pr_err!("MMU: deferred object unmap failed\n");
                }
            }
            state = lifetime.state.lock();
            if state.close_ranges.is_some() || !state.unmaps.is_empty() || !state.objects.is_empty()
            {
                continue;
            }
            state.quarantined = false;
            state.draining = false;
            if let Some(error) = failure {
                // Cleanup is allocation-free for accepted ranges and has a
                // whole-node low-memory fallback for objects. An unexpected
                // invariant error must not reopen admission over stale leaves.
                // GPUVA owners retain backing until final VM close; report the
                // terminal failure instead of silently discarding its outcome.
                state.blocked = true;
                state.cleanup_failed = true;
                if let Some(status) = &self.status { status.record(error); }
            } else if !state.closed && !state.cleanup_failed {
                state.blocked = false;
            }
            drop(state);
            lifetime.changed.notify_all();
            break;
        }
        self.bo_deferred_cleanup();
    }
}
