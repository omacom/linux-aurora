// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Prepared userspace mapping transactions. Host callback resources are reserved for the
//! complete ioctl before its first visible mapping changes; leaf commits retain their
//! individual interruptible GPUVM reservation acquisition and quarantine admission check.

use super::*;

pub(crate) struct PreparedUserMap {
    gem: ARef<gem::Object>,
    vm_bo: ARef<gpuvm::GpuVmBo<VmInner>>,
    resources: VmOpResources,
    prot: Prot,
    addr: u64,
    size: u64,
    offset: u64,
    single_page: bool,
    context: context::PreparedBinding,
    _residency: Option<residency::Lease>,
    _sg_lease: kernel::drm::gem::shmem::SGLease<gem::AsahiObject>,
}

pub(crate) struct PreparedUserUnmap {
    _residency: Option<residency::Lease>,
    pub(super) iova: u64,
    pub(super) size: u64,
    pub(super) resources: VmOpResources,
}

// SAFETY: This owns a thread-safe residency lease, a numeric range and unused,
// unlinked split-node allocations; step_remap moves each node out of ctx before
// linking it, so a retained ctx never owns a linked GPUVA.
unsafe impl Send for PreparedUserUnmap {}

pub(super) enum PreparedUserBindOp {
    Map(PreparedUserMap),
    Unmap(PreparedUserUnmap),
}

pub(crate) struct PreparedUserBindBatch {
    pub(super) ops: KVec<PreparedUserBindOp>,
}

impl PreparedUserBindBatch {
    pub(crate) fn new(capacity: usize) -> Result<Self> {
        Ok(Self {
            ops: KVec::with_capacity(capacity, GFP_KERNEL)?,
        })
    }
    pub(crate) fn push_map(&mut self, map: PreparedUserMap) -> Result {
        Ok(self.ops.push(PreparedUserBindOp::Map(map), GFP_KERNEL)?)
    }
    pub(crate) fn push_unmap(&mut self, unmap: PreparedUserUnmap) -> Result {
        Ok(self
            .ops
            .push(PreparedUserBindOp::Unmap(unmap), GFP_KERNEL)?)
    }
}

impl Vm {
    /// Prepares host state before taking the GPUVM exec lock or changing any leaf.
    pub(crate) fn prepare_bind_object(
        &self,
        gem: &gem::Object,
        addr: u64,
        size: u64,
        offset: u64,
        prot: Prot,
        single_page: bool,
    ) -> Result<PreparedUserMap> {
        gem::validate_vm_binding(gem, self)?;
        let residency = self.enter_residency_metadata()?;
        let context = self.prepare_context_binding(addr, size, offset)?;
        let resources = VmOpResources {
            preallocated_gpuvas: [
                Some(GpuVaAlloc::<VmInner>::new(GFP_KERNEL)?),
                Some(GpuVaAlloc::<VmInner>::new(GFP_KERNEL)?),
                Some(GpuVaAlloc::<VmInner>::new(GFP_KERNEL)?),
            ],
        };
        // SAFETY: As in `Vm::bind_object()`, the device is bound while its file can issue binds.
        let dev = unsafe { self.dev.as_bound() };
        // Populate the scatter-gather table before any GPUVM lock is taken.
        let sg_lease = crate::gem::held_sg_table(gem, dev)?.into_lease();
        let vm_bo = self
            .gpuvm
            .obtain(gem, pin_init!(VmBoData {
                newest_bind: AtomicU64::new(0),
                segments <- new_mutex!(None),
            }))?;
        prepare_sgt_segments(dev, gem, &vm_bo, &sg_lease)?;
        if (addr | size | offset) & UAT_PGMSK as u64 != 0 {
            return Err(EINVAL);
        }
        Ok(PreparedUserMap {
            _residency: residency,
            _sg_lease: sg_lease,
            gem: gem.into(),
            vm_bo,
            resources,
            prot,
            addr,
            size,
            offset,
            single_page,
            context,
        })
    }

    pub(crate) fn prepare_user_unmap(&self, iova: u64, size: u64) -> Result<PreparedUserUnmap> {
        if (iova | size) & UAT_PGMSK as u64 != 0 || !self.gpuvm.range_valid(iova, size) {
            return Err(EINVAL);
        }
        let residency = self.enter_residency_metadata()?;
        Ok(PreparedUserUnmap {
            _residency: residency,
            iova,
            size,
            resources: VmOpResources::for_unmap()?,
        })
    }

    /// Commits in ioctl order. Quarantine may defer an all-unmap batch, but can
    /// never allow a map to replace a mapping still owned by failed work.
    pub(crate) fn commit_prepared_user_bind_batch(
        &self,
        batch: &mut PreparedUserBindBatch,
    ) -> Result {
        if self.defer_bind_batch(batch, 0)? {
            return Ok(());
        }
        let mut index = 0;
        while index < batch.ops.len() {
            let result = match &mut batch.ops[index] {
                PreparedUserBindOp::Map(map) => {
                    // SAFETY: See `lock_vm()`. As in `Vm::bind_object()`, the object itself is
                    // not locked: the map step may still populate its pages.
                    let mut inner = unsafe { self.gpuvm.exec_lock(None, true)? };
                    match self.mapping_commit() {
                        Ok(commit) => {
                            let (flags, repeat) = if map.single_page {
                                (GpuVaFlag::REPEAT.into(), UAT_PGSZ as u32)
                            } else {
                                (GpuVaFlags::empty(), 0)
                            };
                            // SAFETY: As in `Vm::bind_object()`.
                            let dev = unsafe { self.dev.as_bound() };
                            let mut ctx = StepContext::new(Some(dev), map.prot, &mut map.resources)?;
                            let result = inner.unique_gpuvm.sm_map(OpMapRequest {
                                addr: map.addr,
                                range: map.size,
                                gem_offset: map.offset,
                                gem_range: repeat,
                                vm_bo: &map.vm_bo,
                                context: &mut ctx,
                                flags,
                            });
                            let retired = result.is_ok().then(|| {
                                let retired = self.commit_context_binding(
                                    &map.gem,
                                    &map.context,
                                    ctx.bind_sequence,
                                );
                                self.track_shared_binding(
                                    &map.gem,
                                    map.addr,
                                    map.size,
                                    map.offset,
                                    map.single_page,
                                    ctx.bind_sequence,
                                );
                                retired
                            });
                            drop(inner);
                            drop(commit);
                            drop(retired);
                            Some(result)
                        }
                        Err(error) if error == EIO => {
                            drop(inner);
                            None
                        }
                        Err(error) => return Err(error),
                    }
                }
                PreparedUserBindOp::Unmap(unmap) => {
                    // SAFETY: See `lock_vm()`.
                    let mut inner = unsafe { self.gpuvm.exec_lock(None, true)? };
                    match self.mapping_commit() {
                        Ok(commit) => {
                            let mut ctx =
                                StepContext::new(None, Prot::default(), &mut unmap.resources)?;
                            let result =
                                inner
                                    .unique_gpuvm
                                    .sm_unmap(unmap.iova, unmap.size, &mut ctx);
                            let retired = result.is_ok().then(|| {
                                let retired = self.untrack_context_ranges(core::iter::once(
                                    unmap.iova..unmap.iova + unmap.size,
                                ));
                                self.untrack_shared_range(unmap.iova, unmap.size);
                                retired
                            });
                            drop(inner);
                            drop(commit);
                            drop(retired);
                            Some(result)
                        }
                        Err(error) if error == EIO => {
                            drop(inner);
                            None
                        }
                        Err(error) => return Err(error),
                    }
                }
            };
            if let Some(result) = result {
                result?;
                index += 1;
            } else if self.defer_bind_batch(batch, index)? {
                return Ok(());
            }
        }
        Ok(())
    }
}
