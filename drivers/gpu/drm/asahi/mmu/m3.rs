// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! The VM and binding helpers of the M3 T8122 runtime (`crate::m3`), which 7.1.12's `mmu.rs`
//! provided for it.

use super::*;

impl Vm {
    /// Whether `other` is the same DRM VM.
    pub(crate) fn same(&self, other: &Vm) -> bool {
        self.id == other.id
    }

    /// Copy the GPU-readable bytes this VM maps at `iova`, through the point of coherency
    /// ([`Vm::read_user`]). M3 compute validates the client control stream before publishing it.
    pub(crate) fn read_bytes(&self, iova: u64, out: &mut [u8]) -> Result {
        if out.is_empty() {
            return Err(EINVAL);
        }
        self.read_user(iova, out)
    }

    /// Snapshot the first `out.len()` TTBAT entries as `(ttb0, ttb1)` pairs. The TTBAT is
    /// ordinary memory, so this reads no GPU register and is safe with the cores gated.
    pub(crate) fn context_roots(&self, out: &mut [(u64, u64)]) -> Result {
        let inner = lock_vm(&self.gpuvm, None, false)?;
        let shared = inner.uat_inner.lock();
        let ttbs = shared.ttbs();
        for (slot, entry) in out.iter_mut().enumerate().take(UAT_NUM_CTX) {
            *entry = (
                ttbs[slot].ttb0.load(Ordering::Acquire),
                ttbs[slot].ttb1.load(Ordering::Acquire),
            );
        }
        Ok(())
    }

    /// Map `size` bytes of MMIO or DMA memory at `phys` into this Vm, at `iova`, or at a free
    /// address in `range` when `iova` is `None`. Unlike [`Vm::map_io`] this logs nothing on
    /// success: the runtime maps job views on every submission.
    fn map_io_m3_inner(
        &self,
        iova: Option<u64>,
        range: Range<u64>,
        phys: usize,
        size: usize,
        prot: Prot,
    ) -> Result<KernelMapping> {
        if size == 0 || (iova.unwrap_or(0) as usize | phys | size) & UAT_PGMSK != 0 {
            pr_err!(
                "MMU: M3 KernelMapping {:#x}:{:#x} -> {:#x?} is not page-aligned\n",
                phys,
                size,
                iova
            );
            return Err(EINVAL);
        }
        // SAFETY: As in `map_io`, the exec lock is only used for this mapping.
        let mut vm_exec = unsafe { self.gpuvm.exec_lock(None, false)? };
        let inner = vm_exec.unique_gpuvm.data();
        let payload = KernelMappingInner {
            owner: self.gpuvm.clone(),
            uat_inner: inner.uat_inner.clone(),
            prot,
            bo: None,
            _gem: None,
            offset: 0,
            mapped_size: size,
        };
        let node = match iova {
            Some(iova) => inner.mm.reserve_node(payload, iova, size as u64, 0)?,
            None => inner.mm.insert_node_in_range(
                payload,
                size as u64,
                UAT_PGSZ as u64,
                0,
                range.start,
                range.end,
                mm::InsertMode::Best,
            )?,
        };
        let iova = node.start();
        let end = iova.checked_add(size as u64).ok_or(EOVERFLOW)?;
        inner.page_table.prepare_map(iova..end)?;
        let mutation = inner.mapping_mutation();
        let ret = inner
            .page_table
            .map_pages(iova..end, phys as PhysicalAddr, prot, false);
        if ret.is_err() {
            inner.rollback_failed_map(iova, size as u64);
        }
        // Drop the exec lock before the node, as in `map_io`.
        drop(mutation);
        let _ = inner;
        ret?;
        Ok(KernelMapping(ManuallyDrop::new(node)))
    }

    /// [`Vm::map_io`] at `iova`, without the per-mapping log line.
    pub(crate) fn map_io_m3(
        &self,
        iova: u64,
        phys: usize,
        size: usize,
        prot: Prot,
    ) -> Result<KernelMapping> {
        self.map_io_m3_inner(Some(iova), 0..0, phys, size, prot)
    }

    /// [`Vm::map_io`] at a free address in `range`.
    pub(crate) fn map_io_in_range(
        &self,
        range: Range<u64>,
        phys: usize,
        size: usize,
        prot: Prot,
    ) -> Result<KernelMapping> {
        self.map_io_m3_inner(None, range, phys, size, prot)
    }
}

impl VmBind {
    /// The bound `Vm` itself, for diagnostics that need to walk its tables.
    pub(crate) fn vm(&self) -> &Vm {
        &self.0
    }

    /// Whether this retained binding belongs to the supplied DRM VM.
    pub(crate) fn matches(&self, vm: &Vm) -> bool {
        self.0.id == vm.id
    }

    /// Physical base of the bound VM's page-table root.
    pub(crate) fn root(&self) -> u64 {
        self.0.ttb()
    }

    /// The bound VM's driver-assigned id.
    pub(crate) fn vm_id(&self) -> u64 {
        self.0.id
    }
}
