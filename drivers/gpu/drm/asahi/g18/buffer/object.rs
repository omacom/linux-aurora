// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Kernel-owned GEM objects with fixed firmware-VA mappings (`MappedObject`).
//!
//! Items copied from the J873 module-1134 sources (provenance in
//! the port's extraction ledger), then adapted:
//! the type and the methods G18 calls are `pub(crate)` (in module 1134 they
//! were private to g17_resources, the parent of every G18 resource module);
//! the G17P firmware-cached object pool (fields `fw_pool`/`fw_node`, its
//! Drop path and `new_fw_pooled`) and the constructors and capture helpers
//! only G17P uses are not carried. G18 never creates pooled objects, so its
//! objects always took the non-pooled path, which is the one kept.

use core::ops::Range;

use kernel::prelude::*;

use super::live::g17p_live_write_u32;
use crate::{driver::AsahiDrmDevice, gem, mmu};

/// Zero a new kernel object through its cached CPU mapping.
fn zero_object(object: &mut gem::ObjectRef) -> Result {
    let size = object.size();
    let vmap = object.vmap()?;
    // SAFETY: the VMap covers the object's complete allocation of `size` bytes,
    // and the new object is not yet visible to the GPU or another CPU user.
    unsafe { core::ptr::write_bytes(vmap.addr.cast::<u8>(), 0, size) };
    Ok(())
}

pub(crate) struct MappedObject {
    /// Present for an owning GPU mapping and absent for a subview retained by
    /// another `MappedObject` in the same resource bundle.
    pub(crate) mapping: Option<mmu::KernelMapping>,
    pub(crate) object: gem::ObjectRef,
    pub(crate) iova: u64,
    pub(crate) object_offset: usize,
    pub(crate) logical_size: usize,
    /// Records the existing owning CPU mapping; capture never creates a
    /// physical alias or upgrades a WB observation to GPU-coherent evidence.
    pub(crate) cpu_wc: bool,
}

impl MappedObject {
    pub(crate) fn new_wc_in_range_with_guards(
        dev: &AsahiDrmDevice, vm: &mmu::Vm, range: Range<u64>, size: usize,
        alignment: u64, prot: mmu::Prot, guard_pages: usize,
    ) -> Result<Self> {
        let mut object = gem::new_kernel_object_wc(dev, size)?;
        zero_object(&mut object)?;
        let mapping = vm.map_in_range_with_guards(&object.gem, 0..object.size(),
            alignment, range, prot, guard_pages)?;
        let iova = mapping.iova();
        Ok(Self { mapping: Some(mapping), object, iova, object_offset: 0,
            logical_size: size, cpu_wc: true })
    }

    pub(crate) fn map_alias_at_with_guards(&mut self, vm: &mmu::Vm, address: u64,
        prot: mmu::Prot, guard_pages: usize) -> Result<mmu::KernelMapping> {
        if self.object_offset != 0 { return Err(EINVAL); }
        self.object.map_at_with_guards(vm, address, prot, guard_pages)
    }


    pub(crate) fn new_wc_in_range(
        dev: &AsahiDrmDevice,
        vm: &mmu::Vm,
        range: Range<u64>,
        size: usize,
        alignment: u64,
        prot: mmu::Prot,
    ) -> Result<Self> {
        let mut object = gem::new_kernel_object_wc(dev, size)?;
        zero_object(&mut object)?;
        let mapping = object.map_into_range(vm, range, alignment, prot, true)?;
        let iova = mapping.iova();
        Ok(Self {
            mapping: Some(mapping),
            object,
            iova,
            object_offset: 0,
            logical_size: size,
            cpu_wc: true,
        })
    }

    pub(crate) fn new_wc_at(
        dev: &AsahiDrmDevice,
        vm: &mmu::Vm,
        address: u64,
        size: usize,
        prot: mmu::Prot,
    ) -> Result<Self> {
        let mut object = gem::new_kernel_object_wc(dev, size)?;
        zero_object(&mut object)?;
        let mapping = object.map_at(vm, address, prot, false)?;
        let iova = mapping.iova();
        Ok(Self {
            mapping: Some(mapping),
            object,
            iova,
            object_offset: 0,
            logical_size: size,
            cpu_wc: true,
        })
    }

    /// As [`Self::new_wc_at`], for memory whose owner holds a user charge for
    /// it ([`crate::memacct::Charge::user`]) instead of counting it as kernel
    /// memory.
    pub(crate) fn new_wc_at_user_charged(
        dev: &AsahiDrmDevice,
        vm: &mmu::Vm,
        address: u64,
        size: usize,
        prot: mmu::Prot,
    ) -> Result<Self> {
        let mut object = gem::new_kernel_object_wc_user_charged(dev, size)?;
        zero_object(&mut object)?;
        let mapping = object.map_at(vm, address, prot, false)?;
        let iova = mapping.iova();
        Ok(Self {
            mapping: Some(mapping),
            object,
            iova,
            object_offset: 0,
            logical_size: size,
            cpu_wc: true,
        })
    }

    /// Replace this object's backing with a larger one at the SAME address.
    ///
    /// The replacement is allocated and zeroed before the old aperture is
    /// released, so an allocation failure leaves the caller's object intact
    /// and mapped. Only the caller can know that nothing is reading the old
    /// backing; this helper does not synchronize.
    pub(crate) fn regrow_wc_at(
        &mut self,
        dev: &AsahiDrmDevice,
        vm: &mmu::Vm,
        address: u64,
        size: usize,
    ) -> Result {
        if size <= self.logical_size {
            return Ok(());
        }
        let mut object = gem::new_kernel_object_wc(dev, size)?;
        zero_object(&mut object)?;
        self.mapping = None;
        let mapping = object.map_at(vm, address, mmu::PROT_GPU_FW_SHARED_RW, false)?;
        self.iova = mapping.iova();
        self.mapping = Some(mapping);
        self.object = object;
        self.object_offset = 0;
        self.logical_size = size;
        self.cpu_wc = true;
        Ok(())
    }

    pub(crate) fn iova(&self) -> u64 {
        self.iova
    }

    pub(crate) fn with_bytes_mut<R>(&mut self, f: impl FnOnce(&mut [u8]) -> Result<R>) -> Result<R> {
        let vmap = self.object.vmap()?;
        let bytes = unsafe {
            // SAFETY: the VMap covers this object's complete, page-rounded
            // allocation; construction bounds-checks the view offset and size.
            core::slice::from_raw_parts_mut(
                vmap.addr.cast::<u8>().add(self.object_offset),
                self.logical_size,
            )
        };
        f(bytes)
    }

    pub(crate) fn read_u32(&mut self, address: u64) -> Result<u32> {
        let offset = address.checked_sub(self.iova()).ok_or(EINVAL)? as usize;
        let end = offset.checked_add(4).ok_or(EINVAL)?;
        if end > self.logical_size {
            return Err(EINVAL);
        }
        self.with_bytes_mut(|bytes| {
            Ok(u32::from_le_bytes(
                bytes[offset..end].try_into().map_err(|_| EINVAL)?,
            ))
        })
    }

    pub(crate) fn read_u64(&mut self, address: u64) -> Result<u64> {
        let offset = address.checked_sub(self.iova()).ok_or(EINVAL)? as usize;
        let end = offset.checked_add(8).ok_or(EINVAL)?;
        if end > self.logical_size {
            return Err(EINVAL);
        }
        self.with_bytes_mut(|bytes| {
            Ok(u64::from_le_bytes(
                bytes[offset..end].try_into().map_err(|_| EINVAL)?,
            ))
        })
    }

    pub(crate) fn write_u32(&mut self, address: u64, value: u32) -> Result {
        let offset: usize = address.checked_sub(self.iova()).ok_or(EINVAL)?.try_into()?;
        self.with_bytes_mut(|bytes| g17p_live_write_u32(bytes, offset, value))
    }

    pub(crate) fn map_alias_at(
        &mut self,
        vm: &mmu::Vm,
        address: u64,
        prot: mmu::Prot,
    ) -> Result<mmu::KernelMapping> {
        if self.object_offset != 0 {
            return Err(EINVAL);
        }
        self.object.map_at(vm, address, prot, false)
    }

    pub(crate) fn map_alias_in_range(
        &mut self,
        vm: &mmu::Vm,
        range: Range<u64>,
        alignment: u64,
        prot: mmu::Prot,
    ) -> Result<mmu::KernelMapping> {
        if self.object_offset != 0 {
            return Err(EINVAL);
        }
        self.object
            .map_into_range(vm, range, alignment, prot, false)
    }

}
