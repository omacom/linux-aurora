// SPDX-License-Identifier: GPL-2.0-only OR MIT
//! Private storage for one retired/unpublished G15 render pass. A Pass retains
//! all its buffers and mappings until Render retires the shared queues.
use kernel::prelude::*;
use crate::{driver,mmu,pgtable::prot,m3::m3_memory::Buffer,m3::m3_pass_layout as layout};
use layout::{Field,Space};

pub(crate) struct Pass {buffers:KVec<Buffer>}
impl Pass {
    pub(crate) fn new(dev:&driver::AsahiDrmDevice,uat:&mmu::Uat,vm:&mmu::Vm,
        slot:usize,clusters:u32,profile:super::m3_profile::Profile,aliases:&mut KVec<mmu::KernelMapping>)->Result<Self> {
        let mut buffers=KVec::with_capacity(layout::COUNT,GFP_KERNEL)?;
        for field in layout::FIELDS {
            let a=layout::allocation_for(profile,slot,field,clusters).map_err(|_|EINVAL)?;
            let mut b=match a.space {
                Space::ClientGpu=>Buffer::at_prot(dev,uat.kernel_vm(),Some((vm,a.address)),None,
                    a.size,prot::PROT_GPU_FW_SHARED_RW,prot::Prot::from_pte(a.access.pte()))?,
                Space::Firmware=>Buffer::at_prot(dev,uat.kernel_vm(),
                    a.gpu_alias.map(|address|(uat.kernel_lower_vm(),address)),Some(a.address),
                    a.size,prot::Prot::from_pte(a.access.pte()),prot::PROT_GPU_SHARED_RW)?,
            };
            if a.gpu_alias.is_some() {aliases.push(b.map_gpu_view(vm)?,GFP_KERNEL)?;}
            buffers.push(b,GFP_KERNEL)?;
        }
        Ok(Self{buffers})
    }
    pub(crate) fn get(&self,field:Field)->&Buffer {&self.buffers[field as usize]}
    pub(crate) fn get_mut(&mut self,field:Field)->&mut Buffer {&mut self.buffers[field as usize]}
    /// Caller must prove retirement, and must not clear a prepared sibling.
    /// Dependency commands are rewritten completely by the sync encoder.
    pub(crate) fn reset(&mut self,geometry:&crate::m3::agx_render::Geometry)->Result {
        // Tiler arenas grow to retain the largest prior target. The firmware
        // descriptors use the current geometry's strides and layer count, so
        // only that prefix is addressable by this pass. Clear it on every use,
        // including when growing again after a smaller pass; the unused tail
        // may still contain retired data and is never assumed to be zero.
        for field in &layout::FIELDS[..layout::RESET_COUNT] {
            let bytes=match field {
                Field::Tpc=>usize::try_from(geometry.tpc_bytes).map_err(|_|EOVERFLOW)?,
                Field::Tilemap=>usize::try_from(geometry.tilemap_bytes).map_err(|_|EOVERFLOW)?,
                _=>self.get(*field).size(),
            };
            self.get_mut(*field).fill_range(0,bytes,0)?;
        }
        Ok(())
    }
}
/// Cached client views retain their resource identity across switches. A pass
/// field cannot accidentally name a shared allocator or another pass's field.
#[derive(Clone,Copy)]
pub(crate) enum ViewOwner {Shared(usize),Pass{slot:usize,field:Field}}
