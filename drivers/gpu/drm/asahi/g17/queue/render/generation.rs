// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Per-command tiling, fragment-status and auxiliary storage. Mappings precede
//! their backing owners; a generation is reusable only after paired retirement.

use crate::g17::{
    buffer::{self, BlockGeometry},
    fw::render::{storage, RenderPass},
    object::{Allocator, CpuMap, KernelObject, Placement},
};
use crate::{hw::t8140::{dynamic, resources as cfg}, mmu};
use kernel::prelude::*;

/// Offsets within a single client allocation, not fixed GPU addresses.
#[derive(Copy, Clone)]
pub(in crate::g17) struct Layout {
    pub(in crate::g17) bytes: usize,
    pub(in crate::g17) deflake: usize,
    pub(in crate::g17) status: usize,
    pub(in crate::g17) auxiliary: usize,
    pub(in crate::g17) viewport: usize,
    pub(in crate::g17) viewport_control: u32,
}
impl Layout {
    pub(in crate::g17) const NEO: Self = Self {
        bytes: storage::STATE_SIZE, deflake: storage::DEFLAKE,
        status: storage::TA_STATUS, auxiliary: storage::AUXILIARY,
        viewport: 0x900, viewport_control: 0xc00,
    };
    pub(in crate::g17) const M5: Self = Self {
        bytes: 0x10000, deflake: 0, status: 0x4000, auxiliary: 0x8000,
        viewport: 0x9c0, viewport_control: 0x3000,
    };

    pub(in crate::g17) fn prepare(self, state: &mut [u8], pass: &RenderPass) -> Result {
        if state.len() < self.bytes { return Err(EINVAL); }
        let viewport = storage::Viewport::new(pass.width, pass.height)?
            .with_control(self.viewport_control);
        let offset = self.deflake + self.viewport;
        let view = state.get_mut(offset..offset + size_of::<storage::Viewport>())
            .ok_or(EINVAL)?.as_mut_ptr().cast::<storage::Viewport>();
        if view.align_offset(align_of::<storage::Viewport>()) != 0 { return Err(EINVAL); }
        // SAFETY: The complete integer-only view is aligned and in bounds.
        // The caller exclusively owns this unpublished command generation.
        unsafe { viewport.apply(&mut *view) };
        state[self.auxiliary..self.auxiliary + storage::AUXILIARY_SIZE].fill(0);
        Ok(())
    }
}

enum Views {
    Neo {
        deflake: mmu::KernelMapping,
        auxiliary: mmu::KernelMapping,
        fragment_client: mmu::KernelMapping,
        fragment: KernelObject,
    },
    M5 { status_fw: mmu::KernelMapping },
}

pub(in crate::g17) struct Generation {
    views: Views,
    state: KernelObject,
    pub(super) ordinal: Option<u64>,
}
impl Generation {
    pub(in crate::g17) fn allocate_state(alloc: &Allocator<'_>, vm: &mmu::Vm) -> Result<KernelObject> {
        let (range, layout) = if alloc.uat.is_m5() {
            (BlockGeometry::M5_COMPACT, Layout::M5)
        } else { (dynamic::CLIENT_LOWER, Layout::NEO) };
        KernelObject::new(alloc.dev, vm, Placement::In(range, cfg::TVB_PAGE), layout.bytes,
            mmu::PROT_GPU_FW_SHARED_RW, CpuMap::WriteCombined)
    }

    #[inline(never)]
    pub(in crate::g17) fn new(alloc: &Allocator<'_>, vm: &mmu::Vm, state: KernelObject) -> Result<KBox<Self>> {
        let views = if alloc.uat.is_m5() {
            let layout = Layout::M5;
            let status_fw = state.map_range(alloc.uat.kernel_vm(),
                layout.status..layout.status + mmu::UAT_PGSZ,
                alloc.uat.kernel_dynamic_range(), mmu::UAT_PGSZ as u64,
                mmu::PROT_FW_SHARED_RW, 0)?;
            Views::M5 { status_fw }
        } else {
            let deflake = buffer::compact_alias(&state, vm,
                storage::DEFLAKE..storage::DEFLAKE + mmu::UAT_PGSZ,
                mmu::UAT_PGSZ as u64, mmu::PROT_GPU_FW_SHARED_RW, mmu::UAT_PGSZ)?;
            let auxiliary = state.map_range(vm,
                storage::AUXILIARY..storage::AUXILIARY + storage::AUXILIARY_SIZE,
                cfg::AUXILIARY, mmu::UAT_PGSZ as u64, mmu::PROT_GPU_FW_SHARED_RW, 0)?;
            let fragment = alloc.kernel(mmu::UAT_PGSZ, mmu::UAT_PGSZ as u64,
                mmu::PROT_GPU_FW_SHARED_RW, CpuMap::WriteCombined)?;
            let fragment_client = buffer::compact_alias(&fragment, vm, 0..mmu::UAT_PGSZ,
                mmu::UAT_PGSZ as u64, mmu::PROT_GPU_SHARED_RW, mmu::UAT_PGSZ)?;
            Views::Neo { deflake, auxiliary, fragment_client, fragment }
        };
        Ok(KBox::new(Self { views, state, ordinal: None }, GFP_KERNEL)?)
    }

    pub(in crate::g17) fn prepare(&mut self, ordinal: u64, pass: &RenderPass) -> Result {
        if self.ordinal.is_some_and(|old| old != ordinal) { return Err(EBUSY); }
        let layout = match &mut self.views {
            Views::Neo { fragment, .. } => {
                fragment.bytes_mut().fill(0);
                Layout::NEO
            }
            Views::M5 { .. } => {
                let layout = Layout::M5;
                self.state.bytes_mut()[layout.status..layout.status + mmu::UAT_PGSZ].fill(0);
                layout
            }
        };
        layout.prepare(self.state.bytes_mut(), pass)?;
        self.ordinal = Some(ordinal);
        Ok(())
    }

    pub(in crate::g17) fn addresses(&self) -> [u64; 5] {
        match &self.views {
            Views::Neo { deflake, auxiliary, fragment_client, fragment } => [
                deflake.iova(), self.state.gpu_va() + storage::TA_STATUS as u64,
                fragment_client.iova(), fragment.gpu_va(), auxiliary.iova(),
            ],
            Views::M5 { status_fw } => {
                let base = self.state.gpu_va();
                [base + Layout::M5.deflake as u64, base + Layout::M5.status as u64,
                    base + Layout::M5.status as u64 + 8, status_fw.iova() + 8,
                    base + Layout::M5.auxiliary as u64]
            }
        }
    }

    pub(in crate::g17) fn retire(&mut self, ordinal: u64) -> Result {
        if self.ordinal != Some(ordinal) { return Err(EIO); }
        self.ordinal = None;
        Ok(())
    }
}
