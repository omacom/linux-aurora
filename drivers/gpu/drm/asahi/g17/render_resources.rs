// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Immutable linked-program storage, shared by commands in one submission.

use super::fw::m5::{
    pool::PoolGeometry,
    pool_budget::{History, RenderBudget, Sizes},
    render_demand::{ComputeStorage, FragmentStorage, RenderDemand, TileStorage, VertexStorage},
};
use super::fw::m5::compute::{ScratchRequest, IntersectionTables};
use crate::util::Reader;
use kernel::{prelude::*, uapi};

// SAFETY: The UAPI record contains only fixed-width unsigned integers.
unsafe impl crate::util::AnyBitPattern for uapi::drm_asahi_render_program {}
// SAFETY: The versioned record contains only unsigned integer storage.
unsafe impl crate::util::AnyBitPattern for uapi::drm_asahi_render_program_v2 {}

pub(crate) struct Programs {
    entries: KVec<uapi::drm_asahi_render_program_v2>,
}
impl Programs {
    pub(crate) fn parse(bytes: &[u8]) -> Result<Self> {
        let stride = size_of::<uapi::drm_asahi_render_program>();
        if bytes.is_empty() || bytes.len() % stride != 0 { return Err(EINVAL); }
        let mut entries = KVec::with_capacity(bytes.len() / stride, GFP_KERNEL)?;
        let mut reader = Reader::new(bytes);
        for _ in 0..bytes.len() / stride {
            let entry: uapi::drm_asahi_render_program = reader.read()?;
            Self::validate(&entry)?;
            entries.push(Self::wrap(entry), GFP_KERNEL)?;
        }
        Ok(Self { entries })
    }

    pub(crate) fn from_entries(values: &[uapi::drm_asahi_render_program]) -> Result<Self> {
        if values.is_empty() { return Err(EINVAL); }
        let mut entries = KVec::with_capacity(values.len(), GFP_KERNEL)?;
        for entry in values {
            Self::validate(entry)?;
            entries.push(Self::wrap(*entry), GFP_KERNEL)?;
        }
        Ok(Self { entries })
    }

    fn wrap(legacy: uapi::drm_asahi_render_program) -> uapi::drm_asahi_render_program_v2 {
        uapi::drm_asahi_render_program_v2 { legacy, threads: 0, usage_flags: 0,
            scs_layout: 0, auxiliary_allocation: 0, reserved: 0,
            intersection_pool: 0, intersection_pool_bytes: 0,
            intersection_first: 0, intersection_count: 0,
            intersection_default_program: 0, intersection_default_control: 0 }
    }

    pub(crate) fn parse_v2(bytes: &[u8]) -> Result<Self> {
        let stride = size_of::<uapi::drm_asahi_render_program_v2>();
        if bytes.is_empty() || bytes.len() % stride != 0 { return Err(EINVAL); }
        let mut entries = KVec::with_capacity(bytes.len() / stride, GFP_KERNEL)?;
        let mut reader = Reader::new(bytes);
        for _ in 0..bytes.len() / stride {
            let entry: uapi::drm_asahi_render_program_v2 = reader.read()?;
            let mesh = entry.usage_flags & (uapi::DRM_ASAHI_RENDER_PROGRAM_MESH
                | uapi::DRM_ASAHI_RENDER_PROGRAM_OBJECT) != 0;
            let task = entry.usage_flags & crate::file::DRM_ASAHI_RENDER_PROGRAM_TASK != 0;
            if entry.reserved != 0 || entry.usage_flags & !0xf != 0
                || entry.usage_flags & 6 == 6 || entry.legacy.flags != 0
                || (task && entry.usage_flags & uapi::DRM_ASAHI_RENDER_PROGRAM_OBJECT == 0)
                || entry.legacy.stage > 1 || (mesh && (entry.legacy.stage != 0
                    || entry.threads == 0 || entry.legacy.auxiliary_count != 0
                    || entry.legacy.imageblock_bytes != 0 || entry.legacy.multisample_bytes != 0))
                || (!mesh && entry.threads != 0) { return Err(EINVAL); }
            if !mesh { Self::validate(&entry.legacy)?; }
            let scratch = ScratchRequest::new(entry.scs_layout)?;
            let intersection = Self::intersection(&entry)?;
            if scratch.enabled() != (entry.usage_flags & uapi::DRM_ASAHI_RENDER_PROGRAM_SCS != 0)
                || scratch.enabled() != intersection.enabled()
                || (!scratch.enabled() && (entry.scs_layout != 0 || entry.auxiliary_allocation != 0)) {
                return Err(EINVAL);
            }
            entries.push(entry, GFP_KERNEL)?;
        }
        let programs = Self { entries };
        // A render kick has one SCS layout and one intersection-table binding
        // per engine, even when several linked programs contribute demand.
        programs.stage_scs()?;
        Ok(programs)
    }

    fn intersection(e: &uapi::drm_asahi_render_program_v2) -> Result<IntersectionTables> {
        IntersectionTables::new(e.intersection_pool, e.intersection_pool_bytes,
            e.intersection_first, e.intersection_count,
            e.intersection_default_program, e.intersection_default_control)
    }

    /// Any linked program runs as a hardware mesh or object (task) stage.
    pub(crate) fn uses_mesh(&self) -> bool {
        self.entries.iter().any(|e| e.usage_flags
            & (uapi::DRM_ASAHI_RENDER_PROGRAM_MESH | uapi::DRM_ASAHI_RENDER_PROGRAM_OBJECT) != 0)
    }

    /// An API task (object) stage is linked. Only such kicks get the TA mesh
    /// rendering buffer: only the object→mesh TA program binds it, never the
    /// mesh-single, grid or indirect ones.
    pub(crate) fn uses_task(&self) -> bool {
        self.entries.iter().any(|e| e.usage_flags & crate::file::DRM_ASAHI_RENDER_PROGRAM_TASK != 0)
    }

    /// 0 ordinary, 1 hardware mesh (including the mesh-only passthrough object
    /// stage), 2 API task (object) → mesh.
    pub(crate) fn mesh_class(&self) -> u8 {
        if self.uses_task() { 2 }
        else if self.uses_mesh() { 1 }
        else { 0 }
    }

    pub(crate) fn stage_scs(&self) -> Result<[StageScratch; 2]> {
        let mut stages = [StageScratch::NONE; 2];
        for e in &self.entries {
            if e.scs_layout == 0 { continue; }
            let next = StageScratch { scratch: ScratchRequest::new(e.scs_layout)?,
                intersection: Self::intersection(e)?, auxiliary_allocation: e.auxiliary_allocation };
            let out = &mut stages[e.legacy.stage as usize];
            if out.scratch.enabled() && *out != next { return Err(EINVAL); }
            *out = next;
        }
        Ok(stages)
    }

    fn validate(entry: &uapi::drm_asahi_render_program) -> Result {
        if entry.stage > 1 || entry.flags != 0
            || (entry.stage == 0 && (entry.auxiliary_count != 0
                || entry.threadgroup_bytes != 0 || entry.imageblock_bytes != 0
                || entry.multisample_bytes != 0)) { return Err(EINVAL); }
        Ok(())
    }

    fn demands(&self, mut demand: RenderDemand) -> Result<(RenderDemand, [RenderDemand; 2])> {
        let mut stages = [demand; 2];
        stages[0].include_vertex(VertexStorage::default()).map_err(|_| EINVAL)?;
        let empty = FragmentStorage::default().parameters().map_err(|_| EINVAL)?;
        let no_tile = TileStorage::default().parameters(false).map_err(|_| EINVAL)?;
        stages[1].include_fragment(empty, no_tile).map_err(|_| EINVAL)?;
        demand.include_vertex(VertexStorage::default()).map_err(|_| EINVAL)?;
        demand.include_fragment(empty, no_tile).map_err(|_| EINVAL)?;
        for e in &self.entries {
            let entry = &e.legacy;
            if entry.stage == 0 {
                let storage = VertexStorage {
                    primary_bytes: entry.primary_bytes,
                    secondary_bytes: entry.secondary_bytes,
                    linked_bytes: entry.linked_bytes,
                    minimum_primary_bytes: entry.minimum_primary_bytes,
                };
                if e.usage_flags & 6 != 0 {
                    let storage = ComputeStorage {
                        primary_bytes: storage.primary_bytes, secondary_bytes: storage.secondary_bytes,
                        linked_bytes: storage.linked_bytes, minimum_primary_bytes: storage.minimum_primary_bytes,
                        threadgroup_bytes: entry.threadgroup_bytes, threads: e.threads,
                    };
                    demand.include_mesh(storage, 96, 114688).map_err(|_| EINVAL)?;
                    stages[0].include_mesh(storage, 96, 114688).map_err(|_| EINVAL)?;
                } else {
                    demand.include_vertex(storage).map_err(|_| EINVAL)?;
                    stages[0].include_vertex(storage).map_err(|_| EINVAL)?;
                }
            } else {
                let fragment = FragmentStorage {
                    primary_bytes: entry.primary_bytes,
                    secondary_bytes: entry.secondary_bytes,
                    linked_bytes: entry.linked_bytes,
                    minimum_primary_bytes: entry.minimum_primary_bytes,
                    auxiliary_count: entry.auxiliary_count,
                }.parameters().map_err(|_| EINVAL)?;
                let tile = TileStorage {
                    threadgroup_bytes: entry.threadgroup_bytes,
                    imageblock_bytes: entry.imageblock_bytes,
                    multisample_bytes: entry.multisample_bytes,
                }.parameters(false).map_err(|_| EINVAL)?;
                demand.include_fragment(fragment, tile).map_err(|_| EINVAL)?;
                stages[1].include_fragment(fragment, tile).map_err(|_| EINVAL)?;
            }
        }
        Ok((demand, stages))
    }
}

/// Frozen device topology and shared-queue policy. No per-command allocation.
#[derive(Clone, Copy)]
pub(crate) struct M5Policy {
    demand: RenderDemand,
    budgets: [RenderBudget; 2],
    compute_budgets: [RenderBudget; 2],
}

#[derive(Clone, Copy)]
pub(crate) struct Plan {
    pub(crate) sizes: Sizes,
    pub(crate) geometry: PoolGeometry,
    pub(crate) resource_flags: u64,
    pub(crate) pressure: u32,
    pub(crate) stages: Option<[StagePlan; 2]>,
}

#[derive(Clone, Copy)]
pub(crate) struct StagePlan {
    pub(crate) scs: StageScratch,
    pub(crate) resource_flags: u64,
    pub(crate) ptc_max: u32,
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) struct StageScratch {
    pub(crate) scratch: ScratchRequest,
    pub(crate) intersection: IntersectionTables,
    pub(crate) auxiliary_allocation: u32,
}
impl StageScratch {
    pub(crate) const NONE: Self = Self { scratch: ScratchRequest::NONE,
        intersection: IntersectionTables::NONE, auxiliary_allocation: 0 };
}

impl M5Policy {
    pub(crate) fn new(id: &crate::regs::G17Id) -> Result<Self> {
        // Configured G17S context storage per enabled core. This is separate
        // from the final per-kick PTC cap selected during command preparation.
        const CONTEXT_FLOOR: u32 = 96;
        let nominal = id.num_cores.checked_mul(id.num_clusters).ok_or(EOVERFLOW)?;
        Ok(Self {
            demand: RenderDemand::new(&id.core_masks, nominal, CONTEXT_FLOOR)
                .map_err(|_| EINVAL)?,
            compute_budgets: [
                RenderBudget::new(&id.core_masks, CONTEXT_FLOOR, 3).map_err(|_| EINVAL)?,
                RenderBudget::new(&id.core_masks, CONTEXT_FLOOR, 2).map_err(|_| EINVAL)?,
            ],
            budgets: [
                RenderBudget::new(&id.core_masks, CONTEXT_FLOOR, 3).map_err(|_| EINVAL)?,
                RenderBudget::new(&id.core_masks, CONTEXT_FLOOR, 4).map_err(|_| EINVAL)?,
            ],
        })
    }

    pub(crate) fn plan_compute(&self, programs: &ComputePrograms, priority: u8) -> Result<Plan> {
        self.plan_compute_scs(programs, priority, 0, 0)
    }

    pub(crate) fn plan_compute_scs(&self, programs: &ComputePrograms, priority: u8,
        scs_layout: u64, auxiliary_pages: u32) -> Result<Plan> {
        let scratch = super::fw::m5::compute::ScratchRequest::new(scs_layout)?;
        // UMA pool class 1 is realtime; class 2 covers all other
        // public priorities. Its resident-kick count also depends on engine.
        let (budget, active_kicks) = match priority {
            0 => (&self.compute_budgets[0], 3),
            1..=3 => (&self.compute_budgets[1], 2),
            _ => return Err(EINVAL),
        };
        let mut demand = self.demand;
        for entry in &programs.entries {
            demand.include_compute(ComputeStorage {
                primary_bytes: entry.primary_bytes,
                secondary_bytes: entry.secondary_bytes,
                linked_bytes: entry.linked_bytes,
                minimum_primary_bytes: entry.minimum_primary_bytes,
                threadgroup_bytes: entry.threadgroup_bytes,
                threads: entry.threads,
            }, 96, 114688).map_err(|_| EINVAL)?;
        }
        let request = budget.compute_scs_requirements(
            demand.requirements().map_err(|_| EINVAL)?, scratch.pages(), auxiliary_pages)
            .map_err(|_| EINVAL)?;
        let sizes = budget.sizes(request, History::default()).map_err(|_| EINVAL)?;
        Ok(Plan {
            geometry: sizes.geometry().map_err(|_| EINVAL)?, sizes,
            resource_flags: demand.resource_flags().map_err(|_| EINVAL)?,
            pressure: (demand.pressure() / active_kicks).clamp(96, 512),
            stages: None,
        })
    }

    pub(crate) fn plan(&self, programs: &Programs, priority: u8) -> Result<Plan> {
        let (budget, active_kicks) = match priority { 0 => (&self.budgets[0], 3), 1..=3 => (&self.budgets[1], 4),
            _ => return Err(EINVAL) };
        let (demand, stage_demands) = programs.demands(self.demand)?;
        let scs = programs.stage_scs()?;
        let stage = |d: RenderDemand, scs| -> Result<StagePlan> { Ok(StagePlan {
            scs,
            resource_flags: d.resource_flags().map_err(|_| EINVAL)?,
            ptc_max: (d.pressure() / active_kicks).clamp(96, 512),
        }) };
        let request = budget.render_scs_requirements(
            demand.requirements().map_err(|_| EINVAL)?,
            scs[0].scratch.pages().max(scs[1].scratch.pages()),
            scs[0].auxiliary_allocation.max(scs[1].auxiliary_allocation)).map_err(|_| EINVAL)?;
        let sizes = budget.sizes(request, History::default())
            .map_err(|_| EINVAL)?;
        Ok(Plan {
            geometry: sizes.geometry().map_err(|_| EINVAL)?, sizes,
            resource_flags: demand.resource_flags().map_err(|_| EINVAL)?,
            pressure: demand.pressure(),
            stages: Some([stage(stage_demands[0], scs[0])?, stage(stage_demands[1], scs[1])?]),
        })
    }
}

// SAFETY: The record contains only fixed-width unsigned integers.
unsafe impl crate::util::AnyBitPattern for uapi::drm_asahi_compute_program {}

pub(crate) struct ComputePrograms {
    entries: KVec<uapi::drm_asahi_compute_program>,
}
impl ComputePrograms {
    pub(crate) fn parse(bytes: &[u8]) -> Result<Self> {
        let stride = size_of::<uapi::drm_asahi_compute_program>();
        if bytes.is_empty() || bytes.len() % stride != 0 { return Err(EINVAL); }
        let mut entries = KVec::with_capacity(bytes.len() / stride, GFP_KERNEL)?;
        let mut reader = Reader::new(bytes);
        for _ in 0..bytes.len() / stride {
            let entry: uapi::drm_asahi_compute_program = reader.read()?;
            if entry.flags != 0 || entry.threads == 0 { return Err(EINVAL); }
            entries.push(entry, GFP_KERNEL)?;
        }
        Ok(Self { entries })
    }
}
