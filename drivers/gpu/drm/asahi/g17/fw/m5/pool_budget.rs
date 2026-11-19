// SPDX-License-Identifier: GPL-2.0-only OR MIT
//! G17S shared render private-memory demand. Counts describe hardware and
//! simultaneous work; byte requirements come from the submitted programs.

use super::pool::{render_block_bytes, PoolError, PoolGeometry};

#[derive(Clone, Copy)]
pub(crate) struct RenderBudget {
    active_cores: u32,
    constant_pages: u32,
    active_kicks: u32,
}

#[derive(Clone, Copy, Default)]
pub(crate) struct Requirements {
    pub(crate) minimum_bytes: u64,
    pub(crate) ideal_bytes: u64,
    pub(crate) self_reserve_pages: u32,
    pub(crate) render_target_pages: u32,
    pub(crate) has_spill: bool,
}

/// Retained high-water marks, excluding per-kick reservations from the two
/// base byte counts. They may increase demand but never replace live demand.
#[derive(Clone, Copy, Default)]
pub(crate) struct History {
    pub(crate) minimum_base: u64,
    pub(crate) ideal_base: u64,
    pub(crate) self_reserve: u64,
    pub(crate) render_target: u64,
    pub(crate) minimum_pool: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct Sizes {
    pub(crate) minimum: u64,
    pub(crate) ideal: u64,
}

fn add(a: u64, b: u64) -> Result<u64, PoolError> {
    a.checked_add(b).ok_or(PoolError::Geometry)
}
fn mul(a: u64, b: u64) -> Result<u64, PoolError> {
    a.checked_mul(b).ok_or(PoolError::Geometry)
}
fn aligned(bytes: u64) -> Result<u64, PoolError> {
    Ok(add(bytes, render_block_bytes() as u64 - 1)?
        & !(render_block_bytes() as u64 - 1))
}

impl RenderBudget {
    /// `core_masks` contains the enabled cores of each physical cluster.
    /// `ptc_floor` is the configured per-core context count; `active_kicks`
    /// includes simultaneously resident tiling and fragment work.
    pub(crate) fn new(core_masks: &[u32], ptc_floor: u32, active_kicks: u32)
        -> Result<Self, PoolError>
    {
        let mut active_cores = 0u32;
        let mut cluster_cores = 0u32;
        for mask in core_masks {
            let count = mask.count_ones();
            active_cores = active_cores.checked_add(count).ok_or(PoolError::Geometry)?;
            cluster_cores = cluster_cores.max(count);
        }
        if active_cores == 0 || active_kicks == 0 || ptc_floor == 0 {
            return Err(PoolError::Geometry);
        }
        let constant_pages = cluster_cores.checked_mul(ptc_floor)
            .and_then(|n| n.checked_add(32))
            .and_then(|n| n.checked_mul(u32::try_from(core_masks.len()).ok()?))
            .ok_or(PoolError::Geometry)?;
        Ok(Self { active_cores, constant_pages, active_kicks })
    }

    /// Compute SCS storage is private to each enabled USC. The auxiliary
    /// allocation the dispatch publishes, per USC and at least the eight-page
    /// tail, belongs to the compute allocator, in addition to its small/large
    /// SCS page inventories and the execution-state spill reservation. As for
    /// render, the pool covers what the descriptor tells the hardware.
    pub(crate) fn compute_scs_requirements(&self, mut request: Requirements,
        pages: u16, auxiliary_pages: u32) -> Result<Requirements, PoolError>
    {
        if pages != 0 {
            let small = u64::from(pages);
            let per_core = add(add(small, small.div_ceil(256))?, u64::from(auxiliary_pages.max(8)))?;
            let bytes = mul(mul(per_core, u64::from(self.active_cores))?, 4096)?;
            request.minimum_bytes = add(request.minimum_bytes, bytes)?;
            request.ideal_bytes = add(request.ideal_bytes, bytes)?;
            request.has_spill = true;
        }
        Ok(request)
    }

    /// Render's allocator publishes the already rounded auxiliary page count
    /// in the descriptor, including its eight-page tail. This is separate
    /// from the layout-derived small/large SCS inventories. The USC
    /// spill-buffer allocation multiplies their sum by cores.
    pub(crate) fn render_scs_requirements(&self, mut request: Requirements,
        pages: u16, auxiliary_pages: u32) -> Result<Requirements, PoolError>
    {
        if pages != 0 {
            let small = u64::from(pages);
            let per_core = add(add(small, small.div_ceil(256))?,
                u64::from(auxiliary_pages))?;
            let bytes = mul(mul(per_core, u64::from(self.active_cores))?, 4096)?;
            request.minimum_bytes = add(request.minimum_bytes, bytes)?;
            request.ideal_bytes = add(request.ideal_bytes, bytes)?;
            request.has_spill = true;
        }
        Ok(request)
    }

    pub(crate) fn sizes(&self, request: Requirements, history: History)
        -> Result<Sizes, PoolError>
    {
        let reserve = u64::from(request.self_reserve_pages) << 12;
        let rt = u64::from(request.render_target_pages) << 12;
        let reserved = add(reserve, rt)?;
        if request.minimum_bytes < reserved || request.ideal_bytes < reserved {
            return Err(PoolError::Geometry);
        }
        // Every enabled USC has execution-state storage. Programs with spill
        // state add twelve pages; shared queues reserve another twenty-four.
        let execution = mul(u64::from(self.active_cores),
            if request.has_spill { 76 << 12 } else { 64 << 12 })?;
        let constant = mul(u64::from(self.constant_pages),
            u64::from(self.active_kicks - 1) << 12)?;
        let per_kick = mul(add(reserve.max(history.self_reserve),
            rt.max(history.render_target))?, u64::from(self.active_kicks))?;
        let size = |base: u64, high_water: u64| {
            let base = add(base, execution)?.checked_sub(reserved)
                .ok_or(PoolError::Geometry)?.max(high_water);
            add(add(base, constant)?, per_kick)
        };
        let ideal = aligned(size(request.ideal_bytes, history.ideal_base)?)?;
        let minimum = aligned(size(request.minimum_bytes, history.minimum_base)?
            .min(ideal.max(history.minimum_pool)))?;
        if minimum > ideal { return Err(PoolError::Geometry); }
        Ok(Sizes { minimum, ideal })
    }
}

impl Sizes {
    /// Reserve the full address inventory but allocate only the desired
    /// physical blocks. The hardware directory limit is checked by geometry.
    pub(crate) fn geometry(self) -> Result<PoolGeometry, PoolError> {
        if self.minimum == 0 || self.minimum > self.ideal { return Err(PoolError::Geometry); }
        let blocks = usize::try_from(self.ideal / render_block_bytes() as u64)
            .map_err(|_| PoolError::Geometry)?;
        if self.ideal % render_block_bytes() as u64 != 0 { return Err(PoolError::Geometry); }
        PoolGeometry::g17s(blocks)
    }
}
