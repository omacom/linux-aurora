// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Program and tile storage pressure for shared render pools. Program sizes
//! are byte counts after linking; they are independent of register allocation.

use super::pool::PoolError;
use super::pool_budget::Requirements;

type Result<T> = core::result::Result<T, PoolError>;

fn add(a: u64, b: u64) -> Result<u64> { a.checked_add(b).ok_or(PoolError::Geometry) }
fn mul(a: u64, b: u64) -> Result<u64> { a.checked_mul(b).ok_or(PoolError::Geometry) }
fn count(value: u64) -> Result<u32> { value.try_into().map_err(|_| PoolError::Geometry) }
fn pages(value: u64, size: u64) -> Result<u64> { Ok(add(value, size - 1)? / size) }

#[derive(Clone, Copy, Default)]
pub(crate) struct FragmentStorage {
    pub(crate) primary_bytes: u32,
    pub(crate) secondary_bytes: u32,
    pub(crate) linked_bytes: u32,
    pub(crate) minimum_primary_bytes: u32,
    pub(crate) auxiliary_count: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct FragmentParams {
    pub(crate) pressure: u32,
    pub(crate) page_units: u32,
    pub(crate) large_units: u32,
    pub(crate) auxiliary_units: u32,
    pub(crate) primary_large: u32,
    pub(crate) secondary_large: u32,
}

impl FragmentStorage {
    pub(crate) fn parameters(self) -> Result<FragmentParams> {
        let primary = mul(add(u64::from(self.primary_bytes), u64::from(self.linked_bytes))?
            .max(u64::from(self.minimum_primary_bytes)), 32)?;
        let primary = add(primary, 0x4380)?;
        let secondary = mul(u64::from(self.secondary_bytes), 32)?;
        let primary_large = pages(primary, 0x10000)?;
        let secondary_large = pages(secondary, 0x10000)?;
        let small = add(pages(primary, 0x1000)?, pages(secondary, 0x1000)?)?;
        let large = add(primary_large, secondary_large)?;
        let auxiliary = mul(u64::from(self.auxiliary_count), 128)?;
        let (auxiliary, reserved) = if auxiliary == 0 { (0, 0) }
            else { (auxiliary | 64, auxiliary.max(0x4000) | 64) };
        let pressure = add(add(mul(small, 96)?, mul(large, 10)?)?,
            add(mul(pages(auxiliary, 0x1000)?, 64)?, mul(pages(auxiliary, 0x10000)?, 5)?)?)?;
        Ok(FragmentParams {
            pressure: count(pressure)?, page_units: count(add(small, large)?)?,
            large_units: count(large)?,
            auxiliary_units: count(add(pages(reserved, 0x1000)?, pages(reserved, 0x10000)?)?)?,
            primary_large: count(primary_large)?, secondary_large: count(secondary_large)?,
        })
    }
}

#[derive(Clone, Copy, Default)]
pub(crate) struct TileStorage {
    pub(crate) threadgroup_bytes: u32,
    pub(crate) imageblock_bytes: u32,
    pub(crate) multisample_bytes: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct TileParams {
    pub(crate) pressure: u32,
    pub(crate) page_units: u32,
    pub(crate) large_units: u32,
    pub(crate) threadgroup_large: u32,
    pub(crate) imageblock_large: u32,
    pub(crate) multisample_large: u32,
}

impl TileStorage {
    pub(crate) fn parameters(self, preempt_formula: bool) -> Result<TileParams> {
        let imageblock = u64::from(self.imageblock_bytes);
        let sizes = [u64::from(self.threadgroup_bytes),
            if imageblock == 0 { 0 } else { add(imageblock, 64)? },
            u64::from(self.multisample_bytes)];
        let large = [pages(sizes[0], 0x10000)?, pages(sizes[1], 0x10000)?,
            pages(sizes[2], 0x10000)?];
        let small = add(add(pages(sizes[0], 0x1000)?, pages(sizes[1], 0x1000)?)?,
            pages(sizes[2], 0x1000)?)?;
        let total_large = add(add(large[0], large[1])?, large[2])?;
        let factor = if preempt_formula { 11 } else { 6 };
        Ok(TileParams {
            pressure: count(add(mul(small, factor)?, mul(total_large, factor.min(8))?)?)?,
            page_units: count(add(small, total_large)?)?, large_units: count(total_large)?,
            threadgroup_large: count(large[0])?, imageblock_large: count(large[1])?,
            multisample_large: count(large[2])?,
        })
    }
}

/// One submission's maximum pressure across all of its linked render
/// programs. Different programs may maximize different storage dimensions.
#[derive(Clone, Copy)]
pub(crate) struct RenderDemand {
    counts: [u32; 5],
    minimum_pages: u32,
    ideal_pages: u32,
    pressure: u32,
    clusters: u64,
    maximum: u64,
    total: u64,
    floor: u64,
    limited_maximum: u64,
    limited_total: u64,
    included: bool,
}

impl RenderDemand {
    pub(crate) fn new(core_masks: &[u32], nominal_cores: u32, ptc_floor: u32) -> Result<Self> {
        let clusters = core_masks.len() as u64;
        let maximum = u64::from(core_masks.iter().map(|m| m.count_ones()).max().unwrap_or(0));
        let total = core_masks.iter().try_fold(0u64, |n, m| add(n, u64::from(m.count_ones())))?;
        if total == 0 || ptc_floor == 0 || u64::from(nominal_cores) < total {
            return Err(PoolError::Geometry);
        }
        let limit = mul(u64::from(nominal_cores), 2)? / clusters;
        Ok(Self {
            counts: [0; 5], minimum_pages: 0, ideal_pages: 0, pressure: 0,
            clusters, maximum, total, floor: u64::from(ptc_floor),
            limited_maximum: maximum.min(limit), limited_total: total.min(mul(limit, clusters)?),
            included: false,
        })
    }

    pub(crate) fn include_fragment(&mut self, fragment: FragmentParams, tile: TileParams) -> Result<()> {
        let Self { clusters, maximum, total, floor, .. } = *self;
        let base = add(mul(maximum, add(floor, 11)?)?, 58)?;
        let fragment_small = u64::from(fragment.page_units);
        let tile_small = u64::from(tile.page_units);
        let pressure = add(u64::from(fragment.pressure), u64::from(tile.pressure))?;
        let large = add(add(u64::from(fragment.large_units), u64::from(fragment.auxiliary_units))?,
            u64::from(tile.large_units))?;
        let parts = [fragment.primary_large, fragment.secondary_large,
            tile.threadgroup_large, tile.imageblock_large, tile.multisample_large];
        let tails = parts[..4].iter().try_fold(0u64, |n, &v| add(n, u64::from(v)))?;
        let ideal = add(add(add(base, u64::from(tile.multisample_large))?, tails)?,
            add(mul(pressure.saturating_sub(floor), maximum)?, 64)?)?;
        let minimum = add(add(add(base, u64::from(tile.multisample_large))?, tails)?,
            add(add(mul(add(add(fragment_small, tile_small)?.saturating_sub(floor), tile_small)?,
                    maximum)?, mul(large, total)?)?, 64)?)?;
        self.merge(parts, minimum, ideal, pressure, clusters)
    }

    /// Vertex work has no imageblock and uses the tiling execution limits.
    pub(crate) fn include_vertex(&mut self, storage: VertexStorage) -> Result<()> {
        let params = FragmentStorage {
            primary_bytes: storage.primary_bytes, secondary_bytes: storage.secondary_bytes,
            linked_bytes: storage.linked_bytes, minimum_primary_bytes: storage.minimum_primary_bytes,
            auxiliary_count: 0,
        }.parameters()?;
        let Self { clusters, floor, limited_maximum: maximum, limited_total: total, .. } = *self;
        let large = u64::from(params.large_units);
        let base = add(mul(maximum, add(floor, 11)?)?, 58)?;
        let minimum = add(add(add(base, large)?, 64)?,
            add(mul(u64::from(params.page_units).saturating_sub(floor), maximum)?,
                mul(large, total)?)?)?;
        let ideal = add(add(add(base, large)?, 64)?,
            mul(u64::from(params.pressure).saturating_sub(floor), maximum)?)?;
        self.merge([params.primary_large, params.secondary_large, 0, 0, 0], minimum,
            ideal, u64::from(params.pressure), clusters)
    }

    /// Compute storage is evaluated per dispatch before merging component maxima.
    /// `resident_warps` and `shared_budget` are device policy, not API limits.
    pub(crate) fn include_compute(&mut self, storage: ComputeStorage,
        resident_warps: u32, shared_budget: u32) -> Result<()> {
        if storage.threads == 0 || resident_warps == 0 {
            return Err(PoolError::Geometry);
        }
        let frame = mul(add(u64::from(storage.primary_bytes),
            u64::from(storage.linked_bytes))?
            .max(u64::from(storage.minimum_primary_bytes)), 32)?;
        let frame = add(frame, 0x4380)?;
        let secondary = mul(u64::from(storage.secondary_bytes), 32)?;
        let primary_large = pages(frame, 0x10000)?;
        let secondary_large = pages(secondary, 0x10000)?;
        let small = add(pages(frame, 0x1000)?, pages(secondary, 0x1000)?)?;
        let large = add(primary_large, secondary_large)?;
        let shared = u64::from(storage.threadgroup_bytes);
        let shared_small = pages(shared, 0x1000)?;
        let shared_large = pages(shared, 0x10000)?;
        let warps = pages(u64::from(storage.threads), 32)?;
        let mut groups = u64::from(resident_warps) / warps;
        if shared != 0 {
            let aligned_shared = mul(pages(shared, 64)?, 64)?;
            groups = groups.min(u64::from(shared_budget) / aligned_shared);
        }
        if groups == 0 { return Err(PoolError::Geometry); }
        let pressure = add(add(mul(mul(groups, warps)?.min(96), small)?,
            mul(mul(groups, warps)?.min(10), large)?)?,
            add(mul(groups, shared_small)?, mul(groups.min(8), shared_large)?)?)?;
        let Self { clusters, maximum, total, floor, .. } = *self;
        let group_large = mul(warps.min(10), large)?;
        let tile = add(shared_small, shared_large)?;
        let transient = add(add(mul(warps, small)?, group_large)?, tile)?;
        let transient = add(transient.saturating_sub(floor), tile)?;
        let base = add(mul(maximum, add(floor, 11)?)?, 58)?;
        let tail = add(add(large, shared_large)?, 64)?;
        let minimum = add(add(add(base, tail)?, mul(transient, maximum)?)?,
            mul(add(shared_large, group_large)?, total)?)?;
        let ideal = add(add(base, tail)?, mul(pressure.saturating_sub(floor), maximum)?)?;
        self.merge([count(primary_large)?, count(secondary_large)?,
            count(shared_large)?, 0, 0], minimum, ideal, pressure, clusters)
    }

    /// Mesh/object execute on the tiling engine. Reuse compute's occupancy
    /// arithmetic with the tiling-limited core topology, never compute's full
    /// enabled core count. Resident/shared limits are explicit device policy.
    pub(crate) fn include_mesh(&mut self, storage: ComputeStorage,
        resident_warps: u32, shared_budget: u32) -> Result<()> {
        let maximum = self.maximum;
        let total = self.total;
        self.maximum = self.limited_maximum;
        self.total = self.limited_total;
        let result = self.include_compute(storage, resident_warps, shared_budget);
        self.maximum = maximum;
        self.total = total;
        result
    }

    fn merge(&mut self, parts: [u32; 5], minimum: u64, ideal: u64, pressure: u64,
        clusters: u64) -> Result<()> {
        // Stored page counts are signed before conversion to byte counts.
        // Reject an unrepresentable request instead of wrapping into a small pool.
        let round = |value| -> Result<u32> {
            let value = add(mul(value, clusters)?, 7)? & !7;
            if value > i32::MAX as u64 { return Err(PoolError::Geometry); }
            count(value)
        };
        let mut next = *self;
        next.minimum_pages = next.minimum_pages.max(round(minimum)?);
        next.ideal_pages = next.ideal_pages.max(round(ideal)?);
        next.pressure = next.pressure.max(count(pressure)?);
        for (out, value) in next.counts.iter_mut().zip(parts) { *out = (*out).max(value); }
        next.resource_flags()?;
        next.included = true;
        *self = next;
        Ok(())
    }

    pub(crate) fn pressure(&self) -> u32 { self.pressure }

    pub(crate) fn resource_flags(&self) -> Result<u64> {
        let [primary, secondary, shared, imageblock, multisample] = self.counts;
        if primary > 0xff || secondary > 0xf || shared > 3 || imageblock > 0xff
            || multisample > 0xff { return Err(PoolError::Geometry); }
        Ok(0x14b | (u64::from(primary) << 10) | (u64::from(secondary) << 18)
            | (u64::from(shared) << 22) | (u64::from(imageblock) << 24)
            | (u64::from(multisample) << 32))
    }

    pub(crate) fn requirements(&self) -> Result<Requirements> {
        if !self.included {
            return Err(PoolError::Geometry);
        }
        let Self { maximum, clusters, .. } = *self;
        let counts = self.counts.iter().try_fold(0u64, |n, &v| add(n, u64::from(v)))?;
        let render_target = mul(add(add(mul(maximum, 11)?, counts)?, 10)?, clusters)?;
        Ok(Requirements {
            minimum_bytes: u64::from(self.minimum_pages) << 12,
            ideal_bytes: u64::from(self.ideal_pages) << 12,
            self_reserve_pages: 0, render_target_pages: count(render_target)?, has_spill: false,
        })
    }
}

#[derive(Clone, Copy, Default)]
pub(crate) struct VertexStorage {
    pub(crate) primary_bytes: u32,
    pub(crate) secondary_bytes: u32,
    pub(crate) linked_bytes: u32,
    pub(crate) minimum_primary_bytes: u32,
}

/// Storage and execution width of a linked compute dispatch.
#[derive(Clone, Copy, Default)]
pub(crate) struct ComputeStorage {
    pub(crate) primary_bytes: u32,
    pub(crate) secondary_bytes: u32,
    pub(crate) linked_bytes: u32,
    pub(crate) minimum_primary_bytes: u32,
    pub(crate) threadgroup_bytes: u32,
    pub(crate) threads: u32,
}
