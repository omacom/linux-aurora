// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Name capacity refusals at their source, before a common submit or fence
//! failure handler loses their provenance. Counters are device-lifetime
//! independent diagnostics, never admission or ownership state.

use core::sync::atomic::{AtomicU64, Ordering};
use kernel::prelude::*;

#[derive(Clone, Copy, Debug)]
pub(super) enum Site {
    ComputeReserve,
    PhysicalQids,
    RenderSlots,
    LogicalQueues,
    ComputeSlots,
    QosIds,
    ComputeItems,
    RetiredRenderBindings,
    RenderPoolIds,
    RenderWorkSlots,
    ParameterBuffers,
    MetricsIds,
    LowerVa,
    UpperVa,
    AliasVa,
    ExecutionContexts,
}

const SITE_COUNT: usize = Site::ExecutionContexts as usize + 1;
static REFUSALS: [AtomicU64; SITE_COUNT] = [const { AtomicU64::new(0) }; SITE_COUNT];

/// First and power-of-two refusals per site keep a pressure storm bounded in
/// the log. This does not change whether the caller retries or rejects work.
#[track_caller]
pub(super) fn refused(site: Site) -> Error {
    let count = REFUSALS[site as usize].fetch_add(1, Ordering::Relaxed).wrapping_add(1);
    if count.is_power_of_two() {
        let at = core::panic::Location::caller();
        pr_warn!("G17 capacity refusal {:?} count={} at {}:{}\n", site, count,
            at.file().rsplit("asahi/").next().unwrap_or(at.file()), at.line());
    }
    ENOSPC
}

/// Read-only diagnostics hold the live device session while sampling its pools.
/// They never wake firmware, reclaim a resource, or change admission state.
pub(super) struct View(pub(super) kernel::sync::Arc<super::Shared>);

impl kernel::debugfs::Writer for View {
    fn write(&self, f: &mut kernel::fmt::Formatter<'_>) -> kernel::fmt::Result {
        writeln!(f, "version=1 sampled_ns={} crashed={}", super::now_ns(),
                 u8::from(self.0.crashed.load(Ordering::Acquire)))?;
        let uat = {
            let state = self.0.state.lock();
            let Some(firmware) = state.as_deref() else { return writeln!(f, "online=0"); };
            writeln!(f, "online=1")?;
            let (converted, published) = super::sync_input::counters();
            writeln!(f, "fw_inputs converted={} published={} native={} pending={}",
                converted, published, super::sync_input::native_count(),
                super::sync_input::pending_count())?;
            writeln!(f, "compute_admission_demand={}", self.0.compute_demand.load(Ordering::Acquire))?;
            firmware.queues.write_capacity(f)?;
            firmware.pool.write_capacity(f)?;
            writeln!(f, "render_pool_ids used={} total=127", firmware.render_ids.used())?;
            firmware.uat.clone()
        };
        // Mapping destruction can issue firmware flushes while holding VM
        // locks. Never take a VM/TTBAT lock with the device mutex held here.
        // The retained UAT owns valid allocator metadata even during shutdown.
        writeln!(f, "execution_contexts used={} total={}", uat.execution_contexts_used(), crate::mmu::MAX_EXECUTION_CONTEXTS)?;
        let mut upper = uat.geometry().kernel_range();
        upper.start += crate::hw::t8140::dynamic::KERNEL_OFFSET;
        for (name, vm, range) in [
            ("dynamic_lower_bytes", uat.kernel_lower_vm(), crate::hw::t8140::dynamic::LOWER),
            ("dynamic_upper_bytes", uat.kernel_vm(), upper),
        ] {
            match vm.capacity(range.clone()) {
                Ok((used, largest)) => writeln!(f, "{} used={} total={} largest_hole={}", name, used, range.end - range.start, largest)?,
                Err(error) => writeln!(f, "{} unavailable={:?}", name, error)?,
            }
        }
        for (site, count) in [
            Site::ComputeReserve, Site::PhysicalQids, Site::RenderSlots, Site::LogicalQueues,
            Site::ComputeSlots, Site::QosIds, Site::ComputeItems, Site::RetiredRenderBindings,
            Site::RenderPoolIds, Site::RenderWorkSlots, Site::ParameterBuffers, Site::MetricsIds,
            Site::LowerVa, Site::UpperVa, Site::AliasVa, Site::ExecutionContexts,
        ].iter().zip(REFUSALS.iter()) {
            writeln!(f, "refusals site={:?} count={}", site, count.load(Ordering::Relaxed))?;
        }
        Ok(())
    }
}
