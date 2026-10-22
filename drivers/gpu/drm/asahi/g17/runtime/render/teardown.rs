// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Exact logical ownership gates for scheduler release and physical-pair parking.

use super::RenderState;
use crate::g17::{
    buffer::BufferIds,
    context::Context,
    initdata::InitData,
    runtime::{teardown::ReclaimBatch, Registry},
};
use kernel::{prelude::*, sync::Arc};

impl RenderState {
    pub(in crate::g17::runtime) fn observe_teardown_releases(&mut self, init: &InitData) -> Result {
        self.pools.observe(init)
    }

    pub(in crate::g17::runtime) fn teardown_idle(&self, context: &Arc<Context>) -> Result<bool> {
        // A pair that changed owner keeps running this scheduler's queues until
        // the new owner's first command leaves it, whichever context owns it now.
        let scheduler = Some(context.scheduler_va());
        if self.entries.iter().flatten().any(|entry| match entry.pair.as_ref() {
            Some(pair) => pair.switch_in_flight() == scheduler,
            None => (entry.preparing.is_some() || entry.growing) && entry.borrowed_switch == scheduler,
        }) {
            return Ok(false);
        }
        for entry in self.entries.iter().flatten() {
            if !entry
                .context
                .as_ref()
                .is_some_and(|owner| Arc::ptr_eq(owner, context))
            {
                continue;
            }
            if entry.preparing.is_some() || entry.growing {
                return Ok(false);
            }
            if let Some(pair) = entry.pair.as_ref() {
                if !pair.idle_for_context(context)? {
                    return Ok(false);
                }
            }
        }
        Ok(true)
    }

    pub(in crate::g17::runtime) fn teardown_pools_released(&self, context: &Arc<Context>) -> bool {
        context.render_pools_released(|id, control| self.pools.released(id, control))
            && self
                .retired
                .iter()
                .filter(|old| Arc::ptr_eq(old.context(), context))
                .all(|old| {
                    self.pools
                        .released(u32::from(old.pool().id()), old.pool().control_va())
                })
    }

    /// Warm superseded roots need only their exact executed replacement witness.
    /// Graphs with their own memory remain until that Context's scheduler ack.
    pub(in crate::g17::runtime) fn collect_reclaimable(
        &mut self,
        out: &mut ReclaimBatch,
    ) -> Result {
        for slot in 0..self.entries.len() {
            self.collect_retired(slot)?;
        }
        let mut index = 0;
        while index < self.retired.len() {
            if self.retired[index].buffer_id().is_some() {
                index += 1;
                continue;
            }
            let Ok(target) = out.binding_slot() else {
                break;
            };
            *target = Some(self.retired.remove(index).map_err(|_| EIO)?);
        }
        Ok(())
    }

    /// Preflight the entire Context before detaching its first graph. Successor
    /// graphs on the same VM have a different Arc and remain installed.
    pub(in crate::g17::runtime) fn detach_context(
        &mut self,
        context: &Arc<Context>,
        buffers: &mut BufferIds,
        out: &mut ReclaimBatch,
    ) -> Result {
        let mut count = self
            .retired
            .iter()
            .filter(|old| Arc::ptr_eq(old.context(), context))
            .count();
        for entry in self.entries.iter().flatten() {
            if !entry
                .context
                .as_ref()
                .is_some_and(|owner| Arc::ptr_eq(owner, context))
            {
                continue;
            }
            if entry.preparing.is_some() || entry.growing || entry.parked.is_some() {
                return Err(EBUSY);
            }
            if let Some(pair) = entry.pair.as_ref() {
                if !pair.can_park(context)? {
                    return Err(EBUSY);
                }
                count += 1;
            }
        }
        if !out.has_room(count, 1) {
            return Err(EBUSY);
        }
        for entry in self.entries.iter_mut().flatten() {
            if !entry
                .context
                .as_ref()
                .is_some_and(|owner| Arc::ptr_eq(owner, context))
            {
                continue;
            }
            let Some(pair) = entry.pair.as_ref() else {
                continue;
            };
            buffers.unreserve_owner(pair.buffer_id())?;
            let target = out.binding_slot()?;
            let pair = entry.pair.take().ok_or(EIO)?;
            let (mut parked, retired) = KBox::into_inner(pair).park();
            // This park follows the consumed release of `context`.
            parked.release_consumed(context.scheduler_va());
            *target = Some(retired);
            entry.parked = Some(parked);
            // Detached backing retains these references until the worker runs.
            entry.context = None;
            entry.pool = None;
        }
        let mut index = 0;
        while index < self.retired.len() {
            if !Arc::ptr_eq(self.retired[index].context(), context) {
                index += 1;
                continue;
            }
            if let Some(id) = self.retired[index].buffer_id() {
                buffers.unreserve_owner(id)?;
            }
            let target = out.binding_slot()?;
            *target = Some(self.retired.remove(index).map_err(|_| EIO)?);
        }
        Ok(())
    }
}

impl Registry {
    /// Close while a builder owns a pair records intent. Its return path applies
    /// the release before admitting further commands from either logical owner.
    pub(in crate::g17::runtime) fn release_render(&mut self, owner: u64) -> Result {
        for entry in self.render.entries.iter_mut().flatten() {
            if entry.pending_owner == Some(owner) {
                entry.pending_owner_closed = true;
            }
            if entry.reservation.owner != owner {
                continue;
            }
            entry.closed = true;
            if let Some(pair) = entry.pair.as_mut() {
                pair.release_owner();
            }
        }
        Ok(())
    }
}
