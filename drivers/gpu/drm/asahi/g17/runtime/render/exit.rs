// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Physical render queue scope for an exiting logical context. A graph loan
//! remains foreign to the transaction until its exact owner is restored.

use super::RenderState;
use crate::g17::context::Context;
use kernel::sync::Arc;

impl RenderState {
    pub(in crate::g17::runtime) fn exit_scope(&self, context: &Arc<Context>) -> (u128, u128, u64) {
        let (mut own, mut foreign, mut slots) = (0, 0, 0);
        for (slot, entry) in self.entries.iter().enumerate() {
            let Some(entry) = entry else {
                continue;
            };
            let qids = self.qids_for_slot(slot as u8);
            let preparing = entry.preparing.is_some() || entry.constructing || entry.growing;
            if !preparing && entry.context.as_ref().is_some_and(|owner| Arc::ptr_eq(owner, context)) {
                own |= qids;
                slots |= 1u64 << slot;
            } else if preparing || entry.context.is_some() {
                foreign |= qids;
            }
        }
        (own, foreign, slots)
    }

    /// Slots of `context`'s pairs whose commands all ran but stay unretired until its kill.
    pub(in crate::g17::runtime) fn executed_slots(&self, context: &Arc<Context>) -> u64 {
        let mut slots = 0;
        for (slot, entry) in self.entries.iter().enumerate() {
            if entry.as_ref().and_then(|entry| entry.pair.as_ref())
                .is_some_and(|pair| pair.executed_for_kill(context))
            {
                slots |= 1u64 << slot;
            }
        }
        slots
    }

    /// The admission owner whose identities the pair in `slot` was built with.
    pub(in crate::g17::runtime) fn slot_owner(&self, slot: u8) -> Option<u64> {
        self.entries
            .get(usize::from(slot))
            .and_then(Option::as_ref)
            .map(|entry| entry.reservation.owner)
    }

    pub(in crate::g17::runtime) fn qids_for_slot(&self, slot: u8) -> u128 {
        self.entries
            .get(usize::from(slot))
            .and_then(Option::as_ref)
            .map_or(0, |entry| {
                entry.reservation.ids.into_iter().fold(0, |mask, id| {
                    mask | (1u128 << id.qid())
                })
            })
    }
}
