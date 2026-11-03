// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Recovery targets for retained render graphs while firmware is halted.

use crate::g17::{runtime::Registry, status::VmStatus};
use kernel::{prelude::*, sync::Arc};

impl Registry {
    pub(in crate::g17::runtime) fn report_render_recovery(&self, qid: u8, port: &crate::g17::port::Port) {
        pr_warn!("G17: recovery grow pending={} active={:?} reply={:?}\n",
            self.render.grow.requests.len(), self.render.grow.active, self.render.grow.reply);
        for entry in self.render.entries.iter().flatten().filter(|entry| {
            entry.reservation.ids.iter().any(|id| id.qid() == qid)
        }) {
            if let Some(pair) = &entry.pair {
                let qids = pair.qids();
                pair.report_recovery((|| Ok([port.stamp(qids[0])?, port.stamp(qids[1])?]))());
            } else {
                pr_warn!("G17: recovery render queue={} unavailable borrowed={} constructing={}\n",
                    qid, entry.borrowed_active, entry.constructing);
            }
        }
    }

    pub(in crate::g17::runtime) fn render_pass_started(&self, qid: u8, stamp: u64) -> Option<bool> {
        self.render.entries.iter().flatten()
            .filter_map(|entry| entry.pair.as_ref())
            .find(|pair| pair.qids().contains(&qid))?
            .pass_started(stamp)
    }

    /// See `recovery::Queues::render_culprit`.
    pub(in crate::g17::runtime) fn render_culprit(&self, blamed: u8) -> Option<u128> {
        let pairs = || self.render.entries.iter().flatten().filter_map(|entry| entry.pair.as_ref());
        if pairs().find(|pair| pair.qids().contains(&blamed))?.front_untouched() != Some(true) {
            return None;
        }
        let mut culprit: Option<&Arc<VmStatus>> = None;
        let mut qids = 0u128;
        for pair in pairs() {
            let Some(owner) = pair.running_owner() else {
                continue;
            };
            if culprit.is_some_and(|known| !Arc::ptr_eq(known, owner)) {
                return None;
            }
            culprit = Some(owner);
            qids |= pair.qids().into_iter().fold(0, |mask, qid| mask | 1u128 << qid);
        }
        let culprit = culprit?;
        let mut foreign = false;
        for queue in self.compute.iter().flatten().filter_map(|entry| entry.queue.as_deref()) {
            queue.visit_owners(&mut |owner| foreign |= !Arc::ptr_eq(owner, culprit));
        }
        (!foreign).then_some(qids)
    }

    pub(in crate::g17::runtime) fn publish_render_recovery_targets(&self) -> Option<Error> {
        let mut failure = None;
        for pair in self.render.entries.iter().flatten()
            .filter_map(|entry| entry.pair.as_ref())
        {
            if let Err(error) = pair.publish_recovery_targets() {
                failure = failure.or(Some(error));
            }
        }
        failure
    }
}
