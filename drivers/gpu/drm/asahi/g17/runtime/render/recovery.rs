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
        let untouched = pairs().find(|pair| pair.qids().contains(&blamed))?.front_untouched() == Some(true);
        let running = pairs().filter_map(|pair| {
            let qids = pair.qids().into_iter().fold(0, |mask, qid| mask | 1u128 << qid);
            Some((qids, pair.running_owner()?))
        });
        culprit(running, untouched, |culprit| {
            let mut foreign = false;
            for queue in self.compute.iter().flatten().filter_map(|entry| entry.queue.as_deref()) {
                queue.visit_owners(&mut |owner| foreign |= !Arc::ptr_eq(owner, culprit));
            }
            foreign
        })
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

/// Picks the QIDs to blame from each pair's running owner. A VM an earlier recovery found guilty
/// comes first, even when the blamed render had started: its groups queued before that keep
/// hanging the GPU one per restart, while bystanders' renders whose tiler started wait behind
/// them, and that recovery failed its interrupted work. Otherwise the blamed render must be
/// `untouched`, exactly one VM may be running, and `foreign` must find no other VM's compute work.
fn culprit<'a>(
    running: impl Iterator<Item = (u128, &'a Arc<VmStatus>)>,
    untouched: bool,
    foreign: impl FnOnce(&Arc<VmStatus>) -> bool,
) -> Option<u128> {
    let (mut guilty, mut qids, mut several) = (0u128, 0u128, false);
    let mut culprit: Option<&Arc<VmStatus>> = None;
    for (pair, owner) in running {
        if owner.recovery_guilty() {
            guilty |= pair;
        }
        several |= culprit.is_some_and(|known| !Arc::ptr_eq(known, owner));
        culprit = Some(owner);
        qids |= pair;
    }
    if guilty != 0 {
        return Some(guilty);
    }
    let culprit = culprit.filter(|_| untouched && !several)?;
    (!foreign(culprit)).then_some(qids)
}

#[kernel::macros::kunit_tests(asahi_g17_render_culprit)]
mod tests {
    use super::*;

    fn vm() -> Arc<VmStatus> {
        Arc::pin_init(VmStatus::new(), GFP_KERNEL).unwrap()
    }

    #[test]
    fn a_guilty_vm_still_running_is_blamed_before_running_bystanders() {
        let (spinner, bystander) = (vm(), vm());
        let running = || [(0b11, &bystander), (0b1100, &spinner)].into_iter();
        // Two VMs running and no earlier guilt: the firmware's blame stands, even when one of
        // them failed only for its own client.
        bystander.record(EIO);
        assert_eq!(culprit(running(), true, |_| false), None);
        spinner.note_recovery_guilty();
        assert_eq!(culprit(running(), true, |_| panic!("compute is not consulted")), Some(0b1100));
        // The bystander's blamed render whose tiler started is starved too.
        assert_eq!(culprit(running(), false, |_| panic!("compute is not consulted")), Some(0b1100));
    }

    #[test]
    fn one_running_vm_is_blamed_only_without_foreign_compute() {
        let (a, b) = (vm(), vm());
        assert_eq!(culprit([(0b11, &a), (0b1100, &a)].into_iter(), true, |_| false), Some(0b1111));
        // A blamed render that had started may be the hang itself.
        assert_eq!(culprit([(0b11, &a), (0b1100, &a)].into_iter(), false, |_| false), None);
        assert_eq!(culprit([(0b11, &a)].into_iter(), true, |_| true), None);
        assert_eq!(culprit([(0b11, &a), (0b1100, &b)].into_iter(), true, |_| false), None);
        assert_eq!(culprit(core::iter::empty(), true, |_| false), None);
    }
}
