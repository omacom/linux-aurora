// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Render predecessors retained while a physical pair is prepared off-lock.

use super::Pair;
use crate::g17::{
    dependency::Frontier,
    fw::kick::KickDependency,
    job::{self, Packet},
    status::VmStatus,
};
use crate::hw::t8140::queues::RENDER_DEPTH;
use kernel::{prelude::*, sync::Arc};

struct Predecessor {
    packet: Arc<Packet>,
    word: u64,
    ta_word: u64,
}

/// A loan of the graph cannot hide work already published on that pair. Fence
/// status remains live through the retained Packet, even while its storage is away.
pub(crate) struct Dependencies {
    owner: Option<u64>,
    entries: [Option<Predecessor>; RENDER_DEPTH as usize],
}

fn include(
    frontier: &mut Frontier,
    status: &Arc<VmStatus>,
    prefix: Option<u64>,
    packet: &Arc<Packet>,
    word: u64,
    compute: bool,
) -> Result {
    if !Arc::ptr_eq(packet.completion.status(), status) {
        return Ok(());
    }
    if !job::Order::contains(prefix, packet.order.sequence) {
        return Ok(());
    }
    let status = job::fence_status(&packet.completion.scheduler_fence());
    frontier.include(prefix, packet.order.sequence, status, word)?;
    // A compute consumer's firmware wait ends once firmware retires this group, even unexecuted
    // after a restart, so it may then read output the group never wrote. Render consumers of the
    // same owner follow the group on its pair, and a replay keeps that order.
    if compute && status == 0 {
        packet.completion.note_firmware_consumer();
    }
    Ok(())
}

impl Dependencies {
    pub(crate) fn contains_packet(&self, packet: &Arc<Packet>) -> bool {
        self.entries
            .iter()
            .flatten()
            .any(|entry| Arc::ptr_eq(&entry.packet, packet))
    }

    pub(crate) fn collect(
        &self,
        owner: u64,
        status: &Arc<VmStatus>,
        prefix: Option<u64>,
        frontier: &mut Frontier,
        compute: bool,
    ) -> Result {
        if self.owner != Some(owner) {
            return Ok(());
        }
        for entry in self.entries.iter().flatten() {
            include(frontier, status, prefix, &entry.packet, entry.word, compute)?;
        }
        Ok(())
    }
    pub(crate) fn collect_tiling(
        &self,
        owner: u64,
        status: &Arc<VmStatus>,
        prefix: Option<u64>,
        frontier: &mut Frontier,
    ) -> Result {
        if self.owner != Some(owner) {
            return Ok(());
        }
        for entry in self.entries.iter().flatten() {
            include(frontier, status, prefix, &entry.packet, entry.ta_word, false)?;
        }
        Ok(())
    }
}

impl Pair {
    /// The fixed tracker bounds this snapshot; no allocation or packet can be
    /// lost between capture and the registry moving the pair under the same lock.
    pub(crate) fn dependency_snapshot(&self) -> Dependencies {
        let qid = self.qids()[1];
        let mut active = self.active.iter();
        Dependencies {
            owner: self.owner,
            entries: core::array::from_fn(|_| {
                active.next().map(|entry| Predecessor {
                    packet: entry.packet.clone(),
                    ta_word: KickDependency::new(self.qids()[0], entry.kicks[0])
                        .map_or(u64::MAX, KickDependency::word),
                    word: KickDependency::new(qid, entry.kicks[1])
                        .map_or(u64::MAX, KickDependency::word),
                })
            }),
        }
    }

    pub(crate) fn collect_dependencies(
        &self,
        owner: u64,
        status: &Arc<VmStatus>,
        prefix: Option<u64>,
        frontier: &mut Frontier,
        compute: bool,
    ) -> Result {
        if self.owner != Some(owner) {
            return Ok(());
        }
        let qid = self.qids()[1];
        for entry in self.active.iter() {
            let word =
                KickDependency::new(qid, entry.kicks[1]).map_or(u64::MAX, KickDependency::word);
            include(frontier, status, prefix, &entry.packet, word, compute)?;
        }
        Ok(())
    }
    pub(crate) fn collect_tiling_dependencies(
        &self,
        owner: u64,
        status: &Arc<VmStatus>,
        prefix: Option<u64>,
        frontier: &mut Frontier,
    ) -> Result {
        if self.owner != Some(owner) {
            return Ok(());
        }
        let qid = self.qids()[0];
        for entry in self.active.iter() {
            let word =
                KickDependency::new(qid, entry.kicks[0]).map_or(u64::MAX, KickDependency::word);
            include(frontier, status, prefix, &entry.packet, word, false)?;
        }
        Ok(())
    }
}
