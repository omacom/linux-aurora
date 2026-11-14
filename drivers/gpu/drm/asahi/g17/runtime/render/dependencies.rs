// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Both engines preserve the covered render prefix while physical graphs are borrowed.

use crate::g17::{dependency::Frontier, job::Packet, runtime::Registry};
use kernel::{prelude::*, sync::Arc};

impl Registry {
    pub(in crate::g17::runtime) fn render_prefix(
        &self,
        owner: u64,
        packet: &Arc<Packet>,
    ) -> Result<Frontier> {
        self.render_prefix_at(owner, packet, packet.order.wait_through[0])
    }
    pub(in crate::g17::runtime) fn render_prefix_at(
        &self,
        owner: u64,
        packet: &Arc<Packet>,
        prefix: Option<u64>,
    ) -> Result<Frontier> {
        let mut frontier = Frontier::new();
        let compute = packet.engine() == crate::g17::job::Engine::Compute;
        for entry in self.render.entries.iter().flatten() {
            if let Some(pair) = entry.pair.as_ref() {
                pair.collect_dependencies(
                    owner,
                    packet.completion.status(),
                    prefix,
                    &mut frontier,
                    compute,
                )?;
            } else if let Some(dependencies) = entry.borrowed_dependencies.as_ref() {
                dependencies.collect(
                    owner,
                    packet.completion.status(),
                    prefix,
                    &mut frontier,
                    compute,
                )?;
            }
        }
        Ok(frontier)
    }
    pub(in crate::g17::runtime) fn tiling_prefix(
        &self,
        owner: u64,
        packet: &Arc<Packet>,
    ) -> Result<Frontier> {
        let mut frontier = Frontier::new();
        for entry in self.render.entries.iter().flatten() {
            if let Some(pair) = entry.pair.as_ref() {
                pair.collect_tiling_dependencies(
                    owner,
                    packet.completion.status(),
                    packet.order.wait_through[0],
                    &mut frontier,
                )?;
            } else if let Some(dependencies) = entry.borrowed_dependencies.as_ref() {
                dependencies.collect_tiling(
                    owner,
                    packet.completion.status(),
                    packet.order.wait_through[0],
                    &mut frontier,
                )?;
            }
        }
        Ok(frontier)
    }
}
