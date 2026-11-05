// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! The initial M5 configuration can reach firmware before an Active command.
//! Its captured identities therefore have an independent owner. No aggregate
//! work-state node is referenced by this configuration-only item prefix.

use super::{retirement::Cursors, Context};
use crate::mmu::ContextPin;
use kernel::{prelude::*, sync::Arc};

pub(super) struct Configuration {
    context: Arc<Context>,
    pin: ContextPin,
    qos: u8,
    targets: [u32; 2],
    generation: u64,
    visible: bool,
}
impl Configuration {
    pub(super) fn new(context: &Arc<Context>, targets: [u32; 2], generation: u64) -> Result<Self> {
        let pin = context.pin()?;
        let qos = match context.pin_qos() {
            Ok(qos) => qos,
            Err(error) => { context.unpin(pin); return Err(error); }
        };
        Ok(Self { context: context.clone(), pin, qos, targets, generation, visible: false })
    }
    pub(super) fn mark_visible(&mut self) { self.visible = true; }
    pub(super) fn visible(&self) -> bool { self.visible }
    pub(super) fn consumed(&self, cursors: [Cursors; 2], generation: u64) -> bool {
        self.visible && self.generation == generation && cursors.iter().zip(self.targets).all(|(cursor, target)| {
            cursor.done == target && cursor.read == target && cursor.write == target
        })
    }
    /// Only before visibility, after exact configuration consumption, after
    /// transfer to an Active owning the same pinned Packet, or processor stop.
    pub(super) fn release(self) {
        self.context.unpin_qos(self.qos);
        self.context.unpin(self.pin);
    }
}
