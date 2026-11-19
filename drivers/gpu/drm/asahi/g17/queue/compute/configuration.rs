// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Initial M5 queue installation names physical execution/QoS identities
//! before an Active command exists. Retain the exact tokens independently.

use super::{Context, Cursors, ITEM_RING_ENTRIES};
use crate::mmu::ContextPin;
use kernel::{prelude::*, sync::Arc};

pub(super) struct Configuration {
    context: Arc<Context>,
    pin: ContextPin,
    qos: u8,
    target: u32,
    epoch: u64,
    visible: bool,
}
impl Configuration {
    pub(super) fn new(context: &Arc<Context>, target: u32, epoch: u64) -> Result<Self> {
        let pin = context.pin()?;
        let qos = match context.pin_qos() {
            Ok(qos) => qos,
            Err(error) => { context.unpin(pin); return Err(error); }
        };
        Ok(Self { context: context.clone(), pin, qos, target, epoch, visible: false })
    }
    pub(super) fn mark_visible(&mut self) { self.visible = true; }
    pub(super) fn visible(&self) -> bool { self.visible }
    pub(super) fn consumed(&self, cursors: Cursors, epoch: u64) -> bool {
        self.visible && self.epoch == epoch && self.target < ITEM_RING_ENTRIES
            && cursors.count == ITEM_RING_ENTRIES
            && cursors.producer == self.target && cursors.consumer == self.target
    }
    /// Only before visibility, after exact empty-channel consumption, after
    /// transfer to the same pinned Active, or after verified processor stop.
    pub(super) fn release(self) {
        self.context.unpin_qos(self.qos);
        self.context.unpin(self.pin);
    }
}
