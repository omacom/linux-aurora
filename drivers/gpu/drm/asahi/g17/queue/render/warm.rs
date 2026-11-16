// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Fresh render graph cursor initialization at a retained physical ordinal.

use super::{memory, Pair};
use crate::g17::fw::queue::PointerBlock;
use core::sync::atomic::{fence, Ordering};
use kernel::prelude::*;

impl Pair {
    /// A new graph at a retained ordinal starts its variable item rings at zero.
    /// The old graph has retired; firmware has not received this graph's queues.
    pub(crate) fn rebase_fresh_graph(&mut self) -> Result {
        if self.ordinal == 0 {
            return Ok(());
        }
        fence(Ordering::Acquire);
        if self
            .memory
            .graph
            .cursors()?
            .into_iter()
            .any(|cursor| cursor.done != 0 || cursor.read != 0 || cursor.write != 0)
        {
            return Err(EBUSY);
        }
        let mut blocks = [core::ptr::null::<PointerBlock>(); 2];
        for (block, offset) in blocks.iter_mut().zip(memory::POINTERS) {
            let pointer = self
                .memory
                .graph
                .queues
                .pointer(offset, size_of::<PointerBlock>())?;
            if pointer.align_offset(align_of::<PointerBlock>()) != 0 {
                return Err(EINVAL);
            }
            *block = pointer.cast();
        }
        let layout = self.memory.graph.pointers;
        for block in blocks {
            // SAFETY: Both complete pointer blocks were checked above. This fresh
            // graph is unpublished; only the live atomic cursors are accessed.
            unsafe { &*block }.rebase(&layout, 0)?;
        }
        fence(Ordering::SeqCst);
        self.cursors = [0; 2];
        Ok(())
    }

}
