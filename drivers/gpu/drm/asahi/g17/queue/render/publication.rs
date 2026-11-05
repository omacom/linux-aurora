// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Preflighted item-ring publications. Each lane retains an independent cursor
//! and omits its configuration record only when its durable identity matches.

use super::{memory, retirement::Window};
use crate::g17::fw::{
    m5,
    kick::KickTimestamp,
    queue::{DataMaster, KickAnnounce, QueueConfig, ITEM_RING_ENTRIES},
};
use core::sync::atomic::{fence, AtomicU32, AtomicU64, Ordering};
use kernel::prelude::*;

fn identity(mut config: QueueConfig) -> QueueConfig {
    config.install = 0;
    config.context_update = 0;
    config.completion_seed_valid = 0;
    config.completion_seed = 0;
    config.qos_update = 0;
    config
}
pub(in crate::g17::queue) fn required(previous: Option<QueueConfig>, config: QueueConfig) -> bool {
    config.install != 0
        || config.context_update != 0
        || config.completion_seed_valid != 0
        || config.qos_update != 0
        || previous.map(identity) != Some(identity(config))
}

/// The kick announcement record: the G17P one, or the M5 notification.
enum Announcement {
    Neo(KickAnnounce),
    M5(m5::queue::Notification),
}

pub(super) struct Lane<'a> {
    items: [&'a AtomicU64; 4],
    producer: &'a AtomicU32,
    values: [u64; 4],
    config: QueueConfig,
    config_destination: *mut u64,
    config_present: bool,
    announce: Announcement,
    announce_destination: *mut u8,
    prefix: usize,
    prefix_end: u32,
    pub(super) window: Window,
}
impl<'a> Lane<'a> {
    #[expect(
        clippy::too_many_arguments,
        reason = "The publication lane describes independent fields of the firmware wire record."
    )]
    pub(super) fn new(
        graph: &'a memory::Graph,
        stage: usize,
        ordinal: u64,
        base: u32,
        config: QueueConfig,
        present: bool,
        timestamp: KickTimestamp,
        lead: bool,
    ) -> Result<Self> {
        let m5 = graph.is_m5();
        // M5 items carry no descriptor: [configuration], notification.
        let count = u32::from(present) + u32::from(lead) + if m5 { 1 } else { 2 };
        let window = Window::new(base, count, ITEM_RING_ENTRIES)?;
        let offset = ordinal as usize % memory::RECORDS;
        let config_offset = memory::CONFIG[stage] + offset * memory::CONFIG_STRIDE;
        let announce_offset = memory::ANNOUNCE[stage] + offset * memory::ANNOUNCE_STRIDE;
        let config_destination = graph
            .config
            .pointer(config_offset, size_of::<QueueConfig>())?
            .cast::<u64>();
        if config_destination.align_offset(align_of::<u64>()) != 0 {
            return Err(EINVAL);
        }
        let announce_destination = graph
            .queues
            .pointer(announce_offset, if m5 { size_of::<m5::queue::Notification>() }
                else { size_of::<KickAnnounce>() })?;
        let index =
            |delta: u32| memory::ITEMS[stage] + ((base + delta) % ITEM_RING_ENTRIES) as usize * 8;
        let items = [
            graph.queues.dword(index(0))?,
            graph.queues.dword(index(1))?,
            graph.queues.dword(index(2))?,
            graph.queues.dword(index(3))?,
        ];
        let announce_va = graph.queues.gpu_va() + announce_offset as u64;
        let descriptor = graph.descriptor_va(stage, ordinal);
        // Item order: [lead command], descriptor, [configuration], announcement.
        let mut values = [0; 4];
        let mut n = 0;
        let mut push = |value: u64| {
            values[n] = value;
            n += 1;
        };
        if lead {
            push(graph.config.gpu_va() + (config_offset + memory::LEAD) as u64);
        }
        if !m5 {
            push(descriptor);
        }
        if present {
            push(graph.config.gpu_va() + config_offset as u64);
        }
        push(announce_va);
        Ok(Self {
            items,
            producer: graph
                .queues
                .word(memory::POINTERS[stage] + graph.pointers.producer)?,
            values,
            config,
            config_destination,
            config_present: present,
            announce: if m5 {
                Announcement::M5(m5::queue::notification(config.qid.try_into()?, timestamp)?)
            } else {
                Announcement::Neo(KickAnnounce::new(
                    config.qid.try_into()?,
                    if stage == 0 {
                        DataMaster::Tiling
                    } else {
                        DataMaster::Fragment
                    },
                    timestamp,
                    config.priority as u8,
                )?)
            },
            announce_destination,
            prefix: count as usize - 1,
            prefix_end: (base + count - 1) % ITEM_RING_ENTRIES,
            window,
        })
    }
    fn items(&self, range: core::ops::Range<usize>) {
        for index in range {
            self.items[index].store(self.values[index], Ordering::Relaxed);
        }
    }
    fn config(&self) {
        if !self.config_present {
            return;
        }
        let source = core::ptr::from_ref(&self.config).cast::<u64>();
        for index in 0..size_of::<QueueConfig>() / 8 {
            // SAFETY: QueueConfig is packed integer storage with no padding;
            // every source qword is read unaligned. The destination is a
            // checked, aligned record owned by this unpublished ordinal.
            unsafe {
                self.config_destination
                    .add(index)
                    .write(source.add(index).read_unaligned());
            }
        }
    }
    fn announce(&self) {
        // SAFETY: The full record is inside the retained queue graph and is
        // exclusively owned by this ordinal. Publication follows its copy.
        unsafe {
            match &self.announce {
                Announcement::Neo(value) => self.announce_destination
                    .cast::<KickAnnounce>().write_unaligned(*value),
                Announcement::M5(value) => self.announce_destination
                    .cast::<m5::queue::Notification>().write_unaligned(*value),
            }
        }
    }
    /// Fill the retained records without advancing the firmware-visible cursor.
    /// M5's host-port transaction must occur between this and publish_kick.
    pub(super) fn prepare(&self) {
        self.items(0..self.prefix + 1);
        self.config();
        self.announce();
    }
    pub(super) fn publish_prefix(&self) {
        self.items(0..self.prefix);
        self.config();
        fence(Ordering::SeqCst);
        self.producer.store(self.prefix_end, Ordering::Relaxed);
    }
    pub(super) fn publish_kick(&self) {
        self.announce();
        self.items(self.prefix..self.prefix + 1);
        fence(Ordering::SeqCst);
        self.producer.store(self.window.target(), Ordering::Relaxed);
    }
    pub(super) fn publish_tiling(
        &self,
        event_slot: &AtomicU32,
        next_slot: u32,
        inner: &AtomicU32,
        next_inner: u32,
    ) {
        self.items(0..self.prefix + 1);
        self.config();
        self.announce();
        fence(Ordering::SeqCst);
        self.producer.store(self.window.target(), Ordering::Relaxed);
        event_slot.store(next_slot, Ordering::Relaxed);
        inner.store(next_inner, Ordering::Relaxed);
    }
}
