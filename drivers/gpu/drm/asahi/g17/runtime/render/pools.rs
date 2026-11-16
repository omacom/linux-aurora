// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Device-wide references to render free lists. Two physical pairs bound to
//! one logical context share one registration and one final release witness.
//! The device mutex serializes every transition and descriptor-table access.

use crate::g17::{
    channel::Rings,
    freelist::{Lease, Owner, RenderPool, LINGER_NS},
    fw::{
        channels::{ControlRecord, FreeListCompletion},
        initdata::PagePoolDescriptor,
    },
    initdata::InitData,
    object::KernelObject,
};
use core::sync::atomic::{fence, AtomicU64, Ordering};
use kernel::prelude::*;

const POOLS: usize = 256;
const FIRST_RENDER_POOL: usize = 129;

pub(super) struct Pools {
    leases: KVec<Option<Lease>>,
}

/// Undo state cannot leave the device-locked publication transaction. Rollback
/// is permitted only before the packet exposes its first firmware reference.
pub(super) struct Preparation {
    previous: Option<Lease>,
    slot: usize,
    pub(super) generation: u64,
}

impl Pools {
    pub(super) fn new() -> Result<Self> {
        let mut leases = KVec::with_capacity(POOLS, GFP_KERNEL)?;
        for _ in 0..POOLS {
            leases.push(None, GFP_KERNEL)?;
        }
        Ok(Self { leases })
    }

    fn slot(id: u16) -> Result<usize> {
        let slot = usize::from(id);
        if !(FIRST_RENDER_POOL..POOLS).contains(&slot) {
            return Err(EINVAL);
        }
        Ok(slot)
    }

    pub(super) fn owner(pool: &RenderPool) -> Owner {
        Owner {
            serial: pool.serial(),
            epoch: pool.epoch(),
            buffer_slot: u32::from(pool.id()),
            control_va: pool.control_va(),
            ring_va: pool.run_list_va(),
            cookie: pool.end_va(),
        }
    }

    /// The pool's registration has a consumed final release and no references:
    /// the firmware holds nothing of its backing.
    pub(super) fn idle(&self, id: u16) -> bool {
        Self::slot(id)
            .ok()
            .and_then(|slot| self.leases[slot])
            .is_some_and(|lease| lease.released())
    }

    pub(super) fn released(&self, id: u32, _control: u64) -> bool {
        self.leases
            .get(id as usize)
            .is_none_or(|lease| lease.as_ref().is_none_or(|lease| lease.released()))
    }

    /// Install both engine references before either inner queue becomes
    /// visible. Warm overlap keeps the existing generation and pool contents.
    pub(super) fn prepare(&mut self, init: &InitData, pool: &RenderPool) -> Result<Preparation> {
        // A vacated pool is re-armed by the submission path before it retries.
        if pool.vacant() {
            return Err(EAGAIN);
        }
        self.observe(init)?;
        let slot = Self::slot(pool.id())?;
        let owner = Self::owner(pool);
        let previous = self.leases[slot];
        if previous.is_some_and(|lease| lease.owner() != owner && !lease.released()) {
            return Err(EBUSY);
        }
        // Pools arrive fully populated. There is no asynchronous initial grow.
        let next = Lease::acquire(previous.as_ref(), owner)?;
        if previous.is_none_or(|lease| lease.owner() != owner) {
            self.stage(init, pool)?;
        }
        let generation = next.generation();
        self.leases[slot] = Some(next);
        Ok(Preparation {
            previous,
            slot,
            generation,
        })
    }

    /// Warming validates the incoming owner and stages its descriptor without
    /// acquiring command references or publishing an asynchronous request.
    pub(super) fn warm(&self, init: &InitData, pool: &RenderPool) -> Result {
        if pool.vacant() {
            return Err(EAGAIN);
        }
        let previous = self.leases[Self::slot(pool.id())?];
        let owner = Self::owner(pool);
        if previous.is_some_and(|lease| lease.owner() == owner) {
            return Ok(());
        }
        if previous.is_some_and(|lease| !lease.released()) {
            return Err(EBUSY);
        }
        Lease::acquire(previous.as_ref(), owner)?;
        self.stage(init, pool)
    }

    fn stage(&self, init: &InitData, pool: &RenderPool) -> Result {
        let words = descriptor_words(init.page_pool_descriptor_table()?, pool.id())?;
        pool.repopulate()?;
        let descriptor = pool
            .page_pool_state()?
            .descriptor(words[3].load(Ordering::Relaxed));
        for (word, value) in words.into_iter().zip([
            descriptor.page_list,
            descriptor.counters,
            descriptor.page_count,
            descriptor.unk_18,
        ]) {
            word.store(value, Ordering::Relaxed);
        }
        fence(Ordering::SeqCst);
        Ok(())
    }

    /// Retire preparation only after both deferred outer bodies and their
    /// inner commands exist, before publishing either outer producer.
    pub(super) fn activate(&mut self, id: u16, generation: u64) -> Result {
        let lease = self.leases[Self::slot(id)?].as_mut().ok_or(EIO)?;
        if lease.generation() != generation {
            return Err(EIO);
        }
        lease.retire_prepare()
    }

    pub(super) fn rollback(&mut self, preparation: Preparation) {
        self.leases[preparation.slot] = preparation.previous;
    }

    /// Preflight a paired retirement before QoS or PB reference accounting.
    pub(super) fn completed(&self, id: u16, generation: u64, linger: bool) -> Result<Lease> {
        let mut next = self.leases[Self::slot(id)?].ok_or(EIO)?;
        if next.generation() != generation {
            return Err(EIO);
        }
        next.complete_render(linger)?;
        Ok(next)
    }

    /// The matching completed() result and other accounting preflights were
    /// accepted under the same device mutex, so this assignment cannot fail.
    pub(super) fn commit_completed(&mut self, id: u16, next: Lease) {
        self.leases[usize::from(id)] = Some(next);
    }

    /// Ends the linger of every idle registration that `keep` (an open owner
    /// still bound to the pool) does not retain or whose linger has elapsed.
    /// Returns whether a release became pending, and the next expiry delay.
    pub(super) fn expire_lingers(
        &mut self,
        now: u64,
        keep: impl Fn(&Owner) -> bool,
    ) -> (bool, Option<u64>) {
        let mut ended = false;
        let mut next: Option<u64> = None;
        for lease in self.leases[FIRST_RENDER_POOL..].iter_mut().flatten() {
            let Some(since) = lease.lingering() else {
                continue;
            };
            let age = now.saturating_sub(since);
            if age >= LINGER_NS || !keep(&lease.owner()) {
                lease.end_linger();
                ended = true;
            } else {
                let delay = LINGER_NS - age;
                next = Some(next.map_or(delay, |next| next.min(delay)));
            }
        }
        (ended, next)
    }

    pub(super) fn pending(&self) -> bool {
        self.leases
            .iter()
            .flatten()
            .any(|lease| lease.pending_release().is_some())
    }

    pub(super) fn complete_grow(&mut self, event: FreeListCompletion) -> Result {
        let id = u16::try_from(event.buffer_slot).map_err(|_| EINVAL)?;
        let lease = self.leases[Self::slot(id)?].as_mut().ok_or(EIO)?;
        lease.complete_grow(event)
    }

    pub(super) fn observe(&mut self, init: &InitData) -> Result {
        fence(Ordering::Acquire);
        let consumer = init.control_consumer()?.load(Ordering::Relaxed);
        let producer = init.control_producer()?.load(Ordering::Relaxed);
        for lease in self.leases[FIRST_RENDER_POOL..].iter_mut().flatten() {
            lease.observe_releases(consumer, producer)?;
        }
        Ok(())
    }

    /// Store the release witness immediately after the ring producer. A failed
    /// mailbox announcement must never cause the same release to be replayed.
    pub(super) fn publish_releases(
        &mut self,
        init: &InitData,
        mut announce: impl FnMut() -> Result,
    ) -> Result {
        for lease in self.leases[FIRST_RENDER_POOL..].iter_mut().flatten() {
            let Some(release) = lease.pending_release() else {
                continue;
            };
            let cursor = Rings::publish_control(init, &ControlRecord::FreeListRelease(release))?;
            lease.mark_release_sent(cursor)?;
            fence(Ordering::SeqCst);
            announce()?;
        }
        self.observe(init)
    }

    /// A detached pool may capture firmware counters only after its last pair
    /// stopped using this registration. Callers check all retained pair owners.
    pub(super) fn save(&self, init: &InitData, pool: &RenderPool) -> Result {
        let state = pool.page_pool_state()?;
        let words = descriptor_words(init.page_pool_descriptor_table()?, pool.id())?;
        fence(Ordering::Acquire);
        let descriptor = PagePoolDescriptor {
            page_list: words[0].load(Ordering::Relaxed),
            counters: words[1].load(Ordering::Relaxed),
            page_count: words[2].load(Ordering::Relaxed),
            unk_18: words[3].load(Ordering::Relaxed),
        };
        if descriptor.page_list != state.descriptor(descriptor.unk_18).page_list {
            return Err(EIO);
        }
        pool.store_page_pool_state(state.updated(&descriptor))
    }
}

fn descriptor_words(table: &KernelObject, id: u16) -> Result<[&AtomicU64; 4]> {
    let offset = Pools::slot(id)? * size_of::<PagePoolDescriptor>();
    Ok([
        table.dword(offset)?,
        table.dword(offset + 8)?,
        table.dword(offset + 16)?,
        table.dword(offset + 24)?,
    ])
}

impl Pools {
    pub(super) fn write_capacity(&self, f: &mut kernel::fmt::Formatter<'_>) -> kernel::fmt::Result {
        writeln!(f, "render_pool_leases used={} total={} retained={}",
                 self.leases.iter().flatten().filter(|lease| !lease.released()).count(),
                 POOLS - FIRST_RENDER_POOL, self.leases.iter().flatten().count())
    }
}
