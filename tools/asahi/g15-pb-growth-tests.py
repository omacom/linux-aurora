#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Run the G15 PB growth answer and its on-demand backing, actual bodies, against a model.

The harness extracts from drivers/gpu/drm/asahi/g15/pools.rs Firmware::answer_growth(),
answer_request(), map_pb(), refill_backing(), note_idle(), reclaim_check(), free_backing() and
reclaim_delay_ns(), Pools::soft_max(), map_heap(), free_chunks() and back_chunk(), PbChunk,
chunk_gva(), CHUNK_BYTES and Pools::new's PB backing statements; from vm.rs VmState::map_one(),
map_heap(), map_chunk() and unmap_heap_above(); pb_max_blocks(), pb_blocks(), pb_reclaim(),
pb_idle_ns() and pb_reclaim_blocks() from params.rs; the PB sizes, tuning and PB leaf word from
hw.rs; Prot::from_pte() and is_cached_noncoherent() from pgtable.rs; and the parameters' defaults
from asahi_core.rs. It includes pools/growth.rs, fw/buffer.rs and fw/channels.rs whole. The bodies
run against a model of the firmware objects, the GEM backing allocation, the shared layer's map_at()
and the DEVCTL wait, which logs every allocation, alias map, unmap, free, block record, barrier,
producer store, reply and log line in order. Allocation and alias map failures can be injected.
Source checks: publish_prefix_into() starts with the PB catch-up, and pools.rs asserts the PB leaf
GPU-only at compile time.

What it shows:
- a poll pass answers only the requests queued when it starts, in order; one the reply's wait
  queues goes to the next pass. A request that cannot be processed does not end the pass;
- from the driver's initial size to its maximum, every block record is written once at its
  offset, each producer store follows its records and a barrier, each reply follows its producer
  store and a barrier, and every producer is in 1..=4095, ending at 4095;
- at the maximum two refusals are answered; further requests get no reply, count as bad and
  warn once until the ledger is reset, as init() does;
- the soft maximum is the parameter clamped to 64..=4095 and then to the ledger's
  initial..=max; growth answers above it are logged at info level, and the one that reaches it
  exactly is not;
- invalid requests (type, slot, stale counter, epoch, no live render) get no reply, and requests
  rejected so allocate nothing;
- the backing: the chunks of the initial posting and one reserve chunk when the GPU starts;
  chunks of 16 MiB, backed after the replies of a pass (never between them) while fewer than a
  reserve chunk remain, mapped into the VM of the flights before any record or producer names
  their blocks, never past the heap; with the reserve dry a request waits at the head of the
  queue, keeps its counter and is answered once a chunk is backed; a chunk that cannot be mapped
  into the running VM is dropped again; a pending re-initialization backs nothing while no
  render is published, and still backs for a render published before it; a failed reply keeps
  everything;
- a VM's aliases are the chunks, contiguous from the heap's start; a VM that runs after another
  grew the backing maps it first (at publication); a partial catch-up keeps the aliases made; a
  chunk freed and backed again (an idle reclaim) is unmapped and mapped again at publication;
- the one-VM rule: a catch-up with a flight of another VM is refused (EPROTO) before anything
  is mapped;
- the idle reclaim, actual reclaim_check(), free_backing() and note_idle() over a clock: a heavy
  client that exits with nothing rendered after it is re-initialized after 50 ms with the backing
  kept, and the kept backing is freed after the long idle, its posted count kept until the InitBM;
  at reclaim level 1 the backing stays; a recovery's pending re-initialization frees nothing;
  driven as the idle poll work arms it (reclaim_delay_ns()), the same exit wakes at 50 ms and at
  the long idle and arms nothing after; an idle reclaim arms nothing once its re-initialization
  is pending;
- nothing grown since the initialization (an initial posting above half the maximum, or a
  reclaim floor below it): no re-initialization and nothing armed;
- the actual reclaim keeps the posted count until the next render's InitBM (a model of the
  publication: map_pb() first, then for a render prefix with a re-initialization pending the
  ledger reset of Pools::init()): the same client rendering again after 60 ms re-posts the
  initial blocks and grows again from the kept backing with nothing allocated; after 1.1 s it
  grows again over freshly backed chunks, each mapped before it is posted; a compute-only
  submission between the reclaim and the InitBM maps, backs and posts nothing;
- a reclaim whose walk of the VMs fails counts nothing freed; the VM's next publication drops
  its stale aliases and releases the pages;
- the ledger reset of Pools::init() re-posts from the backing without allocating;
- the PB leaf decodes as GPU-only, not cached-noncoherent: dropping an alias is a plain unmap;
- Ledger::new() is Ledger::new() as of dba2a110de00 (the carried upstream constructor), capped at
  4095, over every growth and fixed input in 0..=4200 and at the u32 extremes. The driver's
  values are unchanged.

Not shown: kernel, firmware or GPU execution, timing, or what the firmware does with a reply, a
refusal or an unanswered request. The model's VMs have no open/closed state: that a closed VM
whose jobs still run keeps its aliases comes from the VM lifetime source (Job and VmJobGuard
hold the VM), not from these tests. The ledger reset is not a recovery. The shared layer's
mapping is modelled, not run.

With --mutations, each mutation below must fail its intended test with the expected message, and
the control (a driver maximum of the whole 4096-block heap) must pass, held by the ledger's cap.
A run that hangs (60 s) counts as no test failed.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import resource
import subprocess

ROOT = Path(__file__).resolve().parents[2]
G15 = "drivers/gpu/drm/asahi/g15"
POOLS = f"{G15}/pools.rs"
GROWTH = f"{G15}/pools/growth.rs"
BUFFER = f"{G15}/fw/buffer.rs"
CHANNELS = f"{G15}/fw/channels.rs"
INITDATA = f"{G15}/fw/initdata.rs"
PARAMS = f"{G15}/params.rs"
HW = f"{G15}/hw.rs"
CORE = "drivers/gpu/drm/asahi/asahi_core.rs"
ENGINE = f"{G15}/engine.rs"
VM = f"{G15}/vm.rs"
PGTABLE = "drivers/gpu/drm/asahi/pgtable.rs"
OLD_REF = "dba2a110de00743eeecd41edf23e54ede4491d5e"

# Ledger::new() at OLD_REF, the tip of the four PR363 carries (upstream 027bd7623f37), verbatim:
# the constructor before the cap below K.
OLD_LEDGER_NEW = '''    pub(crate) const fn new(grow: bool, blocks: u32, max_blocks: u32) -> Self {
        let (initial, max) = if grow {
            let nb = clamp(blocks, MIN_BLOCKS, MAX_BLOCKS);
            (nb, clamp(max_blocks, nb, MAX_BLOCKS))
        } else {
            (MAX_BLOCKS, MAX_BLOCKS)
        };
        Self {
            initial,
            max,
            posted: initial,
            next_request: 0,
            refusal_logged: false,
            refusals: 0,
        }
    }'''

# Pools::new's PB backing, run verbatim by the model (up to its log line).
POOLS_NEW_BACKING = ('        // Backing for the initial blocks and the reserve; growth adds the rest.\n',
                     '        dev_info!(\n')
# The compile-time proof that a PB alias drop is a plain unmap.
PB_LEAF_ASSERT = 'static_assert!(!mmu::Prot::from_pte(pte::LEAF_CACHED).is_cached_noncoherent());'

# The two statements of Pools::init() that reset the ledger.
LEDGER_RESET = ['self.ledger.reinit();',
                'self.exhausted_logged = false;',
                'self.reclaim_pending = false;']

MUTATIONS = {
    # Answer until the queue is empty, including requests queued by the replies' waits.
    'drain-until-empty': (POOLS, 'for _ in 0..queued {', 'while !self.events.grow.is_empty() {'),
    # Leave the last request queued before the pass for later.
    'skip-last-queued': (POOLS, 'for _ in 0..queued {', 'for _ in 0..queued.saturating_sub(1) {'),
    # End the pass at a request that cannot be processed.
    'return-after-request-error': (POOLS, '                    self.pools.stats.bad += 1;\n                    dev_err!(',
                                   '                    self.pools.stats.bad += 1;\n                    return Ok(());\n'
                                   '                    dev_err!('),
    # Answer three refusals at the maximum.
    'three-refusals': (GROWTH, 'pub(crate) const MAX_REFUSALS: u32 = 2;', 'pub(crate) const MAX_REFUSALS: u32 = 3;'),
    # Answer refusals for ever.
    'never-exhausted': (GROWTH, 'self.refusals >= MAX_REFUSALS', 'false && self.refusals >= MAX_REFUSALS'),
    # Warn for every unanswered request.
    'warn-every-time': (POOLS, 'if !self.pools.exhausted_logged {', 'if true {'),
    # Store the producer without the barrier after the block records.
    'no-record-barrier': (POOLS, '            // Records before their producer.\n            crate::mem::sync();\n',
                          '            // Records before their producer.\n'),
    # Reply without the barrier after the producer store.
    'no-reply-barrier': (POOLS, '            // Producer before the reply.\n            crate::mem::sync();\n',
                         '            // Producer before the reply.\n'),
    # Log the answer that reaches the soft maximum exactly.
    'soft-boundary-inclusive': (POOLS, 'let past_soft = n != 0 && ledger.posted > soft;',
                                'let past_soft = n != 0 && ledger.posted >= soft;'),
    # Give the firmware the parameter without the ledger's bounds.
    'soft-max-unclamped': (POOLS, 'super::params::pb_max_blocks().clamp(self.ledger.initial, self.ledger.max)',
                           'super::params::pb_max_blocks()'),
    # Ledger::new() without the cap below K.
    'uncapped-ledger': (GROWTH, 'let nb = clamp(blocks, MIN_BLOCKS, MAX_POSTED);\n'
                                '            (nb, clamp(max_blocks, nb, MAX_POSTED))\n'
                                '        } else {\n            (MAX_POSTED, MAX_POSTED)',
                        'let nb = clamp(blocks, MIN_BLOCKS, MAX_BLOCKS);\n'
                        '            (nb, clamp(max_blocks, nb, MAX_BLOCKS))\n'
                        '        } else {\n            (MAX_BLOCKS, MAX_BLOCKS)'),
    # Grant without the backing: neither wait nor clip.
    'grant-without-backing': [(GROWTH, '        if room != 0 && self.spare() == 0 {\n            return Answer::Wait;\n        }\n', ''),
                              (GROWTH, 'count: GROW_STEP.min(room).min(self.spare()),', 'count: GROW_STEP.min(room),')],
    # Allocate before the pass's answers.
    'refill-before-answers': [(POOLS, '        self.refill_backing();\n        Ok(())', '        Ok(())'),
                              (POOLS, '        let queued = self.events.grow.len();\n',
                               '        self.refill_backing();\n        let queued = self.events.grow.len();\n')],
    # Back a chunk without mapping it into the running VM.
    'refill-maps-no-flight': (POOLS, '                    flight.job.vm.map_chunk(i, gem.clone())?;\n',
                              '                    let _ = (flight, i, gem.clone());\n'),
    # Drop a request that waits for backing.
    'no-wait-requeue': (POOLS, '                    if self.events.grow.insert_within_capacity(0, request).is_err() {\n'
                               '                        self.pools.stats.bad += 1;\n                    }\n', ''),
    # Keep a chunk that the running VM could not map.
    'refill-failure-keeps-chunk': (POOLS, '                        self.pools.chunks.truncate(backed);\n', ''),
    # Back chunks while a re-initialization is pending.
    'refill-during-reinit': (POOLS, '        if self.recovery.reinit && !render {\n            return;\n        }\n', ''),
    # Skip the refill on any pending re-initialization, also with a live render (a stall).
    'refill-skip-with-live-render': (POOLS, 'if self.recovery.reinit && !render {', 'if self.recovery.reinit {'),
    # Refill for any live flight, also compute-only ones that cannot ask for growth.
    'refill-for-any-flight': (POOLS, 'if self.recovery.reinit && !render {',
                              'if self.recovery.reinit && self.engine.flights.is_empty() {'),
    # Keep a VM's alias of a chunk that was freed and backed again.
    'map-without-identity': (VM, '                if core::ptr::eq(&*have.gem, &*gem) {', '                if true {'),
    # Stop the reclaim check on a reclaim's pending re-initialization (the kept backing stays).
    'pending-reinit-stops-free': (POOLS, '            || (self.recovery.reinit && !self.pools.reclaim_pending)\n'
                                         '            || self.recovery.armed.is_some()\n'
                                         '            || self.engine.frozen\n'
                                         '            || self.recovery.failed().is_some()\n'
                                         '        {\n            return;\n',
                                  '            || self.recovery.reinit\n'
                                  '            || self.recovery.armed.is_some()\n'
                                  '            || self.engine.frozen\n'
                                  '            || self.recovery.failed().is_some()\n'
                                  '        {\n            return;\n'),
    # Free the kept backing at reclaim level 1.
    'pending-free-at-level-1': (POOLS, 'if level >= 2 && ledger.pending_free_due(idle_ns, min_idle) {',
                                'if ledger.pending_free_due(idle_ns, min_idle) {'),
    # Do not record that the pending re-initialization is a reclaim's.
    'reclaim-pending-not-set': (POOLS, '        self.pools.reclaim_pending = true;\n', ''),
    # Free for any pending re-initialization, also a recovery's.
    'free-on-recovery-reinit': (POOLS, '            || (self.recovery.reinit && !self.pools.reclaim_pending)\n'
                                       '            || self.recovery.armed.is_some()\n'
                                       '            || self.engine.frozen\n'
                                       '            || self.recovery.failed().is_some()\n'
                                       '        {\n            return;\n',
                                '            || self.recovery.armed.is_some()\n'
                                '            || self.engine.frozen\n'
                                '            || self.recovery.failed().is_some()\n'
                                '        {\n            return;\n'),
    # Both checks stop on any pending re-initialization: the code before the kept-backing free.
    'old-reinit-guards': [(POOLS, '            || (self.recovery.reinit && !self.pools.reclaim_pending)\n'
                                  '            || self.recovery.armed.is_some()\n'
                                  '            || self.engine.frozen\n'
                                  '            || self.recovery.failed().is_some()\n'
                                  '        {\n            return;\n',
                           '            || self.recovery.reinit\n'
                           '            || self.recovery.armed.is_some()\n'
                           '            || self.engine.frozen\n'
                           '            || self.recovery.failed().is_some()\n'
                           '        {\n            return;\n'),
                          (POOLS, '            || (self.recovery.reinit && !self.pools.reclaim_pending)\n'
                                  '            || self.recovery.armed.is_some()\n'
                                  '            || self.engine.frozen\n'
                                  '            || self.recovery.failed().is_some()\n'
                                  '        {\n            return None;\n',
                           '            || self.recovery.reinit\n'
                           '            || self.recovery.armed.is_some()\n'
                           '            || self.engine.frozen\n'
                           '            || self.recovery.failed().is_some()\n'
                           '        {\n            return None;\n')],
    # Arm nothing while a reclaim's re-initialization is pending.
    'pending-reinit-arms-nothing': (POOLS, '            || (self.recovery.reinit && !self.pools.reclaim_pending)\n'
                                           '            || self.recovery.armed.is_some()\n'
                                           '            || self.engine.frozen\n'
                                           '            || self.recovery.failed().is_some()\n'
                                           '        {\n            return None;\n',
                                    '            || self.recovery.reinit\n'
                                    '            || self.recovery.armed.is_some()\n'
                                    '            || self.engine.frozen\n'
                                    '            || self.recovery.failed().is_some()\n'
                                    '        {\n            return None;\n'),
    # Arm the poll for the expired 50 ms instead of the rest of the long idle.
    'pending-delay-from-drain': (GROWTH, 'Some(min_idle_ns.saturating_sub(idle_ns))',
                                 'Some(DRAIN_IDLE_NS.saturating_sub(idle_ns))'),
    # Re-initialize at an idle with nothing grown (posted above the floor only).
    'reclaim-due-nothing-grown': (GROWTH, 'self.posted > self.initial && self.posted > floor && idle_ns >= min_idle_ns',
                                  'self.posted > floor && idle_ns >= min_idle_ns'),
    # Re-initialize at a client boundary with nothing grown (initial above half the maximum).
    'drain-due-nothing-grown': (GROWTH, 'self.posted > self.initial && self.posted > self.max / 2 && idle_ns >= DRAIN_IDLE_NS',
                                'self.posted > self.max / 2 && idle_ns >= DRAIN_IDLE_NS'),
    # Arm the poll with nothing grown.
    'delay-nothing-grown': (GROWTH, '        let grown = self.posted > self.initial;\n', '        let grown = true;\n'),
    # Reset the posted count at the idle reclaim instead of at the next render's InitBM.
    'reinit-at-idle-reclaim': (POOLS, '        if level >= 2 && idle {\n            freed = self.free_backing();\n',
                               '        if level >= 2 && idle {\n            self.pools.ledger.reinit();\n            freed = self.free_backing();\n'),
    # Reset the posted count at any reclaim instead of at the next render's InitBM.
    'reinit-at-any-reclaim': (POOLS, '        self.pools.reclaim_pending = true;\n',
                              '        self.pools.reclaim_pending = true;\n        self.pools.ledger.reinit();\n'),
    # Count the chunks freed although the VMs could not be walked.
    'freed-without-walk': (POOLS, '                );\n                0\n            }\n        };\n',
                           '                );\n                dropped\n            }\n        };\n'),
    # Catch up without the one-VM check.
    'no-activation-guard': (POOLS, '.filter(|f| !core::ptr::eq(&*f.job.vm, vm))', '.filter(|_| false)'),
    # Back the whole heap when the GPU starts.
    'initial-backs-whole-heap': (POOLS, 'for _ in 0..Ledger::chunks_for(ledger.initial) {',
                                 'for _ in 0..growth::MAX_CHUNKS {'),
    # Growth chunks of 32 MiB.
    'chunks-above-16mib': (GROWTH, 'pub(crate) const CHUNK_BLOCKS: u32 = 128;', 'pub(crate) const CHUNK_BLOCKS: u32 = 256;'),
    # A firmware-visible cached PB leaf (its drop would ask the firmware to flush).
    'pb-leaf-fw-visible': (HW, 'pub(crate) const LEAF_CACHED: u64 = 0x00c0000000000c83;',
                           'pub(crate) const LEAF_CACHED: u64 = 0x00c0000000000c03;'),
}

# The test each mutation must fail, and a piece of its failure message.
INTENDED = {
    'drain-until-empty': ('queued_snapshot_keeps_order', 'replies in the first pass'),
    'skip-last-queued': ('queued_snapshot_keeps_order', 'replies in the first pass'),
    'return-after-request-error': ('unprocessed_request_does_not_end_the_pass', 'requests taken in the pass'),
    'three-refusals': ('default_growth_to_exhaustion', 'want Some(Log("warn"'),
    'never-exhausted': ('default_growth_to_exhaustion', 'growth did not end'),
    'warn-every-time': ('default_growth_to_exhaustion', 'later request after exhaustion'),
    'no-record-barrier': ('default_growth_to_exhaustion', 'got Some(Producer(74)), want Some(Sync)'),
    'no-reply-barrier': ('default_growth_to_exhaustion',
                         'got Some(Reply([7, 1, 0, 5, 0, 256, 3, 1, 0, 0, 0, 0, 0, 0])), want Some(Sync)'),
    'soft-boundary-inclusive': ('soft_max_boundaries', 'got Some(Log("info", "")), want Some(Log("dbg", ""))'),
    'soft-max-unclamped': ('soft_max_bounds', 'soft_max, param 0'),
    'uncapped-ledger': ('constructor_is_the_old_one_capped', 'new(true, '),
    'grant-without-backing': ('dry_reserve_waits_and_keeps_order', 'record of block'),
    'refill-before-answers': ('allocation_follows_the_replies', 'allocation before a reply'),
    'refill-maps-no-flight': ('growth_backs_before_posting', 'record of block'),
    'no-wait-requeue': ('dry_reserve_waits_and_keeps_order', 'waiting request lost'),
    'refill-failure-keeps-chunk': ('refill_map_failure_drops_the_chunk', 'chunk kept after a failed map'),
    'refill-during-reinit': ('pending_reinit_without_flights_backs_nothing', 'backed during a pending re-initialization'),
    'refill-skip-with-live-render': ('pending_reinit_with_a_live_render_refills', 'growth request stranded'),
    'refill-for-any-flight': ('pending_reinit_with_compute_only_backs_nothing', 'backed for a compute-only flight'),
    'map-without-identity': ('freed_chunks_are_mapped_again', 'alias of another object'),
    'pending-reinit-stops-free': ('client_boundary_reinit_arms_the_long_idle', 'poll woke in a loop'),
    'old-reinit-guards': ('client_boundary_reinit_arms_the_long_idle', 'kept backing not freed'),
    'pending-reinit-arms-nothing': ('client_boundary_reinit_arms_the_long_idle', 'kept backing not freed'),
    'pending-delay-from-drain': ('client_boundary_reinit_arms_the_long_idle', 'poll woke in a loop'),
    'pending-free-at-level-1': ('client_boundary_reinit_at_level_1_keeps_the_backing', 'freed at level 1'),
    'reclaim-pending-not-set': ('client_boundary_reinit_frees_without_a_render', 'kept backing not freed'),
    'free-on-recovery-reinit': ('recovery_reinit_frees_nothing', 'freed for a recovery'),
    'reclaim-due-nothing-grown': ('nothing_grown_reinitializes_nothing', 're-initialization with nothing grown (64 blocks, floor 32)'),
    'drain-due-nothing-grown': ('nothing_grown_reinitializes_nothing', 're-initialization with nothing grown (3000 blocks'),
    'delay-nothing-grown': ('nothing_grown_arms_nothing', 'poll woke in a loop'),
    'reinit-at-idle-reclaim': ('same_client_render_after_1_1s', 'posted count reset before the InitBM (1500)'),
    'reinit-at-any-reclaim': ('same_client_render_after_60ms', 'posted count reset before the InitBM'),
    'freed-without-walk': ('failed_vm_walk_counts_nothing_freed', 'freed counted with the VM walk failed'),
    'no-activation-guard': ('catch_up_refuses_flights_of_another_vm', 'catch-up with a flight of another VM'),
    'initial-backs-whole-heap': ('startup_backs_the_initial_blocks', 'startup allocation'),
    'chunks-above-16mib': ('growth_backs_before_posting', 'a growth chunk above 16 MiB'),
    'pb-leaf-fw-visible': ('pb_leaf_is_gpu_only', 'PB leaf not GPU-only'),
}
assert set(INTENDED) == set(MUTATIONS)

# Changes that must pass: the ledger's cap holds a driver maximum of the whole heap.
CONTROLS = {
    'tuning-whole-heap': (HW, 'pub(crate) const PB_MAX_BLOCKS: u32 = super::sizes::PB_BLOCKS - 1;',
                          'pub(crate) const PB_MAX_BLOCKS: u32 = super::sizes::PB_BLOCKS;'),
}


def item(text, start):
    a = text.index(start)
    b = text.index("{", a) + 1
    depth = 1
    while depth:
        depth += (text[b] == "{") - (text[b] == "}")
        b += 1
    return text[a:b]


def consts(text, names):
    out = []
    for name in names:
        m = re.search(rf'pub\(crate\) const {name}: (\w+) = ([^;]+);', text)
        if not m:
            raise SystemExit(f'{name} not found')
        out.append(f'pub const {name}: {m.group(1)} = {m.group(2)};')
    return '\n'.join(out)


PREFIX = r'''
#![allow(dead_code, unused_imports, unused_macros, unused_variables, unreachable_code, non_upper_case_globals)]
extern crate self as kernel;
use std::cell::{Cell, RefCell};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Error(i32);
impl Error {
    pub fn to_errno(&self) -> i32 {
        self.0
    }
}
pub type Result<T = ()> = core::result::Result<T, Error>;
pub const EINVAL: Error = Error(-22);
pub const ENODEV: Error = Error(-19);
pub const EIO: Error = Error(-5);
pub const ENOMEM: Error = Error(-12);
pub const ENOSPC: Error = Error(-28);
pub const EBUSY: Error = Error(-16);
pub const EPROTO: Error = Error(-71);
pub mod prelude {
    pub use crate::{static_assert, Error, Result, EBUSY, EINVAL, EIO, ENODEV, ENOMEM, ENOSPC, KVec, GFP_KERNEL};
}
pub type ARef<T> = std::sync::Arc<T>;
#[derive(Copy, Clone)]
pub struct Flags;
pub const GFP_KERNEL: Flags = Flags;
/// A kernel vector: pushes past the reserved capacity fail only through push_within_capacity.
pub struct KVec<T>(pub Vec<T>);
impl<T> KVec<T> {
    pub fn with_capacity(n: usize, _: Flags) -> Result<Self> {
        Ok(KVec(Vec::with_capacity(n)))
    }
    pub fn push(&mut self, value: T, _: Flags) -> Result {
        self.0.push(value);
        Ok(())
    }
    pub fn push_within_capacity(&mut self, value: T) -> core::result::Result<(), T> {
        if self.0.len() == self.0.capacity() {
            return Err(value);
        }
        self.0.push(value);
        Ok(())
    }
    pub fn capacity(&self) -> usize {
        self.0.capacity()
    }
    pub fn len(&self) -> usize {
        self.0.len()
    }
    pub fn get(&self, i: usize) -> Option<&T> {
        self.0.get(i)
    }
    pub fn last(&self) -> Option<&T> {
        self.0.last()
    }
    pub fn truncate(&mut self, n: usize) {
        self.0.truncate(n)
    }
    pub fn reserve(&mut self, n: usize, _: Flags) -> Result {
        self.0.reserve(n);
        Ok(())
    }
    pub fn iter(&self) -> std::slice::Iter<'_, T> {
        self.0.iter()
    }
}
/// A kernel mutex, single-threaded here.
pub struct Mutex<T>(pub RefCell<T>);
impl<T> Mutex<T> {
    pub fn new(value: T) -> Self {
        Mutex(RefCell::new(value))
    }
    pub fn lock(&self) -> std::cell::RefMut<'_, T> {
        self.0.borrow_mut()
    }
}
#[macro_export]
macro_rules! static_assert {
    ($e:expr) => {
        const _: () = assert!($e);
    };
}

/// One firmware-visible access, barrier or log line, in program order.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Ev {
    Record(usize, u64),
    Sync,
    Producer(u32),
    Reply([u32; 14]),
    Log(&'static str, String),
    /// Backing object `id` of this many bytes allocated, or freed (its last reference gone).
    Alloc(u32, usize),
    Free(u32),
    /// A PB alias of object `id` at GVA in the VM with this context, made or unmapped.
    Map(u32, u64, usize, u32),
    Unmap(u32, u64),
}
thread_local! {
    pub static LOG: RefCell<Vec<Ev>> = RefCell::new(Vec::new());
    pub static PARAM: Cell<u32> = Cell::new(PARAM_DEFAULT);
    pub static PARAM_BLOCKS: Cell<u32> = Cell::new(PB_BLOCKS_DEFAULT);
    /// The reclaim parameters (g15_pb_reclaim, g15_pb_idle_ms, g15_pb_reclaim_blocks).
    pub static PARAM_RECLAIM: Cell<u32> = Cell::new(PB_RECLAIM_DEFAULT);
    pub static PARAM_IDLE_MS: Cell<u32> = Cell::new(PB_IDLE_MS_DEFAULT);
    pub static PARAM_RECLAIM_BLOCKS: Cell<u32> = Cell::new(PB_RECLAIM_BLOCKS_DEFAULT);
    /// The monotonic clock (engine::now_ns()).
    pub static NOW: Cell<u64> = Cell::new(1_000_000_000);
    /// The VM registry walk (vms.open()) fails this many times.
    pub static FAIL_OPEN: Cell<u32> = Cell::new(0);
    /// Failures to inject: the next this many allocations or alias maps fail.
    pub static FAIL_ALLOC: Cell<u32> = Cell::new(0);
    pub static FAIL_MAP: Cell<u32> = Cell::new(0);
    pub static MAP_OK_BEFORE_FAIL: Cell<u32> = Cell::new(0);
    pub static NEXT_ID: Cell<u32> = Cell::new(1);
}
/// Takes one injected failure of `which`.
pub fn fail(which: &'static std::thread::LocalKey<Cell<u32>>) -> bool {
    which.with(|c| {
        let n = c.get();
        if n != 0 {
            c.set(n - 1);
        }
        n != 0
    })
}
pub fn ev(e: Ev) {
    LOG.with(|l| l.borrow_mut().push(e));
}
pub fn take() -> Vec<Ev> {
    LOG.with(|l| std::mem::take(&mut *l.borrow_mut()))
}
macro_rules! dev_log {
    ($lvl:expr, $dev:expr, $($arg:tt)*) => {{
        let _ = $dev;
        crate::ev(crate::Ev::Log($lvl, format!($($arg)*)));
    }};
}
macro_rules! dev_err { ($($t:tt)*) => { dev_log!("err", $($t)*) }; }
macro_rules! dev_warn { ($($t:tt)*) => { dev_log!("warn", $($t)*) }; }
macro_rules! dev_info { ($($t:tt)*) => { dev_log!("info", $($t)*) }; }
macro_rules! dev_dbg { ($($t:tt)*) => { dev_log!("dbg", $($t)*) }; }

pub mod mem {
    pub fn sync() {
        crate::ev(crate::Ev::Sync);
    }
}

/// A boot parameter: read-only after load.
pub mod module_parameters {
    pub struct Param;
    impl Param {
        pub fn value(&self) -> &'static u32 {
            Box::leak(Box::new(crate::PARAM.with(|p| p.get())))
        }
    }
    pub static g15_pb_max_blocks: Param = Param;
    pub struct Blocks;
    impl Blocks {
        pub fn value(&self) -> &'static u32 {
            Box::leak(Box::new(crate::PARAM_BLOCKS.with(|p| p.get())))
        }
    }
    pub static g15_pb_blocks: Blocks = Blocks;
    pub struct Cellp(pub &'static std::thread::LocalKey<std::cell::Cell<u32>>);
    impl Cellp {
        pub fn value(&self) -> &'static u32 {
            Box::leak(Box::new(self.0.with(|p| p.get())))
        }
    }
    pub static g15_pb_reclaim: Cellp = Cellp(&crate::PARAM_RECLAIM);
    pub static g15_pb_idle_ms: Cellp = Cellp(&crate::PARAM_IDLE_MS);
    pub static g15_pb_reclaim_blocks: Cellp = Cellp(&crate::PARAM_RECLAIM_BLOCKS);
}

pub mod driver {
    pub struct AsahiDevice;
}

/// GEM kernel objects: freed when the last reference (chunk or alias) goes.
pub mod gem {
    pub struct Object {
        pub id: u32,
        pub size: usize,
    }
    impl Drop for Object {
        fn drop(&mut self) {
            crate::ev(crate::Ev::Free(self.id));
        }
    }
    pub struct ObjectRef {
        pub gem: crate::ARef<Object>,
    }
}

/// The shared layer's VM: kernel aliases at fixed addresses.
pub mod mmu {
    use crate::prelude::*;
    pub struct Prot(pub u64);
    impl Prot {
        pub const fn from_pte(pte: u64) -> Self {
            Prot(pte)
        }
    }
    pub struct KernelMapping {
        pub slot: u32,
        pub gva: u64,
        pub size: usize,
        pub prot: u64,
        pub gem: crate::ARef<crate::gem::Object>,
    }
    impl Drop for KernelMapping {
        fn drop(&mut self) {
            crate::ev(crate::Ev::Unmap(self.slot, self.gva));
        }
    }
    pub struct Vm {
        pub slot: u32,
    }
    impl Vm {
        pub fn new(slot: u32) -> Self {
            Vm { slot }
        }
        pub fn map_at(
            &self,
            addr: u64,
            size: usize,
            gem: &crate::ARef<crate::gem::Object>,
            prot: Prot,
            guard: bool,
        ) -> Result<KernelMapping> {
            assert!(!guard);
            // The shared layer maps whatever size it is given: the whole object, exactly.
            assert_eq!(size, gem.size, "alias size differs from its object");
            let skip = crate::MAP_OK_BEFORE_FAIL.with(|c| {
                let n = c.get();
                c.set(n.saturating_sub(1));
                n != 0
            });
            if !skip && crate::fail(&crate::FAIL_MAP) {
                return Err(ENOMEM);
            }
            crate::ev(crate::Ev::Map(self.slot, addr, size, gem.id));
            Ok(KernelMapping { slot: self.slot, gva: addr, size, prot: prot.0, gem: gem.clone() })
        }
    }
}
'''

MODEL = r'''
        pub struct Ctl {
            pub epoch: u64,
        }
        impl Ctl {
            pub fn r32(&self, offset: usize) -> Result<u32> {
                if offset == control::HALT_COUNT {
                    Ok(self.epoch as u32)
                } else if offset == control::HALT_COUNT + 4 {
                    Ok((self.epoch >> 32) as u32)
                } else {
                    panic!("unexpected control read {offset:#x}")
                }
            }
        }
        pub struct Init {
            pub control: Option<std::sync::Arc<Ctl>>,
        }
        /// vm.rs's chunk mapping, its fields as the driver names them.
        pub struct HeapMapping {
            pub gem: ARef<gem::Object>,
            pub _mapping: mmu::KernelMapping,
        }
        pub struct HeapMappings {
            pub list: Mutex<KVec<HeapMapping>>,
        }
        pub struct VmState {
            pub id: u64,
            pub slot: u32,
            pub vm: mmu::Vm,
            pub heap: Arc<HeapMappings>,
        }
        impl VmState {
            pub fn new(slot: u32) -> Arc<Self> {
                Arc::new(VmState {
                    // Keep VM identity distinct from the modeled hardware slot.
                    id: 0x10000 + u64::from(slot),
                    slot,
                    vm: mmu::Vm::new(slot),
                    heap: Arc::new(HeapMappings { list: Mutex::new(KVec(Vec::new())) }),
                })
            }
        }
        pub struct Job {
            pub vm: Arc<VmState>,
        }
        impl Job {
            pub fn slot(&self) -> u32 {
                self.vm.slot
            }
        }
        pub struct Flight {
            pub job: Job,
        }
        pub struct Engine {
            pub flights: Vec<Flight>,
            /// Jobs accepted and not fully published.
            pub pending: Vec<u32>,
            pub frozen: bool,
        }
        pub struct Recovery {
            pub reinit: bool,
            pub armed: Option<(usize, u64)>,
            pub failed: Option<Error>,
        }
        impl Recovery {
            pub fn failed(&self) -> Option<Error> {
                self.failed
            }
        }
        /// vm.rs's Registry: the VMs whose client handle may still be open.
        pub struct Registry {
            pub vms: Vec<Arc<VmState>>,
        }
        impl Registry {
            pub fn open(&self) -> Result<KVec<Arc<VmState>>> {
                if crate::fail(&crate::FAIL_OPEN) {
                    return Err(ENOMEM);
                }
                Ok(KVec(self.vms.clone()))
            }
        }
        pub struct Receipts {
            pub renders: u32,
        }
        /// A firmware object: the block list or block_ctl.
        pub struct FwObject {
            pub gva: Option<u64>,
            pub size: usize,
            /// Writes that fail from now on.
            pub failures: Cell<u32>,
        }
        impl FwObject {
            pub fn w64(&self, offset: usize, value: u64) -> Result {
                assert!(offset % 8 == 0 && offset + 8 <= self.size, "record write at {offset:#x}");
                if self.failures.get() != 0 {
                    self.failures.set(self.failures.get() - 1);
                    return Err(EIO);
                }
                crate::ev(crate::Ev::Record(offset, value));
                Ok(())
            }
            pub fn w32(&self, offset: usize, value: u32) -> Result {
                assert_eq!(offset, block_ctl::PRODUCER);
                assert!(offset + 4 <= self.size);
                crate::ev(crate::Ev::Producer(value));
                Ok(())
            }
        }
        fn gva(object: &FwObject) -> Result<u64> {
            object.gva.ok_or(EINVAL)
        }
        #[derive(Default)]
        pub struct PbStats {
            pub requests: u64,
            pub grown: u64,
            pub refused: u64,
            pub bad: u64,
            pub waits: u64,
            pub refills: u64,
            pub reclaims: u64,
            pub freed: u64,
        }
        pub struct Live {
            pub block_ctl: FwObject,
        }
        pub struct Pools {
            pub ledger: Ledger,
            pub exhausted_logged: bool,
            pub stats: PbStats,
            pub pb_blocks: FwObject,
            pub live: Live,
            pub drm: ARef<AsahiDevice>,
            pub chunks: KVec<PbChunk>,
            pub idle: growth::IdleClock,
            pub reclaim_pending: bool,
        }
        impl Pools {
            pub fn chunk_count(&self) -> usize {
                self.chunks.len()
            }
            pub fn chunk_capacity(&self) -> usize {
                self.chunks.capacity()
            }
            /// The backing objects' ids, in heap order.
            pub fn chunk_ids(&self) -> Vec<u32> {
                self.chunks.iter().map(|c| c.backing.gem.id).collect()
            }
        }
        pub struct Queue(pub std::collections::VecDeque<[u32; 14]>);
        impl Queue {
            pub fn len(&self) -> usize {
                self.0.len()
            }
            pub fn is_empty(&self) -> bool {
                self.0.is_empty()
            }
            pub fn remove(&mut self, i: usize) -> core::result::Result<[u32; 14], ()> {
                assert_eq!(i, 0);
                self.0.pop_front().ok_or(())
            }
            pub fn insert_within_capacity(&mut self, i: usize, q: [u32; 14]) -> core::result::Result<(), [u32; 14]> {
                assert_eq!(i, 0);
                self.0.push_front(q);
                Ok(())
            }
        }
        pub struct Events {
            pub grow: Queue,
        }
        /// The firmware side: its requests, and whether the next one arrives during a reply.
        pub struct FwModel {
            pub ctx: u32,
            pub epoch: u64,
            pub next_counter: u32,
            pub ask_after_reply: bool,
            pub reply_fails: bool,
            pub replies_this_pass: u32,
        }
        impl FwModel {
            pub fn request(&mut self) -> [u32; 14] {
                let mut q = [0u32; 14];
                q[buffer::grow_request::TYPE] = 7;
                q[buffer::grow_request::CONTEXT] = self.ctx;
                q[buffer::grow_request::SLOT] = 0;
                q[buffer::grow_request::COUNTER] = self.next_counter;
                q[buffer::grow_request::POSITION] = 0x100 + self.next_counter;
                q[buffer::grow_request::EPOCH] = self.epoch as u32;
                q[buffer::grow_request::EPOCH + 1] = (self.epoch >> 32) as u32;
                self.next_counter += 1;
                q
            }
        }
        pub struct Firmware {
            pub events: Events,
            pub pools: Pools,
            pub init: Init,
            pub engine: Engine,
            pub recovery: Recovery,
            pub vms: Registry,
            pub renders: Vec<u32>,
            pub fw: FwModel,
        }
        pub const HEAP: u64 = super::hw::client::PB_HEAP;
        impl Firmware {
            /// The VM with context 5 runs: one flight, its PB backing mapped (map_pb, as at the
            /// start of its prefix's publication).
            pub fn new() -> Self {
                let vm = VmState::new(5);
                let mut f = Self::without_flights();
                f.engine.flights.push(Flight { job: Job { vm: vm.clone() } });
                f.vms.vms.push(vm.clone());
                f.map_pb(&vm).unwrap();
                crate::take();
                f
            }
            /// Pools::new's PB backing: the initial chunks and the reserve, room for the heap.
            pub fn without_flights() -> Self {
                let drm = ARef::new(AsahiDevice);
                let (ledger, chunks) = pools_new_backing(&drm).unwrap();
                Firmware {
                    events: Events { grow: Queue(Default::default()) },
                    pools: Pools {
                        ledger,
                        exhausted_logged: false,
                        idle: growth::IdleClock::default(),
                        reclaim_pending: false,
                        stats: PbStats::default(),
                        drm,
                        chunks,
                        pb_blocks: FwObject {
                            gva: Some(super::hw::client::PB_BLOCK_LIST),
                            size: buffer::BLOCK_LIST_SIZE,
                            failures: Cell::new(0),
                        },
                        live: Live {
                            block_ctl: FwObject { gva: None, size: block_ctl::SIZE, failures: Cell::new(0) },
                        },
                    },
                    init: Init { control: Some(std::sync::Arc::new(Ctl { epoch: 0x1_0000_0003 })) },
                    engine: Engine { flights: Vec::new(), pending: Vec::new(), frozen: false },
                    recovery: Recovery { reinit: false, armed: None, failed: None },
                    vms: Registry { vms: Vec::new() },
                    renders: vec![1],
                    fw: FwModel {
                        ctx: 5,
                        epoch: 0x1_0000_0003,
                        next_counter: 0,
                        ask_after_reply: false,
                        reply_fails: false,
                        replies_this_pass: 0,
                    },
                }
            }
            pub fn dev(&self) -> &() {
                &()
            }
            pub fn receipts(&self, i: usize) -> Receipts {
                Receipts { renders: self.renders[i] }
            }
            pub fn devctl(&mut self, reply: &DevCtl, wait: bool) -> Result {
                assert!(wait);
                crate::ev(crate::Ev::Reply(reply.words));
                self.fw.replies_this_pass += 1;
                assert!(self.fw.replies_this_pass <= 64, "unbounded answering in one poll pass");
                // The reply's wait drains the rings: the firmware's next request may queue now.
                if self.fw.ask_after_reply {
                    let q = self.fw.request();
                    self.events.grow.0.push_back(q);
                }
                if self.fw.reply_fails {
                    Err(EIO)
                } else {
                    Ok(())
                }
            }
            pub fn queue(&mut self) {
                let q = self.fw.request();
                self.events.grow.0.push_back(q);
            }
            /// One poll pass.
            pub fn pass(&mut self) {
                self.fw.replies_this_pass = 0;
                assert_eq!(self.answer_growth(), Ok(()));
            }
            /// What Pools::init() does to the ledger.
            pub fn reset_ledger(&mut self) {
                let p = &mut self.pools;
                p.RESET
            }
        }
'''

TESTS = r'''
#[cfg(test)]
mod pb_growth_tests {
    use crate::g15::firmware::{Firmware, VmState, Flight, Job, HEAP};
    use crate::g15::hw::{client, pte};
    use crate::g15::fw::buffer::{BLOCK_BYTES, MAX_BLOCKS};
    use crate::g15::pools::growth::{CHUNK_BLOCKS, MAX_CHUNKS};
    use crate::g15::pools::{chunk_gva, CHUNK_BYTES};
    use crate::g15::pgtable::Prot;
    use crate::{EPROTO, FAIL_ALLOC, FAIL_MAP};
    use std::sync::Arc;
    use crate::g15::fw::buffer::{self, grow_reply, BLOCK_LIST_RECORDS, GROW_STEP};
    use crate::g15::fw::channels::devctl;
    use crate::g15::hw::tuning;
    use crate::g15::pools::growth::Ledger;
    use crate::g15::{params, OldLedger};
    use crate::{take, Ev, ENOMEM, PARAM, PARAM_DEFAULT};

    const CAP: u32 = BLOCK_LIST_RECORDS - 1;
    /// Refusals answered at the maximum, as the commit states them.
    const REFUSALS: u32 = 2;

    fn clamp(v: u32, lo: u32, hi: u32) -> u32 {
        v.max(lo).min(hi)
    }

    fn soft(param: u32, initial: u32, max: u32) -> u32 {
        clamp(clamp(param, 64, tuning::PB_MAX_BLOCKS), initial, max)
    }

    /// Event logs compared with a short message at the first difference.
    fn same(got: &[Ev], want: &[Ev]) {
        if let Some(i) = (0..got.len().max(want.len())).find(|&i| got.get(i) != want.get(i)) {
            panic!("event {i} of {}/{}: got {:?}, want {:?}", got.len(), want.len(), got.get(i), want.get(i));
        }
    }

    fn level(e: &Ev) -> Option<&'static str> {
        match e {
            Ev::Log(l, _) => Some(l),
            _ => None,
        }
    }

    /// The accesses, replies and log levels a ledger at `posted` gives for the firmware's
    /// requests from counter 0 on, if it keeps asking after every reply: growth to the maximum,
    /// MAX_REFUSALS refusals, then one unanswered request.
    fn expected(initial: u32, max: u32, soft_max: u32) -> (Vec<Ev>, u32, u32) {
        let mut out = Vec::new();
        let (mut posted, mut refusals, mut logged_refusal) = (initial, 0, false);
        let mut counter = 0;
        loop {
            assert!(counter < 10_000, "the expected trace does not end");
            if refusals >= REFUSALS {
                out.push(Ev::Log("warn", String::new()));
                break;
            }
            let n = GROW_STEP.min(max - posted);
            for b in posted..posted + n {
                out.push(Ev::Record(8 * b as usize, buffer::block_record(HEAP, b) as u64));
            }
            if n != 0 {
                out.push(Ev::Sync);
                out.push(Ev::Producer(posted + n));
                out.push(Ev::Sync);
            }
            let mut r = [0u32; 14];
            r[grow_reply::OPCODE] = devctl::GROW_REPLY;
            r[grow_reply::GREW] = (n != 0) as u32;
            r[grow_reply::CONTEXT] = 5;
            r[grow_reply::COUNTER] = counter;
            r[grow_reply::POSITION] = 0x100 + counter;
            r[grow_reply::EPOCH] = 3;
            r[grow_reply::EPOCH + 1] = 1;
            out.push(Ev::Reply(r));
            posted += n;
            let first_refusal = n == 0 && !logged_refusal;
            if n == 0 {
                refusals += 1;
                logged_refusal = true;
            }
            let info = counter < 2 || (n != 0 && posted > soft_max) || first_refusal;
            out.push(Ev::Log(if info { "info" } else { "dbg" }, String::new()));
            counter += 1;
        }
        (out, posted, counter + 1)
    }

    fn levels_only(evs: Vec<Ev>) -> Vec<Ev> {
        evs.into_iter()
            .filter(|e| !is_backing(e) && !matches!(e, Ev::Log(_, m) if m.starts_with("PB heap:")))
            .map(|e| match e {
                Ev::Log(l, _) => Ev::Log(l, String::new()),
                e => e,
            })
            .collect()
    }

    /// Growth to the maximum with the firmware asking after every reply, one pass at a time.
    fn run_to_exhaustion(param: u32) {
        defaults();
        PARAM.with(|p| p.set(param));
        let mut f = Firmware::new();
        let (initial, max) = (f.pools.ledger.initial, f.pools.ledger.max);
        assert_eq!(f.pools.soft_max(), soft(param, initial, max));
        f.fw.ask_after_reply = true;
        f.queue();
        take();
        let mut passes = 0;
        while !f.events.grow.is_empty() {
            let queued = f.events.grow.len();
            let before = f.pools.stats.requests;
            f.pass();
            passes += 1;
            // Exactly the requests queued before the pass were taken.
            assert_eq!(f.pools.stats.requests - before, queued as u64);
            assert!(passes <= 1000, "growth did not end");
        }
        let log = take();
        let (want, posted, requests) = expected(initial, max, soft(param, initial, max));
        same(&levels_only(log.clone()), &want);
        assert_eq!(passes, requests);
        assert_eq!(f.pools.ledger.posted, posted);
        assert_eq!(posted, max);
        // Every producer store: increasing, in 1..=4095, the last one the maximum.
        let producers: Vec<u32> = log
            .iter()
            .filter_map(|e| if let Ev::Producer(p) = e { Some(*p) } else { None })
            .collect();
        assert!(producers.windows(2).all(|w| w[0] < w[1]));
        assert!(producers.iter().all(|p| (1..=CAP).contains(p)));
        assert_eq!(*producers.last().unwrap(), max);
        // Every block above the initial ones was written once, below the list's end.
        let records: Vec<usize> = log
            .iter()
            .filter_map(|e| if let Ev::Record(o, _) = e { Some(*o) } else { None })
            .collect();
        assert_eq!(records.len() as u32, max - initial);
        assert!(records.iter().all(|o| o + 8 <= buffer::BLOCK_LIST_SIZE));
        let s = &f.pools.stats;
        assert_eq!((s.grown, s.refused, s.bad), ((max - initial) as u64, REFUSALS as u64, 1));
        let warn: Vec<&String> = log
            .iter()
            .filter_map(|e| if let Ev::Log("warn", m) = e { Some(m) } else { None })
            .collect();
        assert_eq!(warn.len(), 1);
        assert!(warn[0].contains(&format!("left unanswered: {REFUSALS} refusals at the maximum ({max} blocks)")));
        // A later request: no reply, counted, no second warning.
        f.queue();
        f.pass();
        let log = take();
        assert!(log.is_empty(), "later request after exhaustion: {log:?}");
        assert_eq!(f.pools.stats.bad, 2);
        // A new ledger, as Pools::init() makes, answers again.
        f.reset_ledger();
        f.queue();
        f.pass();
        let log = take();
        assert_eq!(log.iter().filter(|e| matches!(e, Ev::Producer(_))).count(), 1);
        assert!(log.contains(&Ev::Producer(initial + GROW_STEP.min(max - initial))));
    }

    #[test]
    fn driver_values() {
        defaults();
        let l = Ledger::new(tuning::PB_GROW, params::pb_blocks(), tuning::PB_MAX_BLOCKS);
        assert!(tuning::PB_GROW);
        assert_eq!(crate::PB_BLOCKS_DEFAULT, 64);
        assert_eq!((l.initial, l.max, l.posted), (64, 4095, 64));
        assert_eq!(PARAM_DEFAULT, 3072);
        PARAM.with(|p| p.set(PARAM_DEFAULT));
        assert_eq!(params::pb_max_blocks(), 3072);
        assert_eq!(Firmware::new().pools.soft_max(), 3072);
    }

    #[test]
    fn soft_max_bounds() {
        defaults();
        let f = Firmware::new();
        for param in [0, 1, 63, 64, 65, 1023, 1024, 1025, 3071, 3072, 3073, 4094, 4095, 4096, 8192, u32::MAX] {
            PARAM.with(|p| p.set(param));
            assert_eq!(params::pb_max_blocks(), clamp(param, 64, tuning::PB_MAX_BLOCKS), "pb_max_blocks, param {param}");
            // Below the initial 64 the firmware still gets 64.
            assert_eq!(f.pools.soft_max(), clamp(param, 64, CAP), "soft_max, param {param}");
        }
        // With a larger initial posting the firmware gets at least that.
        crate::PARAM_BLOCKS.with(|b| b.set(1024));
        let f = Firmware::new();
        assert_eq!(f.pools.ledger.initial, 1024);
        for param in [0, 64, 1023, 1024, 3072, 4095, u32::MAX] {
            PARAM.with(|p| p.set(param));
            assert_eq!(f.pools.soft_max(), clamp(param, 1024, CAP), "soft_max, param {param}, 1024 initial");
        }
    }

    #[test]
    fn default_growth_to_exhaustion() {
        run_to_exhaustion(PARAM_DEFAULT);
    }

    #[test]
    fn soft_max_boundaries() {
        // 3074 is reached exactly (64 + 301 * 10): that answer is not past it.
        for param in [0, 1024, 3072, 3074, 3075, 4094, 4095, u32::MAX] {
            run_to_exhaustion(param);
        }
    }

    #[test]
    fn queued_snapshot_keeps_order() {
        let mut f = Firmware::new();
        f.fw.ask_after_reply = true;
        f.queue();
        f.queue();
        take();
        let mapped = f.pools.ledger.backed as u64;
        f.pass();
        let log = take();
        let replies: Vec<u32> = log
            .iter()
            .filter_map(|e| if let Ev::Reply(r) = e { Some(r[grow_reply::COUNTER]) } else { None })
            .collect();
        assert_eq!(replies, vec![0, 1], "replies in the first pass");
        // Compare both requests' complete record/barrier/producer/reply/log sequences,
        // not just their reply order. Each request finishes before the next one's records.
        let (want, _, _) = expected(f.pools.ledger.initial, f.pools.ledger.max, f.pools.soft_max());
        let end = want.iter().position(|e| matches!(e, Ev::Reply(r) if r[grow_reply::COUNTER] == 1))
            .expect("second expected reply") + 2; // Include its log too.
        same(&levels_only(log), &want[..end]);
        let queued: Vec<u32> = f.events.grow.0.iter().map(|q| q[buffer::grow_request::COUNTER]).collect();
        assert_eq!(queued, vec![2, 3]);
        assert_eq!(f.pools.stats.requests, 2);
        f.pass();
        assert_eq!(f.pools.stats.requests, 4);
        assert_eq!(f.events.grow.len(), 2);
    }

    #[test]
    fn empty_queue_does_nothing() {
        let mut f = Firmware::new();
        take();
        f.pass();
        assert!(take().is_empty());
        assert_eq!(f.pools.stats.requests, 0);
    }

    #[test]
    fn unprocessed_request_does_not_end_the_pass() {
        let mut f = Firmware::new();
        f.queue();
        f.queue();
        f.pools.pb_blocks.failures.set(1);
        take();
        f.pass();
        let log = take();
        // The second request was answered in the same pass, from the same posted count.
        assert_eq!(f.pools.stats.requests, 2, "requests taken in the pass");
        assert_eq!(f.pools.stats.bad, 1);
        assert!(log.iter().any(|e| matches!(e, Ev::Log("err", m) if m.contains("not answered: -5"))));
        assert!(log.contains(&Ev::Producer(64 + GROW_STEP)));
        assert!(f.events.grow.is_empty());
    }

    #[test]
    fn failed_reply_is_logged_and_counted_as_grown() {
        let mut f = Firmware::new();
        f.fw.reply_fails = true;
        f.queue();
        f.queue();
        f.queue();
        take();
        f.pass();
        let log = take();
        assert_eq!(f.pools.ledger.posted, 64 + 3 * GROW_STEP);
        assert_eq!((f.pools.stats.grown, f.pools.stats.bad), (3 * GROW_STEP as u64, 0));
        // Each failed reply is logged at info level.
        let info = log.iter().filter(|e| matches!(e, Ev::Log("info", m) if m.contains("DEVCTL 7: -5"))).count();
        assert_eq!(info, 3);
    }

    #[test]
    fn invalid_requests_get_no_reply() {
        let mut f = Firmware::new();
        let good = f.fw.request();
        let mut cases = Vec::new();
        let mut q = good;
        q[buffer::grow_request::TYPE] = 6;
        cases.push(q);
        let mut q = good;
        q[buffer::grow_request::SLOT] = 1;
        cases.push(q);
        let mut q = good;
        q[buffer::grow_request::EPOCH] = 2;
        cases.push(q);
        let mut q = good;
        q[buffer::grow_request::CONTEXT] = 6;
        cases.push(q);
        for q in cases {
            f.events.grow.0.push_back(q);
        }
        take();
        f.pass();
        let log = take();
        assert!(log.iter().all(|e| matches!(e, Ev::Log("warn", _))), "{log:?}");
        assert_eq!(log.len(), 4);
        assert_eq!(f.pools.stats.bad, 4);
        let mut fresh = Ledger::new(tuning::PB_GROW, params::pb_blocks(), tuning::PB_MAX_BLOCKS);
        fresh.backed = Ledger::chunks_for(fresh.initial) * CHUNK_BLOCKS;
        assert_eq!(f.pools.ledger, fresh);
        // No render published: not live.
        f.renders[0] = 0;
        f.events.grow.0.push_back(good);
        f.pass();
        assert!(!take().iter().any(|e| matches!(e, Ev::Reply(_))));
        // A stale counter after an answer.
        f.renders[0] = 1;
        f.events.grow.0.push_back(good);
        f.pass();
        assert!(take().iter().any(|e| matches!(e, Ev::Reply(_))));
        f.events.grow.0.push_back(good);
        f.pass();
        assert!(!take().iter().any(|e| matches!(e, Ev::Reply(_))));
        assert_eq!(f.pools.stats.bad, 6);
    }

    fn is_backing(e: &Ev) -> bool {
        matches!(e, Ev::Alloc(..) | Ev::Free(_) | Ev::Map(..) | Ev::Unmap(..))
    }

    /// A VM's PB aliases: exactly the first `chunks` chunks, in order, each the whole chunk at
    /// its GVA with the PB leaf, each of the chunk's object.
    fn assert_cursor(f: &Firmware, vm: &VmState, chunks: usize) {
        let heap = vm.heap.list.lock();
        assert_eq!(heap.len(), chunks, "VM aliases differ from the chunks");
        let ids = f.pools.chunk_ids();
        for (i, m) in heap.iter().enumerate() {
            assert_eq!(m._mapping.gva, chunk_gva(i), "aliases not contiguous from the heap's start");
            assert_eq!(m._mapping.size, CHUNK_BYTES);
            assert_eq!(m._mapping.prot, pte::LEAF_CACHED);
            assert_eq!(m.gem.id, ids[i], "alias of another object");
        }
    }

    /// In the order of `log`: no block record and no producer past the blocks the VM with
    /// context `slot` has mapped at that point (`start` blocks before the log).
    fn assert_posted_backed(log: &[Ev], slot: u32, start: u64) {
        let mut mapped = start;
        for e in log {
            match e {
                Ev::Map(s, gva, size, _) if *s == slot => {
                    assert_eq!(*gva, client::PB_HEAP + mapped * BLOCK_BYTES, "map out of order");
                    mapped += *size as u64 / BLOCK_BYTES;
                }
                Ev::Unmap(s, gva) if *s == slot => {
                    mapped = mapped.min((gva - client::PB_HEAP) / BLOCK_BYTES);
                }
                Ev::Record(off, _) => {
                    assert!((*off as u64 / 8) < mapped, "record of block {} with {} mapped", off / 8, mapped);
                }
                Ev::Producer(p) => {
                    assert!(*p as u64 <= mapped, "producer {} with {} mapped", p, mapped);
                }
                _ => {}
            }
        }
    }

    fn count(log: &[Ev], f: impl Fn(&Ev) -> bool) -> usize {
        log.iter().filter(|e| f(e)).count()
    }

    /// The firmware asks until the ledger reaches `target` posted blocks, one request a pass.
    fn grow_to(f: &mut Firmware, target: u32) {
        for _ in 0..1000 {
            if f.pools.ledger.posted >= target {
                return;
            }
            f.queue();
            f.pass();
        }
        panic!("growth stopped at {} blocks", f.pools.ledger.posted);
    }

    #[test]
    fn startup_backs_the_initial_blocks() {
        let mut f = Firmware::without_flights();
        let log = take();
        let (initial, max) = (f.pools.ledger.initial, f.pools.ledger.max);
        let chunks = Ledger::chunks_for(initial) as usize;
        assert_eq!(chunks, 2);
        let want: Vec<Ev> = (1..=chunks as u32).map(|id| Ev::Alloc(id, CHUNK_BYTES)).collect();
        assert_eq!(log, want, "startup allocation");
        assert_eq!((f.pools.ledger.backed, f.pools.chunk_count()), (chunks as u32 * CHUNK_BLOCKS, chunks));
        assert_eq!(f.pools.chunk_capacity(), MAX_CHUNKS as usize);
        assert!(max <= MAX_CHUNKS * CHUNK_BLOCKS);
        // The first prefix of a VM maps the backing, nothing else.
        let vm = VmState::new(7);
        f.engine.flights.push(Flight { job: Job { vm: vm.clone() } });
        f.map_pb(&vm).unwrap();
        let want: Vec<Ev> = (0..chunks).map(|i| Ev::Map(7, chunk_gva(i), CHUNK_BYTES, i as u32 + 1)).collect();
        assert_eq!(take(), want);
        assert_cursor(&f, &vm, chunks);
        // Mapping again maps nothing.
        f.map_pb(&vm).unwrap();
        assert!(take().is_empty());
    }

    #[test]
    fn pb_leaf_is_gpu_only() {
        let prot = Prot::from_pte(pte::LEAF_CACHED);
        assert!(prot.ap_is_gpu() && !prot.is_cached_noncoherent(), "PB leaf not GPU-only");
    }

    #[test]
    fn growth_backs_before_posting() {
        defaults();
        let mut f = Firmware::new();
        let start = f.pools.ledger.backed as u64;
        let first_chunks = f.pools.chunk_count();
        f.fw.ask_after_reply = true;
        f.queue();
        take();
        let mut passes = 0;
        while !f.events.grow.is_empty() {
            f.pass();
            passes += 1;
            assert!(passes <= 1000, "growth did not end");
        }
        let log = take();
        assert_posted_backed(&log, 5, start);
        // 16 MiB chunks, backed as the reserve needs them: up to the whole heap, no further.
        let allocs: Vec<usize> = log
            .iter()
            .filter_map(|e| if let Ev::Alloc(_, b) = e { Some(*b) } else { None })
            .collect();
        assert!(allocs.iter().all(|b| *b <= 16 << 20), "a growth chunk above 16 MiB");
        assert_eq!(CHUNK_BLOCKS as u64 * BLOCK_BYTES, 16 << 20);
        assert_eq!(allocs.len() + first_chunks, MAX_CHUNKS as usize);
        assert_eq!(f.pools.ledger.backed, MAX_BLOCKS);
        assert_eq!(f.pools.stats.waits, 0);
        assert_eq!(f.pools.stats.refills, allocs.len() as u64);
        assert_eq!(count(&log, |e| matches!(e, Ev::Free(_) | Ev::Unmap(..))), 0);
        assert_cursor(&f, &f.engine.flights[0].job.vm, f.pools.chunk_count());
    }

    #[test]
    fn allocation_follows_the_replies() {
        let mut f = Firmware::new();
        // Below the reserve line: the pass's answers come first, then the chunk.
        f.pools.ledger.posted = f.pools.ledger.backed - growth_reserve() + 1;
        f.queue();
        f.queue();
        take();
        f.pass();
        let log = take();
        let last_reply = log.iter().rposition(|e| matches!(e, Ev::Reply(_))).expect("replies");
        let first_alloc = log.iter().position(|e| matches!(e, Ev::Alloc(..))).expect("a refill");
        assert!(first_alloc > last_reply, "allocation before a reply: {log:?}");
        assert_eq!(count(&log, |e| matches!(e, Ev::Reply(_))), 2);
        assert_eq!(f.pools.stats.refills, 1);
    }

    fn growth_reserve() -> u32 {
        crate::g15::pools::growth::RESERVE_BLOCKS
    }

    #[test]
    fn dry_reserve_waits_and_keeps_order() {
        let mut f = Firmware::new();
        let start = f.pools.ledger.backed as u64;
        FAIL_ALLOC.with(|c| c.set(1000));
        let mut log = Vec::new();
        let mut waited = None;
        for _ in 0..40 {
            f.queue();
            f.pass();
            let pass = take();
            let replied = pass.iter().any(|e| matches!(e, Ev::Reply(_)));
            log.extend(pass);
            if !replied {
                waited = Some(f.fw.next_counter - 1);
                break;
            }
        }
        assert_posted_backed(&log, 5, start);
        let waited = waited.expect("no request waited");
        assert_eq!(f.pools.ledger.posted, f.pools.ledger.backed);
        let queued: Vec<u32> = f.events.grow.0.iter().map(|q| q[buffer::grow_request::COUNTER]).collect();
        assert_eq!(queued, vec![waited], "waiting request lost");
        assert_eq!(f.pools.ledger.next_request, waited);
        assert!(f.pools.stats.waits >= 1);
        // The allocator recovers: this pass still waits, then backs a chunk; the next answers.
        FAIL_ALLOC.with(|c| c.set(0));
        f.pass();
        let log = take();
        assert!(!log.iter().any(|e| matches!(e, Ev::Reply(_))));
        assert_eq!(count(&log, |e| matches!(e, Ev::Alloc(..))), 1);
        assert_eq!(count(&log, |e| matches!(e, Ev::Map(5, ..))), 1);
        let mapped = f.pools.ledger.backed as u64;
        f.pass();
        let log = take();
        let replies: Vec<u32> = log
            .iter()
            .filter_map(|e| if let Ev::Reply(r) = e { Some(r[grow_reply::COUNTER]) } else { None })
            .collect();
        assert_eq!(replies, vec![waited]);
        assert!(f.events.grow.is_empty());
        assert_posted_backed(&log, 5, mapped);
    }

    #[test]
    fn refill_map_failure_drops_the_chunk() {
        let mut f = Firmware::new();
        let chunks = f.pools.chunk_count();
        let backed = f.pools.ledger.backed;
        f.pools.ledger.posted = backed - growth_reserve() + 1;
        FAIL_MAP.with(|c| c.set(1));
        take();
        f.pass();
        let log = take();
        assert_eq!(f.pools.chunk_count(), chunks, "chunk kept after a failed map");
        let backing: Vec<&Ev> = log.iter().filter(|e| is_backing(e)).collect();
        assert!(matches!(backing[..], [Ev::Alloc(id, _), Ev::Free(free)] if id == free), "{backing:?}");
        assert_eq!(f.pools.ledger.backed, backed);
        assert!(log.iter().any(|e| matches!(e, Ev::Log("warn", m) if m.contains("backing chunk"))));
        assert_cursor(&f, &f.engine.flights[0].job.vm, chunks);
        // The next pass backs it.
        f.pass();
        assert_eq!(f.pools.chunk_count(), chunks + 1);
        assert_cursor(&f, &f.engine.flights[0].job.vm, chunks + 1);
    }

    /// A pending re-initialization with nothing published (a reclaim, or a recovery that
    /// failed the flights): the posted count is about to drop, so nothing is backed.
    #[test]
    fn pending_reinit_without_flights_backs_nothing() {
        let mut f = Firmware::without_flights();
        f.pools.ledger.posted = f.pools.ledger.backed - growth_reserve() + 1;
        f.recovery.reinit = true;
        take();
        f.pass();
        assert_eq!(count(&take(), is_backing), 0, "backed during a pending re-initialization");
        f.recovery.reinit = false;
        f.pass();
        assert_eq!(f.pools.stats.refills, 1);
    }

    /// A pending re-initialization with only compute published (no render can ask for
    /// growth): nothing is backed.
    #[test]
    fn pending_reinit_with_compute_only_backs_nothing() {
        let mut f = Firmware::new();
        f.renders[0] = 0;
        f.pools.ledger.posted = f.pools.ledger.backed;
        f.recovery.reinit = true;
        take();
        for _ in 0..4 {
            f.pass();
        }
        assert_eq!(count(&take(), is_backing), 0, "backed for a compute-only flight");
        assert_eq!(f.pools.stats.refills, 0);
    }

    /// A render published before the re-initialization was set (the flights behind a fragment
    /// kill) still asks for growth: with its reserve used up, the refill must still back
    /// chunks, or the request waits until the deadline.
    #[test]
    fn pending_reinit_with_a_live_render_refills() {
        let mut f = Firmware::new(); // one live render
        let start = f.pools.ledger.backed as u64;
        f.pools.ledger.posted = f.pools.ledger.backed;
        f.recovery.reinit = true;
        f.queue();
        take();
        for _ in 0..4 {
            f.pass();
        }
        let s = &f.pools.stats;
        assert!(
            f.events.grow.is_empty(),
            "growth request stranded: posted={} backed={} waits={} refills={}",
            f.pools.ledger.posted,
            f.pools.ledger.backed,
            s.waits,
            s.refills
        );
        // The request waits one pass for the chunk; the grant's pass tops the reserve up.
        assert_eq!((s.waits, s.refills, s.requests), (1, 2, 1));
        assert_posted_backed(&take(), 5, start);
    }

    #[test]
    fn transport_failure_keeps_everything() {
        let mut f = Firmware::new();
        let chunks = f.pools.chunk_count();
        f.fw.reply_fails = true;
        f.queue();
        f.pass();
        let log = take();
        assert!(log.contains(&Ev::Producer(74)));
        assert_eq!((f.pools.ledger.posted, f.pools.chunk_count()), (74, chunks));
        assert_cursor(&f, &f.engine.flights[0].job.vm, chunks);
        assert_eq!(count(&log, |e| matches!(e, Ev::Free(_) | Ev::Unmap(..))), 0);
    }

    #[test]
    fn other_vms_catch_up_before_they_run() {
        let mut f = Firmware::new();
        let a = f.engine.flights[0].job.vm.clone();
        grow_to(&mut f, 200);
        take();
        let chunks = f.pools.chunk_count();
        assert_eq!(chunks, 3);
        // A's work retires; a new VM runs next: it maps every chunk first.
        f.engine.flights.clear();
        let b = VmState::new(6);
        f.engine.flights.push(Flight { job: Job { vm: b.clone() } });
        f.fw.ctx = 6;
        f.map_pb(&b).unwrap();
        let log = take();
        assert_eq!(count(&log, |e| matches!(e, Ev::Map(6, ..))), chunks);
        assert_eq!(count(&log, |e| matches!(e, Ev::Alloc(..))), 0);
        assert_cursor(&f, &b, chunks);
        // B grows the backing; A, behind, catches up when it runs again.
        grow_to(&mut f, 300);
        let log = take();
        assert_posted_backed(&log, 6, chunks as u64 * CHUNK_BLOCKS as u64);
        assert_eq!(count(&log, |e| matches!(e, Ev::Map(5, ..))), 0);
        assert_eq!(f.pools.chunk_count(), chunks + 1);
        f.engine.flights.clear();
        f.engine.flights.push(Flight { job: Job { vm: a.clone() } });
        f.map_pb(&a).unwrap();
        assert_cursor(&f, &a, f.pools.chunk_count());
    }

    #[test]
    fn partial_catch_up_keeps_the_aliases_made() {
        let mut f = Firmware::new();
        grow_to(&mut f, 200);
        let chunks = f.pools.chunk_count();
        f.engine.flights.clear();
        let b = VmState::new(6);
        f.engine.flights.push(Flight { job: Job { vm: b.clone() } });
        take();
        // The third alias cannot be made: two held.
        crate::MAP_OK_BEFORE_FAIL.with(|c| c.set(2));
        FAIL_MAP.with(|c| c.set(1));
        assert_eq!(f.map_pb(&b), Err(ENOMEM));
        assert_cursor(&f, &b, 2);
        f.map_pb(&b).unwrap();
        assert_cursor(&f, &b, chunks);
    }

    /// The ledger reset of Pools::init(), not a full recovery.
    #[test]
    fn ledger_reset_reuses_the_backing() {
        let mut f = Firmware::new();
        grow_to(&mut f, 300);
        let chunks = f.pools.chunk_count();
        take();
        f.reset_ledger();
        assert_eq!(f.pools.ledger.posted, 64);
        grow_to(&mut f, 300);
        let log = take();
        assert_eq!(count(&log, |e| matches!(e, Ev::Alloc(..) | Ev::Map(..))), 0);
        assert_eq!(f.pools.chunk_count(), chunks);
        assert_posted_backed(&log, 5, f.pools.ledger.backed as u64);
    }

    #[test]
    fn rejected_requests_allocate_nothing() {
        let mut f = Firmware::new();
        let mut q = f.fw.request();
        q[buffer::grow_request::EPOCH] = 2; // a halt epoch that has passed
        f.events.grow.0.push_back(q);
        f.pass();
        let log = take();
        assert_eq!(count(&log, is_backing), 0);
        assert_eq!(f.pools.stats.bad, 1);
    }

    /// Every PB parameter at its default. libtest gives each test its own thread, but a test
    /// that sets one starts from the defaults all the same.
    fn defaults() {
        PARAM.with(|p| p.set(PARAM_DEFAULT));
        crate::PARAM_BLOCKS.with(|b| b.set(crate::PB_BLOCKS_DEFAULT));
        crate::PARAM_RECLAIM.with(|r| r.set(crate::PB_RECLAIM_DEFAULT));
        crate::PARAM_IDLE_MS.with(|r| r.set(crate::PB_IDLE_MS_DEFAULT));
        crate::PARAM_RECLAIM_BLOCKS.with(|r| r.set(crate::PB_RECLAIM_BLOCKS_DEFAULT));
    }

    /// A prefix of `vm` published as engine.rs does it: publish_prefix_into() first catches the
    /// VM up (map_pb(), before the flight is added), and a render prefix with a
    /// re-initialization pending does the InitBM (Pools::init()'s ledger reset) and clears
    /// `reinit` at its publication. `render`: the prefix has renders.
    fn publish(f: &mut Firmware, vm: &Arc<VmState>, render: bool) {
        f.map_pb(vm).unwrap();
        if render && f.recovery.reinit {
            f.reset_ledger();
            f.recovery.reinit = false;
        }
        f.engine.flights.push(Flight { job: Job { vm: vm.clone() } });
        let i = f.engine.flights.len() - 1;
        if f.renders.len() <= i {
            f.renders.resize(i + 1, 0);
        }
        f.renders[i] = render as u32;
        f.fw.ctx = vm.slot;
        f.note_idle();
    }

    /// What a reclaim leaves behind in a VM that was not walked (a closed VM no longer in the
    /// registry, whose accepted jobs may still run): its aliases of freed chunks go, and chunks
    /// backed again at the same GVA are mapped from their new objects at its next publication.
    /// The reclaims are the actual idle reclaim; the posted count stays until the InitBM.
    #[test]
    fn freed_chunks_are_mapped_again() {
        defaults();
        let mut f = Firmware::new();
        let a = f.engine.flights[0].job.vm.clone();
        grow_to(&mut f, 1100);
        let (posted, chunks) = (f.pools.ledger.posted, f.pools.chunk_count());
        assert!(chunks > 2);
        // A's work retires and A leaves the registry; the idle reclaim frees the chunks above
        // the kept two without walking A.
        f.engine.flights.clear();
        f.vms.vms.clear();
        let t0 = now();
        check_at(&mut f, t0);
        check_at(&mut f, t0 + 1_000_000_000);
        assert_eq!(f.pools.chunk_count(), 2);
        assert_eq!(f.pools.ledger.posted, posted, "posted count reset before the InitBM");
        take();
        // B runs: its first render re-initializes; it grows the backing again, new objects at
        // the old GVAs.
        let b = VmState::new(6);
        f.vms.vms.push(b.clone());
        publish(&mut f, &b, true);
        assert_eq!(f.pools.ledger.posted, 64);
        grow_to(&mut f, 1100);
        assert_eq!(f.pools.chunk_count(), chunks);
        // A runs again: its stale aliases are replaced by the new objects'.
        f.engine.flights.clear();
        take();
        publish(&mut f, &a, true);
        let log = take();
        assert_cursor(&f, &a, chunks);
        assert_eq!(count(&log, |e| matches!(e, Ev::Unmap(5, _))), chunks - 2);
        assert_eq!(count(&log, |e| matches!(e, Ev::Map(5, ..))), chunks - 2);
        // Another idle reclaim walks B, not A: with fewer chunks than A maps, A's aliases past
        // the backing go at its next catch-up.
        f.engine.flights.clear();
        let t1 = now();
        check_at(&mut f, t1);
        check_at(&mut f, t1 + 1_000_000_000);
        assert_eq!(f.pools.chunk_count(), 2);
        assert_cursor(&f, &b, 2);
        f.map_pb(&a).unwrap();
        assert_cursor(&f, &a, 2);
    }

    /// A reclaim whose walk of the VM registry fails: the chunks leave the backing, but a VM's
    /// aliases still hold their pages, so nothing is counted freed; the VM drops its stale
    /// aliases at its next publication, and only then are the pages released.
    #[test]
    fn failed_vm_walk_counts_nothing_freed() {
        defaults();
        let mut f = Firmware::new();
        let a = f.engine.flights[0].job.vm.clone();
        grow_to(&mut f, 1500);
        let chunks = f.pools.chunk_count();
        f.engine.flights.clear();
        let t0 = now();
        check_at(&mut f, t0);
        take();
        crate::FAIL_OPEN.with(|c| c.set(1));
        check_at(&mut f, t0 + 1_000_000_000);
        let log = take();
        assert_eq!(f.pools.chunk_count(), 2);
        assert_eq!(f.pools.stats.freed, 0, "freed counted with the VM walk failed");
        assert!(log.iter().any(|e| matches!(e, Ev::Log("warn", m) if m.contains("could not walk the VMs"))));
        assert!(log.iter().any(|e| matches!(e, Ev::Log("info", m) if m.contains(" 0 chunks freed"))));
        assert_eq!(count(&log, |e| matches!(e, Ev::Free(_))), 0, "pages released while aliases hold them");
        assert_eq!(a.heap.list.lock().len(), chunks);
        // A's next publication drops the stale aliases: the pages go.
        publish(&mut f, &a, true);
        let log = take();
        assert_eq!(count(&log, |e| matches!(e, Ev::Free(_))), chunks - 2);
        assert_cursor(&f, &a, 2);
    }

    /// One client idles 60 ms after a heavy scene (more than half the maximum posted) and
    /// renders again: the client-boundary re-initialization was set at 50 ms with the backing
    /// kept, the posted count stays until the render's InitBM, which posts the initial blocks
    /// again, and the scene grows again from the kept backing with nothing allocated or mapped.
    #[test]
    fn same_client_render_after_60ms() {
        defaults();
        let mut f = Firmware::new();
        let a = f.engine.flights[0].job.vm.clone();
        grow_to(&mut f, 2100);
        let (posted, chunks) = (f.pools.ledger.posted, f.pools.chunk_count());
        f.engine.flights.clear();
        let t0 = now();
        check_at(&mut f, t0);
        check_at(&mut f, t0 + 60_000_000);
        assert!(f.recovery.reinit && f.pools.reclaim_pending);
        assert_eq!(
            (f.pools.ledger.posted, f.pools.chunk_count()),
            (posted, chunks),
            "posted count reset before the InitBM"
        );
        take();
        publish(&mut f, &a, true);
        assert_eq!(f.pools.ledger.posted, 64);
        assert!(!f.recovery.reinit && !f.pools.reclaim_pending);
        grow_to(&mut f, 2100);
        let log = take();
        assert_eq!(count(&log, is_backing), 0, "backing changed: {:?}", log.iter().filter(|e| is_backing(e)).collect::<Vec<_>>());
        assert_posted_backed(&log, 5, f.pools.ledger.backed as u64);
    }

    /// One client idles 1.1 s and renders again, after a scene above the reclaim floor (the
    /// idle reclaim at 1 s) or above half the maximum (the client boundary at 50 ms, then the
    /// free at 1 s): the backing above the kept chunks was freed and unmapped from the client's
    /// VM, the posted count stays until the render's InitBM, and the scene grows again with
    /// every chunk backed and mapped before a record or producer names it.
    #[test]
    fn same_client_render_after_1_1s() {
        for target in [1500, 2100] {
            defaults();
            let mut f = Firmware::new();
            let a = f.engine.flights[0].job.vm.clone();
            grow_to(&mut f, target);
            let posted = f.pools.ledger.posted;
            f.engine.flights.clear();
            let t0 = now();
            for dt in [0, 50_000_000, 1_000_000_000, 1_100_000_000] {
                check_at(&mut f, t0 + dt);
            }
            assert_eq!(f.pools.chunk_count(), 2, "not freed ({target})");
            assert_eq!(f.pools.ledger.posted, posted, "posted count reset before the InitBM ({target})");
            assert_cursor(&f, &a, 2);
            take();
            publish(&mut f, &a, true);
            assert_eq!(f.pools.ledger.posted, 64);
            grow_to(&mut f, target);
            let log = take();
            assert_posted_backed(&log, 5, 2 * CHUNK_BLOCKS as u64);
            assert_cursor(&f, &a, f.pools.chunk_count());
        }
    }

    /// A compute-only submission between a level-2 reclaim and the next render's InitBM: its
    /// publication maps nothing, its passes back and post nothing (the posted count is still
    /// the old one, above the kept backing), and a growth request naming its context is left
    /// unanswered; once it retires, the render after it re-initializes as usual.
    #[test]
    fn compute_between_reclaim_and_initbm() {
        defaults();
        let mut f = Firmware::new();
        let a = f.engine.flights[0].job.vm.clone();
        grow_to(&mut f, 1500);
        let posted = f.pools.ledger.posted;
        f.engine.flights.clear();
        let t0 = now();
        check_at(&mut f, t0);
        check_at(&mut f, t0 + 1_000_000_000);
        assert_eq!(f.pools.chunk_count(), 2);
        let bad = f.pools.stats.bad;
        take();
        publish(&mut f, &a, false);
        f.queue();
        for _ in 0..4 {
            f.pass();
        }
        let log = take();
        let touched: Vec<&Ev> = log
            .iter()
            .filter(|e| is_backing(e) || matches!(e, Ev::Record(..) | Ev::Producer(_) | Ev::Reply(_)))
            .collect();
        assert!(touched.is_empty(), "compute-only flight backed or posted: {touched:?}");
        assert!(f.recovery.reinit && f.pools.ledger.posted == posted);
        assert_eq!(f.pools.stats.bad, bad + 1);
        // The compute retires; the check finds nothing more to do; the next render re-inits.
        f.engine.flights.clear();
        check_at(&mut f, t0 + 2_000_000_000);
        assert_eq!(count(&take(), is_backing), 0);
        publish(&mut f, &a, true);
        assert_eq!(f.pools.ledger.posted, 64);
        grow_to(&mut f, 300);
        assert_posted_backed(&take(), 5, 2 * CHUNK_BLOCKS as u64);
    }

    fn now() -> u64 {
        crate::NOW.with(|n| n.get())
    }

    /// The idle clock at `t` and the reclaim check at that time, as a job arrival or an idle
    /// poll pass runs it.
    fn check_at(f: &mut Firmware, t: u64) {
        crate::NOW.with(|n| n.set(t));
        f.note_idle();
        f.reclaim_check();
    }

    /// A heavy client (more than half the maximum posted) exits and nothing renders after it:
    /// the 50 ms client-boundary re-initialization keeps the backing, and a check after the
    /// long idle frees the backing above the kept chunks. The posted count stays until the next
    /// render's InitBM.
    #[test]
    fn client_boundary_reinit_frees_without_a_render() {
        let mut f = Firmware::new();
        grow_to(&mut f, 2100);
        let (posted, chunks) = (f.pools.ledger.posted, f.pools.chunk_count());
        assert!(posted > 2047 && chunks > 2);
        // The pass that retires the last flight notes the idle.
        f.engine.flights.clear();
        let t0 = now();
        check_at(&mut f, t0);
        take();
        check_at(&mut f, t0 + 50_000_000);
        assert!(f.recovery.reinit && f.pools.chunk_count() == chunks, "client-boundary re-initialization");
        check_at(&mut f, t0 + 999_999_999);
        assert_eq!(f.pools.chunk_count(), chunks, "freed before the long idle");
        check_at(&mut f, t0 + 1_000_000_000);
        let log = take();
        assert_eq!(f.pools.chunk_count(), 2, "kept backing not freed: {} chunks", f.pools.chunk_count());
        assert!(f.recovery.reinit && f.pools.reclaim_pending);
        assert_eq!(f.pools.stats.reclaims, 1);
        assert_eq!(f.pools.ledger.backed, 2 * CHUNK_BLOCKS);
        assert_eq!(f.pools.stats.freed, (chunks - 2) as u64);
        assert_eq!(f.pools.ledger.posted, posted, "posted count reset before the InitBM");
        assert_eq!(count(&log, |e| matches!(e, Ev::Unmap(5, _))), chunks - 2);
        assert_eq!(count(&log, |e| matches!(e, Ev::Free(_))), chunks - 2);
        // Later checks find nothing more to do.
        check_at(&mut f, t0 + 100_000_000_000);
        assert!(take().is_empty());
    }

    /// With g15_pb_reclaim 1 the client-boundary re-initialization keeps the backing for good.
    #[test]
    fn client_boundary_reinit_at_level_1_keeps_the_backing() {
        defaults();
        crate::PARAM_RECLAIM.with(|r| r.set(1));
        let mut f = Firmware::new();
        grow_to(&mut f, 2100);
        let chunks = f.pools.chunk_count();
        f.engine.flights.clear();
        let t0 = now();
        check_at(&mut f, t0);
        check_at(&mut f, t0 + 50_000_000);
        assert!(f.recovery.reinit);
        assert_eq!(f.reclaim_delay_ns(), None, "armed at level 1");
        check_at(&mut f, t0 + 5_000_000_000);
        assert_eq!(f.pools.chunk_count(), chunks, "freed at level 1");
        assert_eq!(f.pools.stats.freed, 0);
    }

    /// Nothing grown since the initialization: an initial posting above half the maximum
    /// (g15_pb_blocks 3000), or a floor below it (g15_pb_reclaim_blocks 32): no
    /// re-initialization at any idle.
    #[test]
    fn nothing_grown_reinitializes_nothing() {
        for (blocks, floor) in [(64, 32), (3000, 1024)] {
            defaults();
            crate::PARAM_BLOCKS.with(|b| b.set(blocks));
            crate::PARAM_RECLAIM_BLOCKS.with(|b| b.set(floor));
            let mut f = Firmware::new();
            assert_eq!(f.pools.ledger.posted, blocks);
            f.engine.flights.clear();
            let t0 = now();
            for dt in [0, 50_000_000, 1_000_000_000, 5_000_000_000] {
                check_at(&mut f, t0 + dt);
            }
            assert!(
                !f.recovery.reinit && f.pools.stats.reclaims == 0,
                "re-initialization with nothing grown ({blocks} blocks, floor {floor})"
            );
        }
    }

    /// A recovery's re-initialization (not a reclaim's) is left alone: nothing freed.
    #[test]
    fn recovery_reinit_frees_nothing() {
        let mut f = Firmware::new();
        grow_to(&mut f, 2100);
        let chunks = f.pools.chunk_count();
        f.engine.flights.clear();
        f.recovery.reinit = true;
        let t0 = now();
        check_at(&mut f, t0);
        assert_eq!(f.reclaim_delay_ns(), None, "armed for a recovery's re-initialization");
        check_at(&mut f, t0 + 5_000_000_000);
        assert_eq!(f.pools.chunk_count(), chunks, "freed for a recovery's re-initialization");
        assert_eq!(f.pools.stats.reclaims, 0);
    }

    /// The idle poll work as poll.rs drives it: a pass with nothing published notes the idle
    /// and runs the reclaim check; an idle pass is re-armed only for reclaim_delay_ns(),
    /// rounded up to whole ms, plus one 4 ms jiffy. Returns the wake times.
    fn idle_polls(f: &mut Firmware, limit: usize) -> Vec<u64> {
        let mut wakes = Vec::new();
        loop {
            f.note_idle();
            f.reclaim_check();
            let Some(ns) = f.reclaim_delay_ns() else {
                return wakes;
            };
            assert!(wakes.len() < limit, "poll woke in a loop: {} wakes, next in {ns} ns", wakes.len());
            let t = now() + ns.div_ceil(1_000_000) * 1_000_000 + 4_000_000;
            crate::NOW.with(|n| n.set(t));
            wakes.push(t);
        }
    }

    /// The same exit with the idle poll work: the poll is armed for the 50 ms
    /// re-initialization, then for the rest of the long idle (not the expired 50 ms), where the
    /// kept backing is freed; then nothing is armed.
    #[test]
    fn client_boundary_reinit_arms_the_long_idle() {
        let mut f = Firmware::new();
        grow_to(&mut f, 2100);
        let (posted, chunks) = (f.pools.ledger.posted, f.pools.chunk_count());
        assert!(posted > 2047 && chunks > 2);
        // The pass that retires the last flight notes the idle.
        f.engine.flights.clear();
        f.note_idle();
        let t0 = now();
        take();
        let wakes = idle_polls(&mut f, 8);
        let log = take();
        assert_eq!(
            f.pools.chunk_count(),
            2,
            "kept backing not freed: {} chunks, wakes {:?} ms",
            f.pools.chunk_count(),
            wakes.iter().map(|t| (t - t0) / 1_000_000).collect::<Vec<_>>()
        );
        assert!(f.recovery.reinit && f.pools.reclaim_pending);
        assert_eq!(f.pools.stats.reclaims, 1);
        assert_eq!(f.pools.ledger.backed, 2 * CHUNK_BLOCKS);
        assert_eq!(f.pools.stats.freed, (chunks - 2) as u64);
        assert_eq!(f.pools.ledger.posted, posted, "posted count reset before the InitBM");
        // Two wakes: the re-initialization after 50 ms, the free after the long idle.
        let ms: Vec<u64> = wakes.iter().map(|t| (t - t0) / 1_000_000).collect();
        assert_eq!(ms, vec![54, 1004], "wakes (ms after the idle began)");
        assert_eq!(count(&log, |e| matches!(e, Ev::Unmap(5, _))), chunks - 2);
        assert_eq!(count(&log, |e| matches!(e, Ev::Free(_))), chunks - 2);
        // Idle much longer, or a job arriving: nothing more is due.
        crate::NOW.with(|n| n.set(t0 + 100_000_000_000));
        f.reclaim_check();
        assert_eq!(f.reclaim_delay_ns(), None);
        assert!(take().is_empty());
    }

    /// Nothing grown: the idle poll work arms nothing.
    #[test]
    fn nothing_grown_arms_nothing() {
        for (blocks, floor) in [(64, 32), (3000, 1024)] {
            defaults();
            crate::PARAM_BLOCKS.with(|b| b.set(blocks));
            crate::PARAM_RECLAIM_BLOCKS.with(|b| b.set(floor));
            let mut f = Firmware::new();
            f.engine.flights.clear();
            let wakes = idle_polls(&mut f, 8);
            assert!(wakes.is_empty(), "armed with nothing grown ({blocks} blocks, floor {floor}): {} wakes", wakes.len());
            assert!(!f.recovery.reinit);
        }
    }

    /// An idle reclaim below half the maximum frees at the long idle; with its re-initialization
    /// pending nothing more is armed.
    #[test]
    fn idle_reclaim_arms_nothing_after() {
        let mut f = Firmware::new();
        grow_to(&mut f, 1500);
        let chunks = f.pools.chunk_count();
        f.engine.flights.clear();
        f.note_idle();
        let t0 = now();
        let wakes = idle_polls(&mut f, 8);
        let ms: Vec<u64> = wakes.iter().map(|t| (t - t0) / 1_000_000).collect();
        assert_eq!(ms, vec![1004]);
        assert_eq!((f.pools.chunk_count(), f.pools.stats.freed), (2, (chunks - 2) as u64));
        assert!(f.recovery.reinit && f.pools.reclaim_pending);
    }

    #[test]
    fn catch_up_refuses_flights_of_another_vm() {
        let mut f = Firmware::new();
        let b = VmState::new(6);
        f.engine.flights.push(Flight { job: Job { vm: b.clone() } });
        assert_eq!(f.map_pb(&b), Err(EPROTO), "catch-up with a flight of another VM");
        assert_eq!(b.heap.list.lock().len(), 0);
        assert_eq!(count(&take(), is_backing), 0);
    }

    #[test]
    fn constructor_is_the_old_one_capped() {
        let mut values: Vec<u32> = (0..=4200).collect();
        values.extend([8191, 8192, u32::MAX - 1, u32::MAX]);
        let mut changed = 0u64;
        let mut cases = 0u64;
        for grow in [true, false] {
            for &blocks in &values {
                for &max_blocks in &values {
                    let old = OldLedger::new(grow, blocks, max_blocks);
                    let new = Ledger::new(grow, blocks, max_blocks);
                    cases += 1;
                    assert_eq!(
                        (new.initial, new.max, new.posted),
                        (old.initial.min(CAP), old.max.min(CAP), old.posted.min(CAP)),
                        "new({grow}, {blocks}, {max_blocks})"
                    );
                    assert!((1..=CAP).contains(&(new.posted % BLOCK_LIST_RECORDS)));
                    if old.max <= CAP {
                        assert_eq!((new.initial, new.max), (old.initial, old.max));
                    } else {
                        changed += 1;
                    }
                }
            }
        }
        // Only inputs the old constructor took to the whole 4096-block heap changed.
        assert!(changed > 0 && changed < cases);
        // The driver's values are below the cap, so its ledger is the old one.
        let d = (tuning::PB_GROW, params::pb_blocks(), tuning::PB_MAX_BLOCKS);
        let (old, new) = (OldLedger::new(d.0, d.1, d.2), Ledger::new(d.0, d.1, d.2));
        if d.2 <= CAP {
            assert_eq!((old.initial, old.max, old.posted), (new.initial, new.max, new.posted));
        }
        println!("constructor cases {cases}, changed {changed}");
    }
}
'''


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--output', type=Path, help='Work directory (default: a temporary one)')
    p.add_argument('--ref', help='Read the tested bodies at this git revision')
    p.add_argument('--mutation', choices=sorted(MUTATIONS) + sorted(CONTROLS))
    p.add_argument('--mutations', action='store_true',
                   help='Also require every mutation to fail and every control to pass')
    a = p.parse_args()
    import tempfile
    out = (a.output or Path(tempfile.mkdtemp(prefix='g15-pb-growth-'))).resolve()
    out.mkdir(parents=True, exist_ok=True)

    def read(path, ref=None):
        ref = ref or a.ref
        return (subprocess.check_output(['git', 'show', f'{ref}:{path}'], cwd=ROOT, text=True,
                                        stderr=subprocess.DEVNULL)
                if ref else (ROOT / path).read_text())

    # The embedded reference is the commit's own text, when the commit is here.
    try:
        old_git = item(read(GROWTH.replace("drivers/gpu/drm/asahi/", "drivers/gpu/drm/asahi-m3/"), OLD_REF), '    pub(crate) const fn new(')
    except (subprocess.CalledProcessError, ValueError):
        old_git = None
    if old_git is not None and old_git != OLD_LEDGER_NEW:
        raise SystemExit('embedded Ledger::new() differs from ' + OLD_REF)

    def run(mutation):
        paths = (POOLS, GROWTH, BUFFER, CHANNELS, INITDATA, PARAMS, HW, CORE, ENGINE, PGTABLE, VM)
        src = {path: read(path) for path in paths}
        if mutation:
            spec = {**MUTATIONS, **CONTROLS}[mutation]
            for path, old, new in (spec if isinstance(spec, list) else [spec]):
                if src[path].count(old) != 1:
                    raise SystemExit(f'mutation {mutation}: anchor not found once')
                src[path] = src[path].replace(old, new)
        pools, hw = src[POOLS], src[HW]
        bodies = [item(pools, s) for s in ('    pub(crate) fn answer_growth(&mut self) -> Result {',
                                           '    fn answer_request(&mut self, request: &[u32; 14]) -> Result<bool> {',
                                           '    pub(crate) fn map_pb(&mut self, vm: &VmState) -> Result {',
                                           '    pub(crate) fn refill_backing(&mut self) {',
                                           '    pub(crate) fn note_idle(&mut self) {',
                                           '    pub(crate) fn reclaim_check(&mut self) {',
                                           '    fn free_backing(&mut self) -> usize {',
                                           '    pub(crate) fn reclaim_delay_ns(&self) -> Option<u64> {')]
        soft_max = item(pools, '    pub(crate) fn soft_max(&self) -> u32 {')
        pools_methods = (item(pools, '    fn map_heap(&self, vm: &VmState) -> Result {') + '\n' +
                         item(pools, '    fn free_chunks(&mut self, keep: usize) -> usize {') + '\n' +
                         item(pools, '    fn back_chunk(&mut self) -> Result<ARef<gem::Object>> {'))
        chunk = item(pools, 'struct PbChunk {') + '\n' + item(pools, 'impl PbChunk {')
        chunk_geometry = (re.search(r'^pub\(crate\) const CHUNK_BYTES: usize = [^;]+;', pools, re.M).group(0) + '\n' +
                          item(pools, 'pub(crate) const fn chunk_gva(i: usize) -> u64 {'))
        vm_src = src[VM]
        vm_methods = '\n'.join(item(vm_src, s) for s in (
            '    fn map_one(&self, heap: &mut KVec<HeapMapping>, i: usize, gem: ARef<gem::Object>) -> Result {',
            '    pub(crate) fn map_heap(&self, gems: impl Iterator<Item = ARef<gem::Object>>) -> Result {',
            '    pub(crate) fn map_chunk(&self, i: usize, gem: ARef<gem::Object>) -> Result {',
            '    pub(crate) fn unmap_heap_above(&self, keep: usize) {'))
        start = pools.index(POOLS_NEW_BACKING[0])
        end = pools.index(POOLS_NEW_BACKING[1], start)
        pools_new_backing = pools[start:end]
        if pools.count(PB_LEAF_ASSERT) != 1:
            raise SystemExit('pools.rs does not assert the PB leaf GPU-only')
        # Every prefix maps the PB into its VM before anything else.
        prefix_into = item(src[ENGINE], '    fn publish_prefix_into(&mut self, job: &Arc<Job>, f: &mut Flight) -> Result {')
        statements = [l.strip() for l in prefix_into.splitlines()[1:] if l.strip() and not l.strip().startswith('//')]
        if not statements[0].startswith('self.map_pb(&job.vm)'):
            raise SystemExit('publish_prefix_into does not start with the PB catch-up')
        pgtable = src[PGTABLE]
        pg_consts = '\n'.join(re.search(rf'^const {n}: \w+ = [^;]+;', pgtable, re.M).group(0)
                              for n in ('UAT_AP_SHIFT', 'UAT_AP_BITS', 'UAT_HIGH_BITS_SHIFT', 'UAT_MEMATTR_SHIFT',
                                        'UAT_MEMATTR_BITS', 'MEMATTR_CACHED', 'AP_GPU'))
        prot = (item(pgtable, 'pub(crate) struct Prot {') + '\nimpl Prot {\n' +
                item(pgtable, '    pub(crate) const fn from_pte(pte: u64) -> Self {') + '\n' +
                item(pgtable, '    pub(crate) const fn is_cached_noncoherent(&self) -> bool {') +
                '\n    pub fn ap_is_gpu(&self) -> bool { self.ap == AP_GPU }\n}\n')
        init = item(pools, '    fn init(&mut self) -> Result {')
        for line in LEDGER_RESET:
            if init.count(line) != 1:
                raise SystemExit('Pools::init() does not hold: ' + line)
        reset = '\n'.join('p.' + line[len('self.'):] for line in LEDGER_RESET)
        pb_max_blocks = '\n'.join(item(src[PARAMS], f'pub(crate) fn {name}() -> ')
                                   for name in ('pb_max_blocks', 'pb_blocks', 'pb_reclaim', 'pb_idle_ns',
                                                'pb_reclaim_blocks'))
        reclaim_defaults = {}
        for param, const in (('g15_pb_reclaim', 'PB_RECLAIM_DEFAULT'), ('g15_pb_idle_ms', 'PB_IDLE_MS_DEFAULT'),
                             ('g15_pb_reclaim_blocks', 'PB_RECLAIM_BLOCKS_DEFAULT')):
            m = re.search(rf'{param}: u32 \{{\s*default: (\d+),', src[CORE])
            if not m:
                raise SystemExit(f'{param} default not found')
            reclaim_defaults[const] = int(m.group(1))
        m = re.search(r'g15_pb_max_blocks: u32 \{\s*default: (\d+),', src[CORE])
        if not m:
            raise SystemExit('g15_pb_max_blocks default not found')
        param_default = int(m.group(1))
        m = re.search(r'g15_pb_blocks: u32 \{\s*default: (\d+),', src[CORE])
        if not m:
            raise SystemExit('g15_pb_blocks default not found')
        blocks_default = int(m.group(1))
        sizes = consts(item(hw, 'pub(crate) mod sizes {'), ['PB_BLOCKS'])
        tuning = consts(item(hw, 'pub(crate) mod tuning {'), ['PB_GROW', 'PB_MAX_BLOCKS'])
        client = consts(item(hw, 'pub(crate) mod client {'), ['PB_HEAP', 'PB_BLOCK_LIST'])
        pte = consts(item(hw, 'pub(crate) mod pte {'), ['LEAF_CACHED'])
        control = consts(item(src[INITDATA], 'pub(crate) mod control {'), ['HALT_COUNT'])
        growth = src[GROWTH]
        clamp_fn = item(growth, 'const fn clamp(v: u32, lo: u32, hi: u32) -> u32 {')
        tested = (''.join(bodies) + soft_max + pools_methods + chunk + chunk_geometry + vm_methods +
                  pools_new_backing + pb_max_blocks + growth +
                  src[BUFFER] + sizes + tuning + pte + prot)
        dir = out / (mutation or 'actual')
        dir.mkdir(exist_ok=True)
        for path in (GROWTH, BUFFER, CHANNELS):
            (dir / Path(path).name).write_text(src[path])
        code = (PREFIX + f'pub const PARAM_DEFAULT: u32 = {param_default};\n' +
                f'pub const PB_BLOCKS_DEFAULT: u32 = {blocks_default};\n' +
                ''.join(f'pub const {k}: u32 = {v};\n' for k, v in reclaim_defaults.items()) +
                'pub mod g15 {\n'
                '    pub mod fw {\n'
                f'        #[path = "{dir / "channels.rs"}"] pub mod channels;\n'
                f'        #[path = "{dir / "buffer.rs"}"] pub mod buffer;\n'
                '        pub mod initdata { pub mod control {\n' + control + '\n} }\n'
                '    }\n'
                f'    pub mod pools {{ #[path = "{dir / "growth.rs"}"] pub mod growth;\n'
                '        use super::fw::buffer;\n        use super::hw::client;\n' + chunk_geometry + '\n    }\n'
                '    pub mod hw {\n'
                '        pub mod sizes {\n' + sizes + '\n}\n'
                '        pub mod tuning {\n' + tuning + '\n}\n'
                '        pub mod client {\n' + client + '\n}\n'
                '        pub mod pte {\n' + pte + '\n}\n'
                '    }\n'
                '    pub mod pgtable {\n' + pg_consts + '\n' + prot + '\n    }\n'
                '    pub mod boot {\n        pub fn eproto() -> crate::Error { crate::EPROTO }\n    }\n'
                '    pub mod engine {\n        pub fn now_ns() -> u64 { crate::NOW.with(|n| n.get()) }\n    }\n'
                '    /// The checked backing allocation, failing where a test injects it.\n'
                '    pub mod object {\n'
                '        use crate::prelude::*;\n'
                '        pub struct HeldPages;\n'
                '        pub fn gpu_backing(_: &crate::driver::AsahiDevice, bytes: usize)\n'
                '            -> Result<(crate::gem::ObjectRef, HeldPages)> {\n'
                '            if crate::fail(&crate::FAIL_ALLOC) { return Err(ENOMEM); }\n'
                '            let id = crate::NEXT_ID.with(|n| { let id = n.get(); n.set(id + 1); id });\n'
                '            crate::ev(crate::Ev::Alloc(id, bytes));\n'
                '            Ok((crate::gem::ObjectRef { gem: crate::ARef::new(crate::gem::Object { id, size: bytes }) }, HeldPages))\n'
                '        }\n'
                '    }\n'
                '    pub mod params {\n        use crate::module_parameters;\n' + pb_max_blocks + '\n    }\n'
                '    /// Ledger::new() at ' + OLD_REF[:12] + '.\n'
                '    pub struct OldLedger { pub initial: u32, pub max: u32, pub posted: u32, pub next_request: u32,\n'
                '                           pub refusal_logged: bool, pub refusals: u32 }\n'
                '    use crate::g15::fw::buffer::{MAX_BLOCKS, MIN_BLOCKS};\n'
                '    impl OldLedger {\n' + OLD_LEDGER_NEW + '\n    }\n' + clamp_fn + '\n'
                '    pub mod firmware {\n'
                '        use crate::prelude::*;\n'
                '        use std::cell::Cell;\n'
                '        use super::fw::buffer::{self, block_ctl};\n'
                '        use super::fw::channels::DevCtl;\n'
                '        use super::fw::initdata::control;\n'
                '        use super::hw::{self, client, pte, tuning};\n'
                '        use super::pools::growth::{self, Answer, Ledger};\n'
                '        use super::pools::{chunk_gva, CHUNK_BYTES};\n'
                '        use super::object::{self, HeldPages};\n'
                '        use crate::driver::AsahiDevice;\n'
                '        use crate::{gem, mmu, ARef, Mutex};\n'
                '        use std::sync::Arc;\n' +
                MODEL.replace('p.RESET', reset).replace('POOLS_NEW_BACKING', '') +
                chunk + '\n'
                '        /// Pools::new\'s PB backing, verbatim.\n'
                '        pub fn pools_new_backing(dev: &AsahiDevice) -> Result<(Ledger, KVec<PbChunk>)> {\n' +
                pools_new_backing + '        Ok((ledger, chunks))\n        }\n'
                '        impl Pools {\n' + soft_max + '\n' + pools_methods + '\n        }\n'
                '        impl VmState {\n' + vm_methods + '\n        }\n'
                '        impl Firmware {\n' + '\n'.join(bodies) + '\n        }\n'
                '    }\n'
                '}\n' + TESTS)
        path = dir / 'growth-answer.rs'
        path.write_text(code)
        binary = dir / 'growth-answer'
        build = subprocess.run(['rustup', 'run', '1.93.1', 'rustc', '--edition=2021', '--test', '-O',
                                '-C', 'overflow-checks=on', str(path), '-o', str(binary)],
                               capture_output=True, text=True)
        if build.returncode != 0:
            raise SystemExit(f'{mutation or "actual"}: build failed\n{build.stderr[-3000:]}')

        def no_core():
            resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
        try:
            r = subprocess.run([str(binary), '--show-output', 'pb_growth_tests::'], capture_output=True,
                               text=True, preexec_fn=no_core, timeout=60)
        except subprocess.TimeoutExpired:
            # A hang is not a test failure: no test is reported failed, so no mutation counts.
            return {'mutation': mutation, 'exit': 'timeout', 'summary': [], 'failed': [], 'output': {},
                    'stderr': 'timed out after 60 s'}
        summary = [line for line in r.stdout.splitlines() if line.startswith('test result:')]
        failed = sorted(line.split()[1].removeprefix('pb_growth_tests::') for line in r.stdout.splitlines()
                        if line.startswith('test ') and line.endswith('FAILED'))
        # Each test's captured output: the panic message of a failure, the counts of a success.
        output = {m.group(1).removeprefix('pb_growth_tests::'): m.group(2).strip()
                  for m in re.finditer(r'---- (\S+) stdout ----\n(.*?)(?=\n---- |\n(?:failures|successes):\n|\Z)',
                                       r.stdout, re.S)}
        source_hashes = {path: hashlib.sha256(src[path].encode()).hexdigest() for path in paths}
        source_inputs = json.dumps(source_hashes, sort_keys=True, separators=(',', ':')).encode()
        return {'mutation': mutation, 'tested_sha256': hashlib.sha256(tested.encode()).hexdigest(),
                'source_sha256': source_hashes,
                'source_inputs_sha256': hashlib.sha256(source_inputs).hexdigest(),
                'exit': r.returncode, 'summary': summary, 'failed': failed, 'output': output,
                'stderr': r.stderr[-2000:]}

    result = run(a.mutation)
    receipt = {'ref': a.ref, 'old_ref': OLD_REF, 'old_reference_checked_against_git': old_git is not None,
               **result,
               'scope': 'Actual answer_growth/answer_request/map_pb/refill_backing/soft_max/pb_max_blocks, '
                        'the PB chunk and VM heap-mapping bodies, pools/growth.rs, '
                        'fw/buffer.rs and fw/channels.rs over a model of the firmware objects with an '
                        'ordered access log, and Ledger::new against its body at '
                        f'{OLD_REF[:12]}. Not kernel, firmware or GPU execution, timing, or recovery.'}
    if a.mutations and not a.mutation:
        receipt['mutations'] = {}
        for m in sorted(MUTATIONS):
            res = run(m)
            test, message = INTENDED[m]
            got = res['output'].get(test, '')
            receipt['mutations'][m] = {'exit': res['exit'], 'failed': res['failed'], 'intended_test': test,
                                       'intended_failure': test in res['failed'] and message in got,
                                       'message': '\n'.join(l for l in got.splitlines()
                                                            if not l.startswith('note:'))[-600:]}
        receipt['controls'] = {}
        for m in sorted(CONTROLS):
            res = run(m)
            receipt['controls'][m] = {'exit': res['exit'], 'failed': res['failed'], 'summary': res['summary']}
    (out / 'result.json').write_text(json.dumps(receipt, indent=2) + '\n')
    print(json.dumps({k: v for k, v in receipt.items() if k not in ('output', 'stderr')}, indent=2))
    # The actual bodies and the controls must pass; a single requested mutation must fail.
    expect_fail = a.mutation in MUTATIONS
    if (receipt['exit'] != 0) != expect_fail:
        raise SystemExit(1)
    if expect_fail:
        test, message = INTENDED[a.mutation]
        if test not in receipt['failed'] or message not in receipt['output'].get(test, ''):
            raise SystemExit('the mutation timed out or failed elsewhere than intended')
    if a.mutations and any(v['exit'] == 0 or not v['intended_failure']
                           for v in receipt.get('mutations', {}).values()):
        raise SystemExit('a mutation survived or failed elsewhere than intended')
    if a.mutations and any(v['exit'] != 0 for v in receipt.get('controls', {}).values()):
        raise SystemExit('a control failed')


if __name__ == '__main__':
    main()
