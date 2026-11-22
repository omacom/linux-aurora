#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""G17 render slot quarantine: the pair's drain witness ends it, nothing earlier.

A render group whose completion stamps never come retires on its pass record
when the watchdog expires its stamp wait, and its pair fails behind it in the
same device-mutex hold, before the deferred retirement runs. The quarantine
then reaches admission through the group's lease, and the retirement gives that
lease back with the client's result. Only a lease ended a slot's quarantine, so
the slot stayed out of service until the processors stopped: one slot per event
under capacity stress (render_slots quarantined=1, held=0, unsettled=0 on the
11-16 Neo release candidate, 12 minutes after the stress).

The real admission state, leases and their quarantine, the real completion
lease handling and deferred finishes, the real pair quarantine and drain
release (queue/render.rs) and the runtime's drain release (runtime/render/
settle.rs) are compiled against a mock pair whose firmware drain witness the
test sets. The slot comes back once the witness released the pair, and not
before it; a drained command still gives its VM pin back before its slot
returns; the last lease ends a quarantine whose pair drained while leases were
left; a failure after the witness is not covered by it; a witness of a pair
that was never quarantined changes nothing. --mutation removes one part of
that and must fail.
"""
from pathlib import Path
import argparse
import os
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2] / 'drivers/gpu/drm/asahi'
G17 = ROOT / 'g17'


def item(source, signature):
    """The item that starts at `signature`, through its closing brace."""
    assert source.count(signature) == 1, signature
    start = source.index(signature)
    begin = source.index('{', start)
    depth, end = 1, begin + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


def ordered(source, *parts):
    """`parts` occur in `source` once each, in this order."""
    at = -1
    for part in parts:
        assert source.count(part) == 1, part
        assert source.index(part) > at, part
        at = source.index(part)


def fixture():
    admission = (G17 / 'queue/admission.rs').read_text()
    completion = (G17 / 'completion.rs').read_text()
    render = (G17 / 'queue/render.rs').read_text()
    settle = (G17 / 'runtime/render/settle.rs').read_text()
    hw = (ROOT / 'hw/t8140.rs').read_text()
    pairs = (G17 / 'pairs.rs').read_text()

    # What the mocks below stand for, in the real code.
    # A stampless retirement defers its Retired before it keeps the group for its stamps.
    ordered(item(render, 'fn retire_front_command('), 'if stampless {\n            retirement::note_stampless(',
            'defer(Deferred::Retired(', 'front.stamps_owed = true;')
    # The watchdog path retires and fails the pair in one hold; its batch finishes after it.
    ordered(item(settle, 'pub(in crate::g17) fn timeout_render('),
            'firmware.fail_render_packet(packet, ETIMEDOUT, &mut deferred)', 'deferred.finish();')
    ordered(item(settle, 'pub(super) fn fail_render_packet('), 'pair.expire_stamp_wait(packet, stamps, state)?;',
            'pair.retire(None, generation, state, host,', 'pair.fail_after_observation(packet, error, host,')
    # A first signal gives the lease back without ending a quarantine (the `complete` mock).
    ordered(item(completion, 'fn complete_with_compute('),
            'if self.signalled.swap(true, Ordering::AcqRel) {\n            return;\n        }',
            'self.release_healthy();')
    # The drain release runs once the drain proof, event service and accounting passed.
    ordered(item(settle, 'fn drain_render_slot('), 'pair.firmware_drained(host)', 'self.service_runtime_events(deferred)',
            'pair.account_drained(host)', 'self.release_drained_render(slot, deferred)')

    constants = '\n'.join(re.search(r'pub\(crate\) const ' + name + r': [^;]+;', hw).group(0)
                          for name in ['RENDER_SLOTS', 'RENDER_PAIRS_MAX', 'RENDER_DEPTH'])
    at = pairs.index('const SIBLING_BACKLOG')
    backlog = pairs[at:pairs.index('\n}\n', pairs.index('fn sibling_backlog', at)) + 3]
    state = '\n'.join(item(admission, name) for name in [
        'const fn new(', 'fn exhausted(', 'fn owner_quarantine(', 'fn refusal(', 'fn advance(',
        'fn take_idle(', 'fn acquire(', 'fn claim_sibling(', 'fn usable(', 'fn position(', 'fn release(',
        'fn rehabilitate(', 'fn pair_drained(&mut self', 'fn retained(', 'fn used('])
    pool = '\n'.join(item(admission, name) for name in [
        'pub(crate) fn release_owner(', 'pub(crate) fn release_retained_owners_after_processor_stop(',
        'pub(crate) fn pair_drained(&self'])
    lease = '\n'.join(item(admission, name) for name in [
        'pub(crate) fn slot(&self) -> u8', 'pub(crate) fn quarantine(&self, spared: bool)',
        'pub(crate) fn rehabilitate_after_processor_stop(', 'pub(crate) fn rehabilitate_after_firmware_drain('])
    command = '\n'.join(item(completion, name) for name in [
        'pub(crate) fn quarantine_render(', 'pub(crate) fn release_after_retirement(', 'fn release_healthy(',
        'pub(crate) fn fail_unpublished(', 'pub(crate) fn fail_while_owned('])
    pair = '\n'.join(item(render, name) for name in [
        'pub(crate) fn quarantine(\n', 'pub(crate) fn release_witnessed('])
    kunit = item(admission, 'fn a_drained_pair_ends_a_quarantine_no_lease_is_left_to_end(')
    return r'''
#![allow(dead_code, unused_macros, unused_imports, unused_variables, unused_mut)]
use std::collections::VecDeque;
use std::sync::{Arc, atomic::{AtomicBool, AtomicUsize, Ordering, Ordering::SeqCst}};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Error(i32);
type Result<T = ()> = std::result::Result<T, Error>;
const EIO: Error = Error(-5);
const EBUSY: Error = Error(-16);
const ENOSPC: Error = Error(-28);
const ENODATA: Error = Error(-61);
const ETIMEDOUT: Error = Error(-110);
const ECANCELED: Error = Error(-125);

mod g17 {
    pub mod trace { pub fn teardown(_: u8, _: u64, _: u64, _: u64) {} }
    pub mod capacity {
        pub enum Site { RenderSlots }
        pub fn refused(_: Site) -> crate::Error { crate::ENOSPC }
    }
    pub mod pairs {
        use std::sync::atomic::{AtomicU8, Ordering::Relaxed};
''' + backlog + r'''
    }
}
mod cfg {
''' + constants + r'''
}
mod recovery { pub enum Class { Spared } }

/// The kernel mutex: `lock` returns the guard.
pub struct Mutex<T>(std::sync::Mutex<T>);
impl<T> Mutex<T> {
    fn new(value: T) -> Self { Mutex(std::sync::Mutex::new(value)) }
    fn lock(&self) -> std::sync::MutexGuard<'_, T> { self.0.lock().unwrap() }
}
type KVec<T> = Vec<T>;
struct InFlight { owner: u64, sequence: u64, slot: Option<u8>, ticket: u64 }

''' + item(admission, 'struct State {') + '\nimpl State {\n' + re.search(r'    const MASK: u64 = .*;', admission).group(0) \
        + '\n' + state + r'''
}
pub struct Pool { state: Mutex<State>, wakes: AtomicUsize }
impl Pool {
    fn returned(&self) { self.wakes.fetch_add(1, SeqCst); }
''' + pool + '\n}\n' + item(admission, 'pub(crate) struct RenderLease {') + '\nimpl RenderLease {\n' + lease + '\n}\n' \
        + item(admission, 'impl Drop for RenderLease {') + r'''

pub struct Status;
impl Status {
    fn record(&self, _: Error) {}
    fn record_if_device_loss(&self, _: Error) {}
    fn report_failure(&self, _: Error) {}
}
pub struct Output;
impl Output { fn finish(&self, _: Result<[u64; 4]>) {} }
pub struct Sources;
impl Sources { fn release(&self) {} }

/// A command's VM pin. It notes whether its slot was still quarantined when it went.
pub struct VmJobGuard { pool: Arc<Pool>, slot: u8, quarantined: Arc<Mutex<Option<bool>>>, vm: Arc<AtomicBool> }
impl VmJobGuard { fn quarantine(&self) { self.vm.store(true, SeqCst); } }
impl Drop for VmJobGuard {
    fn drop(&mut self) {
        let quarantined = self.pool.state.lock().quarantined & (1u64 << self.slot) != 0;
        *self.quarantined.lock() = Some(quarantined);
    }
}

pub struct Completion {
    status: Arc<Status>,
    output: Output,
    sources: Option<Arc<Sources>>,
    vm_job: Mutex<Option<VmJobGuard>>,
    render: Mutex<Option<RenderLease>>,
    signalled: AtomicBool,
    accepted: AtomicBool,
    spared: AtomicBool,
    result: Mutex<Option<Result<[u64; 4]>>>,
}
fn discarded(error: Error) -> bool { error == ECANCELED || error == ENODATA }
impl Completion {
    fn spared(&self) -> bool { self.spared.load(SeqCst) }
    fn classify(&self, _: recovery::Class) { self.spared.store(true, SeqCst); }
    fn status(&self) -> &Arc<Status> { &self.status }
    fn report(&self, _: Error) {}
    fn signal_terminal(&self, result: Result<[u64; 4]>) { *self.result.lock() = Some(result); }
    /// The lease handling of the real complete_with_compute (checked against it above).
    fn complete(&self, result: Result<[u64; 4]>) { self.complete_with_compute(result, None) }
    fn complete_with_compute(&self, result: Result<[u64; 4]>, _: Option<[u64; 2]>) {
        if self.signalled.swap(true, Ordering::AcqRel) {
            return;
        }
        self.release_healthy();
        self.signal_terminal(result);
    }
''' + command + '\n}\n' + item(completion, 'pub(crate) enum Deferred {') + '\n' + item(completion, 'impl Deferred {') + r'''

pub struct Packet { completion: Arc<Completion> }
pub struct Ticket { ordinal: u64 }
pub struct Active {
    packet: Arc<Packet>,
    ticket: Ticket,
    stamps_owed: bool,
    failed: bool,
    flist_completed: bool,
    drained_result: Option<Result<[u64; 4]>>,
}
/// The pair's fixed tracker.
pub struct Ring<T>(VecDeque<T>);
impl<T> Ring<T> {
    fn iter(&self) -> std::collections::vec_deque::Iter<'_, T> { self.0.iter() }
    fn iter_mut(&mut self) -> std::collections::vec_deque::IterMut<'_, T> { self.0.iter_mut() }
    fn front(&self) -> Option<&T> { self.0.front() }
    fn front_mut(&mut self) -> Option<&mut T> { self.0.front_mut() }
    fn pop(&mut self) -> Option<T> { self.0.pop_front() }
    fn len(&self) -> usize { self.0.len() }
}
pub struct Memory { retired: Vec<u64> }
impl Memory { fn retire(&mut self, ordinal: u64) -> Result { self.retired.push(ordinal); Ok(()) } }
/// The firmware state behind `Pair::firmware_drained`: cursors done == write, free-list
/// blocks returned, and stamps covering the last publication or the context's kill acknowledged.
pub trait Host { fn drained(&self) -> bool; }
pub struct FakeHost { drained: bool }
impl Host for FakeHost { fn drained(&self) -> bool { self.drained } }
pub struct Pair {
    quarantined: bool,
    active: Ring<Active>,
    terminal: Option<(Arc<Status>, Error)>,
    memory: Memory,
    drain_failures: u32,
    drain_retry_at: u64,
}
impl Pair {
    fn firmware_drained(&self, host: &impl Host) -> Result<bool> { Ok(host.drained()) }
    fn resync(&mut self) {}
    // The cross-pair task-stage order is not modelled here (KUnit asahi_g17_render_cross_pair_order).
    fn release_task_order_if_held(&self) {}
''' + pair + r'''
}

pub struct DeferredBatch { values: Vec<Deferred> }
impl DeferredBatch {
    fn push(&mut self, value: Deferred) -> Result { self.values.push(value); Ok(()) }
    /// Off the device mutex.
    fn finish(&mut self) { for value in self.values.drain(..) { value.finish(); } }
}
pub struct Queues { admission: Arc<Pool> }
pub struct Firmware { queues: Queues, pair: Option<Box<Pair>>, host: FakeHost }
impl Firmware {
    fn with_render<R>(&mut self, _slot: usize, operation: impl FnOnce(&mut Pair, &mut FakeHost) -> Result<R>) -> Result<R> {
        let mut pair = self.pair.take().ok_or(EBUSY)?;
        let result = operation(&mut pair, &mut self.host);
        self.pair = Some(pair);
        result
    }
''' + item(settle, 'fn release_drained_render(') + r'''
}

''' + kunit + r'''

const SLOTS: usize = cfg::RENDER_SLOTS;
const OWNER: u64 = 5324;
const NEW: u64 = 7777;
const TICKS: [u64; 4] = [10, 10, 10, 20];
fn bit(slot: u8) -> u64 { 1u64 << slot }

/// Every slot is retained by an open owner; slot 19 by OWNER. The queue table is full, so a
/// new owner can only be given a slot an owner closed.
fn pool() -> Arc<Pool> {
    let mut state = State::new();
    state.build_room = false;
    for slot in 1..SLOTS {
        state.owners[slot] = 1000 + slot as u64;
    }
    state.owners[19] = OWNER;
    Arc::new(Pool { state: Mutex::new(state), wakes: AtomicUsize::new(0) })
}
fn lease(pool: &Arc<Pool>, owner: u64) -> RenderLease {
    let slot = pool.state.lock().acquire(owner, 1).expect("the owner's slot");
    RenderLease { pool: pool.clone(), slot, record: None }
}
struct Command { completion: Arc<Completion>, pin: Arc<Mutex<Option<bool>>>, vm: Arc<AtomicBool> }
fn command(pool: &Arc<Pool>, lease: RenderLease) -> Command {
    let pin = Arc::new(Mutex::new(None));
    let vm = Arc::new(AtomicBool::new(false));
    let guard = VmJobGuard { pool: pool.clone(), slot: lease.slot(), quarantined: pin.clone(), vm: vm.clone() };
    let completion = Arc::new(Completion {
        status: Arc::new(Status), output: Output, sources: Some(Arc::new(Sources)),
        vm_job: Mutex::new(Some(guard)), render: Mutex::new(Some(lease)),
        signalled: AtomicBool::new(false), accepted: AtomicBool::new(true), spared: AtomicBool::new(false),
        result: Mutex::new(None),
    });
    Command { completion, pin, vm }
}
fn firmware(pool: &Arc<Pool>, commands: &[(&Command, u64)]) -> Firmware {
    let active = commands.iter().map(|(command, ordinal)| Active {
        packet: Arc::new(Packet { completion: command.completion.clone() }),
        ticket: Ticket { ordinal: *ordinal },
        stamps_owed: false, failed: false, flist_completed: false, drained_result: None,
    }).collect();
    let pair = Pair { quarantined: false, active: Ring(active), terminal: None, memory: Memory { retired: Vec::new() },
                      drain_failures: 0, drain_retry_at: 0 };
    Firmware { queues: Queues { admission: pool.clone() }, pair: Some(Box::new(pair)), host: FakeHost { drained: false } }
}
fn batch() -> DeferredBatch { DeferredBatch { values: Vec::new() } }
/// retire_front_command's stampless branch: the front retires on its pass record.
fn stampless_retirement(firmware: &mut Firmware, deferred: &mut DeferredBatch) {
    let front = firmware.pair.as_mut().unwrap().active.front_mut().unwrap();
    front.flist_completed = true;
    deferred.push(Deferred::Retired(front.packet.completion.clone(), Ok(TICKS))).unwrap();
    front.stamps_owed = true;
}
fn quarantine(firmware: &mut Firmware, error: Error, deferred: &mut DeferredBatch) {
    firmware.pair.as_mut().unwrap().quarantine(error, &mut |event| deferred.push(event)).unwrap();
}
fn quarantined(pool: &Pool, slot: u8) -> bool { pool.state.lock().quarantined & bit(slot) != 0 }
fn wakes(pool: &Pool) -> usize { pool.wakes.load(SeqCst) }
fn result(command: &Command) -> Option<Result<[u64; 4]>> { *command.completion.result.lock() }

/// The 11-16 sequence: the watchdog expires slot 19's stamp wait, the group retires on its
/// pass record and the pair fails behind it in one hold; the owner closes; the kill is
/// acknowledged and the pair drains.
fn watchdog_expiry() {
    let pool = pool();
    let command = command(&pool, lease(&pool, OWNER));
    let slot = command.completion.render.lock().as_ref().unwrap().slot();
    assert_eq!(slot, 19);
    let mut firmware = firmware(&pool, &[(&command, 413)]);
    let mut deferred = batch();
    stampless_retirement(&mut firmware, &mut deferred);
    quarantine(&mut firmware, ETIMEDOUT, &mut deferred);
    assert!(quarantined(&pool, slot), "the failure reaches admission through the still-attached lease");
    deferred.finish();
    assert_eq!(result(&command), Some(Ok(TICKS)), "the client has its result");
    assert!(command.completion.render.lock().is_none() && command.completion.vm_job.lock().is_none());
    assert_eq!(pool.state.lock().refs[19], 0);
    assert_eq!(pool.state.lock().active & bit(slot), 0, "no lease of the slot is left");
    assert!(quarantined(&pool, slot));
    pool.release_owner(OWNER);
    // Before the firmware's drain witness nothing returns the slot.
    firmware.release_drained_render(19, &mut deferred).unwrap();
    deferred.finish();
    assert!(firmware.pair.as_ref().unwrap().quarantined && firmware.pair.as_ref().unwrap().active.len() == 1);
    assert!(quarantined(&pool, slot), "no slot returns before its pair's drain witness");
    assert_eq!(pool.state.lock().acquire(NEW, 1), None, "no new owner is given the slot before the witness");
    let before = wakes(&pool);
    firmware.host.drained = true;
    firmware.release_drained_render(19, &mut deferred).unwrap();
    let pair = firmware.pair.as_ref().unwrap();
    assert!(!pair.quarantined && pair.active.len() == 0 && pair.memory.retired == [413]);
    assert!(!quarantined(&pool, slot), "the pair's drain witness returns the slot no lease is left to return");
    assert_eq!(wakes(&pool), before + 1, "and wakes the waiters");
    assert!(deferred.values.is_empty(), "an owed group's result was reported already");
    assert_eq!(pool.state.lock().acquire(NEW, 1), Some(slot), "a new owner is given the slot again");
}

/// A failed command still holds its lease and its quarantined VM pin when the witness comes.
fn drained_command() {
    let pool = pool();
    let command = command(&pool, lease(&pool, OWNER));
    let slot = command.completion.render.lock().as_ref().unwrap().slot();
    let mut firmware = firmware(&pool, &[(&command, 7)]);
    let mut deferred = batch();
    quarantine(&mut firmware, EIO, &mut deferred);
    deferred.finish();
    assert_eq!(result(&command), Some(Err(EIO)));
    assert!(command.vm.load(SeqCst) && command.completion.render.lock().is_some());
    pool.release_owner(OWNER);
    // account_drained
    let active = firmware.pair.as_mut().unwrap().active.front_mut().unwrap();
    active.flist_completed = true;
    active.drained_result = Some(Ok(TICKS));
    firmware.host.drained = true;
    firmware.release_drained_render(usize::from(slot), &mut deferred).unwrap();
    assert!(quarantined(&pool, slot), "the drained command's lease and VM pin still hold the slot");
    assert_eq!(pool.state.lock().acquire(NEW, 1), None);
    deferred.finish();
    assert_eq!(*command.pin.lock(), Some(true), "a drained command gives its VM pin back before its slot returns");
    assert!(!quarantined(&pool, slot));
    assert_eq!(pool.state.lock().drained, 0);
    assert_eq!(pool.state.lock().acquire(NEW, 1), Some(slot));
}

/// The witness comes in the pass that retired the group, before its Retired ran.
fn witness_before_the_retirement() {
    let pool = pool();
    let command = command(&pool, lease(&pool, OWNER));
    let slot = command.completion.render.lock().as_ref().unwrap().slot();
    let mut firmware = firmware(&pool, &[(&command, 413)]);
    let mut deferred = batch();
    stampless_retirement(&mut firmware, &mut deferred);
    quarantine(&mut firmware, ETIMEDOUT, &mut deferred);
    pool.release_owner(OWNER);
    firmware.host.drained = true;
    firmware.release_drained_render(usize::from(slot), &mut deferred).unwrap();
    assert!(quarantined(&pool, slot), "a lease of the slot is still held");
    assert_ne!(pool.state.lock().drained & bit(slot), 0);
    deferred.finish();
    assert!(!quarantined(&pool, slot), "the last lease of a drained pair returns its slot");
    assert_eq!(pool.state.lock().drained, 0);
    assert_eq!(pool.state.lock().acquire(NEW, 1), Some(slot));
}

/// A lease the witness did not cover fails afterwards.
fn failure_after_the_witness() {
    let pool = pool();
    let command = command(&pool, lease(&pool, OWNER));
    let slot = command.completion.render.lock().as_ref().unwrap().slot();
    let later = lease(&pool, OWNER);
    assert_eq!(later.slot(), slot);
    let mut firmware = firmware(&pool, &[(&command, 413)]);
    let mut deferred = batch();
    stampless_retirement(&mut firmware, &mut deferred);
    quarantine(&mut firmware, ETIMEDOUT, &mut deferred);
    firmware.host.drained = true;
    firmware.release_drained_render(usize::from(slot), &mut deferred).unwrap();
    deferred.finish();
    assert!(quarantined(&pool, slot));
    later.quarantine(false);
    drop(later);
    assert!(quarantined(&pool, slot), "a failure after the drain witness is not covered by it");
    pool.release_owner(OWNER);
    assert_eq!(pool.state.lock().acquire(NEW, 1), None);
    // Both processors stop.
    pool.release_retained_owners_after_processor_stop();
    assert!(!quarantined(&pool, slot));
}

/// A witness of a pair whose slot was never quarantined changes nothing.
fn unquarantined_witness() {
    let pool = pool();
    let held = lease(&pool, OWNER);
    let slot = held.slot();
    let before = wakes(&pool);
    for index in [usize::from(slot), 0, SLOTS, SLOTS + 3] {
        pool.pair_drained(index);
    }
    assert_eq!(wakes(&pool), before, "nothing ended, nobody wakes");
    assert_eq!(pool.state.lock().drained, 0, "nothing is left to end with the lease");
    drop(held);
    let before = wakes(&pool);
    pool.pair_drained(usize::from(slot));
    assert_eq!(wakes(&pool), before);
    assert!(!quarantined(&pool, slot));
}

fn main() {
    watchdog_expiry();
    drained_command();
    witness_before_the_retirement();
    failure_after_the_witness();
    unquarantined_witness();
    // The KUnit case of the same rule.
    a_drained_pair_ends_a_quarantine_no_lease_is_left_to_end();
    println!("PASS render quarantine drain: a slot quarantined through a lease its stampless retirement gave back returns on its pair's drain witness, never before it; a drained command's VM pin goes first; the last lease ends a quarantine drained while leases remained; a failure after the witness waits for its own; an unquarantined witness changes nothing");
}
'''


# Each removes one part of the rule and must fail.
MUTATIONS = {
    # The old behaviour: only a lease ends a quarantine.
    'no-drain-release': [('        if released {\n            self.queues.admission.pair_drained(slot);\n        }\n', '')],
    'without-witness': [('|| !self.firmware_drained(host)? {\n            return Ok(false);',
                         '|| false {\n            return Ok(false);')],
    'ignore-leases': [('        if self.refs[slot] != 0 {\n            self.drained |= bit;', '        if false {\n            self.drained |= bit;')],
    'no-last-lease': [('            if self.drained & bit != 0 {\n                self.quarantined &= !bit;',
                       '            if false {\n                self.quarantined &= !bit;')],
    'witness-covers-later-failure': [('        state.drained &= !bit;\n        drop(state);', '        drop(state);')],
    'unquarantined-drain': [('        if self.quarantined & bit == 0 {\n            return false;\n        }\n        if self.refs[slot] != 0 {',
                             '        if self.refs[slot] != 0 {')],
}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n', 1)[0])
    ap.add_argument('--mutation', choices=sorted(MUTATIONS))
    args = ap.parse_args()
    code = fixture()
    for old, new in MUTATIONS.get(args.mutation, []):
        assert code.count(old) == 1, (args.mutation, old)
        code = code.replace(old, new)
    with tempfile.TemporaryDirectory(prefix='g17-quarantine-drain-') as tmp:
        path = Path(tmp) / 'drain.rs'
        binary = path.with_suffix('')
        path.write_text(code)
        subprocess.run([os.environ.get('RUSTC', 'rustc'), '--edition=2021', str(path), '-o', str(binary)], check=True)
        # A killed mutant exits with the failed control's panic on stderr.
        result = subprocess.run([str(binary)])
        if result.returncode:
            raise SystemExit(result.returncode)


if __name__ == '__main__':
    main()
