#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Exercise the ordering between physical-queue owner changes and scheduler releases.

The production release table, pair switch tracking, release readiness and compute
switch predicate are compiled against host stubs. This is not GPU execution. The
captured firmware crash (a render pair rebound in place and kicked by a same-VM
successor while its previous scheduler's release was published and the pair not
yet parked) must fail the same assertions once either rule is removed.
"""
from pathlib import Path
import os
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2] / 'drivers/gpu/drm/asahi/g17'


def item(source, signature):
    start = source.index(signature)
    opening = source.index('{', start)
    depth = 1
    end = opening + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


def ordered(source, *needles):
    """Every needle occurs, in this order, inside `source`."""
    position = 0
    for needle in needles:
        found = source.find(needle, position)
        assert found >= 0, f'{needle!r} missing or out of order'
        position = found + len(needle)


def check_guards():
    prepare = (ROOT / 'runtime/render/prepare.rs').read_text()
    build = item(prepare, 'fn build_render_pair(')
    # The in-place rebind is refused while the pair's scheduler release is pending.
    ordered(build, 'if !pair.reusable()?', 'firmware.queues.teardown.releasing(pair.context())',
            'return Err(EBUSY);', 'Build::Rebind {')
    rebind = item(prepare, 'fn rebind_pair(')
    # A live donor that closed while the binding was prepared is caught at commit.
    ordered(rebind, 'firmware.queues.teardown.releasing(pair.context())', 'return Err(EBUSY);',
            'firmware.queues.teardown.displace(', 'pair.bind_owner(')
    submit = (ROOT / 'runtime/render/submit.rs').read_text()
    publish = item(submit, 'pub(super) fn publish_render(')
    ordered(publish, '.owner_switch_from(&packet.context)', 'releasing_scheduler(scheduler)',
            'return Err(EBUSY);', 'acquire_owner(buffer_id)', 'pair.publish(')
    render = (ROOT / 'queue/render.rs').read_text()
    pair_publish = item(render, 'pub(crate) fn publish(')
    # The switch is recorded only after the outer publication succeeded.
    ordered(pair_publish, 'let outer_next = match result', 'self.ordinal = next_ordinal;',
            'self.switch = Some((self.executed, ordinal));', 'self.executed = scheduler;',
            'self.announce_publication(')
    parked = (ROOT / 'queue/render/parked.rs').read_text()
    # Parking keeps the last scheduler; only a consumed release clears it.
    ordered(item(parked, 'pub(crate) fn park('), 'executed: self.executed,')
    ordered(item(parked, 'pub(crate) fn commit('), 'executed: self.executed,', 'switch: None,')
    detach = item((ROOT / 'runtime/render/teardown.rs').read_text(), 'fn detach_context(')
    ordered(detach, '.park();', 'parked.release_consumed(context.scheduler_va());')
    runtime = (ROOT / 'runtime.rs').read_text()
    # A compute queue does not change owner while its scheduler is released.
    ordered(item(runtime, 'pub(super) fn rebind_compute('), '.filter(|queue| queue.reusable())',
            'teardown.releasing(bound)', 'continue;', 'queue.bind_owner(owner, context.clone())?;')
    ordered(item(runtime, 'pub(super) fn take_over_idle_compute('), 'self.teardown.releasing(bound)',
            'queue.yield_idle_owner()?;', 'queue.bind_owner(')
    ordered(item(runtime, 'pub(super) fn bind_rearmed('), 'teardown.releasing(bound)', 'return Err(EAGAIN);',
            'queue.bind_owner(')
    ordered(item(runtime, 'fn publish_compute('), '.pending_switch_from()', 'queues.teardown.releasing(previous)',
            'return Err(EAGAIN);', 'queue.publish(')
    # Takeover only under publication's own device mutex hold.
    assert 'take_over_idle_compute' not in item(runtime, 'fn build_compute(')
    for name, path in [('take_pair', 'runtime/render.rs'), ('take_render_observation', 'runtime/render/observe.rs')]:
        body = item((ROOT / path).read_text(), f'fn {name}(')
        ordered(body, 'entry.borrowed_switch = pair.switch_in_flight();', 'entry.pair.take()')
    print('PASS guard placement: rebind decision, rebind commit, first publication, switch record, borrows, parked scheduler, compute owner changes')


def fixture():
    teardown = (ROOT / 'teardown.rs').read_text()
    assert ('#[derive(Clone, Copy, PartialEq, Eq)]\nenum Phase {\n    Deferred,\n    Draining,\n'
            '    Released(u32),\n}') in teardown
    pending = '\n'.join(item(teardown, name) for name in [
        'pub(crate) fn displace(', 'pub(crate) fn close(', 'fn insert(', 'pub(crate) fn releasing(',
        'pub(crate) fn releasing_scheduler(', 'fn find(']).replace('pub(crate) ', '')
    pending = pending.replace('self.entries.push(entry, GFP_KERNEL)?;', 'self.entries.push(entry);')
    render = (ROOT / 'queue/render.rs').read_text()
    switch_methods = '\n'.join(item(render, name) for name in [
        'pub(crate) fn owner_switch_from(', 'pub(crate) fn switch_in_flight(']).replace('pub(crate) ', '')
    record = render[render.index('        let scheduler = packet.context.scheduler_va();'):
                    render.index('        self.executed = scheduler;') + len('        self.executed = scheduler;')]
    idle = item((ROOT / 'runtime/render/teardown.rs').read_text(),
                'pub(in crate::g17::runtime) fn teardown_idle(').replace('pub(in crate::g17::runtime) ', '')
    compute_switch = item((ROOT / 'queue/compute.rs').read_text(),
                          'pub(crate) fn switching_from(').replace('pub(crate) ', '')
    compute_idle = item((ROOT / 'runtime/teardown.rs').read_text(), 'fn compute_idle(')
    return r'''
#![allow(dead_code)]
use std::sync::Arc;
type Error = i32;
type Result<T = ()> = std::result::Result<T, Error>;
struct Context { scheduler: u64, published: bool }
impl Context {
    fn scheduler_va(&self) -> u64 { self.scheduler }
    fn is_published(&self) -> bool { self.published }
}
#[derive(Clone, Copy, PartialEq, Eq)]
enum Phase { Deferred, Draining, Released(u32) }
struct Entry { context: Arc<Context>, phase: Phase, armed: u64, announce_pending: bool }
struct Pending { entries: Vec<Entry> }
impl Pending {
PENDING
}
struct Ticket { ordinal: u64 }
struct Active { ticket: Ticket }
struct Packet { context: Arc<Context> }
struct Pair { executed: u64, switch: Option<(u64, u64)>, replay: Option<()>, active: Vec<Active>, idle: bool }
impl Pair {
SWITCH
    /// Host stand-in for a successful publication of `ordinal` (stand-in, not production).
    fn publish(&mut self, packet: &Packet, ordinal: u64) {
RECORD
        self.active.push(Active { ticket: Ticket { ordinal } });
    }
    fn idle_for_context(&self, _: &Arc<Context>) -> Result<bool> { Ok(self.idle) }
}
struct RenderEntry {
    context: Option<Arc<Context>>,
    preparing: Option<()>,
    growing: bool,
    pair: Option<Box<Pair>>,
    borrowed_switch: Option<u64>,
}
struct RenderState { entries: Vec<Option<RenderEntry>> }
impl RenderState {
IDLE
}
struct Previous { context: Arc<Context>, publication: Option<(u64, u64)> }
struct Queue { previous: Option<Previous>, bound: Option<Arc<Context>> }
impl Queue {
COMPUTE_SWITCH
    fn idle_for_context(&self, context: &Arc<Context>) -> bool {
        !self.bound.as_ref().is_some_and(|bound| Arc::ptr_eq(bound, context))
    }
}
struct ComputeEntry { queue: Option<Box<Queue>> }
struct Service { compute: Vec<Option<ComputeEntry>> }
impl Service {
COMPUTE_IDLE
}
fn context(scheduler: u64) -> Arc<Context> { Arc::new(Context { scheduler, published: true }) }
fn main() {
    let (b, c, d) = (context(0x1000), context(0x2000), context(0x3000));
    // Captured crash: GL context B closes, its release is published, then the same-VM
    // successor C finds B's pairs still installed (not parked) and rebinds them in place.
    let mut table = Pending { entries: Vec::new() };
    table.close(&b, 1).unwrap();
    assert!(table.releasing(&b), "armed release refuses an owner change");
    table.entries[0].phase = Phase::Released(67);
    assert!(table.releasing(&b), "published, unconsumed release refuses an owner change");
    assert!(table.releasing_scheduler(0x1000) && !table.releasing_scheduler(0x2000));
    // Consumption and detach remove the entry; the pair is parked and reinstalled fresh.
    table.entries.clear();
    assert!(!table.releasing(&b) && !table.releasing_scheduler(0x1000));
    // A forfeit from a live owner is not a release; its later close arms one.
    table.displace(&d, false, 2).unwrap();
    assert!(!table.releasing(&d), "a live displaced owner may still lose an idle pair");
    table.close(&d, 3).unwrap();
    assert!(table.releasing(&d), "the donor's close arms its release");
    // An unpublished context has no firmware scheduler state and no release.
    let quiet = Arc::new(Context { scheduler: 0x4000, published: false });
    table.close(&quiet, 4).unwrap();
    assert!(!table.releasing(&quiet));

    // Switch tracking across an owner change of one pair.
    let mut pair = Pair { executed: 0, switch: None, replay: None, active: Vec::new(), idle: true };
    assert_eq!(pair.owner_switch_from(&c), None, "a fresh graph has no previous scheduler");
    pair.publish(&Packet { context: d.clone() }, 0);
    pair.active.clear();
    assert_eq!(pair.owner_switch_from(&d), None, "the same owner never switches");
    assert_eq!(pair.owner_switch_from(&c), Some(0x3000));
    pair.publish(&Packet { context: c.clone() }, 1);
    assert_eq!(pair.switch_in_flight(), Some(0x3000));
    let mut render = RenderState { entries: vec![Some(RenderEntry {
        context: Some(c.clone()), preparing: None, growing: false, pair: Some(Box::new(pair)), borrowed_switch: None,
    })] };
    assert!(!render.teardown_idle(&d).unwrap(), "release waits for the new owner's first command");
    assert!(render.teardown_idle(&b).unwrap(), "unrelated releases are not held");
    // A borrowed pair is judged by its snapshot; a stale snapshot outside a borrow is ignored.
    let borrowed = render.entries[0].as_mut().unwrap();
    let pair = borrowed.pair.take().unwrap();
    borrowed.borrowed_switch = pair.switch_in_flight();
    borrowed.preparing = Some(());
    assert!(!render.teardown_idle(&d).unwrap(), "a borrowed switching pair still holds the release");
    let entry = render.entries[0].as_mut().unwrap();
    entry.preparing = None;
    entry.pair = Some(pair);
    entry.pair.as_mut().unwrap().active.clear();
    assert!(render.teardown_idle(&d).unwrap(), "retirement of the first command releases the hold");
    let entry = render.entries[0].as_mut().unwrap();
    entry.pair.as_mut().unwrap().active.push(Active { ticket: Ticket { ordinal: 1 } });
    entry.pair = None;
    assert!(render.teardown_idle(&d).unwrap(), "a stale snapshot without a borrow holds nothing");

    // Compute: only a published first command after the change holds the release.
    let mut service = Service { compute: vec![Some(ComputeEntry { queue: Some(Box::new(Queue {
        previous: Some(Previous { context: b.clone(), publication: None }), bound: Some(c.clone()),
    })) })] };
    assert!(service.compute_idle(&b), "an unpublished rebind does not hold the release");
    service.compute[0].as_mut().unwrap().queue.as_mut().unwrap().previous.as_mut().unwrap().publication = Some((1, 0));
    assert!(!service.compute_idle(&b), "compute release waits for the new owner's first command");
    service.compute[0].as_mut().unwrap().queue.as_mut().unwrap().previous = None;
    assert!(service.compute_idle(&b));
    assert!(!service.compute_idle(&c), "the bound owner still needs its own retirement");
    println!("PASS release table gate, pair switch hold, borrowed snapshot and compute switch hold");
}
'''.replace('PENDING', pending).replace('SWITCH', switch_methods, 1).replace('RECORD', record) \
   .replace('IDLE', idle, 1).replace('COMPUTE_SWITCH', compute_switch).replace('COMPUTE_IDLE', compute_idle)


def main():
    check_guards()
    code = fixture()
    mutations = {
        'fixed': (None, None, None),
        'no-release-gate': ('.is_some_and(|index| self.entries[index].phase != Phase::Deferred)',
                            '.is_some_and(|_| false)', 'armed release refuses an owner change'),
        'no-render-hold': ('Some(pair) => pair.switch_in_flight() == scheduler,', 'Some(_) => false,',
                           "release waits for the new owner's first command"),
        'no-compute-hold': ('queue.idle_for_context(context) && !queue.switching_from(context)',
                            'queue.idle_for_context(context)',
                            "compute release waits for the new owner's first command"),
    }
    with tempfile.TemporaryDirectory(prefix='g17-owner-release-') as directory:
        root = Path(directory)
        for name, (old, new, expected) in mutations.items():
            current = code
            if old is not None:
                assert current.count(old) == 1, name
                current = current.replace(old, new)
            path = root / (name + '.rs')
            path.write_text(current)
            binary = path.with_suffix('')
            subprocess.run([os.environ.get('RUSTC', 'rustc'), '--edition=2021', str(path), '-o', str(binary)],
                           check=True)
            result = subprocess.run([str(binary)], capture_output=True, text=True)
            if old is None:
                assert result.returncode == 0, result.stderr
                print(result.stdout.strip())
            else:
                assert result.returncode != 0 and expected in result.stderr, result.stderr
                print('PASS negative control:', name)


if __name__ == '__main__':
    main()
