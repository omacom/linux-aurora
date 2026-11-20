#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Exercise N1 recovery with reset-hidden old PCI objects and retained DMA.

Extract 15 production functions from this checkout; PCI core, provider hardware
and scheduling are mocks. Negative controls must compile and fail a C assertion.
This tests teardown/worker ownership, not physical sleep or fresh firmware boot.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[2]
PCI = 'drivers/net/wireless/apple/centauri/t6050/centauri_pci.c'
IPC = 'drivers/net/wireless/apple/centauri/t6050/centauri_ipc.c'
BT = 'drivers/bluetooth/btcentauri_t6050.c'
GUARDS = {
    'bound': '(c->ipc || c->firmware || c->descriptor || c->recovery->arena) &&\n\t    ',
    'owner': '(rw->ipc || rw->firmware || rw->descriptor || rw->arena) &&\n\t    ',
}
OMISSIONS = {
    'missing-control-firmware': ('bound', 'c->firmware || '),
    'missing-control-descriptor': ('bound', 'c->descriptor || '),
    'missing-owner-ipc': ('owner', 'rw->ipc || '),
    'missing-owner-firmware': ('owner', 'rw->firmware || '),
    'missing-owner-descriptor': ('owner', 'rw->descriptor || '),
    'missing-owner-arena': ('owner', ' || rw->arena'),
}
MUTATIONS = ('bound-guard-reverted', 'owner-guard-reverted', *OMISSIONS)


def function(source, name):
    match = re.search(r'^[\w *]+\b' + name + r'\([^;]*?\)\n\{', source, re.M)
    if not match:
        raise ValueError(f'production function not found: {name}')
    start = source.index('{', match.start())
    depth = 1
    end = start + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[match.start():end] + '\n'


def mutate(source, mutation):
    if not mutation:
        return source
    if mutation in OMISSIONS:
        owner, term = OMISSIONS[mutation]
        guard = GUARDS[owner]
        replacement = guard.replace(term, '')
    else:
        guard = GUARDS[mutation.removesuffix('-guard-reverted')]
        replacement = ''
    # A stale mutation is a harness error, never a successful mutant kill.
    if source.count(guard) != 1:
        raise ValueError(f'{mutation}: expected exactly one inventory guard')
    return source.replace(guard, replacement, 1)


def generate(sources, mutation):
    source = mutate(sources[PCI], mutation)
    chunks = [function(source, 'centauri_t6050_stop_dma'),
              function(sources[BT], 'cen_dma_stopped'),
              function(sources[BT], 'cen_release'),
              function(sources[BT], 'cen_retained'),
              function(sources[BT], 'cen_stop'),
              function(sources[IPC], 'centauri_t6050_ipc_fenced'),
              function(sources[IPC], 'centauri_t6050_free_ipc')]
    chunks += [function(source, name) for name in (
        'centauri_t6050_free_boot', 'centauri_t6050_beta_call',
        'centauri_t6050_domain_fence', 'centauri_t6050_quiesce',
        'centauri_t6050_remove', 'centauri_t6050_rewake_teardown',
        'centauri_t6050_link_active', 'centauri_t6050_rewake_worker')]
    fixture = Path(__file__).with_suffix('.c').read_text()
    marker = '/* ACTUAL_FUNCTIONS */'
    if fixture.count(marker) != 1:
        raise ValueError('expected exactly one production insertion marker')
    return fixture.replace(marker, '\n'.join(chunks))


def run(args, out):
    out.mkdir(parents=True, exist_ok=True)
    sources = {path: (ROOT / path).read_text() for path in (PCI, IPC, BT)}
    c = out / 'centauri-recovery.c'
    binary = out / 'centauri-recovery'
    c.write_text(generate(sources, args.mutation))
    command = shlex.split(os.environ.get('CC', 'cc')) + [
        '-std=gnu11', '-Wall', '-Wextra', '-Werror', '-Wno-unused-function',
        '-g0', '-O1', str(c), '-o', str(binary)]
    # Build/extraction errors propagate as Python tracebacks, which the runner
    # rejects for KILL jobs. Only an executed model assertion counts as a kill.
    subprocess.run(command, check=True, timeout=60)
    result = subprocess.run([str(binary)], text=True, capture_output=True, timeout=10)
    (out / 'model.log').write_text(result.stdout + result.stderr)
    (out / 'result.json').write_text(json.dumps({
        'mutation': args.mutation, 'exit': result.returncode,
        'extracted_functions': 15, 'compiler_command': command,
        'sources': {path: hashlib.sha256(text.encode()).hexdigest()
                    for path, text in sources.items()},
    }, indent=2) + '\n')
    print(result.stdout, end='', flush=True)
    print(result.stderr, end='', file=sys.stderr, flush=True)
    return 0 if result.returncode == 0 else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n', 1)[0])
    parser.add_argument('--out', type=Path)
    parser.add_argument('--mutation', choices=MUTATIONS)
    args = parser.parse_args()
    if args.out:
        return run(args, args.out.resolve())
    with tempfile.TemporaryDirectory(prefix='centauri-recovery-') as directory:
        return run(args, Path(directory))


if __name__ == '__main__':
    sys.exit(main())
