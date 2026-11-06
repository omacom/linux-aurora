#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Host test for the T6001 AGX vote policy (drivers/pmdomain/apple/pmp-agx-vote.h).

The policy is a header of static inline transition functions, so the suite
compiles tools/asahi/pmp-agx-vote-tests.c against a copy of that header: a
fake clock, a fake PMP bit and a fake timer drive the same begin, decide,
next and applied calls the kernel glue makes. No kernel, no hardware.

--mutation NAME / --mutations apply textual mutants to the header copy and
require the suite to fail; like the other self-checking suites, exit status 0
means every mutant was caught. --ref reads the header at a Git revision (a
pre-fix header fails the suite itself, which is the point of those cases).
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[2]
HEADER = 'drivers/pmdomain/apple/pmp-agx-vote.h'

# Each mutant removes one guard. The first two are the mutants that survived
# the first review round, when begin() did not refuse while stopped and the
# retry deadline was not distinguished from the 1 ms owed-set kick.
CONTROLS = [
    ('owed-set-branch', '\tif (v->users && !v->voted && !v->failures)\n\t\treturn 1;\n', ''),
    ('owed-failures', 'if (v->users && !v->voted && !v->failures)\n\t\treturn 1;', 'if (v->users && !v->voted)\n\t\treturn 1;'),
    ('stopped-in-begin', 'v->users++;\n	if (!v->enabled || v->stopped)\n		return 0;', 'v->users++;\n	if (!v->enabled)\n		return 0;'),
    ('stopped-in-decide', 'if (!v->enabled || v->stopped)\n\t\treturn v->voted ? AGX_VOTE_CLEAR : AGX_VOTE_NONE;',
     'if (!v->enabled)\n\t\treturn v->voted ? AGX_VOTE_CLEAR : AGX_VOTE_NONE;'),
    ('wrap-reached', 'return (agx_sms_t)(now - t) >= 0;', 'return now >= t;'),
    ('set-write-ends', 'return agx_vote_reached(now, at) ? 1 : at - now;', 'return agx_vote_reached(now, at) ? 0 : at - now;'),
    ('retry-dropped-when-idle',
     '\telse if (!v->users && !v->voted && v->failures)\n'
     '\t\t/* The write outlasted the operation: arm the retry from the failure state\n'
     '\t\t * itself. The caller clears failures when this reconcile finds nobody waiting\n'
     '\t\t * (decide returns NONE), so it fires once. */\n'
     '\t\tat = agx_vote_retry_reached(v, now) ? now : v->retry_at;\n', ''),
    ('begin-backoff-kick', 'return (!v->failures || agx_vote_retry_reached(v, now)) ? 1 : v->retry_at - now;', 'return (void)now, 1;'),
    ('begin-no-retry-arm', 'return (!v->failures || agx_vote_retry_reached(v, now)) ? 1 : v->retry_at - now;', 'return (void)now, 0;'),
    ('begin-drops-users', '	v->users++;\n	if (!v->enabled || v->stopped)', '	if (!v->enabled || v->stopped)'),
    ('retry-revival', '|| (agx_ms_t)(v->retry_at - now) > AGX_VOTE_RETRY_MS', ''),
]


def header_text(ref):
    if ref:
        return subprocess.check_output(['git', '-C', str(ROOT), 'show', f'{ref}:{HEADER}'], text=True)
    return (ROOT / HEADER).read_text()


def compile_and_run(out, text):
    out.mkdir(parents=True, exist_ok=True)
    (out / 'pmp-agx-vote.h').write_text(text)
    compiler = shlex.split(os.environ.get('CC', 'clang'))
    command = compiler + ['-std=gnu11', '-g', '-O1', '-Wall', '-Wextra', '-Werror',
                          '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
                          '-I', str(out), str(ROOT / 'tools/asahi/pmp-agx-vote-tests.c'),
                          '-o', str(out / 'vote')]
    built = subprocess.run(command, capture_output=True, text=True)
    (out / 'build.log').write_text(built.stdout + built.stderr)
    if built.returncode:
        return {'build_rc': built.returncode, 'run_rc': None, 'note': 'compilation failure is not regression evidence'}
    prefix = ['setarch', '-R'] if shutil.which('setarch') else []  # ASan startup can crash under ASLR on a loaded host
    ran = subprocess.run(prefix + [str(out / 'vote')], capture_output=True, text=True)
    (out / 'run.log').write_text(ran.stdout + ran.stderr)
    return {'build_rc': 0, 'run_rc': ran.returncode}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--ref', help='read the header at a Git revision (negative control)')
    p.add_argument('--mutation', help='apply one named mutant and require the suite to fail')
    p.add_argument('--mutations', action='store_true', help='apply every mutant and require each to fail')
    p.add_argument('--output', required=True, type=Path)
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    text = header_text(args.ref)

    base = compile_and_run(args.output, text)
    print(Path(args.output / 'run.log').read_text() if base['run_rc'] is not None else Path(args.output / 'build.log').read_text())

    mutations = []
    if args.mutations or args.mutation:
        # A base failure would fail every mutant at the same case and read as caught.
        # The negative control (--ref) has no passing base: the pre-fix header cannot compile this suite.
        if not args.ref:
            assert base['run_rc'] == 0, base
        names = [c[0] for c in CONTROLS] if args.mutations else [args.mutation]
        for name in names:
            old, new = next((c[1], c[2]) for c in CONTROLS if c[0] == name)
            count = text.count(old)
            result = {}
            result['applied'] = count == 1
            if count == 1:
                result.update(compile_and_run(args.output / name, text.replace(old, new)))
            else:
                # A guard that does not exist in this revision cannot be removed: the
                # behaviour the guard carries is absent, so the mutant survives there.
                result.update({'build_rc': None, 'run_rc': 0, 'note': f'{HEADER} at this revision has no such line ({count} matches)'})
            result['name'] = name
            mutations.append(result)
            caught = result['applied'] and result['build_rc'] == 0 and result['run_rc'] not in (0, None)
            log = args.output / name / ('run.log' if result.get('run_rc') is not None else 'build.log')
            detail = log.read_text().strip().splitlines()[-1][:150] if log.exists() else result.get('note', '')
            print(f"{name}: {'CAUGHT' if caught else 'SURVIVED'} | {detail}")

    if args.mutations:
        assert all(m['applied'] and m['build_rc'] == 0 and m['run_rc'] not in (0, None) for m in mutations), mutations
        print('PASS: every vote-policy case passed and every mutant caught')
        return 0
    if args.mutation:
        m = mutations[0]
        assert m['applied'] and m['build_rc'] == 0 and m['run_rc'] not in (0, None), m
        return 0

    (args.output / 'receipt.json').write_text(json.dumps({
        'source_ref': args.ref, 'source_sha256': hashlib.sha256(text.encode()).hexdigest(),
        'command': [], 'returncode': base['run_rc'] if base['run_rc'] is not None else base['build_rc'],
        'mutations': mutations}, indent=2) + '\n')
    return bool(base['run_rc']) if base['run_rc'] is not None else bool(base['build_rc'])


if __name__ == '__main__':
    raise SystemExit(main())
