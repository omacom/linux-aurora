#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Run every Aurora host test: tools/asahi and the GPU/display selftests.

Each tools/asahi test runs under `python3 -I` with its own output directory,
once as a control and once per mutation or old-code negative control listed
below. A mutation run is only accepted as a kill when the test reports one
(an assertion, an abort or a panic of the built control), so a mutant that
merely stops compiling is a failure, not a pass.

Every tools/asahi/*-tests.{py,rs,sh} file must appear here. A new test that
is not registered fails the run, so it cannot be silently skipped.

Negative controls read older revisions with `git show`, so the checkout needs
the commits (a full or treeless clone, not a depth-1 one).

A test that reads device trees gets '{dtc}', this tree's scripts/dtc/dtc, built
with the kconfig tool unless --dtc names one.

Usage: tools/asahi/run-host-tests.py [--jobs N] [--work DIR] [--only SUBSTR]
"""
import argparse
import concurrent.futures
import os
import re
import shlex
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
ASAHI = ROOT / 'tools/asahi'
GPU = ROOT / 'tools/testing/selftests/drivers/gpu'

PASS = 'pass'
KILL = 'kill'

# A killed mutant shows the failing control: a C assertion or abort (-6), or
# a Rust panic (exit 101). A Python traceback means the harness itself broke.
KILL_SIGN = re.compile(r'FAIL line [0-9]+|Assertion|": -6\b|"exit": 101\b|panicked at')
BROKEN_SIGN = re.compile(r'Traceback \(most recent call last\)')


def out(flag='--out'):
    return [flag, '{out}']


def mutations(flag, names, base, expect=KILL):
    return [(base + [flag, n], expect) for n in names]


# test file -> [(arguments, expectation)]. '{out}' is a fresh directory.
ASAHI_TESTS = {
    'dcpext26-crossbar-union-tests.py': [([], PASS), (['--drop-native26'], KILL), (['--drop-t8132'], KILL)],
    'dcpext26-suspend-route-tests.py': [([], PASS), (['--native-parks'], KILL), (['--drop-h17p-park'], KILL)],
    'j613-mux-order-tests.py': [([], PASS), (['--negative-control'], KILL),
                                 (['--ignore-nomodeset'], KILL), (['--drop-failed-completion'], KILL),
                                 (['--ignore-external-node'], KILL), (['--drop-board-scope'], KILL)],
    'display-sleep-merge-tests.py': [(out(), PASS)],
    'dcpext26-slot-unregister-tests.py': [(out(), PASS)],
    'dcpext26-budget-tests.py': [([], PASS)],
    'dcpext26-queue-tests.py': [([], PASS), (['--drop-v27-close'], KILL)],
    'dcpext26-retrain-tests.py': [([], PASS), (['--negative-control'], KILL)],
    'dcpext26-lifecycle-tests.py': [([], PASS)],
    'dcpext26-modeset-tests.py': [([], PASS)],
    'dcpext26-parity-tests.py': [([], PASS)],
    'dcpext26-present-tests.py': [([], PASS)],
    'dcpext26-profile-tests.py': [([], PASS)],
    'dcpext26-edid-tests.py': [([], PASS)],
    'dcpext26-changeset-tests.py': [([], PASS)],
    'dcpext26-admission-tests.py': [([], PASS)],

    'dcpext26-startup-tests.py': [([], PASS)],
    'iomfb-startup-tests.py': [([], PASS)],
    'dcpext26-wire-tests.py': [([], PASS)],
    'pmp-native26-tests.py': [([], PASS)],


    'm3-stamp-admission-tests.py': [(out(), PASS),
                                     (out() + ['--mutation', 'live-reseed'], KILL)],
    'hardware-defaults-tests.py': [([], PASS)],
    'j613-mtp-s2idle-tests.py': [([], PASS)],
    # 11eccbedc3ed writes frames without a transport-wide lock.
    'mtp-tx-frame-tests.py': [(out(), PASS),
                              (out() + ['--old-transport', '11eccbedc3ed'], PASS)],
    'pmgr-ppt-tests.py': [([], PASS)],
    'queue-priority-tests.py': [([], PASS)],
    # The old revision reports a DART fault that clears before the second latch read as success.
    'ane-tm-transient-fault-tests.py': [([], PASS),
                                        (['--old-ref', 'b50066848322c6799c1433ee3a358aab2e9faabb'], PASS)],
    'test-m3-power.py': [([], PASS)],
    'test-m3-uapi.py': [([], PASS)],
    'test-m3-mailbox.py': [([], PASS)],
    'test-j613-dcp-power.py': [([], PASS)],
    'aop-als-channel-tests.py': [([], PASS)],
    'aop-bootargs-tests.py': [([], PASS)],
    'mtp-pm-tests.py': [(out(), PASS),
                         (out() + ['--old-transport', '4db3a5a22c43'], PASS),
                         (out() + ['--old-mouse', '62d2045d8928'], PASS)],
    'ave-pm-tests.py': [(out(), PASS),
                         (out() + ['--old-ref', '4db3a5a22c43'], PASS)],
    'ave-probe-pm-tests.py': [(out(), PASS), (out() + ['--negative-control'], PASS)],
    'binding-lifetime-tests.py': [(out(), PASS)]
        + mutations('--mutation', ('no-close', 'no-join'), out()),
    'apple-pwm-pm-tests.py': [([], PASS), (['--negative-control'], PASS)],
    'apple-m4-pmu-tests.py': [([], PASS)]
        + mutations('--mutation', ('m1-layout', 'cycles-anywhere', 'expose-8-9', 'flags-in-event'), []),
    'apple-dart-dapf-tests.py': [(out(), PASS)]
        + mutations('--mutation', ('missing-guard', 'ignore-replay', 'ignore-orphan'), out()),
    'apple-dart-fw-tests.py': [(out(), PASS)],
    'apple-dart-fw-harness-tests.py': [([], PASS)],
    'apple-dart-range-tests.py': [([], PASS)],
    # 551b937502b5 gives up on a stalled J700 display invalidate after 2 ms; the
    # every-handoff mutation waits it out on every handed-off DART, not just the display's.
    'apple-dart-tlb-stall-tests.py': [(['--dtc', '{dtc}'], PASS),
                                      (['--ref', '551b937502b54b0d1eb250bba6d9442b3672a0fb'],
                                       KILL),
                                      (['--dtc', '{dtc}', '--mutation', 'every-handoff'],
                                       KILL)],
    'apple-router-sleep-tests.py': [(out('--output'), PASS)],
    'asahi-stats-raw-tests.py': [([], PASS)],
    'agx-stats-busy-tests.py': [([], PASS), (['--negative-control'], PASS)],
    'atc-cio-sleep-tests.py': [(out('--output'), PASS)],
    'atc-usb3-tests.py': [(out(), PASS), (out() + ['--mutations'], PASS)],
    'avd-av1-tail-tests.py': [(out(), PASS)]
        + [(out() + ['--case', c], PASS) for c in ('copy', 'seed', 'aligned')]
        + mutations('--mutation', ('round-up', 'cpu-unaligned', 'seg-cpu-unaligned',
                                   'seg-any-size', 'ref-index-upper', 'frame-width-unchecked'), out()),
    'avd-codec-tail-tests.py': [(out('--output'), PASS), (out('--output') + ['--mutations'], PASS)],
    # The old revision is the one before the J493 hardware-table ID match.
    'bcm4377-id-tests.py': [(out() + ['--old-ref', '87ddae1397f6f70817cb57df76477a86bdf96424'], PASS)],
    # These mutations are caught inside the test, which then exits 0.
    'bcm4377-resume-tests.py': [(out(), PASS)]
        + mutations('--mutation', ('epoch', 'close', 'producer-fence', 'active-fence', 'board'), out(), PASS),
    'bcm4377-sleep-config-tests.py': [([], PASS)],
    'brcmf-assoc-tests.py': [(out('--output'), PASS)]
        + mutations('--mutation', ('short-ie', 'header-length', 'skb-capacity', 'cmplt-header'), out('--output')),
    'brcmf-bss-tests.py': [(out(), PASS)],
    'brcmf-cipher-tests.py': [(out(), PASS)]
        + [(out() + ['--ref', '33bec4016c466985bfbb5ff4c2089fcab8242dcf'], KILL)],
    # The old revision reports a failed join against the last associated BSS.
    'brcmf-join-bssid-tests.py': [(out(), PASS)]
        + mutations('--mutation', ('stale-target', 'zero-named', 'no-call', 'lost-bss-success'), out()),
    'brcmf-join-scan-tests.py': [(out(), PASS)]
        + [(out() + ['--ref', '33bec4016c466985bfbb5ff4c2089fcab8242dcf'], KILL)],
    'brcmf-pcie-dma-tests.py': [(out('--output'), PASS)]
        + mutations('--mutation', ('narrow-stream', 'ignore-aperture', 'revision-wide'), out('--output')),
    'brcmf-recovery-tests.py': [(out(), PASS)]
        + mutations('--mutation', ('stall-limit', 'trap-flag', 'console', 'line-start', 'once',
                                   'rate-limit', 'stall-reset', 'ring-full'), out()),
    'brcmf-roam-tests.py': [(out(), PASS)],
    'brcmf-sae-vendor-tests.py': [(out(), PASS)]
        + [(out() + ['--ref', '9260e1f021e075713931f3c23c0a5fa1da936b0b'], KILL)],
    'brcmf-suspend-tests.py': [(out(), PASS)]
        + mutations('--mutation', ('country-gate', 'queue-reasons', 'worker-gate',
                                   'resume-order', 'tx-drain', 'lpas-resume'), out()),
    'brcmf-wowl-filter-tests.py': [(out(), PASS)]
        + mutations('--mutation', ('base-order', 'mask-last', 'totsize', 'no-track'), out()),
    'brcmfmac-scan-timeout-tests.py': [(out('--output'), PASS)],
    'brcmfmac-sleep-lifetime-tests.py': [([], PASS)],
    'cio-native-tests.py': [(out('--output'), PASS)],
    'centauri-wifi-pm-tests.py': [(out(), PASS), (out() + ['--negative-controls'], PASS)],
    'centauri-bt-arena-tests.py': [(out(), PASS), (out() + ['--negative-controls'], PASS)],
    'centauri-recovery-tests.py': [(out(), PASS), (out() + ['--negative-controls'], PASS)],
    'centauri-physical-recovery-tests.py': [(out(), PASS)]
        + mutations('--mutation', ('bound-guard-reverted', 'owner-guard-reverted',
                                   'missing-control-firmware', 'missing-control-descriptor',
                                   'missing-owner-ipc', 'missing-owner-firmware',
                                   'missing-owner-descriptor', 'missing-owner-arena'), out()),
    'clpc-gpu-demand-tests.py': [([], PASS)],
    'clpc-high-power-cleanup-tests.py': [(out(), PASS),
        (out() + ['--old-ref', '677be395b9d2e15d98696107ebf88191f08efbf9'], PASS)],
    # The old policy always disabled compute preemption.
    'compute-preempt-tests.py': [(out(), PASS),
                                 (out() + ['--old-ref', '6dbfbcb21eb09d07ae59037a1ceca8cc3500841f'], PASS)],
    'dart-cio-owner-tests.py': [(out('--output'), PASS)],
    'dart-cio-power-tests.py': [(out('--output'), PASS)],
    'dart-sleep-power-tests.py': [([], PASS)],
    'dart-sleep-snapshot-tests.py': [([], PASS)],
    'dart-sleep-writer-tests.py': [([], PASS)],
    'backlight-coldboot-tests.py': [(out('--output'), PASS), (out('--output') + ['--negative-controls'], PASS)],
    'dcp14-backlight-tests.py': [([], PASS)],
    'dcp14-panel-tests.py': [([], PASS)],
    'devcoredump-init-tests.py': [([], PASS)],
    'display26-tests.py': [([], PASS)],
    # The old revision publishes the fd before copy_to_user() succeeds.
    'dma-heap-fd-tests.py': [(out() + ['--old-ref', 'cb732d027aa18e1fcf9d2797f47d20b179ebc59c'], PASS)],
    'g15-display-order-tests.py': [([], PASS)],
    'g15-input-error-tests.py': [(out('--output'), PASS), (out('--output') + ['--mutations'], PASS),
                                 (out('--output') + ['--ref', 'ede99b3c1990b4a4a8c045fdf627a2bd0f14446a'], PASS)],
    'g15-maintained-handoff-tests.py': [(out('--output'), PASS), (out('--output') + ['--mutations'], PASS)],
    'g15-pb-growth-tests.py': [(out('--output'), PASS), (out('--output') + ['--mutations'], PASS)],
    'g15-probe-init-tests.py': [(out('--output'), PASS), (out('--output') + ['--mutations'], PASS)],
    'g15-queue-credit-tests.py': [(out('--output'), PASS), (out('--output') + ['--mutations'], PASS)],
    'g15-retention-tests.py': [(out('--output'), PASS), (out('--output') + ['--mutations'], PASS)],
    'g15-stats-drain-tests.py': [(out('--output'), PASS), (out('--output') + ['--mutations'], PASS)],
    'g15-zero-words-tests.py': [([], PASS), (['--mutations'], PASS)],
    'g17-telemetry-integration-tests.py': [(out(), PASS)]
        + mutations('--mutation', ('fields', 'range', 'timestamp', 'age', 'delivery',
                                   'quarantine', 'epoch', 'cutoff', 'whole-batch', 'legacy-format', 'read-drain'), out()),
    'g17-admission-deadline-tests.py': [([], PASS)],
    'g17-admission-order-tests.py': [([], PASS)]
        + mutations('--mutation', ('no-held-free', 'no-held-sibling', 'no-held-refusal', 'no-held-pick',
                                   'no-held-spare', 'no-held-other', 'no-held-cancel', 'no-held-killed',
                                   'no-held-used', 'no-held-wake', 'wake-on-every-clear', 'no-held-stop',
                                   'held-blocks-owner', 'hold-ignores-release', 'hold-ignores-work',
                                   'close-unheld', 'hold-after-release', 'put-back-own-work',
                                   'sync-skips-lent', 'lent-close-unheld', 'parked-stays-held',
                                   'sync-clobbers', 'sync-no-wake'), []),
    'g17-capacity-backoff-tests.py': [([], PASS)],
    'g17-capacity-fault-tests.py': [([], PASS)],
    'g17-compute-constructor-tests.py': [([], PASS)],
    'g17-compute-owed-stamp-tests.py': [([], PASS)]
        + mutations('--mutation', ('own-finished', 'no-retire-glue', 'retire-unchecked', 'no-expiry',
                                   'no-step-past', 'pop-owed', 'head-includes-owed', 'held-stamped-pops',
                                   'crossed-behind-owed', 'owed-classified', 'rebind-check-entry',
                                   'timeout-fails-retired', 'no-owed-poll'), []),
    'g17-compute-deadline-tests.py': [([], PASS)],
    'g17-compute-growth-deadline-tests.py': [([], PASS)],
    'g17-kill-route-tests.py': [([], PASS)]
        + mutations('--mutation', ('no-executed-route', 'no-executed-input', 'kill-twice', 'executed-on-m5',
                                   'unsafe-wait', 'owed-blocks-kill', 'open-owed-skipped'), []),
    'g17-firmware-selection-tests.py': [([], PASS)],
    'g17-growth-gate-tests.py': [([], PASS)],
    'g17-growth-tests.py': [([], PASS)],
    'g17-object-fence-tests.py': [([], PASS)]
        + mutations('--mutation', ('overrun-work-states', 'unseen-strict', 'retain-not-sticky',
                                   'node-unmarked', 'no-release'), []),
    'g17-owner-release-order-tests.py': [([], PASS)],
    'g17-pool-release-tests.py': [([], PASS)],
    # no-drain-release is the old rule: only a lease ends a slot's quarantine.
    'g17-quarantine-drain-tests.py': [([], PASS)]
        + mutations('--mutation', ('no-drain-release', 'without-witness', 'ignore-leases', 'no-last-lease',
                                   'witness-covers-later-failure', 'unquarantined-drain'), []),
    'g17-resource-tests.py': [([], PASS)],
    'g17-shared-pool-tests.py': [([], PASS)],
    'g17-t6041-initdata-tests.py': [([], PASS)],
    'g17-uninstalled-cleanup-tests.py': [([], PASS)],
    'g17-unpublished-tests.py': [([], PASS)],
    'isp-preview-flow-tests.py': [([], PASS)],
    'isp-preview-tests.py': [([], PASS)],
    'isp-tests.py': [(['--suite', suite] + out(), PASS) for suite in ['session', 'buffers', 'receive', 'returns', 'queue']] + [(['--suite', suite, '--control', control] + out(), KILL) for suite, controls in [('session', ['send', 'overwrite', 'stop', 'reset']), ('buffers', ['rollback']), ('receive', ['drain']), ('returns', ['address', 'pool', 'duplicate', 'snapshot']), ('queue', ['queue-error'])] for control in controls],
    'mt7932-association-events-tests.py': [(out(), PASS)],
    'mt7932-connect-result-tests.py': [(out('--output'), PASS)],
    'mt7932-owe-retry-tests.py': [(out(), PASS)],
    'mt7932-policy-6ghz-tests.py': [([], PASS)],
    'mt7932-policy-classes-tests.py': [([], PASS)],
    'mt7932-scan-tests.py': [(out(), PASS)],
    'neo-auxiliary-tests.py': [(out('--output'), PASS), (out('--output') + ['--mutation'], KILL),
                               (out('--output') + ['--old'], KILL)],
    'neo-stability-tests.py': [(out('--output'), PASS)]
        + mutations('--mutation', ('sep-syscore', 'wake-irq', 'rx-suspend', 'stale-policy',
                                   'usb-rpm', 'usb-epoch-syscore', 'vlp-client', 'six-band'), out('--output'))
        + [(out('--output') + ['--old-usb'], KILL), (out('--output') + ['--old-regulatory'], KILL)],
    'pciec-policy-tests.py': [(out(), PASS)],
    'pmgr-bus-tests.py': [([], PASS)],
    'sep-suspend-errors-tests.py': [(out(), PASS)]
        + mutations('--mutation', ('unsupported-as-success', 'read-as-deadline', 'no-capability-check',
                                   'no-transport-check', 'lost-accept', 'nonphysical-wake',
                                   'wake-before-arm', 'bulk-error-as-success', 'omit-pending-wake',
                                   'omit-before-send', 'leak-prepared', 'sent-error-return',
                                   'accept-unanswered-gate', 'ignore-result', 'ignore-gate-timeout',
                                   'finish-without-physical', 'poll-after-callback', 'poll-unsupported',
                                   'stale-observation', 'drain-two', 'post-send-wake-refusal'), out()),
    'pmp-agx-vote-tests.py': [(out('--output'), PASS), (out('--output') + ['--mutations'], PASS)],
    'pmp-j613-floor-tests.py': [([], PASS)] + mutations('--mutant', ['no-hold', 'no-rate', 'no-streak', 'bad-word', 'foreign-ok', 'no-names', 'no-sum', 'no-board', 'no-default', 'no-disable'], []),
    'pmp-j613-hwmon-tests.py': [([], PASS)],
    'pmu-rtc-tests.py': [(out(), PASS)]
        + mutations('--mutation', ('ack-clobber', 'arm-ack', 'offset-seconds',
                                   'offset-sign', 'event-bit', 'wrong-control',
                                   'maverick-event', 'stowe-offset', 'baku-limit',
                                   'set-no-round', 'set-negative'), out()),
    'perf-mmap-alias-lifecycle-tests.py': [([], PASS)],
    'qspi-nvram-tests.py': [(out('--output'), PASS)],
    'radio-pm-tests.py': [([], PASS)],
    'radio-sleep-fence-tests.py': [([], PASS)],
    'radio-sleep-power-tests.py': [([], PASS)],
    'radio-sleep-snapshot-tests.py': [([], PASS)],
    'rtkit-crashlog-tests.py': [(out(), PASS)]
        + mutations('--mutation', ('buffer-header', 'section-header', 'section-size', 'footer-room',
                                   'mailbox-minimum', 'version-string', 'message-string'), out()),
    'rtkit-oslog-tests.py': [(out(), PASS)]
        + mutations('--mutate', ('admission', 'channel', 'counter', 'reset', 'buffer'), out()),
    'rtkit-retained-tests.py': [(out(), PASS)],
    't6030-clock-tests.py': [(out(), PASS)],
    't6030-display-admission-tests.py': [(out('--output'), PASS), (out('--output') + ['--mutations'], PASS)],
    't6030-display-linkage-tests.py': [(out('--output') + ['--conf', '{conf}'], PASS)],
    't6030-pmp-dt-tests.py': [(out('--output') + ['--dtc', '{dtc}'], PASS)],
    't6030-pmp-graph-tests.py': [(out('--output'), PASS), (out('--output') + ['--mutations'], PASS)],
    't6030-pmp-supplier-tests.py': [(out('--output'), PASS), (out('--output') + ['--mutations'], PASS)],
    't6030-wiring-tests.py': [(out(), PASS), (out() + ['--old-classifier', 'tb'], PASS),
                              (out() + ['--old-classifier', 'dcp'], PASS)],
    't6040-calibration-tests.py': [([], PASS)],
    't604x-cpu-order-tests.py': [(out('--output') + ['--dtc', '{dtc}'], PASS)],
    't8132-usb4-policy-tests.py': [(out('--output') + ['--dtc', '{dtc}'], PASS)],
    'tb-cio-sleep-tests.py': [(out('--output'), PASS)],
    'tb-dock-pm-tests.py': [(out('--output'), PASS)],
    'tb-dp-repair-tests.py': [(out('--output'), PASS), (out('--output') + ['--mutations'], PASS)],
    'tbnet-refill-tests.py': [(out(), PASS)]
        + mutations('--mutant', ('refill-count', 'no-retry', 'no-cancel', 'worker-allocation'), out()),
    'tiled-split-tests.py': [(out(), PASS), (out() + ['--mutations'], PASS)],
    'tunnel-route-tests.py': [(out(), PASS)],
}

# Rust unit-test files built with `rustc --test`, and shell drivers.
ASAHI_RUST = ['m3-batching-tests.rs', 'm3-stamp-tests.rs', 'm3-runtime-tests.rs', 'pmp-bootargs-tests.rs', 'pmp-power-tests.rs',
              'usage-tests.rs']
ASAHI_SHELL = ['g17-ray-query-timeout-tests.sh', 'g17-traversal-admission-tests.sh']

# Production G17 decoder vectors also build directly with rustc --test.
ASAHI_RUST += ['g17-telemetry-tests.rs', 'legacy-frequency-tests.rs']
ASAHI_RUST.append('m3-firmware-tests.rs')

# Python that runs on the host but imports its neighbours, so it keeps the
# script directory on sys.path (-E -s instead of -I).
SUITES = [
    ('apple-dcp', GPU / 'apple-dcp', ['python3', '-E', '-s', '-B', 'run.py', '--output', '{out}']),
    ('asahi-g15', GPU / 'asahi-g15', ['python3', '-E', '-s', '-B', 'run.py', '--output', '{out}']),
    ('asahi-m4', GPU / 'asahi-m4', ['python3', '-E', '-s', '-B', 'run-host-tests.py']),
    ('apple-afkv2-gcc', GPU / 'apple-afkv2', ['sh', 'run-afkv2-tests.sh']),
    ('apple-afkv2-clang', GPU / 'apple-afkv2', ['sh', 'run-afkv2-tests.sh']),
    ('t8140-external-bw', GPU, ['python3', '-I', '-B', 'test_t8140_external_bw.py']),
]


def jobs(work, conf, dtc):
    found = {p.name for p in ASAHI.iterdir()
             if re.search(r'-tests\.(py|rs|sh)$', p.name) and p.name != Path(__file__).name}
    # This Rust fixture is compiled and executed by the paired-header wrapper.
    if 'test-m3-uapi.py' in ASAHI_TESTS:
        found.discard('m3-uapi-tests.rs')
    known = set(ASAHI_TESTS) | set(ASAHI_RUST) | set(ASAHI_SHELL)
    missing = sorted(found - known)
    gone = sorted(name for name in known if not (ASAHI / name).is_file())
    if missing or gone:
        for name in missing:
            print(f'not registered in {Path(__file__).name}: tools/asahi/{name}', file=sys.stderr)
        for name in gone:
            print(f'registered but missing: tools/asahi/{name}', file=sys.stderr)
        sys.exit(2)
    result = []
    # The suites take longest, so start them first.
    for name, cwd, cmd in SUITES:
        d = work / name
        # Only ASan catches an unbounded legacy reply copy, so keep it on.
        env = {'AFKV2_SANITIZERS': '1', 'HOSTCC': name.rsplit('-', 1)[1]} if name.startswith('apple-afkv2') else {}
        result.append((name, [a.replace('{out}', str(d / 'out')) for a in cmd], cwd, PASS, d, env))
    for name, runs in ASAHI_TESTS.items():
        for index, (args, expect) in enumerate(runs):
            label = name.removesuffix('.py') + ('' if not args or args == out() or args == out('--output')
                                                 else ' ' + ' '.join(a for a in args if a not in ('{out}', '--out', '--output', '{conf}', '--conf',
                                                                                                   '{dtc}', '--dtc')))
            d = work / f'{name}.{index}'
            cmd = ['python3', '-I', '-B', str(ASAHI / name)] + [a.replace('{out}', str(d / 'out')).replace('{conf}', str(conf))
                                                                 .replace('{dtc}', str(dtc)) for a in args]
            result.append((label, cmd, ASAHI, expect, d, {}))
    for name in ASAHI_RUST:
        d = work / name
        binary = d / 'tests'
        cmd = ['sh', '-c', f'{os.environ.get("RUSTC", "rustc")} --edition=2021 --test {shlex.quote(str(ASAHI / name))} '
               f'-o {shlex.quote(str(binary))} && {shlex.quote(str(binary))}']
        result.append((name.removesuffix('.rs'), cmd, ASAHI, PASS, d, {}))
    for name in ASAHI_SHELL:
        d = work / name
        result.append((name.removesuffix('.sh'), ['sh', str(ASAHI / name), str(d / 'out')], ROOT, PASS, d, {}))
    return result


def kill_group(p):
    try:
        os.killpg(p.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass


def run(job, timeout):
    label, cmd, cwd, expect, d, extra = job
    (d / 'tmp').mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, TMPDIR=str(d / 'tmp'), **extra)
    start = time.monotonic()
    # Each test runs in its own session: a timeout (or an interrupted run) stops the test and everything it started, such
    # as a compiler or a `git show` of an old revision, which would otherwise outlive it, re-parented to init.
    p = subprocess.Popen(cmd, cwd=cwd, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                         text=True, errors='replace', start_new_session=True)
    try:
        text, _ = p.communicate(timeout=timeout)
        rc = p.returncode
    except subprocess.TimeoutExpired:
        kill_group(p)
        text, _ = p.communicate()
        rc = 'timeout'
    except BaseException:
        kill_group(p)
        p.wait()
        raise
    took = time.monotonic() - start
    (d / 'log').write_text(f'$ {shlex.join(cmd)}\n{text}\nexit: {rc}\n')
    if expect == PASS:
        ok = rc == 0
        why = '' if ok else f'exit {rc}'
    else:
        ok = rc not in (0, 'timeout') and KILL_SIGN.search(text) and not BROKEN_SIGN.search(text)
        why = '' if ok else (f'mutant survived (exit {rc})' if rc == 0 else f'exit {rc} without a control failure')
    return label, bool(ok), why, took, d / 'log'


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n', 1)[0])
    ap.add_argument('--jobs', type=int, default=os.cpu_count() or 2)
    ap.add_argument('--work', type=Path, help='keep logs and build products here (default: a temporary directory)')
    ap.add_argument('--conf', type=Path, help='a built scripts/kconfig/conf (default: build one)')
    ap.add_argument('--dtc', type=Path, help='a built scripts/dtc/dtc (default: build one)')
    ap.add_argument('--only', help='run only jobs whose name contains this text')
    ap.add_argument('--timeout', type=int, default=900)
    args = ap.parse_args()

    work = args.work or Path(tempfile.mkdtemp(prefix='aurora-host-tests-'))
    work = work.resolve()
    work.mkdir(parents=True, exist_ok=True)
    conf, dtc = args.conf, args.dtc
    build = work / 'kconfig'
    if conf is None or dtc is None:
        subprocess.run(['make', '-s', '-C', str(ROOT), f'O={build}', 'ARCH=arm64', 'allnoconfig'],
                       check=True, stdout=subprocess.DEVNULL)
    if conf is None:
        conf = build / 'scripts/kconfig/conf'
    if dtc is None:
        subprocess.run(['make', '-s', '-C', str(ROOT), f'O={build}', 'ARCH=arm64', 'scripts_dtc'],
                       check=True, stdout=subprocess.DEVNULL)
        dtc = build / 'scripts/dtc/dtc'
    todo = jobs(work, conf.resolve(), dtc.resolve())
    if args.only:
        todo = [j for j in todo if args.only in j[0]]
    print(f'{len(todo)} runs, {args.jobs} at a time, logs in {work}', flush=True)
    failed = []
    start = time.monotonic()
    with concurrent.futures.ThreadPoolExecutor(args.jobs) as pool:
        for label, ok, why, took, log in pool.map(lambda j: run(j, args.timeout), todo):
            print(f'{"ok" if ok else "FAIL"} {label} ({took:.1f}s){"" if ok else ": " + why}', flush=True)
            if not ok:
                failed.append((label, log))
    print(f'{len(todo) - len(failed)}/{len(todo)} passed in {time.monotonic() - start:.0f}s')
    for label, log in failed:
        text = log.read_text(errors='replace').splitlines()
        print(f'\n=== {label}: last lines of {log}')
        print('\n'.join(text[-40:]))
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
