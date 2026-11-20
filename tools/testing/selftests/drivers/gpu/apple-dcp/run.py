#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
# Copyright 2026 Ryan Murray
"""Host regressions for Apple external display ownership and recovery.

Only public synthetic fixtures and extracted production functions are used.
The transport, MMIO, locks and work queues are mocks, not hardware validation.
"""
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import struct
import tempfile
import traceback

from common import run_unit, block
import edid
import recovery
import lifecycle
import phy
import pm
import quiesce
import transaction
import accounting
import idle
import desktop
import audio
import j616s
import rate
import boot
import m4
import display
import colorimetry


def parking_unit(root):
    fixture = phy.parking(root)
    return ''.join(fixture[k] for k in ('prefix', 'production', 'suffix'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=Path(__file__).resolve().parents[6])
    parser.add_argument('--output', type=Path, help='retain generated C, commands and results here')
    parser.add_argument('--group', choices=('all', 'edid', 'rate', 'phy', 'lifecycle', 'boot', 'quiesce', 'm4', 'display', 'audio'), default='all')
    args = parser.parse_args()
    root = args.source.resolve()
    output = args.output or Path(tempfile.mkdtemp(prefix='apple-dcp-selftest-'))
    output.mkdir(parents=True, exist_ok=True)
    apple = root / 'drivers/gpu/drm/apple'
    includes = (apple, root / 'tools/testing/selftests/drivers/gpu/apple-afkv2/host-include')
    parking_cases = tuple((c,) for c in ('mux', 'power', 'reset', 'cleanup', 'orientation',
                                       'orientation-faults', 'backend-change', 'safe-first',
                                       'failed-flip-return', 'live-park'))
    tests = [
        ('display', 'panel-colorimetry', lambda r: colorimetry.panel(r, output), ((),), ('-Wno-unused-function', '-Wno-unused-const-variable')),
        ('audio', 'compact-audio-reply-bounds', audio.reply_bounds, ((),), ()),
        ('display', 'desktop-gamma', desktop.gamma, ((),), ()),
        ('display', 'hdr-static-metadata', display.hdr_metadata, ((),), ()),
        ('display', 'ctm-encoding', colorimetry.ctm, ((),), ()),
        ('lifecycle', 'desktop-modeset-unmap', desktop.modeset_unmap, ((),), ()),
        ('lifecycle', 'desktop-unknown-modeset-recovery', desktop.modeset_unknown, ((),), ()),
        ('display', 'panel-backlight-ramp', display.backlight, ((),), ()),
        ('display', 'panel-takeover-registration', desktop.brightness_takeover, ((),), ()),
        ('display', 'panel-brightness-commit', display.brightness_commit, ((),), ()),
        ('display', 'panel-brightness-takeover', display.takeover, ((),), ()),
        ('display', 'panel-mode-catalog', display.modes, ((),), ()),
        ('display', 'desktop-brightness-notifications', desktop.brightness_notifications, ((),), ()),
        ('display', 'j616s-panel-identity', lambda r: j616s.identity(r, output), ((),), ('-Wno-unused-function', '-Wno-unused-const-variable')),
        ('display', 'desktop-adt-identity', desktop.adt_identity, ((),), ()),
        ('display', 'desktop-identity-trace-budget', desktop.identity_trace_budget, ((),), ()),
        ('display', 'desktop-identity-keys', desktop.identity_keys, ((),), ('-Wno-unused-function',)),
        ('display', 'desktop-panel-identity', lambda r: desktop.panel_identity(r, output), ((),), ()),
        ('display', 'desktop-powerlog-framing', desktop.powerlog_framing, ((),), ()),
        ('display', 'desktop-brightness-keys-reports', desktop.keys_and_reports, ((),), ()),
        ('display', 'panel-refresh-range', display.refresh, ((),), ()),
        ('display', 'panel-adaptive-watchdog', display.watchdog, ((),), ()),
        ('m4', 'm4-wire-golden', lambda r: m4.golden_check(r, output), (), ()),
        ('m4', 'm4-protocol', m4.protocol, ((),), ('-Wno-unused-function', '-Wno-unused-const-variable')),
        ('m4', 'm4-common-plane', m4.plane, ((),), ('-Wno-unused-function', '-Wno-unused-const-variable')),
        ('m4', 'm4-producer-retirement', m4.prepare, ((),), ('-Wno-unused-function',)),
        ('m4', 'm4-dpms-retirement', m4.blank, ((),), ('-Wno-unused-function', '-Wno-unused-const-variable')),
        ('m4', 'm4-power-gate', m4.gate, ((),), ('-Wno-unused-function',)),
        ('m4', 'm4-dart-consumer', m4.dart, ((),), ('-Wno-unused-function',)),
        ('m4', 'm4-panel-timing', m4.timing, ((),), ('-Wno-unused-function', '-Wno-unused-const-variable')),
        ('boot', 'oslog-events', boot.oslog, ((),), ()),
        ('boot', 'syslog-policy', boot.syslog, ((),), ()),
        ('boot', 't6041-v27-admission', boot.t6041_v27_admission, ((),), ()),
        ('edid', 'dpavserv-start', edid.startup, ((),), ()),
        ('edid', 'service-rebind', edid.rebind, ((),), ()),
        ('edid', 'provider-admission', edid.admission, ((),), ()),
        ('edid', 'ordered-rx', edid.rx_progress, ((),), ('-pthread',)),
        ('rate', 'parser-policy', lambda r: rate.policy(r)['unit'], ((),), ()),
        ('rate', 'publication-hpd', rate.hotplug, ((),), ()),
        ('rate', 'panel-dimensions', rate.panel_dimensions, ((),), ()),
        ('rate', 'metadata-decoder', lambda r: rate.metadata(r, output), ((),), ('-Wno-sign-compare',)),
        ('rate', 'bounded-retry', rate.retry, ((),), ()),
        ('phy', 'pm-ownership', pm.ownership, ((),), ()),
        ('phy', 'controller-lifecycle', pm.controller_lifecycle, ((),), ('-pthread',)),
        ('phy', 'asynchronous-root-state', pm.asynchronous_root_state, ((),), ()),
        ('phy', 'upstream-resume-boundary', pm.upstream_resume_boundary, ((),), ()),
        ('phy', 'usb-unreachable', lambda r: pm.ownership(r, True), ((),), ()),
        ('lifecycle', 'cpuidle-s2idle', idle.s2idle, ((),), ('-Wno-unused-parameter', '-Wno-unused-variable')),
        ('lifecycle', 'typec-probe', pm.probe, ((),), ()),
        ('lifecycle', 'quiesce-before-safe', quiesce.make_unit, tuple((c,) for c in ('ordered','power','ack','role','bolt','mux','duplicate','error-latched','withdrawn-still-latched','legacy','fabric','tunnel','generic-disconnect','ordinary-remove','ordinary-disconnect','ordinary-role-error','ordinary-phy-error','ordinary-role-reprobe','ordinary-unsupported-remove','ordinary-unsupported-disconnect','ordinary-unregistered-remove','ordinary-unregistered-disconnect','noop-fixed-remove','noop-fixed-disconnect')), ()),
        ('lifecycle', 'probe-mux-pm-dispatch', pm.probe_dispatch, ((),), ()),
        ('phy', 'parking', lambda r: parking_unit(r), parking_cases, ()),
        ('phy', 'adverse-retry', phy.adverse, tuple((c,) for c in ('bist', 'init', 'mux', 'gate', 'dp-cache', 'off', 'success',
                                                               'post-tunable')), ()),
        ('phy', 'masked-context', phy.usb_context, ((),), ()),
        ('phy', 'off-hardening', phy.hardening, ((),), ()),
        ('phy', 'warm-reset', phy.warm_reset, ((),), ()),
        ('phy', 'pipe', phy.pipe, tuple((c,) for c in ('backend', 'host', 'mux', 'same', 'park', 'requested', 'scope')), ()),
        ('phy', 'unwind', phy.unwind, ((),), ()),
        ('phy', 'aux', phy.aux, ((),), ()),
        ('lifecycle', 'physical-disconnect', lifecycle.disconnect, ((),),
         ('-Wno-unused-variable', '-Wno-unused-function', '-Wno-unused-but-set-variable')),
        ('lifecycle', 'fast-replug-bounce', lifecycle.bounce, ((),),
         ('-Wno-unused-variable', '-Wno-unused-function', '-Wno-unused-but-set-variable')),
        ('lifecycle', 'stale-crtc-poweroff', lifecycle.poweroff_replug, ((),), ()),
        ('lifecycle', 'dropped-swap-teardown', lifecycle.dropped, ((),),
         ('-Wno-unused-variable', '-Wno-unused-function', '-Wno-unused-but-set-variable')),
        ('lifecycle', 'dwc3-teardown', lifecycle.teardown, (('all',),), ()),
        ('lifecycle', 'resource-profile', lifecycle.profile, ((),), ('-Wno-sign-compare',)),
        ('lifecycle', 'firmware-provider', lifecycle.firmware, ((),), ()),
        ('lifecycle', 'readiness', lifecycle.readiness, ((),), ()),
        ('lifecycle', 'vrr-rejection', lifecycle.vrr, ((),), ()),
        ('lifecycle', 'power-reply', lifecycle.power_reply, ((),), ()),
        ('lifecycle', 'dispatcher-scope', lifecycle.dispatcher, ((),), ()),
    ]
    transaction_tests = transaction.controls(root) + accounting.controls(root)
    tests.extend(('lifecycle', name, lambda r, u=unit: u, cases, flags)
                 for name, unit, cases, flags in transaction_tests)
    tests.append(('phy', 'usb-kconfig', lambda r: pm.kconfig(r, output), (), ()))
    tests = [t for t in tests if args.group in ('all', t[0]) or (args.group == 'quiesce' and t[1] in ('quiesce-before-safe',) + tuple(n for n, _, _, _ in transaction_tests))]
    print('TAP version 13', flush=True)
    print('1..' + str(len(tests)), flush=True)
    results = []
    for index, (group, name, build, cases, flags) in enumerate(tests, 1):
        result = {'name': name, 'group': group, 'passed': False}
        try:
            unit = build(root)
            if unit is None:
                result['producer_passed'] = True
            elif isinstance(unit, tuple):
                unit, cases = unit
            if unit is not None and name in ('provider-admission', 'ordered-rx',
                    'publication-hpd', 'panel-dimensions', 'stale-crtc-poweroff', 'readiness'):
                # These fixtures select older external-display backends. The M4
                # transport is covered by the separate m4 group, not mocked here.
                prefix = ('#include <stdbool.h>\n#include <assert.h>\n'
                          '#define iomfb_is_m4(...) false\n'
                          '#define iomfb_m4_get_mode(...) (-95)\n'
                          '#define iomfb_m4_blank(...) (-95)\n')
                if 'dcp_fail_stop(' in unit:
                    # Firmware 27 fail-stops like H17P; no fixture here runs firmware 27.
                    prefix += ('struct apple_dcp;\n'
                               '#define DCP_FIRMWARE_V_27_0 0x2700\n'
                               'static inline bool dcp_fail_stop(const struct apple_dcp *dcp);\n')
                    unit += '\n' + block((apple / 'dcp-internal.h').read_text(),
                                          'static inline bool dcp_fail_stop(')
                if name == 'ordered-rx':
                    import re
                    rtkit = (root / 'drivers/soc/apple/rtkit.c').read_text()
                    endpoint = re.search(r'APPLE_RTKIT_EP_COREANALYTICS\s*=\s*(\d+)', rtkit).group(1)
                    prefix += ('#define APPLE_RTKIT_EP_COREANALYTICS ' + endpoint + '\n'
                               '#define apple_rtkit_coreanalytics_rx(...) '
                               'assert(!"CoreAnalytics is outside this ordered-display fixture")\n')
                    # The T6050 PMP registry-v2 buffer shares endpoint 7.
                    prefix += ('#define apple_rtkit_pmp_aux_rx(...) '
                               'assert(!"The PMP auxiliary buffer is outside this ordered-display fixture")\n')
                unit = prefix + unit
            if unit is not None:
                result['translation_unit_sha256'] = run_unit(name, unit, output, cases, includes if name == 'service-rebind' else (), flags)
            if name == 'service-rebind':
                recovery.check(root, output)
            if name == 'metadata-decoder':
                rate.check_metadata(output)
            result['passed'] = True
        except Exception:
            result['failure'] = traceback.format_exc()
            (output / (name + '-failure.log')).write_text(result['failure'])
        print(('ok ' if result['passed'] else 'not ok ') + str(index) + ' - ' + name, flush=True)
        results.append(result)
    sources = {}
    for area in ('drivers/gpu/drm/apple', 'drivers/phy/apple', 'drivers/usb/dwc3', 'drivers/usb/typec/tipd'):
        for path in sorted((root / area).rglob('*')):
            if path.is_file() and path.suffix in ('.c', '.h'):
                sources[str(path.relative_to(root))] = hashlib.sha256(path.read_bytes()).hexdigest()
    (output / 'results.json').write_text(json.dumps({'source': str(root), 'sources': sources, 'results': results}, indent=2) + '\n')
    print('# Artifacts: ' + str(output), flush=True)
    return int(any(not r['passed'] for r in results))


if __name__ == '__main__':
    raise SystemExit(main())
