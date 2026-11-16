#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Execute the actual report probe -> entry seed -> DCP -> PMP -> G15 graph.

The actual overlay supplies the report entries. OF storage, allocation, MMIO,
workqueue execution and the PMP hardware operations are typed host boundaries.
The production probe, descriptor validation, startup, waiter and supplier/command
bodies are extracted verbatim. This is not a firmware or hardware test.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import subprocess
from importlib.machinery import SourceFileLoader
ROOT = Path(__file__).resolve().parents[2]
extract = SourceFileLoader('supplier_test', str(ROOT / 'tools/asahi/t6030-pmp-supplier-tests.py')).load_module()

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--ref', help='read report source at a Git ref (negative control)')
    p.add_argument('--mutations', action='store_true')
    p.add_argument('--output', required=True, type=Path)
    args = p.parse_args(); args.output.mkdir(parents=True, exist_ok=False)
    source = subprocess.check_output(['git', '-C', str(ROOT), 'show', args.ref + ':' + extract.REPORT], text=True) if args.ref else (ROOT / extract.REPORT).read_text()
    overlay = (ROOT / 'drivers/soc/apple/t6030-pmp.dtso').read_text()
    entries = []
    for node in re.findall(r'report@[0-9a-f]+\s*\{(.*?)\n\s*\};', overlay, re.S):
        reg = int(re.search(r'reg = <(0x[0-9a-f]+|[0-9]+)>', node)[1], 0)
        entries.append((reg, 'apple,always-on;' in node, 'apple,no-ack;' in node, 'apple,defer-startup-ack;' in node))
    assert len(entries) == 4
    entry_data = ',\n'.join('{%d, %d, %d, %d}' % e for e in entries)
    names = ['static int apple_pmp_report_bit(', 'static int apple_pmp_start_check_layout(',
             'static int apple_pmp_start_describe(', 'static void apple_pmp_start_finish(',
             'static bool apple_pmp_start_stopping(', 'static void apple_pmp_start_work(',
             'static void apple_pmp_start_drain(', 'static int apple_pmp_start_init(',
             'static int apple_pmp_start_wait(', 'int apple_pmp_report_wait_ready(',
             'static int apple_pmp_report_set_state(', 'static int apple_pmp_report_entry_set_state(',
             'static int apple_pmp_report_entry_power_on(', 'static int apple_pmp_report_entry_power_off(',
             'static void apple_pmp_report_entry_free(', 'static int apple_pmp_report_entry_probe(',
             'static int apple_pmp_report_probe(', 'static struct device_node *apple_pmp_report_t6030_entry(',
             'static int apple_pmp_report_wait_t6030_14(', 'int apple_pmp_report_wait_supplier_ready(']
    if 'static int apple_pmp_report_wait_t6030_bound(' in source:
        names.insert(names.index('static struct device_node *apple_pmp_report_t6030_entry('), 'static int apple_pmp_report_wait_t6030_bound(')
    # System sleep (apple,release-for-sleep), where the source has it.
    sleep = 'static int apple_pmp_report_entry_sleep(' in source
    if sleep:
        at = names.index('static int apple_pmp_report_entry_power_on(')
        names[at:at] = ['static int apple_pmp_report_entry_sleep(',
                        'static int apple_pmp_report_entry_suspend_noirq(',
                        'static int apple_pmp_report_entry_resume_noirq(']
    offsets = re.search(r'static const struct apple_pmp_report_offsets apple_pmp_offsets_t6030 = \{.*?\n\};', source, re.S)[0]
    functions = '\n'.join(extract.function(source, n) for n in names)
    # The T8122 14.x wait (CONFIG_APPLE_T8122_GPU_PMP_LINK=0 here) precedes the dispatch; it is
    # added after the mutations are placed, so they target the T6030 path only.
    t8122 = extract.function(source, 'static int apple_pmp_report_wait_t8122_14(') + '\n'
    dispatch = 'int apple_pmp_report_wait_supplier_ready('
    def with_t8122(text):
        return text.replace(dispatch, t8122 + dispatch, 1)
    validation = re.sub(r'^#include .*\n', '', (ROOT / 'drivers/pmdomain/apple/pmp-report-validation.h').read_text(), flags=re.M)
    code = ('#define HAVE_SLEEP_RELEASE 1\n' if sleep else '') + offsets + '\n' + validation + '\n' + functions
    # The actual overlay must still describe a halted supplier and disabled DMA/mailbox.
    assert overlay.count('status = \"disabled\";') >= 3
    (args.output / 'production.h').write_text(with_t8122(code))
    (args.output / 'overlay.h').write_text('static const struct overlay_entry overlay_entries[] = {\n' + entry_data + '\n};\n')
    compiler = shlex.split(os.environ.get('CC', 'clang'))
    command = compiler + ['-std=gnu11', '-g', '-O1', '-Wall', '-Wextra', '-Werror',
                          '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
                          '-I', str(args.output), str(ROOT / 'tools/asahi/t6030-pmp-graph-tests.c'),
                          '-o', str(args.output / 'graph')]
    result = subprocess.run(command, capture_output=True, text=True)
    (args.output / 'build.log').write_text(result.stdout + result.stderr)
    if result.returncode:
        print(result.stderr); return 1
    result = subprocess.run([str(args.output / 'graph')], capture_output=True, text=True)
    (args.output / 'run.log').write_text(result.stdout + result.stderr)
    print(result.stdout + result.stderr)
    mutations = []
    if args.mutations:
        controls = [
            ('no-managed-owner', '.starts_pmp = IS_ENABLED(CONFIG_APPLE_T6030_GPU_PMP_LINK)', '.starts_pmp = false'),
            ('no-managed-probe', 'ret = apple_pmp_start_init(rep);', 'ret = 0; (void)apple_pmp_start_init;'),
            ('no-child-seeding', 'apple_pmp_report_entry_set_state(&ent->genpd, true);', '(void)0;'),
            ('no-storage-policy', 'ent->no_ack = of_property_read_bool(node, "apple,no-ack");', 'ent->no_ack = false;'),
            ('external-startup-ack', '!of_property_read_bool(child, "apple,defer-startup-ack")', 'true'),
            ('ignore-seeded-state', 'status || request != st->seed', 'status'),
            ('ignore-halted-state', 'status || request != st->seed', 'request != st->seed'),
            ('ignore-safety-check', 'ret = st->ops->check(st, &image);', 'ret = 0; (void)image;'),
            ('ignore-startup-ack', '(ack & st->startup_ack) != st->startup_ack', 'false'),
            ('ignore-entry-ack', 'status == PMP_REPORT_READY && (ack & BIT_ULL(id))', '((void)ack, status == PMP_REPORT_READY)'),
            ('storage-ack-layout', 'st->ack & ~st->record_seed,', 'st->seed & ~st->record_seed,'),
            ('skip-supplier-wait', 'apple_pmp_report_wait_ready(entry, msecs_to_jiffies(timeout_ms))', '((void)timeout_ms, 0)'),
        ]
        if sleep:
            controls += [
                ('sleep-releases-unacked', '!(readq(rep->base + rep->offsets->actual) & BIT_ULL(ent->id))', 'false'),
                ('sleep-not-reasserted', 'ret = apple_pmp_report_entry_sleep(ent, false);', 'ret = 0;'),
                ('sleep-ignores-mark', '!ent->sleep_release ||', ''),
                ('sleep-not-rewritten', '++tries < 3', '++tries < 1'),
            ]
        for name, old, new in controls:
            assert code.count(old) == 1, (name, code.count(old))
            (args.output / 'production.h').write_text(with_t8122(code.replace(old, new)))
            built = subprocess.run(command, capture_output=True, text=True)
            # A mutant must compile; compilation failure is not regression evidence.
            ran = subprocess.run([str(args.output / 'graph')], capture_output=True, text=True) if not built.returncode else None
            (args.output / (name + '.log')).write_text(built.stdout + built.stderr + (ran.stdout + ran.stderr if ran else ''))
            mutations.append({'name': name, 'build_rc': built.returncode, 'run_rc': ran.returncode if ran else None})
        (args.output / 'production.h').write_text(with_t8122(code))
        assert all(m['build_rc'] == 0 and m['run_rc'] != 0 for m in mutations), mutations
    (args.output / 'receipt.json').write_text(json.dumps({'source_ref': args.ref, 'source_sha256': hashlib.sha256(code.encode()).hexdigest(),
         'overlay_sha256': hashlib.sha256(overlay.encode()).hexdigest(), 'overlay_entries': entries,
         'command': command, 'returncode': result.returncode, 'mutations': mutations}, indent=2) + '\n')
    return bool(result.returncode)
if __name__ == '__main__':
    raise SystemExit(main())
