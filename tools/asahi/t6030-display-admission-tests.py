#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Actual T6030 display publisher and G15 admission, with pthread boundaries.

The publisher is the shared G15 display slot code; T6030 admission goes through
the shared link helper. Tests initial admission and pointer lifetime, not safe
shutdown of retained G15 firmware after forced display unbind. Full DRM
publication placement is checked as source ordering separately from executable
host boundary controls.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess
from importlib.machinery import SourceFileLoader
ROOT = Path(__file__).resolve().parents[2]
extract = SourceFileLoader('supplier_test', str(ROOT / 'tools/asahi/t6030-pmp-supplier-tests.py')).load_module()

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', required=True, type=Path)
    p.add_argument('--mutations', action='store_true')
    args = p.parse_args(); args.output.mkdir(parents=True, exist_ok=False)
    drv = (ROOT / 'drivers/gpu/drm/apple/apple_drv.c').read_text()
    glue = (ROOT / 'drivers/gpu/drm/asahi/g15/glue.c').read_text()
    init = extract.function(drv, 'static int apple_drm_init(')
    uninit = extract.function(drv, 'static void apple_drm_uninit(')
    assert init.index('ret = drm_dev_register(') < init.index('drm_client_setup_with_fourcc(') < init.index('ret = apple_g15_display_publish(')
    assert uninit.index('apple_g15_display_withdraw(') < uninit.index('apple_drm_quiesce_connectors(') < uninit.index('drm_dev_unplug(')
    assert 'drm_dev_unregister(&apple->drm);\n\t\tgoto err_unbind;' in init
    code = '\n'.join([extract.function(drv, 'static struct device **apple_g15_display_slot('),
                      extract.function(drv, 'static int apple_g15_display_publish('),
                      extract.function(drv, 'static void apple_g15_display_withdraw('),
                      extract.function(drv, 'static int apple_g15_link_device('),
                      extract.function(drv, 'int apple_drm_t6030_link_device('),
                      extract.function(glue, 'int asahi_g15_pmp_prepare(struct device *dev)\n')])
    variants = [('control', code)]
    if args.mutations:
        for name, old, new in [
            ('no-ready-gate', 'if (!*display)', 'if (false)'),
            ('no-display-get', '*slot = get_device(dev);', '*slot = dev;'),
            ('no-display-put', 'put_device(old);', '(void)old;'),
            ('replace-live-provider', 'return *slot == dev ? 0 : -EBUSY;', '*slot = dev;'),
            ('ignore-display-error', 'if (ret)\n\t\treturn ret;', 'if (false)\n\t\treturn ret;'),
            ('wrong-gpu', '!of_device_is_compatible(consumer->of_node, gpu)', '((void)gpu, false)'),
            ('withdraw-wrong-provider', 'if (*slot == dev)', 'if (((void)dev, true))'),
            ('shared-display-slot', 'return &apple_t6031_display;', 'return &apple_t6030_display;'),
            ('missing-t6034-slot', 'return &apple_t6034_display;', 'return NULL;'),
            ('t6034-shared-display-slot', 'return &apple_t6034_display;', 'return &apple_t6031_display;'),
        ]:
            assert code.count(old) == 1, (name, code.count(old))
            variants.append((name, code.replace(old, new)))
    compiler = shlex.split(os.environ.get('CC', 'clang'))
    command = compiler + ['-std=gnu11', '-g', '-O1', '-Wall', '-Wextra', '-Werror',
                '-fsanitize=address,undefined', '-fno-omit-frame-pointer', '-pthread',
                '-I', str(args.output), str(ROOT / 'tools/asahi/t6030-display-admission-tests.c'), '-o', str(args.output / 'admission')]
    results = []
    for name, source in variants:
        (args.output / 'production.h').write_text(source)
        built = subprocess.run(command, capture_output=True, text=True)
        ran = subprocess.run([str(args.output / 'admission')], capture_output=True, text=True) if not built.returncode else None
        (args.output / (name + '.log')).write_text(built.stdout + built.stderr + (ran.stdout + ran.stderr if ran else ''))
        results.append({'name': name, 'build_rc': built.returncode, 'run_rc': ran.returncode if ran else None})
    (args.output / 'production.h').write_text(code)
    good = all(x['build_rc'] == 0 and (x['run_rc'] == 0 if x['name'] == 'control' else x['run_rc'] != 0) for x in results)
    (args.output / 'receipt.json').write_text(json.dumps({'passed': good, 'source_sha256': hashlib.sha256(code.encode()).hexdigest(),
        'publication_order_verified': True, 'forced_supplier_unbind_safety_claimed': False, 'command': command, 'results': results}, indent=2) + '\n')
    print(json.dumps({'passed': good, 'results': results}))
    return not good
if __name__ == '__main__':
    raise SystemExit(main())
