#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Exercise native CM/path replay using actual bodies and modeled router I/O."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]


def function(source, name):
    match = re.search(r'^(?:static )?(?:int|void|bool) ' + name +
                      r'\([^;]+?\{.*?\n\}', source, re.M | re.S)
    assert match, name
    return match.group()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    paths = ['drivers/thunderbolt/path-sleep.c', 'drivers/thunderbolt/tunnel.c',
             'drivers/thunderbolt/tb-cio-sleep.c', 'drivers/thunderbolt/tb_regs.h',
             'drivers/thunderbolt/nhi.h']
    src = {p: (ROOT / p).read_text() for p in paths}
    regs = re.search(r'struct tb_regs_hop \{.*?\} __packed;', src[paths[3]], re.S).group()
    # The host's sleep-in-place quirks, as the NHI driver sets them.
    quirks = '\n'.join(re.findall(r'^#define QUIRK_CIO_\w+\s+BIT\(\d+\)', src[paths[4]], re.M))
    methods = '\n'.join(function(src[paths[1]], n) for n in
                        ('tb_tunnel_sleep_prepare', 'tb_tunnel_sleep',
                         'tb_tunnel_wake', 'tb_tunnel_sleep_complete'))
    unit = Path(__file__).with_suffix('.c').read_text()
    for key, value in [('REGS', regs), ('QUIRKS', quirks), ('PATH', src[paths[0]]),
                       ('TUNNEL', methods), ('CM', src[paths[2]])]:
        unit = unit.replace('/* ACTUAL_' + key + ' */', value)
    mutations = {
        'lose-partial-path': ('path->sleep_valid = true;', 'path->sleep_valid = false;'),
        'skip-hop-replay': ('&path->sleep_hops[i], TB_CFG_HOPS,', '&observed, TB_CFG_HOPS,'),
        'skip-uid-check': ('if (ret || uid != tcm->cio_host_uid)', 'if (ret)'),
        'skip-router-restore': ('ret = apple_router_restore_sleep(tb->root_switch, &tcm->cio_router);', 'ret = 0;'),
        'skip-failed-suspend-unwind': ('recover = tb_cio_wake_locked(tb, NULL);', 'recover = 0;'),
        'lost-dock-fails': ('if (ret && ret != -ENOTCONN)', 'if (ret)'),
        'replay-lost-tunnel': ('if (tb_tunnel_is_invalid(tunnel)) {', 'if (false) {'),
        'hide-lost-pci': ('*host_pci_lost = true;', '(void)host_pci_lost;'),
        'lost-router-topology': ('if (wake && port->remote->sw->is_unplugged)', 'if (false)'),
        'restart-needs-saved-state': ('restarted = tcm->cio_host_slept;',
                                      'restarted = tcm->cio_router.valid;'),
        'unwind-unsaved-sleep': ('tb_warn(tb, "CIO router state not saved: %d\\n", ret);',
                                 'goto unwind;'),
        'keep-display-tunnels': ('tb_tunnel_is_dp(tunnel) || ', ''),
        'allow-host-callbacks': ('tunnel->sleep_replay = true;', 'tunnel->sleep_replay = false;'),
        'skip-system-work-drain': ('cancel_delayed_work_sync(&tcm->groups[i].release_work)', 'false'),
        'skip-pm-roles': ('ret = tb_switch_cio_pm_roles(port->remote->sw);', 'ret = 0;'),
        'save-without-state': ('if (!(tb->nhi->quirks & QUIRK_CIO_NO_ROUTER_STATE)) {', 'if (true) {'),
        'restore-failure-fatal': ('if (ret && !(tb->nhi->quirks & QUIRK_CIO_WAKE_GIVES_UP))', 'if (ret)'),
        'wait-links-long': ('QUIRK_CIO_WAKE_GIVES_UP ? 2 : 30', 'QUIRK_CIO_WAKE_GIVES_UP ? 30 : 30'),
        'drain-macos-paths': ('if (path->tb->nhi->quirks & QUIRK_CIO_MACOS_FRAMES)\n\t\treturn 0;',
                              'if (false)\n\t\treturn 0;'),
        'rewrite-unslept-hops': ('!path->sleep_lost && observed.enable', 'false && observed.enable'),
        'keep-xdomain-plugged': ('\t\txd->is_unplugged = true;\n', ''),
        'skip-xdomain-unregister': ('the host must not hold across its power cycle.\n\t */\n\ttb_domain_unregister_unplugged_xdomains(tb);\n',
                                    'the host must not hold across its power cycle.\n\t */\n'),
        'skip-device-router-xdomains': ('\t\t\ttb_cio_release_xdomains(tb, port->remote->sw);\n', ''),
        'refused-skips-rescan': ('\t\ttb_scan_switch(tb->root_switch);\n}', '\t\t(void)0;\n}'),
        'rescan-while-stopping': ('if (tcm->hotplug_active && !tcm->stopping)\n\t\ttb_scan_switch', 'if (true)\n\t\ttb_scan_switch'),
        'refused-keeps-cleanup': ('\t\t\ttcm->resume_cleanup_pending = false;\n\t\t}\n\t}\n\ttb_domain_unregister', '\t\t}\n\t}\n\ttb_domain_unregister'),
        'keep-xdomain-port': ('\t\tif (port->xdomain == xd)\n\t\t\tport->xdomain = NULL;\n', ''),
    }
    variants = {'real': unit}
    for name, (old, new) in mutations.items():
        assert unit.count(old) == 1, name
        variants[name] = unit.replace(old, new)
    with tempfile.TemporaryDirectory(prefix='tb-cio-') as tmp:
        out = args.output or Path(tmp)
        out.mkdir(parents=True, exist_ok=True)
        results = {}
        for name, text in variants.items():
            source, binary = out / (name + '.c'), out / name
            source.write_text(text)
            subprocess.run(['cc', '-std=gnu11', '-Wall', '-Wextra', '-Werror',
                            '-D__packed=__attribute__((packed))',
                            '-D__maybe_unused=__attribute__((unused))',
                            '-D__cleanup(f)=__attribute__((cleanup(f)))',
                            '-DEXPORT_SYMBOL_FOR_MODULES(...)=',
                            '-Wno-unused-function', '-Wno-unused-parameter',
                            '-Wno-misleading-indentation', '-g', '-fsanitize=address,undefined',
                            str(source), '-o', str(binary)], check=True)
            run = subprocess.run([str(binary)], capture_output=True, text=True, timeout=10)
            results[name] = {'exit': run.returncode, 'stdout': run.stdout, 'stderr': run.stderr}
            assert (run.returncode == 0) == (name == 'real'), (name, results[name])
            if name == 'real':
                print(run.stdout, end='')
        (out / 'receipt.json').write_text(json.dumps({
            'sha256': {p: hashlib.sha256(s.encode()).hexdigest() for p, s in src.items()},
            'results': results}, indent=2) + '\n')
        print(f'PASS {len(mutations)} negative controls')


if __name__ == '__main__':
    main()
