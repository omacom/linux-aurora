#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Exercise actual NHI/PCIe owner callbacks with modeled suppliers and injected failures."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]


def function(source, name):
    match = re.search(r'^(?:static )?(?:int|void) ' + name + r'\([^;]+?\{.*?\n\}',
                      source, re.M | re.S)
    assert match, name
    return match.group()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    paths = ['drivers/pci/controller/pcie-apple-cio.c',
             'drivers/thunderbolt/apple-cio-sleep.c',
             'drivers/pci/controller/pcie-apple.c', 'drivers/thunderbolt/apple.c']
    sources = {p: (ROOT / p).read_text() for p in paths}
    names = ['apple_nhi_cio_phy',
             'apple_nhi_cio_prepare', 'apple_nhi_cio_start_cpu', 'apple_nhi_cio_power_on',
             'apple_nhi_cio_wake', 'apple_nhi_cio_domains_off', 'apple_nhi_cio_dart_off',
             'apple_nhi_cio_suspend', 'apple_nhi_cio_resume', 'apple_nhi_cio_complete']
    # The PHY's steps, as T8132's lane PHY handle takes them (T6050's calls are not modelled).
    steps = re.search(r'^enum \{ CIO_PHY_PREPARE,[^\n]+', sources[paths[1]], re.M).group()
    methods = steps + '\n' + '\n'.join(function(sources[paths[1]], n) for n in names)
    defines = '\n'.join(re.findall(r'^#define\s+(?:PORT_|APPLE_CIO_M3_|APPLE_CIO_LSTX_)[^\n]+',
                                    sources[paths[2]] + sources[paths[3]], re.M))
    fixture = Path(__file__).with_suffix('.c').read_text()
    fixture = fixture.replace('/* ACTUAL_PORT_DEFINES */', defines)
    fixture = fixture.replace('/* ACTUAL_PCIE */', sources[paths[0]])
    fixture = fixture.replace('/* ACTUAL_NHI */', methods)
    mutations = {
        'forget-local-perst-loss': ('\t\tgoto unwind;\n\tpcie->cio_port_reset = true;',
                                    '\t\tgoto unwind;\n\tpcie->cio_port_reset = false;'),
        'inverted-t8132-reset-ack': ('stat & PORT_TUNSTAT_PERST_ACK_PEND,',
                                   '!(stat & PORT_TUNSTAT_PERST_ACK_PEND),'),
        'skip-port-dart-off': ('before APPCLK. */\n\tret = apple_dart_cio_off(pcie->cio_dart);',
                               'before APPCLK. */\n\tret = 0;'),
        'skip-nhi-dart-off': ('ret = apple_nhi_cio_dart_off(anhi);', 'ret = 0;'),
        'skip-dart-restore': ('return apple_dart_cio_on(anhi->cio_dart);', 'return 0;'),
        'lose-partial-domain': ('anhi->cio_pd_suspended |= BIT(n);', '(void)n;'),
        'skip-port-unwind': ('callback failed. */\n\trecover = apple_pcie_cio_resume(pcie);',
                             'callback failed. */\n\trecover = 0;'),
        'skip-nhi-unwind': ('recover = apple_nhi_cio_power_on(anhi) ?: apple_nhi_cio_wake(anhi);',
                            'recover = 0;'),
        'skip-unwind-power-cycle': ('if (!anhi->cio_cycled) {\n\t\trecover', 'if (false) {\n\t\trecover'),
        'panic-on-lost-port': ('return pcie->resume_failed ? 0 : ret;', 'return ret;'),
        'wait-for-lost-link': ('if (pcie->cio_lost)\n\t\treturn -ENOTCONN;', ''),
        'ignore-lost-pci': ('tunnels\\n");\n\t\tif (anhi->cio_pcie)\n\t\t\tapple_pcie_tunnel_cio_lost(anhi->cio_pcie);',
                            'tunnels\\n");\n\t\t(void)0;'),
        'skip-failed-port-stop': ('/* Until it is back, a later teardown must still stop the port. */\n\tport->needs_stop = true;', ''),
        'ignore-physical-state': ('ret = apple_pmgr_ps_verify_state(pd, true);',
                                  'ret = 0;'),
        't6050-port-refuses-late': ('ret = apple_dart_cio_check(pcie->cio_dart, true);',
                                    'ret = 0;'),
        't6050-refuses-after-release': ('if (!pcie && anhi->cio_t6050) {\n\t\tret = -ENODEV;\n\t\tgoto release_cable;',
                                        'if (false) {\n\t\tret = -ENODEV;\n\t\tgoto release_cable;'),
        'refused-without-rescan': ('\ttb_cio_sleep_refused(anhi->tb);\n', '\ttb_cio_sleep_complete(anhi->tb);\n'),
    }
    variants = {'real': fixture}
    for name, (old, new) in mutations.items():
        assert fixture.count(old) == 1, name
        variants[name] = fixture.replace(old, new)
    with tempfile.TemporaryDirectory(prefix='cio-native-') as tmp:
        out = args.output or Path(tmp)
        out.mkdir(parents=True, exist_ok=True)
        results = {}
        for name, text in variants.items():
            unit, binary = out / (name + '.c'), out / name
            unit.write_text(text)
            subprocess.run(['cc', '-std=gnu11', '-Wall', '-Wextra', '-Werror',
                            '-Wno-unused-function', '-Wno-unused-parameter',
                            '-Wno-misleading-indentation', '-g', '-fsanitize=address,undefined',
                            str(unit), '-o', str(binary)], check=True)
            run = subprocess.run([str(binary)], capture_output=True, text=True, timeout=10)
            results[name] = {'exit': run.returncode, 'stdout': run.stdout, 'stderr': run.stderr}
            assert (run.returncode == 0) == (name == 'real'), (name, results[name])
            if name == 'real': print(run.stdout, end='')
        (out / 'receipt.json').write_text(json.dumps({
            'sha256': {p: hashlib.sha256(s.encode()).hexdigest() for p, s in sources.items()},
            'results': results}, indent=2) + '\n')
        print(f'PASS {len(mutations)} negative controls')


if __name__ == '__main__':
    main()
