#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Compile Apple ISP control flow with observing host transport boundaries."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
DRIVER = ROOT / 'drivers/media/platform/apple/isp'


def function(source, name):
    match = re.search(r'^[\w *]+\b' + name + r'\(', source, re.M)
    if match is None:
        raise RuntimeError('missing function: ' + name)
    start = source.index('{', match.end())
    depth, end = 1, start + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[match.start():end] + '\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path)
    parser.add_argument('--emit', type=Path, help='write C for build-time compilation')
    parser.add_argument('--cc', default='clang')
    parser.add_argument('--control', choices=['send', 'overwrite', 'stop', 'reset'])
    args = parser.parse_args()
    inputs = {n: (DRIVER / n).read_text() for n in
              ['isp-ipc.c', 'isp-cmd.c', 'isp-cam.c', 'isp-fw.c']}
    groups = {
        'isp-ipc.c': ['ipc_chan_send'],
        'isp-cmd.c': ['cisp_send'],
        'isp-cam.c': ['apple_isp_stop_capture'],
        'isp-fw.c': ['isp_firmware_stop', 'isp_firmware_shutdown',
                     'apple_isp_firmware_halt', 'isp_firmware_boot',
                     'apple_isp_firmware_boot', 'apple_isp_firmware_shutdown'],
    }
    bodies = {n: function(inputs[f], n) for f, names in groups.items() for n in names}
    if args.control == 'send':
        bodies['ipc_chan_send'] = bodies['ipc_chan_send'].replace(
            'int err = READ_ONCE(isp->ipc_error);', 'int err = 0;')
    elif args.control == 'overwrite':
        bodies['cisp_send'] = bodies['cisp_send'].replace(
            'int err = READ_ONCE(isp->ipc_error);', 'int err = 0;')
    elif args.control == 'stop':
        bodies['apple_isp_stop_capture'] = bodies['apple_isp_stop_capture'].replace(
            'if (!isp_cmd_ch_stop(isp, 0)) /* TODO channel mask */',
            'isp_cmd_ch_stop(isp, 0);')
    elif args.control == 'reset':
        body = bodies['isp_firmware_boot']
        body = body.replace('WRITE_ONCE(isp->ipc_error, 0);', '')
        bodies['isp_firmware_boot'] = body.replace(
            'int err;', 'int err;\n\tWRITE_ONCE(isp->ipc_error, 0);', 1)
    code = (ROOT / 'tools/asahi/isp/session.c').read_text()
    code = code.replace('/* DRIVER FUNCTIONS */', '\n'.join(bodies.values()))

    if args.emit:
        args.emit.write_text(code)
        return 0

    def run(out):
        out.mkdir(parents=True, exist_ok=True)
        source, binary = out / 'session.c', out / 'session'
        source.write_text(code)
        compile_cmd = [args.cc, '-std=gnu11', '-Wall', '-Wextra', '-Werror',
                       '-Wno-unused-parameter', '-O1', '-g', '-pthread',
                       '-fsanitize=address,undefined', '-fno-sanitize-recover=all',
                       str(source), '-o', str(binary)]
        subprocess.run(compile_cmd, check=True)
        result = subprocess.run([str(binary)], capture_output=True, text=True)
        receipt = {'control': args.control, 'compile': compile_cmd,
                   'exit': result.returncode, 'stdout': result.stdout,
                   'stderr': result.stderr, 'sources': {
                       f: hashlib.sha256(s.encode()).hexdigest() for f, s in inputs.items()}}
        (out / 'results.json').write_text(json.dumps(receipt, indent=2) + '\n')
        print(result.stdout or result.stderr, end='')
        return result.returncode

    if args.out:
        return run(args.out)
    with tempfile.TemporaryDirectory(prefix='isp-tests-') as out:
        return run(Path(out))


if __name__ == '__main__':
    raise SystemExit(main())
