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
    parser.add_argument('--base-source', type=Path, help='pinned public baseline for rollback control')
    parser.add_argument('--cc', default='clang')
    parser.add_argument('--suite', choices=['session', 'buffers', 'receive', 'returns', 'queue'], default='session')
    parser.add_argument('--control', choices=['send', 'overwrite', 'stop', 'reset', 'rollback', 'drain', 'address', 'pool', 'duplicate', 'snapshot', 'queue-error'])
    args = parser.parse_args()
    allowed = {'session': {None, 'send', 'overwrite', 'stop', 'reset'},
               'buffers': {None, 'rollback'}, 'receive': {None, 'drain'},
               'returns': {None, 'address', 'pool', 'duplicate', 'snapshot'},
               'queue': {None, 'queue-error'}}
    if args.control not in allowed[args.suite]:
        parser.error('control does not belong to this suite')
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
    if args.suite == 'session':
        groups['isp-ipc.c'] += ['chan_rx_done', 'ipc_chan_handle']
        groups['isp-fw.c'] += ['apple_isp_isr_thread']
    bodies = {n: function(inputs[f], n) for f, names in groups.items() for n in names}
    if args.suite == 'session' and 'apple_isp_video_error(isp);' in bodies['apple_isp_isr_thread']:
        bodies = {'apple_isp_video_error': function(
            (DRIVER / 'isp-v4l2.c').read_text(), 'apple_isp_video_error'), **bodies}
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
    if args.suite in ['buffers', 'returns', 'queue']:
        inputs = {'isp-v4l2.c': (DRIVER / 'isp-v4l2.c').read_text()}
        bodies = {n: function(inputs['isp-v4l2.c'], n) for n in
                  ['isp_submit_buffers', 'apple_isp_start_streaming']}
        bodies['apple_isp_stop_capture'] = function(
            (DRIVER / 'isp-cam.c').read_text(), 'apple_isp_stop_capture')
        if args.control == 'rollback':
            old = args.base_source.read_text() if args.base_source else subprocess.check_output(
                ['git', 'show', '7876c8e8349c4f3d0a2f4ce982e3e13ce5bb5d70:'
                 'drivers/media/platform/apple/isp/isp-v4l2.c'], cwd=ROOT, text=True)
            bodies['isp_submit_buffers'] = function(old, 'isp_submit_buffers')
    if args.suite == 'returns':
        bodies = {n: function(inputs['isp-v4l2.c'], n) for n in
                  ['isp_returned_surface', 'ipc_bt_handle', 'isp_submit_buffers',
                   'apple_isp_start_streaming', 'apple_isp_stop_streaming',
                   'isp_vb2_release_buffers', 'apple_isp_video_suspend',
                   'apple_isp_video_resume']
                  }
        bodies['apple_isp_stop_capture'] = function(
            (DRIVER / 'isp-cam.c').read_text(), 'apple_isp_stop_capture')
        bodies = {**{'apple_isp_stop_capture': bodies.pop('apple_isp_stop_capture')}, **bodies}

        if args.control == 'address':
            bodies['isp_returned_surface'] = bodies['isp_returned_surface'].replace(
                '(u32)buf->surfs[0].iova == iova', 'true')
        elif args.control == 'pool':
            body = bodies['isp_returned_surface']
            bodies['isp_returned_surface'] = body.replace(
                'desc->pool_type == CISP_POOL_TYPE_META', 'false')
        elif args.control == 'duplicate':
            bodies['ipc_bt_handle'] = bodies['ipc_bt_handle'].replace(
                'surfaces[i] == surfaces[j]', 'false')
        elif args.control == 'snapshot':
            body = bodies['ipc_bt_handle']
            body = body.replace('isp_returned_surface(isp, &buffers[i]',
                                'isp_returned_surface(isp, &bl->buffers[i]')
            bodies['ipc_bt_handle'] = body
    if args.suite == 'queue':
        bodies = {n: function(inputs['isp-v4l2.c'], n) for n in
                  ['isp_submit_buffers', 'isp_vb2_release_buffers',
                   'isp_vb2_buf_queue', 'apple_isp_stop_streaming', 'isp_vb2_stop_streaming',
                   'apple_isp_start_streaming', 'apple_isp_video_error']}
        if args.control == 'queue-error':
            bodies['isp_vb2_buf_queue'] = bodies['isp_vb2_buf_queue'].replace(
                'vb2_queue_error(&isp->vbq);', '(void)0;')
    if args.suite == 'receive':
        bodies = {n: function(inputs[f], n) for f, n in [
            ('isp-ipc.c', 'chan_rx_done'), ('isp-ipc.c', 'ipc_chan_handle'),
            ('isp-fw.c', 'apple_isp_isr_thread')]}
        if 'apple_isp_video_error(isp);' in bodies['apple_isp_isr_thread']:
            bodies = {'apple_isp_video_error': function(
                (DRIVER / 'isp-v4l2.c').read_text(), 'apple_isp_video_error'), **bodies}
        if args.control == 'drain':
            bodies['ipc_chan_handle'] = bodies['ipc_chan_handle'].replace(
                'for (u32 i = 0; i < chan->num; i++)', 'while (1)')
    code = (ROOT / ('tools/asahi/isp/' + args.suite + '.c')).read_text()
    if args.suite in ['returns', 'queue']:
        prefix = (ROOT / 'tools/asahi/isp/buffers.c').read_text()
        code = code.replace('/* BUFFER FIXTURE */', prefix.split('/* DRIVER FUNCTIONS */')[0])
    if args.suite in ['buffers', 'returns', 'queue']:
        structures = re.findall(r'^struct isp_buflist(?:_buffer)? \{.*?^} __packed;',
                                inputs['isp-v4l2.c'], re.M | re.S)
        if len(structures) != 2:
            raise RuntimeError('missing buffer list structures')
        code = code.replace('/* DRIVER STRUCTURES */', '\n'.join(structures))
    code = code.replace('/* DRIVER FUNCTIONS */', '\n'.join(bodies.values()))

    if args.emit:
        args.emit.write_text(code)
        return 0

    def run(out):
        out.mkdir(parents=True, exist_ok=True)
        source, binary = out / 'session.c', out / 'session'
        source.write_text(code)
        compile_cmd = [args.cc, '-std=gnu11', '-Wall', '-Wextra', '-Werror',
                       '-Wno-unused-parameter', '-Wno-unused-but-set-variable',
                       '-Wno-sign-compare', '-Wno-unused-function', '-O1', '-g', '-pthread',
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
