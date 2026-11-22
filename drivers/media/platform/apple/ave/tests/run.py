#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Host checks of the actual AVE builders with a fake IPC/DMA boundary.

Only includes and unused kernel-facing declarations are omitted. Function
bodies are compiled unchanged. --baseline accepts an older AVE source dir
for byte comparisons; --build retains the executable for private capture QA.
No firmware, command images, or reference captures are part of these tests.
"""
import argparse
import os
from pathlib import Path
import re
import subprocess
import tempfile


def declaration(text, name):
    match = re.search(r'^(?:struct|enum) ' + name + r' \{', text, re.M)
    end = text.index('\n};', match.start()) + 3
    return text[match.start():end] + '\n'


def function(text, name):
    match = re.search(r'^(?:static )?(?:const )?[\w *]+\b' + name + r'\([^;]+?\n\{', text, re.M)
    end = text.index('\n}', match.end()) + 2
    return text[match.start():end] + '\n'


def source(text):
    return re.sub(r'^#include .*\n', '', text, flags=re.M)


def build(src, out, baseline):
    header = (src / 'ave.h').read_text()
    per_abi = (src / 'ave-writer.c').exists()
    has_layout = (src / 'ave-layout.h').exists()
    code = (Path(__file__).parent / 'shim.h').read_text()
    for name in ('AVE_MAX_SLOTS', 'AVE_MAX_REFS', 'AVE_TILE_BUFS',
                 'AVE_MAX_SESSIONS', 'AVE_FW_TEXT_CRC_9003_78_0',
                 'AVE_6070_LOWRES_RESULTS',
                 'AVE_6070_AVC_ENTROPY_TILES',
                 'AVE_6070_CLIENT_MEM_BYTES', 'AVE_6070_CODED_HEADER_BYTES',
                 'AVE_AVC_FLAT_SCALING_WEIGHT', 'AVE_AVC_RECON_SKIP_MODE',
                 'AVE_AVC_STATIC_AREA_MASK_ALL'):
        code += re.search(r'^#define ' + name + r'\s+[^\n]+', header, re.M)[0] + '\n'
    for name in ('AVE_ENTROPY_SETS', 'AVE_NEIGHBOUR_SETS'):
        match = re.search(r'^#define ' + name + r'\s+[^\n]+', header, re.M)
        if match:
            code += match[0] + '\n'
    # Every firmware build's text CRC and the rate-control modes: ave-layout.c and
    # ave-sess.c name them, and each new build adds one, so take them all.
    defined = set(re.findall(r'^#define (\w+)', code, re.M))
    for m in re.finditer(r'^#define ((?:AVE_FW_TEXT_CRC|AVE_RC)_\w+)\s+[^\n]+', header, re.M):
        if m[1] not in defined:
            code += m[0] + '\n'
    # Later firmware builds; an older baseline does not define them.
    for name in ('AVE_MCTF_BUFS', 'AVE_MCTF_IN_BUFS',
                 'AVE_MCTF_OUT_BUFS', 'AVE_MCTF_POOL_LENT', 'AVE_MCTF_POOL_READY'):
        m = re.search(r'^#define ' + name + r'\s+[^\n]+', header, re.M)
        if m:
            code += m[0] + '\n'
    if (src / 'ave-cmp420.h').exists():
        code += source((src / 'ave-cmp420.h').read_text())
    for name in ('ave_codec', 'ave_buf', 'ave_fw_state', 'ave_image',
                 'ave_coproc_regs', 'ave_variant', 'ave_params', 'ave_slot', 'ave_session'):
        code += declaration(header, name)
    if per_abi:
        code = '#define TEST_PER_ABI 1\n' + code
        code += declaration(header, 'ave_opcode')
        for name in ('AVE_IPC_BYTES', 'AVE_IPC_CMD_OFF', 'AVE_IPC_CMD_BYTES',
                     'AVE_IPC_SCALAR_OFF', 'AVE_IPC_PROC_OFF', 'AVE_IPC_PROC_BYTES'):
            code += re.search(r'^#define ' + name + r'\s+[^\n]+', header, re.M)[0] + '\n'
    if baseline:
        code = '#define TEST_BASELINE 1\n' + code
        code += '\n'.join(re.findall(r'^#define AVE_(?:START|PROCESS)_\w+.*$', header, re.M)) + '\n'
    if has_layout:
        code = '#define TEST_LAYOUT 1\n' + code
        code += source((src / 'ave-layout.h').read_text())
        code += source((src / 'ave-layout.c').read_text())
    code += source((src / 'ave-regs.h').read_text())
    code += source((src / 'ave-sizes.h').read_text())
    code += (Path(__file__).parent / 'boundary.h').read_text()
    code += source((src / 'ave-image.c').read_text())
    if per_abi:
        code += source((src / 'ave-writer.c').read_text())
        code += source((src / 'ave-abi.c').read_text())
    code += source((src / 'ave-cmd.c').read_text())
    session = (src / 'ave-sess.c').read_text()
    code += declaration(session, 'ave_buf_req')
    lines = iter(session.splitlines())
    for line in lines:
        if line.startswith('#define '):
            code += line + '\n'
            while line.endswith('\\'):
                line = next(lines)
                code += line + '\n'
    for name in ('ave_buf_req_cmp', 'ave_req', 'ave_sess_alloc', 'ave_mctf_pool_crc', 'ave_mctf_pool_check', 'ave_mctf_pool_return', 'ave_sess_free_bufs', 'ave_sess_reply'):
        code += function(session, name)
    if not baseline:
        fw = (src / 'ave-fw.c').read_text()
        if per_abi:
            code += function(fw, 'ave_fw_check_boot')
        code += function(fw, 'ave_fw_check_build')
    code += (Path(__file__).parent / 'test.c').read_text()
    out.mkdir(parents=True, exist_ok=True)
    (out / 'host.c').write_text(code)
    subprocess.run([os.environ.get('CC', 'cc'), '-std=gnu11', '-O1', '-g',
                    '-Wall', '-Wextra', '-Werror', '-Wno-unused-parameter',
                    # host.c pastes whole driver files: helpers the tests don't call are fine.
                    '-Wno-unused-function',
                    '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
                    str(out / 'host.c'), '-o', str(out / 'ave-test')], check=True)
    return out / 'ave-test'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path)
    parser.add_argument('--build', type=Path)
    parser.add_argument('--compare-baseline', type=Path)
    args = parser.parse_args()
    src = args.baseline or Path(__file__).resolve().parents[1]
    with tempfile.TemporaryDirectory(prefix='ave-host-') as tmp:
        binary = build(src, args.build or Path(tmp), bool(args.baseline))
        subprocess.run([str(binary)], check=True)
        if args.compare_baseline:
            old = build(args.compare_baseline, Path(tmp) / 'baseline', True)
            before = subprocess.check_output([str(old), '--matrix'])
            after = subprocess.check_output([str(binary), '--matrix'])
            if before != after:
                mismatch = next(i for i, pair in enumerate(zip(before, after)) if pair[0] != pair[1])
                raise AssertionError(f'J700 matrix differs at stream byte {mismatch}')
            print(f'ok: 288 J700 sessions / 1440 commands byte-identical ({len(after)} bytes)')


if __name__ == '__main__':
    main()
