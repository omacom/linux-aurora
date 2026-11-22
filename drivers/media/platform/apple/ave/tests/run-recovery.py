#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Exercise production AVE recovery, PM and queue callbacks at a fake boundary."""
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile

# python3 -I leaves the script directory off sys.path; run.py sits next to this file.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from run import declaration, function, source


def main():
    here = Path(__file__).resolve().parent
    src = here.parent
    header = (src / 'ave.h').read_text()
    code = (here / 'shim.h').read_text()
    code += '\ntypedef uintptr_t phys_addr_t;\n'
    code += '#define HZ 1000\n#define time_before(a, b) ((long)((a) - (b)) < 0)\n'
    code += source((src / 'ave-recovery.h').read_text())
    for name in ('AVE_MAX_SLOTS', 'AVE_MAX_REFS', 'AVE_TILE_BUFS',
                 'AVE_ENTROPY_SETS', 'AVE_NEIGHBOUR_SETS', 'AVE_MAX_SESSIONS',
                 'AVE_MCTF_BUFS', 'AVE_MCTF_IN_BUFS', 'AVE_MCTF_OUT_BUFS',
                 'AVE_MCTF_POOL_LENT', 'AVE_MCTF_POOL_READY'):
        code += re.search(r'^#define ' + name + r'\s+[^\n]+', header, re.M)[0] + '\n'
    code += source((src / 'ave-regs.h').read_text())
    code += 'struct vb2_v4l2_buffer;\n'
    code += source((src / 'ave-cmp420.h').read_text())
    for name in ('ave_codec', 'ave_buf', 'ave_fw_segment', 'ave_ring', 'ave_fw_state',
                 'ave_coproc_regs', 'ave_variant', 'ave_params', 'ave_slot', 'ave_session'):
        code += declaration(header, name)
    code += declaration(header, 'ave_image')
    code += declaration(header, 'ave_opcode')
    code += source((src / 'ave-layout.h').read_text())
    boundary = (here / 'recovery-boundary.h').read_text()
    marker = 'static void *test_ctrl_addr;'
    boundary = boundary.replace(marker, (here / 'vb2-boundary.h').read_text() + '\n' + marker)
    code += boundary
    code += source((src / 'ave-vb2.c').read_text())
    code += (here / 'command-boundary.h').read_text()
    groups = {
        'ave-fw.c': ('ave_fw_vcbar', 'ave_fw_lost', 'ave_fw_mark_broken', 'ave_fw_halt', 'ave_fw_stop_hung', 'ave_fw_shutdown',
                     'ave_fw_runtime_suspend', 'ave_fw_prepare_restart', 'ave_fw_restore_data'),
        'ave-sess.c': ('ave_mctf_pool_crc', 'ave_mctf_pool_check', 'ave_mctf_pool_return', 'ave_sess_domain_unsafe', 'ave_sess_free_bufs', 'ave_sess_destroy', 'ave_sess_reap'),
        'ave-v4l2.c': ('ave_return_bufs', 'ave_buf_queue', 'ave_session_index',
                       'ave_session_count', 'ave_session_close', 'ave_session_free',
                       'ave_session_end', 'ave_stop_streaming',
                       'ave_reset_drain', 'ave_job_ready', 'ave_job_fail', 'ave_poll_work'),
        'ave-ipc.c': ('ave_ipc_pump',),
        'ave-cmd.c': ('ave_cmd_start', 'ave_cmd_process'),
        'ave-drv.c': ('ave_runtime_resume', 'ave_runtime_suspend', 'ave_suspend', 'ave_resume', 'ave_inject_wedge'),
    }
    code += '#define AVE_IRQ_RECHECK_MS 100\n#define AVE_WATCH_MS 25\n'
    for filename, names in groups.items():
        text = (src / filename).read_text()
        for name in names:
            code += function(text, name)
    code += (here / 'recovery-test.c').read_text()
    with tempfile.TemporaryDirectory(prefix='ave-recovery-') as tmp:
        path = Path(tmp)
        (path / 'test.c').write_text(code)
        subprocess.run([os.environ.get('CC', 'cc'), '-std=gnu11', '-O1', '-g',
                        '-Wall', '-Wextra', '-Werror', '-Wno-unused-parameter',
                        '-Wno-unused-function', '-fsanitize=address,undefined',
                        '-fno-omit-frame-pointer', str(path / 'test.c'),
                        '-o', str(path / 'test')], check=True)
        subprocess.run([str(path / 'test')], check=True)


if __name__ == '__main__':
    main()
