#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Run the production dcp_get_bw_scratch_in_window() over edge-case ranges.

The realtime bandwidth slot named by apple,bw-scratch is 4 bytes at an offset
into a PMGR range and must lie inside the DCP's display window. Ranges shorter
than 4 bytes, offsets past the end and windows that end at the top of the
address space must be refused or accepted without unsigned wrap-around.

--mutation names a change to the production source that the matrix must kill.
"""
import argparse
import os
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(os.environ.get('DCP_ROOT', Path(__file__).resolve().parents[2]))

MUTATIONS = {
    # The check before the fix: underflows for a slot range under 4 bytes.
    'old-check': ('if (resource_size(&slot) < 4 || offset > resource_size(&slot) - 4 ||\n\t    resource_size(win) < 4) {',
                  'if (offset > resource_size(&slot) - 4) {'),
    # Accept a slot that only starts inside the window.
    'start-only': ('addr - win->start > resource_size(win) - 4', 'addr > win->end'),
}


def item(text, signature):
    start = text.index(signature)
    begin = text.index('{', start)
    depth, end = 1, begin + 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[start:end]


HARNESS = r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
typedef uint64_t u64; typedef uint32_t u32; typedef u64 resource_size_t;
#define EINVAL 22
struct resource { resource_size_t start, end; };
static resource_size_t resource_size(const struct resource *r) { return r->end - r->start + 1; }
struct device_node { struct resource r; };
struct of_phandle_args { struct device_node *np; };
struct apple_dcp { void *dev; struct resource *disp_registers[8];
	struct resource disp_bw_scratch_res; u32 disp_bw_scratch_index; u64 disp_bw_scratch_offset; };
static int of_address_to_resource(struct device_node *np, u32 i, struct resource *r) { (void)i; *r = np->r; return 0; }
#define dev_err(d, ...) ((void)(d))
FN
static int run(u64 ss, u64 se, u32 off, u64 ws, u64 we, u64 *got)
{
	struct device_node np = { { ss, se } };
	struct of_phandle_args a = { &np };
	struct resource w = { ws, we };
	struct apple_dcp d = { 0 };
	int r;
	d.disp_registers[4] = &w;
	r = dcp_get_bw_scratch_in_window(&d, &a, 0, 4, off);
	if (!r) *got = d.disp_bw_scratch_offset;
	return r;
}
int main(void)
{
	u64 o = 0;
	/* J615: PMGR disp-5 range 0x2d03d0000+0x4000, slot 0x908, window = that range. */
	assert(!run(0x2d03d0000, 0x2d03d3fff, 0x908, 0x2d03d0000, 0x2d03d3fff, &o) && o == 0x908);
	/* Last 4 bytes of the range and window: accepted. */
	assert(!run(0x1000, 0x1fff, 0xffc, 0x1000, 0x1fff, &o) && o == 0xffc);
	/* One byte past: refused. */
	assert(run(0x1000, 0x1fff, 0xffd, 0x1000, 0x1fff, &o) == -EINVAL);
	/* A slot range of 2 bytes: refused (the old check underflowed and took any offset). */
	assert(run(0x1000, 0x1001, 0x0, 0x0, 0xffffffff, &o) == -EINVAL);
	assert(run(0x1000, 0x1001, 0x100, 0x0, 0xffffffff, &o) == -EINVAL);
	/* A window of 2 bytes: refused. */
	assert(run(0x1000, 0x1fff, 0x0, 0x1000, 0x1001, &o) == -EINVAL);
	/* Slot below or straddling the window end: refused. */
	assert(run(0x1000, 0x1fff, 0x0, 0x1004, 0x1fff, &o) == -EINVAL);
	assert(run(0x1000, 0x1fff, 0x1e, 0x1000, 0x101f, &o) == -EINVAL);
	assert(!run(0x1000, 0x1fff, 0x1c, 0x1000, 0x101f, &o) && o == 0x1c);
	/* A window ending at the top of the address space: no wrap. */
	assert(!run(0xfffffffffffff000ull, 0xffffffffffffffffull, 0xffc,
		    0xfffffffffffff000ull, 0xffffffffffffffffull, &o) && o == 0xffc);
	puts("PASS apple,bw-scratch slot inside its range and the display window, no unsigned wrap");
	return 0;
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mutation', choices=sorted(MUTATIONS))
    args = parser.parse_args()
    src = (ROOT / 'drivers/gpu/drm/apple/dcp.c').read_text()
    fn = item(src, 'static int dcp_get_bw_scratch_in_window(')
    if args.mutation:
        old, new = MUTATIONS[args.mutation]
        assert fn.count(old) == 1, args.mutation
        fn = fn.replace(old, new)
    with tempfile.TemporaryDirectory() as tmp:
        c = Path(tmp) / 't.c'
        c.write_text(HARNESS.replace('FN', fn))
        exe = Path(tmp) / 't'
        subprocess.run([os.environ.get('CC', 'cc'), '-std=gnu11', '-Wall', '-Werror', '-Wno-unused-parameter',
                        '-fsanitize=address,undefined', '-o', str(exe), str(c)], check=True)
        res = subprocess.run([str(exe)], capture_output=True, text=True)
        sys.stdout.write(res.stdout)
        sys.stderr.write(res.stderr)
        return res.returncode


if __name__ == '__main__':
    sys.exit(main())
