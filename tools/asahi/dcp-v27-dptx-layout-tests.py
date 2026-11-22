#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Where a firmware-27 dcpext opens its DPTX controller and ports.

Builds the production dcp_v27_dptx_layout() and dptxep_init()'s endpoint
choice, and runs them over firmware, processor role, advertised endpoints and
hardware data. A firmware-27 dcpext whose firmware advertises both
DPTX_CONTROLLER_ENDPOINT (0x24) and DPTX_ENDPOINT (0x2a) gets the controller
and its ports there (J413 and J615 on 27.2); hardware data that names an
endpoint keeps it (T8142, T6050 and firmware-27 T6041 dcpexts); every other
processor is unchanged. The selector must not test a compatible.

--mutation names a change to the production source that the matrix must kill.
"""
import argparse
import re
import subprocess
import sys
import tempfile
from pathlib import Path

MUTATIONS = {
    # Select on firmware 27 alone, without the advertisement.
    'ignore-advertisement': ('\tif (!apple_rtkit_has_endpoint(dcp->rtk, DPTX_CONTROLLER_ENDPOINT) ||\n'
                             '\t    !apple_rtkit_has_endpoint(dcp->rtk, DPTX_ENDPOINT))\n\t\treturn;\n', ''),
    # Override hardware data that already names an endpoint.
    'ignore-pinned': (' ||\n\t    dcp->hw.dptx_endpoint)\n', ')\n'),
    # Apply to internal processors too.
    'ignore-dcpext': (' || !dcp->dcpext', ''),
}

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--mutation', choices=sorted(MUTATIONS))
args = parser.parse_args()

root = Path(__file__).resolve().parents[2]
apple = root / 'drivers/gpu/drm/apple'
dcp_c = (apple / 'dcp.c').read_text()
dptx_c = (apple / 'dptxep.c').read_text()


def function(source, marker):
    start = source.index(marker)
    opening = source.index('{', start)
    end, depth = opening + 1, 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


def table(name):
    return function(dcp_c, f'static const struct apple_dcp_hw_data {name} = ')


layout = function(dcp_c, 'static void dcp_v27_dptx_layout(')
assert 'of_device_is_compatible' not in layout and 'of_node' not in layout, \
    'dcp_v27_dptx_layout() must key on the firmware, not on a compatible'
# It runs in dcp_start() once RTKit is awake, before the DPTX endpoints open.
start = function(dcp_c, 'int dcp_start(struct platform_device *pdev)')
assert start.index('dcp_v27_dptx_layout(dcp);') < start.index('dptx_controller_init(dcp)') \
    < start.index('dptxep_init(dcp)')
assert 'dcp_v27_dptx_layout(dcp)' not in function(dcp_c, 'static int dcp_platform_probe_pipe(')

# The DPTX endpoint dptxep_init() opens, from its own expression.
init = function(dptx_c, 'int dptxep_init(struct apple_dcp *dcp)')
choice = re.search(r'ep = afk_init\(dcp, (.*?),\s*dptxep_ops\);', init, re.S).group(1)
wire = re.search(r'bool wire_v2 = (.*?);', init, re.S).group(1)

# Hardware data that names an endpoint: T8142's (0x2a), T6050's (0x2c).
pinned = {'apple_dcp_hw_t8142_dcpext': 'DPTX_ENDPOINT', 'apple_dcp_hw_t6050_dcpext': 'DPTX_ENDPOINT_V2'}
for name, ep in pinned.items():
    assert f'.dptx_endpoint = {ep},' in table(name), name
t6041 = function(dcp_c, 'static int dcp_platform_probe_pipe(')
t6041 = t6041[t6041.index('"apple,t6041-dcpext")) {'):]
t6041 = t6041[:t6041.index('\n\t}\n')]
assert 'dcp->hw.dptx_endpoint = DPTX_ENDPOINT_V2;' in t6041

if args.mutation:
    old, new = MUTATIONS[args.mutation]
    assert layout.count(old) == 1, args.mutation
    layout = layout.replace(old, new)

program = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
typedef uint8_t u8;
typedef uint32_t u32;
enum { DCP_FIRMWARE_V_13_5 = 1, DCP_FIRMWARE_H17P, DCP_FIRMWARE_V_14_7, DCP_FIRMWARE_V_27_0 };
enum { DCP_IOMFB_METHODS_DEFAULT, DCP_IOMFB_METHODS_H17P_EXTERNAL };
#define DPTX_CONTROLLER_ENDPOINT 0x24
#define DPTX_ENDPOINT 0x2a
#define DPTX_ENDPOINT_V2 0x2c
struct apple_dcp { int fw_compat; bool dcpext, ext26; void *rtk, *dev;
 struct { u8 dptx_endpoint; bool dptx_controller; int iomfb_method_profile; } hw; };
static u32 advertised;
static bool apple_rtkit_has_endpoint(void *rtk, int ep) { return advertised & (1u << (ep - 0x20)); }
static bool dcp_is_ext26(struct apple_dcp *dcp) { return dcp->ext26; }
static int info_lines;
#define dev_info(...) (info_lines++)
''' + layout + r'''
static u8 dptx_ep(struct apple_dcp *dcp)
{
	bool wire_v2 = ''' + wire + r''';
	return ''' + choice + r''';
}
#define BIT(n) (1u << ((n) - 0x20))
int main(void)
{
	/* J413's and J615's 27.2 dcpext advertise 0x20-0x2c, 0x31 and 0x37. */
	const u32 v27_ext = 0x1fff | BIT(0x31) | BIT(0x37);
	const u32 maps[] = { 0, v27_ext, v27_ext & ~BIT(0x2a), v27_ext & ~BIT(0x24),
			     BIT(0x24) | BIT(0x2a), BIT(0x2c) };
	const int fws[] = { DCP_FIRMWARE_V_13_5, DCP_FIRMWARE_H17P, DCP_FIRMWARE_V_14_7,
			    DCP_FIRMWARE_V_27_0 };
	const u8 pins[] = { 0, DPTX_ENDPOINT, DPTX_ENDPOINT_V2 };
	unsigned cases = 0;

	for (unsigned f = 0; f < 4; f++)
	for (unsigned ext = 0; ext < 2; ext++)
	for (unsigned m = 0; m < 6; m++)
	for (unsigned p = 0; p < 3; p++)
	for (unsigned c = 0; c < 2; c++) {
		struct apple_dcp d = { .fw_compat = fws[f], .dcpext = ext,
				       .hw.dptx_endpoint = pins[p], .hw.dptx_controller = c };
		struct apple_dcp before = d;
		bool both = (maps[m] & BIT(0x24)) && (maps[m] & BIT(0x2a));
		bool chosen = fws[f] == DCP_FIRMWARE_V_27_0 && ext && !pins[p] && both;

		advertised = maps[m];
		info_lines = 0;
		dcp_v27_dptx_layout(&d);
		if (chosen) {
			assert(d.hw.dptx_endpoint == DPTX_ENDPOINT && d.hw.dptx_controller);
			assert(dptx_ep(&d) == DPTX_ENDPOINT && info_lines == 1);
		} else {
			/* Nothing else moves: the endpoint and the controller are as before. */
			assert(d.hw.dptx_endpoint == before.hw.dptx_endpoint &&
			       d.hw.dptx_controller == before.hw.dptx_controller && !info_lines);
			assert(dptx_ep(&d) == dptx_ep(&before));
		}
		/* Idempotent: a second start (after a restart) decides the same. */
		before = d;
		dcp_v27_dptx_layout(&d);
		assert(d.hw.dptx_endpoint == before.hw.dptx_endpoint &&
		       d.hw.dptx_controller == before.hw.dptx_controller);
		cases++;
	}

	/* A firmware-27 dcpext that advertises 0x2c only keeps the default 0x2c. */
	struct apple_dcp d = { .fw_compat = DCP_FIRMWARE_V_27_0, .dcpext = true };
	advertised = BIT(0x2c);
	dcp_v27_dptx_layout(&d);
	assert(dptx_ep(&d) == DPTX_ENDPOINT_V2 && !d.hw.dptx_controller);
	/* The pinned T6050 layout stays 0x2c even where 0x24 and 0x2a are advertised. */
	d = (struct apple_dcp){ .fw_compat = DCP_FIRMWARE_V_27_0, .dcpext = true,
				.hw.dptx_endpoint = DPTX_ENDPOINT_V2 };
	advertised = v27_ext;
	dcp_v27_dptx_layout(&d);
	assert(dptx_ep(&d) == DPTX_ENDPOINT_V2 && !d.hw.dptx_controller);
	printf("%u cases\n", cases);
	return 0;
}
'''

with tempfile.TemporaryDirectory() as tmp:
    src = Path(tmp) / 'test.c'
    src.write_text(program)
    exe = src.with_suffix('')
    subprocess.run(['cc', '-Werror', '-Wall', '-Wno-unused-function', '-fsanitize=address,undefined',
                    str(src), '-o', str(exe)], check=True)
    # A failed assertion is the matrix's verdict, reported without a traceback.
    run = subprocess.run([str(exe)])
    if run.returncode:
        sys.exit(f'FAIL: the layout matrix exited {run.returncode}')
print('PASS firmware-27 dcpext DPTX layout from its advertised endpoints; pinned and other processors unchanged')
