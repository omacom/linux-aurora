#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Exercise the production firmware version parser with synthetic device descriptions."""
import importlib.util
from pathlib import Path
import subprocess, sys, tempfile
root = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('isp', root/'tools/asahi/isp-tests.py')
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)
source = (root/'drivers/media/platform/apple/isp/isp-drv.c').read_text()
header = (root/'drivers/media/platform/apple/isp/isp-drv.h').read_text()
a=header.index('enum isp_firmware_version {'); b=header.index('};',a)+2
preamble = r'''
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
typedef uint32_t u32;
struct device {void *of_node;};
static int cells=3, bytes=12;
static u32 values[5]={26,6,2};
#define ISP_FW_VERSION_MIN_LEN 3
#define ISP_FW_VERSION_MAX_LEN 5
#define dev_warn(...) ((void)0)
#define dev_info(...) ((void)0)
static void *of_find_property(void *n,const char *p,int *len) {*len=bytes; return bytes<0?NULL:values;}
static int of_property_read_variable_u32_array(void *n,const char *p,u32 *out,int min,int max) {
 if(cells<min || cells>max) return -EINVAL;
 memcpy(out,values,cells*sizeof(u32));return cells;
}
#define CHECK(x) do {if (!(x)) {fprintf(stderr,"failed line %d\n",__LINE__);return 1;}} while(0)
'''
NEWER_CASES = r'''
 CHECK(ver(27,2,0)==ISP_FIRMWARE_V_26_6_2);
 CHECK(ver(26,6,3)==ISP_FIRMWARE_V_UNKNOWN);
 CHECK(ver(26,7,0)==ISP_FIRMWARE_V_UNKNOWN);
 CHECK(ver(27,0,0)==ISP_FIRMWARE_V_UNKNOWN);
 CHECK(ver(27,2,1)==ISP_FIRMWARE_V_UNKNOWN);
 CHECK(ver(30,0,0)==ISP_FIRMWARE_V_UNKNOWN);
 CHECK(ver(26,5,9)==ISP_FIRMWARE_V_UNKNOWN);
'''
main = r'''
static int ver(u32 a,u32 b,u32 c) {
 struct device dev={0};
 cells=3;bytes=12;values[0]=a;values[1]=b;values[2]=c;
 return isp_read_fw_version(&dev,"apple,firmware-compat");
}
int main(void) {
 struct device dev={0};
 CHECK(ver(12,3,0)==ISP_FIRMWARE_V_12_3);
 CHECK(ver(12,3,1)==ISP_FIRMWARE_V_12_3);
 CHECK(ver(12,3,2)==ISP_FIRMWARE_V_UNKNOWN);
 CHECK(ver(12,4,0)==ISP_FIRMWARE_V_12_4);
 CHECK(ver(13,5,0)==ISP_FIRMWARE_V_13_5);
 CHECK(ver(14,7,0)==ISP_FIRMWARE_V_14_7);
 CHECK(ver(26,6,2)==ISP_FIRMWARE_V_26_6_2);
 CHECK(ver(14,6,0)==ISP_FIRMWARE_V_UNKNOWN);
 CHECK(ver(26,6,1)==ISP_FIRMWARE_V_UNKNOWN);
 CHECK(ver(11,0,0)==ISP_FIRMWARE_V_UNKNOWN);
NEWER_CASES
 values[0]=26;values[1]=6;values[2]=2;
 for(int n=-1;n<=6;n++) {
  if(n==3)continue;
  cells=n;bytes=n*4;
  CHECK(isp_read_fw_version(&dev,"apple,firmware-compat")==ISP_FIRMWARE_V_UNKNOWN);
 }
 cells=3;bytes=13;CHECK(isp_read_fw_version(&dev,"apple,firmware-compat")==ISP_FIRMWARE_V_UNKNOWN);
 puts("PASS: known versions, 27.2.0 -> 26.6.2 layout, other newer and older versions unknown, missing/truncated/extended cells");
 return 0;
}
'''
code=preamble+header[a:b]+'\n'+mod.function(source,'isp_read_fw_version')+main.replace('NEWER_CASES', NEWER_CASES)
with tempfile.TemporaryDirectory() as d:
 p=Path(d);(p/'test.c').write_text(code)
 subprocess.run(['clang','-std=gnu11','-Werror','-Wno-unused-function','-fsanitize=address,undefined',str(p/'test.c'),'-o',str(p/'test')],check=True)
 subprocess.run([str(p/'test')],check=True)
