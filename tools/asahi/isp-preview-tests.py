#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Exercise production OF parsing/admission with synthetic device descriptions."""
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
static bool board, soc;
static int cells=3, bytes=12;
static u32 values[5]={26,6,2};
#define ISP_FW_VERSION_MIN_LEN 3
#define ISP_FW_VERSION_MAX_LEN 5
#define dev_warn(...) ((void)0)
static void *of_find_property(void *n,const char *p,int *len) {*len=bytes; return bytes<0?NULL:values;}
static int of_property_read_variable_u32_array(void *n,const char *p,u32 *out,int min,int max) {
 if(cells<min || cells>max) return -EINVAL;
 memcpy(out,values,cells*sizeof(u32));return cells;
}
static bool of_machine_is_compatible(const char *s) {return board;}
static bool of_device_is_compatible(void *n,const char *s) {return soc;}
#define CHECK(x) do {if (!(x)) {fprintf(stderr,"failed line %d\n",__LINE__);return 1;}} while(0)
'''
main = r'''
int main(void) {
 struct device dev={0};
 board=soc=true;
 CHECK(preview_26);
 CHECK(isp_preview26_admitted(&dev,ISP_FIRMWARE_V_26_6_2));
 preview_26=false;
 CHECK(!isp_preview26_admitted(&dev,ISP_FIRMWARE_V_26_6_2));
 for(int flags=0;flags<8;flags++) {
  preview_26=flags&1;board=flags&2;soc=flags&4;
  CHECK(isp_preview26_admitted(&dev,ISP_FIRMWARE_V_26_6_2)==(flags==7));
 }
 preview_26=board=soc=true;
 CHECK(!isp_preview26_admitted(&dev,ISP_FIRMWARE_V_14_7));
 CHECK(!isp_preview26_admitted(&dev,ISP_FIRMWARE_V_UNKNOWN));
 for(int field=0;field<3;field++) {
  values[field]++; CHECK(!isp_preview26_admitted(&dev,ISP_FIRMWARE_V_26_6_2));values[field]--;
 }
 for(int n=-1;n<=6;n++) {
  if(n==3)continue;
  cells=n;bytes=n*4;
  CHECK(!isp_preview26_admitted(&dev,ISP_FIRMWARE_V_26_6_2));
 }
 cells=3;bytes=13;CHECK(!isp_preview26_admitted(&dev,ISP_FIRMWARE_V_26_6_2));
 puts("PASS: production default, opt-out/board/SoC combinations, compat mismatch, version mismatch, missing/truncated/extended cells");
 return 0;
}
'''
default=source[source.index('static bool preview_26'):source.index(';',source.index('static bool preview_26'))+1]
code=preamble+default+'\n'+header[a:b]+'\n'+mod.function(source,'isp_read_fw_version')+mod.function(source,'isp_preview26_admitted')+main
with tempfile.TemporaryDirectory() as d:
 p=Path(d);(p/'test.c').write_text(code)
 subprocess.run(['clang','-std=gnu11','-Werror','-Wno-unused-function','-fsanitize=address,undefined',str(p/'test.c'),'-o',str(p/'test')],check=True)
 subprocess.run([str(p/'test')],check=True)
