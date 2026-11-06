#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Observe the production capture configuration through fake command boundaries."""
import importlib.util
from pathlib import Path
import re, subprocess, tempfile
root=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('isp',root/'tools/asahi/isp-tests.py')
mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
body=mod.function((root/'drivers/media/platform/apple/isp/isp-cam.c').read_text(),'isp_ch_configure_capture')
names=sorted(set(re.findall(r'\b(isp_(?:cmd_\w+|ch_configure_frame_rate|ch_configure_pools|ch_load_setfile))\(',body)))
code='''
#include <stdbool.h>
#include <stdio.h>
#include <errno.h>
typedef unsigned int u32;
struct coord {int x,y;};
struct preset {int index;struct coord crop_offset,crop_size,output_dim;};
struct isp_format {struct preset *preset;int strides[2];};
struct hw {int fw_abi;bool lpdp;};
struct apple_isp {struct hw *hw;bool preview_26;int temporal_filter;};
static struct preset preset;
static struct isp_format fmt={.preset=&preset};
static struct isp_format *isp_get_format(struct apple_isp *i,u32 ch){return &fmt;}
#define ISP_FW_ABI_H17 1
#define ISP_MBNR_MODE_DISABLE 0
#define ISP_MBNR_MODE_ENABLE 1
#define CHECK(x) do {if(!(x)){fprintf(stderr,"failed %d\\n",__LINE__);return 1;}}while(0)
static int mbnr_mode,temporal_start,history,enable,disable,pools,fail_disable;
'''
for constant in sorted(set(re.findall(r'\bCISP_\w+',body))):code+=f'#define {constant} 0\n'
for name in names:
 if name=='isp_cmd_ch_mbnr_enable':
  code+='static int '+name+'(struct apple_isp *i,u32 ch,int a,int mode,int b){mbnr_mode=mode;return 0;}\n';continue
 effect={'isp_cmd_apple_ch_temporal_filter_start':'temporal_start++;','isp_cmd_apple_ch_motion_history_start':'history++;','isp_cmd_apple_ch_temporal_filter_enable':'enable++;','isp_cmd_apple_ch_temporal_filter_disable':'disable++;if(fail_disable)return -EIO;','isp_ch_configure_pools':'pools++;'}.get(name,'')
 code+='static int '+name+'(struct apple_isp *i,...){'+effect+'return 0;}\n'
code+=body+r'''
int main(void){
 struct hw hw={0};struct apple_isp isp={.hw=&hw};
 for(int profile=0;profile<3;profile++){
  temporal_start=history=enable=disable=pools=0;mbnr_mode=-1;
  isp.preview_26=profile==1;hw.fw_abi=profile==2;
  CHECK(!isp_ch_configure_capture(&isp,0));
  CHECK(mbnr_mode==(profile==1?0:1));
  CHECK(disable==(profile==1));
  CHECK(temporal_start==(profile!=1));
  CHECK(history==(profile!=1));CHECK(enable==(profile!=1));CHECK(pools==1);
 }
 isp.preview_26=true;hw.fw_abi=0;pools=0;fail_disable=1;
 CHECK(isp_ch_configure_capture(&isp,0)==-EIO);CHECK(pools==0);
 puts("PASS: explicit MBNR disable, no temporal/history startup, legacy/H17 unchanged, disable failure propagation");return 0;
}
'''
with tempfile.TemporaryDirectory() as d:
 p=Path(d);(p/'test.c').write_text(code)
 subprocess.run(['clang','-std=gnu11','-Werror','-fsanitize=address,undefined',str(p/'test.c'),'-o',str(p/'test')],check=True)
 subprocess.run([str(p/'test')],check=True)
