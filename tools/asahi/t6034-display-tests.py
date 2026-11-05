#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Run production J514M display selection and opt-out policy on the host."""
from pathlib import Path
import subprocess
import tempfile
ROOT = Path(__file__).resolve().parents[2]
gate = (ROOT / 'drivers/soc/apple/t6030-display-gate.c').read_text()
driver = (ROOT / 'drivers/gpu/drm/apple/iomfb_v14_7.c').read_text()
header = (ROOT / 'drivers/gpu/drm/apple/iomfb_v14_7_board.h').read_text()
def item(s, marker):
    a=s.index(marker);b=s.index('\n}',a)+2
    return s[a:b]
prefix = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#define __init
#define ARRAY_SIZE(a) (sizeof(a)/sizeof((a)[0]))
#define A(n) (n)
typedef uint32_t u32;
static const char *board, *soc, *dcp;
static bool of_machine_is_compatible(const char *c)
{ return (board && !strcmp(board,c)) || (soc && !strcmp(soc,c)); }
static bool machine_is(const void *ctx,const char *c) { return of_machine_is_compatible(c); }
static bool dcp_is(const void *ctx,const char *c) { return dcp && !strcmp(dcp,c); }
'''
types=header[header.index('struct dcp_v14_board {'):header.index('#endif')]
defines='\n'.join(l for l in driver.splitlines() if l.startswith('#define DCP_V14_') and 'FIRMWARE_UUID' in l)
records=driver[driver.index('static const struct dcp_v14_board dcp_v14_board_t6030'):driver.index('struct dcp_v14_property {')]
functions='\n'.join([item(gate,'static bool __init gate_t6031_wanted('),
 item(gate,'static bool __init gate_t6034_default_board('),item(gate,'static bool __init gate_t6034_wanted('),
 item(driver,'const struct dcp_v14_board *dcp_v14_board_select('),item(driver,'bool dcp_v14_board_admits(')])
main = r'''
int main(void)
{
 const char *boards[]={"apple,j514m","apple,j516m","apple,j514c","apple,j999"};
 const char *socs[]={"apple,t6034","apple,t6031","apple,t6030",NULL};
 dcp="apple,t6034-dcp";
 for(unsigned i=0;i<ARRAY_SIZE(boards);i++) for(unsigned j=0;j<ARRAY_SIZE(socs);j++) {
  board=boards[i];soc=socs[j];bool expected=i==0&&j==0,known;
  assert(gate_t6034_default_board()==expected);
  assert(gate_t6034_wanted(-1)==expected);
  assert(!gate_t6034_wanted(0));
  assert(gate_t6034_wanted(1)==expected);
  const struct dcp_v14_board *b=dcp_v14_board_select(dcp_is,machine_is,NULL,&known);
  assert(known && (!!b)==expected);
  if(!b)continue;
  assert(!strcmp(b->handoff,"apple,t6034-handoff"));
  assert(b->promotion && b->log_bringup && !b->ctm_set && !b->ctm_get && !b->panel_nits);
  assert(dcp_v14_board_admits(b,"F493F03E-CC40-3D89-9A08-5F259BA2B246"));
  assert(!dcp_v14_board_admits(b,"F493F03E-CC40-3D89-9A08-5F259BA2B247"));
  assert(!dcp_v14_board_admits(b,"f493f03e-cc40-3d89-9a08-5f259ba2b246"));
  assert(!dcp_v14_board_admits(b,NULL));
 }
 board="apple,j514m";soc="apple,t6034";dcp="apple,t6031-dcp";bool known;
 assert(!dcp_v14_board_select(dcp_is,machine_is,NULL,&known));
 return 0;
}
'''
source="\n".join([prefix,types,defines,records,functions,main])
mutants={
 'control':source,
 'lost-board-soc':source.replace('.soc = "apple,t6034",','.soc = NULL,'),
 'lost-owner-optout':source.replace('return sw < 0 ? default_board : sw;','return sw < 0 ? default_board : true;'),
 'lost-firmware-identity':source.replace('return uuid && !strcmp(uuid, board->firmware_uuid);','return uuid != NULL;'),
}
with tempfile.TemporaryDirectory(prefix='t6034-display-') as directory:
 for name,c in mutants.items():
  exe=Path(directory)/name
  subprocess.run(['cc','-std=gnu11','-Wall','-Wextra','-Werror','-Wno-unused-parameter','-x','c','-','-o',str(exe)],input=c,text=True,check=True)
  result=subprocess.run([str(exe)],capture_output=True)
  assert (result.returncode==0)==(name=='control'),(name,result.stderr)
  print(name+': '+('pass' if name=='control' else 'caught'))
assert 'gate_t6034_requested = gate_t6034_wanted(gate_t6034_switch);' in gate
print('T6034 normal display defaults, owner disable, exact root pair and firmware identity: PASS')

# Compile the production report layout records and OF match table. Matching
# T6034 through its generic T6020 fallback binds a report that never creates
# the halted-PMP startup owner; wait_ready() then correctly returns -ENODEV.
pmp = (ROOT / 'drivers/pmdomain/apple/pmp-report.c').read_text()
pmp_type = item(pmp, 'struct apple_pmp_report_offsets {') + ';'
pmp_tables = pmp[pmp.index('static const struct apple_pmp_report_offsets apple_pmp_offsets_t600x = {'):pmp.index('static struct platform_driver apple_pmp_report_driver = {')]
pmp_main = r'''
static const struct apple_pmp_report_offsets *report(const char *specific)
{
 const char *compatibles[]={specific,"apple,t6020-pmp-v2-report"};
 for(unsigned i=0;i<ARRAY_SIZE(compatibles);i++)
  for(unsigned j=0;apple_pmp_report_of_match[j].compatible;j++)
   if(!strcmp(compatibles[i],apple_pmp_report_of_match[j].compatible))
    return apple_pmp_report_of_match[j].data;
 return NULL;
}
int main(void)
{
 const struct apple_pmp_report_offsets *o=report("apple,t6034-pmp-v2-report");
 assert(o && o->starts_pmp);
 assert(!strcmp(o->name,"T6034"));
 assert(!strcmp(o->fallback,"apple_t6034_display.enable=0"));
 assert(!strcmp(o->pwrstate,"apple,t6031-pmgr-pwrstate"));
 assert(o->tgt_read==0x2000 && o->tgt_write==0x11000 && o->actual==0x2080 && o->status==0x10);
 assert(o->devices_ordered);
 o=report("apple,t6031-pmp-v2-report");
 assert(o && o->starts_pmp && !strcmp(o->fallback,"apple_t6031_display.enable=0"));
 o=report("apple,t6020-pmp-v2-report");
 assert(o && !o->starts_pmp);
 return 0;
}
'''
pmp_source='\n'.join([prefix,'#define IS_ENABLED(x) 0',pmp_type,
 'struct of_device_id {const char *compatible; const struct apple_pmp_report_offsets *data;};',pmp_tables,pmp_main])
pmp_mutants={
 'control':pmp_source,
 'missing-t6034-match':pmp_source.replace('.compatible = "apple,t6034-pmp-v2-report"','.compatible = "apple,missing-t6034-report"'),
 'lost-halted-pmp-owner':pmp_source.replace('.starts_pmp = true,\n\t.name = "T6034"','.starts_pmp = false,\n\t.name = "T6034"'),
 'wrong-ack-aperture':pmp_source.replace('.actual = 0x2080,','.actual = 0x2088,'),
}
with tempfile.TemporaryDirectory(prefix='t6034-pmp-report-') as directory:
 for name,c in pmp_mutants.items():
  exe=Path(directory)/name
  subprocess.run(['cc','-std=gnu11','-Wall','-Wextra','-Werror','-Wno-unused-parameter','-Wno-unused-function','-Wno-unused-variable','-x','c','-','-o',str(exe)],input=c,text=True,check=True)
  result=subprocess.run([str(exe)],capture_output=True)
  assert (result.returncode==0)==(name=='control'),(name,result.stderr)
  print('PMP '+name+': '+('pass' if name=='control' else 'caught'))
print('T6034 exact PMP report binding, halted owner, offsets and explicit fallback: PASS')
