#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Execute native26 report admission on synthetic OF/tunable boundaries."""
from pathlib import Path
import os
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
source = (ROOT / 'drivers/pmdomain/apple/pmp-report.c').read_text()
start = source.index('static int apple_pmp_native_records(')
body = source[start:source.index('\n}', start)+2]
validation = re.sub(r'^#include .*\n', '', (ROOT / 'drivers/pmdomain/apple/pmp-report-validation.h').read_text(), flags=re.M)
pre = r'''
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
typedef uint8_t u8; typedef uint32_t u32; typedef uint64_t u64;
#define BIT(x) (1U<<(x))
#define BIT_ULL(x) (1ULL<<(x))
#define __ffs64(x) __builtin_ctzll(x)
static u32 get_unaligned_le32(const void *p) { const u8 *b=p; return (u32)b[0]|(u32)b[1]<<8|(u32)b[2]<<16|(u32)b[3]<<24; }
static u64 get_unaligned_le64(const void *p) { return get_unaligned_le32(p)|(u64)get_unaligned_le32((const u8 *)p+4)<<32; }
static void put32(u32 v,void *p) { u8 *b=p; for(unsigned i=0;i<4;i++) b[i]=v>>(8*i); }
struct device_node { bool available; struct device_node *report; };
struct device { struct device_node *of_node; };
struct apple_pmp_report { struct device *dev; struct device_node *native_pmp; };
static struct device_node chosen, report, suppliers[2];
static bool board=true, native=true, optin=true, baduuid;
static int nodes=1, nt=20*0x7c, ng=8, nr=64;
static u8 table[20*0x7c], group[8], ranges[64];
#define dev_info(...) ((void)0)
#define dev_dbg(...) ((void)0)
#define for_each_compatible_node(np,t,c) for(int it=0;it<nodes && ((np)=&suppliers[it]);it++)
static struct device_node *of_find_node_by_path(const char *p) { (void)p; return &chosen; }
static void of_node_put(struct device_node *p) { (void)p; }
static struct device_node *of_node_get(struct device_node *p) { return p; }
static bool of_machine_is_compatible(const char *c) { (void)c; return board; }
static bool of_device_is_compatible(struct device_node *p,const char *c) { (void)p;(void)c;return native; }
static bool of_device_is_available(struct device_node *p) { return p->available; }
static struct device_node *of_parse_phandle(struct device_node *p,const char *k,int i) { (void)k;(void)i;return p->report; }
static int of_property_read_string(struct device_node *p,const char *k,const char **s) {
 (void)p;(void)k;*s=baduuid?"bad":"EFD60284-7C58-33A9-8522-CB3366AF6040";return 0;
}
static const void *of_get_property(struct device_node *p,const char *k,int *len) {
 (void)p;
 if (!strcmp(k,"asahi,j613-25g83-dcpext")) { *len=2;return optin?"1":NULL; }
 if (!strcmp(k,"apple,tunable-soc-device")) { *len=nt;return table; }
 if (!strcmp(k,"apple,tunable-soc-device-ps-group")) { *len=ng;return group; }
 if (!strcmp(k,"apple,tunable-ptd-range")) { *len=nr;return ranges; }
 return NULL;
}
'''
main = r'''
static void reset(void) {
 board=native=optin=true;baduuid=false;nodes=1;nt=sizeof(table);ng=8;nr=64;
 memset(table,0,sizeof(table));memset(group,0,sizeof(group));memset(ranges,0,sizeof(ranges));
 for(unsigned i=0;i<20;i++) put32(100+i,table+i*0x7c);
 put32(8,table+11*0x7c);put32(9,table+12*0x7c);put32(18,table+19*0x7c);
 put32(2,table+11*0x7c+8);put32(2,table+12*0x7c+8);
 memcpy(table+11*0x7c+0x74,"DISP",5);memcpy(table+12*0x7c+0x74,"DISPEXT",8);
 memcpy(table+19*0x7c+0x74,"ANS",4);put32(0x3fff0,group);
 put32(10,ranges);put32(256,ranges+4);put32(8,ranges+8);
 put32(11,ranges+32);put32(264,ranges+36);put32(8,ranges+40);
 for(unsigned i=0;i<2;i++) { suppliers[i].available=true;suppliers[i].report=&report; }
}
static int admit(void) { struct device d={.of_node=&report};struct apple_pmp_report r={.dev=&d};return apple_pmp_native_records(&r); }
int main(void) {
 reset();assert(!admit());
 reset();optin=false;board=false;assert(!admit()); /* Off leaves the old path intact. */
 reset();native=false;assert(!admit());
 reset();board=false;assert(admit());
 reset();nodes=0;assert(admit());
 reset();nodes=2;assert(admit());
 reset();baduuid=true;assert(admit());
 reset();suppliers[0].available=false;assert(admit());
 reset();nt--;assert(admit());
 reset();ng--;assert(admit());
 reset();nr--;assert(admit());
 reset();group[0]^=1;assert(admit());
 for(unsigned i=0;i<20;i++) {
  if(i==11 || i==12 || i==19) continue;
  reset();put32(8,table+i*0x7c);assert(admit());
 }
 const unsigned positions[]={11,12,19};
 for(unsigned i=0;i<3;i++) {
  reset();table[positions[i]*0x7c+0x74]^=1;assert(admit());
  reset();table[positions[i]*0x7c+8]^=2;assert(admit());
 }
 reset();put32(257,ranges+4);assert(admit());
 puts("PASS native26 PTD personality, unique ordinals, acknowledgment and range refusals");
 return 0;
}
'''
with tempfile.TemporaryDirectory() as tmp:
    p = Path(tmp) / 'native.c'
    p.write_text(pre + validation + body + main)
    binary = p.with_suffix('')
    subprocess.run([os.environ.get('CC', 'cc'), '-std=gnu11', '-Wall', '-Wextra', '-Werror',
                    '-Wno-unused-function', '-O2', str(p), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
