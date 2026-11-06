#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Native26 callback/startup/readiness helpers with a synthetic peer boundary."""
from pathlib import Path
import os
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
source = (ROOT / 'drivers/gpu/drm/apple/iomfb_h17p.c').read_text()
header = (ROOT / 'drivers/gpu/drm/apple/iomfb.h').read_text()


def function(name):
    start = source.index('static ', source.index(name) - 40)
    # Find the signature's start without relying on the return type's line wrapping.
    pos = source.index(name + '(')
    start = source.rfind('static ', 0, pos)
    return source[start:source.index('\n}', pos)+2]


def block(text, name):
    start = text.index(name)
    return text[start:text.index('\n};', start)+3]


pre = r'''
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
typedef uint8_t u8; typedef uint32_t u32; typedef uint64_t u64;
#define GFP_KERNEL 0
#define kzalloc_obj(v) calloc(1,sizeof(v))
#define kfree free
#define READ_ONCE(v) (v)
#define WRITE_ONCE(v,x) ((v)=(x))
#define dev_err(...) ((void)0)
#define dev_info(...) ((void)0)
#define dev_dbg(...) ((void)0)
static u32 get_unaligned_le32(const void *p) { const u8 *b=p; return (u32)b[0]|(u32)b[1]<<8|(u32)b[2]<<16|(u32)b[3]<<24; }
static void put_unaligned_le32(u32 v,void *p) { u8 *b=p;for(unsigned i=0;i<4;i++) b[i]=v>>(8*i); }
static void put_unaligned_le64(u64 v,void *p) { u8 *b=p;for(unsigned i=0;i<8;i++) b[i]=v>>(8*i); }
struct mock_connector { bool connected; };
struct apple_dcp { struct mock_connector *connector; bool crashed, ext26_normal, ext26_power_sent; u64 ext26_rx_ns,ext26_power_ns,ext26_generation,typec_generation; int ext26_ready; };
static unsigned withdrawn,invalidated,notifications;
static void dcp_modes_begin_attachment(struct apple_dcp *d) { (void)d;withdrawn++; }
static void apple_connector_invalidate_edid(struct mock_connector *c) { (void)c;invalidated++; }
static void dcp_queue_hotplug(struct mock_connector *c) { (void)c;notifications++; }
static void complete_all(int *c) { (*c)++; }
static bool iomfb_validate_callback_h17p(int t,u32 i,u32 o) { (void)t;(void)i;(void)o;return false; }
struct dcp_method_entry { const char *name; char tag[4],tag_h17g[4],tag_ext26[4]; };
#define IOMFB_METHOD(t,n) [n]={#n,{t[0],t[1],t[2],t[3]},{0},{0}}
#define IOMFB_METHOD_H17(t,h,n) [n]={#n,{t[0],t[1],t[2],t[3]},{h[0],h[1],h[2],h[3]},{0}}
#define IOMFB_METHOD_EXT26(t,e,n) [n]={#n,{t[0],t[1],t[2],t[3]},{0},{e[0],e[1],e[2],e[3]}}
static unsigned count,done,fault,bad_status;
static void boot_done(struct apple_dcp *d,void *o,void *c) { (void)d;(void)o;(void)c;done++; }
static void dcp_push(struct apple_dcp *,bool,const struct dcp_method_entry *,u32,u32,void *,void (*)(struct apple_dcp *,void *,void *),void *);
'''
peer = r'''
static void dcp_push(struct apple_dcp *d,bool oob,const struct dcp_method_entry *m,u32 in,u32 out,void *input,void (*next)(struct apple_dcp *,void *,void *),void *cookie) {
 const char *expected[]={"A389","A446","A029","A468","A000","A465"};
 const u32 ins[]={0,0,0,4,4,0},outs[]={0,4,0,0,4,4};
 assert(count<6 && !oob);
 const char *tag=m->tag_ext26[0]?m->tag_ext26:m->tag;
 assert(!memcmp(tag,expected[count],4));
 assert(in==ins[count] && out==outs[count]);
 if(in) assert(get_unaligned_le32(input)==1);
 u32 reply=count>=4 ? 1 : 0;
 count++;
 if (bad_status==count) reply=0;
 next(d,fault==count?NULL:&reply,cookie);
}
int main(void) {
 struct apple_dcp d={0};struct ext26_boot *b=calloc(1,sizeof(*b));b->one=1;
 ext26_boot_next(&d,NULL,b);assert(done==1 && count==6 && !d.crashed);
 for(fault=1;fault<=6;fault++) {
  d=(struct apple_dcp){0};done=count=0;b=calloc(1,sizeof(*b));b->one=1;
  ext26_boot_next(&d,NULL,b);assert(!done && d.crashed && count==fault);
 }
 fault=0;
 for(bad_status=5;bad_status<=6;bad_status++) {
  d=(struct apple_dcp){0};done=count=0;b=calloc(1,sizeof(*b));b->one=1;
  ext26_boot_next(&d,NULL,b);assert(!done && d.crashed && count==bad_status);
 }
 unsigned char input[128]={0}, output[128];
 memset(output,0xcd,sizeof(output));ext26_bandwidth(&d,3,output+1,input);
 for(unsigned i=0;i<60;i++) assert(output[i+1]==(i==56));
 assert(output[0]==0xcd && output[61]==0xcd);
 for(unsigned i=0;i<80;i++) input[i]=i;
 input[80]=0;ext26_frame_sync(&d,6,output,input);assert(!memcmp(output,input,80));
 input[80]=1;ext26_frame_sync(&d,6,output,input);
 for(unsigned i=0;i<80;i++) assert(!output[i]);
 memcpy(input,"VORP",4);put_unaligned_le32(0,input+4);ext26_clock(&d,408,output,input);
 assert(get_unaligned_le32(output)==48000000);
 put_unaligned_le32(1,input+4);ext26_clock(&d,408,output,input);assert(get_unaligned_le32(output)==24000000);
 put_unaligned_le32(2,input+4);ext26_clock(&d,408,output,input);assert(!get_unaligned_le32(output));
 d=(struct apple_dcp){.ext26_power_sent=true,.ext26_power_ns=10,.ext26_generation=7,.typec_generation=7};
 memset(input,0,sizeof(input));memcpy(input,"NormalModeActive",17);input[64]=1;
 for(unsigned t=0;t<=10;t++) { d.ext26_rx_ns=t;ext26_property(&d,104,NULL,input);assert(!d.ext26_normal && !d.ext26_ready); }
 d.ext26_rx_ns=11;ext26_property(&d,104,NULL,input);assert(d.ext26_normal && d.ext26_ready==1);
 input[64]=0;ext26_property(&d,104,NULL,input);assert(!d.ext26_normal);
 input[64]=1;d.typec_generation++;ext26_property(&d,104,NULL,input);assert(!d.ext26_normal);
 d.typec_generation--;d.ext26_power_sent=false;ext26_property(&d,104,NULL,input);assert(!d.ext26_normal);
 d.ext26_power_sent=true;ext26_property(&d,104,NULL,input);assert(d.ext26_normal);
 ext26_property(&d,107,NULL,input);assert(!d.ext26_normal);
 struct mock_connector connector={true};d.connector=&connector;
 memset(input,0,sizeof(input));memcpy(input,"TimingElements",15);ext26_property(&d,107,NULL,input);
 assert(withdrawn==1&&invalidated==1&&notifications==1&&!connector.connected);
 const unsigned tags[]={3,6,107,115,116,134,408,454,567,572,576,588};
 const unsigned ins[]={4,84,64,16,16,0,8,4,128,4,4,0},outs[]={60,80,0,8,4,4,8,4,4,4,0,0};
 for(unsigned i=0;i<sizeof(tags)/sizeof(tags[0]);i++) {
  assert(ext26_callback_size(tags[i],ins[i],outs[i]));
  assert(!ext26_callback_size(tags[i],ins[i]+1,outs[i]));
  assert(!ext26_callback_size(tags[i],ins[i],outs[i]+1));
 }
 puts("PASS native26 startup sequence, transport/status failures, byte-bounded replies and causal readiness");
 return 0;
}
'''
code = pre + block(header, 'enum dcpep_method {') + '\n' + block(source, 'static const struct dcp_method_entry dcp_methods[') + '\n'
code += block(source, 'struct ext26_boot {') + '\n'
for name in ['ext26_boot_next', 'ext26_callback_size', 'ext26_property', 'ext26_bandwidth', 'ext26_frame_sync', 'ext26_clock']:
    code += function(name) + '\n'
with tempfile.TemporaryDirectory() as tmp:
    p = Path(tmp) / 'profile.c'
    p.write_text(code + peer)
    binary = p.with_suffix('')
    subprocess.run([os.environ.get('CC', 'cc'), '-std=gnu11', '-O1', '-g', '-Wall', '-Wextra',
                    '-Werror', '-Wno-unused-parameter', '-fsanitize=address,undefined',
                    str(p), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
