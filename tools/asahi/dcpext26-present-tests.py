#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Actual DRM primary start/submit callbacks: faults and early exact completion."""
from pathlib import Path
import os,re,subprocess,tempfile
ROOT=Path(__file__).resolve().parents[2]
s=(ROOT/'drivers/gpu/drm/apple/iomfb_template.c').read_text()
def function(name):
 m=re.search(r'^static void '+name+r'\(',s,re.M);assert m
 return s[m.start():s.index('\n}',m.start())+2]
pre=r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <errno.h>
#include <stdio.h>
typedef uint32_t u32;typedef uint64_t u64;
#define WRITE_ONCE(x,y) ((x)=(y))
#define DCP_FW_NAME(n) n
#define DCP_FW_UNION(x) ((x).h17p)
#define dev_err(...) ((void)0)
#define trace_iomfb_swap_submit(...) ((void)0)
struct dc_swap_complete_resp { u32 swap_id; };
struct dcp_swap_submit_resp { u32 ret; };
struct dcp_swap_start_resp { u32 swap_id,ret; };
struct apple_dcp { bool native,crashed,ext26_blocked,ext26_early_complete;u32 ext26_early_id;
 u64 ext26_rx_ns,ext26_early_ns,swap_start,swap_submit_timestamp;void *dev,*crtc;
 struct { struct { struct {u32 swap_id;} swap; } h17p; } swap; };
static int submit_result;static bool begin_allowed=true;
static unsigned failures,events,armed,watchdogs,completed,sent;static u64 completion_time;
static bool dcp_is_ext26(struct apple_dcp *d) { return d->native; }
static int dcp_present_submit(struct apple_dcp *d,u32 id,bool ok) { (void)d;(void)id;(void)ok;return submit_result; }
static void dcp_present_failed(struct apple_dcp *d) { (void)d;failures++; }
static bool dcp_present_retires(struct apple_dcp *d) { (void)d;return true; }
static void dcp_unarm_retained_framebuffers(struct apple_dcp *d,u32 id) { (void)d;(void)id; }
static void dcp_drm_crtc_vblank(void *c) { (void)c;events++; }
static void dcpep_cb_swap_complete(struct apple_dcp *d,struct dc_swap_complete_resp *r) { assert(r->swap_id==d->swap.h17p.swap.swap_id);completed++;completion_time=d->ext26_rx_ns; }
static u64 ktime_get(void) { return 30; }
static u64 arch_timer_read_counter(void) { return 40; }
static void dcp_swap_watchdog_arm(struct apple_dcp *d) { (void)d;watchdogs++; }
static bool dcp_present_begin(struct apple_dcp *d,u32 id) { (void)d;assert(id);return begin_allowed; }
static void dcp_arm_retained_framebuffers(struct apple_dcp *d,u32 id) { (void)d;(void)id;armed++; }
static void dcp_h17p_prepare_swap(struct apple_dcp *d) { assert(!d->native); }
static void dcp_swap_submit(struct apple_dcp *d,bool oob,void *r,void (*cb)(struct apple_dcp *,void *,void *),void *c) { (void)d;(void)r;(void)cb;(void)c;assert(!oob);sent++; }
'''
check=r'''
static struct apple_dcp fresh(bool native) {
 struct apple_dcp d={.native=native};d.swap.h17p.swap.swap_id=7;
 failures=events=armed=watchdogs=completed=sent=0;submit_result=0;begin_allowed=true;return d;
}
int main(void) {
 struct dcp_swap_submit_resp bad={1},ok={0};struct apple_dcp d=fresh(true);
 dcp_swapped(&d,&bad,NULL);assert(d.crashed&&d.ext26_blocked&&failures==1&&events==1&&!watchdogs);
 d=fresh(false);dcp_swapped(&d,&bad,NULL);assert(!d.crashed&&!d.ext26_blocked&&failures==1);
 d=fresh(true);dcp_swapped(&d,NULL,NULL);assert(d.crashed&&d.ext26_blocked&&!watchdogs);
 d=fresh(true);submit_result=-ECANCELED;dcp_swapped(&d,&ok,NULL);assert(d.crashed&&d.ext26_blocked);
 d=fresh(true);d.ext26_early_complete=true;d.ext26_early_id=7;d.ext26_early_ns=15;d.ext26_rx_ns=20;
 dcp_swapped(&d,&ok,NULL);assert(completed==1&&completion_time==15&&d.ext26_rx_ns==20&&!d.ext26_early_complete&&!watchdogs&&!d.crashed);
 d=fresh(true);dcp_swapped(&d,&ok,NULL);assert(watchdogs==1&&!d.crashed&&!failures);
 struct dcp_swap_start_resp start={8,1};d=fresh(true);dcp_swap_started(&d,&start,NULL);assert(d.crashed&&d.ext26_blocked&&!armed&&!sent);
 d=fresh(false);dcp_swap_started(&d,&start,NULL);assert(!d.crashed&&!sent);
 d=fresh(true);start=(struct dcp_swap_start_resp){0,0};dcp_swap_started(&d,&start,NULL);assert(d.crashed&&!sent);
 d=fresh(true);start.swap_id=8;begin_allowed=false;dcp_swap_started(&d,&start,NULL);assert(d.crashed&&!sent&&!armed);
 d=fresh(true);dcp_swap_started(&d,&start,NULL);assert(!d.crashed&&sent==1&&armed==1&&d.swap.h17p.swap.swap_id==8);
 puts("PASS actual native DRM start/submit rejection, classic controls and original-time early D590 joining");
}
'''
with tempfile.TemporaryDirectory() as t:
 p=Path(t);(p/'check.c').write_text(pre+'\n'.join(function(n) for n in ['dcp_present_rejected','dcp_swapped','dcp_swap_started'])+check)
 subprocess.run([os.environ.get('CC','clang'),'-std=gnu11','-O1','-Wall','-Wextra','-Werror','-Wno-unused-parameter','-fsanitize=address,undefined',str(p/'check.c'),'-o',str(p/'check')],check=True)
 subprocess.run([str(p/'check')],check=True)
