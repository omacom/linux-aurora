#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Execute native null-control and retained-stop bodies against a fault peer."""
from pathlib import Path
import os
import re
import subprocess
import tempfile
ROOT = Path(__file__).resolve().parents[2]

def body(file, name):
    s = (ROOT / file).read_text()
    pos = re.search(r'^(?:static )?(?:void|bool|int) ' + name + r'\(', s, re.M).start()
    pos = s.index(name + '(', pos)
    start = s.rfind('\n', 0, s.rfind('\n', 0, pos)) + 1
    # Return types of these routines live on one line with the signature.
    start = s.rfind('\n', 0, pos) + 1
    return s[start:s.index('\n}', pos)+2]

pre = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
typedef uint32_t u32; typedef uint64_t u64; typedef uint8_t u8;
#define READ_ONCE(x) (x)
#define WRITE_ONCE(x,y) ((x)=(y))
#define lockdep_assert_held(x) ((void)0)
#define ARRAY_SIZE(x) (sizeof(x)/sizeof(*(x)))
#define IS_ERR_OR_NULL(x) (!(x))
#define scoped_guard(...) for (bool once=true;once;once=false)
#define dev_info(...) ((void)0)
#define dev_dbg(...) ((void)0)
#define dev_warn(...) ((void)0)
#define dev_err(...) mock_log(__VA_ARGS__)
static void mock_log(int dev,const char *fmt,...) { (void)dev;(void)fmt; }
#define msecs_to_jiffies(x) (x)
#define DCP_SWAP_WATCHDOG_MS 2000
#define DCP_EXTERNAL_STOP_RETRIES 3
#define AFK_MAX_CHANNEL 3
#define MAX_PENDING_CMDS 16
#define spin_lock_irqsave(l,f) ((void)(l),(void)(&(f)))
#define spin_unlock_irqrestore(l,f) ((void)(l),(void)(&(f)))
static bool bitmap_empty(const unsigned long *v,unsigned n) { (void)n;return !*v; }
struct apple_epic_service { int lock; unsigned long cmd_map[1]; };
struct ring { bool ready; u32 *rptr,*wptr; };
struct apple_dcp_afkep { int lock,wq; struct ring rxbfr,txbfr; struct apple_epic_service services[AFK_MAX_CHANNEL]; };
struct port { bool connected; int *service; };
struct ext26_control;
#define SWAP_SURFACES 4
#define guard(x) (void)
struct drm_gem_object { int tag; };
struct drm_framebuffer { struct drm_gem_object *obj[1]; };
struct iomfb_scanout_h17p { struct drm_framebuffer *fb[4]; struct { struct iomfb_scanout_h17p *next; } link; };
#define list_for_each_entry(pos,head,member) for ((pos)=(head)->first;(pos);(pos)=(pos)->member.next)
struct apple_dcp {
 struct { int lock; } iomfb; struct { struct iomfb_scanout_h17p *first; } ext26_retained;
 bool external_suspended,ext26_pm_preparing,ext26_pm_work_off;u8 *coproc_reg;
 u64 external_generation; int dev,external_error; bool external_stopped,external_power_attempted,external_running;
 bool external_stop_deferred,pipe_enabled_h17p,typec_crtc_off,ext26_normal,ext26_stopping;
 unsigned external_stop_retries; int external_lock,hpd_mutex,external_stop_retry_wq,rtk,avep;
 struct port dptxport[1];
 struct apple_dcp_afkep *ext26_aux[3];
 struct apple_dcp_afkep *systemep,*ibootep,*dptx_controller_ep,*dptxep,*dcpavservep;
 struct ext26_control *ext26_control; u32 ext26_control_id; bool ext26_control_pending,crashed,ext26_blocked;
};
#define APPLE_DCP_COPROC_CPU_CONTROL 0x44
#define APPLE_DCP_COPROC_CPU_CONTROL_RUN 16
static u32 readl(const void *p) { u32 v;memcpy(&v,p,4);return v; }
static unsigned discoveries;
static void dcp_enable_typec_work(struct apple_dcp *d) { (void)d;discoveries++; }
static bool dcp_is_ext26(struct apple_dcp *d) { (void)d;return true; }
static char trace[128]; static unsigned used; static char fault; static unsigned settlements;
static void note(char c) { assert(used<sizeof(trace)-1);trace[used++]=c;trace[used]=0; }
static int result(char c) { note(c);return c==fault?-EIO:0; }
static void iomfb_pause_presentation(struct apple_dcp *d,bool b) { (void)d;note(b?'F':'U'); }
static int iomfb_queue_settle(struct apple_dcp *d,int t) { (void)d;(void)t;settlements++;return result('Q'); }
static int dcp_external_link_locked(struct apple_dcp *d) { (void)d;return result('L'); }
static int iomfb_external_power_h17p(struct apple_dcp *d,bool on) { (void)d;assert(!on);return result('P'); }
static bool dptx_service_available(struct port *p) { return p->service; }
static int dptxport_set_hpd(int *s,bool on) { assert(s&&!on);return result('H'); }
static int dptxport_release_display(int *s) { assert(s);return result('R'); }
static void apple_rtkit_flush_rx(int r) { (void)r;note('X'); }
static void flush_workqueue(int q) { (void)q;note('W'); }
static int iomfb_ext26_retain_epoch(struct apple_dcp *d) { (void)d;return result('K'); }
static void dpavservep_detach(struct apple_dcp *d) { (void)d;note('E'); }
static void dcp_modes_begin_attachment(struct apple_dcp *d) { (void)d;note('M'); }
static void av_service_disconnect(struct apple_dcp *d) { (void)d;note('A'); }
static int system_dfl_wq;
static void mod_delayed_work(int q,int *w,int t) { (void)q;(void)w;(void)t;note('D'); }
struct completion { unsigned signaled; };
static void complete_all(struct completion *c) { c->signaled++; }
#define DCPEXT26_SUBMIT_BYTES 0x1bd8
struct ext26_control { struct dcpext26_exchange exchange; struct completion done; u8 wire[DCPEXT26_SUBMIT_BYTES]; };
'''
checks = r'''
static struct apple_dcp fresh(struct apple_dcp_afkep *ep) {
 static int service; struct apple_dcp d={0}; d.external_generation=7; d.external_power_attempted=true;
 d.external_running=true; d.pipe_enabled_h17p=true; d.ext26_normal=true;
 d.dptxport[0].connected=true; d.dptxport[0].service=&service; d.systemep=ep;
 used=settlements=0;trace[0]=0;fault=0;return d;
}
static void native_pm_complete(struct apple_dcp *dcp);
static int native_pm_resume(struct apple_dcp *dcp);
int main(void) {
 struct ext26_control c={0}; struct apple_dcp d={0};d.ext26_control=&c;d.ext26_control_pending=true;
 assert(!dcpext26_exchange_begin(&c.exchange,0));
 assert(dcpext26_exchange_begin(&c.exchange,42));assert(!dcpext26_exchange_begin(&c.exchange,43));
 assert(!iomfb_ext26_control_complete(&d,41,false));assert(!c.done.signaled);
 assert(iomfb_ext26_control_complete(&d,42,false));assert(!c.done.signaled&&d.ext26_control_pending);
 c.exchange.replied=true; ext26_control_signal(&d,&c);assert(c.done.signaled&&!d.ext26_control_pending);
 c=(struct ext26_control){0};d.ext26_control=&c;d.ext26_control_pending=true;
 assert(dcpext26_exchange_begin(&c.exchange,51));c.exchange.replied=true;
 ext26_control_signal(&d,&c);assert(!c.done.signaled);
 assert(iomfb_ext26_control_complete(&d,51,true));assert(c.exchange.failed&&c.done.signaled);
 assert(!dcpext26_epoch_fits(8,1,1)); assert(dcpext26_epoch_fits(7,0,(960ULL<<20)));
 assert(!dcpext26_epoch_fits(7,(960ULL<<20),1));assert(!dcpext26_epoch_fits(0,~0ULL,1));
 struct apple_dcp_afkep ep={0};u32 read=0,write=0;
 ep.rxbfr=(struct ring){true,&read,&write};assert(afk_host_idle(&ep));
 write=64;assert(!afk_host_idle(&ep));read=64;assert(afk_host_idle(&ep));
 ep.services[1].cmd_map[0]=1;assert(!afk_host_idle(&ep));ep.services[1].cmd_map[0]=0;
 d=fresh(&ep);assert(!dcp_external_stop_locked(&d,6)&&!used); /* stale stop token */
 assert(!dcp_external_stop_locked(&d,7));assert(!strcmp(trace,"FEQPQHRXWXQKM"));
 assert(d.external_stopped&&!d.pipe_enabled_h17p&&!d.ext26_normal&&!d.ext26_stopping);
 assert(d.typec_crtc_off&&!d.dptxport[0].connected);
 assert(!dcp_external_stop_locked(&d,7)); /* idempotent stop */
 d=fresh(&ep);fault='P';assert(dcp_external_stop_locked(&d,7)==-EIO);
 assert(d.external_error==-EIO&&!d.external_stopped&&d.dptxport[0].connected);
 assert(!strcmp(trace,"FEQP"));assert(dcp_external_stop_locked(&d,7)==-EIO);
 d=fresh(&ep);fault='H';assert(dcp_external_stop_locked(&d,7)==-EIO);assert(!strchr(trace,'R'));
 d=fresh(&ep);fault='R';assert(dcp_external_stop_locked(&d,7)==-EIO);assert(!strchr(trace,'K'));
 d=fresh(&ep);ep.services[0].cmd_map[0]=1;assert(dcp_external_stop_locked(&d,7)==-EBUSY);
 assert(!strchr(trace,'K'));ep.services[0].cmd_map[0]=0;
 d=fresh(&ep);d.external_power_attempted=false;assert(!dcp_external_stop_locked(&d,7));
 assert(!strchr(trace,'P')&&strchr(trace,'K')); /* no-sink / pre-picture stop */
 d=fresh(&ep);assert(d.external_running);assert(!dcp_external_start_locked(&d));assert(!strcmp(trace,"LU"));
 d=fresh(&ep);d.external_running=false;assert(!dcp_external_start_locked(&d));assert(d.external_running&&!strcmp(trace,"LU"));
 struct drm_gem_object oldobj={1},newobj={2};
 struct drm_framebuffer oldfb={{&oldobj}},aliasfb={{&oldobj}},newfb={{&newobj}};
 struct iomfb_scanout_h17p retained={.fb={&oldfb}};d.ext26_retained.first=&retained;
 assert(!iomfb_ext26_backing_fresh(&d,&oldfb));assert(!iomfb_ext26_backing_fresh(&d,&aliasfb));
 assert(iomfb_ext26_backing_fresh(&d,&newfb));
 u8 registers[0x50]={0};u32 running=16;memcpy(registers+0x44,&running,4);
 d=(struct apple_dcp){.coproc_reg=registers,.ext26_pm_preparing=true,.ext26_pm_work_off=true,.external_suspended=true};
 assert(!native_pm_resume(&d));assert(!discoveries&&d.ext26_pm_preparing&&d.ext26_pm_work_off);
 native_pm_complete(&d);assert(discoveries==1&&!d.ext26_pm_preparing&&!d.ext26_pm_work_off);
 native_pm_complete(&d);assert(discoveries==1);
 memset(registers,0,sizeof(registers));d.ext26_pm_preparing=d.ext26_pm_work_off=true;
 assert(native_pm_resume(&d)==-EHOSTDOWN&&d.ext26_blocked&&d.external_error==-EHOSTDOWN);
 native_pm_complete(&d);assert(discoveries==1); /* a dead coprocessor gets no discovery */
 puts("PASS native26 exact control order, stale IDs, abort, retained-stop sequencing/faults, host drain, old-GEM rejection, epoch bounds and PM-complete ordering");
}
'''
life = (ROOT/'drivers/gpu/drm/apple/dcpext26-lifecycle.h').read_text().replace('#include <linux/types.h>','')
# Put the pure state before the mock control container.
pre = pre.replace('struct ext26_control { struct dcpext26_exchange',life+'\nstruct ext26_control { struct dcpext26_exchange')
code = pre + '\n' + '\n'.join([
 body('drivers/gpu/drm/apple/afk.c','afk_host_idle'),
 body('drivers/gpu/drm/apple/iomfb.c','iomfb_ext26_backing_fresh'),
 body('drivers/gpu/drm/apple/iomfb_h17p.c','ext26_control_signal'),
 body('drivers/gpu/drm/apple/iomfb_h17p.c','iomfb_ext26_control_complete'),
 body('drivers/gpu/drm/apple/dcp.c','dcp_external_stop_locked')]) + checks
# dcp.c has a forward declaration; select the actual definition.
s=(ROOT/'drivers/gpu/drm/apple/dcp.c').read_text();pos=s.index('static int dcp_external_stop_locked(struct apple_dcp *dcp, u64 generation)\n{');actual=s[pos:s.index('\n}',pos)+2]
code=code[:code.index('static int dcp_external_stop_locked')] + actual + '\n' + body('drivers/gpu/drm/apple/dcp.c','dcp_external_start_locked') + checks
# Execute the native branches of the actual PM hooks, preserving their returns.
def native_branch(name):
    pos=s.index('static '+('void' if name.endswith('complete') else 'int')+' '+name+'(')
    pos=s.index('if (dcp_is_ext26(dcp)) {',pos)
    opening=s.index('{',pos);end=opening+1;depth=1
    while depth:
        depth+=(s[end]=='{')-(s[end]=='}');end+=1
    return s[pos:end]
code += '\nstatic void native_pm_complete(struct apple_dcp *dcp) { '+native_branch('dcp_platform_complete')+' }\n'
code += 'static int native_pm_resume(struct apple_dcp *dcp) { int dev=0; '+native_branch('dcp_platform_resume')+' return -EINVAL; }\n'

with tempfile.TemporaryDirectory() as t:
    p=Path(t);(p/'check.c').write_text(code)
    subprocess.run([os.environ.get('CC','clang'),'-std=gnu11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined',str(p/'check.c'),'-o',str(p/'check')],check=True)
    subprocess.run([str(p/'check')],check=True)
