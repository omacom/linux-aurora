#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Exercise native preflight with DRM's detached CRTC state after state swap."""
from pathlib import Path
import subprocess,tempfile
R=Path(__file__).resolve().parents[2];s=(R/'drivers/gpu/drm/apple/iomfb_template.c').read_text();caller=(R/'drivers/gpu/drm/apple/iomfb.c').read_text()
a=s.index('\tif (dcp_is_ext26(dcp)) {',s.index('int DCP_FW_NAME(iomfb_modeset)'))
b=s.index('\n\tif (!lookup_mode',a)
a=s.rfind('#if DCP_FW_VER',0,a)
body=s[a:b]
assert 'crtc_state->state' not in body
assert 'drm_atomic_get_new_plane_state(commit, crtc->primary)' in caller
pre=r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <errno.h>
typedef uint32_t u32;
#define DCP_FW_VERSION(a,b,c) ((a)*10000+(b)*100+(c))
#define DCP_FW_VER DCP_FW_VERSION(26,6,0)
#define READ_ONCE(x) (x)
#define WRITE_ONCE(x,v) ((x)=(v))
struct mode {int id;};
struct drm_framebuffer {u32 pitches[1];};
struct drm_plane_state {struct drm_framebuffer *fb;};
struct drm_crtc_state {void *state;struct mode mode;};
struct apple_dcp {bool ext26_blocked,pipe_enabled_h17p,ext26_blanked,ext26_normal,ext26_power_sent;u32 ext26_stride,ext26_generation,typec_generation;struct mode ext26_mode;int ext26_ready;};
static bool dcp_is_ext26(struct apple_dcp *d){return true;}
static bool drm_mode_equal(const struct mode *a,const struct mode *b){return a->id==b->id;}
static unsigned stops;
static int dcp_ext26_layout_stop(struct apple_dcp *d){stops++;d->pipe_enabled_h17p=false;return 0;}
#define reinit_completion(c) (*(c)=0)
'''
code=pre+'static int prepare(struct apple_dcp *dcp,struct drm_crtc_state *crtc_state,struct drm_plane_state *primary){int ret;\n'+body+'\nreturn 0;}'+r'''
int main(void){
 struct drm_crtc_state c={.state=0,.mode.id=1};struct drm_framebuffer fb={.pitches={15360}};struct drm_plane_state p={.fb=&fb};struct apple_dcp d={.typec_generation=7};
 assert(!prepare(&d,&c,&p)&&d.ext26_stride==15360&&d.ext26_generation==7);
 d.pipe_enabled_h17p=true;d.ext26_blanked=true;d.ext26_mode=c.mode;
 assert(!prepare(&d,&c,&p)&&!d.ext26_blanked&&!stops);
 // G's post-resume repeat must preserve the already proven active mode.
 d.ext26_normal=true;d.ext26_power_sent=true;d.ext26_ready=1;
 assert(!prepare(&d,&c,&p)&&d.ext26_normal&&d.ext26_power_sent&&d.ext26_ready==1&&!stops);
 // A different physical generation must still acquire fresh readiness.
 d.typec_generation++;assert(!prepare(&d,&c,&p)&&!d.ext26_normal&&!d.ext26_power_sent&&d.ext26_ready==0);
 fb.pitches[0]=16384;assert(!prepare(&d,&c,&p)&&stops==1&&d.ext26_stride==16384);
 d.ext26_blocked=true;assert(prepare(&d,&c,&p)==-EIO);
 d.ext26_blocked=false;assert(!prepare(&d,&c,0));
}
'''
with tempfile.TemporaryDirectory() as t:
 p=Path(t)/'test.c';p.write_text(code);subprocess.run(['clang','-std=gnu11','-Wall','-Werror','-fsanitize=address,undefined',str(p),'-o',str(p.with_suffix(''))],check=True);subprocess.run([str(p.with_suffix(''))],check=True)
print('PASS actual native preflight: detached CRTC atomic backlink, 4K pitch, gentle resume, changed layout, missing plane and blocked session')

# The driver enable callback must finish route ownership and wake a pending flip
# on native modeset errors; the DRM core, not this callback, owns commit completion.
drv=(R/'drivers/gpu/drm/apple/apple_drv.c').read_text()
a=drv.index('static void apple_crtc_atomic_enable(');b=drv.index('\n}',a)+2
actual=drv[a:b]
preamble=r'''
#include <assert.h>
#include <stdbool.h>
#include <errno.h>
#define dev_warn(...) ((void)0)
#define dev_err(...) ((void)0)
struct drm_crtc {int unused;};
struct drm_crtc_state {bool active,active_changed,color_mgmt_changed;};
struct drm_atomic_commit {struct drm_crtc_state *state;unsigned failed_routes;};
struct apple_dcp {void *dev;bool native,main_display;int mode_state,vblank_wq;void *connector;};
struct apple_crtc {struct apple_dcp *dcp;bool fabric_held;};
static struct apple_crtc owner;
static struct apple_crtc *to_apple_crtc(struct drm_crtc *c){return &owner;}
static struct apple_dcp *platform_get_drvdata(struct apple_dcp *d){return d;}
static struct drm_crtc_state *drm_atomic_get_new_crtc_state(struct drm_atomic_commit *c,struct drm_crtc *r){return c->state;}
#define to_apple_atomic_state(c) (c)
static unsigned drm_crtc_mask(struct drm_crtc *c){return 1;}
static bool dcp_is_ext26(struct apple_dcp *d){return d->native;}
static unsigned done,woken,notified,invalidated,powered,programmed;
static int mode_error;
static int dcp_typec_follow_crtc(struct drm_crtc *c,struct drm_atomic_commit *a,bool *locked){*locked=true;return 0;}
static void dcp_typec_follow_done(bool locked){assert(locked);done++;}
static bool dcp_crtc_route_ready(struct drm_crtc *c,struct drm_atomic_commit *a,bool enable){return true;}
static bool dcp_crtc_needs_route_start(struct apple_dcp *d){return false;}
static int dcp_typec_prepare_route(struct apple_dcp *d){return 0;}
static int apple_drm_fabric_hold(void){return 0;}
static void dcp_poweron(struct apple_dcp *d){powered++;}
static void dcp_external_recover(struct apple_dcp *d){}
static bool drm_atomic_crtc_needs_modeset(struct drm_crtc_state *s){return true;}
static int dcp_crtc_atomic_modeset(struct drm_crtc *c,struct drm_atomic_commit *a){programmed++;return mode_error;}
static void dcp_mode_invalidate(int *m){invalidated++;*m=0;}
static void schedule_work(int *w){woken++;}
static void dcp_route_failure_notify(void *c){notified++;}
'''
checks=r'''
int main(void){
 struct drm_crtc crtc={0};struct drm_crtc_state state={.active=true,.active_changed=true};
 struct drm_atomic_commit commit={.state=&state};struct apple_dcp d={.native=true,.connector=(void *)1};owner.dcp=&d;
 const int errors[]={-EIO,-ENOMEM,-ETIMEDOUT,-EPIPE};
 for(unsigned i=0;i<4;i++){
  done=woken=notified=invalidated=powered=programmed=0;mode_error=errors[i];d.mode_state=1;
  apple_crtc_atomic_enable(&crtc,&commit);
  assert(done==1&&woken==1&&notified==1&&invalidated==1&&programmed==1&&powered==1&&!d.mode_state);
 }
 done=woken=notified=invalidated=0;mode_error=0;
 apple_crtc_atomic_enable(&crtc,&commit);assert(done==1&&!woken&&!notified&&!invalidated);
 // Existing non-native error policy is preserved.
 d.native=false;done=woken=notified=invalidated=0;mode_error=-EIO;
 apple_crtc_atomic_enable(&crtc,&commit);assert(done==1&&!woken&&!notified&&!invalidated);
}
'''
with tempfile.TemporaryDirectory() as t:
 p=Path(t)/'enable.c';p.write_text(preamble+actual+checks)
 subprocess.run(['clang','-std=gnu11','-Wall','-Werror','-fsanitize=address,undefined',str(p),'-o',str(p.with_suffix(''))],check=True)
 subprocess.run([str(p.with_suffix(''))],check=True)
print('PASS actual enable callback: native mode errors invalidate/wake flips/finish route once; success and classic policy preserved')
