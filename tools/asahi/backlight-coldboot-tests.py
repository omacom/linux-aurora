#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Execute production DCP power-on/fallback and backlight DPMS state callbacks.

Kernel services and firmware responses are controlled host stubs. These tests
establish callback ordering/state, not panel operation or hardware sleep.
"""
import argparse
import os
from pathlib import Path
import re
import resource
import shlex
import signal
import subprocess
import tempfile


def function(source, name):
    match = re.search(r'^(?:static )?(?:inline )?(?:void|bool|int) ' + re.escape(name) + r'\([^;]*?\)\n\{', source, re.M)
    if not match:
        raise RuntimeError('missing production function: ' + name)
    begin = source.index('{', match.start())
    depth, end = 1, begin + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[match.start():end]


PRELUDE = r'''
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <limits.h>
typedef uint32_t u32; typedef uint64_t u64;
#define READ_ONCE(x) (x)
#define WRITE_ONCE(x,v) ((x)=(v))
#define min(a,b) ((a)<(b)?(a):(b))
#define spin_lock_irqsave(lock,flags) do { (void)(lock);(flags)=0; } while(0)
#define spin_unlock_irqrestore(lock,flags) do { (void)(lock);(void)(flags); } while(0)
#define guard(type) fake_guard
#define scoped_guard(type,p) if (fake_scoped_guard(p))
#define container_of(p,t,m) ((t *)((char *)(p)-offsetof(t,m)))
#define to_delayed_work(p) container_of(p,struct delayed_work,work)
#define DCP_BL_FALLBACK_DELAY 2
#define DPTX_RECONNECT_DELAY 3
#define DRM_MODE_CONNECTOR_eDP 1
enum { DCP_FIRMWARE_V_12_3, DCP_FIRMWARE_V_13_5,
 DCP_FIRMWARE_V_14_7_TEMPLATE, DCP_FIRMWARE_V_26_6_DESKTOP,
 DCP_FIRMWARE_V_26_6, DCP_FIRMWARE_V_14_7,
 DCP_FIRMWARE_H17P, DCP_FIRMWARE_V_27_0 };
enum { DCP_IOMFB_METHODS_H17G, DCP_IOMFB_METHODS_PANEL };
struct work_struct { int unused; };
struct delayed_work { struct work_struct work; };
struct dcp_backlight_state {
 u32 maximum,target,level,fade_from,actual,sent_nits,reported;
 u64 sequence;
 unsigned retries;
 bool ready,controlled,preserve_inherited,dpms_off,core_blank,suspended,
 dirty,in_flight,fade,fading,level_known,report_valid;
};
struct desktop { bool trial_powered,ext_pipe_off; };
struct apple_dcp {
 int fw_compat,connector_type;
 struct { int iomfb_method_profile; } hw;
 struct desktop *desktop;
 struct { struct dcp_backlight_state state; int lock; } backlight;
 struct { bool update; u32 maximum; } brightness;
 bool panel,m4,v14,external,typec,pipe_enabled_h17p,crashed,quiescing;
 bool typec_crtc_off,typec_cable_connected,typec_follow_start,typec_poweron_hpd,hdmi_yielded;
 unsigned crtc_unplugs,typec_unplugs,typec_generation,typec_reconnect_tries,external_generation,external_crtc_generation;
 int external_lock,hpd_mutex,relink_lock,relink,typec_iomfb_hpd_ready;
 struct delayed_work typec_reconnect_wq,placeholder_edid_wq,external_stop_retry_wq,bl_fallback_wq;
 struct work_struct external_stop_work,ext26_blank_work;
 void *dev,*hdmi_hpd,*avep;
};
struct platform_device { struct apple_dcp *data; };
static unsigned dpms_calls,kicks,fallbacks,warnings,errors,infos,powers;
static int forced_error;
static bool pipe_failure;
static unsigned long jiffies,system_freezable_wq;
#define dev_warn(...) (++warnings)
#define dev_err(...) (++errors)
#define dev_info(...) (++infos)
#define WARN_ONCE(...) (++warnings)
static void fake_guard(int *p) { (void)p; }
static bool fake_scoped_guard(int *p) { (void)p;return true; }
static void *platform_get_drvdata(struct platform_device *p) { return p->data; }
static bool iomfb_is_m4(struct apple_dcp *p) { return p->m4; }
static bool iomfb_v14_7_panel_nits(struct apple_dcp *p) { return p->v14; }
static bool dcp_has_panel(struct apple_dcp *p) { return p->panel; }
static bool dcp_is_external_h17p(struct apple_dcp *p) { return p->external; }
/* These fixtures exercise the internal panel and the H17P external path. */
static bool dcp_is_ext26(struct apple_dcp *p) { (void)p;return false; }
static bool dcp_is_retained_external(struct apple_dcp *p) { return dcp_is_external_h17p(p); }
static void flush_work(struct work_struct *p) { (void)p;assert(!"unexpected native26 work flush"); }
static bool dcp_is_typec_output(struct apple_dcp *p) { return p->typec; }
static bool dcp_h17g_panel(struct apple_dcp *p) { return p->hw.iomfb_method_profile==DCP_IOMFB_METHODS_H17G; }
static void dcp_backlight_kick(struct apple_dcp *p) { assert(p->backlight.state.ready);kicks++; }
static void dcp_dpms_power_on(struct apple_dcp *p) { (void)p;powers++; }
static void enable_pipe(struct apple_dcp *p) { if(!pipe_failure)p->pipe_enabled_h17p=true; }
static void iomfb_poweron_h17p(struct apple_dcp *p) { enable_pipe(p); }
static void iomfb_poweron_v27(struct apple_dcp *p) { enable_pipe(p); }
#define POWER_STUB(name) static void name(struct apple_dcp *p) { (void)p; }
POWER_STUB(iomfb_poweron_v12_3)
POWER_STUB(iomfb_poweron_v13_3)
POWER_STUB(iomfb_poweron_v14_7_0)
POWER_STUB(iomfb_poweron_v26_6_desktop)
POWER_STUB(iomfb_v26_6_poweron)
POWER_STUB(iomfb_v14_7_poweron)
POWER_STUB(av_service_connect)
POWER_STUB(dcp_cycle_handed_over_panel)
static void cancel_delayed_work_sync(struct delayed_work *p) { (void)p; }
static void cancel_delayed_work(struct delayed_work *p) { (void)p; }
static void cancel_work_sync(struct work_struct *p) { (void)p; }
static int dcp_external_start_locked(struct apple_dcp *p) { (void)p;return 0; }
static void dcp_relink_wake(int *r,unsigned g,unsigned long j) { (void)r;(void)g;(void)j; }
static void dcp_relink_link_up(int *r,unsigned g,unsigned long j) { (void)r;(void)g;(void)j; }
static int dcp_dptx_connect(struct apple_dcp *p,int n) { (void)p;(void)n;return 0; }
static void dcp_queue_typec_reconnect(struct apple_dcp *p,int n) { (void)p;(void)n; }
static unsigned long wait_for_completion_timeout(int *p,unsigned long n) { (void)p;return n; }
static unsigned long msecs_to_jiffies(unsigned long n) { return n; }
static bool gpiod_get_value_cansleep(void *p) { return p!=NULL; }
static void queue_delayed_work(unsigned long q,struct delayed_work *w,unsigned delay)
{ (void)q;(void)w;assert(delay==DCP_BL_FALLBACK_DELAY);fallbacks++; }
'''

TESTS = r'''
static struct apple_dcp fresh(int firmware)
{
 struct apple_dcp d={0};
 d.fw_compat=firmware;d.panel=true;d.connector_type=DRM_MODE_CONNECTOR_eDP;
 d.hw.iomfb_method_profile=DCP_IOMFB_METHODS_PANEL;d.brightness.maximum=1000;
 dpms_calls=kicks=fallbacks=warnings=errors=infos=powers=0;forced_error=0;pipe_failure=false;
 return d;
}
int main(void)
{
 for(int firmware=DCP_FIRMWARE_H17P;firmware<=DCP_FIRMWARE_V_27_0;firmware++) {
  struct apple_dcp d=fresh(firmware);struct platform_device p={&d};
  dcp_poweron(&p);
  assert(d.pipe_enabled_h17p && fallbacks==1 && powers==1);
  assert(dpms_calls==0 && warnings==0 && kicks==0);
  assert(!d.backlight.state.ready && !d.backlight.state.controlled);
  dcp_work_backlight_fallback(&d.bl_fallback_wq.work);
  assert(d.backlight.state.ready && !d.backlight.state.level_known);
  assert(!d.backlight.state.controlled && !d.backlight.state.dirty && kicks==0 && infos==1);
  assert(test_v27_backlight_admission(&d)==-EBUSY);
  assert(dcp_bl_report(&d.backlight.state,120));
  assert(d.backlight.state.target==120 && d.backlight.state.level_known);
  assert(test_v27_backlight_admission(&d)==0);
  assert(dcp_bl_dpms(&d.backlight.state,false)==0);
  dcp_poweron(&p);
  assert(dpms_calls==1 && kicks==1 && warnings==0 && !d.backlight.state.dpms_off);
  assert(d.backlight.state.target==120 && d.backlight.state.level==120 && d.backlight.state.dirty);

  d=fresh(firmware);p.data=&d;d.pipe_enabled_h17p=true;
  dcp_poweron(&p);
  assert(fallbacks==1 && powers==1 && dpms_calls==0);

  d=fresh(firmware);p.data=&d;d.pipe_enabled_h17p=true;
  assert(!dcp_bl_init(&d.backlight.state,1000,true,80,false,0));
  assert(!dcp_bl_dpms(&d.backlight.state,false));
  dcp_poweron(&p);
  assert(dpms_calls==1 && kicks==1 && fallbacks==0 && warnings==0);
  assert(d.backlight.state.target==80 && d.backlight.state.level==80 && !d.backlight.state.dpms_off);

  for(int fault=0;fault<2;fault++) {
   d=fresh(firmware);p.data=&d;d.pipe_enabled_h17p=true;
   assert(!dcp_bl_init(&d.backlight.state,1000,true,80,false,0));
   forced_error=fault?-EIO:-ENODATA;
   dcp_poweron(&p);
   assert(dpms_calls==1 && warnings==1 && kicks==0);
   assert(d.backlight.state.target==80 && !d.backlight.state.controlled);
  }
  d=fresh(firmware);p.data=&d;pipe_failure=true;
  dcp_poweron(&p);
  assert(d.crashed && errors==1 && dpms_calls==0 && kicks==0 && fallbacks==0);
 }
 /* The distinct M4 path retains its original ENODATA/error behavior. */
 struct apple_dcp d=fresh(DCP_FIRMWARE_V_27_0);struct platform_device p={&d};d.m4=true;
 dcp_poweron(&p);assert(dpms_calls==1 && warnings==1 && !d.backlight.state.ready);
 puts("coldboot/late-hint/fallback, ready restore, error and M4 callback paths PASS");
}
'''


def harness(tree, source):
    backlight = (tree / 'drivers/gpu/drm/apple/dcp_backlight.c').read_text()
    header = (tree / 'drivers/gpu/drm/apple/dcp_backlight.h').read_text()
    state = '\n'.join(function(header, n) for n in ('dcp_bl_init', 'dcp_bl_seed',
                     'dcp_bl_report', 'dcp_bl_jump', 'dcp_bl_dpms'))
    active = '\n'.join(function(backlight, n) for n in
                       ('dcp_backlight_presents_nits', 'dcp_backlight_active', 'dcp_backlight_level_known'))
    sleep = (tree / 'drivers/gpu/drm/apple/dcp-v27-sleep.inc').read_text()
    prepare = function(sleep, 'dcp_v27_sleep_prepare')
    guard = re.search(r'^\tif \(!dcp_v27_external\(dcp\) && dcp_has_panel\(dcp\).*?^\t}', prepare, re.M | re.S)
    assert guard, 'missing production V27 backlight guard'
    admission = ('static bool dcp_v27_external(struct apple_dcp *dcp) { return dcp->external; }\n'
                 'static int test_v27_backlight_admission(struct apple_dcp *dcp) {\n'
                 + guard.group() + '\nreturn 0;\n}\n')
    dpms = function(backlight, 'dcp_backlight_dpms').replace('int dcp_backlight_dpms(', 'int production_dcpms(')
    adapter = '''
static int dcp_backlight_dpms(struct apple_dcp *dcp,bool on) {
 dpms_calls++;return forced_error?forced_error:production_dcpms(dcp,on);
}
static int iomfb_configure_backlight_h17p(struct apple_dcp *dcp,u32 max,bool inherited,u32 level,bool fallback,u32 nits) {
 if(dcp->backlight.state.ready)return -EBUSY;
 return dcp_bl_init(&dcp->backlight.state,max,inherited,level,fallback,nits);
}
'''
    return PRELUDE + state + active + admission + dpms + adapter + '\n'.join(function(source, n) for n in
        ('dcp_uses_soft_dpms', 'dcp_poweron', 'dcp_work_backlight_fallback')) + TESTS


def execute(tree, source, directory, name, expect_failure=False, assertion=''):
    path = directory / (name + '.c');path.write_text(harness(tree, source))
    binary = path.with_suffix('')
    compiler = shlex.split(os.environ.get('CC', 'cc'))
    subprocess.run(compiler + ['-std=gnu11', '-Wall', '-Wextra', '-Werror',
        '-Wno-unused-function', '-Wno-unused-parameter', '-Wno-unused-variable',
        str(path), '-o', str(binary)], check=True)
    def limits(): resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
    result = subprocess.run([binary], capture_output=True, text=True, preexec_fn=limits)
    (directory / (name + '.log')).write_text(result.stdout + result.stderr)
    if expect_failure:
        assert result.returncode == -signal.SIGABRT, (name, result.returncode, result.stderr)
        assert assertion in result.stderr, (name, result.stderr)
        print('Compiled assertion control PASS:', name)
    else:
        assert result.returncode == 0, (name, result.returncode, result.stderr)
        print(result.stdout.strip())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tree', type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument('--source', type=Path)
    parser.add_argument('--old-source', type=Path)
    parser.add_argument('--negative-controls', action='store_true')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    source = (args.source or args.tree / 'drivers/gpu/drm/apple/dcp.c').read_text()
    with tempfile.TemporaryDirectory(prefix='backlight-coldboot-') as temporary:
        directory = args.output or Path(temporary);directory.mkdir(parents=True, exist_ok=True)
        execute(args.tree, source, directory, 'current')
        if args.old_source:
            execute(args.tree, args.old_source.read_text(), directory, 'old-unready-restore', True,
                    'dpms_calls==0 && warnings==0 && kicks==0')
        if args.negative_controls:
            guard = '\t\tif (!dcp_backlight_active(dcp))\n\t\t\treturn;\n'
            warning = '\t\tif (ret)\n\t\t\tdev_warn(dcp->dev, "backlight restore unavailable: %d\\n", ret);'
            controls = (
                ('unready-restore', guard, '', 'dpms_calls==0 && warnings==0 && kicks==0'),
                ('premature-control', guard, '\t\tdcp->backlight.state.controlled = true;\n' + guard,
                 '!d.backlight.state.ready && !d.backlight.state.controlled'),
                ('lost-ready-restore', guard, '\t\treturn;\n', 'dpms_calls==1 && kicks==1'),
                ('muted-real-error', warning, warning.replace('if (ret)', 'if (false)'),
                 'dpms_calls==1 && warnings==1 && kicks==0'))
            for name, old, new, assertion in controls:
                assert source.count(old) == 1, (name, source.count(old))
                execute(args.tree, source.replace(old, new), directory, name, True, assertion)
            poweron = function(source, 'dcp_poweron')
            fallback = re.compile(r'queue_delayed_work\(system_freezable_wq,\s*&dcp->bl_fallback_wq,\s*'
                                  r'DCP_BL_FALLBACK_DELAY\);')
            mutated, count = fallback.subn('(void)0;', poweron)
            assert count == 1, ('lost-fallback', count)
            execute(args.tree, source.replace(poweron, mutated, 1), directory,
                    'lost-fallback', True, 'd.pipe_enabled_h17p && fallbacks==1 && powers==1')


if __name__ == '__main__': main()
