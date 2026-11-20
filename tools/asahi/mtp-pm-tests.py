#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Run production MTP PM and magicmouse suspend/resume callbacks without a device.

Checks notification order, suspend rollback, resume-error handling, that
no actuator request is sent while the transport is suspended, the actuator
mode asserted on resume and the multitouch enable work held across sleep. The old transport must compile and fail at runtime.
"""
import argparse
import os
import re
from pathlib import Path
import resource
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
TRANSPORT = 'drivers/hid/dockchannel-hid/dockchannel-hid.c'


def function(source, name):
    match = re.search(r'static (?:int|void|bool) ' + re.escape(name) + r'\([^;]*?\)\n\{', source)
    if not match:
        raise ValueError(name)
    return source[match.start():source.index('\n}', match.end())+2]


PREFIX = r'''
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#define MAX_INTERFACES 16
#define PMSG_SUSPEND 1
#define HID_HAPTIC_MODE_DEVICE 0
#define HID_HAPTIC_MODE_HOST 1
#define READ_ONCE(x) (x)
#define WRITE_ONCE(x,v) ((x) = (v))
#define FF_MAX_EFFECTS 4
typedef unsigned u32;
typedef unsigned short u16;
typedef unsigned char u8;
#define HID_HP_WAVEFORMPRESS 2
#define HID_HP_WAVEFORMRELEASE 3
#define APPLE_HP_WAVEFORMDEEPCLICK 4
#define HID_USAGE 0xffff
#define HID_TYPE_USBMOUSE 1
#define BUS_SPI 2
#define min(a,b) ((a) < (b) ? (a) : (b))
#define min_t(t,a,b) min(a,b)
#define guard(type) (void)
#define scoped_guard(type, lock) for (int _once = ((void)(lock), 1); _once; _once = 0)
#define spin_lock_irqsave(lock, flags) ((void)(lock), (flags) = 0)
#define spin_unlock_irqrestore(lock, flags) ((void)(lock), (void)(flags))
#define hid_warn(...) (warnings++)
#define hid_dbg(...) ((void)0)
#define dev_err(...) ((void)0)
static unsigned warnings;
struct hid_haptic_device { int mode; void *wq; int auto_trigger_mutex; };
struct delayed_work { bool pending; unsigned queued; };
struct work_struct { unsigned cancels, queued; };
struct hid_device;
struct input_dev { struct hid_device *data; };
struct effect_job { int effect_type, lock; struct work_struct work;
    struct hid_device *taptic_hdev; u8 strength, softness; };
struct ff_effect { int id; struct { struct { u16 hid_usage, intensity; } haptic; } u; };
struct magicmouse_sc { struct hid_haptic_device *haptics; bool haptic_button_down, haptic_deep_down, haptic_suspended;
    int haptic_lock;
    struct work_struct haptic_press_work, haptic_release_work, haptic_deep_work, haptic_deep_release_work;
    struct effect_job *haptic_effects; struct hid_device *hdev;
    struct delayed_work work; bool work_held; };
typedef int pm_message_t;
static bool haptic_press;
static unsigned haptic_press_threshold=100, haptic_release_threshold=40;
static unsigned haptic_deep_threshold=200, haptic_deep_release_threshold=150;
static void cancel_work_sync(struct work_struct *w) { w->cancels++; }
static bool queue_work(void *q, struct work_struct *w) { (void)q; w->queued++; return true; }
static void flush_workqueue(void *q) { (void)q; }
static unsigned long msecs_to_jiffies(unsigned m) { return m; }
static bool cancel_delayed_work_sync(struct delayed_work *w) { bool p = w->pending; w->pending = false; return p; }
static bool schedule_delayed_work(struct delayed_work *w, unsigned long d) { w->queued++; w->pending = true; return true; }
struct hid_device { struct magicmouse_sc *data; int id, type, bus; };
static struct hid_device *input_get_drvdata(struct input_dev *d) { return d->data; }
struct dchid_iface { struct hid_device *hid; };
struct dockchannel_hid { struct dchid_iface *ifaces[MAX_INTERFACES]; bool suspended, host_asleep; };
struct device { struct dockchannel_hid *data; };
static struct dockchannel_hid *transport;
static unsigned suspends[16], resumes[16], key_releases;
static int suspend_error_at = -1, resume_error_at = -1;
static void *dev_get_drvdata(struct device *d) { return d->data; }
#define HOST_STATE_SLEEP 1
#define HOST_STATE_AWAKE 2
#define dev_warn(...) ((void)0)
/* Not a firmware-driven suspend: the host state is never set here. */
static bool pm_suspend_via_firmware(void) { return false; }
static int dchid_host_state(struct dockchannel_hid *d, int state) { (void)d; (void)state; assert(0); return 0; }
/* Sent only with the host state, in a firmware-driven suspend. */
static void dchid_send_sleep_reports(struct dockchannel_hid *d) { (void)d; assert(0); }
static void *hid_get_drvdata(struct hid_device *d) { return d->data; }
static void magicmouse_force_key(struct magicmouse_sc *m, bool down) {
    assert(!down); key_releases++;
}
/* The transport refuses requests (-EBUSY) while it is suspended: never send one then. */
static struct hid_device *actuator;
static unsigned switches; static u32 switched_to; static int switch_error;
static struct hid_device *magicmouse_get_actuator(struct magicmouse_sc *m) { (void)m; return actuator; }
static int apple_taptic_switch_modes(struct hid_device *a, u32 mode) {
    assert(a && !transport->suspended);
    switches++; switched_to = mode; return switch_error;
}
static bool magicmouse_effect_is_deep(struct ff_effect *e) { return e->u.haptic.hid_usage == APPLE_HP_WAVEFORMDEEPCLICK; }
static u8 magicmouse_haptic_strength(struct hid_device *h, bool deep) { (void)h; (void)deep; return 100; }
static u8 magicmouse_haptic_softness(struct hid_device *h, bool deep) { (void)h; (void)deep; return 100; }
static int hid_driver_suspend(struct hid_device *h, int state) {
    assert(state == PMSG_SUSPEND && !transport->suspended);
    suspends[h->id]++;
    if (h->id != suspend_error_at)
        magicmouse_suspend(h, state);
    return h->id == suspend_error_at ? -EBUSY : 0;
}
'''
MIDDLE = r'''
static int hid_driver_resume(struct hid_device *h) {
    assert(!transport->suspended);
    resumes[h->id]++;
    magicmouse_resume(h);
    return h->id == resume_error_at ? -EIO : 0;
}
'''
CASES = r'''
int main(void) {
    struct hid_haptic_device hap = {.mode = HID_HAPTIC_MODE_HOST, .wq = &hap};
    struct magicmouse_sc m = {.haptics = &hap, .haptic_button_down = true, .haptic_deep_down = true};
    struct hid_device h0 = {.data = &m, .id = 0}, h4 = {.id = 4}, h8 = {.id = 8};
    struct effect_job effects[FF_MAX_EFFECTS] = {0};
    actuator = &h0;
    struct dchid_iface i0 = {.hid = &h0}, i4 = {.hid = &h4}, i8 = {.hid = &h8}, empty = {0};
    struct dockchannel_hid t = {.ifaces = {[0] = &i0, [3] = &empty, [4] = &i4, [8] = &i8}};
    struct device d = {.data = &t}; transport = &t;
    assert(!dockchannel_hid_suspend(&d) && t.suspended);
    /* Suspend stops the click workers; nothing is sent until resume. */
    assert(m.haptic_suspended && m.haptic_press_work.cancels == 1 && m.haptic_deep_release_work.cancels == 1);
    assert(!dockchannel_hid_resume(&d) && !t.suspended && !m.haptic_suspended);
    /* A cached host mode is not trusted: device mode is asserted once the transport is up. */
    assert(hap.mode == HID_HAPTIC_MODE_DEVICE && switches == 1 && switched_to == HID_HAPTIC_MODE_DEVICE);
    assert(!m.haptic_button_down && !m.haptic_deep_down && key_releases == 1);
    assert(suspends[0] == 1 && resumes[0] == 1 && resumes[8] == 1);
    /* A later HID failure rolls back only the drivers that suspended. */
    suspend_error_at = 4;
    assert(dockchannel_hid_suspend(&d) == -EBUSY && !t.suspended);
    assert(resumes[0] == 2 && resumes[4] == 1 && suspends[8] == 1);
    suspend_error_at = 0;
    assert(dockchannel_hid_suspend(&d) == -EBUSY && !t.suspended);
    assert(resumes[0] == 2 && suspends[4] == 2);
    suspend_error_at = -1;
    assert(!dockchannel_hid_suspend(&d) && t.suspended);
    /* One failed resume cannot keep the other interfaces suspended. */
    resume_error_at = 4;
    assert(dockchannel_hid_resume(&d) == -EIO && !t.suspended);
    assert(resumes[0] == 3 && resumes[4] == 2 && resumes[8] == 2);
    resume_error_at = -1;
    for (unsigned n = 0; n < 3; n++) {
        hap.mode = HID_HAPTIC_MODE_HOST;
        assert(!dockchannel_hid_suspend(&d));
        assert(!dockchannel_hid_resume(&d));
        assert(hap.mode == HID_HAPTIC_MODE_DEVICE);
    }
    /* Host clicks wanted: host mode is taken again on resume. */
    haptic_press = true; switches = 0;
    assert(!dockchannel_hid_suspend(&d) && !dockchannel_hid_resume(&d));
    assert(switches == 1 && switched_to == HID_HAPTIC_MODE_HOST && hap.mode == HID_HAPTIC_MODE_HOST);
    haptic_press = false;
    /* Pending multitouch enable work is held across sleep and re-queued once. */
    m.work.pending = true; m.work.queued = 0;
    assert(!dockchannel_hid_suspend(&d) && !m.work.pending && m.work_held);
    assert(!dockchannel_hid_resume(&d) && m.work.pending && !m.work_held && m.work.queued == 1);
    m.work.pending = false;
    assert(!dockchannel_hid_suspend(&d) && !m.work_held);
    assert(!dockchannel_hid_resume(&d) && !m.work.pending && m.work.queued == 1);
    /* A report after the cancel must not leave work to replay after resume. */
    m.haptic_effects = effects;
    struct input_dev input = {.data = &h0};
    haptic_press = true; m.haptic_button_down = false;
    assert(!dockchannel_hid_suspend(&d) && t.suspended);
    unsigned queued = m.haptic_press_work.queued;
    assert(!magicmouse_haptic_click(&m, haptic_press_threshold, false));
    assert(m.haptic_press_work.queued == queued && !m.haptic_button_down);
    assert(!apple_taptic_playback(&input, 0, 1) && !effects[0].work.queued);
    for (unsigned i=0; i<FF_MAX_EFFECTS; i++) assert(effects[i].work.cancels);
    struct ff_effect effect = {.id=0, .u.haptic={.hid_usage=HID_HP_WAVEFORMPRESS,.intensity=50}};
    switches=0;
    assert(apple_upload_effects(&input, &effect, NULL) == -EBUSY && !switches);
    effects[0].effect_type=HID_HP_WAVEFORMPRESS; hap.mode=HID_HAPTIC_MODE_HOST;
    assert(!apple_taptic_erase(&input,0) && !effects[0].effect_type && !switches);
    assert(!dockchannel_hid_resume(&d) && !m.haptic_suspended);
    /* Default device mode adds no I/O; loaded FF effects still request host. */
    haptic_press=false; hap.mode=HID_HAPTIC_MODE_DEVICE; switches=0;
    assert(!dockchannel_hid_suspend(&d) && !dockchannel_hid_resume(&d) && !switches);
    effects[0].effect_type=HID_HP_WAVEFORMPRESS;
    assert(!dockchannel_hid_suspend(&d) && !dockchannel_hid_resume(&d));
    assert(switches == 1 && switched_to == HID_HAPTIC_MODE_HOST && hap.mode == HID_HAPTIC_MODE_HOST);
    effects[0].effect_type=0;
    /* Failed restores preserve the cache value needed for a later retry. */
    switch_error=-EIO; haptic_press=true;
    assert(!dockchannel_hid_suspend(&d) && !dockchannel_hid_resume(&d));
    assert(hap.mode == HID_HAPTIC_MODE_DEVICE && haptic_press);
    haptic_press=false; hap.mode=HID_HAPTIC_MODE_HOST;
    assert(!dockchannel_hid_suspend(&d) && !dockchannel_hid_resume(&d));
    assert(hap.mode == HID_HAPTIC_MODE_HOST);
    switch_error=-EOPNOTSUPP; haptic_press=true; unsigned warned=warnings;
    assert(!dockchannel_hid_suspend(&d) && !dockchannel_hid_resume(&d));
    assert(hap.mode == HID_HAPTIC_MODE_DEVICE && warnings == warned);
    switch_error=0;
    assert(!magicmouse_reset_resume(&h0) && !m.haptic_suspended && hap.mode == HID_HAPTIC_MODE_HOST);
    assert(!magicmouse_suspend(&h4,PMSG_SUSPEND) && !magicmouse_resume(&h4));
    puts("MTP PM: HID notifications, rollback, all-interface resume, no request while suspended, haptic mode on resume and held enable work passed");
}
'''


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out')
    p.add_argument('--old-transport')
    p.add_argument('--old-mouse', help='negative control: production mouse source at this revision')
    args = p.parse_args()
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
    source = ((ROOT / TRANSPORT).read_text() if not args.old_transport else subprocess.check_output(
        ['git', 'show', args.old_transport + ':' + TRANSPORT], cwd=ROOT, text=True))
    mouse = ((ROOT / 'drivers/hid/hid-magicmouse.c').read_text() if not args.old_mouse else
             subprocess.check_output(['git', 'show', args.old_mouse + ':drivers/hid/hid-magicmouse.c'], cwd=ROOT, text=True))
    with tempfile.TemporaryDirectory(prefix='mtp-pm-') as tmp:
        out = Path(args.out or tmp)
        out.mkdir(parents=True, exist_ok=True)
        c, exe = out/'mtp-pm.c', out/'mtp-pm'
        haptics = '\n'.join(function(mouse, name) for name in (
            'magicmouse_haptic_effects_loaded', 'magicmouse_haptic_click',
            'magicmouse_haptic_suspend', 'magicmouse_haptic_resume', 'magicmouse_switch_mode',
            'apple_upload_effects', 'apple_taptic_playback', 'apple_taptic_erase')) + '\n'
        c.write_text(PREFIX.replace('static int hid_driver_suspend', haptics + function(mouse, 'magicmouse_suspend') +
                                    '\nstatic int hid_driver_suspend', 1) +
                     function(mouse, 'magicmouse_resume') + function(mouse, 'magicmouse_reset_resume') + MIDDLE +
                     function(source, 'dockchannel_hid_suspend') + '\n' +
                     function(source, 'dockchannel_hid_resume') + CASES)
        subprocess.run([os.environ.get('CC', 'cc'), '-std=c11', '-Wall', '-Wextra',
                        '-Wno-unused-parameter', '-Wno-unused-function',
                        str(c), '-o', str(exe)], check=True)
        r = subprocess.run([str(exe)], capture_output=True, text=True)
        (out/'run.log').write_text(r.stdout+r.stderr)
        if args.old_transport or args.old_mouse:
            if r.returncode != -6 or 'Assertion' not in r.stderr:
                raise SystemExit('negative control did not fail its runtime assertion')
            print('MTP PM: negative control compiled and failed a runtime assertion')
        elif r.returncode:
            raise SystemExit(r.stdout+r.stderr)
        else:
            print(r.stdout, end='')


if __name__ == '__main__':
    main()
