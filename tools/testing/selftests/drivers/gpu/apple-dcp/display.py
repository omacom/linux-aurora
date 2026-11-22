# SPDX-License-Identifier: GPL-2.0-only OR MIT
# Copyright 2026 Ryan Murray
"""Internal panel policy and real TimingElements parser, without hardware."""
import copy
import struct
from common import block
import rate


def indicator_dpms(root):
    source = (root / 'drivers/gpu/drm/apple/iomfb.c').read_text()
    return r'''
#include <assert.h>
#include <stdbool.h>
#include <string.h>
#define SWAP_SURFACES 4
#define BIT(i) (1U << (i))
#define IOMFB_SET_BACKGROUND BIT(31)
#define lockdep_assert_held(lock) ((void)(lock))
#define scoped_guard(kind, lock) for (int once = ((void)(lock), 1); once; once = 0)
struct drm_framebuffer { unsigned int refs; };
struct dcp_swap_submit_req_h17p {
    struct {
        unsigned int swap_enabled, swap_completed, bg_color;
        unsigned int src_rect[4], dst_rect[4], surf_ids[4], surf_flags[4], surf_unk[4];
    } swap;
    unsigned int surf[4], surf_iova[4];
    bool surf_null[4];
};
struct iomfb_scanout_h17p {
    struct drm_framebuffer *fb[4];
    struct dcp_swap_submit_req_h17p request;
};
struct apple_dcp {
    struct dcp_camera_indicator {
        bool has_surf;
        unsigned int surf;
        int lock;
        struct { bool wanted; } state;
    } camera_ind;
    struct { struct dcp_swap_submit_req_h17p h17p; } swap;
    struct { int lock; struct iomfb_scanout_h17p *scanout, *next_scanout; } iomfb;
};
static void drm_framebuffer_put(struct drm_framebuffer *fb)
{
    assert(fb->refs);
    fb->refs--;
}
static void iomfb_scanout_release_h17p(struct iomfb_scanout_h17p *scanout)
{
    if (scanout)
        for (unsigned int i = 0; i < SWAP_SURFACES; ++i)
            if (scanout->fb[i])
                drm_framebuffer_put(scanout->fb[i]);
}
static struct drm_framebuffer *confirmed;
static void dcp_indicator_swap_done(struct apple_dcp *dcp, struct drm_framebuffer *fb)
{
    assert(dcp->camera_ind.has_surf);
    confirmed = fb;
}
''' + '\n'.join(block(source, name) for name in (
    'static void iomfb_null_surface_h17p(',
    'void iomfb_prepare_indicator_h17p(',
    'static void iomfb_retire_indicator_h17p(',
    'void iomfb_scanout_complete_h17p(',
)) + r'''
int main(void)
{
    struct drm_framebuffer desktop = { 1 }, dot = { 2 };
    struct iomfb_scanout_h17p old = { .fb = { [3] = &dot } };
    struct iomfb_scanout_h17p next = { .fb = { [0] = &desktop, [3] = &dot } };
    struct apple_dcp dcp = {
        .camera_ind = { .has_surf = true, .surf = 3 },
        .iomfb = { .scanout = &old, .next_scanout = &next },
        .swap.h17p = {
            .swap.swap_enabled = BIT(0),
            .swap.swap_completed = BIT(0),
            .surf_null = { false, true, true, true },
        },
    };
    iomfb_prepare_indicator_h17p(&dcp);
    assert(dcp.swap.h17p.swap.swap_enabled == (BIT(0) | BIT(3)));
    assert(dcp.swap.h17p.swap.swap_completed == dcp.swap.h17p.swap.swap_enabled);
    assert(old.fb[3] == &dot && dot.refs == 2);
    confirmed = &dot;
    iomfb_scanout_complete_h17p(&dcp);
    assert(dcp.iomfb.scanout == &next && !dcp.iomfb.next_scanout);
    assert(!next.fb[3] && !dot.refs && !confirmed);
    assert(next.request.surf_null[3] && !(next.request.swap.swap_enabled & BIT(3)));
    assert(next.fb[0] == &desktop && desktop.refs == 1);
    assert(next.request.swap.swap_enabled == BIT(0));

    /* A brightness or wake replay shares the retained scanout, with no next one. */
    dot.refs = 1;
    next.fb[3] = &dot;
    next.request.surf_null[3] = false;
    next.request.surf_iova[3] = 42;
    next.request.swap.swap_enabled |= BIT(3);
    dcp.swap.h17p = next.request;
    dcp.camera_ind.state.wanted = true;
    iomfb_prepare_indicator_h17p(&dcp);
    assert(!dcp.swap.h17p.surf_null[3] && dcp.swap.h17p.surf_iova[3] == 42);
    dcp.camera_ind.state.wanted = false;
    iomfb_prepare_indicator_h17p(&dcp);
    assert(dcp.swap.h17p.surf_null[3] && !dcp.swap.h17p.surf_iova[3]);
    assert(next.fb[3] == &dot && dot.refs == 1);
    iomfb_retire_indicator_h17p(&dcp, &next);
    assert(!next.fb[3] && !dot.refs && !next.request.surf_iova[3]);
    assert(next.request.surf_null[3] && next.request.swap.swap_enabled == BIT(0));
    return 0;
}
'''


def backlight(root):
    header = (root / 'drivers/gpu/drm/apple/dcp_backlight.h').read_text()
    header = '\n'.join(line for line in header.splitlines()
                       if not line.startswith('#include'))
    header = header.replace('static inline ', 'static inline __attribute__((unused)) ')
    return r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <limits.h>
#include <errno.h>
typedef uint32_t u32;
typedef uint64_t u64;
typedef uint8_t u8;
#define min(a,b) ((a)<(b)?(a):(b))
#define mul_u32_u32(a,b) ((u64)(a)*(b))
#define div_u64(a,b) ((a)/(b))
''' + header + r'''
static void completed(struct dcp_backlight_state *s, struct dcp_backlight_swap *p,
                      u32 id, bool early)
{
    u32 old=s->actual;
    if(early) {
        assert(!dcp_bl_swap_finish(s,p,id,false,true));
        assert(s->actual==old && s->in_flight);
        assert(dcp_bl_swap_finish(s,p,id,true,true));
    } else {
        assert(!dcp_bl_swap_finish(s,p,id,true,true));
        assert(s->actual==old && s->in_flight);
        assert(dcp_bl_swap_finish(s,p,id,false,true));
    }
    assert(!dcp_bl_swap_finish(s,p,id,false,true));
}
int main(void)
{
    struct dcp_backlight_state s;
    struct dcp_backlight_swap p={0};
    u32 id=1, nits;
    assert(!dcp_bl_takeover_nits(600,600000,&nits) && nits==600);
    /* A registration default must never change the loader's visible level. */
    assert(!dcp_bl_init(&s,600,false,0,true,300)); s.fade=true;s.preserve_inherited=true;
    assert(!s.dirty && !s.level_known && dcp_bl_actual(&s)==300);
    assert(!dcp_bl_request(&s,300,true,true));
    assert(!dcp_bl_request(&s,300,false,false));
    assert(!s.controlled && !s.dirty && dcp_bl_actual(&s)==300);
    assert(dcp_bl_swap_prepare(&s,&p,id,true)==-EALREADY);
    assert(!dcp_bl_request(&s,250,false,false));
    assert(s.fading && !s.dirty);
    assert(dcp_bl_fade_step(&s,125,250) && s.level==275);
    assert(!dcp_bl_fade_step(&s,250,250) && s.level==250);
    assert(dcp_bl_swap_prepare(&s,&p,id,false)==-ENODATA);
    assert(!dcp_bl_swap_prepare(&s,&p,id,true));
    completed(&s,&p,id++,true);
    assert(s.actual==250 && s.level_known && dcp_bl_actual(&s)==250);
    assert(!dcp_bl_seed(&s,300));
    /* Same state as a slider request or an ALS retarget through backlight. */
    assert(!dcp_bl_request(&s,600,false,false) && s.fading);
    for(u32 t=16;t<250;t+=16) {
        u32 before=s.level;
        assert(dcp_bl_fade_step(&s,t,250));
        assert(s.level>=before && s.level<600);
        if(t==16) {
            assert(!dcp_bl_swap_prepare(&s,&p,id,true));
            assert(dcp_bl_swap_finish(&s,&p,id++,true,false));
            assert(s.actual==250 && s.dirty && dcp_bl_retry_delay(&s)==100);
        }
    }
    assert(!dcp_bl_fade_step(&s,250,250));
    assert(!dcp_bl_swap_prepare(&s,&p,id,true));
    assert(!dcp_bl_swap_finish(&s,&p,id+100,false,true));
    completed(&s,&p,id++,false); assert(s.actual==600);
    assert(dcp_bl_request(&s,601,false,false)==-ERANGE);
    assert(!dcp_bl_request(&s,0,false,false));
    assert(dcp_bl_fade_step(&s,125,250) && s.level==300);
    assert(!dcp_bl_request(&s,100,false,false) && s.fade_from==300);
    assert(dcp_bl_fade_step(&s,125,250) && s.level==200);
    assert(!dcp_bl_request(&s,100,true,false));
    assert(!s.fading && dcp_bl_effective(&s)==0);
    assert(!dcp_bl_request(&s,450,true,false) && dcp_bl_effective(&s)==0);
    assert(!dcp_bl_request(&s,450,false,false) && s.level==450);
    assert(!dcp_bl_request(&s,0,false,true) && dcp_bl_effective(&s)==0);
    assert(!dcp_bl_request(&s,450,false,false) && s.level==450);
    assert(!dcp_bl_dpms(&s,false) && dcp_bl_effective(&s)==0);
    assert(!dcp_bl_dpms(&s,true) && dcp_bl_effective(&s)==450);
    assert(!dcp_bl_resend(&s));
    /*
     * DAC firmware: the report stays 0 after a display power cycle while the
     * panel is lit again (J414s, 13.5). Boot and resume end at the same level.
     */
    {
        struct dcp_bl_presented boot={0}, resume={0};
        assert(dcp_bl_presented_actual(&boot,140)==140);
        dcp_bl_presented_set(&boot,500);
        dcp_bl_presented_set(&boot,0);
        assert(dcp_bl_presented_actual(&boot,0)==0);
        dcp_bl_presented_set(&boot,500);
        assert(dcp_bl_presented_actual(&boot,0)==500);
        dcp_bl_presented_set(&resume,500);
        dcp_bl_presented_set(&resume,0);
        dcp_bl_presented_set(&resume,0);
        assert(dcp_bl_presented_actual(&resume,0)==0);
        dcp_bl_presented_set(&resume,500);
        assert(dcp_bl_presented_actual(&resume,500)==dcp_bl_presented_actual(&boot,0));
    }
    return 0;
}
'''



def dac_words(root):
    """The production DAC tables, interpolate() and calculate_dac() at every level."""
    import re
    source = (root / 'drivers/gpu/drm/apple/dcp_backlight.c').read_text()
    defines = '\n'.join(re.findall(r'^#define (?:MIN|MAX)_BRIGHTNESS_PART[12]\s.*$', source, re.M))
    tables = '\n'.join(re.findall(r'^static u32 brightness_part[0-9]+\[\] = \{.*?\};', source, re.M | re.S))
    functions = '\n'.join(block(source, marker) for marker in (
        'static u32 interpolate(', 'static u32 calculate_dac('))
    return r"""
#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
typedef uint32_t u32;
typedef uint64_t u64;
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define WARN(c, ...) (c)
#define SCALE_FACTOR (1 << 10)
struct apple_dcp;
""" + defines + '\n' + tables + '\n' + functions + r"""
/* Every word is two 16-bit DAC halves, each a table value times 16. */
static void check_word(u32 word, u32 *hi, u32 *lo)
{
    *hi = word >> 16;
    *lo = word & 0xffff;
    assert(!(*hi & 0xf) && !(*lo & 0xf));
    assert(*hi <= 0x7ff0 && *lo <= *hi);
}
/* The widest gap between a table word's two halves, times 16. */
static u32 widest(const u32 *tbl, size_t n, u32 gap)
{
    for (size_t i = 0; i < n; i++)
        if (16 * ((tbl[i] >> 16) - (tbl[i] & 0xffff)) > gap)
            gap = 16 * ((tbl[i] >> 16) - (tbl[i] & 0xffff));
    return gap;
}
int main(void)
{
    u32 prev_hi = 0, prev_lo = 0, hi, lo, gap;
    gap = widest(brightness_part1, ARRAY_SIZE(brightness_part1), 0);
    gap = widest(brightness_part12, ARRAY_SIZE(brightness_part12), gap);
    gap = widest(brightness_part2, ARRAY_SIZE(brightness_part2), gap);
    /* The table points themselves, as before. */
    assert(calculate_dac(NULL, 2) == 0);
    assert(calculate_dac(NULL, MAX_BRIGHTNESS_PART1) == 16 * brightness_part1[ARRAY_SIZE(brightness_part1) - 1]);
    assert(calculate_dac(NULL, MIN_BRIGHTNESS_PART2) == 16 * brightness_part2[0]);
    assert(calculate_dac(NULL, MAX_BRIGHTNESS_PART2) == 16 * brightness_part2[ARRAY_SIZE(brightness_part2) - 1]);
    for (int nits = 0; nits <= (int)MAX_BRIGHTNESS_PART2; nits++) {
        check_word(calculate_dac(NULL, nits), &hi, &lo);
        /* Both halves rise with the level; neither picks up the other's fraction. */
        assert(hi >= prev_hi && lo >= prev_lo);
        /* The lower half stays the half step below, as in the table. */
        assert(hi - lo <= gap);
        prev_hi = hi;
        prev_lo = lo;
    }
    /* 370 nits (J313): the lower half was 0x10a0, a 4 to 5 nit value. */
    check_word(calculate_dac(NULL, 370), &hi, &lo);
    assert(hi > 0x7800 && lo > 0x7800);
    return 0;
}
"""


def modes(root):
    f = rate.parser_fixture(root)
    # Use the exact production parser and public serializer already used by rate.
    parser = f['parser']
    prefix = (f['prefix'] + f['epoch'] + f['metadata'] + f['header']).replace('static inline ', 'static inline __attribute__((unused)) ')
    baseline = copy.deepcopy(f['timing_fixture'][0])
    checks = []
    blobs = []

    def case(elements, expected, ranges=None, desktop=True):
        raw = bytearray(struct.pack('<I', 0xd3))
        f['wire'](elements, raw)
        i = len(blobs)
        blobs.append('static const unsigned char b%u[]={%s};' %
                     (i, ','.join(map(str, raw))))
        check = (f'{{ struct dcp_parse_ctx c; unsigned n=99; '
                 f'assert(!parse(b{i},sizeof(b{i}),&c)); c.zero_vrr_range_fallback={int(desktop)}; c.internal_present_deadline={int(desktop)}; '
                 'struct dcp_display_mode *m=enumerate_modes(&c,&n,355,230,0,true,false); '
                 f'assert(!IS_ERR(m) && n=={len(expected)});')
        for j, hz in enumerate(expected):
            check += (f'assert(m[{j}].mode.clock=={round(2200*1125*hz/1000)}); '
                      f'assert(m[{j}].timing_mode_id=={elements[j]["ID"]}); '
                      f'assert(m[{j}].mode.htotal==2200 && m[{j}].mode.vtotal==1125); '
                      f'assert(m[{j}].mode.flags==(DRM_MODE_FLAG_PHSYNC|DRM_MODE_FLAG_PVSYNC)); ')
        for j, hz in enumerate(expected):
            # A desktop internal panel takes the deadline at every timing.
            if desktop or hz >= 119.88:
                check += f'assert(m[{j}].needs_timestamps); '
            elif not (ranges and ranges[j][0]):
                check += f'assert(!m[{j}].needs_timestamps); '
        if ranges:
            for j, (lo, hi) in enumerate(ranges):
                check += f'assert(m[{j}].vrr=={int(bool(lo))}); '
                if lo:
                    check += f'assert(m[{j}].min_vrr=={lo} && m[{j}].max_vrr=={hi}); '
        checks.append(check + 'free(m); }')

    supported = []
    # Synthetic rates exercise catalog handling, not a claim about a panel dump.
    for i, hz in enumerate((120, 60, 59.94, 50, 48, 47.952)):
        m = copy.deepcopy(baseline)
        m['ID'] = 100+i
        m['VerticalAttributes']['PreciseSyncRate'] = round(hz*65536)
        supported.append(m)
    case(supported, (120, 60, 59.94, 50, 48, 47.952),
         [(24<<16, 120<<16)] + [(0,0)]*5)
    for hz in (119.88, 120, 120.01):
        m=copy.deepcopy(supported[0]); m['VerticalAttributes']['PreciseSyncRate']=round(hz*65536)
        case([m], [hz], [(24<<16, round(hz*65536))])
    for lo, hi, valid in ((48<<16,120<<16,True), (0,120<<16,False),
                          (60<<16,0,False), (0,0,True), (120<<16,24<<16,False),
                          (24<<16,144<<16,False), (120<<16,120<<16,False)):
        m=copy.deepcopy(supported[0]); m['MinimumVariableRefreshRate']=lo; m['MaximumVariableRefreshRate']=hi
        case([m],[120],[(lo or 24<<16,hi or 120<<16) if valid else (0,0)])
    m=copy.deepcopy(supported[0]);m['MinimumVariableRefreshRate']=0;m['MaximumVariableRefreshRate']=0
    case([m],[120],[(0,0)],desktop=False)
    for key in ('MinimumVariableRefreshRate', 'MaximumVariableRefreshRate'):
        m=copy.deepcopy(supported[0]);m[key]=0
        case([m],[120],[(0,0)])
    m=copy.deepcopy(supported[0]);m['MinimumVariableRefreshRate']=24<<16
    m=f['ObjectPairs'](list(m.items())+[('MinimumVariableRefreshRate',48<<16)])
    # Check duplicates against the parser directly without dict indexing in case().
    raw=bytearray(struct.pack('<I',0xd3));f['wire']([m],raw)
    blobs.append('static const unsigned char dup[]={%s};' % ','.join(map(str,raw)))
    checks.append('{struct dcp_parse_ctx c;unsigned n;assert(!parse(dup,sizeof(dup),&c));'
                  'struct dcp_display_mode *m=enumerate_modes(&c,&n,355,230,0,true,false);'
                  'assert(!IS_ERR(m) && n==1 && !m[0].vrr);free(m);}')
    # Bad geometry and virtual modes are still rejected.
    for field, value in (('Total', 1000), ('FrontPorch', 65536)):
        m=copy.deepcopy(supported[0]);m['HorizontalAttributes'][field]=value
        case([m],[])
    m=copy.deepcopy(supported[0]);m['IsVirtual']=True;case([m],[])
    return prefix + parser + '\n'.join(blobs) + '\nint main(void){' + '\n'.join(checks) + 'return 0;}'


def refresh(root):
    h=(root/'drivers/gpu/drm/apple/dcp-refresh.h').read_text().replace('#include <linux/types.h>','')
    return r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
typedef uint32_t u32;
typedef uint64_t u64;
''' + h + r'''
int main(void) {
    u32 lo=0,hi=0;
    assert(dcp_refresh_range(120U<<16,0,0,true,&lo,&hi));
    assert(lo==24U<<16 && hi==120U<<16);
    assert(!dcp_refresh_range(120U<<16,0,0,false,&lo,&hi));
    assert(!dcp_refresh_range(60U<<16,0,0,true,&lo,&hi));
    assert(!dcp_refresh_range(120U<<16,0,120U<<16,true,&lo,&hi));
    assert(!dcp_refresh_range(120U<<16,120U<<16,24U<<16,true,&lo,&hi));
    assert(!dcp_refresh_range(120U<<16,24U<<16,144U<<16,true,&lo,&hi));
    assert(dcp_desktop_present_deadline(true,true)==120);
    assert(dcp_desktop_present_deadline(true,false)==0);
    assert(dcp_desktop_present_deadline(false,true)==0);
    assert(dcp_refresh_needs_deadline(48U<<16,false,true,true));
    assert(dcp_refresh_needs_deadline(60U<<16,false,true,true));
    assert(!dcp_refresh_needs_deadline(48U<<16,false,true,false));
    assert(!dcp_refresh_needs_deadline(60U<<16,false,false,true));
    assert(dcp_refresh_needs_deadline(120U<<16,false,true,false));
    assert(dcp_refresh_needs_deadline(48U<<16,true,false,false));
    return 0;
}
'''


def takeover(root):
    prefix=backlight(root).split('static void completed(')[0]
    prefix=prefix.replace('static inline ', 'static inline __attribute__((unused)) ')
    function=block((root/'drivers/gpu/drm/apple/dcp_backlight.c').read_text(),
                   'int dcp_backlight_takeover(')
    return prefix + r'''
#define DCP_FIRMWARE_V_26_6_DESKTOP 5
#define READ_ONCE(x) (x)
#define spin_lock_irqsave(p,f) do {assert(!*(p));*(p)=true;(f)=0;} while(0)
#define spin_unlock_irqrestore(p,f) do {assert(*(p));*(p)=false;(void)(f);} while(0)
struct apple_dcp {
    int fw_compat;
    struct {u32 maximum;void *bl_dev;} brightness;
    struct {bool lock;struct dcp_backlight_state state;} backlight;
    struct {bool inherited_brightness_valid;u32 inherited_brightness;} *desktop;
};
static unsigned legacy,notifications;
static void dcp_backlight_sync_hint(struct apple_dcp *d)
{assert(d->brightness.bl_dev);notifications++;}
bool dcp_backlight_active(struct apple_dcp *d) {return d->fw_compat==1 && d->backlight.state.ready;}
static int iomfb_configure_backlight_h17p(struct apple_dcp *d,u32 max,bool valid,u32 nits,bool fallback,u32 guessed)
{assert(max==600 && valid && nits==100 && !fallback && !guessed);legacy++;return 0;}
''' + function + r'''
int main(void) {
    struct apple_dcp d={.fw_compat=5,.brightness={600}};
    __typeof__(*d.desktop) saved={0};d.desktop=&saved;
    assert(!dcp_backlight_takeover(&d,0));
    assert(saved.inherited_brightness_valid && !saved.inherited_brightness && !legacy);
    assert(!dcp_backlight_takeover(&d,500000) && !saved.inherited_brightness);
    saved.inherited_brightness_valid=false;
    assert(dcp_backlight_takeover(&d,1300000)==-ERANGE && !saved.inherited_brightness_valid);
    assert(!dcp_backlight_takeover(&d,700000) && saved.inherited_brightness==600);
    assert(!dcp_bl_init(&d.backlight.state,600,false,0,true,300));
    d.brightness.bl_dev=&d;saved.inherited_brightness_valid=false;
    assert(!dcp_backlight_takeover(&d,100000) && saved.inherited_brightness_valid);
    assert(dcp_bl_actual(&d.backlight.state)==100 && !d.backlight.state.dirty && notifications==1);
    assert(!dcp_backlight_takeover(&d,200000) && d.backlight.state.target==100 && notifications==1);
    saved.inherited_brightness_valid=false;
    assert(!dcp_bl_request(&d.backlight.state,250,false,false));
    assert(!dcp_backlight_takeover(&d,100000) && !saved.inherited_brightness_valid);
    d.fw_compat=1;d.backlight.state.ready=false;
    assert(!dcp_backlight_takeover(&d,100000) && legacy==1);
    return 0;
}
'''


def watchdog(root):
    source=(root/'drivers/gpu/drm/apple/dcp.c').read_text()
    production='\n'.join(block(source, marker) for marker in (
        'bool dcp_desktop_present_complete(',
        'VISIBLE_IF_KUNIT void dcp_swap_watchdog(',
        'void dcp_swap_watchdog_arm(',
        'void dcp_swap_watchdog_complete('))
    internal=(root/'drivers/gpu/drm/apple/dcp-internal.h').read_text()
    helpers='\n'.join(block(internal, marker) for marker in (
        'static inline bool dcp_desktop_swap_watched(',
        'static inline void dcp_takeover_adopt_proven('))
    definitions='\n'.join(line for line in source.splitlines()
                          if line.startswith('#define DCP_SWAP_WATCHDOG_'))
    return r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
typedef uint32_t u32;
#define READ_ONCE(x) (x)
#define WRITE_ONCE(x,v) ((x)=(v))
#define xchg(p,v) __atomic_exchange_n((p),(v),__ATOMIC_SEQ_CST)
#define VISIBLE_IF_KUNIT
#define DCP_FIRMWARE_V_26_6_DESKTOP 5
#define dev_warn(...) ((void)0)
#define container_of(p,t,m) ((t *)((char *)(p)-offsetof(t,m)))
#define to_delayed_work(p) container_of(p,struct delayed_work,work)
#define msecs_to_jiffies(n) (n)
#define time_before(a,b) ((long)((a)-(b))<0)
#define system_dfl_wq NULL
static unsigned long jiffies;
struct work_struct {unsigned unused;};
struct delayed_work {struct work_struct work;};
struct apple_dcp {
    int fw_compat;bool vrr_enabled,typec;
    struct {u32 present_id;bool present_pending,vrr_failed,takeover_adopted,mode_live;} *desktop;
    struct {bool valid;} mode_state;
    struct delayed_work swap_watchdog_wq;
    unsigned long swap_watchdog_deadline;
    unsigned swap_watchdog_retrains;
    void *crtc,*connector,*dev;
};
static unsigned scheduled,cancelled,vblanks,hotplugs,unarmed,rejected;
static bool allow_retire=true;
static bool iomfb_retire_dropped_present_h17p(struct apple_dcp *d) {return allow_retire;}
static void dcp_unarm_retained_framebuffers(struct apple_dcp *d,u32 id)
{assert(id==d->desktop->present_id);unarmed++;}
static void dcp_backlight_desktop_finish(struct apple_dcp *d,u32 id,bool reply,bool accepted)
{assert(id==d->desktop->present_id && reply && !accepted);rejected++;}
static void dcp_mode_invalidate(__typeof__(((struct apple_dcp *)0)->mode_state) *m) {m->valid=false;}
static void dcp_drm_crtc_vblank(void *c) {vblanks++;}
static void dcp_queue_hotplug(void *c) {hotplugs++;}
static bool dcp_is_typec_output(struct apple_dcp *d) {return d->typec;}
static void mod_delayed_work(void *q,struct delayed_work *w,unsigned long delay)
{assert(delay==1000);scheduled++;}
static void cancel_delayed_work(struct delayed_work *w) {cancelled++;}
''' + definitions + '\n' + helpers + '\n' + production + r'''
int main(void) {
    struct apple_dcp d={.fw_compat=5,.vrr_enabled=true,.mode_state={true},.connector=(void *)1};
    __typeof__(*d.desktop) state={.present_id=42,.present_pending=true};d.desktop=&state;
    dcp_swap_watchdog_arm(&d);assert(scheduled==1 && d.swap_watchdog_deadline==1000);
    jiffies=999;dcp_swap_watchdog(&d.swap_watchdog_wq.work);
    assert(d.mode_state.valid && !hotplugs && !unarmed && state.present_pending);
    assert(!dcp_desktop_present_complete(&d,41) && state.present_pending);
    assert(dcp_desktop_present_complete(&d,42) && !state.present_pending);
    assert(!dcp_desktop_present_complete(&d,42));
    dcp_swap_watchdog_complete(&d);assert(cancelled==1);
    jiffies=1000;dcp_swap_watchdog(&d.swap_watchdog_wq.work);assert(!hotplugs);
    /* Expiry owns this dropped swap, retains its buffers and triggers reset. */
    state.present_id=43;state.present_pending=true;
    dcp_swap_watchdog(&d.swap_watchdog_wq.work);
    assert(state.vrr_failed && !d.vrr_enabled && !state.present_pending);
    assert(!d.mode_state.valid && hotplugs==1 && vblanks==1 && unarmed==1 && rejected==1);
    assert(!dcp_desktop_present_complete(&d,43));
    /* A late completion cannot finish the new fixed-refresh present. */
    state.present_id=44;state.present_pending=true;
    assert(!dcp_desktop_present_complete(&d,43) && state.present_pending);
    assert(dcp_desktop_present_complete(&d,44));
    unsigned before=scheduled;dcp_swap_watchdog_arm(&d);assert(scheduled==before);
    /* The established external watchdog remains armed on a Type-C output. */
    d.fw_compat=1;d.typec=true;dcp_swap_watchdog_arm(&d);assert(scheduled==before+1);
    allow_retire=false;d.mode_state.valid=true;
    dcp_swap_watchdog(&d.swap_watchdog_wq.work);assert(d.mode_state.valid && hotplugs==1);
    /* A takeover that adopted the loader's mode is watched until a swap
     * completes; with none, the mode is set through the hotplug worker. */
    allow_retire=true;d.fw_compat=5;d.typec=false;d.vrr_enabled=false;
    state.takeover_adopted=true;state.mode_live=true;
    before=scheduled;dcp_swap_watchdog_arm(&d);assert(scheduled==before+1);
    dcp_swap_watchdog_complete(&d);assert(!state.takeover_adopted);
    before=scheduled;dcp_swap_watchdog_arm(&d);assert(scheduled==before);
    state.takeover_adopted=true;state.present_id=45;state.present_pending=true;
    jiffies=d.swap_watchdog_deadline;unsigned hp=hotplugs;
    dcp_swap_watchdog(&d.swap_watchdog_wq.work);
    assert(!state.takeover_adopted && !state.mode_live && !d.mode_state.valid && hotplugs==hp+1);
    return 0;
}
'''


def brightness_commit(root):
    source=(root/'drivers/gpu/drm/apple/dcp_backlight.c').read_text()
    production=block(source,'static int drm_crtc_set_brightness(')
    return r'''
#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <errno.h>
#define DRM_MODESET_ACQUIRE_INTERRUPTIBLE 0
#define IS_ERR(p) false
#define PTR_ERR(p) (-EINVAL)
struct drm_crtc_state {bool active,color_mgmt_changed,planes_changed;void *gamma_lut;};
struct drm_crtc {void *dev;int mutex;};
struct drm_modeset_acquire_ctx {int unused;};
struct drm_atomic_commit {struct drm_modeset_acquire_ctx *acquire_ctx;};
struct apple_dcp {bool desktop,pending;struct {bool update;} brightness;struct {struct drm_crtc base;} *crtc;};
static struct drm_atomic_commit commit;
static struct drm_crtc_state state;
static unsigned allocations,commits,planes;
static bool fail_alloc;
static bool dcp_backlight_desktop(struct apple_dcp *d) {return d->desktop;}
static bool dcp_backlight_pending(struct apple_dcp *d) {return d->pending;}
static struct drm_atomic_commit *drm_atomic_commit_alloc(void *dev)
{allocations++;return fail_alloc?NULL:&commit;}
static void drm_modeset_acquire_init(struct drm_modeset_acquire_ctx *c,int flags) {}
static int drm_modeset_lock(int *m,struct drm_modeset_acquire_ctx *c) {assert(!*m);*m=1;return 0;}
static struct drm_crtc_state *drm_atomic_get_crtc_state(struct drm_atomic_commit *a,struct drm_crtc *c)
{assert(c->mutex);return &state;}
static int drm_atomic_add_affected_planes(struct drm_atomic_commit *a,struct drm_crtc *c)
{assert(c->mutex);planes++;return 0;}
static int drm_atomic_commit(struct drm_atomic_commit *a) {commits++;return 0;}
static void drm_atomic_commit_clear(struct drm_atomic_commit *a) {assert(false);}
static int drm_modeset_backoff(struct drm_modeset_acquire_ctx *c) {assert(false);return 0;}
static void drm_atomic_commit_put(struct drm_atomic_commit *a) {}
static void drm_modeset_drop_locks(struct drm_modeset_acquire_ctx *c) {}
static void drm_modeset_acquire_fini(struct drm_modeset_acquire_ctx *c) {}
''' + production + r'''
int main(void) {
    struct apple_dcp d={.desktop=true,.pending=true,.brightness={true}};
    __typeof__(*d.crtc) crtc={0};d.crtc=&crtc;
    state.active=true;state.gamma_lut=&d;
    assert(!drm_crtc_set_brightness(&d));
    assert(commits==1 && planes==1 && state.planes_changed && !state.color_mgmt_changed);
    crtc.base.mutex=0;state=(struct drm_crtc_state){.active=true};d.desktop=false;
    assert(!drm_crtc_set_brightness(&d));
    assert(commits==2 && planes==2 && state.color_mgmt_changed);
    crtc.base.mutex=0;state=(struct drm_crtc_state){.active=false};d.desktop=true;
    assert(!drm_crtc_set_brightness(&d) && commits==2);
    crtc.base.mutex=0;state.active=true;d.pending=false;
    assert(!drm_crtc_set_brightness(&d) && commits==2);
    fail_alloc=true;assert(drm_crtc_set_brightness(&d)==-ENOMEM && commits==2);
    assert(allocations==5);
    return 0;
}
'''


def hdr_metadata(root):
    """HDR10 static metadata message for DCPAV selector 0x15, byte for byte."""
    header = (root / 'drivers/gpu/drm/apple/dcp-hdr-metadata.h').read_text()
    header = '\n'.join(line for line in header.splitlines()
                       if not line.startswith('#include'))
    header = header.replace('static inline ', 'static inline __attribute__((unused)) ')
    # An independent encoding of the same message: the DCPAV IPC header, then
    # the serialized { "Type": "CEA-861:Type1", "Data": <24 bytes> }.
    def tag(last, kind, size):
        return struct.pack('<I', (last << 31) | (kind << 24) | size)

    def pad(raw):
        return raw + b'\0' * (-len(raw) % 4)

    body = struct.pack('<12H', 35400, 14600, 8500, 39850, 6550, 2300,
                       15635, 16450, 1000, 50, 1000, 400)
    obj = struct.pack('<I', 0xd3) + tag(1, 1, 2)
    obj += tag(0, 8, 5) + pad(b'Type\0')
    obj += tag(0, 9, 13) + pad(b'CEA-861:Type1')
    obj += tag(0, 8, 5) + pad(b'Data\0')
    obj += tag(1, 10, 24) + body
    msg = bytearray(0x60)
    struct.pack_into('<HHIII', msg, 0, 0, 1, 0x15, len(obj) + 0x20, 0x69706378)
    struct.pack_into('<I', msg, 0x50, len(obj))
    msg += obj
    expected = ','.join(map(str, msg))
    # selector 0x14: a fixed 0x80-byte message clearing InfoFrame type 0x87
    stop = bytearray(0x80)
    struct.pack_into('<HHIII', stop, 0, 0, 0, 0x14, 0x40, 0x69706378)
    stop[0x40] = 0x87
    expected_stop = ','.join(map(str, stop))
    return r"""
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
typedef uint8_t u8; typedef uint16_t u16; typedef uint32_t u32;
typedef uint8_t __u8; typedef uint16_t __u16; typedef uint32_t __u32;
static void put_unaligned_le16(u16 v, void *p) { u8 *b = p; b[0] = v; b[1] = v >> 8; }
static void put_unaligned_le32(u32 v, void *p) { u8 *b = p; for (int i = 0; i < 4; i++) b[i] = v >> (8 * i); }
static u16 __attribute__((unused)) get_unaligned_le16(const void *p) { const u8 *b = p; return b[0] | b[1] << 8; }
static u32 get_unaligned_le32(const void *p) { const u8 *b = p; return b[0] | b[1] << 8 | b[2] << 16 | (u32)b[3] << 24; }
/* include/uapi/drm/drm_mode.h */
struct hdr_metadata_infoframe {
    __u8 eotf; __u8 metadata_type;
    struct { __u16 x, y; } display_primaries[3];
    struct { __u16 x, y; } white_point;
    __u16 max_display_mastering_luminance, min_display_mastering_luminance;
    __u16 max_cll, max_fall;
};
struct hdr_output_metadata {
    __u32 metadata_type;
    union { struct hdr_metadata_infoframe hdmi_metadata_type1; };
};
""" + header + r"""
static const u8 expected[] = {""" + expected + r"""};
static const u8 expected_stop[] = {""" + expected_stop + r"""};
int main(void)
{
    struct hdr_output_metadata hdr = { .metadata_type = 0, .hdmi_metadata_type1 = {
        .eotf = 2, .metadata_type = 0,
        .display_primaries = { { 35400, 14600 }, { 8500, 39850 }, { 6550, 2300 } },
        .white_point = { 15635, 16450 },
        .max_display_mastering_luminance = 1000, .min_display_mastering_luminance = 50,
        .max_cll = 1000, .max_fall = 400 } };
    u8 data[DCP_HDR_DATA_SIZE], msg[DCP_HDR_MSG_SIZE], reply[DCP_HDR_MSG_SIZE] = {0};

    assert(sizeof(expected) == DCP_HDR_MSG_SIZE);
    assert(!dcp_hdr_data(data, &hdr));
    dcp_hdr_message(msg, data);
    for (size_t i = 0; i < sizeof(msg); i++)
        if (msg[i] != expected[i]) {
            fprintf(stderr, "byte %#zx: %#x, expected %#x\n", i, msg[i], expected[i]);
            return 1;
        }
    /* Only static metadata Type 1 is sent. */
    hdr.metadata_type = 1;
    assert(dcp_hdr_data(data, &hdr) == -EINVAL);
    hdr.metadata_type = 0; hdr.hdmi_metadata_type1.metadata_type = 1;
    assert(dcp_hdr_data(data, &hdr) == -EINVAL);
    /* Transport error, AFK return code, short reply, DCP status at 0x40. */
    assert(dcp_hdr_reply(-ETIMEDOUT, 0, reply, sizeof(reply)) == -ETIMEDOUT);
    assert(dcp_hdr_reply(0, 5, reply, sizeof(reply)) == -EIO);
    assert(dcp_hdr_reply(0, 0, reply, 0x43) == -EPROTO);
    assert(dcp_hdr_reply(0, 0, reply, 0x44) == 0);
    put_unaligned_le32(0xe00002c2, reply + 0x40);
    assert(dcp_hdr_reply(0, 0, reply, sizeof(reply)) == -EIO);
    /* Ending HDR: the selector 0x14 stop message, DCP status at 0x70. */
    {
        u8 stop[DCP_HDR_STOP_SIZE], sreply[DCP_HDR_STOP_SIZE] = {0};

        assert(sizeof(expected_stop) == DCP_HDR_STOP_SIZE);
        memset(stop, 0xa5, sizeof(stop));
        dcp_hdr_stop_message(stop);
        assert(!memcmp(stop, expected_stop, sizeof(stop)));
        assert(dcp_hdr_stop_reply(-EIO, 0, sreply, sizeof(sreply)) == -EIO);
        assert(dcp_hdr_stop_reply(0, 3, sreply, sizeof(sreply)) == -EIO);
        assert(dcp_hdr_stop_reply(0, 0, sreply, 0x73) == -EPROTO);
        assert(dcp_hdr_stop_reply(0, 0, sreply, 0x74) == 0);
        /* the 0x15 status offset does not count for a stop */
        put_unaligned_le32(1, sreply + 0x40);
        assert(dcp_hdr_stop_reply(0, 0, sreply, sizeof(sreply)) == 0);
        put_unaligned_le32(1, sreply + 0x70);
        assert(dcp_hdr_stop_reply(0, 0, sreply, sizeof(sreply)) == -EIO);
    }
    printf("PASS: HDR10 static metadata message (%zu bytes), InfoFrame stop and reply checks\n", sizeof(msg));
    return 0;
}
"""
