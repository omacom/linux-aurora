#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Run the real Apple plane check/update with synthetic framebuffer objects.

The format descriptors, block helpers, surface structs and plane functions are
extracted from public kernel sources. GEM objects contain sizes, not DMA memory;
DRM atomic/clipping and device logging are stand-ins. This tests layout admission
and emitted geometry, not firmware packing, GPU rendering or DMA execution.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import subprocess
import tempfile

from common import block

FORMATS = ('XRGB8888', 'XRGB2101010', 'ABGR16161616F', 'NV12', 'NV16',
           'NV24', 'P010', 'P210', 'P030', 'P230')


def read(root, name, ref=None):
    if ref:
        return subprocess.check_output(['git', '-C', str(root), 'show', ref + ':' + name], text=True)
    path = root / name
    if path.exists():
        return path.read_text()
    return subprocess.check_output(['git', '-C', str(root), 'show', 'HEAD:' + name], text=True)


def unit(root, ref=None, mutation=None, callback_ref=None):
    plane = read(root, 'drivers/gpu/drm/apple/plane.c', ref)
    if callback_ref:
        old = read(root, 'drivers/gpu/drm/apple/plane.c', callback_ref)
        marker = 'static bool apple_plane_format_mod_supported('
        plane = plane.replace(block(plane, marker), block(old, marker), 1)
    if mutation:
        before, after = MUTATIONS[mutation]
        pattern = r'\s+'.join(re.escape(word) for word in before.split())
        plane, count = re.subn(pattern, lambda match: after, plane)
        if count != 1:
            raise RuntimeError(mutation + ': NOT_APPLIED')
    fourcc = read(root, 'drivers/gpu/drm/drm_fourcc.c', ref)
    header = read(root, 'include/drm/drm_fourcc.h', ref)
    uapi = read(root, 'include/uapi/drm/drm_fourcc.h', ref)
    surface = read(root, 'drivers/gpu/drm/apple/iomfb_plane.h', ref)
    plane_header = read(root, 'drivers/gpu/drm/apple/plane.h', ref)
    plane_core = read(root, 'drivers/gpu/drm/drm_plane.c', ref)
    dma_header = read(root, 'include/drm/drm_fb_dma_helper.h', ref)
    prefix = r'''
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
typedef uint8_t u8; typedef uint16_t u16; typedef uint32_t u32; typedef uint64_t u64;
#define U16_MAX UINT16_MAX
#define U32_MAX UINT32_MAX
#define BIT(n) (1UL << (n))
#define __packed __attribute__((packed))
#define check_mul_overflow(a,b,c) __builtin_mul_overflow((a),(b),(c))
#define check_add_overflow(a,b,c) __builtin_add_overflow((a),(b),(c))
#define DIV_ROUND_UP(n,d) (((n)+(d)-1)/(d))
#define DIV_ROUND_UP_ULL(n,d) (((u64)(n)+(d)-1)/(d))
#define ALIGN(n,a) (((n)+(a)-1)&~((a)-1))
#define roundup_pow_of_two(n) ((n) == 1 ? 1UL : 1UL << (64-__builtin_clzl((unsigned long)(n)-1)))
#define fourcc_code(a,b,c,d) ((u32)(a)|((u32)(b)<<8)|((u32)(c)<<16)|((u32)(d)<<24))
#define DRM_FORMAT_MAX_PLANES 4
#define DRM_FORMAT_MOD_INVALID UINT64_MAX
#define DRM_FORMAT_MOD_LINEAR 0
#define DRM_FORMAT_MOD_APPLE_INTERCHANGE 3
#define pr_warn(...) ((void)0)
#define dev_err_once(...) ((void)0)
#define dev_err_ratelimited(...) ((void)0)
#define IS_ERR(p) false
#define PTR_ERR(p) (-EINVAL)
static int warnings;
#define WARN_ON_ONCE(x) ({ bool _x = !!(x); warnings += _x; _x; })
#define container_of(p,t,m) ((t *)((char *)(p)-offsetof(t,m)))
'''
    prefix += '\n'.join(line for line in uapi.splitlines()
                        if re.match(r'#define DRM_FORMAT_\w+\s+fourcc_code\(', line)) + '\n'
    prefix += block(header, 'struct drm_format_info {') + ';\n'
    prefix += '\n'.join(block(fourcc, name) for name in (
        'unsigned int drm_format_info_block_width(',
        'unsigned int drm_format_info_block_height(',
        'unsigned int drm_format_info_bpp(')) + '\n'
    prefix += '\n'.join('static inline ' + block(header, name) for name in (
        'int drm_format_info_plane_width(', 'int drm_format_info_plane_height(')) + '\n'
    for name in FORMATS:
        row = re.search(r'\{\s*\.format\s*=\s*DRM_FORMAT_' + name + r'\s*,', fourcc)
        prefix += 'static const struct drm_format_info fmt_' + name + ' = ' + block(fourcc, row.start()) + ';\n'
    prefix += 'static inline const struct drm_format_info *drm_format_info(u32 format) {\n'
    for name in FORMATS:
        prefix += 'if (format == DRM_FORMAT_' + name + ') return &fmt_' + name + ';\n'
    prefix += 'return NULL; }\n'
    prefix += '\n'.join(line for line in surface.splitlines() if not line.startswith('#include'))
    prefix += r'''
struct drm_gem_object { size_t size; };
struct drm_gem_dma_object { struct drm_gem_object base; u64 dma_addr; };
struct drm_framebuffer { const struct drm_format_info *format; u32 width,height,pitches[4],offsets[4];u64 modifier;struct drm_gem_object *obj[4]; };
struct drm_rect { int x1,y1,x2,y2; };
enum drm_color_range { DRM_COLOR_YCBCR_LIMITED_RANGE, DRM_COLOR_YCBCR_FULL_RANGE };
enum drm_color_encoding { DRM_COLOR_YCBCR_BT601, DRM_COLOR_YCBCR_BT709, DRM_COLOR_YCBCR_BT2020 };
struct drm_crtc_state { struct { int hdisplay,vdisplay; } mode; bool vrr_enabled; };
struct drm_plane_state { u32 rotation,src_x,src_y,src_w,src_h; int crtc_x,crtc_y,crtc_w,crtc_h; void *crtc; bool visible; struct drm_framebuffer *fb; struct drm_rect src,dst; enum drm_color_range color_range; enum drm_color_encoding color_encoding; };
struct drm_plane;
struct drm_plane_funcs { bool (*format_mod_supported)(struct drm_plane *,u32,u64); };
struct drm_plane { int type; struct { u32 id; } base;struct drm_plane_state *state;u32 *format_types;unsigned int format_count;u64 *modifiers;unsigned int modifier_count;const struct drm_plane_funcs *funcs; };
struct drm_atomic_commit { struct { void *dev; } *dev; };
'''
    prefix += block(plane_header, 'struct apple_plane_state {') + ';\n'
    prefix += r'''
/* Layout controls use the legacy DCP; ext26 admission has its own suite. */
#define DRM_PLANE_TYPE_PRIMARY 1
#define DRM_MODE_ROTATE_0 1
struct platform_device { int unused; };
struct apple_dcp { int unused; };
struct apple_crtc { struct platform_device *dcp; };
static struct apple_crtc host_crtc;
static struct apple_crtc *to_apple_crtc(void *crtc) { return &host_crtc; }
static struct apple_dcp *platform_get_drvdata(struct platform_device *pdev) { return NULL; }
static bool dcp_is_ext26(struct apple_dcp *dcp) { return false; }
static bool iomfb_ext26_backing_fresh(struct apple_dcp *dcp,struct drm_framebuffer *fb) { return true; }
#define to_apple_plane_state(p) container_of(p,struct apple_plane_state,base)
static struct drm_crtc_state crtc_state = { .mode = { 3840,2160 } };
static struct drm_plane_state *drm_atomic_get_new_plane_state(struct drm_atomic_commit *c,struct drm_plane *p) { return p->state; }
static struct drm_crtc_state *drm_atomic_get_crtc_state(struct drm_atomic_commit *c,void *crtc) { return &crtc_state; }
static int drm_atomic_helper_check_plane_state(struct drm_plane_state *s,struct drm_crtc_state *c,int min,int max,bool pos,bool disable) { return 0; }
static int drm_rect_width(const struct drm_rect *r) { return r->x2-r->x1; }
static int drm_rect_height(const struct drm_rect *r) { return r->y2-r->y1; }
static void drm_rect_fp_to_int(struct drm_rect *dst,const struct drm_rect *src) { *dst = (struct drm_rect){src->x1>>16,src->y1>>16,src->x2>>16,src->y2>>16}; }
#define APPLE_GPU_CACHELINE 16
#define FRAC_16_16(mult,div) (((mult)<<16)/(div))
'''
    prototype = dma_header[dma_header.index('struct drm_gem_dma_object *drm_fb_dma_get_gem_obj('):].split(';', 1)[0]
    prefix += prototype + ' { return (void *)fb->obj[plane]; }\n'
    prefix += block(plane_core, 'bool drm_plane_has_format(') + '\n'
    if 'struct apple_plane_layout {' in plane:
        prefix += block(plane, 'struct apple_plane_layout {') + ';\n'
        prefix += '#define HAS_MOD_CALLBACK 1\n'
        prefix += block(plane, 'static int apple_plane_ix_bpp(') + '\n'
        prefix += block(plane, 'static bool apple_plane_format_mod_supported(') + '\n'
        prefix += block(plane, 'static int apple_get_ix_layout(') + '\n'
        prefix += block(plane, 'static int apple_plane_get_fb_layout(') + '\n'
    else:
        prefix += '#define HAS_MOD_CALLBACK 0\n'
        # The baseline uses a struct-returning helper. Keep it unmodified so
        # --ref can reproduce the check/update mismatch rather than a model.
        start = plane.index('static struct dcp_image_layout\napple_get_ix_layout')
        prefix += block(plane, start) + '\n'
    for name in ('static int apple_plane_atomic_check(',
                 'static struct dcp_rect drm_to_dcp_rect(',
                 'static struct dcp_rect drm_to_dcp_rect_fp(',
                 'static u32 drm_format_to_dcp(',
                 'static enum dcp_xfer_func get_xfer_func(',
                 'static enum dcp_colorspace get_colorspace(',
                 'static void apple_plane_atomic_update('):
        prefix += block(plane, name) + '\n'
    return prefix + MAIN


MAIN = r'''
static struct drm_gem_dma_object gem = { .base.size = SIZE_MAX, .dma_addr = 0x100000000 };
static struct drm_gem_dma_object other = { .base.size = SIZE_MAX };
static struct drm_framebuffer fb;
static struct apple_plane_state state;
static struct drm_plane plane = { .base.id = 1, .state = &state.base };
static struct drm_atomic_commit commit;
static u64 all_modifiers[]={DRM_FORMAT_MOD_APPLE_INTERCHANGE,DRM_FORMAT_MOD_LINEAR};
static u64 linear_modifier[]={DRM_FORMAT_MOD_LINEAR};
static u32 advertised_format;
static const struct drm_plane_funcs plane_funcs={
#if HAS_MOD_CALLBACK
 .format_mod_supported=apple_plane_format_mod_supported,
#endif
};
static void init(const struct drm_format_info *fmt,u32 w,u32 h,u64 modifier) {
 assert(drm_format_info(fmt->format));
 memset(&fb,0,sizeof(fb));memset(&state,0,sizeof(state));warnings=0;
 advertised_format=fmt->format;
 plane.format_types=&advertised_format;plane.format_count=1;
 plane.modifiers=all_modifiers;plane.modifier_count=2;plane.funcs=&plane_funcs;
 fb.format=fmt;fb.width=w;fb.height=h;fb.modifier=modifier;
 gem.base.size=SIZE_MAX;
 state.base=(struct drm_plane_state){.crtc=&crtc_state,.fb=&fb,.visible=true,.dst={0,0,64,64}};
 u64 next=128;
 for(u32 i=0;i<fmt->num_planes;i++) {
  u32 pw=drm_format_info_plane_width(fmt,w,i),ph=drm_format_info_plane_height(fmt,h,i);
  u32 bw=drm_format_info_block_width(fmt,i),bh=drm_format_info_block_height(fmt,i);
  fb.pitches[i]=ALIGN(DIV_ROUND_UP_ULL(pw, bw)*fmt->char_per_block[i],64);
  fb.offsets[i]=next;fb.obj[i]=&gem.base;
  if(modifier==DRM_FORMAT_MOD_LINEAR) next+=(u64)ph*fb.pitches[i]+64;
  else {
   u32 tw=fmt->is_yuv&&!i?32:16,th=tw;
   u64 data=DIV_ROUND_UP_ULL(pw,tw)*DIV_ROUND_UP_ULL(ph,th)*tw*th*fmt->char_per_block[i]/(bw*bh);
   u64 meta=DIV_ROUND_UP_ULL(roundup_pow_of_two(pw),tw)*DIV_ROUND_UP_ULL(roundup_pow_of_two(ph),th)*(tw==32?32:8);
   next+=ALIGN(data,16)+ALIGN(meta,16)+64;
  }
 }
 gem.base.size=next;
}
static int check(void) { return apple_plane_atomic_check(&plane,&commit); }
static void update(void) { apple_plane_atomic_update(&plane,&commit);assert(!warnings); }
static void reference_layout(void) {
 const struct drm_format_info *f=fb.format;u64 total=0;
 for(u32 i=0;i<f->num_planes;i++) {
  const struct dcp_plane_info *p=&state.surf.planes[i];
  u32 w=i?DIV_ROUND_UP(fb.width,f->hsub):fb.width,h=i?DIV_ROUND_UP(fb.height,f->vsub):fb.height;
  u32 bw=drm_format_info_block_width(f,i),bh=drm_format_info_block_height(f,i);
  assert(p->width==w&&p->height==h&&p->offset==fb.offsets[i]-fb.offsets[0]);
  u64 size;
  if(fb.modifier==DRM_FORMAT_MOD_LINEAR) {
   size=(u64)h*fb.pitches[i];assert(p->stride==fb.pitches[i]);
   assert(p->tile_w==bw&&p->tile_h==bh&&p->tile_size==bw*bh);
  } else {
   u32 tw=f->is_yuv&&!i?32:16,th=tw;
   u64 wt=DIV_ROUND_UP_ULL(w,tw),ht=DIV_ROUND_UP_ULL(h,th),tile=(u64)tw*th*f->char_per_block[i]/(bw*bh);
   u64 data=ALIGN(wt*ht*tile,16);
   u64 meta=ALIGN(DIV_ROUND_UP_ULL(roundup_pow_of_two(w),tw)*DIV_ROUND_UP_ULL(roundup_pow_of_two(h),th)*(tw==32?32:8),16);
   size=data+meta;
   assert(p->tile_w==tw&&p->tile_h==th&&p->tile_size==tile&&p->stride==wt*tile);
   const struct dcp_compression_info *c=&state.surf.compression_info[i];
   assert(c->metadata_offset==p->offset+data&&c->data_offset==p->offset);
   assert(c->row_stride==p->stride&&c->tiles_w==wt&&c->tiles_h==ht&&c->tile_bytes==tile);
  }
  assert(p->size==size);assert((u64)fb.offsets[i]+size<=gem.base.size);total+=size;
 }
 assert(state.surf.buf_size==total&&state.iova==gem.dma_addr+fb.offsets[0]);
}
int main(int argc,char **argv) {
 assert(argc==2);const char *test=argv[1];
 if(!strcmp(test,"matrix")) {
  const struct drm_format_info *formats[]={&fmt_XRGB8888,&fmt_XRGB2101010,&fmt_ABGR16161616F,&fmt_NV12,&fmt_NV16,&fmt_NV24,&fmt_P010,&fmt_P210,&fmt_P030,&fmt_P230};
  u32 dims[][2]={{1,1},{15,17},{16,16},{17,15},{31,33},{32,32},{33,31},{127,129},{128,128},{129,127},{1920,1080},{2560,1600},{3840,2160}};
  unsigned admitted=0,refused=0;
  for(unsigned f=0;f<sizeof(formats)/sizeof(*formats);f++)
   for(unsigned d=0;d<sizeof(dims)/sizeof(*dims);d++)
    for(unsigned linear=0;linear<2;linear++) {
     init(formats[f],dims[d][0],dims[d][1],linear?DRM_FORMAT_MOD_LINEAR:DRM_FORMAT_MOD_APPLE_INTERCHANGE);
     #if HAS_MOD_CALLBACK
     assert(drm_plane_has_format(&plane,formats[f]->format,fb.modifier)==(linear||!(f==8||f==9)));
#endif
     if(!linear&&(f==8||f==9)) { assert(check()==-EINVAL);refused++;continue; }
     assert(check()==0);update();reference_layout();admitted++;
    }
  printf("PASS matrix %u admitted / %u fractional interchange refused\n",admitted,refused);return 0;
 }
 if(!strcmp(test,"reproduce")) {
  init(&fmt_P030,33,33,DRM_FORMAT_MOD_APPLE_INTERCHANGE);fb.offsets[0]=0;fb.offsets[1]=2112;gem.base.size=4096;
  int ret=check();printf("check=%d ",ret);
  if(!ret) { update();printf("plane0-size=%u plane1-size=%u GEM=%zu\n",state.surf.planes[0].size,state.surf.planes[1].size,gem.base.size); }
  else puts("fractional interchange refused");return 0;
 }
 if(!strcmp(test,"format-advertisement")) {
  init(&fmt_P030,33,33,DRM_FORMAT_MOD_LINEAR);
  assert(!drm_plane_has_format(&plane,0,DRM_FORMAT_MOD_LINEAR));
  assert(!drm_plane_has_format(&plane,DRM_FORMAT_P030,0x42));
  assert(drm_plane_has_format(&plane,DRM_FORMAT_P030,DRM_FORMAT_MOD_LINEAR));
  assert(!drm_plane_has_format(&plane,DRM_FORMAT_P030,DRM_FORMAT_MOD_APPLE_INTERCHANGE));
  puts("PASS format/modifier advertisement through real DRM framework");return 0;
 }
 if(!strcmp(test,"linear-only-framework")) {
  init(&fmt_XRGB8888,64,64,DRM_FORMAT_MOD_LINEAR);
  plane.modifiers=linear_modifier;plane.modifier_count=1;
  assert(drm_plane_has_format(&plane,DRM_FORMAT_XRGB8888,DRM_FORMAT_MOD_LINEAR));
  assert(!drm_plane_has_format(&plane,DRM_FORMAT_XRGB8888,DRM_FORMAT_MOD_APPLE_INTERCHANGE));
  plane.modifier_count=0;
#if HAS_MOD_CALLBACK
  assert(!drm_plane_has_format(&plane,DRM_FORMAT_XRGB8888,DRM_FORMAT_MOD_LINEAR));
#endif
  puts("PASS linear-only plane cannot gain Interchange through callback");return 0;
 }
 init(&fmt_XRGB8888,33,33,DRM_FORMAT_MOD_APPLE_INTERCHANGE);
 if(!strcmp(test,"gem-short")) {
  assert(check()==0);update();gem.base.size=fb.offsets[0]+state.surf.planes[0].size-1;
 } else if(!strcmp(test,"gem-exact")) {
  assert(check()==0);update();gem.base.size=fb.offsets[0]+state.surf.planes[0].size;
  assert(check()==0);update();reference_layout();puts("PASS exact GEM boundary");return 0;
 } else if(!strcmp(test,"offset-wrap")) { fb.offsets[0]=U32_MAX-15;gem.base.size=U32_MAX;
 } else if(!strcmp(test,"relative-wrap")) {
  init(&fmt_NV12,64,64,DRM_FORMAT_MOD_LINEAR);fb.offsets[0]=0;fb.offsets[1]=U32_MAX-15;gem.base.size=(u64)U32_MAX*2;
 } else if(!strcmp(test,"backwards-offset")) {
  init(&fmt_NV12,64,64,DRM_FORMAT_MOD_LINEAR);fb.offsets[1]=fb.offsets[0]-1;
 } else if(!strcmp(test,"different-object")) { init(&fmt_NV12,64,64,DRM_FORMAT_MOD_LINEAR);fb.obj[1]=&other.base;
 } else if(!strcmp(test,"missing-object")) { init(&fmt_NV12,64,64,DRM_FORMAT_MOD_LINEAR);fb.obj[1]=NULL;
 } else if(!strcmp(test,"later-plane-short")) {
  init(&fmt_NV12,64,64,DRM_FORMAT_MOD_LINEAR);fb.offsets[1]=16384;gem.base.size=16384+32*64-1;
 } else if(!strcmp(test,"data-overflow")) { fb.width=65536;fb.height=65536;gem.base.size=SIZE_MAX;
 } else if(!strcmp(test,"stride-overflow")) { fb.width=0x40000000;fb.height=1;gem.base.size=SIZE_MAX;
 } else if(!strcmp(test,"meta-sum-overflow")) { fb.width=32752;fb.height=32768;gem.base.size=SIZE_MAX;
 } else if(!strcmp(test,"linear-overflow")) { init(&fmt_XRGB8888,64,64,DRM_FORMAT_MOD_LINEAR);fb.pitches[0]=0x10000000;gem.base.size=SIZE_MAX;
 } else if(!strcmp(test,"aggregate-overflow")) {
  init(&fmt_P010,64,1,DRM_FORMAT_MOD_LINEAR);fb.offsets[0]=fb.offsets[1]=0;fb.pitches[0]=fb.pitches[1]=0x80000000;gem.base.size=SIZE_MAX;
 } else if(!strcmp(test,"linear-short")) {
  init(&fmt_XRGB8888,33,33,DRM_FORMAT_MOD_LINEAR);gem.base.size=fb.offsets[0]+fb.height*fb.pitches[0]-1;
 } else if(!strcmp(test,"nonvisible-fractional")) { init(&fmt_P230,33,33,DRM_FORMAT_MOD_APPLE_INTERCHANGE);state.base.visible=false;
 } else if(!strcmp(test,"nonvisible-valid")) {
  state.base.visible=false;assert(check()==0);update();reference_layout();puts("PASS nonvisible valid geometry");return 0;
 } else if(!strcmp(test,"invisible-disable")) {
  state.base.crtc=NULL;state.base.fb=NULL;assert(check()==0);update();assert(!state.surf.buf_size);puts("PASS disabled plane");return 0;
 } else { fprintf(stderr,"unknown case %s\n",test);return 2; }
 assert(check()==-EINVAL);printf("PASS refusal %s\n",test);return 0;
}
'''

CASES = ('matrix', 'format-advertisement', 'linear-only-framework', 'gem-short', 'gem-exact', 'offset-wrap', 'relative-wrap',
         'backwards-offset', 'different-object', 'missing-object', 'later-plane-short',
         'data-overflow', 'stride-overflow', 'meta-sum-overflow', 'linear-overflow',
         'aggregate-overflow', 'linear-short', 'nonvisible-fractional',
         'nonvisible-valid', 'invisible-disable')
MUTATIONS = {
    'callback-ignores-plane-modifiers': ('if (i == plane->modifier_count)', 'if (i == plane->modifier_count && false)'),
    'fractional-allowed': ('bits % block_pixels)', 'false)'),
    'callback-advertises-fractional': ('if (apple_plane_ix_bpp(fmt, i, &bpp))', 'if (apple_plane_ix_bpp(fmt, i, &bpp) && false)'),
    'gem-bound-removed': ('(u64)fb->offsets[i] + p->size > obj->base.size ||', 'false ||'),
    'stride-overflow-ignored': ('check_mul_overflow(layout->width_tiles, layout->tile_bytes,\n\t\t\t       &out->stride)', '(out->stride = layout->width_tiles * layout->tile_bytes, false)'),
    'relative-overflow-ignored': ('check_add_overflow(p->offset, p->size, &end)', '(end = p->offset + p->size, (void)end, false)'),
    'aggregate-overflow-ignored': ('check_add_overflow(*buf_size, p->size, buf_size)', '(*buf_size += p->size, false)'),
    'data-overflow-ignored': ('check_mul_overflow(out->stride, layout->height_tiles, &bytes)', '(bytes = out->stride * layout->height_tiles, false)'),
    'meta-sum-overflow-ignored': ('check_add_overflow(layout->meta_offset, layout->meta_size,\n\t\t\t       &out->size)', '(out->size = layout->meta_offset + layout->meta_size, false)'),
    'linear-overflow-ignored': ('check_mul_overflow(p->height, fb->pitches[i], &p->size)', '(p->size = p->height * fb->pitches[i], false)'),
}


def run(label, text, output):
    path = output / label
    path.mkdir(parents=True, exist_ok=True)
    source = path / 'host.c'
    source.write_text(text)
    command = shlex.split(os.environ.get('HOSTCC', 'clang')) + [
        '-std=gnu23', '-Wall', '-Werror', '-O1', '-g', '-fsanitize=address,undefined',
        '-fno-omit-frame-pointer', str(source), '-o', str(path / 'host')]
    (path / 'command.json').write_text(json.dumps(command) + '\n')
    built = subprocess.run(command, text=True, capture_output=True, timeout=120)
    (path / 'build.log').write_text(built.stdout + built.stderr)
    result = {'class': 'NOBUILD' if built.returncode else 'RAN', 'source_sha256': hashlib.sha256(text.encode()).hexdigest(), 'cases': {}}
    if built.returncode:
        return result
    for case in (*CASES, 'reproduce'):
        ran = subprocess.run([str(path / 'host'), case], text=True, capture_output=True, timeout=30)
        (path / (case + '.log')).write_text(ran.stdout + ran.stderr)
        result['cases'][case] = ran.returncode
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=Path(__file__).resolve().parents[6])
    parser.add_argument('--output', type=Path, default=None)
    parser.add_argument('--ref', help='also reproduce admission on this old public source revision')
    parser.add_argument('--mutations', action='store_true')
    parser.add_argument('--callback-ref', help='also test this old callback on the fixed helper and real DRM framework')
    args = parser.parse_args()
    root = args.source.resolve()
    output = args.output or Path(tempfile.mkdtemp(prefix='apple-plane-layout-'))
    output.mkdir(parents=True, exist_ok=True)
    report = {'control': run('control', unit(root), output)}
    valid = report['control']['class'] == 'RAN' and not any(report['control']['cases'].values())
    if args.ref:
        report['baseline'] = run('baseline', unit(root, args.ref), output)
        valid &= report['baseline']['class'] == 'RAN' and any(report['baseline']['cases'].values())
    if args.callback_ref:
        report['old_callback'] = run('old-callback', unit(root, callback_ref=args.callback_ref), output)
        valid &= report['old_callback']['class'] == 'RAN' and report['old_callback']['cases'].get('linear-only-framework', 0) != 0
    if args.mutations:
        report['mutants'] = {}
        for name in MUTATIONS:
            result = run(name, unit(root, mutation=name), output)
            result['class'] = 'KILLED' if result['class'] == 'RAN' and any(result['cases'].values()) else result['class']
            report['mutants'][name] = result
            valid &= result['class'] == 'KILLED'
        report['after_control'] = run('after-control', unit(root), output)
        valid &= report['after_control']['class'] == 'RAN' and not any(report['after_control']['cases'].values())
    report['valid'] = bool(valid)
    (output / 'receipt.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({'valid': bool(valid), 'output': str(output), 'control': report['control']['class'],
                      'baseline': report.get('baseline', {}).get('class'),
                      'mutants': {k: v['class'] for k, v in report.get('mutants', {}).items()}}))
    return 0 if valid else 1


if __name__ == '__main__':
    raise SystemExit(main())
