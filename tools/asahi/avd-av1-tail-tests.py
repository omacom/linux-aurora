#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""AV1 auxiliary alignment, probability and segmentation map inheritance, and reservation controls."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import resource
import subprocess

ROOT = Path(__file__).resolve().parents[2]
DRIVER = Path('drivers/media/platform/apple/avd')


def function(source, name):
    match = re.search(r'^(?:static )?(?:int|void|u64)\s+' + name + r'\(', source, re.M)
    if not match:
        raise ValueError(name)
    start = match.start()
    position = source.index('{', match.end()) + 1
    depth = 1
    while depth:
        depth += (source[position] == '{') - (source[position] == '}')
        position += 1
    return source[start:position] + '\n'


def structure(source, name):
    start = source.index('struct ' + name + ' {')
    end = source.index('\n};', start) + 3
    return source[start:end] + '\n'


PREFIX = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <stddef.h>
#include "controls.h"
typedef uint8_t u8;
typedef int8_t s8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
#define U64_MAX UINT64_MAX
typedef uint64_t dma_addr_t;
#define ALIGN(x,a) (((x)+(a)-1)&~((__typeof__(x))(a)-1))
#define ALIGN_DOWN(x,a) ((x)&~((__typeof__(x))(a)-1))
#define WARN_ON(x) (x)
#define DIV_ROUND_UP(n,d) (((n)+(d)-1)/(d))
#define dev_err_ratelimited(...) ((void)0)
@ALIGNMENT@
@CDF_HEADER@
@CDF_DEFAULTS@
struct vb2_buffer {struct {unsigned int length;} planes[1]; unsigned char *memory; dma_addr_t dma;};
struct vb2_v4l2_buffer {struct vb2_buffer vb2_buf;};
struct avd_decoded_buffer {
 struct {struct vb2_v4l2_buffer vb;} base;
 struct {unsigned int color_size; unsigned short width,height; size_t seg_size;} av1;
};
struct avd_buf {void *cpu; dma_addr_t addr;};
struct avd_run {struct {struct vb2_v4l2_buffer *src,*dst;} bufs; dma_addr_t y_out;};
@RUN_STRUCT@
@CTX_STRUCT@
struct v4l2_ctrl {unsigned int elems; struct {void *p;} p_cur;};
struct v4l2_pix_format_mplane {unsigned int width,height; struct {unsigned int sizeimage;} plane_fmt[1];};
struct avd_ctx {void *priv; int ctrl_hdl; struct {struct {struct {unsigned int width,height;} pix_mp;} fmt;} decoded_fmt;};
static struct avd_decoded_buffer *reference;
static struct v4l2_ctrl_av1_frame frame;
static struct v4l2_ctrl_av1_sequence sequence;
static struct v4l2_ctrl_av1_tile_group_entry tile;
static struct v4l2_ctrl_av1_film_grain grain;
static struct v4l2_ctrl controls[4];
static unsigned int lookup_calls, preamble_calls;
static void *vb2_plane_vaddr(struct vb2_buffer *b,unsigned int p){assert(p==0);return b->memory;}
static unsigned long vb2_get_plane_payload(struct vb2_buffer *b,unsigned int p){assert(p==0);return b->planes[0].length;}
static dma_addr_t vb2_dma_contig_plane_dma_addr(struct vb2_buffer *b,unsigned int p){assert(p==0);return b->dma;}
static struct avd_decoded_buffer *vb2_to_avd_decoded_buf(struct vb2_buffer *b){return (void *)b;}
static struct avd_decoded_buffer *avd_get_ref_buf(struct avd_ctx *ctx,struct vb2_v4l2_buffer *dst,uint64_t timestamp){(void)ctx;(void)dst;assert(timestamp==1000);lookup_calls++;return reference;}
static void avd_run_preamble(struct avd_ctx *ctx,struct avd_run *run){(void)ctx;(void)run;preamble_calls++;}
static struct v4l2_ctrl *v4l2_ctrl_find(int *h,unsigned int id){
 (void)h;
 if(id==V4L2_CID_STATELESS_AV1_SEQUENCE)return &controls[0];
 if(id==V4L2_CID_STATELESS_AV1_FRAME)return &controls[1];
 if(id==V4L2_CID_STATELESS_AV1_TILE_GROUP_ENTRY)return &controls[2];
 if(id==V4L2_CID_STATELESS_AV1_FILM_GRAIN)return &controls[3];
 return NULL;
}
'''

TESTS = r'''
static size_t aligned_cdf(size_t length,size_t color){return (length&~255UL)-((color+255)&~255UL)-9216;}
/* 3 bits per 4x4 block over 128x128 superblocks, 256-byte aligned. */
static size_t seg_bytes(size_t w,size_t h){return ALIGN(ALIGN(w,128)*ALIGN(h,128)/128*3,256);}
static void set_frame_ctx(struct avd_ctx *ctx){ctx->decoded_fmt.fmt.pix_mp.width=1920;ctx->decoded_fmt.fmt.pix_mp.height=1080;controls[0].p_cur.p=&sequence;controls[1].p_cur.p=&frame;controls[2].p_cur.p=&tile;controls[3].p_cur.p=&grain;}
static void read_exact(const char *path,void *data,size_t n){FILE *f=fopen(path,"rb");assert(f);assert(fread(data,1,n,f)==n);assert(fgetc(f)==EOF);assert(fclose(f)==0);}
static void inheritance(unsigned int residue,unsigned int mode,const char *neighbors,bool check_addresses){
 const size_t ref_len=6964992+residue,dst_len=7000064+(residue?((residue+79)%256):0);
 const size_t color=138240,ref_offset=aligned_cdf(ref_len,color),dst_offset=aligned_cdf(dst_len,color);
 struct avd_decoded_buffer src={0},dst={0};struct avd_av1_ctx av1={0};struct avd_ctx ctx={.priv=&av1};struct avd_av1_run run={0};set_frame_ctx(&ctx);
 struct avd_av1_cdfs expected={0};unsigned char probabilities[sizeof(expected)];
 src.base.vb.vb2_buf.planes[0].length=ref_len;src.base.vb.vb2_buf.memory=malloc(ref_len);assert(src.base.vb.vb2_buf.memory);
 dst.base.vb.vb2_buf.planes[0].length=dst_len;dst.base.vb.vb2_buf.memory=malloc(dst_len);assert(dst.base.vb.vb2_buf.memory);
 src.av1.color_size=color;dst.av1.color_size=color;reference=&src;
 memset(src.base.vb.vb2_buf.memory,0x55,ref_len);memset(dst.base.vb.vb2_buf.memory,0xa5,dst_len);memset(probabilities,0,sizeof(probabilities));
 avd_av1_default_coeff_probs(47,&expected);avd_av1_set_default_cdfs(&expected);
 if(neighbors){unsigned char observation[9728];read_exact(neighbors,observation,sizeof(observation));assert(residue==64);memcpy(src.base.vb.vb2_buf.memory+6817344,observation,sizeof(observation));memcpy(&expected,observation+192,sizeof(expected));}
 else {expected.intra_inter_cdf[0]^=37;expected.coeff_base_cdf[0][0][0][0]^=59;memcpy(src.base.vb.vb2_buf.memory+ref_offset,&expected,sizeof(expected));}
 memset(&frame,0,sizeof(frame));frame.frame_width_minus_1=1919;frame.frame_height_minus_1=1079;frame.quantization.base_q_idx=47;
 frame.frame_type=V4L2_AV1_INTER_FRAME;frame.primary_ref_frame=0;frame.ref_frame_idx[0]=0;frame.reference_frame_ts[0]=1000;
 if(mode){frame.frame_type=mode==1?V4L2_AV1_KEY_FRAME:mode==2?V4L2_AV1_INTRA_ONLY_FRAME:V4L2_AV1_INTER_FRAME;if(mode==3)frame.flags=V4L2_AV1_FRAME_FLAG_ERROR_RESILIENT_MODE;if(mode==4)frame.primary_ref_frame=7;memset(&expected,0,sizeof(expected));avd_av1_default_coeff_probs(47,&expected);avd_av1_set_default_cdfs(&expected);}
 controls[0].p_cur.p=&sequence;controls[1].p_cur.p=&frame;controls[2].p_cur.p=&tile;controls[3].p_cur.p=&grain;
 run.base.bufs.dst=&dst.base.vb;run.base.y_out=0x120000000UL;av1.bufs.probs.cpu=probabilities;lookup_calls=0;preamble_calls=0;
 assert(avd_av1_run_preamble(&ctx,&run)==0);assert(preamble_calls==1);
 if(check_addresses){assert(run.addresses.probs_out==run.base.y_out+dst_offset);assert(run.addresses.color==run.base.y_out+dst_offset+9216);assert(run.addresses.seg==run.base.y_out+dst_offset-seg_bytes(1920,1080));}
 avd_av1_set_prob(&ctx,&run);
 assert(lookup_calls==(mode?0:1));assert(memcmp(probabilities,&expected,sizeof(expected))==0);
 assert(memcmp(dst.base.vb.vb2_buf.memory+dst_offset,&expected,sizeof(expected))==0);
 for(size_t i=0;i<dst_offset;i++)assert(dst.base.vb.vb2_buf.memory[i]==0xa5);
 for(size_t i=dst_offset+sizeof(expected);i<dst_len;i++)assert(dst.base.vb.vb2_buf.memory[i]==0xa5);
 free(src.base.vb.vb2_buf.memory);free(dst.base.vb.vb2_buf.memory);
}
static void reservation(unsigned int width,unsigned int height,unsigned int compressed,unsigned int residue){
 struct v4l2_pix_format_mplane pix={.width=width,.height=height};struct avd_ctx ctx={0};pix.plane_fmt[0].sizeimage=compressed;
 avd_av1_adjust_decoded_fmt(&ctx,&pix);
 size_t min=pix.plane_fmt[0].sizeimage,color=avd_color_size(width,height),length=min+residue;
 size_t seg=seg_bytes(width,height);
 assert(min%256==0);assert(min==ALIGN(compressed,256)+9216+ALIGN(color,256)+seg);
 size_t cdf=AVD_AV1_CDFS_OFFSET(length,color),co=AVD_AV1_COLOR_OFFSET(length,color),sg=AVD_AV1_SEG_OFFSET(length,color,seg);
 assert(sg%256==0);assert(sg>=ALIGN(compressed,256));assert(sg+seg==cdf);
 assert(cdf%256==0&&co%256==0);assert(cdf>=compressed);assert(cdf+sizeof(struct avd_av1_cdfs)<=co);assert(co+color<=length);
 assert(cdf==aligned_cdf(length,color));assert(co==cdf+9216);
}
/* The preamble refuses frames larger than the format and out-of-range reference indices. */
static int preamble(unsigned int w,unsigned int h,unsigned char primary,signed char idx){
 struct avd_decoded_buffer dst={0};struct avd_av1_ctx av1={0};struct avd_ctx ctx={.priv=&av1};struct avd_av1_run run={0};set_frame_ctx(&ctx);
 dst.base.vb.vb2_buf.planes[0].length=7000064;memset(&frame,0,sizeof(frame));
 frame.frame_width_minus_1=w-1;frame.frame_height_minus_1=h-1;frame.primary_ref_frame=primary;if(primary<7)frame.ref_frame_idx[primary]=idx;
 run.base.bufs.dst=&dst.base.vb;run.base.y_out=0x120000000UL;return avd_av1_run_preamble(&ctx,&run);
}
static void guards(void){
 assert(preamble(1920,1080,0,0)==0);assert(preamble(1280,720,0,0)==0);
 assert(preamble(1921,1080,0,0)==-EINVAL);assert(preamble(1920,1081,0,0)==-EINVAL);
 assert(preamble(1920,1080,7,0)==0);assert(preamble(1920,1080,8,0)==-EINVAL);assert(preamble(1920,1080,255,0)==-EINVAL);
 assert(preamble(1920,1080,6,7)==0);assert(preamble(1920,1080,6,8)==-EINVAL);assert(preamble(1920,1080,0,-1)==-EINVAL);assert(preamble(1920,1080,3,-128)==-EINVAL);
}
/* Segmentation map kept with each frame: expect 0 zeroes it, 1 copies the reference's, 2 leaves the AVD's map. */
static void segmap(unsigned char flags,unsigned char primary,unsigned int ref_w,unsigned int ref_h,bool missing,int expect){
 const size_t len=7000064+79,ref_len=6964992+64+(ref_w*ref_h>1920*1080?65536:0),color=138240,seg=seg_bytes(1920,1080),ref_seg=seg_bytes(ref_w,ref_h);
 const size_t off=aligned_cdf(len,color)-seg,roff=aligned_cdf(ref_len,color)-ref_seg;const dma_addr_t none=0x140000000UL;
 struct avd_decoded_buffer src={0},dst={0};struct avd_av1_ctx av1={0};struct avd_ctx ctx={.priv=&av1};struct avd_av1_run run={0};set_frame_ctx(&ctx);
 dst.base.vb.vb2_buf.planes[0].length=len;dst.base.vb.vb2_buf.memory=malloc(len);dst.base.vb.vb2_buf.dma=0x120000000UL;assert(dst.base.vb.vb2_buf.memory);
 src.base.vb.vb2_buf.planes[0].length=ref_len;src.base.vb.vb2_buf.memory=malloc(ref_len);src.base.vb.vb2_buf.dma=0x130000000UL;assert(src.base.vb.vb2_buf.memory);
 dst.av1.color_size=color;dst.av1.seg_size=seg;src.av1.color_size=color;src.av1.seg_size=ref_seg;src.av1.width=ref_w-1;src.av1.height=ref_h-1;
 memset(dst.base.vb.vb2_buf.memory,0xa5,len);memset(src.base.vb.vb2_buf.memory,0x3c,ref_len);
 for(size_t i=0;i<ref_seg;i++)src.base.vb.vb2_buf.memory[roff+i]=(unsigned char)(i*7+1);
 reference=missing?&dst:&src;av1.bufs.no_prev_seg.addr=none;
 memset(&frame,0,sizeof(frame));frame.frame_width_minus_1=1919;frame.frame_height_minus_1=1079;frame.frame_type=V4L2_AV1_INTER_FRAME;
 frame.primary_ref_frame=primary;frame.ref_frame_idx[0]=0;frame.reference_frame_ts[0]=1000;frame.segmentation.flags=flags;
 run.base.bufs.dst=&dst.base.vb;run.base.y_out=dst.base.vb.vb2_buf.dma;
 assert(avd_av1_run_preamble(&ctx,&run)==0);assert(run.addresses.seg==run.base.y_out+off);
 lookup_calls=0;avd_av1_set_seg(&ctx,&run);assert(lookup_calls==(primary!=7));
 bool usable=primary!=7&&!missing&&(ref_w+7)/8==240&&(ref_h+7)/8==135;
 assert(run.addresses.prev_seg==(usable?src.base.vb.vb2_buf.dma+roff:none));
 for(size_t i=0;i<seg;i++){unsigned char b=dst.base.vb.vb2_buf.memory[off+i];assert(expect==0?b==0:expect==1?b==(unsigned char)(i*7+1):b==0xa5);}
 for(size_t i=0;i<off;i++)assert(dst.base.vb.vb2_buf.memory[i]==0xa5);
 for(size_t i=off+seg;i<len;i++)assert(dst.base.vb.vb2_buf.memory[i]==0xa5);
 for(size_t i=0;i<ref_len;i++)assert(src.base.vb.vb2_buf.memory[i]==(i>=roff&&i<roff+ref_seg?(unsigned char)((i-roff)*7+1):0x3c));
 free(src.base.vb.vb2_buf.memory);free(dst.base.vb.vb2_buf.memory);
}
static unsigned int segmaps(void){
 const unsigned char on=V4L2_AV1_SEGMENTATION_FLAG_ENABLED,update=V4L2_AV1_SEGMENTATION_FLAG_UPDATE_MAP;
 segmap(0,0,1920,1080,false,0);segmap(update,0,1920,1080,false,0);
 segmap(on,0,1920,1080,false,1);segmap(on,0,1916,1080,false,1);
 segmap(on,0,1928,1080,false,0);segmap(on,0,1920,1088,false,0);segmap(on,0,1280,720,false,0);
 segmap(on|update,0,1920,1080,false,2);segmap(on|update,0,1280,720,false,2);
 segmap(on,7,1920,1080,false,0);segmap(on,0,1920,1080,true,0);
 return 11;
}
int main(int argc,char **argv){
 assert(sizeof(struct avd_av1_cdfs)==9166);
 assert(seg_bytes(1920,1080)==51968&&seg_bytes(1281,721)==25344);
 if(argc>1&&(strcmp(argv[1],"snapshot")==0||strcmp(argv[1],"snapshot-copy")==0)){assert(argc==3);inheritance(64,0,argv[2],strcmp(argv[1],"snapshot")==0);puts("snapshot inheritance PASS");return 0;}
 if(argc>1&&strcmp(argv[1],"copy")==0){inheritance(64,0,NULL,false);puts("probability inheritance PASS");return 0;}
 if(argc>1&&strcmp(argv[1],"seed")==0){inheritance(64,1,NULL,false);puts("default destination seed PASS");return 0;}
 if(argc>1&&strcmp(argv[1],"aligned")==0){for(unsigned int mode=0;mode<=4;mode++)inheritance(0,mode,NULL,true);reservation(1920,1080,6425600,0);puts("aligned compatibility PASS");return 0;}
 unsigned int cases=0;
 guards();cases++;cases+=segmaps();
 for(unsigned int residue=0;residue<256;residue++){
  inheritance(residue,0,NULL,true);cases++;
  for(unsigned int mode=1;mode<=4;mode++){inheritance(residue,mode,NULL,true);cases++;}
  reservation(1920,1080,6425727,residue);cases++;reservation(1281,721,3000017,residue);cases++;
 }
 printf("%u production controls PASS\n",cases);return 0;
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=ROOT)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--neighbors', type=Path)
    parser.add_argument('--case', choices=['all', 'snapshot', 'snapshot-copy', 'copy', 'seed', 'aligned'], default='all')
    parser.add_argument('--mutation', choices=['round-up', 'cpu-unaligned', 'seg-cpu-unaligned', 'seg-any-size', 'ref-index-upper', 'frame-width-unchecked'])
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    paths = {name: args.source / DRIVER / name for name in ['avd-av1.c', 'avd-av1-entropymode.h', 'avd-av1-entropymode.c', 'avd.h']}
    text = {name: p.read_text() for name, p in paths.items()}
    src = text['avd-av1.c']
    macros = src[src.index('#define AVD_CDFS_SIZE'):src.index('struct avd_av1_run {')]
    functions = {n: function(src, n) for n in ['avd_av1_ref_ts', 'avd_av1_set_prob', 'avd_av1_set_seg', 'avd_av1_seg_size', 'avd_color_size', 'avd_av1_check_tiles', 'avd_av1_run_preamble', 'avd_av1_adjust_decoded_fmt']}
    unaligned = ('\n#define UNALIGNED_CDFS_OFFSET(dst,cl) ((dst)-ALIGN(cl,AVD_ALIGN)-ALIGN(AVD_CDFS_SIZE,AVD_ALIGN))\n'
                 '#define UNALIGNED_SEG_OFFSET(dst,cl,sl) (UNALIGNED_CDFS_OFFSET(dst,cl)-(sl))\n')
    # Each mutation names the extracted text it rewrites; a stale one is refused, never run as a no-op.
    mutations = {
        'round-up': (None, 'ALIGN_DOWN((dst), AVD_ALIGN)', 'ALIGN((dst), AVD_ALIGN)'),
        'cpu-unaligned': ('avd_av1_set_prob', 'AVD_AV1_CDFS_OFFSET(', 'UNALIGNED_CDFS_OFFSET('),
        'seg-cpu-unaligned': ('avd_av1_set_seg', 'AVD_AV1_SEG_OFFSET(dst->', 'UNALIGNED_SEG_OFFSET(dst->'),
        'seg-any-size': ('avd_av1_set_seg', 'DIV_ROUND_UP(ref->av1.width + 1, 8)', 'DIV_ROUND_UP(frame->frame_width_minus_1 + 1, 8)'),
        'ref-index-upper': ('avd_av1_run_preamble', 'idx < 0 || idx >= V4L2_AV1_TOTAL_REFS_PER_FRAME', 'idx < 0'),
        'frame-width-unchecked': ('avd_av1_run_preamble', 'run->frame->frame_width_minus_1 + 1 > ctx->decoded_fmt.fmt.pix_mp.width ||', '0 ||'),
    }
    if args.mutation:
        target, old, new = mutations[args.mutation]
        body = macros if target is None else functions[target]
        if old not in body:
            parser.error(args.mutation + ' mutation no longer matches the driver source')
        body = body.replace(old, new)
        if target is None:
            macros = body
        else:
            functions[target] = body
            macros += unaligned
    header = text['avd-av1-entropymode.h'].replace('#include <linux/types.h>', '')
    defaults = re.sub(r'^#include .*$', '', text['avd-av1-entropymode.c'], flags=re.M)
    alignment = re.search(r'^#define AVD_ALIGN\s+\d+', text['avd.h'], re.M).group()
    generated = PREFIX.replace('@ALIGNMENT@', alignment).replace('@CDF_HEADER@', header).replace('@CDF_DEFAULTS@', defaults).replace('@RUN_STRUCT@', structure(src, 'avd_av1_run')).replace('@CTX_STRUCT@', structure(src, 'avd_av1_ctx'))
    generated += macros + '\n' + '\n'.join(functions.values()) + TESTS
    uapi = args.source / 'include/uapi/linux/v4l2-controls.h'
    (args.out / 'controls.h').write_bytes(uapi.read_bytes())
    cfile = args.out / 'controls.c'
    cfile.write_text(generated)
    exe = args.out / 'controls'
    command = ['cc', '-std=gnu11', '-O1', '-g', '-fsanitize=address,undefined', '-fno-omit-frame-pointer', '-fno-pie', '-no-pie', '-Werror', '-Wno-unused-variable', '-Wno-unused-function', str(cfile), '-o', str(exe)]
    environment = dict(os.environ, TMPDIR=str(args.out.resolve()))
    build = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=environment)
    (args.out / 'build.log').write_text(build.stdout)
    if build.returncode:
        raise SystemExit(build.returncode)
    run_command = [str(exe)]
    if args.case != 'all':
        run_command.append(args.case)
    if args.case in ['snapshot', 'snapshot-copy']:
        if not args.neighbors:
            parser.error('snapshot cases require --neighbors')
        run_command.append(str(args.neighbors.resolve()))
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
    run = subprocess.run(run_command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    (args.out / 'run.log').write_text(run.stdout)
    receipt = {
        'scope': 'Extracted production AV1 probability copy, preamble address assignment and format reservation with exact CDF defaults. VB2 storage/lookup and control/preamble services are host fixtures; no firmware execution.',
        'source': str(args.source.resolve()), 'source_hashes': {name: hashlib.sha256(p.read_bytes()).hexdigest() for name, p in paths.items()},
        'uapi_sha256': hashlib.sha256(uapi.read_bytes()).hexdigest(),
        'extracted_sha256': {name: hashlib.sha256(body.encode()).hexdigest() for name, body in functions.items()},
        'offset_macros_sha256': hashlib.sha256(macros.encode()).hexdigest(),
        'generated_sha256': hashlib.sha256(cfile.read_bytes()).hexdigest(),
        'executable_sha256': hashlib.sha256(exe.read_bytes()).hexdigest(),
        'build_command': command, 'run_command': run_command, 'returncode': run.returncode,
        'stdout': run.stdout,
        'mutation': args.mutation,
    }
    if args.neighbors:
        receipt['private_external_neighbors_sha256'] = hashlib.sha256(args.neighbors.read_bytes()).hexdigest()
    (args.out / 'receipt.json').write_text(json.dumps(receipt, indent=2) + '\n')
    print(run.stdout, end='')
    raise SystemExit(run.returncode)


if __name__ == '__main__':
    main()
