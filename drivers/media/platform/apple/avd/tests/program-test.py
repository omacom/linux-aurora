#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Compile the production H.264 emitters and open-program builder on a host.

Only allocation, controls, and buffer addresses are fixtures. No copied hardware
program, firmware payload, or device capture is used. Requires Linux UAPI headers.
The output is a codec-generated 1920x1088 IDR with two I slices, not a
pixel decoder. Run in a build slot: program-test.py OUTPUT_DIRECTORY.
"""
import os
import json
import hashlib
from pathlib import Path
import re
import signal
import subprocess
import sys

from host_source import function

ROOT = Path(sys.argv[2]).resolve() if len(sys.argv) > 2 else Path(__file__).resolve().parent.parent




PRELUDE = r'''
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <linux/v4l2-controls.h>
typedef uint8_t u8;
typedef uint32_t u32;
typedef int32_t s32;
typedef uint64_t u64;
typedef uint64_t dma_addr_t;
#define AVD_H_
#define BIT(n) (1U << (n))
#define GENMASK(h,l) ((UINT32_MAX >> (31-(h))) & (UINT32_MAX << (l)))
#define FIELD_PREP(m,v) (((u32)(v) << __builtin_ctz(m)) & (m))
#define ARRAY_SIZE(a) (sizeof(a)/sizeof((a)[0]))
#define DIV_ROUND_UP(n,d) (((n)+(d)-1)/(d))
#define VISIBLE_IF_KUNIT
#define AVD_QUIRK_LSR 1
#define AVD_QUIRK_NO_PIPE_STATE 2
#define AVD_CODEC_HEVC 0
#define AVD_CODEC_H264 1
#define AVD_CODEC_VP9 2
#define AVD_MAX_INST 512
#define AVD_CODEC_AV1 3
#define AVD_MAX_FRAMES 16
#define AVD_IMG_FMT_420_10BIT 1
#define AVD_IMG_FMT_422_10BIT 2
#define AVD_FRAME_SUBMITTED 2
#define AVD_FRAME_DONE 3
#define IRQ_HANDLED 1
#define system_wq NULL
typedef int irqreturn_t;
#define dev_info_once(d,...) ((void)(d))
#define dev_warn_once(d,...) ((void)(d))
#define dev_warn_ratelimited(d,...) ((void)(d))
#define __iomem
#define USEC_PER_MSEC 1000
#define min_t(t,a,b) ((t)(a)<(t)(b)?(t)(a):(t)(b))
#define min3(a,b,c) ((a)<(b) ? ((a)<(c)?(a):(c)) : ((b)<(c)?(b):(c)))
#define wmb() ((void)0)
#define dev_dbg(d,...) ((void)(d))
#define dev_err_ratelimited(d,...) ((void)(d))
static unsigned long jiffies;
static unsigned long usecs_to_jiffies(unsigned long t) { return t/1000; }
static bool time_after(unsigned long a,unsigned long b) { return (long)(b-a)<0; }
static void usleep_range(unsigned int a,unsigned int b) { (void)a;(void)b;jiffies++; }
static u32 readl(void *p) { assert(!((uintptr_t)p&3));return *(u32 *)p; }
static void writel(u32 v,void *p) { assert(!((uintptr_t)p&3));*(u32 *)p=v; }
#define writel_relaxed writel
static void __iowrite32_copy(void *p,const void *s,size_t n) {
 const u32 *src=s;for(size_t i=0;i<n;i++)writel(src[i],(char *)p+i*4);
}
typedef unsigned long resource_size_t;
struct avd_qfw;
#define check_add_overflow(a,b,c) __builtin_add_overflow(a,b,c)
#define kvfree(p) free(p)
struct avd_buf { void *cpu; dma_addr_t addr; size_t size; };
struct avd_frame { unsigned int idx; struct avd_buf *prog; size_t prog_words;
                   dma_addr_t fifo_iova; u32 cmd[24], tag; unsigned long deadline; int state,status;
                   struct avd_ctx *ctx; };
struct avd_segment { u32 instructions[AVD_MAX_INST]; size_t num; };
struct avd_job { struct avd_segment *segments; unsigned int num, codec; };
struct avd_dev;
struct avd_variant { unsigned int quirks, revision, fifo_slots, vp_slots[4], vp_slot_base[4];
 void (*configure_stream)(struct avd_dev *,dma_addr_t,u8,u32); };
struct avd_dev { const struct avd_variant *variant; struct avd_frame *building; bool queued;
 struct avd_qfw *qfw; void *ctrl,*mbox,*dev; u32 capabilities;
 struct avd_frame frames[16]; int queue_work; };
struct vb2_buffer { struct { size_t length; } planes[1]; dma_addr_t dma; };
struct vb2_v4l2_buffer { struct vb2_buffer vb2_buf; };
struct v4l2_m2m_buffer { struct vb2_v4l2_buffer vb; };
struct avd_comp { size_t start_offset; u32 offsets[4]; };
struct avd_decoded_buffer { struct v4l2_m2m_buffer base; struct avd_comp comp; unsigned int bank; };
struct avd_ctx;
struct avd_coded_fmt_ops { unsigned int (*pp_kicks)(struct avd_ctx *);
 void (*vp_done)(struct avd_ctx *,struct vb2_v4l2_buffer *); };
struct fmt_desc { void *frame_ops; const struct avd_coded_fmt_ops *ops; };
struct avd_ctx { struct avd_dev *dev; void *priv; struct avd_job job; unsigned int image_fmt;
 const struct fmt_desc *coded_fmt_desc;
 struct { struct { struct { int q; } cap_q_ctx; } *m2m_ctx; } fh;
 struct { struct { struct { struct { u32 bytesperline; } plane_fmt[1]; } pix_mp; } fmt; } decoded_fmt;
 struct avd_comp comp; bool decomp; unsigned int fifo_idx, pp_total;
 dma_addr_t job_fifo; struct avd_buf inst;
};
struct avd_run { struct { struct vb2_v4l2_buffer *dst; } bufs;
 dma_addr_t comp_out, y_out, uv_out; };
struct dma_site { unsigned int segment,word,shift,width; dma_addr_t addr; };
static struct dma_site dma_sites[256];static unsigned int dma_count;
static void record_dma(struct avd_ctx *ctx,dma_addr_t addr,unsigned int shift,unsigned int width) {
 assert(addr && (addr>>shift) && dma_count<ARRAY_SIZE(dma_sites));
 dma_sites[dma_count++]=(struct dma_site){ctx->job.num,ctx->job.segments[ctx->job.num].num,shift,width,addr};
}
static bool avd_backend_is_qfw(const struct avd_dev *d) { return d->queued; }
static dma_addr_t next_iova=0x204000000ULL;
static int avd_buf_alloc(struct avd_dev *d, struct avd_buf *b, size_t n)
{ (void)d; free(b->cpu); b->cpu = calloc(1,n); b->size = n;
 b->addr=next_iova; next_iova+=(n+0xffff)&~0xffffULL; return b->cpu ? 0 : -ENOMEM; }
static void avd_buf_free(struct avd_dev *d, struct avd_buf *b) { (void)d;free(b->cpu);memset(b,0,sizeof(*b)); }
static struct avd_decoded_buffer *vb2_to_avd_decoded_buf(struct vb2_buffer *b)
{ return (void *)b; }
static struct avd_decoded_buffer reference;
static struct avd_decoded_buffer *avd_get_ref_buf(struct avd_ctx *c,
 struct vb2_v4l2_buffer *b, u64 t) { (void)c; (void)b; (void)t; return &reference; }
static dma_addr_t vb2_dma_contig_plane_dma_addr(struct vb2_buffer *b, int p)
{ (void)p; return b->dma; }
static struct vb2_buffer *vb2_find_buffer(void *q, u64 t)
{ (void)q; (void)t; return NULL; }
static unsigned int fmt_width(struct avd_ctx *c) { (void)c; return 1920; }
static unsigned int fmt_height(struct avd_ctx *c) { (void)c; return 1088; }
static void avd_qfw_debug(struct avd_dev *d,struct avd_frame *f,const char *w) { (void)d;(void)f;(void)w; }
static unsigned int pp_kicks(struct avd_ctx *c) { return c->pp_total; }
static u32 messages[16];static unsigned int msg_read,msg_count,completed,reset_requested;
static bool avd_mbox_pop(struct avd_dev *d,u32 *word) {
 (void)d;if(msg_read==msg_count)return false;*word=messages[msg_read++];return true;
}
static unsigned int vp_completed;
static void avd_frame_vp_complete(struct avd_dev *d,u32 tag) { assert(d->frames[tag&15].tag==tag);vp_completed++; }
static void avd_frame_complete(struct avd_dev *d,u32 tag,int status) {
 struct avd_frame *f=&d->frames[tag&15];assert(f->tag==tag&&f->state==AVD_FRAME_SUBMITTED);
 f->state=AVD_FRAME_DONE;f->status=status;completed++;
}
static void mod_delayed_work(void *w,void *v,int t) { (void)w;(void)v;(void)t;reset_requested++; }
'''

MAIN = r'''
int main(int argc, char **argv)
{
 struct avd_variant variant = { .revision=4, .quirks=AVD_QUIRK_LSR,
  .fifo_slots=15, .vp_slots={4,4,1,0}, .configure_stream=t8112_configure_stream };
 struct avd_buf program = {};
 struct avd_frame frame = { .prog=&program };
 struct avd_dev dev = { .variant=&variant, .building=&frame, .queued=true };
 struct avd_ctx ctx = { .dev=&dev, .job.codec=AVD_CODEC_H264 };
 typeof(*ctx.fh.m2m_ctx) m2m = {};
 struct avd_h264_ctx h264[8] = {};
 struct avd_qfw q = { .sram=calloc(1,0x10000), .aperture=0x1090000,
  .ring_words=0x1800, .fifo_slots=15 };
 static const struct avd_coded_fmt_ops ops={.pp_kicks=pp_kicks};
 static const struct fmt_desc fmt={.ops=&ops};
 dev.qfw=&q;dev.ctrl=calloc(1,0x10000);dev.mbox=calloc(1,0x100);dev.capabilities=7;
 ctx.coded_fmt_desc=&fmt;frame.ctx=&ctx;ctx.fh.m2m_ctx=&m2m;
 /* Optional VP notifications are enabled only by the matching feature bit. */
 u32 info[16]={AVDQ_MAGIC,2,16,64,0x1090000,0x8000,256,0,9,7,15,3,0,0x1800,0,0};
 memcpy(q.sram+AVDQ_INFO,info,sizeof(info));assert(avd_qfw_check(&dev)&&!q.vp_done);
 info[11]=7;memcpy(q.sram+AVDQ_INFO,info,sizeof(info));assert(avd_qfw_check(&dev)&&q.vp_done);

 u32 pool[8]={AVDQ_POOL_MAGIC,AVDQ_POOL_VERSION,15,240,256,0,0,0};
 info[11]=15;memcpy(q.sram+AVDQ_INFO,info,sizeof(info));
 memcpy(q.sram+AVDQ_POOL_OFFSET,pool,sizeof(pool));
 assert(avd_qfw_check(&dev)&&q.multi_vp&&q.vp_done);
 assert(avd_qfw_bank_limit(&q,AVD_CODEC_HEVC)==5&&avd_qfw_bank_limit(&q,AVD_CODEC_H264)==5);
 for(unsigned int n=0;n<8;n++) {
  assert(avd_qfw_pick_vp(&q,0,99)==n%4);
  assert(avd_qfw_pick_vp(&q,1,99)==4+n%4);
  assert(avd_qfw_pick_vp(&q,2,99)==8);
 }
 for(unsigned int n=0;n<8;n++) {
  u32 bad[8];memcpy(bad,pool,sizeof(bad));
  switch(n) {
   case 0:bad[0]=0;break;case 1:bad[1]++;break;
   case 2:bad[2]=0;break;case 3:bad[3]|=1;break;
   case 4:bad[3]|=1<<12;break;case 5:bad[5]=1<<9;break;
   case 6:bad[6]=8;break;case 7:bad[7]=1;break;
  }
  memcpy(q.sram+AVDQ_POOL_OFFSET,bad,sizeof(bad));
  assert(avd_qfw_check(&dev)&&!q.multi_vp&&q.vp_done);
  assert(avd_qfw_pick_vp(&q,1,4)==4&&avd_qfw_bank_limit(&q,1)==2);
 }
 memcpy(q.sram+AVDQ_POOL_OFFSET,pool,sizeof(pool));
 assert(avd_qfw_check(&dev)&&q.multi_vp);
 info[11]=7;memcpy(q.sram+AVDQ_INFO,info,sizeof(info));
 assert(avd_qfw_check(&dev)&&!q.multi_vp&&q.vp_done);
 assert(avd_qfw_pick_vp(&q,1,4)==4&&avd_qfw_bank_limit(&q,1)==2);
 info[1]=3;memcpy(q.sram+AVDQ_INFO,info,sizeof(info));assert(!avd_qfw_check(&dev)&&!q.vp_done&&!q.multi_vp);
 memset(q.sram,0,0x10000);
 struct avd_buf slice = { .size=4096, .addr=0x200800000ULL };
 struct v4l2_ctrl_h264_sps sps = { .pic_width_in_mbs_minus1=119,
  .pic_height_in_map_units_minus1=67, .chroma_format_idc=1,
  .flags=V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY | V4L2_H264_SPS_FLAG_DIRECT_8X8_INFERENCE };
 struct v4l2_ctrl_h264_decode_params decode = { .flags=V4L2_H264_DECODE_PARAM_FLAG_IDR_PIC };
 struct v4l2_ctrl_h264_pps pps = {};
 struct v4l2_ctrl_h264_pred_weights weights = {};
 struct avd_decoded_buffer dst = { .base.vb.vb2_buf.dma=0x203000000ULL };
 struct v4l2_ctrl_h264_slice_params sl = { .slice_type=V4L2_H264_SLICE_TYPE_I, .header_bit_size=32 };
 struct avd_h264_run run = { .sps=&sps, .pps=&pps, .decode_params=&decode,
  .slice_params=&sl, .slice_data_off=4, .pred_weights=&weights,
  .addresses.mv_color=0x203800000ULL, .num_valid=1,
  .base={ .comp_out=0x200100000ULL, .y_out=0x200200000ULL, .uv_out=0x200400000ULL } };
 assert(argc==3);
 bool v5=!strncmp(argv[2],"v5",2);
 if (v5) {
  variant.quirks=0;variant.configure_stream=t8122_configure_stream;
  if (!strcmp(argv[2],"v5-t0")) { variant.fifo_slots=7;variant.vp_slot_base[1]=4; }
 } else if (!strcmp(argv[2],"v3-t0")) variant.quirks|=AVD_QUIRK_NO_PIPE_STATE;
 q.fifo_slots=variant.fifo_slots;
 bool multi=getenv("AVD_MULTI_VP")!=NULL;
 q.multi_vp=multi;q.vp_pool[0]=15;q.vp_pool[1]=!strcmp(argv[2],"v5-t0")?16:240;
 unsigned int test_banks=multi?avd_qfw_bank_limit(&q,AVD_CODEC_H264):2;
 ctx.decoded_fmt.fmt.pix_mp.plane_fmt[0].bytesperline=1920;
 run.base.bufs.dst=&dst.base.vb;
 for(unsigned int c=0;c<8;c++) {
  ctx.priv=&h264[c];h264[c].nbanks=test_banks;h264[c].b=&h264[c].bank[0];
  for(unsigned int b=0;b<test_banks;b++) assert(!avd_h264_alloc_bufs(&ctx,&h264[c].bank[b]));
 }
 /* Fault injection proves every bank-1 DMA fixture is actually checked. */
 const char *zero=getenv("AVD_ZERO_BANK_ADDR");
 if(zero) {
#define ZERO(field) if(!strcmp(zero,#field)) h264[0].bank[1].bufs.field.addr=0
  ZERO(inst);ZERO(pipe_state);ZERO(above_info);ZERO(ip_above);
  ZERO(lf_above);ZERO(lf_above_info);ZERO(mv_above_info);
#undef ZERO
 }
 for (unsigned int mode=0; mode<2; mode++) {
  dev.queued=!mode;
  for(unsigned int c=0;c<8;c++) for(unsigned int b=0;b<test_banks;b++) {h264[c].bank[b].busy=false;h264[c].bank[b].submitted=false;}
  for (unsigned int idx=0; idx<variant.fifo_slots; idx++) {
   char path[4096]; FILE *out;
   ctx.priv=&h264[idx/test_banks];struct avd_h264_ctx *hc=ctx.priv;
   assert(!avd_h264_pick_bank(&ctx,&dst.base.vb));
   assert(hc->b==&hc->bank[idx%test_banks]);hc->b->active_slice=&slice;
   hc->b->submitted=true;
   ctx.job_fifo=hc->b->bufs.inst.addr;
   assert(ctx.job_fifo && hc->b->bufs.above_info.addr && hc->b->bufs.ip_above.addr &&
          hc->b->bufs.lf_above.addr && hc->b->bufs.lf_above_info.addr && hc->b->bufs.mv_above_info.addr);
   assert((variant.quirks&AVD_QUIRK_NO_PIPE_STATE)||hc->b->bufs.pipe_state.addr);
   frame.idx=idx; ctx.job.num=0;ctx.pp_total=1;
   dma_count=0;
   run.base.comp_out=0x230000000ULL+idx*0x1000000ULL;
   run.base.y_out=run.base.comp_out+0x100000;
   run.base.uv_out=run.base.comp_out+0x300000;
   run.addresses.mv_color=run.base.y_out+0x700000;
   reference.base.vb.vb2_buf.dma=idx?run.base.comp_out-0x1000000:run.base.comp_out;
   decode.flags=idx%3?0:V4L2_H264_DECODE_PARAM_FLAG_IDR_PIC;
   decode.dpb[0].flags=idx%3?V4L2_H264_DPB_ENTRY_FLAG_VALID:0;
   sl.slice_type=idx%3==0?V4L2_H264_SLICE_TYPE_I:idx%3==1?V4L2_H264_SLICE_TYPE_P:V4L2_H264_SLICE_TYPE_B;
   ctx.job.segments=calloc(3,sizeof(*ctx.job.segments)); assert(ctx.job.segments);
   stream_hdr(&ctx,&run); ctx.job.num=1;sl.first_mb_in_slice=0;slice.addr=0x200800000ULL;
   stream_slice(&ctx,&run);
   size_t continuation=1+ctx.job.segments[0].num+ctx.job.segments[1].num;
   ctx.job.num=2;sl.first_mb_in_slice=4080;slice.addr+=0x1000;
   stream_slice(&ctx,&run);
   size_t starts[3]={1,1+ctx.job.segments[0].num,2+ctx.job.segments[0].num+ctx.job.segments[1].num};
   assert(!avd_open_build_program(&ctx));
   u32 fifo=dev.queued ? idx<<4 : 0;
   assert(((u32 *)program.cpu)[0]==(0x2b000200U|fifo));
   assert(((u32 *)program.cpu)[continuation]==(0x2b000000U|fifo));
   assert(((u32 *)program.cpu)[frame.prog_words-1]==(0x2b000400U|fifo));
   if (dev.queued) {
    frame.tag=0x10|idx;assert(frame.fifo_iova==hc->b->bufs.inst.addr && !ctx.job_fifo);
    q.wp=0;memset(q.sram,0,0x10000);memset(dev.ctrl,0,0x10000);
    writel(1,dev.mbox+0x50);
    assert(avd_qfw_submit(&dev,&frame)==-EBUSY);
    unsigned int lo=v5?0x1d0:0x150;
    assert(!readl(q.sram+0x8000)&&!q.wp&&!readl(dev.ctrl+lo+idx*4));
    for(unsigned int other=0;other<variant.fifo_slots;other++)
     if(other!=idx)writel(0xabc000+other,dev.ctrl+lo+other*4);
    writel(0x20001,dev.mbox+0x50);
    ctx.fifo_idx=99;ctx.job.num=256;ctx.pp_total=256;
    /* Retry uses the saved FIFO and PP metadata. */
    assert(!avd_qfw_submit(&dev,&frame));
    for(unsigned int other=0;other<variant.fifo_slots;other++)
     if(other!=idx)assert(readl(dev.ctrl+lo+other*4)==0xabc000+other);
    assert(readl(dev.mbox+0x54)==0x1098000U+idx*64);
    assert(readl(q.sram+0x7ffc)==frame.prog_words);
    assert(!readl(q.sram+0x8000+idx*64+D_PP_EXTRA));
    assert(readl(dev.ctrl+lo+idx*4)==(v5?(u32)frame.fifo_iova:frame.fifo_iova>>8));
    unsigned int selected=readl(q.sram+0x8000+idx*64+D_VP);
    unsigned int expected=multi?(!strcmp(argv[2],"v5-t0")?4:4+idx%4):4;
    assert(selected==expected);
    assert(readl(dev.ctrl+(v5?0x15c:0xfc)+selected*4)==7&&readl(dev.ctrl+(v5?0x190:0x120))==5);
    snprintf(path,sizeof(path),"%s/mmio-%u.bin",argv[1],idx);
    out=fopen(path,"wb");assert(out);assert(fwrite(dev.ctrl,0x10000,1,out)==1);assert(!fclose(out));
    snprintf(path,sizeof(path),"%s/descriptor-%u.bin",argv[1],idx);
    out=fopen(path,"wb");assert(out);assert(fwrite(q.sram+0x8000+idx*64,64,1,out)==1);assert(!fclose(out));
    snprintf(path,sizeof(path),"%s/dma-sites-%u.txt",argv[1],idx);
    out=fopen(path,"w");assert(out);
    for(unsigned int a=0;a<dma_count;a++) {
     struct dma_site *site=&dma_sites[a];
     fprintf(out,"%zu %u %u %llx\n",starts[site->segment]+site->word,site->width,site->shift,(unsigned long long)site->addr);
    }
    assert(!fclose(out));
   }
   snprintf(path,sizeof(path),"%s/%s-%u.bin",argv[1],dev.queued?"h264":"legacy",idx);
   out=fopen(path,"wb"); assert(out);
   /* Explicit LE wire format, independent of the host byte order. */
   for (size_t i=0; i<frame.prog_words; i++) {
    u32 w=((u32 *)program.cpu)[i];
    for (unsigned int b=0;b<4;b++) fputc((w>>(8*b))&255,out);
   }
   assert(!fclose(out));
  }
  if (multi && dev.queued) {
   ctx.priv=&h264[0];
   for(unsigned int b=0;b<test_banks;b++)assert(h264[0].bank[b].busy&&h264[0].bank[b].submitted);
   assert(avd_h264_pick_bank(&ctx,&dst.base.vb)==-EBUSY);
  }
 }
 /* Both ring-position wrap and u32-counter wrap must split the I/O copy. */
 for (unsigned int test=0;test<2;test++) {
  u32 words[32];u32 start=test?UINT32_MAX-10:0x1800-8;
  for(unsigned int i=0;i<32;i++)words[i]=0x12340000U+i;
  memset(q.sram,0,0x10000);q.wp=start;writel(start,q.sram+0x7ff8);
  assert(!avd_qfw_stream(&dev,words,32));
  assert(q.wp==(u32)(start+32));
  for(unsigned int i=0;i<32;i++)assert(readl(q.sram+4*((u32)(start+i)%0x1800))==words[i]);
 }
 /* A sole fatal completion must keep DMA buffers owned until reset. */
 memset(q.sram,0,0x10000);jiffies=123;
 dev.frames[0].tag=0x10;dev.frames[0].state=AVD_FRAME_SUBMITTED;
 void *desc=q.sram+0x8000;
 writel(2,desc+D_STATE);writel(2,desc+D_STATUS);writel(0x10,desc+D_CTAG);
 messages[0]=0x54530010;msg_count=1;msg_read=0;
 assert(avd_qfw_irq(&dev)==IRQ_HANDLED);
 assert(!completed&&reset_requested&&dev.frames[0].state==AVD_FRAME_SUBMITTED);
 assert(dev.frames[0].deadline==jiffies);
 /* Drain both valid tagged words and reject a stale full tag. */
 dev.frames[1].tag=0x11;dev.frames[1].state=AVD_FRAME_SUBMITTED;
 writel(0,desc+D_STATUS);
 writel(2,desc+64+D_STATE);writel(0x11,desc+64+D_CTAG);
 messages[1]=0x54530011;msg_count=2;msg_read=0;
 avd_qfw_irq(&dev);assert(completed==2&&msg_read==2);
 dev.frames[0].state=AVD_FRAME_SUBMITTED;dev.frames[0].tag=0x20;
 writel(2,desc+D_STATE);messages[0]=0x54530010;msg_count=1;msg_read=0;
 avd_qfw_irq(&dev);assert(completed==2&&dev.frames[0].state==AVD_FRAME_SUBMITTED);
 /* A missing saved FIFO must fail before any ownership/MMIO changes, even
  * when the mutable context happens to contain another nonzero address. */
 dev.queued=true;frame.fifo_iova=0;ctx.inst.addr=0x208000000ULL;
 writel(0x20001,dev.mbox+0x50);u32 old_wp=q.wp;
 for(unsigned int idx=0;idx<variant.fifo_slots;idx++) {
  frame.idx=idx;assert(avd_qfw_submit(&dev,&frame)==-EINVAL && q.wp==old_wp);
 }
 /* VP records have their own full tag, independent of the PP ctag. */
 q.vp_done=true;dev.frames[0].tag=0x12345020;dev.frames[0].state=AVD_FRAME_SUBMITTED;
 writel(1,desc+D_RSVD1);writel(0x22345020,desc+D_RSVD0);writel(0x12345020,desc+D_CTAG);
 messages[0]=0x54545020;msg_count=1;msg_read=0;avd_qfw_irq(&dev);assert(!vp_completed);
 writel(0x12345020,desc+D_RSVD0);writel(0,desc+D_RSVD1);msg_read=0;avd_qfw_irq(&dev);assert(!vp_completed);
 writel(1,desc+D_RSVD1);msg_read=0;avd_qfw_irq(&dev);assert(vp_completed==1&&dev.frames[0].state==AVD_FRAME_SUBMITTED);
 messages[0]=0x54545030;msg_read=0;avd_qfw_irq(&dev);assert(vp_completed==1);
 for(unsigned int c=0;c<8;c++) for(unsigned int b=0;b<AVD_H264_BANKS;b++) {
  struct avd_h264_bufs *buf=&h264[c].bank[b].bufs;
  free(buf->inst.cpu);free(buf->pipe_state.cpu);free(buf->above_info.cpu);
  free(buf->lf_above_info.cpu);free(buf->lf_above.cpu);free(buf->ip_above.cpu);free(buf->mv_above_info.cpu);
 }
 free(program.cpu);free(q.sram);free(dev.ctrl);free(dev.mbox);
 puts("PASS: production H.264 banks, IDR/P/B DMA operands, saved FIFO addresses and legacy EXECs");
}
'''



def h264_prelude():
    # Keep the shared MMIO/allocation boundary stable for the VP9 suite.
    pre = PRELUDE.replace('static struct avd_decoded_buffer reference;',
                          'static struct avd_decoded_buffer references[16];\n#define reference references[0]')
    pre = pre.replace('return &reference;', 'return &references[t % 16];')
    return '#include <linux/videodev2.h>\n' + pre + '#define dev_err(d,...) ((void)(d))\n'


RUN_FIXTURE = r'''
#define dev_dbg_ratelimited(d,...) ((void)(d))
#define PAGE_SIZE 4096
#define PAGE_ALIGN(n) (((n)+4095)&~4095UL)
struct v4l2_h264_reflist_builder { unsigned int num_valid; s32 cur_pic_order_count; };
static void v4l2_h264_init_reflist_builder(struct v4l2_h264_reflist_builder *b,
 const struct v4l2_ctrl_h264_decode_params *d,const struct v4l2_ctrl_h264_sps *s,
 const struct v4l2_h264_dpb_entry *dpb) {
 (void)d;(void)s;b->num_valid=0;b->cur_pic_order_count=40;
 for(unsigned int i=0;i<16;i++) if(dpb[i].flags&V4L2_H264_DPB_ENTRY_FLAG_ACTIVE)b->num_valid++;
}
static void v4l2_h264_build_p_ref_list(void *b,void *r) { (void)b;(void)r; }
static void v4l2_h264_build_b_ref_lists(void *b,void *r,void *s) { (void)b;(void)r;(void)s; }
static struct avd_h264_run fixture_run;
static unsigned int postambles,held_finishes,submissions;
static bool valid_data=true;
static struct vb2_v4l2_buffer *v4l2_m2m_next_src_buf(void *m) { (void)m;return fixture_run.base.bufs.src; }
static size_t vb2_get_plane_payload(struct vb2_buffer *b,int p) { (void)b;(void)p;return valid_data?16:2; }
static void *vb2_plane_vaddr(struct vb2_buffer *b,int p) { (void)b;(void)p;static u8 data[16];return data; }
static void avd_h264_run_preamble(struct avd_ctx *ctx,struct avd_h264_run *r) { (void)ctx;*r=fixture_run; }
static void avd_run_postamble(struct avd_ctx *ctx,struct avd_run *r) { (void)ctx;(void)r;postambles++; }
static bool avd_h264_slice_fits(const struct v4l2_ctrl_h264_sps *s,size_t n,size_t b,u32 l,size_t z)
{ (void)s;(void)b;(void)l;(void)z;return n<20; }
static size_t avd_h264_max_slices(const struct v4l2_ctrl_h264_sps *s) { (void)s;return 20; }
static int avd_h264_realloc_slices(struct avd_ctx *c,size_t n) { (void)c;(void)n;abort(); }
static int avd_init_job(struct avd_ctx *c,unsigned int codec,size_t n) {
 free(c->job.segments);c->job.codec=codec;c->job.num=0;
 c->job.segments=calloc(n,sizeof(*c->job.segments));return c->job.segments?0:-ENOMEM;
}
static void avd_job_finish(struct avd_ctx *c,enum vb2_buffer_state s) { (void)c;assert(s==VB2_BUF_STATE_DONE);held_finishes++; }
static int avd_submit_job(struct avd_ctx *c) { submissions++;return avd_open_build_program(c); }
'''

RUN_MAIN = r'''
int main(void) {
 struct avd_variant variant={.revision=4,.quirks=AVD_QUIRK_LSR,.fifo_slots=15};
 struct avd_buf programs[2]={};
 struct avd_frame frames[2]={{.idx=0,.prog=&programs[0]},{.idx=1,.prog=&programs[1]}};
 struct avd_dev dev={.variant=&variant,.queued=true,.building=&frames[0]};
 static const struct avd_coded_fmt_ops ops={0};static const struct fmt_desc fmt={.ops=&ops};
 struct avd_h264_ctx hc={.nbanks=2};hc.b=&hc.bank[0];
 struct avd_ctx ctx={.dev=&dev,.priv=&hc,.coded_fmt_desc=&fmt};
 struct avd_decoded_buffer dst[2]={};struct vb2_v4l2_buffer src={0};
 struct v4l2_ctrl_h264_sps sps={.chroma_format_idc=1,.pic_width_in_mbs_minus1=119,
  .pic_height_in_map_units_minus1=67,.flags=V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY};
 struct v4l2_ctrl_h264_pps pps={0};struct v4l2_ctrl_h264_slice_params sl={.slice_type=V4L2_H264_SLICE_TYPE_I,.header_bit_size=32};
 struct v4l2_ctrl_h264_decode_params decode={0};struct v4l2_ctrl_h264_pred_weights weights={0};
 fixture_run=(struct avd_h264_run){.sps=&sps,.pps=&pps,.slice_params=&sl,.decode_params=&decode,
  .pred_weights=&weights,.base={.bufs={.src=&src,.dst=&dst[0].base.vb},
  .y_out=0x220000000,.uv_out=0x220100000,.comp_out=0x220200000},.addresses.mv_color=0x220300000};
 ctx.decoded_fmt.fmt.pix_mp.plane_fmt[0].bytesperline=1920;
 reference.base.vb.vb2_buf.dma=0x240000000;
 for(unsigned int b=0;b<2;b++) {
  assert(!avd_h264_alloc_bufs(&ctx,&hc.bank[b]));
  hc.bank[b].alloc_slice_num=20;hc.bank[b].slices=calloc(20,sizeof(struct avd_buf));
 }
 /* Applied SPS controls reject unequal depths for both codecs. */
 struct v4l2_ctrl_hevc_sps hevc={.chroma_format_idc=1,
  .pic_width_in_luma_samples=1920,.pic_height_in_luma_samples=1088};
 ctx.coded_fmt.fmt.pix_mp.width=1920;ctx.coded_fmt.fmt.pix_mp.height=1088;
 for(unsigned int luma=0;luma<5;luma++) for(unsigned int chroma=0;chroma<5;chroma++) {
  sps.bit_depth_luma_minus8=hevc.bit_depth_luma_minus8=luma;
  sps.bit_depth_chroma_minus8=hevc.bit_depth_chroma_minus8=chroma;
  int expected=luma==chroma&&(luma==0||luma==2)?0:-EINVAL;
  assert(avd_h264_validate_sps(&ctx,&sps)==expected);
  assert(avd_hevc_validate_sps(&ctx,&hevc)==expected);
 }
 sps.bit_depth_luma_minus8=sps.bit_depth_chroma_minus8=0;
 /* A VALID but inactive DPB entry must be included in the header's count. */
 decode.dpb[0].flags=V4L2_H264_DPB_ENTRY_FLAG_VALID|V4L2_H264_DPB_ENTRY_FLAG_ACTIVE;
 decode.dpb[7].flags=V4L2_H264_DPB_ENTRY_FLAG_VALID;
 decode.dpb[7].reference_ts=0;
 for(unsigned int i=0;i<16;i++)references[i].base.vb.vb2_buf.dma=0x240000000+i*0x100000;
 assert(!avd_h264_run(&ctx));assert(submissions==1&&hc.bank[0].submitted&&dst[0].bank==1);
 /* Compare the complete header to one built with the emitted VALID count. */
 struct avd_segment expected={0};struct avd_job saved=ctx.job;
 ctx.job=(struct avd_job){.segments=&expected};fixture_run.num_valid=2;fixture_run.cur_poc=40;
 stream_hdr(&ctx,&fixture_run);
 assert(!memcmp((u32 *)programs[0].cpu+1,expected.instructions,expected.num*sizeof(u32)));
 ctx.job=saved;
 /* Keep picture 0 DMA-owned while picture 1 holds and rejects a slice. */
 dev.building=&frames[1];fixture_run.base.bufs.dst=&dst[1].base.vb;
 src.flags=V4L2_BUF_FLAG_M2M_HOLD_CAPTURE_BUF;sl.first_mb_in_slice=0;dma_count=0;
 assert(!avd_h264_run(&ctx));assert(held_finishes==1&&hc.bank[1].busy&&!hc.bank[1].submitted);
 assert(hc.bank[0].busy&&hc.bank[0].submitted&&dst[0].bank==1);
 sl.first_mb_in_slice=33;sl.slice_type=V4L2_H264_SLICE_TYPE_SP;
 assert(avd_h264_run(&ctx)==-EINVAL);assert(!hc.bank[1].busy&&!hc.bank[1].slice_num&&!dst[1].bank&&!ctx.job.segments);
 assert(hc.bank[0].busy&&hc.bank[0].submitted&&hc.bank[0].slice_num==1);
 /* A missing-first continuation cannot borrow the submitted picture's bank. */
 sl.slice_type=V4L2_H264_SLICE_TYPE_I;assert(avd_h264_run(&ctx)==-EINVAL);
 /* Repeated/decreasing macroblock positions, FMO, unequal depths and bad data abort only the builder. */
 for(unsigned int failure=0;failure<7;failure++) {
  sl.first_mb_in_slice=0;dma_count=0;assert(!avd_h264_run(&ctx));
  sl.first_mb_in_slice=33;dma_count=0;assert(!avd_h264_run(&ctx));
  sl.first_mb_in_slice=failure==0?33:failure==1?16:failure==6?0:66;
  if(failure==2)pps.num_slice_groups_minus1=1;
  if(failure==3)sps.bit_depth_chroma_minus8=2;
  if(failure==4)valid_data=false;
  if(failure==5)sl.slice_type=V4L2_H264_SLICE_TYPE_SI;
  assert(avd_h264_run(&ctx)==-EINVAL);
  assert(!hc.bank[1].busy&&!hc.bank[1].slice_num&&!ctx.job.segments&&!dst[1].bank);
  assert(hc.bank[0].busy&&hc.bank[0].submitted&&dst[0].bank==1);
  pps.num_slice_groups_minus1=0;sps.bit_depth_chroma_minus8=0;valid_data=true;sl.slice_type=V4L2_H264_SLICE_TYPE_I;
 }
 /* Final slice submits once; retirement selects CAPTURE's bank, not hc.b. */
 sl.first_mb_in_slice=0;dma_count=0;assert(!avd_h264_run(&ctx));
 sl.first_mb_in_slice=33;src.flags=0;dma_count=0;assert(!avd_h264_run(&ctx));
 assert(submissions==2&&hc.bank[0].submitted&&hc.bank[1].submitted);
 assert(avd_h264_pick_bank(&ctx,&dst[1].base.vb)==-EBUSY);
 avd_h264_done(&ctx,&src,&dst[0].base.vb,VB2_BUF_STATE_DONE);
 assert(!hc.bank[0].busy&&hc.bank[1].busy&&hc.b==&hc.bank[1]);
 avd_h264_done(&ctx,&src,&dst[1].base.vb,VB2_BUF_STATE_DONE);assert(!hc.bank[1].busy);
 assert(postambles==27&&held_finishes==16&&submissions==2);
 /* The legacy transport collects the same held slices in its single bank. */
 dev.queued=false;hc.nbanks=1;dev.building=&frames[0];fixture_run.base.bufs.dst=&dst[0].base.vb;
 src.flags=V4L2_BUF_FLAG_M2M_HOLD_CAPTURE_BUF;sl.first_mb_in_slice=0;dma_count=0;
 assert(!avd_h264_run(&ctx));assert(hc.b==&hc.bank[0]&&!hc.bank[0].submitted);
 sl.first_mb_in_slice=33;dma_count=0;assert(!avd_h264_run(&ctx));assert(submissions==2);
 sl.first_mb_in_slice=66;src.flags=0;dma_count=0;assert(!avd_h264_run(&ctx));
 assert(submissions==3&&frames[0].fifo_iova==hc.bank[0].bufs.inst.addr&&!ctx.fifo_idx);
 avd_h264_done(&ctx,&src,&dst[0].base.vb,VB2_BUF_STATE_DONE);assert(!hc.bank[0].busy);
 for(unsigned int b=0;b<2;b++) {
  struct avd_h264_bufs *buf=&hc.bank[b].bufs;
  free(buf->inst.cpu);free(buf->pipe_state.cpu);free(buf->above_info.cpu);free(buf->ip_above.cpu);
  free(buf->lf_above.cpu);free(buf->lf_above_info.cpu);free(buf->mv_above_info.cpu);free(hc.bank[b].slices);free(programs[b].cpu);
 }
 puts("PASS: production H.264 prepare: VALID/inactive count, two-bank HOLD/rejection/final submission, PP retirement ownership");
}
'''

def prepare_regressions(out):
    source = (ROOT / 'avd-h264.c').read_text()
    h264 = re.sub(r'^#include.*\n', '', source.split('static int avd_h264_alloc_bufs(')[0], flags=re.M)
    inst = re.sub(r'^#include.*\n', '', (ROOT / 'avd-inst.h').read_text(), flags=re.M)
    helpers = re.sub(r'^#include.*\n', '', (ROOT / 'avd-h264-params.h').read_text(), flags=re.M)
    pre = h264_prelude().replace('struct vb2_v4l2_buffer *dst; } bufs;', 'struct vb2_v4l2_buffer *src,*dst; } bufs;')
    pre = pre.replace('struct { struct { struct { struct { u32 bytesperline; } plane_fmt[1]; } pix_mp; } fmt; } decoded_fmt;',
                      'struct { struct { struct v4l2_pix_format_mplane pix_mp; } fmt; } decoded_fmt,coded_fmt;')
    pre = pre.replace('struct vb2_buffer vb2_buf; };', 'struct vb2_buffer vb2_buf; unsigned int flags; };')
    pre = pre.replace('unsigned int bank; };', 'unsigned int bank; size_t grey_chroma_offset; bool grey_chroma_10bit; };')
    pre += 'enum vb2_buffer_state { VB2_BUF_STATE_DONE, VB2_BUF_STATE_ERROR };\n'
    code = pre + helpers + inst + h264
    for name in ('avd_h264_alloc_bufs', 'avd_h264_validate_sps', 'avd_h264_validate_pps',
                 'avd_h264_validate_slice', 'avd_h264_drop_bank_slices', 'avd_h264_pick_bank',
                 'avd_h264_abort_picture', 'avd_h264_refs_valid', 'avd_h264_done'):
        code += function(source, name)
    code += function((ROOT / 'avd-backend-open.c').read_text(), 'avd_open_build_program')
    code += function((ROOT / 'avd-hevc.c').read_text(), 'avd_hevc_validate_sps')
    code += RUN_FIXTURE + function(source, 'avd_h264_run') + RUN_MAIN
    dest = out / 'prepare.c'
    dest.write_text(code)
    exe = dest.with_suffix('')
    subprocess.run([os.environ.get('CC', 'cc'), '-std=gnu11', '-Wall', '-Wextra', '-Werror',
                    '-Wno-sign-compare', '-Wno-unused-function', '-fsanitize=address,undefined',
                    '-g', str(dest), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)


CONFORMANCE_MAIN = r'''

int main(void)
{
 struct avd_variant variant = { .revision = 4 };
 struct avd_dev dev = { .variant = &variant };
 struct avd_h264_ctx hc = {0};
 typeof(*((struct avd_ctx *)0)->fh.m2m_ctx) m2m = {0};
 struct avd_decoded_buffer destination = {0};
 struct avd_segment segments[2];
 struct avd_ctx ctx_storage = { .priv = &hc, .dev = &dev,
  .fh.m2m_ctx = &m2m, .job.segments = segments, .decomp = true };
 struct avd_ctx *ctx = &ctx_storage;
 struct v4l2_ctrl_h264_decode_params dec = {0};
 struct v4l2_ctrl_h264_sps sps = { .chroma_format_idc = 1,
  .pic_width_in_mbs_minus1 = 11, .pic_height_in_map_units_minus1 = 8,
  .flags = V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY };
 struct v4l2_ctrl_h264_pps pps = {0};
 struct v4l2_ctrl_h264_slice_params sl = {0};
 struct v4l2_ctrl_h264_scaling_matrix scaling = {0};
 struct v4l2_ctrl_h264_pred_weights weights = {0};
 struct avd_buf coded = { .addr = 0x123456780ULL, .size = 256 };
 struct avd_h264_run run = { .decode_params = &dec, .sps = &sps,
  .pps = &pps, .slice_params = &sl, .scaling_matrix = &scaling,
  .pred_weights = &weights, .cur_poc = 40, .slice_data_off = 7,
  .base.bufs.dst = &destination.base.vb,
  .base.y_out = 0x20000000, .base.uv_out = 0x20006c00,
  .base.comp_out = 0x20010000, .addresses.mv_color = 0x20020000 };
 hc.nbanks = 2; hc.b = &hc.bank[0]; hc.b->active_slice = &coded;
 hc.b->bufs.pipe_state.addr=0x10000000;
 hc.b->bufs.above_info.addr=0x10010000;
 hc.b->bufs.ip_above.addr=0x10020000;
 hc.b->bufs.lf_above.addr=0x10030000;
 hc.b->bufs.lf_above_info.addr=0x10040000;
 hc.b->bufs.mv_above_info.addr=0x10050000;
 ctx->decoded_fmt.fmt.pix_mp.plane_fmt[0].bytesperline = 192;
 for (unsigned int i = 0; i < 16; i++) {
  references[i].base.vb.vb2_buf.dma = 0x30000000 + i * 0x100000;
  references[i].base.vb.vb2_buf.planes[0].length = 0x30000;
  references[i].comp.start_offset = 0x10000;
  for (unsigned int j = 0; j < 4; j++)
   references[i].comp.offsets[j] = ctx->comp.offsets[j] = j * 0x1000;
 }
 /* Dense DPBs cover every reference-count, slice type and weight mode.
  * Sparse DPBs deliberately place references on either side of holes. */
 for (unsigned int mask_case = 0; mask_case < 18; mask_case++) {
  unsigned int mask = mask_case <= 16 ? (1U << mask_case) - 1 : 0x8429;
  unsigned int count = __builtin_popcount(mask), ids[16], n = 0;
  for (unsigned int i = 0; i < 16; i++) {
   dec.dpb[i].flags = mask & BIT(i) ?
    V4L2_H264_DPB_ENTRY_FLAG_VALID | V4L2_H264_DPB_ENTRY_FLAG_ACTIVE : 0;
   dec.dpb[i].reference_ts = i;
   dec.dpb[i].top_field_order_cnt = 2 * i;
   if (mask & BIT(i)) ids[n++] = i;
  }
  for (unsigned int rev = 0; rev < 4; rev++) {
   variant.revision = rev ? 4 : 3;
   variant.quirks = rev < 2 ? AVD_QUIRK_LSR | AVD_QUIRK_NO_PIPE_STATE :
                    rev == 2 ? AVD_QUIRK_LSR : 0;
   for (unsigned int type = 0; type < 3; type++) {
    if (type != V4L2_H264_SLICE_TYPE_I && !count) continue;
    for (unsigned int mode = 0; mode < 4; mode++) {
     for (unsigned int mb = 0; mb < 2; mb++) {
      pps.flags = mode & 1 ? V4L2_H264_PPS_FLAG_ENTROPY_CODING_MODE : 0;
      if (mode == 2) pps.flags |= V4L2_H264_PPS_FLAG_WEIGHTED_PRED;
      pps.weighted_bipred_idc = mode == 3 ? 2 : 0;
      sl.slice_type = type;
      sl.first_mb_in_slice = mb ? 33 : 0;
      sl.header_bit_size = 53;
      sl.cabac_init_idc = mode % 3;
      sl.slice_qp_delta = mode ? -2 : 3;
      sl.flags = mode & 1 ? V4L2_H264_SLICE_FLAG_DIRECT_SPATIAL_MV_PRED : 0;
      sl.num_ref_idx_l0_active_minus1 = count ? count - 1 : 0;
      sl.num_ref_idx_l1_active_minus1 = count > 1 ? 1 : 0;
      for (unsigned int i = 0; i < count; i++) {
       sl.ref_pic_list0[i].index = ids[i];
       sl.ref_pic_list1[i].index = ids[count - 1 - i];
       for (unsigned int j = 0; j < 2; j++) {
        weights.weight_factors[j].luma_weight[i] = i % 2 ? -1 : 1;
        weights.weight_factors[j].luma_offset[i] = i % 3;
        weights.weight_factors[j].chroma_weight[i][0] = 1;
        weights.weight_factors[j].chroma_weight[i][1] = 1;
       }
      }
      run.num_valid = count;
      dec.flags = count ? 0 : V4L2_H264_DECODE_PARAM_FLAG_IDR_PIC;
      ctx->job.num = 0; dma_count = 0;
      segments[0].num = segments[1].num = 0;
      stream_hdr(ctx, &run);
      ctx->job.num = 1;
      stream_slice(ctx, &run);
      printf("%u/%u/%u/%u/%u", mask_case, rev, type, mode, mb);
      for (unsigned int seg = 0; seg < 2; seg++)
       for (unsigned int i = 0; i < segments[seg].num; i++)
        printf(" %08x", segments[seg].instructions[i]);
      puts("");
      if (!count) {
       dec.flags = 0;
       ctx->job.num = 0; dma_count = 0;
       segments[0].num = segments[1].num = 0;
       stream_hdr(ctx, &run);
       ctx->job.num = 1;
       stream_slice(ctx, &run);
       printf("empty-intra/%u/%u/%u", rev, mode, mb);
       for (unsigned int seg = 0; seg < 2; seg++)
        for (unsigned int i = 0; i < segments[seg].num; i++)
         printf(" %08x", segments[seg].instructions[i]);
       puts("");
      }

     }
    }
   }
  }
 }
 for (unsigned int type = 0; type < 10; type++) {
  sl.slice_type = type;
  printf("guard/type/%u %d\n", type, VALIDATE_SLICE(ctx, &sl));
 }
 for (unsigned int groups = 0; groups < 8; groups++) {
  pps.num_slice_groups_minus1 = groups;
  printf("guard/groups/%u %d\n", groups, avd_h264_validate_pps(ctx, &pps));
 }
 if (!avd_h264_slice_order_supported(0, 0, 0) ||
     !avd_h264_slice_order_supported(1, 0, 33) ||
     !avd_h264_slice_order_supported(2, 33, 66) ||
     avd_h264_slice_order_supported(0, 0, 33) ||
     avd_h264_slice_order_supported(1, 33, 33) ||
     avd_h264_slice_order_supported(2, 66, 33) ||
     avd_h264_slice_order_supported(2, 66, 0)) return 4;
 /* Every possible DPB occupancy must map to dense header slots. */
 for (unsigned int mask = 0; mask < 65536; mask++) {
  unsigned int count = 0;
  for (unsigned int i = 0; i < 16; i++) {
   dec.dpb[i].flags = mask & BIT(i) ? V4L2_H264_DPB_ENTRY_FLAG_VALID : 0;
   if (mask & BIT(i)) {
    if (avd_h264_ref_slot(&dec, i) != count++) return 2;
   }
  }
  if (avd_h264_num_refs(&dec) != count) return 3;
 }
 return 0;
}

'''

def conformance(out):
    if os.environ.get('AVD_H264_BASELINE_ROOT'):
        baseline = Path(os.environ['AVD_H264_BASELINE_ROOT']).resolve()
    else:
        baseline = out / 'baseline'
        baseline.mkdir(exist_ok=True)
        ref = os.environ.get('AVD_H264_BASELINE_REF', '757fc07a6930ee12b04fe034138dc9880dc4ff3f')
        for name in ('avd-h264.c', 'avd-inst.h'):
            data = subprocess.check_output(['git', '-C', str(ROOT), 'show',
                                            f'{ref}:drivers/media/platform/apple/avd/{name}'])
            (baseline / name).write_bytes(data)
    (out / 'baseline-inputs.sha256').write_text(''.join(
        hashlib.sha256((baseline / name).read_bytes()).hexdigest() + '  ' + name + '\n'
        for name in ('avd-h264.c', 'avd-inst.h')))
    programs = {}
    for label, root in (('before', baseline), ('after', ROOT)):
        source = (root / 'avd-h264.c').read_text()
        h264 = source.split('static int avd_h264_alloc_bufs(')[0]
        h264 = re.sub(r'^#include.*\n', '', h264, flags=re.M)
        inst = re.sub(r'^#include.*\n', '', (root / 'avd-inst.h').read_text(), flags=re.M)
        # Helpers are exercised separately; the baseline emitters do not use them.
        helpers = re.sub(r'^#include.*\n', '', (ROOT / 'avd-h264-params.h').read_text(), flags=re.M)
        checks = function(source, 'avd_h264_validate_pps')
        if 'static int avd_h264_validate_slice(' in source:
            checks += function(source, 'avd_h264_validate_slice')
            checks += '#define VALIDATE_SLICE avd_h264_validate_slice\n'
        else:
            checks += '#define VALIDATE_SLICE(ctx, sl) 0\n'
        dest = out / ('conformance-' + label + '.c')
        dest.write_text(h264_prelude() + helpers + inst + h264 + checks + CONFORMANCE_MAIN)
        exe = dest.with_suffix('')
        subprocess.run([os.environ.get('CC', 'cc'), '-std=gnu11', '-O1', '-Wall', '-Wextra',
                        '-Werror', '-Wno-unused-function', '-Wno-unused-parameter', '-Wno-sign-compare',
                        '-fsanitize=address,undefined', '-g', str(dest), '-o', str(exe)], check=True)
        data = subprocess.check_output([str(exe)], text=True)
        programs[label] = dict(line.split(' ', 1) for line in data.splitlines())
        (out / (label + '.json')).write_text(json.dumps(programs[label], indent=2) + '\n')
    unchanged = sparse = normalized = 0
    for key, program in programs['after'].items():
        if key.startswith('guard/type/'):
            assert program == ('0' if int(key.rsplit('/', 1)[1]) < 3 else '-22'), key
        elif key.startswith('guard/groups/'):
            assert program == ('0' if key.endswith('/0') else '-22'), key
        elif key.startswith('empty-intra/'):
            _, revision, mode, mb = key.split('/')
            idr = f'0/{revision}/2/{mode}/{mb}'
            assert program == programs['before'][idr], key
            assert program == programs['after'][idr], key
            assert program != programs['before'][key], key
            normalized += 1
        elif not key.startswith('17/') or key.split('/')[2] == '2':
            assert program == programs['before'][key], key
            unchanged += 1
        else:
            before, after = programs['before'][key].split(), program.split()
            assert len(before) == len(after) and before != after, key
            for old, new in zip(before, after):
                if old != new:
                    assert int(old, 16) >> 20 == 0x2dc, (key, old, new)
                    assert int(old, 16) & ~15 == int(new, 16) & ~15, (key, old, new)
            sparse += 1
    print(f'PASS: {unchanged} complete legacy programs identical; {sparse} sparse programs change only reference slots; {normalized} empty-DPB non-IDR I programs equal legacy IDR; all 65536 DPB maps; syntax guards', flush=True)


def check_hevc_banks(out, qfw):
    hevc = (ROOT / 'avd-hevc.c').read_text()
    declarations = hevc[hevc.index('#define AVD_HEVC_BANKS'):hevc.index('static void stream_refs')]
    wire = qfw[qfw.index('#define AVDQ_MAGIC'):qfw.index('/* Arguments of dev_dbg()')]
    code = out / 'hevc-banks.c'
    code.write_text(PRELUDE + wire + declarations +
                    'enum vb2_buffer_state { VB2_BUF_STATE_DONE, VB2_BUF_STATE_ERROR };\n' +
                    function(qfw, 'avd_qfw_picture_banks') +
                    function(hevc, 'avd_hevc_pick_bank') +
                    function(hevc, 'avd_hevc_done') + r'''
int main(void) {
 struct avd_qfw q={.fifo_slots=15,.vp_pool={15,240,256,0}};
 struct avd_dev dev={.qfw=&q};struct avd_ctx ctx={.dev=&dev};
 struct avd_hevc_ctx hc={};ctx.priv=&hc;
 struct avd_decoded_buffer dst[AVD_MAX_FRAMES]={};
 for(unsigned int mode=0;mode<3;mode++) {
  dev.queued=mode!=0;q.multi_vp=mode==2;
  hc.nbanks=avd_qfw_picture_banks(&dev,AVD_CODEC_HEVC);
  assert(hc.nbanks==(mode==0?1:mode==1?2:5));
  for(unsigned int n=0;n<hc.nbanks;n++) {
   assert(!avd_hevc_pick_bank(&ctx,&dst[n]));
   assert(hc.b==&hc.bank[n]&&dst[n].bank==n+1&&hc.bank[n].busy);
  }
  assert(avd_hevc_pick_bank(&ctx,&dst[hc.nbanks])==-EBUSY);
  /* Completion selects CAPTURE's saved bank, not the current builder. */
  avd_hevc_done(&ctx,NULL,&dst[0].base.vb,VB2_BUF_STATE_DONE);
  assert(!hc.bank[0].busy&&!dst[0].bank);
  for(unsigned int n=1;n<hc.nbanks;n++)assert(hc.bank[n].busy);
  assert(!avd_hevc_pick_bank(&ctx,&dst[0])&&hc.b==&hc.bank[0]);
  for(unsigned int n=hc.nbanks;n>0;n--)
   avd_hevc_done(&ctx,NULL,&dst[n-1].base.vb,VB2_BUF_STATE_ERROR);
  for(unsigned int n=0;n<hc.nbanks;n++)assert(!hc.bank[n].busy&&!dst[n].bank);
 }
 puts("PASS: production HEVC pool-sized banks, old firmware/legacy fallback, exhaustion, saved-bank PP and error retirement");
}
''')
    exe = out / 'hevc-banks'
    subprocess.run([os.environ.get('CC', 'cc'), '-std=gnu11', '-Wall', '-Wextra',
                    '-Werror', '-Wno-unused-function', '-Wno-unused-parameter',
                    '-fsanitize=address,undefined', '-g', str(code), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)


def main():
    out = Path(sys.argv[1]).resolve()
    out.mkdir(parents=True, exist_ok=True)
    names = ['avd-h264-params.h', 'avd-h264.c', 'avd-hevc.c', 'avd-inst.h', 'avd-backend-open.c', 'avd-backend-queue.c', 'avd-hw.c']
    (out / 'inputs.sha256').write_text(''.join(hashlib.sha256((ROOT / n).read_bytes()).hexdigest()+'  '+n+'\n' for n in names))
    prepare_regressions(out)
    conformance(out)
    h264 = (ROOT / 'avd-h264.c').read_text().split('static int avd_h264_alloc_bufs(')[0]
    h264 = re.sub(r'^#include.*\n', '', h264, flags=re.M)
    inst = (ROOT / 'avd-inst.h').read_text()
    inst = re.sub(r'^#include.*\n', '', inst, flags=re.M)
    # Use the actual codec push/address/comp helpers, not fixture emitters.
    builder = function((ROOT / 'avd-backend-open.c').read_text(), 'avd_open_build_program')
    code = out / 'program.c'
    qfw = (ROOT / 'avd-backend-queue.c').read_text()
    check_hevc_banks(out, qfw)
    wire = qfw[qfw.index('#define AVDQ_MAGIC'):qfw.index('/* Arguments of dev_dbg()')]
    hw = (ROOT / 'avd-hw.c').read_text()
    regs = hw[hw.index('#define AVD_V3_'):hw.index('int avd_boot(')]
    config = '#define w32(r,v) writel((v),avd->ctrl+(r))\n#define m32(r,v) w32(r,(v)|readl(avd->ctrl+(r)))\n'
    config += function(hw, 't8112_configure_stream') + function(hw, 't8122_configure_stream')
    # Address emitters must never consume a zero bank/output/reference address.
    inst = inst.replace('static inline void push_address(struct avd_ctx *ctx, dma_addr_t addr)\n{',
                        'static inline void push_address(struct avd_ctx *ctx, dma_addr_t addr)\n{\n record_dma(ctx,addr,(ctx->dev->variant->quirks&AVD_QUIRK_LSR)?8:0,(ctx->dev->variant->quirks&AVD_QUIRK_LSR)?1:2);')
    inst = inst.replace('u32 offsets[4])\n{', 'u32 offsets[4])\n{\n assert(addr);')
    inst = inst.replace('push(ctx, (addr + offsets[i]) >> 7);', '{ record_dma(ctx,addr+offsets[i],7,1); push(ctx, (addr + offsets[i]) >> 7); }')
    helpers = re.sub(r'^#include.*\n', '', (ROOT / 'avd-h264-params.h').read_text(), flags=re.M)
    code.write_text(PRELUDE + helpers + inst + h264 + function((ROOT / 'avd-h264.c').read_text(), 'avd_h264_alloc_bufs') + function((ROOT / 'avd-h264.c').read_text(), 'avd_h264_drop_bank_slices') + function((ROOT / 'avd-h264.c').read_text(), 'avd_h264_pick_bank') + builder + wire + regs + config +
                    function(qfw, 'avd_qfw_check') + function(qfw, 'avd_qfw_stream') + function(qfw, 'avd_qfw_submit') +
                    function(qfw, 'avd_qfw_fail_all') + function(qfw, 'avd_qfw_irq') + MAIN)
    exe = out / 'program-test'
    subprocess.run([os.environ.get('CC', 'cc'), '-std=gnu11', '-Wall', '-Wextra',
                    '-Werror', '-Wno-sign-compare', '-Wno-unused-function',
                    '-fsanitize=address,undefined', '-g', str(code), '-o', str(exe)], check=True)
    for variant in ('v3-t0', 'v3-t1', 'v5-t0', 'v5-t2'):
        target = out / variant
        target.mkdir(exist_ok=True)
        subprocess.run([str(exe), str(target), variant], check=True)
        target = out / ('multi-' + variant)
        target.mkdir(exist_ok=True)
        subprocess.run([str(exe), str(target), variant], check=True,
                       env=dict(os.environ, AVD_MULTI_VP='1'))
    for field in ('inst', 'pipe_state', 'above_info', 'ip_above', 'lf_above', 'lf_above_info', 'mv_above_info'):
        target = out / ('zero-' + field)
        target.mkdir(exist_ok=True)
        with (target / 'expected-failure.log').open('w') as log:
            result = subprocess.run([str(exe), str(target), 'v3-t1'],
                                    env=dict(os.environ, AVD_ZERO_BANK_ADDR=field),
                                    stdout=log, stderr=subprocess.STDOUT)
        assert result.returncode == -signal.SIGABRT, (field, result.returncode)
        print('PASS: host test rejects zero bank-1 ' + field, flush=True)


if __name__ == '__main__':
    main()
