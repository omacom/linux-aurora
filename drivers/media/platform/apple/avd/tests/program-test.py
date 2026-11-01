#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Compile the production H.264 emitters and open-program builder on a host.

Only allocation, controls, and buffer addresses are fixtures. No copied hardware
program, firmware payload, or device capture is used. Requires Linux UAPI headers.
The output is a codec-generated 1920x1088 IDR with two I slices, not a
pixel decoder. Run in a build slot: program-test.py OUTPUT_DIRECTORY.
"""
import os
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
struct avd_coded_fmt_ops { unsigned int (*pp_kicks)(struct avd_ctx *); };
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
static bool avd_backend_is_qfw(struct avd_dev *d) { return d->queued; }
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
 ctx.decoded_fmt.fmt.pix_mp.plane_fmt[0].bytesperline=1920;
 run.base.bufs.dst=&dst.base.vb;
 for(unsigned int c=0;c<8;c++) {
  ctx.priv=&h264[c];h264[c].nbanks=AVD_H264_BANKS;h264[c].b=&h264[c].bank[0];
  for(unsigned int b=0;b<AVD_H264_BANKS;b++) assert(!avd_h264_alloc_bufs(&ctx,&h264[c].bank[b]));
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
  for(unsigned int c=0;c<8;c++) for(unsigned int b=0;b<2;b++) {h264[c].bank[b].busy=false;h264[c].bank[b].submitted=false;}
  for (unsigned int idx=0; idx<variant.fifo_slots; idx++) {
   char path[4096]; FILE *out;
   ctx.priv=&h264[idx/2];struct avd_h264_ctx *hc=ctx.priv;
   assert(!avd_h264_pick_bank(&ctx,&dst.base.vb));
   assert(hc->b==&hc->bank[idx%AVD_H264_BANKS]);hc->b->active_slice=&slice;
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
    assert(readl(dev.ctrl+(v5?0x16c:0x10c))==7&&readl(dev.ctrl+(v5?0x190:0x120))==5);
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
 for(unsigned int c=0;c<8;c++) for(unsigned int b=0;b<AVD_H264_BANKS;b++) {
  struct avd_h264_bufs *buf=&h264[c].bank[b].bufs;
  free(buf->inst.cpu);free(buf->pipe_state.cpu);free(buf->above_info.cpu);
  free(buf->lf_above_info.cpu);free(buf->lf_above.cpu);free(buf->ip_above.cpu);free(buf->mv_above_info.cpu);
 }
 free(program.cpu);free(q.sram);free(dev.ctrl);free(dev.mbox);
 puts("PASS: production H.264 banks, IDR/P/B DMA operands, saved FIFO addresses and legacy EXECs");
}
'''


def main():
    out = Path(sys.argv[1]).resolve()
    out.mkdir(parents=True, exist_ok=True)
    names = ['avd-h264.c', 'avd-inst.h', 'avd-backend-open.c', 'avd-backend-queue.c', 'avd-hw.c']
    (out / 'inputs.sha256').write_text(''.join(hashlib.sha256((ROOT / n).read_bytes()).hexdigest()+'  '+n+'\n' for n in names))
    h264 = (ROOT / 'avd-h264.c').read_text().split('static int avd_h264_alloc_bufs(')[0]
    h264 = re.sub(r'^#include.*\n', '', h264, flags=re.M)
    inst = (ROOT / 'avd-inst.h').read_text()
    inst = re.sub(r'^#include.*\n', '', inst, flags=re.M)
    # Use the actual codec push/address/comp helpers, not fixture emitters.
    builder = function((ROOT / 'avd-backend-open.c').read_text(), 'avd_open_build_program')
    code = out / 'program.c'
    qfw = (ROOT / 'avd-backend-queue.c').read_text()
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
    code.write_text(PRELUDE + inst + h264 + function((ROOT / 'avd-h264.c').read_text(), 'avd_h264_alloc_bufs') + function((ROOT / 'avd-h264.c').read_text(), 'avd_h264_drop_bank_slices') + function((ROOT / 'avd-h264.c').read_text(), 'avd_h264_pick_bank') + builder + wire + regs + config +
                    function(qfw, 'avd_qfw_stream') + function(qfw, 'avd_qfw_submit') +
                    function(qfw, 'avd_qfw_fail_all') + function(qfw, 'avd_qfw_irq') + MAIN)
    exe = out / 'program-test'
    subprocess.run([os.environ.get('CC', 'cc'), '-std=gnu11', '-Wall', '-Wextra',
                    '-Werror', '-Wno-sign-compare', '-Wno-unused-function',
                    '-fsanitize=address,undefined', '-g', str(code), '-o', str(exe)], check=True)
    for variant in ('v3-t0', 'v3-t1', 'v5-t0', 'v5-t2'):
        target = out / variant
        target.mkdir(exist_ok=True)
        subprocess.run([str(exe), str(target), variant], check=True)
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
