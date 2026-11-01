#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Production AV1 bank/dependency/emitters, with explicit host DMA boundaries.

Run in a controller build slot: av1-test.py OUT [AVD_DIR [KERNEL_ROOT]].
AVD_AV1_BASELINE_REF optionally compares complete legacy programs/CDF/map bytes.
No decoder arithmetic or DMA visibility is modeled by these host fixtures.
"""
import hashlib
import importlib.util
import os
from pathlib import Path
import re
import subprocess
import sys

from host_source import function

HERE = Path(__file__).resolve().parent
AVD = Path(sys.argv[2]).resolve() if len(sys.argv) > 2 else HERE.parent
KERNEL = Path(sys.argv[3]).resolve() if len(sys.argv) > 3 else AVD.parents[4]
OUT = Path(sys.argv[1]).resolve()
OUT.mkdir(parents=True, exist_ok=True)
spec = importlib.util.spec_from_file_location('program_fixture', HERE / 'program-test.py')
fixture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fixture)
pre = fixture.PRELUDE
pre = pre.replace('typedef uint8_t u8;', 'typedef uint8_t u8; typedef uint16_t u16; typedef int8_t s8; typedef int16_t s16; typedef int64_t s64;')
pre = pre.replace('#define AVD_MAX_INST 512', '#define AVD_MAX_INST 512\n#define AVD_CODEC_HEVC 0')
pre = pre.replace('struct vb2_buffer { struct { size_t length; } planes[1]; dma_addr_t dma; };',
                  'struct vb2_buffer { struct { size_t length; } planes[1]; dma_addr_t dma; void *cpu; size_t payload; u64 timestamp; struct {void *req;} req_obj; };')
header = (AVD / 'avd.h').read_text()
a = header.index('struct avd_av1_decoded_buffer_info {')
b = header.index('\n};', a) + 3
pre = pre.replace('struct avd_decoded_buffer {', header[a:b] + '\nstruct avd_decoded_buffer {')
pre = pre.replace('struct avd_comp comp; unsigned int bank;', 'struct avd_comp comp; unsigned int bank; struct avd_av1_decoded_buffer_info av1;')
pre = pre.replace('struct vb2_v4l2_buffer *dst; } bufs;', 'struct vb2_v4l2_buffer *src,*dst; } bufs;')
pre = pre.replace('unsigned int fifo_idx, pp_total;', 'unsigned int fifo_idx, pp_total, max_frames; int ctrl_hdl;')
pre = pre.replace('dma_addr_t comp_out, y_out, uv_out;', 'dma_addr_t comp_out, y_out, uv_out, coded_in;')
pre = pre.replace('struct { u32 bytesperline; } plane_fmt[1];', 'u32 width,height; struct { u32 bytesperline; } plane_fmt[1];')
pre = pre.replace('static struct avd_decoded_buffer reference;', 'static struct avd_decoded_buffer reference; static struct avd_decoded_buffer *refs[5];')
pre = pre.replace('(void)c; (void)b; (void)t; return &reference;', '(void)c;for(unsigned int i=0;i<5;i++)if(refs[i]&&refs[i]->base.vb.vb2_buf.timestamp==t)return refs[i];return (void *)b;')
pre = pre.replace('static int avd_buf_alloc(struct avd_dev *d, struct avd_buf *b, size_t n)',
                  'static struct avd_ctx *allocating_ctx;\nstatic int fail_alloc_after;\nstatic bool avd_av1_ready(struct avd_ctx *);\nstatic int avd_buf_alloc(struct avd_dev *d, struct avd_buf *b, size_t n)')
pre = pre.replace('{ (void)d; free(b->cpu); b->cpu = calloc(1,n);',
                  '{ if(allocating_ctx){assert(!avd_av1_ready(allocating_ctx));if(fail_alloc_after&&!--fail_alloc_after)return -ENOMEM;} (void)d; free(b->cpu); b->cpu = calloc(1,n);')
pre += r'''
#define AVD_ALIGN 256
#define S16_MIN INT16_MIN
#define S16_MAX INT16_MAX
#define clamp_val(x,l,h) clamp(x,l,h)
#define U64_MAX UINT64_MAX
#define BIT_ULL(n) (1ULL<<(n))
#define ALIGN(x,a) (((x)+(a)-1)&~((__typeof__(x))(a)-1))
#define ALIGN_DOWN(x,a) ((x)&~((__typeof__(x))(a)-1))
#define max_t(t,a,b) ((t)(a)>(t)(b)?(t)(a):(t)(b))
#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#define clamp(x,l,h) ((x)<(l)?(l):(x)>(h)?(h):(x))
#define WARN_ON(v) (v)
#define GFP_KERNEL 0
#define smp_load_acquire(p) (*(p))
#define smp_store_release(p,v) (*(p)=(v))
#define READ_ONCE(x) (x)
#define WRITE_ONCE(x,v) ((x)=(v))
#define static_assert _Static_assert
#define VB2_BUF_STATE_DONE 0
#define VB2_BUF_STATE_ERROR 1
enum vb2_buffer_state { STATE_DUMMY };
#define kzalloc(n,f) calloc(1,n)
#define kfree free
static unsigned int avd_qfw_picture_banks(const struct avd_dev *, unsigned int);
static bool vp_feature=true;
static bool avd_qfw_has_vp_done(const struct avd_dev *d){return d->queued&&vp_feature;}
static void *vb2_plane_vaddr(struct vb2_buffer *b,int p){(void)p;return b->cpu;}
static struct vb2_v4l2_buffer *current_src,*current_dst;
static struct vb2_v4l2_buffer *v4l2_m2m_next_src_buf(void *m){(void)m;return current_src;}
struct v4l2_ctrl {unsigned int id; union {void *p;} p_cur;};
struct v4l2_ctrl_ref {struct v4l2_ctrl_ref *next; struct v4l2_ctrl *ctrl; bool p_req_valid; union {void *p;} p_req;};
struct v4l2_ctrl_handler {struct v4l2_ctrl_ref *ctrl_refs;};
#define list_for_each_entry(ref,head,node) for((ref)=*(head);(ref);(ref)=(ref)->next)
static struct v4l2_ctrl_av1_frame params,request_params;
static struct v4l2_ctrl_av1_sequence seq;
static struct v4l2_ctrl_av1_tile_group_entry tile;
static struct v4l2_ctrl_av1_film_grain grain;
static struct v4l2_ctrl controls[4];
static struct v4l2_ctrl_ref request_ref;
static struct v4l2_ctrl_handler request_hdl;
static unsigned int request_gets,request_puts;
static struct v4l2_ctrl_handler *v4l2_ctrl_request_hdl_find(void *req,void *h){(void)h;assert(req==&request_hdl);request_gets++;return &request_hdl;}
static void v4l2_ctrl_request_hdl_put(struct v4l2_ctrl_handler *h){assert(h==&request_hdl);request_puts++;}
static struct v4l2_ctrl *v4l2_ctrl_find(void *h,u32 id){
 (void)h;controls[0].p_cur.p=&seq;controls[1].p_cur.p=&params;controls[2].p_cur.p=&tile;controls[3].p_cur.p=&grain;
 if(id==V4L2_CID_STATELESS_AV1_SEQUENCE)return &controls[0];
 if(id==V4L2_CID_STATELESS_AV1_FRAME)return &controls[1];
 if(id==V4L2_CID_STATELESS_AV1_TILE_GROUP_ENTRY)return &controls[2];
 return &controls[3];
}
static void avd_run_preamble(struct avd_ctx *ctx,struct avd_run *run){
 (void)ctx;memset(run,0,sizeof(*run));run->bufs.src=current_src;run->bufs.dst=current_dst;
 run->coded_in=current_src->vb2_buf.dma;run->comp_out=current_dst->vb2_buf.dma;
 run->y_out=run->comp_out+0x100000;run->uv_out=run->comp_out+0x300000;
}
static void avd_run_postamble(struct avd_ctx *ctx,struct avd_run *run){(void)ctx;(void)run;}
static int avd_init_job(struct avd_ctx *ctx,int codec,size_t segments){
 free(ctx->job.segments);ctx->job.codec=codec;ctx->job.num=0;
 ctx->job.segments=calloc(segments,sizeof(*ctx->job.segments));return ctx->job.segments?0:-ENOMEM;
}
static int avd_submit_job(struct avd_ctx *ctx);
'''


def without_includes(s):
    return re.sub(r'^#include[^\n]*\n', '', s, flags=re.M)


av1 = (AVD / 'avd-av1.c').read_text()
common = pre + without_includes((AVD / 'avd-av1-entropymode.h').read_text())
common += without_includes((AVD / 'avd-av1-entropymode.c').read_text())
common += without_includes((AVD / 'avd-inst.h').read_text())
qfw = (AVD / 'avd-backend-queue.c').read_text()
hw = (AVD / 'avd-hw.c').read_text()
post = function((AVD / 'avd-backend-open.c').read_text(), 'avd_open_build_program')
post += qfw[qfw.index('#define AVDQ_MAGIC'):qfw.index('/* Arguments of dev_dbg()')]
post += hw[hw.index('#define AVD_V3_'):hw.index('int avd_boot(')]
post += '#define w32(r,v) writel((v),avd->ctrl+(r))\n#define m32(r,v) w32(r,(v)|readl(avd->ctrl+(r)))\n'
post += function(hw, 't8112_configure_stream') + function(hw, 't8122_configure_stream')
post += function(qfw, 'avd_qfw_picture_banks')
post += function(qfw, 'avd_qfw_stream') + function(qfw, 'avd_qfw_submit')
post += '\nstatic int avd_submit_job(struct avd_ctx *ctx){return avd_open_build_program(ctx);}\n'


def codec(s):
    return without_includes(s[s.index('#define V4L2_AV1_SEG_LVL_ALT_LF_Y_H'):s.index('static enum avd_image_fmt avd_av1_get_image_fmt')]) + function(s, 'avd_av1_pp_kicks')


MAIN = r'''
static void write_program(const char *dir,const char *name,struct avd_frame *f){
 char path[4096];snprintf(path,sizeof(path),"%s/%s.bin",dir,name);FILE *out=fopen(path,"wb");assert(out);
 assert(fwrite(f->prog->cpu,4,f->prog_words,out)==f->prog_words);fclose(out);
 struct avd_dev *dev=f->ctx->dev;struct avd_qfw *q=dev->qfw;
 q->wp=0;memset(q->sram,0,0x10000);f->tag=0x10|f->idx;assert(!avd_qfw_submit(dev,f));
 snprintf(path,sizeof(path),"%s/%s-desc.bin",dir,name);out=fopen(path,"wb");assert(out);
 assert(fwrite(q->sram+0x8000+f->idx*64,1,64,out)==64);fclose(out);
 snprintf(path,sizeof(path),"%s/%s-mmio.bin",dir,name);out=fopen(path,"wb");assert(out);
 assert(fwrite(dev->ctrl,1,0x10000,out)==0x10000);fclose(out);
}
static void queue_request(struct vb2_v4l2_buffer *src){
 request_params=params;controls[1].id=V4L2_CID_STATELESS_AV1_FRAME;
 request_ref=(struct v4l2_ctrl_ref){.ctrl=&controls[1],.p_req_valid=true,.p_req.p=&request_params};
 request_hdl.ctrl_refs=&request_ref;src->vb2_buf.req_obj.req=&request_hdl;
}
static void flag_table(struct avd_av1_bank *b){
 b->vp_done=true;
 /* Actual producer timestamp is slot 5, not the immediately prior slot 0. */
 struct v4l2_ctrl_av1_frame f={.frame_type=V4L2_AV1_INTER_FRAME,.primary_ref_frame=2,
  .frame_width_minus_1=1919,.frame_height_minus_1=1087};
 for(unsigned int i=0;i<7;i++)f.ref_frame_idx[i]=i;
 f.ref_frame_idx[2]=5;f.reference_frame_ts[5]=b->timestamp;
 for(unsigned int defaults=0;defaults<4;defaults++)
 for(unsigned int output_disabled=0;output_disabled<2;output_disabled++)
 for(unsigned int input_disabled=0;input_disabled<2;input_disabled++)
 for(unsigned int producer_seg=0;producer_seg<4;producer_seg++)
 for(unsigned int consumer_seg=0;consumer_seg<4;consumer_seg++)
 for(unsigned int refresh=0;refresh<256;refresh++){
  f.primary_ref_frame=defaults==1?7:2;
  f.frame_type=defaults==2?V4L2_AV1_INTRA_ONLY_FRAME:V4L2_AV1_INTER_FRAME;
  f.flags=(defaults==3?V4L2_AV1_FRAME_FLAG_ERROR_RESILIENT_MODE:0)|
   (output_disabled?V4L2_AV1_FRAME_FLAG_DISABLE_FRAME_END_UPDATE_CDF:0);
  f.refresh_frame_flags=refresh;
  f.segmentation.flags=(consumer_seg&1?V4L2_AV1_SEGMENTATION_FLAG_ENABLED:0)|
   (consumer_seg&2?V4L2_AV1_SEGMENTATION_FLAG_UPDATE_MAP:0);
  b->flags=input_disabled?V4L2_AV1_FRAME_FLAG_DISABLE_FRAME_END_UPDATE_CDF:0;
  b->seg_flags=(producer_seg&1?V4L2_AV1_SEGMENTATION_FLAG_ENABLED:0)|
   (producer_seg&2?V4L2_AV1_SEGMENTATION_FLAG_UPDATE_MAP:0);
  bool expect=defaults!=1&&((!defaults&&!input_disabled)||((consumer_seg&1)&&producer_seg==3));
  assert(avd_av1_depends(&f,b)==expect);
 }
 for(unsigned int primary=0;primary<7;primary++){
  f.primary_ref_frame=primary;f.flags=0;f.frame_type=V4L2_AV1_INTER_FRAME;f.segmentation.flags=0;
  b->flags=0;f.ref_frame_idx[primary]=5;assert(avd_av1_depends(&f,b));
  f.reference_frame_ts[5]++;assert(!avd_av1_depends(&f,b));f.reference_frame_ts[5]--;
 }
 f.primary_ref_frame=2;f.flags=V4L2_AV1_FRAME_FLAG_USE_REF_FRAME_MVS;f.reference_frame_ts[5]=b->timestamp;
 assert(avd_av1_depends(&f,b));f.primary_ref_frame=7;assert(avd_av1_depends(&f,b));
 f.reference_frame_ts[5]++;assert(!avd_av1_depends(&f,b));
 puts("PASS: AV1 flag table (primary, consumer/producer CDF flags, segmentation enabled/update_map, every refresh mask), actual slot and MV dependencies");
}
int main(int argc,char **argv){
 assert(argc==2);
 struct avd_variant variant={.revision=4,.quirks=0,.fifo_slots=15,.vp_slots={4,4,1,3}};
 struct avd_dev dev={.variant=&variant,.queued=true,.capabilities=15};
 struct avd_qfw queue={.sram=calloc(1,0x10000),.aperture=0x1094000,.ring_words=0x1800,.fifo_slots=15,.vp_done=true};
 dev.qfw=&queue;dev.ctrl=calloc(1,0x10000);dev.mbox=calloc(1,0x100);variant.configure_stream=t8122_configure_stream;
 writel(0x20001,dev.mbox+0x50);
 static const struct avd_coded_fmt_ops ops={.pp_kicks=avd_av1_pp_kicks,.vp_done=avd_av1_vp_done};
 static const struct fmt_desc fmt={.ops=&ops};
 struct avd_ctx ctx={.dev=&dev,.coded_fmt_desc=&fmt,.pp_total=1};
 struct avd_buf program={};struct avd_frame frame={.prog=&program,.ctx=&ctx};dev.building=&frame;
 struct avd_decoded_buffer dst[5]={};struct vb2_v4l2_buffer src={};u8 coded[512]={};
 src.vb2_buf=(struct vb2_buffer){.dma=0x220000000ULL,.cpu=coded,.payload=512,.timestamp=1000};current_src=&src;
 for(unsigned int i=0;i<5;i++){
  dst[i].base.vb.vb2_buf=(struct vb2_buffer){.dma=0x230000000ULL+i*0x1000000ULL,.cpu=calloc(1,0x800000),.planes[0].length=0x800000,.timestamp=1000+i};refs[i]=&dst[i];
 }
 ctx.decoded_fmt.fmt.pix_mp.width=1920;ctx.decoded_fmt.fmt.pix_mp.height=1088;
 ctx.decoded_fmt.fmt.pix_mp.plane_fmt[0].bytesperline=1920;
 assert(!avd_av1_ready(&ctx));allocating_ctx=&ctx;assert(!avd_av1_start(&ctx));allocating_ctx=NULL;
 struct avd_av1_stream *p=((struct avd_av1_ctx *)ctx.priv)->stream;assert(p->nbanks==2&&ctx.max_frames==2);
 assert(p->bank[0].codec.bufs.probs.addr!=p->bank[1].codec.bufs.probs.addr);
 seq=(struct v4l2_ctrl_av1_sequence){.bit_depth=8,.flags=V4L2_AV1_SEQUENCE_FLAG_SUBSAMPLING_X|V4L2_AV1_SEQUENCE_FLAG_SUBSAMPLING_Y};
 params=(struct v4l2_ctrl_av1_frame){.frame_type=V4L2_AV1_KEY_FRAME,.primary_ref_frame=7,
  .flags=V4L2_AV1_FRAME_FLAG_DISABLE_FRAME_END_UPDATE_CDF,.frame_width_minus_1=1919,.frame_height_minus_1=1087,.upscaled_width=1920};
 params.tile_info.tile_cols=params.tile_info.tile_rows=1;params.tile_info.width_in_sbs_minus_1[0]=29;params.tile_info.height_in_sbs_minus_1[0]=16;
 tile.tile_size=512;tile.tile_col=tile.tile_row=0;
 current_dst=&dst[0].base.vb;frame.idx=0;assert(avd_av1_ready(&ctx));assert(!avd_av1_run(&ctx));
 assert(dst[0].bank==1&&p->bank[0].busy&&p->bank[0].prepared);
 assert(frame.fifo_iova==p->bank[0].codec.bufs.inst.addr);
 write_program(argv[1],"av1-overlap-0",&frame);flag_table(&p->bank[0]);
 p->bank[0].flags=params.flags;p->bank[0].seg_flags=0;
 params.frame_type=V4L2_AV1_INTER_FRAME;params.primary_ref_frame=0;params.ref_frame_idx[0]=0;params.reference_frame_ts[0]=1000;
 params.segmentation.flags=V4L2_AV1_SEGMENTATION_FLAG_ENABLED;
 queue_request(&src);
 /* p_cur is deliberately stale and different from the pending request. */
 params.flags=0;request_params.flags=V4L2_AV1_FRAME_FLAG_DISABLE_FRAME_END_UPDATE_CDF;
 p->bank[0].vp_done=false;assert(!avd_av1_ready(&ctx));
 avd_av1_vp_done(&ctx,&dst[0].base.vb);assert(p->bank[0].busy&&avd_av1_ready(&ctx));
 avd_av1_vp_done(&ctx,&dst[0].base.vb);assert(p->bank[0].busy);
 p->bank[0].flags=0;assert(!avd_av1_ready(&ctx));p->bank[0].flags=V4L2_AV1_FRAME_FLAG_DISABLE_FRAME_END_UPDATE_CDF;
 p->bank[0].seg_flags=V4L2_AV1_SEGMENTATION_FLAG_ENABLED|V4L2_AV1_SEGMENTATION_FLAG_UPDATE_MAP;
 assert(!avd_av1_ready(&ctx));p->bank[0].seg_flags=0;
 request_params.flags|=V4L2_AV1_FRAME_FLAG_USE_REF_FRAME_MVS;assert(!avd_av1_ready(&ctx));request_params.flags&=~V4L2_AV1_FRAME_FLAG_USE_REF_FRAME_MVS;
 assert(avd_av1_ready(&ctx));params=request_params;
 /* CAPTURE tails are poisoned: successor must consume preparation snapshots. */
 size_t tail=AVD_AV1_CDFS_OFFSET(0x800000,dst[0].av1.color_size);
 memset((u8 *)dst[0].base.vb.vb2_buf.cpu+tail,0xee,sizeof(struct avd_av1_cdfs));
 memset(p->bank[0].codec.bufs.saved_seg.cpu,0x35,dst[0].av1.seg_size);
 current_dst=&dst[1].base.vb;src.vb2_buf.timestamp=1001;frame.idx=1;
 assert(!avd_av1_run(&ctx));assert(dst[1].bank==2&&p->bank[0].busy&&p->bank[1].busy);
 assert(!memcmp(p->bank[0].codec.bufs.probs.cpu,p->bank[1].codec.bufs.probs.cpu,sizeof(struct avd_av1_cdfs)));
 assert(!memcmp(p->bank[0].codec.bufs.saved_seg.cpu,p->bank[1].codec.bufs.prev_seg.cpu,dst[1].av1.seg_size));
 assert(p->bank[1].codec.bufs.prev_seg.addr!=p->bank[0].codec.bufs.saved_seg.addr);
 write_program(argv[1],"av1-overlap-1",&frame);assert(!avd_av1_ready(&ctx));
 current_dst=&dst[2].base.vb;assert(avd_av1_run(&ctx)==-EBUSY);assert(!dst[2].bank);
 avd_av1_done(&ctx,&src,&dst[0].base.vb,VB2_BUF_STATE_DONE);
 assert(!p->bank[0].busy&&p->bank[1].busy&&dst[1].bank==2);
 /* Producer bank reuse must not invalidate the consumer's VP map input. */
 memset(p->bank[0].codec.bufs.saved_seg.cpu,0x98,dst[0].av1.seg_size);
 assert(((u8 *)p->bank[1].codec.bufs.prev_seg.cpu)[0]==0x35);
 avd_av1_done(&ctx,&src,&dst[0].base.vb,VB2_BUF_STATE_DONE);assert(p->bank[1].busy);
 avd_av1_done(&ctx,&src,&dst[1].base.vb,VB2_BUF_STATE_ERROR);assert(!p->bank[1].busy&&avd_av1_ready(&ctx));
 assert(request_gets==request_puts);
 avd_av1_stop(&ctx);assert(!avd_av1_ready(&ctx));
 for(unsigned int mode=0;mode<2;mode++){
  dev.queued=mode;vp_feature=false;assert(!avd_av1_start(&ctx));p=((struct avd_av1_ctx *)ctx.priv)->stream;
  assert(p->nbanks==1&&ctx.max_frames==1);avd_av1_stop(&ctx);
 }
 dev.queued=true;vp_feature=true;
 for(int n=1;n<=6;n++){
  fail_alloc_after=n;allocating_ctx=&ctx;assert(avd_av1_start(&ctx)==-ENOMEM);allocating_ctx=NULL;
  assert(!ctx.priv&&!avd_av1_ready(&ctx));
 }

 /* AV1 pool size is firmware-owned, including sparse masks and FIFO bounds. */
 for(unsigned int count=1;count<=3;count++){
  queue.multi_vp=true;queue.vp_pool[3]=((1u<<count)-1)<<9;
  assert(!avd_av1_start(&ctx));p=((struct avd_av1_ctx *)ctx.priv)->stream;
  assert(p->nbanks==count+1&&ctx.max_frames==count+1);
  avd_av1_stop(&ctx);assert(ctx.max_frames==1);
 }
 queue.vp_pool[3]=BIT(9)|BIT(11);queue.fifo_slots=2;
 assert(!avd_av1_start(&ctx));p=((struct avd_av1_ctx *)ctx.priv)->stream;
 assert(p->nbanks==2);avd_av1_stop(&ctx);
 queue.fifo_slots=15;queue.vp_pool[3]=BIT(9)|BIT(10)|BIT(11);queue.next_vp[3]=0;
 assert(!avd_av1_start(&ctx));p=((struct avd_av1_ctx *)ctx.priv)->stream;
 assert(p->nbanks==4&&ctx.max_frames==4);
 params.primary_ref_frame=7;params.flags=V4L2_AV1_FRAME_FLAG_DISABLE_FRAME_END_UPDATE_CDF;
 params.segmentation.flags=0;
 for(unsigned int i=0;i<7;i++)params.ref_frame_idx[i]=-1;
 for(unsigned int n=0;n<4;n++){
  src.vb2_buf.timestamp=1000+n;current_dst=&dst[n].base.vb;frame.idx=n;
  queue_request(&src);assert(avd_av1_ready(&ctx));assert(!avd_av1_run(&ctx));
  char stem[64];snprintf(stem,sizeof(stem),"av1-multi-%u",n);write_program(argv[1],stem,&frame);
  assert(dst[n].bank==n+1&&p->bank[n].busy&&p->bank[n].prepared);
  assert(readl(queue.sram+0x8000+n*64+D_VP)==9+n%3);
  for(unsigned int prev=0;prev<n;prev++){
   assert(p->bank[prev].codec.bufs.inst.addr!=p->bank[n].codec.bufs.inst.addr);
   assert(p->bank[prev].codec.bufs.probs.addr!=p->bank[n].codec.bufs.probs.addr);
  }
 }
 assert(!avd_av1_ready(&ctx));current_dst=&dst[4].base.vb;
 assert(avd_av1_run(&ctx)==-EBUSY&&!dst[4].bank);
 /* Free a bank, then reference each of the other engines' live producers. */
 avd_av1_done(&ctx,&src,&dst[3].base.vb,VB2_BUF_STATE_DONE);
 for(unsigned int n=0;n<3;n++){
  request_params.primary_ref_frame=0;request_params.ref_frame_idx[0]=n;
  request_params.reference_frame_ts[n]=1000+n;
  p->bank[n].flags=V4L2_AV1_FRAME_FLAG_DISABLE_FRAME_END_UPDATE_CDF;
  assert(!avd_av1_ready(&ctx));avd_av1_vp_done(&ctx,&dst[n].base.vb);
  assert(p->bank[n].busy&&p->bank[n].vp_done&&avd_av1_ready(&ctx));
  p->bank[n].flags=0;assert(!avd_av1_ready(&ctx));
  p->bank[n].flags=V4L2_AV1_FRAME_FLAG_DISABLE_FRAME_END_UPDATE_CDF;
  assert(avd_av1_ready(&ctx));
  p->bank[n].seg_flags=V4L2_AV1_SEGMENTATION_FLAG_ENABLED|V4L2_AV1_SEGMENTATION_FLAG_UPDATE_MAP;
  request_params.segmentation.flags=V4L2_AV1_SEGMENTATION_FLAG_ENABLED;
  assert(!avd_av1_ready(&ctx));request_params.segmentation.flags=0;p->bank[n].seg_flags=0;
  request_params.flags|=V4L2_AV1_FRAME_FLAG_USE_REF_FRAME_MVS;
  assert(!avd_av1_ready(&ctx));request_params.flags&=~V4L2_AV1_FRAME_FLAG_USE_REF_FRAME_MVS;
 }
 /* Reset/error retirement must release the saved bank, irrespective of builder. */
 for(unsigned int n=3;n>0;n--){
  avd_av1_done(&ctx,&src,&dst[n-1].base.vb,VB2_BUF_STATE_ERROR);
  assert(!p->bank[n-1].busy&&!dst[n-1].bank);
 }
 assert(avd_av1_ready(&ctx));
 params.primary_ref_frame=7;params.segmentation.flags=0;
 current_dst=&dst[0].base.vb;frame.idx=0;assert(!avd_av1_run(&ctx));
 assert(dst[0].bank==1&&p->bank[0].busy);
 avd_av1_done(&ctx,&src,&dst[0].base.vb,VB2_BUF_STATE_DONE);avd_av1_stop(&ctx);
 /* Fail allocation in every bank, including after earlier banks succeeded. */
 for(int n=1;n<=12;n++){
  fail_alloc_after=n;allocating_ctx=&ctx;assert(avd_av1_start(&ctx)==-ENOMEM);allocating_ctx=NULL;
  assert(!ctx.priv&&!avd_av1_ready(&ctx));
 }
 puts("PASS: AV1 firmware pool sizes, sparse masks, FIFO bounds, per-engine dispatch, all producer barriers, exhaustion, reset/error and reuse");
 for(unsigned int i=0;i<5;i++)free(dst[i].base.vb.vb2_buf.cpu);
 free(program.cpu);free(queue.sram);free(dev.ctrl);free(dev.mbox);
 puts("PASS: AV1 snapshot inheritance, poisoned CAPTURE, request lookahead, banks, PP retirement, exhaustion, failures and legacy fallback");
}
'''


def build_run(name, source):
    (OUT / f'{name}.c').write_text(source)
    cmd = [os.environ.get('CC', 'cc'), '-std=gnu11', '-g', '-O1', '-Wall', '-Wextra', '-Werror',
           '-Wno-unused-function', '-Wno-unused-variable', '-Wno-unused-parameter', '-Wno-sign-compare', '-Wno-pointer-sign',
           '-fsanitize=address,undefined', '-fno-omit-frame-pointer', str(OUT / f'{name}.c'), '-o', str(OUT / name)]
    subprocess.run(cmd, check=True)
    subprocess.run([str(OUT / name), str(OUT)], check=True)


build_run('av1-host', common + codec(av1) + post + MAIN)
(OUT / 'source.sha256').write_text('\n'.join(hashlib.sha256(p.read_bytes()).hexdigest() + '  ' + str(p) for p in (
    AVD / 'avd-av1.c', AVD / 'avd-av1-entropymode.c', AVD / 'avd-backend-open.c', AVD / 'avd-backend-queue.c')) + '\n')

base_ref = os.environ.get('AVD_AV1_BASELINE_REF')
if base_ref:
    old = subprocess.check_output(['git', '-C', str(KERNEL), 'show',
                                   base_ref + ':drivers/media/platform/apple/avd/avd-av1.c'], text=True)
    legacy = r'''
int main(int argc,char **argv){
 assert(argc==2);
 struct avd_variant variant={.revision=4,.fifo_slots=15};
 struct avd_dev dev={.variant=&variant,.queued=false};
 struct avd_coded_fmt_ops ops={};struct fmt_desc fmt={.ops=&ops};
 struct avd_ctx ctx={.dev=&dev,.coded_fmt_desc=&fmt};
 struct avd_buf program={};struct avd_frame f={.prog=&program,.ctx=&ctx};dev.building=&f;
 struct avd_decoded_buffer dst[2]={};struct vb2_v4l2_buffer src={};u8 coded[512]={};
 current_src=&src;src.vb2_buf=(struct vb2_buffer){.dma=0x220000000ULL,.cpu=coded,.payload=512};
 for(unsigned int i=0;i<2;i++){
  dst[i].base.vb.vb2_buf=(struct vb2_buffer){.dma=0x230000000ULL+i*0x1000000ULL,.cpu=calloc(1,0x800000),.planes[0].length=0x800000,.timestamp=1000+i};refs[i]=&dst[i];
 }
 ctx.inst.addr=0x205000000ULL;
 ctx.decoded_fmt.fmt.pix_mp.width=1920;ctx.decoded_fmt.fmt.pix_mp.height=1088;
 ctx.decoded_fmt.fmt.pix_mp.plane_fmt[0].bytesperline=1920;
 seq=(struct v4l2_ctrl_av1_sequence){.bit_depth=8,.flags=V4L2_AV1_SEQUENCE_FLAG_SUBSAMPLING_X|V4L2_AV1_SEQUENCE_FLAG_SUBSAMPLING_Y};
 params=(struct v4l2_ctrl_av1_frame){.frame_type=V4L2_AV1_KEY_FRAME,.primary_ref_frame=7,
  .frame_width_minus_1=1919,.frame_height_minus_1=1087,.upscaled_width=1920};
 params.tile_info.tile_cols=params.tile_info.tile_rows=1;params.tile_info.width_in_sbs_minus_1[0]=29;params.tile_info.height_in_sbs_minus_1[0]=16;tile.tile_size=512;
 assert(!avd_av1_start(&ctx));
 for(unsigned int mode=0;mode<12;mode++){
  current_dst=&dst[!!mode].base.vb;
  params.frame_type=mode==0?V4L2_AV1_KEY_FRAME:mode==2?V4L2_AV1_INTRA_ONLY_FRAME:V4L2_AV1_INTER_FRAME;
  params.primary_ref_frame=mode==0||mode==4?7:0;
  params.ref_frame_idx[0]=0;params.reference_frame_ts[0]=1000;
  params.flags=(mode==3?V4L2_AV1_FRAME_FLAG_ERROR_RESILIENT_MODE:0)|(mode==5?V4L2_AV1_FRAME_FLAG_DISABLE_FRAME_END_UPDATE_CDF:0);
  seq.bit_depth=mode>=10?10:8;
  if(mode>=8){
   seq.flags|=V4L2_AV1_SEQUENCE_FLAG_FILM_GRAIN_PARAMS_PRESENT;
   grain.flags=V4L2_AV1_FILM_GRAIN_FLAG_APPLY_GRAIN|V4L2_AV1_FILM_GRAIN_FLAG_UPDATE_GRAIN;
   grain.grain_seed=1234+mode;
   params.flags|=mode&1?V4L2_AV1_FRAME_FLAG_SHOW_FRAME:V4L2_AV1_FRAME_FLAG_SHOWABLE_FRAME;
  }
  params.segmentation.flags=mode>=6?V4L2_AV1_SEGMENTATION_FLAG_ENABLED:0;
  if(mode==7)params.segmentation.flags|=V4L2_AV1_SEGMENTATION_FLAG_UPDATE_MAP;
  assert(!avd_av1_run(&ctx));
  char path[4096];snprintf(path,sizeof(path),"%s/@NAME@-%u.bin",argv[1],mode);FILE *out=fopen(path,"wb");assert(out);
  assert(fwrite(program.cpu,4,f.prog_words,out)==f.prog_words);
  struct avd_decoded_buffer *d=&dst[!!mode];size_t off=AVD_AV1_SEG_OFFSET(0x800000,d->av1.color_size,d->av1.seg_size);
  assert(fwrite((u8 *)d->base.vb.vb2_buf.cpu+off,1,d->av1.seg_size+ALIGN(AVD_CDFS_SIZE,AVD_ALIGN),out)==d->av1.seg_size+ALIGN(AVD_CDFS_SIZE,AVD_ALIGN));fclose(out);
  if(mode==0){
   size_t cdf=AVD_AV1_CDFS_OFFSET(0x800000,dst[0].av1.color_size);
   memset((u8 *)dst[0].base.vb.vb2_buf.cpu+cdf,0x57,AVD_CDFS_SIZE);
   memset((u8 *)dst[0].base.vb.vb2_buf.cpu+cdf-dst[0].av1.seg_size,0x36,dst[0].av1.seg_size);
  }
 }
 avd_av1_stop(&ctx);free(program.cpu);
 for(unsigned int i=0;i<2;i++)free(dst[i].base.vb.vb2_buf.cpu);
}
'''
    for name, body in [('legacy-before', old), ('legacy-after', av1)]:
        shim = 'static bool avd_av1_ready(struct avd_ctx *ctx){(void)ctx;return false;}\n' if name == 'legacy-before' else ''
        build_run(name, common + codec(body) + shim + post + legacy.replace('@NAME@', name))
    for mode in range(12):
        assert (OUT / f'legacy-before-{mode}.bin').read_bytes() == (OUT / f'legacy-after-{mode}.bin').read_bytes(), mode
    print('PASS: separate baseline/current executables emit byte-identical legacy AV1 programs and CDF/map tails for twelve header cases including film grain/show flags at 8/10 bits')
