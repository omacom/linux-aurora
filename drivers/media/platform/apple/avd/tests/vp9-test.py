#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Run the production VP9 banks, probability helpers and program emitters.

Usage: vp9-test.py OUT [AVD_DIR [KERNEL_ROOT]]. Builds require Linux UAPI and
ASan/UBSan. Synthetic symbol counts exercise arithmetic; no pixels are decoded.
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

# Share only the explicit MMIO/allocation boundary with the H.264 tests.
spec = importlib.util.spec_from_file_location('program_fixture', HERE / 'program-test.py')
fixture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fixture)
pre = fixture.PRELUDE
pre = pre.replace('typedef uint8_t u8;', 'typedef uint8_t u8; typedef uint16_t u16;')
pre = pre.replace('#define AVD_MAX_INST 512', '#define AVD_MAX_INST 512\n#define AVD_CODEC_HEVC 0')
pre = pre.replace('struct vb2_buffer { struct { size_t length; } planes[1]; dma_addr_t dma; };',
                  'struct vb2_buffer { struct { size_t length; } planes[1]; dma_addr_t dma; void *cpu; size_t payload; u64 timestamp; };')
pre = pre.replace('struct avd_comp comp; unsigned int bank;', 'struct avd_comp comp; unsigned int bank; struct { unsigned int width,height,bit_depth; } vp9;')
pre = pre.replace('struct vb2_v4l2_buffer *dst; } bufs;', 'struct vb2_v4l2_buffer *src,*dst; } bufs;')
pre = pre.replace('unsigned int fifo_idx, pp_total;', 'unsigned int fifo_idx, pp_total, max_frames; int ctrl_hdl;')
pre += r'''
#define round_up(x,a) (((x)+(a)-1)&~((a)-1))
#define max_t(t,a,b) ((t)(a)>(t)(b)?(t)(a):(t)(b))
#define min(a,b) ((a)<(b)?(a):(b))
#define clamp(x,l,h) ((x)<(l)?(l):(x)>(h)?(h):(x))
#define WARN_ON(v) (assert(!(v)),false)
#define GFP_KERNEL 0
#define READ_ONCE(x) (x)
#define WRITE_ONCE(x,v) ((x)=(v))
#define noinline_for_stack __attribute__((noinline))
#define static_assert _Static_assert
#define EXPORT_SYMBOL_GPL(x)
#define MODULE_LICENSE(x)
#define MODULE_DESCRIPTION(x)
#define MODULE_AUTHOR(x)
#define VB2_BUF_STATE_DONE 0
#define VB2_BUF_STATE_ERROR 1
enum vb2_buffer_state { STATE_DUMMY };
#define dev_err(d,...) ((void)(d))
#define kzalloc(n,f) calloc(1,n)
#define kfree free
static bool vp_feature = true;
static bool avd_qfw_has_vp_done(const struct avd_dev *d) { return d->queued && vp_feature; }
static void *vb2_plane_vaddr(struct vb2_buffer *b,int p) { (void)p;return b->cpu; }
static size_t vb2_get_plane_payload(struct vb2_buffer *b,int p) { (void)p;return b->payload; }
static struct vb2_v4l2_buffer *current_src,*current_dst;
static struct vb2_v4l2_buffer *v4l2_m2m_next_src_buf(void *m) { (void)m;return current_src; }
static u32 get_unaligned_be32(const void *p) { const u8 *v=p;return (u32)v[0]<<24|(u32)v[1]<<16|(u32)v[2]<<8|v[3]; }
struct v4l2_ctrl { union { void *p; } p_cur; };
static struct v4l2_ctrl_vp9_frame params;
static struct v4l2_ctrl_vp9_compressed_hdr updates;
static struct v4l2_ctrl frame_ctrl,update_ctrl;
static struct v4l2_ctrl *v4l2_ctrl_find(void *h,u32 id) {
 (void)h;frame_ctrl.p_cur.p=&params;update_ctrl.p_cur.p=&updates;
 return id==V4L2_CID_STATELESS_VP9_FRAME?&frame_ctrl:&update_ctrl;
}
static void avd_run_preamble(struct avd_ctx *ctx,struct avd_run *run) {
 (void)ctx;memset(run,0,sizeof(*run));run->bufs.src=current_src;run->bufs.dst=current_dst;
 run->coded_in=current_src->vb2_buf.dma;run->comp_out=current_dst->vb2_buf.dma;
 run->y_out=run->comp_out+0x100000;run->uv_out=run->comp_out+0x300000;
}
static void avd_run_postamble(struct avd_ctx *ctx,struct avd_run *run) { (void)ctx;(void)run; }
static int avd_init_job(struct avd_ctx *ctx,int codec,size_t segments) {
 free(ctx->job.segments);ctx->job.codec=codec;ctx->job.num=0;
 ctx->job.segments=calloc(segments,sizeof(*ctx->job.segments));return ctx->job.segments?0:-ENOMEM;
}
static int avd_submit_job(struct avd_ctx *ctx);
'''
pre = pre.replace('dma_addr_t comp_out, y_out, uv_out;', 'dma_addr_t comp_out, y_out, uv_out, coded_in;')


def without_includes(s):
    return re.sub(r'^#include[^\n]*\n', '', s, flags=re.M)

vp9 = (AVD / 'avd-vp9.c').read_text()
helper_h = (KERNEL / 'include/media/v4l2-vp9.h').read_text()
helper_c = (KERNEL / 'drivers/media/v4l2-core/v4l2-vp9.c').read_text()
source = pre + '\n' + without_includes(helper_h) + '\n' + without_includes(helper_c)
source += '\n' + without_includes((AVD / 'avd-inst.h').read_text())
# Include unchanged codec declarations/functions, excluding only the ops table.
source += '\n' + without_includes(vp9[vp9.index('#define VP9_Q_IDX'):vp9.index('static enum avd_image_fmt avd_vp9_get_image_fmt')])
source += '\n' + function(vp9, 'avd_vp9_pp_kicks')
source += '\n' + function((AVD / 'avd-backend-open.c').read_text(), 'avd_open_build_program')
qfw = (AVD / 'avd-backend-queue.c').read_text()
source += qfw[qfw.index('#define AVDQ_MAGIC'):qfw.index('/* Arguments of dev_dbg()')]
hw = (AVD / 'avd-hw.c').read_text()
source += hw[hw.index('#define AVD_V3_'):hw.index('int avd_boot(')]
source += '#define w32(r,v) writel((v),avd->ctrl+(r))\n#define m32(r,v) w32(r,(v)|readl(avd->ctrl+(r)))\n'
source += function(hw, 't8112_configure_stream')
source += function(qfw, 'avd_qfw_stream') + function(qfw, 'avd_qfw_submit')
source += '\nstatic int avd_submit_job(struct avd_ctx *ctx) { return avd_open_build_program(ctx); }\n'
source += r'''
static void write_program(const char *dir,const char *name,struct avd_frame *f) {
 char path[4096];snprintf(path,sizeof(path),"%s/%s.bin",dir,name);FILE *out=fopen(path,"wb");assert(out);
 assert(fwrite(f->prog->cpu,4,f->prog_words,out)==f->prog_words);fclose(out);
 struct avd_dev *dev=f->ctx->dev;struct avd_qfw *q=dev->qfw;
 /* The descriptor, PP command and configure_stream snapshot are production. */
 q->wp=0;memset(q->sram,0,0x10000);f->tag=0x10|f->idx;
 assert(!avd_qfw_submit(dev,f));
 snprintf(path,sizeof(path),"%s/%s-desc.bin",dir,name);out=fopen(path,"wb");assert(out);
 assert(fwrite(q->sram+0x8000+f->idx*64,1,64,out)==64);fclose(out);
 snprintf(path,sizeof(path),"%s/%s-mmio.bin",dir,name);out=fopen(path,"wb");assert(out);
 assert(fwrite(dev->ctrl,1,0x10000,out)==0x10000);fclose(out);
}
static unsigned int offset_words[256];
static void save_dma(const char *dir,const char *name) {
 char path[4096];snprintf(path,sizeof(path),"%s/%s-dma.txt",dir,name);FILE *out=fopen(path,"w");assert(out);
 for(unsigned int i=0;i<dma_count;i++)fprintf(out,"%u %u %u %llu\n",offset_words[i],dma_sites[i].shift,dma_sites[i].width,(unsigned long long)dma_sites[i].addr);
 fclose(out);
}
int main(int argc,char **argv) {
 assert(argc==2);
 struct avd_variant variant={.revision=4,.quirks=AVD_QUIRK_LSR,.fifo_slots=15,.vp_slots={4,4,1,0}};
 struct avd_dev dev={.variant=&variant,.queued=true,.capabilities=7};
 struct avd_qfw queue={.sram=calloc(1,0x10000),.aperture=0x1090000,.ring_words=0x1800,.fifo_slots=15,.vp_done=true};
 dev.qfw=&queue;dev.ctrl=calloc(1,0x10000);dev.mbox=calloc(1,0x100);variant.configure_stream=t8112_configure_stream;
 writel(0x20001,dev.mbox+0x50);
 static const struct avd_coded_fmt_ops ops={.pp_kicks=avd_vp9_pp_kicks,.vp_done=avd_vp9_vp_done};
 static const struct fmt_desc fmt={.ops=&ops};
 struct avd_ctx ctx={.dev=&dev,.coded_fmt_desc=&fmt,.pp_total=1};
 struct avd_buf program={};struct avd_frame frame={.prog=&program,.ctx=&ctx};dev.building=&frame;
 struct avd_decoded_buffer dst[3]={};struct vb2_v4l2_buffer src={};
 u8 coded[512]={};src.vb2_buf=(struct vb2_buffer){.dma=0x220000000ULL,.cpu=coded,.payload=512};
 current_src=&src;
 for(unsigned int i=0;i<3;i++)dst[i].base.vb.vb2_buf.dma=0x230000000ULL+i*0x1000000ULL;
 ctx.decoded_fmt.fmt.pix_mp.width=1920;ctx.decoded_fmt.fmt.pix_mp.height=1088;
 ctx.decoded_fmt.fmt.pix_mp.plane_fmt[0].bytesperline=1920;
 assert(!avd_vp9_start(&ctx));struct avd_vp9_ctx *p=ctx.priv;
 assert(ctx.max_frames==2&&p->nbanks==2);
 assert(p->bank[0].bufs.counts.addr!=p->bank[1].bufs.counts.addr);
 assert(p->bank[0].bufs.probs.addr!=p->bank[1].bufs.probs.addr);
 assert(p->bank[0].cnts.skip!=p->bank[1].cnts.skip);
 assert(p->bank[0].cnts.coeff[0][0][0][0][0]!=p->bank[1].cnts.coeff[0][0][0][0][0]);
 assert(p->bank[0].cnts.eob[0][0][0][0][0][0]!=p->bank[1].cnts.eob[0][0][0][0][0][0]);
 /* All header-flag combinations, including malformed error-resilient refresh. */
 for(unsigned int bits=0;bits<8;bits++) {
  u32 flags=(bits&1?V4L2_VP9_FRAME_FLAG_REFRESH_FRAME_CTX:0)|
   (bits&2?V4L2_VP9_FRAME_FLAG_PARALLEL_DEC_MODE:0)|(bits&4?V4L2_VP9_FRAME_FLAG_ERROR_RESILIENT:0);
  assert(avd_vp9_needs_counts(flags)==(bits==1));
 }
 params=(struct v4l2_ctrl_vp9_frame){.flags=V4L2_VP9_FRAME_FLAG_KEY_FRAME|V4L2_VP9_FRAME_FLAG_SHOW_FRAME|V4L2_VP9_FRAME_FLAG_REFRESH_FRAME_CTX,
  .frame_width_minus_1=1919,.frame_height_minus_1=1087,.bit_depth=8};
 updates.tx_mode=V4L2_VP9_TX_MODE_SELECT;
 current_dst=&dst[0].base.vb;frame.idx=0;dma_count=0;
 assert(avd_vp9_ready(&ctx));assert(!avd_vp9_run(&ctx));
 assert(dst[0].bank==1&&p->bank[0].busy&&!avd_vp9_ready(&ctx));
 assert(frame.fifo_iova==p->bank[0].bufs.inst.addr);
 write_program(argv[1],"vp9-adaptive-0",&frame);
 /* Symbol-count fixture must change a saved coefficient probability. */
 struct avd_vp9_frame_symbol_counts *counts=p->bank[0].bufs.counts.cpu;
 for(size_t i=0;i<528;i++){counts->eob_0[i]=16;counts->ref_cnt[i*4]=1;counts->ref_cnt[i*4+1]=4;counts->ref_cnt[i*4+2]=11;counts->ref_cnt[i*4+3]=8;}
 struct v4l2_vp9_frame_context before=p->frame_context[0];
 avd_vp9_vp_done(&ctx,&dst[0].base.vb);
 assert(avd_vp9_ready(&ctx)&&p->bank[0].busy&&p->bank[0].updated&&dst[0].bank==1);
 assert(memcmp(&before,&p->frame_context[0],sizeof(before)));
 struct v4l2_vp9_frame_context adapted=p->frame_context[0];
 avd_vp9_vp_done(&ctx,&dst[0].base.vb);assert(!memcmp(&adapted,&p->frame_context[0],sizeof(adapted)));
 current_dst=&dst[1].base.vb;frame.idx=1;dma_count=0;params.flags=V4L2_VP9_FRAME_FLAG_SHOW_FRAME|V4L2_VP9_FRAME_FLAG_REFRESH_FRAME_CTX;
 reference=dst[0];reference.vp9.width=1920;reference.vp9.height=1088;
 assert(!avd_vp9_run(&ctx));assert(dst[1].bank==2&&p->bank[0].busy&&p->bank[1].busy);
 assert(!memcmp(&adapted,&p->bank[1].probability_tables,sizeof(adapted)));
 write_program(argv[1],"vp9-adaptive-1",&frame);
 assert(!avd_vp9_ready(&ctx));
 struct avd_vp9_run attempt;
 current_dst=&dst[2].base.vb;assert(avd_vp9_run_preamble(&ctx,&attempt)==-EBUSY);assert(!dst[2].bank);
 /* PP(0) retires while b points to bank 1: only bank 0 may be released. */
 avd_vp9_done(&ctx,&src,&dst[0].base.vb,VB2_BUF_STATE_DONE);
 assert(!p->bank[0].busy&&p->bank[1].busy&&!dst[0].bank);
 assert(!memcmp(&adapted,&p->frame_context[0],sizeof(adapted)));
 /* Failed VP/prepare must never adapt partial counts. */
 avd_vp9_done(&ctx,&src,&dst[1].base.vb,VB2_BUF_STATE_ERROR);
 assert(!p->bank[1].busy&&avd_vp9_ready(&ctx));
 assert(!memcmp(&adapted,&p->frame_context[0],sizeof(adapted)));
 for(unsigned int mode=0;mode<3;mode++) {
  params.flags=mode==0?0:mode==1?V4L2_VP9_FRAME_FLAG_REFRESH_FRAME_CTX|V4L2_VP9_FRAME_FLAG_PARALLEL_DEC_MODE:V4L2_VP9_FRAME_FLAG_ERROR_RESILIENT;
  current_dst=&dst[0].base.vb;frame.idx=0;assert(!avd_vp9_run(&ctx));
  assert(avd_vp9_ready(&ctx)&&p->bank[0].busy);
  if(mode==0) {
   /* The third inter picture's temporal-MV input stays in the prior bank. */
   assert(p->last.color==p->bank[0].bufs.color.addr);
   bool prior_color=false;u32 *words=program.cpu;
   for(size_t j=0;j<frame.prog_words;j++)if(words[j]==(p->bank[1].bufs.color.addr>>8))prior_color=true;
   assert(prior_color);
  }
  current_dst=&dst[1].base.vb;frame.idx=1;assert(!avd_vp9_run(&ctx));
  assert(p->bank[0].busy&&p->bank[1].busy);
  avd_vp9_done(&ctx,&src,&dst[0].base.vb,VB2_BUF_STATE_DONE);avd_vp9_done(&ctx,&src,&dst[1].base.vb,VB2_BUF_STATE_DONE);
 }
 avd_vp9_stop(&ctx);
 /* Old firmware and the legacy transport allocate only one bank. */
 for(unsigned int mode=0;mode<2;mode++) {
  dev.queued=mode;vp_feature=false;assert(!avd_vp9_start(&ctx));p=ctx.priv;
  assert(p->nbanks==1&&ctx.max_frames==1);avd_vp9_stop(&ctx);
 }
 free(program.cpu);free(queue.sram);free(dev.ctrl);free(dev.mbox);puts("PASS: VP9 dependency table, bank ownership, real adaptation, early release and legacy fallback");
}
'''
# Extra pixel-format shape and CPU-reference dimensions in this boundary.
source = source.replace('struct { u32 bytesperline; } plane_fmt[1];', 'u32 width,height; struct { u32 bytesperline; } plane_fmt[1];')
(OUT/'vp9-host.c').write_text(source)
cmd = [os.environ.get('CC','cc'), '-std=gnu11','-g','-O1','-Wall','-Wextra','-Werror',
       '-Wno-unused-function','-Wno-unused-parameter','-Wno-sign-compare',
       '-fsanitize=address,undefined','-fno-omit-frame-pointer',str(OUT/'vp9-host.c'),'-o',str(OUT/'vp9-host')]
subprocess.run(cmd,check=True)
subprocess.run([str(OUT/'vp9-host'),str(OUT)],check=True)
(OUT/'source.sha256').write_text('\n'.join(hashlib.sha256(p.read_bytes()).hexdigest()+'  '+str(p) for p in (AVD/'avd-vp9.c',AVD/'avd-backend-open.c',KERNEL/'include/media/v4l2-vp9.h',KERNEL/'drivers/media/v4l2-core/v4l2-vp9.c'))+'\n')

# Optional independent before/after executable comparison. The baseline is read
# from public Git history, never from a captured stock firmware program.
base_ref = os.environ.get('AVD_VP9_BASELINE_REF')
if base_ref:
    old = subprocess.check_output(['git','-C',str(KERNEL),'show',
                                  base_ref + ':drivers/media/platform/apple/avd/avd-vp9.c'],text=True)
    legacy_main = r'''
int main(int argc,char **argv) {
 assert(argc==2);
 struct avd_variant variant={.revision=4,.quirks=AVD_QUIRK_LSR,.fifo_slots=15};
 struct avd_dev dev={.variant=&variant,.queued=false};
 struct avd_coded_fmt_ops ops={};struct fmt_desc fmt={.ops=&ops};
 struct avd_ctx ctx={.dev=&dev,.coded_fmt_desc=&fmt};
 struct avd_buf program={};struct avd_frame f={.prog=&program,.ctx=&ctx};dev.building=&f;
 struct avd_decoded_buffer dst={};struct vb2_v4l2_buffer src={};u8 coded[512]={};
 dst.base.vb.vb2_buf.dma=0x230000000ULL;src.vb2_buf=(struct vb2_buffer){.dma=0x220000000ULL,.cpu=coded,.payload=512};
 current_dst=&dst.base.vb;current_src=&src;ctx.inst.addr=0x205000000ULL;
 ctx.decoded_fmt.fmt.pix_mp.width=1920;ctx.decoded_fmt.fmt.pix_mp.height=1088;
 ctx.decoded_fmt.fmt.pix_mp.plane_fmt[0].bytesperline=1920;
 assert(!avd_vp9_start(&ctx));struct avd_vp9_ctx *p=ctx.priv;
 params=(struct v4l2_ctrl_vp9_frame){.frame_width_minus_1=1919,.frame_height_minus_1=1087,.bit_depth=8};
 updates.tx_mode=V4L2_VP9_TX_MODE_SELECT;
 const u32 flags[]={V4L2_VP9_FRAME_FLAG_KEY_FRAME|V4L2_VP9_FRAME_FLAG_REFRESH_FRAME_CTX,
  V4L2_VP9_FRAME_FLAG_REFRESH_FRAME_CTX,V4L2_VP9_FRAME_FLAG_PARALLEL_DEC_MODE|V4L2_VP9_FRAME_FLAG_REFRESH_FRAME_CTX,
  0,V4L2_VP9_FRAME_FLAG_ERROR_RESILIENT|V4L2_VP9_FRAME_FLAG_PARALLEL_DEC_MODE,
  V4L2_VP9_FRAME_FLAG_KEY_FRAME|V4L2_VP9_FRAME_FLAG_REFRESH_FRAME_CTX|V4L2_VP9_FRAME_FLAG_PARALLEL_DEC_MODE};
 for(unsigned int i=0;i<ARRAY_SIZE(flags);i++) {
  params.flags=flags[i]|V4L2_VP9_FRAME_FLAG_SHOW_FRAME;
  reference=dst;reference.vp9.width=1920;reference.vp9.height=1088;
  assert(!avd_vp9_run(&ctx));
  char path[4096];snprintf(path,sizeof(path),"%s/legacy-%u.bin",argv[1],i);FILE *out=fopen(path,"wb");assert(out);
  assert(fwrite(program.cpu,4,f.prog_words,out)==f.prog_words);fclose(out);
  struct avd_vp9_frame_symbol_counts *c=WORK(p)->bufs.counts.cpu;
  for(size_t j=0;j<528;j++){c->eob_0[j]=16;c->ref_cnt[j*4]=1;c->ref_cnt[j*4+1]=4;c->ref_cnt[j*4+2]=11;c->ref_cnt[j*4+3]=8;}
  avd_vp9_done(&ctx,&src,&dst.base.vb,VB2_BUF_STATE_DONE);
  snprintf(path,sizeof(path),"%s/context-%u.bin",argv[1],i);out=fopen(path,"wb");assert(out);
  assert(fwrite(p->frame_context,1,sizeof(p->frame_context),out)==sizeof(p->frame_context));fclose(out);
 }
 avd_vp9_stop(&ctx);free(program.cpu);
}
'''
    for name, body in (('before',old),('after',vp9)):
        target = OUT / ('legacy-' + name)
        target.mkdir(exist_ok=True)
        src = pre + without_includes(helper_h) + without_includes(helper_c)
        src += without_includes((AVD/'avd-inst.h').read_text())
        src += without_includes(body[body.index('#define VP9_Q_IDX'):body.index('static enum avd_image_fmt avd_vp9_get_image_fmt')])
        src += function((AVD/'avd-backend-open.c').read_text(),'avd_open_build_program')
        src += '\nstatic int avd_submit_job(struct avd_ctx *ctx) { return avd_open_build_program(ctx); }\n'
        src += '#define WORK(p) ' + ('((p)->b)' if name=='after' else '(p)') + '\n' + legacy_main
        src = src.replace('struct { u32 bytesperline; } plane_fmt[1];','u32 width,height; struct { u32 bytesperline; } plane_fmt[1];')
        (target/'legacy.c').write_text(src)
        subprocess.run(cmd[:-3]+[str(target/'legacy.c'),'-o',str(target/'legacy')],check=True)
        subprocess.run([str(target/'legacy'),str(target)],check=True)
    for p in sorted((OUT/'legacy-before').glob('*.bin')):
        assert p.read_bytes()==(OUT/'legacy-after'/p.name).read_bytes(),p.name
    print('PASS: legacy VP9 programs and adapted frame contexts byte-identical to '+base_ref)
