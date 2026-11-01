#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Exercise production submission/retirement with a strict fake m2m scheduler.

The boundary rejects job_finish on HOLD_CAPTURE queues and a finish for any
context other than the running one. PM, work scheduling and codec preparation
are fixtures; avd_queue_submit_work(), avd_retire() and slot helpers are real.
"""
import os
from pathlib import Path
import hashlib
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = Path(sys.argv[2]).resolve() if len(sys.argv) > 2 else HERE.parent
CORE = Path(sys.argv[3]).resolve() if len(sys.argv) > 3 else ROOT.parents[4] / 'drivers/media/v4l2-core/v4l2-mem2mem.c'
from host_source import function

PRELUDE = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <media/videobuf2-v4l2.h>
#define AVD_H_
#define AVD_QUEUE_H_
#define AVD_MAX_FRAMES 16
#define VB2_V4L2_FL_SUPPORTS_M2M_HOLD_CAPTURE_BUF 1
#define V4L2_BUF_FLAG_M2M_HOLD_CAPTURE_BUF 1
#define AVD_FRAME_TIMEOUT 2000
#define AVD_CTX_MAX_HW_ERRORS 1
#define AVD_PROGRAM_SIZE 4096
#define GFP_KERNEL 0
#define dma_wmb() ((void)0)
#define system_wq NULL
#define READ_ONCE(v) (v)
static bool warn_on(bool v) { assert(!v);return v; }
#define WARN_ON(v) warn_on(!!(v))
#define container_of(p,t,m) ((t *)((char *)(p)-offsetof(t,m)))
#define spin_lock_irqsave(l,f) do { (void)(l); (f)=0; } while(0)
#define spin_unlock_irqrestore(l,f) do { (void)(l); (void)(f); } while(0)
#define mutex_lock(l) ((void)(l))
#define mutex_unlock(l) ((void)(l))
#define dev_err(d,...) ((void)(d))
static unsigned long jiffies=1;
struct work_struct { int unused; };
struct vb2_v4l2_buffer { unsigned int flags; bool is_held; int done;
 struct { struct { void *req; } req_obj; } vb2_buf;
};
struct avd_ctx;
struct avd_dev;
struct avd_buf { void *cpu; size_t size; };
'''

BOUNDARY = r'''
struct fake_m2m {
 struct { struct { unsigned int subsystem_flags; } q; } out_q_ctx;
 struct vb2_v4l2_buffer *src,*dst;
 struct avd_ctx *ctx;
 void *priv;
};
#define v4l2_m2m_ctx fake_m2m
struct v4l2_m2m_dev { struct fake_m2m *curr_ctx; int job_spinlock; };
static struct v4l2_m2m_dev core;
struct avd_codec_frame_ops {
 int (*prepare)(struct avd_ctx *,struct avd_frame *);
 void (*retire)(struct avd_ctx *,struct avd_frame *,enum vb2_buffer_state);
};
struct avd_coded_fmt_ops {
 int (*run)(struct avd_ctx *);
 void (*done)(struct avd_ctx *,struct vb2_v4l2_buffer *,struct vb2_v4l2_buffer *,enum vb2_buffer_state);
};
struct format { const struct avd_codec_frame_ops *frame_ops; const struct avd_coded_fmt_ops *ops; };
struct avd_ctx {
 struct avd_dev *dev; struct { struct fake_m2m *m2m_ctx; } fh;
 const struct format *coded_fmt_desc; struct work_struct submit_work;
 bool draining,broken; unsigned int hw_errors; enum avd_frame_dep next_dep;
 bool session_started; u64 session_generation;
 int ctrl_hdl;
};
struct avd_variant { size_t fp_size; };
struct avd_backend_ops {
 int (*session_start)(struct avd_ctx *);
 int (*submit)(struct avd_dev *,struct avd_frame *);
};
struct avd_dev {
 const struct avd_variant *variant;
 const struct avd_backend_ops *backend; u64 fw_generation;
 struct avd_frame frames[AVD_MAX_FRAMES]; struct avd_buf *frame_fp[AVD_MAX_FRAMES];
 struct avd_ctx *deferred_ctx; struct avd_frame *building;
 unsigned int inflight; u64 frame_seq; bool compat_held,stopping,faulted;
 int queue_lock,queue_mutex,queue_work; void *dev,*m2m_dev;
};
static struct avd_ctx *running;
static unsigned int finishes,raw_finishes,pm_refs;
static int prepare_result,pm_error,send_result;
static u32 send_tags[64];static unsigned int send_count;
static unsigned int capacity=1;
static struct vb2_v4l2_buffer *v4l2_m2m_next_src_buf(struct fake_m2m *m) { return m->src; }
static struct vb2_v4l2_buffer *v4l2_m2m_next_dst_buf(struct fake_m2m *m) { return m->dst; }
static struct vb2_v4l2_buffer *v4l2_m2m_src_buf_remove(struct fake_m2m *m) {
 struct vb2_v4l2_buffer *b=m->src;m->src=NULL;return b;
}
static struct vb2_v4l2_buffer *v4l2_m2m_dst_buf_remove(struct fake_m2m *m) {
 struct vb2_v4l2_buffer *b=m->dst;m->dst=NULL;return b;
}
static void finish(struct fake_m2m *m) {
 assert(running==m->ctx&&core.curr_ctx==m);running=NULL;core.curr_ctx=NULL;finishes++;
}
static bool _v4l2_m2m_job_finish(struct v4l2_m2m_dev *d,struct fake_m2m *m)
{ assert(d==&core);finish(m);return true; }
static void v4l2_m2m_schedule_next_job(struct v4l2_m2m_dev *d,struct fake_m2m *m)
{ (void)d;assert(!m->src&&(!m->dst||m->dst->is_held)); }
#define EXPORT_SYMBOL_GPL(f)
static void __attribute__((unused)) v4l2_m2m_job_finish(void *d,struct fake_m2m *m) {
 (void)d; assert(!(m->out_q_ctx.q.subsystem_flags&VB2_V4L2_FL_SUPPORTS_M2M_HOLD_CAPTURE_BUF));
 raw_finishes++; finish(m);
}
static void v4l2_m2m_buf_done(struct vb2_v4l2_buffer *b,enum vb2_buffer_state s)
{ (void)s; assert(b && !b->done); b->done++; }
static void v4l2_m2m_buf_copy_metadata(void *s,void *d) { (void)s;(void)d; }
static void v4l2_ctrl_request_complete(void *r,void *h) { (void)r;(void)h; }
static int v4l2_ctrl_request_setup(void *r,void *h) { (void)r;(void)h; return 0; }
static unsigned int avd_max_inflight(struct avd_dev *d) { (void)d; return capacity; }
static bool avd_backend_is_apple(struct avd_dev *d) { (void)d; return false; }
static int pm_runtime_resume_and_get(void *d) { (void)d; if(pm_error)return pm_error; pm_refs++;return 0; }
static void pm_runtime_mark_last_busy(void *d) { (void)d; }
static void pm_runtime_put_autosuspend(void *d) { (void)d; assert(pm_refs);pm_refs--; }
static void avd_pm_queue_changed(struct avd_dev *d) { (void)d; }
static void mod_delayed_work(void *w,void *v,int t) { (void)w;(void)v;(void)t; }
static int avd_buf_alloc(struct avd_dev *d,struct avd_buf *b,size_t n) { (void)d;(void)b;(void)n;return -ENOMEM; }
static void *devm_kzalloc(void *d,size_t n,int f) { (void)d;(void)f;return calloc(1,n); }
static int transport_submit(struct avd_dev *d,struct avd_frame *f) {
 (void)d;if(!send_result)send_tags[send_count++]=f->tag;return send_result;
}
static int codec_prepare(struct avd_ctx *c,struct avd_frame *f) { (void)c;(void)f;return prepare_result; }
void avd_job_finish(struct avd_ctx *c,enum vb2_buffer_state state);
static int codec_run(struct avd_ctx *c) {
 if(!prepare_result&&(c->fh.m2m_ctx->src->flags&V4L2_BUF_FLAG_M2M_HOLD_CAPTURE_BUF))
  avd_job_finish(c,VB2_BUF_STATE_DONE);
 return prepare_result;
}
'''

MAIN = r'''
static void submit(struct avd_ctx *c) { assert(!running);running=c;core.curr_ctx=c->fh.m2m_ctx;avd_queue_submit_work(&c->submit_work); }
static struct avd_frame *owned(struct avd_dev *d,struct avd_ctx *c) {
 for(unsigned int i=0;i<16;i++)if(d->frames[i].ctx==c&&d->frames[i].state!=AVD_FRAME_FREE)return &d->frames[i];
 return NULL;
}
static void complete(struct avd_dev *d,struct avd_ctx *c,int status) {
 struct avd_frame *f=owned(d,c);assert(f);f->state=AVD_FRAME_DONE;f->status=status;avd_retire(d);
}
int main(void) {
 static const struct avd_backend_ops backend={.submit=transport_submit};
 struct avd_dev d={.m2m_dev=&core,.backend=&backend};struct avd_buf programs[16]={};
 static const struct avd_coded_fmt_ops ops={.run=codec_run};
 static const struct format fmt={.ops=&ops};
 struct fake_m2m m[2]={};struct avd_ctx c[2]={};
 struct vb2_v4l2_buffer src[20]={},dst[20]={};
 for(unsigned int i=0;i<16;i++){d.frames[i].idx=i;d.frames[i].prog=&programs[i];}
 for(unsigned int i=0;i<2;i++){c[i].dev=&d;c[i].fh.m2m_ctx=&m[i];c[i].coded_fmt_desc=&fmt;m[i].ctx=&c[i];m[i].priv=&c[i];}
 /* Non-HOLD jobs detach once, finish dispatch once, retire buffers later. */
 m[0].src=&src[0];m[0].dst=&dst[0];submit(&c[0]);assert(!running&&d.inflight==1&&finishes==1);
 assert(!src[0].done&&!dst[0].done);
 /* The next context keeps the running dispatch while capacity is full. */
 m[1].src=&src[1];m[1].dst=&dst[1];submit(&c[1]);
 assert(running==&c[1]&&d.deferred_ctx==&c[1]&&finishes==1);
 complete(&d,&c[0],0);assert(src[0].done&&dst[0].done&&!d.inflight);
 d.deferred_ctx=NULL;avd_queue_submit_work(&c[1].submit_work);
 assert(!running&&finishes==2);complete(&d,&c[1],0);assert(finishes==2);
 /* Final slices release dispatch early without returning either DMA buffer. */
 m[0].out_q_ctx.q.subsystem_flags=VB2_V4L2_FL_SUPPORTS_M2M_HOLD_CAPTURE_BUF;
 m[0].src=&src[2];m[0].dst=&dst[2];submit(&c[0]);
 assert(!running&&finishes==3&&!owned(&d,&c[0])->scheduler_held);
 assert(!m[0].src&&!m[0].dst&&!src[2].done&&!dst[2].done);
 complete(&d,&c[0],0);assert(finishes==3&&!raw_finishes);
 /* A held intermediate slice returns only OUTPUT and keeps CAPTURE. */
 src[3].flags=V4L2_BUF_FLAG_M2M_HOLD_CAPTURE_BUF;
 m[0].src=&src[3];m[0].dst=&dst[3];submit(&c[0]);
 assert(!d.inflight&&!running&&src[3].done&&!dst[3].done&&dst[3].is_held);
 src[13].flags=V4L2_BUF_FLAG_M2M_HOLD_CAPTURE_BUF;m[0].src=&src[13];submit(&c[0]);
 assert(!d.inflight&&!running&&src[13].done&&!dst[3].done&&dst[3].is_held);
 m[0].src=&src[4];submit(&c[0]);
 assert(!running&&!src[4].done&&!dst[3].done&&!dst[3].is_held);
 complete(&d,&c[0],0);assert(src[4].done&&dst[3].done);
 /* Final-slice prepare failures detach early and retire buffers with error. */
 prepare_result=-EINVAL;m[0].src=&src[5];m[0].dst=&dst[5];submit(&c[0]);
 assert(!running&&!src[5].done&&!dst[5].done);avd_retire(&d);
 assert(src[5].done&&dst[5].done);
 c[0].broken=false;prepare_result=0;
 /* PM failure and stopped-stream dispatch finish the current job once. */
 pm_error=-EIO;m[0].src=&src[6];m[0].dst=&dst[6];submit(&c[0]);assert(!running&&!d.inflight);
 pm_error=0;c[0].draining=true;m[0].src=&src[7];m[0].dst=&dst[7];submit(&c[0]);
 assert(!running&&src[7].done&&dst[7].done&&!d.inflight&&!pm_refs);
 /* The native prepare failure path detaches and finishes a non-HOLD job. */
 static const struct avd_codec_frame_ops native_ops={.prepare=codec_prepare};
 static const struct format native_fmt={.frame_ops=&native_ops};
 c[1].coded_fmt_desc=&native_fmt;prepare_result=-EINVAL;
 programs[0].cpu=&programs[0];m[1].src=&src[8];m[1].dst=&dst[8];
 submit(&c[1]);assert(!running);avd_retire(&d);
 assert(src[8].done&&dst[8].done&&!d.inflight&&!pm_refs);
 /* The next picture really prepares while the first still owns DMA buffers. */
 c[0].draining=false;c[0].broken=false;prepare_result=0;capacity=2;
 m[0].src=&src[9];m[0].dst=&dst[9];submit(&c[0]);
 m[0].src=&src[10];m[0].dst=&dst[10];submit(&c[0]);
 assert(d.inflight==2&&!running&&!src[9].done&&!dst[9].done&&!src[10].done&&!dst[10].done);
 complete(&d,&c[0],0);assert(src[9].done&&dst[9].done&&!src[10].done&&!dst[10].done);
 complete(&d,&c[0],0);assert(src[10].done&&dst[10].done&&!d.inflight&&!pm_refs&&!raw_finishes);
 unsigned int before=finishes;
 avd_queue_submit_work(&c[0].submit_work);assert(finishes==before&&!d.inflight);
 capacity=1;c[1].coded_fmt_desc=&fmt;c[1].broken=false;
 m[0].src=&src[11];m[0].dst=&dst[11];submit(&c[0]);
 m[1].src=&src[12];m[1].dst=&dst[12];submit(&c[1]);
 assert(running==&c[1]&&d.deferred_ctx==&c[1]);before=finishes;
 avd_queue_submit_work(&c[0].submit_work);
 assert(running==&c[1]&&d.deferred_ctx==&c[1]&&finishes==before);
 complete(&d,&c[0],0);avd_queue_submit_work(&c[1].submit_work);complete(&d,&c[1],0);
 assert(!running&&!d.inflight&&!pm_refs&&!raw_finishes);
 /* Inbox pressure cannot reverse two pictures of a context. */
 capacity=3;send_result=-EBUSY;unsigned int sent=send_count;
 m[0].src=&src[14];m[0].dst=&dst[14];submit(&c[0]);
 u32 first=d.frames[0].tag;
 assert(d.frames[0].state==AVD_FRAME_RESERVED&&send_count==sent);
 send_result=0;m[0].src=&src[15];m[0].dst=&dst[15];submit(&c[0]);
 u32 second=d.frames[1].tag;
 assert(d.frames[1].state==AVD_FRAME_RESERVED&&send_count==sent);
 m[1].src=&src[16];m[1].dst=&dst[16];submit(&c[1]);
 assert(d.frames[2].state==AVD_FRAME_SUBMITTED&&send_count==sent+1);
 assert(!avd_send_frame(&d,&d.frames[0]));assert(!avd_send_frame(&d,&d.frames[1]));
 assert(send_tags[sent+1]==first&&send_tags[sent+2]==second);
 complete(&d,&c[1],0);complete(&d,&c[0],0);complete(&d,&c[0],0);
 assert(!running&&!d.inflight&&!pm_refs&&!raw_finishes);
 puts("PASS: production dispatch/retire: final/held slices, two contexts, capacity retry, prepare/PM/abort errors, exactly-once job_finish");
}
'''


def main():
    out = Path(sys.argv[1]).resolve()
    out.mkdir(parents=True, exist_ok=True)
    names = ['avd-queue.c', 'avd-queue-internal.h', 'avd-queue.h', 'avd-v4l2.c']
    (out / 'core.sha256').write_text(hashlib.sha256(CORE.read_bytes()).hexdigest()+'  v4l2-mem2mem.c\n')
    (out / 'inputs.sha256').write_text(''.join(hashlib.sha256((ROOT / n).read_bytes()).hexdigest()+'  '+n+'\n' for n in names))
    qh = (ROOT / 'avd-queue.h').read_text()
    frame = qh[qh.index('enum avd_frame_state'):qh.index('struct avd_backend_ops {')]
    source = (ROOT / 'avd-queue.c').read_text()
    legacy = (ROOT / 'avd-v4l2.c').read_text()
    code = out / 'dispatch.c'
    code.write_text(PRELUDE + frame + BOUNDARY + '\n#include "avd-queue-internal.h"\n' +
                    function(CORE.read_text(), 'v4l2_m2m_get_curr_priv') +
                    function(CORE.read_text(), 'v4l2_m2m_job_finish_detach') +
                    function(CORE.read_text(), 'v4l2_m2m_buf_done_and_job_finish') +
                    function(legacy, 'avd_job_finish_no_pm') + function(legacy, 'avd_job_finish') +
                    function(source, 'avd_send_frame') +
                    function(source, 'avd_retire') + function(source, 'avd_queue_submit_work') + MAIN)
    exe = out / 'dispatch-test'
    subprocess.run([os.environ.get('CC', 'cc'), '-std=gnu11', '-Wall', '-Wextra', '-Werror',
                    '-fsanitize=address,undefined', '-g', '-I' + str(HERE / 'include'),
                    '-I' + str(ROOT), str(code), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)


if __name__ == '__main__':
    main()
