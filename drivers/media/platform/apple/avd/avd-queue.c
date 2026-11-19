// SPDX-License-Identifier: GPL-2.0-only OR MIT
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/sizes.h>

#include "avd.h"
#include "avd-queue-internal.h"

#define AVD_FRAME_TIMEOUT msecs_to_jiffies(2000)
/* A stream is stopped at its first frame that fails in hardware. */
#define AVD_CTX_MAX_HW_ERRORS 1
/* Never reset the decoder more often than this. */
#define AVD_RECOVER_GAP_MS 20
/* A third recovery parks the decoder until AVD_STORM_WINDOW ends. */
#define AVD_STORM_MAX 2
#define AVD_STORM_WINDOW msecs_to_jiffies(60000)
#define AVD_RETRY_DELAY msecs_to_jiffies(1)
#define AVD_PROGRAM_SIZE SZ_2M

/* PM test fixtures use backends without a transport. */
static unsigned int avd_max_inflight(struct avd_dev *avd)
{
	return avd->backend->max_inflight ? avd->backend->max_inflight(avd) : 1;
}

bool avd_backend_is_apple(const struct avd_dev *avd)
{
	return avd->backend == &avd_apple_backend;
}

/* Only the CPU callback releases a probability dependency, never the IRQ. */
static void avd_retire_vp(struct avd_dev *avd)
{
	unsigned int i;

	for (i = 0; i < AVD_MAX_FRAMES; i++) {
		struct avd_frame *f = &avd->frames[i];
		const struct avd_coded_fmt_ops *ops;

		if (f->state == AVD_FRAME_FREE || !f->vp_done || f->vp_retired)
			continue;
		ops = f->ctx->coded_fmt_desc->ops;
		/* A reset/error must not publish partial symbol counts. */
		if (!f->status && ops && ops->vp_done) {
			dma_rmb();
			ops->vp_done(f->ctx, f->dst);
		}
		f->vp_retired = true;
	}
}

/* queue_mutex excludes callbacks and context destruction from retirement. */
static void avd_retire(struct avd_dev *avd)
{
	struct avd_frame *f;
	unsigned long flags;
	unsigned int i;
	bool progress;

	do {
		progress = false;
		for (i = 0; i < AVD_MAX_FRAMES; i++) {
			struct avd_ctx *ctx;
			enum vb2_buffer_state state;

			f = &avd->frames[i];
			spin_lock_irqsave(&avd->queue_lock, flags);
			if (f->state != AVD_FRAME_DONE ||
			    !avd_slot_first(avd->frames, f)) {
				spin_unlock_irqrestore(&avd->queue_lock, flags);
				continue;
			}
			spin_unlock_irqrestore(&avd->queue_lock, flags);
			ctx = f->ctx;
			state = f->status ? VB2_BUF_STATE_ERROR : VB2_BUF_STATE_DONE;
			if (!f->status) {
				ctx->hw_errors = 0;
			} else if (++ctx->hw_errors >= AVD_CTX_MAX_HW_ERRORS && !ctx->broken) {
				ctx->broken = true;
				dev_err(avd->dev, "stream stopped after %u failed frames; restart streaming to retry\n",
					ctx->hw_errors);
			}
			if (ctx->coded_fmt_desc->frame_ops) {
				if (ctx->coded_fmt_desc->frame_ops->retire)
					ctx->coded_fmt_desc->frame_ops->retire(ctx, f, state);
			} else if (ctx->coded_fmt_desc->ops->done) {
				ctx->coded_fmt_desc->ops->done(ctx, f->src, f->dst, state);
			}
			if (ctx->coded_fmt_desc->frame_ops && f->src->vb2_buf.req_obj.req)
				v4l2_ctrl_request_complete(f->src->vb2_buf.req_obj.req,
							   &ctx->ctrl_hdl);
			/* CAPTURE first: OUTPUT completion signals the media request. */
			if (f->scheduler_held) {
				v4l2_m2m_buf_done_and_job_finish(avd->m2m_dev,
								 ctx->fh.m2m_ctx, state);
			} else {
				v4l2_m2m_buf_done(f->dst, state);
				v4l2_m2m_buf_done(f->src, state);
			}
			spin_lock_irqsave(&avd->queue_lock, flags);
			f->ctx = NULL;
			f->src = NULL;
			f->dst = NULL;
			f->state = AVD_FRAME_FREE;
			avd->inflight--;
			spin_unlock_irqrestore(&avd->queue_lock, flags);
			avd_pm_queue_changed(avd);
			pm_runtime_mark_last_busy(avd->dev);
			pm_runtime_put_autosuspend(avd->dev);
			progress = true;
		}
	} while (progress);
}

/* A reset invalidates every firmware-owned slot, including other clients. */
static void avd_reset_queue(struct avd_dev *avd, int status)
{
	struct avd_ctx *ctx;
	unsigned long flags;
	unsigned int i;
	int ret;

	/*
	 * Every decoder reset is a risk to the whole machine (three hard hangs
	 * on the J616s followed timeouts and resets), so recover sparingly:
	 * at least AVD_RECOVER_GAP_MS apart, at most AVD_STORM_MAX times in
	 * AVD_STORM_WINDOW, and never again after a recovery fails. Exhausting
	 * the budget parks DMA until the window ends; only a failed recovery
	 * retires the decoder until the module is reloaded.
	 */
	if (avd->faulted) {
		ret = -ENODEV;
	} else {
		if (!avd->storm_count ||
		    time_after(jiffies, avd->storm_start + AVD_STORM_WINDOW)) {
			avd->storm_start = jiffies;
			avd->storm_count = 0;
		}
		if (++avd->storm_count > AVD_STORM_MAX) {
			ret = avd_pm_park(avd);
			if (!ret) {
				avd->rest_until = avd->storm_start + AVD_STORM_WINDOW;
				dev_warn(avd->dev, "decoder resting after repeated resets, new frames fail for up to %u s\n",
					 jiffies_to_msecs(avd->rest_until - jiffies) / 1000 + 1);
				ret = -EBUSY;
			}
		} else {
			if (avd->last_recover &&
			    time_before(jiffies, avd->last_recover +
					msecs_to_jiffies(AVD_RECOVER_GAP_MS)))
				msleep(AVD_RECOVER_GAP_MS);
			ret = avd_pm_recover(avd);
			avd->last_recover = jiffies ?: 1;
		}
		if (ret && !avd->rest_until) {
			WRITE_ONCE(avd->faulted, true);
			dev_err(avd->dev, "decoder retired after a failed reset (%d): reload the module to use it again\n",
				ret);
		}
	}
	avd->compat_active = NULL;
	list_for_each_entry(ctx, &avd->contexts, queue_node)
		ctx->session_started = false;
	spin_lock_irqsave(&avd->queue_lock, flags);
	for (i = 0; i < AVD_MAX_FRAMES; i++) {
		if (avd->frames[i].state == AVD_FRAME_FREE)
			continue;
		avd->frames[i].status = status;
		avd->frames[i].state = AVD_FRAME_DONE;
	}
	spin_unlock_irqrestore(&avd->queue_lock, flags);
	if (!ret)
		avd_pm_enable_irq(avd);
}

static int avd_send_frame(struct avd_dev *avd, struct avd_frame *f)
{
	unsigned long flags;
	int ret;

	if (!avd_slot_submit_first(avd->frames, f))
		return -EBUSY;
	if (f->dep == AVD_DEP_PREV_DONE && !avd_slot_first(avd->frames, f))
		return -EBUSY;
	if ((!f->ctx->session_started ||
	     f->ctx->session_generation != avd->fw_generation) &&
	    avd->backend->session_start) {
		ret = avd->backend->session_start(f->ctx);
		if (ret)
			return ret;
		f->ctx->session_started = true;
		f->ctx->session_generation = avd->fw_generation;
	}
	/* Publish ownership before the doorbell; IRQs may arrive in submit(). */
	spin_lock_irqsave(&avd->queue_lock, flags);
	f->state = AVD_FRAME_SUBMITTED;
	spin_unlock_irqrestore(&avd->queue_lock, flags);
	dma_wmb();
	ret = avd->backend->submit(avd, f);
	spin_lock_irqsave(&avd->queue_lock, flags);
	avd_slot_submit_result(f, ret);
	spin_unlock_irqrestore(&avd->queue_lock, flags);
	return ret;
}

static void avd_queue_worker(struct work_struct *work)
{
	struct avd_dev *avd = container_of(to_delayed_work(work),
					 struct avd_dev, queue_work);
	struct avd_ctx *ctx;
	unsigned long flags;
	unsigned int i;
	bool pending;
	unsigned long delay = AVD_FRAME_TIMEOUT;

	mutex_lock(&avd->queue_mutex);
	if (avd->stopping)
		goto unlock;
	for (i = 0; i < AVD_MAX_FRAMES; i++) {
		bool expired;

		spin_lock_irqsave(&avd->queue_lock, flags);
		expired = avd_slot_expired(&avd->frames[i], jiffies);
		spin_unlock_irqrestore(&avd->queue_lock, flags);
		if (expired) {
			struct avd_frame *t = &avd->frames[i];

			dev_err(avd->dev, "frame tag %#x timed out (%p4cc, state %d, %u in flight, %s)\n",
				t->tag, t->ctx ? &t->ctx->coded_fmt_desc->fourcc : &(u32){0},
				t->state, avd->inflight, avd->backend->name);
			avd_qfw_debug(avd, t, "timeout");
			avd_reset_queue(avd, -ETIMEDOUT);
			break;
		}
	}
	avd_retire_vp(avd);
	avd_retire(avd);
	for (i = 0; i < AVD_MAX_FRAMES; i++) {
		struct avd_frame *f = &avd->frames[i];
		int ret;

		if (f->state != AVD_FRAME_RESERVED)
			continue;
		ret = avd_send_frame(avd, f);
		if (ret && ret != -EBUSY) {
			spin_lock_irqsave(&avd->queue_lock, flags);
			f->status = ret;
			f->state = AVD_FRAME_DONE;
			spin_unlock_irqrestore(&avd->queue_lock, flags);
		}
	}
	avd_retire(avd);
	/* Restart a job that found no free slot. */
	spin_lock_irqsave(&avd->queue_lock, flags);
	ctx = avd->inflight < avd_max_inflight(avd) ? avd->deferred_ctx : NULL;
	if (ctx)
		avd->deferred_ctx = NULL;
	spin_unlock_irqrestore(&avd->queue_lock, flags);
	if (ctx)
		schedule_work(&ctx->submit_work);
	/* Freed slots/dependencies must wake contexts whose job_ready was false. */
	list_for_each_entry(ctx, &avd->contexts, queue_node)
		v4l2_m2m_try_schedule(ctx->fh.m2m_ctx);
	spin_lock_irqsave(&avd->queue_lock, flags);
	pending = avd->inflight != 0;
	for (i = 0; i < AVD_MAX_FRAMES; i++) {
		struct avd_frame *f = &avd->frames[i];
		unsigned long remaining;

		if (f->state == AVD_FRAME_FREE)
			continue;
		remaining = time_after(f->deadline, jiffies) ? f->deadline - jiffies : 0;
		if (f->state == AVD_FRAME_RESERVED)
			remaining = min(remaining, AVD_RETRY_DELAY);
		delay = min(delay, remaining);
	}
	spin_unlock_irqrestore(&avd->queue_lock, flags);
	if (pending)
		mod_delayed_work(system_dfl_wq, &avd->queue_work, delay);
 unlock:
	mutex_unlock(&avd->queue_mutex);
}

void avd_frame_complete(struct avd_dev *avd, u32 tag, int status)
{
	struct avd_frame *f = &avd->frames[tag & (AVD_MAX_FRAMES - 1)];
	unsigned long flags;
	bool accepted;

	spin_lock_irqsave(&avd->queue_lock, flags);
	accepted = avd_slot_complete(f, tag, status);
	spin_unlock_irqrestore(&avd->queue_lock, flags);
	if (accepted)
		mod_delayed_work(system_dfl_wq, &avd->queue_work, 0);
}

/* No buffer, FIFO or descriptor ownership is released by a VP event. */
void avd_frame_vp_complete(struct avd_dev *avd, u32 tag)
{
	struct avd_frame *f = &avd->frames[tag & (AVD_MAX_FRAMES - 1)];
	unsigned long flags;
	bool accepted;

	spin_lock_irqsave(&avd->queue_lock, flags);
	accepted = f->state == AVD_FRAME_SUBMITTED && f->tag == tag && !f->vp_done;
	if (accepted)
		f->vp_done = true;
	spin_unlock_irqrestore(&avd->queue_lock, flags);
	if (accepted)
		mod_delayed_work(system_dfl_wq, &avd->queue_work, 0);
}

int avd_queue_ready(void *priv)
{
	struct avd_ctx *ctx = priv;
	struct avd_dev *avd = ctx->dev;
	unsigned long flags;
	unsigned int i;
	bool ready;

	spin_lock_irqsave(&avd->queue_lock, flags);
	/*
	 * A retired decoder still schedules: submission fails
	 * those frames at once, so clients see errors instead of waiting.
	 */
	/*
	 * Slot capacity is not a readiness condition: every context with work
	 * enters the m2m job queue, which runs them in order, and a job that
	 * finds no free slot is deferred until one retires. Gating readiness on
	 * one reserved context let the first context in the list win every
	 * time and starved the others (multi-stream decodes timing out).
	 */
	ready = !READ_ONCE(ctx->draining) && !READ_ONCE(avd->stopping);
	if (ready && ctx->next_dep == AVD_DEP_PREV_DONE) {
		/* Legacy codecs may hold several pictures in separate banks. */
		unsigned int limit = ctx->coded_fmt_desc && !ctx->coded_fmt_desc->frame_ops ?
				     max(ctx->max_frames, 1U) : 1, held = 0;

		for (i = 0; i < AVD_MAX_FRAMES; i++)
			if (avd->frames[i].state != AVD_FRAME_FREE &&
			    avd->frames[i].ctx == ctx)
				held++;
		ready = held < limit;
	}
	if (ready && ctx->coded_fmt_desc && !ctx->coded_fmt_desc->frame_ops &&
	    ctx->coded_fmt_desc->ops->ready)
		ready = ctx->coded_fmt_desc->ops->ready(ctx);
	spin_unlock_irqrestore(&avd->queue_lock, flags);
	return ready;
}

static void avd_queue_submit_work(struct work_struct *work)
{
	struct avd_ctx *ctx = container_of(work, struct avd_ctx, submit_work);
	struct avd_dev *avd = ctx->dev;
	const struct avd_codec_frame_ops *ops = ctx->coded_fmt_desc->frame_ops;
	struct avd_frame *f = NULL;
	unsigned long flags;
	unsigned int i;
	int ret;

	mutex_lock(&avd->queue_mutex);
	/* A queued retry can outlive drain/abort finishing its m2m dispatch. */
	if (v4l2_m2m_get_curr_priv(avd->m2m_dev) != ctx)
		goto unlock;
	spin_lock_irqsave(&avd->queue_lock, flags);
	if (avd->deferred_ctx == ctx)
		avd->deferred_ctx = NULL;
	if (avd->stopping || ctx->draining || READ_ONCE(avd->faulted) || ctx->broken ||
	    (avd->rest_until && time_before(jiffies, avd->rest_until))) {
		struct vb2_v4l2_buffer *src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);

		spin_unlock_irqrestore(&avd->queue_lock, flags);
		/* Complete the request's controls too, or its fd never signals. */
		if (src && src->vb2_buf.req_obj.req)
			v4l2_ctrl_request_complete(src->vb2_buf.req_obj.req, &ctx->ctrl_hdl);
		v4l2_m2m_buf_done_and_job_finish(avd->m2m_dev, ctx->fh.m2m_ctx,
						 VB2_BUF_STATE_ERROR);
		goto unlock;
	}
	if (avd->inflight < avd_max_inflight(avd)) {
		for (i = 0; i < AVD_MAX_FRAMES; i++) {
			if (avd->frames[i].state == AVD_FRAME_FREE) {
				f = &avd->frames[i];
				break;
			}
		}
	}
	if (!f) {
		/* Keep the m2m job; the worker restarts it when a slot frees. */
		avd->deferred_ctx = ctx;
		spin_unlock_irqrestore(&avd->queue_lock, flags);
		mod_delayed_work(system_dfl_wq, &avd->queue_work, 0);
		goto unlock;
	}
	ret = avd_slot_reserve(f, ctx, ++avd->frame_seq);
	if (!ret)
		avd->inflight++;
	spin_unlock_irqrestore(&avd->queue_lock, flags);
	if (WARN_ON(ret)) {
		v4l2_m2m_buf_done_and_job_finish(avd->m2m_dev, ctx->fh.m2m_ctx,
						 VB2_BUF_STATE_ERROR);
		goto unlock;
	}
	ret = pm_runtime_resume_and_get(avd->dev);
	if (ret < 0)
		goto fail_no_pm;
	f->src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	f->dst = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);
	/*
	 * Partial slices retain CAPTURE through the ordinary m2m HOLD API.
	 * Final slices detach both buffers and release dispatch atomically;
	 * neither DMA-owned buffer is completed by that early handoff.
	 */
	f->scheduler_held = (ctx->fh.m2m_ctx->out_q_ctx.q.subsystem_flags &
			     VB2_V4L2_FL_SUPPORTS_M2M_HOLD_CAPTURE_BUF) &&
			    (f->src->flags & V4L2_BUF_FLAG_M2M_HOLD_CAPTURE_BUF);
	f->frame_num = f->tag;
	f->deadline = jiffies + AVD_FRAME_TIMEOUT;
	if (avd->rest_until) {
		/* Reset while still parked; retain the recovery pacing at expiry. */
		if (avd->last_recover &&
		    time_before(jiffies, avd->last_recover +
				msecs_to_jiffies(AVD_RECOVER_GAP_MS)))
			msleep(AVD_RECOVER_GAP_MS);
		ret = avd_pm_recover(avd);
		avd->last_recover = jiffies ?: 1;
		if (ret) {
			WRITE_ONCE(avd->faulted, true);
			dev_err(avd->dev, "decoder retired after a failed wake (%d)\n", ret);
			goto fail;
		}
		avd->rest_until = 0;
		avd->storm_count = 0;
		avd_pm_enable_irq(avd);
	}
	memset(f->cmd, 0, sizeof(f->cmd));
	if (ops) {
		if (!f->prog->cpu) {
			ret = avd_buf_alloc(avd, f->prog, AVD_PROGRAM_SIZE);
			if (ret)
				goto fail;
		}
		/* NULL fp is also the codec-facing indication of an open program. */
		f->fp = NULL;
		if (avd_backend_is_apple(avd)) {
			struct avd_buf *fp = avd->frame_fp[f->idx];

			if (!fp) {
				fp = devm_kzalloc(avd->dev, sizeof(*fp), GFP_KERNEL);
				avd->frame_fp[f->idx] = fp;
			}
			f->fp = fp;
			if (!f->fp) {
				ret = -ENOMEM;
				goto fail;
			}
			if (!f->fp->cpu) {
				ret = avd_buf_alloc(avd, f->fp, avd->variant->fp_size);
				if (ret)
					goto fail;
			}
		}
		if (f->src->vb2_buf.req_obj.req) {
			ret = v4l2_ctrl_request_setup(f->src->vb2_buf.req_obj.req, &ctx->ctrl_hdl);
			if (ret)
				goto fail;
		}
		ret = ops->prepare(ctx, f);
	} else {
		avd->building = f;
		avd->compat_held = false;
		ret = ctx->coded_fmt_desc->ops->run(ctx);
		avd->building = NULL;
	}
	if (ret)
		goto fail;
	ctx->next_dep = ops ? f->dep : AVD_DEP_PREV_DONE;
	if ((ops && (f->src->flags & V4L2_BUF_FLAG_M2M_HOLD_CAPTURE_BUF)) ||
	    (!ops && avd->compat_held)) {
		if (ops && ops->retire)
			ops->retire(ctx, f, VB2_BUF_STATE_DONE);
		else if (!ops && ctx->coded_fmt_desc->ops->done)
			ctx->coded_fmt_desc->ops->done(ctx, f->src, f->dst, VB2_BUF_STATE_DONE);
		if (ops && f->src->vb2_buf.req_obj.req)
			v4l2_ctrl_request_complete(f->src->vb2_buf.req_obj.req, &ctx->ctrl_hdl);
		spin_lock_irqsave(&avd->queue_lock, flags);
		f->state = AVD_FRAME_FREE;
		f->ctx = NULL;
		avd->inflight--;
		spin_unlock_irqrestore(&avd->queue_lock, flags);
		avd_pm_queue_changed(avd);
		pm_runtime_mark_last_busy(avd->dev);
		pm_runtime_put_autosuspend(avd->dev);
		v4l2_m2m_buf_done_and_job_finish(avd->m2m_dev, ctx->fh.m2m_ctx,
						 VB2_BUF_STATE_DONE);
		goto unlock;
	}
	v4l2_m2m_buf_copy_metadata(f->src, f->dst);
	ret = avd_send_frame(avd, f);
	if (ret && ret != -EBUSY) {
		spin_lock_irqsave(&avd->queue_lock, flags);
		f->status = ret;
		f->state = AVD_FRAME_DONE;
		spin_unlock_irqrestore(&avd->queue_lock, flags);
	}
	/* Atomically detach both buffers to the slot and release dispatch. */
	if (!f->scheduler_held)
		v4l2_m2m_job_finish_detach(avd->m2m_dev, ctx->fh.m2m_ctx);
	mod_delayed_work(system_dfl_wq, &avd->queue_work, 0);
	goto unlock;
 fail:
	/* Failed preparation still transfers its buffers before retirement. */
	spin_lock_irqsave(&avd->queue_lock, flags);
	f->status = ret;
	f->state = AVD_FRAME_DONE;
	spin_unlock_irqrestore(&avd->queue_lock, flags);
	if (!f->scheduler_held)
		v4l2_m2m_job_finish_detach(avd->m2m_dev, ctx->fh.m2m_ctx);
	mod_delayed_work(system_dfl_wq, &avd->queue_work, 0);
	goto unlock;
 fail_no_pm:
	spin_lock_irqsave(&avd->queue_lock, flags);
	f->state = AVD_FRAME_FREE;
	f->ctx = NULL;
	avd->inflight--;
	spin_unlock_irqrestore(&avd->queue_lock, flags);
	avd_pm_queue_changed(avd);
	v4l2_m2m_buf_done_and_job_finish(avd->m2m_dev, ctx->fh.m2m_ctx,
					 VB2_BUF_STATE_ERROR);
 unlock:
	mutex_unlock(&avd->queue_mutex);
}

void avd_queue_run(void *priv)
{
	struct avd_ctx *ctx = priv;

	/* try_schedule() may invoke device_run synchronously under queue_mutex. */
	schedule_work(&ctx->submit_work);
}

/* True while any slot still belongs to @ctx; the caller holds no queue lock. */
static bool avd_ctx_has_frames(struct avd_dev *avd, struct avd_ctx *ctx)
{
	unsigned long flags;
	unsigned int i;
	bool busy = false;

	spin_lock_irqsave(&avd->queue_lock, flags);
	for (i = 0; i < AVD_MAX_FRAMES; i++)
		if (avd->frames[i].state != AVD_FRAME_FREE && avd->frames[i].ctx == ctx)
			busy = true;
	spin_unlock_irqrestore(&avd->queue_lock, flags);
	return busy;
}

void avd_queue_drain(struct avd_ctx *ctx)
{
	struct avd_dev *avd = ctx->dev;
	unsigned long flags;
	unsigned int i;
	bool finish_deferred = false;
	int ret;

	/* Session stop/reset accesses registers even when there are no frames. */
	ret = pm_runtime_resume_and_get(avd->dev);
	avd_pm_drain_begin(avd);
	mutex_lock(&avd->queue_mutex);
	spin_lock_irqsave(&avd->queue_lock, flags);
	WRITE_ONCE(ctx->draining, true);
	if (avd->deferred_ctx == ctx) {
		avd->deferred_ctx = NULL;
		finish_deferred = true;
	}
	/* Prepared but never handed to the decoder: nothing to wait for. */
	for (i = 0; i < AVD_MAX_FRAMES; i++) {
		struct avd_frame *f = &avd->frames[i];

		if (f->ctx == ctx && f->state == AVD_FRAME_RESERVED) {
			f->status = -ECANCELED;
			f->state = AVD_FRAME_DONE;
		}
	}
	spin_unlock_irqrestore(&avd->queue_lock, flags);
	/* A deferred job is still m2m's running job: end it so abort can wait. */
	if (finish_deferred)
		v4l2_m2m_buf_done_and_job_finish(avd->m2m_dev, ctx->fh.m2m_ctx,
						 VB2_BUF_STATE_ERROR);
	avd_retire(avd);

	/*
	 * Frames the decoder already owns finish on their own, or the queue
	 * worker times them out and recovers. Closing one stream must not reset
	 * the decoder under every other stream's frames: that reset, with their
	 * DMA in flight, preceded the J616s hangs. Wait for this context's own
	 * frames instead, without the queue mutex so the worker can retire them.
	 */
	if (ret >= 0 && !avd->stopping && avd_ctx_has_frames(avd, ctx)) {
		mod_delayed_work(system_dfl_wq, &avd->queue_work, 0);
		mutex_unlock(&avd->queue_mutex);
		wait_event_timeout(avd->pm.idle_wait, !avd_ctx_has_frames(avd, ctx),
				   AVD_FRAME_TIMEOUT + msecs_to_jiffies(1000));
		mutex_lock(&avd->queue_mutex);
		avd_retire(avd);
	}
	/* Last resort (worker stopped or wedged): cancel through a reset. */
	for (i = 0; ret >= 0 && i < AVD_MAX_FRAMES; i++) {
		if (avd->frames[i].state != AVD_FRAME_FREE && avd->frames[i].ctx == ctx) {
			dev_warn(avd->dev, "stream close: frames still owned by the decoder, resetting\n");
			if (avd->backend->abort)
				avd->backend->abort(ctx);
			avd_reset_queue(avd, -ECANCELED);
			break;
		}
	}
	avd_retire(avd);
	if (ret >= 0 && ctx->session_started && avd->backend->session_stop &&
	    ctx->session_generation == avd->fw_generation)
		avd->backend->session_stop(ctx);
	ctx->session_started = false;
	if (!avd->stopping)
		mod_delayed_work(system_dfl_wq, &avd->queue_work, 0);
	mutex_unlock(&avd->queue_mutex);
	avd_pm_drain_end(avd);
	if (ret >= 0) {
		pm_runtime_mark_last_busy(avd->dev);
		pm_runtime_put_autosuspend(avd->dev);
	}
}

void avd_queue_abort(void *priv)
{
	avd_queue_drain(priv);
}

void avd_queue_add_ctx(struct avd_ctx *ctx)
{
	struct avd_ctx *other;
	u32 id;

	INIT_WORK(&ctx->submit_work, avd_queue_submit_work);
	mutex_lock(&ctx->dev->queue_mutex);
	/* Borage decode prefix has ten client bits. */
	for (id = 0; id < 1024; id++) {
		bool used = false;

		list_for_each_entry(other, &ctx->dev->contexts, queue_node)
			used |= other->client_id == id;
		if (!used)
			break;
	}
	ctx->client_id = id;
	list_add_tail(&ctx->queue_node, &ctx->dev->contexts);
	mutex_unlock(&ctx->dev->queue_mutex);
}

void avd_queue_del_ctx(struct avd_ctx *ctx)
{
	avd_queue_drain(ctx);
	mutex_lock(&ctx->dev->queue_mutex);
	if (ctx->dev->compat_ctx == ctx)
		ctx->dev->compat_ctx = NULL;
	list_del_init(&ctx->queue_node);
	mutex_unlock(&ctx->dev->queue_mutex);
}

int avd_queue_init(struct avd_dev *avd)
{
	unsigned int i;

	spin_lock_init(&avd->queue_lock);
	mutex_init(&avd->queue_mutex);
	INIT_LIST_HEAD(&avd->contexts);
	INIT_DELAYED_WORK(&avd->queue_work, avd_queue_worker);
	for (i = 0; i < AVD_MAX_FRAMES; i++) {
		struct avd_frame *f = &avd->frames[i];

		f->idx = i;
		f->prog = devm_kzalloc(avd->dev, sizeof(*f->prog), GFP_KERNEL);
		if (!f->prog)
			return -ENOMEM;
	}
	return 0;
}

void avd_queue_cleanup(struct avd_dev *avd)
{
	unsigned int i;

	mutex_lock(&avd->queue_mutex);
	WRITE_ONCE(avd->stopping, true);
	if (avd->inflight) {
		avd_reset_queue(avd, -ESHUTDOWN);
		avd_retire(avd);
	}
	mutex_unlock(&avd->queue_mutex);
	cancel_delayed_work_sync(&avd->queue_work);
	for (i = 0; i < AVD_MAX_FRAMES; i++) {
		avd_buf_free(avd, avd->frames[i].prog);
		if (avd->frame_fp[i])
			avd_buf_free(avd, avd->frame_fp[i]);
	}
}
