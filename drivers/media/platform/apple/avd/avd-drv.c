/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Apple Video Decoder driver
 *
 * Copyright The Asahi Linux Contributors
 *
 * Based on rkvdec driver by Collabora, Ltd.
 * Copyright (C) 2019 Collabora, Ltd.
 * Based on rkvdec driver by Google LLC. (Tomasz Figa <tfiga@chromium.org>)
 * Based on s5p-mfc driver by Samsung Electronics Co., Ltd.
 * Copyright (C) 2011 Samsung Electronics Co., Ltd.
 */

#include <linux/pm_runtime.h>
#include <linux/iommu.h>
#include <linux/reset.h>
#include <linux/delay.h>

#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-v4l2.h>

#include "avd.h"
#include "avd-inst.h"
#include "avd-regs.h"

#define AVD_CODEC_INFO_MAX_SIZE	0x40

static void calc_tile_meta(u32 w, u32 h, u32 bpb, u32 tile_dim,
			   u32 meta_hdr_bytes, u32 *tile, u32 *meta)
{
	u32 tiles_width, tiles_height, meta_tile_w, meta_tile_h, tile_bytes;
	tiles_width = DIV_ROUND_UP(w, tile_dim);
	tiles_height = DIV_ROUND_UP(h, tile_dim);
	tile_bytes = tile_dim * tile_dim * DIV_ROUND_UP(bpb, 8);
	*tile = ALIGN(tiles_width * tiles_height * tile_bytes, 16);

	meta_tile_w = roundup_pow_of_two(tiles_width);
	meta_tile_h = roundup_pow_of_two(tiles_height);

	*meta = ALIGN(meta_tile_w * meta_tile_h * meta_hdr_bytes, 16);
}

void fill_comp(struct avd_comp *comp, enum avd_image_fmt image_fmt, u32 width,
	       u32 height)
{
	u32 y_meta, y, uv_meta, uv;
	int bit_depth;

	switch (image_fmt) {
	case AVD_IMG_FMT_420_10BIT:
	case AVD_IMG_FMT_422_10BIT:
		bit_depth = 10;
		break;
	default:
		bit_depth = 8;
		break;
	}

	/* y has 32x32 tiles and 32 bytes of metadata per tile */
	calc_tile_meta(width, height, bit_depth, 32, 32, &y, &y_meta);
	/* uv has 16x16 tiles and 8 bytes of metadata per tile; 4:2:2 chroma is full height */
	calc_tile_meta(width / 2,
		       (image_fmt == AVD_IMG_FMT_422_8BIT ||
			image_fmt == AVD_IMG_FMT_422_10BIT) ? height : height / 2,
		       bit_depth * 2, 16, 8, &uv, &uv_meta);

	/* output like DCP driver expects */
	comp->offsets[0] = y;
	comp->offsets[1] = 0;
	comp->offsets[2] = y + y_meta + uv;
	comp->offsets[3] = y + y_meta;

	comp->size = y_meta + y + uv_meta + uv;
}

int avd_buf_alloc(struct avd_dev *avd, struct avd_buf *buf, size_t size)
{
	/* Keep size equal to the payload length used by the H.264 slice path. */
	avd_buf_free(avd, buf);

	if (size <= 0)
		return -ENOMEM;

	buf->size = size;
	buf->cpu =
		dma_alloc_coherent(avd->dev, buf->size, &buf->addr, GFP_KERNEL);
	if (!buf->cpu) {
		memset(buf, 0, sizeof(*buf));
		return -ENOMEM;
	}
	return 0;
}

void avd_buf_free(struct avd_dev *avd, struct avd_buf *buf)
{
	if (buf->cpu)
		dma_free_coherent(avd->dev, buf->size, buf->cpu, buf->addr);
	memset(buf, 0, sizeof(*buf));
}

struct avd_decoded_buffer *
avd_get_ref_buf(struct avd_ctx *ctx, struct vb2_v4l2_buffer *dst, u64 timestamp)
{
	struct v4l2_m2m_ctx *m2m_ctx = ctx->fh.m2m_ctx;
	struct vb2_queue *cap_q = &m2m_ctx->cap_q_ctx.q;
	struct vb2_buffer *buf;

	/*
	 * If a ref is unused or invalid, address of current destination
	 * buffer is returned.
	 */
	buf = vb2_find_buffer(cap_q, timestamp);
	if (!buf)
		buf = &dst->vb2_buf;

	return vb2_to_avd_decoded_buf(buf);
}

static int avd_wait_submission_queue(struct avd_ctx *ctx, int vp)
{
	struct avd_dev *avd = ctx->dev;
	u32 max = readl_relaxed(avd->ctrl +
				avd->variant->submit_queue_max_offset + vp * 4);
	u32 cur = readl_relaxed(
		avd->ctrl + avd->variant->submit_queue_status_offset + vp * 4);

	if (cur == max) {
		dev_err(avd->dev, "instruction que full! %d/%d", cur, max);
		return 1;
	}

	if (cur >= max / 2) {
		if (avd->variant->quirks & AVD_QUIRK_V5_QUEUE_PACING)
			usleep_range(100, 150);
		else
			usleep_range(500, 650);
	}
	return 0;
}

int avd_init_job(struct avd_ctx *ctx, enum avd_codec codec, size_t segments)
{
	int ret = 0;
	struct avd_job *job = &ctx->job;

	job->codec = codec;
	job->num = 0;
	/* A table that was built but never submitted still belongs to us. */
	kvfree(job->segments);
	job->segments = kvcalloc(segments, sizeof(*job->segments), GFP_KERNEL);
	if (!job->segments)
		ret = -ENOMEM;
	return ret;
}

int avd_submit_job(struct avd_ctx *ctx)
{
	struct avd_dev *avd = ctx->dev;
	struct avd_job *sub = &ctx->job;
	struct avd_segment *segments, *seg;
	unsigned long flags;
	size_t num;
	int i, idx = 0, vp = 0;
	void __iomem *reg;
	u32 exec_mask = sub->codec == AVD_CODEC_VP9 &&
					avd->variant->revision == 3 ?
				AVD_OP_EXEC_REV3_VP9_MASK :
				0;
	u32 exec_rev_flag =
		AVD_OP_EXEC_FLAG_START_REV4(avd->variant->revision == 4) |
		AVD_OP_EXEC_FLAG_START_REV3(avd->variant->revision == 3);

	ctx->fifo_idx = 0;
	for (i = 0; i < sub->codec; i++)
		vp += avd->variant->vp_slots[i];
	if (avd->variant->vp_slot_base[sub->codec])
		vp = avd->variant->vp_slot_base[sub->codec];

	/*
	 * avd_device_run() keeps this job from being finished (and the next one
	 * from starting) until it returns, and arms the watchdog itself once the
	 * job belongs to the hardware. Still walk a local copy of the table.
	 */
	segments = sub->segments;
	num = sub->num;
	if (WARN_ON_ONCE(!segments))
		return -EINVAL;

	avd->variant->configure_stream(avd, ctx->inst.addr, ctx->fifo_idx, vp);
	reg = avd->ctrl + avd->variant->vp_slot_offset + vp * 4;

	/* the first segment is always the header (needs special handling) */

	writel(AVD_OP_EXEC | exec_mask | exec_rev_flag |
		       AVD_OP_EXEC_FIFO_IDX(ctx->fifo_idx),
	       reg);
	seg = &segments[idx++];
	for (i = 0; i < seg->num; i++)
		writel(seg->instructions[i], reg);
	for (; idx <= num; idx++) {
		seg = &segments[idx];
		for (i = 0; i < seg->num; i++)
			writel(seg->instructions[i], reg);
		if ((!(avd->variant->quirks & AVD_QUIRK_V5_QUEUE_PACING) ||
		     sub->codec == AVD_CODEC_HEVC) &&
		    avd_wait_submission_queue(ctx, vp))
			break;
		if (idx == num) {
			/* A completion from here on belongs to this job. */
			spin_lock_irqsave(&avd->job_lock, flags);
			avd->job_end_sent = true;
			spin_unlock_irqrestore(&avd->job_lock, flags);
		}
		writel(AVD_OP_EXEC | exec_mask |
			       AVD_OP_EXEC_FLAG_END(idx == num),
		       reg);
	}

	if (sub->segments == segments)
		sub->segments = NULL;
	kvfree(segments);
	return 0;
}

static int avd_reset(struct avd_dev *avd)
{
	int ret = 0;

	ret = pm_runtime_resume_and_get(avd->dev);
	if (ret < 0)
		return ret;

	ret = reset_control_reset(avd->rstc);
	if (ret)
		dev_err(avd->dev, "reset: failed: %d", ret);

	if (avd->empty_domain) {
		iommu_attach_device(avd->empty_domain, avd->dev);
		iommu_detach_device(avd->empty_domain, avd->dev);
	}

	ret = avd_boot(avd);
	if (ret)
		dev_err(avd->dev, "reset: failed to boot");

	pm_runtime_put_autosuspend(avd->dev);

	return ret;
}

static void avd_watchdog_func(struct work_struct *work)
{
	struct avd_dev *avd;
	struct avd_ctx *ctx;
	unsigned long flags;
	bool claimed;
	int ret;
	ctx = container_of(to_delayed_work(work), struct avd_ctx,
			   watchdog_work);
	if (!ctx)
		return;

	avd = ctx->dev;

	/*
	 * Time out only the job this watchdog was armed for, and only while the
	 * hardware still owns it. Otherwise that job already finished, and
	 * resetting the hardware would break whichever job is running now.
	 */
	spin_lock_irqsave(&avd->job_lock, flags);
	claimed = avd->job_ctx == ctx && avd->job_seq == ctx->wd_seq &&
		  avd->job_state == AVD_JOB_SUBMITTED;
	if (claimed)
		avd->job_state = AVD_JOB_FINISHING;
	spin_unlock_irqrestore(&avd->job_lock, flags);
	dev_dbg(avd->dev, "avd-job: watchdog seq %llu: %s\n", ctx->wd_seq,
		claimed ? "timeout" : "stale, ignored");
	if (!claimed)
		return;

	dev_err(avd->dev, "Frame processing timed out!");

	writel(0, avd->mbox + AVD_REG_MBOX_IRQ_ENABLE);
	ret = avd_reset(avd);
	if (ret)
		dev_err(avd->dev, "failed to reset: %d", ret);

	avd_job_finish(ctx, VB2_BUF_STATE_ERROR);
}

static irqreturn_t avd_irq_handler(int irq, void *data)
{
	struct avd_dev *avd = data;
	struct avd_ctx *ctx = v4l2_m2m_get_curr_priv(avd->m2m_dev);

	enum vb2_buffer_state state;
	unsigned long flags;
	bool finish = false, stale = false;
	u32 status;
	if (!ctx)
		return IRQ_HANDLED;

	status = readl(avd->mbox + AVD_REG_MBOX1_RETRIEVE);

	writel(AVD_MBOX1_NOT_EMPTY, avd->mbox + AVD_REG_MBOX_IRQ_CLR);

	if (status & 0x10000) { /* dbg */
		dev_warn(avd->dev, "no handler for IRQ: %3d", status &~0x10000);
		writel_relaxed(0, avd->mbox + AVD_REG_MBOX_IRQ_ENABLE);
		return IRQ_HANDLED;
	}

	if (status & 0x1000) {
		/* pp is done ! we are done */
		state = VB2_BUF_STATE_DONE;
	} else if (status & 0x100) {
		/* a vp is done, kick the pp and hope for the best */
		if(ctx->coded_fmt_desc->ops->submit)
			ctx->coded_fmt_desc->ops->submit(ctx);
		goto done;
	} else {
		dev_err(avd->dev, "H%d error", status);
		/*
		 * avd-fw err(n): VP n raised its error interrupt and stopped,
		 * and the PP only starts on a VP done, so the job cannot
		 * complete any more. Run the watchdog now instead of after its
		 * timeout: it resets the AVD and fails only this frame, and it
		 * ignores a job the hardware does not own yet.
		 */
		mod_delayed_work(system_wq, &ctx->watchdog_work, 0);
		goto done;
	}

	/*
	 * Finish only a job the hardware owns. While avd_device_run() is still
	 * setting up or submitting, a completion is left over from an older job
	 * unless this job's END command already went out; then remember it and
	 * let avd_device_run() finish the job when it returns.
	 */
	spin_lock_irqsave(&avd->job_lock, flags);
	if (avd->job_ctx != ctx) {
		stale = true;
	} else if (avd->job_state == AVD_JOB_SUBMITTED) {
		avd->job_state = AVD_JOB_FINISHING;
		finish = true;
	} else if (avd->job_state == AVD_JOB_RUNNING && avd->job_end_sent) {
		avd->job_pending = true;
		avd->job_pending_result = state;
	} else {
		stale = true;
	}
	spin_unlock_irqrestore(&avd->job_lock, flags);
	dev_dbg(avd->dev, "avd-job: irq %#x: %s\n", status,
		finish ? "finish" : stale ? "stale, ignored" : "pending");
	if (stale)
		dev_warn_ratelimited(avd->dev, "ignoring stale completion");
	if (finish) {
		cancel_delayed_work(&ctx->watchdog_work);
		avd_job_finish(ctx, state);
	}

done:
	return IRQ_HANDLED;
}

static void avd_device_run(void *priv)
{
	struct avd_ctx *ctx = priv;
	struct avd_dev *avd = ctx->dev;
	const struct avd_coded_fmt_desc *desc = ctx->coded_fmt_desc;
	enum vb2_buffer_state result = VB2_BUF_STATE_ERROR;
	const char *outcome = "already finished";
	unsigned long flags;
	bool finish = false;
	u64 seq;
	int ret;

	if (WARN_ON(!desc))
		return;

	/*
	 * IRQ completion cannot wait for an already running watchdog. Drain it
	 * here, before reusing wd_seq, so an old callback cannot time out this
	 * context's next job. device_run is sleepable; hold no job_lock here.
	 */
	cancel_delayed_work_sync(&ctx->watchdog_work);

	ret = pm_runtime_resume_and_get(avd->dev);
	if (ret < 0) {
		avd_job_finish_no_pm(ctx, VB2_BUF_STATE_ERROR);
		return;
	}

	/*
	 * While the job is RUNNING neither completion IRQs nor the watchdog may
	 * finish it: v4l2-mem2mem would start the next job on a kworker while
	 * this one still builds and submits ctx->job.
	 */
	spin_lock_irqsave(&avd->job_lock, flags);
	seq = ++avd->job_seq;
	avd->job_ctx = ctx;
	avd->job_state = AVD_JOB_RUNNING;
	avd->job_end_sent = false;
	avd->job_pending = false;
	spin_unlock_irqrestore(&avd->job_lock, flags);
	dev_dbg(avd->dev, "avd-job: run seq %llu\n", seq);

	ret = desc->ops->run(ctx);
	if (ret) {
		dev_dbg(avd->dev, "avd-job: run seq %llu failed: %d\n", seq, ret);
		avd_job_finish(ctx, VB2_BUF_STATE_ERROR);
		return;
	}

	/*
	 * The run either finished the job itself (held capture buffer) or handed
	 * it to the hardware: arm the watchdog, or finish right away if the
	 * completion already arrived.
	 */
	spin_lock_irqsave(&avd->job_lock, flags);
	if (avd->job_seq == seq && avd->job_state == AVD_JOB_RUNNING) {
		if (avd->job_pending) {
			avd->job_state = AVD_JOB_FINISHING;
			result = avd->job_pending_result;
			finish = true;
			outcome = "completed during submission";
		} else {
			avd->job_state = AVD_JOB_SUBMITTED;
			ctx->wd_seq = seq;
			schedule_delayed_work(&ctx->watchdog_work, msecs_to_jiffies(2000));
			outcome = "submitted";
		}
	}
	spin_unlock_irqrestore(&avd->job_lock, flags);
	dev_dbg(avd->dev, "avd-job: run seq %llu: %s\n", seq, outcome);

	if (finish)
		avd_job_finish(ctx, result);
}

/*
 * Capture buffers are found by timestamp and handed to the hardware by DMA
 * address, so nothing limits them to VB2_MAX_FRAME. A larger pool lets
 * userspace decode ahead to ride out bitrate spikes the hardware can't
 * decode in real time (e.g. 150+ Mbit/s 4K60 H.264).
 */
static unsigned int max_capture_buffers = 128;
module_param(max_capture_buffers, uint, 0444);
MODULE_PARM_DESC(max_capture_buffers,
		 "Maximum number of capture buffers per context (default 128)");

static int avd_queue_init(void *priv, struct vb2_queue *src_vq,
			  struct vb2_queue *dst_vq)
{
	struct avd_ctx *ctx = priv;
	int ret;

	/* TODO: what flags to give to dart */
	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	src_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	src_vq->drv_priv = ctx;
	src_vq->ops = &avd_queue_ops;
	src_vq->mem_ops = &vb2_dma_contig_memops;

	src_vq->dma_attrs = 0 /* DMA_ATTR_NO_KERNEL_MAPPING */;
	src_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock = &ctx->dev->vdev_lock;
	src_vq->dev = ctx->dev->v4l2_dev.dev;
	src_vq->supports_requests = true;
	src_vq->allow_cache_hints = true;

	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->bidirectional = true;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->dma_attrs = 0;
	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	dst_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	dst_vq->drv_priv = ctx;
	dst_vq->ops = &avd_queue_ops;
	dst_vq->buf_struct_size = sizeof(struct avd_decoded_buffer);
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock = &ctx->dev->vdev_lock;
	dst_vq->dev = ctx->dev->v4l2_dev.dev;
	dst_vq->allow_cache_hints = true;
	dst_vq->max_num_buffers = max(max_capture_buffers, VB2_MAX_FRAME);

	return vb2_queue_init(dst_vq);
}

static int avd_open(struct file *filp)
{
	struct avd_dev *avd = video_drvdata(filp);
	struct avd_ctx *ctx;
	int ret;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->dev = avd;

	ret = avd_buf_alloc(avd, &ctx->inst, fifo_size());
	if (ret)
		goto err_free_ctx;

	INIT_DELAYED_WORK(&ctx->watchdog_work, avd_watchdog_func);

	avd_reset_coded_fmt(ctx);
	avd_reset_decoded_fmt(ctx);

	v4l2_fh_init(&ctx->fh, video_devdata(filp));

	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(avd->m2m_dev, ctx, avd_queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		goto err_free_ctx;
	}

	ret = avd_init_ctrls(ctx);
	if (ret)
		goto err_cleanup_m2m_ctx;

	v4l2_fh_add(&ctx->fh, filp);

	return 0;

err_cleanup_m2m_ctx:
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);

err_free_ctx:
	avd_buf_free(avd, &ctx->inst);
	kfree(ctx);
	return ret;
}

static int avd_release(struct file *filp)
{
	struct avd_ctx *ctx = file_to_ctx(filp);

	v4l2_fh_del(&ctx->fh, filp);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	/* The job has finished; its watchdog must not run on the freed ctx. */
	cancel_delayed_work_sync(&ctx->watchdog_work);
	v4l2_ctrl_handler_free(&ctx->ctrl_hdl);
	v4l2_fh_exit(&ctx->fh);
	avd_buf_free(ctx->dev, &ctx->inst);
	kvfree(ctx->job.segments);
	kfree(ctx);

	return 0;
}

static const struct v4l2_file_operations avd_fops = {
	.owner = THIS_MODULE,
	.open = avd_open,
	.release = avd_release,
	.poll = v4l2_m2m_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = v4l2_m2m_fop_mmap,
};

static const struct v4l2_m2m_ops avd_m2m_ops = {
	.device_run = avd_device_run,
};

static const struct media_device_ops avd_media_ops = {
	.req_validate = vb2_request_validate,
	.req_queue = v4l2_m2m_request_queue,
};

static int avd_v4l2_init(struct avd_dev *avd)
{
	int ret;

	ret = v4l2_device_register(avd->dev, &avd->v4l2_dev);
	if (ret) {
		dev_err(avd->dev, "Failed to register V4L2 device\n");
		return ret;
	}

	avd->m2m_dev = v4l2_m2m_init(&avd_m2m_ops);
	if (IS_ERR(avd->m2m_dev)) {
		v4l2_err(&avd->v4l2_dev, "Failed to init mem2mem device\n");
		ret = PTR_ERR(avd->m2m_dev);
		goto err_unregister_v4l2;
	}

	avd->mdev.dev = avd->dev;
	strscpy(avd->mdev.model, "avd", sizeof(avd->mdev.model));
	strscpy(avd->mdev.bus_info, "platform:avd", sizeof(avd->mdev.bus_info));
	media_device_init(&avd->mdev);
	avd->mdev.ops = &avd_media_ops;
	avd->v4l2_dev.mdev = &avd->mdev;

	avd->vdev.lock = &avd->vdev_lock;
	avd->vdev.v4l2_dev = &avd->v4l2_dev;
	avd->vdev.fops = &avd_fops;
	avd->vdev.release = video_device_release_empty;
	avd->vdev.vfl_dir = VFL_DIR_M2M;
	avd->vdev.device_caps = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_M2M_MPLANE;
	avd->vdev.ioctl_ops = &avd_ioctl_ops;
	video_set_drvdata(&avd->vdev, avd);
	strscpy(avd->vdev.name, "avd", sizeof(avd->vdev.name));

	ret = video_register_device(&avd->vdev, VFL_TYPE_VIDEO, -1);
	if (ret) {
		v4l2_err(&avd->v4l2_dev, "Failed to register video device\n");
		goto err_cleanup_mc;
	}

	ret = v4l2_m2m_register_media_controller(
		avd->m2m_dev, &avd->vdev, MEDIA_ENT_F_PROC_VIDEO_DECODER);
	if (ret) {
		v4l2_err(&avd->v4l2_dev,
			 "Failed to initialize V4L2 M2M media controller\n");
		goto err_unregister_vdev;
	}

	ret = media_device_register(&avd->mdev);
	if (ret) {
		v4l2_err(&avd->v4l2_dev, "Failed to register media device\n");
		goto err_unregister_mc;
	}

	return 0;

err_unregister_mc:
	v4l2_m2m_unregister_media_controller(avd->m2m_dev);

err_unregister_vdev:
	video_unregister_device(&avd->vdev);

err_cleanup_mc:
	media_device_cleanup(&avd->mdev);
	v4l2_m2m_release(avd->m2m_dev);

err_unregister_v4l2:
	v4l2_device_unregister(&avd->v4l2_dev);
	return ret;
}

static void avd_v4l2_cleanup(struct avd_dev *avd)
{
	media_device_unregister(&avd->mdev);
	v4l2_m2m_unregister_media_controller(avd->m2m_dev);
	video_unregister_device(&avd->vdev);
	media_device_cleanup(&avd->mdev);
	v4l2_m2m_release(avd->m2m_dev);
	v4l2_device_unregister(&avd->v4l2_dev);
}

static const struct avd_variant avd_t8103_variant = {
	.vp_slots = {
		[AVD_CODEC_HEVC] = 2, /* no sure */
		[AVD_CODEC_H264] = 1,
		[AVD_CODEC_VP9] = 1,
	},
	.fifo_slots = 7,
	.capabilities = AVD_CAPABILITY_HEVC |
			AVD_CAPABILITY_H264 |
			AVD_CAPABILITY_VP9,
	.configure_stream = t8103_configure_stream,
	.fw_name = "apple/avd-fw-v2-t0.bin",
	.revision = 3,
	.quirks = AVD_QUIRK_LSR | AVD_QUIRK_NO_PIPE_STATE,
	.vp_slot_offset = 0x4004,
	.submit_offset = 0x4014,
	.submit_queue_max_offset = 0x4018,
	.submit_queue_status_offset = 0x402c, /* (vp slots + 1) * 4 */
};

static const struct avd_variant avd_t6000_variant = {
	.vp_slots = {
		[AVD_CODEC_HEVC] = 4,
		[AVD_CODEC_H264] = 4,
		[AVD_CODEC_VP9] = 1,
	},
	.fifo_slots = 15,
	.capabilities = AVD_CAPABILITY_HEVC |
			AVD_CAPABILITY_H264 |
			AVD_CAPABILITY_VP9,
	.configure_stream = t8112_configure_stream,
	.fw_name = "apple/avd-fw-v3-t0.bin",
	.revision = 4,
	.quirks = AVD_QUIRK_LSR | AVD_QUIRK_NO_PIPE_STATE,
	.vp_slot_offset = 0xc,
	.submit_offset = 0x30,
	.submit_queue_max_offset = 0x34,
	.submit_queue_status_offset = 0x5c,
};

static const struct avd_variant avd_t8112_variant = {
	.vp_slots = {
		[AVD_CODEC_HEVC] = 4,
		[AVD_CODEC_H264] = 4,
		[AVD_CODEC_VP9] = 1,
	},
	.fifo_slots = 15,
	.capabilities = AVD_CAPABILITY_HEVC |
			AVD_CAPABILITY_H264 |
			AVD_CAPABILITY_VP9,
	.configure_stream = t8112_configure_stream,
	.fw_name = "apple/avd-fw-v3-t1.bin",
	.revision = 4,
	.quirks = AVD_QUIRK_LSR,
	.vp_slot_offset = 0xc,
	.submit_offset = 0x30,
	.submit_queue_max_offset = 0x34,
	.submit_queue_status_offset = 0x5c,
};

static const struct avd_variant avd_t6020_variant = {
	.vp_slots = {
		[AVD_CODEC_HEVC] = 4,
		[AVD_CODEC_H264] = 4,
		[AVD_CODEC_VP9] = 1,
	},
	.fifo_slots = 15,
	.capabilities = AVD_CAPABILITY_HEVC |
			AVD_CAPABILITY_H264 |
			AVD_CAPABILITY_VP9,
	.configure_stream = t8112_configure_stream,
	.fw_name = "apple/avd-fw-v3-t1.bin",
	.revision = 4,
	.vp_slot_offset = 0xc,
	.submit_offset = 0x30,
	.submit_queue_max_offset = 0x34,
	.submit_queue_status_offset = 0x5c,
};

static const struct avd_variant avd_t8122_variant = {
	.vp_slots = {
		[AVD_CODEC_HEVC] = 4,
		[AVD_CODEC_H264] = 4,
		[AVD_CODEC_VP9] = 1,
		[AVD_CODEC_AV1] = 2,
	},
	.fifo_slots = 15,
	.configure_stream = t8122_configure_stream,
	.capabilities = AVD_CAPABILITY_HEVC |
			AVD_CAPABILITY_H264 |
			AVD_CAPABILITY_VP9 |
			AVD_CAPABILITY_AV1,
	.fw_name = "apple/avd-fw-v4-t0.bin",
	.revision = 4,
	.vp_slot_offset = 0xc,
	.submit_offset = 0x40,
	.submit_queue_max_offset = 0x44,
	.submit_queue_status_offset = 0x7c, /* v4 and newer have fixed offsets */
};

static const struct avd_variant avd_t8140_variant = {
	.vp_slots = {
		[AVD_CODEC_HEVC] = 2,
		[AVD_CODEC_H264] = 1,
		[AVD_CODEC_VP9] = 1,
		[AVD_CODEC_AV1] = 1,
	},
	/* Tier 0 has holes between the codec-specific VP blocks. */
	.vp_slot_base = {
		[AVD_CODEC_HEVC] = 0,
		[AVD_CODEC_H264] = 4,
		[AVD_CODEC_VP9] = 8,
		[AVD_CODEC_AV1] = 9,
	},
	.fifo_slots = 7,
	.configure_stream = t8122_configure_stream,
	.capabilities = AVD_CAPABILITY_HEVC |
			AVD_CAPABILITY_H264 |
			AVD_CAPABILITY_VP9 |
			AVD_CAPABILITY_AV1,
	.fw_name = "apple/avd-fw-v5-t0.bin",
	.revision = 4,
	.vp_slot_offset = 0xc,
	.submit_offset = 0x40,
	.submit_queue_max_offset = 0x44,
	.submit_queue_status_offset = 0x7c,
	.quirks = AVD_QUIRK_HEVC_V5_HEADER | AVD_QUIRK_V5_QUEUE_PACING,
};

static const struct avd_variant avd_t8132_variant = {
	.vp_slots = {
		[AVD_CODEC_HEVC] = 4,
		[AVD_CODEC_H264] = 4,
		[AVD_CODEC_VP9] = 1,
		[AVD_CODEC_AV1] = 3,
	},
	.fifo_slots = 15,
	.configure_stream = t8122_configure_stream,
	.capabilities = AVD_CAPABILITY_HEVC |
			AVD_CAPABILITY_H264 |
			AVD_CAPABILITY_VP9 |
			AVD_CAPABILITY_AV1,
	.fw_name = "apple/avd-fw-v5-t1.bin",
	.revision = 4,
	.vp_slot_offset = 0xc,
	.submit_offset = 0x40,
	.submit_queue_max_offset = 0x44,
	.submit_queue_status_offset = 0x7c,
};

/*
 * T6040 (M4 Pro): the T8132 decoder. Its first HEVC job stalled the VP when
 * it was sent the pre-v5 header (no DMA configuration words, no reserved
 * scratch slot); T6040 runs the same v5 firmware generation as T8140, so it
 * gets the v5 header. HEVC stays hidden from V4L2 clients until that is
 * verified on the hardware.
 */
static const struct avd_variant avd_t6040_variant = {
	.vp_slots = {
		[AVD_CODEC_HEVC] = 4,
		[AVD_CODEC_H264] = 4,
		[AVD_CODEC_VP9] = 1,
		[AVD_CODEC_AV1] = 3,
	},
	.fifo_slots = 15,
	.configure_stream = t8122_configure_stream,
	.capabilities = AVD_CAPABILITY_H264 |
			AVD_CAPABILITY_VP9 |
			AVD_CAPABILITY_AV1,
	.fw_name = "apple/avd-fw-v5-t1.bin",
	.revision = 4,
	.vp_slot_offset = 0xc,
	.submit_offset = 0x40,
	.submit_queue_max_offset = 0x44,
	.submit_queue_status_offset = 0x7c,
	.quirks = AVD_QUIRK_HEVC_V5_HEADER,
};

/* can also be derived from a version register */
static const struct of_device_id avd_of_match[] = {
	{
		.compatible = "apple,t8103-avd",
		.data = &avd_t8103_variant
	},
	{
		.compatible = "apple,t6000-avd",
		.data = &avd_t6000_variant
	},
	{
		.compatible = "apple,t8112-avd",
		.data = &avd_t8112_variant
	},
	{
		.compatible = "apple,t6020-avd",
		.data = &avd_t6020_variant
	},
	{
		.compatible = "apple,t8122-avd",
		.data = &avd_t8122_variant
	},
	{
		.compatible = "apple,t6041-avd",
		.data = &avd_t6040_variant
	},
	{
		.compatible = "apple,t8132-avd",
		.data = &avd_t8132_variant
	},
	{
		.compatible = "apple,t8140-avd",
		.data = &avd_t8140_variant
	},
	{},
};

MODULE_DEVICE_TABLE(of, avd_of_match);

static int avd_probe(struct platform_device *pdev)
{
	struct avd_dev *avd;
	const struct of_device_id *match;
	const char *codec_info = NULL;
	int ret, irq;

	avd = devm_kzalloc(&pdev->dev, sizeof(*avd), GFP_KERNEL);
	if (!avd)
		return -ENOMEM;

	platform_set_drvdata(pdev, avd);
	avd->dev = &pdev->dev;
	avd->pdev = pdev;

	mutex_init(&avd->vdev_lock);
	spin_lock_init(&avd->job_lock);

	match = of_match_node(avd_of_match, pdev->dev.of_node);
	avd->variant = match->data;

	avd->rstc = devm_reset_control_get_exclusive(avd->dev, NULL);

	avd->code = devm_platform_ioremap_resource_byname(pdev, "code");
	if (IS_ERR(avd->code))
		return PTR_ERR(avd->code);
	avd->mbox = devm_platform_ioremap_resource_byname(pdev, "mbox");
	if (IS_ERR(avd->mbox))
		return PTR_ERR(avd->mbox);
	avd->ctrl = devm_platform_ioremap_resource_byname(pdev, "ctrl");
	if (IS_ERR(avd->ctrl))
		return PTR_ERR(avd->ctrl);

	avd->domain = iommu_get_domain_for_dev(avd->dev);
	if (avd->domain) {
		avd->empty_domain = iommu_paging_domain_alloc(avd->dev);
		if (IS_ERR(avd->empty_domain)) {
			avd->empty_domain = NULL;
			dev_warn(avd->dev, "cannot alloc new empty domain");
		}
	}

	ret = request_firmware(&avd->fw, avd->variant->fw_name, avd->dev);

	if (ret) {
		dev_err(avd->dev, "failed to load firmware: %d", ret);
		return ret;
	}

	/* Determine firmware and HW variant capabilities */
	if (avd->fw->size >= AVD_CODEC_INFO_MAX_SIZE)
		codec_info = avd->fw->data + avd->fw->size - AVD_CODEC_INFO_MAX_SIZE;
	if (codec_info && strncmp(codec_info, "CODECS:", 7) == 0) {
		unsigned int caps = 0;
		unsigned int variant_caps = avd->variant->capabilities;

		if (strnstr(codec_info, "av1", AVD_CODEC_INFO_MAX_SIZE))
			caps |= variant_caps & AVD_CAPABILITY_AV1;
		if (strnstr(codec_info, "h264", AVD_CODEC_INFO_MAX_SIZE))
			caps |= variant_caps & AVD_CAPABILITY_H264;
		if (strnstr(codec_info, "h265", AVD_CODEC_INFO_MAX_SIZE))
			caps |= variant_caps & AVD_CAPABILITY_HEVC;
		if (strnstr(codec_info, "vp9", AVD_CODEC_INFO_MAX_SIZE))
			caps |= variant_caps & AVD_CAPABILITY_VP9;

		dev_info(avd->dev, "Firmware codecs: %.27s\n", codec_info + 7);

		avd->capabilities = caps;
	} else {
		/* Backward compatibility to old firmware, do not upstream. */
		avd->capabilities = avd->variant->capabilities;
	}

	ret = dma_set_mask_and_coherent(avd->dev,
			DMA_BIT_MASK((avd->variant->quirks & AVD_QUIRK_LSR) ? 38 : 64));

	if (ret) {
		dev_err(avd->dev, "Failed to set DMA mask");
		return ret;
	}

	irq = platform_get_irq(pdev, 1);
	if (irq < 0)
		return irq;
	ret = devm_request_threaded_irq(&pdev->dev, irq, NULL,
			avd_irq_handler, IRQF_ONESHOT,
			dev_name(&pdev->dev), avd);
	if (ret) {
		dev_err(avd->dev, "Could not request IRQ 1");
		return ret;
	}

	pm_runtime_set_autosuspend_delay(avd->dev, 100);
	pm_runtime_use_autosuspend(avd->dev);
	pm_runtime_enable(avd->dev);

	ret = avd_v4l2_init(avd);
	if (ret)
		goto err_disable_runtime_pm;

	return 0;

err_disable_runtime_pm:
	pm_runtime_dont_use_autosuspend(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	return ret;
}

static void avd_remove(struct platform_device *pdev)
{
	struct avd_dev *avd = platform_get_drvdata(pdev);

	release_firmware(avd->fw);

	avd_v4l2_cleanup(avd);

	if (avd->empty_domain)
		iommu_domain_free(avd->empty_domain);

	pm_runtime_disable(avd->dev);
	pm_runtime_dont_use_autosuspend(avd->dev);
}

static __maybe_unused int avd_runtime_resume(struct device *dev)
{
	int ret;
	struct avd_dev *avd = platform_get_drvdata(to_platform_device(dev));

	ret = avd_boot(avd);
	if (ret)
		dev_err(dev, "failed to boot");
	return ret;
}

static __maybe_unused int avd_runtime_suspend(struct device *dev)
{
	struct avd_dev *avd = platform_get_drvdata(to_platform_device(dev));

	avd_shutdown(avd);
	return 0;
}

static const struct dev_pm_ops avd_pm_ops = { SET_SYSTEM_SLEEP_PM_OPS(
	pm_runtime_force_suspend,
	pm_runtime_force_resume) SET_RUNTIME_PM_OPS(avd_runtime_suspend,
						    avd_runtime_resume, NULL) };

static struct platform_driver avd_driver = {
	.probe = avd_probe,
	.remove = avd_remove,
	.driver = {
		.name = "avd",
		.of_match_table = avd_of_match,
		.pm = &avd_pm_ops,
	},
};
module_platform_driver(avd_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Apple avd v4l2 sl m2m");
MODULE_FIRMWARE("apple/avd-fw-v2-t0.bin");
MODULE_FIRMWARE("apple/avd-fw-v3-t0.bin");
MODULE_FIRMWARE("apple/avd-fw-v3-t1.bin");
MODULE_FIRMWARE("apple/avd-fw-v4-t0.bin");
MODULE_FIRMWARE("apple/avd-fw-v5-t0.bin");
MODULE_FIRMWARE("apple/avd-fw-v5-t1.bin");
