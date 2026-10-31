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
#include "avd-apple-protocol.h"

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
	return avd_open_build_program(ctx);
}

static irqreturn_t avd_irq_handler(int irq, void *data)
{
	struct avd_dev *avd = data;
	/* An interrupt raised before a reset or power-off belongs to that instance. */
	unsigned int gen = READ_ONCE(avd->pm.irq_gen);
	irqreturn_t ret;

	mutex_lock(&avd->queue_mutex);
	ret = avd_pm_irq(avd, gen);
	mutex_unlock(&avd->queue_mutex);
	return ret;
}

/*
 * Capture buffers are found by timestamp and handed to the hardware by DMA
 * address, so nothing limits them to VB2_MAX_FRAME. A larger pool lets
 * userspace decode ahead to ride out bitrate spikes the hardware can't
 * decode in real time (e.g. 150+ Mbit/s 4K60 H.264).
 */
static int avd_m2m_queue_init(void *priv, struct vb2_queue *src_vq,
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
	dst_vq->max_num_buffers = max(128U, VB2_MAX_FRAME);

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

	INIT_LIST_HEAD(&ctx->queue_node);
	ctx->next_dep = AVD_DEP_PREV_DONE;

	avd_reset_coded_fmt(ctx);
	avd_reset_decoded_fmt(ctx);

	v4l2_fh_init(&ctx->fh, video_devdata(filp));

	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(avd->m2m_dev, ctx, avd_m2m_queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		goto err_free_ctx;
	}

	ret = avd_init_ctrls(ctx);
	if (ret)
		goto err_cleanup_m2m_ctx;

	avd_queue_add_ctx(ctx);
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
	avd_queue_del_ctx(ctx);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	cancel_work_sync(&ctx->submit_work);
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
	.device_run = avd_queue_run,
	.job_ready = avd_queue_ready,
	.job_abort = avd_queue_abort,
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
 * gets the v5 header. HEVC is advertised only when the loaded firmware
 * names h265 in its CODECS trailer.
 */
static const struct avd_variant avd_t6040_variant = {
	.apple = &avd_borage_hw,
	.fp_size = 0x1130d0,
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
	/* HEVC only with firmware whose CODECS trailer lists h265. */
	.fw_caps_required = AVD_CAPABILITY_HEVC,
	/* v5-t2 matches T6040's SRAM aperture, which the queued transport needs. */
	.fw_name = "apple/avd-fw-v5-t2.bin",
	.fw_name_fallback = "apple/avd-fw-v5-t1.bin",
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

static void avd_release_open_firmware(void *data)
{
	struct avd_dev *avd = data;

	release_firmware(avd->fw);
}

static int avd_probe(struct platform_device *pdev)
{
	struct avd_dev *avd;
	const struct of_device_id *match;
	int ret, irq;

	avd = devm_kzalloc(&pdev->dev, sizeof(*avd), GFP_KERNEL);
	if (!avd)
		return -ENOMEM;

	platform_set_drvdata(pdev, avd);
	avd->dev = &pdev->dev;
	avd->pdev = pdev;

	mutex_init(&avd->vdev_lock);
	avd_pm_init(avd, NULL, &avd_pm_queue_ops);
	ret = avd_queue_init(avd);
	if (ret)
		return ret;

	match = of_match_node(avd_of_match, pdev->dev.of_node);
	avd->variant = match->data;

	avd->rstc = devm_reset_control_get_exclusive(avd->dev, NULL);
	if (IS_ERR(avd->rstc))
		return PTR_ERR(avd->rstc);

	avd->code = devm_platform_ioremap_resource_byname(pdev, "code");
	if (IS_ERR(avd->code))
		return PTR_ERR(avd->code);
	avd->mbox = devm_platform_ioremap_resource_byname(pdev, "mbox");
	if (IS_ERR(avd->mbox))
		return PTR_ERR(avd->mbox);
	avd->ctrl = devm_platform_ioremap_resource_byname(pdev, "ctrl");
	if (IS_ERR(avd->ctrl))
		return PTR_ERR(avd->ctrl);
	ret = avd_pm_map_piodma(avd);
	if (ret)
		return ret;

	avd->domain = iommu_get_domain_for_dev(avd->dev);
	if (avd->domain) {
		avd->empty_domain = iommu_paging_domain_alloc(avd->dev);
		if (IS_ERR(avd->empty_domain)) {
			avd->empty_domain = NULL;
			dev_warn(avd->dev, "cannot alloc new empty domain");
		}
	}

	ret = avd_backend_select(avd);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(avd->dev, avd_release_open_firmware, avd);
	if (ret)
		return ret;

	ret = dma_set_mask_and_coherent(avd->dev,
			DMA_BIT_MASK(avd_backend_is_apple(avd) ? 42 :
			     (avd->variant->quirks & AVD_QUIRK_LSR) ? 38 : 64));

	if (ret) {
		dev_err(avd->dev, "Failed to set DMA mask");
		return ret;
	}

	irq = platform_get_irq(pdev, 1);
	if (irq < 0)
		return irq;
	avd->pm.irq = irq;
	ret = devm_request_threaded_irq(&pdev->dev, irq, NULL,
			avd_irq_handler, IRQF_ONESHOT | IRQF_NO_AUTOEN,
			dev_name(&pdev->dev), avd);
	if (ret) {
		dev_err(avd->dev, "Could not request IRQ 1");
		return ret;
	}

	avd_pm_enable(avd);

	/*
	 * Boot the firmware once now: the transport and the codec list depend
	 * on what the loaded image reports, and clients enumerate formats and
	 * size their contexts before the first decode would power it up.
	 */
	ret = pm_runtime_resume_and_get(avd->dev);
	if (ret < 0)
		dev_warn(avd->dev, "firmware probe boot failed: %d\n", ret);
	else
		pm_runtime_put_autosuspend(avd->dev);

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
	int ret;

	v4l2_m2m_suspend(avd->m2m_dev);
	ret = avd->pm.queue->drain(avd);
	if (ret)
		dev_warn(avd->dev, "queue drain failed, resetting before teardown: %d\n", ret);
	avd_queue_cleanup(avd);
	avd_pm_disable(avd);
	avd_v4l2_cleanup(avd);
	if (avd->empty_domain)
		iommu_domain_free(avd->empty_domain);
}

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
