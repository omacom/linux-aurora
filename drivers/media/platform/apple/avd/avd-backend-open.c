// SPDX-License-Identifier: GPL-2.0-only
/* Legacy transport split from avd-drv.c; Copyright The Asahi Linux Contributors. */
#include <linux/delay.h>
#include <linux/iopoll.h>
#include <linux/jiffies.h>
#include <linux/overflow.h>
#include <linux/slab.h>
#include <linux/string.h>

#include "avd.h"
#include "avd-inst.h"
#include "avd-regs.h"

/*
 * Compatibility transport for firmware without the queued-program ABI. The
 * codec's CPU builder emits a per-slot DMA-visible instruction stream. Only
 * this backend touches the VP FIFO. Its untagged PP interrupt limits it to one
 * frame; the queued avd-fw transport can replace these ops without changing
 * buffer ownership in the core.
 */
int avd_open_build_program(struct avd_ctx *ctx)
{
	struct avd_dev *avd = ctx->dev;
	struct avd_frame *f = avd->building;
	struct avd_job *job = &ctx->job;
	size_t count = 1, pos = 0, i, j;
	u32 *words, mask, start;
	int ret;

	if (!f || !job->segments)
		return -EINVAL;
	for (i = 0; i <= job->num; i++) {
		if (job->segments[i].num > AVD_MAX_INST ||
		    check_add_overflow(count, job->segments[i].num + !!i, &count))
			return -EOVERFLOW;
	}
	if (count > SIZE_MAX / sizeof(u32))
		return -EOVERFLOW;
	if (f->prog->size < count * sizeof(u32)) {
		ret = avd_buf_alloc(avd, f->prog, count * sizeof(u32));
		if (ret)
			return ret;
	}
	mask = job->codec == AVD_CODEC_VP9 && avd->variant->revision == 3 ?
		AVD_OP_EXEC_REV3_VP9_MASK : 0;
	start = AVD_OP_EXEC_FLAG_START_REV4(avd->variant->revision == 4) |
		AVD_OP_EXEC_FLAG_START_REV3(avd->variant->revision == 3);
	/* The queued transport gives every frame in flight its own FIFO. */
	ctx->fifo_idx = avd_backend_is_qfw(avd) ? f->idx : 0;
	f->fifo_iova = ctx->job_fifo ?: ctx->inst.addr;
	ctx->job_fifo = 0;
	words = f->prog->cpu;
	words[pos++] = AVD_OP_EXEC | mask | start | AVD_OP_EXEC_FIFO_IDX(ctx->fifo_idx);
	for (i = 0; i <= job->num; i++) {
		for (j = 0; j < job->segments[i].num; j++)
			words[pos++] = job->segments[i].instructions[j];
		if (i)
			words[pos++] = AVD_OP_EXEC | mask | AVD_OP_EXEC_FLAG_END(i == job->num);
	}
	f->prog_words = pos;
	f->cmd[0] = job->codec;
	kvfree(job->segments);
	job->segments = NULL;
	return 0;
}

static unsigned int avd_open_max_inflight(struct avd_dev *avd)
{
	return 1;
}

static int avd_open_submit(struct avd_dev *avd, struct avd_frame *f)
{
	struct avd_ctx *ctx = f->ctx;
	u32 codec = f->cmd[0], vp = 0;
	u32 *words = f->prog->cpu;
	unsigned int i;
	void __iomem *reg;

	if (avd->compat_active)
		return -EBUSY;
	/* Native frame_ops require firmware-side PP chaining and tagged replies. */
	if (ctx->coded_fmt_desc->frame_ops)
		return -EOPNOTSUPP;
	if (codec > AVD_CODEC_AV1 || !words || !f->prog_words ||
	    f->prog_words > f->prog->size / sizeof(u32))
		return -EINVAL;
	if (!(avd->capabilities & BIT(codec)))
		return -EOPNOTSUPP;
	for (i = 0; i < codec; i++)
		vp += avd->variant->vp_slots[i];
	if (avd->variant->vp_slot_base[codec])
		vp = avd->variant->vp_slot_base[codec];
	avd->variant->configure_stream(avd, ctx->inst.addr, ctx->fifo_idx, vp);
	reg = avd->ctrl + avd->variant->vp_slot_offset + vp * 4;
	avd->compat_active = f;
	avd->compat_ctx = ctx;
	for (i = 0; i < f->prog_words; i++) {
		/* Preserve the old firmware's segment FIFO pacing in this shim. */
		if (i && (words[i] & GENMASK(31, 24)) == AVD_OP_EXEC &&
		    (!(avd->variant->quirks & AVD_QUIRK_V5_QUEUE_PACING) ||
		     codec == AVD_CODEC_HEVC)) {
			u32 max = readl(avd->ctrl + avd->variant->submit_queue_max_offset + vp * 4);
			u32 cur = readl(avd->ctrl +
					avd->variant->submit_queue_status_offset + vp * 4);

			if (cur >= max) {
				/* Already emitted words: only reset may release this slot. */
				f->deadline = jiffies;
				return 0;
			}
			if (cur >= max / 2) {
				/*
				 * Wait for the VP to drain below half full, but
				 * only as long as it takes. The old fixed sleep
				 * (100-150 us on v5 pacing parts, 500-650 us
				 * elsewhere) stays the upper bound.
				 */
				u32 wait_us = avd->variant->quirks &
					      AVD_QUIRK_V5_QUEUE_PACING ? 150 : 650;

				readl_relaxed_poll_timeout(avd->ctrl +
					avd->variant->submit_queue_status_offset + vp * 4,
					cur, cur < max / 2, 10, wait_us);
			}
		}
		writel(words[i], reg);
	}
	return 0;
}

static irqreturn_t avd_open_irq(struct avd_dev *avd)
{
	struct avd_frame *f = avd->compat_active;
	u32 status;

	if (!avd_mbox_pop(avd, &status))
		return IRQ_HANDLED;
	if (!f) {
		/*
		 * The old driver kicked PP for every VP DONE. A VP DONE with no
		 * frame in flight (its frame already completed on a surplus PP
		 * DONE of a multi-kick picture) still needs that kick, or the
		 * next picture's PP is never started and it times out.
		 */
		if ((status & 0x11000) == 0 && (status & 0x100) && avd->compat_ctx &&
		    avd->compat_ctx->coded_fmt_desc->ops->submit)
			avd->compat_ctx->coded_fmt_desc->ops->submit(avd->compat_ctx);
		return IRQ_HANDLED;
	}
	if (status & 0x1000) {
		avd->compat_active = NULL;
		avd_frame_complete(avd, f->tag, 0);
	} else if (status & 0x100) {
		/* Old firmware only: no request tag and no firmware-side PP kick. */
		if (f->ctx->coded_fmt_desc->ops->submit)
			f->ctx->coded_fmt_desc->ops->submit(f->ctx);
	} else {
		dev_err_ratelimited(avd->dev, "open firmware error %#x\n", status);
		f->deadline = jiffies;
		mod_delayed_work(system_wq, &avd->queue_work, 0);
	}
	return IRQ_HANDLED;
}

const struct avd_backend_ops avd_open_backend = {
	.name = "open-compat",
	.boot = avd_boot,
	.shutdown = avd_shutdown,
	.max_inflight = avd_open_max_inflight,
	.submit = avd_open_submit,
	.irq = avd_open_irq,
	.suspend = avd_pm_backend_suspend,
	.resume = avd_pm_backend_resume,
};

static void avd_open_capabilities(struct avd_dev *avd)
{
	static const struct {
		const char *name;
		u32 bit;
	} codecs[] = {
		{ "h265", AVD_CAPABILITY_HEVC },
		{ "h264", AVD_CAPABILITY_H264 },
		{ "vp9", AVD_CAPABILITY_VP9 },
		{ "av1", AVD_CAPABILITY_AV1 },
	};
	const char *info;
	unsigned int i;

	/* Without a CODECS trailer, keep codecs that need a newer firmware hidden. */
	avd->capabilities = avd->variant->capabilities &
			    ~avd->variant->fw_caps_required;
	avd->fw_capabilities = avd->capabilities;
	if (!avd->fw || avd->fw->size < 0x40)
		return;
	info = (const char *)avd->fw->data + avd->fw->size - 0x40;
	if (memcmp(info, "CODECS:", 7))
		return;	/* fw_capabilities already set above */
	avd->capabilities = 0;
	for (i = 0; i < ARRAY_SIZE(codecs); i++)
		if (strnstr(info, codecs[i].name, 0x40))
			avd->capabilities |= avd->variant->capabilities & codecs[i].bit;
	avd->fw_capabilities = avd->capabilities;
}

static int avd_open_request_firmware(struct avd_dev *avd)
{
	int ret;

	if (!avd->variant->fw_name_fallback)
		return request_firmware(&avd->fw, avd->variant->fw_name, avd->dev);
	ret = firmware_request_nowarn(&avd->fw, avd->variant->fw_name, avd->dev);
	if (ret != -ENOENT)
		return ret;
	dev_info(avd->dev, "%s not installed, using %s\n",
		 avd->variant->fw_name, avd->variant->fw_name_fallback);
	return request_firmware(&avd->fw, avd->variant->fw_name_fallback, avd->dev);
}

int avd_backend_select(struct avd_dev *avd)
{
	int ret;

	/* Apple probe returns -ENOENT only when no usable image is present. */
	if (avd->variant->apple) {
		ret = avd_apple_backend.probe(avd);
		if (!ret) {
			avd->backend = &avd_apple_backend;
			avd->capabilities = avd->variant->capabilities;
			return 0;
		}
		if (ret != -ENOENT && ret != -EOPNOTSUPP)
			return ret;
	}
	avd->backend = &avd_open_backend;
	ret = avd_open_request_firmware(avd);
	if (!ret)
		avd_open_capabilities(avd);
	if (!ret)
		ret = avd_qfw_probe(avd);
	return ret;
}

int avd_backend_boot(struct avd_dev *avd)
{
	int ret;

	if (!avd_backend_is_apple(avd)) {
		avd_pm_prepare_piodma(avd);
		avd_qfw_prepare_boot(avd);
	}
	ret = avd->backend->boot(avd);
	/* Never switch command ABIs after frames have used this backend. */
	if (ret == -ENOENT && !avd->fw_generation && avd_backend_is_apple(avd)) {
		ret = avd_open_request_firmware(avd);
		if (!ret)
			ret = avd_qfw_probe(avd);
		if (ret)
			return ret;
		avd->backend = &avd_open_backend;
		avd_open_capabilities(avd);
		avd_pm_prepare_piodma(avd);
		avd_qfw_prepare_boot(avd);
		ret = avd->backend->boot(avd);
	}
	if (!ret)
		avd_qfw_select(avd);
	if (!ret)
		avd->fw_generation++;
	return ret;
}
