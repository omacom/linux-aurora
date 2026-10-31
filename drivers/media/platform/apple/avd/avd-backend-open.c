// SPDX-License-Identifier: GPL-2.0-only OR MIT
#include <linux/delay.h>
#include <linux/overflow.h>
#include <linux/slab.h>

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
	ctx->fifo_idx = 0;
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
	for (i = 0; i < codec; i++)
		vp += avd->variant->vp_slots[i];
	if (avd->variant->vp_slot_base[codec])
		vp = avd->variant->vp_slot_base[codec];
	avd->variant->configure_stream(avd, ctx->inst.addr, ctx->fifo_idx, vp);
	reg = avd->ctrl + avd->variant->vp_slot_offset + vp * 4;
	avd->compat_active = f;
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
				if (avd->variant->quirks & AVD_QUIRK_V5_QUEUE_PACING)
					usleep_range(100, 150);
				else
					usleep_range(500, 650);
			}
		}
		writel(words[i], reg);
	}
	return 0;
}

static irqreturn_t avd_open_irq(struct avd_dev *avd)
{
	struct avd_frame *f = avd->compat_active;
	u32 status = readl(avd->mbox + AVD_REG_MBOX1_RETRIEVE);

	writel(AVD_MBOX1_NOT_EMPTY, avd->mbox + AVD_REG_MBOX_IRQ_CLR);
	if (!f)
		return IRQ_HANDLED;
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
};

int avd_backend_select(struct avd_dev *avd)
{
	int ret;

	/* Apple probe returns -ENOENT only when no usable image is present. */
	if (avd->variant->apple) {
		ret = avd_apple_backend.probe(avd);
		if (!ret) {
			avd->backend = &avd_apple_backend;
			return 0;
		}
		if (ret != -ENOENT && ret != -EOPNOTSUPP)
			return ret;
	}
	avd->backend = &avd_open_backend;
	return request_firmware(&avd->fw, avd->variant->fw_name, avd->dev);
}

int avd_backend_boot(struct avd_dev *avd)
{
	int ret = avd->backend->boot(avd);

	if (ret == -ENOENT && avd_backend_is_apple(avd)) {
		ret = request_firmware(&avd->fw, avd->variant->fw_name, avd->dev);
		if (ret)
			return ret;
		avd->backend = &avd_open_backend;
		ret = avd->backend->boot(avd);
	}
	if (!ret)
		avd->fw_generation++;
	return ret;
}
