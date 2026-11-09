// SPDX-License-Identifier: GPL-2.0-only OR MIT
#include <linux/firmware.h>
#include <linux/iopoll.h>
#include <linux/slab.h>
#include <linux/unaligned.h>

#include "avd.h"
#include "avd-apple-protocol.h"
#include "avd-queue-internal.h"

/* SPEC 3.2. Unknown field meanings are intentionally not named. */
const u8 avd_borage_config[64] = {
	[0x0a] = 0x55, [0x0b] = 0x01, /* +0x08: 0x01558000 */
	[0x09] = 0x80,
	[0x11] = 0x8a, [0x12] = 0x55, [0x13] = 0x01,
	[0x19] = 0x8e, [0x1a] = 0x55, [0x1b] = 0x01,
	[0x28] = 0x0a, [0x2a] = 0x04,
	[0x2c] = 0x04,
	[0x38] = 0x08, [0x3a] = 0x10, [0x3b] = 0x01,
};

/*
 * Source: clean SPEC 3-5, AppleAVD 993.1 / T6040 Borage. The SPEC does
 * not supply the ITCM/version register offsets, link register operations or
 * reply field layout. Do not substitute V3/open-firmware offsets for them.
 * avd-protocol.rst records every unresolved field and its source/capture.
 */
const struct avd_apple_hw avd_borage_hw = {
	.fw_name = "apple/avd-apple-t6040.bin",
	.verified = false,
};

struct avd_apple {
	void __iomem *regs;
	const struct avd_apple_hw *hw;
	const struct firmware *image;
	u32 shadow[AVD_APPLE_SHADOW_SIZE];
	unsigned int shadow_read;
	unsigned int shadow_write;
	unsigned int producer;
	u32 version;
};

static void avd_apple_release_image(void *data)
{
	struct avd_apple *apple = data;

	release_firmware(apple->image);
}

static int avd_apple_image_present(struct avd_apple *apple)
{
	const struct avd_apple_hw *hw = apple->hw;
	u32 start = readl(apple->regs + hw->itcm_start);
	u32 end = readl(apple->regs + hw->itcm_end);
	u32 version;

	if (end <= start || end - start < 4 || end - start > hw->imem_size ||
	    hw->version_offset > end - start - 4)
		return -ENOENT;
	version = readl(apple->regs + hw->imem + hw->version_offset);
	if (!version || version == U32_MAX)
		return -ENOENT;
	apple->version = version;
	return 0;
}

static int avd_apple_probe(struct avd_dev *avd)
{
	const struct avd_apple_hw *hw = avd->variant->apple;
	struct avd_apple *apple;
	int ret;

	if (!hw || !hw->verified || !hw->reply.verified) {
		dev_info(avd->dev, "Apple transport awaits Borage register/reply facts; using open firmware\n");
		return -EOPNOTSUPP;
	}
	apple = devm_kzalloc(avd->dev, sizeof(*apple), GFP_KERNEL);
	if (!apple)
		return -ENOMEM;
	apple->hw = hw;
	apple->regs = devm_platform_ioremap_resource_byname(avd->pdev, "apple");
	if (IS_ERR(apple->regs))
		return PTR_ERR(apple->regs);
	/* Firmware bytes remain in vendorfw; this driver never embeds an image. */
	ret = firmware_request_nowarn(&apple->image, hw->fw_name, avd->dev);
	if (ret && ret != -ENOENT)
		return ret;
	ret = devm_add_action_or_reset(avd->dev, avd_apple_release_image, apple);
	if (ret)
		return ret;
	avd->backend_priv = apple;
	return 0;
}

static int avd_apple_load(struct avd_dev *avd)
{
	struct avd_apple *apple = avd->backend_priv;
	const struct avd_apple_hw *hw = apple->hw;
	const struct firmware *fw = apple->image;
	size_t i, rounded;

	if (!fw)
		return -ENOENT;
	if (!fw->size || fw->size > hw->imem_size ||
	    hw->imem_size < 4 || hw->version_offset > hw->imem_size - 4)
		return -EINVAL;
	rounded = ALIGN(fw->size, 4);
	if (rounded > hw->imem_size)
		return -EFBIG;
	writel(hw->itcm_clear_value, apple->regs + hw->itcm_clear);
	writel(0, apple->regs + hw->itcm_start);
	writel(rounded, apple->regs + hw->itcm_end);
	/* Bounded word copies pad the last word without reading past fw->data. */
	for (i = 0; i < rounded; i += 4) {
		u8 bytes[4] = {};

		memcpy(bytes, fw->data + i, min_t(size_t, 4, fw->size - i));
		writel(get_unaligned_le32(bytes), apple->regs + hw->imem + i);
	}
	writel(hw->itcm_enable_value, apple->regs + hw->itcm_enable);
	return avd_apple_image_present(apple);
}

static int avd_apple_boot(struct avd_dev *avd)
{
	struct avd_apple *apple = avd->backend_priv;
	const struct avd_apple_hw *hw = apple->hw;
	u32 value;
	int ret;

	ret = avd_apple_image_present(apple);
	if (ret) {
		ret = avd_apple_load(avd);
		if (ret)
			return ret;
	} else {
		/* Suspend stopped the MCPU without clearing the retained image. */
		writel(hw->itcm_enable_value, apple->regs + hw->itcm_enable);
	}
	dev_info(avd->dev, "Apple MCPU firmware %u.%u.%u (%08x)\n",
		 apple->version >> 8, (apple->version >> 4) & 0xf,
		 apple->version & 0xf, apple->version);
	/* SPEC 3.2: decode connection, aperture 1, state 4, then LinkUp. */
	writel(hw->link_setup_value, apple->regs + hw->link_setup);
	ret = readl_poll_timeout(apple->regs + hw->link_state, value,
				 value == 4, 10, 10000);
	if (ret)
		return ret;
	writel(hw->link_up_value, apple->regs + hw->link_up);
	ret = readl_poll_timeout(apple->regs + hw->ready, value,
				 value == 1, 10, 10000);
	if (!ret) {
		apple->producer = 0;
		apple->shadow_read = 0;
		apple->shadow_write = 0;
	}
	return ret;
}

static void avd_apple_shutdown(struct avd_dev *avd)
{
	struct avd_apple *apple = avd->backend_priv;

	/* MCPU disable value must be proved together with enable in the ledger. */
	writel(0, apple->regs + apple->hw->itcm_enable);
}

static unsigned int avd_apple_max_inflight(struct avd_dev *avd)
{
	/*
	 * Every frame in VP->PP flight owns one hardware instruction FIFO
	 * (EXEC bits 7:4), so the FIFO count bounds the frames in flight:
	 * 15 on v3+ tier 1 parts, 7 on T8103 and T8140.
	 */
	return min_t(unsigned int, AVD_MAX_FRAMES, avd->variant->fifo_slots ?: 1);
}

static int avd_apple_send(struct avd_dev *avd, const u32 cmd[24])
{
	struct avd_apple *apple = avd->backend_priv;
	const struct avd_apple_hw *hw = apple->hw;
	unsigned int consumer = readl(apple->regs + hw->inbox_consumer) & 7;
	u32 address = AVD_APPLE_INBOX_BASE + apple->producer * AVD_APPLE_COMMAND_SIZE;
	unsigned int i;

	if (avd_inbox_full(apple->producer, consumer))
		return -EBUSY;
	for (i = 0; i < 24; i++)
		writel(cmd[i], apple->regs + address - hw->aperture_origin + i * 4);
	/* MMIO commands and coherent fp payload precede the four-byte doorbell. */
	wmb();
	writel(avd_apple_notification(apple->producer), apple->regs + hw->notify);
	apple->producer = (apple->producer + 1) & 7;
	return 0;
}

static int avd_apple_session_start(struct avd_ctx *ctx)
{
	u32 cmd[24] = {};

	if (avd_apple_session_command(cmd, ctx->client_id, 0, 0, 0))
		return -EINVAL;
	return avd_apple_send(ctx->dev, cmd);
}

static void avd_apple_session_stop(struct avd_ctx *ctx)
{
	u32 cmd[24] = {};

	if (avd_apple_session_command(cmd, ctx->client_id, 2, 0, 0))
		return;
	if (avd_apple_send(ctx->dev, cmd))
		dev_warn(ctx->dev->dev, "session stop inbox full\n");
}

static void avd_apple_abort(struct avd_ctx *ctx)
{
	u32 cmd[24] = {};

	if (avd_apple_session_command(cmd, ctx->client_id, 5, 0, 0))
		return;
	/* Core follows abort with reset before returning any DMA buffer. */
	avd_apple_send(ctx->dev, cmd);
}

static int avd_apple_set_property(struct avd_ctx *ctx, u32 property, u32 value)
{
	u32 cmd[24] = {};

	if (avd_apple_session_command(cmd, ctx->client_id, 6, property, value))
		return -EINVAL;
	return avd_apple_send(ctx->dev, cmd);
}

static int avd_apple_submit(struct avd_dev *avd, struct avd_frame *f)
{
	const struct avd_codec_frame_ops *ops = f->ctx->coded_fmt_desc->frame_ops;
	u32 selector;
	int ret;

	if (!ops || !ops->pack_cmd || !f->fp->cpu || f->ctx->client_id >= 1024)
		return -EOPNOTSUPP;
	/* SPEC 5.2 prefix precedes the codec-specific builder. */
	selector = get_unaligned_le32(f->fp->cpu + 4) & 0xf;
	ret = avd_apple_decode_prefix(f, f->ctx->client_id, selector, f->seq & 3);
	if (ret)
		return ret;
	ret = ops->pack_cmd(f->ctx, f);
	if (ret)
		return ret;
	return avd_apple_send(avd, f->cmd);
}

static irqreturn_t avd_apple_irq(struct avd_dev *avd)
{
	struct avd_apple *apple = avd->backend_priv;
	const struct avd_apple_hw *hw = apple->hw;
	unsigned int budget = AVD_APPLE_SHADOW_SIZE;

	while (budget-- && (readl(apple->regs + hw->receive_status) & hw->receive_pending)) {
		unsigned int next = (apple->shadow_write + 1) & 63;

		if (next == apple->shadow_read)
			break;
		apple->shadow[apple->shadow_write] = readl(apple->regs + hw->receive);
		apple->shadow_write = next;
		writel(hw->receive_ack_value, apple->regs + hw->receive_ack);
	}
	while (apple->shadow_read != apple->shadow_write) {
		u32 record[AVD_APPLE_RESULT_WORDS];
		struct avd_apple_reply reply;
		unsigned int i;
		int ret;

		/* Result notification indexing must be verified with the layout. */
		u32 index = apple->shadow[apple->shadow_read];

		apple->shadow_read = (apple->shadow_read + 1) & 63;
		if (index >= AVD_APPLE_SHADOW_SIZE)
			continue;
		for (i = 0; i < AVD_APPLE_RESULT_WORDS; i++)
			record[i] = readl(apple->regs + hw->result_base +
					  index * sizeof(record) + i * 4);
		ret = avd_apple_parse_reply(record, ARRAY_SIZE(record), &hw->reply, &reply);
		if (ret)
			continue;
		for (i = 0; i < AVD_MAX_FRAMES; i++) {
			struct avd_frame *f = &avd->frames[i];

			if (f->state == AVD_FRAME_SUBMITTED && f->frame_num == reply.frame &&
			    f->ctx->client_id == reply.client) {
				dma_rmb();
				avd_frame_complete(avd, f->tag, reply.status);
				break;
			}
		}
	}
	mod_delayed_work(system_dfl_wq, &avd->queue_work, 0);
	return IRQ_HANDLED;
}

const struct avd_backend_ops avd_apple_backend = {
	.name = "apple-mcpu",
	.probe = avd_apple_probe,
	.boot = avd_apple_boot,
	.shutdown = avd_apple_shutdown,
	.max_inflight = avd_apple_max_inflight,
	.session_start = avd_apple_session_start,
	.session_stop = avd_apple_session_stop,
	.submit = avd_apple_submit,
	.abort = avd_apple_abort,
	.set_property = avd_apple_set_property,
	.irq = avd_apple_irq,
	.suspend = avd_pm_backend_suspend,
	.resume = avd_pm_backend_resume,
};
