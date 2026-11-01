// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Queued transport for the open AVD firmware (avd-fw queue ABI 2).
 *
 * The codec's CPU builder still produces the per-frame instruction program.
 * Instead of writing it word by word into the VP FIFO register and kicking
 * the PP from the VP-done interrupt, the host streams the program into a word
 * ring in CM3 SRAM and posts one descriptor per frame. The firmware feeds the
 * VP FIFO from SRAM, starts PP itself when the VP is done, and posts a tagged
 * completion. No PIODMA (and so no second DART stream) is involved.
 */
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "avd.h"
#include "avd-inst.h"
#include "avd-regs.h"

#define AVDQ_MAGIC		0x41565131	/* "AVQ1" */
#define AVDQ_VERSION		2
#define AVDQ_COUNT		16
#define AVDQ_DESC_BYTES		64
#define AVDQ_RING_OFFSET	0x8000
#define AVDQ_INFO		0x7fc0
/* Firmware private state starts here; the SRAM ring must end before it. */
#define AVDQ_RING_LIMIT		0x6000
#define AVDQ_INFO_RP		(AVDQ_INFO + 0x38)
#define AVDQ_INFO_WP		(AVDQ_INFO + 0x3c)
#define AVDQ_FEAT_SRAM_RING	BIT(0)
#define AVDQ_FEAT_MULTI_KICK	BIT(1)
#define AVDQ_FLAG_SRAM		BIT(0)
#define AVDQ_MAX_PP_EXTRA	255
#define AVDQ_STATE_SUBMITTED	1
#define AVDQ_STATE_COMPLETE	2
#define AVDQ_NOTIFY_MASK	0xffff0000
#define AVDQ_NOTIFY		0x54530000
/* SRAM addresses as the CM3 doorbell sees them: 0x01000000 + offset from AVD base. */
#define AVDQ_CM3_APERTURE(off)	(0x01000000 + (off))
#define AVD_REG_MBOX0_STATUS	0x50
#define AVD_REG_MBOX0_SEND	0x54
#define AVDQ_MBOX_STATUS_EMPTY	BIT(17)
#define AVDQ_STREAM_TIMEOUT_US	(1000 * USEC_PER_MSEC)

/* Descriptor word offsets (bytes), avd-fw docs/queue.md. */
enum {
	D_MAGIC = 0x00, D_TAG = 0x04, D_CODEC = 0x08, D_VP = 0x0c,
	D_PROG_LO = 0x10, D_PROG_HI = 0x14, D_WORDS = 0x18, D_KICK = 0x1c,
	D_STATE = 0x20, D_STATUS = 0x24, D_CTAG = 0x28, D_FLAGS = 0x2c,
	D_PP_EXTRA = 0x30, D_PP_CONT = 0x34, D_RSVD0 = 0x38, D_RSVD1 = 0x3c,
};

struct avd_qfw {
	void __iomem *sram;
	resource_size_t sram_size;
	u32 aperture;		/* expected CM3 address of SRAM byte 0 */
	u32 ring_off;
	u32 ring_words;
	u32 wp;			/* host write count, free-running */
	u32 fifo_slots;
	u32 codec_mask;		/* info+0x24, bit n = codec n */
	bool usable;		/* last boot found a matching ABI 2 image */
};

static void __iomem *avd_qfw_desc(struct avd_qfw *q, unsigned int idx)
{
	return q->sram + AVDQ_RING_OFFSET + idx * AVDQ_DESC_BYTES;
}

/* Arguments of dev_dbg() are evaluated only when dynamic debug is enabled. */
void avd_qfw_debug(struct avd_dev *avd, struct avd_frame *f, const char *where)
{
	struct avd_qfw *q = avd->qfw;
	void __iomem *d;

	if (!avd_backend_is_qfw(avd))
		return;
	d = avd_qfw_desc(q, f->idx);
	dev_dbg(avd->dev,
		"queue %s: tag %#x slot %u host %u desc %u status %u ctag %#x rp %#x wp %#x mbox0 %#x mbox1 %#x\n",
		where, f->tag, f->idx, f->state, readl(d + D_STATE),
		readl(d + D_STATUS), readl(d + D_CTAG),
		readl(q->sram + AVDQ_INFO_RP), readl(q->sram + AVDQ_INFO_WP),
		readl(avd->mbox + AVD_REG_MBOX0_STATUS),
		readl(avd->mbox + AVD_REG_MBOX1_STATUS));
}

int avd_qfw_probe(struct avd_dev *avd)
{
	struct resource *code, *sram;
	struct avd_qfw *q;

	code = platform_get_resource_byname(avd->pdev, IORESOURCE_MEM, "code");
	sram = platform_get_resource_byname(avd->pdev, IORESOURCE_MEM, "sram");
	if (!code || !sram || sram->start < code->start ||
	    resource_size(sram) < AVDQ_RING_OFFSET + AVDQ_COUNT * AVDQ_DESC_BYTES)
		return 0;	/* no queue mode on this node: legacy transport only */
	q = devm_kzalloc(avd->dev, sizeof(*q), GFP_KERNEL);
	if (!q)
		return -ENOMEM;
	q->sram = devm_ioremap_resource(avd->dev, sram);
	if (IS_ERR(q->sram))
		return PTR_ERR(q->sram);
	q->sram_size = resource_size(sram);
	/* The code window sits at +0x80000 from the decoder base on every SoC. */
	q->aperture = AVDQ_CM3_APERTURE(0x80000 + (u32)(sram->start - code->start));
	avd->qfw = q;
	return 0;
}

/* Read the firmware's info block after boot; true when ABI 2 queue mode fits. */
static bool avd_qfw_check(struct avd_dev *avd)
{
	struct avd_qfw *q = avd->qfw;
	u32 info[16];
	unsigned int i;

	if (!q)
		return false;
	for (i = 0; i < ARRAY_SIZE(info); i++)
		info[i] = readl(q->sram + AVDQ_INFO + 4 * i);
	q->usable = false;
	if (info[0] != AVDQ_MAGIC)
		return false;
	if (info[1] != AVDQ_VERSION || info[2] != AVDQ_COUNT ||
	    info[3] != AVDQ_DESC_BYTES || info[5] != AVDQ_RING_OFFSET ||
	    !(info[11] & AVDQ_FEAT_SRAM_RING) ||
	    !(info[11] & AVDQ_FEAT_MULTI_KICK)) {
		dev_info_once(avd->dev, "open firmware queue ABI %u not supported, using legacy transport\n",
			      info[1]);
		return false;
	}
	if (info[4] != q->aperture) {
		dev_warn_once(avd->dev, "firmware SRAM aperture %#x, node needs %#x: wrong image tier, using legacy transport\n",
			      info[4], q->aperture);
		return false;
	}
	if (info[13] < 1024 || info[12] % 4 ||
	    (u64)info[12] + 4ULL * info[13] > AVDQ_RING_LIMIT ||
	    !info[10] || info[10] > AVDQ_COUNT || info[14] || info[15]) {
		dev_warn_once(avd->dev, "firmware SRAM ring geometry invalid, using legacy transport\n");
		return false;
	}
	q->ring_off = info[12];
	q->ring_words = info[13];
	q->fifo_slots = min_t(u32, info[10], avd->variant->fifo_slots ?: 1);
	q->codec_mask = info[9];
	q->wp = 0;
	q->usable = true;
	dev_dbg(avd->dev, "queue ABI %u: aperture %#x ring %#x/%u words FIFO slots %u codecs %#x\n",
		info[1], info[4], q->ring_off, q->ring_words, q->fifo_slots, q->codec_mask);
	return true;
}

static int avd_qfw_boot(struct avd_dev *avd)
{
	int ret = avd_boot(avd);

	if (ret)
		return ret;
	/* AP -> CM3 doorbells use mailbox 0; completions stay on mailbox 1. */
	writel_relaxed(AVD_MBOX_ENABLE, avd->mbox + AVD_REG_MBOX0_STATUS);
	return 0;
}

static unsigned int avd_qfw_max_inflight(struct avd_dev *avd)
{
	/* Each frame in flight owns one hardware instruction FIFO (EXEC 7:4). */
	return min_t(unsigned int, AVD_MAX_FRAMES, avd->qfw->fifo_slots);
}

static void avd_qfw_fail_all(struct avd_dev *avd)
{
	unsigned int i;

	/* ABI: any fatal word or I/O status needs a whole-queue reset. */
	for (i = 0; i < AVD_MAX_FRAMES; i++)
		if (avd->frames[i].state == AVD_FRAME_SUBMITTED)
			avd->frames[i].deadline = jiffies;
	mod_delayed_work(system_wq, &avd->queue_work, 0);
}

static int avd_qfw_stream(struct avd_dev *avd, const u32 *words, u32 count)
{
	struct avd_qfw *q = avd->qfw;
	unsigned long timeout = jiffies + usecs_to_jiffies(AVDQ_STREAM_TIMEOUT_US);
	u32 done = 0;

	while (done < count) {
		u32 rp = readl(q->sram + AVDQ_INFO_RP);
		u32 used = q->wp - rp, space, pos, chunk;

		if (used > q->ring_words)
			return -EIO;
		space = q->ring_words - used;
		if (!space) {
			if (time_after(jiffies, timeout))
				return -ETIMEDOUT;
			usleep_range(20, 50);
			continue;
		}
		pos = q->wp % q->ring_words;
		chunk = min3(space, count - done, q->ring_words - pos);
		/* The free-running u32 counter can wrap before the word ring does. */
		chunk = min_t(u64, chunk, (1ULL << 32) - q->wp);
		/*
		 * Word stores only: memcpy_toio() would use 64-bit and byte
		 * stores, and the CM3 data SRAM is only ever accessed with
		 * 32-bit ones elsewhere (info block, descriptors).
		 */
		__iowrite32_copy(q->sram + q->ring_off + 4 * pos, words + done, chunk);
		q->wp += chunk;
		done += chunk;
		/* Words before the count that publishes them. */
		wmb();
		writel(q->wp, q->sram + AVDQ_INFO_WP);
	}
	return 0;
}

static int avd_qfw_submit(struct avd_dev *avd, struct avd_frame *f)
{
	struct avd_qfw *q = avd->qfw;
	struct avd_ctx *ctx = f->ctx;
	u32 codec = f->cmd[0], vp = 0, kick, cont, mask, extra = f->cmd[1];
	u32 *words = f->prog->cpu;
	void __iomem *d = avd_qfw_desc(q, f->idx);
	unsigned int i;
	int ret;

	if (ctx->coded_fmt_desc->frame_ops)
		return -EOPNOTSUPP;
	if (codec > AVD_CODEC_AV1 || !words || !f->prog_words ||
	    f->prog_words > f->prog->size / sizeof(u32))
		return -EINVAL;
	if (!(avd->capabilities & BIT(codec)))
		return -EOPNOTSUPP;
	/* The program's EXEC words name FIFO f->idx; it must be a real FIFO. */
	if (f->idx >= q->fifo_slots)
		return -EINVAL;
	/* A busy inbox owns nothing; retry this same prepared frame later. */
	if (!(readl(avd->mbox + AVD_REG_MBOX0_STATUS) & AVDQ_MBOX_STATUS_EMPTY))
		return -EBUSY;
	for (i = 0; i < codec; i++)
		vp += avd->variant->vp_slots[i];
	if (avd->variant->vp_slot_base[codec])
		vp = avd->variant->vp_slot_base[codec];
	if (extra > AVDQ_MAX_PP_EXTRA)
		return -E2BIG;

	mask = codec == AVD_CODEC_VP9 && avd->variant->revision == 3 ?
		AVD_OP_EXEC_REV3_VP9_MASK : 0;
	cont = AVD_OP_EXEC | mask | AVD_OP_EXEC_FIFO_IDX(f->idx) |
	       AVD_OP_EXEC_FIFO_MASK(avd->variant->fifo_slots);
	kick = cont | AVD_OP_EXEC_FLAG_START_REV4(avd->variant->revision == 4) |
	       AVD_OP_EXEC_FLAG_START_REV3(avd->variant->revision == 3 &&
					   codec != AVD_CODEC_AV1);

	/* FIFO registers of other live frames are never touched. */
	avd->variant->configure_stream(avd, f->fifo_iova ?: ctx->inst.addr, f->idx, vp);

	writel(AVDQ_MAGIC, d + D_MAGIC);
	writel(f->tag, d + D_TAG);
	writel(codec, d + D_CODEC);
	writel(vp, d + D_VP);
	writel(q->wp, d + D_PROG_LO);
	writel(0, d + D_PROG_HI);
	writel(f->prog_words, d + D_WORDS);
	writel(kick, d + D_KICK);
	writel(0, d + D_STATUS);
	writel(0, d + D_CTAG);
	writel(AVDQ_FLAG_SRAM, d + D_FLAGS);
	writel(extra, d + D_PP_EXTRA);
	writel(extra ? cont : 0, d + D_PP_CONT);
	writel(0, d + D_RSVD0);
	writel(0, d + D_RSVD1);
	writel(AVDQ_STATE_SUBMITTED, d + D_STATE);
	/* Descriptor before the doorbell that hands it to the firmware. */
	wmb();
	(void)readl(d + D_STATE);	/* flush posted aperture writes */
	avd_qfw_debug(avd, f, "doorbell");
	dev_dbg(avd->dev, "queue program: tag %#x vp %u words %zu kick %#x start %#x end %#x\n",
		f->tag, vp, f->prog_words, kick, words[0], words[f->prog_words - 1]);
	writel(q->aperture + AVDQ_RING_OFFSET + f->idx * AVDQ_DESC_BYTES,
	       avd->mbox + AVD_REG_MBOX0_SEND);

	/* The firmware may start feeding while the rest of the program streams in. */
	ret = avd_qfw_stream(avd, words, f->prog_words);
	avd_qfw_debug(avd, f, "streamed");
	if (ret) {
		dev_err_ratelimited(avd->dev, "queue: program stream failed: %d\n", ret);
		/* The descriptor is already posted: only a reset releases it. */
		f->deadline = jiffies;
	}
	return 0;
}

static irqreturn_t avd_qfw_irq(struct avd_dev *avd)
{
	struct avd_qfw *q = avd->qfw;
	unsigned int n;
	u32 word;

	/* At most sixteen completions and one fatal token can be outstanding. */
	for (n = 0; n < AVDQ_COUNT + 1 && avd_mbox_pop(avd, &word); n++) {
		struct avd_frame *f;
		void __iomem *d;
		u32 state, status, ctag;
		unsigned int idx;

		if ((word & AVDQ_NOTIFY_MASK) != AVDQ_NOTIFY) {
			dev_err_ratelimited(avd->dev,
					    "queue: firmware reported %#x, resetting\n", word);
			avd_qfw_fail_all(avd);
			continue;
		}
		idx = word & 15;
		d = avd_qfw_desc(q, idx);
		state = readl(d + D_STATE);
		status = readl(d + D_STATUS);
		ctag = readl(d + D_CTAG);
		f = &avd->frames[idx];
		avd_qfw_debug(avd, f, "completion");
		if (state != AVDQ_STATE_COMPLETE || ((ctag ^ word) & 0xfff0) ||
		    f->state != AVD_FRAME_SUBMITTED || ctag != f->tag) {
			dev_warn_ratelimited(avd->dev, "queue: stale completion %#x (state %u tag %#x)\n",
					     word, state, ctag);
			continue;
		}
		if (status > 1) {
			/* An I/O error is not proof of quiescence, even for one frame. */
			avd_qfw_fail_all(avd);
			continue;
		}
		writel(0, d + D_STATE);
		avd_frame_complete(avd, ctag, status ? -EINVAL : 0);
	}
	return IRQ_HANDLED;
}

const struct avd_backend_ops avd_qfw_backend = {
	.name = "open-queue",
	.boot = avd_qfw_boot,
	.shutdown = avd_shutdown,
	.max_inflight = avd_qfw_max_inflight,
	.submit = avd_qfw_submit,
	.irq = avd_qfw_irq,
	.suspend = avd_pm_backend_suspend,
	.resume = avd_pm_backend_resume,
};

bool avd_backend_is_qfw(const struct avd_dev *avd)
{
	return avd->backend == &avd_qfw_backend;
}

/*
 * SRAM survives a CM3 reload, so a stale info block from an earlier ABI 2
 * image could make a legacy image look queue capable. Clear the magic before
 * every open-firmware boot; only an image that publishes it again qualifies.
 */
void avd_qfw_prepare_boot(struct avd_dev *avd)
{
	if (avd->qfw)
		writel(0, avd->qfw->sram + AVDQ_INFO);
}

/*
 * Called after every successful open-firmware boot (probe, resume, reset),
 * with no frame owned by the firmware. Picks the queued transport when the
 * loaded image speaks ABI 2 for this node, else the legacy one.
 */
void avd_qfw_select(struct avd_dev *avd)
{
	const struct avd_backend_ops *old = avd->backend;

	if (old != &avd_open_backend && old != &avd_qfw_backend)
		return;
	if (avd_qfw_check(avd)) {
		/* Legacy boot left mailbox 0 disabled. */
		writel_relaxed(AVD_MBOX_ENABLE, avd->mbox + AVD_REG_MBOX0_STATUS);
		avd->backend = &avd_qfw_backend;
		/*
		 * Codecs held back for a newer firmware (T6040 HEVC) are
		 * enabled by a queue image that lists them in its codec mask.
		 */
		avd->capabilities = avd->fw_capabilities |
			(avd->variant->capabilities & avd->qfw->codec_mask);
	} else {
		avd->backend = &avd_open_backend;
		avd->capabilities = avd->fw_capabilities;
	}
	if (avd->backend != old)
		dev_info(avd->dev, "using %s transport (%u frames in flight)\n",
			 avd->backend->name, avd->backend->max_inflight(avd));
}
