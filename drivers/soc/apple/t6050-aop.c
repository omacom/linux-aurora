// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple T6050 (M5 Pro) AOP2: started, and stopped for a system sleep.
 *
 * iBoot loads its firmware and leaves its CPU stopped. It starts with the mailbox's receive on,
 * then the CPU's run bit, then RTKit's boot.
 *
 * Its service endpoints open from the AP's side: INIT, then the firmware's GETBUF, its two rings,
 * and the AP's START. For a sleep the AP puts one element on each TX ring and waits for its
 * answer, then AP IDLE; the CPU keeps running. The INIT's argument and the element's message
 * come from the device tree.
 *
 * The rings live as long as the firmware runs: an element goes where the ring's write pointer
 * stands, and an answer is taken off the firmware's ring by moving its read pointer.
 *
 * After the sleep: AP ON, then one element on each TX ring again, the device tree's wake message,
 * each answered by interrupt. An endpoint whose wake element was not answered stays closed and
 * gets no sleep element at the next sleep.
 *
 * macOS stops AOP2 before AOP, and both about 200 ms before the commit: AOP2's elements and AP
 * IDLE first, then AOP's endpoint stops, and AOP's AP IDLE 18-25 ms after AOP2's (J714s, macOS 27
 * under the hypervisor, five captures). AOP2 is therefore a consumer of AOP for the PM core, so
 * that its late suspend, which stops it, runs before AOP's, which stops AOP; the commit's pass
 * then finds it stopped. Without that link it is stopped in that pass, as before.
 */

#include <linux/bitfield.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/soc/apple/rtkit.h>
#include <linux/soc/apple/sleep.h>
#include <linux/suspend.h>

#define AOP_CPU_CONTROL		0x44
#define AOP_CPU_CONTROL_RUN	BIT(4)

#define AFK_TYPE		GENMASK_ULL(63, 48)
#define AFK_INIT		0x80
#define AFK_INIT_ACK		0xa0
#define AFK_GETBUF		0x89
#define AFK_GETBUF_ACK		0xa1
#define AFK_GETBUF_SIZE		GENMASK_ULL(31, 16)	/* 64-byte blocks */
#define AFK_GETBUF_IOVA		GENMASK_ULL(47, 0)
#define AFK_INIT_TX		0x8a
#define AFK_INIT_RX		0x8b
#define AFK_RING_OFFSET		GENMASK_ULL(47, 32)	/* 64-byte blocks */
#define AFK_RING_SIZE		GENMASK_ULL(31, 16)	/* 64-byte blocks */
#define AFK_START		0xa3
#define AFK_START_ACK		0x86
#define AFK_RING		0x85
#define AFK_RING_RPTR		GENMASK_ULL(47, 32)	/* the sender's ring, as its reader left it */
#define AFK_RING_NEW_WPTR	GENMASK_ULL(31, 16)
#define AFK_RING_OLD_WPTR	GENMASK_ULL(15, 0)
#define AFK_BLOCK_SHIFT		6
#define AFK_QE_MAGIC		0x20504f49	/* "IOP " */
#define AFK_QE_HDR_WORDS	4		/* magic, size, channel, type */
#define AFK_QE_SLOT		0x80
#define AFK_RING_MAX		0x10000			/* its pointers are 16 bits of the message */
#define AFK_QE_SEQ		GENMASK(7, 0)		/* the message's first word: its count */
#define AFK_QE_ANSWER_WORDS	3			/* count, 4, code */
#define AFK_QE_ANSWER_CODE	GENMASK(7, 0)
#define AFK_QE_CODE_WORD	2			/* the message's word with its code */
#define AOP2_WAKE_ANSWER_MS	250
#define AOP2_SLEEP_MSG_MAX	(AFK_QE_SLOT / 4 - AFK_QE_HDR_WORDS)

#define AOP_EP_FIRST		0x20
#define AOP_EP_LAST		0x2c
#define AOP_EP_COUNT		(AOP_EP_LAST - AOP_EP_FIRST + 1)

struct t6050_aop_ring {
	void *buf;
	size_t size;
	u32 tx_off, tx_size;
	u32 rx_off, rx_size;
	bool tx_ready, rx_ready;
	u8 seq;			/* the next element's count */
	bool closed;		/* its sleep element was answered, a wake element not yet */
	u32 tx_rptr, tx_wptr;	/* as the last element found them */
};

/* A ring in its buffer: size, read pointer and write pointer, a block each, then the data. */
struct t6050_aop_view {
	u8 *hdr;
	u32 bufsz, stride;
};

struct t6050_aop {
	struct device *dev;
	void __iomem *asc;
	struct apple_rtkit *rtk;
	bool registered;
	/* Ordered before AOP by the PM core: stopped at its late suspend, not at the commit */
	struct device_link *aop_link;
	/* This sleep's stop ran at the late suspend: the commit's pass has nothing to send */
	bool stopped_late;
	unsigned long opened;	/* bit (ep - AOP_EP_FIRST): open as its side of the protocol needs */
	unsigned long seen;	/* bit (ep - AOP_EP_FIRST): the firmware sent something */
	struct t6050_aop_ring ring[AOP_EP_COUNT];
	unsigned int stopped, stop_errors;
	unsigned int answers;	/* the last stop's answers read off the firmware's rings */
	unsigned long answer_codes;	/* bit n: an answer carried code n */
	int idle_ret;
	/* Since a sleep's stop, until AP ON is answered after it */
	bool asleep;
	/* The wake: the endpoint whose answer is waited for, and the code that answers */
	struct completion wake_answered;
	u8 wake_ep;
	u32 wake_code;
	/* From the device tree: the INIT's argument, and the sleep and wake elements' messages */
	u32 init_arg;
	u32 sleep_msg[AOP2_SLEEP_MSG_MAX], wake_msg[AOP2_SLEEP_MSG_MAX];
	unsigned int sleep_msg_len, wake_msg_len;
	/* The order the endpoints open in, and the order they get the sleep message in */
	u8 open_order[AOP_EP_COUNT], sleep_order[AOP_EP_COUNT];
	unsigned int open_len, sleep_len;
};

static void t6050_aop_crashed(void *cookie, const void *crashlog, size_t size)
{
	struct t6050_aop *aop = cookie;

	dev_err(aop->dev, "firmware crashed\n");
}

static void t6050_aop_send(struct t6050_aop *aop, u8 ep, u64 msg)
{
	int ret = apple_rtkit_send_message(aop->rtk, ep, msg, NULL, false);

	if (ret)
		dev_err(aop->dev, "endpoint %#x: send %#llx: %d\n", ep, msg, ret);
}

static void t6050_aop_getbuf(struct t6050_aop *aop, u8 ep, u64 msg)
{
	size_t size = (size_t)FIELD_GET(AFK_GETBUF_SIZE, msg) << AFK_BLOCK_SHIFT;
	dma_addr_t iova;
	void *buf;

	buf = dmam_alloc_coherent(aop->dev, size, &iova, GFP_KERNEL);
	if (!buf) {
		dev_err(aop->dev, "endpoint %#x: no %#zx byte buffer\n", ep, size);
		return;
	}
	aop->ring[ep - AOP_EP_FIRST].buf = buf;
	aop->ring[ep - AOP_EP_FIRST].size = size;
	t6050_aop_send(aop, ep, FIELD_PREP(AFK_TYPE, AFK_GETBUF_ACK) |
				FIELD_PREP(AFK_GETBUF_IOVA, iova));
}

static int t6050_aop2_take(struct t6050_aop *aop, u8 ep, u32 old, u32 new);

static void t6050_aop_recv(void *cookie, u8 ep, u64 msg)
{
	struct t6050_aop *aop = cookie;
	struct t6050_aop_ring *r;
	int code;

	if (ep < AOP_EP_FIRST || ep > AOP_EP_LAST)
		return;
	r = &aop->ring[ep - AOP_EP_FIRST];
	set_bit(ep - AOP_EP_FIRST, &aop->seen);
	switch (FIELD_GET(AFK_TYPE, msg)) {
	case AFK_INIT:
		t6050_aop_send(aop, ep, FIELD_PREP(AFK_TYPE, AFK_INIT_ACK));
		break;
	case AFK_GETBUF:
		t6050_aop_getbuf(aop, ep, msg);
		break;
	case AFK_INIT_TX:
		r->tx_off = FIELD_GET(AFK_RING_OFFSET, msg) << AFK_BLOCK_SHIFT;
		r->tx_size = FIELD_GET(AFK_RING_SIZE, msg) << AFK_BLOCK_SHIFT;
		r->tx_ready = true;
		break;
	case AFK_INIT_RX:
		r->rx_off = FIELD_GET(AFK_RING_OFFSET, msg) << AFK_BLOCK_SHIFT;
		r->rx_size = FIELD_GET(AFK_RING_SIZE, msg) << AFK_BLOCK_SHIFT;
		r->rx_ready = true;
		t6050_aop_send(aop, ep, FIELD_PREP(AFK_TYPE, AFK_START));
		break;
	case AFK_START_ACK:
		set_bit(ep - AOP_EP_FIRST, &aop->opened);
		break;
	case AFK_RING:
		/* Only the wake's answers are taken here; the sleep's are, interrupts off. */
		if (READ_ONCE(aop->wake_ep) != ep)
			break;
		code = t6050_aop2_take(aop, ep, FIELD_GET(AFK_RING_OLD_WPTR, msg),
				       FIELD_GET(AFK_RING_NEW_WPTR, msg));
		/* Without a ring to read, the firmware's message itself is the answer. */
		if (code == -EINVAL || code == aop->wake_code) {
			r->closed = false;
			WRITE_ONCE(aop->wake_ep, 0);
			complete(&aop->wake_answered);
		} else {
			dev_dbg(aop->dev, "endpoint %#x: %#llx is not the wake's answer: %d\n", ep,
				msg, code);
		}
		break;
	default:
		/* The rings' announcements and the firmware's own traffic need no answer. */
		break;
	}
}

/* Its endpoints start from the AP's side, after boot. */
static const struct apple_rtkit_ops t6050_aop2_rtkit_ops = {
	.flags = APPLE_RTKIT_FIRMWARE_OWNED_LOGS,
	.crashed = t6050_aop_crashed,
	.recv_message = t6050_aop_recv,
};

static int t6050_aop2_view(struct t6050_aop_ring *r, u32 off, u32 size, bool ready,
			   struct t6050_aop_view *v)
{
	if (!ready || !r->buf || size < sizeof(__le32) || off > r->size || size > r->size - off)
		return -EINVAL;
	v->hdr = (u8 *)r->buf + off;
	v->bufsz = le32_to_cpu(READ_ONCE(*(__le32 *)v->hdr));
	if (v->bufsz >= size || (size - v->bufsz) % 3 || v->bufsz < 2 * AFK_QE_SLOT ||
	    v->bufsz > AFK_RING_MAX)
		return -EINVAL;
	v->stride = (size - v->bufsz) / 3;
	if (v->stride < sizeof(__le32))
		return -EINVAL;

	return 0;
}

static u32 t6050_aop2_ptr(const struct t6050_aop_view *v, unsigned int which)
{
	return le32_to_cpu(READ_ONCE(*(__le32 *)(v->hdr + which * v->stride)));
}

#define AFK_PTR_READ	1
#define AFK_PTR_WRITE	2

/*
 * One element on an AOP2 TX ring, where its write pointer stands: the "IOP " header, then the
 * message with the endpoint's count in its first word. An element takes one slot; one that would
 * cross the ring's end goes to its start, as the display's does. *kick is the message that tells
 * the firmware: its read pointer, the new write pointer, the old one.
 */
static int t6050_aop2_put(struct t6050_aop *aop, u8 ep, const u32 *msg, unsigned int len,
			  u64 *kick)
{
	struct t6050_aop_ring *r = &aop->ring[ep - AOP_EP_FIRST];
	struct t6050_aop_view v;
	u32 rptr, wptr, at, next, used;
	unsigned int i;
	__le32 *qe;
	int ret;

	ret = t6050_aop2_view(r, r->tx_off, r->tx_size, r->tx_ready, &v);
	if (ret)
		return ret;
	dma_rmb();
	rptr = t6050_aop2_ptr(&v, AFK_PTR_READ);
	wptr = t6050_aop2_ptr(&v, AFK_PTR_WRITE);
	r->tx_rptr = rptr;
	r->tx_wptr = wptr;
	if (rptr >= v.bufsz || wptr >= v.bufsz || (wptr & 3))
		return -EPROTO;
	at = wptr;
	next = wptr + AFK_QE_SLOT;
	if (next > v.bufsz) {
		at = 0;
		next = AFK_QE_SLOT;
	} else if (next == v.bufsz) {
		next = 0;
	}
	/* What the firmware has not read yet, this element and one slot kept empty. */
	used = wptr >= rptr ? wptr - rptr : v.bufsz - rptr + wptr;
	if (used + (at == wptr ? 0 : v.bufsz - wptr) + 2 * AFK_QE_SLOT > v.bufsz)
		return -EAGAIN;

	qe = (__le32 *)(v.hdr + 3 * v.stride + at);
	if (at != wptr && v.bufsz - wptr >= AFK_QE_HDR_WORDS * sizeof(*qe)) {
		/* The header alone where the pointer stood says the element is at the start. */
		__le32 *stub = (__le32 *)(v.hdr + 3 * v.stride + wptr);

		stub[0] = cpu_to_le32(AFK_QE_MAGIC);
		stub[1] = cpu_to_le32(4 * len);
		stub[2] = 0;
		stub[3] = 0;
	}
	qe[0] = cpu_to_le32(AFK_QE_MAGIC);
	qe[1] = cpu_to_le32(4 * len);
	qe[2] = 0;	/* channel */
	qe[3] = 0;	/* type */
	for (i = 0; i < len; i++)
		qe[AFK_QE_HDR_WORDS + i] = cpu_to_le32(i ? msg[i] :
						       (msg[0] & ~AFK_QE_SEQ) | r->seq);
	r->seq++;
	dma_wmb();
	WRITE_ONCE(*(__le32 *)(v.hdr + AFK_PTR_WRITE * v.stride), cpu_to_le32(next));
	dma_wmb();
	*kick = FIELD_PREP(AFK_TYPE, AFK_RING) | FIELD_PREP(AFK_RING_RPTR, rptr) |
		FIELD_PREP(AFK_RING_NEW_WPTR, next) | FIELD_PREP(AFK_RING_OLD_WPTR, wptr);

	return 0;
}

/* The firmware's ring's write pointer; -EINVAL without a ring to read. */
static int t6050_aop2_rx_wptr(struct t6050_aop *aop, u8 ep)
{
	struct t6050_aop_ring *r = &aop->ring[ep - AOP_EP_FIRST];
	struct t6050_aop_view v;
	u32 wptr;

	if (t6050_aop2_view(r, r->rx_off, r->rx_size, r->rx_ready, &v))
		return -EINVAL;
	dma_rmb();
	wptr = t6050_aop2_ptr(&v, AFK_PTR_WRITE);

	return wptr < v.bufsz ? wptr : -EINVAL;
}

/* The code of an answer to a sleep or wake element at this place of a ring's data, or -ENOMSG. */
static int t6050_aop2_answer_at(const struct t6050_aop_view *v, u32 at)
{
	const __le32 *qe = (const __le32 *)(v->hdr + 3 * v->stride + at);

	if (at > v->bufsz - (AFK_QE_HDR_WORDS + AFK_QE_ANSWER_WORDS) * sizeof(*qe) ||
	    le32_to_cpu(READ_ONCE(qe[0])) != AFK_QE_MAGIC ||
	    le32_to_cpu(READ_ONCE(qe[1])) != AFK_QE_ANSWER_WORDS * sizeof(*qe) ||
	    le32_to_cpu(READ_ONCE(qe[2])) || le32_to_cpu(READ_ONCE(qe[3])) ||
	    le32_to_cpu(READ_ONCE(qe[AFK_QE_HDR_WORDS + 1])) != 4)
		return -ENOMSG;

	return FIELD_GET(AFK_QE_ANSWER_CODE, le32_to_cpu(READ_ONCE(qe[AFK_QE_HDR_WORDS + 2])));
}

/*
 * What the firmware put on its ring between two of its write pointers is taken: the read pointer
 * moves up to the new one. The answer's code if the element at the old one is an answer to a
 * sleep or wake element (at the ring's start if it went around), -ENOMSG if it is something
 * else or nothing came, -EINVAL without a ring to read.
 */
static int t6050_aop2_take(struct t6050_aop *aop, u8 ep, u32 old, u32 new)
{
	struct t6050_aop_ring *r = &aop->ring[ep - AOP_EP_FIRST];
	struct t6050_aop_view v;
	int code;

	if (t6050_aop2_view(r, r->rx_off, r->rx_size, r->rx_ready, &v) || old >= v.bufsz ||
	    new >= v.bufsz)
		return -EINVAL;
	if (old == new)
		return -ENOMSG;
	dma_rmb();
	code = t6050_aop2_answer_at(&v, old);
	if (code < 0 && new < old)
		code = t6050_aop2_answer_at(&v, 0);
	WRITE_ONCE(*(__le32 *)(v.hdr + AFK_PTR_READ * v.stride), cpu_to_le32(new));
	dma_wmb();

	return code;
}

/* The sleep's element on one endpoint, interrupts off: put, told, and its answer waited for. */
static int t6050_aop2_element(struct t6050_aop *aop, u8 ep)
{
	int before = t6050_aop2_rx_wptr(aop, ep), after, code, ret;
	u64 kick;

	ret = t6050_aop2_put(aop, ep, aop->sleep_msg, aop->sleep_msg_len, &kick);
	if (ret)
		return ret;
	ret = apple_rtkit_polled_exchange(aop->rtk, ep, kick, AFK_TYPE,
					  FIELD_PREP(AFK_TYPE, AFK_RING));
	if (ret)
		return ret;
	aop->ring[ep - AOP_EP_FIRST].closed = true;
	after = t6050_aop2_rx_wptr(aop, ep);
	if (before < 0 || after < 0)
		return 0;
	code = t6050_aop2_take(aop, ep, before, after);
	if (code >= 0) {
		aop->answers++;
		if (code < BITS_PER_LONG)
			aop->answer_codes |= BIT(code);
	}

	return 0;
}

/*
 * At the late suspend, before AOP's, or else in the sleep's first pass with interrupts off: the
 * CPU keeps running.
 */
static int t6050_aop_sleep_stop(void *data)
{
	struct t6050_aop *aop = data;
	unsigned int i, us, skipped = 0;
	int first_err = 0, ret;
	u8 first_ep = 0;

	aop->stopped = aop->stop_errors = aop->answers = 0;
	aop->answer_codes = 0;
	/*
	 * Not restarted after the last sleep: the firmware is still in that sleep's AP IDLE with
	 * its endpoints closed, which is what this stop is for. Nothing more may be sent to it.
	 */
	if (aop->asleep || apple_rtkit_is_crashed(aop->rtk)) {
		aop->idle_ret = 0;
		dev_warn(aop->dev, "sleep stop: nothing sent: %s\n",
			 aop->asleep ? "still in the last sleep's AP IDLE" : "the firmware crashed");
		return 0;
	}
	aop->asleep = true;
	apple_rtkit_polled_begin(aop->rtk);
	/* Without a message from the device tree, only the AP's IDLE. */
	for (i = 0; aop->sleep_msg_len && i < aop->sleep_len; i++) {
		u8 ep = aop->sleep_order[i];

		if (!test_bit(ep - AOP_EP_FIRST, &aop->opened))
			continue;
		/* One the last wake did not open again is closed already. */
		if (aop->ring[ep - AOP_EP_FIRST].closed) {
			skipped++;
			continue;
		}
		ret = t6050_aop2_element(aop, ep);
		if (!ret) {
			aop->stopped++;
			continue;
		}
		if (!aop->stop_errors++) {
			first_err = ret;
			first_ep = ep;
		}
	}
	aop->idle_ret = apple_rtkit_polled_ap_idle(aop->rtk, &us);
	apple_rtkit_polled_end(aop->rtk);
	dev_info(aop->dev, "sleep stop: endpoints seen %#lx, open %#lx: %u stopped, %u missed, AP IDLE %d in %u us\n",
		 aop->seen, aop->opened, aop->stopped, aop->stop_errors, aop->idle_ret, us);
	/* The first sleep element's ring, and what the answers said. */
	if (aop->sleep_msg_len && aop->sleep_len) {
		struct t6050_aop_ring *r = &aop->ring[aop->sleep_order[0] - AOP_EP_FIRST];

		dev_info(aop->dev, "sleep stop: endpoint %#x's ring stood at read %#x write %#x, count %u; %u answers read, codes %#lx\n",
			 aop->sleep_order[0], r->tx_rptr, r->tx_wptr, (u8)(r->seq - 1), aop->answers,
			 aop->answer_codes);
	}
	if (aop->stop_errors)
		dev_err(aop->dev, "sleep stop: endpoint %#x was the first missed: %d\n", first_ep,
			first_err);
	if (skipped)
		dev_warn(aop->dev, "sleep stop: %u endpoints were still closed from the last sleep, no element\n",
			 skipped);

	return aop->stop_errors || aop->idle_ret;
}

/* The commit's first pass: nothing more once the late suspend has stopped it. */
static int t6050_aop_commit_stop(void *data)
{
	struct t6050_aop *aop = data;

	if (aop->stopped_late)
		return aop->stop_errors || aop->idle_ret;

	return t6050_aop_sleep_stop(aop);
}

static int t6050_aop_suspend(struct device *dev)
{
	struct t6050_aop *aop = dev_get_drvdata(dev);

	if (!pm_suspend_via_firmware() || !aop->registered)
		return 0;
	aop->stopped_late = false;
	/* Ready for the commit either way: a registered coprocessor that isn't refuses the sleep. */
	apple_sleep_coproc_set_ready(dev, true);

	return 0;
}

/*
 * Before AOP's late suspend stops AOP, as macOS stops AOP2 first. A stop that fails leaves the
 * sleep to go on, as at the commit; the resume restarts what it closed.
 */
static int t6050_aop_suspend_late(struct device *dev)
{
	struct t6050_aop *aop = dev_get_drvdata(dev);

	if (!pm_suspend_via_firmware() || !aop->registered || !aop->aop_link)
		return 0;
	t6050_aop_sleep_stop(aop);
	aop->stopped_late = true;

	return 0;
}

/*
 * The wake's element on one endpoint: put, told, and its answer waited for; the answer comes by
 * interrupt (t6050_aop_recv).
 */
static int t6050_aop2_wake_element(struct t6050_aop *aop, u8 ep)
{
	u64 kick;
	int ret;

	reinit_completion(&aop->wake_answered);
	ret = t6050_aop2_put(aop, ep, aop->wake_msg, aop->wake_msg_len, &kick);
	if (ret)
		return ret;
	WRITE_ONCE(aop->wake_ep, ep);
	ret = apple_rtkit_send_message(aop->rtk, ep, kick, NULL, false);
	if (!ret && !wait_for_completion_timeout(&aop->wake_answered,
						 msecs_to_jiffies(AOP2_WAKE_ANSWER_MS)))
		ret = -ETIMEDOUT;
	WRITE_ONCE(aop->wake_ep, 0);

	return ret;
}

/* After a sleep whose stop ran: AP ON, then the wake element on every endpoint the stop closed. */
static void t6050_aop_restart(struct t6050_aop *aop)
{
	unsigned int i, closed = 0, opened = 0;
	unsigned long failed = 0;
	int ret, first_err = 0;

	if (apple_rtkit_is_crashed(aop->rtk)) {
		dev_err(aop->dev, "AOP: not restarted after the sleep: the firmware crashed (CPU control %#x)\n",
			readl(aop->asc + AOP_CPU_CONTROL));
		return;
	}
	ret = apple_rtkit_ap_power(aop->rtk, true);
	if (ret) {
		dev_err(aop->dev, "AOP: not restarted after the sleep: AP ON not answered: %d (CPU control %#x); it stays in AP IDLE\n",
			ret, readl(aop->asc + AOP_CPU_CONTROL));
		return;
	}
	aop->asleep = false;
	for (i = 0; i < aop->sleep_len; i++) {
		u8 ep = aop->sleep_order[i];

		if (!aop->ring[ep - AOP_EP_FIRST].closed)
			continue;
		closed++;
		if (!aop->wake_msg_len)
			continue;
		ret = t6050_aop2_wake_element(aop, ep);
		if (!ret) {
			opened++;
			continue;
		}
		if (!failed)
			first_err = ret;
		failed |= BIT(ep - AOP_EP_FIRST);
	}
	if (closed && !aop->wake_msg_len)
		dev_warn(aop->dev, "AOP: restarted after the sleep, AP ON; no wake message in the device tree: %u endpoints stay closed (CPU control %#x)\n",
			 closed, readl(aop->asc + AOP_CPU_CONTROL));
	else if (failed)
		dev_err(aop->dev, "AOP: restarted after the sleep, AP ON, but only %u of %u endpoints open again: not answered %#lx, first error %d (CPU control %#x)\n",
			opened, closed, failed, first_err, readl(aop->asc + AOP_CPU_CONTROL));
	else
		dev_info(aop->dev, "AOP: restarted after the sleep, AP ON, %u of %u endpoints open again (CPU control %#x)\n",
			 opened, closed, readl(aop->asc + AOP_CPU_CONTROL));
}

static int t6050_aop_resume(struct device *dev)
{
	struct t6050_aop *aop = dev_get_drvdata(dev);
	bool stopped;

	if (!aop->registered)
		return 0;
	/* Read before the ready flag goes: clearing that clears the stopped flag with it. */
	stopped = apple_sleep_coproc_stopped(dev);
	apple_sleep_coproc_set_ready(dev, false);
	aop->stopped_late = false;
	if (!stopped && !aop->asleep)
		return 0;
	t6050_aop_restart(aop);

	return 0;
}

static const struct dev_pm_ops t6050_aop_pm_ops = {
	SYSTEM_SLEEP_PM_OPS(t6050_aop_suspend, t6050_aop_resume)
	.suspend_late = pm_sleep_ptr(t6050_aop_suspend_late),
};

/* AOP2 becomes a consumer of AOP (the Rust apple_aop driver's device) for the PM core's order. */
static void t6050_aop_order_before_aop(struct t6050_aop *aop)
{
	struct device_node *np;
	struct platform_device *aop_pdev = NULL;

	for_each_compatible_node(np, NULL, "apple,t6050-aop") {
		if (!of_device_is_available(np))
			continue;
		aop_pdev = of_find_device_by_node(np);
		of_node_put(np);
		break;
	}
	if (!aop_pdev) {
		dev_info(aop->dev, "no AOP to stop before: stopped at the commit\n");
		return;
	}
	aop->aop_link = device_link_add(aop->dev, &aop_pdev->dev, DL_FLAG_STATELESS);
	if (aop->aop_link)
		dev_info(aop->dev, "stopped before %s for a sleep, as macOS\n", dev_name(&aop_pdev->dev));
	else
		dev_warn(aop->dev, "not ordered before %s: stopped at the commit\n",
			 dev_name(&aop_pdev->dev));
	put_device(&aop_pdev->dev);
}

/*
 * An endpoint order from the device tree. Without the property, or with one that is not a list
 * of service endpoints each named once, the order is the endpoints' own.
 */
static unsigned int t6050_aop2_order(struct device *dev, const char *prop, u8 *order)
{
	u32 list[AOP_EP_COUNT];
	unsigned long named = 0;
	int i, n = 0;

	if (of_property_present(dev->of_node, prop))
		n = of_property_read_variable_u32_array(dev->of_node, prop, list, 1, AOP_EP_COUNT);
	for (i = 0; i < n; i++) {
		if (list[i] < AOP_EP_FIRST || list[i] > AOP_EP_LAST ||
		    (named & BIT(list[i] - AOP_EP_FIRST))) {
			n = -EINVAL;
			break;
		}
		named |= BIT(list[i] - AOP_EP_FIRST);
		order[i] = list[i];
	}
	if (n > 0)
		return n;
	if (n < 0)
		dev_warn(dev, "%s is not a list of service endpoints, each once: endpoint order\n",
			 prop);
	for (i = 0; i < AOP_EP_COUNT; i++)
		order[i] = AOP_EP_FIRST + i;

	return AOP_EP_COUNT;
}

static int t6050_aop_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct t6050_aop *aop;
	unsigned int i, ep, started = 0;
	u32 control;
	int ret;

	aop = devm_kzalloc(dev, sizeof(*aop), GFP_KERNEL);
	if (!aop)
		return -ENOMEM;
	aop->dev = dev;
	platform_set_drvdata(pdev, aop);

	of_property_read_u32(dev->of_node, "apple,init-arg", &aop->init_arg);
	ret = of_property_read_variable_u32_array(dev->of_node, "apple,sleep-message", aop->sleep_msg,
						  1, AOP2_SLEEP_MSG_MAX);
	if (ret > 0)
		aop->sleep_msg_len = ret;
	else
		dev_dbg(dev, "no sleep message: its endpoints get only AP IDLE before a sleep\n");
	ret = of_property_read_variable_u32_array(dev->of_node, "apple,wake-message", aop->wake_msg,
						  AFK_QE_CODE_WORD + 1, AOP2_SLEEP_MSG_MAX);
	if (ret > 0) {
		aop->wake_msg_len = ret;
		/* The firmware answers with the code after the message's. */
		aop->wake_code = FIELD_GET(AFK_QE_ANSWER_CODE, aop->wake_msg[AFK_QE_CODE_WORD]) + 1;
	} else if (aop->sleep_msg_len) {
		dev_warn(dev, "a sleep message and no wake message: endpoints closed for a sleep stay closed\n");
	}
	init_completion(&aop->wake_answered);
	aop->open_len = t6050_aop2_order(dev, "apple,open-endpoints", aop->open_order);
	aop->sleep_len = t6050_aop2_order(dev, "apple,sleep-endpoints", aop->sleep_order);

	aop->asc = devm_platform_ioremap_resource_byname(pdev, "asc");
	if (IS_ERR(aop->asc))
		return PTR_ERR(aop->asc);
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(42));
	if (ret)
		return ret;

	/* Only a stopped CPU, as iBoot leaves it, is started here. */
	control = readl(aop->asc + AOP_CPU_CONTROL);
	if (control & AOP_CPU_CONTROL_RUN)
		return dev_err_probe(dev, -EBUSY, "already running (control %#x)\n", control);

	/* The RTKit's mailbox turns its receive on; then the run bit. */
	aop->rtk = devm_apple_rtkit_init(dev, aop, "mbox", 0, &t6050_aop2_rtkit_ops);
	if (IS_ERR(aop->rtk))
		return dev_err_probe(dev, PTR_ERR(aop->rtk), "no RTKit\n");
	writel(AOP_CPU_CONTROL_RUN, aop->asc + AOP_CPU_CONTROL);

	ret = apple_rtkit_boot(aop->rtk);
	if (ret)
		return dev_err_probe(dev, ret, "firmware didn't start\n");

	/* One the firmware does not have is passed over. */
	for (i = 0; i < aop->open_len; i++) {
		ep = aop->open_order[i];
		if (apple_rtkit_start_ep(aop->rtk, ep))
			continue;
		started++;
		t6050_aop_send(aop, ep, FIELD_PREP(AFK_TYPE, AFK_INIT) | aop->init_arg);
	}

	ret = apple_sleep_coproc_register_first(dev, t6050_aop_commit_stop, aop);
	if (ret)
		dev_warn(dev, "not told of a system sleep: %d\n", ret);
	else
		aop->registered = true;
	if (aop->registered)
		t6050_aop_order_before_aop(aop);
	dev_info(dev, "firmware started (control %#x), %u endpoints started\n",
		 readl(aop->asc + AOP_CPU_CONTROL), started);

	return 0;
}

static const struct of_device_id t6050_aop_of_match[] = {
	/* AOP itself is the Rust apple_aop driver's (its setup port); AOP2 stays here. */
	{ .compatible = "apple,t6050-aop2" },
	{}
};

static struct platform_driver t6050_aop_driver = {
	.driver = {
		.name = "t6050-aop",
		.of_match_table = t6050_aop_of_match,
		.pm = pm_sleep_ptr(&t6050_aop_pm_ops),
		.suppress_bind_attrs = true,
	},
	.probe = t6050_aop_probe,
};
builtin_platform_driver(t6050_aop_driver);
