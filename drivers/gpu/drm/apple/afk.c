// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Copyright 2022 Sven Peter <sven@svenpeter.dev> */

#include <linux/bitfield.h>
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/jiffies.h>
#include <linux/kconfig.h>
#include <linux/moduleparam.h>
#include <linux/of_platform.h>
#include <linux/overflow.h>
#include <linux/slab.h>
#include <linux/sizes.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>
#include <linux/ioport.h>
#include <linux/soc/apple/dart.h>
#include <linux/soc/apple/rtkit.h>

#include <kunit/visibility.h>

#include "afk.h"
#include "afkv2-codec.h"
#include "afk_reply.h"
#include "trace.h"

struct afk_receive_message_work {
	struct apple_dcp_afkep *ep;
	u64 message;
	struct work_struct work;
};

#define RBEP_TYPE GENMASK(63, 48)

enum rbep_msg_type {
	RBEP_INIT = 0x80,
	RBEP_INIT_ACK = 0xa0,
	RBEP_GETBUF = 0x89,
	RBEP_GETBUF_ACK = 0xa1,
	RBEP_INIT_TX = 0x8a,
	RBEP_INIT_RX = 0x8b,
	RBEP_START = 0xa3,
	RBEP_START_ACK = 0x86,
	RBEP_SEND = 0xa2,
	RBEP_RECV = 0x85,
	RBEP_SPACE_AVAILABLE = 0xa7,
	RBEP_WAIT_FOR_SPACE = 0xa8,
	RBEP_SHUTDOWN = 0xc0,
	RBEP_SHUTDOWN_ACK = 0xc1,
};

#define BLOCK_SHIFT 6

#define GETBUF_SIZE GENMASK(31, 16)
#define GETBUF_TAG GENMASK(15, 0)
#define GETBUF_ACK_DVA GENMASK(47, 0)

#define INITRB_OFFSET GENMASK(47, 32)
#define INITRB_SIZE GENMASK(31, 16)
#define INITRB_TAG GENMASK(15, 0)

#define SEND_WPTR GENMASK(31, 0)

static void afk_service_worker(struct work_struct *work);
static bool afk_compact_capable(struct apple_dcp_afkep *ep);
static void afk_discard_pending_open(struct apple_epic_service *service);

void afk_service_retry(struct apple_dcp_afkep *ep)
{
	unsigned long flags;

	if (!ep || !ep->wire_v2 ||
	    (!dcp_is_ext26(ep->dcp) && ep != ep->dcp->dcpavservep))
		return;
	spin_lock_irqsave(&ep->lock, flags);
	if (!ep->stopping)
		queue_work(ep->wq, &ep->service_work);
	spin_unlock_irqrestore(&ep->lock, flags);
}
EXPORT_SYMBOL_IF_KUNIT(afk_service_retry);

static void afk_send(struct apple_dcp_afkep *ep, u64 message)
{
	dcp_send_message(ep->dcp, ep->endpoint, message);
}

static void afk_release_context(void *data)
{
	struct apple_dcp_afkep *afkep = data;

	afk_quiesce(afkep);
	/* A superseded endpoint's devres must not erase its replacement route. */
	if (afkep->dcp->v27_sleep_transport &&
	    afkep->endpoint >= 0x20 && afkep->endpoint < 0x40)
		cmpxchg(&afkep->dcp->sleep_v27_ep[afkep->endpoint - 0x20], afkep, NULL);
	if (!afkep->dcp->retain_dma) {
		kvfree(afkep->command_tags);
		kfree(afkep);
	}
}

struct apple_dcp_afkep *afk_init(struct apple_dcp *dcp, u32 endpoint,
				 const struct apple_epic_service_ops *ops)
{
	struct apple_dcp_afkep *afkep;
	int ret;

	afkep = kzalloc_obj(*afkep);
	if (!afkep)
		return ERR_PTR(-ENOMEM);

	afkep->ops = ops;
	afkep->dcp = dcp;
	afkep->wire_v2 = dcp->fw_compat == DCP_FIRMWARE_V_27_0 || dcp_is_ext26(dcp);
	afkep->endpoint = endpoint;
	afkep->wq = alloc_ordered_workqueue("apple-dcp-afkep%02x",
					    WQ_MEM_RECLAIM, endpoint);
	if (!afkep->wq) {
		ret = -ENOMEM;
		goto out_free_afkep;
	}

	/* Compact EPIC replies to firmware calls go out from their own queue. */
	if (afk_compact_capable(afkep)) {
		afkep->tx_wq = alloc_ordered_workqueue("apple-dcp-afkep%02x-tx",
						       WQ_MEM_RECLAIM, endpoint);
		if (!afkep->tx_wq) {
			destroy_workqueue(afkep->wq);
			ret = -ENOMEM;
			goto out_free_afkep;
		}
	}
	INIT_WORK(&afkep->service_work, afk_service_worker);
	afk_ep_init_state(afkep);
	ret = devm_add_action_or_reset(dcp->dev, afk_release_context, afkep);
	if (ret)
		return ERR_PTR(ret);

	if (dcp->v27_sleep_transport && endpoint >= 0x20 && endpoint < 0x40)
		WRITE_ONCE(dcp->sleep_v27_ep[endpoint - 0x20], afkep);
	return afkep;

out_free_afkep:
	kfree(afkep);
	return ERR_PTR(ret);
}

void afk_quiesce(struct apple_dcp_afkep *afkep)
{
	unsigned long flags;
	u32 i;

	if (IS_ERR_OR_NULL(afkep))
		return;
	spin_lock_irqsave(&afkep->lock, flags);
	afkep->stopping = true;
	spin_unlock_irqrestore(&afkep->lock, flags);
	afk_cancel_commands(afkep);
	if (afkep->wq) {
		destroy_workqueue(afkep->wq);
		afkep->wq = NULL;
	}
	if (afkep->tx_wq) {
		destroy_workqueue(afkep->tx_wq);
		afkep->tx_wq = NULL;
	}
	kvfree(afkep->compact_rx_data);
	afkep->compact_rx_data = NULL;
	for (i = 0; i < afkep->num_channels; i++)
		if (afkep->services[i].pending_open)
			afk_discard_pending_open(&afkep->services[i]);
	debugfs_remove_recursive(afkep->debugfs_entry);
	afkep->debugfs_entry = NULL;
}

void afk_shutdown(struct apple_dcp_afkep *afkep)
{
	unsigned long flags;

	afk_send(afkep, FIELD_PREP(RBEP_TYPE, RBEP_SHUTDOWN));
	int ret;

	ret = wait_for_completion_timeout(&afkep->stopped, msecs_to_jiffies(1000));
	if (ret <= 0) {
		dev_err(afkep->dcp->dev, "Timed out shutting down AFK endpoint %02x", afkep->endpoint);
	}

	spin_lock_irqsave(&afkep->lock, flags);
	afkep->stopping = true;
	spin_unlock_irqrestore(&afkep->lock, flags);
	afk_quiesce(afkep);
	if (afkep->rx_scratch) {
		devm_kfree(afkep->dcp->dev, afkep->rx_scratch);
		afkep->rx_scratch = NULL;
	}
}

/*
 * A processor that Linux starts itself with 14.7 firmware (the T8122
 * dcpext): as in AsahiLinux/linux#607, give the firmware time to set up
 * the endpoint after START_ENDPOINT, and poll the mailbox while waiting,
 * because the mailbox can leave messages queued without another interrupt
 * after RTKit bootstrap.
 */
static int afk_start_polled(struct apple_dcp_afkep *ep)
{
	unsigned long timeout;
	int ret;

	usleep_range(10000, 11000);
	afk_send(ep, FIELD_PREP(RBEP_TYPE, RBEP_INIT));

	timeout = jiffies + msecs_to_jiffies(1000);
	do {
		ret = apple_rtkit_poll(ep->dcp->rtk);
		if (ret < 0)
			return ret;

		ret = wait_for_completion_timeout(&ep->started,
						  msecs_to_jiffies(10));
		if (ret > 0)
			return 0;
	} while (time_before(jiffies, timeout));

	/* Completed after the last wait, before the deadline check. */
	if (try_wait_for_completion(&ep->started))
		return 0;
	dev_err(ep->dcp->dev,
		"RTKit AFK endpoint %02x did not acknowledge init\n",
		ep->endpoint);
	return -ETIMEDOUT;
}

int afk_start(struct apple_dcp_afkep *ep)
{
	int ret;

	/* Allocate before firmware can announce services or issue commands. */
	if (ep->wire_v2 && !ep->command_tags) {
		ep->command_tags = kvcalloc(AFKV2_INTERFACES, sizeof(*ep->command_tags),
					    GFP_KERNEL);
		if (!ep->command_tags)
			return -ENOMEM;
	}

	reinit_completion(&ep->started);
	ret = apple_rtkit_start_ep(ep->dcp->rtk, ep->endpoint);
	if (ret)
		return ret;
	if (ep->dcp->fw_compat == DCP_FIRMWARE_V_14_7_TEMPLATE &&
	    ep->dcp->hw.fw_14_7_endpoints)
		return afk_start_polled(ep);
	afk_send(ep, FIELD_PREP(RBEP_TYPE, RBEP_INIT));

	ret = wait_for_completion_timeout(&ep->started, msecs_to_jiffies(1000));
	if (ret <= 0 && ep->dcp->hw.iomfb_method_profile ==
	    DCP_IOMFB_METHODS_H17P_EXTERNAL)
		dev_warn(ep->dcp->dev,
			 "external AFK startup ep=%#x tx_ready=%u rx_ready=%u bytes=%zu\n",
			 ep->endpoint, ep->txbfr.ready, ep->rxbfr.ready, ep->bfr_size);
	if (ret <= 0)
		return -ETIMEDOUT;
	else
		return 0;
}

static void afk_getbuf(struct apple_dcp_afkep *ep, u64 message)
{
	u32 size = FIELD_GET(GETBUF_SIZE, message) << BLOCK_SHIFT;
	u16 tag = FIELD_GET(GETBUF_TAG, message);
	u64 reply;

	trace_afk_getbuf(ep, size, tag);

	if (ep->bfr) {
		dev_err(ep->dcp->dev,
			"Got GETBUF message but buffer already exists\n");
		return;
	}

	/* An unstopped adopted session owns this ring until the machine resets. */
	if (ep->dcp->afk_alloc) {
		ep->bfr = ep->dcp->afk_alloc(ep->dcp, size, &ep->bfr_dma);
		if (IS_ERR(ep->bfr))
			ep->bfr = NULL;
	} else if (READ_ONCE(ep->dcp->retain_dma)) {
		ep->bfr = dma_alloc_coherent(ep->dcp->dev, size, &ep->bfr_dma, GFP_KERNEL);
	} else {
		ep->bfr = dmam_alloc_coherent(ep->dcp->dev, size, &ep->bfr_dma, GFP_KERNEL);
	}
	if (!ep->bfr) {
		dev_err(ep->dcp->dev, "Failed to allocate %d bytes buffer\n",
			size);
		return;
	}

	ep->bfr_size = size;
	ep->bfr_tag = tag;

	reply = FIELD_PREP(RBEP_TYPE, RBEP_GETBUF_ACK);
	reply |= FIELD_PREP(GETBUF_ACK_DVA, ep->bfr_dma);
	afk_send(ep, reply);
}

static void afk_init_rxtx(struct apple_dcp_afkep *ep, u64 message,
			  struct afk_ringbuffer *bfr)
{
	u32 base = FIELD_GET(INITRB_OFFSET, message) << BLOCK_SHIFT;
	u32 size = FIELD_GET(INITRB_SIZE, message) << BLOCK_SHIFT;
	u16 tag = FIELD_GET(INITRB_TAG, message);
	u32 bufsz, end, stride;
	u8 *hdr;

	if (tag != ep->bfr_tag) {
		dev_err(ep->dcp->dev, "AFK[ep:%02x]: expected tag 0x%x but got 0x%x\n",
			ep->endpoint, ep->bfr_tag, tag);
		return;
	}

	if (bfr->ready) {
		dev_err(ep->dcp->dev, "AFK[ep:%02x]: buffer is already initialized\n",
			ep->endpoint);
		return;
	}

	if (!ep->bfr || base >= ep->bfr_size || size < sizeof(u32)) {
		dev_err(ep->dcp->dev,
			"AFK[ep:%02x]: invalid ring base 0x%x or size 0x%x\n",
			ep->endpoint, base, size);
		return;
	}

	end = base + size;
	if (end > ep->bfr_size) {
		dev_err(ep->dcp->dev,
			"AFK[ep:%02x]: requested end 0x%x > max size 0x%lx\n",
			ep->endpoint, end, ep->bfr_size);
		return;
	}

	hdr = (u8 *)ep->bfr + base;
	dma_rmb();
	bufsz = le32_to_cpu(READ_ONCE(*(__le32 *)hdr));
	if (afk_ring_stride(size, bufsz, &stride)) {
		dev_err(ep->dcp->dev,
			"AFK[ep:%02x]: invalid ring payload size 0x%x of 0x%x\n",
			ep->endpoint, bufsz, size);
		return;
	}
	if (bufsz < BIT(BLOCK_SHIFT)) {
		dev_err(ep->dcp->dev, "AFK[ep:%02x]: ring cannot hold an entry\n",
			ep->endpoint);
		return;
	}
	if (bfr == &ep->rxbfr) {
		/* Receive dispatch cannot recurse on the ordered endpoint queue. */
		ep->rx_scratch = devm_kmalloc(ep->dcp->dev, bufsz, GFP_KERNEL);
		if (!ep->rx_scratch)
			return;
	}

	bfr->rptr = (__le32 *)(hdr + stride);
	bfr->wptr = (__le32 *)(hdr + 2 * stride);
	bfr->buf = hdr + 3 * stride;
	bfr->bufsz = bufsz;
	bfr->stride = stride;
	bfr->ready = true;

	if (ep->rxbfr.ready && ep->txbfr.ready)
		afk_send(ep, FIELD_PREP(RBEP_TYPE, RBEP_START));
}

#if IS_ENABLED(CONFIG_DRM_APPLE_DEBUG)
static void afk_populate_service_debugfs(struct apple_epic_service *srv);
static void afk_remove_service_debugfs(struct apple_epic_service *srv);
#else
static void afk_populate_service_debugfs(struct apple_epic_service *srv)
{
}
static void afk_remove_service_debugfs(struct apple_epic_service *srv)
{
}
#endif

static const struct apple_epic_service_ops *
afk_match_service(struct apple_dcp_afkep *ep, const char *name)
{
	const struct apple_epic_service_ops *ops;

	if (!name || !name[0])
		return NULL;
	if (!ep->ops)
		return NULL;

	for (ops = ep->ops; ops->name[0]; ops++) {
		if (strcmp(ops->name, name))
			continue;

		return ops;
	}

	return NULL;
}

static struct apple_epic_service *afk_epic_find_service(struct apple_dcp_afkep *ep,
						     u32 channel)
{
	struct apple_epic_service *service;
	u32 i;

	for (i = 0; i < ep->num_channels; i++) {
		service = &ep->services[i];
		if (afk_service_matches(service, channel))
			return service;
	}

	return NULL;
}

static bool afk_epic_channel_retired(struct apple_dcp_afkep *ep, u32 channel)
{
	u32 i;

	for (i = 0; i < ep->num_channels; i++)
		if (afk_service_retired(&ep->services[i], channel))
			return true;

	return false;
}

/*
 * Compact EPIC, as the desktop 26.x firmware (T6041, M4 Pro/Max) speaks it
 * on its system and auxiliary endpoints.  A message is a transport header
 * (sequence, flags, interface, total length) followed by the timestamp, a
 * sub-header and the payload; messages longer than a fragment arrive in
 * fragments that repeat the header.  Services announce themselves with
 * type 0x11 and are opened with type 0x12; commands, replies and calls
 * carry a status/length descriptor whose status the reply echoes.
 *
 * Only DCP_FIRMWARE_V_26_6_DESKTOP endpoints switch to it, on the first
 * compact message they receive.
 */
struct afk_compact_cmd {
	struct list_head node;
	struct completion done;
	u16 channel;
	u8 type;
	bool service_call;
	u16 group;
	u32 command;
	u32 request_status;
	void *output;
	size_t output_len;
	size_t copied;
	u32 retcode;
	int status;
	/* timed out, kept until its late reply, see compact_keep_orphans */
	bool orphan;
};

static bool afk_compact_capable(struct apple_dcp_afkep *ep)
{
	/*
	 * An adopted J713 external DCP runs compact EPIC on its auxiliary
	 * endpoints too, and the J700 (H17P) and firmware 27 external
	 * processors on their audio endpoint, as the desktop 26.x one does.
	 */
	return ep->dcp->fw_compat == DCP_FIRMWARE_V_26_6_DESKTOP ||
	       (ep->dcp->fw_compat == DCP_FIRMWARE_M4 && ep->dcp->afk_alloc) ||
	       ((ep->dcp->hw.iomfb_method_profile == DCP_IOMFB_METHODS_H17P_EXTERNAL ||
		 dcp_is_v27_external(ep->dcp)) &&
		ep->endpoint == AV_AUDIO_ENDPOINT_DESKTOP);
}

bool afk_compact_av_capable(struct apple_dcp *dcp)
{
	return (dcp->fw_compat == DCP_FIRMWARE_V_26_6_DESKTOP &&
		dcp->desktop_external) ||
	       dcp->hw.iomfb_method_profile == DCP_IOMFB_METHODS_H17P_EXTERNAL ||
	       dcp_is_v27_external(dcp);
}

static u32 afk_compact_next_status(struct apple_epic_service *service)
{
	unsigned long flags;
	u32 status;

	spin_lock_irqsave(&service->lock, flags);
	status = (u32)service->cmd_tag++ << 8;
	spin_unlock_irqrestore(&service->lock, flags);

	return status;
}

/* The caller holds ep->lock. */
static void afk_fail_commands_locked(struct apple_dcp_afkep *ep, int error)
{
	struct afk_compact_cmd *cmd, *tmp;

	list_for_each_entry_safe(cmd, tmp, &ep->compact_cmds, node) {
		list_del_init(&cmd->node);
		/* Nobody waits for an orphan: it goes. */
		if (cmd->orphan) {
			WRITE_ONCE(ep->compact_orphans, ep->compact_orphans - 1);
			kfree(cmd);
			continue;
		}
		cmd->status = error;
		complete(&cmd->done);
	}
	wake_up_all(&ep->compact_orphan_wq);
}

/*
 * Wait up to @timeout for the late replies of every compact command
 * that timed out on @ep (see compact_keep_orphans). False if some are still
 * unanswered.
 */
bool afk_compact_orphans_wait(struct apple_dcp_afkep *ep, unsigned long timeout)
{
	return wait_event_timeout(ep->compact_orphan_wq, !READ_ONCE(ep->compact_orphans),
				  timeout);
}

/* Give up on the late replies of @ep's timed-out compact commands. */
unsigned int afk_compact_orphans_drop(struct apple_dcp_afkep *ep)
{
	struct afk_compact_cmd *cmd, *tmp;
	unsigned int dropped = 0;
	unsigned long flags;

	spin_lock_irqsave(&ep->lock, flags);
	list_for_each_entry_safe(cmd, tmp, &ep->compact_cmds, node) {
		if (!cmd->orphan)
			continue;
		list_del_init(&cmd->node);
		WRITE_ONCE(ep->compact_orphans, ep->compact_orphans - 1);
		kfree(cmd);
		dropped++;
	}
	spin_unlock_irqrestore(&ep->lock, flags);
	wake_up_all(&ep->compact_orphan_wq);
	if (dropped)
		dev_warn(ep->dcp->dev,
			 "AFK[ep:%02x]: %u timed-out command(s) still unanswered, dropped\n",
			 ep->endpoint, dropped);
	return dropped;
}

/* Whether a timed-out compact command of @ep still waits for its reply. */
bool afk_compact_orphans_pending(struct apple_dcp_afkep *ep)
{
	return READ_ONCE(ep->compact_orphans);
}

void afk_cancel_commands(struct apple_dcp_afkep *ep)
{
	unsigned long flags;

	spin_lock_irqsave(&ep->lock, flags);
	ep->compact_shutting_down = true;
	afk_fail_commands_locked(ep, -ESHUTDOWN);
	spin_unlock_irqrestore(&ep->lock, flags);
}

/*
 * A message was sent only in part: the firmware waits for the rest, and a
 * new message header would be taken as its continuation.  Fail what is in
 * flight and refuse further sends; the endpoint stays unusable until it is
 * started again.
 */
static void afk_compact_tx_desync(struct apple_dcp_afkep *ep, int error)
{
	unsigned long flags;

	spin_lock_irqsave(&ep->lock, flags);
	if (!ep->compact_tx_desync)
		dev_err(ep->dcp->dev,
			"AFK[ep:%02x]: compact message sent in part (%d); endpoint unusable until restarted\n",
			ep->endpoint, error);
	ep->compact_tx_desync = true;
	afk_fail_commands_locked(ep, -EIO);
	spin_unlock_irqrestore(&ep->lock, flags);
}

static int afk_send_compact_fragment(struct apple_dcp_afkep *ep, u16 intf_id,
				     const void *body, size_t size, u32 total)
{
	struct epic_compact_hdr *chdr;
	struct afk_qe *qhdr;
	unsigned long flags;
	void *message;
	size_t message_size, advance;
	u32 rptr, wptr, old_wptr;
	int ret = 0;

	if (!ep->txbfr.ready || !ep->txbfr.stride)
		return -EIO;

	message_size = offsetof(struct epic_compact_hdr, timestamp) + size;
	message = kzalloc(message_size, GFP_KERNEL);
	if (!message)
		return -ENOMEM;

	chdr = message;
	chdr->intf_id = cpu_to_le16(intf_id);
	chdr->length = cpu_to_le32(total);
	memcpy(message + offsetof(struct epic_compact_hdr, timestamp), body, size);

	spin_lock_irqsave(&ep->lock, flags);
	if (ep->compact_shutting_down || ep->stopping) {
		ret = -ESHUTDOWN;
		goto out_unlock;
	}
	if (ep->compact_tx_desync) {
		ret = -EIO;
		goto out_unlock;
	}

	dma_rmb();
	rptr = le32_to_cpu(READ_ONCE(*ep->txbfr.rptr));
	wptr = le32_to_cpu(READ_ONCE(*ep->txbfr.wptr));
	old_wptr = wptr;
	advance = ALIGN(sizeof(*qhdr) + message_size, ep->txbfr.stride);

	/* Both pointers live in shared memory: never trust them unchecked. */
	if (!afk_ring_ptrs_valid(rptr, wptr, ep->txbfr.bufsz, ep->txbfr.stride) ||
	    ep->txbfr.bufsz - wptr < sizeof(*qhdr)) {
		dev_warn_ratelimited(ep->dcp->dev,
				     "AFK[ep:%02x]: invalid transmit ring pointers: rptr 0x%x wptr 0x%x\n",
				     ep->endpoint, rptr, wptr);
		ret = -EPROTO;
		goto out_unlock;
	}
	if (advance > ep->txbfr.bufsz) {
		ret = -EMSGSIZE;
		goto out_unlock;
	}
	if (wptr < rptr && advance >= rptr - wptr) {
		ret = -EAGAIN;
		goto out_unlock;
	}
	if (wptr >= rptr) {
		bool fits_above = advance < ep->txbfr.bufsz - wptr ||
			(advance == ep->txbfr.bufsz - wptr && rptr != 0);

		if (!fits_above && advance >= rptr) {
			ret = -EAGAIN;
			goto out_unlock;
		}
	}

	chdr->seq = ep->qe_seq++;
	qhdr = ep->txbfr.buf + wptr;
	qhdr->magic = cpu_to_le32(QE_MAGIC);
	qhdr->size = cpu_to_le32(message_size);
	qhdr->channel = cpu_to_le32(0);
	qhdr->type = cpu_to_le32(0);
	wptr += sizeof(*qhdr);

	/* No room for the payload: wrap, repeating the queue entry at 0. */
	if (message_size > ep->txbfr.bufsz - wptr) {
		struct afk_qe *qhdr2 = ep->txbfr.buf;

		memcpy(qhdr2, qhdr, sizeof(*qhdr));
		qhdr = qhdr2;
		wptr = sizeof(*qhdr);
	}

	memcpy(qhdr->data, message, message_size);
	wptr = ALIGN(wptr + message_size, ep->txbfr.stride);
	if (wptr == ep->txbfr.bufsz)
		wptr = 0;

	dma_wmb();
	WRITE_ONCE(*ep->txbfr.wptr, cpu_to_le32(wptr));
	dma_wmb();
	/* The firmware's own client rings with RECV and both pointers. */
	afk_send(ep, FIELD_PREP(RBEP_TYPE, RBEP_RECV) |
		     ((u64)(rptr & 0xffff) << 32) |
		     ((u64)(wptr & 0xffff) << 16) | (old_wptr & 0xffff));

out_unlock:
	spin_unlock_irqrestore(&ep->lock, flags);
	kfree(message);
	return ret;
}

/*
 * Send one complete message.  Fragments carry no offset or tag beyond the
 * repeated interface/length and the per-fragment sequence number, so the
 * fragments of one message must not interleave with another's.
 */
static int afk_send_compact(struct apple_dcp_afkep *ep, u16 intf_id, u8 type,
			    u8 category, const void *payload, size_t payload_size,
			    size_t wire_size)
{
	struct epic_compact_sub_hdr *sub;
	size_t total, offset = 0, chunk;
	unsigned long deadline;
	bool waiting_for_space = false;
	void *body;
	int ret = 0;

	if (payload_size > wire_size || wire_size > AFK_COMPACT_MESSAGE_MAX - 16)
		return -EMSGSIZE;
	total = 8 + sizeof(*sub) + wire_size;
	body = kvzalloc(total, GFP_KERNEL);
	if (!body)
		return -ENOMEM;
	sub = body + 8; /* the timestamp precedes the sub-header */
	sub->type = type;
	sub->category = category;
	sub->unk = type == 0x12;
	if (payload_size)
		memcpy(sub + 1, payload, payload_size);

	mutex_lock(&ep->compact_tx_lock);
	deadline = jiffies + msecs_to_jiffies(5000);
	while (offset < total) {
		chunk = min_t(size_t, total - offset, AFK_COMPACT_FRAGMENT_SIZE);
		ret = afk_send_compact_fragment(ep, intf_id, body + offset,
						chunk, total);
		if (ret == -EAGAIN) {
			if (!waiting_for_space) {
				afk_send(ep, FIELD_PREP(RBEP_TYPE, RBEP_WAIT_FOR_SPACE) |
					 (chunk + offsetof(struct epic_compact_hdr, timestamp)));
				waiting_for_space = true;
			}
			if (time_after_eq(jiffies, deadline)) {
				ret = -ETIMEDOUT;
				break;
			}
			/* The firmware advances the shared consumer pointer itself. */
			usleep_range(500, 1000);
			continue;
		}
		if (ret)
			break;
		waiting_for_space = false;
		offset += chunk;
	}
	/* Never start a new header into an unfinished remote assembly. */
	if (ret && offset)
		afk_compact_tx_desync(ep, ret);
	mutex_unlock(&ep->compact_tx_lock);
	kvfree(body);
	return ret;
}

int afk_start_service(struct apple_epic_service *service)
{
	int ret;

	if (!service || !service->enabled)
		return -EINVAL;
	if (!READ_ONCE(service->ep->compact) || service->started)
		return 0;

	ret = afk_send_compact(service->ep, service->channel, 0x12, 0,
			       NULL, 0, 0);
	if (!ret)
		service->started = true;
	return ret;
}

/* "prefix:name[:suffix]": match on name, cut or not; "*" takes every other service. */
static const struct apple_epic_service_ops *
afk_compact_match_service(struct apple_dcp_afkep *ep, const char *name)
{
	const struct apple_epic_service_ops *ops, *any = NULL;
	const char *service_name;
	size_t len;

	if (!name[0] || !ep->ops)
		return NULL;

	service_name = strchr(name, ':');
	if (service_name)
		service_name++;
	else
		service_name = name;

	for (ops = ep->ops; ops->name[0]; ops++) {
		if (!strcmp(ops->name, "*")) {
			any = any ?: ops;
			continue;
		}

		len = strlen(ops->name);
		if (strncmp(ops->name, service_name, len))
			continue;
		if (service_name[len] && service_name[len] != ':')
			continue;

		return ops;
	}

	/* An announcement that filled its field may have lost its end. */
	if (strlen(name) < AFK_COMPACT_NAME_LEN - 1)
		return any;
	for (ops = ep->ops; ops->name[0]; ops++)
		if (afk_compact_cut_of(ops, service_name))
			return ops;

	return any;
}

/*
 * Whether @service_name, the part after the role of an announcement that
 * filled its field, is the firmware's cut of @ops's long name.
 */
bool afk_compact_cut_of(const struct apple_epic_service_ops *ops, const char *service_name)
{
	size_t len = strlen(service_name);

	return ops->compact_full && len >= AFK_COMPACT_CUT_MIN &&
	       len < strlen(ops->compact_full) &&
	       !strncmp(ops->compact_full, service_name, len);
}

/* A free slot for a newly announced service, or the one it replaces. */
static struct apple_epic_service *
afk_compact_slot(struct apple_dcp_afkep *ep,
		 const struct apple_epic_service_ops *ops,
		 const char *name, s64 unit)
{
	u32 gen = READ_ONCE(ep->dcp->afk_power_gen);
	struct apple_epic_service *service;
	unsigned long flags;
	bool hotplug, free;
	u32 i;

	/*
	 * The firmware announces every compact service again under a fresh
	 * interface after a DCP poweron, on every endpoint, without closing
	 * the previous one. The DPDEV, DPAVSERV and AV endpoints (0x27-0x29)
	 * also do so after an external hotplug without a poweron. Replace an
	 * enabled service with the same binding, name and unit in place when
	 * it comes from an earlier poweron, from any announcement on the
	 * hotplug endpoints, or when the service is a singleton (one that is
	 * re-announced on every DP relink and of which only one is ever live,
	 * such as the external DCP's AV interfaces on AV_AUDIO_ENDPOINT_DESKTOP
	 * (0x2b), which a relink across suspend would otherwise pile up until
	 * the endpoint has no slot left). Services announced since the latest
	 * poweron stay: DPAVCTRL announces several services with the same name
	 * and no unit in one generation, and those are siblings, not
	 * replacements.
	 */
	hotplug = ep->endpoint >= 0x27 && ep->endpoint <= 0x29;
	for (i = 0; i < ep->num_channels; i++) {
		service = &ep->services[i];
		if (!service->enabled || service->ops != ops ||
		    service->unit != unit || strcmp(service->name, name))
			continue;
		if (!hotplug && !ops->singleton && service->power_gen == gen)
			continue;
		dev_info(ep->dcp->dev,
			 "AFK[ep:%02x]: replacing compact service %s interface %u\n",
			 ep->endpoint, name, service->channel);
		afk_remove_service_debugfs(service);
		afk_service_retire(service);
		if (service->ops->teardown)
			service->ops->teardown(service);
		service->power_gen = gen;
		return service;
	}

	for (i = 0; i < ep->num_channels; i++) {
		service = &ep->services[i];
		spin_lock_irqsave(&service->lock, flags);
		free = !service->enabled && !service->users &&
		       bitmap_empty(service->cmd_map, MAX_PENDING_CMDS);
		spin_unlock_irqrestore(&service->lock, flags);
		if (free) {
			service->power_gen = gen;
			return service;
		}
	}

	if (ep->num_channels >= AFK_MAX_CHANNEL)
		return NULL;
	service = &ep->services[ep->num_channels++];
	service->power_gen = gen;
	return service;
}

static void afk_recv_handle_compact_init(struct apple_dcp_afkep *ep,
					 u16 channel, const void *payload,
					 size_t payload_size)
{
	char wire_name[33], service_name[33];
	const struct apple_epic_service_ops *ops;
	struct apple_epic_service *service;
	unsigned long flags;
	const char *name;
	char *unit_suffix;
	s64 unit = -1;
	int ret;

	if (payload_size < 32) {
		dev_err(ep->dcp->dev,
			"AFK[ep:%02x]: short compact service announcement\n",
			ep->endpoint);
		return;
	}
	if (afk_epic_find_service(ep, channel)) {
		dev_warn(ep->dcp->dev,
			 "AFK[ep:%02x]: duplicate compact service on interface %u\n",
			 ep->endpoint, channel);
		return;
	}
	memcpy(wire_name, payload, AFK_COMPACT_NAME_LEN);
	wire_name[AFK_COMPACT_NAME_LEN] = '\0';
	name = strchr(wire_name, ':');
	name = name ? name + 1 : wire_name;
	strscpy(service_name, name, sizeof(service_name));

	unit_suffix = strrchr(service_name, ':');
	if (unit_suffix && !kstrtos64(unit_suffix + 1, 10, &unit))
		*unit_suffix = '\0';
	else
		unit = -1;

	ops = afk_compact_match_service(ep, wire_name);
	if (!ops) {
		dev_err(ep->dcp->dev,
			"AFK[ep:%02x]: unable to match compact service %s on interface %u\n",
			ep->endpoint, wire_name, channel);
		return;
	}

	service = afk_compact_slot(ep, ops, service_name, unit);
	if (!service) {
		dev_err(ep->dcp->dev,
			"AFK[ep:%02x]: too many enabled compact services\n",
			ep->endpoint);
		return;
	}

	spin_lock_irqsave(&service->lock, flags);
	service->ops = ops;
	service->ep = ep;
	service->channel = channel;
	service->cookie = NULL;
	service->cmd_tag = 0;
	strscpy(service->name, service_name, sizeof(service->name));
	service->unit = unit;
	service->defer_start = false;
	service->started = false;
	service->torndown = false;
	/* A service with an admission step is published once admitted. */
	service->enabled = !ops->prepare;
	spin_unlock_irqrestore(&service->lock, flags);

	if (ops->prepare) {
		ret = ops->prepare(service, service_name, service_name, unit);
		if (ret) {
			dev_info(ep->dcp->dev,
				 "AFK[ep:%02x]: compact service %s on interface %u not admitted: %d\n",
				 ep->endpoint, wire_name, channel, ret);
			afk_service_discard(service);
			return;
		}
		afk_service_publish(service);
	}

	if (ops->init)
		ops->init(service, service_name, service_name, unit);

	dev_info(ep->dcp->dev,
		 "AFK[ep:%02x]: new compact service %s on interface %u%s\n",
		 ep->endpoint, wire_name, channel,
		 service->defer_start ? " (deferred)" : "");
	afk_populate_service_debugfs(service);

	if (!service->defer_start) {
		ret = afk_start_service(service);
		if (ret)
			dev_err(ep->dcp->dev,
				"AFK[ep:%02x]: failed to start compact service %u: %d\n",
				ep->endpoint, channel, ret);
	}
}

/* IOReturn values of a refused call (IOKit's sys_iokit error space) */
#define AFK_IORETURN_NO_DEVICE	0xe00002c0
#define AFK_IORETURN_OFFLINE	0xe00002d7
#define AFK_IORETURN_NOT_READY	0xe00002d8

/*
 * An interface that is offline or not there (yet) is -ENODEV, as a call to
 * an interface already gone is; anything else the firmware refused, -EREMOTEIO.
 */
static int afk_compact_refusal_errno(u32 ioreturn)
{
	switch (ioreturn) {
	case AFK_IORETURN_NO_DEVICE:
	case AFK_IORETURN_OFFLINE:
	case AFK_IORETURN_NOT_READY:
		return -ENODEV;
	default:
		return -EREMOTEIO;
	}
}

static void afk_recv_handle_compact_reply(struct apple_dcp_afkep *ep,
					  u16 channel, u8 type,
					  const void *payload,
					  size_t payload_size)
{
	const struct epic_compact_desc *desc = payload;
	const struct epic_service_call *call = NULL;
	struct afk_compact_cmd *pending, *match = NULL;
	unsigned long flags;
	size_t copy_size;
	bool refused;

	if (payload_size < sizeof(*desc)) {
		dev_err(ep->dcp->dev,
			"AFK[ep:%02x]: compact reply is missing its descriptor\n",
			ep->endpoint);
		return;
	}

	if (payload_size >= sizeof(*desc) + sizeof(*call)) {
		call = (const void *)(desc + 1);
		if (le32_to_cpu(call->magic) != EPIC_SERVICE_CALL_MAGIC)
			call = NULL;
	}

	/*
	 * A call the firmware refuses is answered with its descriptor alone:
	 * the request's status, and an IOReturn where the length goes (an
	 * audio interface the firmware is replacing answers an Open with
	 * kIOReturnOffline). It carries no call header, so it matches by its
	 * status only.
	 */
	refused = payload_size == sizeof(*desc);

	spin_lock_irqsave(&ep->lock, flags);
	list_for_each_entry(pending, &ep->compact_cmds, node) {
		if (pending->channel != channel || pending->type != type)
			continue;
		if (pending->request_status != le32_to_cpu(desc->status))
			continue;
		if (pending->service_call && !refused &&
		    (!call || pending->group != le16_to_cpu(call->group) ||
		     pending->command != le32_to_cpu(call->command)))
			continue;
		match = pending;
		break;
	}
	if (!match) {
		spin_unlock_irqrestore(&ep->lock, flags);
		dev_warn(ep->dcp->dev,
			 "AFK[ep:%02x]: unexpected compact reply %02x on interface %u\n",
			 ep->endpoint, type, channel);
		return;
	}

	list_del_init(&match->node);
	/* The late reply of a timed-out command is its orphan's. */
	if (match->orphan) {
		WRITE_ONCE(ep->compact_orphans, ep->compact_orphans - 1);
		spin_unlock_irqrestore(&ep->lock, flags);
		dev_dbg(ep->dcp->dev,
			"AFK[ep:%02x]: late reply to timed-out call group %u cmd %u on interface %u: status %#x\n",
			ep->endpoint, match->group, match->command, channel,
			le32_to_cpu(desc->status));
		wake_up_all(&ep->compact_orphan_wq);
		kfree(match);
		return;
	}
	if (refused && match->service_call) {
		u32 ioreturn = le32_to_cpu(desc->length);

		match->retcode = ioreturn;
		match->status = afk_compact_refusal_errno(ioreturn);
		complete(&match->done);
		spin_unlock_irqrestore(&ep->lock, flags);
		dev_dbg(ep->dcp->dev,
			"AFK[ep:%02x]: call group %u cmd %u on interface %u refused: %#x\n",
			ep->endpoint, match->group, match->command, channel, ioreturn);
		return;
	}
	match->retcode = le32_to_cpu(desc->status);
	match->status = 0;
	if (call && le32_to_cpu(call->data_len) >
	    payload_size - sizeof(*desc) - sizeof(*call)) {
		match->status = -EMSGSIZE;
	} else {
		copy_size = min(match->output_len, payload_size - sizeof(*desc));
		if (copy_size && match->output)
			memcpy(match->output, desc + 1, copy_size);
		match->copied = copy_size;
	}
	complete(&match->done);
	spin_unlock_irqrestore(&ep->lock, flags);
}

/*
 * A reply to a firmware call, sent from the endpoint's transmit queue: a
 * send may wait for ring space, which the receive worker frees by draining
 * the other direction.
 */
struct afk_compact_reply {
	struct work_struct work;
	struct apple_dcp_afkep *ep;
	u16 channel;
	size_t size;
	u8 data[];
};

static void afk_compact_reply_worker(struct work_struct *work)
{
	struct afk_compact_reply *r = container_of(work, struct afk_compact_reply, work);
	int ret;

	ret = afk_send_compact(r->ep, r->channel, EPIC_SUBTYPE_STD_SERVICE, 2,
			       r->data, r->size, r->size);
	if (ret)
		dev_err(r->ep->dcp->dev,
			"AFK[ep:%02x]: failed to reply to compact service call: %d\n",
			r->ep->endpoint, ret);
	if (ret && r->ep->dcp->v27_sleep_transport) {
		guard(spinlock_irqsave)(&r->ep->lock);
		r->ep->sleep_poisoned = true;
	}
	atomic_dec_return_release(&r->ep->sleep_jobs);
	kvfree(r);
}

static void afk_recv_handle_compact_call(struct apple_dcp_afkep *ep,
					 u16 channel, const void *payload,
					 size_t payload_size)
{
	const struct epic_compact_desc *request_desc = payload;
	const struct epic_service_call *call;
	struct epic_compact_desc *reply_desc;
	struct epic_service_call *reply_call;
	struct apple_epic_service *service;
	size_t inner_size, call_size;
	struct afk_compact_reply *r;
	u16 group;
	u32 command;
	void *reply;
	int ret;

	if (payload_size < sizeof(*request_desc) + sizeof(*call) || !ep->tx_wq)
		return;

	call = payload + sizeof(*request_desc);
	inner_size = payload_size - sizeof(*request_desc);
	call_size = le32_to_cpu(call->data_len);
	if (le32_to_cpu(call->magic) != EPIC_SERVICE_CALL_MAGIC ||
	    inner_size < sizeof(*call) + call_size) {
		dev_err(ep->dcp->dev,
			"AFK[ep:%02x]: malformed compact service call on interface %u\n",
			ep->endpoint, channel);
		return;
	}
	group = le16_to_cpu(call->group);
	command = le32_to_cpu(call->command);

	r = kvzalloc(struct_size(r, data, payload_size), GFP_KERNEL);
	if (!r)
		return;
	INIT_WORK(&r->work, afk_compact_reply_worker);
	r->ep = ep;
	r->channel = channel;
	r->size = payload_size;
	reply = r->data;
	reply_desc = reply;
	reply_desc->status = request_desc->status;
	reply_call = reply + sizeof(*reply_desc);
	memcpy(reply_call, call, sizeof(*call));

	service = afk_service_get(afk_epic_find_service(ep, channel));
	if (service && service->ops->call_v2)
		ret = service->ops->call_v2(service, group, command, call + 1,
					    call_size, reply_call + 1, call_size);
	else if (service && service->ops->call && !group)
		ret = service->ops->call(service, command, call + 1, call_size,
					 reply_call + 1, call_size);
	else
		ret = -EOPNOTSUPP;
	if (service)
		afk_service_put(service);
	if (ret == -EOPNOTSUPP)
		dev_warn(ep->dcp->dev,
			 "AFK[ep:%02x]: no handler for compact service %u call %u:%u\n",
			 ep->endpoint, channel, group, command);
	if (ret)
		reply_desc->status = cpu_to_le32(ret);

	atomic_inc(&ep->sleep_jobs);
	queue_work(ep->tx_wq, &r->work);
}

/* A serialized report starts at its serialization header, without a command descriptor. */
static bool afk_compact_report_dictionary(const void *payload, size_t size)
{
	return size >= 2 * sizeof(u32) && get_unaligned_le32(payload) == 0xd3 &&
	       ((get_unaligned_le32(payload + sizeof(u32)) >> 24) & 0x1f) == 1;
}

/*
 * A bare compact report starts with the service-call header. Its magic
 * is at the usual offset from the start of the payload.
 */
static bool afk_compact_report_service_call(const void *payload, size_t size)
{
	const struct epic_service_call *call = payload;

	return size >= sizeof(*call) &&
	       get_unaligned_le32(&call->magic) == EPIC_SERVICE_CALL_MAGIC;
}

static void afk_recv_compact_message(struct apple_dcp_afkep *ep, u16 intf_id,
				     const void *body, size_t size)
{
	const struct epic_compact_sub_hdr *sub = body + 8;
	struct apple_epic_service *service;
	const void *payload = sub + 1;
	size_t payload_size = size - 8 - sizeof(*sub);

	if (sub->category == 0 && sub->type == 0x11) {
		afk_recv_handle_compact_init(ep, intf_id, payload, payload_size);
		return;
	}
	if (sub->category == 1 && sub->type == EPIC_SUBTYPE_STD_SERVICE) {
		afk_recv_handle_compact_call(ep, intf_id, payload, payload_size);
		return;
	}
	if (sub->category == 2) {
		afk_recv_handle_compact_reply(ep, intf_id, sub->type, payload,
					      payload_size);
		return;
	}
	if (sub->category == 0) {
		/* Preserve bare dictionaries and EPIC call headers; others carry a descriptor. */
		if (sub->type == EPIC_SUBTYPE_STD_SERVICE &&
		    payload_size >= sizeof(struct epic_compact_desc) &&
		    !afk_compact_report_dictionary(payload, payload_size) &&
		    !afk_compact_report_service_call(payload, payload_size)) {
			payload += sizeof(struct epic_compact_desc);
			payload_size -= sizeof(struct epic_compact_desc);
		}
		service = afk_service_get(afk_epic_find_service(ep, intf_id));
		if (service && service->ops->report)
			service->ops->report(service, sub->type, payload,
					     payload_size);
		if (service)
			afk_service_put(service);
		return;
	}

	dev_dbg(ep->dcp->dev,
		"AFK[ep:%02x]: unhandled compact message %02x:%02x on interface %u\n",
		ep->endpoint, sub->category, sub->type, intf_id);
}

/* True if @data was a compact message (or fragment) and has been handled. */
static bool afk_recv_handle_compact(struct apple_dcp_afkep *ep, u32 channel,
				    u32 type, const void *data, size_t data_size)
{
	const struct epic_compact_hdr *hdr = data;
	const size_t header_size = offsetof(struct epic_compact_hdr, timestamp);
	struct afk_fragment_state *s = &ep->compact_rx;
	const struct epic_compact_sub_hdr *sub;
	u32 offset, total;
	u16 intf_id;
	int ret;

	if (!afk_compact_capable(ep) || channel || type ||
	    data_size < header_size)
		return false;
	total = le32_to_cpu(hdr->length);
	intf_id = le16_to_cpu(hdr->intf_id);
retry:
	offset = s->received;
	/* Eight-byte empty control messages occur in the firmware's traffic. */
	if (!offset && total == 8 && data_size == header_size + 8)
		return true;
	/* A message starts with its timestamp and a plausible sub-header. */
	if (!offset) {
		sub = data + header_size + 8;
		if (data_size < header_size + 8 + sizeof(*sub) || sub->category > 2) {
			if (!READ_ONCE(ep->compact))
				return false;
			ret = -EPROTO;
			goto discard;
		}
	}
	ret = afk_fragment_accept(s, intf_id, hdr->seq, total,
				  data_size - header_size);
	if (ret < 0) {
		if (!READ_ONCE(ep->compact) && !offset)
			return false;
		/*
		 * A fragment that does not continue the message in assembly
		 * may start the next one, e.g. after a firmware restart: drop
		 * the partial message and take this one as a fresh start.
		 */
		if (offset) {
			dev_err_ratelimited(ep->dcp->dev,
				"AFK[ep:%02x]: compact message on interface %u abandoned at %u of %u bytes: %d\n",
				ep->endpoint, s->intf_id, offset, s->total, ret);
			kvfree(ep->compact_rx_data);
			ep->compact_rx_data = NULL;
			memset(s, 0, sizeof(*s));
			goto retry;
		}
		goto discard;
	}
	WRITE_ONCE(ep->compact, true);
	if (!offset && ret == 1) {
		memset(s, 0, sizeof(*s));
		afk_recv_compact_message(ep, intf_id, data + header_size, total);
		return true;
	}
	if (!offset) {
		ep->compact_rx_data = kvzalloc(total, GFP_KERNEL);
		if (!ep->compact_rx_data) {
			ret = -ENOMEM;
			goto discard;
		}
	}
	memcpy(ep->compact_rx_data + offset, data + header_size,
	       data_size - header_size);
	if (ret == 1) {
		void *message = ep->compact_rx_data;

		ep->compact_rx_data = NULL;
		memset(s, 0, sizeof(*s));
		afk_recv_compact_message(ep, intf_id, message, total);
		kvfree(message);
	}
	return true;

discard:
	dev_err_ratelimited(ep->dcp->dev,
		"AFK[ep:%02x]: invalid compact fragment intf=%u seq=%u total=%u offset=%u: %d\n",
		ep->endpoint, intf_id, hdr->seq, total, offset, ret);
	kvfree(ep->compact_rx_data);
	ep->compact_rx_data = NULL;
	memset(s, 0, sizeof(*s));
	return true;
}

/*
 * A command on a compact endpoint: the payload behind a descriptor whose
 * status the reply echoes.  @retcode is the reply's status.
 */
static int afk_send_compact_command(struct apple_epic_service *service, u8 type,
				    const void *payload, size_t payload_len,
				    void *output, size_t output_len,
				    u32 request_status, u32 *retcode,
				    size_t *copied, unsigned long timeout)
{
	struct apple_dcp_afkep *ep = service->ep;
	const struct epic_service_call *call = payload;
	struct afk_compact_cmd pending, *orphan = NULL;
	struct epic_compact_desc *desc;
	size_t wire_size = max(payload_len, output_len);
	unsigned long waited, flags;
	void *wrapped;
	int ret;

	wrapped = kvzalloc(sizeof(*desc) + wire_size, GFP_KERNEL);
	if (!wrapped)
		return -ENOMEM;
	/* Ready to stand in for this command if it times out. */
	if (READ_ONCE(ep->compact_keep_orphans)) {
		orphan = kzalloc_obj(*orphan);
		/* Without its orphan a timeout could not be kept apart: not sent. */
		if (!orphan) {
			kvfree(wrapped);
			return -ENOMEM;
		}
	}
	desc = wrapped;
	desc->status = cpu_to_le32(request_status);
	desc->length = cpu_to_le32(wire_size);
	if (payload_len)
		memcpy(desc + 1, payload, payload_len);

	memset(&pending, 0, sizeof(pending));
	INIT_LIST_HEAD(&pending.node);
	init_completion(&pending.done);
	pending.channel = service->channel;
	pending.type = type;
	pending.request_status = request_status;
	pending.output = output;
	pending.output_len = output_len;
	pending.status = -EINPROGRESS;
	if (payload_len >= sizeof(*call) &&
	    le32_to_cpu(call->magic) == EPIC_SERVICE_CALL_MAGIC) {
		pending.service_call = true;
		pending.group = le16_to_cpu(call->group);
		pending.command = le32_to_cpu(call->command);
	}

	spin_lock_irqsave(&ep->lock, flags);
	if (ep->compact_shutting_down || ep->stopping) {
		spin_unlock_irqrestore(&ep->lock, flags);
		ret = -ESHUTDOWN;
		goto out;
	}
	list_add_tail(&pending.node, &ep->compact_cmds);
	spin_unlock_irqrestore(&ep->lock, flags);

	ret = afk_send_compact(ep, service->channel, type, 1, wrapped,
			       sizeof(*desc) + payload_len,
			       sizeof(*desc) + wire_size);
	if (ret)
		goto remove_pending;

	waited = wait_for_completion_timeout(&pending.done, timeout);
	spin_lock_irqsave(&ep->lock, flags);
	if (!waited && !list_empty(&pending.node)) {
		/*
		 * An orphan takes the command's place, so that its late
		 * reply never completes a later identical command.
		 */
		if (orphan) {
			INIT_LIST_HEAD(&orphan->node);
			init_completion(&orphan->done);
			orphan->channel = pending.channel;
			orphan->type = pending.type;
			orphan->service_call = pending.service_call;
			orphan->group = pending.group;
			orphan->command = pending.command;
			orphan->request_status = pending.request_status;
			orphan->orphan = true;
			list_replace_init(&pending.node, &orphan->node);
			WRITE_ONCE(ep->compact_orphans, ep->compact_orphans + 1);
			orphan = NULL;
		} else {
			list_del_init(&pending.node);
		}
		pending.status = -ETIMEDOUT;
	}
	ret = pending.status;
	if (retcode)
		*retcode = pending.retcode;
	if (copied)
		*copied = pending.copied;
	spin_unlock_irqrestore(&ep->lock, flags);
	goto out;

remove_pending:
	spin_lock_irqsave(&ep->lock, flags);
	if (!list_empty(&pending.node))
		list_del_init(&pending.node);
	spin_unlock_irqrestore(&ep->lock, flags);
out:
	kfree(orphan);
	kvfree(wrapped);
	return ret;
}

static int afk_send_epic_token(struct apple_dcp_afkep *ep, u32 channel, u16 tag,
			       enum epic_type etype, enum epic_category ecat, u8 stype,
			       const void *payload, size_t payload_len, u64 token);

/* Caller has closed host publication and settled ordinary commands first. */
int afk_v27_sleep_check(struct apple_dcp_afkep *ep)
{
	unsigned long flags;
	unsigned int i;
	int ret = 0;

	if (!ep || !ep->wire_v2 || !ep->txbfr.ready || !ep->rxbfr.ready ||
	    !ep->txbfr.buf || !ep->txbfr.rptr || !ep->txbfr.wptr ||
	    !ep->rxbfr.buf || !ep->rxbfr.rptr || !ep->rxbfr.wptr)
		return -ENODEV;
	spin_lock_irqsave(&ep->lock, flags);
	if (ep->stopping || ep->sleep_waiting || ep->sleep_poisoned ||
	    ep->compact_tx_desync || ep->compact_rx.received ||
	    atomic_read_acquire(&ep->sleep_jobs) || work_busy(&ep->service_work) ||
	    !list_empty(&ep->compact_cmds) || ep->compact_orphans)
		ret = -EBUSY;
	if (!ret && ep->sleep_closed) {
		dma_rmb();
		if (READ_ONCE(*ep->rxbfr.rptr) != READ_ONCE(*ep->rxbfr.wptr) ||
		    READ_ONCE(*ep->txbfr.rptr) != READ_ONCE(*ep->txbfr.wptr))
			ret = -EBUSY;
	}
	spin_unlock_irqrestore(&ep->lock, flags);
	/*
	 * An external processor's AV and DP interfaces are the firmware's to end: one that has
	 * not answered does not hold the sleep (afk_v27_sleep_unanswered() counts them).
	 */
	for (i = 0; !ret && !ep->v27_closed_teardown && i < ep->num_channels; i++) {
		struct apple_epic_service *srv = &ep->services[i];

		spin_lock_irqsave(&srv->lock, flags);
		if (!bitmap_empty(srv->cmd_map, MAX_PENDING_CMDS) ||
		    srv->pending_open)
			ret = -EBUSY;
		spin_unlock_irqrestore(&srv->lock, flags);
	}
	return ret;
}

/* The interfaces of @ep with a call not answered or an opening not finished. */
unsigned int afk_v27_sleep_unanswered(struct apple_dcp_afkep *ep)
{
	unsigned long flags;
	unsigned int i, n = 0;

	if (!ep)
		return 0;
	for (i = 0; i < ep->num_channels; i++) {
		struct apple_epic_service *srv = &ep->services[i];

		spin_lock_irqsave(&srv->lock, flags);
		if (!bitmap_empty(srv->cmd_map, MAX_PENDING_CMDS) || srv->pending_open)
			n++;
		spin_unlock_irqrestore(&srv->lock, flags);
	}
	return n;
}

int afk_v27_sleep_control(struct apple_dcp_afkep *ep, bool open)
{
	struct afk_qe *hdr;
	unsigned long flags;
	u32 rptr, wptr, next, old;
	u8 payload[12] = {};
	int ret;

	if (!IS_ENABLED(CONFIG_APPLE_SOC_SLEEP) ||
	    !ep || !ep->dcp->v27_sleep_transport ||
	    ep->dcp->fw_compat != DCP_FIRMWARE_V_27_0)
		return -EOPNOTSUPP;
	ret = afk_v27_sleep_check(ep);
	if (ret)
		return ret;
	spin_lock_irqsave(&ep->lock, flags);
	/* A callback reply may have failed since the inventory check. */
	if (ep->stopping || ep->sleep_poisoned || ep->sleep_waiting) {
		ret = -EBUSY;
		goto out;
	}
	if (ep->sleep_closed == !open) {
		ret = -EALREADY;
		goto out;
	}
	dma_rmb();
	rptr = le32_to_cpu(READ_ONCE(*ep->txbfr.rptr));
	old = wptr = le32_to_cpu(READ_ONCE(*ep->txbfr.wptr));
	if (!afk_ring_ptrs_valid(rptr, wptr, ep->txbfr.bufsz, ep->txbfr.stride) ||
	    ep->txbfr.bufsz > 0x10000 || ep->txbfr.stride < 32 ||
	    (ep->txbfr.stride & (ep->txbfr.stride - 1)) ||
	    ep->txbfr.bufsz % ep->txbfr.stride ||
	    ep->txbfr.bufsz - wptr < sizeof(*hdr)) {
		ret = -EPROTO;
		goto out;
	}
	/* Use the existing dummy-header wrap format and keep one slot empty. */
	next = ALIGN(wptr + sizeof(*hdr) + sizeof(payload), ep->txbfr.stride);
	if (next > ep->txbfr.bufsz)
		next = ep->txbfr.stride;
	else if (next == ep->txbfr.bufsz)
		next = 0;
	if ((wptr < rptr && next >= rptr) ||
	    (wptr >= rptr && next <= wptr && next >= rptr)) {
		ret = -EAGAIN;
		goto out;
	}
	hdr = ep->txbfr.buf + wptr;
	hdr->magic = cpu_to_le32(QE_MAGIC);
	hdr->size = cpu_to_le32(sizeof(payload));
	hdr->channel = 0;
	hdr->type = 0;
	if (sizeof(*hdr) + sizeof(payload) > ep->txbfr.bufsz - wptr) {
		memcpy(ep->txbfr.buf, hdr, sizeof(*hdr));
		hdr = ep->txbfr.buf;
	}
	payload[0] = ep->qe_seq++;
	put_unaligned_le32(4, payload + 4);
	put_unaligned_le32(open ? 4 : 2, payload + 8);
	memcpy(hdr->data, payload, sizeof(payload));
	reinit_completion(&ep->sleep_done);
	ep->sleep_expected = open ? 5 : 3;
	ep->sleep_waiting = true;
	dma_wmb();
	WRITE_ONCE(*ep->txbfr.wptr, cpu_to_le32(next));
	dma_wmb();
	/* Return the transport's send error; a published entry still poisons it. */
	ret = apple_rtkit_send_message(ep->dcp->rtk, ep->endpoint,
		FIELD_PREP(RBEP_TYPE, RBEP_RECV) | (u64)rptr << 32 |
		(u64)next << 16 | old, NULL, true);
	spin_unlock_irqrestore(&ep->lock, flags);
	if (!ret && !wait_for_completion_timeout(&ep->sleep_done,
						 msecs_to_jiffies(250)))
		ret = -ETIMEDOUT;
	spin_lock_irqsave(&ep->lock, flags);
	if (ret) {
		ep->sleep_poisoned = true;
		ep->sleep_waiting = false;
	}
 out:
	spin_unlock_irqrestore(&ep->lock, flags);
	return ret;
}

/* Pending identities are owned only by the ordered endpoint workqueue. */
static void afk_discard_pending_open(struct apple_epic_service *service)
{
	service->pending_open = false;
	kfree(service->pending_name);
	kfree(service->pending_class);
	service->pending_name = NULL;
	service->pending_class = NULL;
	afk_service_discard(service);
}

static void afk_retry_service_init(struct apple_dcp_afkep *ep)
{
	u32 i;

	for (i = 0; i < ep->num_channels; i++) {
		struct apple_epic_service *service = &ep->services[i];
		const struct apple_epic_service_ops *ops = service->ops;
		int ret;

		if (!service->pending_open)
			continue;
		if (dcp_is_ext26(ep->dcp) &&
		    (!READ_ONCE(ep->dcp->ext26_services_open) ||
		     (ep == ep->dcp->dptxep && !dptx_native_controllers_ready(ep->dcp))))
			continue;
		ret = ops->revalidate ? ops->revalidate(service, service->pending_name,
				      service->pending_class, service->pending_unit) : 0;
		if (ret) {
			afk_discard_pending_open(service);
			continue;
		}
		/* Old replies keep their original slot, tag and buffers until drained. */
		if (afk_epic_find_service(ep, service->channel))
			continue;
		ret = afk_send_epic(ep, service->channel, 0, EPIC_TYPE_NOTIFY,
				    EPIC_CAT_REPORT, AFKV2_OPEN, NULL, 0);
		if (ret) {
			dev_err(ep->dcp->dev, "AFKV2 interface %u deferred OPEN failed: %d\n",
				service->channel, ret);
			afk_discard_pending_open(service);
			continue;
		}
		service->pending_open = false;
		afk_service_publish(service);
		if (ops->init)
			ops->init(service, service->pending_name, service->pending_class,
				  service->pending_unit);
		if (service->enabled)
			afk_populate_service_debugfs(service);
		kfree(service->pending_name);
		kfree(service->pending_class);
		service->pending_name = NULL;
		service->pending_class = NULL;
	}
}

static void afk_service_worker(struct work_struct *work)
{
	struct apple_dcp_afkep *ep = container_of(work, struct apple_dcp_afkep, service_work);

	if (!READ_ONCE(ep->stopping))
		afk_retry_service_init(ep);
}

static void afk_recv_handle_teardown(struct apple_dcp_afkep *ep, u32 channel);

static void afk_recv_handle_init(struct apple_dcp_afkep *ep, u32 channel,
				 u8 *payload, size_t payload_size)
{
	char name[32];
	s64 epic_unit = -1;
	struct apple_epic_service *service = NULL, *previous;
	unsigned long flags;
	bool retired = false;
	u32 ch_idx;
	const char *service_name = name;
	const char *epic_name = NULL, *epic_class = NULL;
	const struct apple_epic_service_ops *ops;
	struct dcp_parse_ctx ctx;
	u8 *props;
	size_t props_size;

	previous = afk_epic_find_service(ep, channel);
	/*
	 * The 14.7 external role announces its AV service again on the same
	 * channel after a replug: that is a new instance, and the old one is
	 * gone with the sink it served.
	 */
	if (previous && dcp_is_v14_7_external(ep->dcp) && ep == ep->dcp->dcpavservep) {
		afk_recv_handle_teardown(ep, channel);
		previous = afk_epic_find_service(ep, channel);
	}
	if (previous) {
		spin_lock_irqsave(&previous->lock, flags);
		retired = previous->torndown;
		spin_unlock_irqrestore(&previous->lock, flags);
		if (!ep->wire_v2 || ep != ep->dcp->dcpavservep ||
		    !retired || !previous->ops->revalidate)
			return;
	}
	for (ch_idx = 0; ch_idx < ep->num_channels; ch_idx++) {
		struct apple_epic_service *pending = &ep->services[ch_idx];

		if (!pending->pending_open || pending->channel != channel)
			continue;
		if (!pending->ops->revalidate ||
		    !pending->ops->revalidate(pending, pending->pending_name,
					      pending->pending_class, pending->pending_unit))
			return;
		afk_discard_pending_open(pending);
	}

	if (payload_size < sizeof(name)) {
		dev_err(ep->dcp->dev, "AFK[ep:%02x]: payload too small: %lx\n",
			ep->endpoint, payload_size);
		return;
	}

	strscpy(name, payload, sizeof(name));
	props = payload + sizeof(name);
	props_size = payload_size - sizeof(name);

	/*
	 * in DCP firmware 13.2 DCP reports interface-name as name which starts
	 * with "dispext%d" using -1 s ID for "dcp". In the 12.3 firmware
	 * EPICProviderClass was used. If the init call has props parse them and
	 * use EPICProviderClass to match the service.
	 */
	if (props_size > 36) {
		int ret = parse(props, props_size, &ctx);
		if (ret) {
			dev_err(ep->dcp->dev,
				"AFK[ep:%02x]: Failed to parse service init props for %s\n",
				ep->endpoint, name);
			return;
		}
		ret = parse_epic_service_init(&ctx, &epic_name, &epic_class, &epic_unit,
					      (ep->dcp->fw_compat == DCP_FIRMWARE_H17P &&
					       !dcp_is_ext26(ep->dcp)) ||
					      ep->dcp->fw_compat == DCP_FIRMWARE_V_27_0,
					      ep->dcp->fw_compat == DCP_FIRMWARE_V_27_0);
		/* These native infrastructure reports have no EPIC identity. */
		if (dcp_is_ext26(ep->dcp) && ret == -ENOENT &&
		    (ep->endpoint == SYSTEM_ENDPOINT ||
		     ep->endpoint == DISP0_ENDPOINT)) {
			dev_dbg(ep->dcp->dev,
				 "native26 AFK ep=%#x interface=%u report=%s has no EPIC identity; ignored\n",
				 ep->endpoint, channel, name);
			return;
		}
		if (ret == -ENOENT &&
		    (ep->dcp->hw.iomfb_method_profile == DCP_IOMFB_METHODS_H17G ||
		     (ep->dcp->fw_compat == DCP_FIRMWARE_V_27_0 &&
		      ep->dcp->hw.firmware_compat == DCP_FIRMWARE_H17P) ||
		     ep->dcp->v27_j714s ||
		     (ep->dcp->fw_compat == DCP_FIRMWARE_V_27_0 && ep->dcp->dcpext) ||
		     (ep->dcp->v27_sleep_transport && !strcmp(name, "md-manager")))) {
			/*
			 * H17G and firmware-27 T8142 can omit EPIC identity
			 * for powerlog-service. J716c's sleep transport
			 * admits only md-manager by name here.
			 */
			epic_name = NULL;
			epic_class = NULL;
		} else if (ret) {
			dev_err(ep->dcp->dev,
				"AFK[ep:%02x]: failed to extract init props: %d\n",
				ep->endpoint, ret);
			return;
		} else {
			service_name = epic_class;
		}
	} else {
            service_name = name;
        }

	if (ep->match_epic_name)
		ops = afk_match_service(ep, epic_name);
	else
		ops = afk_match_service(ep, service_name);
	if (dcp_is_ext26(ep->dcp))
		dev_dbg(ep->dcp->dev,
			 "native26 AFK identity ep=%#x interface=%u name=%s class=%s unit=%lld\n",
			 ep->endpoint, channel, epic_name ?: name, epic_class ?: "", epic_unit);

	/* T6030 14.7: the DPAV controller endpoint binds whatever it announces. */
	if (!ops && ep->dcp->fw_compat == DCP_FIRMWARE_V_14_7 && ep->ops &&
	    !strcmp(ep->ops->name, "dcpav-controller-epic")) {
		dev_info(ep->dcp->dev,
			 "AFK[ep:%02x]: binding unmatched DPAV service %s on channel %d\n",
			 ep->endpoint, service_name, channel);
		ops = ep->ops;
	}

	if (!ops && dcp_is_ext26(ep->dcp) && ep->ops &&
	    !strcmp(ep->ops->name, "native26-observer"))
		ops = ep->ops;
	if (!ops) {
		dev_err(ep->dcp->dev,
			"AFK[ep:%02x]: unable to match service %s on channel %d\n",
			ep->endpoint, service_name, channel);
		goto free;
	}

	for (ch_idx = 0; ch_idx < AFK_MAX_CHANNEL; ch_idx++) {
		struct apple_epic_service *candidate = &ep->services[ch_idx];

		if (afk_service_reinit(candidate, ep, ops, channel)) {
			service = candidate;
			break;
		}
	}
	if (!service) {
		dev_err(ep->dcp->dev, "AFK[ep:%02x]: no reusable service slots\n",
			ep->endpoint);
		goto free;
	}
	ep->num_channels = max(ep->num_channels, ch_idx + 1);
	if (ops->prepare) {
		int ret = ops->prepare(service, epic_name, epic_class, epic_unit);

		if (ret) {
			if (ops->optional && ret == -EOPNOTSUPP)
				dev_info(ep->dcp->dev,
					 "AFK optional %s (%s) is unsupported on channel %u\n",
					 ops->name, service_name, channel);
			else
				dev_err(ep->dcp->dev,
					"AFK service %s admission failed: %d (ep=%#x interface=%u name=%s unit=%lld)\n",
					service_name, ret, ep->endpoint, channel, epic_name ?: name, epic_unit);
			afk_service_discard(service);
			goto free;
		}
	}
	if ((previous && retired && ops->revalidate) ||
	    (dcp_is_ext26(ep->dcp) &&
	     (!READ_ONCE(ep->dcp->ext26_services_open) ||
	      (ep == ep->dcp->dptxep && !dptx_native_controllers_ready(ep->dcp))))) {
		/* No reader may use a native interface before its deferred OPEN. */
		WRITE_ONCE(service->enabled, false);
		/* Preserve the admitted identity and its original provider ticket. */
		service->pending_open = true;
		service->pending_name = epic_name;
		service->pending_class = epic_class;
		service->pending_unit = epic_unit;
		afk_service_retry(ep);
		return;
	}
	if (ep->wire_v2 && !ops->open_later) {
		int ret = afk_send_epic(ep, channel, 0, EPIC_TYPE_NOTIFY,
					EPIC_CAT_REPORT, AFKV2_OPEN, NULL, 0);

		if (ret) {
			unsigned long flags;

			if (ops->prepare) {
				afk_service_discard(service);
			} else {
				spin_lock_irqsave(&service->lock, flags);
				service->enabled = false;
				service->torndown = true;
				spin_unlock_irqrestore(&service->lock, flags);
			}
			dev_err(ep->dcp->dev, "AFKV2 interface %u OPEN failed: %d\n",
				channel, ret);
			goto free;
		}
	}
	if (ops->prepare)
		afk_service_publish(service);
	if (ops->init_props)
		ops->init_props(service, epic_unit, props, props_size);
	if (ops->init)
		ops->init(service, epic_name, epic_class, epic_unit);
	dev_info(ep->dcp->dev, "AFK[ep:%02x]: new service %s on channel %d\n",
		 ep->endpoint, service_name, channel);

	afk_populate_service_debugfs(service);

free:
	kfree(epic_name);
	kfree(epic_class);
}

static void afk_recv_handle_teardown(struct apple_dcp_afkep *ep, u32 channel)
{
	struct apple_epic_service *service;
	const struct apple_epic_service_ops *ops;
	unsigned long flags;

	/* Accepted CLOSE/CLOSED supersedes unpublished recovery for this interface. */
	for (u32 i = 0; i < ep->num_channels; i++) {
		struct apple_epic_service *pending = &ep->services[i];

		if (pending->pending_open && pending->channel == channel)
			afk_discard_pending_open(pending);
	}

	service = afk_epic_find_service(ep, channel);
	if (!service) {
		if (afk_epic_channel_retired(ep, channel))
			dev_dbg(ep->dcp->dev, "AFK[ep:%02x]: teardown for retired channel %u\n",
				ep->endpoint, channel);
		else
			dev_warn(ep->dcp->dev, "AFK[ep:%02x]: teardown for unknown channel %u\n",
				 ep->endpoint, channel);
		return;
	}

	afk_remove_service_debugfs(service);

	spin_lock_irqsave(&service->lock, flags);
	if (service->torndown) {
		spin_unlock_irqrestore(&service->lock, flags);
		return;
	}
	/* Outstanding commands remain discoverable until their replies arrive. */
	service->torndown = true;
	service->enabled = false;
	ops = service->ops;
	spin_unlock_irqrestore(&service->lock, flags);

	if (ops->teardown)
		ops->teardown(service);
}

static void afk_recv_handle_reply(struct apple_dcp_afkep *ep, u32 channel,
				  u16 tag, void *payload, size_t payload_size)
{
	struct epic_cmd *cmd = payload;
	struct apple_epic_service *service;
	unsigned long flags;
	u8 idx = tag & (ep->wire_v2 ? 0xf : 0xff);
	void *rxbuf, *txbuf;
	dma_addr_t rxbuf_dma, txbuf_dma;
	size_t rxlen, txlen;

	service = afk_epic_find_service(ep, channel);
	if (!service) {
		dev_warn(ep->dcp->dev, "AFK[ep:%02x]: command reply on disabled channel %u\n",
			 ep->endpoint, channel);
		return;
	}

	if (payload_size < sizeof(*cmd)) {
		dev_err(ep->dcp->dev,
			"AFK[ep:%02x]: command reply on channel %d too small: %ld\n",
			ep->endpoint, channel, payload_size);
		return;
	}

	if (idx >= MAX_PENDING_CMDS) {
		dev_err(ep->dcp->dev,
			"AFK[ep:%02x]: command reply on channel %d out of range: %d\n",
			ep->endpoint, channel, idx);
		return;
	}

	spin_lock_irqsave(&service->lock, flags);
	if (!test_bit(idx, service->cmd_map) || service->cmds[idx].done) {
		dev_err(ep->dcp->dev,
			"AFK[ep:%02x]: command reply on channel %d already handled\n",
			ep->endpoint, channel);
		spin_unlock_irqrestore(&service->lock, flags);
		return;
	}

	if (tag != service->cmds[idx].tag) {
		dev_err(ep->dcp->dev,
			"AFK[ep:%02x]: command reply on channel %d has invalid tag: expected 0x%04x != 0x%04x\n",
			ep->endpoint, channel, tag, service->cmds[idx].tag);
		spin_unlock_irqrestore(&service->lock, flags);
		return;
	}

	service->cmds[idx].done = true;
	service->cmds[idx].retcode = le32_to_cpu(cmd->retcode);
	service->cmds[idx].reply_len = le32_to_cpu(cmd->rxlen);
	service->cmds[idx].reply_len_valid = true;
	if (service->cmds[idx].free_on_ack) {
		/* defer freeing until we're no longer in atomic context */
		rxbuf = service->cmds[idx].rxbuf;
		txbuf = service->cmds[idx].txbuf;
		rxlen = service->cmds[idx].rxlen;
		txlen = service->cmds[idx].txlen;
		rxbuf_dma = service->cmds[idx].rxbuf_dma;
		txbuf_dma = service->cmds[idx].txbuf_dma;
		bitmap_release_region(service->cmd_map, idx, 0);
	} else {
		rxbuf = txbuf = NULL;
		rxlen = txlen = 0;
	}
	if (service->cmds[idx].completion)
		complete(service->cmds[idx].completion);

	spin_unlock_irqrestore(&service->lock, flags);

	if (rxbuf && rxlen)
		dma_free_coherent(ep->dcp->dev, rxlen, rxbuf, rxbuf_dma);
	if (txbuf && txlen)
		dma_free_coherent(ep->dcp->dev, txlen, txbuf, txbuf_dma);
}

/* A call from DCP: one word ahead of the call header AP sends. */
struct epic_std_service_ap_call {
	__le32 unk0;
	struct epic_service_call call;
} __attribute__((packed));
static_assert(sizeof(struct epic_std_service_ap_call) == 68);

static void afk_recv_handle_std_service(struct apple_dcp_afkep *ep, u32 channel,
					u32 type, struct epic_hdr *ehdr,
					struct epic_sub_hdr *eshdr,
					void *payload, size_t payload_size)
{
	struct apple_epic_service *service = afk_epic_find_service(ep, channel);

	if (!service) {
		dev_warn(ep->dcp->dev,
			 "AFK[ep:%02x]: std service notify on disabled channel %u\n",
			 ep->endpoint, channel);
		return;
	}
	/*
	 * 14.7 can return only an IOReturn, including after a host timeout or
	 * service teardown. Use the normal result path to release late-ack DMA
	 * and command slots with their original allocation lengths.
	 */
	if (dcp_fw_is_14_7(ep->dcp) &&
	    eshdr->category == EPIC_CAT_REPLY && payload_size >= 4 &&
	    payload_size < sizeof(struct epic_cmd)) {
		struct epic_cmd cmd = {};
		u16 tag = le16_to_cpu(eshdr->tag);

		memcpy(&cmd.retcode, payload, sizeof(cmd.retcode));
		dev_err(ep->dcp->dev,
			"AFK[ep:%02x]: channel %d short reply type %#x tag %#x ret %#x\n",
			ep->endpoint, channel, type, tag, le32_to_cpu(cmd.retcode));
		return afk_recv_handle_reply(ep, channel, tag, &cmd, sizeof(cmd));
	}

	if (service->torndown) {
		dev_warn(ep->dcp->dev,
			 "AFK[ep:%02x]: std service notify on torn down service "
			 "(chan:%u)\n", ep->endpoint, channel);
		return;
	}

	if (type == EPIC_TYPE_NOTIFY && eshdr->category == EPIC_CAT_NOTIFY) {
		struct epic_std_service_ap_call *call = payload;
		size_t call_size;
		u32 command;
		void *reply;
		u16 group;
		int ret;

		if (payload_size < sizeof(*call))
			return;

		call_size = le32_to_cpu(call->call.data_len);
		if (payload_size < sizeof(*call) + call_size)
			return;

		group = le16_to_cpu(call->call.group);
		command = le32_to_cpu(call->call.command);

		/*
		 * Command numbers are per group: a group-less .call handler
		 * would take another group's call for a group 0 one. Without
		 * call_v2 there is no handler for it, and AFK cannot guess a
		 * reply, so it gets none, as a call without any handler.
		 */
		if (!service->ops->call_v2 && (group || !service->ops->call)) {
			if (group)
				dev_warn_ratelimited(ep->dcp->dev,
						     "AFK[ep:%02x]: %s (chan:%u) apcall %u:%u has no handler, sending NO reply\n",
						     ep->endpoint, service->ops->name,
						     channel, group, command);
			return;
		}
		reply = kzalloc(payload_size, GFP_KERNEL);
		if (!reply)
			return;

		if (service->ops->call_v2)
			ret = service->ops->call_v2(service, group, command,
						    payload + sizeof(*call),
						    call_size,
						    reply + sizeof(*call),
						    call_size);
		else
			ret = service->ops->call(service, command,
						 payload + sizeof(*call),
						 call_size,
						 reply + sizeof(*call),
						 call_size);
		if (ret) {
			dev_warn(ep->dcp->dev,
				 "AFK[ep:%02x]: %s (chan:%u) apcall %u:%u handler returned %d, sending NO reply\n",
				 ep->endpoint, service->ops->name, channel,
				 group, command, ret);
			kfree(reply);
			return;
		}

		memcpy(reply, call, sizeof(*call));
		afk_send_epic(ep, channel, le16_to_cpu(eshdr->tag),
			      EPIC_TYPE_NOTIFY_ACK, EPIC_CAT_REPLY,
			      EPIC_SUBTYPE_STD_SERVICE, reply, payload_size);
		kfree(reply);

		return;
	}

	if (type == EPIC_TYPE_NOTIFY && eshdr->category == EPIC_CAT_REPORT) {
		if (service->ops->report)
			service->ops->report(service, le16_to_cpu(eshdr->type),
					     payload, payload_size);
		return;
	}

	dev_err(ep->dcp->dev,
		"AFK[ep:%02x]: channel %d received unhandled standard service message: %x / %x\n",
		ep->endpoint, channel, type, eshdr->category);
	if (ep->dcp->hw.iomfb_method_profile !=
	    DCP_IOMFB_METHODS_H17P_EXTERNAL)
		print_hex_dump(KERN_INFO, "AFK: ", DUMP_PREFIX_NONE, 16, 1,
			       payload, payload_size, true);
}

static bool afk_validate_h17p_header(const u8 *data, size_t size)
{
	/* The measured length counts bytes after the first eight header bytes. */
	return size >= sizeof(struct epic_hdr) +
		       sizeof(struct epic_sub_hdr_h17p) &&
	       get_unaligned_le32(data + 4) == size - 8;
}

static void afk_v2_status(struct apple_dcp_afkep *ep,
			  const struct afkv2_frame *frame,
			  const struct afkv2_command *command, u32 status)
{
	u8 record[AFKV2_HEADER_SIZE + AFKV2_COMMAND_SIZE + AFKV2_OOB_SIZE];
	size_t size;
	int ret;

	ret = afkv2_encode_error_reply(record, sizeof(record), 0, frame, command,
				       status, &size);
	if (!ret)
		ret = afk_send_epic_token(ep, frame->interface, command->id,
					  EPIC_TYPE_REPLY, EPIC_CAT_REPLY, frame->type,
					  record + AFKV2_HEADER_SIZE,
					  size - AFKV2_HEADER_SIZE, frame->token);
	if (ret)
		dev_warn(ep->dcp->dev, "AFKV2 error reply failed: %d\n", ret);
}

/*
 * A buffer the DCP firmware names that is none of the host's pending command
 * buffers. Only the firmware's own retained mappings are followed, with the
 * permission they grant: a reply is written only where the firmware's leaf
 * allows writes, and a DVA that Linux mapped itself, or that names Linux's RAM,
 * a claimed resource or anything below DRAM (a device's registers, claimed or
 * not), is refused. The copy goes through a write-combined mapping, as the
 * RTKit buffers in the same firmware segments are mapped (dcp_rtk_shmem_setup()):
 * the DCP is not described as DMA-coherent, so a cacheable alias could see
 * stale data.
 */
static int afk_v27_access_remote(struct apple_dcp_afkep *ep, u64 dva, void *buffer,
			   size_t size, bool to_remote, bool copy)
{
	size_t off = 0;

	if (!dva || size > SZ_1M || size > U64_MAX - dva)
		return -EINVAL;
	while (off < size) {
		phys_addr_t pa = 0;
		size_t chunk = min_t(size_t, size - off,
				    PAGE_SIZE - ((dva + off) & ~PAGE_MASK));
		int ret = apple_dart_fw_remote(ep->dcp->dev, dva + off, to_remote, &pa);
		void *cpu;

		if (!ret && (!dcp_fw_phys_in_dram(pa) ||
			     region_intersects(pa, chunk, IORESOURCE_MEM,
					       IORES_DESC_NONE) != REGION_DISJOINT))
			ret = -EACCES;
		if (ret) {
			dev_err_ratelimited(ep->dcp->dev,
				"AFK[ep:%02x]: refused remote buffer %#llx + %#zx (%s): %d\n",
				ep->endpoint, dva, off, to_remote ? "reply" : "request", ret);
			return ret;
		}
		if (!copy) {
			off += chunk;
			continue;
		}
		cpu = memremap(pa, chunk, MEMREMAP_WC);
		if (!cpu)
			return -ENOMEM;
		if (to_remote) {
			memcpy(cpu, buffer + off, chunk);
			dma_wmb();
		} else {
			dma_rmb();
			memcpy(buffer + off, cpu, chunk);
		}
		memunmap(cpu);
		off += chunk;
	}
	return 0;
}

static int afk_v2_access_owned(struct apple_epic_service *service, u64 dva,
			       void *buffer, size_t size, bool to_peer, bool copy)
{
	unsigned long flags;
	unsigned int i;
	int ret = -EACCES;

	if (!service || !service->ep->wire_v2 || size > SZ_1M || (copy && size && !buffer))
		return -EINVAL;
	if (!size)
		return 0;
	spin_lock_irqsave(&service->lock, flags);
	for (i = 0; i < MAX_PENDING_CMDS; i++) {
		struct epic_cmd_info *cmd = &service->cmds[i];
		void *owned = NULL;
		u64 offset;

		if (!test_bit(i, service->cmd_map) || cmd->done)
			continue;
		if (cmd->rxbuf && afkv2_owned_span(cmd->rxbuf_dma, cmd->rxlen, dva, size)) {
			offset = dva - cmd->rxbuf_dma;
			owned = cmd->rxbuf + offset;
		} else if (cmd->txbuf && afkv2_owned_span(cmd->txbuf_dma, cmd->txlen, dva, size)) {
			offset = dva - cmd->txbuf_dma;
			owned = cmd->txbuf + offset;
		}
		if (!owned)
			continue;
		if (!copy) {
			ret = 0;
			break;
		}
		if (to_peer) {
			memcpy(owned, buffer, size);
			dma_wmb();
		} else {
			dma_rmb();
			memcpy(buffer, owned, size);
		}
		ret = 0;
		break;
	}
	spin_unlock_irqrestore(&service->lock, flags);
	/* T6050 callbacks may refer to firmware-owned buffers in the retained DART. */
	if (ret && service->ep->dcp->fw_compat == DCP_FIRMWARE_V_27_0)
		ret = afk_v27_access_remote(service->ep, dva, buffer, size, to_peer, copy);
	if (ret)
		dev_warn_ratelimited(service->ep->dcp->dev,
				     "AFKV2 unowned buffer: ep=%u interface=%u size=%zu write=%u\n",
				     service->ep->endpoint, service->channel, size, to_peer);
	return ret;
}

int afk_v2_copy_owned(struct apple_epic_service *service, u64 dva,
		      void *buffer, size_t size, bool to_peer)
{
	return afk_v2_access_owned(service, dva, buffer, size, to_peer, true);
}

static void afk_v2_callback(struct apple_dcp_afkep *ep,
			    const struct afkv2_frame *frame,
			    const struct afkv2_command *command)
{
	struct apple_epic_service *service;
	const struct epic_service_call *call;
	const u8 *request = command->inline_data;
	u8 *owned_request = NULL, *reply = NULL;
	size_t request_size, response_size, response_body;
	size_t body_size, reply_size;
	u32 status = 0xe00002c7;
	int ret;

	dev_dbg(ep->dcp->dev, "AFKV2 callback sizes: tx=%u rx=%u value=%u inline=%zu\n",
		command->tx_size, command->rx_size, command->value, command->inline_size);
	if (frame->type != EPIC_SUBTYPE_STD_SERVICE ||
	    afkv2_callback_lengths(command, sizeof(*call) + sizeof(u32),
				   SZ_1M - AFKV2_COMMAND_SIZE,
				   &request_size, &response_size))
		goto reject;
	service = afk_service_get(afk_epic_find_service(ep, frame->interface));
	if (!service || READ_ONCE(service->torndown) ||
	    (!service->ops->call && !service->ops->call_v2)) {
		if (service)
			afk_service_put(service);
		goto reject;
	}
	if (command->oob) {
		/* Reject an unowned response destination before any callback side effect. */
		ret = afk_v2_access_owned(service, command->rx_address, NULL,
					  response_size, true, false);
		if (ret)
			goto release;
		owned_request = kmalloc(request_size, GFP_KERNEL);
		if (!owned_request) {
			status = 0xe00002bd;
			goto release;
		}
		ret = afk_v2_copy_owned(service, command->tx_address,
					owned_request, request_size, false);
		if (ret)
			goto release;
		request = owned_request;
	}
	call = (const void *)request;
	body_size = request_size - sizeof(*call);
	if ((le16_to_cpu(call->group) && !service->ops->call_v2) ||
	    le32_to_cpu(call->magic) != EPIC_SERVICE_CALL_MAGIC ||
	    le32_to_cpu(call->data_len) != body_size)
		goto release;
	response_body = response_size - sizeof(*call);
	reply = kzalloc(AFKV2_COMMAND_SIZE + response_size, GFP_KERNEL);
	if (!reply) {
		status = 0xe00002bd;
		goto release;
	}
	reply[1] = command->id;
	memcpy(reply + AFKV2_COMMAND_SIZE, call, sizeof(*call));
	((struct epic_service_call *)(reply + AFKV2_COMMAND_SIZE))->data_len =
		cpu_to_le32(response_body);
	dev_dbg(ep->dcp->dev, "AFKV2 callback begin: group=%u command=%u in=%zu out=%zu\n",
		le16_to_cpu(call->group), le32_to_cpu(call->command), body_size, response_body);
	if (service->ops->call_v2)
		ret = service->ops->call_v2(service, le16_to_cpu(call->group),
					  le32_to_cpu(call->command),
					  request + sizeof(*call), body_size,
					  reply + AFKV2_COMMAND_SIZE + sizeof(*call),
					  response_body);
	else
		ret = service->ops->call(service, le32_to_cpu(call->command),
					 request + sizeof(*call), body_size,
					 reply + AFKV2_COMMAND_SIZE + sizeof(*call), response_body);
	dev_dbg(ep->dcp->dev, "AFKV2 callback end: group=%u command=%u result=%d\n",
		le16_to_cpu(call->group), le32_to_cpu(call->command), ret);
	if (ret)
		goto release;
	reply_size = AFKV2_COMMAND_SIZE + response_size;
	if (command->oob) {
		ret = afk_v2_copy_owned(service, command->rx_address,
					reply + AFKV2_COMMAND_SIZE, response_size, true);
		if (ret)
			goto release;
		ret = afkv2_encode_oob_command(reply, reply_size, command->id,
					       command->rx_address, response_size,
					       command->tx_address, command->tx_size);
		if (ret)
			goto release;
		afkv2_put32(reply + 4, 0);
		reply_size = AFKV2_COMMAND_SIZE + AFKV2_OOB_SIZE;
	}
	ret = afk_send_epic_token(ep, frame->interface, command->id, EPIC_TYPE_REPLY,
				  EPIC_CAT_REPLY, frame->type, reply, reply_size, frame->token);
	kfree(reply);
	kfree(owned_request);
	afk_service_put(service);
	if (ret)
		dev_warn(ep->dcp->dev, "AFKV2 callback reply failed: %d\n", ret);
	return;
release:
	kfree(reply);
	kfree(owned_request);
	afk_service_put(service);
reject:
	afk_v2_status(ep, frame, command, status);
}

/* A firmware 27 panel's AV and DP service endpoint (epic/panelavep.c). */
static bool afk_is_panelav(struct apple_dcp_afkep *ep)
{
	return ep->dcp->v27_j714s && ep->endpoint >= 0x20 && ep->endpoint < 0x40 &&
	       READ_ONCE(ep->dcp->panelav[ep->endpoint - 0x20]) == ep;
}

static void afk_v2_receive(struct apple_dcp_afkep *ep, u32 channel, u32 type,
			   const void *data, size_t size)
{
	struct afkv2_frame frame;
	struct afkv2_command response;
	struct apple_epic_service *service;
	struct epic_cmd legacy = {};
	unsigned long flags;
	unsigned int slot;
	int ret;

	if (channel || type || afkv2_decode_frame(data, size, false, &frame) ||
	    !frame.interface)
		return;
	if (frame.category == AFKV2_REPORT) {
		if (frame.type == EPIC_SUBTYPE_ANNOUNCE_H17P)
			afk_recv_handle_init(ep, frame.interface, (void *)frame.payload,
					     frame.payload_size);
		else if ((frame.type == AFKV2_CLOSE && frame.interface_flags == 1) ||
			 ((ep == ep->dcp->dcpavservep || afk_is_panelav(ep) ||
			   ep->v27_closed_teardown) &&
			  afkv2_closed_report(&frame)))
			afk_recv_handle_teardown(ep, frame.interface);
		return;
	}
	if (afkv2_decode_command(frame.payload, frame.payload_size, &response))
		return;
	if (frame.category == AFKV2_COMMAND) {
		afk_v2_callback(ep, &frame, &response);
		return;
	}
	service = afk_epic_find_service(ep, frame.interface);
	if (!service)
		return;
	spin_lock_irqsave(&service->lock, flags);
	if (bitmap_empty(service->cmd_map, MAX_PENDING_CMDS)) {
		spin_unlock_irqrestore(&service->lock, flags);
		return;
	}
	service->users++;
	spin_unlock_irqrestore(&service->lock, flags);
	slot = response.id & 0xf;
	spin_lock_irqsave(&service->lock, flags);
	ret = -EPROTO;
	if (slot < MAX_PENDING_CMDS && test_bit(slot, service->cmd_map) &&
	    !service->cmds[slot].done && response.id == service->cmds[slot].tag) {
		struct epic_cmd_info *cmd = &service->cmds[slot];

		if (ep->inline_v2)
			ret = afkv2_check_inline_response(&frame, &response, service->channel,
							  cmd->type, cmd->tag, cmd->rxlen);
		else
			ret = afkv2_check_response(&frame, &response, service->channel,
						   cmd->type, cmd->tag, cmd->rxbuf_dma,
						   cmd->rxlen, cmd->txbuf_dma, cmd->txlen);
		if (!ret) {
			cmd->reply_length = response.value ? 0 :
				(ep->inline_v2 ? response.inline_size : response.rx_size);
			if (ep->inline_v2 && !response.value && cmd->reply_length)
				memcpy(cmd->rxbuf, response.inline_data, cmd->reply_length);
		}
	}
	spin_unlock_irqrestore(&service->lock, flags);
	if (!ret) {
		legacy.retcode = cpu_to_le32(response.value);
		afk_recv_handle_reply(ep, frame.interface, response.id,
				      &legacy, sizeof(legacy));
	} else {
		dev_warn(ep->dcp->dev, "AFKV2 reply rejected: %d\n", ret);
	}
	afk_service_put(service);
}

/* Channel/type zero, three words, length four, code 2/3 or 4/5.
 * The sender's sequence is NOT echoed; there is no transaction ID. Never
 * retry a timed-out exchange in the same endpoint lifetime.
 */
static bool afk_v27_sleep_reply(struct apple_dcp_afkep *ep, u32 channel,
			       u32 type, const u8 *data, size_t size)
{
	unsigned long flags;
	u8 code;

	if (!IS_ENABLED(CONFIG_APPLE_SOC_SLEEP) ||
	    !ep->dcp->v27_sleep_transport || channel || type ||
	    size != 12 || get_unaligned_le32(data + 4) != 4 ||
	    get_unaligned_le16(data + 2))
		return false;
	code = data[8];
	if (code != 3 && code != 5)
		return false;
	spin_lock_irqsave(&ep->lock, flags);
	if (!ep->stopping && !ep->sleep_poisoned &&
	    ep->sleep_waiting && code == ep->sleep_expected) {
		ep->sleep_waiting = false;
		ep->sleep_closed = code == 3;
		complete(&ep->sleep_done);
	}
	spin_unlock_irqrestore(&ep->lock, flags);
	return true;
}

static void afk_recv_handle(struct apple_dcp_afkep *ep, u32 channel, u32 type,
			    u8 *data, size_t data_size)
{
	struct apple_epic_service *service;
	struct epic_hdr *ehdr = (struct epic_hdr *)data;
	struct epic_sub_hdr *eshdr =
		(struct epic_sub_hdr *)(data + sizeof(*ehdr));
	struct epic_sub_hdr h17p_eshdr;
	size_t hdr_len = sizeof(*ehdr) + sizeof(*eshdr);
	u16 subtype;
	u8 *payload;
	size_t payload_size;

	/*
	 * The sleep exchange's answer first: its three words read as a compact header with
	 * four bytes behind it, which no compact message is, and a compact endpoint that has
	 * carried traffic would drop it as a broken fragment.
	 */
	if (afk_v27_sleep_reply(ep, channel, type, data, data_size))
		return;

	if (afk_recv_handle_compact(ep, channel, type, data, data_size))
		return;
	if (ep->wire_v2) {
		afk_v2_receive(ep, channel, type, data, data_size);
		return;
	}

	if (ep->dcp->fw_compat == DCP_FIRMWARE_H17P) {
		const struct epic_sub_hdr_h17p *c =
			(const struct epic_sub_hdr_h17p *)(data + sizeof(*ehdr));

		hdr_len = sizeof(*ehdr) + sizeof(*c);
		if (data_size < hdr_len)
			goto too_small;
		if (ep->dcp->fw_compat == DCP_FIRMWARE_H17P &&
		    ep->dcp->hw.iomfb_method_profile != DCP_IOMFB_METHODS_H17G &&
		    !afk_validate_h17p_header(data, data_size)) {
			dev_err(ep->dcp->dev,
				"AFK[ep:%02x]: invalid compact message length\n",
				ep->endpoint);
			return;
		}

		/*
		 * H17P multiplexes every service of an endpoint onto queue
		 * channel 0 and carries the real channel in the EPIC header
		 * instead: byte 0 is a sequence counter, byte 1 a flags byte
		 * and bytes 2-3 the channel.  ep:20 announces "system" as
		 * channel 1 and "powerlog-service" as channel 3, then sends the
		 * mNits/uAmps/iDAC backlight reports on channel 3.  Taking the
		 * queue channel at face value binds "system" to 0 and makes
		 * every later announce and report look like a duplicate.
		 */
		channel = get_unaligned_le16(data + 2);

		/*
		 * Normalise into the wide sub-header the rest of this file
		 * expects, mapping H17P's announce subtype onto the canonical
		 * one so no downstream check has to know the difference.
		 */
		memset(&h17p_eshdr, 0, sizeof(h17p_eshdr));
		h17p_eshdr.category = c->category;
		h17p_eshdr.type = cpu_to_le16(c->type == EPIC_SUBTYPE_ANNOUNCE_H17P ?
					      EPIC_SUBTYPE_ANNOUNCE : c->type);
		h17p_eshdr.tag = cpu_to_le16(le32_to_cpu(c->tag));
		eshdr = &h17p_eshdr;
	}

	if (data_size < hdr_len)
		goto too_small;

	subtype = le16_to_cpu(eshdr->type);
	payload = data + hdr_len;
	payload_size = data_size - hdr_len;


	trace_afk_recv_handle(ep, channel, type, data_size, ehdr, eshdr);


	service = afk_epic_find_service(ep, channel);

	if (!service) {
		if (type != EPIC_TYPE_NOTIFY && type != EPIC_TYPE_REPLY) {
			dev_err(ep->dcp->dev,
				"AFK[ep:%02x]: expected notify but got 0x%x on channel %d\n",
				ep->endpoint, type, channel);
			return;
		}
		if (eshdr->category != EPIC_CAT_REPORT) {
			dev_err(ep->dcp->dev,
				"AFK[ep:%02x]: expected report but got 0x%x on channel %d\n",
				ep->endpoint, eshdr->category, channel);
			return;
		}
		if (subtype == EPIC_SUBTYPE_TEARDOWN) {
			dev_dbg(ep->dcp->dev,
				"AFK[ep:%02x]: teardown without service on channel %d\n",
				ep->endpoint, channel);
			return;
		}
		if (subtype != EPIC_SUBTYPE_ANNOUNCE) {
			dev_err(ep->dcp->dev,
				"AFK[ep:%02x]: expected announce but got 0x%x on channel %d\n",
				ep->endpoint, subtype, channel);
			return;
		}

		return afk_recv_handle_init(ep, channel, payload, payload_size);
	}

	if (!service) {
		dev_err(ep->dcp->dev, "AFK[ep:%02x]: channel %d has no service\n",
			ep->endpoint, channel);
		return;
	}

	if (type == EPIC_TYPE_NOTIFY && eshdr->category == EPIC_CAT_REPORT &&
	    subtype == EPIC_SUBTYPE_TEARDOWN)
		return afk_recv_handle_teardown(ep, channel);

	/*
	 * T6030 14.7 returns a command result as a NOTIFY whose category is
	 * REPLY and whose body is a normal epic_cmd. The tag check inside the
	 * reply handler ignores anything that is not one of our commands.
	 */
	if (dcp_fw_is_14_7(ep->dcp) &&
	    (type == EPIC_TYPE_REPLY || type == EPIC_TYPE_NOTIFY) &&
	    eshdr->category == EPIC_CAT_REPLY &&
	    payload_size >= sizeof(struct epic_cmd))
		return afk_recv_handle_reply(ep, channel,
					     le16_to_cpu(eshdr->tag), payload,
					     payload_size);

	if (!dcp_fw_is_14_7(ep->dcp) &&
	    type == EPIC_TYPE_REPLY && eshdr->category == EPIC_CAT_REPLY)
		return afk_recv_handle_reply(ep, channel,
					     le16_to_cpu(eshdr->tag), payload,
					     payload_size);

	if (subtype == EPIC_SUBTYPE_STD_SERVICE)
		return afk_recv_handle_std_service(
			ep, channel, type, ehdr, eshdr, payload, payload_size);

	dev_err(ep->dcp->dev, "AFK[ep:%02x]: channel %d received unhandled message "
		"(type %x subtype %x)\n", ep->endpoint, channel, type, subtype);
	if (ep->dcp->hw.iomfb_method_profile !=
	    DCP_IOMFB_METHODS_H17P_EXTERNAL)
		print_hex_dump(KERN_INFO, "AFK: ", DUMP_PREFIX_NONE, 16, 1,
			       payload, payload_size, true);
	return;

too_small:
	dev_err(ep->dcp->dev, "AFK[ep:%02x]: payload too small: %lx\n",
		ep->endpoint, data_size);
}

static bool afk_recv(struct apple_dcp_afkep *ep)
{
	struct afk_qe *hdr;
	u32 rptr, wptr;
	u32 magic, size, channel, type;

	if (!ep->rxbfr.ready) {
		dev_err(ep->dcp->dev, "AFK[ep:%02x]: got RECV but not ready\n",
			ep->endpoint);
		return false;
	}

	rptr = le32_to_cpu(*ep->rxbfr.rptr);
	wptr = le32_to_cpu(*ep->rxbfr.wptr);
	trace_afk_recv_rwptr_pre(ep, rptr, wptr);

	if (!afk_ring_ptrs_valid(rptr, wptr, ep->rxbfr.bufsz, ep->rxbfr.stride)) {
		dev_warn_ratelimited(ep->dcp->dev,
				     "AFK[ep:%02x]: invalid receive ring pointers: rptr 0x%x wptr 0x%x\n",
				     ep->endpoint, rptr, wptr);
		return false;
	}

	if (rptr == wptr)
		return false;

	dma_rmb();

	hdr = ep->rxbfr.buf + rptr;
	magic = le32_to_cpu(hdr->magic);
	size = le32_to_cpu(hdr->size);
	trace_afk_recv_qe(ep, rptr, magic, size);

	if (magic != QE_MAGIC) {
		dev_warn(ep->dcp->dev, "AFK[ep:%02x]: invalid queue entry magic: 0x%x\n",
			 ep->endpoint, magic);
		return false;
	}

	/*
	 * If there's not enough space for the payload the co-processor inserted
	 * the current dummy queue entry and we have to advance to the next one
	 * which will contain the real data.
	*/
	if (!afk_ring_payload_fits(rptr, size, ep->rxbfr.bufsz)) {
		rptr = 0;
		hdr = ep->rxbfr.buf + rptr;
		magic = le32_to_cpu(hdr->magic);
		size = le32_to_cpu(hdr->size);
		trace_afk_recv_qe(ep, rptr, magic, size);

		if (magic != QE_MAGIC) {
			dev_warn(ep->dcp->dev,
				 "AFK[ep:%02x]: invalid next queue entry magic: 0x%x\n",
				 ep->endpoint, magic);
			return false;
		}
	}

	if (!afk_ring_payload_fits(rptr, size, ep->rxbfr.bufsz)) {
		dev_warn(ep->dcp->dev,
			 "AFK[ep:%02x]: queue entry of 0x%x bytes at 0x%x out of bounds\n",
			 ep->endpoint, size, rptr);
		return false;
	}

	channel = le32_to_cpu(hdr->channel);
	type = le32_to_cpu(hdr->type);
	/*
	 * Publishing rptr releases this entry to the firmware. Handlers can
	 * sleep and send replies, so retain a private copy before allowing
	 * the producer to reuse the ring storage.
	 */
	memcpy(ep->rx_scratch, hdr->data, size);

	rptr = afk_ring_advance(rptr, sizeof(*hdr) + size,
				ep->rxbfr.bufsz, ep->rxbfr.stride);

	dma_mb();

	*ep->rxbfr.rptr = cpu_to_le32(rptr);
	trace_afk_recv_rwptr_post(ep, rptr, wptr);

	afk_recv_handle(ep, channel, type, ep->rx_scratch, size);

	return true;
}

static bool afk_recv_entry(void *ep)
{
	return afk_recv(ep);
}

static void afk_recv_drain(struct apple_dcp_afkep *ep)
{
	unsigned int budget = afk_ring_drain_budget(ep->rxbfr.bufsz,
						    ep->rxbfr.stride);

	if (afk_ring_drain(afk_recv_entry, ep, budget) == budget &&
	    ep->rxbfr.ready && le32_to_cpu(*ep->rxbfr.rptr) != le32_to_cpu(*ep->rxbfr.wptr))
		dev_warn_ratelimited(ep->dcp->dev,
				     "AFK[ep:%02x]: receive ring did not drain in one pass\n",
				     ep->endpoint);
}

static void afk_receive_message_worker(struct work_struct *work_)
{
	struct afk_receive_message_work *work;
	u16 type;

	work = container_of(work_, struct afk_receive_message_work, work);

	type = FIELD_GET(RBEP_TYPE, work->message);

	switch (type) {
	case RBEP_INIT_ACK:
		break;

	case RBEP_START_ACK:
		/* 14.7 may queue messages before it acks the start. */
		if (dcp_fw_is_14_7(work->ep->dcp) &&
		    work->ep->rxbfr.ready)
			afk_recv_drain(work->ep);
		complete_all(&work->ep->started);
		break;

	case RBEP_SHUTDOWN_ACK:
		complete_all(&work->ep->stopped);
		break;

	case RBEP_GETBUF:
		afk_getbuf(work->ep, work->message);
		break;

	case RBEP_INIT_TX:
		afk_init_rxtx(work->ep, work->message, &work->ep->txbfr);
		break;

	case RBEP_INIT_RX:
		afk_init_rxtx(work->ep, work->message, &work->ep->rxbfr);
		break;

	case RBEP_RECV:
		afk_recv_drain(work->ep);
		break;

	case RBEP_SPACE_AVAILABLE:
		/* The compact sender polls the shared consumer pointer. */
		if (!afk_compact_capable(work->ep))
			goto unknown;
		break;

	case RBEP_WAIT_FOR_SPACE:
		/*
		 * Compact endpoints ask for a wakeup when their ring fills:
		 * drain and publish rptr before acknowledging.
		 */
		if (!afk_compact_capable(work->ep))
			goto unknown;
		afk_recv_drain(work->ep);
		dma_wmb();
		afk_send(work->ep, FIELD_PREP(RBEP_TYPE, RBEP_SPACE_AVAILABLE));
		break;

	default:
unknown:
		dev_err(work->ep->dcp->dev,
			"Received unknown AFK message type: 0x%x\n", type);
	}

	atomic_dec_return_release(&work->ep->sleep_jobs);
	kfree(work);
}

int afk_receive_message(struct apple_dcp_afkep *ep, u64 message)
{
	struct afk_receive_message_work *work;
	unsigned long flags;

	if (!ep)
		return -ENODEV;

	// TODO: comment why decoupling from rtkit thread is required here
	work = kzalloc(sizeof(*work), GFP_KERNEL);
	if (!work)
		return -ENOMEM;

	work->ep = ep;
	work->message = message;
	INIT_WORK(&work->work, afk_receive_message_worker);
	spin_lock_irqsave(&ep->lock, flags);
	if (ep->stopping) {
		spin_unlock_irqrestore(&ep->lock, flags);
		kfree(work);
		return -ESHUTDOWN;
	}
	atomic_inc(&ep->sleep_jobs);
	queue_work(ep->wq, &work->work);
	spin_unlock_irqrestore(&ep->lock, flags);

	return 0;
}

static int afk_send_epic_token(struct apple_dcp_afkep *ep, u32 channel, u16 tag,
			       enum epic_type etype, enum epic_category ecat, u8 stype,
			       const void *payload, size_t payload_len, u64 token)
{
	u32 rptr, wptr;
	struct afk_qe *hdr, *hdr2;
	struct epic_hdr *ehdr;
	struct epic_sub_hdr *eshdr;
	unsigned long flags;
	size_t total_epic_size, total_size;
	u8 v2_command[AFKV2_COMMAND_SIZE + AFKV2_OOB_SIZE];
	u8 v2_category = 0;
	u16 v2_flags = 0;
	int ret;

	/* External H17P services use AFKV2; the internal profile has no EPIC sender. */
	if (ep->dcp->fw_compat == DCP_FIRMWARE_H17P && !ep->wire_v2) {
		dev_dbg_once(ep->dcp->dev,
			     "AFK[ep:%02x]: not sending EPIC message on H17P\n",
			     ep->endpoint);
		return -EOPNOTSUPP;
	}

	if (ep->wire_v2) {
		if (!channel || channel > U16_MAX || tag > U8_MAX || payload_len > SZ_1M)
			return -EINVAL;
		if (ecat == EPIC_CAT_REPORT) {
			if ((stype != AFKV2_OPEN && stype != AFKV2_CLOSE) || payload_len)
				return -EOPNOTSUPP;
			v2_flags = 1;
		} else if (ecat == EPIC_CAT_COMMAND) {
			if (ep->inline_v2) {
				struct afkv2_command command;

				if (stype != EPIC_SUBTYPE_STD_SERVICE ||
				    afkv2_decode_command(payload, payload_len, &command) ||
				    command.oob || command.id != tag)
					return -EINVAL;
			} else {
				const struct epic_cmd *cmd = payload;

				if (stype != EPIC_SUBTYPE_STD_SERVICE || !cmd ||
				    payload_len != sizeof(*cmd))
					return -EINVAL;
				ret = afkv2_encode_oob_command(v2_command, sizeof(v2_command), tag,
							       le64_to_cpu(cmd->rxbuf),
							       le32_to_cpu(cmd->rxlen),
							       le64_to_cpu(cmd->txbuf),
							       le32_to_cpu(cmd->txlen));
				if (ret)
					return ret;
				payload = v2_command;
				payload_len = sizeof(v2_command);
			}
			v2_category = AFKV2_COMMAND;
		} else if (ecat == EPIC_CAT_REPLY) {
			v2_category = AFKV2_RESPONSE;
		} else {
			return -EOPNOTSUPP;
		}
	}

	spin_lock_irqsave(&ep->lock, flags);
	if (READ_ONCE(ep->dcp->sleep_v27_closed) && ecat != EPIC_CAT_REPLY) {
		ret = -EBUSY;
		goto out;
	}
	if (ep->stopping || !ep->txbfr.ready) {
		ret = -ESHUTDOWN;
		goto out;
	}

	if (ep->wire_v2 &&
	    (!ep->txbfr.ready || !ep->txbfr.rptr || !ep->txbfr.wptr || !ep->txbfr.buf)) {
		ret = -ESHUTDOWN;
		goto out;
	}

	dma_rmb();
	rptr = le32_to_cpu(*ep->txbfr.rptr);
	wptr = le32_to_cpu(*ep->txbfr.wptr);
	if (rptr >= ep->txbfr.bufsz || wptr >= ep->txbfr.bufsz ||
	    !IS_ALIGNED(rptr, ep->txbfr.stride) ||
	    !IS_ALIGNED(wptr, ep->txbfr.stride)) {
		ret = -EPROTO;
		goto out;
	}
	trace_afk_send_rwptr_pre(ep, rptr, wptr);
	total_epic_size = sizeof(*ehdr) +
		(ep->wire_v2 ? sizeof(struct epic_sub_hdr_h17p) : sizeof(*eshdr)) +
		payload_len;
	total_size = sizeof(*hdr) + total_epic_size;

	if (ep->wire_v2) {
		u32 available, needed;

		if (!ep->txbfr.ready || !ep->txbfr.stride ||
		    (ep->txbfr.stride & (ep->txbfr.stride - 1)) ||
		    rptr >= ep->txbfr.bufsz || wptr >= ep->txbfr.bufsz ||
		    !IS_ALIGNED(rptr, ep->txbfr.stride) ||
		    !IS_ALIGNED(wptr, ep->txbfr.stride) ||
		    total_size >= ep->txbfr.bufsz) {
			ret = -EPROTO;
			goto out;
		}
		available = rptr > wptr ? rptr - wptr : ep->txbfr.bufsz - wptr + rptr;
		needed = ALIGN(total_size, ep->txbfr.stride);
		if (total_size > ep->txbfr.bufsz - wptr)
			needed += ep->txbfr.bufsz - wptr;
		/* Retain an empty slot; equal read/write pointers mean empty. */
		if (needed >= available) {
			ret = -ENOSPC;
			goto out;
		}
	}

	hdr = hdr2 = NULL;

	/*
	 * We need to figure out how to place the entire headers and payload
	 * into the ring buffer:
	 * - If the write pointer is in front of the read pointer we just need
	 *   enough space inbetween to store everything.
	 * - If the read pointer has already wrapper around the end of the
	 *   buffer we can
	 *    a) either store the entire payload at the writer pointer if
	 *       there's enough space until the end,
	 *    b) or just store the queue entry at the write pointer to indicate
	 *       that we need to wrap to the start and then store the headers
	 *       and the payload at the beginning of the buffer. The queue
	 *       header has to be store twice in this case.
	 * In either case we have to ensure that there's always enough space
	 * so that we don't accidentally overwrite other buffers.
	 */
	if (wptr < rptr) {
		/*
		 * If wptr < rptr we can't wrap around and only have to make
		 * sure that there's enough space for the entire payload.
		 */
		if (wptr + total_size > rptr) {
			ret = -ENOMEM;
			goto out;
		}

		hdr = ep->txbfr.buf + wptr;
		wptr += sizeof(*hdr);
	} else {
		/* We need enough space to place at least a queue entry */
		if (wptr + sizeof(*hdr) > ep->txbfr.bufsz) {
			ret = -ENOMEM;
			goto out;
		}

		/*
		 * If we can place a single queue entry but not the full payload
		 * we need to place one queue entry at the end of the ring
		 * buffer and then another one together with the entire
		 * payload at the beginning.
		 */
		if (wptr + total_size > ep->txbfr.bufsz) {
			/*
			 * Ensure there's space for the  queue entry at the
			 * beginning
			 */
			if (sizeof(*hdr) > rptr) {
				ret = -ENOMEM;
				goto out;
			}

			/*
			 * Place two queue entries to indicate we want to wrap
			 * around to the firmware.
			 */
			hdr = ep->txbfr.buf + wptr;
			hdr2 = ep->txbfr.buf;
			wptr = sizeof(*hdr);

			/* Ensure there's enough space for the entire payload */
			if (wptr + total_epic_size > rptr) {
				ret = -ENOMEM;
				goto out;
			}
		} else {
			/* We have enough space to place the entire payload */
			hdr = ep->txbfr.buf + wptr;
			wptr += sizeof(*hdr);
		}
	}
	/*
	 * At this point we're guaranteed that hdr (and possibly hdr2) point
	 * to a buffer large enough to fit the queue entry and that we have
	 * enough space at wptr to store the payload.
	 */

	hdr->magic = cpu_to_le32(QE_MAGIC);
	hdr->size = cpu_to_le32(total_epic_size);
	hdr->channel = cpu_to_le32(ep->wire_v2 ? 0 : channel);
	hdr->type = cpu_to_le32(ep->wire_v2 ? 0 : etype);
	if (hdr2)
		memcpy(hdr2, hdr, sizeof(*hdr));

	if (ep->wire_v2) {
		size_t written;

		ret = afkv2_encode_frame(ep->txbfr.buf + wptr, total_epic_size,
					 ep->qe_seq++, channel, token, stype, v2_category,
					 v2_flags, payload, payload_len, &written);
		if (ret)
			goto out;
		wptr += written;
	} else {
		ehdr = ep->txbfr.buf + wptr;
		memset(ehdr, 0, sizeof(*ehdr));
		ehdr->version = 2;
		ehdr->seq = cpu_to_le16(ep->qe_seq++);
		ehdr->timestamp = cpu_to_le64(0);
		wptr += sizeof(*ehdr);

		eshdr = ep->txbfr.buf + wptr;
		memset(eshdr, 0, sizeof(*eshdr));
		eshdr->length = cpu_to_le32(payload_len);
		eshdr->version = 4;
		eshdr->category = ecat;
		eshdr->type = cpu_to_le16(stype);
		eshdr->timestamp = cpu_to_le64(0);
		eshdr->tag = cpu_to_le16(tag);
		/* The 14.7 external role's callback replies carry 8 in byte 35. */
		if (ecat == EPIC_CAT_REPLY && dcp_is_v14_7_external(ep->dcp))
			eshdr->unk = cpu_to_le16(8 << 8);
		if (ecat == EPIC_CAT_REPLY)
			eshdr->inline_len = cpu_to_le32(payload_len - 4);
		else
			eshdr->inline_len = cpu_to_le32(0);
		wptr += sizeof(*eshdr);
	}

	if (!ep->wire_v2) {
		memcpy(ep->txbfr.buf + wptr, payload, payload_len);
		wptr += payload_len;
	}
	wptr = afk_ring_advance(wptr, 0, ep->txbfr.bufsz,
				ep->txbfr.stride);
	trace_afk_send_rwptr_post(ep, rptr, wptr);

	/* Publish the completed entry before allowing the consumer to read it. */
	dma_wmb();
	*ep->txbfr.wptr = cpu_to_le32(wptr);
	afk_send(ep, FIELD_PREP(RBEP_TYPE, RBEP_SEND) |
			     FIELD_PREP(SEND_WPTR, wptr));
	ret = 0;

out:
	spin_unlock_irqrestore(&ep->lock, flags);
	return ret;
}

int afk_send_epic(struct apple_dcp_afkep *ep, u32 channel, u16 tag,
		  enum epic_type etype, enum epic_category ecat, u8 stype,
		  const void *payload, size_t payload_len)
{
	return afk_send_epic_token(ep, channel, tag, etype, ecat, stype,
				   payload, payload_len, 0);
}

/* AFKV2: the OPEN of a service whose provider sends it itself (open_later). */
int afk_service_open(struct apple_epic_service *service)
{
	if (!service->ep->wire_v2)
		return -EOPNOTSUPP;
	return afk_send_epic(service->ep, service->channel, 0, EPIC_TYPE_NOTIFY,
			     EPIC_CAT_REPORT, AFKV2_OPEN, NULL, 0);
}

int afk_send_command(struct apple_epic_service *service, u8 type,
		     const void *payload, size_t payload_len, void *output,
		     size_t output_len, u32 *retcode)
{
	return afk_send_command_timeout(service, type, payload, payload_len,
					output, output_len, retcode,
					MSEC_PER_SEC);
}

static u16 afk_command_tag(struct apple_epic_service *service, u8 slot)
{
	struct apple_dcp_afkep *ep = service->ep;
	unsigned long flags;
	u16 tag;

	lockdep_assert_held(&service->lock);
	if (!ep->wire_v2)
		return ((service->cmd_tag++ & 0xff) << 8) | slot;

	/* Send already nests the endpoint lock below the service lock. */
	spin_lock_irqsave(&ep->lock, flags);
	tag = afkv2_command_id(ep->command_tags[service->channel]++, slot);
	spin_unlock_irqrestore(&ep->lock, flags);
	return tag;
}

static int afk_send_command_common(struct apple_epic_service *service, u8 type,
				   const void *payload, size_t payload_len,
				   void *output, size_t output_len, u32 *retcode,
				   unsigned int timeout_ms, size_t *received,
				   size_t *reply_len);

static int afk_send_command_timeout_len(struct apple_epic_service *service, u8 type,
					const void *payload, size_t payload_len,
					void *output, size_t output_len, u32 *retcode,
					unsigned int timeout_ms, size_t *received)
{
	return afk_send_command_common(service, type, payload, payload_len,
				       output, output_len, retcode, timeout_ms,
				       received, NULL);
}

int afk_send_command_timeout(struct apple_epic_service *service, u8 type,
			     const void *payload, size_t payload_len,
			     void *output, size_t output_len, u32 *retcode,
			     unsigned int timeout_ms)
{
	return afk_send_command_timeout_len(service, type, payload, payload_len,
					    output, output_len, retcode, timeout_ms, NULL);
}

/*
 * @reply_len (legacy EPIC only): return just the bytes the firmware reported
 * in its reply instead of the whole allocated output.
 */
static int afk_send_command_common(struct apple_epic_service *service, u8 type,
				   const void *payload, size_t payload_len,
				   void *output, size_t output_len, u32 *retcode,
				   unsigned int timeout_ms, size_t *received,
				   size_t *reply_len)
{
	struct epic_cmd cmd;
	void *inline_command = NULL;
	size_t copy_len = output_len;
	void *rxbuf, *txbuf;
	dma_addr_t rxbuf_dma, txbuf_dma;
	unsigned long flags;
	int ret, idx;
	u16 tag;
	struct apple_dcp_afkep *ep;
	DECLARE_COMPLETION_ONSTACK(completion);

	if (received)
		*received = 0;
	if (reply_len) {
		*reply_len = 0;
		if (payload_len > U32_MAX || output_len > U32_MAX)
			return -EOVERFLOW;
	}
	service = afk_service_get(service);
	if (!service)
		return -ENODEV;
	ep = service->ep;
	if (READ_ONCE(ep->compact)) {
		u32 request_status = afk_compact_next_status(service);
		size_t copied = 0;
		u32 status = 0;

		ret = afk_send_compact_command(service, type, payload, payload_len,
					       output, output_len, request_status,
					       &status, &copied,
					       msecs_to_jiffies(timeout_ms));
		/* Success is the reply echoing the request's status. */
		if (!ret && retcode)
			*retcode = status == request_status ? 0 : (status ?: ~0U);
		if (!ret && received)
			*received = copied;
		if (!ret && reply_len)
			*reply_len = copied;
		goto err_put_service;
	}
	/* AFKv2 replies report their length through reply_length instead. */
	if (reply_len && ep->wire_v2) {
		ret = -EOPNOTSUPP;
		goto err_put_service;
	}
	if (ep->wire_v2 && (!service->channel || service->channel >= AFKV2_INTERFACES ||
			    !ep->command_tags)) {
		ret = -ENODEV;
		goto err_put_service;
	}
	if (READ_ONCE(ep->dcp->quiescing)) {
		ret = -ENODEV;
		goto err_put_service;
	}

	if (ep->wire_v2 && (payload_len > SZ_1M || output_len > SZ_1M ||
			    (payload_len && !payload))) {
		ret = -EINVAL;
		goto err_put_service;
	}

	if (dcp_is_ext26(ep->dcp)) {
		size_t charge = ALIGN(payload_len, SZ_16K) + ALIGN(output_len, SZ_16K);

		spin_lock_irqsave(&ep->lock, flags);
		ret = READ_ONCE(ep->dcp->crashed) ? -EIO :
			(ep->ext26_command_count >= 255 ||
			 charge > SZ_16M - ep->ext26_command_bytes) ? -ENOSPC : 0;
		if (!ret) {
			ep->ext26_command_bytes += charge;
			ep->ext26_command_count++;
		}
		spin_unlock_irqrestore(&ep->lock, flags);
		if (ret)
			goto err_put_service;
	}

	rxbuf = dma_alloc_coherent(ep->dcp->dev, output_len, &rxbuf_dma,
				   GFP_KERNEL);
	if (!rxbuf) {
		ret = -ENOMEM;
		goto err_put_service;
	}
	txbuf = dma_alloc_coherent(ep->dcp->dev, payload_len, &txbuf_dma,
				   GFP_KERNEL);
	if (!txbuf) {
		ret = -ENOMEM;
		goto err_free_rxbuf;
	}

	memcpy(txbuf, payload, payload_len);
	if (ep->inline_v2) {
		if (payload_len > AFKV2_MAX_PAYLOAD - AFKV2_COMMAND_SIZE) {
			ret = -E2BIG;
			goto err_free_txbuf;
		}
		inline_command = kmalloc(AFKV2_COMMAND_SIZE + payload_len, GFP_KERNEL);
		if (!inline_command) {
			ret = -ENOMEM;
			goto err_free_txbuf;
		}
	}

	memset(&cmd, 0, sizeof(cmd));
	cmd.retcode = cpu_to_le32(0);
	cmd.rxbuf = cpu_to_le64(rxbuf_dma);
	cmd.rxlen = cpu_to_le32(output_len);
	cmd.txbuf = cpu_to_le64(txbuf_dma);
	cmd.txlen = cpu_to_le32(payload_len);

	spin_lock_irqsave(&service->lock, flags);
	if (service->torndown || !service->enabled) {
		ret = -ENODEV;
		goto err_unlock;
	}
	idx = bitmap_find_free_region(service->cmd_map, MAX_PENDING_CMDS, 0);
	if (idx < 0) {
		ret = -ENOSPC;
		goto err_unlock;
	}

	tag = afk_command_tag(service, idx);

	service->cmds[idx].tag = tag;
	service->cmds[idx].type = type;
	service->cmds[idx].reply_length = 0;
	service->cmds[idx].reply_len = 0;
	service->cmds[idx].reply_len_valid = false;
	service->cmds[idx].rxbuf = rxbuf;
	service->cmds[idx].txbuf = txbuf;
	service->cmds[idx].rxbuf_dma = rxbuf_dma;
	service->cmds[idx].txbuf_dma = txbuf_dma;
	service->cmds[idx].rxlen = output_len;
	service->cmds[idx].txlen = payload_len;
	service->cmds[idx].free_on_ack = false;
	service->cmds[idx].done = false;
	service->cmds[idx].completion = &completion;
	init_completion(&completion);

	if (ep->inline_v2) {
		ret = afkv2_encode_inline_command(inline_command,
						  AFKV2_COMMAND_SIZE + payload_len,
						  tag, payload, payload_len, output_len);
		if (!ret)
			ret = afk_send_epic(ep, service->channel, tag, EPIC_TYPE_COMMAND,
					    EPIC_CAT_COMMAND, type, inline_command,
					    AFKV2_COMMAND_SIZE + payload_len);
	} else {
		ret = afk_send_epic(service->ep, service->channel, tag,
				    EPIC_TYPE_COMMAND, EPIC_CAT_COMMAND, type, &cmd,
				    sizeof(cmd));
	}
	kfree(inline_command);
	inline_command = NULL;
	spin_unlock_irqrestore(&service->lock, flags);
	if (ret)
		goto err_free_cmd;

	ret = wait_for_completion_timeout(&completion,
					  msecs_to_jiffies(timeout_ms));

	if (ret <= 0) {
		spin_lock_irqsave(&service->lock, flags);
		/*
		 * Check again while we're inside the lock to make sure
		 * the command wasn't completed just after
		 * wait_for_completion_timeout returned.
		 */
		if (!service->cmds[idx].done) {
			service->cmds[idx].completion = NULL;
			service->cmds[idx].free_on_ack = !dcp_is_ext26(ep->dcp);
			if (dcp_is_ext26(ep->dcp))
				WRITE_ONCE(ep->dcp->crashed, true);
			spin_unlock_irqrestore(&service->lock, flags);
			dev_warn(ep->dcp->dev,
				 "AFK[ep:%02x]: %s (chan:%u) command type 0x%x tag 0x%04x timed out after %u ms\n",
				 ep->endpoint, service->ops->name, service->channel,
				 type, tag, timeout_ms);
			if (ep->dcp->external && ep->txbfr.ready && ep->rxbfr.ready) {
				/* Diagnostic snapshot of shared RAM, not coprocessor MMIO.
				 * Unequal TX pointers mean firmware has not drained the queue.
				 */
				dma_rmb();
				dev_warn(ep->dcp->dev,
					 "AFK timeout channel=%u tag=%#x TX r/w=%#x/%#x RX r/w=%#x/%#x running=%d crashed=%d\n",
					 service->channel, tag,
					 le32_to_cpu(READ_ONCE(*ep->txbfr.rptr)),
					 le32_to_cpu(READ_ONCE(*ep->txbfr.wptr)),
					 le32_to_cpu(READ_ONCE(*ep->rxbfr.rptr)),
					 le32_to_cpu(READ_ONCE(*ep->rxbfr.wptr)),
					 apple_rtkit_is_running(ep->dcp->rtk),
					 apple_rtkit_is_crashed(ep->dcp->rtk));
			}
			afk_service_put(service);
			return -ETIMEDOUT;
		}
		spin_unlock_irqrestore(&service->lock, flags);
	}

	ret = 0;
	if (retcode)
		*retcode = service->cmds[idx].retcode;
	if (reply_len) {
		ret = afk_reply_size_valid(service->cmds[idx].reply_len_valid,
					   service->cmds[idx].reply_len, output_len, 0);
		if (!ret) {
			*reply_len = service->cmds[idx].reply_len;
			dma_rmb();
			if (output && output_len) {
				memset(output, 0, output_len);
				memcpy(output, rxbuf, *reply_len);
			}
		}
		goto err_free_cmd;
	}
	if (ep->wire_v2 && received)
		copy_len = service->cmds[idx].reply_length;
	if (ep->wire_v2 && !service->cmds[idx].retcode &&
	    service->cmds[idx].reply_length < copy_len)
		ret = -EMSGSIZE;
	else if (output && output_len && (!ep->wire_v2 || !service->cmds[idx].retcode)) {
		if (ep->wire_v2)
			dma_rmb();
		memcpy(output, rxbuf, copy_len);
		if (received)
			*received = copy_len;
	}

err_free_cmd:
	spin_lock_irqsave(&service->lock, flags);
	service->cmds[idx].completion = NULL;
	bitmap_release_region(service->cmd_map, idx, 0);
err_unlock:
	spin_unlock_irqrestore(&service->lock, flags);
err_free_txbuf:
	kfree(inline_command);
	if (!dcp_is_ext26(ep->dcp))
		dma_free_coherent(ep->dcp->dev, payload_len, txbuf, txbuf_dma);
err_free_rxbuf:
	if (!dcp_is_ext26(ep->dcp))
		dma_free_coherent(ep->dcp->dev, output_len, rxbuf, rxbuf_dma);
err_put_service:
	afk_service_put(service);
	return ret;
}

int afk_service_call(struct apple_epic_service *service, u16 group, u32 command,
		     const void *data, size_t data_len, size_t data_pad,
		     void *output, size_t output_len, size_t output_pad)
{
	return afk_service_call_timeout(service, group, command, data, data_len,
					data_pad, output, output_len,
					output_pad, MSEC_PER_SEC);
}

int afk_service_call_len(struct apple_epic_service *service, u16 group,
			 u32 command, const void *data, size_t data_len,
			 size_t data_pad, void *output, size_t output_len,
			 size_t output_pad, size_t *received)
{
	return afk_service_call_timeout_len(service, group, command, data, data_len,
					    data_pad, output, output_len, output_pad,
					    MSEC_PER_SEC, received);
}

int afk_service_call_timeout(struct apple_epic_service *service, u16 group,
			     u32 command, const void *data, size_t data_len,
			     size_t data_pad, void *output, size_t output_len,
			     size_t output_pad, unsigned int timeout_ms)
{
	return afk_service_call_timeout_len(service, group, command, data, data_len,
					    data_pad, output, output_len, output_pad,
					    timeout_ms, NULL);
}

int afk_service_call_timeout_len(struct apple_epic_service *service, u16 group,
				 u32 command, const void *data, size_t data_len,
				 size_t data_pad, void *output, size_t output_len,
				 size_t output_pad, unsigned int timeout_ms,
				 size_t *received)
{
	struct epic_service_call *call;
	size_t wire_received = 0, result_size;
	void *bfr;
	size_t bfr_len = max(data_len + data_pad, output_len + output_pad) +
			 sizeof(*call);
	int ret;
	u32 retcode;
	u32 retlen;

	if (received)
		*received = 0;
	if (!service)
		return -ENODEV;

	bfr = kzalloc(bfr_len, GFP_KERNEL);
	if (!bfr)
		return -ENOMEM;

	call = bfr;

	memset(call, 0, sizeof(*call));
	call->group = cpu_to_le16(group);
	call->command = cpu_to_le32(command);
	call->data_len = cpu_to_le32(service->ep->inline_v2 ? data_len :
				   data_len + data_pad);
	call->magic = cpu_to_le32(EPIC_SERVICE_CALL_MAGIC);

	memcpy(bfr + sizeof(*call), data, data_len);

	/* Inline carries only the request; its response capacity is independent. */
	ret = afk_send_command_timeout_len(service, EPIC_SUBTYPE_STD_SERVICE, bfr,
					   service->ep->inline_v2 ? sizeof(*call) + data_len :
					   bfr_len, bfr, bfr_len, &retcode, timeout_ms,
					   received ? &wire_received : NULL);
	if (ret) {
		dev_warn(service->ep->dcp->dev,
			 "AFK[ep:%02x]: %s (chan:%u) service call group %u cmd %u failed to complete: %d\n",
			 service->ep->endpoint, service->ops->name,
			 service->channel, group, command, ret);
		goto out;
	}
	if (retcode) {
		dev_warn(service->ep->dcp->dev,
			 "AFK[ep:%02x]: %s (chan:%u) service call group %u cmd %u returned retcode 0x%08x (reply data_len %u)\n",
			 service->ep->endpoint, service->ops->name,
			 service->channel, group, command, retcode,
			 le32_to_cpu(call->data_len));
		ret = -EINVAL;
		goto out;
	}
	if (received) {
		/* The reply may use the padding reserved after the output. */
		ret = afkv2_service_reply_size(bfr, wire_received, group, command,
					       output_len + output_pad, &result_size);
		if (ret)
			goto out;
		*received = min(result_size, output_len);
	}
	if (le32_to_cpu(call->magic) != EPIC_SERVICE_CALL_MAGIC ||
	    le16_to_cpu(call->group) != group ||
	    le32_to_cpu(call->command) != command) {
		dev_warn(service->ep->dcp->dev,
			 "AFK[ep:%02x]: %s (chan:%u) service call reply mismatch: sent magic 0x%08x group %u cmd %u, got magic 0x%08x group %u cmd %u\n",
			 service->ep->endpoint, service->ops->name,
			 service->channel, EPIC_SERVICE_CALL_MAGIC, group,
			 command, le32_to_cpu(call->magic),
			 le16_to_cpu(call->group), le32_to_cpu(call->command));
		ret = -EINVAL;
		goto out;
	}
	dev_dbg(service->ep->dcp->dev,
		"AFK[ep:%02x]: %s (chan:%u) service call group %u cmd %u ok (reply data_len %u)\n",
		service->ep->endpoint, service->ops->name, service->channel,
		group, command, le32_to_cpu(call->data_len));

	retlen = le32_to_cpu(call->data_len);
	if (output_len < retlen)
		retlen = output_len;
	if (output && output_len) {
		memset(output, 0, output_len);
		memcpy(output, bfr + sizeof(*call), retlen);
	}

out:
	kfree(bfr);
	return ret;
}

int afk_send_command_with_reply_len(struct apple_epic_service *service, u8 type,
		const void *payload, size_t payload_len, void *output,
		size_t output_len, u32 *retcode, size_t *reply_len)
{
	if (!reply_len)
		return -EINVAL;
	return afk_send_command_common(service, type, payload, payload_len,
				       output, output_len, retcode, MSEC_PER_SEC,
				       NULL, reply_len);
}

int afk_service_call_with_reply_len(struct apple_epic_service *service,
		u16 group, u32 command, const void *data, size_t data_len,
		size_t data_pad, void *output, size_t output_len,
		size_t output_pad, size_t *reply_len)
{
	struct epic_service_call *call;
	void *bfr;
	size_t bfr_len, input_size, output_size, received = 0;
	int ret;
	u32 retcode;
	u32 retlen;

	if (!reply_len)
		return -EINVAL;
	*reply_len = 0;
	if (!service)
		return -ENODEV;
	if (check_add_overflow(data_len, data_pad, &input_size) ||
	    check_add_overflow(output_len, output_pad, &output_size) ||
	    check_add_overflow(max(input_size, output_size), sizeof(*call),
			       &bfr_len) || bfr_len > U32_MAX)
		return -EOVERFLOW;

	bfr = kzalloc(bfr_len, GFP_KERNEL);
	if (!bfr)
		return -ENOMEM;

	call = bfr;

	memset(call, 0, sizeof(*call));
	call->group = cpu_to_le16(group);
	call->command = cpu_to_le32(command);
	call->data_len = cpu_to_le32(data_len + data_pad);
	call->magic = cpu_to_le32(EPIC_SERVICE_CALL_MAGIC);

	memcpy(bfr + sizeof(*call), data, data_len);

	ret = afk_send_command_common(service, EPIC_SUBTYPE_STD_SERVICE, bfr,
				      bfr_len, bfr, bfr_len, &retcode,
				      MSEC_PER_SEC, NULL, &received);
	if (ret)
		goto out;
	if (retcode) {
		ret = -EINVAL;
		goto out;
	}
	ret = afk_reply_size_valid(true, received, bfr_len, sizeof(*call));
	if (ret)
		goto out;
	if (le32_to_cpu(call->magic) != EPIC_SERVICE_CALL_MAGIC ||
	    le16_to_cpu(call->group) != group ||
	    le32_to_cpu(call->command) != command) {
		ret = -EINVAL;
		goto out;
	}

	retlen = le32_to_cpu(call->data_len);
	ret = afk_service_body_size_valid(received, sizeof(*call), retlen,
					  output_len);
	if (ret)
		goto out;
	*reply_len = retlen;
	if (output && output_len) {
		memset(output, 0, output_len);
		memcpy(output, bfr + sizeof(*call), retlen);
	}

out:
	kfree(bfr);
	return ret;
}

#if IS_ENABLED(CONFIG_DRM_APPLE_DEBUG)

#define AFK_DEBUGFS_MAX_REPLY 8192

static ssize_t service_call_write_file(struct file *file, const char __user *user_buf,
				       size_t count, loff_t *ppos)
{
	struct apple_epic_service *srv = file->private_data;
	void *buf;
	int ret;
	struct {
		u32 group;
		u32 command;
	} call_info;

	if (count < sizeof(call_info))
		return -EINVAL;
	if (!srv->debugfs.scratch) {
		srv->debugfs.scratch = \
			devm_kzalloc(srv->ep->dcp->dev, AFK_DEBUGFS_MAX_REPLY, GFP_KERNEL);
		if (!srv->debugfs.scratch)
			return -ENOMEM;
	}

	if (copy_from_user(&call_info, user_buf, sizeof(call_info)))
		return -EFAULT;
	user_buf += sizeof(call_info);
	count -= sizeof(call_info);

	buf = kmalloc(count, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	if (copy_from_user(buf, user_buf, count)) {
		kfree(buf);
		return -EFAULT;
	}

	memset(srv->debugfs.scratch, 0, AFK_DEBUGFS_MAX_REPLY);
	dma_mb();

	ret = afk_service_call(srv, call_info.group, call_info.command, buf, count, 0,
			       srv->debugfs.scratch, AFK_DEBUGFS_MAX_REPLY, 0);
	kfree(buf);

	if (ret < 0)
		return ret;

	return count + sizeof(call_info);
}

static ssize_t service_call_read_file(struct file *file, char __user *user_buf,
				      size_t count, loff_t *ppos)
{
	struct apple_epic_service *srv = file->private_data;

	if (!srv->debugfs.scratch)
		return -EINVAL;

	return simple_read_from_buffer(user_buf, count, ppos,
				       srv->debugfs.scratch, AFK_DEBUGFS_MAX_REPLY);
}

static const struct file_operations service_call_fops = {
	.open = simple_open,
	.write = service_call_write_file,
	.read = service_call_read_file,
};

static ssize_t service_raw_call_write_file(struct file *file, const char __user *user_buf,
					   size_t count, loff_t *ppos)
{
	struct apple_epic_service *srv = file->private_data;
	u32 retcode;
	int ret;

	if (!count)
		return 0;
	if (count > AFK_DEBUGFS_MAX_REPLY)
		return -E2BIG;

	if (!srv->debugfs.scratch) {
		srv->debugfs.scratch = \
			devm_kzalloc(srv->ep->dcp->dev, AFK_DEBUGFS_MAX_REPLY, GFP_KERNEL);
		if (!srv->debugfs.scratch)
			return -ENOMEM;
	}

	memset(srv->debugfs.scratch, 0, AFK_DEBUGFS_MAX_REPLY);
	if (copy_from_user(srv->debugfs.scratch, user_buf, count))
		return -EFAULT;

	ret = afk_send_command(srv, EPIC_SUBTYPE_STD_SERVICE, srv->debugfs.scratch, count,
			       srv->debugfs.scratch, AFK_DEBUGFS_MAX_REPLY, &retcode);
	if (ret < 0)
		return ret;
	if (retcode)
		return -EINVAL;

	return count;
}

static ssize_t service_raw_call_read_file(struct file *file, char __user *user_buf,
					  size_t count, loff_t *ppos)
{
	struct apple_epic_service *srv = file->private_data;

	if (!srv->debugfs.scratch)
		return -EINVAL;

	return simple_read_from_buffer(user_buf, count, ppos,
				       srv->debugfs.scratch, AFK_DEBUGFS_MAX_REPLY);
}

static const struct file_operations service_raw_call_fops = {
	.open = simple_open,
	.write = service_raw_call_write_file,
	.read = service_raw_call_read_file,
};

static void afk_populate_service_debugfs(struct apple_epic_service *srv)
{
	if (!srv->ep->debugfs_entry || !srv->ops)
		return;

	if (strcmp(srv->ops->name, "DCPAVAudioInterface") == 0) {
		srv->debugfs.entry = debugfs_create_dir(srv->ops->name,
							srv->ep->debugfs_entry);
		debugfs_create_file("call", 0600, srv->debugfs.entry, srv,
				&service_call_fops);
		debugfs_create_file("raw_call", 0600, srv->debugfs.entry, srv,
				&service_raw_call_fops);
	}
}

static void afk_remove_service_debugfs(struct apple_epic_service *srv)
{
	if (srv->debugfs.entry) {
		debugfs_remove_recursive(srv->debugfs.entry);
		srv->debugfs.entry = NULL;
	}
}

#endif

#if IS_ENABLED(CONFIG_DRM_APPLE_PARSER_KUNIT_TEST)
#include "afk-parser-test.c"
#endif

/* Called after RX and the ordered endpoint worker have been joined. */
bool afk_host_idle(struct apple_dcp_afkep *ep)
{
	unsigned long flags;
	unsigned int i;
	bool idle = true;

	spin_lock_irqsave(&ep->lock, flags);
	if (ep->rxbfr.ready)
		idle = READ_ONCE(*ep->rxbfr.rptr) == READ_ONCE(*ep->rxbfr.wptr);
	if (ep->txbfr.ready)
		idle &= READ_ONCE(*ep->txbfr.rptr) == READ_ONCE(*ep->txbfr.wptr);
	spin_unlock_irqrestore(&ep->lock, flags);
	for (i = 0; i < AFK_MAX_CHANNEL && idle; i++) {
		struct apple_epic_service *service = &ep->services[i];

		spin_lock_irqsave(&service->lock, flags);
		idle = bitmap_empty(service->cmd_map, MAX_PENDING_CMDS);
		spin_unlock_irqrestore(&service->lock, flags);
	}
	return idle;
}
