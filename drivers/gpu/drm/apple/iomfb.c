// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Copyright 2021 Alyssa Rosenzweig */

#include <linux/align.h>
#include <linux/bitfield.h>
#include <linux/bitmap.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/iommu.h>
#include <linux/kref.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/ratelimit.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/soc/apple/dp-tunnel.h>
#include <linux/soc/apple/rtkit.h>
#include <linux/unaligned.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_edid.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_modeset_lock.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_vblank.h>

#include "dcp.h"
#include "dcp-internal.h"
#include "iomfb.h"
#include "iomfb_internal.h"
#include "parser.h"
#include "trace.h"

static int dcp_tx_offset(enum dcp_context_id id)
{
	switch (id) {
	case DCP_CONTEXT_CB:
	case DCP_CONTEXT_CMD:
		return 0x00000;
	case DCP_CONTEXT_OOBCB:
	case DCP_CONTEXT_OOBCMD:
		return 0x08000;
	default:
		return -EINVAL;
	}
}

static int dcp_channel_offset(enum dcp_context_id id)
{
	switch (id) {
	case DCP_CONTEXT_ASYNC:
		return 0x40000;
	case DCP_CONTEXT_OOBASYNC:
		return 0x48000;
	case DCP_CONTEXT_CB:
		return 0x60000;
	case DCP_CONTEXT_OOBCB:
		return 0x68000;
	default:
		return dcp_tx_offset(id);
	}
}

static inline u64 dcpep_set_shmem(u64 dart_va)
{
	return FIELD_PREP(IOMFB_MESSAGE_TYPE, IOMFB_MESSAGE_TYPE_SET_SHMEM) |
	       FIELD_PREP(IOMFB_SHMEM_FLAG, IOMFB_SHMEM_FLAG_VALUE) |
	       FIELD_PREP(IOMFB_SHMEM_DVA, dart_va);
}

static inline u64 dcpep_msg(enum dcp_context_id id, u32 length, u16 offset)
{
	return FIELD_PREP(IOMFB_MESSAGE_TYPE, IOMFB_MESSAGE_TYPE_MSG) |
		FIELD_PREP(IOMFB_MSG_CONTEXT, id) |
		FIELD_PREP(IOMFB_MSG_OFFSET, offset) |
		FIELD_PREP(IOMFB_MSG_LENGTH, length);
}

static inline u64 dcpep_ack(enum dcp_context_id id)
{
	return dcpep_msg(id, 0, 0) | IOMFB_MSG_ACK;
}

/*
 * A channel is busy if we have sent a message that has yet to be
 * acked. The driver must not sent a message to a busy channel.
 */
static bool dcp_channel_busy(struct dcp_channel *ch)
{
	return (ch->depth != 0);
}

/*
 * Get the context ID passed to the DCP for a command we push. The rule is
 * simple: callback contexts are used when replying to the DCP, command
 * contexts are used otherwise. That corresponds to a non/zero call stack
 * depth. This rule frees the caller from tracking the call context manually.
 */
static enum dcp_context_id dcp_call_context(struct apple_dcp *dcp, bool oob)
{
	u8 depth = oob ? dcp->ch_oobcmd.depth : dcp->ch_cmd.depth;

	if (depth)
		return oob ? DCP_CONTEXT_OOBCB : DCP_CONTEXT_CB;
	else
		return oob ? DCP_CONTEXT_OOBCMD : DCP_CONTEXT_CMD;
}

/* Get a channel for a context */
static struct dcp_channel *dcp_get_channel(struct apple_dcp *dcp,
					   enum dcp_context_id context)
{
	switch (context) {
	case DCP_CONTEXT_CB:
		return &dcp->ch_cb;
	case DCP_CONTEXT_CMD:
		return &dcp->ch_cmd;
	case DCP_CONTEXT_OOBCB:
		return &dcp->ch_oobcb;
	case DCP_CONTEXT_OOBCMD:
		return &dcp->ch_oobcmd;
	case DCP_CONTEXT_ASYNC:
		return &dcp->ch_async;
	case DCP_CONTEXT_OOBASYNC:
		return &dcp->ch_oobasync;
	default:
		return NULL;
	}
}

/* Get the start of a packet: after the end of the previous packet */
static u16 dcp_packet_start(struct dcp_channel *ch, u8 depth)
{
	if (depth > 0)
		return ch->end[depth - 1];
	else
		return 0;
}

/* Pushes and pops the depth of the call stack with safety checks */
static u8 dcp_push_depth(u8 *depth)
{
	u8 ret = (*depth)++;

	WARN_ON(ret >= DCP_MAX_CALL_DEPTH);
	return ret;
}

static u8 dcp_pop_depth(u8 *depth)
{
	WARN_ON((*depth) == 0);

	return --(*depth);
}

/* Older firmware and the H17G method profile retain their existing transport. */
static bool iomfb_uses_queue(struct apple_dcp *dcp)
{
	return dcp->fw_compat == DCP_FIRMWARE_H17P &&
	       dcp->hw.iomfb_method_profile != DCP_IOMFB_METHODS_H17G;
}

static bool iomfb_channels_idle(struct apple_dcp *dcp)
{
	return !dcp->ch_cmd.depth && !dcp->ch_cb.depth &&
	       !dcp->ch_oobcmd.depth && !dcp->ch_oobcb.depth &&
	       !dcp->ch_async.depth && !dcp->ch_oobasync.depth;
}

enum iomfb_opaque_x_state {
	IOMFB_OPAQUE_X_WAITING,
	IOMFB_OPAQUE_X_NEEDED,
	IOMFB_OPAQUE_X_QUEUED,
	IOMFB_OPAQUE_X_READY,
};

struct iomfb_opaque_x_transaction {
	struct iomfb_transaction transaction;
};

static void iomfb_opaque_x_start(struct apple_dcp *dcp,
				 struct iomfb_transaction *transaction);
static void iomfb_opaque_x_release(struct iomfb_transaction *transaction);

static int iomfb_enqueue_opaque_x(struct apple_dcp *dcp)
{
	struct iomfb_opaque_x_transaction *opaque;

	lockdep_assert_held(&dcp->iomfb.lock);
	/* DCPEXT rejects enableGPAlphaDiv; retain the panel-only workaround. */
	if (dcp->hw.iomfb_method_profile ==
	    DCP_IOMFB_METHODS_H17P_EXTERNAL)
		return 0;
	if (atomic_read(&dcp->iomfb.opaque_x_state) != IOMFB_OPAQUE_X_NEEDED)
		return 0;
	if (dcp->iomfb.queued >= 32)
		return -EBUSY;
	opaque = kzalloc_obj(*opaque);
	if (!opaque)
		return -ENOMEM;
	if (atomic_cmpxchg(&dcp->iomfb.opaque_x_state, IOMFB_OPAQUE_X_NEEDED,
			   IOMFB_OPAQUE_X_QUEUED) != IOMFB_OPAQUE_X_NEEDED) {
		kfree(opaque);
		return 0;
	}

	opaque->transaction.start = iomfb_opaque_x_start;
	opaque->transaction.release = iomfb_opaque_x_release;
	dcp->iomfb.queued++;
	list_add(&opaque->transaction.link, &dcp->iomfb.pending);
	return 0;
}

static void iomfb_discard_pending(struct apple_dcp *dcp)
{
	struct iomfb_transaction *transaction, *next;

	lockdep_assert_held(&dcp->iomfb.lock);
	list_for_each_entry_safe(transaction, next, &dcp->iomfb.pending, link) {
		list_del(&transaction->link);
		dcp->iomfb.queued--;
		if (transaction->brightness_only)
			dcp->iomfb.backlight_queued = false;
		transaction->release(transaction);
	}
}

static void iomfb_queue_advance(struct apple_dcp *dcp)
{
	struct iomfb_transaction *transaction = dcp->iomfb.active;

	lockdep_assert_held(&dcp->iomfb.lock);
	if (READ_ONCE(dcp->crashed) || dcp->iomfb.stopped) {
		iomfb_discard_pending(dcp);
		/* The active operation may still be visible to firmware. */
		return;
	}
	if (!iomfb_channels_idle(dcp) || dcp->present_state_h17p.pending)
		return;

	if (transaction) {
		dcp->iomfb.active = NULL;
		cancel_delayed_work(&dcp->iomfb.timeout);
		if (transaction->brightness_only)
			dcp->iomfb.backlight_queued = false;
		/* A new level requested during a successful present stays pending. */
		if (dcp_backlight_pending(dcp)) {
			if (transaction->completed) {
				schedule_work(&dcp->bl_update_wq);
			} else if (transaction->backlight_failed) {
				unsigned int delay = dcp_backlight_retry_delay(dcp);

				if (delay)
					mod_delayed_work(system_wq, &dcp->iomfb.backlight_retry,
							 msecs_to_jiffies(delay));
				else
					dev_warn_ratelimited(dcp->dev,
							     "backlight retry limit reached\n");
			}
		}
		transaction->release(transaction);
	}
	if (iomfb_enqueue_opaque_x(dcp)) {
		WRITE_ONCE(dcp->crashed, true);
		iomfb_discard_pending(dcp);
		schedule_work(&dcp->vblank_wq);
		return;
	}
	if (!list_empty(&dcp->iomfb.pending))
		schedule_work(&dcp->iomfb.work);
}

static void iomfb_queue_work(struct work_struct *work)
{
	struct apple_dcp *dcp = container_of(work, struct apple_dcp, iomfb.work);
	struct iomfb_transaction *transaction;

	mutex_lock(&dcp->iomfb.lock);
	if (READ_ONCE(dcp->crashed) || dcp->iomfb.stopped) {
		iomfb_discard_pending(dcp);
		goto unlock;
	}
	if (dcp->iomfb.active || !iomfb_channels_idle(dcp) ||
	    dcp->present_state_h17p.pending || list_empty(&dcp->iomfb.pending))
		goto unlock;

	transaction = list_first_entry(&dcp->iomfb.pending,
				       struct iomfb_transaction, link);
	list_del(&transaction->link);
	dcp->iomfb.queued--;
	dcp->iomfb.active = transaction;
	WRITE_ONCE(dcp->iomfb.owner, current);
	dcp->iomfb.deadline = jiffies + msecs_to_jiffies(10000);
	mod_delayed_work(system_wq, &dcp->iomfb.timeout, msecs_to_jiffies(10000));
	transaction->start(dcp, transaction);
	WRITE_ONCE(dcp->iomfb.owner, NULL);
	iomfb_queue_advance(dcp);
unlock:
	mutex_unlock(&dcp->iomfb.lock);
}

static void iomfb_queue_timeout(struct work_struct *work)
{
	struct apple_dcp *dcp = container_of(to_delayed_work(work),
					  struct apple_dcp, iomfb.timeout);

	mutex_lock(&dcp->iomfb.lock);
	if (dcp->iomfb.active && time_before(jiffies, dcp->iomfb.deadline)) {
		mod_delayed_work(system_wq, &dcp->iomfb.timeout,
				 dcp->iomfb.deadline - jiffies);
	} else if (dcp->iomfb.active) {
		WRITE_ONCE(dcp->crashed, true);
		dev_err(dcp->dev,
			"IOMFB transaction timed out: cmd=%u oob=%u present=%u accepted=%u swap=%u\n",
			dcp->ch_cmd.depth, dcp->ch_oobcmd.depth,
			dcp->present_state_h17p.pending, dcp->present_state_h17p.accepted,
			dcp->present_state_h17p.swap_id);
		if (dcp->ch_cmd.depth && dcp->ch_cmd.depth <= DCP_MAX_CALL_DEPTH) {
			const struct dcp_packet_header *h =
				&dcp->ch_cmd.header[dcp->ch_cmd.depth - 1];

			dev_err(dcp->dev, "pending IOMFB %c%c%c%c in=%u out=%u\n",
				h->tag[3], h->tag[2], h->tag[1], h->tag[0],
				h->in_len, h->out_len);
		}
		iomfb_discard_pending(dcp);
		schedule_work(&dcp->vblank_wq);
	}
	mutex_unlock(&dcp->iomfb.lock);
}

static void iomfb_opaque_x_complete(struct apple_dcp *dcp, void *out,
				    void *cookie)
{
	u32 status = out ? *(u32 *)out : ~0U;

	(void)cookie;

	if (status) {
		dev_err(dcp->dev, "opaque X property failed: %u\n", status);
		WRITE_ONCE(dcp->crashed, true);
		schedule_work(&dcp->vblank_wq);
		return;
	}

	atomic_set(&dcp->iomfb.opaque_x_state, IOMFB_OPAQUE_X_READY);
}

static void iomfb_opaque_x_start(struct apple_dcp *dcp,
				 struct iomfb_transaction *transaction)
{
	(void)transaction;
	iomfb_apply_opaque_x_h17p(dcp, iomfb_opaque_x_complete, NULL);
}

static void iomfb_opaque_x_release(struct iomfb_transaction *transaction)
{
	kfree(container_of(transaction, struct iomfb_opaque_x_transaction,
			   transaction));
}

void iomfb_opaque_x_reset_h17p(struct apple_dcp *dcp)
{
	atomic_set(&dcp->iomfb.opaque_x_state, IOMFB_OPAQUE_X_WAITING);
}

static void iomfb_backlight_retry(struct work_struct *work)
{
	struct apple_dcp *dcp = container_of(to_delayed_work(work), struct apple_dcp,
					  iomfb.backlight_retry);

	if (!READ_ONCE(dcp->crashed) && !READ_ONCE(dcp->iomfb.stopped) &&
	    dcp_backlight_pending(dcp))
		schedule_work(&dcp->bl_update_wq);
}

void iomfb_queue_init(struct apple_dcp *dcp)
{
	mutex_init(&dcp->iomfb.lock);
	INIT_LIST_HEAD(&dcp->iomfb.pending);
	INIT_WORK(&dcp->iomfb.work, iomfb_queue_work);
	INIT_DELAYED_WORK(&dcp->iomfb.timeout, iomfb_queue_timeout);
	INIT_DELAYED_WORK(&dcp->iomfb.backlight_retry, iomfb_backlight_retry);
	atomic_set(&dcp->iomfb.opaque_x_state, IOMFB_OPAQUE_X_WAITING);
}

void iomfb_queue_stop(struct apple_dcp *dcp)
{
	if (!iomfb_uses_queue(dcp))
		return;

	mutex_lock(&dcp->iomfb.lock);
	dcp->iomfb.stopped = true;
	iomfb_discard_pending(dcp);
	mutex_unlock(&dcp->iomfb.lock);
	cancel_work_sync(&dcp->iomfb.work);
	cancel_delayed_work_sync(&dcp->iomfb.timeout);
	cancel_delayed_work_sync(&dcp->iomfb.backlight_retry);
}

/*
 * Wait until every queued transaction, including a present's completion, has
 * finished.  A stopped or crashed queue has nothing left to deliver.
 */
bool iomfb_queue_drain(struct apple_dcp *dcp, unsigned long timeout)
{
	unsigned long deadline = jiffies + timeout;
	bool idle;

	if (!iomfb_uses_queue(dcp))
		return true;

	for (;;) {
		mutex_lock(&dcp->iomfb.lock);
		idle = READ_ONCE(dcp->crashed) || dcp->iomfb.stopped ||
		       (!dcp->iomfb.active && list_empty(&dcp->iomfb.pending) &&
			!dcp->present_state_h17p.pending);
		mutex_unlock(&dcp->iomfb.lock);
		if (idle)
			return true;
		if (time_after(jiffies, deadline))
			return false;
		usleep_range(2000, 4000);
	}
}

/* Admission is independent of transport: replies and control RPCs remain live. */
void iomfb_pause_presentation(struct apple_dcp *dcp, bool pause)
{
	guard(mutex)(&dcp->iomfb.lock);
	dcp->iomfb.presentation_paused = pause;
	if (!pause && dcp->external_hotplug_pending) {
		unsigned int action;

		dcp->external_hotplug_pending = false;
		action = dcp_mode_hotplug(&dcp->mode_state, dcp->external_hotplug_connected,
					  dcp->connector ? &dcp->connector->connected : NULL);
		dcp_handle_hotplug_actions(dcp, action);
	}
}

/* A dead transport is not proof that DMA or a pending present has stopped. */
int iomfb_queue_settle(struct apple_dcp *dcp, unsigned long timeout)
{
	unsigned long deadline = jiffies + timeout;
	bool idle;

	for (;;) {
		mutex_lock(&dcp->iomfb.lock);
		if (READ_ONCE(dcp->crashed) || dcp->iomfb.stopped) {
			mutex_unlock(&dcp->iomfb.lock);
			return -EIO;
		}
		idle = !dcp->iomfb.active && !dcp->iomfb.queued &&
		       list_empty(&dcp->iomfb.pending) &&
		       !dcp->present_state_h17p.pending && iomfb_channels_idle(dcp);
		mutex_unlock(&dcp->iomfb.lock);
		if (idle)
			return 0;
		if (time_after_eq(jiffies, deadline))
			return -ETIMEDOUT;
		usleep_range(2000, 4000);
	}
}

int iomfb_queue(struct apple_dcp *dcp, struct iomfb_transaction *transaction)
{
	int ret = 0;

	mutex_lock(&dcp->iomfb.lock);
	if (READ_ONCE(dcp->crashed) || dcp->iomfb.stopped) {
		ret = -EIO;
	} else if (transaction->presentation && dcp->iomfb.presentation_paused) {
		ret = -EAGAIN;
	} else if (transaction->brightness_only && dcp->iomfb.backlight_queued) {
		ret = -EALREADY;
	} else if (dcp->iomfb.queued >= 32) {
		ret = -EBUSY;
	} else {
		if (transaction->brightness_only)
			dcp->iomfb.backlight_queued = true;
		dcp->iomfb.queued++;
		list_add_tail(&transaction->link, &dcp->iomfb.pending);
		schedule_work(&dcp->iomfb.work);
	}
	mutex_unlock(&dcp->iomfb.lock);
	return ret;
}

struct iomfb_crc_transaction {
	struct iomfb_transaction transaction;
	u32 swap_id;
};

static void iomfb_crc_complete(struct apple_dcp *dcp, void *out, void *cookie)
{
	struct iomfb_crc_transaction *crc = cookie;
	struct dcp_packet_header *header;
	u32 value;

	/* A106 has no input and a single little-endian blend-output CRC. */
	if (!out)
		goto invalid;
	header = out - sizeof(*header);
	if (header->in_len || header->out_len != sizeof(__le32))
		goto invalid;
	value = get_unaligned_le32(out);
	if (READ_ONCE(dcp->crc_enabled))
		drm_crtc_add_crc_entry(&dcp->crtc->base, true, crc->swap_id, &value);
	return;

invalid:
	dev_err(dcp->dev, "invalid blend CRC response\n");
	WRITE_ONCE(dcp->crashed, true);
}

static void iomfb_crc_start(struct apple_dcp *dcp,
			    struct iomfb_transaction *transaction)
{
	static const struct dcp_method_entry method = {
		.tag = { 'A', '1', '0', '6' },
		.name = "read_blend_crc",
	};
	struct iomfb_crc_transaction *crc = container_of(transaction,
						       struct iomfb_crc_transaction,
						       transaction);

	if (READ_ONCE(dcp->crc_enabled))
		dcp_push(dcp, false, &method, 0, sizeof(__le32), NULL,
			 iomfb_crc_complete, crc);
}

static void iomfb_crc_release(struct iomfb_transaction *transaction)
{
	kfree(container_of(transaction, struct iomfb_crc_transaction, transaction));
}

void iomfb_queue_crc_h17p(struct apple_dcp *dcp, u32 swap_id)
{
	struct iomfb_crc_transaction *crc;

	lockdep_assert_held(&dcp->iomfb.lock);
	if (!READ_ONCE(dcp->crc_enabled) || READ_ONCE(dcp->crashed) ||
	    dcp->iomfb.stopped || dcp->iomfb.presentation_paused || dcp->iomfb.queued >= 32)
		return;
	crc = kzalloc_obj(*crc);
	if (!crc)
		return;
	crc->swap_id = swap_id;
	crc->transaction.start = iomfb_crc_start;
	crc->transaction.release = iomfb_crc_release;
	/* Read the completed frame before another queued present changes it. */
	list_add(&crc->transaction.link, &dcp->iomfb.pending);
	dcp->iomfb.queued++;
	/* The receiver advances the queue only after acknowledging completion. */
}

struct iomfb_command {
	struct iomfb_transaction transaction;
	struct dcp_method_entry method;
	dcp_callback_t callback;
	void *cookie;
	u32 in_len;
	u32 out_len;
	bool oob;
	u8 data[];
};

static void iomfb_command_start(struct apple_dcp *dcp,
				struct iomfb_transaction *transaction)
{
	struct iomfb_command *command = container_of(transaction,
						    struct iomfb_command, transaction);

	dcp_push(dcp, command->oob, &command->method, command->in_len,
		 command->out_len, command->data, command->callback, command->cookie);
}

static void iomfb_command_release(struct iomfb_transaction *transaction)
{
	kfree(container_of(transaction, struct iomfb_command, transaction));
}

static int iomfb_queue_command(struct apple_dcp *dcp, bool oob,
			       const struct dcp_method_entry *method,
			       u32 in_len, u32 out_len, void *data,
			       dcp_callback_t callback, void *cookie)
{
	struct iomfb_command *command;
	int ret;

	if ((u64)sizeof(struct dcp_packet_header) + in_len + out_len > 0x8000)
		return -EMSGSIZE;
	command = kzalloc(struct_size(command, data, in_len), GFP_KERNEL);
	if (!command)
		return -ENOMEM;
	command->transaction.start = iomfb_command_start;
	command->transaction.release = iomfb_command_release;
	command->method = *method;
	command->callback = callback;
	command->cookie = cookie;
	command->in_len = in_len;
	command->out_len = out_len;
	command->oob = oob;
	if (in_len)
		memcpy(command->data, data, in_len);
	ret = iomfb_queue(dcp, &command->transaction);
	if (ret)
		iomfb_command_release(&command->transaction);
	return ret;
}


/* Call a DCP function given by a tag */
void dcp_push(struct apple_dcp *dcp, bool oob, const struct dcp_method_entry *call,
		     u32 in_len, u32 out_len, void *data, dcp_callback_t cb,
		     void *cookie)
{
	struct dcp_method_entry resolved = *call;

	/* Reply chains stay on the serialized receiver; callers enqueue copies. */
	if (iomfb_uses_queue(dcp) && READ_ONCE(dcp->iomfb.owner) != current) {
		if (iomfb_queue_command(dcp, oob, call, in_len, out_len,
					data, cb, cookie))
			WRITE_ONCE(dcp->crashed, true);
		return;
	}

	if (dcp->fw_compat == DCP_FIRMWARE_H17P && READ_ONCE(dcp->crashed))
		return;

	if (dcp->hw.iomfb_method_profile == DCP_IOMFB_METHODS_H17G &&
	    call->tag_h17g[0])
		memcpy(resolved.tag, call->tag_h17g, sizeof(resolved.tag));
	call = &resolved;

	enum dcp_context_id context = dcp_call_context(dcp, oob);
	struct dcp_channel *ch = dcp_get_channel(dcp, context);

	struct dcp_packet_header header = {
		.in_len = in_len,
		.out_len = out_len,

		/* Tag is reversed due to endianness of the fourcc */
		.tag[0] = call->tag[3],
		.tag[1] = call->tag[2],
		.tag[2] = call->tag[1],
		.tag[3] = call->tag[0],
	};

	u8 depth;
	u16 offset;

	if (iomfb_uses_queue(dcp) &&
	    (!ch || ch->depth >= DCP_MAX_CALL_DEPTH ||
	     (u64)sizeof(header) + in_len + out_len > 0x8000 ||
	     dcp_packet_start(ch, ch->depth) >
		0x8000 - ALIGN(sizeof(header) + in_len + out_len,
			       DCP_PACKET_ALIGNMENT))) {
		dev_err(dcp->dev, "invalid IOMFB command envelope\n");
		WRITE_ONCE(dcp->crashed, true);
		return;
	}
	depth = dcp_push_depth(&ch->depth);
	offset = dcp_packet_start(ch, depth);

	void *out = dcp->shmem + dcp_tx_offset(context) + offset;
	void *out_data = out + sizeof(header);
	size_t data_len = sizeof(header) + in_len + out_len;

	memcpy(out, &header, sizeof(header));

	if (in_len > 0)
		memcpy(out_data, data, in_len);
	/*
	 * H17P: an unwritten status must not look like a successful
	 * response.
	 */
	if (out_len && dcp->fw_compat == DCP_FIRMWARE_H17P)
		memset(out_data + in_len, 0xff, out_len);

	trace_iomfb_push(dcp, call, context, offset, depth);

	ch->callbacks[depth] = cb;
	ch->cookies[depth] = cookie;
	ch->output[depth] = out + sizeof(header) + in_len;
	ch->header[depth] = header;
	ch->end[depth] = offset + ALIGN(data_len, DCP_PACKET_ALIGNMENT);

	dcp_send_message(dcp, IOMFB_ENDPOINT,
			 dcpep_msg(context, data_len, offset));
}

/* Parse a callback tag "D123" into the ID 123. Returns -EINVAL on failure. */
int dcp_parse_tag(char tag[4])
{
	u32 d[3];
	int i;

	if (tag[3] != 'D')
		return -EINVAL;

	for (i = 0; i < 3; ++i) {
		d[i] = (u32)(tag[i] - '0');

		if (d[i] > 9)
			return -EINVAL;
	}

	return d[0] + (d[1] * 10) + (d[2] * 100);
}

/* Ack a callback from the DCP */
void dcp_ack(struct apple_dcp *dcp, enum dcp_context_id context)
{
	struct dcp_channel *ch = dcp_get_channel(dcp, context);

	dcp_pop_depth(&ch->depth);
	dcp_send_message(dcp, IOMFB_ENDPOINT,
			 dcpep_ack(context));
}

/*
 * Helper to send a DRM hotplug event. The DCP is accessed from a single
 * (RTKit) thread. To handle hotplug callbacks, we need to call
 * drm_kms_helper_hotplug_event, which does an atomic commit (via DCP) and
 * waits for vblank (a DCP callback). That means we deadlock if we call from
 * the RTKit thread! Instead, move the call to another thread via a workqueue.
 */
bool dcp_crtc_can_retrain(struct drm_crtc *crtc, struct apple_connector *connector)
{
	struct platform_device *pdev = READ_ONCE(connector->dcp);
	struct apple_dcp_typec_route *route;
	struct apple_dcp *dcp;

	if (!connector->port_encoder)
		return true;
	if (!pdev)
		return false;
	dcp = platform_get_drvdata(pdev);
	route = READ_ONCE(dcp->active_typec_route);
	if (!dcp_typec_follows_crtc(dcp) || (route && READ_ONCE(route->tunnel)))
		return true;
	/* Rollback HPD must not retry the same failed crossing from another reset. */
	return to_apple_crtc(crtc)->dcp == pdev;
}

static int dcp_retrain_active_crtc(struct apple_connector *connector)
{
	struct drm_device *dev = connector->base.dev;
	struct drm_modeset_acquire_ctx ctx;
	struct drm_crtc *crtc;
	int ret;

	DRM_MODESET_LOCK_ALL_BEGIN(dev, ctx, 0, ret);

	crtc = connector->base.state ? connector->base.state->crtc : NULL;
	if (crtc && crtc->state && crtc->state->active &&
	    dcp_crtc_can_retrain(crtc, connector))
		ret = drm_atomic_helper_reset_crtc(crtc, &ctx);
	else
		ret = 0;

	DRM_MODESET_LOCK_ALL_END(dev, ctx, ret);

	return ret;
}

void dcp_handle_hotplug_actions(struct apple_dcp *dcp, unsigned int action)
{
	if (action & DCP_HOTPLUG_VBLANK)
		schedule_work(&dcp->vblank_wq);
	if ((action & DCP_HOTPLUG_NOTIFY) && dcp->connector)
		dcp_queue_hotplug(dcp->connector);
}

void dcp_retrain_oob(struct apple_connector *connector)
{
	struct platform_device *pdev = READ_ONCE(connector->dcp);
	struct apple_dcp *dcp;

	/* a Type-C port with no pipeline behind it has nothing to retrain */
	if (!pdev || !READ_ONCE(connector->connected))
		return;
	dcp = platform_get_drvdata(pdev);

	/*
	 * Bringing up another high-speed Type-C route can disturb an active DPTX
	 * stream without changing its HPD state.  Invalidate the IOMFB mode and
	 * use the normal hotplug worker to replay the active CRTC from process
	 * context; sending a synthetic disconnect would tear down the connector.
	 */
	dcp_mode_invalidate(&dcp->mode_state);
	dcp_queue_hotplug(connector);
}

static void dcp_notify_hotplug(struct apple_connector *connector)
{
	/* Same-state retrains also need a notification after refreshing status. */
	if (!(connector->base.polled & DRM_CONNECTOR_POLL_HPD) ||
	    !drm_connector_helper_hpd_irq_event(&connector->base))
		drm_kms_helper_connector_hotplug_event(&connector->base);
}

enum dcp_hotplug_reason {
	DCP_HOTPLUG_READY = BIT(0),
	DCP_HOTPLUG_ROUTE_FAILURE = BIT(1),
};

void dcp_queue_hotplug(struct apple_connector *connector)
{
	atomic_or(DCP_HOTPLUG_READY, &connector->hotplug_reasons);
	schedule_work(&connector->hotplug_wq);
}

void dcp_route_failure_notify(struct apple_connector *connector)
{
	/* A failed reset queues notification only; real readiness wins coalescing. */
	atomic_or(DCP_HOTPLUG_ROUTE_FAILURE, &connector->hotplug_reasons);
	schedule_work(&connector->hotplug_wq);
}


/* Does @connector belong to a DRM device of H17-generation pipelines? */
static bool dcp_connector_h17p(struct apple_connector *connector)
{
	struct platform_device *pdev = READ_ONCE(connector->dcp);
	struct drm_crtc *crtc;

	if (pdev)
		return ((struct apple_dcp *)platform_get_drvdata(pdev))->fw_compat ==
			DCP_FIRMWARE_H17P;
	drm_for_each_crtc(crtc, connector->base.dev) {
		struct apple_dcp *dcp = platform_get_drvdata(to_apple_crtc(crtc)->dcp);

		if (dcp && dcp->fw_compat == DCP_FIRMWARE_H17P)
			return true;
	}
	return false;
}

/*
 * The cached EDID itself is owned under edid_lock, which integration's
 * route and sink changes take to drop it (apple_connector_invalidate_edid()).
 */
static void dcp_clear_connector_edid(struct apple_connector *connector)
{
	lockdep_assert_held(&connector->base.dev->mode_config.mutex);
	drm_edid_connector_update(&connector->base, NULL);
	apple_connector_invalidate_edid(connector);
	connector->drm_edid_firmware_derived = false;
}

static void dcp_consume_edid_invalidation(struct apple_connector *connector)
{
	lockdep_assert_held(&connector->base.dev->mode_config.mutex);
	/* Preserve a disconnect even if the port reconnected before probing. */
	if (xchg(&connector->edid_invalidation_pending, false))
		dcp_clear_connector_edid(connector);
}

/* H17P (T8140, T8142): aurora-wip's hotplug work. */
static void dcp_hotplug_h17p(struct apple_connector *connector)
{
	struct platform_device *pdev;
	struct apple_dcp *dcp;
	int ret;

	scoped_guard(mutex, &connector->base.dev->mode_config.mutex)
		dcp_consume_edid_invalidation(connector);

	pdev = READ_ONCE(connector->dcp);
	if (!pdev) {	/* a Type-C port unrouted after this was queued */
		scoped_guard(mutex, &connector->base.dev->mode_config.mutex) {
			if (!READ_ONCE(connector->dcp) && connector->drm_edid_firmware_derived) {
				drm_edid_connector_update(&connector->base, NULL);
				connector->drm_edid_firmware_derived = false;
			}
		}
		drm_kms_helper_connector_hotplug_event(&connector->base);
		return;
	}
	dcp = platform_get_drvdata(pdev);
	dev_info(dcp->dev, "%s() connected:%d valid_mode:%d nr_modes:%u\n", __func__,
		 connector->connected, READ_ONCE(dcp->mode_state.valid), dcp->nr_modes);

	scoped_guard(mutex, &connector->base.dev->mode_config.mutex) {
		if (READ_ONCE(connector->dcp) == pdev)
			dcp_update_connector_dimensions(dcp, &connector->base);
		if (!READ_ONCE(connector->connected))
			dcp_clear_connector_edid(connector);
	}

	/*
	 * DCP defers link training until we set a display mode. But we set
	 * display modes from atomic_flush, so userspace needs to trigger a
	 * flush, or the CRTC gets no signal.
	 */
	if (connector->base.state && !READ_ONCE(dcp->mode_state.valid) && connector->connected) {
		drm_connector_set_link_status_property(&connector->base,
						       DRM_MODE_LINK_STATUS_BAD);

		/*
		 * A short Type-C route interruption can leave the DRM CRTC active
		 * while DCP has discarded its display mode.  Userspace is then free
		 * to keep submitting plane-only commits, none of which retrains the
		 * link.  Re-apply the active CRTC state once the DPTX link-config
		 * callback has completed.
		 */
		ret = dcp_retrain_active_crtc(connector);
		if (ret)
			dev_warn(dcp->dev,
				 "failed to retrain active CRTC after hotplug: %d\n",
				 ret);
	}

	drm_kms_helper_connector_hotplug_event(&connector->base);
}

void dcp_hotplug(struct work_struct *work)
{
	struct apple_connector *connector;
	struct platform_device *pdev;
	struct apple_dcp *dcp;
	bool notify_only;
	unsigned int reasons;
	int ret;

	connector = container_of(work, struct apple_connector, hotplug_wq);
	if (dcp_connector_h17p(connector)) {
		dcp_hotplug_h17p(connector);
		return;
	}
	reasons = atomic_xchg(&connector->hotplug_reasons, 0);
	notify_only = (reasons & DCP_HOTPLUG_ROUTE_FAILURE) &&
		      !(reasons & DCP_HOTPLUG_READY);

	pdev = READ_ONCE(connector->dcp);
	if (!pdev) {	/* a Type-C port unrouted after this was queued */
		apple_connector_invalidate_edid(connector);
		dcp_notify_hotplug(connector);
		apple_connector_backlight_sync(connector);
		return;
	}
	dcp = platform_get_drvdata(pdev);
	dev_info(dcp->dev, "%s() connected:%d valid_mode:%d nr_modes:%u\n", __func__,
		 connector->connected, READ_ONCE(dcp->mode_state.valid), dcp->nr_modes);

	if (!connector->connected)
		apple_connector_invalidate_edid(connector);

	/*
	 * DCP defers link training until we set a display mode. But we set
	 * display modes from atomic_flush, so userspace needs to trigger a
	 * flush, or the CRTC gets no signal.
	 */
	/*
	 * Not for a Type-C CRTC powered off with the display attached: it has
	 * no link, and dcp_poweron() retrains one on wake. DCP tears its AV
	 * service down at power off, which lands here; a link reported bad
	 * then makes wlroots replace the output on wake, which powers the
	 * CRTC off and on again, and that wake does the same, forever.
	 */
	if (!notify_only && connector->base.state &&
	    !READ_ONCE(dcp->mode_state.valid) && connector->connected &&
	    !READ_ONCE(dcp->typec_crtc_off) &&
	    !(dcp_uses_t6020_tunnel_flow(dcp))) {
		drm_connector_set_link_status_property(&connector->base,
						       DRM_MODE_LINK_STATUS_BAD);

		/*
		 * A short Type-C route interruption can leave the DRM CRTC active
		 * while DCP has discarded its display mode.  Userspace is then free
		 * to keep submitting plane-only commits, none of which retrains the
		 * link.  Re-apply the active CRTC state once the DPTX link-config
		 * callback has completed.
		 */
		ret = dcp_retrain_active_crtc(connector);
		if (ret)
			dev_warn(dcp->dev,
				 "failed to retrain active CRTC after hotplug: %d\n",
				 ret);
	}

	dcp_notify_hotplug(connector);
	/* after the event, so registering a backlight cannot delay it */
	apple_connector_backlight_sync(connector);
}

/*
 * J700/25G83 asks D134 after publishing its external video interface.
 * The observed async record has no input and a four-byte result. Use the
 * existing zero reply for this record only; its feature remains unsupported.
 */
static bool dcp_external_zero_reply(struct apple_dcp *dcp, int tag,
				    enum dcp_context_id context,
				    u32 in_len, u32 out_len)
{
	return tag == 134 && context == DCP_CONTEXT_ASYNC &&
	       in_len == 0 && out_len == sizeof(u32) &&
	       dcp->hw.iomfb_method_profile ==
	       DCP_IOMFB_METHODS_H17P_EXTERNAL &&
	       of_machine_is_compatible("apple,j700") &&
	       of_device_is_compatible(dcp->dev->of_node, "apple,t8140-dcpext");
}

static void dcpep_handle_cb_h17p(struct apple_dcp *dcp, enum dcp_context_id context,
				 void *data, u32 length, u16 offset)
{
	struct device *dev = dcp->dev;
	struct dcp_packet_header *hdr = data;
	void *in, *out;
	int tag;
	struct dcp_channel *ch = dcp_get_channel(dcp, context);
	u8 depth;
	bool handled, zero_reply;

	if (dcp->fw_compat == DCP_FIRMWARE_H17P &&
	    (length < sizeof(*hdr) || !ch || ch->depth >= DCP_MAX_CALL_DEPTH ||
	     offset >= 0x8000 || length > 0x8000 - offset)) {
		dev_err(dev, "invalid IOMFB callback envelope\n");
		dcp->crashed = true;
		return;
	}

	tag = dcp_parse_tag(hdr->tag);


	handled = tag >= 0 && tag < IOMFB_MAX_CB && dcp->cb_handlers &&
		  dcp->cb_handlers[tag];
	if (dcp->fw_compat == DCP_FIRMWARE_H17P &&
	    (tag < 0 || tag >= IOMFB_MAX_CB ||
	     (u64)sizeof(*hdr) + hdr->in_len + hdr->out_len != length ||
	     (dcp->hw.iomfb_method_profile != DCP_IOMFB_METHODS_H17G &&
	      handled &&
	      !iomfb_check_callback_h17p(dcp, tag, hdr->in_len, hdr->out_len)))) {
		dev_err(dev, "unsupported IOMFB callback %d (%u, %u)\n",
			tag, hdr->in_len, hdr->out_len);
		dcp->crashed = true;
		return;
	}

	in = data + sizeof(*hdr);
	out = in + hdr->in_len;

	if (!handled) {
		zero_reply = dcp_external_zero_reply(dcp, tag, context,
						     hdr->in_len, hdr->out_len);
		if (!zero_reply &&
		    (!iomfb_uses_queue(dcp) ||
		     !test_and_set_bit(tag, dcp->unknown_callbacks)))
			dev_warn(dev, "received unknown callback %c%c%c%c (%u, %u) context=%u\n",
				 hdr->tag[3], hdr->tag[2], hdr->tag[1], hdr->tag[0],
				 hdr->in_len, hdr->out_len, context);
		if (dcp->hw.iomfb_method_profile ==
		    DCP_IOMFB_METHODS_H17P_EXTERNAL && !zero_reply) {
			/* Retain the failed session without inventing a successful reply. */
			dcp->crashed = true;
			complete(&dcp->start_done);
			return;
		}
		/*
		 * Leaving a callback unanswered wedges the coprocessor: it
		 * waits for the ack forever and the outer call never returns.
		 * On H17P answer it with a zeroed output instead.
		 */
		if (dcp->fw_compat == DCP_FIRMWARE_H17P) {
			if (hdr->out_len)
				memset(out, 0, hdr->out_len);
			depth = dcp_push_depth(&ch->depth);
			ch->output[depth] = out;
			ch->end[depth] = offset +
					 ALIGN(length, DCP_PACKET_ALIGNMENT);
			dcp_ack(dcp, context);
		}
		return;
	}

	// TODO: verify that in_len and out_len match our prototypes
	// for now just clear the out data to have at least consistent results
	if (hdr->out_len)
		memset(out, 0, hdr->out_len);

	depth = dcp_push_depth(&ch->depth);
	ch->output[depth] = out;
	ch->end[depth] = offset + ALIGN(length, DCP_PACKET_ALIGNMENT);

	if (dcp->cb_handlers[tag](dcp, tag, out, in))
		dcp_ack(dcp, context);
}

/* H17P: the reply is written in place into the record dcp_push() saved. */
static void dcpep_handle_ack_h17p(struct apple_dcp *dcp, enum dcp_context_id context)
{
	struct dcp_channel *ch = dcp_get_channel(dcp, context);
	const struct dcp_packet_header *record, *sent;
	void *cookie, *out;
	dcp_callback_t cb;

	if (!ch) {
		dev_warn(dcp->dev, "ignoring ack on context %X\n", context);
		return;
	}

	if (!ch->depth) {
		dev_warn(dcp->dev, "ignoring ack on idle context %X\n", context);
		return;
	}

	dcp_pop_depth(&ch->depth);

	cb = ch->callbacks[ch->depth];
	cookie = ch->cookies[ch->depth];
	out = ch->output[ch->depth];
	sent = &ch->header[ch->depth];

	ch->callbacks[ch->depth] = NULL;
	ch->cookies[ch->depth] = NULL;
	ch->output[ch->depth] = NULL;

	/*
	 * An ack completes the innermost pending command on its context.  The
	 * firmware writes the reply in place into that AP command record, whose
	 * address dcp_push() saved; the ack itself carries neither offset nor
	 * length on any firmware.  A command sent from within a callback is
	 * acked on the callback context, whose receive area holds the callback
	 * record rather than the reply, so the ack cannot be used to find it.
	 *
	 * On older firmware the lengths in the record header must still be
	 * the ones that were sent; otherwise the record was overwritten and
	 * its output cannot be trusted.  The tag is left out of the check: it
	 * has only been seen preserved on 13.5 firmware.
	 */
	if (dcp->fw_compat != DCP_FIRMWARE_H17P) {
		record = out - sent->in_len - sizeof(*record);
		if (record->in_len != sent->in_len ||
		    record->out_len != sent->out_len) {
			dev_err(dcp->dev,
				"corrupt %c%c%c%c command record on context %X\n",
				sent->tag[3], sent->tag[2], sent->tag[1],
				sent->tag[0], context);
			WRITE_ONCE(dcp->crashed, true);
			/*
			 * Still complete the command, without an output, so
			 * that its callback drops the cookie reference and
			 * wakes the waiter instead of leaving it to time out.
			 */
			out = NULL;
		}
	}

	if (cb)
		cb(dcp, out, cookie);
}

static void dcpep_handle_cb(struct apple_dcp *dcp, enum dcp_context_id context,
			    void *data, u32 length, u16 offset)
{
	struct device *dev = dcp->dev;
	struct dcp_packet_header *hdr = data;
	void *in, *out;
	int tag = dcp_parse_tag(hdr->tag);
	struct dcp_channel *ch = dcp_get_channel(dcp, context);
	u8 depth;

	if (tag < 0 || tag >= IOMFB_MAX_CB || !dcp->cb_handlers || !dcp->cb_handlers[tag]) {
		dev_warn(dev, "received unknown callback %c%c%c%c\n",
			 hdr->tag[3], hdr->tag[2], hdr->tag[1], hdr->tag[0]);
		return;
	}

	in = data + sizeof(*hdr);
	out = in + hdr->in_len;

	// TODO: verify that in_len and out_len match our prototypes
	// for now just clear the out data to have at least consistent results
	if (hdr->out_len)
		memset(out, 0, hdr->out_len);

	depth = dcp_push_depth(&ch->depth);
	ch->output[depth] = out;
	ch->end[depth] = offset + ALIGN(length, DCP_PACKET_ALIGNMENT);

	if (dcp->cb_handlers[tag](dcp, tag, out, in))
		dcp_ack(dcp, context);
}

static void dcpep_handle_ack(struct apple_dcp *dcp, enum dcp_context_id context,
			     void *data, u32 length)
{
	struct dcp_packet_header *header = data;
	struct dcp_channel *ch = dcp_get_channel(dcp, context);
	void *cookie;
	dcp_callback_t cb;

	if (!ch) {
		dev_warn(dcp->dev, "ignoring ack on context %X\n", context);
		return;
	}

	dcp_pop_depth(&ch->depth);

	cb = ch->callbacks[ch->depth];
	cookie = ch->cookies[ch->depth];

	ch->callbacks[ch->depth] = NULL;
	ch->cookies[ch->depth] = NULL;

	if (cb)
		cb(dcp, data + sizeof(*header) + header->in_len, cookie);
}

static void dcpep_got_msg(struct apple_dcp *dcp, u64 message)
{
	enum dcp_context_id ctx_id;
	u16 offset;
	u32 length;
	int channel_offset;
	void *data;

	ctx_id = FIELD_GET(IOMFB_MSG_CONTEXT, message);
	offset = FIELD_GET(IOMFB_MSG_OFFSET, message);
	length = FIELD_GET(IOMFB_MSG_LENGTH, message);

	channel_offset = dcp_channel_offset(ctx_id);

	if (channel_offset < 0) {
		dev_warn(dcp->dev, "invalid context received %u\n", ctx_id);
		return;
	}

	data = dcp->shmem + channel_offset + offset;

	if (dcp->fw_compat == DCP_FIRMWARE_H17P) {
		if (FIELD_GET(IOMFB_MSG_ACK, message))
			dcpep_handle_ack_h17p(dcp, ctx_id);
		else
			dcpep_handle_cb_h17p(dcp, ctx_id, data, length, offset);
		return;
	}

	if (FIELD_GET(IOMFB_MSG_ACK, message))
		dcpep_handle_ack(dcp, ctx_id, data, length);
	else
		dcpep_handle_cb(dcp, ctx_id, data, length, offset);
}

static void dcp_modes_dimensions_work(struct work_struct *work)
{
	struct apple_dcp *dcp = container_of(work, struct apple_dcp, dimensions_wq);
	u64 generation;

	scoped_guard(mutex, &dcp->modes_lock)
		generation = dcp->dimensions_generation;
	dcp_set_dimensions(dcp, generation);
}

void dcp_modes_init(struct apple_dcp *dcp)
{
	mutex_init(&dcp->modes_lock);
	INIT_WORK(&dcp->dimensions_wq, dcp_modes_dimensions_work);
	disable_work(&dcp->dimensions_wq);
}

bool dcp_modes_for_connector(struct apple_dcp *dcp,
			     struct apple_connector *connector)
{
	lockdep_assert_held(&dcp->modes_lock);
	return dcp->modes_admitted && READ_ONCE(dcp->connector) == connector;
}

void dcp_modes_begin_attachment(struct apple_dcp *dcp)
{
	guard(mutex)(&dcp->modes_lock);
	dcp->modes_generation++;
	dcp->modes_admitted = false;
	dcp->modes_provisional = false;
}

bool dcp_modes_end_typec(struct apple_dcp *dcp, struct apple_dcp_typec_route *route)
{
	guard(mutex)(&dcp->modes_lock);
	if (READ_ONCE(dcp->active_typec_route) != route)
		return false;
	dcp->modes_generation++;
	dcp->modes_admitted = false;
	dcp->modes_provisional = false;
	return true;
}

/* A copy of the modes @dcp knows for @connector's display, or NULL. */
struct dcp_display_mode *dcp_modes_dup(struct apple_dcp *dcp,
				       struct apple_connector *connector,
				       unsigned int *count, u64 *generation)
{
	struct dcp_display_mode *modes;

	guard(mutex)(&dcp->modes_lock);
	*count = 0;
	if (!connector || !dcp->nr_modes || !dcp->modes_admitted ||
	    READ_ONCE(dcp->connector) != connector)
		return ERR_PTR(-ENODATA);
	*generation = dcp->modes_generation;
	modes = kmemdup_array(dcp->modes, dcp->nr_modes, sizeof(*dcp->modes),
			      GFP_KERNEL);
	if (!modes)
		return ERR_PTR(-ENOMEM);
	*count = dcp->nr_modes;
	return modes;
}

/*
 * A display came to @dcp from another pipeline without being unplugged:
 * offer the modes it had there until the firmware here describes it.
 * Takes @modes.
 */
void dcp_modes_adopt(struct apple_dcp *dcp, struct apple_connector *connector,
		     u64 generation, struct dcp_display_mode *modes,
		     unsigned int count)
{
	if (!modes)
		return;
	guard(mutex)(&dcp->modes_lock);
	/* Never overwrite a fresh catalog or lend it to a different attachment. */
	if (generation != dcp->modes_generation ||
	    READ_ONCE(dcp->connector) != connector || dcp->modes_admitted) {
		kfree(modes);
		return;
	}
	kfree(dcp->modes);
	dcp->modes = modes;
	dcp->nr_modes = count;
	dcp->modes_admitted = true;
	dcp->modes_provisional = true;
}

u64 dcp_modes_transfer_begin(struct apple_dcp *dcp)
{
	guard(mutex)(&dcp->modes_lock);
	return dcp->modes_generation;
}

int dcp_modes_replace(struct apple_dcp *dcp, struct dcp_parse_ctx *handle,
		      u64 generation)
{
	struct apple_connector *ready = NULL;
	int ret;

	scoped_guard(mutex, &dcp->modes_lock) {
		bool provisional = dcp->modes_provisional;

		if (generation != dcp->modes_generation)
			return -ESTALE;
		/*
		 * The T6030 external processor's catalog takes the external SDR
		 * path: timing colour restrictions, sync polarity, IsPreferred,
		 * and the fw14 profile the parse context carries.
		 */
		ret = replace_modes(handle, &dcp->modes, &dcp->nr_modes,
				    dcp->width_mm, dcp->height_mm, dcp->notch_height,
				    dcp->fixed_connector_type == DRM_MODE_CONNECTOR_eDP,
				    dcp_is_v14_7_external(dcp));
		if (!ret) {
			dcp->modes_admitted = true;
			dcp->modes_provisional = false;
			/*
			 * Described again after a withdrawal it was kept
			 * connected through, the same display keeps its EDID,
			 * unless that is a placeholder being retried.
			 */
			if (!atomic_read(&dcp->external_held) ||
			    READ_ONCE(dcp->placeholder_retried))
				apple_connector_invalidate_edid(dcp->connector);
			if (provisional && dcp->connector && dcp->dev &&
			    dcp_modes_for_connector(dcp, dcp->connector) &&
			    READ_ONCE(dcp->connector->dcp) == to_platform_device(dcp->dev))
				ready = dcp->connector;
		}
	}
	/* RX lifetime is drained before connector cleanup; work rechecks ownership. */
	if (ready)
		dcp_queue_hotplug(ready);
	return ret;
}

int dcp_attributes_replace(struct apple_dcp *dcp, struct dcp_parse_ctx *handle,
			   u64 generation)
{
	int width_mm, height_mm, ret, i;
	bool backlight_control, ext;

	guard(mutex)(&dcp->modes_lock);
	if (generation != dcp->modes_generation)
		return -ESTALE;
	ret = parse_display_attributes(handle, &width_mm, &height_mm, &backlight_control);
	if (ret)
		return ret;
	dcp->width_mm = width_mm;
	dcp->height_mm = height_mm;
	if (!width_mm || !height_mm) {
		width_mm = dcp->panel.width_mm;
		height_mm = dcp->panel.height_mm;
	}
	for (i = 0; i < dcp->nr_modes; ++i) {
		dcp->modes[i].mode.width_mm = width_mm;
		dcp->modes[i].mode.height_mm = height_mm;
	}
	dcp->dimensions_generation = generation;
	schedule_work(&dcp->dimensions_wq);
	/* 14.7 swaps carry no external backlight level. */
	ext = backlight_control && !dcp_has_panel(dcp) &&
	      !dcp_fw_is_14_7(dcp);
	if (ext != READ_ONCE(dcp->ext_backlight)) {
		WRITE_ONCE(dcp->ext_backlight, ext);
		if (dcp->connector)
			schedule_work(&dcp->connector->bl_sync_wq);
	}
	return 0;
}

void dcp_modes_release(void *data)
{
	struct apple_dcp *dcp = data;

	disable_work_sync(&dcp->dimensions_wq);
	guard(mutex)(&dcp->modes_lock);
	dcp->modes_generation++;
	dcp->modes_admitted = false;
	kfree(dcp->modes);
	dcp->modes = NULL;
	dcp->nr_modes = 0;
}


/* Retained arrays are not freed while mode readers can still refer to them. */
static struct dcp_display_mode *dcp_modes_snapshot(struct apple_dcp *dcp,
						 unsigned int *count)
{
	struct dcp_display_mode *modes;
	unsigned long flags;

	spin_lock_irqsave(&dcp->metadata.lock, flags);
	modes = dcp->modes;
	*count = dcp->nr_modes;
	spin_unlock_irqrestore(&dcp->metadata.lock, flags);
	return modes;
}

/* Caller holds hpd_mutex -> dcpavserv.lock -> metadata.lock. */
static bool dcp_exception_mode_current_locked(struct apple_dcp *dcp,
					     const struct dcp_display_mode *mode)
{
	if (!mode->automatic_rgb8_quirk)
		return true;
	return dcp_180hz_mode_current(dcp, mode);
}

static bool dcp_publish_mode(struct apple_dcp *dcp, struct drm_connector *connector,
			     const struct dcp_display_mode *source,
			     struct drm_display_mode *copy)
{
	unsigned long flags;
	bool eligible;

	/* Ordinary modes keep their existing publication policy. */
	if (!source->automatic_rgb8_quirk) {
		drm_mode_probed_add(connector, copy);
		return true;
	}
	/* External get_modes holds hpd_mutex after its final EDID generation check. */
	/* Allocation happened outside metadata.lock; recheck before linking. */
	spin_lock_irqsave(&dcp->metadata.lock, flags);
	eligible = dcp->connector && &dcp->connector->base == connector &&
		dcp_exception_mode_current_locked(dcp, source);
	if (eligible)
		/* drm_mode_probed_add only links a list under mode_config.mutex. */
		drm_mode_probed_add(connector, copy);
	spin_unlock_irqrestore(&dcp->metadata.lock, flags);
	if (!eligible)
		drm_mode_destroy(connector->dev, copy);
	return eligible;
}

/* H17P (T8140, T8142): aurora-wip's mode and EDID publication. */
static int dcp_get_modes_h17p(struct drm_connector *connector)
{
	struct apple_connector *apple_connector = to_apple_connector(connector);
	struct platform_device *pdev = READ_ONCE(apple_connector->dcp);
	struct apple_dcp *dcp;

	struct drm_device *dev = connector->dev;
	struct drm_display_mode *mode;
	struct dcp_display_mode *modes;
	unsigned int nr_modes, published = 0;
	u16 min_vfreq = 0, max_vfreq = 0;
	bool vrr_capable = false;
	bool physical_edid;
	bool pending_180hz = false, published_180hz = false;
	u64 edid_generation = 0;
	u64 service_epoch = 0;
	const struct drm_edid *placeholder;
	unsigned int i;

	dcp_consume_edid_invalidation(apple_connector);

	/* A Type-C port has no pipeline while the fabric is moving it. */
	if (!pdev) {
		if (apple_connector->drm_edid_firmware_derived) {
			drm_edid_connector_update(connector, NULL);
			apple_connector->drm_edid_firmware_derived = false;
		}
		return 0;
	}
	dcp = platform_get_drvdata(pdev);
	modes = dcp_modes_snapshot(dcp, &nr_modes);

	physical_edid = dcp->hw.iomfb_method_profile ==
		DCP_IOMFB_METHODS_H17P_EXTERNAL;
	/* get_modes holds mode_config.mutex; HPD paths never acquire it. */
	if (physical_edid) {
		mutex_lock(&dcp->hpd_mutex);
		edid_generation = dcp->typec_generation;
	}
	mutex_lock(&dcp->dcpavserv.lock);
	/* Consume CLOSE invalidation under the publication/retirement lock. */
	dcp_consume_edid_invalidation(apple_connector);
	if (apple_connector->drm_edid &&
	    (!dcpavserv_edid_service_current(dcp, apple_connector->drm_edid_service_epoch) ||
	     (physical_edid && apple_connector->drm_edid_generation != edid_generation)))
		dcp_clear_connector_edid(apple_connector);
	if (physical_edid) {
		if (!dcpavserv_edid_current(dcp, edid_generation)) {
			dcp_clear_connector_edid(apple_connector);
			goto edid_done;
		}
	}

	/* Internal H17P still has no supported EDID command transport. */
	if (nr_modes && !apple_connector->drm_edid &&
	    (dcp->fw_compat != DCP_FIRMWARE_H17P ||
	     (dcp->dcpavservep && dcp->dcpavservep->wire_v2))) {
		const struct drm_edid *edid;
		/* Let physical disconnect advance the generation during the request. */
		mutex_unlock(&dcp->dcpavserv.lock);
		if (physical_edid)
			mutex_unlock(&dcp->hpd_mutex);
		edid = dcpavserv_copy_edid(dcp, edid_generation, &service_epoch);
		if (physical_edid)
			mutex_lock(&dcp->hpd_mutex);
		mutex_lock(&dcp->dcpavserv.lock);
		if (physical_edid) {
			if (!dcpavserv_edid_current(dcp, edid_generation) ||
			    READ_ONCE(apple_connector->dcp) != pdev) {
				if (!IS_ERR_OR_NULL(edid))
					drm_edid_free(edid);
				dcp_clear_connector_edid(apple_connector);
				goto edid_done;
			}
		}
		if (!IS_ERR_OR_NULL(edid) &&
		    !dcpavserv_edid_service_current(dcp, service_epoch)) {
			drm_edid_free(edid);
			dcp_clear_connector_edid(apple_connector);
			goto edid_done;
		}
		/* Retirement may have queued cleanup while this request waited. */
		dcp_consume_edid_invalidation(apple_connector);
		if (IS_ERR_OR_NULL(edid)) {
			/*
			 * There is no AV service for an internal panel, nor
			 * for an external display while its link is down;
			 * do not report that on every mode probe.
			 */
			if (PTR_ERR(edid) == -ENODEV)
				dev_dbg(dcp->dev, "no EDID source\n");
			else
				dev_warn(dcp->dev, "copy_edid failed: %pe\n", edid);
		} else {
			scoped_guard(mutex, &apple_connector->edid_lock) {
				drm_edid_free(apple_connector->drm_edid);
				apple_connector->drm_edid = edid;
			}
			apple_connector->drm_edid_generation = edid_generation;
			apple_connector->drm_edid_service_epoch = service_epoch;
		}
	}
	if (nr_modes && apple_connector->drm_edid) {
		int update;

		/* Integration's route changes drop the cached EDID under edid_lock. */
		scoped_guard(mutex, &apple_connector->edid_lock)
			update = apple_connector->drm_edid ?
				drm_edid_connector_update(connector, apple_connector->drm_edid) :
				-ENODATA;

		if (update) {
			dev_warn(dcp->dev, "raw EDID publication failed: %d\n", update);
			dcp_clear_connector_edid(apple_connector);
		} else {
			apple_connector->drm_edid_firmware_derived = false;
		}
	} else {
		u64 generation = 0;
		const struct drm_edid *derived = nr_modes ?
			dcp_firmware_edid(dcp, &generation) : ERR_PTR(-ENODATA);

		if (!IS_ERR(derived)) {
			bool log = !apple_connector->drm_edid_firmware_derived ||
				apple_connector->drm_edid_generation != generation;
			int update;

			apple_connector->drm_edid_firmware_derived = true;
			apple_connector->drm_edid_generation = generation;
			update = drm_edid_connector_update(connector, derived);
			drm_edid_free(derived);
			if (update || !dcp_firmware_metadata_current(dcp, generation)) {
				drm_edid_connector_update(connector, NULL);
				apple_connector->drm_edid_firmware_derived = false;
			} else if (log) {
				dev_info(dcp->dev, "using firmware-derived EDID metadata for connection %llu\n",
					 generation);
			}
		} else if (apple_connector->drm_edid_firmware_derived) {
			drm_edid_connector_update(connector, NULL);
			apple_connector->drm_edid_firmware_derived = false;
		}
	}

edid_done:
	/* The 180 Hz exception checks below read the cached EDID. */
	mutex_lock(&apple_connector->edid_lock);
	/* Publish after a possibly delayed raw read, under its physical generation. */
	for (i = 0; i < nr_modes; ++i) {
		mode = drm_mode_duplicate(dev, &modes[i].mode);
		if (!mode) {
			dev_err(dev->dev, "Failed to duplicate display mode\n");
			published = 0;
			goto modes_done;
		}
		pending_180hz |= modes[i].automatic_rgb8_quirk &&
			dcp_180hz_pending(dcp, &modes[i]);
		if (!dcp_publish_mode(dcp, connector, &modes[i], mode))
			continue;
		published_180hz |= modes[i].automatic_rgb8_quirk;
		published++;
		if (modes[i].vrr) {
			u16 lo = modes[i].min_vrr >> 16;
			u16 hi = modes[i].max_vrr >> 16;

			if (!min_vfreq || lo < min_vfreq)
				min_vfreq = lo;
			if (hi > max_vfreq)
				max_vfreq = hi;
		}
		vrr_capable |= modes[i].vrr;
	}
	drm_connector_set_vrr_capable_property(connector, vrr_capable);
modes_done:
	mutex_unlock(&apple_connector->edid_lock);
	mutex_unlock(&dcp->dcpavserv.lock);
	if (physical_edid)
		mutex_unlock(&dcp->hpd_mutex);
	if (pending_180hz && !published_180hz)
		dcp_180hz_retry(dcp);
	scoped_guard(mutex, &apple_connector->edid_lock)
		placeholder = apple_connector->drm_edid ?
			drm_edid_dup(apple_connector->drm_edid) : NULL;
	if (placeholder) {
		dcp_retry_placeholder_edid(dcp, placeholder);
		drm_edid_free(placeholder);
	}

	/*
	 * An internal panel has no EDID, so nothing fills in the refresh range
	 * that userspace needs before it will drive VRR. Supply the range DCP
	 * reported for the mode. This has to follow the EDID update, which
	 * resets display_info.
	 */
	if (vrr_capable && max_vfreq &&
	    !connector->display_info.monitor_range.max_vfreq) {
		connector->display_info.monitor_range.min_vfreq = min_vfreq;
		connector->display_info.monitor_range.max_vfreq = max_vfreq;
	}

	return published;
}

/* The user may own drm_display_mode, so we need to search for our copy */
struct dcp_display_mode *lookup_mode_h17p(struct apple_dcp *dcp,
					  const struct drm_display_mode *mode)
{
	struct apple_connector *con;
	struct dcp_display_mode *modes;
	unsigned int count, i;
	unsigned long flags;
	bool eligible;

	modes = dcp_modes_snapshot(dcp, &count);
	for (i = 0; i < count; ++i) {
		if (drm_mode_match(mode, &modes[i].mode,
				   DRM_MODE_MATCH_TIMINGS |
					   DRM_MODE_MATCH_CLOCK)) {
			if (!modes[i].automatic_rgb8_quirk)
				return &modes[i];
			mutex_lock(&dcp->hpd_mutex);
			mutex_lock(&dcp->dcpavserv.lock);
			con = READ_ONCE(dcp->connector);
			/* The exception check reads the cached EDID. */
			if (con)
				mutex_lock(&con->edid_lock);
			spin_lock_irqsave(&dcp->metadata.lock, flags);
			eligible = dcp_exception_mode_current_locked(dcp, &modes[i]);
			spin_unlock_irqrestore(&dcp->metadata.lock, flags);
			if (con)
				mutex_unlock(&con->edid_lock);
			mutex_unlock(&dcp->dcpavserv.lock);
			mutex_unlock(&dcp->hpd_mutex);
			if (eligible)
				return &modes[i];
		}
	}

	return NULL;
}

/*
 * H17P keeps the mode the bootloader programmed and never sends
 * set_digital_out_mode (see DCP_INHERIT_BOOT_MODE in iomfb_template.c), so a
 * modeset to any other mode would silently not be applied.  The bootloader
 * brings the panel up in its native timing, which is the mode the firmware
 * scores highest and enumerate_modes() marks preferred; offer only that one.
 */
static bool dcp_mode_settable(struct apple_dcp *dcp,
			      const struct drm_display_mode *mode)
{
	struct dcp_display_mode *dcp_mode = lookup_mode_h17p(dcp, mode);

	if (!dcp_mode)
		return false;

	if (dcp->fw_compat == DCP_FIRMWARE_H17P &&
	    dcp->hw.iomfb_method_profile !=
	    DCP_IOMFB_METHODS_H17P_EXTERNAL)
		return dcp_mode->mode.type & DRM_MODE_TYPE_PREFERRED;

	return true;
}

int dcp_get_modes(struct drm_connector *connector)
{
	struct apple_connector *apple_connector = to_apple_connector(connector);
	struct platform_device *pdev = READ_ONCE(apple_connector->dcp);
	struct apple_dcp *dcp;

	struct drm_device *dev = connector->dev;
	struct drm_display_mode *mode;
	u16 min_vfreq = 0, max_vfreq = 0;
	bool vrr_capable = false;
	unsigned int count;
	int i;

	if (dcp_connector_h17p(apple_connector))
		return dcp_get_modes_h17p(connector);

	/* A Type-C port has no pipeline while the fabric is moving it. */
	if (!pdev) {
		drm_edid_connector_update(connector, NULL);
		return 0;
	}
	dcp = platform_get_drvdata(pdev);

	mutex_lock(&dcp->modes_lock);
	count = dcp_modes_for_connector(dcp, apple_connector) ? dcp->nr_modes : 0;
	for (i = 0; i < count; ++i) {
		if (dcp->modes[i].vrr) {
			u16 lo = dcp->modes[i].min_vrr >> 16;
			u16 hi = dcp->modes[i].max_vrr >> 16;

			if (!min_vfreq || lo < min_vfreq)
				min_vfreq = lo;
			if (hi > max_vfreq)
				max_vfreq = hi;
		}
		vrr_capable |= dcp->modes[i].vrr;
		mode = drm_mode_duplicate(dev, &dcp->modes[i].mode);

		if (!mode) {
			dev_err(dev->dev, "Failed to duplicate display mode\n");
			mutex_unlock(&dcp->modes_lock);
			return 0;
		}

		drm_mode_probed_add(connector, mode);
	}
	mutex_unlock(&dcp->modes_lock);
	drm_connector_set_vrr_capable_property(connector, vrr_capable);

	if (count) {
		const struct drm_edid *edid;
		u64 generation, epoch;

		/* The RPC can sleep while a disconnect or route change clears EDID. */
		if (apple_connector_edid_begin(apple_connector, pdev, &generation)) {
			edid = dcpavserv_copy_edid(dcp, 0, &epoch);
			if (IS_ERR_OR_NULL(edid))
				dev_info(dcp->dev, "copy_edid failed: %pe\n", edid);
			else if (apple_connector_edid_install(apple_connector, pdev,
							      generation, edid))
				/* A native T6030 pipe: its audio, after the first EDID. */
				dcp_external_audio_start(dcp);
		}

		edid = apple_connector_edid_dup(apple_connector, pdev);
		drm_edid_connector_update(connector, edid);
		if (edid) {
			dcp_retry_placeholder_edid(dcp, edid);
			drm_edid_free(edid);
		}
	} else {
		drm_edid_connector_update(connector, NULL);
	}

	/*
	 * An internal panel has no EDID, so nothing fills in the refresh range
	 * that userspace needs before it will drive VRR. Supply the range DCP
	 * reported for the mode. This has to follow the EDID update, which
	 * resets display_info.
	 */
	if (vrr_capable && max_vfreq &&
	    !connector->display_info.monitor_range.max_vfreq) {
		connector->display_info.monitor_range.min_vfreq = min_vfreq;
		connector->display_info.monitor_range.max_vfreq = max_vfreq;
	}

	return count;
}

/* The user may own drm_display_mode, so we need to search for our copy */
static bool lookup_mode_locked(struct apple_dcp *dcp,
			       const struct drm_display_mode *mode,
			       struct dcp_display_mode *out)
{
	bool found = false;
	int i;

	lockdep_assert_held(&dcp->modes_lock);
	if (!dcp->modes_admitted)
		goto out_unlock;
	for (i = 0; i < dcp->nr_modes; ++i) {
		if (drm_mode_match(mode, &dcp->modes[i].mode,
				   DRM_MODE_MATCH_TIMINGS | DRM_MODE_MATCH_CLOCK)) {
			if (out)
				*out = dcp->modes[i];
			found = true;
			break;
		}
	}
out_unlock:
	return found;
}

bool lookup_mode(struct apple_dcp *dcp, const struct drm_display_mode *mode,
		 struct dcp_display_mode *out)
{
	if (dcp->fw_compat == DCP_FIRMWARE_H17P) {
		struct dcp_display_mode *found = lookup_mode_h17p(dcp, mode);

		if (found && out)
			*out = *found;
		return found;
	}

	guard(mutex)(&dcp->modes_lock);
	return lookup_mode_locked(dcp, mode, out);
}

enum drm_mode_status dcp_mode_valid(struct drm_connector *connector,
				    const struct drm_display_mode *mode)
{
	struct apple_connector *apple_connector = to_apple_connector(connector);
	struct platform_device *pdev = READ_ONCE(apple_connector->dcp);
	struct apple_dcp *dcp;

	if (!pdev)
		return MODE_ERROR;
	dcp = platform_get_drvdata(pdev);

	if (dcp->fw_compat == DCP_FIRMWARE_H17P)
		return dcp_mode_settable(dcp, mode) ? MODE_OK : MODE_BAD;

	guard(mutex)(&dcp->modes_lock);
	if (!dcp_modes_for_connector(dcp, apple_connector))
		return MODE_BAD;
	return lookup_mode_locked(dcp, mode, NULL) ? MODE_OK : MODE_BAD;
}

/* H17P (T8140, T8142): aurora-wip's modeset admission. */
static int dcp_crtc_atomic_modeset_h17p(struct drm_crtc *crtc,
					struct drm_atomic_state *state)
{
	struct apple_crtc *apple_crtc = to_apple_crtc(crtc);
	struct apple_dcp *dcp = platform_get_drvdata(apple_crtc->dcp);
	struct drm_crtc_state *crtc_state;
	unsigned int action;
	int ret = -EIO;
	bool modeset;

	/* Disconnect cannot pass the admission check before these RPCs finish. */
	guard(mutex)(&dcp->external_lock);
	if (dcp->hw.iomfb_method_profile == DCP_IOMFB_METHODS_H17P_EXTERNAL &&
	    (!dcp->external_running || dcp->external_error || READ_ONCE(dcp->quiescing) ||
	     READ_ONCE(dcp->crashed) || READ_ONCE(dcp->iomfb.presentation_paused)))
		return -EPIPE;

	crtc_state = drm_atomic_get_new_crtc_state(state, crtc);
	if (!crtc_state)
		return 0;

	modeset = drm_atomic_crtc_needs_modeset(crtc_state) ||
		  !READ_ONCE(dcp->mode_state.valid);

	if (!modeset)
		return 0;

	/* ignore no mode, poweroff is handled elsewhere */
	if (crtc_state->mode.hdisplay == 0 && crtc_state->mode.vdisplay == 0)
		return 0;

	dcp_mode_begin(&dcp->mode_state);
	switch (dcp->fw_compat) {
	case DCP_FIRMWARE_V_12_3:
		ret = iomfb_modeset_v12_3(dcp, crtc_state);
		break;
	case DCP_FIRMWARE_V_13_5:
		ret = iomfb_modeset_v13_3(dcp, crtc_state);
		break;
	case DCP_FIRMWARE_H17P:
		ret = iomfb_modeset_h17p(dcp, crtc_state);
		break;
	default:
		WARN_ONCE(true, "Unexpected firmware version: %u\n",
			  dcp->fw_compat);
		break;
	}

	action = dcp_mode_finish(&dcp->mode_state, !ret, dcp->connector ?
				&dcp->connector->connected : NULL);
	dcp_handle_hotplug_actions(dcp, action);

	return ret;
}

int dcp_crtc_atomic_modeset(struct drm_crtc *crtc,
			    struct drm_atomic_state *state)
{
	struct apple_crtc *apple_crtc = to_apple_crtc(crtc);
	struct apple_dcp *dcp = platform_get_drvdata(apple_crtc->dcp);
	struct drm_crtc_state *crtc_state;
	unsigned int action;
	int ret = -EIO;
	bool modeset;

	if (dcp->fw_compat == DCP_FIRMWARE_H17P)
		return dcp_crtc_atomic_modeset_h17p(crtc, state);

	crtc_state = drm_atomic_get_new_crtc_state(state, crtc);
	if (!crtc_state)
		return 0;

	modeset = drm_atomic_crtc_needs_modeset(crtc_state) ||
		  !READ_ONCE(dcp->mode_state.valid);

	if (!modeset)
		return 0;

	/* ignore no mode, poweroff is handled elsewhere */
	if (crtc_state->mode.hdisplay == 0 && crtc_state->mode.vdisplay == 0)
		return 0;

	dcp_mode_begin(&dcp->mode_state);
	switch (dcp->fw_compat) {
	case DCP_FIRMWARE_V_12_3:
		ret = iomfb_modeset_v12_3(dcp, crtc_state);
		break;
	case DCP_FIRMWARE_V_13_5:
		ret = iomfb_modeset_v13_3(dcp, crtc_state);
		break;
	case DCP_FIRMWARE_V_26_6:
		ret = iomfb_v26_6_modeset(dcp, crtc_state);
		break;
	case DCP_FIRMWARE_V_14_7:
		ret = iomfb_v14_7_modeset(dcp, crtc_state);
		break;
	case DCP_FIRMWARE_V_14_7_TEMPLATE:
		ret = iomfb_modeset_v14_7_0(dcp, crtc_state);
		break;
	default:
		WARN_ONCE(true, "Unexpected firmware version: %u\n",
			  dcp->fw_compat);
		break;
	}

	action = dcp_mode_finish(&dcp->mode_state, !ret, dcp->connector ?
				&dcp->connector->connected : NULL);
	dcp_handle_hotplug_actions(dcp, action);

	return ret;
}

bool dcp_crtc_needs_route_start(struct apple_dcp *dcp)
{
	struct apple_dcp_typec_route *route = READ_ONCE(dcp->active_typec_route);

	return dcp_typec_follows_crtc(dcp) && route &&
	       READ_ONCE(dcp->typec_crtc_off) && READ_ONCE(dcp->typec_follow_start) &&
	       READ_ONCE(dcp->typec_follow_gen) == READ_ONCE(dcp->typec_generation);
}

bool dcp_crtc_route_ready(struct drm_crtc *crtc, struct drm_atomic_state *state,
			  bool fresh)
{
	struct apple_dcp *dcp = platform_get_drvdata(to_apple_crtc(crtc)->dcp);
	struct drm_connector_list_iter iter;
	struct drm_crtc_state *crtc_state;
	struct drm_connector *conn;
	u64 mask, seen = 0;
	bool ready = true, owned = false;

	if (!dcp_typec_follows_crtc(dcp))
		return true;
	guard(mutex)(&dcp->modes_lock);
	crtc_state = drm_atomic_get_new_crtc_state(state, crtc);
	if (!crtc_state)
		crtc_state = crtc->state;
	mask = crtc_state ? crtc_state->connector_mask : 0;
	if (!mask)
		return !dcp->connector || !dcp->connector->port_encoder;
	/* The mask is the complete assignment, even on a plane-only commit. */
	drm_connector_list_iter_begin(crtc->dev, &iter);
	drm_for_each_connector_iter(conn, &iter) {
		struct apple_connector *connector = to_apple_connector(conn);
		struct platform_device *owner;
		struct drm_connector_state *assignment;

		if (!(mask & drm_connector_mask(conn)))
			continue;
		seen |= drm_connector_mask(conn);
		owner = READ_ONCE(connector->dcp);
		/* Preserve the unrouted, disconnected resume/deferred-flush path. */
		if (!owner && !READ_ONCE(connector->connected))
			continue;
		owned = true;
		assignment = drm_atomic_get_new_connector_state(state, conn);
		if (!assignment)
			assignment = conn->state;
		if (!assignment || assignment->crtc != crtc ||
		    owner != to_apple_crtc(crtc)->dcp || dcp->connector != connector)
			ready = false;
	}
	drm_connector_list_iter_end(&iter);
	if (!ready || seen != mask)
		return false;
	if (fresh && dcp->modes_provisional)
		return false;
	if (fresh && owned && READ_ONCE(dcp->typec_follow_start) &&
	    READ_ONCE(dcp->typec_follow_gen) == READ_ONCE(dcp->typec_generation))
		return dcp_modes_for_connector(dcp, dcp->connector);
	return true;
}

/* Does the display @dcp drives offer @mode? */
bool dcp_has_mode(struct apple_dcp *dcp, const struct drm_display_mode *mode)
{
	return lookup_mode(dcp, mode, NULL);
}

bool dcp_crtc_mode_fixup(struct drm_crtc *crtc,
			 const struct drm_display_mode *mode,
			 struct drm_display_mode *adjusted_mode)
{
	struct apple_crtc *apple_crtc = to_apple_crtc(crtc);
	struct platform_device *pdev = apple_crtc->dcp;
	struct apple_dcp *dcp = platform_get_drvdata(pdev);

	if (dcp->fw_compat == DCP_FIRMWARE_H17P)
		return dcp_mode_settable(dcp, mode);

	/*
	 * TODO: support synthesized modes through scaling
	 *
	 * A pipeline whose Type-C routes follow their CRTC may be given a
	 * display that another pipeline still drives: dcp_crtc_atomic_check()
	 * checks the mode against that one.
	 */
	return lookup_mode(dcp, mode, NULL) || dcp_typec_follows_crtc(dcp);
}


static void iomfb_scanout_release_h17p(struct iomfb_scanout_h17p *scanout)
{
	unsigned int i;

	if (!scanout)
		return;
	for (i = 0; i < SWAP_SURFACES; i++)
		if (scanout->fb[i])
			drm_framebuffer_put(scanout->fb[i]);
	kfree(scanout);
}

static struct iomfb_scanout_h17p *
iomfb_scanout_prepare_h17p(struct apple_dcp *dcp, struct drm_crtc *crtc,
			   struct drm_atomic_state *state)
{
	struct iomfb_scanout_h17p *scanout;
	struct drm_plane *plane;
	struct drm_plane_state *old_state, *new_state;
	unsigned int i, slot;
	int index;

	lockdep_assert_held(&dcp->iomfb.lock);
	scanout = kzalloc_obj(*scanout);
	if (!scanout)
		return NULL;
	if (dcp->iomfb.scanout) {
		scanout->request = dcp->iomfb.scanout->request;
		for (i = 0; i < SWAP_SURFACES; i++) {
			scanout->fb[i] = dcp->iomfb.scanout->fb[i];
			if (scanout->fb[i])
				drm_framebuffer_get(scanout->fb[i]);
		}
	}

	for_each_oldnew_plane_in_state(state, plane, old_state, new_state, index) {
		if (old_state->crtc != crtc && new_state->crtc != crtc)
			continue;
		slot = to_apple_plane(plane)->iomfb_surf;
		if (slot >= SWAP_SURFACES) {
			iomfb_scanout_release_h17p(scanout);
			return NULL;
		}
		if (scanout->fb[slot])
			drm_framebuffer_put(scanout->fb[slot]);
		scanout->fb[slot] = NULL;
		if (new_state->crtc == crtc && new_state->visible && new_state->fb) {
			scanout->fb[slot] = new_state->fb;
			drm_framebuffer_get(new_state->fb);
		}
	}
	return scanout;
}

void iomfb_scanout_complete_h17p(struct apple_dcp *dcp)
{
	struct iomfb_scanout_h17p *scanout = dcp->iomfb.next_scanout;
	struct iomfb_scanout_h17p *previous = dcp->iomfb.scanout;
	struct dcp_swap_submit_req_h17p *request = &dcp->swap.h17p;
	unsigned int i;

	lockdep_assert_held(&dcp->iomfb.lock);
	if (!scanout)
		return;

	/* Keep common present arguments, including the null output pointers. */
	scanout->request = *request;
	/* Preserve unchanged planes when only a subset was presented. */
	for (i = 0; i < SWAP_SURFACES; i++) {
		if (!(request->swap.swap_enabled & BIT(i)) && previous) {
			scanout->request.surf[i] = previous->request.surf[i];
			scanout->request.surf_iova[i] = previous->request.surf_iova[i];
			scanout->request.swap.src_rect[i] = previous->request.swap.src_rect[i];
			scanout->request.swap.dst_rect[i] = previous->request.swap.dst_rect[i];
			scanout->request.swap.surf_ids[i] = previous->request.swap.surf_ids[i];
			scanout->request.swap.surf_flags[i] = previous->request.swap.surf_flags[i];
			scanout->request.swap.surf_unk[i] = previous->request.swap.surf_unk[i];
		}
		scanout->request.surf_null[i] = !scanout->fb[i];
	}
	/* Brightness re-presents all pinned surfaces with a fresh swap ID. */
	/* Replay only a background established by a completed present. */
	scanout->request.swap.swap_enabled =
		request->swap.swap_enabled & IOMFB_SET_BACKGROUND;
	if (previous)
		scanout->request.swap.swap_enabled |=
			previous->request.swap.swap_enabled & IOMFB_SET_BACKGROUND;
	for (i = 0; i < SWAP_SURFACES; i++)
		if (scanout->fb[i])
			scanout->request.swap.swap_enabled |= BIT(i);
	scanout->request.swap.swap_completed = scanout->request.swap.swap_enabled;
	if (!(request->swap.swap_enabled & IOMFB_SET_BACKGROUND) && previous)
		scanout->request.swap.bg_color = previous->request.swap.bg_color;
	dcp->iomfb.scanout = scanout;
	dcp->iomfb.next_scanout = NULL;
	iomfb_scanout_release_h17p(previous);
}

bool iomfb_present_brightness_only_h17p(struct apple_dcp *dcp)
{
	return dcp->iomfb.active && dcp->iomfb.active->brightness_only;
}

bool iomfb_apply_backlight_h17p(struct apple_dcp *dcp,
				const struct dcp_swap_submit_req_h17p *request,
				struct dcp_present_h17p *wire)
{
	struct iomfb_transaction *transaction = dcp->iomfb.active;
	struct dcp_backlight_present present;
	bool have_surface = false;
	unsigned int i;
	int ret;

	lockdep_assert_held(&dcp->iomfb.lock);
	if (!dcp_backlight_active(dcp)) {
		if (!dcp_has_panel(dcp))
			return true;
		if (!dcp->brightness.maximum)
			return false;
		iomfb_encode_backlight_h17p(wire, 0,
					    dcp->brightness.maximum, false);
		return true;
	}
	if (!transaction || !dcp->brightness.maximum)
		return false;
	for (i = 0; i < SWAP_SURFACES; i++)
		have_surface |= !request->surf_null[i];
	ret = dcp_backlight_prepare(dcp, have_surface, &present);
	if (ret && ret != -EALREADY)
		return false;
	if (!ret) {
		transaction->backlight = present;
		transaction->backlight_reserved = true;
	}
	iomfb_encode_backlight_h17p(wire, present.nits,
				    dcp->brightness.maximum, !ret);
	return true;
}

void iomfb_present_failed_h17p(struct apple_dcp *dcp)
{
	struct iomfb_transaction *transaction = dcp->iomfb.active;

	if (transaction && transaction->backlight_reserved) {
		dcp_backlight_complete(dcp, transaction->backlight.sequence, false);
		transaction->backlight_reserved = false;
		transaction->backlight_failed = true;
	}
}

bool iomfb_present_complete_h17p(struct apple_dcp *dcp)
{
	struct iomfb_transaction *transaction = dcp->iomfb.active;

	lockdep_assert_held(&dcp->iomfb.lock);
	if (transaction) {
		transaction->completed = true;
		if (transaction->backlight_reserved) {
			dcp_backlight_complete(dcp, transaction->backlight.sequence, true);
			transaction->backlight_reserved = false;
		}
		if (transaction->brightness_only)
			return false;
	}
	atomic_cmpxchg(&dcp->iomfb.opaque_x_state, IOMFB_OPAQUE_X_WAITING,
		       IOMFB_OPAQUE_X_NEEDED);
	iomfb_scanout_complete_h17p(dcp);
	return true;
}

static void iomfb_backlight_start(struct apple_dcp *dcp,
				  struct iomfb_transaction *transaction)
{
	struct iomfb_scanout_h17p *scanout = dcp->iomfb.scanout;
	bool have_surface = false;
	unsigned int i;

	if (!dcp_backlight_pending(dcp) || !READ_ONCE(dcp->mode_state.valid) || !scanout ||
	    !dcp->connector || !dcp->connector->connected)
		return;
	for (i = 0; i < SWAP_SURFACES; i++)
		have_surface |= !!scanout->fb[i];
	if (!have_surface)
		return;

	dcp->swap.h17p = scanout->request;
	iomfb_present_backlight_h17p(dcp);
}

static void iomfb_backlight_release(struct iomfb_transaction *transaction)
{
	kfree(transaction);
}

static void iomfb_backlight_kick(struct apple_dcp *dcp)
{
	struct iomfb_transaction *transaction;

	transaction = kzalloc_obj(*transaction);
	if (!transaction)
		return;
	transaction->start = iomfb_backlight_start;
	transaction->release = iomfb_backlight_release;
	transaction->brightness_only = true;
	transaction->presentation = true;
	if (iomfb_queue(dcp, transaction))
		kfree(transaction);
}

int iomfb_configure_backlight_h17p(struct apple_dcp *dcp, u32 maximum,
				   bool inherited_valid, u32 inherited,
				   bool default_valid, u32 default_nits)
{
	if (!iomfb_uses_queue(dcp))
		return -EINVAL;
	return dcp_backlight_configure(dcp, maximum, inherited_valid, inherited,
				       default_valid, default_nits, iomfb_backlight_kick);
}

struct iomfb_atomic_transaction {
	struct iomfb_transaction transaction;
	struct drm_atomic_state *state;
	struct drm_crtc *crtc;
	struct apple_dcp *dcp;
	struct iomfb_scanout_h17p *scanout;
};

static void iomfb_atomic_start(struct apple_dcp *dcp,
			       struct iomfb_transaction *transaction)
{
	struct iomfb_atomic_transaction *atomic = container_of(transaction,
					 struct iomfb_atomic_transaction, transaction);

	if (!READ_ONCE(dcp->mode_state.valid) || !dcp->connector || !dcp->connector->connected) {
		schedule_work(&dcp->vblank_wq);
		return;
	}
	atomic->scanout = iomfb_scanout_prepare_h17p(dcp, atomic->crtc, atomic->state);
	if (!atomic->scanout) {
		WRITE_ONCE(dcp->crashed, true);
		schedule_work(&dcp->vblank_wq);
		return;
	}
	dcp->iomfb.next_scanout = atomic->scanout;
	iomfb_flush_h17p(dcp, atomic->crtc, atomic->state);
}

static void iomfb_atomic_release(struct iomfb_transaction *transaction)
{
	struct iomfb_atomic_transaction *atomic = container_of(transaction,
					 struct iomfb_atomic_transaction, transaction);

	if (atomic->scanout && atomic->dcp->iomfb.scanout != atomic->scanout) {
		if (atomic->dcp->iomfb.next_scanout == atomic->scanout)
			atomic->dcp->iomfb.next_scanout = NULL;
		iomfb_scanout_release_h17p(atomic->scanout);
	}
	drm_atomic_state_put(atomic->state);
	kfree(atomic);
}

static void iomfb_queue_atomic(struct apple_dcp *dcp, struct drm_crtc *crtc,
			       struct drm_atomic_state *state)
{
	struct iomfb_atomic_transaction *atomic;
	int ret;

	if (READ_ONCE(dcp->iomfb.presentation_paused)) {
		schedule_work(&dcp->vblank_wq);
		return;
	}
	atomic = kzalloc_obj(*atomic);
	if (!atomic)
		goto failed;
	atomic->transaction.start = iomfb_atomic_start;
	atomic->transaction.presentation = true;
	atomic->transaction.release = iomfb_atomic_release;
	atomic->state = drm_atomic_state_get(state);
	atomic->crtc = crtc;
	atomic->dcp = dcp;
	ret = iomfb_queue(dcp, &atomic->transaction);
	if (!ret)
		return;
	iomfb_atomic_release(&atomic->transaction);
	if (ret == -EAGAIN) {
		schedule_work(&dcp->vblank_wq);
		return;
	}
failed:
	WRITE_ONCE(dcp->crashed, true);
	schedule_work(&dcp->vblank_wq);
}

/* H17P (T8140, T8142): aurora-wip's flush. */
static void dcp_flush_h17p(struct drm_crtc *crtc, struct drm_atomic_state *state)
{
	struct platform_device *pdev = to_apple_crtc(crtc)->dcp;
	struct apple_dcp *dcp = platform_get_drvdata(pdev);

	if (iomfb_uses_queue(dcp)) {
		iomfb_queue_atomic(dcp, crtc, state);
		return;
	}

	/*
	 * DCP does not complete swaps after a link loss.  A plane-only commit
	 * arriving between the reconnect callback and the required modeset
	 * would otherwise leave an unsignalled flip event in front of the
	 * recovery commit.  Modesets set valid_mode before reaching flush.
	 *
	 * The same applies while a connector is still disconnected: resume
	 * re-runs the modeset, which marks the mode valid again, before the
	 * firmware has reported the display back.
	 */
	if (!READ_ONCE(dcp->mode_state.valid) || !dcp->connector || !dcp->connector->connected) {
		schedule_work(&dcp->vblank_wq);
		return;
	}

	if (dcp_channel_busy(&dcp->ch_cmd))
	{
		if (!dcp->ch_cmd.warned_busy) {
			dev_err(dcp->dev, "unexpected busy command channel\n");
			dcp->ch_cmd.warned_busy = true;
		}
		/* HACK: issue a delayed vblank event to avoid timeouts in
		 * drm_atomic_helper_wait_for_vblanks().
		 */
		schedule_work(&dcp->vblank_wq);
		return;
	} else if (dcp->ch_cmd.warned_busy) {
		dcp->ch_cmd.warned_busy = false;
	}

	switch (dcp->fw_compat) {
	case DCP_FIRMWARE_V_12_3:
		iomfb_flush_v12_3(dcp, crtc, state);
		break;
	case DCP_FIRMWARE_V_13_5:
		iomfb_flush_v13_3(dcp, crtc, state);
		break;
	case DCP_FIRMWARE_H17P:
		iomfb_flush_h17p(dcp, crtc, state);
		break;
	default:
		WARN_ONCE(true, "Unexpected firmware version: %u\n", dcp->fw_compat);
		break;
	}
}

void dcp_flush(struct drm_crtc *crtc, struct drm_atomic_state *state)
{
	struct platform_device *pdev = to_apple_crtc(crtc)->dcp;
	struct apple_dcp *dcp = platform_get_drvdata(pdev);

	if (dcp->fw_compat == DCP_FIRMWARE_H17P) {
		dcp_flush_h17p(crtc, state);
		return;
	}

	if (dcp->fw_compat == DCP_FIRMWARE_V_26_6) {
		iomfb_v26_6_flush(dcp, crtc, state);
		return;
	}
	if (dcp->fw_compat == DCP_FIRMWARE_V_14_7) {
		iomfb_v14_7_flush(dcp, crtc, state);
		return;
	}

	/*
	 * DCP does not complete swaps after a link loss.  A plane-only commit
	 * arriving between the reconnect callback and the required modeset
	 * would otherwise leave an unsignalled flip event in front of the
	 * recovery commit.  Modesets set valid_mode before reaching flush.
	 *
	 * The same applies while a connector is still disconnected: resume
	 * re-runs the modeset, which marks the mode valid again, before the
	 * firmware has reported the display back.
	 */
	if ((to_apple_atomic_state(state)->failed_routes & drm_crtc_mask(crtc)) ||
	    !dcp_crtc_route_ready(crtc, state, true) ||
	    !READ_ONCE(dcp->mode_state.valid) || !dcp->connector || !dcp->connector->connected) {
		schedule_work(&dcp->vblank_wq);
		return;
	}

	if (dcp_channel_busy(&dcp->ch_cmd))
	{
		if (!dcp->ch_cmd.warned_busy) {
			dev_err(dcp->dev, "unexpected busy command channel\n");
			dcp->ch_cmd.warned_busy = true;
		}
		/* HACK: issue a delayed vblank event to avoid timeouts in
		 * drm_atomic_helper_wait_for_vblanks().
		 */
		schedule_work(&dcp->vblank_wq);
		return;
	} else if (dcp->ch_cmd.warned_busy) {
		dcp->ch_cmd.warned_busy = false;
	}

	switch (dcp->fw_compat) {
	case DCP_FIRMWARE_V_12_3:
		iomfb_flush_v12_3(dcp, crtc, state);
		break;
	case DCP_FIRMWARE_V_13_5:
		iomfb_flush_v13_3(dcp, crtc, state);
		break;
	case DCP_FIRMWARE_V_14_7_TEMPLATE:
		iomfb_flush_v14_7_0(dcp, crtc, state);
		break;
	default:
		WARN_ONCE(true, "Unexpected firmware version: %u\n", dcp->fw_compat);
		break;
	}
}

static void iomfb_start(struct apple_dcp *dcp)
{
	switch (dcp->fw_compat) {
	case DCP_FIRMWARE_V_12_3:
		iomfb_start_v12_3(dcp);
		break;
	case DCP_FIRMWARE_V_13_5:
		iomfb_start_v13_3(dcp);
		break;
	case DCP_FIRMWARE_V_14_7_TEMPLATE:
		iomfb_start_v14_7_0(dcp);
		break;
	case DCP_FIRMWARE_H17P:
		iomfb_start_h17p(dcp);
		break;
	default:
		WARN_ONCE(true, "Unexpected firmware version: %u\n", dcp->fw_compat);
		break;
	}
}

bool dcp_is_initialized(struct platform_device *pdev)
{
	struct apple_dcp *dcp = platform_get_drvdata(pdev);

	return dcp->active;
}

static void iomfb_recv_message(struct apple_dcp *dcp, u64 message)
{
	enum dcpep_type type = FIELD_GET(IOMFB_MESSAGE_TYPE, message);

	if (dcp->fw_compat == DCP_FIRMWARE_H17P && READ_ONCE(dcp->crashed))
		return;

	if (type == IOMFB_MESSAGE_TYPE_INITIALIZED) {
		/*
		 * H17P reports its interface version (bits 63:48) and a
		 * firmware hash in the high half of the InitComplete word.
		 * The transport below it is unchanged.
		 */
		if (dcp->fw_compat == DCP_FIRMWARE_H17P)
			dev_info(dcp->dev,
				 "IOMFB: init complete %#llx (version %llu)\n",
				 message,
				 FIELD_GET(GENMASK_ULL(63, 48), message));
		iomfb_start(dcp);
	} else if (type == IOMFB_MESSAGE_TYPE_MSG)
		dcpep_got_msg(dcp, message);
	else
		dev_warn(dcp->dev, "Ignoring unknown message %llx\n", message);
}

void iomfb_recv_msg(struct apple_dcp *dcp, u64 message)
{
	if (!iomfb_uses_queue(dcp)) {
		iomfb_recv_message(dcp, message);
		return;
	}

	mutex_lock(&dcp->iomfb.lock);
	WRITE_ONCE(dcp->iomfb.owner, current);
	if (!dcp->iomfb.stopped)
		iomfb_recv_message(dcp, message);
	WRITE_ONCE(dcp->iomfb.owner, NULL);
	iomfb_queue_advance(dcp);
	mutex_unlock(&dcp->iomfb.lock);
}

int iomfb_start_rtkit(struct apple_dcp *dcp)
{
	dma_addr_t shmem_iova;
	int ret;

	if (iomfb_uses_queue(dcp)) {
		mutex_lock(&dcp->iomfb.lock);
		if (dcp->iomfb.active || dcp->iomfb.scanout) {
			mutex_unlock(&dcp->iomfb.lock);
			return -EBUSY;
		}
		dcp->iomfb.stopped = false;
		mutex_unlock(&dcp->iomfb.lock);
	}

	/*
	 * H17P firmware expects the remote allocator endpoint to be started
	 * before IOMFB.  A firmware subsystem whose endpoint was never started
	 * has nothing answering it on the AP side, and without this one the
	 * display DART faults.
	 */
	if (dcp->fw_compat == DCP_FIRMWARE_H17P &&
	    dcp->hw.iomfb_method_profile != DCP_IOMFB_METHODS_H17P_EXTERNAL) {
		ret = apple_rtkit_start_ep(dcp->rtk, REMOTE_ALLOC_ENDPOINT);
		if (ret)
			return ret;
	}
	ret = apple_rtkit_start_ep(dcp->rtk, IOMFB_ENDPOINT);
	if (ret)
		return ret;

	dcp->shmem = dma_alloc_coherent(dcp->dev, DCP_SHMEM_SIZE, &shmem_iova,
					GFP_KERNEL);
	if (!dcp->shmem && dcp->fw_compat == DCP_FIRMWARE_H17P)
		return -ENOMEM;

	dcp_send_message(dcp, IOMFB_ENDPOINT, dcpep_set_shmem(shmem_iova));

	return 0;
}

void iomfb_shutdown(struct apple_dcp *dcp)
{
	/* We're going down */
	dcp->active = false;
	dcp_mode_invalidate(&dcp->mode_state);

	switch (dcp->fw_compat) {
	case DCP_FIRMWARE_V_12_3:
		iomfb_shutdown_v12_3(dcp);
		break;
	case DCP_FIRMWARE_V_13_5:
		iomfb_shutdown_v13_3(dcp);
		break;
	case DCP_FIRMWARE_V_14_7_TEMPLATE:
		iomfb_shutdown_v14_7_0(dcp);
		break;
	case DCP_FIRMWARE_H17P:
		iomfb_shutdown_h17p(dcp);
		break;
	default:
		WARN_ONCE(true, "Unexpected firmware version: %u\n", dcp->fw_compat);
		break;
	}
}
