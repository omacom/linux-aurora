// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Copyright 2021 Alyssa Rosenzweig */

#include <kunit/visibility.h>
#include <linux/align.h>
#include <linux/bitfield.h>
#include <linux/bitmap.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/crc32.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/iommu.h>
#include <linux/kref.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/power_supply.h>
#include <linux/ratelimit.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/soc/apple/dp-tunnel.h>
#include <linux/soc/apple/rtkit.h>
#include <linux/unaligned.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_color_mgmt.h>
#include <drm/drm_drv.h>
#include <drm/drm_edid.h>
#include "dcp-sdr-edid.h"
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_modeset_lock.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_print.h>
#include <drm/drm_vblank.h>

#include "dcp.h"
#include "dcp-hdr-metadata.h"
#include "dcp-internal.h"
#include "iomfb.h"
#include "iomfb_internal.h"
#include "iomfb_m4.h"
#include "dcpext26-lifecycle.h"
#include "parser.h"
#include "trace.h"

struct dcp_identity_trace {
	struct apple_dcp *dcp;
	atomic_t *count;
};

/*
 * DCP asks the AP for "PMUS"/"Temperature" once at start-up and uses it as
 * the initial panel temperature when no 2D panel temperature has been set.
 * macOS answers with the system battery's gas gauge temperature in units of
 * 0.1 K (the AppleSmartBattery "Temperature" property, the SMC B0AT key);
 * the firmware takes off 273.2 K itself. Observed on the J700 under macOS
 * 26.6: a reply of 2997 gave an InitialPanelTemperature of 26.5 C, equal to
 * the battery temperature at that boot.
 *
 * Linux power supplies report temperature in 0.1 C, so add the 273.2 K back.
 */
struct dcp_battery_temp {
	u64 value;
	bool found;
};

static int dcp_battery_temp_cb(struct power_supply *psy, void *data)
{
	struct dcp_battery_temp *bt = data;
	union power_supply_propval val;

	if (psy->desc->type != POWER_SUPPLY_TYPE_BATTERY)
		return 0;
	if (!power_supply_get_property(psy, POWER_SUPPLY_PROP_SCOPE, &val) &&
	    val.intval == POWER_SUPPLY_SCOPE_DEVICE)
		return 0;
	if (power_supply_get_property(psy, POWER_SUPPLY_PROP_TEMP, &val))
		return 0;
	if (val.intval <= -2732)
		return 0;

	bt->value = val.intval + 2732;
	bt->found = true;
	return 1;
}

u64 dcp_pmu_temperature(struct device *dev)
{
	struct dcp_battery_temp bt = { 0 };

	if (IS_REACHABLE(CONFIG_POWER_SUPPLY))
		power_supply_for_each_psy(&bt, dcp_battery_temp_cb);

	if (bt.found) {
		dev_dbg(dev, "PMU temperature from the battery: %llu (0.1 K)\n",
			bt.value);
		return bt.value;
	}

	/*
	 * No system battery (desktops, or the battery driver is not bound):
	 * keep the reply Linux has always sent, 302.9 K.
	 */
	dev_dbg(dev, "no system battery, PMU temperature fallback\n");
	return DCP_PMU_TEMPERATURE_FALLBACK;
}

/*
 * set_matrix coefficients for @ctm, the identity when @ctm is NULL. DRM
 * carries S31.32 sign-magnitude; the firmware takes signed (two's
 * complement) Q32.32, so positive values are identical and only negative
 * coefficients need converting.
 */
void dcp_ctm_to_fw(u64 fw[9], const struct drm_color_ctm *ctm)
{
	for (unsigned int i = 0; i < 9; i++) {
		if (ctm)
			fw[i] = drm_color_ctm_s31_32_to_qm_n(ctm->matrix[i], 32, 32);
		else
			fw[i] = (i % 4 == 0) ? 1ULL << 32 : 0;
	}
}

static void dcp_identity_key_log(void *data, const char *key, u32 key_len,
				 u32 type, u32 size, u32 depth)
{
	struct dcp_identity_trace *trace = data;
	struct apple_dcp *dcp = trace->dcp;
	char safe[65];
	u32 i;

	if (!atomic_add_unless(trace->count, 1, 128))
		return;
	for (i = 0; i < min_t(u32, key_len, sizeof(safe) - 1); i++)
		safe[i] = key[i] >= 0x20 && key[i] <= 0x7e ? key[i] : '?';
	safe[i] = 0;
	dev_info(dcp->dev, "panel identity depth=%u key=%s type=%u size=%u\n",
		 depth, safe, type, size);
}

void dcp_trace_panel_property(struct apple_dcp *dcp, int tag, const char *key,
			      size_t key_len, const void *blob, size_t len)
{
	struct dcp_identity_trace trace;
	u32 slot = tag == 552 ? 0 : tag == 561 ? 1 : tag == 114 ? 2 : 3;
	int ret;

	if (!drm_debug_enabled(DRM_UT_KMS) ||
	    dcp->fw_compat != DCP_FIRMWARE_V_26_6_DESKTOP || !dcp_has_panel(dcp))
		return;
	trace.dcp = dcp;
	trace.count = &dcp->desktop->identity_log_count[slot];
	/* Periodic statistics cannot consume the panel setters' budget. */
	if (!atomic_add_unless(trace.count, 1, 128))
		return;
	dev_info(dcp->dev, "panel identity callback=D%03d bytes=%zu null=%d\n",
		 tag, len, !blob);
	dcp_identity_key_log(&trace, key, key_len, 0, len, 0);
	if (!blob)
		return;
	ret = parse_identity_keys(blob, len, dcp_identity_key_log, &trace);
	if (ret)
		dev_info(dcp->dev, "panel identity inspection stopped: %d\n", ret);
}

/* Other property setters use fixed scalar records, not serialization. */
static void dcp_trace_panel_setter(struct apple_dcp *dcp, int tag,
				   const void *data, size_t len)
{
	struct dcp_identity_trace trace = { .dcp = dcp };
	const char *in = data;
	u32 offset = 0, type, size, expected;
	bool is_null = false;

	if (!drm_debug_enabled(DRM_UT_KMS) ||
	    dcp->fw_compat != DCP_FIRMWARE_V_26_6_DESKTOP || !dcp_has_panel(dcp))
		return;
	switch (tag) {
	case 414:
		offset = 4;
		fallthrough;
	case 563:
		type = 4;
		size = 8;
		expected = offset + 76;
		break;
	case 415:
		offset = 4;
		fallthrough;
	case 565:
		type = 11;
		size = 4;
		expected = offset + 72;
		break;
	case 567:
		type = 9;
		size = 64;
		expected = 128;
		break;
	case 102:
		type = 4;
		size = 4;
		expected = 68;
		break;
	case 104:
		type = 11;
		size = 1;
		expected = 68;
		break;
	default:
		return;
	}
	/* Inspect only the public wire shape, without changing its handler. */
	if (len != expected)
		return;
	if (tag == 414 || tag == 415 || tag == 563 || tag == 565)
		is_null = in[offset + 64 + size] & 1;
	if (tag == 567)
		size = strnlen(in + 64, 64);
	/* Scalar/service setters share the bounded miscellaneous-property budget. */
	trace.count = &dcp->desktop->identity_log_count[3];
	if (!atomic_add_unless(trace.count, 1, 128))
		return;
	dev_info(dcp->dev, "panel identity callback=D%03d bytes=%zu null=%d\n",
		 tag, len, is_null);
	dcp_identity_key_log(&trace, in + offset, strnlen(in + offset, 64),
			     type, is_null ? 0 : size, 0);
}

static int dcp_tx_offset(enum dcp_context_id id)
{
	switch (id) {
	case DCP_CONTEXT_CB:
	case DCP_CONTEXT_CMD:
		return 0x00000;
	case DCP_CONTEXT_OOBCB:
	case DCP_CONTEXT_OOBCMD:
		return 0x08000;
	case DCP_CONTEXT_ASYNC_CB:
		return 0x20000;
	case DCP_CONTEXT_OOBASYNC_CB:
		return 0x28000;
	default:
		return -EINVAL;
	}
}

static int dcp_channel_offset(enum dcp_context_id id)
{
	switch (id) {
	case DCP_CONTEXT_ASYNC_CB:
	case DCP_CONTEXT_ASYNC:
		return 0x40000;
	case DCP_CONTEXT_OOBASYNC_CB:
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
	case DCP_CONTEXT_ASYNC_CB:
	case DCP_CONTEXT_ASYNC:
		return &dcp->ch_async;
	case DCP_CONTEXT_OOBASYNC_CB:
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
static u64 ext26_scanout_bytes(struct iomfb_scanout_h17p *scanout);

static bool iomfb_uses_queue(struct apple_dcp *dcp)
{
	return (dcp->fw_compat == DCP_FIRMWARE_H17P || dcp->fw_compat == DCP_FIRMWARE_V_27_0) &&
	       dcp->hw.iomfb_method_profile != DCP_IOMFB_METHODS_H17G;
}

bool iomfb_channels_idle(struct apple_dcp *dcp)
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
	    DCP_IOMFB_METHODS_H17P_EXTERNAL || dcp_is_ext26(dcp))
		return 0;
	if (dcp->fw_compat == DCP_FIRMWARE_V_27_0 ||
	    atomic_read(&dcp->iomfb.opaque_x_state) != IOMFB_OPAQUE_X_NEEDED)
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
	if (!iomfb_channels_idle(dcp) || dcp->present_state_h17p.pending ||
	    dcp->ext26_control_pending ||
	    READ_ONCE(dcp->iomfb.callback_count))
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
					mod_delayed_work(system_dfl_wq, &dcp->iomfb.backlight_retry,
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
	    dcp->present_state_h17p.pending || READ_ONCE(dcp->iomfb.callback_count) ||
	    list_empty(&dcp->iomfb.pending))
		goto unlock;

	transaction = list_first_entry(&dcp->iomfb.pending,
				       struct iomfb_transaction, link);
	list_del(&transaction->link);
	dcp->iomfb.queued--;
	dcp->iomfb.active = transaction;
	WRITE_ONCE(dcp->iomfb.owner, current);
	dcp->iomfb.deadline = jiffies + msecs_to_jiffies(10000);
	mod_delayed_work(system_dfl_wq, &dcp->iomfb.timeout, msecs_to_jiffies(10000));
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
		mod_delayed_work(system_dfl_wq, &dcp->iomfb.timeout,
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

/* A peer endpoint may finish a dependency while this endpoint has an active
 * transaction. Continue on its serialized owner, not by taking another
 * endpoint's IOMFB mutex from a receive callback. */
struct iomfb_callback {
	struct list_head link;
	dcp_callback_t callback;
	void *cookie;
	u8 data[];
};

int iomfb_queue_callback(struct apple_dcp *dcp, dcp_callback_t callback,
			 const void *data, size_t size, void *cookie)
{
	struct iomfb_callback *item;
	unsigned long flags;
	int ret = 0;

	/* This queue carries control replies and completion watermarks only. */
	if (!callback || size > 64 || (size && !data))
		return -EINVAL;
	item = kzalloc(struct_size(item, data, size), GFP_KERNEL);
	if (!item)
		return -ENOMEM;
	item->callback = callback;
	item->cookie = cookie;
	if (size)
		memcpy(item->data, data, size);
	spin_lock_irqsave(&dcp->iomfb.callback_lock, flags);
	if (READ_ONCE(dcp->iomfb.stopped) || READ_ONCE(dcp->crashed)) {
		ret = -ESHUTDOWN;
	} else if (dcp->iomfb.callback_count >= 64) {
		ret = -EBUSY;
	} else {
		list_add_tail(&item->link, &dcp->iomfb.callbacks);
		dcp->iomfb.callback_count++;
		/* Stop takes this lock before cancelling work, closing enqueue/stop. */
		schedule_work(&dcp->iomfb.callback_work);
	}
	spin_unlock_irqrestore(&dcp->iomfb.callback_lock, flags);
	if (ret)
		kfree(item);
	return ret;
}

static struct iomfb_callback *iomfb_pop_callback(struct apple_dcp *dcp)
{
	struct iomfb_callback *item = NULL;
	unsigned long flags;

	spin_lock_irqsave(&dcp->iomfb.callback_lock, flags);
	if (!list_empty(&dcp->iomfb.callbacks)) {
		item = list_first_entry(&dcp->iomfb.callbacks, struct iomfb_callback, link);
		list_del(&item->link);
		dcp->iomfb.callback_count--;
	}
	spin_unlock_irqrestore(&dcp->iomfb.callback_lock, flags);
	return item;
}

static void iomfb_callback_work(struct work_struct *work)
{
	struct apple_dcp *dcp = container_of(work, struct apple_dcp, iomfb.callback_work);
	struct iomfb_callback *item;

	mutex_lock(&dcp->iomfb.lock);
	while ((item = iomfb_pop_callback(dcp))) {
		if (!dcp->iomfb.stopped && !READ_ONCE(dcp->crashed)) {
			WRITE_ONCE(dcp->iomfb.owner, current);
			item->callback(dcp, item->data, item->cookie);
			WRITE_ONCE(dcp->iomfb.owner, NULL);
		}
		kfree(item);
	}
	iomfb_queue_advance(dcp);
	mutex_unlock(&dcp->iomfb.lock);
}

static void iomfb_edr_tick(struct work_struct *work);

void iomfb_queue_init(struct apple_dcp *dcp)
{
	mutex_init(&dcp->iomfb.lock);
	INIT_LIST_HEAD(&dcp->iomfb.pending);
	INIT_LIST_HEAD(&dcp->iomfb.dropped);
	INIT_WORK(&dcp->iomfb.work, iomfb_queue_work);
	spin_lock_init(&dcp->iomfb.callback_lock);
	INIT_LIST_HEAD(&dcp->iomfb.callbacks);
	INIT_WORK(&dcp->iomfb.callback_work, iomfb_callback_work);
	INIT_DELAYED_WORK(&dcp->iomfb.timeout, iomfb_queue_timeout);
	INIT_DELAYED_WORK(&dcp->iomfb.backlight_retry, iomfb_backlight_retry);
	INIT_DELAYED_WORK(&dcp->iomfb.edr_tick, iomfb_edr_tick);
	atomic_set(&dcp->iomfb.opaque_x_state, IOMFB_OPAQUE_X_WAITING);
}

/*
 * For firmware stopped cold and started again: the calls and the transaction the old
 * one never answered went with it, and would hold the channels and the queue forever.
 */
void iomfb_reset_transport(struct apple_dcp *dcp)
{
	struct iomfb_transaction *transaction;

	if (!iomfb_uses_queue(dcp))
		return;

	cancel_delayed_work_sync(&dcp->iomfb.timeout);
	mutex_lock(&dcp->iomfb.lock);
	transaction = dcp->iomfb.active;
	dcp->iomfb.active = NULL;
	if (transaction) {
		if (transaction->brightness_only)
			dcp->iomfb.backlight_queued = false;
		transaction->release(transaction);
	}
	iomfb_discard_pending(dcp);
	dcp->ch_cmd.depth = 0;
	dcp->ch_oobcmd.depth = 0;
	dcp->ch_cb.depth = 0;
	dcp->ch_oobcb.depth = 0;
	dcp->ch_async.depth = 0;
	dcp->ch_oobasync.depth = 0;
	dcp->present_state_h17p.pending = false;
	dcp->present_state_h17p.accepted = false;
	mutex_unlock(&dcp->iomfb.lock);
}

void iomfb_queue_stop(struct apple_dcp *dcp)
{
	struct iomfb_callback *item;
	unsigned long flags;

	if (!iomfb_uses_queue(dcp))
		return;

	mutex_lock(&dcp->iomfb.lock);
	spin_lock_irqsave(&dcp->iomfb.callback_lock, flags);
	WRITE_ONCE(dcp->iomfb.stopped, true);
	spin_unlock_irqrestore(&dcp->iomfb.callback_lock, flags);
	iomfb_discard_pending(dcp);
	mutex_unlock(&dcp->iomfb.lock);
	cancel_work_sync(&dcp->iomfb.work);
	cancel_work_sync(&dcp->iomfb.callback_work);
	while ((item = iomfb_pop_callback(dcp)))
		kfree(item);
	cancel_delayed_work_sync(&dcp->iomfb.timeout);
	cancel_delayed_work_sync(&dcp->iomfb.backlight_retry);
	cancel_delayed_work_sync(&dcp->iomfb.edr_tick);
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
			!dcp->present_state_h17p.pending && !READ_ONCE(dcp->iomfb.callback_count));
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
		       !dcp->present_state_h17p.pending && iomfb_channels_idle(dcp) &&
		       (!READ_ONCE(dcp->sleep_v27_closed) ||
			!READ_ONCE(dcp->iomfb.callback_count));
		mutex_unlock(&dcp->iomfb.lock);
		if (idle)
			return 0;
		if (time_after_eq(jiffies, deadline))
			return -ETIMEDOUT;
		usleep_range(2000, 4000);
	}
}

static void iomfb_v27_sleep_start(struct apple_dcp *dcp,
				  struct iomfb_transaction *transaction);

int iomfb_queue(struct apple_dcp *dcp, struct iomfb_transaction *transaction)
{
	int ret = 0;

	mutex_lock(&dcp->iomfb.lock);
	if (READ_ONCE(dcp->crashed) || dcp->iomfb.stopped) {
		ret = -EIO;
	} else if (READ_ONCE(dcp->sleep_v27_closed) &&
		   transaction->start != iomfb_v27_sleep_start) {
		ret = -EBUSY;
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

/* Firmware 27 sleep RPCs have no callback chain after a timed-out wait. The queue
 * owns one reference until actual completion or teardown; the waiter owns one.
 * No stack completion and no resetting stacks/rings on timeout.
 */
struct iomfb_v27_sleep_rpc {
	struct iomfb_transaction transaction;
	struct kref ref;
	struct completion done;
	struct dcp_method_entry method;
	u8 data[16];
	u32 input, output, status, power_state;
	bool oob, flags_form, any_reply, inout, a442;
	u8 a442_key;	/* an external processor's A442 rides this call, with this key */
};

static void iomfb_v27_sleep_free(struct kref *ref)
{
	kfree(container_of(ref, struct iomfb_v27_sleep_rpc, ref));
}

static void iomfb_v27_sleep_release(struct iomfb_transaction *transaction)
{
	struct iomfb_v27_sleep_rpc *rpc = container_of(transaction,
		struct iomfb_v27_sleep_rpc, transaction);

	kref_put(&rpc->ref, iomfb_v27_sleep_free);
}

static void iomfb_v27_sleep_done(struct apple_dcp *dcp, void *out, void *cookie)
{
	struct iomfb_v27_sleep_rpc *rpc = cookie;

	rpc->status = rpc->any_reply ? 0 :
		      rpc->output ? (out ? get_unaligned_le32(out + 4) : ~0U) : 0;
	/* With the sleep's flags the reply's first word is not the state. */
	if (rpc->output && !rpc->status && !rpc->flags_form) {
		if (get_unaligned_le32(out) != rpc->power_state)
			rpc->status = ~0U;
	}
	complete(&rpc->done);
}

static void iomfb_v27_a442_done(struct apple_dcp *dcp, void *out, void *cookie)
{
}

static void iomfb_v27_sleep_start(struct apple_dcp *dcp,
				  struct iomfb_transaction *transaction)
{
	struct iomfb_v27_sleep_rpc *rpc = container_of(transaction,
		struct iomfb_v27_sleep_rpc, transaction);

	if (rpc->inout)
		dcp->push_out_prefill = rpc->data;
	dcp_push(dcp, rpc->oob, &rpc->method, rpc->input, rpc->output,
		 rpc->data, iomfb_v27_sleep_done, rpc);
	dcp->push_out_prefill = NULL;
	if ((rpc->a442 || rpc->a442_key) && !rpc->oob) {
		static const struct dcp_method_entry a442 = {
			.name = "v27_a442", .tag = { 'A', '4', '4', '2' },
		};
		u8 in[92];

		memset(in, 0xaa, sizeof(in));
		memset(in, 0, 16);
		in[8] = rpc->a442_key ?: 0x41;
		/* An external processor's wake: no indexes, and the block's size word. */
		if (rpc->a442_key) {
			memset(in + 80, 0, 12);
			in[84] = 0x20;
		}
		dcp_push(dcp, true, &a442, sizeof(in), 4100, in, iomfb_v27_a442_done, NULL);
	}
}

/*
 * The sleep RPCs' argument bytes, from the DT: A484's eight after the power state, or A500's
 * four for the sleep followed by its four for the wake.
 */
#define IOMFB_V27_SLEEP_ARGS	8

static int iomfb_v27_sleep_args(struct apple_dcp *dcp, bool a500, u8 *args)
{
	return of_property_read_u8_array(dcp->dev->of_node,
					 a500 ? "apple,sleep-a500-args" : "apple,sleep-power-args",
					 args, IOMFB_V27_SLEEP_ARGS);
}

/* Whether the DT gives both sleep RPCs' arguments; without them neither is sent. */
bool iomfb_v27_sleep_described(struct apple_dcp *dcp)
{
	u8 args[IOMFB_V27_SLEEP_ARGS];

	return !iomfb_v27_sleep_args(dcp, false, args) &&
	       !iomfb_v27_sleep_args(dcp, true, args);
}

/*
 * The display's own power state around the sleep, from the DT: the eight bytes after the
 * power state in A484's plain form, which moves that state without the sleep's hard one; or
 * the display stage's arguments made before it, A414's eight followed by A487's four.
 */
int iomfb_v27_display_args(struct apple_dcp *dcp, bool stage, u8 *args)
{
	return of_property_read_u8_array(dcp->dev->of_node,
					 stage ? "apple,sleep-display-stage-args" :
						 "apple,sleep-display-power-args",
					 args, stage ? IOMFB_V27_STAGE_ARGS : IOMFB_V27_SLEEP_ARGS);
}

/* Whether the DT gives both; without them the display's own power state is left alone. */
bool iomfb_v27_display_described(struct apple_dcp *dcp)
{
	u8 args[IOMFB_V27_STAGE_ARGS];

	return !iomfb_v27_display_args(dcp, false, args) &&
	       !iomfb_v27_display_args(dcp, true, args);
}

/*
 * macOS sends every A484 of a V27 sleep and wake, both forms, on the command context, and its
 * power-on callbacks come back on the callback context.
 */
static bool v27_sleep_power_oob;
module_param(v27_sleep_power_oob, bool, 0644);
MODULE_PARM_DESC(v27_sleep_power_oob, "V27 sleep: the power state calls on the out-of-band context (0, default: the command context)");

/* macOS's A442 (key 0x41) on the out-of-band context alongside the display's power-on. */
static bool v27_a442 = true;
module_param(v27_a442, bool, 0644);
MODULE_PARM_DESC(v27_a442, "Firmware 27 wake: macOS's out-of-band A442 alongside the plain A484(1)");

static int iomfb_v27_rpc(struct apple_dcp *dcp, bool a500, bool on, bool flags);

int iomfb_v27_sleep_rpc(struct apple_dcp *dcp, bool a500, bool on)
{
	return iomfb_v27_rpc(dcp, a500, on, true);
}

/* A484's plain form: the display's own (soft) power state, off and on. */
int iomfb_v27_sleep_plain(struct apple_dcp *dcp, bool on)
{
	return iomfb_v27_rpc(dcp, false, on, false);
}

static int iomfb_v27_rpc(struct apple_dcp *dcp, bool a500, bool on, bool flags)
{
	struct iomfb_v27_sleep_rpc *rpc;
	u8 args[IOMFB_V27_SLEEP_ARGS];
	int ret;

	if (!IS_ENABLED(CONFIG_APPLE_SOC_SLEEP) ||
	    dcp->fw_compat != DCP_FIRMWARE_V_27_0 ||
	    dcp->hw.iomfb_method_profile == DCP_IOMFB_METHODS_H17G ||
	    (a500 || flags ? iomfb_v27_sleep_args(dcp, a500, args) :
			     iomfb_v27_display_args(dcp, false, args)))
		return -EOPNOTSUPP;
	rpc = kzalloc_obj(*rpc);
	if (!rpc)
		return -ENOMEM;
	kref_init(&rpc->ref);
	init_completion(&rpc->done);
	rpc->status = ~0U;
	rpc->transaction.start = iomfb_v27_sleep_start;
	rpc->transaction.release = iomfb_v27_sleep_release;
	rpc->method = (struct dcp_method_entry) {
		.name = a500 ? "v27_sleep_500" : "v27_sleep_power",
	};
	memcpy(rpc->method.tag, a500 ? "A500" : "A484", 4);
	/*
	 * Off, before each DCP's and DCPEXT's sleep idle: A484 with the sleep's flags, then
	 * A500's sleep form. On, after the wake: A500's wake form, then A484 with the flags.
	 */
	if (a500) {
		rpc->input = 4;
		memcpy(rpc->data, args + (on ? 4 : 0), 4);
	} else {
		rpc->input = 16;
		rpc->output = sizeof(struct dcp_set_power_state_resp);
		rpc->oob = READ_ONCE(v27_sleep_power_oob);
		rpc->power_state = on;
		rpc->a442 = on && !flags && READ_ONCE(v27_a442);
		put_unaligned_le64(on, rpc->data);
		/*
		 * The firmware keeps a hard and a soft power state, and the flags move the hard one,
		 * so both directions carry them; the plain form moves the soft one. The plain form
		 * after a flagged off answers ok and changes nothing: the panel stays dark and the
		 * next swap times out.
		 */
		memcpy(rpc->data + 8, args, sizeof(args));
		rpc->flags_form = flags;
	}
	kref_get(&rpc->ref);
	ret = iomfb_queue(dcp, &rpc->transaction);
	if (ret)
		iomfb_v27_sleep_release(&rpc->transaction);
	else if (!wait_for_completion_timeout(&rpc->done, msecs_to_jiffies(1000)))
		ret = -ETIMEDOUT;
	else if (rpc->status)
		ret = -EREMOTEIO;
	else if (rpc->output)
		WRITE_ONCE(dcp->pipe_enabled_h17p, !!rpc->power_state);
	kref_put(&rpc->ref, iomfb_v27_sleep_free);
	return ret;
}

static int iomfb_v27_call(struct apple_dcp *dcp, const char *tag, const void *in,
			  u32 in_len, u32 out_len, bool inout, u8 a442_key);

/* One call of the display stage around a V27 sleep, by tag; any reply is accepted. */
int iomfb_v27_sleep_call(struct apple_dcp *dcp, const char *tag, const void *in,
			 u32 in_len, u32 out_len)
{
	return iomfb_v27_call(dcp, tag, in, in_len, out_len, false, 0);
}

/*
 * A call without input, and macOS's A442 with @key on the out-of-band context right behind
 * it, before the call's answer: an external processor's wake sends A489 so.
 * A442's own answer is not waited for.
 */
int iomfb_v27_sleep_call_a442(struct apple_dcp *dcp, const char *tag, u32 out_len, u8 key)
{
	return iomfb_v27_call(dcp, tag, NULL, 0, out_len, false, key);
}

/*
 * A115, an in/out word: 0 after the sleep's flags-form A484(0), 1 after the wake's flags-form
 * A484(1), as macOS.
 */
int iomfb_v27_sleep_a115(struct apple_dcp *dcp, bool awake)
{
	u32 v = awake;

	return iomfb_v27_call(dcp, "A115", &v, 0, sizeof(v), true, 0);
}

static int iomfb_v27_call(struct apple_dcp *dcp, const char *tag, const void *in,
			  u32 in_len, u32 out_len, bool inout, u8 a442_key)
{
	struct iomfb_v27_sleep_rpc *rpc;
	int ret;

	if (!IS_ENABLED(CONFIG_APPLE_SOC_SLEEP) ||
	    dcp->fw_compat != DCP_FIRMWARE_V_27_0 || in_len > sizeof(rpc->data) ||
	    (inout && out_len > sizeof(rpc->data)))
		return -EOPNOTSUPP;
	rpc = kzalloc_obj(*rpc);
	if (!rpc)
		return -ENOMEM;
	kref_init(&rpc->ref);
	init_completion(&rpc->done);
	rpc->status = ~0U;
	rpc->any_reply = true;
	rpc->transaction.start = iomfb_v27_sleep_start;
	rpc->transaction.release = iomfb_v27_sleep_release;
	rpc->method = (struct dcp_method_entry) { .name = "v27_display_stage" };
	memcpy(rpc->method.tag, tag, 4);
	rpc->input = in_len;
	rpc->output = out_len;
	rpc->inout = inout;
	rpc->a442_key = a442_key;
	if (in)
		memcpy(rpc->data, in, inout ? out_len : in_len);
	kref_get(&rpc->ref);
	ret = iomfb_queue(dcp, &rpc->transaction);
	if (ret)
		iomfb_v27_sleep_release(&rpc->transaction);
	else if (!wait_for_completion_timeout(&rpc->done, msecs_to_jiffies(1000)))
		ret = -ETIMEDOUT;
	kref_put(&rpc->ref, iomfb_v27_sleep_free);
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
static void dcp_push_context(struct apple_dcp *dcp, enum dcp_context_id context,
			     const struct dcp_method_entry *call, u32 in_len,
			     u32 out_len, void *data, dcp_callback_t cb, void *cookie)
{
	struct dcp_method_entry resolved = *call;

#if IS_ENABLED(CONFIG_DRM_APPLE_KUNIT_TEST)
	/* A KUnit test answers for the firmware. */
	if (dcp->test_push) {
		dcp->test_push(dcp, call, cb, cookie);
		return;
	}
#endif

	if (dcp_fail_stop(dcp) && READ_ONCE(dcp->crashed))
		return;

	if (dcp->hw.iomfb_method_profile == DCP_IOMFB_METHODS_H17G &&
	    call->tag_h17g[0])
		memcpy(resolved.tag, call->tag_h17g, sizeof(resolved.tag));
	if (dcp_is_ext26(dcp) && call->tag_ext26[0])
		memcpy(resolved.tag, call->tag_ext26, sizeof(resolved.tag));
	call = &resolved;

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
	/* An unwritten status must not look like a successful response. */
	if (out_len && dcp->push_out_prefill)
		memcpy(out_data + in_len, dcp->push_out_prefill, out_len);
	else if (out_len)
		memset(out_data + in_len, dcp_is_ext26(dcp) ? 0 : 0xff, out_len);

	trace_iomfb_push(dcp, call, context, offset, depth);

	ch->callbacks[depth] = cb;
	ch->cookies[depth] = cookie;
	ch->output[depth] = out + sizeof(header) + in_len;
	ch->header[depth] = header;
	ch->end[depth] = offset + ALIGN(data_len, DCP_PACKET_ALIGNMENT);

	if (dcp_is_ext26(dcp))
		dma_wmb();
	dcp_send_message(dcp, IOMFB_ENDPOINT,
			 dcpep_msg(context, data_len, offset));
	/* Only a successful transport send can establish the readiness boundary. */
	if (dcp_is_ext26(dcp) && !memcmp(call->tag, "A473", 4) && in_len == 16 &&
	    get_unaligned_le64(data) == 1) {
		bool sent = !READ_ONCE(dcp->crashed) && !READ_ONCE(dcp->quiescing) &&
			    !READ_ONCE(dcp->ext26_blocked);

		WRITE_ONCE(dcp->ext26_power_ns, sent ? ktime_get_ns() : 0);
		WRITE_ONCE(dcp->ext26_power_sent, sent);
	}
}

void dcp_push(struct apple_dcp *dcp, bool oob,
	      const struct dcp_method_entry *call, u32 in_len, u32 out_len,
	      void *data, dcp_callback_t cb, void *cookie)
{
	/* Reply chains stay on the serialized receiver; callers enqueue copies. */
	if (iomfb_uses_queue(dcp) && READ_ONCE(dcp->iomfb.owner) != current) {
		if (iomfb_queue_command(dcp, oob, call, in_len, out_len,
					data, cb, cookie))
			WRITE_ONCE(dcp->crashed, true);
		return;
	}

	dcp_push_context(dcp, dcp_call_context(dcp, oob), call,
			 in_len, out_len, data, cb, cookie);
}

/*
 * Push one command on the independent OOB command stream while a transaction
 * on the command stream is still pending (dcp_push() would queue it behind
 * that transaction). For queries the firmware serves concurrently, such as a
 * paired modeset's DSC handoff while the primary's A413 waits for its peer.
 */
int dcp_push_oob_concurrent(struct apple_dcp *dcp, const struct dcp_method_entry *call,
			    u32 in_len, u32 out_len, void *data, dcp_callback_t cb,
			    void *cookie)
{
	int ret = 0;

	mutex_lock(&dcp->iomfb.lock);
	if (READ_ONCE(dcp->crashed) || dcp->iomfb.stopped) {
		ret = -ESHUTDOWN;
	} else if (dcp->ch_oobcmd.depth || dcp->ch_oobcb.depth || dcp->ch_oobasync.depth ||
		   READ_ONCE(dcp->iomfb.owner)) {
		ret = -EBUSY;
	} else {
		WRITE_ONCE(dcp->iomfb.owner, current);
		dcp_push_context(dcp, dcp_call_context(dcp, true), call,
				 in_len, out_len, data, cb, cookie);
		WRITE_ONCE(dcp->iomfb.owner, NULL);
	}
	mutex_unlock(&dcp->iomfb.lock);
	return ret;
}

/* A nested query must return on the firmware-initiated stream it answers. */
int dcp_push_nested(struct apple_dcp *dcp, enum dcp_context_id origin,
		    const struct dcp_method_entry *call, u32 in_len, u32 out_len,
		    void *data, dcp_callback_t cb, void *cookie)
{
	enum dcp_context_id context;
	struct dcp_channel *ch;

	if (READ_ONCE(dcp->iomfb.owner) != current || READ_ONCE(dcp->crashed))
		return -ESHUTDOWN;
	switch (origin) {
	case DCP_CONTEXT_CB:
	case DCP_CONTEXT_OOBCB:
		context = origin;
		break;
	case DCP_CONTEXT_ASYNC:
		context = DCP_CONTEXT_ASYNC_CB;
		break;
	case DCP_CONTEXT_OOBASYNC:
		context = DCP_CONTEXT_OOBASYNC_CB;
		break;
	default:
		return -EINVAL;
	}
	ch = dcp_get_channel(dcp, context);
	if (!ch || !ch->depth || ch->depth >= DCP_MAX_CALL_DEPTH ||
	    (u64)sizeof(struct dcp_packet_header) + in_len + out_len > 0x8000)
		return -EINVAL;
	dcp_push_context(dcp, context, call, in_len, out_len, data, cb, cookie);
	return READ_ONCE(dcp->crashed) ? -EPROTO : 0;
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
	if (dcp_is_ext26(dcp))
		dma_wmb();
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

/*
 * A Type-C port whose pipelines do not follow their CRTC is driven by the
 * pipeline its route is on; @crtc belonging to another one has lost it.
 */
static bool dcp_crtc_lost_route(struct drm_crtc *crtc, struct apple_connector *connector)
{
	struct platform_device *pdev = READ_ONCE(connector->dcp);

	return connector->port_encoder && pdev &&
	       !dcp_typec_follows_crtc(platform_get_drvdata(pdev)) &&
	       to_apple_crtc(crtc)->dcp != pdev;
}

static int dcp_set_crtc_active(struct drm_crtc *crtc,
			       struct drm_modeset_acquire_ctx *ctx, bool active)
{
	struct drm_crtc_state *crtc_state;
	struct drm_atomic_commit *state;
	int ret;

	state = drm_atomic_commit_alloc(crtc->dev);
	if (!state)
		return -ENOMEM;
	state->acquire_ctx = ctx;
	crtc_state = drm_atomic_get_crtc_state(state, crtc);
	if (IS_ERR(crtc_state)) {
		ret = PTR_ERR(crtc_state);
	} else {
		crtc_state->active = active;
		ret = drm_atomic_commit(state);
	}
	drm_atomic_commit_put(state);
	return ret;
}

/*
 * @power_cycle: the firmware powered the pipe down with the lost link, so a
 * modeset alone cannot bring it back. Turn the CRTC off and on instead, which
 * reconnects the DPTX and powers the pipe on before the mode is set.
 */
static int dcp_retrain_active_crtc(struct apple_connector *connector,
				   struct apple_dcp *dcp, bool power_cycle)
{
	struct drm_device *dev = connector->base.dev;
	struct drm_modeset_acquire_ctx ctx;
	struct drm_crtc *crtc;
	int ret;

	DRM_MODESET_LOCK_ALL_BEGIN(dev, ctx, 0, ret);

	/*
	 * A queued READY can wait here behind a layout-changing commit. That
	 * commit withdraws the native26 catalogue and queues rediscovery; the
	 * reconnect can already have cleared external_stopped again. Recheck
	 * catalogue admission under the modeset locks, before resetting a CRTC
	 * against a mode that mode_fixup must reject. A completed modeset also
	 * needs no replay. Rediscovery's READY will retry when modes are admitted.
	 */
	crtc = connector->base.state ? connector->base.state->crtc : NULL;
	if (crtc && crtc->state && crtc->state->active &&
	    dcp_crtc_lost_route(crtc, connector)) {
		/*
		 * The port's route moved to another pipeline (an HDMI display
		 * took the hybrid): this CRTC cannot drive the display any
		 * more, and a reset would only be refused.  Release it, so the
		 * hotplug that follows lets the compositor pair the display
		 * with the pipeline that now drives it, and this pipeline's
		 * own display with this CRTC.
		 */
		struct drm_mode_set set = { .crtc = crtc };

		ret = drm_atomic_helper_set_config(&set, &ctx);
	} else if (dcp_is_ext26(dcp) && crtc && crtc->state &&
		   (READ_ONCE(dcp->mode_state.valid) ||
		    !dcp_has_mode(dcp, &crtc->state->mode))) {
		ret = 0;
	} else if (crtc && crtc->state && crtc->state->active &&
		   dcp_crtc_can_retrain(crtc, connector) && power_cycle) {
		/* A commit that held the locks first may have set the mode. */
		ret = READ_ONCE(dcp->mode_state.valid) ? 0 :
		      dcp_set_crtc_active(crtc, &ctx, false);
		if (!ret && !crtc->state->active)
			ret = dcp_set_crtc_active(crtc, &ctx, true);
	} else if (crtc && crtc->state && crtc->state->active &&
		   dcp_crtc_can_retrain(crtc, connector)) {
		ret = drm_atomic_helper_reset_crtc(crtc, &ctx);
	} else {
		ret = 0;
	}

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

static void dcp_clear_connector_edid(struct apple_connector *connector)
{
	lockdep_assert_held(&connector->base.dev->mode_config.mutex);
	drm_edid_connector_update(&connector->base, NULL);
	drm_edid_free(connector->drm_edid);
	connector->drm_edid = NULL;
	connector->drm_edid_firmware_derived = false;
}

static void dcp_consume_edid_invalidation(struct apple_connector *connector)
{
	lockdep_assert_held(&connector->base.dev->mode_config.mutex);
	/* Preserve a disconnect even if the port reconnected before probing. */
	if (xchg(&connector->edid_invalidation_pending, false))
		dcp_clear_connector_edid(connector);
}

/*
 * The external H17P pipeline learns the modes of each cable connection from
 * firmware messages that can lag the link; see dcp-announce.h.
 */
static bool dcp_announce_gated(struct apple_dcp *dcp,
			       struct apple_connector *connector)
{
	return dcp->hw.iomfb_method_profile == DCP_IOMFB_METHODS_H17P_EXTERNAL &&
	       READ_ONCE(dcp->connector) == connector;
}

/*
 * Queue the timeout work of a held connection: at once for a hold whose
 * timeout has not started for the current cable generation, as only that
 * work starts it.
 */
void dcp_announce_arm(struct apple_dcp *dcp)
{
	struct dcp_announce *a = &dcp->announce;
	unsigned long now = jiffies, delay = 0;

	lockdep_assert_held(&dcp->hpd_mutex);
	if (a->stopped || !a->holding)
		return;
	if (a->timed && a->generation == dcp->typec_generation &&
	    time_before(now, a->deadline))
		delay = a->deadline - now;
	mod_delayed_work(system_freezable_wq, &dcp->announce_wq, delay);
}

/*
 * Evaluate what DRM is to report for the connection.  @start starts the
 * timeout of a held connection; only the timeout work does that.
 */
static enum dcp_announce_status
dcp_announce_eval(struct apple_dcp *dcp, struct apple_connector *connector,
		  bool start)
{
	struct dcp_announce *a = &dcp->announce;
	struct dcp_announce_facts facts = {
		.generation = dcp->typec_generation,
		.connected = READ_ONCE(connector->connected),
		.present = !READ_ONCE(dcp->external_physically_absent),
		.failed = READ_ONCE(dcp->external_error) || READ_ONCE(dcp->crashed),
		.lost = dcp_announce_lost(a, dcp->typec_generation),
	};
	enum dcp_announce_status status;
	unsigned long now = jiffies;
	bool held = a->holding, announced = a->announced;

	lockdep_assert_held(&dcp->hpd_mutex);
	if (facts.connected && facts.present && !facts.failed)
		facts.ready = dcp_external_modes_ready(dcp);
	status = dcp_announce_update(a, &facts, now);
	if (status == DCP_ANNOUNCE_HOLD) {
		if (start)
			dcp_announce_start(a, now);
		dcp_announce_arm(dcp);
	} else if (held) {
		/* The hold ended: its timeout work has nothing left to do. */
		cancel_delayed_work(&dcp->announce_wq);
		if (status == DCP_ANNOUNCE_CONNECTED && !facts.ready && !facts.failed)
			dev_info(dcp->dev, "display modes not ready after %u ms\n",
				 announced ? DCP_ANNOUNCE_RESUME_TIMEOUT_MS :
				 DCP_ANNOUNCE_TIMEOUT_MS);
	}
	return status;
}

/*
 * Evaluate the connection outside the hotplug worker.  An event that this
 * makes owed is the worker's to send, so queue it, unless the announcement
 * is stopped: dcp_enable_typec_work() queues it then.
 */
static void dcp_announce_poll(struct apple_dcp *dcp, struct apple_connector *connector,
			      bool start)
{
	bool owed = dcp_announce_owed(&dcp->announce);

	dcp_announce_eval(dcp, connector, start);
	if (!owed && dcp_announce_owed(&dcp->announce) && !dcp->announce.stopped)
		schedule_work(&connector->hotplug_wq);
}

/*
 * The timeout of a held connection.  It runs on the freezable workqueue, as
 * the link work does, so the timeout of a hold starts here, once the link
 * can come back for the hold's generation.  Everything it does is under
 * hpd_mutex: a hold that ended meanwhile, or an announcement stopped for
 * teardown or sleep, gets nothing more from here.
 */
void dcp_announce_work(struct work_struct *work)
{
	struct apple_dcp *dcp = container_of(to_delayed_work(work), struct apple_dcp,
					     announce_wq);
	struct apple_connector *con = READ_ONCE(dcp->connector);

	scoped_guard(mutex, &dcp->hpd_mutex) {
		if (dcp->announce.holding && !dcp->announce.stopped && con &&
		    dcp_announce_gated(dcp, con) &&
		    READ_ONCE(con->dcp) == to_platform_device(dcp->dev))
			dcp_announce_poll(dcp, con, true);
	}
	dcp_test_point(dcp, DCP_TEST_ANNOUNCE_TIMER_DONE);
}

/* The connection state DRM reports for @connector. */
bool dcp_connector_connected(struct apple_connector *connector)
{
	struct platform_device *pdev = READ_ONCE(connector->dcp);
	struct apple_dcp *dcp = pdev ? platform_get_drvdata(pdev) : NULL;

	if (!dcp || !dcp_announce_gated(dcp, connector)) {
		/*
		 * A display behind a Thunderbolt tunnel whose catalog has no
		 * usable timing (every colour element virtual: no pixel clock
		 * yet) is not reported: compositors abort on a connected output
		 * without a mode. The catalog that brings modes ends with the
		 * firmware's hotplug notification, which probes again.
		 */
		if (dcp && READ_ONCE(connector->connected) && dcp_is_usb4_output(dcp) &&
		    !READ_ONCE(dcp->nr_modes)) {
			dev_dbg_ratelimited(dcp->dev,
					    "tunnel display without usable modes: reported disconnected\n");
			return false;
		}
		return READ_ONCE(connector->connected);
	}

	guard(mutex)(&dcp->hpd_mutex);
	dcp_announce_poll(dcp, connector, false);
	return dcp->announce.announced;
}

/* The CRTC userspace keeps active on @connector while its link is down. */
static struct drm_crtc *dcp_external_unlinked_crtc(struct apple_dcp *dcp,
						   struct apple_connector *connector)
{
	struct drm_connector_state *conn_state = connector->base.state;
	struct drm_crtc *crtc = conn_state ? conn_state->crtc : NULL;

	drm_modeset_lock_assert_held(&connector->base.dev->mode_config.connection_mutex);
	if (!crtc || !crtc->state || !crtc->state->active ||
	    READ_ONCE(dcp->mode_state.valid))
		return NULL;
	return crtc;
}

static bool dcp_external_needs_relink(struct apple_dcp *dcp,
				      struct apple_connector *connector)
{
	struct drm_device *dev = connector->base.dev;
	struct drm_modeset_acquire_ctx ctx;
	bool unlinked = false;
	int ret;

	DRM_MODESET_LOCK_ALL_BEGIN(dev, ctx, 0, ret);
	unlinked = !!dcp_external_unlinked_crtc(dcp, connector);
	DRM_MODESET_LOCK_ALL_END(dev, ctx, ret);

	return unlinked;
}

/* Commit @crtc with its mode set again and the link of @connector good. */
static int dcp_external_commit_relink(struct apple_connector *connector,
				      struct drm_crtc *crtc,
				      struct drm_modeset_acquire_ctx *ctx)
{
	struct drm_connector_state *conn_state;
	struct drm_crtc_state *crtc_state;
	struct drm_atomic_commit *state;
	int ret;

	state = drm_atomic_commit_alloc(crtc->dev);
	if (!state)
		return -ENOMEM;
	state->acquire_ctx = ctx;
	crtc_state = drm_atomic_get_crtc_state(state, crtc);
	if (IS_ERR(crtc_state)) {
		ret = PTR_ERR(crtc_state);
		goto out;
	}
	conn_state = drm_atomic_get_connector_state(state, &connector->base);
	if (IS_ERR(conn_state)) {
		ret = PTR_ERR(conn_state);
		goto out;
	}
	crtc_state->connectors_changed = true;
	conn_state->link_status = DRM_LINK_STATUS_GOOD;
	ret = drm_atomic_commit(state);
out:
	drm_atomic_commit_put(state);

	return ret;
}

/*
 * Set the mode of the CRTC userspace keeps active on @connector again.  If
 * that fails, or leaves the link down, mark the link bad and lose the
 * connection of cable generation @generation.  Both are decided under the
 * modeset locks, so a CRTC that userspace switches off is either switched
 * off first and left alone, or found lost by dcp_external_crtc_released().
 *
 * Returns the error that lost the connection, or 0.
 */
static int dcp_external_reset_crtc(struct apple_dcp *dcp,
				   struct apple_connector *connector, u64 generation)
{
	struct drm_device *dev = connector->base.dev;
	struct drm_modeset_acquire_ctx ctx;
	struct drm_crtc *crtc;
	int ret, err = 0;

	DRM_MODESET_LOCK_ALL_BEGIN(dev, ctx, 0, ret);

	err = 0;
	crtc = dcp_external_unlinked_crtc(dcp, connector);
	if (!crtc)
		goto unlock;
	ret = dcp_external_commit_relink(connector, crtc, &ctx);
	if (ret == -EDEADLK)
		goto unlock;
	if (!ret && READ_ONCE(dcp->mode_state.valid))
		goto unlock;

	err = ret ?: -EIO;
	ret = 0;
	scoped_guard(mutex, &dcp->hpd_mutex) {
		if (generation == dcp->typec_generation) {
			connector->base.state->link_status = DRM_LINK_STATUS_BAD;
			dcp_announce_lose(&dcp->announce, generation);
			/* The caller's event is the one this owes. */
			dcp_announce_notify(&dcp->announce,
					    dcp_announce_eval(dcp, connector, false));
		} else {
			/* A newer connection gets a link of its own. */
			err = 0;
		}
	}
unlock:
	DRM_MODESET_LOCK_ALL_END(dev, ctx, ret);

	return ret ?: err;
}

/*
 * The link of an external display came back, or was disturbed, under a CRTC
 * that userspace keeps active.  Userspace sees the display it had and need
 * not set its mode again, so set it here.  A mode that needs its link
 * proven, such as the 180 Hz exception mode, is admitted only against the
 * EDID a probe read for the current cable generation (see
 * dcp_180hz_mode_current()), so probe first, as userspace would.
 *
 * A link that cannot be set up again is marked bad, and the connection is
 * lost: reported gone until userspace releases the CRTC, see
 * dcp-announce.h.  The caller sends the hotplug event.
 */
static void dcp_external_relink(struct apple_dcp *dcp,
				struct apple_connector *connector)
{
	struct drm_device *dev = connector->base.dev;
	u64 generation;
	int ret;

	if (!dcp_external_needs_relink(dcp, connector))
		return;

	scoped_guard(mutex, &dcp->hpd_mutex)
		generation = dcp->typec_generation;
	scoped_guard(mutex, &dev->mode_config.mutex)
		connector->base.funcs->fill_modes(&connector->base,
						  dev->mode_config.max_width,
						  dev->mode_config.max_height);

	ret = dcp_external_reset_crtc(dcp, connector, generation);
	if (ret)
		dev_warn(dcp->dev, "display link not restored (%d), reporting it disconnected\n",
			 ret);
}

/*
 * The CRTC of the external display was switched off.  If its connection was
 * lost, report the display again, as a new connection, once per cable
 * generation (see dcp_announce_retry()).
 */
void dcp_external_crtc_released(struct apple_dcp *dcp)
{
	struct apple_connector *con = READ_ONCE(dcp->connector);

	guard(mutex)(&dcp->hpd_mutex);
	if (dcp_announce_retry(&dcp->announce, dcp->typec_generation) &&
	    !dcp->announce.stopped && con &&
	    READ_ONCE(con->dcp) == to_platform_device(dcp->dev))
		schedule_work(&con->hotplug_wq);
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

void dcp_hotplug(struct work_struct *work)
{
	struct apple_connector *connector;
	struct platform_device *pdev;
	struct apple_dcp *dcp;
	bool connected, gated;
	bool notify_only;
	unsigned int reasons;
	int ret;

	connector = container_of(work, struct apple_connector, hotplug_wq);
	reasons = atomic_xchg(&connector->hotplug_reasons, 0);
	notify_only = (reasons & DCP_HOTPLUG_ROUTE_FAILURE) &&
		      !(reasons & DCP_HOTPLUG_READY);
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
		dcp_notify_hotplug(connector);
		apple_connector_backlight_sync(connector);
		/* The display left: another may get its pair back. */
		dcp_peer_limit_update_v27(connector->base.dev);
		return;
	}
	dcp = platform_get_drvdata(pdev);
	dev_info(dcp->dev, "%s() connected:%d valid_mode:%d nr_modes:%u\n", __func__,
		 connector->connected, READ_ONCE(dcp->mode_state.valid), dcp->nr_modes);
	if (READ_ONCE(dcp->mode_state.valid))
		dcp->tunnel_power_cycles = 0;

	scoped_guard(mutex, &connector->base.dev->mode_config.mutex) {
		if (READ_ONCE(connector->dcp) == pdev)
			dcp_update_connector_dimensions(dcp, &connector->base);
		if (!READ_ONCE(connector->connected))
			dcp_clear_connector_edid(connector);
	}

	gated = dcp_announce_gated(dcp, connector);
	if (gated) {
		enum dcp_announce_status status;
		bool notify;

		scoped_guard(mutex, &dcp->hpd_mutex) {
			status = dcp_announce_eval(dcp, connector, false);
			notify = dcp_announce_notify(&dcp->announce, status);
			connected = dcp->announce.announced;
		}
		/* The last missing fact or the timeout queues this work again. */
		if (!notify)
			return;
	} else {
		connected = READ_ONCE(connector->connected);
	}

	/*
	 * DCP defers link training until we set a display mode. But we set
	 * display modes from atomic_flush, so userspace needs to trigger a
	 * flush, or the CRTC gets no signal.
	 */
	if (gated) {
		if (!notify_only && connector->base.state &&
		    !READ_ONCE(dcp->mode_state.valid) && connected)
			dcp_external_relink(dcp, connector);
	} else if (!notify_only && connector->base.state &&
		   !READ_ONCE(dcp->mode_state.valid) && connected &&
		   /*
		    * Not for a Type-C CRTC powered off with the display
		    * attached: it has no link, and dcp_poweron() retrains one
		    * on wake. A link reported bad then makes wlroots replace
		    * the output on wake, which powers the CRTC off and on
		    * again, and that wake does the same, forever.
		    */
		   !READ_ONCE(dcp->typec_crtc_off) &&
		   /*
		    * Nor while a Type-C CRTC enable runs: dcp_poweron() reconnects
		    * the link, and the HPD that answers it comes before the mode
		    * set of that enable, not after a failed one. A bad link there
		    * makes wlroots destroy the output it is enabling, which
		    * disables the CRTC, and the next enable does the same, forever.
		    */
		   !READ_ONCE(dcp->typec_enable_pending) &&
		   !READ_ONCE(dcp->mode_state.changing) &&
		   (!dcp_uses_t6020_tunnel_flow(dcp) ||
		    dcp->tunnel_power_cycles < DCP_TUNNEL_POWER_CYCLES)) {
		bool power_cycle = dcp_uses_t6020_tunnel_flow(dcp);

		/*
		 * A rebuilt T6020 tunnel is power cycled here; a link reported
		 * bad would only have userspace retry the modeset that fails.
		 */
		if (power_cycle)
			dcp->tunnel_power_cycles++;
		else
			drm_connector_set_link_status_property(&connector->base,
							       DRM_MODE_LINK_STATUS_BAD);

		/*
		 * A short Type-C route interruption can leave the DRM CRTC active
		 * while DCP has discarded its display mode.  Userspace is then free
		 * to keep submitting plane-only commits, none of which retrains the
		 * link.  Re-apply the active CRTC state once the DPTX link-config
		 * callback has completed.
		 */
		ret = dcp_retrain_active_crtc(connector, dcp, power_cycle);
		if (ret) {
			dev_warn(dcp->dev,
				 "failed to retrain active CRTC after hotplug: %d\n",
				 ret);
			dcp_link_recovery_schedule(dcp);
		}
	}

	dcp_test_point(dcp, DCP_TEST_HOTPLUG_EVENT);
	/* The announcement already decided what an H17P external reports. */
	if (gated)
		drm_kms_helper_connector_hotplug_event(&connector->base);
	else
		dcp_notify_hotplug(connector);
	/* after the event, so registering a backlight cannot delay it */
	apple_connector_backlight_sync(connector);
	/* A display came or went: the platform limit may now differ for another. */
	dcp_peer_limit_update_v27(connector->base.dev);
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

static void dcpep_handle_cb(struct apple_dcp *dcp, enum dcp_context_id context,
			    void *data, u32 length, u16 offset)
{
	struct device *dev = dcp->dev;
	struct dcp_packet_header *hdr = data;
	void *in, *out;
	int tag;
	struct dcp_channel *ch = dcp_get_channel(dcp, context);
	u8 depth;
	bool handled, zero_reply;

	if ((dcp->fw_compat == DCP_FIRMWARE_H17P ||
	     dcp->fw_compat == DCP_FIRMWARE_V_27_0 ||
	     dcp->fw_compat == DCP_FIRMWARE_V_26_6_DESKTOP) &&
	    (length < sizeof(*hdr) || !ch || ch->depth >= DCP_MAX_CALL_DEPTH ||
	     offset >= 0x8000 || length > 0x8000 - offset)) {
		dev_err(dev, "invalid IOMFB callback envelope\n");
		dcp->crashed = true;
		return;
	}

	/* The header is read below: check the record lies in the shared buffer first. */
	if (dcp->fw_compat == DCP_FIRMWARE_V_26_6_DESKTOP &&
	    (length < sizeof(*hdr) || offset >= 0x8000 || length > 0x8000 - offset)) {
		dev_err(dev, "invalid IOMFB callback envelope\n");
		dcp->crashed = true;
		return;
	}

	tag = dcp_parse_tag(hdr->tag);

	if (dcp->fw_compat == DCP_FIRMWARE_V_26_6_DESKTOP &&
	    (!iomfb_check_callback_v26_6_desktop(tag, hdr->in_len, hdr->out_len) ||
	     (u64)sizeof(*hdr) + hdr->in_len + hdr->out_len != length)) {
		dev_err(dev, "invalid desktop IOMFB callback %d (%u, %u)\n",
			tag, hdr->in_len, hdr->out_len);
		dcp->crashed = true;
		return;
	}

	handled = tag >= 0 && tag < IOMFB_MAX_CB && dcp->cb_handlers &&
		  dcp->cb_handlers[tag];
	if (dcp->fw_compat == DCP_FIRMWARE_V_26_6_DESKTOP && handled &&
	    ((u64)sizeof(*hdr) + hdr->in_len + hdr->out_len > length ||
	     !iomfb_check_callback_v26_6_desktop(tag, hdr->in_len, hdr->out_len))) {
		dev_err(dev, "short IOMFB callback %d (%u, %u, length %u)\n",
			tag, hdr->in_len, hdr->out_len, length);
		dcp->crashed = true;
		return;
	}
	if ((dcp->fw_compat == DCP_FIRMWARE_H17P || dcp->fw_compat == DCP_FIRMWARE_V_27_0) &&
	    (tag < 0 || tag >= IOMFB_MAX_CB ||
	     (u64)sizeof(*hdr) + hdr->in_len + hdr->out_len != length ||
	     (dcp->hw.iomfb_method_profile != DCP_IOMFB_METHODS_H17G &&
	      handled &&
	      !(dcp->fw_compat == DCP_FIRMWARE_V_27_0 ?
		iomfb_v27_callback_size_valid(tag, hdr->in_len, hdr->out_len) :
		iomfb_check_callback_h17p(dcp, tag, hdr->in_len, hdr->out_len))))) {
		dev_err(dev, "unqualified IOMFB callback %d (%u, %u)\n",
			tag, hdr->in_len, hdr->out_len);
		dcp->crashed = true;
		return;
	}

	in = data + sizeof(*hdr);
	out = in + hdr->in_len;

	dcp_trace_panel_setter(dcp, tag, in, hdr->in_len);

	if (!handled) {
		if (dcp_is_ext26(dcp)) {
			dev_err(dev, "native26 unsupported callback D%03d (%u/%u); session retained\n",
				tag, hdr->in_len, hdr->out_len);
			WRITE_ONCE(dcp->crashed, true);
			complete(&dcp->start_done);
			return;
		}
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
		 * Drain an unsupported desktop callback too. The outer modeset
		 * must see an error even if firmware accepts the zero reply;
		 * never pop its command before its real ACK arrives.
		 */
		if (dcp->fw_compat == DCP_FIRMWARE_V_26_6_DESKTOP &&
		    READ_ONCE(dcp->desktop->modeset_active))
			WRITE_ONCE(dcp->desktop->modeset_callback_error, true);
		if (dcp->fw_compat == DCP_FIRMWARE_H17P ||
		    dcp->fw_compat == DCP_FIRMWARE_V_27_0 ||
		    dcp->fw_compat == DCP_FIRMWARE_V_26_6_DESKTOP) {
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

static void dcpep_handle_ack(struct apple_dcp *dcp, enum dcp_context_id context)
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
	 * Native26 preserves the command header, including its tag. Validate it
	 * before using the reply. Older firmware keeps its length-only check;
	 * other H17 profiles retain their existing admission policy.
	 */
	if ((dcp->fw_compat != DCP_FIRMWARE_H17P &&
	     dcp->fw_compat != DCP_FIRMWARE_V_27_0) || dcp_is_ext26(dcp)) {
		record = out - sent->in_len - sizeof(*record);
		if (record->in_len != sent->in_len ||
		    record->out_len != sent->out_len ||
		    (dcp_is_ext26(dcp) &&
		     memcmp(record->tag, sent->tag, sizeof(record->tag)))) {
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
	if (dcp_is_ext26(dcp))
		dma_rmb();

	if (FIELD_GET(IOMFB_MSG_ACK, message))
		dcpep_handle_ack(dcp, ctx_id);
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

/*
 * Retained arrays are not freed while mode readers can still refer to them.
 * With @connector, only the connector that owns the admitted catalog sees it.
 */
static struct dcp_display_mode *dcp_modes_snapshot(struct apple_dcp *dcp,
						 struct apple_connector *connector,
						 unsigned int *count)
{
	struct dcp_display_mode *modes;
	unsigned long flags;

	/* An attachment sees no catalog until one is admitted for it. */
	guard(mutex)(&dcp->modes_lock);
	if (!dcp->modes_admitted ||
	    (connector && !dcp_modes_for_connector(dcp, connector))) {
		*count = 0;
		return NULL;
	}
	spin_lock_irqsave(&dcp->metadata.lock, flags);
	modes = dcp->modes;
	*count = dcp->nr_modes;
	spin_unlock_irqrestore(&dcp->metadata.lock, flags);
	return modes;
}
void dcp_modes_begin_attachment(struct apple_dcp *dcp)
{
	/* H17P external catalogs are fenced by their connection receipt. */
	if (dcp->hw.iomfb_method_profile == DCP_IOMFB_METHODS_H17P_EXTERNAL)
		return;
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
	/* H17P external catalogs are fenced by their connection receipt. */
	if (dcp->hw.iomfb_method_profile != DCP_IOMFB_METHODS_H17P_EXTERNAL) {
		dcp->modes_generation++;
		dcp->modes_admitted = false;
		dcp->modes_provisional = false;
	}
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
	unsigned long flags;

	if (!modes)
		return;
	guard(mutex)(&dcp->modes_lock);
	/* Never overwrite a fresh catalog or lend it to a different attachment. */
	if (generation != dcp->modes_generation ||
	    READ_ONCE(dcp->connector) != connector || dcp->modes_admitted) {
		kfree(modes);
		return;
	}
	/* The replaced array is retained, as for dcp_modes_replace(). */
	spin_lock_irqsave(&dcp->metadata.lock, flags);
	dcp->modes = modes;
	dcp->nr_modes = count;
	spin_unlock_irqrestore(&dcp->metadata.lock, flags);
	dcp->modes_admitted = true;
	dcp->modes_provisional = true;
}

u64 dcp_modes_transfer_begin(struct apple_dcp *dcp)
{
	guard(mutex)(&dcp->modes_lock);
	return dcp->modes_generation;
}

/*
 * Publish a TimingElements catalog. Replaced arrays stay allocated, as
 * readers may still hold a snapshot. A failed parse keeps an admitted
 * catalog, except on H17P external, which publishes an empty one. A
 * transfer started before the current attachment is never published.
 */
int dcp_modes_replace(struct apple_dcp *dcp, struct dcp_parse_ctx *ctx,
		      u64 generation)
{
	bool h17p_external = dcp->hw.iomfb_method_profile ==
		DCP_IOMFB_METHODS_H17P_EXTERNAL;
	struct apple_connector *ready = NULL;
	struct dcp_display_mode *modes;
	struct dcp_metadata_ticket rate_ticket = {};
	u64 rate_service_epoch = 0;
	unsigned int nr_modes = 0, i;
	unsigned long flags;
	bool provisional;
	u32 entries = 0;
	int ret = 0;

	/* A stale transfer is refused before it is parsed, as it is published. */
	scoped_guard(mutex, &dcp->modes_lock) {
		if (generation != dcp->modes_generation)
			return -ESTALE;
	}

	if (h17p_external && dcp->chunks.payload_complete &&
	    dcp->chunks.mode_ticket.associated) {
		ctx->automatic_1080p180_rgb8 = true;
		rate_ticket = dcp->chunks.mode_ticket;
		rate_service_epoch = dcp->chunks.av_service_epoch;
	}
	if (!h17p_external) {
		ret = parse_array_count(ctx, &entries);
		if (ret)
			return ret;
		ctx->metadata_timing = dcp_firmware_metadata_enabled(dcp);
		ctx->sdr_default_rgb = dcp->fw_compat == DCP_FIRMWARE_V_27_0 &&
				       !dcp_has_panel(dcp);
		ctx->pq_dsc = dcp->fw_compat == DCP_FIRMWARE_V_27_0;
	}
	ctx->zero_vrr_range_fallback = dcp->fw_compat == DCP_FIRMWARE_V_26_6_DESKTOP;
	ctx->internal_present_deadline = dcp->fw_compat == DCP_FIRMWARE_V_26_6_DESKTOP;
	if (dcp->fixed_connector_type != DRM_MODE_CONNECTOR_eDP &&
	    smp_load_acquire(&dcp->dptxport[0].tile_hint)) {
		ctx->tiled_width = dcp->dptxport[0].tile_width;
		ctx->tiled_height = dcp->dptxport[0].tile_height;
	}
	modes = enumerate_modes(ctx, &nr_modes, dcp->width_mm, dcp->height_mm,
				dcp->notch_height,
				dcp->fixed_connector_type == DRM_MODE_CONNECTOR_eDP,
				h17p_external);
	if (IS_ERR(modes)) {
		ret = PTR_ERR(modes);
		if (!h17p_external)
			return ret;
		modes = NULL;
		nr_modes = 0;
	} else if (!h17p_external && entries && !nr_modes) {
		/* No valid record: keep an admitted catalog, as for a parse error. */
		kfree(modes);
		return -EINVAL;
	} else {
		for (i = 0; i < nr_modes; i++)
			if (modes[i].automatic_rgb8_quirk) {
				modes[i].unsafe_rgb8_ticket = rate_ticket;
				modes[i].unsafe_rgb8_service_epoch = rate_service_epoch;
				modes[i].automatic_connection_sequence = dcp->chunks.automatic_connection_sequence;
				modes[i].automatic_service_pending = dcp->chunks.automatic_service_pending;
			}
	}

	mutex_lock(&dcp->modes_lock);
	if (generation != dcp->modes_generation) {
		mutex_unlock(&dcp->modes_lock);
		kfree(modes);
		return -ESTALE;
	}
	provisional = dcp->modes_provisional;
	/* Publish one catalog revision with its immutable receipt tickets. */
	spin_lock_irqsave(&dcp->metadata.lock, flags);
	dcp->rate_proof.catalog_sequence++;
	for (i = 0; i < nr_modes; i++)
		if (modes[i].automatic_rgb8_quirk)
			modes[i].automatic_catalog_sequence = dcp->rate_proof.catalog_sequence;
	dcp->modes = modes;
	dcp->nr_modes = nr_modes;
	/* Only a complete receipt for a current cable is that cable's catalogue. */
	dcp->metadata.catalog_ticket = rate_ticket;
	dcp->metadata.catalog_ticket.associated &= nr_modes > 0;
	dcp->metadata.catalog_connection_sequence =
		dcp->chunks.automatic_connection_sequence;
	spin_unlock_irqrestore(&dcp->metadata.lock, flags);
	dcp->modes_admitted = !ret;
	dcp->modes_provisional = false;
	/* A fresh catalog replaced one adopted across a route move. */
	if (!ret && provisional && dcp->connector && dcp->dev &&
	    dcp_modes_for_connector(dcp, dcp->connector) &&
	    READ_ONCE(dcp->connector->dcp) == to_platform_device(dcp->dev))
		ready = dcp->connector;
	mutex_unlock(&dcp->modes_lock);

	/*
	 * Described again after a withdrawal it was kept connected through,
	 * the same display keeps its EDID, unless that is a placeholder being
	 * retried.
	 */
	if (!ret && !h17p_external &&
	    (!atomic_read(&dcp->external_held) || READ_ONCE(dcp->placeholder_retried)))
		apple_connector_invalidate_edid(dcp->connector);
	/* Notify removals as well as newly eligible modes, outside the lock. */
	dcp_180hz_catalog_changed(dcp);
	/* RX lifetime is drained before connector cleanup; work rechecks ownership. */
	if (ready)
		dcp_queue_hotplug(ready);
	return ret;
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

/* Apply DisplayAttributes only for the attachment its transfer started on. */
int dcp_attributes_replace(struct apple_dcp *dcp, struct dcp_parse_ctx *handle,
			   u64 generation)
{
	int width_mm, height_mm, ret, i;
	bool backlight_control = false, ext;

	scoped_guard(mutex, &dcp->modes_lock) {
		if (generation != dcp->modes_generation)
			return -ESTALE;
		ret = parse_display_attributes(handle, &width_mm, &height_mm,
					       &backlight_control);
		if (!ret) {
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
		}
	}

	/* Malformed attributes keep the display's previous description. */
	if (ret)
		return ret;

	/*
	 * Written once, so the hotplug work never sees a working display
	 * without its backlight. The attributes can arrive after the
	 * hotplug work has run, so sync the backlight when the answer
	 * changes. 14.7 swaps carry no external backlight level.
	 */
	ext = backlight_control && !dcp_has_panel(dcp) && !dcp_fw_is_14_7(dcp);
	if (ext != READ_ONCE(dcp->ext_backlight)) {
		WRITE_ONCE(dcp->ext_backlight, ext);
		if (dcp->connector)
			schedule_work(&dcp->connector->bl_sync_wq);
	}

	/*
	 * This callback runs on the ordered RTKit RX worker. A mode probe
	 * can hold mode_config.mutex while waiting for an AFK reply which
	 * that same worker still has to dispatch. The hotplug work
	 * publishes the connector dimensions.
	 */
	if (dcp->connector && !READ_ONCE(dcp->quiescing))
		schedule_work(&dcp->connector->hotplug_wq);
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
/* Under mode_config.mutex, in get_modes: the identity of the EDID being published. */
static void dcp_note_edid(struct apple_connector *connector, const struct drm_edid *drm_edid)
{
	const struct edid *edid = drm_edid_raw(drm_edid);

	if (!edid)
		return;
	WRITE_ONCE(connector->edid_maker, get_unaligned_be16(edid->mfg_id));
	WRITE_ONCE(connector->edid_product, get_unaligned_le16(edid->prod_code));
	WRITE_ONCE(connector->edid_crc, crc32_le(~0, (const u8 *)edid, EDID_LENGTH));
}

/*
 * An external display after a mode set: the link the firmware trained (the rate and lane count
 * of its last link configuration), whether its mode set used DSC (its own line says so; "?"
 * before the first), the size of its mode list, the published EDID's maker, product and CRC,
 * and the mode. Printed once per connection and again when any of it changes, so that a
 * display that returns from a sleep with another link, list or mode shows in the log.
 * Only for a processor whose hardware description asks for it (T6050's external ones).
 */
void dcp_link_report(struct apple_dcp *dcp, const struct drm_display_mode *mode)
{
	static const char dsc[] = { '?', 'y', 'n' };
	struct apple_connector *connector = dcp->connector;
	u32 code = 0, lanes = 0, rate;
	char line[sizeof(dcp->link_line)];
	u64 generation;
	unsigned int i;

	if (!dcp->hw.link_report || !connector || dcp_has_panel(dcp))
		return;
	for (i = 0; i < ARRAY_SIZE(dcp->dptxport); i++) {
		if (!READ_ONCE(dcp->dptxport[i].link_rate))
			continue;
		code = READ_ONCE(dcp->dptxport[i].link_rate);
		lanes = READ_ONCE(dcp->dptxport[i].lane_count);
		break;
	}
	switch (code) {
	case LINK_RATE_UHBR10:
		rate = 10000;
		break;
	case LINK_RATE_UHBR20:
		rate = 20000;
		break;
	default:
		/* The DPCD code: units of 270 Mb/s per lane */
		rate = code * 270;
	}
	generation = READ_ONCE(connector->drm_edid_generation);
	scnprintf(line, sizeof(line),
		  "link %u Mb/s x %u, DSC %c, %u modes, EDID %04x:%04x crc %08x%s, mode %ux%u@%d",
		  rate, lanes, dsc[READ_ONCE(dcp->link_dsc) % ARRAY_SIZE(dsc)],
		  READ_ONCE(dcp->nr_modes), READ_ONCE(connector->edid_maker),
		  READ_ONCE(connector->edid_product), READ_ONCE(connector->edid_crc),
		  READ_ONCE(connector->drm_edid_firmware_derived) ? " (from the firmware's metadata)" : "",
		  mode->hdisplay, mode->vdisplay, drm_mode_vrefresh(mode));
	if (generation == dcp->link_line_generation && !strcmp(line, dcp->link_line))
		return;
	strscpy(dcp->link_line, line, sizeof(dcp->link_line));
	dcp->link_line_generation = generation;
	dev_info(dcp->dev, "ext %u: %s\n", dcp->index ? dcp->index - 1 : 0, line);
}

int dcp_get_modes(struct drm_connector *connector)
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
	if (iomfb_is_m4(dcp)) {
		struct drm_display_mode fixed;

		if (iomfb_m4_get_mode(dcp, &fixed))
			return 0;
		mode = drm_mode_duplicate(dev, &fixed);
		if (!mode)
			return 0;
		drm_mode_probed_add(connector, mode);
		of_property_read_u32(dcp->dev->of_node, "apple,panel-width-mm",
				     &connector->display_info.width_mm);
		of_property_read_u32(dcp->dev->of_node, "apple,panel-height-mm",
				     &connector->display_info.height_mm);
		return 1;
	}
	modes = dcp_modes_snapshot(dcp, apple_connector, &nr_modes);

	physical_edid = dcp_is_ext26(dcp) || dcp->hw.iomfb_method_profile ==
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
	    (!dcp_is_ext26(dcp) || !dcp->ext26_edid_tried ||
	     dcp->ext26_edid_generation != edid_generation) &&
	    (dcp->fw_compat != DCP_FIRMWARE_H17P ||
	     (dcp->dcpavservep && dcp->dcpavservep->wire_v2))) {
		const struct drm_edid *edid;
		/* Let physical disconnect advance the generation during the request. */
		mutex_unlock(&dcp->dcpavserv.lock);
		if (physical_edid)
			mutex_unlock(&dcp->hpd_mutex);
		if (dcp_is_ext26(dcp)) {
			dcp->ext26_edid_tried = true;
			dcp->ext26_edid_generation = edid_generation;
		}
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
			if (dcp_is_ext26(dcp) && IS_ERR(edid) &&
			    PTR_ERR(edid) != -ENODEV && PTR_ERR(edid) != -ESTALE) {
				WRITE_ONCE(dcp->ext26_blocked, true);
				WRITE_ONCE(dcp->crashed, true);
				apple_connector->connected = false;
				nr_modes = 0;
				dev_err(dcp->dev, "native26 initial EDID discovery failed: %pe; session retained\n", edid);
				goto edid_done;
			}
			/*
			 * There is no AV service for an internal panel, nor
			 * for an external display while its link is down;
			 * do not report that on every mode probe.
			 */
			if (PTR_ERR(edid) == -ENODEV)
				dev_dbg(dcp->dev, "no EDID source\n");
			/*
			 * After resume the firmware publishes the AV service
			 * before its link to the sink is back, and answers a
			 * read then without data; probe again once it has read
			 * the sink.
			 */
			else if (PTR_ERR(edid) == -ENODATA &&
				 dcpavserv_edid_retry(dcp, service_epoch))
				dev_dbg(dcp->dev, "sink EDID not read yet, probing again\n");
			else
				dev_warn(dcp->dev, "copy_edid failed: %pe\n", edid);
		} else {
			dcpavserv_edid_retry_reset(dcp);
			drm_edid_free(apple_connector->drm_edid);
			apple_connector->drm_edid = edid;
			apple_connector->drm_edid_generation = edid_generation;
			apple_connector->drm_edid_service_epoch = service_epoch;
			/* A native T6030 pipe: its audio, after the first EDID. */
			dcp_external_audio_start(dcp);
		}
	}
	if (nr_modes && apple_connector->drm_edid) {
		const struct drm_edid *published = apple_connector->drm_edid;
		const struct drm_edid *sdr = NULL;
		int update;

		if (dcp_is_ext26(dcp)) {
			const struct edid *raw = drm_edid_raw(published);
			size_t size = (raw->extensions + 1) * EDID_LENGTH;
			u8 *copy = kmemdup(raw, size, GFP_KERNEL);

			if (!copy) {
				drm_edid_connector_update(connector, NULL);
				goto edid_done;
			}
			size = dcp_edid_sdr(copy, size);
			if (size)
				sdr = drm_edid_alloc(copy, size);
			kfree(copy);
			if (!sdr) {
				drm_edid_connector_update(connector, NULL);
				goto edid_done;
			}
			published = sdr;
		}
		update = drm_edid_connector_update(connector, published);
		drm_edid_free(sdr);

		if (update) {
			dev_warn(dcp->dev, "raw EDID publication failed: %d\n", update);
			dcp_clear_connector_edid(apple_connector);
		} else {
			apple_connector->drm_edid_firmware_derived = false;
			if (dcp->hw.link_report)
				dcp_note_edid(apple_connector, apple_connector->drm_edid);
		}
	} else if (nr_modes && dcp_is_v14_7_panel(dcp) && dcp->panel.id.vendor[0]) {
		/*
		 * No EDID comes for this panel: publish the base block of its
		 * node, sized as the connector is (the firmware's size, else the
		 * node's), then give the connector back its size in millimetres.
		 */
		int update;

		dcp_update_connector_dimensions(dcp, connector);
		update = dcp_panel_edid_update(connector, &dcp->panel.id,
					       connector->display_info.width_mm,
					       connector->display_info.height_mm);
		dcp_update_connector_dimensions(dcp, connector);
		apple_connector->drm_edid_firmware_derived = false;
		if (update)
			dev_warn_once(dcp->dev, "panel EDID publication failed: %d\n", update);
	} else {
		u64 generation = 0;
		u32 defaults = 0;
		bool panel = dcp->panel_identity && dcp_has_panel(dcp);
		const struct drm_edid *derived = nr_modes ?
			(panel ? dcp_panel_edid_snapshot(dcp, modes, nr_modes, &generation) :
			 dcp_firmware_edid(dcp, &generation, &defaults)) : ERR_PTR(-ENODATA);

		if (!IS_ERR(derived)) {
			bool log = !apple_connector->drm_edid_firmware_derived ||
				apple_connector->drm_edid_generation != generation;
			int update;

			apple_connector->drm_edid_firmware_derived = true;
			apple_connector->drm_edid_generation = generation;
			update = drm_edid_connector_update(connector, derived);
			if (!update && dcp->hw.link_report)
				dcp_note_edid(apple_connector, derived);
			drm_edid_free(derived);
			if (update || !(panel ? dcp_panel_metadata_current(dcp, generation) :
				       dcp_firmware_metadata_current(dcp, generation))) {
				drm_edid_connector_update(connector, NULL);
				apple_connector->drm_edid_firmware_derived = false;
			} else if (log) {
				dev_info(dcp->dev, "using firmware-derived EDID metadata for connection %llu%s%s%s%s%s%s%s\n",
					 generation, defaults ? "; not reported:" : "",
					 defaults & DCP_METADATA_DEFAULT_SIZE ? " size" : "",
					 defaults & DCP_METADATA_DEFAULT_COLOR ? " colour" : "",
					 defaults & DCP_METADATA_DEFAULT_NAME ? " name" : "",
					 defaults & DCP_METADATA_DEFAULT_SERIAL ? " serial" : "",
					 defaults & DCP_METADATA_DEFAULT_DATE ? " date" : "",
					 defaults & DCP_METADATA_DEFAULT_TIMING ? " timing" : "");
			}
		} else {
			if (panel && nr_modes)
				dev_dbg(dcp->dev, "panel EDID generation failed: %pe\n", derived);
			if (apple_connector->drm_edid_firmware_derived) {
				drm_edid_connector_update(connector, NULL);
				apple_connector->drm_edid_firmware_derived = false;
			}
		}
	}

edid_done:
	/* Discovery can replace the catalogue while the OOB read waits. */
	if (dcp_is_ext26(dcp)) {
		if (READ_ONCE(dcp->crashed) || READ_ONCE(dcp->ext26_blocked))
			nr_modes = 0;
		else
			modes = dcp_modes_snapshot(dcp, apple_connector, &nr_modes);
	}
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
			u16 lo = DIV_ROUND_UP(modes[i].min_vrr, 1U << 16);
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
	mutex_unlock(&dcp->dcpavserv.lock);
	if (physical_edid)
		mutex_unlock(&dcp->hpd_mutex);
	if (pending_180hz && !published_180hz)
		dcp_180hz_retry(dcp);
	if (apple_connector->drm_edid)
		dcp_retry_placeholder_edid(dcp, apple_connector->drm_edid);

	/* EDID updates clear display_info even for panels without an EDID. */
	if (published && dcp_has_panel(dcp))
		dcp_update_connector_dimensions(dcp, connector);

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

	/* EDID base dimensions are rounded to cm; the panel's OF/ADT size is exact. */
	if (dcp->fw_compat == DCP_FIRMWARE_V_26_6_DESKTOP && dcp_has_panel(dcp))
		dcp_update_connector_dimensions(dcp, connector);

	return published;
}

/* The pixel clock of the J616s boot loader's panel mode. */
#define DCP_DESKTOP_BOOT_PIXEL_CLOCK_KHZ	521065
#define DCP_DESKTOP_TRIAL_MAX_CAND		2

/* Reuses the mode the takeover trial set, once; see dcp_takeover_trial(). */
static int dcp_desktop_modeset(struct apple_dcp *dcp,
			       struct drm_crtc_state *crtc_state)
{
	struct dcp_desktop_state *d = dcp->desktop;
	struct dcp_display_mode matched;

	if (d->trial_modeset_ok) {
		d->trial_modeset_ok = false;
		/* The trial installed a VRR timing variable too (see the mode set). */
		if (d->mode_live &&
		    lookup_mode(dcp, &crtc_state->mode, &matched) &&
		    matched.timing_mode_id == d->trial_timing_mode_id &&
		    matched.color_mode_id == d->trial_color_mode_id) {
			dev_dbg(dcp->dev, "keeping the takeover mode " DRM_MODE_FMT "\n",
				DRM_MODE_ARG(&crtc_state->mode));
			return 0;
		}
	}
	return iomfb_modeset_v26_6_desktop(dcp, crtc_state);
}

static bool takeover_adopt = true;
module_param(takeover_adopt, bool, 0444);
MODULE_PARM_DESC(takeover_adopt,
		 "Desktop 26.x panel takeover: present on the loader's live mode instead of setting it again (default: on)");

/*
 * The loader leaves the panel running, and the firmware keeps that pipe when
 * it restarts.  Setting the same mode again blanks the panel for ~200 ms, so
 * adopt it: record the candidate's timing and colour element as the live
 * mode without set_digital_out_mode.  The swap watchdog follows the swaps
 * until one completes; if none does, it sets the mode after all
 * (dcp_swap_watchdog()).
 */
static int dcp_takeover_adopt(struct apple_dcp *dcp, const struct drm_display_mode *m)
{
	struct dcp_desktop_state *d = dcp->desktop;
	struct dcp_display_mode sel;

	if (!lookup_mode(dcp, m, &sel))
		return -ENOENT;
	dcp->mode = (struct dcp_set_digital_out_mode_req) {
		.color_mode_id = sel.color_mode_id,
		.timing_mode_id = sel.timing_mode_id,
	};
	dcp->use_timestamps = sel.needs_timestamps;
	dcp->vrr_enabled = false;
	d->mode_live = true;
	d->takeover_adopted = true;
	dev_info(dcp->dev, "takeover: adopting the live mode (color:%u timing:%u) " DRM_MODE_FMT "\n",
		 sel.color_mode_id, sel.timing_mode_id, DRM_MODE_ARG(m));
	return 0;
}

/*
 * Takeover gate of a desktop 26.x panel, called after the DCP started and
 * before the boot framebuffer is evicted.  Everything that can still be
 * refused is tried while the boot framebuffer owns the screen: DCP ready,
 * modes known, panel connected, power on, one mode set.  Returns 0 when it
 * is safe to evict, otherwise a negative errno and a short reason.  A
 * failure leaves the DCP running: its shutdown sequence can crash it.
 */
int dcp_takeover_trial(struct platform_device *pdev, const char **reason)
{
	struct apple_dcp *dcp = platform_get_drvdata(pdev);
	struct drm_display_mode cand[DCP_DESKTOP_TRIAL_MAX_CAND];
	unsigned int i, k, n = 0, pref = UINT_MAX, boot = UINT_MAX;
	u32 best = U32_MAX;
	struct dcp_desktop_state *d;
	struct drm_crtc_state *st;
	int win = -1, ret = -EIO;

	*reason = "none";
	if (!dcp || dcp->fw_compat != DCP_FIRMWARE_V_26_6_DESKTOP ||
	    !dcp_has_panel(dcp))
		return 0;
	d = dcp->desktop;
	if (dcp->crashed) {
		*reason = "dcp-crashed";
		return -ENODEV;
	}
	if (!dcp->active) {
		*reason = "dcp-not-ready";
		return -ETIMEDOUT;
	}
	if (!READ_ONCE(dcp->nr_modes)) {
		*reason = "no-modes";
		return -ENODEV;
	}
	if (!dcp->connector || !dcp->connector->connected) {
		*reason = "not-connected";
		return -ENODEV;
	}
	/*
	 * The preferred mode first, the one the firmware ranks highest (the
	 * panel's native refresh), then the boot-like one: the mode that works
	 * stays the preferred one, which is also what a desktop picks.
	 */
	mutex_lock(&dcp->modes_lock);
	for (i = 0; i < dcp->nr_modes; i++) {
		const struct dcp_display_mode *m = &dcp->modes[i];

		if ((m->mode.type & DRM_MODE_TYPE_PREFERRED) && pref == UINT_MAX)
			pref = i;
		if (!m->vrr) {
			u32 delta = abs((int)m->mode.clock -
					DCP_DESKTOP_BOOT_PIXEL_CLOCK_KHZ);

			if (delta < best) {
				best = delta;
				boot = i;
			}
		}
	}
	if (pref == UINT_MAX)
		pref = 0;
	cand[n++] = dcp->modes[pref].mode;
	/* Only a mode within 1% of the boot loader's clock counts as boot-like. */
	if (boot != UINT_MAX && best <= DCP_DESKTOP_BOOT_PIXEL_CLOCK_KHZ / 100 &&
	    boot != pref)
		cand[n++] = dcp->modes[boot].mode;
	mutex_unlock(&dcp->modes_lock);

	st = kzalloc_obj(*st);
	if (!st) {
		*reason = "no-memory";
		return -ENOMEM;
	}

	dcp_poweron(pdev);
	if (dcp->crashed || d->poweron_result) {
		*reason = dcp->crashed ? "dcp-crashed-in-poweron" : "poweron-failed";
		goto out;
	}

	for (k = 0; takeover_adopt && k < n; k++) {
		if (!dcp_takeover_adopt(dcp, &cand[k])) {
			win = k;
			break;
		}
	}
	for (k = 0; win < 0 && k < n; k++) {
		int r;

		st->mode = cand[k];
		dev_dbg(dcp->dev, "takeover trial mode set " DRM_MODE_FMT "\n",
			DRM_MODE_ARG(&st->mode));
		r = iomfb_modeset_v26_6_desktop(dcp, st);
		if (!r && d->mode_live) {
			win = k;
			break;
		}
		if (dcp->crashed)
			break;
	}
	if (win < 0) {
		*reason = dcp->crashed ? "dcp-crashed-in-modeset" : "modeset-failed";
		goto out;
	}

	d->trial_timing_mode_id = dcp->mode.timing_mode_id;
	d->trial_color_mode_id = dcp->mode.color_mode_id;
	d->trial_powered = true;
	d->trial_modeset_ok = true;

	/* Make the mode that worked the one the console client picks. */
	mutex_lock(&dcp->modes_lock);
	for (i = 0; i < dcp->nr_modes; i++) {
		struct dcp_display_mode *m = &dcp->modes[i];

		if (m->timing_mode_id == d->trial_timing_mode_id &&
		    m->color_mode_id == d->trial_color_mode_id)
			m->mode.type |= DRM_MODE_TYPE_PREFERRED;
		else
			m->mode.type &= ~DRM_MODE_TYPE_PREFERRED;
	}
	mutex_unlock(&dcp->modes_lock);
	dev_info(dcp->dev, "takeover trial mode " DRM_MODE_FMT " accepted\n",
		 DRM_MODE_ARG(&cand[win]));
	ret = 0;
out:
	kfree(st);
	return ret;
}

/*
 * PQ output goes through the IOMFB template's mode set, which picks the
 * timing's PQ element (see dcp_display_mode.pq): the desktop 26.x firmware,
 * the external H17P processor and the firmware 27 external processors.  The
 * other processors, and the firmware 27 panel, keep SDR.
 */
static bool dcp_hdr_output_supported(struct apple_dcp *dcp)
{
	if (iomfb_is_m4(dcp))
		return false;
	return dcp->fw_compat == DCP_FIRMWARE_V_26_6_DESKTOP ||
	       dcp->hw.iomfb_method_profile == DCP_IOMFB_METHODS_H17P_EXTERNAL ||
	       (dcp->fw_compat == DCP_FIRMWARE_V_27_0 && !dcp_has_panel(dcp));
}

/*
 * A desktop 26.x or firmware 27 panel keeps its SDR link and shows PQ
 * content as EDR, given the panel's peak (see res_max_panel_brightness()).
 */
static bool dcp_panel_edr_supported(struct apple_dcp *dcp)
{
	return (dcp->fw_compat == DCP_FIRMWARE_V_26_6_DESKTOP ||
		dcp->fw_compat == DCP_FIRMWARE_V_27_0) &&
	       dcp_has_panel(dcp) && READ_ONCE(dcp->panel.peak_nits);
}

/* The state of the connector on @crtc that @commit sets up. */
static const struct drm_connector_state *
dcp_crtc_connector_state(struct apple_dcp *dcp, struct drm_crtc *crtc,
			 struct drm_atomic_commit *commit)
{
	const struct drm_connector_state *conn_state = NULL;
	struct drm_connector_state *state;
	struct drm_connector *connector;
	int i;

	for_each_new_connector_in_state(commit, connector, state, i)
		if (state->crtc == crtc)
			conn_state = state;
	/* A mode set the driver starts itself keeps the connector's state. */
	if (!conn_state && dcp->connector && dcp->connector->base.state &&
	    dcp->connector->base.state->crtc == crtc)
		conn_state = dcp->connector->base.state;
	return conn_state;
}

/* The EOTF the compositor asks of the display on @crtc, an HDMI_EOTF_* value. */
u8 dcp_crtc_eotf(struct apple_dcp *dcp, struct drm_crtc *crtc,
		 struct drm_atomic_commit *commit)
{
	const struct drm_connector_state *conn_state =
		dcp_crtc_connector_state(dcp, crtc, commit);
	const struct hdr_output_metadata *hdr;

	if (!conn_state || !conn_state->hdr_output_metadata)
		return HDMI_EOTF_TRADITIONAL_GAMMA_SDR;
	hdr = conn_state->hdr_output_metadata->data;
	return hdr->hdmi_metadata_type1.eotf;
}

/*
 * A panel shows PQ content as EDR only while BT.2020 RGB is asked for too.
 * Hyprland 0.56, going from HDR to a wide-gamut SDR mode, sets Colorspace back
 * to Default but leaves the PQ metadata; on that alone the panel showed SDR
 * frames as PQ at its peak.
 */
static bool dcp_crtc_bt2020(struct apple_dcp *dcp, struct drm_crtc *crtc,
			    struct drm_atomic_commit *commit)
{
	const struct drm_connector_state *conn_state =
		dcp_crtc_connector_state(dcp, crtc, commit);

	return conn_state && conn_state->colorspace == DRM_MODE_COLORIMETRY_BT2020_RGB;
}

/*
 * HDR10 static metadata is link state on the external processors: a link
 * set up again (mode set, power off) starts without it.
 */
void dcp_hdr_metadata_forget(struct apple_dcp *dcp)
{
	struct dcp_desktop_state *d = dcp->desktop;

	if (!d)
		return;
	scoped_guard(spinlock_irqsave, &d->hdr_lock) {
		d->hdr_sent_valid = false;
		d->hdr_infoframe = false;
	}
}

/*
 * Clear the Dynamic Range and Mastering InfoFrame of a live link before it
 * leaves PQ, as macOS does: DCPAV selector 0x14 first, then the SDR colour
 * mode.  Setting the SDR colour mode alone would leave the sink an HDR
 * InfoFrame for an SDR signal.
 */
static void dcp_hdr_infoframe_stop(struct apple_dcp *dcp)
{
	struct dcp_desktop_state *d = dcp->desktop;
	struct apple_epic_service *service = NULL;
	u8 msg[DCP_HDR_STOP_SIZE], reply[DCP_HDR_STOP_SIZE] = {};
	size_t received = 0;
	u32 retcode = 0;
	int ret;

	if (!d || READ_ONCE(dcp->crashed))
		return;
	scoped_guard(spinlock_irqsave, &d->hdr_lock)
		if (d->hdr_infoframe)
			service = afk_service_get(d->av_service);
	if (!service)
		return;

	dcp_hdr_stop_message(msg);
	ret = afk_send_command_with_reply_len(service, EPIC_SUBTYPE_STD_SERVICE,
					      msg, sizeof(msg), reply, sizeof(reply),
					      &retcode, &received);
	ret = dcp_hdr_stop_reply(ret, retcode, reply, received);
	if (ret)
		dev_warn(dcp->dev, "HDR InfoFrame not stopped: %d\n", ret);
	else
		dev_dbg(dcp->dev, "HDR InfoFrame stopped\n");
	scoped_guard(spinlock_irqsave, &d->hdr_lock)
		if (d->av_service == service)
			d->hdr_infoframe = false;
	afk_service_put(service);
}

/* Whether @pdev takes a change of HDR_OUTPUT_METADATA without a mode set. */
bool dcp_hdr_metadata_live(struct platform_device *pdev)
{
	struct apple_dcp *dcp = platform_get_drvdata(pdev);

	return dcp->fw_compat == DCP_FIRMWARE_V_26_6_DESKTOP &&
	       !dcp_has_panel(dcp);
}

/*
 * Whether @pdev switches between SDR and PQ shown as EDR without a mode set:
 * a desktop 26.x panel keeps its SDR link either way, and PQ only changes how
 * the swaps tag their surfaces and the EDR values they carry
 * (dcp_edr_prepare()).  macOS turns EDR on and off on a live panel too.
 */
bool dcp_edr_live(struct platform_device *pdev)
{
	return dcp_panel_edr_supported(platform_get_drvdata(pdev));
}

/* Follow a colour state change that dcp_edr_live() let through without a mode set. */
static void dcp_edr_sync(struct apple_dcp *dcp, struct drm_crtc *crtc,
			 struct drm_atomic_commit *commit)
{
	bool edr;

	if (!dcp_panel_edr_supported(dcp))
		return;
	edr = dcp_crtc_eotf(dcp, crtc, commit) == HDMI_EOTF_SMPTE_ST2084 &&
	      dcp_crtc_bt2020(dcp, crtc, commit);
	if (edr == dcp->output_edr)
		return;
	dev_dbg(dcp->dev, "EDR %s without a mode set\n", edr ? "on" : "off");
	dcp->output_edr = edr;
}

/*
 * Send the connector's HDR10 static metadata to the DCPAV service once the
 * link carries PQ: after the mode set that selected a PQ colour mode, and
 * whenever the metadata changes on it.  Never per frame, and never on an
 * SDR link (the firmware would build a DRM InfoFrame with the SDR EOTF).
 * Going back to SDR is a mode set that first stops the InfoFrame
 * (dcp_hdr_infoframe_stop()).
 */
static void dcp_hdr_metadata_sync(struct apple_dcp *dcp, struct drm_crtc *crtc,
				  struct drm_atomic_commit *commit)
{
	struct dcp_desktop_state *d = dcp->desktop;
	const struct drm_connector_state *conn_state = NULL;
	struct drm_connector_state *state;
	struct drm_connector *connector;
	struct apple_epic_service *service;
	u8 data[DCP_HDR_DATA_SIZE], msg[DCP_HDR_MSG_SIZE];
	u8 reply[DCP_HDR_MSG_SIZE] = {};
	size_t received = 0;
	u32 retcode = 0;
	bool same;
	int i, ret;

	if (!d || !dcp->output_pq || READ_ONCE(dcp->crashed) ||
	    !dcp_hdr_metadata_live(to_apple_crtc(crtc)->dcp))
		return;
	for_each_new_connector_in_state(commit, connector, state, i)
		if (state->crtc == crtc)
			conn_state = state;
	if (!conn_state && dcp->connector && dcp->connector->base.state &&
	    dcp->connector->base.state->crtc == crtc)
		conn_state = dcp->connector->base.state;
	if (!conn_state || !conn_state->hdr_output_metadata ||
	    conn_state->hdr_output_metadata->length != sizeof(struct hdr_output_metadata))
		return;
	if (((const struct hdr_output_metadata *)conn_state->hdr_output_metadata->data)
		    ->hdmi_metadata_type1.eotf != HDMI_EOTF_SMPTE_ST2084 ||
	    dcp_hdr_data(data, conn_state->hdr_output_metadata->data))
		return;

	/* Most frames change nothing: compare before pinning the service. */
	scoped_guard(spinlock_irqsave, &d->hdr_lock) {
		same = d->hdr_sent_valid && !memcmp(d->hdr_sent, data, sizeof(data));
		service = same ? NULL : afk_service_get(d->av_service);
	}
	if (!service)
		return;

	dcp_hdr_message(msg, data);
	ret = afk_send_command_with_reply_len(service, EPIC_SUBTYPE_STD_SERVICE,
					      msg, sizeof(msg), reply, sizeof(reply),
					      &retcode, &received);
	ret = dcp_hdr_reply(ret, retcode, reply, received);
	/*
	 * A block the firmware refused is not sent again on every frame (the
	 * command is synchronous): only a new block, a new mode set or a new
	 * service instance retries.
	 */
	if (ret)
		dev_warn_ratelimited(dcp->dev, "HDR static metadata not taken: %d\n", ret);
	else
		dev_dbg(dcp->dev, "HDR static metadata: MaxCLL %u MaxFALL %u mastering %u/%u\n",
			get_unaligned_le16(data + 20), get_unaligned_le16(data + 22),
			get_unaligned_le16(data + 16), get_unaligned_le16(data + 18));
	scoped_guard(spinlock_irqsave, &d->hdr_lock) {
		if (d->av_service == service) {
			memcpy(d->hdr_sent, data, sizeof(data));
			d->hdr_sent_valid = true;
			if (!ret)
				d->hdr_infoframe = true;
		}
	}
	afk_service_put(service);
}

/* Only an SDR or a PQ signal, and PQ only where the timing has an element. */
int dcp_crtc_check_hdr(struct apple_dcp *dcp, struct drm_crtc *crtc,
		       struct drm_atomic_commit *commit)
{
	struct drm_crtc_state *crtc_state = drm_atomic_get_new_crtc_state(commit, crtc);
	struct dcp_display_mode mode;

	if (!crtc_state || !crtc_state->enable ||
	    !drm_atomic_crtc_needs_modeset(crtc_state))
		return 0;
	switch (dcp_crtc_eotf(dcp, crtc, commit)) {
	case HDMI_EOTF_TRADITIONAL_GAMMA_SDR:
		return 0;
	case HDMI_EOTF_SMPTE_ST2084:
		if (dcp_panel_edr_supported(dcp))
			return 0;
		if (dcp_hdr_output_supported(dcp) &&
		    lookup_mode(dcp, &crtc_state->mode, &mode) && mode.pq.score >= 0)
			return 0;
		fallthrough;
	default:
		return -EINVAL;
	}
}

/* The user may own drm_display_mode, so we need to search for our copy */
bool lookup_mode(struct apple_dcp *dcp, const struct drm_display_mode *mode,
		 struct dcp_display_mode *out)
{
	struct dcp_display_mode *modes;
	unsigned int count, i;
	unsigned long flags;
	bool eligible;

	modes = dcp_modes_snapshot(dcp, NULL, &count);
	for (i = 0; i < count; ++i) {
		if (drm_mode_match(mode, &modes[i].mode,
				   DRM_MODE_MATCH_TIMINGS |
					   DRM_MODE_MATCH_CLOCK)) {
			if (!modes[i].automatic_rgb8_quirk)
				goto found;
			mutex_lock(&dcp->hpd_mutex);
			mutex_lock(&dcp->dcpavserv.lock);
			spin_lock_irqsave(&dcp->metadata.lock, flags);
			eligible = dcp_exception_mode_current_locked(dcp, &modes[i]);
			spin_unlock_irqrestore(&dcp->metadata.lock, flags);
			mutex_unlock(&dcp->dcpavserv.lock);
			mutex_unlock(&dcp->hpd_mutex);
			if (eligible)
				goto found;
		}
	}

	return false;

found:
	/* Callers keep a copy, never a pointer into the catalog. */
	if (out)
		*out = modes[i];
	return true;
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
	struct dcp_display_mode dcp_mode;

	if (!lookup_mode(dcp, mode, &dcp_mode))
		return false;

	if (dcp->fw_compat == DCP_FIRMWARE_H17P && !dcp_is_ext26(dcp) &&
	    dcp->hw.iomfb_method_profile !=
	    DCP_IOMFB_METHODS_H17P_EXTERNAL)
		return dcp_mode.mode.type & DRM_MODE_TYPE_PREFERRED;

	/*
	 * The T6050 internal DCP (M5 Max) crashes on 3024x1986@60 and the
	 * other 60 Hz timings it lists.  Only the 120 Hz timing is known to
	 * survive a modeset, so refuse the rest until they are understood.
	 */
	if (dcp->hw.only_120hz_modes && drm_mode_vrefresh(mode) != 120)
		return false;

	return true;
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
	if (iomfb_is_m4(dcp)) {
		struct drm_display_mode fixed;

		return !iomfb_m4_get_mode(dcp, &fixed) && drm_mode_equal(mode, &fixed) ?
			MODE_OK : MODE_BAD;
	}

	scoped_guard(mutex, &dcp->modes_lock)
		if (!dcp_modes_for_connector(dcp, apple_connector))
			return MODE_BAD;
	/*
	 * A mode only a pair drives is not offered while the platform limit
	 * leaves this display no pair; dcp_peer_limit_update_v27() has the
	 * connector probed again when that changes.
	 */
	if (dcp_peer_mode_limited_v27(dcp, mode->clock))
		return MODE_CLOCK_HIGH;
	return dcp_mode_settable(dcp, mode) ? MODE_OK : MODE_BAD;
}

int dcp_crtc_atomic_modeset(struct drm_crtc *crtc,
			    struct drm_atomic_commit *commit)
{
	struct apple_crtc *apple_crtc = to_apple_crtc(crtc);
	struct apple_dcp *dcp = platform_get_drvdata(apple_crtc->dcp);
	struct drm_crtc_state *crtc_state;
	unsigned int action;
	int ret = -EIO;
	bool modeset;

	if (iomfb_is_m4(dcp)) {
		struct drm_display_mode fixed;

		crtc_state = drm_atomic_get_new_crtc_state(commit, crtc);
		if (!crtc_state || !crtc_state->active)
			return 0;
		ret = iomfb_m4_get_mode(dcp, &fixed);
		return ret ? ret : drm_mode_equal(&crtc_state->mode, &fixed) ? 0 : -EINVAL;
	}

	/* Disconnect cannot pass the admission check before these RPCs finish. */
	guard(mutex)(&dcp->external_lock);
	if (dcp_is_retained_external(dcp) &&
	    (!dcp->external_running || dcp->external_error || READ_ONCE(dcp->quiescing) ||
	     READ_ONCE(dcp->crashed) || READ_ONCE(dcp->iomfb.presentation_paused))) {
		/*
		 * The pipe of this CRTC did not start, so it has no link and its
		 * old mode is gone with it: the hotplug worker sets it up again
		 * or reports the display lost.  A pipe that runs with its
		 * presentation paused on purpose keeps its mode.
		 */
		if (!dcp->external_running || dcp->external_error) {
			dcp_mode_invalidate(&dcp->mode_state);
			dcp_handle_hotplug_actions(dcp, DCP_HOTPLUG_NOTIFY);
		}
		return -EPIPE;
	}

	crtc_state = drm_atomic_get_new_crtc_state(commit, crtc);
	if (!crtc_state)
		return 0;

	modeset = drm_atomic_crtc_needs_modeset(crtc_state) ||
		  !READ_ONCE(dcp->mode_state.valid);

	if (!modeset)
		return 0;

	/* ignore no mode, poweroff is handled elsewhere */
	if (crtc_state->mode.hdisplay == 0 && crtc_state->mode.vdisplay == 0)
		return 0;

	dcp->pq_requested = dcp_crtc_eotf(dcp, crtc, commit) == HDMI_EOTF_SMPTE_ST2084;
	dcp->output_edr = dcp->pq_requested && dcp_panel_edr_supported(dcp) &&
			  dcp_crtc_bt2020(dcp, crtc, commit);
	dcp->pq_requested &= dcp_hdr_output_supported(dcp);
	if (!dcp->pq_requested)
		dcp_hdr_infoframe_stop(dcp);
	dcp_hdr_metadata_forget(dcp);
	dcp_mode_begin(&dcp->mode_state);
	switch (dcp->fw_compat) {
	case DCP_FIRMWARE_V_12_3:
		ret = iomfb_modeset_v12_3(dcp, crtc_state);
		break;
	case DCP_FIRMWARE_V_13_5:
		ret = iomfb_modeset_v13_3(dcp, crtc_state);
		break;
	case DCP_FIRMWARE_V_14_7_TEMPLATE:
		ret = iomfb_modeset_v14_7_0(dcp, crtc_state);
		break;
	case DCP_FIRMWARE_V_26_6_DESKTOP:
		ret = dcp_desktop_modeset(dcp, crtc_state);
		break;
	case DCP_FIRMWARE_V_26_6:
		ret = iomfb_v26_6_modeset(dcp, crtc_state);
		break;
	case DCP_FIRMWARE_V_14_7:
		ret = iomfb_v14_7_modeset(dcp, crtc_state);
		break;
	case DCP_FIRMWARE_H17P:
		ret = iomfb_modeset_h17p(dcp, crtc_state,
			drm_atomic_get_new_plane_state(commit, crtc->primary));
		break;
	case DCP_FIRMWARE_V_27_0:
		ret = iomfb_modeset_v27(dcp, crtc_state);
		break;
	default:
		WARN_ONCE(true, "Unexpected firmware version: %u\n",
			  dcp->fw_compat);
		break;
	}

	action = dcp_mode_finish(&dcp->mode_state, !ret, dcp->connector ?
				&dcp->connector->connected : NULL);
	/* A successful mode set makes the CRTC owner of this Type-C session. */
	if (!ret && dcp_is_typec_output(dcp)) {
		scoped_guard(mutex, &dcp->hpd_mutex)
			dcp->crtc_unplugs = dcp->typec_unplugs;
	}
	/* An external mode set that failed leaves the CRTC without a link too. */
	if (ret && dcp->hw.iomfb_method_profile == DCP_IOMFB_METHODS_H17P_EXTERNAL)
		action |= DCP_HOTPLUG_NOTIFY;
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

bool dcp_crtc_route_ready(struct drm_crtc *crtc, struct drm_atomic_commit *state,
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

	/* Can't be lit: a resume then restores its CRTC disabled, not failing every output. */
	if (READ_ONCE(dcp->crashed))
		return false;
	/* Likewise the CRTC of a display released for a firmware sleep, until it is back. */
	if (READ_ONCE(dcp->sleep_v27_torn))
		return false;

	if (iomfb_is_m4(dcp)) {
		struct drm_display_mode fixed;

		return !iomfb_m4_get_mode(dcp, &fixed) && drm_mode_equal(mode, &fixed);
	}

	/*
	 * TODO: support synthesized modes through scaling
	 *
	 * A pipeline whose Type-C routes follow their CRTC may be given a
	 * display that another pipeline still drives: dcp_crtc_atomic_check()
	 * checks the mode against that one.
	 */
	return dcp_mode_settable(dcp, mode) || dcp_typec_follows_crtc(dcp);
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
			   struct drm_atomic_commit *state)
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
	INIT_LIST_HEAD(&scanout->link);
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
	if (dcp_is_ext26(dcp)) {
		struct iomfb_scanout_h17p *dropped;
		u64 bytes = ext26_scanout_bytes(scanout);

		/* Reserve every exposed object, including an unsuccessful present. */
		if (dcp->iomfb.scanout)
			bytes = dcpext26_backing_add(bytes,
						   ext26_scanout_bytes(dcp->iomfb.scanout));
		list_for_each_entry(dropped, &dcp->iomfb.dropped, link)
			bytes = dcpext26_backing_add(bytes, ext26_scanout_bytes(dropped));
		if (!dcpext26_backing_fits(dcp->ext26_retained_bytes, bytes)) {
			dev_warn_ratelimited(dcp->dev,
				"native26 backing budget exhausted; new present refused\n");
			/* This scanout has not been exposed to firmware. */
			iomfb_scanout_release_h17p(scanout);
			return NULL;
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

bool iomfb_present_has_surface_h17p(const struct dcp_swap_submit_req_h17p *request)
{
	unsigned int i;

	for (i = 0; i < SWAP_SURFACES; i++)
		if (!request->surf_null[i])
			return true;
	return false;
}

bool iomfb_apply_backlight_h17p(struct apple_dcp *dcp,
				const struct dcp_swap_submit_req_h17p *request,
				struct dcp_present_h17p *wire)
{
	struct iomfb_transaction *transaction = dcp->iomfb.active;
	struct dcp_backlight_present present;
	bool have_surface;
	int ret;

	lockdep_assert_held(&dcp->iomfb.lock);
	if (!dcp_backlight_active(dcp)) {
		if (!dcp_has_panel(dcp))
			return true;
		if (!dcp->brightness.maximum)
			return false;
		dcp->edr_present_sdr = 0;
		iomfb_encode_backlight_h17p(wire, 0,
					    dcp->brightness.maximum, false);
		return true;
	}
	if (!transaction || !dcp->brightness.maximum) {
		/* The caller drops this present ("swap failed"): say why. */
		dev_warn_ratelimited(dcp->dev,
				     "present dropped: transaction %d brightness maximum %u\n",
				     !!transaction, dcp->brightness.maximum);
		return false;
	}
	have_surface = iomfb_present_has_surface_h17p(request);
	/*
	 * Firmware 27 takes a swap with no surface of its own (a plane removed
	 * alone, every plane off) with the brightness block idle, as macOS sends
	 * its cursor and clear swaps. The level waits for a present of a surface.
	 */
	if (!have_surface && dcp->fw_compat == DCP_FIRMWARE_V_27_0) {
		dcp->edr_present_sdr = 0;
		iomfb_encode_backlight_h17p(wire, 0, dcp->brightness.maximum, false);
		return true;
	}
	ret = dcp_backlight_prepare(dcp, have_surface, &present);
	if (ret && ret != -EALREADY) {
		dev_warn_ratelimited(dcp->dev,
				     "present dropped: backlight prepare %d\n", ret);
		return false;
	}
	if (!ret) {
		transaction->backlight = present;
		transaction->backlight_reserved = true;
	}
	dcp->edr_present_sdr = present.nits;
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
	struct iomfb_scanout_h17p *dropped, *next;

	lockdep_assert_held(&dcp->iomfb.lock);
	/* The firmware is past every present before this one. */
	list_for_each_entry_safe(dropped, next, &dcp->iomfb.dropped, link) {
		list_del(&dropped->link);
		iomfb_scanout_release_h17p(dropped);
	}
	if (transaction) {
		transaction->completed = true;
		if (transaction->backlight_reserved) {
			dcp_backlight_complete(dcp, transaction->backlight.sequence, true);
			transaction->backlight_reserved = false;
		}
		if (transaction->brightness_only)
			return false;
	}
	if (dcp->fw_compat != DCP_FIRMWARE_V_27_0)
		atomic_cmpxchg(&dcp->iomfb.opaque_x_state, IOMFB_OPAQUE_X_WAITING,
			       IOMFB_OPAQUE_X_NEEDED);
	iomfb_scanout_complete_h17p(dcp);
	return true;
}

/*
 * The firmware accepts presents while an external pipe is not enabled, for
 * instance after a mode set that failed because the sink went away, and then
 * drops them: neither a completion nor an abort follows.  Such a present
 * would hold the queue until its timeout declares the coprocessor crashed,
 * and a teardown waiting for the queue to settle would fail.
 *
 * Called by the swap watchdog: retire the present.  The firmware might still
 * complete it or read it: until the firmware completes a later present, a
 * completion of it is ignored, and the framebuffers of the present and those
 * it displaced stay referenced.  Without a completed present for
 * IOMFB_DROPPED_MAX retired ones, the queue is left to its timeout.  Returns
 * false if the present ended or a newer one re-armed the watchdog in the
 * meantime.  Other firmware has no present state.
 */
bool iomfb_retire_dropped_present_h17p(struct apple_dcp *dcp)
{
	struct dcp_present_state_h17p *state = &dcp->present_state_h17p;
	u32 swap_id;

	if (!iomfb_uses_queue(dcp))
		return true;

	guard(mutex)(&dcp->iomfb.lock);
	swap_id = state->swap_id;
	if (time_before(jiffies, dcp->swap_watchdog_deadline) ||
	    !state->pending || !state->accepted)
		return false;
	if (!dcp_present_retire_h17p(state, swap_id))
		return true;

	iomfb_present_failed_h17p(dcp);
	if (!iomfb_present_brightness_only_h17p(dcp)) {
		dcp_unarm_retained_framebuffers(dcp, swap_id);
		if (dcp->iomfb.next_scanout) {
			list_add_tail(&dcp->iomfb.next_scanout->link,
				      &dcp->iomfb.dropped);
			dcp->iomfb.next_scanout = NULL;
		}
	}
	iomfb_queue_advance(dcp);
	return true;
}

/* Release a present's scanout unless it is displayed or was dropped. */
VISIBLE_IF_KUNIT void iomfb_scanout_put_h17p(struct apple_dcp *dcp,
					     struct iomfb_scanout_h17p *scanout)
{
	/* A transaction that never started is released without the lock. */
	if (!scanout)
		return;
	lockdep_assert_held(&dcp->iomfb.lock);
	if (scanout == dcp->iomfb.scanout || !list_empty(&scanout->link))
		return;
	if (dcp->iomfb.next_scanout == scanout)
		dcp->iomfb.next_scanout = NULL;
	if (dcp_is_ext26(dcp)) {
		/* An unsuccessful present can still have exposed its IOVAs. */
		list_add_tail(&scanout->link, &dcp->ext26_retained);
		dcp->ext26_retained_bytes = dcpext26_backing_add(
			dcp->ext26_retained_bytes, ext26_scanout_bytes(scanout));
		return;
	}
	iomfb_scanout_release_h17p(scanout);
}

static void iomfb_backlight_start(struct apple_dcp *dcp,
				  struct iomfb_transaction *transaction)
{
	struct iomfb_scanout_h17p *scanout = dcp->iomfb.scanout;
	bool have_surface = false;
	unsigned int i;

	if ((!dcp_backlight_pending(dcp) && !READ_ONCE(dcp->edr_ramping)) ||
	    !READ_ONCE(dcp->mode_state.valid) || !scanout ||
	    !dcp->connector || !dcp->connector->connected)
		return;
	for (i = 0; i < SWAP_SURFACES; i++)
		have_surface |= !!scanout->fb[i];
	if (!have_surface)
		return;
	/*
	 * That surface is the frame from before the sleep. With hold, the level waits for
	 * the compositor's first frame, which then carries it, as macOS lights its backlight
	 * after the first frame.
	 */
	if (READ_ONCE(dcp->s2r_scanout_stale)) {
		WRITE_ONCE(dcp->s2r_scanout_stale, false);
		dev_info(dcp->dev, "wake: brightness present %s\n",
			 READ_ONCE(dcp->s2r_scanout_hold) ? "held for the first new frame" :
			 "on the frame from before the sleep");
	}
	if (READ_ONCE(dcp->s2r_scanout_hold))
		return;

	dcp->swap.h17p = scanout->request;
	if (dcp->fw_compat == DCP_FIRMWARE_V_27_0)
		iomfb_present_backlight_v27(dcp);
	else
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

/*
 * A firmware 27 panel's EDR ramp short of its target with no frame for a
 * while: present the frame again, brightness only, and its swap takes the
 * next step (dcp_edr_prepare_v27()).  A compositor that presents a still
 * HDR frame once reaches the full headroom as on macOS.
 */
static void iomfb_edr_tick(struct work_struct *work)
{
	struct apple_dcp *dcp = container_of(to_delayed_work(work), struct apple_dcp,
					  iomfb.edr_tick);

	if (READ_ONCE(dcp->edr_ramping))
		iomfb_backlight_kick(dcp);
}

/*
 * After a V27 sleep's power-on: the last frame again at once. macOS has
 * frames queued before the panel's plug; the firmware brings the
 * link up and exports the panel's modes only with one pending.
 */
static void iomfb_wake_present_start(struct apple_dcp *dcp,
				     struct iomfb_transaction *transaction)
{
	struct iomfb_scanout_h17p *scanout = dcp->iomfb.scanout;
	bool have_surface = false;
	unsigned int i;

	if (!READ_ONCE(dcp->mode_state.valid) || !scanout || !dcp->connector)
		return;
	for (i = 0; i < SWAP_SURFACES; i++)
		have_surface |= !!scanout->fb[i];
	if (!have_surface)
		return;
	dev_info(dcp->dev, "presenting the last frame again after the power-on\n");
	/*
	 * This present comes before DRM's DPMS-on and the backlight core's resume, so it carries
	 * the panel's level rather than the blanked 0 (dcp_backlight_wake_lit()). It is not a
	 * resend either: the once-a-second resend budget stays for the firmware's enable-backlight
	 * request after the power-on, which must be answered by a present before the firmware
	 * enables its stage-1 blocks.
	 */
	if (dcp_has_panel(dcp) && dcp_backlight_active(dcp)) {
		int nits = dcp_backlight_wake_lit(dcp);

		dev_dbg(dcp->dev, "the first frame after the power-on carries %d nits\n", nits);
	}
	dcp->swap.h17p = scanout->request;
	/* macOS sends the colour matrix and its correction before the first swap. */
	iomfb_present_matrix_v27(dcp);
}

void iomfb_v27_wake_present(struct apple_dcp *dcp)
{
	struct iomfb_transaction *transaction;

	transaction = kzalloc_obj(*transaction);
	if (!transaction)
		return;
	transaction->start = iomfb_wake_present_start;
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
	struct drm_atomic_commit *state;
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
		/* Only native26 admission refuses unexposed backing nonfatally. */
		if (!dcp_is_ext26(dcp))
			WRITE_ONCE(dcp->crashed, true);
		schedule_work(&dcp->vblank_wq);
		return;
	}
	dcp->iomfb.next_scanout = atomic->scanout;
	/* The first frame since the wake, the compositor's: the held level goes with it. */
	WRITE_ONCE(dcp->s2r_scanout_stale, false);
	if (READ_ONCE(dcp->s2r_scanout_hold)) {
		WRITE_ONCE(dcp->s2r_scanout_hold, false);
		dev_info(dcp->dev, "wake: first new frame, backlight back with it\n");
	}
	if (dcp->fw_compat == DCP_FIRMWARE_V_27_0)
		iomfb_flush_v27(dcp, atomic->crtc, atomic->state);
	else
		iomfb_flush_h17p(dcp, atomic->crtc, atomic->state);
}

static void iomfb_atomic_release(struct iomfb_transaction *transaction)
{
	struct iomfb_atomic_transaction *atomic = container_of(transaction,
					 struct iomfb_atomic_transaction, transaction);

	iomfb_scanout_put_h17p(atomic->dcp, atomic->scanout);
	drm_atomic_commit_put(atomic->state);
	kfree(atomic);
}

static void iomfb_queue_atomic(struct apple_dcp *dcp, struct drm_crtc *crtc,
			       struct drm_atomic_commit *state)
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
	atomic->state = drm_atomic_commit_get(state);
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

void dcp_flush(struct drm_crtc *crtc, struct drm_atomic_commit *commit)
{
	struct platform_device *pdev = to_apple_crtc(crtc)->dcp;
	struct apple_dcp *dcp = platform_get_drvdata(pdev);

	if (iomfb_is_m4(dcp)) {
		struct drm_plane_state *plane = drm_atomic_get_new_plane_state(commit,
									     crtc->primary);
		struct drm_crtc_state *new = drm_atomic_get_new_crtc_state(commit, crtc);
		struct drm_gem_dma_object *obj;
		int ret;

		if (!new || !new->active)
			return;
		ret = iomfb_m4_error(dcp);
		/* A CRTC-only commit publishes no new framebuffer. */
		if (!ret && plane) {
			obj = plane->fb ? drm_fb_dma_get_gem_obj(plane->fb, 0) : NULL;
			ret = obj ? iomfb_m4_present(dcp, plane->fb, obj->dma_addr,
						       obj->base.size) : -EINVAL;
		}
		if (ret)
			dcp_drm_crtc_error(to_apple_crtc(crtc), new, -EIO);
		else
			dcp_drm_crtc_vblank(to_apple_crtc(crtc));
		return;
	}

	if (dcp->fw_compat == DCP_FIRMWARE_V_26_6) {
		iomfb_v26_6_flush(dcp, crtc, commit);
		return;
	}

	/*
	 * Desktop 26.x external display replugged while the compositor kept
	 * its CRTC active: no atomic enable follows, and the firmware, which
	 * powered the pipe off with the unplug, swallows every swap.  Run the
	 * power-on and mode set the enable would have run, once.
	 */
	if (dcp->fw_compat == DCP_FIRMWARE_V_26_6_DESKTOP &&
	    dcp->desktop->ext_pipe_off && !dcp->main_display &&
	    !READ_ONCE(dcp->mode_state.valid) && dcp->connector &&
	    READ_ONCE(dcp->connector->connected)) {
		struct drm_crtc_state *cs = drm_atomic_get_new_crtc_state(commit, crtc);

		if (cs && cs->active && cs->mode.hdisplay) {
			dev_dbg(dcp->dev, "re-powering the external pipe of an active CRTC\n");
			dcp_poweron(pdev);
			dcp_crtc_atomic_modeset(crtc, commit);
		}
	}
	if (dcp->fw_compat == DCP_FIRMWARE_V_14_7) {
		iomfb_v14_7_flush(dcp, crtc, commit);
		return;
	}

	if ((dcp->fw_compat == DCP_FIRMWARE_V_26_6_DESKTOP ||
	     dcp->fw_compat == DCP_FIRMWARE_V_27_0) &&
	    READ_ONCE(dcp->mode_state.valid) && dcp->connector &&
	    READ_ONCE(dcp->connector->connected)) {
		/* A panel's EDR switches without a mode set (dcp_edr_live()). */
		dcp_edr_sync(dcp, crtc, commit);
		if (dcp->fw_compat == DCP_FIRMWARE_V_26_6_DESKTOP)
			dcp_hdr_metadata_sync(dcp, crtc, commit);
	}

	if (iomfb_uses_queue(dcp)) {
		iomfb_queue_atomic(dcp, crtc, commit);
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
	if ((to_apple_atomic_state(commit)->failed_routes & drm_crtc_mask(crtc)) ||
	    !dcp_crtc_route_ready(crtc, commit, true) ||
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
		iomfb_flush_v12_3(dcp, crtc, commit);
		break;
	case DCP_FIRMWARE_V_13_5:
		iomfb_flush_v13_3(dcp, crtc, commit);
		break;
	case DCP_FIRMWARE_V_14_7_TEMPLATE:
		iomfb_flush_v14_7_0(dcp, crtc, commit);
		break;
	case DCP_FIRMWARE_V_26_6_DESKTOP:
		iomfb_flush_v26_6_desktop(dcp, crtc, commit);
		break;
	case DCP_FIRMWARE_H17P:
		iomfb_flush_h17p(dcp, crtc, commit);
		break;
	case DCP_FIRMWARE_V_27_0:
		iomfb_flush_v27(dcp, crtc, commit);
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
	case DCP_FIRMWARE_V_26_6_DESKTOP:
		iomfb_start_v26_6_desktop(dcp);
		break;
	case DCP_FIRMWARE_H17P:
		iomfb_start_h17p(dcp);
		break;
	case DCP_FIRMWARE_V_27_0:
		iomfb_start_v27(dcp);
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

	if (dcp_fail_stop(dcp) && READ_ONCE(dcp->crashed))
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
		if (dcp_is_ext26(dcp)) {
			dev_dbg(dcp->dev, "native26 DCPLink ready before AFK startup\n");
			complete_all(&dcp->ext26_link_ready);
		} else {
			iomfb_start(dcp);
		}
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

void iomfb_ext26_start(struct apple_dcp *dcp)
{
	guard(mutex)(&dcp->iomfb.lock);
	WRITE_ONCE(dcp->iomfb.owner, current);
	iomfb_start(dcp);
	WRITE_ONCE(dcp->iomfb.owner, NULL);
	iomfb_queue_advance(dcp);
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
	if ((dcp->fw_compat == DCP_FIRMWARE_H17P ||
	     dcp->fw_compat == DCP_FIRMWARE_V_27_0) &&
	    dcp->hw.iomfb_method_profile != DCP_IOMFB_METHODS_H17P_EXTERNAL &&
	    !dcp_is_ext26(dcp)) {
		ret = apple_rtkit_start_ep(dcp->rtk, REMOTE_ALLOC_ENDPOINT);
		if (ret)
			return ret;
	}
	/* Desktop 26.x started the transport with its auxiliary endpoints. */
	if (dcp->fw_compat != DCP_FIRMWARE_V_26_6_DESKTOP) {
		ret = apple_rtkit_start_ep(dcp->rtk, IOMFB_ENDPOINT);
		if (ret)
			return ret;
	}

	dcp->shmem = dma_alloc_coherent(dcp->dev, DCP_SHMEM_SIZE, &shmem_iova,
					GFP_KERNEL);
	if (!dcp->shmem)
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
	case DCP_FIRMWARE_V_26_6_DESKTOP:
		iomfb_shutdown_v26_6_desktop(dcp);
		break;
	case DCP_FIRMWARE_H17P:
		iomfb_shutdown_h17p(dcp);
		break;
	case DCP_FIRMWARE_V_27_0:
		iomfb_shutdown_v27(dcp);
		break;
	default:
		WARN_ONCE(true, "Unexpected firmware version: %u\n", dcp->fw_compat);
		break;
	}
}

/*
 * The IOMFB parameters macOS sends to an external display after each mode set,
 * in 16.16 fixed point: parameter 3 at 1.0, and parameter 6 at 1.0 for an SDR
 * colour mode or 2.0 for a PQ (HDR) one.  Fills @out[0] and @out[1].
 */
void iomfb_external_hdr_hint_values(bool pq, struct iomfb_property out[2])
{
	out[0].id = 3;
	out[0].value = 1U << 16;			/* 1.0 */
	out[1].id = 6;
	out[1].value = pq ? (2U << 16) : (1U << 16);	/* 2.0 PQ, 1.0 SDR */
}

#if IS_ENABLED(CONFIG_DRM_APPLE_PARSER_KUNIT_TEST)
#include "iomfb-parser-test.c"
#endif

static u64 ext26_scanout_bytes(struct iomfb_scanout_h17p *scanout)
{
	u64 bytes = 0;
	unsigned int i;

	for (i = 0; i < SWAP_SURFACES; i++)
		if (scanout->fb[i] && scanout->fb[i]->obj[0])
			bytes = dcpext26_backing_add(bytes, scanout->fb[i]->obj[0]->size);
	return bytes;
}

/* A release/transport boundary is not a backing-retirement witness. */
int iomfb_ext26_retain_epoch(struct apple_dcp *dcp)
{
	struct iomfb_scanout_h17p *scanout;
	u64 bytes = 0;

	guard(mutex)(&dcp->iomfb.lock);
	if (!iomfb_channels_idle(dcp) || dcp->present_state_h17p.pending ||
	    dcp->iomfb.active || dcp->iomfb.next_scanout || dcp->chunks.data ||
	    dcp->ext26_control_pending)
		return -EBUSY;
	if (dcp->iomfb.scanout)
		bytes = dcpext26_backing_add(bytes, ext26_scanout_bytes(dcp->iomfb.scanout));
	list_for_each_entry(scanout, &dcp->iomfb.dropped, link)
		bytes = dcpext26_backing_add(bytes, ext26_scanout_bytes(scanout));
	/*
	 * Admission reserved this backing before exposure. Retaining it here
	 * allocates nothing: a quota refusal after power/link withdrawal would
	 * incorrectly latch external_error and strand the shared Type-C route.
	 */
	if (dcp->iomfb.scanout) {
		list_add_tail(&dcp->iomfb.scanout->link, &dcp->ext26_retained);
		dcp->iomfb.scanout = NULL;
	}
	list_splice_tail_init(&dcp->iomfb.dropped, &dcp->ext26_retained);
	dcp->ext26_retained_bytes = dcpext26_backing_add(dcp->ext26_retained_bytes, bytes);
	if (bytes)
		dcp->ext26_epochs++;
	dcp->ext26_blanked = false;
	dcp->ext26_normal = false;
	dcp->ext26_early_complete = false;
	dev_dbg(dcp->dev, "native26 retained epoch %u: %llu backing bytes; remote/DMA fence unproved\n",
		 dcp->ext26_epochs, dcp->ext26_retained_bytes);
	return 0;
}

/* A retained old attachment may still address this GEM object. */
bool iomfb_ext26_backing_fresh(struct apple_dcp *dcp, struct drm_framebuffer *fb)
{
	struct iomfb_scanout_h17p *scanout;
	unsigned int i;

	guard(mutex)(&dcp->iomfb.lock);
	list_for_each_entry(scanout, &dcp->ext26_retained, link)
		for (i = 0; i < SWAP_SURFACES; i++)
			if (scanout->fb[i] && scanout->fb[i]->obj[0] == fb->obj[0])
				return false;
	return true;
}
