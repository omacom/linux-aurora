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
#include <linux/soc/apple/dp-tunnel.h>
#include <linux/soc/apple/rtkit.h>

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

/* Call a DCP function given by a tag */
void dcp_push(struct apple_dcp *dcp, bool oob, const struct dcp_method_entry *call,
		     u32 in_len, u32 out_len, void *data, dcp_callback_t cb,
		     void *cookie)
{
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

	u8 depth = dcp_push_depth(&ch->depth);
	u16 offset = dcp_packet_start(ch, depth);

	void *out = dcp->shmem + dcp_tx_offset(context) + offset;
	void *out_data = out + sizeof(header);
	size_t data_len = sizeof(header) + in_len + out_len;

	memcpy(out, &header, sizeof(header));

	if (in_len > 0)
		memcpy(out_data, data, in_len);

	trace_iomfb_push(dcp, call, context, offset, depth);

	ch->callbacks[depth] = cb;
	ch->cookies[depth] = cookie;
	ch->output[depth] = out + sizeof(header) + in_len;
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

void dcp_hotplug(struct work_struct *work)
{
	struct apple_connector *connector;
	struct platform_device *pdev;
	struct apple_dcp *dcp;
	bool notify_only;
	unsigned int reasons;
	int ret;

	connector = container_of(work, struct apple_connector, hotplug_wq);
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
	if (!notify_only && connector->base.state &&
	    !READ_ONCE(dcp->mode_state.valid) && connector->connected &&
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
		ret = replace_modes(handle, &dcp->modes, &dcp->nr_modes,
				    dcp->width_mm, dcp->height_mm, dcp->notch_height,
				    dcp->fixed_connector_type == DRM_MODE_CONNECTOR_eDP);
		if (!ret) {
			dcp->modes_admitted = true;
			dcp->modes_provisional = false;
			/*
			 * Described again after a withdrawal it was kept
			 * connected through, the same display keeps its EDID,
			 * unless that is a placeholder being retried.
			 */
			if (!iomfb_v14_7_external_holding(dcp) ||
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
		u64 generation;

		/* The RPC can sleep while a disconnect or route change clears EDID. */
		if (apple_connector_edid_begin(apple_connector, pdev, &generation)) {
			edid = dcpavserv_copy_edid(dcp);
			if (IS_ERR_OR_NULL(edid))
				dev_info(dcp->dev, "copy_edid failed: %pe\n", edid);
			else
				apple_connector_edid_install(apple_connector, pdev,
							     generation, edid);
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

	guard(mutex)(&dcp->modes_lock);
	if (!dcp_modes_for_connector(dcp, apple_connector))
		return MODE_BAD;
	return lookup_mode_locked(dcp, mode, NULL) ? MODE_OK : MODE_BAD;
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

	/*
	 * TODO: support synthesized modes through scaling
	 *
	 * A pipeline whose Type-C routes follow their CRTC may be given a
	 * display that another pipeline still drives: dcp_crtc_atomic_check()
	 * checks the mode against that one.
	 */
	return lookup_mode(dcp, mode, NULL) || dcp_typec_follows_crtc(dcp);
}


void dcp_flush(struct drm_crtc *crtc, struct drm_atomic_state *state)
{
	struct platform_device *pdev = to_apple_crtc(crtc)->dcp;
	struct apple_dcp *dcp = platform_get_drvdata(pdev);

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

void iomfb_recv_msg(struct apple_dcp *dcp, u64 message)
{
	enum dcpep_type type = FIELD_GET(IOMFB_MESSAGE_TYPE, message);

	if (type == IOMFB_MESSAGE_TYPE_INITIALIZED)
		iomfb_start(dcp);
	else if (type == IOMFB_MESSAGE_TYPE_MSG)
		dcpep_got_msg(dcp, message);
	else
		dev_warn(dcp->dev, "Ignoring unknown message %llx\n", message);
}

int iomfb_start_rtkit(struct apple_dcp *dcp)
{
	dma_addr_t shmem_iova;
	int ret;

	ret = apple_rtkit_start_ep(dcp->rtk, IOMFB_ENDPOINT);
	if (ret)
		return ret;

	dcp->shmem = dma_alloc_coherent(dcp->dev, DCP_SHMEM_SIZE, &shmem_iova,
					GFP_KERNEL);

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
	default:
		WARN_ONCE(true, "Unexpected firmware version: %u\n", dcp->fw_compat);
		break;
	}
}
