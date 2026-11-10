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
#include <linux/soc/apple/rtkit.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_atomic.h>
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


/*
 * A353 / A352(0x70,0) / A438 x5 / A352(0x70,1) form one bracket: the A352 pair
 * parks and resumes the APT timing sequencer around the A438 config push
 * ("apt_normal_mode_changed: requesting transtition to auto mode" ->
 * set_run_mode_safe 4 -> 2 -> power_down_M3).  A438 makes our firmware take a
 * CPU exception, and opening the bracket without closing it leaves M3 powered
 * down, so the swap is accepted but never scans out.  Off = skip all of it and
 * leave the sequencer in normal mode.
 */
/*
 * The scanout engine reads through disp0_dart stream 0 (display-subsystem),
 * not through the DCP's own stream.  The only surface memory mapped in that
 * stream is the display carveout: reserved-memory/framebuffer@101eb4b8000
 * advertises iommu-addresses = <&dcp 0x1ea0000 ...>, <&display-subsystem
 * 0x1ea0000 ...>.  A GEM buffer's DVA exists only in the DCP domain, so a swap
 * naming one validates and retires while the scanout engine reads an address
 * it cannot translate -- a black panel with no visible fault, because the
 * DART's error IRQ is masked once the fault budget is spent.
 */
/*
 * macOS's full StartEP set; see iomfb_start_rtkit().  Off: bringing all sixteen
 * up made the firmware NMI (panic(app): app NMI, pidx: 1) before the first swap
 * could retire, so the endpoints past 0x2d/0x37 need to be introduced one at a
 * time rather than as a block.
 */

/*
 * Bitmask of the endpoints in 0x20..0x3f to start beyond 0x20/0x2d/0x37, bit n
 * meaning endpoint 0x20+n.  Starting macOS's whole set at once made the
 * firmware NMI, so bring them up one at a time: disp0 (0x23) is extra_eps=0x8.
 */
static unsigned int dcp_extra_eps;
module_param_named(extra_eps, dcp_extra_eps, uint, 0644);
MODULE_PARM_DESC(extra_eps, "bitmask of extra endpoints to start, bit n = 0x20+n");

static void dcp_start_extra_eps(struct apple_dcp *dcp)
{
	unsigned int n;

	for (n = 0; n < 32; n++) {
		if (!(dcp_extra_eps & BIT(n)))
			continue;
		dev_info(dcp->dev, "starting extra endpoint %#04x\n", 0x20 + n);
		dcp_start_rtkit_endpoint(dcp, 0x20 + n);
	}
}


bool dcp_defer_client_open;
module_param_named(defer_client_open, dcp_defer_client_open, bool, 0644);
MODULE_PARM_DESC(defer_client_open, "H17P: do not send first_client_open during boot");

static bool skip_hotplug;
module_param(skip_hotplug, bool, 0644);
MODULE_PARM_DESC(skip_hotplug, "do not raise the DRM hotplug event (bring-up bisect)");

bool dcp_swap_trace;
module_param_named(swap_trace, dcp_swap_trace, bool, 0644);
MODULE_PARM_DESC(swap_trace, "log every swap start/submit/complete with its id and time (flicker diagnosis)");

/*
 * H17P per-layer swap flags.  macOS 25G83 carries surf_flags[0] = 1 in every
 * captured A408 and m1n1's dcp experiment uses the same [1, 0, 0, 0]; the
 * Linux driver has always sent 0 and relied on is_premultiplied, which the
 * H17P surface no longer has (the firmware then blends the XRGB X byte as
 * straight alpha: black wherever a Vulkan-rendered frame leaves it 0).
 * Runtime knob so a Vulkan session can flip it without a reboot; 0 keeps
 * today's behaviour until a capture proves what bit 0 does.
 */
unsigned int dcp_h17p_surf_flags;
module_param_named(h17p_surf_flags, dcp_h17p_surf_flags, uint, 0644);
MODULE_PARM_DESC(h17p_surf_flags, "value for the enabled layer's swap.surf_flags on H17P (macOS: 1; default 0)");

/*
 * Camera-free check of what the panel is fed: after each of the next N
 * swap_complete callbacks read the blend-output CRC (A106,
 * IOMFB::UPPipe2::read_blend_crc) and log it with the swap id.  Two frames
 * that differ only in bytes the pipe should ignore (the XRGB X byte) must
 * give the same CRC; a static desktop must give a constant one.
 */
int dcp_blend_crc_reads;
module_param_named(blend_crc_reads, dcp_blend_crc_reads, int, 0644);
MODULE_PARM_DESC(blend_crc_reads, "read and log the blend-output CRC after the next N completed swaps (H17P)");

/*
 * Byte pokes into the outgoing H17P swap record, applied after the layout
 * fix-ups: "off=val[,off=val...]" with hex offsets into the 0xe9c A408
 * request and hex byte values.  Together with blend_crc_reads this lets a
 * script search the record for the bit that marks a layer opaque without
 * a rebuild per guess.  Empty = no pokes.
 */
/*
 * Panel dither control: A448 IOMobileFramebufferAP::enable_disable_dithering(u32)
 * (in u32, out u32 status).  Writing 0 or 1 issues the call after the next
 * completed swap and logs the status; the value then returns to -1.  The
 * DCP's UPBlock_Dither_v3 temporally dithers the pipe's output to the panel
 * depth ("IOMFB: dither, enabled: %s legacy: %s"), which is the first
 * suspect for the CRT-like shimmer Ryan sees on 10-bit output and faintly
 * on 8-bit.
 */
int dcp_dither_request = -1;
module_param_named(dither, dcp_dither_request, int, 0644);
MODULE_PARM_DESC(dither, "write 0/1 to disable/enable panel dithering after the next completed swap (H17P, A448)");

/*
 * A030 IOMFB::UPPipeAP_H17P::read_crc(Notify_Info_Index, u32): pipe CRCs
 * at points other than the blend output (the index enum is unknown; probe
 * it).  pipe_crc_reads=N reads index pipe_crc_idx after the next N
 * completed swaps and logs the value.
 */
int dcp_pipe_crc_reads;
module_param_named(pipe_crc_reads, dcp_pipe_crc_reads, int, 0644);
MODULE_PARM_DESC(pipe_crc_reads, "read A030 read_crc(pipe_crc_idx, 0) after the next N completed swaps (H17P)");
unsigned int dcp_pipe_crc_idx;
module_param_named(pipe_crc_idx, dcp_pipe_crc_idx, uint, 0644);
MODULE_PARM_DESC(pipe_crc_idx, "Notify_Info_Index for pipe_crc_reads");

/*
 * Runtime properties (the RuntimeProperty enum is the firmware's name table
 * in the 25G83 dcpf: 0x49 enableGPAlphaDiv, 0x86 IOMFBBlendDisabledKey, 0x1e
 * enableDither ...).  rtp="<prop>=<val>" (hex) issues one A352
 * {u32 prop, u32 value} after the next completed swap and logs the status.
 * The A200/A201 PropRelay stubs this knob used before made the firmware
 * panic ("rpc_callee_gated: payload size mismatch: 16 != 24").
 */
char dcp_rtp_request[32];
module_param_string(rtp, dcp_rtp_request, sizeof(dcp_rtp_request), 0644);
MODULE_PARM_DESC(rtp, "<prop>=<val> (hex): set a DCP runtime property (A352) after the next completed swap (H17P)");

char dcp_h17p_swap_poke[128];
module_param_string(h17p_swap_poke, dcp_h17p_swap_poke, sizeof(dcp_h17p_swap_poke), 0644);
MODULE_PARM_DESC(h17p_swap_poke, "hex off=val byte pokes into the fixed-up H17P swap record (debug)");

static int dcp_trace_budget;
module_param_named(trace_rpc, dcp_trace_budget, int, 0644);
static bool dcp_trace_calls;
module_param_named(trace_calls, dcp_trace_calls, bool, 0644);
MODULE_PARM_DESC(trace_calls, "Trace AP calls after the bounded RPC trace budget is consumed");
MODULE_PARM_DESC(trace_rpc,
		 "log the first N DCP RPC messages (0 = off; the vblank callback\n"
		 "\t\t alone is 60/s, so anything non-zero fills the kernel ring)");

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
	return READ_ONCE(ch->depth) != 0;
}

static void dcp_dispatch_pending_flushes(struct apple_dcp *dcp);

/*
 * Get the context ID passed to the DCP for a command we push. The rule is
 * simple: callback contexts are used when replying to the DCP, command
 * contexts are used otherwise. That corresponds to a non/zero call stack
 * depth. This rule frees the caller from tracking the call context manually.
 */
static enum dcp_context_id dcp_call_context(struct apple_dcp *dcp, bool oob)
{
	u8 depth = oob ? dcp->ch_oobcmd.depth : dcp->ch_cmd.depth;

	/*
	 * A nested call goes on the callback channel, past the record we are
	 * answering.  This matches macOS: its A401 carries context 2 in the
	 * wire word while every call nested inside it carries context 0 with
	 * offset 0x40, which is exactly what this produces.
	 */
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
	struct dcp_method_entry resolved = *call;

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

	u8 depth = dcp_push_depth(&ch->depth);
	u16 offset = dcp_packet_start(ch, depth);

	void *out = dcp->shmem + dcp_tx_offset(context) + offset;
	void *out_data = out + sizeof(header);
	size_t data_len = sizeof(header) + in_len + out_len;

	memcpy(out, &header, sizeof(header));

	if (in_len > 0)
		memcpy(out_data, data, in_len);

	trace_iomfb_push(dcp, call, context, offset, depth);
	/* Explicit AP tracing can outlast the callback/property trace budget.
	 * Leave it disabled for normal desktop operation.
	 */
	if (dcp_trace_budget > 0 || dcp_trace_calls) {
		if (dcp_trace_budget > 0)
			dcp_trace_budget--;
		dev_info(dcp->dev, "CALL %.4s ctx=%x off=%#x in=%u out=%u\n",
			 call->tag, context, offset, in_len, out_len);
	}


	memcpy(ch->tag[depth], call->tag, sizeof(ch->tag[depth]));
	ch->pushed_at[depth] = jiffies;
	if (context == DCP_CONTEXT_CMD && depth == 0) {
		dcp->warned_stall = false;
		mod_delayed_work(system_wq, &dcp->stall_wq,
				 msecs_to_jiffies(1000));
	}
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

/*
 * The firmware silently stops answering a method it is not ready for, and the
 * call then sits on the command channel forever -- every later push sees a busy
 * channel and the whole bring-up stalls with no clue which call did it.  Name it.
 */
static void dcp_dump_callback_trace(struct apple_dcp *dcp)
{
	struct dcp_callback_trace records[DCP_CALLBACK_TRACE_COUNT];
	unsigned long flags;
	u32 count, first, i;

	if (!dcp->hw.adopt_live_session)
		return;
	spin_lock_irqsave(&dcp->callback_trace_lock, flags);
	count = dcp->callback_trace_count;
	memcpy(records, dcp->callback_trace, sizeof(records));
	spin_unlock_irqrestore(&dcp->callback_trace_lock, flags);
	first = count > DCP_CALLBACK_TRACE_COUNT ? count - DCP_CALLBACK_TRACE_COUNT : 0;
	dev_info(dcp->dev, "DCP callback history: %u received, last %u follow\n",
		 count, count - first);
	for (i = first; i < count; ++i) {
		struct dcp_callback_trace *r = &records[i % DCP_CALLBACK_TRACE_COUNT];
		dev_info(dcp->dev, "CB[%u] %c%c%c%c in=%u out=%u data=%*ph\n",
			 i, r->tag[3], r->tag[2], r->tag[1], r->tag[0],
			 r->in_len, r->out_len, min_t(u32, r->in_len, sizeof(r->input)),
			 r->input);
	}
}

void dcp_stall_check(struct work_struct *work)
{
	struct apple_dcp *dcp = container_of(to_delayed_work(work),
					     struct apple_dcp, stall_wq);
	struct dcp_channel *ch = &dcp->ch_cmd;
	u8 d = ch->depth;

	if (!d) {
		dcp->warned_stall = false;
		return;
	}

	if (!dcp->warned_stall &&
	    time_after(jiffies, ch->pushed_at[d - 1] + msecs_to_jiffies(2000))) {
		dcp->warned_stall = true;
		dev_err(dcp->dev,
			"DCP stopped answering %.4s (depth %u, %u ms outstanding)\n",
			ch->tag[d - 1], d,
			jiffies_to_msecs(jiffies - ch->pushed_at[d - 1]));
		/* by now a fresh panic record has been written, if there is one */
		if (dcp->hw.adopt_live_session) {
			struct dcp_channel *nested = &dcp->ch_cb;
			u8 depth = READ_ONCE(nested->depth);

			if (depth && depth <= DCP_MAX_CALL_DEPTH)
				dev_info(dcp->dev,
					 "DCP nested callback channel: %.4s depth=%u\n",
					 nested->tag[depth - 1], depth);
		}
		dcp_dump_callback_trace(dcp);
		dcp_dump_crashlog(dcp);
	}

	schedule_delayed_work(&dcp->stall_wq, msecs_to_jiffies(1000));
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
static int dcp_retrain_active_crtc(struct apple_connector *connector)
{
	struct drm_device *dev = connector->base.dev;
	struct drm_modeset_acquire_ctx ctx;
	struct drm_crtc *crtc;
	int ret;

	DRM_MODESET_LOCK_ALL_BEGIN(dev, ctx, 0, ret);

	crtc = connector->base.state ? connector->base.state->crtc : NULL;
	if (crtc && crtc->state && crtc->state->active)
		ret = drm_atomic_helper_reset_crtc(crtc, &ctx);
	else
		ret = 0;

	DRM_MODESET_LOCK_ALL_END(dev, ctx, ret);

	return ret;
}

void dcp_retrain_oob(struct apple_connector *connector)
{
	struct apple_dcp *dcp = platform_get_drvdata(connector->dcp);

	if (!READ_ONCE(connector->connected) || !READ_ONCE(dcp->valid_mode))
		return;

	/*
	 * Bringing up another high-speed Type-C route can disturb an active DPTX
	 * stream without changing its HPD state.  Invalidate the IOMFB mode and
	 * use the normal hotplug worker to replay the active CRTC from process
	 * context; sending a synthetic disconnect would tear down the connector.
	 */
	WRITE_ONCE(dcp->valid_mode, false);
	schedule_work(&connector->hotplug_wq);
}

void dcp_hotplug(struct work_struct *work)
{
	struct apple_connector *connector;
	struct apple_dcp *dcp;
	int ret;

	connector = container_of(work, struct apple_connector, hotplug_wq);

	dcp = platform_get_drvdata(connector->dcp);
	dev_info(dcp->dev, "%s() connected:%d valid_mode:%d nr_modes:%u\n", __func__,
		 connector->connected, dcp->valid_mode, dcp->nr_modes);

	if (skip_hotplug) {
		dev_info(dcp->dev, "skip_hotplug set; not notifying DRM\n");
		return;
	}

	if (!connector->connected) {
		drm_edid_free(connector->drm_edid);
		connector->drm_edid = NULL;
	}

	/*
	 * DCP defers link training until we set a display mode. But we set
	 * display modes from atomic_flush, so userspace needs to trigger a
	 * flush, or the CRTC gets no signal.
	 */
	if (connector->base.state && !dcp->valid_mode && connector->connected) {
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

static void dcpep_handle_cb(struct apple_dcp *dcp, enum dcp_context_id context,
			    void *data, u32 length, u16 offset)
{
	struct device *dev = dcp->dev;
	struct dcp_packet_header *hdr = data;
	void *in, *out;
	int tag = dcp_parse_tag(hdr->tag);
	struct dcp_channel *ch = dcp_get_channel(dcp, context);
	u8 depth;

	if (dcp_trace_budget > 0 && tag != 300) {
		dcp_trace_budget--;
		char nm[41] = "";
		const char *ib = (const char *)(hdr + 1);
		u32 k;

		/* most property callbacks lead with a printable key */
		for (k = 0; k < hdr->in_len && k < sizeof(nm) - 1; k++) {
			if (ib[k] == '\0')
				break;
			nm[k] = (ib[k] >= 0x20 && ib[k] < 0x7f) ? ib[k] : '.';
		}
		nm[k] = '\0';
		dev_info(dev, "CB %c%c%c%c ctx=%x in=%u out=%u %s\n",
			 hdr->tag[3], hdr->tag[2], hdr->tag[1], hdr->tag[0],
			 context, hdr->in_len, hdr->out_len, nm);
		dev_info(dev, "CB input %*ph\n", min_t(u32, hdr->in_len, 32), ib);
	}

	in = data + sizeof(*hdr);
	out = in + hdr->in_len;
	if (dcp->hw.adopt_live_session && length >= sizeof(*hdr)) {
		struct dcp_callback_trace *r;
		unsigned long flags;
		u32 size = min_t(u32, hdr->in_len, length - sizeof(*hdr));

		spin_lock_irqsave(&dcp->callback_trace_lock, flags);
		r = &dcp->callback_trace[dcp->callback_trace_count % DCP_CALLBACK_TRACE_COUNT];
		memset(r, 0, sizeof(*r));
		memcpy(r->tag, hdr->tag, sizeof(r->tag));
		r->in_len = hdr->in_len;
		r->out_len = hdr->out_len;
		memcpy(r->input, in, min_t(u32, size, sizeof(r->input)));
		dcp->callback_trace_count++;
		spin_unlock_irqrestore(&dcp->callback_trace_lock, flags);
	}
	/* Keep the suspect frequency callback visible even when the startup
	 * property traffic has consumed the optional RPC trace budget.
	 */
	if (tag == 408 && dcp->hw.adopt_live_session)
		dev_info(dev, "D408 clock request in=%u out=%u data=%*ph\n",
			 hdr->in_len, hdr->out_len, min_t(u32, hdr->in_len, 16), in);

	if (tag < 0 || tag >= IOMFB_MAX_CB || !dcp->cb_handlers ||
	    !dcp->cb_handlers[tag]) {
		dev_warn(dev, "received unknown callback %c%c%c%c\n",
			 hdr->tag[3], hdr->tag[2], hdr->tag[1], hdr->tag[0]);
		/*
		 * Leaving a callback unanswered wedges the coprocessor: it
		 * waits for the ack forever and the outer call never returns.
		 * On H17P answer it with a zeroed output instead, so the
		 * warning above names the tag we still have to implement
		 * rather than costing a reboot to discover one tag at a time.
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

	if (tag < 0 || tag >= IOMFB_MAX_CB || !dcp->cb_handlers ||
	    !dcp->cb_handlers[tag]) {
		dev_warn(dev, "received unknown callback %c%c%c%c\n",
			 hdr->tag[3], hdr->tag[2], hdr->tag[1], hdr->tag[0]);
		/*
		 * Leaving a callback unanswered wedges the coprocessor: it
		 * waits for the ack forever and the outer call never returns.
		 * On H17P answer it with a zeroed output instead, so the
		 * warning above names the tag we still have to implement
		 * rather than costing a reboot to discover one tag at a time.
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
	ch->tag[depth][0] = hdr->tag[3];
	ch->tag[depth][1] = hdr->tag[2];
	ch->tag[depth][2] = hdr->tag[1];
	ch->tag[depth][3] = hdr->tag[0];
	ch->pushed_at[depth] = jiffies;

	{
		bool acked = dcp->cb_handlers[tag](dcp, tag, out, in);

		if (dcp_trace_budget > 0 && hdr->out_len && tag != 300) {
			dcp_trace_budget--;
			dev_info(dev, "RPL %c%c%c%c %*ph\n",
				 hdr->tag[3], hdr->tag[2], hdr->tag[1],
				 hdr->tag[0],
				 (int)min_t(u32, hdr->out_len, 16), out);
		}
		if (acked)
			dcp_ack(dcp, context);
	}
}

static void dcpep_handle_ack(struct apple_dcp *dcp, enum dcp_context_id context)
{
	struct dcp_channel *ch = dcp_get_channel(dcp, context);
	void *cookie, *out;
	dcp_callback_t cb;

	if (!ch || !ch->depth) {
		dev_warn(dcp->dev, "ignoring ack on empty/invalid context %X\n", context);
		return;
	}

	dcp_pop_depth(&ch->depth);

	cb = ch->callbacks[ch->depth];
	cookie = ch->cookies[ch->depth];
	/* ACKs carry no payload and have offset zero, including nested calls.
	 * Their output is in the original AP command record, not the receive
	 * callback region selected by context0. dcp_push saved that address.
	 */
	out = ch->output[ch->depth];

	ch->callbacks[ch->depth] = NULL;
	ch->cookies[ch->depth] = NULL;
	ch->output[ch->depth] = NULL;

	if (cb)
		cb(dcp, out, cookie);

	if (context == DCP_CONTEXT_CMD) {
		mutex_lock(&dcp->pending_flush_lock);
		if (!dcp_channel_busy(ch))
			dcp_dispatch_pending_flushes(dcp);
		mutex_unlock(&dcp->pending_flush_lock);
	}
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

	if (dcp_trace_budget > 0) {
		dcp_trace_budget--;
		dev_info(dcp->dev, "RX %#018llx ctx=%x off=%#x len=%u ack=%d\n",
			 message, ctx_id, offset, length,
			 !!FIELD_GET(IOMFB_MSG_ACK, message));
	}

	channel_offset = dcp_channel_offset(ctx_id);

	if (channel_offset < 0) {
		dev_warn(dcp->dev, "invalid context received %u\n", ctx_id);
		return;
	}

	data = dcp->shmem + channel_offset + offset;

	if (FIELD_GET(IOMFB_MSG_ACK, message))
		dcpep_handle_ack(dcp, ctx_id);
	else
		dcpep_handle_cb(dcp, ctx_id, data, length, offset);
}

int dcp_get_modes(struct drm_connector *connector)
{
	struct apple_connector *apple_connector = to_apple_connector(connector);
	struct platform_device *pdev = apple_connector->dcp;
	struct apple_dcp *dcp;

	struct drm_device *dev = connector->dev;
	struct drm_display_mode *mode;
	u16 min_vfreq = 0, max_vfreq = 0;
	bool vrr_capable = false;
	int i;

	/* A Type-C port has no pipeline while the fabric is moving it. */
	if (!pdev)
		return 0;
	dcp = platform_get_drvdata(pdev);

	for (i = 0; i < dcp->nr_modes; ++i) {
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
			return 0;
		}

		drm_mode_probed_add(connector, mode);
	}
	drm_connector_set_vrr_capable_property(connector, vrr_capable);

	if (dcp->nr_modes && dcp->dcpavserv.enabled &&
	    !apple_connector->drm_edid) {
		const struct drm_edid *edid;
		edid = dcpavserv_copy_edid(dcp->dcpavserv.service);
		if (IS_ERR_OR_NULL(edid)) {
			dev_info(dcp->dev, "copy_edid failed: %pe\n", edid);
		} else {
			drm_edid_free(apple_connector->drm_edid);
			apple_connector->drm_edid = edid;
		}
	}
	if (dcp->nr_modes && apple_connector->drm_edid)
		drm_edid_connector_update(connector, apple_connector->drm_edid);

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

	return dcp->nr_modes;
}

/* The user may own drm_display_mode, so we need to search for our copy */
struct dcp_display_mode *lookup_mode(struct apple_dcp *dcp,
					    const struct drm_display_mode *mode)
{
	int i;

	for (i = 0; i < dcp->nr_modes; ++i) {
		if (drm_mode_match(mode, &dcp->modes[i].mode,
				   DRM_MODE_MATCH_TIMINGS |
					   DRM_MODE_MATCH_CLOCK))
			return &dcp->modes[i];
	}

	return NULL;
}

enum drm_mode_status dcp_mode_valid(struct drm_connector *connector,
				    const struct drm_display_mode *mode)
{
	struct apple_connector *apple_connector = to_apple_connector(connector);
	struct platform_device *pdev = apple_connector->dcp;
	struct apple_dcp *dcp;

	if (!pdev)
		return MODE_ERROR;
	dcp = platform_get_drvdata(pdev);

	return lookup_mode(dcp, mode) ? MODE_OK : MODE_BAD;
}

int dcp_crtc_atomic_modeset(struct drm_crtc *crtc,
			    struct drm_atomic_state *state)
{
	struct apple_crtc *apple_crtc = to_apple_crtc(crtc);
	struct apple_dcp *dcp = platform_get_drvdata(apple_crtc->dcp);
	struct drm_crtc_state *crtc_state;
	int ret = -EIO;
	bool modeset;

	crtc_state = drm_atomic_get_new_crtc_state(state, crtc);
	if (!crtc_state)
		return 0;

	modeset = drm_atomic_crtc_needs_modeset(crtc_state) || !dcp->valid_mode;

	if (!modeset)
		return 0;

	/* ignore no mode, poweroff is handled elsewhere */
	if (crtc_state->mode.hdisplay == 0 && crtc_state->mode.vdisplay == 0)
		return 0;

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

	return ret;
}

bool dcp_crtc_mode_fixup(struct drm_crtc *crtc,
			 const struct drm_display_mode *mode,
			 struct drm_display_mode *adjusted_mode)
{
	struct apple_crtc *apple_crtc = to_apple_crtc(crtc);
	struct platform_device *pdev = apple_crtc->dcp;
	struct apple_dcp *dcp = platform_get_drvdata(pdev);

	/* TODO: support synthesized modes through scaling */
	return lookup_mode(dcp, mode) != NULL;
}

/*
 * Retain every framebuffer this commit unbinds from the CRTC's planes until
 * a later swap has replaced it on the panel.
 *
 * This has to happen for every commit, whether or not it results in a swap:
 * the coprocessor keeps scanning out the previous surface until a swap
 * replaces it, so a commit that is skipped -- the command channel busy, a
 * plane disabled with nothing to show, the CRTC going off -- must not let
 * the framebuffer (and with it the display DART mapping the firmware is
 * still fetching through) go away.  The J700 showed what happens
 * otherwise: on DPMS off the disp0 DART logged "read PTE invalid" faults
 * across the whole old surface 400 us after the CRTC disable, and the
 * firmware took an axi_rd_err NMI (hb100).
 *
 * Entries are queued unarmed; the swap that unbinds them stamps its id in
 * dcp_arm_swapped_out_fbs(), and a commit without a swap leaves them held
 * until the next swap does.
 *
 * commit_tail_rpm() skips plane hooks on inactive CRTCs, even when an update
 * replaces their framebuffer. The driver's commit-tail wrapper retains all
 * displaced framebuffers before calling that helper. The prepared entry is
 * consumed once.
 */
static void dcp_retain_unbound_fbs(struct apple_dcp *dcp, struct drm_crtc *crtc,
			    struct drm_atomic_state *state)
{
	struct drm_plane *plane;
	struct drm_plane_state *old_state, *new_state;
	int i;

	for_each_oldnew_plane_in_state(state, plane, old_state, new_state, i) {
		struct apple_plane_state *apple_state = to_apple_plane_state(new_state);
		struct dcp_fb_reference *entry;
		unsigned long flags;

		if (old_state->crtc == crtc && old_state->fb &&
		    (new_state->fb != old_state->fb || new_state->crtc != crtc)) {
			/* prepare_fb reserved this before the state swap. */
			entry = apple_state->retiring_fb;
			/* Disable and flush can both visit the same state. */
			if (!entry)
				continue;
			apple_state->retiring_fb = NULL;
			entry->fb = old_state->fb;
			drm_framebuffer_get(old_state->fb);
			spin_lock_irqsave(&dcp->swapped_out_lock, flags);
			list_add_tail(&entry->head, &dcp->swapped_out_fbs);
			spin_unlock_irqrestore(&dcp->swapped_out_lock, flags);
		}
	}
}

void dcp_retain_unbound_fbs_pdev(struct platform_device *pdev, struct drm_crtc *crtc,
				 struct drm_atomic_state *state)
{
	dcp_retain_unbound_fbs(platform_get_drvdata(pdev), crtc, state);
}

static void dcp_dispatch_flush(struct apple_dcp *dcp, struct drm_crtc *crtc,
			       struct drm_atomic_state *state)
{


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
	if (!dcp->valid_mode || !dcp->connector || !dcp->connector->connected) {
		schedule_delayed_work(&dcp->vblank_wq, 0);
		return;
	}
	if (dcp_channel_busy(&dcp->ch_cmd))
	{
		if (!dcp->ch_cmd.warned_busy) {
			u8 d = dcp->ch_cmd.depth;

			dev_err(dcp->dev,
				"unexpected busy command channel: depth %u, oldest outstanding %.4s pushed %u ms ago\n",
				d, d ? dcp->ch_cmd.tag[d - 1] : "----",
				d ? jiffies_to_msecs(jiffies -
						     dcp->ch_cmd.pushed_at[d - 1])
				  : 0);
			dcp->ch_cmd.warned_busy = true;
		}
		/* HACK: issue a delayed vblank event to avoid timeouts in
		 * drm_atomic_helper_wait_for_vblanks().
		 */
		schedule_delayed_work(&dcp->vblank_wq, 0);
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

static void dcp_dispatch_pending_flushes(struct apple_dcp *dcp)
{
	struct apple_crtc_state *apple_state;
	struct drm_atomic_state *state;

again:
	lockdep_assert_held(&dcp->pending_flush_lock);
	if (dcp_channel_busy(&dcp->ch_cmd) ||
	    list_empty(&dcp->pending_flushes))
		return;

	apple_state = list_first_entry(&dcp->pending_flushes,
				       struct apple_crtc_state, pending_flush);
	list_del_init(&apple_state->pending_flush);
	state = apple_state->base.state;

	dcp_dispatch_flush(dcp, apple_state->base.crtc, state);
	drm_atomic_state_put(state);

	if (!dcp_channel_busy(&dcp->ch_cmd))
		goto again;
}

void dcp_flush(struct drm_crtc *crtc, struct drm_atomic_state *state)
{
	struct platform_device *pdev = to_apple_crtc(crtc)->dcp;
	struct apple_dcp *dcp = platform_get_drvdata(pdev);
	struct drm_crtc_state *base;
	struct apple_crtc_state *apple_state;

	base = drm_atomic_get_new_crtc_state(state, crtc);
	apple_state = to_apple_crtc_state(base);
	drm_atomic_state_get(state);

	mutex_lock(&dcp->pending_flush_lock);
	WARN_ON(!list_empty(&apple_state->pending_flush));
	list_add_tail(&apple_state->pending_flush, &dcp->pending_flushes);
	dcp_dispatch_pending_flushes(dcp);
	mutex_unlock(&dcp->pending_flush_lock);
}

void dcp_flush_shutdown(struct apple_dcp *dcp)
{
	struct apple_crtc_state *apple_state, *tmp;
	LIST_HEAD(pending);

	mutex_lock(&dcp->pending_flush_lock);
	list_splice_init(&dcp->pending_flushes, &pending);
	mutex_unlock(&dcp->pending_flush_lock);

	list_for_each_entry_safe(apple_state, tmp, &pending, pending_flush) {
		struct drm_atomic_state *state = apple_state->base.state;

		list_del_init(&apple_state->pending_flush);
		drm_atomic_state_put(state);
	}
}

/*
 * Apply the backlight level to an H17P panel whose CRTC is off by
 * re-presenting the surface already on it (iomfb_present_brightness()).
 * The caller holds the CRTC lock and has waited for the last commit.
 */
int dcp_h17p_present_brightness(struct apple_dcp *dcp)
{
	if (dcp->fw_compat != DCP_FIRMWARE_H17P)
		return -ENODEV;
	if (dcp->crashed || !dcp->active || !dcp->valid_mode ||
	    !dcp->connector || !dcp->connector->connected)
		return -ENODEV;
	if (dcp_channel_busy(&dcp->ch_cmd))
		return -EBUSY;

	return iomfb_present_brightness_h17p(dcp);
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

void iomfb_recv_msg(struct apple_dcp *dcp, u64 message)
{
	enum dcpep_type type = FIELD_GET(IOMFB_MESSAGE_TYPE, message);

	if (type == IOMFB_MESSAGE_TYPE_INITIALIZED) {
		/*
		 * H17P reports its firmware version and hash in the high half
		 * of the same InitComplete word (macOS 25G83 sends e.g.
		 * 0x004045bc9ef40101).  The transport below it is unchanged.
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

int iomfb_start_rtkit(struct apple_dcp *dcp)
{
	dma_addr_t shmem_iova;
	int ret;

	/*
	 * macOS starts sixteen DCP endpoints; we started three.  Its StartEP
	 * order, straight from the hv capture, is
	 *   24 28 21 2c 31 25 2d 32 2b 2e 2a 37 29 20 23 22
	 * with 0x20 (system) coming from systemep_init earlier.  A firmware
	 * subsystem whose endpoint was never started has nothing answering it
	 * on the AP side -- which is how 0x2d's absence used to produce DART
	 * faults -- so bring the whole set up, in that order.
	 */
	ret = dcp_start_rtkit_endpoint(dcp, REMOTE_ALLOC_ENDPOINT);
	if (ret)
		return ret;
	ret = dcp_start_rtkit_endpoint(dcp, IOMFB_ENDPOINT);
	if (ret)
		return ret;
	dcp_start_extra_eps(dcp);

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
	dcp->valid_mode = false;

	switch (dcp->fw_compat) {
	case DCP_FIRMWARE_V_12_3:
		iomfb_shutdown_v12_3(dcp);
		break;
	case DCP_FIRMWARE_V_13_5:
		iomfb_shutdown_v13_3(dcp);
		break;
	case DCP_FIRMWARE_H17P:
		iomfb_shutdown_h17p(dcp);
		break;
	default:
		WARN_ONCE(true, "Unexpected firmware version: %u\n", dcp->fw_compat);
		break;
	}
}
