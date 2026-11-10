// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Copyright 2021 Alyssa Rosenzweig
 * Copyright The Asahi Linux Contributors
 */

#include <clocksource/arm_arch_timer.h>
#include <linux/align.h>
#include <linux/bitmap.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/iommu.h>
#include <linux/kref.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/unaligned.h>

#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fourcc.h>

/* macOS uses 6 for its single layer in the captured swaps */
#define DCP_SWAP_SURFACE_ID_BASE 6
/* from the captured macOS swap records */
#define DCP_SWAP_FLAGS1 0x34865202ULL
#define DCP_SWAP_FLAGS2 0x4ULL
#include <drm/drm_framebuffer.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_vblank.h>

#include "dcp.h"
#include "dcp-internal.h"
#include "iomfb.h"
#include "iomfb_internal.h"
#include "parser.h"
#include "trace.h"
#include "version_utils.h"

/* Register defines used in bandwidth setup structure */
#define REG_DOORBELL_BIT(idx) (2 + (idx))

static_assert(offsetof(struct DCP_FW_NAME(dcp_swap), timestamp[6]) == 0x30);
static_assert(offsetof(struct DCP_FW_NAME(dcp_swap), flags1) == 0x40);

struct dcp_wait_cookie {
	struct kref refcount;
	struct completion done;
};

static void release_wait_cookie(struct kref *ref)
{
	struct dcp_wait_cookie *cookie;
	cookie = container_of(ref, struct dcp_wait_cookie, refcount);

        kfree(cookie);
}

DCP_THUNK_OUT(iomfb_a131_pmu_service_matched, iomfbep_a131_pmu_service_matched, u32);
DCP_THUNK_OUT(iomfb_a132_backlight_service_matched, iomfbep_a132_backlight_service_matched, u32);
DCP_THUNK_OUT(iomfb_a358_vi_set_temperature_hint, iomfbep_a358_vi_set_temperature_hint, u32);

IOMFB_THUNK_INOUT(set_matrix);
IOMFB_THUNK_INOUT(get_color_remap_mode);
IOMFB_THUNK_INOUT(last_client_close);
IOMFB_THUNK_INOUT(abort_swaps_dcp);

DCP_THUNK_INOUT(dcp_swap_submit, dcpep_swap_submit,
		struct DCP_FW_NAME(dcp_swap_submit_req),
		struct DCP_FW_NAME(dcp_swap_submit_resp));

DCP_THUNK_INOUT(dcp_swap_start, dcpep_swap_start, struct DCP_FW_NAME(dcp_swap_start_req),
		struct DCP_FW_NAME(dcp_swap_start_resp));

DCP_THUNK_INOUT(dcp_set_power_state, dcpep_set_power_state,
		struct DCP_FW_NAME(dcp_set_power_state_req),
		struct dcp_set_power_state_resp);

/*
 * macOS issues A473 out-of-band: the traced call is "OOBCMD 0x0 A473 0x10 0x8"
 * with unklong = 1.  Sent on the ordinary command channel the firmware simply
 * never answers it, which strands the power transition and every later call
 * behind it.
 */
#if DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0)
#define DCP_POWER_OOB	true
#else
#define DCP_POWER_OOB	false
#endif

DCP_THUNK_INOUT(dcp_set_digital_out_mode, dcpep_set_digital_out_mode,
		struct dcp_set_digital_out_mode_req, u32);

DCP_THUNK_INOUT(dcp_set_display_device, dcpep_set_display_device, u32, u32);

DCP_THUNK_OUT(dcp_set_display_refresh_properties,
	      dcpep_set_display_refresh_properties, u32);

#if DCP_FW_VER >= DCP_FW_VERSION(13, 2, 0)
DCP_THUNK_INOUT(dcp_late_init_signal, dcpep_late_init_signal, u32, u32);
#else
DCP_THUNK_OUT(dcp_late_init_signal, dcpep_late_init_signal, u32);
#endif
DCP_THUNK_IN(dcp_flush_supports_power, dcpep_flush_supports_power, u32);
DCP_THUNK_OUT(dcp_create_default_fb, dcpep_create_default_fb, u32);
DCP_THUNK_OUT(dcp_start_signal, dcpep_start_signal, u32);
DCP_THUNK_VOID(dcp_setup_video_limits, dcpep_setup_video_limits);
DCP_THUNK_VOID(dcp_set_create_dfb, dcpep_set_create_dfb);

#if DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0)
/*
 * The default-framebuffer interrogation macOS performs inside D121.  The
 * getters are called with a zeroed input of the traced size; macOS leaves the
 * tail as poison, so only the size matters.
 */
struct dcp_dfb_info_h17p {
	u8 data[0x2c];
} __packed;

struct dcp_dfb_layout_req_h17p {
	u8 data[0xc];
} __packed;

struct dcp_dfb_dimensions_h17p {
	u32 width;
	u32 height;
} __packed;

DCP_THUNK_INOUT(dcp_get_dfb_compression_info, dcpep_get_dfb_compression_info,
		u64, u64);
DCP_THUNK_INOUT(dcp_get_dfb_info, dcpep_get_dfb_info,
		struct dcp_dfb_info_h17p, struct dcp_dfb_info_h17p);
DCP_THUNK_INOUT(dcp_get_dfb_layout, dcpep_get_dfb_layout,
		struct dcp_dfb_layout_req_h17p, u64);
DCP_THUNK_INOUT(dcp_get_dfb_state, dcpep_get_dfb_state, u32, u64);
DCP_THUNK_IN(dcp_set_dfb_dimensions, dcpep_set_dfb_dimensions,
	     struct dcp_dfb_dimensions_h17p);
DCP_THUNK_VOID(dcp_commit_dfb_info, dcpep_commit_dfb_info);
DCP_THUNK_OUT(dcp_dfb_query, dcpep_dfb_query, u32);
DCP_THUNK_OUT(dcp_dfb_ready_query, dcpep_dfb_ready_query, u32);

/*
 * macOS runs a fixed sequence of pipe-configuration calls between the boot
 * chain and its first swap.  We do not have names for most of them, but the
 * capture gives exact sizes and payloads, and replaying it verbatim is what
 * takes the firmware from "booted" to "willing to scan out".
 */
struct dcp_pipe_cfg_415 {
	u8 data[0xc];
} __packed;

/*
 * A031 is update_notify_clients_dcp(u32[27]): a bitmap-as-array of the DCP
 * notification clients the AP wants delivered.  macOS enables them
 * cumulatively over nine calls, ending at {6,7,8,10,11,12,25,26}; we used to
 * send nine zeroed records, so we finished with no notify clients registered
 * at all and never received the callbacks they gate.
 */
struct dcp_pipe_cfg_031 {
	__le32 clients[27];
} __packed;
static_assert(sizeof(struct dcp_pipe_cfg_031) == 0x6c);

DCP_THUNK_OUT(dcp_pipe_query_353, dcpep_pipe_query_353, u64);
DCP_THUNK_INOUT(dcp_pipe_set_352, dcpep_pipe_set_352, u64, u32);
DCP_THUNK_INOUT(dcp_pipe_cfg_415, dcpep_pipe_cfg_415,
		struct dcp_pipe_cfg_415, struct dcp_pipe_cfg_415);
DCP_THUNK_IN(dcp_pipe_cfg_031, dcpep_pipe_cfg_031, struct dcp_pipe_cfg_031);
DCP_THUNK_INOUT(dcp_pipe_cfg_414, dcpep_pipe_cfg_414, u64, u64);
DCP_THUNK_OUT(dcp_pipe_query_478, dcpep_pipe_query_478, u32);
DCP_THUNK_OUT(dcp_pipe_query_474, dcpep_pipe_query_474, u32);
DCP_THUNK_IN(dcp_pipe_set_476, dcpep_pipe_set_476, u32);
DCP_THUNK_INOUT(dcp_pipe_set_428, dcpep_pipe_set_428, u32, u32);

/*
 * A352 and the five A438 pushes macOS runs between A353 and the client-open
 * sequence.  Every byte is taken from the 25G83 capture; the 8-byte host
 * leading 8 bytes are sent
 * verbatim: zeroing them made the firmware take a CPU exception
 * (exception.c:59) on the third push.
 */
struct dcp_pipe_cfg_438 {
	u8 data[0x106c];
} __packed;

DCP_THUNK_INOUT(dcp_pipe_cfg_438, dcpep_pipe_cfg_438,
		struct dcp_pipe_cfg_438, u32);

/* Generated from the macOS 25G83 capture: five A438 pushes, 0x106c each,
 * byte-exact including the leading 8 bytes. */

#endif
DCP_THUNK_VOID(dcp_first_client_open, dcpep_first_client_open);

DCP_THUNK_INOUT(dcp_set_parameter_dcp, dcpep_set_parameter_dcp,
		struct dcp_set_parameter_dcp, u32);

DCP_THUNK_INOUT(dcp_enable_disable_video_power_savings,
		dcpep_enable_disable_video_power_savings, u32, int);

DCP_THUNK_OUT(dcp_is_main_display, dcpep_is_main_display, u32);
#if DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0)
DCP_THUNK_OUT(dcp_read_blend_crc, dcpep_read_blend_crc, u32);
DCP_THUNK_INOUT(dcp_enable_disable_dithering, dcpep_enable_disable_dithering, u32, u32);
DCP_THUNK_INOUT(dcp_read_crc, dcpep_read_crc, struct dcp_read_crc_req, u32);
#endif

/* DCP callback handlers */
static void dcpep_cb_nop(struct apple_dcp *dcp)
{
	/* No operation */
}

static u8 dcpep_cb_true(struct apple_dcp *dcp)
{
	return true;
}

static u8 dcpep_cb_false(struct apple_dcp *dcp)
{
	return false;
}

static u32 dcpep_cb_zero(struct apple_dcp *dcp)
{
	return 0;
}

/*
 * Bind every framebuffer displaced by the commit that produced this swap to
 * that swap's id.
 *
 * The id only exists once the firmware answers swap_start, after the atomic
 * commit has queued its entries, so they are queued unarmed and stamped here.
 * Swap-start requests are serialized by the DCP command stack.
 */
static void dcp_arm_swapped_out_fbs(struct apple_dcp *dcp, u32 swap_id)
{
	struct dcp_fb_reference *entry;
	unsigned long flags;

	spin_lock_irqsave(&dcp->swapped_out_lock, flags);
	list_for_each_entry(entry, &dcp->swapped_out_fbs, head) {
		if (entry->armed)
			continue;
		entry->swap_id = swap_id;
		entry->armed = true;
	}
	spin_unlock_irqrestore(&dcp->swapped_out_lock, flags);
}

/* A rejected replacement did not retire anything the pipe was reading. */
static void dcp_unarm_swapped_out_fbs(struct apple_dcp *dcp, u32 swap_id)
{
	struct dcp_fb_reference *entry;
	unsigned long flags;

	spin_lock_irqsave(&dcp->swapped_out_lock, flags);
	list_for_each_entry(entry, &dcp->swapped_out_fbs, head) {
		if (entry->armed && entry->swap_id == swap_id)
			entry->armed = false;
	}
	spin_unlock_irqrestore(&dcp->swapped_out_lock, flags);
}

/*
 * Drop framebuffers whose unbinding swap has completed.
 *
 * Entries must be keyed to the swap that unbinds them, rather than the last
 * swap that happened to have completed when the atomic commit ran.  A later
 * successful completion also retires entries whose own completion was lost;
 * the signed difference keeps that comparison valid across a 32-bit wrap.
 */
static void dcp_release_swapped_out_fbs(struct apple_dcp *dcp)
{
	struct dcp_fb_reference *entry, *tmp;
	unsigned long flags;
	LIST_HEAD(done);

	/*
	 * last_swap_id / have_swap_complete are only ever touched by the RTKit
	 * worker, which is also the only caller of this function, so the lock
	 * is here for the list alone.
	 */
	spin_lock_irqsave(&dcp->swapped_out_lock, flags);
	while (!list_empty(&dcp->swapped_out_fbs)) {
		entry = list_first_entry(&dcp->swapped_out_fbs,
					 struct dcp_fb_reference, head);

		/* Newest entries; their swap has not been started yet. */
		if (!entry->armed)
			break;
		/* last_swap_id is 0 from kzalloc until a swap really lands. */
		if (!dcp->have_swap_complete)
			break;
		/* Hold the old framebuffer for one additional completed swap. */
		if ((s32)(dcp->last_swap_id - entry->swap_id) <= 0)
			break;

		list_move_tail(&entry->head, &done);
	}
	spin_unlock_irqrestore(&dcp->swapped_out_lock, flags);

	/* drm_framebuffer_put() can sleep, so never call it under the lock. */
	list_for_each_entry_safe(entry, tmp, &done, head) {
		if (entry->fb)
			drm_framebuffer_put(entry->fb);
		list_del(&entry->head);
		kfree(entry);
	}
}

#if DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0)
static void res_blend_crc(struct apple_dcp *dcp, void *out, void *cookie)
{
	u32 crc = out ? *(u32 *)out : 0;

	dev_info(dcp->dev, "blend-crc: swap %u crc %#010x t=%lld us\n",
		 (u32)(uintptr_t)cookie, crc, ktime_to_us(ktime_get()));
}

static void res_dither(struct apple_dcp *dcp, void *out, void *cookie)
{
	u32 st = out ? *(u32 *)out : 0xffffffff;

	dev_info(dcp->dev, "enable_disable_dithering(%u): status %#x\n",
		 (u32)(uintptr_t)cookie, st);
}

static void res_rtp(struct apple_dcp *dcp, void *out, void *cookie)
{
	u32 st = out ? *(u32 *)out : 0xffffffff;

	dev_info(dcp->dev, "rtp set %#lx: status %#x\n",
		 (unsigned long)(uintptr_t)cookie, st);
}

#define DCP_H17P_RTP_ENABLE_GP_ALPHA_DIV 0x49

static void res_h17p_alpha_div(struct apple_dcp *dcp, void *out, void *cookie)
{
	u32 st = out ? *(u32 *)out : 0xffffffff;

	if (st)
		dev_warn(dcp->dev, "GP alpha divide off: status %#x\n", st);
}

static void dcp_issue_rtp_request(struct apple_dcp *dcp, dcp_callback_t cb)
{
	char *e;
	unsigned long prop, val;
	u64 rq;

	prop = simple_strtoul(dcp_rtp_request, &e, 16);
	if (*e != '=' || prop > U32_MAX)
		goto done;
	val = simple_strtoul(e + 1, &e, 16);
	if (val > U32_MAX)
		goto done;
	/* A352 in: {u32 prop, u32 value} */
	rq = (u64)val << 32 | prop;
	dcp->diag_res = res_rtp;
	dcp->diag_cookie = (void *)(uintptr_t)((prop << 8) | (val & 0xff));
	dcp_rtp_request[0] = '\0';
	dcp_pipe_set_352(dcp, false, &rq, cb, NULL);
	return;
done:
	dcp_rtp_request[0] = '\0';
	cb(dcp, NULL, NULL);
}

static void res_pipe_crc(struct apple_dcp *dcp, void *out, void *cookie)
{
	u32 v = out ? *(u32 *)out : 0xffffffff;

	dev_info(dcp->dev, "pipe-crc: idx %u swap %u value %#010x t=%lld us\n",
		 dcp_pipe_crc_idx, (u32)(uintptr_t)cookie, v,
		 ktime_to_us(ktime_get()));
}
#endif

static void dcpep_cb_swap_complete(struct apple_dcp *dcp,
				   struct DCP_FW_NAME(dc_swap_complete_resp) *resp)
{
	ktime_t now = ktime_get();
	trace_iomfb_swap_complete(dcp, resp->swap_id);
	if (dcp_swap_trace)
		dev_info(dcp->dev, "swap-trace: complete id=%u t=%lld us\n",
			 resp->swap_id, ktime_to_us(now));
	dcp->last_swap_id = resp->swap_id;
	dcp->have_swap_complete = true;
	dcp->h17p_swapped_since_on = true;

	dcp_drm_crtc_page_flip(dcp, now);
	dcp_release_swapped_out_fbs(dcp);
	if (dcp->crc_enabled) {
		u32 crc32 = 0;
		drm_crtc_add_crc_entry(&dcp->crtc->base, true, resp->swap_id, &crc32);
	}
}

/* special */
static void complete_vi_set_temperature_hint(struct apple_dcp *dcp, void *out, void *cookie)
{
	// ack D100 cb_match_pmu_service
	dcp_ack(dcp, DCP_CONTEXT_CB);
}

static bool iomfbep_cb_match_pmu_service(struct apple_dcp *dcp, int tag, void *out, void *in)
{
	trace_iomfb_callback(dcp, tag, __func__);
	iomfb_a358_vi_set_temperature_hint(dcp, false,
					   complete_vi_set_temperature_hint,
					   NULL);

	// return false for deferred ACK
	return false;
}

/*
 * H17P asks the AP to make the default framebuffer surface (D582).  Answering
 * with a bare boolean is not enough: macOS registers an actual surface from
 * inside that callback with a nested A472 carrying a 0x22c descriptor, and
 * only then reports success.  Without it the firmware has no surface, programs
 * the display engine from a null base and the piodma DART faults forever.
 *
 * The descriptor below is the one macOS 25G83 sends on this panel; every byte
 * it did not write was left as tracer poison, so the rest is zero.  ARGB at
 * +0x0b is the format field of struct dcp_surface.
 */
struct dcp_h17p_dfb_surface_req {
	u8 surface[0x22c];
	u8 surface_null;
	u8 padding[3];
} __packed;
static_assert(sizeof(struct dcp_h17p_dfb_surface_req) == 0x230);

/* macOS sends this with a 4-byte output; with out_len 0 the firmware has
 * nowhere to write its result and simply stops answering. */
DCP_THUNK_INOUT(dcp_register_dfb_surface, dcpep_register_dfb_surface,
		struct dcp_h17p_dfb_surface_req, u32);

/* Fill the common SurfaceContents fields, matching the traced macOS bytes. */

static void dcp_ack_dfb_surface(struct apple_dcp *dcp)
{
	struct dcp_channel *ch = &dcp->ch_cb;
	u8 *succ = ch->output[ch->depth - 1];

	*succ = true;
	dcp_ack(dcp, DCP_CONTEXT_CB);
}

static void dcp_fill_dfb_surface(struct apple_dcp *dcp, u8 *s)
{
	s[0x002] = 0x01;
	s[0x00b] = 'A';
	s[0x00c] = 'R';
	s[0x00d] = 'G';
	s[0x00e] = 'B';
	s[0x019] = 0x01;
	s[0x01b] = 0x01;
	s[0x01c] = 0x01;
	s[0x02a] = 0x40;
	s[0x02b] = 0xe4;
	s[0x035] = 0x04;	/* IOSurface.surface_id; macOS writes 4, we wrote 3 */
	s[0x051] = 0x01;
	s[0x149] = 0x01;

	if (dcp->hw.iomfb_method_profile == DCP_IOMFB_METHODS_H17G) {
		/* H17G 25G76 A472: b3a8 surface and the native default surface ID.
		 * The word at0x29 remains opaque; these are its observed bytes,
		 * not a size inferred from panel geometry. No addresses are copied.
		 */
		memcpy(s + 0x00b, "8a3b", 4);
		s[0x02a] = 0;
		s[0x02b] = 0x4d;
		s[0x02c] = 1;
		s[0x035] = 2;
	}
}

/*
 * A471 (update_dfb) is never sent.  It hands the firmware a default-framebuffer
 * surface descriptor, and on H17P it makes the coprocessor take an AXI read
 * error and NMI immediately (axi_rd_err -> "panic(app): app NMI").  Scanout does
 * not need it: the swap carries its own surface, whose DVA comes from the DRM
 * GEM object on the display-subsystem device and is therefore already mapped in
 * the stream the pipe reads.  Just acknowledge the callback.
 */
static void complete_dfb_surface(struct apple_dcp *dcp, void *out, void *cookie)
{
	dcp_ack_dfb_surface(dcp);
}

static bool iomfbep_cb_create_dfb_surface(struct apple_dcp *dcp, int tag,
					  void *out, void *in)
{
	struct dcp_h17p_dfb_surface_req *req;

	trace_iomfb_callback(dcp, tag, __func__);

	/*
	 * 0x230 bytes is far too much for the kernel stack here: this runs deep
	 * inside the RTKit receive workqueue (rx work -> iomfb_recv_msg ->
	 * dcpep_handle_cb -> here -> dcp_push), and putting it on the stack
	 * overflows it and panics the machine.  dcp_push() copies the payload
	 * into shared memory before returning, so a short-lived allocation is
	 * all we need.
	 */
	req = kzalloc(sizeof(*req), GFP_KERNEL);
	if (!req) {
		dev_err(dcp->dev, "no memory for the default FB surface\n");
		return true;
	}

	dcp_fill_dfb_surface(dcp, req->surface);

	dcp_register_dfb_surface(dcp, false, req, complete_dfb_surface, NULL);
	kfree(req);

	/* deferred ACK: the A472 completion reports success for us */
	return false;
}

static void complete_pmu_service_matched(struct apple_dcp *dcp, void *out, void *cookie)
{
	struct dcp_channel *ch = &dcp->ch_cb;
	u8 *succ = ch->output[ch->depth - 1];

	*succ = true;

	// ack D206 cb_match_pmu_service_2
	dcp_ack(dcp, DCP_CONTEXT_CB);
}

static bool iomfbep_cb_match_pmu_service_2(struct apple_dcp *dcp, int tag, void *out, void *in)
{
	trace_iomfb_callback(dcp, tag, __func__);

	iomfb_a131_pmu_service_matched(dcp, false, complete_pmu_service_matched,
				       out);

	// return false for deferred ACK
	return false;
}

static void complete_backlight_service_matched(struct apple_dcp *dcp, void *out, void *cookie)
{
	struct dcp_channel *ch = &dcp->ch_cb;
	u8 *succ = ch->output[ch->depth - 1];

	*succ = true;

	// ack D206 cb_match_backlight_service
	dcp_ack(dcp, DCP_CONTEXT_CB);
}

static bool iomfbep_cb_match_backlight_service(struct apple_dcp *dcp, int tag, void *out, void *in)
{
	trace_iomfb_callback(dcp, tag, __func__);

	if (!dcp_has_panel(dcp)) {
		u8 *succ = out;
		*succ = true;
		return true;
	}

	iomfb_a132_backlight_service_matched(dcp, false, complete_backlight_service_matched, out);

	// return false for deferred ACK
	return false;
}

static void iomfb_cb_pr_publish(struct apple_dcp *dcp, struct iomfb_property *prop)
{
	switch (prop->id) {
	case IOMFB_PROPERTY_NITS:
	{
		if (dcp_has_panel(dcp)) {
			dcp->brightness.nits = prop->value / dcp->brightness.scale;
			/* notify backlight device of the initial brightness */
			if (!dcp->brightness.bl_dev && dcp->brightness.maximum > 0)
				schedule_work(&dcp->bl_register_wq);
			trace_iomfb_brightness(dcp, prop->value);
		}
		break;
	}
	default:
		dev_dbg(dcp->dev, "pr_publish: id: %d = %u\n", prop->id, prop->value);
	}
}

static struct dcp_get_uint_prop_resp
dcpep_cb_get_uint_prop(struct apple_dcp *dcp, struct dcp_get_uint_prop_req *req)
{
	struct dcp_get_uint_prop_resp resp = (struct dcp_get_uint_prop_resp){
	    .value = 0
	};

	if ((dcp->panel.has_mini_led ||
	     (DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0) && dcp_has_panel(dcp))) &&
	    memcmp(req->obj, "SUMP", sizeof(req->obj)) == 0) { /* "PMUS */
	    if (strncmp(req->key, "Temperature", sizeof(req->key)) == 0) {
		/*
		 * TODO: value from j314c, find out if it is temperature in
		 *       centigrade C and which temperature sensor reports it
		 *
		 * H17P (J700) asks for this too and logs "set_PMU_service:
		 * failed to read PMU temperature property!" without an
		 * answer; macOS 25G83 answered 2997 (29.97 C) in the capture,
		 * which the firmware then used as the backlight temperature
		 * compensator's initial guess.
		 */
		resp.value = 3029;
		resp.ret = true;
	    }
	}

	return resp;
}

static u8 iomfbep_cb_sr_set_property_int(struct apple_dcp *dcp,
					 struct iomfb_sr_set_property_int_req *req)
{
	if (memcmp(req->obj, "FMOI", sizeof(req->obj)) == 0) { /* "IOMF */
		if (strncmp(req->key, "Brightness_Scale", sizeof(req->key)) == 0) {
			if (!req->value_null)
				dcp->brightness.scale = req->value;
		}
	}

	return 1;
}

static void iomfbep_cb_set_fx_prop(struct apple_dcp *dcp, struct iomfb_set_fx_prop_req *req)
{
    // TODO: trace this, see if there properties which needs to used later
}

/*
 * Callback to map a buffer allocated with allocate_buf for PIODMA usage.
 * PIODMA is separate from the main DCP and uses own IOVA space on a dedicated
 * stream of the display DART, rather than the expected DCP DART.
 */
static struct DCP_FW_NAME(dcp_map_buf_resp) dcpep_cb_map_piodma(struct apple_dcp *dcp,
						   struct dcp_map_buf_req *req)
{
	struct dcp_mem_descriptor *memdesc;
	struct sg_table *map;
	ssize_t ret;

	if (req->buffer >= ARRAY_SIZE(dcp->memdesc))
		goto reject;

	memdesc = &dcp->memdesc[req->buffer];
	map = &memdesc->map;

	if (!map->sgl)
		goto reject;

	/* use the piodma iommu domain to map against the right IOMMU */
	ret = iommu_map_sgtable(dcp->iommu_dom, memdesc->dva, map,
				IOMMU_READ | IOMMU_WRITE);

	/* HACK: expect size to be 16K aligned since the iommu API only maps
	 *       full pages
	 */
	if (ret < 0 || ret != ALIGN(memdesc->size, SZ_16K)) {
		dev_err(dcp->dev, "iommu_map_sgtable() returned %zd instead of expected buffer size of %zu\n", ret, memdesc->size);
		goto reject;
	}

	/* The PIODMA mapping belongs only to the DCP IOMMU domain. */
	memdesc->piodma_mapped = true;

#if DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0)
	/* echo the index back, as macOS does */
	return (struct DCP_FW_NAME(dcp_map_buf_resp)){
		.buffer = req->buffer,
		.dva = memdesc->dva,
	};
#else
	return (struct DCP_FW_NAME(dcp_map_buf_resp)){ .dva = memdesc->dva };
#endif

reject:
	dev_err(dcp->dev, "denying map of invalid buffer %llx for piodma\n",
		req->buffer);
#if DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0)
#if DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0)
	return (struct DCP_FW_NAME(dcp_map_buf_resp)){
		.buffer = req->buffer,
		.dva = 0,
	};
#else
	return (struct DCP_FW_NAME(dcp_map_buf_resp)){ .dva = 0 };
#endif
#else
	return (struct DCP_FW_NAME(dcp_map_buf_resp)){ .ret = EINVAL };
#endif
}

static void dcpep_cb_unmap_piodma(struct apple_dcp *dcp,
				  struct dcp_unmap_buf_resp *resp)
{
	struct dcp_mem_descriptor *memdesc;

	if (resp->buffer >= ARRAY_SIZE(dcp->memdesc)) {
		dev_warn(dcp->dev, "unmap request for out of range buffer %llu\n",
			 resp->buffer);
		return;
	}

	memdesc = &dcp->memdesc[resp->buffer];

	if (!memdesc->buf) {
		dev_warn(dcp->dev,
			 "unmap for non-mapped buffer %llu iova:0x%08llx\n",
			 resp->buffer, resp->dva);
		return;
	}

	if (memdesc->dva != resp->dva) {
		dev_warn(dcp->dev, "unmap buffer %llu address mismatch "
			 "memdesc.dva:%llx dva:%llx\n", resp->buffer,
			 memdesc->dva, resp->dva);
		return;
	}

	/* use the piodma iommu domain to unmap from the right IOMMU */
	/* HACK: expect size to be 16K aligned since the iommu API only maps
	 *       full pages
	 */
	iommu_unmap(dcp->iommu_dom, memdesc->dva, ALIGN(memdesc->size, SZ_16K));
	memdesc->piodma_mapped = false;
}

/*
 * Allocate an IOVA contiguous buffer mapped to the DCP. The buffer need not be
 * physically contiguous, however we should save the sgtable in case the
 * buffer needs to be later mapped for PIODMA.
 */
static struct dcp_allocate_buffer_resp
dcpep_cb_allocate_buffer(struct apple_dcp *dcp,
			 struct dcp_allocate_buffer_req *req)
{
	struct dcp_allocate_buffer_resp resp = { 0 };
	struct dcp_mem_descriptor *memdesc;
	size_t size;
	u32 id;

	resp.dva_size = ALIGN(req->size, 4096);
	resp.mem_desc_id =
		find_first_zero_bit(dcp->memdesc_map, DCP_MAX_MAPPINGS);

	if (resp.mem_desc_id >= DCP_MAX_MAPPINGS) {
		dev_warn(dcp->dev, "DCP overflowed mapping table, ignoring\n");
		resp.dva_size = 0;
		resp.mem_desc_id = 0;
		return resp;
	}
	id = resp.mem_desc_id;
	set_bit(id, dcp->memdesc_map);

	memdesc = &dcp->memdesc[id];

	memdesc->piodma_mapped = false;
	memdesc->size = resp.dva_size;
	/* HACK: align size to 16K since the iommu API only maps full pages */
	size = ALIGN(resp.dva_size, SZ_16K);
	memdesc->buf = dma_alloc_coherent(dcp->dev, size,
					  &memdesc->dva, GFP_KERNEL);

	dma_get_sgtable(dcp->dev, &memdesc->map, memdesc->buf, memdesc->dva,
			size);
	resp.dva = memdesc->dva;

	/*
	 * Report the physical address too.  We only ever filled in the device
	 * address, leaving paddr zero; macOS answers D451 with a real physical
	 * address (e.g. 0x100_2651_4000) alongside the DVA.  The firmware loads
	 * its APT M3 microcode -- "load APT M3 IMem/DMem" appears immediately
	 * after these allocations -- through that address, so with zero there
	 * the timing sequencer comes up and is powered straight back down.
	 */
	if (memdesc->map.sgl)
		resp.paddr = sg_phys(memdesc->map.sgl);

	return resp;
}

static u8 dcpep_cb_release_mem_desc(struct apple_dcp *dcp, u32 *mem_desc_id)
{
	struct dcp_mem_descriptor *memdesc;
	size_t size;
	u32 id = *mem_desc_id;

	if (id >= DCP_MAX_MAPPINGS) {
		dev_warn(dcp->dev,
			 "unmap request for out of range mem_desc_id %u", id);
		return 0;
	}

	if (!test_and_clear_bit(id, dcp->memdesc_map)) {
		dev_warn(dcp->dev, "unmap request for unused mem_desc_id %u\n",
			 id);
		return 0;
	}

	memdesc = &dcp->memdesc[id];
	size = ALIGN(memdesc->size, SZ_16K);

	/*
	 * Tear down any translation still standing before the pages go back to
	 * the allocator.  The firmware is expected to unmap piodma first, but
	 * if it releases a descriptor without doing so we would hand the pages
	 * back while the DART still resolves them -- the display engine would
	 * then read whoever owns them next.
	 */
	if (memdesc->piodma_mapped) {
		dev_warn(dcp->dev,
			 "releasing buffer %u while still mapped for piodma; unmapping first\n",
			 id);
		iommu_unmap(dcp->iommu_dom, memdesc->dva, size);
		memdesc->piodma_mapped = false;
	}

	if (memdesc->buf) {
		dma_free_coherent(dcp->dev, size, memdesc->buf, memdesc->dva);
		memdesc->buf = NULL;
		memset(&memdesc->map, 0, sizeof(memdesc->map));
	} else {
		memdesc->reg = 0;
	}

	memdesc->size = 0;

	return 1;
}

/* Validate that the specified region is a display register */
static bool is_disp_register(struct apple_dcp *dcp, u64 start, u64 end)
{
	int i;

	for (i = 0; i < dcp->nr_disp_registers; ++i) {
		struct resource *r = dcp->disp_registers[i];

		if ((start >= r->start) && (end <= r->end))
			return true;
	}

	return false;
}

/*
 * Map contiguous physical memory into the DCP's address space. The firmware
 * uses this to map the display registers we advertise in
 * sr_map_device_memory_with_index, so we bounds check against that to guard
 * safe against malicious coprocessors.
 */
static struct dcp_map_physical_resp
dcpep_cb_map_physical(struct apple_dcp *dcp, struct dcp_map_physical_req *req)
{
	int size = ALIGN(req->size, 4096);
	dma_addr_t dva;
	u32 id;

	if (!is_disp_register(dcp, req->paddr, req->paddr + size - 1)) {
		dev_err(dcp->dev, "refusing to map phys address %llx size %llx\n",
			req->paddr, req->size);
		return (struct dcp_map_physical_resp){};
	}

	id = find_first_zero_bit(dcp->memdesc_map, DCP_MAX_MAPPINGS);
	set_bit(id, dcp->memdesc_map);
	dcp->memdesc[id].size = size;
	dcp->memdesc[id].reg = req->paddr;

	dva = dma_map_resource(dcp->dev, req->paddr, size, DMA_BIDIRECTIONAL, 0);
	WARN_ON(dva == DMA_MAPPING_ERROR);

	return (struct dcp_map_physical_resp){
		.dva_size = size,
		.mem_desc_id = id,
		.dva = dva,
	};
}

static u64 dcpep_cb_get_frequency(struct apple_dcp *dcp)
{
	return clk_get_rate(dcp->clk);
}

static struct DCP_FW_NAME(dcp_map_reg_resp) dcpep_cb_map_reg(struct apple_dcp *dcp,
						struct DCP_FW_NAME(dcp_map_reg_req) *req)
{
#if DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0)
	/*
	 * H17P asks for one aperture the device tree does not describe on the
	 * dcp node: index 5, the disp0 window 0x302800000..0x3028bbfff.  That
	 * window holds the PMC block the firmware drives while bringing the
	 * display up, and macOS answers this exact request with
	 * addr = dva = 0x302800000, length = 0xbc000.  Refusing it (our
	 * reg-names stop at disp-4) leaves the firmware unable to program that
	 * hardware.  Answer it the way macOS does; these apertures are reached
	 * physically, so dva == addr.
	 */
	if (dcp->hw.legacy_pmc_binding && req->index == DCP_H17P_DISP0_REG_INDEX &&
	    !of_device_is_compatible(dcp->dev->of_node, "apple,t8140-dcpext")) {
		dev_info(dcp->dev, "map_reg[5]: disp0 window %#llx len %#llx\n",
			 (u64)DCP_H17P_DISP0_REG_BASE,
			 (u64)DCP_H17P_DISP0_REG_SIZE);
		return (struct DCP_FW_NAME(dcp_map_reg_resp)){
			.addr = DCP_H17P_DISP0_REG_BASE,
			.length = DCP_H17P_DISP0_REG_SIZE,
			.dva = DCP_H17P_DISP0_REG_BASE,
		};
	}
#endif

	if (req->index >= dcp->nr_disp_registers) {
		dev_warn(dcp->dev, "attempted to read invalid reg index %u\n",
			 req->index);

		return (struct DCP_FW_NAME(dcp_map_reg_resp)){ .ret = 1 };
	} else {
		struct resource *rsrc = dcp->disp_registers[req->index];
#if DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0)
		/*
		 * H17P reaches these apertures physically, not through its
		 * DART: macOS 25G83 answers this callback with dva == addr
		 * (0x302800000 for index 5, length 0xbc000).  Trying to
		 * dma_map_resource() them instead fails and hands the firmware
		 * DMA_MAPPING_ERROR as a device address.
		 */
		dma_addr_t dva = rsrc->start;
		dev_info(dcp->dev, "map_reg[%u]: addr %pa len %#llx dva %#llx\n",
			 req->index, &rsrc->start,
			 (u64)resource_size(rsrc), (u64)dva);
#elif DCP_FW_VER >= DCP_FW_VERSION(13, 2, 0)
		dma_addr_t dva = dma_map_resource(dcp->dev, rsrc->start, resource_size(rsrc),
						  DMA_BIDIRECTIONAL, 0);
		WARN_ON(dva == DMA_MAPPING_ERROR);
		dev_info(dcp->dev, "map_reg[%u]: addr %pa len %#llx dva %#llx\n",
			 req->index, &rsrc->start,
			 (u64)resource_size(rsrc), (u64)dva);
#else
		dev_info(dcp->dev, "map_reg[%u]: addr %pa len %#llx\n",
			 req->index, &rsrc->start, (u64)resource_size(rsrc));
#endif

		return (struct DCP_FW_NAME(dcp_map_reg_resp)){
			.addr = rsrc->start,
			.length = resource_size(rsrc),
#if DCP_FW_VER >= DCP_FW_VERSION(13, 2, 0)
			.dva = dva,
#endif
		};
	}
}

static struct dcp_read_edt_data_resp
dcpep_cb_read_edt_data(struct apple_dcp *dcp, struct dcp_read_edt_data_req *req)
{
	return (struct dcp_read_edt_data_resp){
		.value[0] = req->value[0],
		.ret = 0,
	};
}

static void iomfbep_cb_enable_backlight_message_ap_gated(struct apple_dcp *dcp,
							 u8 *enabled)
{
	/*
	 * update backlight brightness on next swap, on non mini-LED displays
	 * DCP seems to set an invalid iDAC value after coming out of DPMS.
	 * syslog: "[BrightnessLCD.cpp:743][AFK]nitsToDBV: iDAC out of range"
	 */
	dcp->brightness.update = true;
	mod_delayed_work(system_wq, &dcp->bl_update_wq,
			 msecs_to_jiffies(DIV_ROUND_UP(1000, 24)));
}

/* Chunked data transfer for property dictionaries */
static u8 dcpep_cb_prop_start(struct apple_dcp *dcp, u32 *length)
{
	if (dcp->chunks.data != NULL) {
		dev_warn(dcp->dev, "ignoring spurious transfer start\n");
		return false;
	}

	/*
	 * This length comes straight off the wire.  A bogus value turns the
	 * allocation below into a multi-gigabyte GFP_KERNEL request, and the
	 * box vanishes into reclaim with no oops and no fault -- it just stops.
	 * Property blobs here are a few KiB, so bound it and never let the
	 * allocator retry.
	 */
	if (*length == 0 || *length > SZ_1M) {
		dev_warn(dcp->dev, "rejecting %u byte property transfer\n",
			 *length);
		return false;
	}

	dcp->chunks.length = *length;
	dcp->chunks.data = kzalloc(*length, GFP_KERNEL | __GFP_NORETRY |
				      __GFP_NOWARN);

	if (!dcp->chunks.data) {
		dev_warn(dcp->dev, "failed to allocate %u chunk bytes\n",
			 *length);
		return false;
	}
	dev_info(dcp->dev, "property transfer: %u bytes\n", *length);

	return true;
}

static u8 dcpep_cb_prop_chunk(struct apple_dcp *dcp,
			      struct dcp_set_dcpav_prop_chunk_req *req)
{
	if (!dcp->chunks.data) {
		dev_warn(dcp->dev, "ignoring spurious chunk\n");
		return false;
	}

	if (req->offset + req->length > dcp->chunks.length) {
		dev_warn(dcp->dev, "ignoring overflowing chunk\n");
		return false;
	}

	memcpy(dcp->chunks.data + req->offset, req->data, req->length);
	return true;
}

static bool dcpep_process_chunks(struct apple_dcp *dcp,
				 struct dcp_set_dcpav_prop_end_req *req)
{
	struct dcp_parse_ctx ctx;
	int ret;

	if (!dcp->chunks.data) {
		dev_warn(dcp->dev, "ignoring spurious end\n");
		return false;
	}

	/* used just as opaque pointer for tracing */
	ctx.dcp = dcp;

	ret = parse(dcp->chunks.data, dcp->chunks.length, &ctx);

	if (ret) {
		dev_warn(dcp->dev, "bad header on dcpav props\n");
		return false;
	}

	if (!strcmp(req->key, "TimingElements")) {
		dcp->modes = enumerate_modes(&ctx, &dcp->nr_modes,
					     dcp->width_mm, dcp->height_mm,
					     dcp->notch_height,
					     dcp->fixed_connector_type ==
						     DRM_MODE_CONNECTOR_eDP);

		if (IS_ERR(dcp->modes)) {
			dev_warn(dcp->dev, "failed to parse modes\n");
			dcp->modes = NULL;
			dcp->nr_modes = 0;
			return false;
		}
		if (dcp->nr_modes == 0)
			dev_warn(dcp->dev, "TimingElements without valid modes!\n");
	} else if (!strcmp(req->key, "DisplayAttributes")) {
		ret = parse_display_attributes(&ctx, &dcp->width_mm,
					&dcp->height_mm);

		if (ret) {
			dev_warn(dcp->dev, "failed to parse display attribs\n");
			return false;
		}

		dcp_set_dimensions(dcp);
	}

	return true;
}

static u8 dcpep_cb_prop_end(struct apple_dcp *dcp,
			    struct dcp_set_dcpav_prop_end_req *req)
{
	u8 resp = dcpep_process_chunks(dcp, req);

	/* move chunked data to connector to provide it via debugfs */
	dcp_connector_update_dict(dcp->connector, req->key, &dcp->chunks);
	dcp->chunks.data = NULL;
	dcp->chunks.length = 0;

	return resp;
}

/* Boot sequence */
static void boot_done(struct apple_dcp *dcp, void *out, void *cookie)
{
	struct dcp_channel *ch = &dcp->ch_cb;
	u8 *succ = ch->output[ch->depth - 1];
	dev_dbg(dcp->dev, "boot done\n");

	*succ = true;
	dcp_ack(dcp, DCP_CONTEXT_CB);
}

static void boot_5b(struct apple_dcp *dcp, void *out, void *cookie)
{
	dcp_set_display_refresh_properties(dcp, false, boot_done, NULL);
}

static void boot_5(struct apple_dcp *dcp, void *out, void *cookie)
{
#if DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0)
	/* macOS issues A380 right before set_display_refresh_properties */
	dcp_dfb_ready_query(dcp, false, boot_5b, NULL);
#else
	boot_5b(dcp, out, cookie);
#endif
}

static void boot_4b(struct apple_dcp *dcp, void *out, void *cookie)
{
#if DCP_FW_VER >= DCP_FW_VERSION(13, 2, 0)
	/*
	 * macOS 25G83 sends 0 here, not 1: the traced A000 payload is
	 * "00 aa aa aa", one meaningful byte followed by poison.
	 */
	u32 arg = dcp->hw.iomfb_method_profile == DCP_IOMFB_METHODS_H17G;

	dcp_late_init_signal(dcp, false, &arg, boot_5, NULL);
#else
	dcp_late_init_signal(dcp, false, boot_5, NULL);
#endif
}

static void boot_4(struct apple_dcp *dcp, void *out, void *cookie)
{
#if DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0)
	/* macOS issues A033 between flush_supports_power and late_init_signal */
	dcp_dfb_query(dcp, false, boot_4b, NULL);
#else
	boot_4b(dcp, out, cookie);
#endif
}

static void boot_3(struct apple_dcp *dcp, void *out, void *cookie)
{
	u32 v_true = true;

	dcp_flush_supports_power(dcp, false, &v_true, boot_4, NULL);
}

static void boot_2(struct apple_dcp *dcp, void *out, void *cookie)
{
	dcp_setup_video_limits(dcp, false, boot_3, NULL);
}

#if DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0)
/*
 * macOS 25G83 runs this between A446 and A025, every boot:
 *   A035 -> A034 -> A103 -> A445 -> A104(w,h) -> A105
 * We were jumping straight from A446 to A025, so the firmware never got told
 * the default framebuffer's geometry and never finished its own DFB setup.
 */
static void boot_1_11(struct apple_dcp *dcp, void *out, void *cookie)
{
	dcp_commit_dfb_info(dcp, false, boot_2, NULL);
}

static void boot_1_10(struct apple_dcp *dcp, void *out, void *cookie)
{
	struct dcp_dfb_dimensions_h17p dim;

	/* A445 returns the dimensions that native macOS passes to A104.
	 * The H17G capture is2560x1664; the H17P capture is2408x1506.
	 */
	memcpy(&dim, out, sizeof(dim));
	if (!dim.width || !dim.height || dim.width > 16384 || dim.height > 16384) {
		struct dcp_channel *ch = &dcp->ch_cb;

		dev_err(dcp->dev, "invalid default framebuffer dimensions %ux%u\n",
			dim.width, dim.height);
		*(u8 *)ch->output[ch->depth - 1] = false;
		dcp_ack(dcp, DCP_CONTEXT_CB);
		return;
	}
	dev_info(dcp->dev, "default framebuffer dimensions %ux%u\n",
		 dim.width, dim.height);

	dcp_set_dfb_dimensions(dcp, false, &dim, boot_1_11, NULL);
}

static void boot_1_9(struct apple_dcp *dcp, void *out, void *cookie)
{
	u32 zero = 0;

	dcp_get_dfb_state(dcp, false, &zero, boot_1_10, NULL);
}

static void boot_1_8(struct apple_dcp *dcp, void *out, void *cookie)
{
	struct dcp_dfb_layout_req_h17p req = {};

	dcp_get_dfb_layout(dcp, false, &req, boot_1_9, NULL);
}

static void boot_1_7(struct apple_dcp *dcp, void *out, void *cookie)
{
	struct dcp_dfb_info_h17p info = {};

	dcp_get_dfb_info(dcp, false, &info, boot_1_8, NULL);
}

static void boot_1_6(struct apple_dcp *dcp, void *out, void *cookie)
{
	u64 zero = 0;

	dcp_get_dfb_compression_info(dcp, false, &zero, boot_1_7, NULL);
}
#endif

static void boot_1_5(struct apple_dcp *dcp, void *out, void *cookie)
{
#if DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0)
	dcp_create_default_fb(dcp, false, boot_1_6, NULL);
#else
	dcp_create_default_fb(dcp, false, boot_2, NULL);
#endif
}

#if DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0)
static void d000_done(struct apple_dcp *dcp, void *out, void *cookie)
{
	struct dcp_channel *ch = &dcp->ch_cb;
	u8 *succ = ch->output[ch->depth - 1];

	*succ = true;
	dcp_ack(dcp, DCP_CONTEXT_CB);
}

/*
 * macOS does not answer D000 straight away: it issues A033 first and only then
 * acks with 1.  Defer the ACK the same way.
 */
static bool dcpep_cb_d000_h17p(struct apple_dcp *dcp, int tag, void *out, void *in)
{
	trace_iomfb_callback(dcp, tag, __func__);
	dcp_dfb_query(dcp, false, d000_done, NULL);
	return false;
}
#endif

/* Use special function signature to defer the ACK */
static bool dcpep_cb_boot_1(struct apple_dcp *dcp, int tag, void *out, void *in)
{
	trace_iomfb_callback(dcp, tag, __func__);

#if DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0)
	/*
	 * Park the display before the firmware configures the pipe, not after.
	 *
	 * m1n1 stops the ASC, so our firmware boots with no surface (macOS
	 * adopts the live generation and always has one).  It enables the pipe
	 * anyway while finishing start_hardware_boot and fetches from a null
	 * base -- an address below the DART's window, so it cannot be mapped,
	 * only prevented.  The reject becomes an AXI error and NMIs the
	 * coprocessor before a modeset can ever run.  Powering the display down
	 * first keeps the pipe quiet until we drive a real mode.
	 */
	/*
	 * Do not park here.  macOS never powers the display down at all --
	 * start_hardware_boot has no blank or pipe-disable step, and its only
	 * A473 in a full capture is unklong=1 (power on).  Parking at the head
	 * of the boot chain stops the firmware booting outright.
	 */
#endif

	dcp_set_create_dfb(dcp, false, boot_1_5, NULL);
	return false;
}

static struct dcp_allocate_bandwidth_resp dcpep_cb_allocate_bandwidth(struct apple_dcp *dcp,
						struct dcp_allocate_bandwidth_req *req)
{
	return (struct dcp_allocate_bandwidth_resp){
		.unk1 = req->unk1,
		.unk2 = req->unk2,
		.ret = 1,
	};
}

static struct dcp_rt_bandwidth dcpep_cb_rt_bandwidth(struct apple_dcp *dcp)
{
	struct dcp_rt_bandwidth rt_bw = (struct dcp_rt_bandwidth){
			.reg_scratch = 0,
			.reg_doorbell = 0,
			.doorbell_bit = 0,
	};


	if (dcp->disp_bw_scratch_index) {
		u32 offset = dcp->disp_bw_scratch_offset;
		u32 index = dcp->disp_bw_scratch_index;
		rt_bw.reg_scratch = dcp->disp_registers[index]->start + offset;
	}

	if (dcp->disp_bw_doorbell_index) {
		u32 index = dcp->disp_bw_doorbell_index;
		rt_bw.reg_doorbell = dcp->disp_registers[index]->start;
		rt_bw.doorbell_bit = REG_DOORBELL_BIT(dcp->index);
		/*
		 * This is most certainly not padding. t8103-dcp crashes without
		 * setting this immediately during modeset on 12.3 and 13.5
		 * firmware.
		 */
		rt_bw.padding[3] = 0x4;
	}

	return rt_bw;
}

#if DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0)
/*
 * H17P's D006 is 0x54 in / 0x50 out, far wider than the 28-byte
 * frame_sync_props of older firmware, so the fields that matter sit past
 * anything the old struct could describe.
 *
 * macOS 25G83 answers it with zeros except three u32 at 0x24, 0x28 and 0x2c,
 * each 0x10000 -- 1.0 in 16.16 fixed point, the identity scale.  Answering all
 * zeros hands the pipe a zero scale, and it then programs a null surface base:
 * the display DART immediately faults at DVA 0, that reject becomes an AXI
 * error, and the firmware takes an NMI (apiodma_irq_AxiErr).
 */
#define DCP_FRAME_SYNC_IDENTITY	0x10000

struct dcp_set_frame_sync_props_req_h17p {
	u8 unk[0x54];
} __packed;

struct dcp_set_frame_sync_props_resp_h17p {
	u8 unk[0x50];
} __packed;

static struct dcp_set_frame_sync_props_resp_h17p
dcpep_cb_set_frame_sync_props_h17p(struct apple_dcp *dcp,
				   struct dcp_set_frame_sync_props_req_h17p *req)
{
	struct dcp_set_frame_sync_props_resp_h17p resp = {};
	static const u16 identity_at[] = { 0x24, 0x28, 0x2c };
	unsigned int i;
	u32 millinits;

	/*
	 * +0x04 is the brightness level applied by the frame this D006
	 * belongs to, in 16.16 nits (5957818 = 90.909 nits in the capture,
	 * +0x08 = 500.0 alongside it).  It is not a level readback: it is
	 * non-zero only on the frame whose swap carried a new level and reads
	 * 0 on every other frame, while the panel stays lit.  hb94 shows it
	 * plainly: "reports 60.000", then 4 ms later the firmware logs
	 * "send_brightness: dbv: 1259" (60 nits) again and D006 says 0.000,
	 * with PSTR at the 60-nit figure throughout.  Logging every change
	 * therefore printed "0 / 166 / 0 / 156" for each step of a fade.
	 * A real 0 on the panel is only ever one we asked for, and
	 * dcp_h17p_set_swap_brightness() logs that; keep this at debug and
	 * never let a 0 through as a level.
	 */
	millinits = (u32)(((u64)get_unaligned_le32(&req->unk[4]) * 1000) >> 16);
	if (millinits &&
	    millinits != dcp->brightness.h17p_reported_millinits) {
		dcp->brightness.h17p_reported_millinits = millinits;
		dev_dbg(dcp->dev, "DCP applied brightness %u.%03u nits\n",
			millinits / 1000, millinits % 1000);
	}

	for (i = 0; i < ARRAY_SIZE(identity_at); i++) {
		u32 v = DCP_FRAME_SYNC_IDENTITY;

		memcpy(&resp.unk[identity_at[i]], &v, sizeof(v));
	}

	return resp;
}

/*
 * Fill the H17P swap record's backlight block the way the one
 * brightness-changing A408 of the macOS 25G83 capture does (see the field
 * comments in iomfb_template.h).  The firmware converts the nits to a DAC
 * code with the ADT's iDAC/mA tables itself and drives the Melbourne PMU
 * over its own SPMI transport, and the backlight enable through its SMC
 * service -- nothing of that goes through the AP.
 */
static void dcp_h17p_set_swap_brightness(struct apple_dcp *dcp,
					 struct DCP_FW_NAME(dcp_swap) *swap,
					 u32 nits)
{
	static const u8 flags[6] = { 1, 1, 1, 0, 1, 0 };
	const u64 one = 0x3ff0000000000000ULL;	/* 1.0 */
	u64 max = dcp_nits_to_double_bits(dcp->brightness.maximum);
	int prev = dcp->brightness.h17p_sent_nits;

	/*
	 * A fade sends a level every frame; only the backlight going off,
	 * coming back, or the first level after power-on is worth a line.
	 */
	if (prev < 0 || !prev != !nits)
		dev_info(dcp->dev, "panel backlight %s (%u nits)\n",
			 nits ? "on" : "off", nits);
	else
		dev_dbg(dcp->dev, "panel backlight %u nits\n", nits);
	dcp->brightness.h17p_sent_nits = nits;

	swap->bl_h17p_update = 1;
	memcpy(swap->bl_h17p_flags, flags, sizeof(flags));
	memset(swap->bl_h17p_d, 0, sizeof(swap->bl_h17p_d));
	swap->bl_h17p_d[0] = cpu_to_le64(dcp_nits_to_double_bits(nits));
	/*
	 * [2]/[3] were 477.38 in the capture's brightness swap and 462.32 in
	 * the next one, both below the panel's 525-nit LmaxProduct and never
	 * zero while a level was in flight.  Whether that is an ambient
	 * reading or a ceiling, the panel maximum is the value that cannot
	 * clamp the request.
	 */
	swap->bl_h17p_d[2] = cpu_to_le64(max);
	swap->bl_h17p_d[3] = cpu_to_le64(max);
	swap->bl_h17p_d[4] = cpu_to_le64(one);
	swap->bl_h17p_d[17] = cpu_to_le64(one);
}
#endif

static struct dcp_set_frame_sync_props_resp
dcpep_cb_set_frame_sync_props(struct apple_dcp *dcp,
			      struct dcp_set_frame_sync_props_req *req)
{
	return (struct dcp_set_frame_sync_props_resp){};
}

/* Callback to get the current time as milliseconds since the UNIX epoch */
static u64 dcpep_cb_get_time(struct apple_dcp *dcp)
{
	return ktime_to_ms(ktime_get_real());
}

struct dcp_swap_cookie {
	struct kref refcount;
	struct completion done;
	u32 swap_id;
};

static void release_swap_cookie(struct kref *ref)
{
	struct dcp_swap_cookie *cookie;
	cookie = container_of(ref, struct dcp_swap_cookie, refcount);

        kfree(cookie);
}

#if DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0)
/*
 * Final H17P wire fix-ups, applied to the request in place just before every
 * A408.  The swap record is 0x588 bytes (see h17p_tail), so the surfaces keep
 * the common layout: plane_cnt at +3, format at +0xb, and surf_iova[0] at
 * +0xe38 of the 0xe9c request.  A record built as if the swap were 0x58a
 * bytes puts surf_iova two bytes late; the firmware then reads the DVA two
 * bytes early and faults on an address with no L1 entry.
 */
static void dcp_h17p_fix_swap_surfaces(struct apple_dcp *dcp)
{
	struct DCP_FW_NAME(dcp_swap_submit_req) *req = &DCP_FW_UNION(dcp->swap);
	unsigned int i;

	/*
	 * The swap's own flag words.  Our driver never sets either; macOS's
	 * captured swaps carry flags1 = 0x34865202 and flags2 = 4 in every
	 * record, at +0x40 and +0x48.
	 */
	req->swap.flags1 = DCP_SWAP_FLAGS1;
	req->swap.flags2 = DCP_SWAP_FLAGS2;

	/*
	 * The two counters inside the H17P gap that we never modelled: macOS
	 * carries 2 at +0x50 and 3 at +0x78 in every captured swap.  The gap
	 * starts at +0x50, so these are indices 0 and 0x28 of the blob.
	 */
	req->swap.h17p_pre_swap_id[0x00] = 2;
	req->swap.h17p_pre_swap_id[0x28] = 3;

	/*
	 * The H17P format table has no 8-bit RGB layout without alpha, so the
	 * fourth byte of XRGB/XBGR is always blended.  As on earlier DCPs,
	 * scan X formats out as premultiplied over the black background: with
	 * the GP alpha divide off (runtime property 0x49, sent after the first
	 * completed swap) that leaves the colour untouched for any X value,
	 * and the blend CRC of X = 00, 80 and ff frames equals the opaque one.
	 * plane.c sets is_premultiplied for X formats only; straight-alpha
	 * surfaces do not depend on 0x49.  Other firmware keeps straight alpha.
	 */
	for (i = 0; i < SWAP_SURFACES; i++) {
		req->surf[i].base.is_tearing_allowed = false;
		if (dcp->fw_compat != DCP_FIRMWARE_H17P)
			req->surf[i].base.is_premultiplied = false;
	}

	/* debug pokes, last so they land in the final layout */
	if (dcp_h17p_swap_poke[0]) {
		const char *c = dcp_h17p_swap_poke;
		u8 *r = (u8 *)req;
		size_t n = sizeof(*req);

		while (*c) {
			unsigned long o, v;
			char *e;

			o = simple_strtoul(c, &e, 16);
			if (*e != '=')
				break;
			v = simple_strtoul(e + 1, &e, 16);
			if (o < n)
				r[o] = v;
			if (*e != ',')
				break;
			c = e + 1;
		}
	}
}
#else
static void dcp_h17p_fix_swap_surfaces(struct apple_dcp *dcp) { }
#endif

static void dcp_swap_cleared(struct apple_dcp *dcp, void *data, void *cookie)
{
	struct DCP_FW_NAME(dcp_swap_submit_resp) *resp = data;

	if (cookie) {
		struct dcp_swap_cookie *info = cookie;
		complete(&info->done);
		kref_put(&info->refcount, release_swap_cookie);
	}

	if (resp->ret) {
		dev_err(dcp->dev, "swap_clear failed! status %u\n", resp->ret);
		dcp_unarm_swapped_out_fbs(dcp, DCP_FW_UNION(dcp->swap).swap.swap_id);
		dcp_drm_crtc_vblank(dcp->crtc);
		return;
	}

	dcp_release_swapped_out_fbs(dcp);
}

static void dcp_swap_clear_started(struct apple_dcp *dcp, void *data,
				   void *cookie)
{
	struct DCP_FW_NAME(dcp_swap_start_resp) *resp = data;

	if (resp->ret) {
		struct dcp_swap_cookie *info = cookie;

		dev_err(dcp->dev, "swap_clear start failed! status %u\n", resp->ret);
		if (info) {
			complete(&info->done);
			kref_put(&info->refcount, release_swap_cookie);
		}
		dcp_drm_crtc_vblank(dcp->crtc);
		return;
	}

	DCP_FW_UNION(dcp->swap).swap.swap_id = resp->swap_id;
	dcp_arm_swapped_out_fbs(dcp, resp->swap_id);

	if (cookie) {
		struct dcp_swap_cookie *info = cookie;
		info->swap_id = resp->swap_id;
	}

	dcp_h17p_fix_swap_surfaces(dcp);
	dcp_swap_submit(dcp, false, &DCP_FW_UNION(dcp->swap), dcp_swap_cleared, cookie);
}

static void dcp_on_final(struct apple_dcp *dcp, void *out, void *cookie)
{
	struct dcp_wait_cookie *wait = cookie;

	if (wait) {
		complete(&wait->done);
		kref_put(&wait->refcount, release_wait_cookie);
	}
}

static void dcp_on_set_power_state(struct apple_dcp *dcp, void *out, void *cookie)
{
	struct DCP_FW_NAME(dcp_set_power_state_req) req = {
		.unklong = 1,
	};

	dcp_set_power_state(dcp, DCP_POWER_OOB, &req, dcp_on_final, cookie);
}

static void dcp_on_set_parameter(struct apple_dcp *dcp, void *out, void *cookie)
{
	struct dcp_set_parameter_dcp param = {
		.param = IOMFBPARAM_ADAPTIVE_SYNC,
		.value = { 0 },
#if DCP_FW_VER >= DCP_FW_VERSION(13, 2, 0)
		.count = 3,
#else
		.count = 1,
#endif
	};

	dcp_set_parameter_dcp(dcp, false, &param, dcp_on_set_power_state, cookie);
}

void DCP_FW_NAME(iomfb_poweron)(struct apple_dcp *dcp)
{
	struct dcp_wait_cookie *cookie;
	int ret;
	u32 handle;
	dev_info(dcp->dev, "dcp_poweron() starting\n");

#if DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0)
	/*
	 * Resuming from the H17P soft power-off (see iomfb_poweroff): the
	 * pipe never went down, so there is no power state to restore; the
	 * backlight gets the level from before the power-off back, and the
	 * next swap applies it.
	 */
	if (dcp->brightness.soft_off) {
		dcp_backlight_soft_on(dcp);
		dev_info(dcp->dev, "dcp_poweron() from soft power-off: pipe was never down, backlight back to %d nits\n",
			 dcp->brightness.nits);
		return;
	}
#endif

	/*
	 * A full power-on may reset runtime properties: send 0x49 again, after
	 * the first swap completes (A352 is not answered during client open).
	 */
	dcp->h17p_alpha_div_off = false;
	dcp->h17p_swapped_since_on = false;

	cookie = kzalloc(sizeof(*cookie), GFP_KERNEL);
	if (!cookie)
		return;

	init_completion(&cookie->done);
	kref_init(&cookie->refcount);
	/* increase refcount to ensure the receiver has a reference */
	kref_get(&cookie->refcount);

	if (dcp->main_display) {
#if DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0)
		/*
		 * macOS never repeats A411 at power-on -- it issues it once, in
		 * the client-open sequence, and goes straight to the power
		 * state from here.  Repeating it after the client is open is a
		 * call the firmware simply stops answering, which wedges the
		 * command channel and with it every later swap.
		 */
		dcp_on_set_power_state(dcp, NULL, cookie);
#else
		handle = 0;
		dcp_set_display_device(dcp, false, &handle, dcp_on_set_power_state,
				       cookie);
#endif
	} else {
		handle = 2;
		dcp_set_display_device(dcp, false, &handle,
				       dcp_on_set_parameter, cookie);
	}
	ret = wait_for_completion_timeout(&cookie->done, msecs_to_jiffies(10000));

	if (ret == 0) {
		dev_warn(dcp->dev, "wait for power timed out, connector will be broken\n");
	} else if (ret > 0) {
		int msecs = jiffies_to_msecs(ret);
		if (msecs > 6000)
			dev_info(dcp->dev, "dcp_set_power_state_req returned, %d ms remaining\n", msecs);
		else
			dev_warn(dcp->dev, "dcp_set_power_state_req returned, %d ms remaining\n", msecs);
	} else {
		drm_connector_set_link_status_property(&dcp->connector->base,
						       DRM_MODE_LINK_STATUS_BAD);
		dev_warn(dcp->dev, "wait for completion error: %d\n", ret);
	}

	kref_put(&cookie->refcount, release_wait_cookie);;

#if DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0)
	/*
	 * H17P never publishes IOMFB_PROPERTY_NITS, so nothing registers the
	 * backlight device for us, and the firmware reports 0 nits in D006
	 * until the AP has set a brightness (macOS's DCP does the same: the
	 * first D006 records of the capture carry 0 while the panel is lit at
	 * iBoot's level).  Seed the level once and register the device; the
	 * forced update below applies it with the first swap, so what sysfs
	 * shows is what the panel does.
	 */
	if (dcp_has_panel(dcp) && dcp->brightness.maximum > 0 &&
	    !dcp->brightness.bl_dev) {
		if (!dcp->brightness.nits)
			dcp->brightness.nits = dcp_h17p_initial_nits(dcp);
		schedule_work(&dcp->bl_register_wq);
	}
#endif

	/* Force a brightness update after poweron, to restore the brightness */
	dcp->brightness.update = true;
}

static void complete_set_powerstate(struct apple_dcp *dcp, void *out,
				    void *cookie)
{
	struct dcp_wait_cookie *wait = cookie;

	if (wait) {
		complete(&wait->done);
		kref_put(&wait->refcount, release_wait_cookie);
	}
}

static void last_client_closed_poff(struct apple_dcp *dcp, void *out, void *cookie)
{
	struct DCP_FW_NAME(dcp_set_power_state_req) power_req = {
		.unklong = 0,
	};
	dcp_set_power_state(dcp, DCP_POWER_OOB, &power_req, complete_set_powerstate,
			    cookie);
}

static void aborted_swaps_dcp_poff(struct apple_dcp *dcp, void *out, void *cookie)
{
	struct iomfb_last_client_close_req last_client_req = {};
	iomfb_last_client_close(dcp, false, &last_client_req,
				last_client_closed_poff, cookie);
}

void DCP_FW_NAME(iomfb_poweroff)(struct apple_dcp *dcp)
{
	int ret, swap_id;
	struct iomfb_abort_swaps_dcp_req abort_req = {
		.client = {
			.flag2 = 1,
		},
	};
	struct dcp_swap_cookie *cookie;
	struct dcp_wait_cookie *poff_cookie;
	struct DCP_FW_NAME(dcp_swap_start_req) swap_req = { 0 };
	struct DCP_FW_NAME(dcp_swap_submit_req) *swap = &DCP_FW_UNION(dcp->swap);

#if DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0)
	/*
	 * The pre-H17P power-off -- a swap that unbinds every surface, then
	 * A473 power 0 -- kills this firmware: with no surface bound the pipe
	 * fetches from a null base, the display DART rejects it and the
	 * coprocessor panics ("panic(app): app NMI, pidx: 1", crashlog on
	 * endpoint 1, then A466/A406 never answered; J700 hb94, sway
	 * "output * power off").  Cutting the backlight supply from the AP
	 * (SMC pmBL) while the pipe runs is just as fatal ("IOMFB
	 * int_handler_gated: failure: axi_rd_err [0x40300000]" five
	 * milliseconds later, hb95), and so is re-presenting the last frame
	 * at 0 nits from here, outside a live commit (the firmware answers
	 * "send_brightness: dbv: 0" and takes the same axi_rd_err at
	 * +0x34 14 ms later, hb99) -- while the very same 0-nits level
	 * requested through the backlight device on a live pipe is clean
	 * (hb94: 2.27 W gone at 210 nits, no send_brightness, no crash).
	 * Only a reboot brings the panel back from any of the three.
	 *
	 * So a DPMS-off does nothing to the coprocessor's power state: the
	 * pipe keeps scanning out the last frame, and the backlight stands in
	 * for the power-off (dcp_backlight_soft_off): it drops to blank_nits
	 * (0 by default) and DPMS on restores the level asked for last.  The
	 * level goes out the way the backlight device sends every level, in
	 * a swap on the live pipe -- a re-present of the surface already on
	 * the panel (iomfb_present_brightness), issued from the backlight
	 * worker once this disable commit has completed.  Unlike the hb99
	 * swap, which ran from here while the disable commit was releasing
	 * the scanned-out buffer (hb100's "read PTE invalid" across the old
	 * surface), that buffer is now retained until a real swap replaces
	 * it, and the re-present does not arm its release.
	 * appledrm.blank_nits=-1 keeps the panel lit through DPMS off, as
	 * before.
	 */
	if (dcp_has_panel(dcp)) {
		dcp_backlight_soft_off(dcp);
		dev_info(dcp->dev,
			 "DPMS off: keeping the H17P pipe up (the firmware panics on every off path), backlight %s\n",
			 dcp->brightness.blanked ? "to blank_nits" : "left at the current level");
		return;
	}
	/*
	 * Physical Type-C unplug is quiesced before deasserting HPD. Keep the
	 * last scanout surface pinned and send only the native OOB power call;
	 * a clear-surface swap is neither needed nor safe during this teardown.
	 */
	if (READ_ONCE(dcp->typec_unplugged))
		goto set_power_state;
#endif

	cookie = kzalloc(sizeof(*cookie), GFP_KERNEL);
	if (!cookie)
		return;
	init_completion(&cookie->done);
	kref_init(&cookie->refcount);
	/* increase refcount to ensure the receiver has a reference */
	kref_get(&cookie->refcount);

	// clear surfaces
	memset(swap, 0, sizeof(*swap));

	swap->swap.swap_enabled =
		swap->swap.swap_completed = IOMFB_SET_BACKGROUND | 0x7;
	swap->swap.bg_color = 0xFF000000;

	/*
	 * Turn off the backlight. This matters because the DCP's idea of
	 * backlight brightness gets desynced after a power change, and it
	 * needs to be told it's going to turn off so it will consider the
	 * subsequent update on poweron an actual change and restore the
	 * brightness.
	 */
	if (dcp_has_panel(dcp)) {
#if DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0)
		dcp_h17p_set_swap_brightness(dcp, &swap->swap, 0);
#else
		swap->swap.bl_unk = 1;
		swap->swap.bl_value = 0;
		swap->swap.bl_power = 0;
#endif
	}

	/* Null all surfaces */
	for (int l = 0; l < SWAP_SURFACES; l++)
		swap->surf_null[l] = true;
#if DCP_FW_VER >= DCP_FW_VERSION(13, 2, 0)
#if DCP_FW_VER < DCP_FW_VERSION(26, 0, 0)
	for (int l = 0; l < ARRAY_SIZE(swap->surf2_null); l++)
		swap->surf2_null[l] = true;
#endif
	swap->unkU32Ptr_null = true;
	swap->unkU32out_null = true;
#endif

	dcp_swap_start(dcp, false, &swap_req, dcp_swap_clear_started, cookie);

	ret = wait_for_completion_timeout(&cookie->done, msecs_to_jiffies(50));
	swap_id = cookie->swap_id;
	kref_put(&cookie->refcount, release_swap_cookie);
	if (ret <= 0) {
		/*
		 * The DCP did not acknowledge the poweroff clear swap.
		 *
		 * Do not latch ->crashed here: dcp_crtc_atomic_check() bails
		 * out on that flag, so every subsequent atomic commit for this
		 * device is rejected with -EINVAL, permanently. Nothing ever
		 * clears it, so the display stays dark until reboot.
		 *
		 * This is not a crash. The DCP keeps working afterwards: it
		 * renegotiates the link, publishes its mode list and asserts
		 * HPD. ->crashed belongs to dcp_rtk_crashed(), the RTKit crash
		 * callback, which also logs.
		 *
		 * Warn instead, so the timeout is at least visible.
		 */
		dev_warn(dcp->dev,
			 "%s: timed out waiting for the poweroff clear swap; continuing\n",
			 __func__);
		return;
	}

	dev_dbg(dcp->dev, "%s: clear swap submitted: %u\n", __func__, swap_id);

#if DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0)
set_power_state:
#endif
	poff_cookie = kzalloc(sizeof(*poff_cookie), GFP_KERNEL);
	if (!poff_cookie)
		return;
	init_completion(&poff_cookie->done);
	kref_init(&poff_cookie->refcount);
	/* increase refcount to ensure the receiver has a reference */
	kref_get(&poff_cookie->refcount);

	/*
	 * The H17P tags for abort_swaps and last_client_close are unknown. The
	 * old placeholders send A478 and A466; A478 is a different live method,
	 * while A466 makes the firmware NMI. A473 is the captured H17P power
	 * method and must run out-of-band, so go to it directly after the clear
	 * swap has retired.
	 */
	if (DCP_FW_VERSION(26, 0, 0) <= DCP_FW_VER)
		last_client_closed_poff(dcp, NULL, poff_cookie);
	else
		iomfb_abort_swaps_dcp(dcp, false, &abort_req,
					aborted_swaps_dcp_poff, poff_cookie);
	ret = wait_for_completion_timeout(&poff_cookie->done,
					  msecs_to_jiffies(1000));

	if (ret == 0)
		dev_warn(dcp->dev, "setPowerState(0) timeout %u ms\n", 1000);
	else if (ret > 0)
		dev_dbg(dcp->dev,
			"setPowerState(0) finished with %d ms to spare",
			jiffies_to_msecs(ret));

	kref_put(&poff_cookie->refcount, release_wait_cookie);

	dev_info(dcp->dev, "dcp_poweroff() done\n");
}

static void last_client_closed_sleep(struct apple_dcp *dcp, void *out, void *cookie)
{
	struct DCP_FW_NAME(dcp_set_power_state_req) power_req = {
		.unklong = 0,
	};
	dcp_set_power_state(dcp, DCP_POWER_OOB, &power_req, complete_set_powerstate, cookie);
}

static void aborted_swaps_dcp_sleep(struct apple_dcp *dcp, void *out, void *cookie)
{
	struct iomfb_last_client_close_req req = { 0 };
	iomfb_last_client_close(dcp, false, &req, last_client_closed_sleep, cookie);
}

void DCP_FW_NAME(iomfb_sleep)(struct apple_dcp *dcp)
{
	int ret;
	struct iomfb_abort_swaps_dcp_req req = {
		.client = {
			.flag2 = 1,
		},
	};

	struct dcp_wait_cookie *cookie;

	cookie = kzalloc(sizeof(*cookie), GFP_KERNEL);
	if (!cookie)
		return;
	init_completion(&cookie->done);
	kref_init(&cookie->refcount);
	/* increase refcount to ensure the receiver has a reference */
	kref_get(&cookie->refcount);

	if (DCP_FW_VERSION(26, 0, 0) <= DCP_FW_VER)
		last_client_closed_sleep(dcp, NULL, cookie);
	else
		iomfb_abort_swaps_dcp(dcp, false, &req,
					aborted_swaps_dcp_sleep, cookie);
	ret = wait_for_completion_timeout(&cookie->done,
					  msecs_to_jiffies(1000));

	if (ret == 0)
		dev_warn(dcp->dev, "setDCPPower(0) timeout %u ms\n", 1000);

	kref_put(&cookie->refcount, release_wait_cookie);
	dev_info(dcp->dev, "dcp_sleep() done\n");
}

static void dcpep_cb_hotplug(struct apple_dcp *dcp, u64 *connected)
{
	struct apple_connector *connector = dcp->connector;

	/* DCP issues hotplug_gated callbacks after SetPowerState() calls on
	 * devices with display (macbooks, imacs). This must not result in
	 * connector state changes on DRM side. Some applications won't enable
	 * a CRTC with a connector in disconnected state. Weston after DPMS off
	 * is one example. dcp_is_main_display() returns true on devices with
	 * integrated display. Ignore the hotplug_gated() callbacks there.
	 */
	if (dcp->main_display)
		return;

	/*
	 * Same for the unplug a Type-C output reports after its CRTC was
	 * powered off with the cable still attached (see dcp_poweroff()).
	 */
	if (!(*connected) && READ_ONCE(dcp->typec_crtc_off) &&
	    READ_ONCE(dcp->typec_cable_connected)) {
		dev_dbg(dcp->dev, "cb_hotplug() ignoring unplug of powered-off Type-C output\n");
		dcp->valid_mode = false;
		schedule_delayed_work(&dcp->vblank_wq, 0);
		return;
	}

	if (dcp->during_modeset) {
		/*
		 * Remember it rather than dropping it.  Resume re-runs the
		 * modeset and the firmware reports the display back while that
		 * is still in flight; discarding it leaves the connector marked
		 * disconnected forever, with no further event to correct it.
		 */
		dev_info(dcp->dev,
			 "cb_hotplug() deferred during modeset connected:%llu\n",
			 *connected);
		dcp->pending_hotplug = true;
		dcp->pending_hotplug_connected = !!(*connected);
		return;
	}

	dev_info(dcp->dev, "cb_hotplug() connected:%llu, valid_mode:%d\n",
		 *connected, dcp->valid_mode);

	/* Hotplug invalidates mode. DRM doesn't always handle this. */
	if (!(*connected)) {
		dcp->valid_mode = false;
		/* after unplug swap will not complete until the next
		 * set_digital_out_mode */
		schedule_delayed_work(&dcp->vblank_wq, msecs_to_jiffies(5));
	}

	if (connector && connector->connected != !!(*connected)) {
		connector->connected = !!(*connected);
		dcp->valid_mode = false;
		schedule_work(&connector->hotplug_wq);
	}
}

static void
dcpep_cb_swap_complete_intent_gated(struct apple_dcp *dcp,
				    struct dcp_swap_complete_intent_gated *info)
{
	trace_iomfb_swap_complete_intent_gated(dcp, info->swap_id,
		info->width, info->height);
}

static void
dcpep_cb_abort_swap_ap_gated(struct apple_dcp *dcp, u32 *swap_id)
{
	trace_iomfb_abort_swap_ap_gated(dcp, *swap_id);
}

static struct dcpep_get_tiling_state_resp
dcpep_cb_get_tiling_state(struct apple_dcp *dcp,
			  struct dcpep_get_tiling_state_req *req)
{
	return (struct dcpep_get_tiling_state_resp){
		.value = 0,
		.ret = 1,
	};
}

static u8 dcpep_cb_create_backlight_service(struct apple_dcp *dcp)
{
	/*
	 * dcp_has_panel() only becomes true once the panel dimensions have been
	 * parsed out of the EDID, which happens long after this callback: the
	 * firmware asks during start_ios_gated, before any mode is known.  An
	 * internal panel always has a backlight, so answer for the connector.
	 * macOS 25G83 answers true here in both captured boots; answering false
	 * makes the firmware log "Could not find Backlight service".
	 */
	return dcp_has_panel(dcp) ||
	       dcp->connector_type == DRM_MODE_CONNECTOR_eDP;
}

TRAMPOLINE_VOID(trampoline_nop, dcpep_cb_nop);
TRAMPOLINE_OUT(trampoline_true, dcpep_cb_true, u8);
TRAMPOLINE_OUT(trampoline_false, dcpep_cb_false, u8);
TRAMPOLINE_OUT(trampoline_zero, dcpep_cb_zero, u32);
TRAMPOLINE_IN(trampoline_swap_complete, dcpep_cb_swap_complete,
	      struct DCP_FW_NAME(dc_swap_complete_resp));
TRAMPOLINE_INOUT(trampoline_get_uint_prop, dcpep_cb_get_uint_prop,
		 struct dcp_get_uint_prop_req, struct dcp_get_uint_prop_resp);
TRAMPOLINE_IN(trampoline_set_fx_prop, iomfbep_cb_set_fx_prop,
	      struct iomfb_set_fx_prop_req)
TRAMPOLINE_INOUT(trampoline_map_piodma, dcpep_cb_map_piodma,
		 struct dcp_map_buf_req, struct DCP_FW_NAME(dcp_map_buf_resp));
TRAMPOLINE_IN(trampoline_unmap_piodma, dcpep_cb_unmap_piodma,
	      struct dcp_unmap_buf_resp);
TRAMPOLINE_INOUT(trampoline_sr_set_property_int, iomfbep_cb_sr_set_property_int,
		 struct iomfb_sr_set_property_int_req, u8);
TRAMPOLINE_INOUT(trampoline_allocate_buffer, dcpep_cb_allocate_buffer,
		 struct dcp_allocate_buffer_req,
		 struct dcp_allocate_buffer_resp);
TRAMPOLINE_INOUT(trampoline_map_physical, dcpep_cb_map_physical,
		 struct dcp_map_physical_req, struct dcp_map_physical_resp);
TRAMPOLINE_INOUT(trampoline_release_mem_desc, dcpep_cb_release_mem_desc, u32,
		 u8);
TRAMPOLINE_INOUT(trampoline_map_reg, dcpep_cb_map_reg,
		 struct DCP_FW_NAME(dcp_map_reg_req),
		 struct DCP_FW_NAME(dcp_map_reg_resp));
TRAMPOLINE_INOUT(trampoline_read_edt_data, dcpep_cb_read_edt_data,
		 struct dcp_read_edt_data_req, struct dcp_read_edt_data_resp);
TRAMPOLINE_INOUT(trampoline_prop_start, dcpep_cb_prop_start, u32, u8);
TRAMPOLINE_INOUT(trampoline_prop_chunk, dcpep_cb_prop_chunk,
		 struct dcp_set_dcpav_prop_chunk_req, u8);
TRAMPOLINE_INOUT(trampoline_prop_end, dcpep_cb_prop_end,
		 struct dcp_set_dcpav_prop_end_req, u8);
TRAMPOLINE_INOUT(trampoline_allocate_bandwidth, dcpep_cb_allocate_bandwidth,
	       struct dcp_allocate_bandwidth_req, struct dcp_allocate_bandwidth_resp);
TRAMPOLINE_OUT(trampoline_rt_bandwidth, dcpep_cb_rt_bandwidth,
	       struct dcp_rt_bandwidth);
TRAMPOLINE_INOUT(trampoline_set_frame_sync_props, dcpep_cb_set_frame_sync_props,
	       struct dcp_set_frame_sync_props_req,
	       struct dcp_set_frame_sync_props_resp);
#if DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0)
TRAMPOLINE_INOUT(trampoline_set_frame_sync_props_h17p,
	       dcpep_cb_set_frame_sync_props_h17p,
	       struct dcp_set_frame_sync_props_req_h17p,
	       struct dcp_set_frame_sync_props_resp_h17p);
#endif
TRAMPOLINE_OUT(trampoline_get_frequency, dcpep_cb_get_frequency, u64);
TRAMPOLINE_OUT(trampoline_get_time, dcpep_cb_get_time, u64);
TRAMPOLINE_IN(trampoline_hotplug, dcpep_cb_hotplug, u64);
TRAMPOLINE_IN(trampoline_swap_complete_intent_gated,
	      dcpep_cb_swap_complete_intent_gated,
	      struct dcp_swap_complete_intent_gated);
TRAMPOLINE_IN(trampoline_abort_swap_ap_gated, dcpep_cb_abort_swap_ap_gated, u32);
TRAMPOLINE_IN(trampoline_enable_backlight_message_ap_gated,
	      iomfbep_cb_enable_backlight_message_ap_gated, u8);
TRAMPOLINE_IN(trampoline_pr_publish, iomfb_cb_pr_publish,
	      struct iomfb_property);
TRAMPOLINE_INOUT(trampoline_get_tiling_state, dcpep_cb_get_tiling_state,
		 struct dcpep_get_tiling_state_req, struct dcpep_get_tiling_state_resp);
TRAMPOLINE_OUT(trampoline_create_backlight_service, dcpep_cb_create_backlight_service, u8);

/*
 * Callback for swap requests. If a swap failed, we'll never get a swap
 * complete event so we need to fake a vblank event early to avoid a hang.
 */

static void dcp_swapped(struct apple_dcp *dcp, void *data, void *cookie)
{
	struct DCP_FW_NAME(dcp_swap_submit_resp) *resp = data;

	if (resp->ret) {
		dev_err(dcp->dev, "swap failed! status %u\n", resp->ret);
		dcp_unarm_swapped_out_fbs(dcp, DCP_FW_UNION(dcp->swap).swap.swap_id);
		dcp_drm_crtc_vblank(dcp->crtc);
		return;
	}
#if DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0)
	/* Brightness must replay an accepted surface, never a rejected update. */
	memcpy(&dcp->h17p_last_swap, &dcp->h17p_pending_swap,
	       sizeof(dcp->h17p_last_swap));
	dcp->h17p_have_last_swap = true;
#endif
	dcp->swap_start = ktime_get();
	dcp->swap_submit_timestamp = arch_timer_read_counter();
	if (dcp_swap_trace)
		dev_info(dcp->dev, "swap-trace: submitted id=%u t=%lld us\n",
			 DCP_FW_UNION(dcp->swap).swap.swap_id,
			 ktime_to_us(dcp->swap_start));

	dcp_release_swapped_out_fbs(dcp);
}

static void dcp_swap_started(struct apple_dcp *dcp, void *data, void *cookie)
{
	struct DCP_FW_NAME(dcp_swap_start_resp) *resp = data;

	if (resp->ret) {
		dev_err(dcp->dev, "swap_start failed! status %u\n", resp->ret);
		dcp_drm_crtc_vblank(dcp->crtc);
		return;
	}

	DCP_FW_UNION(dcp->swap).swap.swap_id = resp->swap_id;
	dcp_arm_swapped_out_fbs(dcp, resp->swap_id);

	trace_iomfb_swap_submit(dcp, resp->swap_id);
	if (dcp_swap_trace)
		dev_info(dcp->dev, "swap-trace: started id=%u t=%lld us surf0=%#llx\n",
			 resp->swap_id, ktime_to_us(ktime_get()),
			 (unsigned long long)DCP_FW_UNION(dcp->swap).surf_iova[0]);
	dcp_h17p_fix_swap_surfaces(dcp);
	dcp_swap_submit(dcp, false, &DCP_FW_UNION(dcp->swap), dcp_swapped, NULL);
}

/* Helpers to modeset and swap, used to flush */
static void do_swap_start(struct apple_dcp *dcp)
{
	struct DCP_FW_NAME(dcp_swap_start_req) start_req = { 0 };

	if (dcp->connector && dcp->connector->connected)
		dcp_swap_start(dcp, false, &start_req, dcp_swap_started, NULL);
	else
		dcp_drm_crtc_vblank(dcp->crtc);
}

#if DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0)
static void dcp_diag_then_swap(struct apple_dcp *dcp, void *out, void *cookie)
{
	dcp_callback_t res = dcp->diag_res;

	dcp->diag_res = NULL;
	if (res)
		res(dcp, out, dcp->diag_cookie);
	do_swap_start(dcp);
}

/*
 * One pending runtime property or diagnostic call, issued from the flush
 * path ahead of the swap and chained on its reply.  Calls pushed from the
 * swap_complete callback raced the next flush: dcp_call_context() then saw
 * the command channel busy and sent the swap_start (A406) as a nested call,
 * and the firmware panicked ("rpc_callee_gated: payload size mismatch:
 * 36 != 16", an A406 record against an in-flight A106).
 */
static bool dcp_issue_pre_swap_call(struct apple_dcp *dcp)
{
	dcp->diag_res = NULL;
	if (dcp->fw_compat == DCP_FIRMWARE_H17P && !dcp->h17p_alpha_div_off &&
	    dcp->h17p_swapped_since_on) {
		u64 rq = DCP_H17P_RTP_ENABLE_GP_ALPHA_DIV; /* {prop, value 0} */

		dcp->h17p_alpha_div_off = true;
		dcp->diag_res = res_h17p_alpha_div;
		dcp_pipe_set_352(dcp, false, &rq, dcp_diag_then_swap, NULL);
	} else if (dcp_rtp_request[0]) {
		dcp_issue_rtp_request(dcp, dcp_diag_then_swap);
	} else if (dcp_dither_request >= 0) {
		u32 v = dcp_dither_request ? 1 : 0;

		dcp_dither_request = -1;
		dcp->diag_res = res_dither;
		dcp->diag_cookie = (void *)(uintptr_t)v;
		dcp_enable_disable_dithering(dcp, false, &v, dcp_diag_then_swap, NULL);
	} else if (dcp_blend_crc_reads > 0 && dcp->have_swap_complete) {
		dcp_blend_crc_reads--;
		dcp->diag_res = res_blend_crc;
		dcp->diag_cookie = (void *)(uintptr_t)dcp->last_swap_id;
		dcp_read_blend_crc(dcp, false, dcp_diag_then_swap, NULL);
	} else if (dcp_pipe_crc_reads > 0 && dcp->have_swap_complete) {
		struct dcp_read_crc_req rq = { .index = dcp_pipe_crc_idx, .arg = 0 };

		dcp_pipe_crc_reads--;
		dcp->diag_res = res_pipe_crc;
		dcp->diag_cookie = (void *)(uintptr_t)dcp->last_swap_id;
		dcp_read_crc(dcp, false, &rq, dcp_diag_then_swap, NULL);
	} else {
		return false;
	}
	return true;
}
#else
static bool dcp_issue_pre_swap_call(struct apple_dcp *dcp)
{
	return false;
}
#endif

static void do_swap(struct apple_dcp *dcp, void *data, void *cookie)
{
	if (dcp->connector && dcp->connector->connected &&
	    dcp_issue_pre_swap_call(dcp))
		return;
	do_swap_start(dcp);
}

#if DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0)
/*
 * macOS issues A428 between set_matrix and the first swap: CMD, in 4, out 4,
 * payload 00 00 01 00 -- 0x10000, 1.0 in 16.16, the same identity value it
 * answers D006 with.  We went straight from A423 to A406.
 */
static void dcp_pre_swap_a428(struct apple_dcp *dcp, void *out, void *cookie)
{
	u32 identity = 0x10000;

	dcp_pipe_set_428(dcp, false, &identity, do_swap, cookie);
}
#endif

static void dcp_swap_set_timestamps(struct apple_dcp *dcp,
				    struct DCP_FW_NAME(dcp_swap_submit_req) *req)
{
	u64 submit_timestamp, timestamp;

	if (!dcp->use_timestamps && !dcp->vrr_enabled)
		return;

	submit_timestamp = dcp->swap_submit_timestamp;
	timestamp = arch_timer_read_counter();

	/*
	 * IOMobileFramebuffer uses Mach continuous-time values here. On
	 * Apple Silicon that is the ARM architectural counter. Empirical
	 * testing shows that using the current submission time together
	 * with the previous accepted swap makes DCP follow swap pacing.
	 */
	if (!submit_timestamp)
		submit_timestamp = timestamp;

	/* Firmware 12.x/13.x requires timestamp types 1, 2, and 7. */
	req->swap.timestamp[0] = timestamp;
	req->swap.timestamp[1] = submit_timestamp;
	req->swap.timestamp[6] = timestamp;

	trace_iomfb_vrr_timestamps(dcp, dcp->vrr_enabled, timestamp,
				   submit_timestamp);
}

#if DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0)
/*
 * A brightness change on H17P travels inside a swap, and while the CRTC is
 * off nothing flushes one: the ACTIVE_ONLY plane pass of commit_tail_rpm()
 * skips an inactive CRTC, so the backlight device's own commit
 * (drm_crtc_set_brightness()) never reaches iomfb_flush() and the level sat
 * in sysfs while the panel stayed lit at 5 W (2026-09-28 daily, GNOME idle
 * blank).  Re-present the swap that put the current surface on the panel
 * instead, with the new level in its backlight block.
 *
 * That surface is safe to hand over again: its framebuffer is either still
 * bound to the plane or held by dcp_retain_unbound_fbs() since the commit
 * that unbound it, and it stays held because this swap does not arm the
 * retained entries -- it displaces nothing, and arming them would release
 * the very buffer it shows once it completed (the hb100 DART fault).  The
 * next real swap arms them as usual.
 *
 * The caller holds the CRTC lock, has waited for the last commit, and has
 * checked that the command channel is idle.
 */
static void dcp_bl_swapped(struct apple_dcp *dcp, void *data, void *cookie)
{
	struct DCP_FW_NAME(dcp_swap_submit_resp) *resp = data;
	struct dcp_swap_cookie *info = cookie;

	if (resp->ret)
		dev_err(dcp->dev, "brightness swap failed! status %u\n",
			resp->ret);

	complete(&info->done);
	kref_put(&info->refcount, release_swap_cookie);
}

static void dcp_bl_swap_started(struct apple_dcp *dcp, void *data,
				void *cookie)
{
	struct DCP_FW_NAME(dcp_swap_start_resp) *resp = data;
	struct dcp_swap_cookie *info = cookie;

	if (resp->ret) {
		dev_err(dcp->dev, "brightness swap_start failed! status %u\n", resp->ret);
		complete(&info->done);
		kref_put(&info->refcount, release_swap_cookie);
		return;
	}

	DCP_FW_UNION(dcp->swap).swap.swap_id = resp->swap_id;
	info->swap_id = resp->swap_id;
	/* deliberately no dcp_arm_swapped_out_fbs(), see above */
	dcp_h17p_fix_swap_surfaces(dcp);
	dcp_swap_submit(dcp, false, &DCP_FW_UNION(dcp->swap), dcp_bl_swapped,
			cookie);
}

int DCP_FW_NAME(iomfb_present_brightness)(struct apple_dcp *dcp)
{
	struct DCP_FW_NAME(dcp_swap_submit_req) *req = &DCP_FW_UNION(dcp->swap);
	struct DCP_FW_NAME(dcp_swap_start_req) start_req = { 0 };
	struct dcp_swap_cookie *cookie;
	int ret;

	if (!dcp_has_panel(dcp) || !dcp->h17p_have_last_swap)
		return -ENODEV;

	cookie = kzalloc(sizeof(*cookie), GFP_KERNEL);
	if (!cookie)
		return -ENOMEM;
	init_completion(&cookie->done);
	kref_init(&cookie->refcount);
	/* increase refcount to ensure the receiver has a reference */
	kref_get(&cookie->refcount);

	memcpy(req, &dcp->h17p_last_swap, sizeof(*req));
	/* the surface is already there: no background fill, no clear */
	req->swap.swap_enabled &= ~IOMFB_SET_BACKGROUND;
	req->swap.bg_color = 0;
	req->clear = 0;
	dcp_swap_set_timestamps(dcp, req);
	req->swap.swap_completed = req->swap.swap_enabled;
	dcp_h17p_set_swap_brightness(dcp, &req->swap, dcp->brightness.nits);
	dcp->brightness.update = false;

	dcp_swap_start(dcp, false, &start_req, dcp_bl_swap_started, cookie);

	ret = wait_for_completion_timeout(&cookie->done, msecs_to_jiffies(100));
	kref_put(&cookie->refcount, release_swap_cookie);
	if (ret <= 0) {
		dev_warn(dcp->dev,
			 "brightness swap at %d nits not acknowledged\n",
			 dcp->brightness.nits);
		return -ETIMEDOUT;
	}

	return 0;
}
#endif

static void complete_set_digital_out_mode(struct apple_dcp *dcp, void *data,
					  void *cookie)
{
	struct dcp_wait_cookie *wait = cookie;

	if (wait) {
		complete(&wait->done);
		kref_put(&wait->refcount, release_wait_cookie);
	}
}

/* DCP applies Adaptive Sync changes when the display mode is reselected. */
static void dcp_on_set_adaptive_sync(struct apple_dcp *dcp, void *out,
				     void *cookie)
{
	dcp_set_digital_out_mode(dcp, false, &dcp->mode,
				 complete_set_digital_out_mode, cookie);
}

static void dcp_set_adaptive_sync(struct apple_dcp *dcp, u32 min_vrr,
				  void *cookie)
{
	struct dcp_set_parameter_dcp param = {
		.param = IOMFBPARAM_ADAPTIVE_SYNC,
		.value = {
			min_vrr, /* minRR, 16.16 fixed-point Hz */
			0,       /* mediaTargetRate */
			0,       /* fractional rate */
		},
#if DCP_FW_VER >= DCP_FW_VERSION(13, 2, 0)
		.count = 3,
#else
		.count = 1,
#endif
	};

	dcp_set_parameter_dcp(dcp, false, &param, dcp_on_set_adaptive_sync,
			      cookie);
}

int DCP_FW_NAME(iomfb_modeset)(struct apple_dcp *dcp,
			       struct drm_crtc_state *crtc_state)
{
	struct dcp_display_mode *mode;
	struct dcp_wait_cookie *cookie;
	struct dcp_color_mode *cmode = NULL;
	int ret;

	mode = lookup_mode(dcp, &crtc_state->mode);
	if (!mode) {
		dev_err(dcp->dev, "no match for " DRM_MODE_FMT "\n",
			DRM_MODE_ARG(&crtc_state->mode));
		return -EIO;
	}

	dev_info(dcp->dev,
		 "set_digital_out_mode(color:%d timing:%d) " DRM_MODE_FMT "\n",
		 mode->color_mode_id, mode->timing_mode_id,
		 DRM_MODE_ARG(&crtc_state->mode));
	if (mode->color_mode_id == mode->sdr_rgb.id)
		cmode = &mode->sdr_rgb;
	else if (mode->color_mode_id == mode->sdr_444.id)
		cmode = &mode->sdr_444;
	else if (mode->color_mode_id == mode->sdr.id)
		cmode = &mode->sdr;
	else if (mode->color_mode_id == mode->best.id)
		cmode = &mode->best;
	if (cmode)
		dev_info(dcp->dev,
			"set_digital_out_mode() color mode depth:%hhu format:%u "
			"colorimetry:%u eotf:%u range:%u vrr:%u\n", cmode->depth,
			cmode->format, cmode->colorimetry, cmode->eotf,
			cmode->range, mode->vrr);

	dcp->mode = (struct dcp_set_digital_out_mode_req){
		.color_mode_id = mode->color_mode_id,
		.timing_mode_id = mode->timing_mode_id
	};

	/* Built-in ProMotion panels require timestamps even in fixed-120 mode. */
	dcp->use_timestamps = mode->vrr && dcp->main_display;

	cookie = kzalloc(sizeof(*cookie), GFP_KERNEL);
	if (!cookie) {
		return -ENOMEM;
	}

	init_completion(&cookie->done);
	kref_init(&cookie->refcount);
	/* increase refcount to ensure the receiver has a reference */
	kref_get(&cookie->refcount);

	dcp->during_modeset = true;
	dcp->swap_submit_timestamp = 0;

#if DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0)
	/*
	 * For the built-in panel, inherit the mode iBoot already programmed.
	 * A hotplugged external display has no inherited timings: it needs
	 * the real modeset below before firmware will submit a frame.
	 *
	 * macOS never issues A413 in a full capture -- it attaches to a display
	 * iBoot has already configured.  Ours asks for colorID 1 / timingID 2
	 * and the firmware answers 0x8000000b, so the timing generator never
	 * starts: no vblank, no swap completion, black panel.  The panel is
	 * already running at its native 2408x1506, so skip the modeset and let
	 * that timing stand.
	 */
static bool dcp_send_modeset;

	if (!dcp_send_modeset &&
	    dcp->connector_type == DRM_MODE_CONNECTOR_eDP) {
		dev_info(dcp->dev, "inheriting the bootloader's display mode\n");
		complete_set_digital_out_mode(dcp, NULL, cookie);
	} else
#endif
	if (mode->vrr)
		dcp_set_adaptive_sync(dcp,
				      crtc_state->vrr_enabled ? mode->min_vrr : 0,
				      cookie);
	else
		dcp_set_digital_out_mode(dcp, false, &dcp->mode,
					 complete_set_digital_out_mode, cookie);


	/*
	 * The DCP firmware has an internal timeout of ~8 seconds for
	 * modesets. Add an extra 500ms to safe side that the modeset
	 * call has returned.
	 */
	ret = wait_for_completion_timeout(&cookie->done,
					  msecs_to_jiffies(8500));

	dcp->during_modeset = false;

	if (dcp->pending_hotplug) {
		bool connected = dcp->pending_hotplug_connected;
		struct apple_connector *connector = dcp->connector;

		dcp->pending_hotplug = false;
		dev_info(dcp->dev, "replaying deferred hotplug connected:%d\n",
			 connected);

		if (!connected)
			dcp->valid_mode = false;

		if (connector && connector->connected != connected) {
			connector->connected = connected;
			dcp->valid_mode = false;
			schedule_work(&connector->hotplug_wq);
		}
	}
	dev_info(dcp->dev, "set_digital_out_mode finished:%d\n", ret);

	if (ret == 0) {
		dev_info(dcp->dev, "set_digital_out_mode timed out\n");
		kref_put(&cookie->refcount, release_wait_cookie);
		return -EIO;
	} else if (ret < 0) {
		dev_info(dcp->dev,
			 "waiting on set_digital_out_mode failed:%d\n", ret);
		kref_put(&cookie->refcount, release_wait_cookie);
		return -EIO;
	} else {
		dev_dbg(dcp->dev,
			"set_digital_out_mode finished with %d to spare\n",
			jiffies_to_msecs(ret));
	}
	kref_put(&cookie->refcount, release_wait_cookie);
	dcp->valid_mode = true;
	dcp->vrr_enabled = mode->vrr && crtc_state->vrr_enabled;

	return 0;
}

void DCP_FW_NAME(iomfb_flush)(struct apple_dcp *dcp, struct drm_crtc *crtc, struct drm_atomic_state *state)
{
	struct drm_plane *plane;
	struct drm_plane_state *new_state, *old_state;
	struct drm_crtc_state *crtc_state;
	struct DCP_FW_NAME(dcp_swap_submit_req) *req = &DCP_FW_UNION(dcp->swap);
	int plane_idx, l;
	int has_surface = 0;

	crtc_state = drm_atomic_get_new_crtc_state(state, crtc);

	/* Reset all surfaces to defaults */
	memset(req, 0, sizeof(*req));
	for (l = 0; l < SWAP_SURFACES; l++)
		req->surf_null[l] = true;
#if DCP_FW_VER >= DCP_FW_VERSION(13, 2, 0)
#if DCP_FW_VER < DCP_FW_VERSION(26, 0, 0)
	for (l = 0; l < ARRAY_SIZE(req->surf2_null); l++)
		req->surf2_null[l] = true;
#endif
	req->unkU32Ptr_null = true;
	req->unkU32out_null = true;
#endif
#if DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0)
	/*
	 * macOS 25G83 never sends an H17P swap with bl_h17p_d[2]/[3] at zero:
	 * every one of the 223 captured swaps carries ~477 nits there even
	 * when the block is otherwise idle (see the field comments in
	 * iomfb_template.h).  Ours left them at zero after the memset above.
	 * With wlroots' Vulkan renderer submitting swaps back to back, the
	 * firmware reported "brightness 0.000 nits" and the panel went dark
	 * while the cursor moved (BRIO captures, 2026-09-19); paced swaps
	 * (gles2/pixman/synchronous submits) never did.  Seed the pair with
	 * the panel maximum on every swap so an idle block can never read as
	 * a zero ceiling.
	 */
	if (dcp_has_panel(dcp) && dcp->brightness.maximum > 0) {
		u64 max = dcp_nits_to_double_bits(dcp->brightness.maximum);

		req->swap.bl_h17p_d[2] = cpu_to_le64(max);
		req->swap.bl_h17p_d[3] = cpu_to_le64(max);
	}
#endif

	/*
	 * Clear all surfaces on startup. The boot framebuffer in surface 0
	 * sticks around.
	 */
	if (!dcp->surfaces_cleared) {
		req->swap.swap_enabled = IOMFB_SET_BACKGROUND | 0x7;
		req->swap.bg_color = 0xFF000000;
		dcp->surfaces_cleared = true;
	}

	for_each_oldnew_plane_in_state(state, plane, old_state, new_state, plane_idx) {
		struct apple_plane_state *apple_state = to_apple_plane_state(new_state);
		struct apple_plane *apple_plane = to_apple_plane(plane);

		/* skip planes not for this crtc */
		if (old_state->crtc != crtc && new_state->crtc != crtc)
			continue;

		l = apple_plane->iomfb_surface;
		req->swap.swap_enabled |= BIT(l);

		/* the old framebuffer was retained by dcp_flush() already */

		if (!new_state->fb || !new_state->visible) {
			continue;
		}
#if DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0)
		/*
		 * The firmware fetches straight from the display DART; a
		 * surface it cannot translate is the app NMI of
		 * iomfb_poweroff.  Check the surface's first and last page in
		 * the display domain before handing the address over, and
		 * keep the last good surface instead of a refused one.
		 */
		if (dcp->disp_iommu_dom && apple_state->surf.buf_size) {
			dma_addr_t first = apple_state->iova;
			dma_addr_t last = first + apple_state->surf.buf_size - 1;

			if (!iommu_iova_to_phys(dcp->disp_iommu_dom, first) ||
			    !iommu_iova_to_phys(dcp->disp_iommu_dom, last)) {
				dev_warn_ratelimited(dcp->dev,
					"fb %u: surface %pad+%#x is not mapped in the display DART, not scanning it out\n",
					new_state->fb->base.id, &first,
					apple_state->surf.buf_size);
				continue;
			}
		}
#endif
		req->surf_null[l] = false;
		has_surface = 1;

		req->swap.src_rect[l] = apple_state->src_rect;
		req->swap.dst_rect[l] = apple_state->dst_rect;

		if (dcp->notch_height > 0)
			req->swap.dst_rect[l].y += dcp->notch_height;

		req->surf_iova[l] = apple_state->iova;
		req->surf[l].base = apple_state->surf;
#if DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0)
		req->swap.surf_flags[l] = dcp_h17p_surf_flags;
#endif

		/* Use sRGB colorspace only for internal panels. External
		 * displays are expected to have EDID and user space can use
		 * the contained colorimetry information to provide native
		 * colors.
		 */
		if (dcp->connector_type == DRM_MODE_CONNECTOR_eDP &&
		    req->surf[l].base.colorspace == DCP_COLORSPACE_BG_SRGB)
			req->surf[l].base.colorspace = DCP_COLORSPACE_NATIVE;
	}

	if (!has_surface && !crtc_state->color_mgmt_changed) {
		if (crtc_state->enable && crtc_state->active &&
		    !crtc_state->planes_changed) {
			schedule_delayed_work(&dcp->vblank_wq,
					      msecs_to_jiffies(5));
			return;
		}

		/* Set black background */
		req->swap.swap_enabled |= IOMFB_SET_BACKGROUND;
		req->swap.bg_color = 0xFF000000;
		req->clear = 1;
	}

#if DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0)
	/*
	 * A swap with no surface at all makes this firmware fetch from a null
	 * base and take the app NMI described in iomfb_poweroff.  Keep the
	 * last surface on the panel instead and let the commit complete.
	 */
	if (!has_surface) {
		dev_warn_once(dcp->dev,
			      "refusing a surface-less swap on H17P; keeping the last surface\n");
		schedule_delayed_work(&dcp->vblank_wq, 0);
		return;
	}
#endif

#if DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0)
	/*
	 * Stage the surface before the wire fix-up. Publish it for brightness
	 * replay only when the submit response accepts it.
	 */
	memcpy(&dcp->h17p_pending_swap, req, sizeof(*req));
#endif

	if (has_surface)
		dcp_swap_set_timestamps(dcp, req);

	/* These fields should be set together */
	req->swap.swap_completed = req->swap.swap_enabled;

	/* update brightness if changed */
	if (dcp_has_panel(dcp) && dcp->brightness.update) {
#if DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0)
		dcp_h17p_set_swap_brightness(dcp, &req->swap, dcp->brightness.nits);
#else
		req->swap.bl_unk = 1;
		req->swap.bl_value = dcp->brightness.dac;
		req->swap.bl_power = 0x40;
#endif
		dcp->brightness.update = false;
	}

	if (crtc_state->color_mgmt_changed) {
		struct iomfb_set_matrix_req mat = {
			.location = 9,
		};

		if (crtc_state->ctm) {
			struct drm_color_ctm *ctm = (struct drm_color_ctm *)crtc_state->ctm->data;
			memcpy(mat.matrix, ctm->matrix, sizeof(mat.matrix));
		} else {
			mat.matrix[0] = mat.matrix[4] = mat.matrix[8] = 1LLU << 32;
		}

#if DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0)
		iomfb_set_matrix(dcp, false, &mat, dcp_pre_swap_a428, NULL);
#else
		iomfb_set_matrix(dcp, false, &mat, do_swap, NULL);
#endif
	} else
		do_swap(dcp, NULL, NULL);
}

static void res_is_main_display(struct apple_dcp *dcp, void *out, void *cookie)
{
	struct apple_connector *connector;
	int result = *(int *)out;
	dev_info(dcp->dev, "DCP is_main_display: %d\n", result);

	dcp->main_display = result != 0;


	connector = dcp->connector;
	if (connector) {
		connector->connected = dcp->nr_modes > 0;
		schedule_work(&connector->hotplug_wq);
	}

	dcp->active = true;
	complete(&dcp->start_done);
}

static void init_3(struct apple_dcp *dcp, void *out, void *cookie);

static void init_3(struct apple_dcp *dcp, void *out, void *cookie)
{
	dcp_is_main_display(dcp, false, res_is_main_display, NULL);
}

static void init_2(struct apple_dcp *dcp, void *out, void *cookie)
{
#if DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0)
	/*
	 * macOS always calls first_client_open here; skipping it leaves the
	 * firmware with no client and it never accepts a swap.  The skip is
	 * kept only as a bisect lever and defaults off.
	 */
	if (dcp_defer_client_open) {
		init_3(dcp, NULL, NULL);
		return;
	}
#endif
	dcp_first_client_open(dcp, false, init_3, NULL);
}

#if DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0)
/*
 * The client-open sequence, in the order macOS 25G83 issues it:
 *
 *   A445 A411 A445 A415 A450 A031 x9 A414 A478 A457 A474 A478 A476 A478 A412
 *
 * Every payload below is the traced one.  A412 is not in the table: its result
 * feeds res_is_main_display(), so it stays on the existing path.
 */
enum dcp_client_step_kind {
	DCP_STEP_A445, DCP_STEP_A411, DCP_STEP_A415, DCP_STEP_A450,
	DCP_STEP_A031, DCP_STEP_A414, DCP_STEP_A478, DCP_STEP_A457,
	DCP_STEP_A474, DCP_STEP_A476,
};

static const u8 dcp_client_open_seq[] = {
	/*
	 * macOS reaches the client phase only after A353 and two A352s (with
	 * five A438 property pushes in between, which we cannot replay yet).
	 * Without them the firmware never answers the first A445.
	 */
	/*
	 * A352 is left out: the firmware stops answering it here, and macOS
	 * only reaches it after five A438 property pushes we cannot replay yet.
	 */
	DCP_STEP_A445, DCP_STEP_A411, DCP_STEP_A445, DCP_STEP_A415,
	DCP_STEP_A450,
	DCP_STEP_A031, DCP_STEP_A031, DCP_STEP_A031, DCP_STEP_A031,
	DCP_STEP_A031, DCP_STEP_A031, DCP_STEP_A031, DCP_STEP_A031,
	DCP_STEP_A031,
	DCP_STEP_A414, DCP_STEP_A478, DCP_STEP_A457, DCP_STEP_A474,
	DCP_STEP_A478, DCP_STEP_A476, DCP_STEP_A478,
};


static void dcp_client_open_run(struct apple_dcp *dcp, void *out, void *cookie)
{
	unsigned long idx = (unsigned long)cookie;
	dcp_callback_t next = dcp_client_open_run;
	void *ncookie = (void *)(idx + 1);

	if (idx >= ARRAY_SIZE(dcp_client_open_seq)) {
		init_3(dcp, NULL, NULL);
		return;
	}

	switch (dcp_client_open_seq[idx]) {
		case DCP_STEP_A445: {
		u32 zero = 0;

		dcp_get_dfb_state(dcp, false, &zero, next, ncookie);
		break;
	}
	case DCP_STEP_A411: {
		u32 handle = 0;

		dcp_set_display_device(dcp, false, &handle, next, ncookie);
		break;
	}
	case DCP_STEP_A415: {
		struct dcp_pipe_cfg_415 cfg = {};

		dcp_pipe_cfg_415(dcp, false, &cfg, next, ncookie);
		break;
	}
	case DCP_STEP_A450: {
		u32 val = 0;

		dcp_enable_disable_video_power_savings(dcp, false, &val, next,
						       ncookie);
		break;
	}
	case DCP_STEP_A031: {
		/*
		 * The nine cumulative states macOS sends, from the 25G83
		 * capture (dcp-calls.log lines 1507-1531).  Each call repeats
		 * the previous set and adds one client.
		 */
		static const u8 notify_clients[9][8] = {
			{},
			{ 6 },
			{ 6, 7 },
			{ 6, 7, 8 },
			{ 6, 7, 8, 10 },
			{ 6, 7, 8, 10, 11 },
			{ 6, 7, 8, 10, 11, 12 },
			{ 6, 7, 8, 10, 11, 12, 25 },
			{ 6, 7, 8, 10, 11, 12, 25, 26 },
		};
		static const u8 notify_count[9] = { 0, 1, 2, 3, 4, 5, 6, 7, 8 };
		struct dcp_pipe_cfg_031 cfg = {};
		unsigned int step = dcp->a031_step;
		unsigned int i;

		if (step >= ARRAY_SIZE(notify_clients))
			step = ARRAY_SIZE(notify_clients) - 1;
		for (i = 0; i < notify_count[step]; i++)
			cfg.clients[i] = cpu_to_le32(notify_clients[step][i]);
		dcp->a031_step++;

		dcp_pipe_cfg_031(dcp, false, &cfg, next, ncookie);
		break;
	}
	case DCP_STEP_A414: {
		u64 val = 0;

		dcp_pipe_cfg_414(dcp, false, &val, next, ncookie);
		break;
	}
	case DCP_STEP_A478:
		dcp_pipe_query_478(dcp, false, next, ncookie);
		break;
	case DCP_STEP_A457:
		dcp_first_client_open(dcp, false, next, ncookie);
		break;
	case DCP_STEP_A474:
		dcp_pipe_query_474(dcp, false, next, ncookie);
		break;
	case DCP_STEP_A476: {
		/*
		 * A476 is IOMobileFramebufferAP::set_dcp_clamshellstate(uint).
		 * We were sending 1 -- copied from the capture without knowing
		 * what it was -- which tells the firmware the lid is shut, so
		 * it parks the internal pipe (power_down_M3) and never scans
		 * out even though the panel and backlight are on.
		 */
		u32 state = DCP_CLAMSHELL_STATE;

		dcp_pipe_set_476(dcp, false, &state, next, ncookie);
		break;
	}
	default:
		dev_warn(dcp->dev, "unknown client-open step %u\n",
			 dcp_client_open_seq[idx]);
		init_3(dcp, NULL, NULL);
		break;
	}
}
#endif

static void init_1b(struct apple_dcp *dcp, void *out, void *cookie)
{
	u32 val = 0;
	dcp_enable_disable_video_power_savings(dcp, false, &val, init_2, NULL);
}

static void init_1(struct apple_dcp *dcp, void *out, void *cookie)
{
#if DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0)
	/*
	 * macOS opens the client sequence with A411 set_display_device(0),
	 * before A450: traced as "CMD 0x0 A411 0x4 0x4 00000000".
	 */
	/* the sequence starts with A445, then A411 */
	dcp_client_open_run(dcp, NULL, (void *)0UL);
#else
	init_1b(dcp, out, cookie);
#endif
}

static void dcp_started(struct apple_dcp *dcp, void *data, void *cookie)
{
	struct iomfb_get_color_remap_mode_req color_remap =
		(struct iomfb_get_color_remap_mode_req){
			.mode = 6,
		};

	dev_info(dcp->dev, "DCP booted\n");

#if DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0)
	/*
	 * H17P starts scanning out the moment it finishes booting, but it has
	 * no surface yet and the stream it uses has no translation table, so
	 * the DART faults continuously and starves the machine.  Park the
	 * display first thing; the normal poweron path brings it back with a
	 * real framebuffer when a mode is set.
	 */
	/* the display is parked at the head of the boot chain, not here */
#endif

	iomfb_get_color_remap_mode(dcp, false, &color_remap, init_1, cookie);
}

void DCP_FW_NAME(iomfb_shutdown)(struct apple_dcp *dcp)
{
	struct DCP_FW_NAME(dcp_set_power_state_req) req = {
		/* defaults are ok */
	};

	dcp_set_power_state(dcp, DCP_POWER_OOB, &req, NULL, NULL);
}
