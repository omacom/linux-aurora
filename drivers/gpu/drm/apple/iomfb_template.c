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
#include <linux/overflow.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/unaligned.h>

#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_vblank.h>

#include "dcp.h"
#include "dcp-internal.h"
#include "iomfb.h"
#include "iomfb_internal.h"
#include "parser.h"
#include "dcp-property.h"
#include "trace.h"
#include "version_utils.h"

/* Register defines used in bandwidth setup structure */
#define REG_DOORBELL_BIT(idx) (2 + (idx))

static_assert(offsetof(struct DCP_FW_NAME(dcp_swap), timestamp[6]) == 0x30);
static_assert(offsetof(struct DCP_FW_NAME(dcp_swap), flags1) == 0x40);

struct dcp_wait_cookie {
	struct kref refcount;
	struct completion done;
	u32 status;
	bool replied;
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

#if DCP_FW_VERSION(26, 0, 0) <= DCP_FW_VER
static void dcp_swap_submit(struct apple_dcp *dcp, bool oob,
			    struct dcp_swap_submit_req_h17p *request,
			    dcp_callback_t cb, void *cookie)
{
	const void *data = request;

	if (dcp->hw.iomfb_method_profile != DCP_IOMFB_METHODS_H17G) {
		iomfb_serialize_present_h17p(&dcp->present_h17p, request);
		if (!iomfb_apply_backlight_h17p(dcp, request, &dcp->present_h17p)) {
			if (cb)
				cb(dcp, NULL, cookie);
			return;
		}
		data = &dcp->present_h17p;
	}

	dcp_push(dcp, oob, &dcp_methods[dcpep_swap_submit], sizeof(*request),
		 sizeof(struct dcp_swap_submit_resp_h17p), (void *)data, cb, cookie);
}
#else
DCP_THUNK_INOUT(dcp_swap_submit, dcpep_swap_submit,
		struct DCP_FW_NAME(dcp_swap_submit_req),
		struct DCP_FW_NAME(dcp_swap_submit_resp));
#endif

DCP_THUNK_INOUT(dcp_swap_start, dcpep_swap_start, struct DCP_FW_NAME(dcp_swap_start_req),
		struct DCP_FW_NAME(dcp_swap_start_resp));

DCP_THUNK_INOUT(dcp_set_power_state, dcpep_set_power_state,
		struct DCP_FW_NAME(dcp_set_power_state_req),
		struct dcp_set_power_state_resp);

/*
 * H17P firmware expects set_power_state (A473) on the out-of-band command
 * channel.  Sent on the ordinary command channel it is never answered, which
 * strands the power transition and every later call behind it.
 */
#if DCP_FW_VERSION(26, 0, 0) <= DCP_FW_VER
#define DCP_POWER_OOB	true
#else
#define DCP_POWER_OOB	false
#endif

/*
 * H17P firmware is attached to a display the bootloader has already brought
 * up, and it does not accept set_digital_out_mode (A413) for that display:
 * the call fails (0x8000000b) and the timing generator never starts, so there
 * is no vblank and no swap completion.  Keep the mode the bootloader
 * programmed, which is the panel's native mode, instead of requesting one.
 */
#if DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0)
#define DCP_INHERIT_BOOT_MODE	true
#else
#define DCP_INHERIT_BOOT_MODE	false
#endif

/*
 * abort_swaps_dcp and last_client_close have no verified H17P method numbers,
 * and the H17P table leaves them out.  Rather than send the firmware a record
 * it would decode as some other call, power-off and sleep skip both and go
 * straight to set_power_state, whose number and sizes are known.  Power-off
 * still sends its clear swap first.
 */
#if DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0)
#define DCP_HAS_CLIENT_TEARDOWN	false
#else
#define DCP_HAS_CLIENT_TEARDOWN	true
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
 * Default-framebuffer queries issued from inside start_hardware_boot (D121).
 * The getters take a zeroed input of the expected size; the firmware only
 * checks the size.
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
 * Pipe-configuration calls the firmware expects between the boot chain and
 * the first swap.  Most of them have no known name; they are identified by
 * method number and wire size.
 */
struct dcp_pipe_cfg_415 {
	u8 data[0xc];
} __packed;

/*
 * A031 is update_notify_clients_dcp(u32[27]): an array of the DCP
 * notification clients the AP wants delivered.  Clients that are not
 * registered here never send the callbacks they gate.
 */
struct dcp_pipe_cfg_031 {
	__le32 clients[27];
} __packed;
static_assert(sizeof(struct dcp_pipe_cfg_031) == 0x6c);

DCP_THUNK_INOUT(dcp_pipe_cfg_415, dcpep_pipe_cfg_415,
		struct dcp_pipe_cfg_415, struct dcp_pipe_cfg_415);
DCP_THUNK_IN(dcp_pipe_cfg_031, dcpep_pipe_cfg_031, struct dcp_pipe_cfg_031);
DCP_THUNK_INOUT(dcp_pipe_cfg_414, dcpep_pipe_cfg_414, u64, u64);
DCP_THUNK_OUT(dcp_pipe_query_478, dcpep_pipe_query_478, u32);
DCP_THUNK_OUT(dcp_pipe_query_474, dcpep_pipe_query_474, u32);
DCP_THUNK_IN(dcp_pipe_set_476, dcpep_pipe_set_476, u32);
DCP_THUNK_INOUT(dcp_pipe_set_428, dcpep_pipe_set_428, u32, u32);
#endif

DCP_THUNK_VOID(dcp_first_client_open, dcpep_first_client_open);

DCP_THUNK_INOUT(dcp_set_parameter_dcp, dcpep_set_parameter_dcp,
		struct dcp_set_parameter_dcp, u32);

DCP_THUNK_INOUT(dcp_enable_disable_video_power_savings,
		dcpep_enable_disable_video_power_savings, u32, int);

DCP_THUNK_OUT(dcp_is_main_display, dcpep_is_main_display, u32);

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

static void dcpep_cb_swap_complete(struct apple_dcp *dcp,
				   struct DCP_FW_NAME(dc_swap_complete_resp) *resp)
{
	ktime_t now = ktime_get();
	bool retire = true;

#if DCP_FW_VERSION(26, 0, 0) <= DCP_FW_VER
	if (dcp->hw.iomfb_method_profile != DCP_IOMFB_METHODS_H17G &&
	    !dcp_present_complete_h17p(&dcp->present_state_h17p, resp->swap_id)) {
		/*
		 * Abort semantics are not captured on H17P.  If an aborted
		 * present completes after all, its framebuffers were already
		 * unarmed and its event signalled; record it and carry on.
		 */
		if (dcp_present_was_aborted_h17p(&dcp->present_state_h17p,
						 resp->swap_id)) {
			dev_warn_ratelimited(dcp->dev,
					     "completion for aborted present %u ignored\n",
					     resp->swap_id);
			return;
		}
		/*
		 * The swap watchdog retired it: its framebuffers stay
		 * referenced until a later present completes, and a present
		 * in flight now is a different one.
		 */
		if (dcp_present_was_retired_h17p(&dcp->present_state_h17p,
						 resp->swap_id)) {
			dev_warn_ratelimited(dcp->dev,
					     "completion for retired present %u ignored\n",
					     resp->swap_id);
			return;
		}
		dev_err(dcp->dev, "unexpected present completion %u\n", resp->swap_id);
		dcp->crashed = true;
		return;
	}
	if (dcp->hw.iomfb_method_profile != DCP_IOMFB_METHODS_H17G)
		retire = iomfb_present_complete_h17p(dcp);
#endif
	trace_iomfb_swap_complete(dcp, resp->swap_id);
	dcp->last_swap_id = resp->swap_id;
	dcp_swap_watchdog_complete(dcp);
	if (retire) {
		dcp_release_retained_framebuffers(dcp, resp->swap_id);
		dcp_drm_crtc_page_flip(dcp, now);
	}
	if (READ_ONCE(dcp->crc_enabled)) {
#if DCP_FW_VERSION(26, 0, 0) <= DCP_FW_VER
		if (dcp->hw.iomfb_method_profile != DCP_IOMFB_METHODS_H17G) {
			iomfb_queue_crc_h17p(dcp, resp->swap_id);
			return;
		}
#endif
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

#if DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0)
/*
 * H17P asks the AP to make the default framebuffer surface (D582).  The
 * firmware expects the AP to register a surface from inside that callback,
 * with a nested A472 carrying a 0x22c-byte surface descriptor, and only then
 * to report success.  Without it the firmware has no surface, programs the
 * display engine from a null base and the piodma DART faults.
 */
struct dcp_h17p_dfb_surface_req {
	u8 surface[0x22c];
	u8 surface_null;
	u8 padding[3];
} __packed;
static_assert(sizeof(struct dcp_h17p_dfb_surface_req) == 0x230);

/*
 * A472 has a 4-byte output; with out_len 0 the firmware has nowhere to write
 * its result and stops answering.
 */
DCP_THUNK_INOUT(dcp_register_dfb_surface, dcpep_register_dfb_surface,
		struct dcp_h17p_dfb_surface_req, u32);

static void dcp_ack_dfb_surface(struct apple_dcp *dcp)
{
	struct dcp_channel *ch = &dcp->ch_cb;
	u8 *succ = ch->output[ch->depth - 1];

	*succ = true;
	dcp_ack(dcp, DCP_CONTEXT_CB);
}

/*
 * Fill the default surface descriptor.  Fields not set here are zero.  The
 * format at +0x0b is the fourcc 'b3a8', stored byte-reversed; +0x35 is the
 * default surface ID.  The bytes at +0x29..+0x2c are an opaque word whose
 * meaning is not known.
 */
static void dcp_fill_dfb_surface(struct apple_dcp *dcp, u8 *s)
{
	s[0x002] = 0x01;
	memcpy(s + 0x00b, "8a3b", 4);
	s[0x019] = 0x01;
	s[0x01b] = 0x01;
	s[0x01c] = 0x01;
	s[0x02b] = 0x4d;
	s[0x02c] = 0x01;
	s[0x035] = 0x02;
	s[0x051] = 0x01;
	s[0x149] = 0x01;

	if (dcp->hw.iomfb_method_profile != DCP_IOMFB_METHODS_H17G) {
		/* Internal H17P default registration differs from the older profile. */
		memcpy(s + 11, "ARGB", 4);
		put_unaligned_le32(0x00e44000, s + 41);
		s[53] = BIT(2);
	}
}

/*
 * A471 (update_dfb) is never sent: on H17P it makes the coprocessor take an
 * AXI read error.  Scanout does not need it, since every swap carries its own
 * surface.  Just acknowledge the callback.
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
	 * The request is too large for the stack this deep in the RTKit
	 * receive work (rx work -> iomfb_recv_msg -> dcpep_handle_cb -> here ->
	 * dcp_push).  dcp_push() copies the payload into shared memory before
	 * returning, so a short-lived allocation is enough.
	 */
	req = kzalloc_obj(*req);
	if (!req)
		return true;

	dcp_fill_dfb_surface(dcp, req->surface);

	dcp_register_dfb_surface(dcp, false, req, complete_dfb_surface, NULL);
	kfree(req);

	/* deferred ACK: the A472 completion reports success for us */
	return false;
}
#endif

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
	case IOMFB_PROPERTY_NITS_V13_3:
	/*
	 * 13.x firmware publishes another property under the 14.7 ID, with
	 * value 1, right before the panel level. Read as nits it is 0, and a
	 * backlight registered from it starts at the lowest level.
	 */
#if DCP_FW_VERSION(14, 7, 0) <= DCP_FW_VER
	case IOMFB_PROPERTY_NITS_V14_7:
#endif
	{
#if DCP_FW_VERSION(26, 0, 0) <= DCP_FW_VER
		/* The H17P takeover level comes from the powerlog hint interface. */
		if (dcp->hw.iomfb_method_profile != DCP_IOMFB_METHODS_H17G)
			break;
#endif
		if (dcp_has_panel(dcp)) {
			u32 nits = prop->value / dcp->brightness.scale;

			/* Firmware can report the old 420-nit scale above the cap. */
			if (dcp->brightness.maximum && nits > dcp->brightness.maximum)
				nits = dcp->brightness.maximum;
			dcp->brightness.nits = nits;
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

	if (dcp->panel.has_mini_led &&
	    memcmp(req->obj, "SUMP", sizeof(req->obj)) == 0) { /* "PMUS */
	    if (strncmp(req->key, "Temperature", sizeof(req->key)) == 0) {
		/*
		 * TODO: value from j314c, find out if it is temperature in
		 *       centigrade C and which temperature sensor reports it
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
static bool dcpep_map_piodma(struct apple_dcp *dcp,
			     struct dcp_map_buf_req *req,
			     struct DCP_FW_NAME(dcp_map_buf_resp) * resp)
{
	struct dcp_mem_descriptor *memdesc;
	struct sg_table *map;
	size_t size;
	ssize_t ret;

	if (req->buffer >= ARRAY_SIZE(dcp->memdesc))
		goto reject;

	memdesc = &dcp->memdesc[req->buffer];
	map = &memdesc->map;
	size = ALIGN(memdesc->size, SZ_16K);

	if (!test_bit(req->buffer, dcp->memdesc_map) || !map->sgl ||
	    memdesc->piodma_mapped || !dcp->iommu_dom)
		goto reject;

	/* use the piodma iommu domain to map against the right IOMMU */
	ret = iommu_map_sgtable(dcp->iommu_dom, memdesc->dva, map,
				IOMMU_READ | IOMMU_WRITE);

	if (ret != (ssize_t)size) {
		dev_err(dcp->dev,
			"iommu_map_sgtable() returned %zd instead of %zu\n",
			ret, size);
		if (ret > 0)
			iommu_unmap(dcp->iommu_dom, memdesc->dva, ret);
		goto reject;
	}

#if DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0)
	memdesc->piodma_mapped = true;

	/* the H17P reply echoes the buffer index */
	*resp = (struct DCP_FW_NAME(dcp_map_buf_resp)) {
		.buffer = req->buffer,
		.dva = memdesc->dva,
	};
#else
	*resp = (struct DCP_FW_NAME(dcp_map_buf_resp)) { .dva = memdesc->dva };
#endif
	return true;

reject:
	dev_err(dcp->dev, "denying map of invalid buffer %llx for piodma\n",
		req->buffer);
	return false;
}

static struct DCP_FW_NAME(dcp_map_buf_resp) dcpep_cb_map_piodma(struct apple_dcp *dcp,
						   struct dcp_map_buf_req *req)
{
	struct DCP_FW_NAME(dcp_map_buf_resp) resp = { 0 };

	if (dcpep_map_piodma(dcp, req, &resp))
		return resp;

#if DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0)
	resp.buffer = req->buffer;
#else
	resp.ret = EINVAL;
#endif
	return resp;
}

static bool dcpep_unmap_piodma(struct apple_dcp *dcp,
			       struct dcp_unmap_buf_resp *request)
{
	struct dcp_mem_descriptor *memdesc;
	size_t size, unmapped;

	if (request->buffer >= ARRAY_SIZE(dcp->memdesc)) {
		dev_warn(dcp->dev, "unmap request for out of range buffer %llu\n",
			 request->buffer);
		return false;
	}

	memdesc = &dcp->memdesc[request->buffer];

	if (!test_bit(request->buffer, dcp->memdesc_map) || !memdesc->buf ||
	    !dcp->iommu_dom)
		goto not_mapped;
#if DCP_FW_VERSION(26, 0, 0) <= DCP_FW_VER
	if (!memdesc->piodma_mapped)
		goto not_mapped;
#endif

	if (memdesc->dva != request->dva) {
		dev_warn(dcp->dev,
			 "unmap buffer %llu address mismatch memdesc.dva:%llx dva:%llx\n",
			 request->buffer,
			 memdesc->dva, request->dva);
		return false;
	}

	/* use the piodma iommu domain to unmap from the right IOMMU */
	size = ALIGN(memdesc->size, SZ_16K);
	unmapped = iommu_unmap(dcp->iommu_dom, memdesc->dva, size);
	if (unmapped != size) {
		dev_err(dcp->dev, "iommu_unmap() returned %zu instead of %zu\n",
			unmapped, size);
		return false;
	}
	memdesc->piodma_mapped = false;
	return true;

not_mapped:
	dev_warn(dcp->dev,
		 "unmap for non-mapped buffer %llu iova:0x%08llx\n",
		 request->buffer, request->dva);
	return false;
}

static void dcpep_cb_unmap_piodma(struct apple_dcp *dcp,
				  struct dcp_unmap_buf_resp *request)
{
	dcpep_unmap_piodma(dcp, request);
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
	struct dcp_mem_descriptor allocated = { 0 };
	size_t size, rounded;
	u32 id;
	int ret;

	/* Validate both wire-size and IOMMU-page rounding before allocating. */
	if (!req->size || req->size > SIZE_MAX ||
	    check_add_overflow((size_t)req->size, (size_t)4095, &rounded))
		return resp;
	allocated.size = round_down(rounded, 4096);
	if (check_add_overflow(allocated.size, (size_t)SZ_16K - 1, &rounded))
		return resp;
	size = round_down(rounded, SZ_16K);

	id = find_first_zero_bit(dcp->memdesc_map, DCP_MAX_MAPPINGS);

	if (id >= DCP_MAX_MAPPINGS) {
		dev_warn(dcp->dev, "DCP overflowed mapping table, ignoring\n");
		return resp;
	}

	allocated.buf = dma_alloc_coherent(dcp->dev, size, &allocated.dva,
					   GFP_KERNEL);
	if (!allocated.buf)
		return resp;

	ret = dma_get_sgtable(dcp->dev, &allocated.map, allocated.buf,
			      allocated.dva, size);
	if (ret) {
		dma_free_coherent(dcp->dev, size, allocated.buf, allocated.dva);
		return resp;
	}

	/* Callbacks are serialized; publish only a fully initialized descriptor. */
	dcp->memdesc[id] = allocated;
	set_bit(id, dcp->memdesc_map);
	resp.mem_desc_id = id;
	resp.dva_size = allocated.size;
	resp.dva = allocated.dva;

#if DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0)
	/*
	 * Linux allocation traces correlate the first H17P reply address with
	 * the first SG page. The remaining pages need not be contiguous.
	 */
	if (allocated.map.sgl)
		resp.paddr = sg_phys(allocated.map.sgl);
#endif

	return resp;
}

static u8 dcpep_cb_release_mem_desc(struct apple_dcp *dcp, u32 *mem_desc_id)
{
	struct dcp_mem_descriptor *memdesc;
	size_t size, unmapped;
	u32 id = *mem_desc_id;

	if (id >= DCP_MAX_MAPPINGS) {
		dev_warn(dcp->dev,
			 "unmap request for out of range mem_desc_id %u", id);
		return 0;
	}

	if (!test_bit(id, dcp->memdesc_map)) {
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
	 * then read whoever owns them next.  Only the H17P map_piodma handler
	 * sets piodma_mapped, so older firmware is unaffected.
	 */
	if (memdesc->piodma_mapped) {
		dev_warn(dcp->dev,
			 "releasing buffer %u while still mapped for piodma; unmapping first\n",
			 id);
		if (!dcp->iommu_dom)
			return 0;
		unmapped = iommu_unmap(dcp->iommu_dom, memdesc->dva, size);
		if (unmapped != size) {
			dev_err(dcp->dev,
				"failed to unmap buffer %u before release: %zu of %zu\n",
				id, unmapped, size);
			return 0;
		}
		memdesc->piodma_mapped = false;
	}
	clear_bit(id, dcp->memdesc_map);

	if (memdesc->buf) {
		sg_free_table(&memdesc->map);
		dma_free_coherent(dcp->dev, size, memdesc->buf, memdesc->dva);
		memdesc->buf = NULL;
		memset(&memdesc->map, 0, sizeof(memdesc->map));
	} else if (memdesc->size) {
		/* A register range mapped by map_physical */
		dma_unmap_resource(dcp->dev, memdesc->dva, memdesc->size,
				   DMA_BIDIRECTIONAL, 0);
		memdesc->reg = 0;
	}
	memdesc->dva = 0;

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
	u64 size, end;
	dma_addr_t dva;
	u32 id;

	/* Both values come from the coprocessor; reject wrapping ranges. */
	if (!req->size || check_add_overflow(req->size, 4095ULL, &size) ||
	    check_add_overflow(req->paddr, round_down(size, 4096) - 1, &end) ||
	    !is_disp_register(dcp, req->paddr, end)) {
		dev_err(dcp->dev, "refusing to map phys address %llx size %llx\n",
			req->paddr, req->size);
		return (struct dcp_map_physical_resp){};
	}
	size = round_down(size, 4096);

	id = find_first_zero_bit(dcp->memdesc_map, DCP_MAX_MAPPINGS);
	if (id >= DCP_MAX_MAPPINGS) {
		dev_warn(dcp->dev, "DCP overflowed mapping table, ignoring\n");
		return (struct dcp_map_physical_resp){};
	}

	dva = dma_map_resource(dcp->dev, req->paddr, size, DMA_BIDIRECTIONAL, 0);
	if (dma_mapping_error(dcp->dev, dva)) {
		dev_err(dcp->dev, "failed to map phys address %llx size %llx\n",
			req->paddr, size);
		return (struct dcp_map_physical_resp){};
	}

	set_bit(id, dcp->memdesc_map);
	dcp->memdesc[id].size = size;
	dcp->memdesc[id].reg = req->paddr;
	dcp->memdesc[id].dva = dva;

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

#if DCP_FW_VER >= DCP_FW_VERSION(13, 2, 0) && DCP_FW_VER < DCP_FW_VERSION(26, 0, 0)
/* The DVA that map_reg hands out for display register range @index. */
static dma_addr_t dcpep_map_reg_dva(struct apple_dcp *dcp, u32 index,
				    struct resource *rsrc)
{
#if DCP_FW_LAYOUT_14_7(DCP_FW_VER)
	/*
	 * T8122 dcpext: with the PMGR bandwidth page mapped, the DART faults
	 * at display power-off and the DCP crashes. Give it a RAM page
	 * instead. Generic M1/M2 DCPs on 14.7.0 also use this template
	 * instance and keep the MMIO page.
	 */
	if (of_device_is_compatible(dcp->dev->of_node, "apple,t8122-dcpext") &&
	    dcp->disp_bw_scratch_index && !dcp->disp_bw_doorbell_index &&
	    index == dcp->disp_bw_scratch_index) {
		dma_addr_t dva;

		if (!dmam_alloc_coherent(dcp->dev, resource_size(rsrc), &dva,
					 GFP_KERNEL))
			return DMA_MAPPING_ERROR;
		return dva;
	}
#endif
	return dma_map_resource(dcp->dev, rsrc->start, resource_size(rsrc),
				DMA_BIDIRECTIONAL, 0);
}
#endif

static struct DCP_FW_NAME(dcp_map_reg_resp) dcpep_cb_map_reg(struct apple_dcp *dcp,
						struct DCP_FW_NAME(dcp_map_reg_req) *req)
{
	if (req->index >= dcp->nr_disp_registers) {
		dev_warn(dcp->dev, "attempted to read invalid reg index %u\n",
			 req->index);

		return (struct DCP_FW_NAME(dcp_map_reg_resp)){ .ret = 1 };
	} else {
		struct resource *rsrc = dcp->disp_registers[req->index];
#if DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0)
		/*
		 * H17P reaches these apertures physically, not through its
		 * DART, and expects dva == addr in the reply.  Trying to
		 * dma_map_resource() them instead fails and hands the firmware
		 * DMA_MAPPING_ERROR as a device address.
		 */
		dma_addr_t dva = rsrc->start;
#elif DCP_FW_VER >= DCP_FW_VERSION(13, 2, 0)
		dma_addr_t dva = dcpep_map_reg_dva(dcp, req->index, rsrc);
		WARN_ON(dva == DMA_MAPPING_ERROR);
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
	/* Observe boot-property requests without inventing firmware timings. */
	if (dcp->fixed_connector_type != DRM_MODE_CONNECTOR_eDP)
		dev_dbg(dcp->dev,
			"read_edt_data key=%.*s count=%u default0=%#x ret=0\n",
			 (int)sizeof(req->key), req->key, req->count,
			 req->value[0]);

	return (struct dcp_read_edt_data_resp){
		.value[0] = req->value[0],
		.ret = 0,
	};
}

static void iomfbep_cb_enable_backlight_message_ap_gated(struct apple_dcp *dcp,
							 u8 *enabled)
{
#if DCP_FW_VERSION(26, 0, 0) <= DCP_FW_VER
	/* H17P carries the level only in presents; ask for one more. */
	if (dcp->hw.iomfb_method_profile != DCP_IOMFB_METHODS_H17G) {
		if (dcp_has_panel(dcp) && dcp_backlight_active(dcp) &&
		    dcp_backlight_resend(dcp))
			schedule_work(&dcp->bl_update_wq);
		return;
	}
#endif
	/*
	 * Reapply brightness on the next swap after DPMS wake on panels
	 * without mini-LED backlights.
	 */
	dcp->brightness.update = true;
	/* Backlight handling is only set up for panels described in the DT. */
	if (dcp_has_panel(dcp))
		schedule_work(&dcp->bl_update_wq);
}

/* Chunked data transfer for property dictionaries */
static u8 dcpep_cb_prop_start(struct apple_dcp *dcp, u32 *length)
{
	if (dcp->chunks.data != NULL) {
		dev_warn(dcp->dev, "ignoring spurious transfer start\n");
		return false;
	}

#if DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0)
	/*
	 * This length comes straight off the wire.  A bogus value would turn
	 * the allocation below into a multi-gigabyte GFP_KERNEL request.
	 * Property blobs are a few KiB, so bound it.
	 */
	if (*length == 0 || *length > SZ_1M) {
		dev_warn(dcp->dev, "rejecting %u byte property transfer\n",
			 *length);
		return false;
	}
#endif

	if (!dcp_property_size_valid(*length))
		return false;

	dcp->chunks.modes_generation = dcp_modes_transfer_begin(dcp);
	dcp->chunks.length = *length;
	dcp->chunks.av_service_epoch = 0;
	dcp->chunks.automatic_service_pending = false;
	if (dcp->hw.iomfb_method_profile == DCP_IOMFB_METHODS_H17P_EXTERNAL) {
		/* Match the receipt's start, not a replacement service at its end. */
		mutex_lock(&dcp->dcpavserv.lock);
		dcp->chunks.av_service_epoch = dcp->dcpavserv.epoch;
		dcp->chunks.automatic_service_pending =
			!dcp->dcpavserv.enabled && !dcp->dcpavserv.service;
		dcp_firmware_metadata_begin(dcp);
		{
			unsigned long flags;

			spin_lock_irqsave(&dcp->metadata.lock, flags);
			dcp->chunks.automatic_connection_sequence = dcp->rate_proof.connection_sequence;
			dcp->chunks.mode_ticket = dcp_metadata_begin(&dcp->metadata.epoch);
			dcp->chunks.mode_ticket.associated =
				dcp->metadata.epoch.generation == READ_ONCE(dcp->typec_generation) &&
				READ_ONCE(dcp->typec_cable_connected);
			spin_unlock_irqrestore(&dcp->metadata.lock, flags);
		}
		mutex_unlock(&dcp->dcpavserv.lock);
	} else {
		dcp_firmware_metadata_begin(dcp);
	}
	dcp->chunks.data = kzalloc(*length, GFP_KERNEL);

	if (!dcp->chunks.data) {
		dev_warn(dcp->dev, "failed to allocate chunks\n");
		return false;
	}

	return true;
}

static u8 dcpep_cb_prop_chunk(struct apple_dcp *dcp,
			      struct dcp_set_dcpav_prop_chunk_req *req)
{
	if (!dcp->chunks.data) {
		dev_warn(dcp->dev, "ignoring spurious chunk\n");
		return false;
	}

	if (req->offset > dcp->chunks.length || req->length > sizeof(req->data) ||
	    req->length > dcp->chunks.length - req->offset) {
		dcp->chunks.contiguous = false;
		dev_warn(dcp->dev, "ignoring overflowing chunk\n");
		return false;
	}

	if (req->offset != dcp->chunks.received)
		dcp->chunks.contiguous = false;
	if (dcp->chunks.contiguous)
		dcp->chunks.received += req->length;
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

	/* Validate serialized extent outside the metadata spinlock. */
	if (dcp->hw.iomfb_method_profile == DCP_IOMFB_METHODS_H17P_EXTERNAL)
		dcp->chunks.payload_complete = dcp->chunks.contiguous &&
			dcp_property_complete(dcp->chunks.data, dcp->chunks.received,
					      dcp->chunks.length);

	ret = parse(dcp->chunks.data, dcp->chunks.length, &ctx);

	if (ret) {
		dev_warn(dcp->dev, "bad header on dcpav props\n");
		/* A failed replacement cannot keep an unsafe catalog admitted. */
		if (dcp->hw.iomfb_method_profile == DCP_IOMFB_METHODS_H17P_EXTERNAL &&
		    !strcmp(req->key, "TimingElements")) {
			unsigned long flags;

			spin_lock_irqsave(&dcp->metadata.lock, flags);
			dcp->rate_proof.catalog_sequence++;
			dcp->metadata.catalog_ticket.associated = false;
			spin_unlock_irqrestore(&dcp->metadata.lock, flags);
			dcp_180hz_catalog_changed(dcp);
		}
		return false;
	}

	/* parse() resets the context: set the tracing pointer afterwards. */
	ctx.dcp = dcp;

	if (!strcmp(req->key, "TimingElements")) {
		ret = dcp_modes_replace(dcp, &ctx, dcp->chunks.modes_generation);
		if (ret) {
			if (ret == -ESTALE) {
				kfree(dcp->chunks.data);
				dcp->chunks.data = NULL;
				dcp->chunks.length = 0;
			}
			dev_warn(dcp->dev, "failed to parse modes\n");
			dcp_firmware_metadata_timing(dcp);
			return false;
		}
		if (!READ_ONCE(dcp->nr_modes))
			dev_warn(dcp->dev, "TimingElements without valid modes!\n");
		dcp_180hz_notify(dcp);
		dcp_firmware_metadata_timing(dcp);
	} else if (!strcmp(req->key, "DisplayAttributes")) {
		struct dcp_parse_ctx metadata_ctx = ctx;

		ret = dcp_attributes_replace(dcp, &ctx, dcp->chunks.modes_generation);
		if (ret) {
			if (ret == -ESTALE) {
				kfree(dcp->chunks.data);
				dcp->chunks.data = NULL;
				dcp->chunks.length = 0;
			}
			dev_warn(dcp->dev, "failed to parse display attribs\n");
			return false;
		}

		if (dcp->hw.iomfb_method_profile == DCP_IOMFB_METHODS_H17P_EXTERNAL) {
			struct dcp_display_metadata metadata;

			ret = parse_display_metadata(&metadata_ctx, &metadata);
			dcp_firmware_metadata_attributes(dcp, ret ? NULL : &metadata);
		}
	}

	return true;
}

static u8 dcpep_cb_prop_end(struct apple_dcp *dcp,
			    struct dcp_set_dcpav_prop_end_req *req)
{
	u8 resp;

	if (!dcp_property_key_valid(req->key, sizeof(req->key))) {
		kfree(dcp->chunks.data);
		dcp->chunks.data = NULL;
		dcp->chunks.length = 0;
		return false;
	}
	resp = dcpep_process_chunks(dcp, req);
	if (!dcp->chunks.data)
		return resp;

	if (dcp->fixed_connector_type != DRM_MODE_CONNECTOR_eDP)
		dev_dbg(dcp->dev,
			"DCP property key=%.*s bytes=%zu accepted=%u nr_modes=%u\n",
			 (int)sizeof(req->key), req->key, dcp->chunks.length,
			 resp, dcp->nr_modes);

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
	/* H17P expects A380 right before set_display_refresh_properties */
	dcp_dfb_ready_query(dcp, false, boot_5b, NULL);
#else
	boot_5b(dcp, out, cookie);
#endif
}

static void boot_4b(struct apple_dcp *dcp, void *out, void *cookie)
{
#if DCP_FW_VER >= DCP_FW_VERSION(13, 2, 0)
#if DCP_FW_VERSION(26, 0, 0) <= DCP_FW_VER
	u32 value = dcp->hw.iomfb_method_profile == DCP_IOMFB_METHODS_H17G;
#else
	u32 value = 1;
#endif
	dcp_late_init_signal(dcp, false, &value, boot_5, NULL);
#else
	dcp_late_init_signal(dcp, false, boot_5, NULL);
#endif
}

static void boot_4(struct apple_dcp *dcp, void *out, void *cookie)
{
#if DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0)
	/* H17P expects A033 between flush_supports_power and late_init_signal */
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
 * H17P expects the default framebuffer to be described between
 * create_default_fb and setup_video_limits:
 *   get_dfb_compression_info -> get_dfb_info -> get_dfb_layout ->
 *   get_dfb_state -> set_dfb_dimensions(w, h) -> commit_dfb_info
 * Without it the firmware never finishes its own default framebuffer setup.
 */
static void boot_1_11(struct apple_dcp *dcp, void *out, void *cookie)
{
	dcp_commit_dfb_info(dcp, false, boot_2, NULL);
}

static void boot_1_10(struct apple_dcp *dcp, void *out, void *cookie)
{
	struct dcp_dfb_dimensions_h17p dim;

	/*
	 * get_dfb_state returns the default framebuffer dimensions, which are
	 * passed back unchanged to set_dfb_dimensions.
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
	dev_dbg(dcp->dev, "default framebuffer dimensions %ux%u\n",
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
 * The firmware expects D000 to be answered only after a nested A033
 * (dfb_query), and then with 1.  Defer the ACK until that call completes.
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
 * The firmware expects zeros except for three u32 at 0x24, 0x28 and 0x2c,
 * each 0x10000 -- 1.0 in 16.16 fixed point, the identity scale.  An all-zero
 * reply hands the pipe a zero scale, and it then programs a null surface
 * base: the display DART faults at DVA 0 and the coprocessor takes an AXI
 * error.
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

	for (i = 0; i < ARRAY_SIZE(identity_at); i++) {
		u32 v = DCP_FRAME_SYNC_IDENTITY;

		memcpy(&resp.unk[identity_at[i]], &v, sizeof(v));
	}

	return resp;
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
	u32 status;
	u32 swap_id;
};

static void release_swap_cookie(struct kref *ref)
{
	struct dcp_swap_cookie *cookie;
	cookie = container_of(ref, struct dcp_swap_cookie, refcount);

        kfree(cookie);
}

#if DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0)
#define DCP_SWAP_FLAGS1 0x34865202ULL
#define DCP_SWAP_FLAGS2 0x4ULL

/*
 * H17P's swap surface has no is_tearing_allowed and no is_premultiplied:
 * plane_cnt sits at +1, plane_cnt2 at +5 and format at +9, two bytes earlier
 * than on every other firmware.  With the older layout the firmware reads the
 * format two bytes late and its surface check rejects the swap as an
 * unsupported format.
 *
 * H17G retains this public packing path. The H17P profile uses
 * a separate wire record with the SPEC's 0x588 swap boundary.
 */
static void dcp_h17p_prepare_swap(struct apple_dcp *dcp)
{
	struct DCP_FW_NAME(dcp_swap_submit_req) *req = &DCP_FW_UNION(dcp->swap);
	unsigned int i;

	/*
	 * The firmware expects flags1 = 0x34865202 and flags2 = 4 (at +0x40
	 * and +0x48) in every swap.
	 */
	req->swap.flags1 = DCP_SWAP_FLAGS1;
	req->swap.flags2 = DCP_SWAP_FLAGS2;

	/*
	 * The H17P gap before swap_id carries two small counters the firmware
	 * expects to be 2 (at +0x50) and 3 (at +0x78).  The gap starts at
	 * +0x50, so these are indices 0 and 0x28 of the blob.
	 */
	req->swap.h17p_pre_swap_id[0x00] = 2;
	req->swap.h17p_pre_swap_id[0x28] = 3;

	/* The H17P layout is serialized without shifting its surfaces. */
	if (dcp->hw.iomfb_method_profile != DCP_IOMFB_METHODS_H17G)
		return;

	for (i = 0; i < SWAP_SURFACES; i++) {
		u8 *p = (u8 *)&req->surf[i];
		size_t n = sizeof(req->surf[i]);

		memmove(p + 1, p + 3, n - 3);
		memset(p + n - 2, 0, 2);
	}

	/*
	 * Everything from surf_iova onward also sits two bytes earlier on
	 * H17P: surf_iova[0] is at +0xe38 of the 0xe9c-byte record, not
	 * +0xe3a.  Left in place, the firmware reads every IOVA two bytes off
	 * and the trailing null-flag block misaligned, so it faults on a bogus
	 * address or verifies layers that are not present.  Shift the whole
	 * block down.
	 */
	{
		u8 *r = (u8 *)req;
		size_t n = sizeof(*req);
		size_t off = offsetof(typeof(*req), surf_iova);

		static_assert(offsetof(typeof(*req), surf_iova) == 0xe3a,
			      "H17P source IOVA offset");
		static_assert(sizeof(*req) == 0xe9c, "H17P request size");
		memmove(r + off - 2, r + off, n - off);
		memset(r + n - 2, 0, 2);
	}
}
#else
static void dcp_h17p_prepare_swap(struct apple_dcp *dcp) { }
#endif

static bool dcp_present_retires(struct apple_dcp *dcp)
{
#if DCP_FW_VERSION(26, 0, 0) <= DCP_FW_VER
	if (dcp->hw.iomfb_method_profile != DCP_IOMFB_METHODS_H17G)
		return !iomfb_present_brightness_only_h17p(dcp);
#endif
	return true;
}

static void dcp_present_failed(struct apple_dcp *dcp)
{
#if DCP_FW_VERSION(26, 0, 0) <= DCP_FW_VER
	if (dcp->hw.iomfb_method_profile != DCP_IOMFB_METHODS_H17G)
		iomfb_present_failed_h17p(dcp);
#endif
}

static bool dcp_present_begin(struct apple_dcp *dcp, u32 swap_id)
{
#if DCP_FW_VERSION(26, 0, 0) <= DCP_FW_VER
	if (dcp->hw.iomfb_method_profile != DCP_IOMFB_METHODS_H17G &&
	    !dcp_present_begin_h17p(&dcp->present_state_h17p, swap_id)) {
		dev_err(dcp->dev, "overlapping present %u\n", swap_id);
		dcp->crashed = true;
		return false;
	}
#endif
	return true;
}

/*
 * Returns 0 if the submitted present stands, -ECANCELED if the firmware
 * aborted it before replying (already reported), or -EPROTO if no present
 * was expecting this reply.
 */
static int dcp_present_submit(struct apple_dcp *dcp, u32 swap_id, bool accepted)
{
#if DCP_FW_VERSION(26, 0, 0) <= DCP_FW_VER
	if (dcp->hw.iomfb_method_profile != DCP_IOMFB_METHODS_H17G &&
	    !dcp_present_submit_h17p(&dcp->present_state_h17p, swap_id, accepted)) {
		dev_err(dcp->dev, "unexpected present submission %u\n", swap_id);
		dcp->crashed = true;
		return -EPROTO;
	}
	/* An abort received before this reply turns acceptance into failure. */
	if (dcp->hw.iomfb_method_profile != DCP_IOMFB_METHODS_H17G &&
	    accepted && dcp->present_state_h17p.aborted) {
		dev_warn_ratelimited(dcp->dev, "firmware aborted present %u\n",
				     swap_id);
		return -ECANCELED;
	}
#endif
	return 0;
}

static void dcp_prepare_clear_swap(struct apple_dcp *dcp, void *request)
{
	typeof(DCP_FW_UNION(dcp->swap)) *swap = request;

	/* Clear surfaces. */
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
#if !DCP_FW_LAYOUT_14_7(DCP_FW_VER)
		swap->swap.bl_unk = 1;
#else
		swap->swap.bl_update = 1;
#endif
		swap->swap.bl_value = 0;
		swap->swap.bl_power = 0;
	}

	/* Null all surfaces */
	for (int l = 0; l < SWAP_SURFACES; l++)
		swap->surf_null[l] = true;
#if DCP_FW_VERSION(13, 2, 0) <= DCP_FW_VER
#if DCP_FW_VERSION(26, 0, 0) > DCP_FW_VER
	for (int l = 0; l < 5; l++)
		swap->surf2_null[l] = true;
#endif
	swap->unknown_pointer_null = true;
	swap->unknown_output_null = true;
#endif
}

static void dcp_swap_cleared(struct apple_dcp *dcp, void *data, void *cookie)
{
	struct DCP_FW_NAME(dcp_swap_submit_resp) *resp = data;
	struct dcp_swap_cookie *info = cookie;
	u32 swap_id = DCP_FW_UNION(dcp->swap).swap.swap_id;
	u32 status = resp ? resp->ret : ~0U;
	int ret;

	ret = dcp_present_submit(dcp, swap_id, !status);
	if (ret)
		status = ~0U;

	if (status) {
		/* An aborted present has already been reported. */
		if (ret != -ECANCELED)
			dev_err(dcp->dev, "swap_clear failed! status %u\n", status);
		dcp_present_failed(dcp);
		if (dcp_present_retires(dcp)) {
			dcp_unarm_retained_framebuffers(dcp, swap_id);
			dcp_drm_crtc_vblank(dcp->crtc);
		}
	}
	if (info) {
		WRITE_ONCE(info->status, status);
		complete(&info->done);
		kref_put(&info->refcount, release_swap_cookie);
	}
}

static void dcp_swap_clear_started(struct apple_dcp *dcp, void *data,
				   void *cookie)
{
	struct DCP_FW_NAME(dcp_swap_start_resp) *resp = data;
	struct dcp_swap_cookie *info = cookie;

	if (!resp || resp->ret) {
		if (info) {
			WRITE_ONCE(info->status, resp ? resp->ret : ~0U);
			complete(&info->done);
			kref_put(&info->refcount, release_swap_cookie);
		}
		dcp_drm_crtc_vblank(dcp->crtc);
		return;
	}
	if (!dcp_present_begin(dcp, resp->swap_id)) {
		if (info) {
			WRITE_ONCE(info->status, ~0U);
			complete(&info->done);
			kref_put(&info->refcount, release_swap_cookie);
		}
		dcp_drm_crtc_vblank(dcp->crtc);
		return;
	}
#if DCP_FW_VERSION(26, 0, 0) <= DCP_FW_VER
	if (dcp->hw.iomfb_method_profile != DCP_IOMFB_METHODS_H17G)
		dcp_prepare_clear_swap(dcp, &DCP_FW_UNION(dcp->swap));
#endif
	DCP_FW_UNION(dcp->swap).swap.swap_id = resp->swap_id;
	if (dcp_present_retires(dcp))
		dcp_arm_retained_framebuffers(dcp, resp->swap_id);

	if (info)
		info->swap_id = resp->swap_id;

	dcp_h17p_prepare_swap(dcp);
	dcp_swap_submit(dcp, false, &DCP_FW_UNION(dcp->swap), dcp_swap_cleared, cookie);
}

static void dcp_on_final(struct apple_dcp *dcp, void *out, void *cookie)
{
	struct dcp_wait_cookie *wait = cookie;

#if DCP_FW_VERSION(26, 0, 0) <= DCP_FW_VER
	if (dcp->hw.iomfb_method_profile != DCP_IOMFB_METHODS_H17G &&
	    out && get_unaligned_le64(out) == 1)
		WRITE_ONCE(dcp->pipe_enabled_h17p, true);
#endif

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

#if DCP_FW_VERSION(26, 0, 0) <= DCP_FW_VER
	if (dcp->hw.iomfb_method_profile != DCP_IOMFB_METHODS_H17G)
		iomfb_opaque_x_reset_h17p(dcp);
#endif

	cookie = kzalloc(sizeof(*cookie), GFP_KERNEL);
	if (!cookie)
		return;

	cookie->status = ~0U;
	init_completion(&cookie->done);
	kref_init(&cookie->refcount);
	/* increase refcount to ensure the receiver has a reference */
	kref_get(&cookie->refcount);

	if (dcp->main_display) {
#if DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0)
		/*
		 * H17P takes set_display_device (A411) once, in the
		 * client-open sequence.  Repeating it after the client is open
		 * is a call the firmware stops answering, which wedges the
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

	/* Force a brightness update after poweron, to restore the brightness */
	dcp->brightness.update = true;
}

static void complete_set_powerstate(struct apple_dcp *dcp, void *out,
				    void *cookie)
{
	struct dcp_set_power_state_resp *resp = out;
	struct dcp_wait_cookie *wait = cookie;

	if (wait) {
		WRITE_ONCE(wait->status, resp ? resp->ret : ~0U);
		complete(&wait->done);
		kref_put(&wait->refcount, release_wait_cookie);
	}
}

static bool dcp_power_stop_confirmed(struct dcp_wait_cookie *cookie)
{
	/* H17P does not confirm that scanout has stopped. */
#if DCP_FW_VERSION(26, 0, 0) <= DCP_FW_VER
	return false;
#else
	return READ_ONCE(cookie->status) == 0;
#endif
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
#if DCP_FW_VER >= DCP_FW_VERSION(14, 7, 0)
	/* iomfb_last_client_close times out on 14.7 */
	last_client_closed_poff(dcp, NULL, cookie);
#else
	struct iomfb_last_client_close_req last_client_req = {};
	iomfb_last_client_close(dcp, false, &last_client_req,
				last_client_closed_poff, cookie);
#endif
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
	typeof(DCP_FW_UNION(dcp->swap)) *swap = &DCP_FW_UNION(dcp->swap);

#if DCP_FW_VERSION(26, 0, 0) <= DCP_FW_VER
	/* H17P has no known safe pipe-off or surface-free transition. */
	if (dcp->hw.iomfb_method_profile != DCP_IOMFB_METHODS_H17G)
		return;
#endif

	cookie = kzalloc(sizeof(*cookie), GFP_KERNEL);
	if (!cookie)
		return;
	cookie->status = ~0U;
	init_completion(&cookie->done);
	kref_init(&cookie->refcount);
	/* increase refcount to ensure the receiver has a reference */
	kref_get(&cookie->refcount);

#if DCP_FW_VERSION(26, 0, 0) <= DCP_FW_VER
	if (dcp->hw.iomfb_method_profile == DCP_IOMFB_METHODS_H17G)
		dcp_prepare_clear_swap(dcp, swap);
#else
	dcp_prepare_clear_swap(dcp, swap);
#endif

	dcp_swap_start(dcp, false, &swap_req, dcp_swap_clear_started, cookie);

	/*
	 * On unplug the firmware powers the external pipe down on its own and
	 * can take tens of milliseconds before it answers (and swallows) the
	 * clear swap. That is not a crash: a real one is reported through the
	 * RTKit crash callback. Wait longer and carry on with the power-off
	 * either way, otherwise every later modeset fails with -EINVAL.
	 */
	ret = wait_for_completion_timeout(&cookie->done, msecs_to_jiffies(500));
	swap_id = cookie->swap_id;
	if (READ_ONCE(cookie->status))
		ret = 0;
	kref_put(&cookie->refcount, release_swap_cookie);
	if (ret <= 0)
		dev_warn(dcp->dev, "%s: clear swap timed out\n", __func__);
	else
		dev_dbg(dcp->dev, "%s: clear swap submitted: %u\n", __func__,
			swap_id);

	poff_cookie = kzalloc(sizeof(*poff_cookie), GFP_KERNEL);
	if (!poff_cookie)
		return;
	poff_cookie->status = ~0U;
	init_completion(&poff_cookie->done);
	kref_init(&poff_cookie->refcount);
	/* increase refcount to ensure the receiver has a reference */
	kref_get(&poff_cookie->refcount);

	if (DCP_HAS_CLIENT_TEARDOWN) {
		iomfb_abort_swaps_dcp(dcp, false, &abort_req,
					aborted_swaps_dcp_poff, poff_cookie);
	} else {
		dev_dbg_once(dcp->dev,
			     "power-off: skipping abort_swaps/last_client_close\n");
		last_client_closed_poff(dcp, NULL, poff_cookie);
	}
	ret = wait_for_completion_timeout(&poff_cookie->done,
					  msecs_to_jiffies(1000));

	if (ret == 0)
		dev_warn(dcp->dev, "setPowerState(0) timeout %u ms\n", 1000);
	else if (ret > 0) {
		dev_dbg(dcp->dev,
			"setPowerState(0) finished with %d ms to spare",
			jiffies_to_msecs(ret));
		if (dcp_power_stop_confirmed(poff_cookie))
			dcp_release_all_retained_framebuffers(dcp);
		else
			dev_warn(dcp->dev, "scanout stop was not confirmed\n");
	}

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

#if DCP_FW_VERSION(26, 0, 0) <= DCP_FW_VER
	if (dcp->hw.iomfb_method_profile != DCP_IOMFB_METHODS_H17G)
		return;
#endif

	cookie = kzalloc(sizeof(*cookie), GFP_KERNEL);
	if (!cookie)
		return;
	cookie->status = ~0U;
	init_completion(&cookie->done);
	kref_init(&cookie->refcount);
	/* increase refcount to ensure the receiver has a reference */
	kref_get(&cookie->refcount);

	if (DCP_HAS_CLIENT_TEARDOWN) {
		iomfb_abort_swaps_dcp(dcp, false, &req, aborted_swaps_dcp_sleep,
					cookie);
	} else {
		dev_dbg_once(dcp->dev,
			     "sleep: skipping abort_swaps/last_client_close\n");
		last_client_closed_sleep(dcp, NULL, cookie);
	}
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
	/* sampled before complete_all() lets dcp_poweron() clear it */
	bool poweron_hpd = READ_ONCE(dcp->typec_poweron_hpd);
	unsigned int action;

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
	 * Report firmware hotplug independently of the USB4 PHY.
	 * Reassigning lpdptxphy blanked eDP even with these callbacks ignored;
	 * suppressing connector notifications does not protect the panel.
	 * Mode probing still uses this DCP's firmware modes, and mode_valid
	 * rejects modes absent from that list. Do not synthesize a mode here.
	 */

	/*
	 * Same for the unplug a Type-C output reports after its CRTC was
	 * powered off with the cable still attached (see dcp_poweroff()).
	 */
	if (!(*connected) && READ_ONCE(dcp->typec_crtc_off) &&
	    READ_ONCE(dcp->typec_cable_connected)) {
		dev_dbg(dcp->dev, "cb_hotplug() ignoring unplug of powered-off Type-C output\n");
		dcp_mode_invalidate(&dcp->mode_state);
		schedule_work(&dcp->vblank_wq);
		return;
	}
	/* DCP lost a sink which is still attached, see dcp_typec_relink() */
	if (!(*connected) && dcp_typec_relink(dcp)) {
		dcp_mode_invalidate(&dcp->mode_state);
		schedule_work(&dcp->vblank_wq);
		return;
	}
	if (dcp_is_typec_output(dcp) && *connected && dcp->nr_modes)
		complete_all(&dcp->typec_iomfb_hpd_ready);

	/* A firmware HPD loss can bypass disconnected_hpd_event(), for
	 * example on the fixed output. Do not lend its capability to the next
	 * sink while that sink's DisplayAttributes are still arriving.
	 */
	if (!*connected)
		WRITE_ONCE(dcp->ext_backlight, false);

	if (dcp_external_hotplug(dcp, !!(*connected)))
		return;

	dcp_firmware_metadata_hotplug(dcp, !!(*connected));
	apple_connector_edid_set_live(connector, !!(*connected));
	action = dcp_mode_hotplug(&dcp->mode_state, !!(*connected),
				  connector ? &connector->connected : NULL);
	/*
	 * A Type-C sink can assert HPD only after a modeset has already failed,
	 * as a TV behind a DP-to-HDMI converter does when it wakes from standby.
	 * The connector state does not change then; re-apply the mode anyway.
	 *
	 * Not for the HPD dcp_poweron() waits for: the modeset of that commit
	 * has not started yet, rather than failed. Reporting the link as bad
	 * then makes wlroots replace the output, which powers the CRTC off and
	 * on again, and the next wake does the same, forever.
	 */
	if (*connected && dcp_is_typec_output(dcp) && !poweron_hpd &&
	    !READ_ONCE(dcp->mode_state.valid) &&
	    !READ_ONCE(dcp->mode_state.changing)) {
		dcp->swap_watchdog_retrains = 0;
		action |= DCP_HOTPLUG_NOTIFY;
	}
	dcp_handle_hotplug_actions(dcp, action);
}

static void
dcpep_cb_swap_complete_intent_gated(struct apple_dcp *dcp,
				    struct dcp_swap_complete_intent_gated *info)
{
	trace_iomfb_swap_complete_intent_gated(dcp, info->swap_id,
		info->width, info->height);
}

#if DCP_FW_VERSION(26, 0, 0) <= DCP_FW_VER
/*
 * An aborted H17P present never completes.  Finish it like a rejected one:
 * keep the displaced framebuffers for the next completed present and signal
 * the DRM event, so neither the queue nor userspace waits for a completion.
 */
static void dcp_present_aborted(struct apple_dcp *dcp, u32 swap_id)
{
	if (!dcp_present_abort_h17p(&dcp->present_state_h17p, swap_id)) {
		dev_warn_ratelimited(dcp->dev,
				     "abort for present %u, which is not in flight\n",
				     swap_id);
		return;
	}
	/* The submit reply has not arrived; it finishes the present. */
	if (dcp->present_state_h17p.pending)
		return;

	dev_warn_ratelimited(dcp->dev, "firmware aborted present %u\n", swap_id);
	/* No completion follows, so the swap watchdog must not wait for one. */
	dcp_swap_watchdog_complete(dcp);
	dcp_present_failed(dcp);
	if (dcp_present_retires(dcp)) {
		dcp_unarm_retained_framebuffers(dcp, swap_id);
		dcp_drm_crtc_vblank(dcp->crtc);
	}
}
#endif

static void
dcpep_cb_abort_swap_ap_gated(struct apple_dcp *dcp, u32 *swap_id)
{
	trace_iomfb_abort_swap_ap_gated(dcp, *swap_id);
#if DCP_FW_VERSION(26, 0, 0) <= DCP_FW_VER
	if (dcp->hw.iomfb_method_profile != DCP_IOMFB_METHODS_H17G)
		dcp_present_aborted(dcp, *swap_id);
#endif
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
	 * Firmware asks during start_ios_gated. On J313 the panel node is
	 * read first, so width_mm is already set and dcp_has_panel() is
	 * already true. Answering eDP as well does not change this return
	 * value. DCP 13.5 still logs that it could not find the backlight
	 * service, and writes still apply only after a modeset.
	 * External displays are not eDP.
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
	int ret;
	struct DCP_FW_NAME(dcp_swap_submit_resp) *resp = data;
	u32 swap_id = DCP_FW_UNION(dcp->swap).swap.swap_id;
	u32 status = resp ? resp->ret : ~0U;

	ret = dcp_present_submit(dcp, swap_id, !status);
	if (ret)
		status = ~0U;

	if (status) {
		/* An aborted present has already been reported. */
		if (ret != -ECANCELED)
			dev_err(dcp->dev, "swap failed! status %u\n", status);
		dcp_present_failed(dcp);
		if (dcp_present_retires(dcp)) {
			dcp_unarm_retained_framebuffers(dcp, swap_id);
			dcp_drm_crtc_vblank(dcp->crtc);
		}
		return;
	}
	dcp->swap_start = ktime_get();
	dcp->swap_submit_timestamp = arch_timer_read_counter();
	dcp_swap_watchdog_arm(dcp);
}

static void dcp_swap_started(struct apple_dcp *dcp, void *data, void *cookie)
{
	struct DCP_FW_NAME(dcp_swap_start_resp) *resp = data;

	if (!resp || resp->ret) {
		dev_err(dcp->dev, "swap_start was rejected\n");
		if (dcp_present_retires(dcp))
			dcp_drm_crtc_vblank(dcp->crtc);
		return;
	}
	if (!dcp_present_begin(dcp, resp->swap_id)) {
		if (dcp_present_retires(dcp))
			dcp_drm_crtc_vblank(dcp->crtc);
		return;
	}
	DCP_FW_UNION(dcp->swap).swap.swap_id = resp->swap_id;
	if (dcp_present_retires(dcp))
		dcp_arm_retained_framebuffers(dcp, resp->swap_id);

	trace_iomfb_swap_submit(dcp, resp->swap_id);
	dcp_h17p_prepare_swap(dcp);
	dcp_swap_submit(dcp, false, &DCP_FW_UNION(dcp->swap), dcp_swapped, NULL);
}

/* Helpers to modeset and swap, used to flush */
static void do_swap(struct apple_dcp *dcp, void *data, void *cookie)
{
	struct DCP_FW_NAME(dcp_swap_start_req) start_req = { 0 };
	struct DCP_FW_NAME(dcp_swap_submit_req) *req = &DCP_FW_UNION(dcp->swap);

	if (READ_ONCE(dcp->mode_state.valid) && dcp->connector &&
	    READ_ONCE(dcp->connector->connected)) {
		/* The CTM reply may arrive after a link loss. Consume the
		 * external level only here, once the swap can be submitted.
		 */
		if (!dcp_has_panel(dcp) &&
		    dcp->hw.iomfb_method_profile != DCP_IOMFB_METHODS_H17P_EXTERNAL &&
		    xchg(&dcp->brightness.update, false)) {
			s32 bl = dcp_ext_backlight_value(dcp);

			if (bl >= 0) {
				req->swap.bl_unk = 1;
				req->swap.bl_value = bl;
				req->swap.bl_power = 0x40;
			} else {
				/* DisplayAttributes may still be on their way. */
				smp_store_release(&dcp->brightness.update, true);
			}
		}
		dcp_swap_start(dcp, false, &start_req, dcp_swap_started, NULL);
	} else {
		dcp_drm_crtc_vblank(dcp->crtc);
	}
}

#if DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0)
/*
 * H17P expects A428 between set_matrix and swap_start, with 0x10000 (1.0 in
 * 16.16 fixed point), the same identity value the D006 reply carries.
 */
#if DCP_FW_VERSION(26, 0, 0) <= DCP_FW_VER
void iomfb_present_backlight_h17p(struct apple_dcp *dcp)
{
	do_swap(dcp, NULL, NULL);
}
#endif

static void dcp_post_swap_a428(struct apple_dcp *dcp, void *out, void *cookie)
{
	do_swap(dcp, out, cookie);
}

static void dcp_pre_swap_a428(struct apple_dcp *dcp, void *out, void *cookie)
{
	u32 identity = 0x10000;

	dcp_pipe_set_428(dcp, false, &identity, dcp_post_swap_a428, cookie);
}
#endif

static void complete_set_digital_out_mode(struct apple_dcp *dcp, void *data,
					  void *cookie)
{
	struct dcp_wait_cookie *wait = cookie;

	if (wait) {
		if (data) {
			WRITE_ONCE(wait->status, *(u32 *)data);
			WRITE_ONCE(wait->replied, true);
		}
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
	struct dcp_display_mode selected, *mode = &selected;
	struct dcp_wait_cookie *cookie;
	struct dcp_color_mode *cmode = NULL;
	int ret;

	if (!lookup_mode(dcp, &crtc_state->mode, mode)) {
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

	cookie->status = ~0U;
	init_completion(&cookie->done);
	kref_init(&cookie->refcount);
	/* increase refcount to ensure the receiver has a reference */
	kref_get(&cookie->refcount);

	dcp->swap_submit_timestamp = 0;

	if (DCP_INHERIT_BOOT_MODE &&
	    dcp->hw.iomfb_method_profile !=
	    DCP_IOMFB_METHODS_H17P_EXTERNAL) {
		dev_dbg(dcp->dev, "inheriting the bootloader's display mode\n");
		complete_set_digital_out_mode(dcp, NULL, cookie);
	} else if (mode->vrr &&
		   dcp->hw.iomfb_method_profile !=
		   DCP_IOMFB_METHODS_H17P_EXTERNAL)
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
	/*
	 * The external H17P display reports a link that does not come up
	 * through its DPTX fault callbacks.  What the 32-bit result of the
	 * mode set means is not established: a nonzero one is seen both for
	 * a mode set that failed and for one repeating the current mode.  Log
	 * it, and fail only a mode set that got no reply at all.
	 */
	if (dcp->hw.iomfb_method_profile == DCP_IOMFB_METHODS_H17P_EXTERNAL) {
		if (!READ_ONCE(cookie->replied)) {
			dev_err(dcp->dev, "set_digital_out_mode got no reply\n");
			kref_put(&cookie->refcount, release_wait_cookie);
			return -EIO;
		}
		if (READ_ONCE(cookie->status))
			dev_info(dcp->dev, "set_digital_out_mode returned %#x\n",
				 READ_ONCE(cookie->status));
	}
	kref_put(&cookie->refcount, release_wait_cookie);
	dcp->vrr_enabled = mode->vrr && crtc_state->vrr_enabled;

	return 0;
}

void DCP_FW_NAME(iomfb_flush)(struct apple_dcp *dcp, struct drm_crtc *crtc, struct drm_atomic_commit *commit)
{
	struct drm_plane *plane;
	struct drm_plane_state *new_state, *old_state;
	struct drm_crtc_state *crtc_state;
	struct DCP_FW_NAME(dcp_swap_submit_req) *req = &DCP_FW_UNION(dcp->swap);
	int plane_idx, l;
	int has_surface = 0;
	bool update_brightness;

	crtc_state = drm_atomic_get_new_crtc_state(commit, crtc);

	/* Reset all surfaces to defaults */
	memset(req, 0, sizeof(*req));
	for (l = 0; l < SWAP_SURFACES; l++)
		req->surf_null[l] = true;
#if DCP_FW_VER >= DCP_FW_VERSION(13, 2, 0)
#if DCP_FW_VER < DCP_FW_VERSION(26, 0, 0)
	for (l = 0; l < 5; l++)
		req->surf2_null[l] = true;
#endif
	req->unknown_pointer_null = true;
	req->unknown_output_null = true;
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

	for_each_oldnew_plane_in_state(commit, plane, old_state, new_state, plane_idx) {
		struct apple_plane_state *apple_state = to_apple_plane_state(new_state);
		struct apple_plane *apl_plane = to_apple_plane(plane);

		/* skip planes not for this crtc */
		if (old_state->crtc != crtc && new_state->crtc != crtc)
			continue;

		l = apl_plane->iomfb_surf;
		req->swap.swap_enabled |= BIT(l);

		if (!new_state->fb || !new_state->visible) {
			continue;
		}
		req->surf_null[l] = false;
		has_surface = 1;

		req->swap.src_rect[l] = apple_state->src_rect;
		req->swap.dst_rect[l] = apple_state->dst_rect;

#if DCP_FW_VER < DCP_FW_VERSION(26, 0, 0)
		req->swap.surf_ids[apl_plane->iomfb_surf] = plane->base.id;
#endif

		if (dcp->notch_height > 0)
			req->swap.dst_rect[l].y += dcp->notch_height;

		req->surf_iova[l] = apple_state->iova;
		req->surf[l].base = apple_state->surf;
#if DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0)
		req->surf[l].base.surface_id = 0;
#endif

		/* Use sRGB colorspace only for internal panels. External
		 * displays are expected to have EDID and user space can use
		 * the contained colorimetry information to provide native
		 * colors.
		 */
		if (dcp->connector_type == DRM_MODE_CONNECTOR_eDP &&
		    req->surf[l].base.colorspace == DCP_COLORSPACE_BG_SRGB)
			req->surf[l].base.colorspace = DCP_COLORSPACE_NATIVE;

#if DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0)
		/* H17P external SDR RGB needs an sRGB source tag. Native
		 * tagging washes out the same pixel values on this profile.
		 * Keep YUV, other source spaces and other profiles unchanged.
		 */
		if (dcp->hw.iomfb_method_profile ==
			    DCP_IOMFB_METHODS_H17P_EXTERNAL &&
		    !new_state->fb->format->is_yuv &&
		    req->surf[l].base.colorspace == DCP_COLORSPACE_NATIVE)
			req->surf[l].base.colorspace = DCP_COLORSPACE_BG_SRGB;
#endif
	}

	if (!has_surface && !crtc_state->color_mgmt_changed) {
		if (crtc_state->enable && crtc_state->active &&
		    !crtc_state->planes_changed) {
			schedule_work(&dcp->vblank_wq);
			return;
		}

		/* Set black background */
		req->swap.swap_enabled |= IOMFB_SET_BACKGROUND;
		req->swap.bg_color = 0xFF000000;
		req->clear = 1;
	}

	if (has_surface && (dcp->use_timestamps || dcp->vrr_enabled)) {
		u64 submit_timestamp = dcp->swap_submit_timestamp;
		u64 timestamp = arch_timer_read_counter();

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

	/* These fields should be set together */
	req->swap.swap_completed = req->swap.swap_enabled;

	/* update brightness if changed */
	update_brightness = dcp_has_panel(dcp) && dcp->brightness.update;
#if DCP_FW_VERSION(26, 0, 0) <= DCP_FW_VER
	update_brightness = update_brightness &&
			    dcp->hw.iomfb_method_profile == DCP_IOMFB_METHODS_H17G;
#endif
	if (update_brightness) {
#if DCP_FW_LAYOUT_14_7(DCP_FW_VER)
		req->swap.bl_update = 1;
#else
		req->swap.bl_unk = 1;
#endif
		req->swap.bl_value = dcp->brightness.dac;
		req->swap.bl_power = 0x40;
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
	bool result;

	/* No reply: the DCP is marked crashed, so only end the start-up wait. */
	if (!out) {
		complete(&dcp->start_done);
		return;
	}

	/*
	 * The reply is a one-byte boolean.  The firmware leaves the rest of
	 * the 32-bit slot alone, and dcp_push() fills that with 0xff.
	 */
	result = *(u8 *)out;

	dev_info(dcp->dev, "DCP is_main_display: %d\n", result);

	dcp->main_display = result;

	connector = dcp->connector;
	if (connector) {
		connector->connected = dcp->nr_modes > 0;
		dcp_queue_hotplug(connector);
	}

	dcp->active = true;
	complete(&dcp->start_done);
}

static void init_3(struct apple_dcp *dcp, void *out, void *cookie)
{
	dcp_is_main_display(dcp, false, res_is_main_display, NULL);
}

#if DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0)
/*
 * The client-open sequence H17P firmware expects before the first swap, in
 * H17P method numbers (dcp_push() applies the H17G aliases):
 *
 *   A445 A411 A445 A415 A450 A031 x9 A414 A478 A457 A474 A478 A476 A478 A412
 *
 * A412 is not in the table: its result feeds res_is_main_display(), so it
 * stays on the existing path.
 */
enum dcp_client_step_kind {
	DCP_STEP_A445, DCP_STEP_A411, DCP_STEP_A415, DCP_STEP_A450,
	DCP_STEP_A031, DCP_STEP_A414, DCP_STEP_A478, DCP_STEP_A457,
	DCP_STEP_A474, DCP_STEP_A476,
};

static const u8 dcp_client_open_seq[] = {
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
		 * Nine cumulative states: each call repeats the previous set
		 * of notification clients and adds one, so state n lists n
		 * clients.
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
		struct dcp_pipe_cfg_031 cfg = {};
		unsigned int step = dcp->a031_step;
		unsigned int i;

		if (step >= ARRAY_SIZE(notify_clients))
			step = ARRAY_SIZE(notify_clients) - 1;
		for (i = 0; i < step; i++)
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
		 * A476 takes a single u32, which the firmware expects to be 1.
		 * Its name is not known for certain: the shape (in 4, out 0)
		 * fits both set_dcp_clamshellstate() and
		 * set_has_frame_swap_function().
		 */
		u32 state = 1;

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

static void init_1(struct apple_dcp *dcp, void *out, void *cookie)
{
	/* every client-open sequence starts from the empty A031 state */
	dcp->a031_step = 0;
	dcp_client_open_run(dcp, NULL, (void *)0UL);
}
#else
static void init_2(struct apple_dcp *dcp, void *out, void *cookie)
{
	dcp_first_client_open(dcp, false, init_3, NULL);
}

static void init_1(struct apple_dcp *dcp, void *out, void *cookie)
{
	u32 val = 0;
	dcp_enable_disable_video_power_savings(dcp, false, &val, init_2, NULL);
}
#endif

static void dcp_started(struct apple_dcp *dcp, void *data, void *cookie)
{
	struct iomfb_get_color_remap_mode_req color_remap =
		(struct iomfb_get_color_remap_mode_req){
			.mode = 6,
		};

	dev_info(dcp->dev, "DCP booted\n");

	iomfb_get_color_remap_mode(dcp, false, &color_remap, init_1, cookie);
}

void DCP_FW_NAME(iomfb_shutdown)(struct apple_dcp *dcp)
{
	struct DCP_FW_NAME(dcp_set_power_state_req) req = {
		/* defaults are ok */
	};
	struct dcp_wait_cookie *cookie;
	int ret;

#if DCP_FW_VERSION(26, 0, 0) <= DCP_FW_VER
	if (dcp->hw.iomfb_method_profile != DCP_IOMFB_METHODS_H17G) {
		iomfb_queue_stop(dcp);
		return;
	}
#endif

	cookie = kzalloc_obj(*cookie);
	if (!cookie)
		return;
	cookie->status = ~0U;
	init_completion(&cookie->done);
	kref_init(&cookie->refcount);
	kref_get(&cookie->refcount);
	dcp_set_power_state(dcp, DCP_POWER_OOB, &req, complete_set_powerstate, cookie);
	ret = wait_for_completion_timeout(&cookie->done, msecs_to_jiffies(1000));
	if (ret > 0 && dcp_power_stop_confirmed(cookie))
		dcp_release_all_retained_framebuffers(dcp);
	else
		dev_warn(dcp->dev, "shutdown scanout stop was not confirmed\n");
	kref_put(&cookie->refcount, release_wait_cookie);
}
