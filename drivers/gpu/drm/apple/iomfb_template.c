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
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/overflow.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/unaligned.h>

#include <drm/drm_color_mgmt.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_vblank.h>

#include "dcp.h"
#include "dcp-internal.h"
#include "dcp-edr.h"
#include "iomfb.h"
#include "iomfb_internal.h"
#include "iomfb-dfb.h"
#include "parser.h"
#include "av.h"
#include "dcp-refresh.h"
#include "dcp-property.h"
#include "trace.h"
#include "version_utils.h"

/* Register defines used in bandwidth setup structure */
#define REG_DOORBELL_BIT(idx) (2 + (idx))

static_assert(offsetof(struct DCP_FW_NAME(dcp_swap), timestamp[6]) == 0x30);
static_assert(offsetof(struct DCP_FW_NAME(dcp_swap), flags1) == 0x40);

/* The queued swap of this instance. */
#if DCP_FW_DESKTOP_26(DCP_FW_VER)
#define DCP_FW_SWAP(dcp) ((dcp)->desktop->swap)
#else
#define DCP_FW_SWAP(dcp) DCP_FW_UNION((dcp)->swap)
#endif

#if DCP_FW_DESKTOP_26(DCP_FW_VER)
enum dcp_power_transition {
	DCP_POWER_TRANSITION_NONE,
	DCP_POWER_TRANSITION_DISPLAY_OFF,
	DCP_POWER_TRANSITION_RESTARTABLE_SLEEP,
	DCP_POWER_TRANSITION_SHUTDOWN,
};

/* The records of this instance that differ from the shared ones. */
#define dcp_bool_arg_t			struct dcp_bool_arg
#define dcp_abort_swaps_req_t		struct iomfb_abort_swaps_dcp_req_v26_6
#define dcp_read_edt_data_resp_t	struct dcp_read_edt_data_resp_v26_6
#define dcp_swap_intent_t		struct dcp_swap_complete_intent_gated_v26_6

/*
 * set_power_state (A473) carries four independent transition flags.  The
 * firmware's own sleep sets the first and the last, its shutdown the second;
 * both then pass no result.
 */
static struct DCP_FW_NAME(dcp_set_power_state_req)
dcp_power_state_req(u64 state, enum dcp_power_transition transition)
{
	struct DCP_FW_NAME(dcp_set_power_state_req) req = {
		.unklong = state,
		.padding = { 0xaa, 0xaa, 0xaa },
	};

	switch (transition) {
	case DCP_POWER_TRANSITION_RESTARTABLE_SLEEP:
		req.flags[0] = 1;
		req.flags[3] = 1;
		req.result_null = 1;
		break;
	case DCP_POWER_TRANSITION_SHUTDOWN:
		req.flags[1] = 1;
		req.result_null = 1;
		break;
	case DCP_POWER_TRANSITION_NONE:
	case DCP_POWER_TRANSITION_DISPLAY_OFF:
		break;
	}

	return req;
}

static dcp_bool_arg_t dcp_bool_arg(bool value)
{
	return (dcp_bool_arg_t) {
		.value = value,
		.padding = { 0xaa, 0xaa, 0xaa },
	};
}

/*
 * Brightness and display-off swaps carry the level in nits, as a binary64.
 * Every u32 is exactly representable, so no kernel floating point is needed.
 */
static void dcp_swap_set_brightness(struct DCP_FW_NAME(dcp_swap) *swap, u32 nits)
{
	u64 bits = 0;

	if (nits) {
		unsigned int exponent = fls(nits) - 1;

		bits = (u64)(exponent + 1023) << 52;
		bits |= ((u64)nits << (52 - exponent)) & GENMASK_ULL(51, 0);
	}

	swap->bl_update = 1;
	put_unaligned_le64(bits, &swap->bl_nits);
}

/*
 * EDR on a desktop 26.x panel, as macOS drives it.  With applied headroom H
 * (x1000 below), a swap asks for brightness SDR white x H, display EDR
 * scale 1 / H and, while the frame uses EDR, headroom used H; the firmware
 * shows a PQ surface at (PQ nits / 100) x the scale x the panel level, so
 * PQ 100 lands at SDR white and highlights reach the panel peak.  H rises
 * in steps linear in stops, one per swap, about 1 s when EDR starts, with
 * a brightness limit of 1.3 x the brightness in the middle of the ramp.
 * H never exceeds the panel's potential headroom from the ADT, as on
 * macOS: below the SDR white at which the peak needs more, H stays at that
 * ceiling and the panel level follows SDR white.
 * When EDR ends, headroom used drops to 1.0 and H returns to 1.0.
 * Unchanged values are left out: the firmware keeps them.
 */

#define dcp_swap_set_ratio(swap, field, flag, num, den)				\
	do {								\
		(swap)->bl_flags[flag] = 1;				\
		put_unaligned_le64(dcp_binary64_ratio(num, den),	\
				   &(swap)->field);			\
	} while (0)

/*
 * @nits is the level a swap carries itself (@carries: a brightness change
 * or power on), else the current one stands.  A swap that turns the panel
 * off keeps its zero.  After a power off the ramp starts again from 1.0:
 * asking for the peak at once does not take after a power on, on macOS
 * either.
 */
static void dcp_edr_prepare(struct apple_dcp *dcp, struct DCP_FW_NAME(dcp_swap) *swap,
			    bool carries, u32 nits)
{
	u32 peak = READ_ONCE(dcp->panel.peak_nits);
	u32 sdr = carries ? nits : dcp_backlight_desktop_nits(dcp);
	u32 cur = dcp->edr_hm ?: DCP_EDR_UNITY, target = DCP_EDR_UNITY, next, bri;
	u32 prev_sdr = dcp->edr_sdr;
	bool restart = dcp->edr_restart;

	dcp->edr_restart = false;
	if (!dcp->output_edr && !dcp->edr_hm)
		return;		/* no EDR, and none to wind down */
	if (!sdr)
		return;
	if (dcp->output_edr)
		target = dcp_edr_target(peak, sdr, dcp->panel.edr_ceiling);
	if (restart && dcp->output_edr) {
		cur = DCP_EDR_UNITY;
		prev_sdr = sdr;
	}
	dcp->edr_sdr = sdr;
	if (cur == target && !carries && !restart && dcp->edr_sent == dcp->output_edr)
		return;

	/*
	 * Up in steps; down at once.  macOS ramps down too, but its compositor
	 * keeps presenting until the ramp ends.  Here a still desktop may send
	 * no further swap, and a ramp stuck partway leaves SDR content dimmed
	 * by the scale (headroom used is already 1.0).
	 */
	next = dcp_edr_next(cur, target, prev_sdr, sdr);

	bri = (u64)sdr * next / DCP_EDR_UNITY;
	dcp_swap_set_brightness(swap, bri);
	dcp_swap_set_ratio(swap, edr_scale, 2, 1000, next);
	if (dcp->output_edr) {
		dcp_swap_set_ratio(swap, edr_headroom, 4, next, 1000);
		dcp->edr_sent = true;
	} else if (dcp->edr_sent) {
		dcp_swap_set_ratio(swap, edr_headroom, 4, 1, 1);
		dcp->edr_sent = false;
	}
	if (next != target && next != cur) {
		swap->bl_flags[1] = 1;
		put_unaligned_le64(dcp_binary64_ratio(bri * 13, 10), &swap->bl_limit);
	}
	/* Back at 1.0 without EDR: leave the firmware's values alone again. */
	dcp->edr_hm = (!dcp->output_edr && next == 1000) ? 0 : next;
}

/*
 * The firmware's own client passes its IOUserClient pointer as an opaque
 * client identity; this one passes a cookie chosen at probe.
 */
static struct DCP_FW_NAME(dcp_swap_start_req)
dcp_swap_start_request(struct apple_dcp *dcp)
{
	return (struct DCP_FW_NAME(dcp_swap_start_req)) {
		.client = dcp->desktop->client,
		.padding = { 0xaa, 0xaa, 0xaa },
	};
}

static dcp_abort_swaps_req_t dcp_abort_swaps_request(struct apple_dcp *dcp)
{
	return (dcp_abort_swaps_req_t) { .client = dcp->desktop->client };
}

/*
 * A414 and A474, with these arguments, bracket the display power changes.
 * The request is five zero bytes followed by three bytes of 0xaa.
 */
struct dcp_wake_state_req_v26_6 {
	u8 data[8];
} __packed;

static const struct dcp_wake_state_req_v26_6 dcp_wake_state_req = {
	.data = { 0, 0, 0, 0, 0, 0xaa, 0xaa, 0xaa },
};

/* Methods without a slot in the shared method table. */
static const struct dcp_method_entry dcp_get_mode_info_method = {
	.name = "dcpep_get_mode_info",
	.tag = { 'A', '3', '6', '4' },
};

static const struct dcp_method_entry dcp_get_gamma_table_method = {
	.name = "iomfbep_get_gamma_table",
	.tag = { 'A', '4', '2', '0' },
};

static const struct dcp_method_entry dcp_set_gamma_table_method = {
	.name = "iomfbep_set_gamma_table",
	.tag = { 'A', '4', '2', '1' },
};

/* The peak luminance of the panel, in 16.16 nits. */
static const struct dcp_method_entry dcp_max_panel_brightness_method = {
	.name = "iomfbep_max_panel_brightness",
	.tag = { 'A', '1', '2', '8' },
};

static_assert(sizeof(struct DCP_FW_NAME(dcp_swap)) == 0x588);
static_assert(offsetof(struct DCP_FW_NAME(dcp_swap), swap_id) == 0x98);
static_assert(offsetof(struct DCP_FW_NAME(dcp_swap), bl_update) == 0x354);
static_assert(offsetof(struct DCP_FW_NAME(dcp_swap), bl_flags) == 0x358);
static_assert(offsetof(struct DCP_FW_NAME(dcp_swap), bl_nits) == 0x35e);
static_assert(offsetof(struct DCP_FW_NAME(dcp_swap), bl_limit) == 0x376);
static_assert(offsetof(struct DCP_FW_NAME(dcp_swap), edr_scale) == 0x37e);
static_assert(offsetof(struct DCP_FW_NAME(dcp_swap), edr_headroom) == 0x3e6);
static_assert(offsetof(struct DCP_FW_NAME(dcp_swap), presents_surface) == 0x51e);
static_assert(offsetof(struct DCP_FW_NAME(dcp_swap), has_completion) == 0x570);
static_assert(sizeof(struct DCP_FW_NAME(dcp_surface)) == 0x22c);
static_assert(sizeof(struct DCP_FW_NAME(dcp_swap_submit_req)) == 0x1bd8);
static_assert(sizeof(struct DCP_FW_NAME(dcp_swap_submit_resp)) == 0xc);
static_assert(sizeof(struct DCP_FW_NAME(dc_swap_complete_resp)) == 0x730);
static_assert(sizeof(struct DCP_FW_NAME(dcp_map_reg_req)) == 0x10);
static_assert(sizeof(struct dcp_set_parameter_dcp) == 0x28);
static_assert(sizeof(struct dcp_get_mode_info_req_v26_6) == 0x7c);
static_assert(sizeof(struct dcp_get_mode_info_resp_v26_6) == 0x74);
static_assert(sizeof(struct iomfb_abort_swaps_dcp_req_v26_6) == 0x8);
static_assert(sizeof(struct iomfb_abort_swaps_dcp_resp_v26_6) == 0x4);
static_assert(sizeof(struct dcp_swap_complete_intent_gated_v26_6) == 0x14);
static_assert(sizeof(struct dcp_read_edt_data_resp_v26_6) == 0x24);
static_assert(sizeof(struct dcp_hotplug_req_v26_6) == 0x58);
static_assert(sizeof(struct dcp_hotplug_resp_v26_6) == 0x4c);
static_assert(sizeof(struct dcp_bool_arg) == 0x4);
static_assert(sizeof(struct dcp_wake_state_req_v26_6) == 0x8);
static_assert(sizeof(struct iomfb_gamma_table_req) == 3088);
static_assert(sizeof(struct iomfb_gamma_table_resp) == 3088);
static_assert(sizeof(struct dcp_set_frame_sync_props_req_v26_6_desktop) == 0x54);
static_assert(sizeof(struct dcp_set_frame_sync_props_resp_v26_6_desktop) == 0x50);
#else
#define dcp_bool_arg_t			u32
#define dcp_abort_swaps_req_t		struct iomfb_abort_swaps_dcp_req
#define dcp_read_edt_data_resp_t	struct dcp_read_edt_data_resp
#define dcp_swap_intent_t		struct dcp_swap_complete_intent_gated
#endif

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
#if DCP_FW_DESKTOP_26(DCP_FW_VER)
DCP_THUNK_INOUT(iomfb_abort_swaps_dcp, iomfbep_abort_swaps_dcp,
		struct iomfb_abort_swaps_dcp_req_v26_6,
		struct iomfb_abort_swaps_dcp_resp_v26_6);
/* The A414/A474 slots of the shared table, see dcp_wake_state_req. */
DCP_THUNK_INOUT(dcp_prepare_wake, dcpep_pipe_cfg_414,
		struct dcp_wake_state_req_v26_6, u64);
DCP_THUNK_OUT(dcp_finish_wake, dcpep_pipe_query_474, u32);
#else
IOMFB_THUNK_INOUT(abort_swaps_dcp);
#endif

#if (DCP_FW_H17(DCP_FW_VER) && DCP_FW_VER >= DCP_FW_VERSION(27, 0, 0))
static u64 __maybe_unused dcp_swap_time(void);

/*
 * macOS stamps every external swap, adaptive sync or not, in the firmware's
 * 24 MHz continuous time: word 1 at the swap start, words 0, 4 and 6 at the
 * submission. The panel keeps its own pacing timestamps.
 */
static bool dcp_external_swap_stamps(struct apple_dcp *dcp)
{
	return !dcp->main_display && !dcp_has_panel(dcp);
}

static void dcp_external_stamp_submit(struct apple_dcp *dcp, struct dcp_swap_v27 *swap)
{
	if (dcp_external_swap_stamps(dcp) && swap->timestamp[1])
		swap->timestamp[0] = swap->timestamp[4] = swap->timestamp[6] = dcp_swap_time();
}

/*
 * EDR on a firmware 27 panel, as macOS 27 drives it (a DCP trace on the M5
 * Pro): its swaps carry the desktop 26.x brightness block, at the same
 * offsets.  With headroom H a swap asks for brightness SDR white x H, EDR
 * scale 1 / H and, while the frame uses EDR, headroom used H.  macOS also
 * sends the brightness limit, 1.3 x the brightness, whenever that limit is
 * above 400 nits.  H follows macOS's ramp in time, up and down
 * (dcp_edr_next_timed()); short of its target, a still frame gets
 * brightness-only presents to carry it on (iomfb_edr_tick()).  @carries:
 * the swap already carries a brightness change; @sdr: its SDR white.
 */
static void dcp_edr_prepare_v27(struct apple_dcp *dcp, struct dcp_swap_v27 *swap,
				bool carries, u32 sdr)
{
	u32 cur = dcp->edr_hm ?: DCP_EDR_UNITY, target = DCP_EDR_UNITY, cap, next;
	u32 prev_sdr = dcp->edr_sdr;
	bool restart = dcp->edr_restart;
	ktime_t now = ktime_get();
	s64 us;
	u64 bri;

	dcp->edr_restart = false;
	WRITE_ONCE(dcp->edr_ramping, false);
	if ((!dcp->output_edr && !dcp->edr_hm) || !sdr)
		return;
	cap = dcp_edr_target(READ_ONCE(dcp->panel.peak_nits), sdr, dcp->panel.edr_ceiling);
	if (dcp->output_edr)
		target = cap;
	if (restart && dcp->output_edr) {
		cur = DCP_EDR_UNITY;
		prev_sdr = sdr;
	}
	dcp->edr_sdr = sdr;
	if (cur == target && !carries && !restart && dcp->edr_sent == dcp->output_edr)
		return;

	/* A ramp that starts, or follows a pause, moves one holdoff's worth. */
	us = clamp_t(s64, ktime_us_delta(now, dcp->edr_step), 0, DCP_EDR_HOLDOFF_US);
	next = dcp_edr_next_timed(cur, target, cap, prev_sdr, sdr, us);
	dcp->edr_step = now;
	bri = (u64)sdr * next / DCP_EDR_UNITY;
	swap->brightness_update = 1;
	swap->brightness_nits = dcp_binary64_ratio(bri, 1);
	swap->bl_flags[2] = 1;
	swap->edr_scale = dcp_binary64_ratio(DCP_EDR_UNITY, next);
	/* macOS sends the headroom on the way down too. */
	if (dcp->output_edr || next != DCP_EDR_UNITY) {
		swap->bl_flags[4] = 1;
		swap->edr_headroom = dcp_binary64_ratio(next, DCP_EDR_UNITY);
		dcp->edr_sent = true;
	} else if (dcp->edr_sent) {
		swap->bl_flags[4] = 1;
		swap->edr_headroom = dcp_binary64_ratio(1, 1);
		dcp->edr_sent = false;
	}
	if (bri * 13 > 4000) {
		swap->bl_flags[1] = 1;
		swap->bl_limit = dcp_binary64_ratio(bri * 13, 10);
	}
	/* Back at 1.0 without EDR: leave the firmware's values alone again. */
	dcp->edr_hm = (!dcp->output_edr && next == DCP_EDR_UNITY) ? 0 : next;
	if (next != target) {
		WRITE_ONCE(dcp->edr_ramping, true);
		mod_delayed_work(system_dfl_wq, &dcp->iomfb.edr_tick,
				 usecs_to_jiffies(iomfb_present_brightness_only_h17p(dcp) ?
						  DCP_EDR_TICK_US : DCP_EDR_HOLDOFF_US));
	}
}
#endif

#if DCP_FW_H17(DCP_FW_VER)
static void dcp_swap_submit(struct apple_dcp *dcp, bool oob,
			    struct dcp_swap_submit_req_h17p *request,
			    dcp_callback_t cb, void *cookie)
{
	const void *data = request;

#if DCP_FW_H17(DCP_FW_VER) && DCP_FW_VER < DCP_FW_VERSION(27, 0, 0)
	if (dcp_is_ext26(dcp)) {
		if (ext26_encode(dcp, request)) {
			WRITE_ONCE(dcp->crashed, true);
			if (cb)
				cb(dcp, NULL, cookie);
			return;
		}
		dcp_push(dcp, oob, &dcp_methods[dcpep_swap_submit], 0x1bd8, 12,
			 dcp->ext26_present, cb, cookie);
		return;
	}

#endif

	if (dcp->hw.iomfb_method_profile != DCP_IOMFB_METHODS_H17G) {
		iomfb_serialize_present_h17p(&dcp->present_h17p, request);
		if (!iomfb_apply_backlight_h17p(dcp, request, &dcp->present_h17p)) {
			if (cb)
				cb(dcp, NULL, cookie);
			return;
		}
#if (DCP_FW_H17(DCP_FW_VER) && DCP_FW_VER >= DCP_FW_VERSION(27, 0, 0))
		if (dcp->v27_a407) {
			/* The statements below on the A407 record. */
			iomfb_serialize_present_v27_a407(&dcp->present_v27_a407, request);
			dcp->present_v27_a407.swap.brightness_update =
				dcp->present_h17p.swap[0x354] & 1;
			dcp->present_v27_a407.swap.brightness_nits =
				get_unaligned_le64(dcp->present_h17p.swap + 0x35e);
			dcp_external_stamp_submit(dcp, &dcp->present_v27_a407.swap);
			if (dcp_has_panel(dcp) && iomfb_present_has_surface_h17p(request))
				dcp_edr_prepare_v27(dcp, &dcp->present_v27_a407.swap,
						    dcp->present_v27_a407.swap.brightness_update,
						    dcp->edr_present_sdr);
			dcp_push(dcp, oob, &dcp_methods[dcpep_swap_submit],
				 sizeof(struct dcp_swap_submit_req_v27_a407),
				 sizeof(struct dcp_swap_submit_resp_h17p),
				 &dcp->present_v27_a407, cb, cookie);
			return;
		}
		iomfb_serialize_present_v27(&dcp->present_v27, request);
		dcp->present_v27.swap.brightness_update =
			dcp->present_h17p.swap[0x354] & 1;
		dcp->present_v27.swap.brightness_nits =
			get_unaligned_le64(dcp->present_h17p.swap + 0x35e);
		dcp_external_stamp_submit(dcp, &dcp->present_v27.swap);
		/* A swap with no surface of its own carries no EDR step either. */
		if (dcp_has_panel(dcp) && iomfb_present_has_surface_h17p(request))
			dcp_edr_prepare_v27(dcp, &dcp->present_v27.swap,
					    dcp->present_v27.swap.brightness_update,
					    dcp->edr_present_sdr);
		data = &dcp->present_v27;
#else
		data = &dcp->present_h17p;
#endif
	}

	dcp_push(dcp, oob, &dcp_methods[dcpep_swap_submit],
#if (DCP_FW_H17(DCP_FW_VER) && DCP_FW_VER >= DCP_FW_VERSION(27, 0, 0))
		 sizeof(struct dcp_swap_submit_req_v27),
#else
		 sizeof(*request),
#endif
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
#if DCP_FW_H17(DCP_FW_VER)
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
#if DCP_FW_H17(DCP_FW_VER)
#define DCP_INHERIT_BOOT_MODE	(DCP_FW_VER < DCP_FW_VERSION(27, 0, 0))
#else
#define DCP_INHERIT_BOOT_MODE	false
#endif

/*
 * Firmware 27 changes a fixed internal panel's picture through the swap
 * geometry; the panel iBoot lit keeps its timing.  J815 rejects
 * set_digital_out_mode for its single timing (0x8000000b) and never scans out
 * afterwards, so keep the boot mode there (appledrm.fw27_panel_boot_mode).
 */
static bool __maybe_unused DCP_FW_NAME(dcp_fw27_keep_panel_mode)(struct apple_dcp *dcp)
{
#if (DCP_FW_H17(DCP_FW_VER) && DCP_FW_VER >= DCP_FW_VERSION(27, 0, 0))
	int policy = READ_ONCE(fw27_panel_boot_mode);

	if (policy == 0 || !dcp->main_display || !dcp_has_panel(dcp))
		return false;
	return policy > 0 || READ_ONCE(dcp->nr_modes) == 1;
#else
	return false;
#endif
}

/*
 * abort_swaps_dcp and last_client_close have no verified H17P method numbers,
 * and the H17P table leaves them out.  Rather than send the firmware a record
 * it would decode as some other call, power-off and sleep skip both and go
 * straight to set_power_state, whose number and sizes are known.  Power-off
 * still sends its clear swap first.
 */
#if DCP_FW_H17(DCP_FW_VER)
#define DCP_HAS_CLIENT_TEARDOWN	((DCP_FW_H17(DCP_FW_VER) && DCP_FW_VER >= DCP_FW_VERSION(27, 0, 0)))
#else
#define DCP_HAS_CLIENT_TEARDOWN	true
#endif

DCP_THUNK_INOUT(dcp_set_digital_out_mode, dcpep_set_digital_out_mode,
		struct dcp_set_digital_out_mode_req, u32);

DCP_THUNK_INOUT(dcp_set_display_device, dcpep_set_display_device, u32, u32);

DCP_THUNK_OUT(dcp_set_display_refresh_properties,
	      dcpep_set_display_refresh_properties, u32);

#if DCP_FW_VER >= DCP_FW_VERSION(13, 2, 0)
DCP_THUNK_INOUT(dcp_late_init_signal, dcpep_late_init_signal, dcp_bool_arg_t,
		u32);
#else
DCP_THUNK_OUT(dcp_late_init_signal, dcpep_late_init_signal, u32);
#endif
DCP_THUNK_IN(dcp_flush_supports_power, dcpep_flush_supports_power,
	     dcp_bool_arg_t);
DCP_THUNK_OUT(dcp_create_default_fb, dcpep_create_default_fb, u32);
DCP_THUNK_OUT(dcp_start_signal, dcpep_start_signal, u32);
DCP_THUNK_VOID(dcp_setup_video_limits, dcpep_setup_video_limits);
DCP_THUNK_VOID(dcp_set_create_dfb, dcpep_set_create_dfb);

#if DCP_FW_H17(DCP_FW_VER)
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
/* Built for 27.0 too, which still dispatches it as A500; the sleep checks the build has it. */
#if DCP_FW_VER <= DCP_FW_VERSION(27, 0, 0)
DCP_THUNK_IN(dcp_unnamed_500, dcpep_unnamed_500, u32);
#endif
#endif
DCP_THUNK_INOUT(dcp_set_brightness_correction, iomfbep_set_brightness_correction, u32, u32);

DCP_THUNK_VOID(dcp_first_client_open, dcpep_first_client_open);

DCP_THUNK_INOUT(dcp_set_parameter_dcp, dcpep_set_parameter_dcp,
		struct dcp_set_parameter_dcp, u32);

DCP_THUNK_INOUT(dcp_enable_disable_video_power_savings,
		dcpep_enable_disable_video_power_savings, u32, int);

#if !DCP_FW_DESKTOP_26(DCP_FW_VER)
static void dcp_external_hint_done(struct apple_dcp *dcp, void *out, void *cookie)
{
	struct dcp_wait_cookie *wait = cookie;

	if (out) {
		WRITE_ONCE(wait->status, *(u32 *)out);
		WRITE_ONCE(wait->replied, true);
	}
	complete(&wait->done);
	kref_put(&wait->refcount, release_wait_cookie);
}

/*
 * After every mode set on an external display macOS sends IOMFB
 * parameters 3 and 6 (16.16): 3 at 1.0, and 6 at 1.0 for an SDR colour
 * mode or 2.0 for a PQ one.  Send the same, after the mode set.  Only the
 * H17P external path does so for now: the desktop 26.x external processors
 * have not been checked with an HDR display yet.
 */
static void dcp_external_hdr_hints(struct apple_dcp *dcp, bool pq)
{
	struct iomfb_property hints[2];
	unsigned int i;

	iomfb_external_hdr_hint_values(pq, hints);

	for (i = 0; i < ARRAY_SIZE(hints); i++) {
		struct dcp_set_parameter_dcp param = {
			.param = hints[i].id,
			.value = { hints[i].value },
			.count = 1,
		};
		struct dcp_wait_cookie *cookie;
		unsigned long left;

		cookie = kzalloc(sizeof(*cookie), GFP_KERNEL);
		if (!cookie)
			return;
		init_completion(&cookie->done);
		kref_init(&cookie->refcount);
		kref_get(&cookie->refcount);
		dcp_set_parameter_dcp(dcp, false, &param, dcp_external_hint_done,
				      cookie);
		left = wait_for_completion_timeout(&cookie->done,
						   msecs_to_jiffies(500));
		if (!left || !READ_ONCE(cookie->replied) ||
		    READ_ONCE(cookie->status))
			dev_warn(dcp->dev,
				 "IOMFB parameter %u = %#x not taken (%s %u)\n",
				 param.param, param.value[0],
				 left ? "status" : "timeout",
				 READ_ONCE(cookie->status));
		else
			dev_dbg(dcp->dev, "IOMFB parameter %u = %#x\n",
				param.param, param.value[0]);
		kref_put(&cookie->refcount, release_wait_cookie);
	}
}
#endif

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

static void dcp_report_swap_complete(struct apple_dcp *dcp, u32 id, ktime_t now)
{
	bool retire = true;
	if (dcp_is_ext26(dcp)) {
		if (iomfb_ext26_control_complete(dcp, id, false))
			return;
		if (id == dcp->last_swap_id)
			return;
		if (dcp->present_state_h17p.pending && !dcp->present_state_h17p.accepted &&
		    id == dcp->present_state_h17p.swap_id) {
			dcp->ext26_early_complete = true;
			dcp->ext26_early_id = id;
			dcp->ext26_early_ns = dcp->ext26_rx_ns;
			return;
		}
	}


#if DCP_FW_DESKTOP_26(DCP_FW_VER)
	if (dcp_peer_swap_complete_26_6(dcp, id))
		return;
#endif
#if DCP_FW_H17(DCP_FW_VER)
	if (dcp->hw.iomfb_method_profile != DCP_IOMFB_METHODS_H17G &&
	    !dcp_present_complete_h17p(&dcp->present_state_h17p, id)) {
		/*
		 * Abort semantics are not captured on H17P.  If an aborted
		 * present completes after all, its framebuffers were already
		 * unarmed and its event signalled; record it and carry on.
		 */
		if (dcp_present_was_aborted_h17p(&dcp->present_state_h17p,
						 id)) {
			dev_warn_ratelimited(dcp->dev,
					     "completion for aborted present %u ignored\n",
					     id);
			return;
		}
		/*
		 * The swap watchdog retired it: its framebuffers stay
		 * referenced until a later present completes, and a present
		 * in flight now is a different one.
		 */
		if (dcp_present_was_retired_h17p(&dcp->present_state_h17p,
						 id)) {
			dev_warn_ratelimited(dcp->dev,
					     "completion for retired present %u ignored\n",
					     id);
			return;
		}
		dev_err(dcp->dev, "unexpected present completion %u\n", id);
		dcp->crashed = true;
		return;
	}
	if (dcp->hw.iomfb_method_profile != DCP_IOMFB_METHODS_H17G)
		retire = iomfb_present_complete_h17p(dcp);
#endif
#if DCP_FW_DESKTOP_26(DCP_FW_VER)
	/* Late or duplicate completions cannot retire the replacement's flip. */
	if (!dcp_desktop_present_complete(dcp, id))
		return;
	dcp_backlight_desktop_finish(dcp, id, false, true);
#else
	/* Nor can a lost flip's on firmware that does not track its presents. */
	if ((!DCP_FW_H17(DCP_FW_VER) ||
	     dcp->hw.iomfb_method_profile == DCP_IOMFB_METHODS_H17G) &&
	    dcp_swap_lost(dcp, id))
		return;
#endif
	dcp->last_swap_id = id;
	dcp->have_swap_complete = true;
	dcp_swap_watchdog_complete(dcp);
	if (retire) {
		dcp_release_retained_framebuffers(dcp, id);
		dcp_drm_crtc_page_flip(dcp, now);
	}
	if (READ_ONCE(dcp->crc_enabled)) {
#if DCP_FW_H17(DCP_FW_VER)
		if (dcp->hw.iomfb_method_profile != DCP_IOMFB_METHODS_H17G) {
			iomfb_queue_crc_h17p(dcp, id);
			return;
		}
#endif
		u32 crc32 = 0;
		drm_crtc_add_crc_entry(&dcp->crtc->base, true, id, &crc32);
	}
}

#if (DCP_FW_H17(DCP_FW_VER) && DCP_FW_VER >= DCP_FW_VERSION(27, 0, 0))
struct dcp_peer_completion { u32 id; ktime_t time; };
static void dcp_peer_report_on_owner(struct apple_dcp *owner, void *data, void *cookie)
{
	struct dcp_peer_completion *completion = data;
	dcp_report_swap_complete(owner, completion->id, completion->time);
}
static void dcp_peer_queue_completion(struct apple_dcp *owner, u32 id, ktime_t time)
{
	struct dcp_peer_completion completion = { id, time };
	if (iomfb_queue_callback(owner, dcp_peer_report_on_owner, &completion,
				 sizeof(completion), NULL)) {
		WRITE_ONCE(owner->crashed, true);
		schedule_work(&owner->vblank_wq);
	}
}
#endif

static void dcpep_cb_swap_complete(struct apple_dcp *dcp,
				   struct DCP_FW_NAME(dc_swap_complete_resp) *resp)
{
	ktime_t now = dcp_is_ext26(dcp) ? ns_to_ktime(dcp->ext26_rx_ns) : ktime_get();
	trace_iomfb_swap_complete(dcp, resp->swap_id);
#if (DCP_FW_H17(DCP_FW_VER) && DCP_FW_VER >= DCP_FW_VERSION(27, 0, 0))
	if (dcp_peer_swap_complete_v27(dcp, resp->swap_id, now, dcp_peer_queue_completion))
		return;
#endif
	dcp_report_swap_complete(dcp, resp->swap_id, now);
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

#if DCP_FW_H17(DCP_FW_VER)
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

static void __maybe_unused dcp_ack_dfb_surface(struct apple_dcp *dcp)
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
static void __maybe_unused dcp_fill_dfb_surface(struct apple_dcp *dcp, u8 *s)
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
#if (DCP_FW_H17(DCP_FW_VER) && DCP_FW_VER >= DCP_FW_VERSION(27, 0, 0))
	struct dcp_channel *ch = &dcp->ch_cb;

	*(u8 *)ch->output[ch->depth - 1] = out && !get_unaligned_le32(out);
	dcp_ack(dcp, DCP_CONTEXT_CB);
#else
	dcp_ack_dfb_surface(dcp);
#endif
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


#if (DCP_FW_H17(DCP_FW_VER) && DCP_FW_VER >= DCP_FW_VERSION(27, 0, 0))
	{
		struct dcp_dfb_dimensions_h17p dim;
		struct dcp_surface *surface = (void *)req->surface;
		u32 format, allocation, bpe;
		bool alpha_plane;
		int ret = -EINVAL;

		memcpy(&dim, in, sizeof(dim));
		if (dcp->dfb_format_valid && dcp->dfb_compression_valid)
			ret = apple_dcp_dfb_allocation(dim.width, dim.height,
				dcp->dfb_format, dcp->dfb_compression,
				dcp->dfb_region_size, &format, &allocation);
		if (ret) {
			dev_err(dcp->dev, "unsupported DFB layout %ux%u format=%#x compression=%u (%d)\n",
				dim.width, dim.height, dcp->dfb_format,
				dcp->dfb_compression, ret);
			*(u8 *)out = false;
			kfree(req);
			return true;
		}

		apple_dcp_dfb_format(dcp->dfb_format, dcp->dfb_compression,
				     &format, &bpe, &alpha_plane);
		surface->is_premultiplied = 1;
		surface->format = format;
		surface->pix_size = dcp_surface_element_geometry(dcp) ? bpe : 1;
		surface->pel_w = 1;
		surface->pel_h = 1;
		surface->buf_size = allocation;
		surface->surface_id = 5;
		surface->has_comp = 1;
		surface->has_planes = 1;
	}
#else
	dcp_fill_dfb_surface(dcp, req->surface);
#endif

	dcp_register_dfb_surface(dcp, false, req, complete_dfb_surface, NULL);
	kfree(req);

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
#if !DCP_FW_DESKTOP_26(DCP_FW_VER)
	/* The desktop 26.x firmware publishes no panel level here. */
	#if (DCP_FW_H17(DCP_FW_VER) && DCP_FW_VER >= DCP_FW_VERSION(27, 0, 0))
	case 20:
#else
	case IOMFB_PROPERTY_NITS_V13_3:
	/*
	 * 13.x firmware publishes another property under the 14.7 ID, with
	 * value 1, right before the panel level. Read as nits it is 0, and a
	 * backlight registered from it starts at the lowest level.
	 */
#if DCP_FW_VERSION(14, 7, 0) <= DCP_FW_VER
	case IOMFB_PROPERTY_NITS_V14_7:
#endif
#endif
	{
#if DCP_FW_H17(DCP_FW_VER)
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
#endif
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
		resp.value = dcp_pmu_temperature(dcp->dev);
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

#if DCP_FW_H17(DCP_FW_VER)
	memdesc->piodma_mapped = true;

	/* the H17P reply echoes the buffer index */
	*resp = (struct DCP_FW_NAME(dcp_map_buf_resp)) {
		.buffer = req->buffer,
		.dva = memdesc->dva,
	};
#elif DCP_FW_DESKTOP_26(DCP_FW_VER)
	/* the desktop 26.x reply echoes the buffer index too */
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

#if DCP_FW_H17(DCP_FW_VER)
	resp.buffer = req->buffer;
#elif DCP_FW_DESKTOP_26(DCP_FW_VER)
	resp.buffer = req->buffer;
	resp.ret = EINVAL;
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
#if DCP_FW_H17(DCP_FW_VER)
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

	if (dcp_is_ext26(dcp)) {
		allocated.size = size;
		allocated.buf = dma_alloc_attrs(dcp->dev, size, &allocated.dva,
					    GFP_KERNEL, DMA_ATTR_FORCE_CONTIGUOUS);
	} else {
		allocated.buf = dma_alloc_coherent(dcp->dev, size, &allocated.dva, GFP_KERNEL);
	}
	if (!allocated.buf)
		return resp;

	ret = dma_get_sgtable(dcp->dev, &allocated.map, allocated.buf,
			      allocated.dva, size);
	if (ret || (dcp_is_ext26(dcp) && (allocated.map.orig_nents != 1 ||
	    ((sg_phys(allocated.map.sgl) | allocated.dva) & (SZ_16K - 1))))) {
		if (!ret)
			sg_free_table(&allocated.map);
		if (dcp_is_ext26(dcp))
			dma_free_attrs(dcp->dev, size, allocated.buf, allocated.dva,
				       DMA_ATTR_FORCE_CONTIGUOUS);
		else
			dma_free_coherent(dcp->dev, size, allocated.buf, allocated.dva);
		return resp;
	}

	/* Callbacks are serialized; publish only a fully initialized descriptor. */
	dcp->memdesc[id] = allocated;
	set_bit(id, dcp->memdesc_map);
	resp.mem_desc_id = id;
	resp.dva_size = allocated.size;
	resp.dva = allocated.dva;

#if DCP_FW_H17(DCP_FW_VER)
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

#if DCP_FW_VER >= DCP_FW_VERSION(13, 2, 0) && !DCP_FW_H17(DCP_FW_VER) && \
	!DCP_FW_DESKTOP_26(DCP_FW_VER)
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
#if DCP_FW_H17(DCP_FW_VER)
		/*
		 * H17P reaches these apertures physically, not through its
		 * DART, and expects dva == addr in the reply.  Trying to
		 * dma_map_resource() them instead fails and hands the firmware
		 * DMA_MAPPING_ERROR as a device address.
		 */
		dma_addr_t dva = dcp->fw_compat == DCP_FIRMWARE_V_27_0 ?
			dcp->disp_firmware_base[req->index] : rsrc->start;
#elif DCP_FW_DESKTOP_26(DCP_FW_VER)
		/*
		 * The desktop 26.x firmware takes the register windows'
		 * physical addresses as well.  It reaches them on its
		 * DART-bypass stream; a mapping in its DMA stream would not
		 * affect those accesses.
		 */
		dma_addr_t dva = rsrc->start;
#elif DCP_FW_VER >= DCP_FW_VERSION(13, 2, 0)
		dma_addr_t dva = dcpep_map_reg_dva(dcp, req->index, rsrc);
		WARN_ON(dva == DMA_MAPPING_ERROR);
#endif

		return (struct DCP_FW_NAME(dcp_map_reg_resp)){
			.addr =
#if (DCP_FW_H17(DCP_FW_VER) && DCP_FW_VER >= DCP_FW_VERSION(27, 0, 0))
			dcp->disp_firmware_base[req->index],
#else
			rsrc->start,
#endif
			.length = resource_size(rsrc),
#if DCP_FW_VER >= DCP_FW_VERSION(13, 2, 0)
			.dva = dva,
#endif
		};
	}
}

static dcp_read_edt_data_resp_t
dcpep_cb_read_edt_data(struct apple_dcp *dcp, struct dcp_read_edt_data_req *req)
{
	/* Observe boot-property requests without inventing firmware timings. */
	if (dcp->fixed_connector_type != DRM_MODE_CONNECTOR_eDP)
		dev_dbg(dcp->dev,
			"read_edt_data key=%.*s count=%u default0=%#x ret=0\n",
			 (int)sizeof(req->key), req->key, req->count,
			 req->value[0]);

	return (dcp_read_edt_data_resp_t){
		.value[0] = req->value[0],
		.ret = 0,
	};
}

static void iomfbep_cb_enable_backlight_message_ap_gated(struct apple_dcp *dcp,
							 u8 *enabled)
{
#if DCP_FW_DESKTOP_26(DCP_FW_VER)
	if (dcp_has_panel(dcp) && dcp_backlight_resend(dcp))
		schedule_work(&dcp->bl_update_wq);
	return;
#elif DCP_FW_H17(DCP_FW_VER)
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
#if DCP_FW_H17(DCP_FW_VER) && DCP_FW_VER < DCP_FW_VERSION(27, 0, 0)
	if (dcp_is_ext26(dcp))
		return ext26_prop_start(dcp, *length);
#endif
	if (dcp->chunks.data != NULL) {
		dev_warn(dcp->dev, "ignoring spurious transfer start\n");
		return false;
	}

#if DCP_FW_H17(DCP_FW_VER)
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
	dcp->chunks.data = kvzalloc(*length, GFP_KERNEL);

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

	if (req->offset != dcp->chunks.received) {
		dcp->chunks.contiguous = false;
		if (dcp_is_ext26(dcp))
			return false;
	}
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
	if (dcp_firmware_metadata_enabled(dcp))
		dcp->chunks.payload_complete = dcp->chunks.contiguous &&
			dcp_property_complete(dcp->chunks.data, dcp->chunks.received,
					      dcp->chunks.length);

	dcp_trace_panel_property(dcp, 0, req->key,
				 strnlen(req->key, sizeof(req->key)),
				 dcp->chunks.data, dcp->chunks.length);

	if (dcp->fw_compat == DCP_FIRMWARE_V_26_6_DESKTOP)
		dcp_panel_edid_receive(dcp, dcp->chunks.data, dcp->chunks.length);
	if (dcp->fw_compat == DCP_FIRMWARE_V_26_6_DESKTOP &&
	    (!strcmp(req->key, "EDID") || !strcmp(req->key, "edid")))
		return true;
	if (dcp_is_ext26(dcp) && (!dcp->chunks.contiguous ||
	    dcp->chunks.received != dcp->chunks.length))
		return false;
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
				kvfree(dcp->chunks.data);
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
#if (DCP_FW_H17(DCP_FW_VER) && DCP_FW_VER >= DCP_FW_VERSION(27, 0, 0))
		if (dcp->peer_reserved)
			for (unsigned int i = 0; i < dcp->nr_modes; i++) {
				struct dcp_display_mode *m = &dcp->modes[i];

				if ((u64)m->mode.clock * 1000 > dcp->dptxport[0].caps.total_rate)
					dev_info(dcp->dev, "peer mode timing=%u color=%u " DRM_MODE_FMT "\n",
						 m->timing_mode_id, m->color_mode_id,
						 DRM_MODE_ARG(&m->mode));
			}
#endif
	} else if (!strcmp(req->key, "DisplayAttributes")) {
		struct dcp_parse_ctx metadata_ctx = ctx;

		ret = dcp_attributes_replace(dcp, &ctx, dcp->chunks.modes_generation);
		if (ret) {
			if (ret == -ESTALE) {
				kvfree(dcp->chunks.data);
				dcp->chunks.data = NULL;
				dcp->chunks.length = 0;
			}
			dev_warn(dcp->dev, "failed to parse display attribs\n");
			return false;
		}

		if (dcp_firmware_metadata_enabled(dcp)) {
			struct dcp_display_metadata metadata;

			ret = parse_display_metadata(&metadata_ctx, &metadata);
			if (ret)
				dev_dbg(dcp->dev, "DisplayAttributes without sink identity: %d\n",
					ret);
			dcp_firmware_metadata_attributes(dcp, ret ? NULL : &metadata);
		} else if (dcp->panel_identity && dcp_has_panel(dcp)) {
			struct dcp_display_metadata metadata;

			ret = parse_panel_metadata(&metadata_ctx, &metadata);
			dcp_panel_metadata_replace(dcp, ret ? NULL : &metadata);
		}
	}

	return true;
}

static u8 dcpep_cb_prop_end(struct apple_dcp *dcp,
			    struct dcp_set_dcpav_prop_end_req *req)
{
	u8 resp;

	if (!dcp_property_key_valid(req->key, sizeof(req->key))) {
		kvfree(dcp->chunks.data);
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
#if DCP_FW_H17(DCP_FW_VER)
	/* H17P expects A380 right before set_display_refresh_properties */
	if (!dcp_methods[dcpep_dfb_ready_query].tag[0]) {
		/* No tag is known for this firmware build: do not send one. */
		boot_5b(dcp, out, cookie);
		return;
	}
	dcp_dfb_ready_query(dcp, false, boot_5b, NULL);
#else
	boot_5b(dcp, out, cookie);
#endif
}

static void boot_4b(struct apple_dcp *dcp, void *out, void *cookie)
{
#if DCP_FW_VER >= DCP_FW_VERSION(13, 2, 0)
#if DCP_FW_H17(DCP_FW_VER)
	u32 value = dcp->hw.iomfb_method_profile == DCP_IOMFB_METHODS_H17G;
#elif DCP_FW_DESKTOP_26(DCP_FW_VER)
	dcp_bool_arg_t value = dcp_bool_arg(true);
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
#if DCP_FW_H17(DCP_FW_VER)
	/* H17P expects A033 between flush_supports_power and late_init_signal */
	dcp_dfb_query(dcp, false, boot_4b, NULL);
#else
	boot_4b(dcp, out, cookie);
#endif
}

static void boot_3(struct apple_dcp *dcp, void *out, void *cookie)
{
#if DCP_FW_DESKTOP_26(DCP_FW_VER)
	dcp_bool_arg_t v_true = dcp_bool_arg(true);
#else
	u32 v_true = true;
#endif

	dcp_flush_supports_power(dcp, false, &v_true, boot_4, NULL);
}

static void boot_2(struct apple_dcp *dcp, void *out, void *cookie)
{
	dcp_setup_video_limits(dcp, false, boot_3, NULL);
}

#if DCP_FW_H17(DCP_FW_VER)
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
#if (DCP_FW_H17(DCP_FW_VER) && DCP_FW_VER >= DCP_FW_VERSION(27, 0, 0))
	struct dcp_dfb_dimensions_h17p dim;

	if (!out)
		return;
	memcpy(&dim, out, sizeof(dim));

	if (dim.width > 16384 || dim.height > 16384
	   ) {
		struct dcp_channel *ch = &dcp->ch_cb;

		dev_err(dcp->dev, "invalid default framebuffer dimensions %ux%u\n",
			dim.width, dim.height);
		*(u8 *)ch->output[ch->depth - 1] = false;
		dcp_ack(dcp, DCP_CONTEXT_CB);
		return;
	}

	dcp_set_dfb_dimensions(dcp, false, &dim, boot_1_11, NULL);

#else
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

#endif
}

static void boot_1_9(struct apple_dcp *dcp, void *out, void *cookie)
{
	u32 zero = 0;

#if (DCP_FW_H17(DCP_FW_VER) && DCP_FW_VER >= DCP_FW_VERSION(27, 0, 0))
	struct dcp_dfb_dimensions_h17p dim;

	if (!out)
		return;
	memcpy(&dim, out, sizeof(dim));
	if (dim.width && dim.height) {
		boot_2(dcp, out, cookie);
		return;
	}
#endif
	dcp_get_dfb_state(dcp, false, &zero, boot_1_10, NULL);
}

static void boot_1_8(struct apple_dcp *dcp, void *out, void *cookie)
{
	struct dcp_dfb_layout_req_h17p req = {};

#if (DCP_FW_H17(DCP_FW_VER) && DCP_FW_VER >= DCP_FW_VERSION(27, 0, 0))
	if (!out)
		return;
	dcp->dfb_format_valid = *((u8 *)out + 0x28) & 1;
	memcpy(&dcp->dfb_format, out, sizeof(dcp->dfb_format));
#endif
	dcp_get_dfb_layout(dcp, false, &req, boot_1_9, NULL);
}

static void boot_1_7(struct apple_dcp *dcp, void *out, void *cookie)
{
	struct dcp_dfb_info_h17p info = {};

#if (DCP_FW_H17(DCP_FW_VER) && DCP_FW_VER >= DCP_FW_VERSION(27, 0, 0))
	if (!out)
		return;
	dcp->dfb_compression_valid = *((u8 *)out + 4) & 1;
	memcpy(&dcp->dfb_compression, out, sizeof(dcp->dfb_compression));
#endif
	dcp_get_dfb_info(dcp, false, &info, boot_1_8, NULL);
}

static void boot_1_6(struct apple_dcp *dcp, void *out, void *cookie)
{
	u64 zero = 0;

#if (DCP_FW_H17(DCP_FW_VER) && DCP_FW_VER >= DCP_FW_VERSION(27, 0, 0))
	if (!out)
		return;
	if (!(*(u8 *)out & 1)) {
		struct dcp_dfb_layout_req_h17p req = {};

		dcp_get_dfb_layout(dcp, false, &req, boot_1_9, NULL);
		return;
	}
#endif
	dcp_get_dfb_compression_info(dcp, false, &zero, boot_1_7, NULL);
}
#endif

static void boot_1_5(struct apple_dcp *dcp, void *out, void *cookie)
{
#if DCP_FW_H17(DCP_FW_VER)
	dcp_create_default_fb(dcp, false, boot_1_6, NULL);
#else
	dcp_create_default_fb(dcp, false, boot_2, NULL);
#endif
}

#if DCP_FW_H17(DCP_FW_VER)
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

#if DCP_FW_H17(DCP_FW_VER)
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

#if DCP_FW_DESKTOP_26(DCP_FW_VER)
/* D006: echo the properties with the flag bytes 73..75 cleared. */
static struct dcp_set_frame_sync_props_resp_v26_6_desktop
dcpep_cb_set_frame_sync_props(struct apple_dcp *dcp,
			      struct dcp_set_frame_sync_props_req_v26_6_desktop *req)
{
	struct dcp_set_frame_sync_props_resp_v26_6_desktop resp;

	memcpy(resp.props, req->props, sizeof(resp.props));
	memset(&resp.props[73], 0, 3);
	return resp;
}
#else
static struct dcp_set_frame_sync_props_resp
dcpep_cb_set_frame_sync_props(struct apple_dcp *dcp,
			      struct dcp_set_frame_sync_props_req *req)
{
	return (struct dcp_set_frame_sync_props_resp){};
}
#endif

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

#if DCP_FW_H17(DCP_FW_VER)
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
#if DCP_FW_VER < DCP_FW_VERSION(27, 0, 0)
	struct DCP_FW_NAME(dcp_swap_submit_req) *req = &DCP_FW_SWAP(dcp);
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
#endif
}
#else
static void dcp_h17p_prepare_swap(struct apple_dcp *dcp) { }
#endif

static bool dcp_present_retires(struct apple_dcp *dcp)
{
#if DCP_FW_H17(DCP_FW_VER)
	if (dcp->hw.iomfb_method_profile != DCP_IOMFB_METHODS_H17G)
		return !iomfb_present_brightness_only_h17p(dcp);
#endif
	return true;
}

static void dcp_present_failed(struct apple_dcp *dcp)
{
#if DCP_FW_H17(DCP_FW_VER)
	if (dcp->hw.iomfb_method_profile != DCP_IOMFB_METHODS_H17G)
		iomfb_present_failed_h17p(dcp);
#endif
}

static bool dcp_present_begin(struct apple_dcp *dcp, u32 swap_id)
{
#if DCP_FW_H17(DCP_FW_VER)
	if (dcp->hw.iomfb_method_profile != DCP_IOMFB_METHODS_H17G &&
	    !dcp_present_begin_h17p(&dcp->present_state_h17p, swap_id)) {
		dev_err(dcp->dev, "overlapping present %u\n", swap_id);
		dcp->crashed = true;
		return false;
	}
#elif DCP_FW_DESKTOP_26(DCP_FW_VER)
	WRITE_ONCE(dcp->desktop->present_id, swap_id);
	WRITE_ONCE(dcp->desktop->present_pending, true);
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
#if DCP_FW_H17(DCP_FW_VER)
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
	typeof(DCP_FW_SWAP(dcp)) *swap = request;

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
#if DCP_FW_DESKTOP_26(DCP_FW_VER)
		dcp_swap_set_brightness(&swap->swap, 0);
#else
#if !DCP_FW_LAYOUT_14_7(DCP_FW_VER)
		swap->swap.bl_unk = 1;
#else
		swap->swap.bl_update = 1;
#endif
		swap->swap.bl_value = 0;
		swap->swap.bl_power = 0;
#endif
	}

	/* Null all surfaces */
	for (int l = 0; l < SWAP_SURFACES; l++)
		swap->surf_null[l] = true;
#if DCP_FW_VERSION(13, 2, 0) <= DCP_FW_VER
#if !DCP_FW_H17(DCP_FW_VER)
	for (int l = 0; l < ARRAY_SIZE(swap->surf2_null); l++)
		swap->surf2_null[l] = true;
#endif
	swap->unknown_pointer_null = true;
	swap->unknown_output_null = true;
#endif
}

static void dcp_present_rejected(struct apple_dcp *dcp)
{
	if (dcp_is_ext26(dcp)) {
		WRITE_ONCE(dcp->ext26_blocked, true);
		WRITE_ONCE(dcp->crashed, true);
	}
}

static void dcp_swap_cleared(struct apple_dcp *dcp, void *data, void *cookie)
{
	struct DCP_FW_NAME(dcp_swap_submit_resp) *resp = data;
	struct dcp_swap_cookie *info = cookie;
	u32 swap_id = DCP_FW_SWAP(dcp).swap.swap_id;
	u32 status = resp ? resp->ret : ~0U;
	int ret;

	ret = dcp_present_submit(dcp, swap_id, !status);
	if (ret)
		status = ~0U;

#if DCP_FW_DESKTOP_26(DCP_FW_VER)
	if (status && swap_id == READ_ONCE(dcp->desktop->present_id))
		WRITE_ONCE(dcp->desktop->present_pending, false);
	dcp_backlight_desktop_finish(dcp, swap_id, true, !status);
#endif

	if (status) {
		dcp_present_rejected(dcp);
		dcp->ext26_early_complete = false;
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
#if DCP_FW_H17(DCP_FW_VER)
	if (dcp->hw.iomfb_method_profile != DCP_IOMFB_METHODS_H17G)
		dcp_prepare_clear_swap(dcp, &DCP_FW_SWAP(dcp));
#endif
	DCP_FW_SWAP(dcp).swap.swap_id = resp->swap_id;
	WRITE_ONCE(dcp->started_swap_id, resp->swap_id);
	if (dcp_present_retires(dcp))
		dcp_arm_retained_framebuffers(dcp, resp->swap_id);

	if (info)
		info->swap_id = resp->swap_id;

	if (!dcp_is_ext26(dcp))
		dcp_h17p_prepare_swap(dcp);
	dcp_swap_submit(dcp, false, &DCP_FW_SWAP(dcp), dcp_swap_cleared, cookie);
}

static void dcp_on_final(struct apple_dcp *dcp, void *out, void *cookie)
{
	struct dcp_wait_cookie *wait = cookie;

#if DCP_FW_H17(DCP_FW_VER)
	if (dcp->hw.iomfb_method_profile != DCP_IOMFB_METHODS_H17G &&
	    out && get_unaligned_le64(out) == 1)
		WRITE_ONCE(dcp->pipe_enabled_h17p, true);
#endif

	if (wait) {
		complete(&wait->done);
		kref_put(&wait->refcount, release_wait_cookie);
	}
}

#if DCP_FW_DESKTOP_26(DCP_FW_VER)
static void dcp_on_set_parameter(struct apple_dcp *dcp, void *out, void *cookie);

/* After set_power_state(1): select the display, then adaptive sync off. */
static void dcp_on_set_power_state(struct apple_dcp *dcp, void *out, void *cookie)
{
	u32 handle = dcp->main_display ? 0 : 2;

	dcp_set_display_device(dcp, false, &handle,
			       dcp->main_display ? dcp_on_final :
						   dcp_on_set_parameter,
			       cookie);
}

static void dcp_on_set_parameter(struct apple_dcp *dcp, void *out, void *cookie)
{
	struct dcp_set_parameter_dcp param;

	/* minRR, mediaTargetRate and fractional rate, as three u64 */
	memset(&param, 0xaa, sizeof(param));
	param.param = IOMFBPARAM_ADAPTIVE_SYNC;
	memset(param.value, 0, 3 * sizeof(u64));
	param.count = 3;

	dcp_set_parameter_dcp(dcp, false, &param, dcp_on_final, cookie);
}

static void dcp_poweron_after_main_display(struct apple_dcp *dcp, void *out,
					   void *cookie)
{
	struct DCP_FW_NAME(dcp_set_power_state_req) req =
		dcp_power_state_req(1, DCP_POWER_TRANSITION_NONE);

	dcp_set_power_state(dcp, DCP_POWER_OOB, &req, dcp_on_set_power_state,
			    cookie);
}

static void dcp_poweron_after_finish_wake(struct apple_dcp *dcp, void *out,
					  void *cookie)
{
	dcp_is_main_display(dcp, false, dcp_poweron_after_main_display, cookie);
}

static void dcp_poweron_after_first_client(struct apple_dcp *dcp, void *out,
					   void *cookie)
{
	dcp_finish_wake(dcp, false, dcp_poweron_after_finish_wake, cookie);
}

static void dcp_poweron_after_prepare_wake(struct apple_dcp *dcp, void *out,
					   void *cookie)
{
	dcp_first_client_open(dcp, false, dcp_poweron_after_first_client, cookie);
}
#else
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
		.param =
#if (DCP_FW_H17(DCP_FW_VER) && DCP_FW_VER >= DCP_FW_VERSION(27, 0, 0))
		15,
#else
		IOMFBPARAM_ADAPTIVE_SYNC,
#endif
		.value = { 0 },
#if DCP_FW_VER >= DCP_FW_VERSION(13, 2, 0)
		.count = 3,
#else
		.count = 1,
#endif
	};

	dcp_set_parameter_dcp(dcp, false, &param, dcp_on_set_power_state, cookie);
}
#endif

void DCP_FW_NAME(iomfb_poweron)(struct apple_dcp *dcp)
{
	struct dcp_wait_cookie *cookie;
	int ret;
#if !DCP_FW_DESKTOP_26(DCP_FW_VER)
	u32 handle;
#endif
	dev_info(dcp->dev, "dcp_poweron() starting\n");
#if DCP_FW_H17(DCP_FW_VER)
	if (dcp_is_ext26(dcp))
		return; /* Native26 publishes A473 only after a successful A413. */
#endif

	/* The firmware announces its compact services again after poweron. */
	WRITE_ONCE(dcp->afk_power_gen, dcp->afk_power_gen + 1);

#if DCP_FW_H17(DCP_FW_VER)
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

#if DCP_FW_DESKTOP_26(DCP_FW_VER)
	/*
	 * The firmware's own wake order: A414, first_client_open, A474,
	 * is_main_display, set_power_state(1), then select the display.
	 * Without A414/A474, or with the display selected before A473, the
	 * controller stays powered off and every swap is swallowed.
	 */
	{
		struct dcp_wake_state_req_v26_6 req = dcp_wake_state_req;

		dcp_prepare_wake(dcp, false, &req, dcp_poweron_after_prepare_wake,
				 cookie);
	}
#else
	if (dcp->main_display) {
#if DCP_FW_H17(DCP_FW_VER)
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
#endif
	ret = wait_for_completion_timeout(&cookie->done, msecs_to_jiffies(10000));
#if DCP_FW_DESKTOP_26(DCP_FW_VER)
	dcp->desktop->poweron_result = ret > 0 ? 0 : ret == 0 ? 1 : 2;
	if (ret > 0)
		dcp_backlight_desktop_power(dcp, true);
#endif

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

/* V27 shares the h17p template prefix; iomfb_v27.c renames these two to their _v27 names. */
#if DCP_FW_H17(DCP_FW_VER)
/*
 * set_power_state around a firmware-driven suspend. What macOS sends before it in
 * either direction: A414(0), A478 twice, A476(1), A478, A412. All are named in
 * this firmware's table, and their arguments are measured from macOS.
 *
 * For display on, macOS also sends A439 at the same moment. Its name isn't known,
 * it carries a macOS kernel pointer and its reply wasn't captured, so it isn't
 * sent here.
 */
enum dcp_s2r_step {
	DCP_S2R_A414, DCP_S2R_A478, DCP_S2R_A476, DCP_S2R_A412, DCP_S2R_A473,
};

static const u8 dcp_s2r_seq[] = {
	DCP_S2R_A414, DCP_S2R_A478, DCP_S2R_A478, DCP_S2R_A476, DCP_S2R_A478,
	DCP_S2R_A412, DCP_S2R_A473,
};

static void dcp_s2r_run(struct apple_dcp *dcp, void *out, void *cookie)
{
	unsigned int step = dcp->s2r_step++;

	switch (dcp_s2r_seq[step]) {
	case DCP_S2R_A414: {
		u64 val = 0;

		dcp_pipe_cfg_414(dcp, false, &val, dcp_s2r_run, cookie);
		break;
	}
	case DCP_S2R_A478:
		dcp_pipe_query_478(dcp, false, dcp_s2r_run, cookie);
		break;
	case DCP_S2R_A476: {
		u32 val = 1;

		dcp_pipe_set_476(dcp, false, &val, dcp_s2r_run, cookie);
		break;
	}
	case DCP_S2R_A412:
		dcp_is_main_display(dcp, false, dcp_s2r_run, cookie);
		break;
	case DCP_S2R_A473: {
		/* Measured: the state, then 00 00 00 00 00 aa aa aa. */
		struct DCP_FW_NAME(dcp_set_power_state_req) req = {
			.unklong = dcp->s2r_power_on,
			.padding = { 0xaa, 0xaa, 0xaa },
		};

		dcp_set_power_state(dcp, dcp->s2r_power_oob, &req,
				    dcp->s2r_power_on ? dcp_on_final : complete_set_powerstate,
				    cookie);
		break;
	}
	}
}

static void dcp_s2r_500_done(struct apple_dcp *dcp, void *out, void *cookie)
{
	struct dcp_wait_cookie *wait = cookie;

	complete(&wait->done);
	kref_put(&wait->refcount, release_wait_cookie);
}

/* Unnamed. macOS sends it after D572 on every display off, with these bytes. */
int DCP_FW_NAME(iomfb_s2r_500)(struct apple_dcp *dcp)
{
	struct dcp_wait_cookie *cookie;
	unsigned long left;
	u32 val = 0xaaaa0000;

	cookie = kzalloc_obj(*cookie);
	if (!cookie)
		return -ENOMEM;
	init_completion(&cookie->done);
	kref_init(&cookie->refcount);
	/* increase refcount to ensure the receiver has a reference */
	kref_get(&cookie->refcount);

	dcp_unnamed_500(dcp, false, &val, dcp_s2r_500_done, cookie);

	left = wait_for_completion_timeout(&cookie->done, msecs_to_jiffies(1000));
	kref_put(&cookie->refcount, release_wait_cookie);

	return left ? 0 : -ETIMEDOUT;
}

int DCP_FW_NAME(iomfb_s2r_power)(struct apple_dcp *dcp, bool on, bool named_calls, bool oob)
{
	struct dcp_wait_cookie *cookie;
	unsigned long left;

	cookie = kzalloc_obj(*cookie);
	if (!cookie)
		return -ENOMEM;
	cookie->status = ~0U;
	init_completion(&cookie->done);
	kref_init(&cookie->refcount);
	/* increase refcount to ensure the receiver has a reference */
	kref_get(&cookie->refcount);

	dcp->s2r_power_on = on;
	dcp->s2r_power_oob = oob;
	dcp->s2r_step = named_calls ? 0 : ARRAY_SIZE(dcp_s2r_seq) - 1;
	dcp_s2r_run(dcp, NULL, cookie);

	left = wait_for_completion_timeout(&cookie->done, msecs_to_jiffies(1000));
	kref_put(&cookie->refcount, release_wait_cookie);

	return left ? 0 : -ETIMEDOUT;
}
#endif

static bool dcp_power_stop_confirmed(struct dcp_wait_cookie *cookie)
{
	/* H17P does not confirm that scanout has stopped. */
#if DCP_FW_H17(DCP_FW_VER)
	return false;
#else
	return READ_ONCE(cookie->status) == 0;
#endif
}

#if DCP_FW_DESKTOP_26(DCP_FW_VER)
static void dcp_poweroff_after_main_display(struct apple_dcp *dcp, void *out,
					    void *cookie)
{
	struct DCP_FW_NAME(dcp_set_power_state_req) power_req =
		dcp_power_state_req(0, DCP_POWER_TRANSITION_DISPLAY_OFF);

	dcp_set_power_state(dcp, DCP_POWER_OOB, &power_req,
			    complete_set_powerstate, cookie);
}

static void dcp_poweroff_after_prepare_state(struct apple_dcp *dcp, void *out,
					     void *cookie)
{
	dcp_is_main_display(dcp, false, dcp_poweroff_after_main_display, cookie);
}
#else
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
#endif

void DCP_FW_NAME(iomfb_poweroff)(struct apple_dcp *dcp)
{
	int ret, swap_id;
#if DCP_FW_DESKTOP_26(DCP_FW_VER)
	struct DCP_FW_NAME(dcp_swap_start_req) swap_req =
		dcp_swap_start_request(dcp);
#else
	dcp_abort_swaps_req_t abort_req = {
		.client = {
			.flag2 = 1,
		},
	};
	struct DCP_FW_NAME(dcp_swap_start_req) swap_req = { 0 };
#endif
	struct dcp_swap_cookie *cookie;
	struct dcp_wait_cookie *poff_cookie;
	typeof(DCP_FW_SWAP(dcp)) *swap = &DCP_FW_SWAP(dcp);

#if (DCP_FW_H17(DCP_FW_VER) && DCP_FW_VER >= DCP_FW_VERSION(27, 0, 0))
	/*
	 * Firmware 27 has its own paired stop sequence. Its hardware profile
	 * does not use H17G's single-pipe methods, so stop an active pair
	 * before applying the guard for the single-pipe path below.
	 */
	ret = dcp_peer_stop_v27(dcp);
	if (ret < 0) {
		dev_err(dcp->dev, "paired power-off failed: %d\n", ret);
		return;
	}
	if (ret > 0)
		goto close_client;
#endif

#if DCP_FW_H17(DCP_FW_VER)
	/* H17P has no known safe pipe-off or surface-free transition. */
	if (dcp->hw.iomfb_method_profile != DCP_IOMFB_METHODS_H17G)
		return;
#endif

#if DCP_FW_DESKTOP_26(DCP_FW_VER)
	/* A merged pair stops as a pair; its clear swap would not complete. */
	ret = dcp_peer_stop_26_6(dcp);
	if (ret < 0)
		dev_err(dcp->dev, "paired power-off failed: %d\n", ret);
	if (ret > 0)
		goto peer_stopped;
#endif
	cookie = kzalloc(sizeof(*cookie), GFP_KERNEL);
	if (!cookie)
		return;
	cookie->status = ~0U;
	init_completion(&cookie->done);
	kref_init(&cookie->refcount);
	/* increase refcount to ensure the receiver has a reference */
	kref_get(&cookie->refcount);

#if DCP_FW_H17(DCP_FW_VER)
	if (dcp->hw.iomfb_method_profile == DCP_IOMFB_METHODS_H17G)
		dcp_prepare_clear_swap(dcp, swap);
#else
#if DCP_FW_DESKTOP_26(DCP_FW_VER)
	dcp_backlight_desktop_power(dcp, false);
#endif
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

#if DCP_FW_DESKTOP_26(DCP_FW_VER)
peer_stopped:
#endif
#if (DCP_FW_H17(DCP_FW_VER) && DCP_FW_VER >= DCP_FW_VERSION(27, 0, 0))
close_client:
#endif
	poff_cookie = kzalloc(sizeof(*poff_cookie), GFP_KERNEL);
	if (!poff_cookie)
		return;
	poff_cookie->status = ~0U;
	init_completion(&poff_cookie->done);
	kref_init(&poff_cookie->refcount);
	/* increase refcount to ensure the receiver has a reference */
	kref_get(&poff_cookie->refcount);

#if DCP_FW_DESKTOP_26(DCP_FW_VER)
	/*
	 * The firmware's own display power-off goes from the clear swap
	 * through A414, is_main_display and set_power_state(0).  It sends
	 * abort_swaps and last_client_close only for sleep and shutdown;
	 * here they could tear down state the link power-off still needs.
	 */
	{
		struct dcp_wake_state_req_v26_6 req = dcp_wake_state_req;

		dcp_prepare_wake(dcp, false, &req, dcp_poweroff_after_prepare_state,
				 poff_cookie);
	}
#else
	if (DCP_HAS_CLIENT_TEARDOWN) {
		iomfb_abort_swaps_dcp(dcp, false, &abort_req,
					aborted_swaps_dcp_poff, poff_cookie);
	} else {
		dev_dbg_once(dcp->dev,
			     "power-off: skipping abort_swaps/last_client_close\n");
		last_client_closed_poff(dcp, NULL, poff_cookie);
	}
#endif
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

#if DCP_FW_VERSION(26, 0, 0) <= DCP_FW_VER && DCP_FW_VER < DCP_FW_VERSION(27, 0, 0)
/*
 * H17G (M5) internal panel DPMS off.  A pipe stop does not come back:
 * setPowerState(0) times out and the next setPowerState(1) is never
 * answered.  Blank instead and keep the pipe running: a black,
 * surface-free swap that also takes the backlight to 0 nits.  The next CRTC
 * enable presents the scanout and the brightness again.
 */
void DCP_FW_NAME(iomfb_blank)(struct apple_dcp *dcp)
{
	struct DCP_FW_NAME(dcp_swap_start_req) swap_req = { 0 };
	typeof(DCP_FW_UNION(dcp->swap)) *swap = &DCP_FW_UNION(dcp->swap);
	struct dcp_swap_cookie *cookie;
	unsigned long ret;

	cookie = kzalloc(sizeof(*cookie), GFP_KERNEL);
	if (!cookie)
		return;
	cookie->status = ~0U;
	init_completion(&cookie->done);
	kref_init(&cookie->refcount);
	/* increase refcount to ensure the receiver has a reference */
	kref_get(&cookie->refcount);

	dcp_prepare_clear_swap(dcp, swap);
	if (dcp_has_panel(dcp))
		iomfb_encode_backlight_swap_h17p((u8 *)&swap->swap, 0,
						 dcp->brightness.maximum, true);

	dcp_swap_start(dcp, false, &swap_req, dcp_swap_clear_started, cookie);
	ret = wait_for_completion_timeout(&cookie->done, msecs_to_jiffies(500));
	if (!ret || READ_ONCE(cookie->status))
		dev_warn(dcp->dev, "%s: blank swap %s\n", __func__,
			 ret ? "failed" : "timed out");
	kref_put(&cookie->refcount, release_swap_cookie);
}
#endif

static void last_client_closed_sleep(struct apple_dcp *dcp, void *out, void *cookie)
{
#if DCP_FW_DESKTOP_26(DCP_FW_VER)
	struct DCP_FW_NAME(dcp_set_power_state_req) power_req =
		dcp_power_state_req(0, DCP_POWER_TRANSITION_RESTARTABLE_SLEEP);
#else
	struct DCP_FW_NAME(dcp_set_power_state_req) power_req = {
		.unklong = 0,
	};
#endif
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
#if DCP_FW_DESKTOP_26(DCP_FW_VER)
	/* abort_swaps, last_client_close, set_power_state(0) as for sleep */
	dcp_abort_swaps_req_t req = dcp_abort_swaps_request(dcp);
#else
	dcp_abort_swaps_req_t req = {
		.client = {
			.flag2 = 1,
		},
	};
#endif

	struct dcp_wait_cookie *cookie;

#if (DCP_FW_H17(DCP_FW_VER) && DCP_FW_VER >= DCP_FW_VERSION(27, 0, 0))
	ret = dcp_peer_stop_v27(dcp);
	if (ret < 0) {
		dev_err(dcp->dev, "paired sleep failed: %d\n", ret);
		return;
	}
#endif

#if DCP_FW_H17(DCP_FW_VER)
	if (dcp->hw.iomfb_method_profile != DCP_IOMFB_METHODS_H17G)
		return;
#endif

#if DCP_FW_DESKTOP_26(DCP_FW_VER)
	ret = dcp_peer_stop_26_6(dcp);
	if (ret < 0)
		dev_err(dcp->dev, "paired sleep failed: %d\n", ret);
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
	/* Fresh modes and positive firmware HPD, not lane-count negotiation,
	 * make IOMFB ready for the resume modeset. */
	if (*connected && dcp->fw_compat == DCP_FIRMWARE_V_27_0 &&
	    dcp_is_typec_output(dcp))
		complete_all(&dcp->typec_display_ready);

}

#if DCP_FW_DESKTOP_26(DCP_FW_VER)
/*
 * D575 carries a tiled-display record next to the connection state and
 * wants a 0x4c-byte record back.  Every captured connected reply has an
 * unaligned 0xffffffff at 71..74; byte 75 is padding.  The disconnected
 * reply holds more state, which is not decoded.
 */
static struct dcp_hotplug_resp_v26_6
dcpep_cb_hotplug_v26_6(struct apple_dcp *dcp, struct dcp_hotplug_req_v26_6 *req)
{
	struct dcp_hotplug_resp_v26_6 resp = { 0 };
	u64 connected = req->connected;
	const u8 *tiled = req->opaque;
	bool tiled_valid = !(req->opaque[sizeof(resp.opaque)] & 1);
	bool paired = READ_ONCE(dcp->peer_reserved) || READ_ONCE(dcp->peer_owner);

	/*
	 * With a peer reserved, the record names this pipe's and the related
	 * pipe's display at 0x3b and 0x47, as on firmware 27; it is echoed
	 * back as that firmware's client does.
	 */
	if (paired) {
		dev_dbg(dcp->dev, "peer HPD connected=%llu null=%u rec=%*ph\n",
			connected, req->opaque[sizeof(resp.opaque)],
			(int)sizeof(resp.opaque), tiled);
		if (tiled_valid && connected &&
		    !(get_unaligned_le32(tiled + 0xc) & BIT(3)))
			dcp_peer_display_ids(dcp, get_unaligned_le32(tiled + 0x3b),
					     get_unaligned_le32(tiled + 0x47));
	}

	/* A display that went away is bootstrapped again on its next mode. */
	if (!dcp->main_display &&
	    (!connected || (dcp->connector &&
			    READ_ONCE(dcp->connector->connected) != !!connected)))
		dcp->desktop->mode_live = false;
	/* The firmware shuts an external pipe down with its display. */
	if (!dcp->main_display && !connected)
		dcp->desktop->ext_pipe_off = true;

	dcpep_cb_hotplug(dcp, &connected);

	if (paired && tiled_valid)
		memcpy(resp.opaque, tiled, sizeof(resp.opaque));
	else if (connected)
		memset(&resp.opaque[0x47], 0xff, sizeof(u32));

	return resp;
}
#endif

static void
dcpep_cb_swap_complete_intent_gated(struct apple_dcp *dcp,
				    dcp_swap_intent_t *info)
{
	trace_iomfb_swap_complete_intent_gated(dcp, info->swap_id,
		info->width, info->height);
}

#if DCP_FW_H17(DCP_FW_VER)
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
#if DCP_FW_H17(DCP_FW_VER)
	if (dcp->hw.iomfb_method_profile != DCP_IOMFB_METHODS_H17G)
		dcp_present_aborted(dcp, *swap_id);
#endif
}

static struct dcpep_get_tiling_state_resp
dcpep_cb_get_tiling_state(struct apple_dcp *dcp,
			  struct dcpep_get_tiling_state_req *req)
{
#if DCP_FW_DESKTOP_26(DCP_FW_VER)
	/* D115 reads the tiling state of a paired pipe; D116 writes it. */
	u32 value = req->value;
	bool handled = dcp_peer_tiling_state_26_6(dcp, false, req->event,
						  req->param, &value);

	dev_dbg(dcp->dev, "peer tiling get event %u param %u value %u handled %d\n",
		req->event, req->param, value, handled);
	if (handled)
		return (struct dcpep_get_tiling_state_resp){
			.value = value,
			.ret = 1,
		};
#endif
	return (struct dcpep_get_tiling_state_resp){
		.value = 0,
		.ret = 1,
	};
}

#if DCP_FW_DESKTOP_26(DCP_FW_VER)
/* D116 set_tiling_state: the D115 record in, a boolean (padded to 4) out. */
static u32 dcpep_cb_set_tiling_state_v26_6(struct apple_dcp *dcp,
					   struct dcpep_get_tiling_state_req *req)
{
	u32 value = req->value;
	bool handled = !req->value_null &&
		       dcp_peer_tiling_state_26_6(dcp, true, req->event,
						  req->param, &value);

	dev_dbg(dcp->dev, "peer tiling set event %u param %u value %u null %u handled %d\n",
		req->event, req->param, req->value, req->value_null, handled);
	return handled;
}
#endif

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
		 struct dcp_read_edt_data_req, dcp_read_edt_data_resp_t);
TRAMPOLINE_INOUT(trampoline_prop_start, dcpep_cb_prop_start, u32, u8);
TRAMPOLINE_INOUT(trampoline_prop_chunk, dcpep_cb_prop_chunk,
		 struct dcp_set_dcpav_prop_chunk_req, u8);
TRAMPOLINE_INOUT(trampoline_prop_end, dcpep_cb_prop_end,
		 struct dcp_set_dcpav_prop_end_req, u8);
TRAMPOLINE_INOUT(trampoline_allocate_bandwidth, dcpep_cb_allocate_bandwidth,
	       struct dcp_allocate_bandwidth_req, struct dcp_allocate_bandwidth_resp);
TRAMPOLINE_OUT(trampoline_rt_bandwidth, dcpep_cb_rt_bandwidth,
	       struct dcp_rt_bandwidth);
#if DCP_FW_DESKTOP_26(DCP_FW_VER)
TRAMPOLINE_INOUT(trampoline_set_frame_sync_props, dcpep_cb_set_frame_sync_props,
	       struct dcp_set_frame_sync_props_req_v26_6_desktop,
	       struct dcp_set_frame_sync_props_resp_v26_6_desktop);
#else
TRAMPOLINE_INOUT(trampoline_set_frame_sync_props, dcpep_cb_set_frame_sync_props,
	       struct dcp_set_frame_sync_props_req,
	       struct dcp_set_frame_sync_props_resp);
#endif
#if DCP_FW_H17(DCP_FW_VER)
TRAMPOLINE_INOUT(trampoline_set_frame_sync_props_h17p,
	       dcpep_cb_set_frame_sync_props_h17p,
	       struct dcp_set_frame_sync_props_req_h17p,
	       struct dcp_set_frame_sync_props_resp_h17p);
#endif
TRAMPOLINE_OUT(trampoline_get_frequency, dcpep_cb_get_frequency, u64);
TRAMPOLINE_OUT(trampoline_get_time, dcpep_cb_get_time, u64);
#if DCP_FW_DESKTOP_26(DCP_FW_VER)
TRAMPOLINE_INOUT(trampoline_hotplug, dcpep_cb_hotplug_v26_6,
		 struct dcp_hotplug_req_v26_6, struct dcp_hotplug_resp_v26_6);
#else
TRAMPOLINE_IN(trampoline_hotplug, dcpep_cb_hotplug, u64);
#endif
TRAMPOLINE_IN(trampoline_swap_complete_intent_gated,
	      dcpep_cb_swap_complete_intent_gated,
	      dcp_swap_intent_t);
TRAMPOLINE_IN(trampoline_abort_swap_ap_gated, dcpep_cb_abort_swap_ap_gated, u32);
TRAMPOLINE_IN(trampoline_enable_backlight_message_ap_gated,
	      iomfbep_cb_enable_backlight_message_ap_gated, u8);
TRAMPOLINE_IN(trampoline_pr_publish, iomfb_cb_pr_publish,
	      struct iomfb_property);
TRAMPOLINE_INOUT(trampoline_get_tiling_state, dcpep_cb_get_tiling_state,
		 struct dcpep_get_tiling_state_req, struct dcpep_get_tiling_state_resp);
#if DCP_FW_DESKTOP_26(DCP_FW_VER)
TRAMPOLINE_INOUT(trampoline_set_tiling_state_v26_6, dcpep_cb_set_tiling_state_v26_6,
		 struct dcpep_get_tiling_state_req, u32);
#endif
TRAMPOLINE_OUT(trampoline_create_backlight_service, dcpep_cb_create_backlight_service, u8);

/*
 * Swap timestamps are in the firmware's continuous time. Firmware 27 counts
 * it at 24 MHz, while the ARM counter Linux reads may run faster (1 GHz on
 * T6050). A raw counter value then names a moment far in the future and the
 * firmware holds the swap for it, so scale the counter to the firmware's
 * timebase.
 */
#define DCP_V27_TIMEBASE_HZ 24000000U

static u64 __maybe_unused dcp_swap_time(void)
{
	u64 now = arch_timer_read_counter();
#if (DCP_FW_H17(DCP_FW_VER) && DCP_FW_VER >= DCP_FW_VERSION(27, 0, 0))
	u32 rate = arch_timer_get_cntfrq();

	if (rate && rate != DCP_V27_TIMEBASE_HZ)
		return mul_u64_u32_div(now, DCP_V27_TIMEBASE_HZ, rate);
#endif
	return now;
}

/*
 * The previous accepted swap's time, or now without one. On firmware 27 also
 * now when it is from the future or more than a second old, as after an idle
 * period: the firmware paces the swap from it.
 */
static u64 __maybe_unused dcp_swap_prev_time(u64 now, u64 prev)
{
	if (!prev)
		return now;
#if (DCP_FW_H17(DCP_FW_VER) && DCP_FW_VER >= DCP_FW_VERSION(27, 0, 0))
	if (prev > now || now - prev > DCP_V27_TIMEBASE_HZ)
		return now;
#endif
	return prev;
}

/*
 * Callback for swap requests. If a swap failed, we'll never get a swap
 * complete event so we need to fake a vblank event early to avoid a hang.
 */

static void dcp_swapped(struct apple_dcp *dcp, void *data, void *cookie)
{
	int ret;
	struct DCP_FW_NAME(dcp_swap_submit_resp) *resp = data;
	u32 swap_id = DCP_FW_SWAP(dcp).swap.swap_id;
	u32 status = resp ? resp->ret : ~0U;

	ret = dcp_present_submit(dcp, swap_id, !status);
	if (ret)
		status = ~0U;

#if DCP_FW_DESKTOP_26(DCP_FW_VER)
	if (status && swap_id == READ_ONCE(dcp->desktop->present_id))
		WRITE_ONCE(dcp->desktop->present_pending, false);
	dcp_backlight_desktop_finish(dcp, swap_id, true, !status);
#endif

	if (status) {
		dcp_present_rejected(dcp);
		dcp->ext26_early_complete = false;
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
	if (dcp_is_ext26(dcp) && dcp->ext26_early_complete && dcp->ext26_early_id == swap_id) {
		struct DCP_FW_NAME(dc_swap_complete_resp) completed = { .swap_id = swap_id };

		u64 received = dcp->ext26_rx_ns;

		dcp->ext26_early_complete = false;
		dcp->ext26_rx_ns = dcp->ext26_early_ns;
		dcpep_cb_swap_complete(dcp, &completed);
		dcp->ext26_rx_ns = received;
		return;
	}
	dcp->swap_start = ktime_get();
	dcp->swap_submit_timestamp = dcp_swap_time();
	dcp_swap_watchdog_arm(dcp);
}

#if (DCP_FW_H17(DCP_FW_VER) && DCP_FW_VER >= DCP_FW_VERSION(27, 0, 0))
#include "iomfb-peer-runtime-v27.inc"
#endif

static void dcp_swap_started(struct apple_dcp *dcp, void *data, void *cookie)
{
	struct DCP_FW_NAME(dcp_swap_start_resp) *resp = data;

	if (!resp || resp->ret || (dcp_is_ext26(dcp) && !resp->swap_id)) {
		dcp_present_rejected(dcp);
		dev_err(dcp->dev, "swap_start was rejected\n");
		if (dcp_present_retires(dcp))
			dcp_drm_crtc_vblank(dcp->crtc);
		return;
	}
	if (!dcp_present_begin(dcp, resp->swap_id)) {
		dcp_present_rejected(dcp);
		if (dcp_present_retires(dcp))
			dcp_drm_crtc_vblank(dcp->crtc);
		return;
	}
#if (DCP_FW_H17(DCP_FW_VER) && DCP_FW_VER >= DCP_FW_VERSION(27, 0, 0))
	if (dcp_peer_start_second(dcp, resp))
		return;
#endif
	DCP_FW_SWAP(dcp).swap.swap_id = resp->swap_id;
	WRITE_ONCE(dcp->started_swap_id, resp->swap_id);
	if (dcp_present_retires(dcp))
		dcp_arm_retained_framebuffers(dcp, resp->swap_id);

#if DCP_FW_DESKTOP_26(DCP_FW_VER)
	if (dcp_has_panel(dcp)) {
		u32 nits = 0;
		bool have_surface = false, carries;

		for (unsigned int i = 0; i < SWAP_SURFACES; i++)
			have_surface |= !DCP_FW_SWAP(dcp).surf_null[i];
		carries = dcp_backlight_desktop_prepare(dcp, resp->swap_id,
							have_surface, &nits);
		if (carries)
			dcp_swap_set_brightness(&DCP_FW_SWAP(dcp).swap, nits);
		dcp_edr_prepare(dcp, &DCP_FW_SWAP(dcp).swap, carries, nits);
	}
#endif
	trace_iomfb_swap_submit(dcp, resp->swap_id);
	if (!dcp_is_ext26(dcp))
		dcp_h17p_prepare_swap(dcp);
#if DCP_FW_DESKTOP_26(DCP_FW_VER)
	dcp_peer_swap_submit_26_6(dcp, &DCP_FW_SWAP(dcp), resp->swap_id);
#endif
	dcp_swap_submit(dcp, false, &DCP_FW_SWAP(dcp), dcp_swapped, NULL);
}

/* Helpers to modeset and swap, used to flush */
static void do_swap(struct apple_dcp *dcp, void *data, void *cookie)
{
#if DCP_FW_DESKTOP_26(DCP_FW_VER)
	struct DCP_FW_NAME(dcp_swap_start_req) start_req =
		dcp_swap_start_request(dcp);

	if (READ_ONCE(dcp->mode_state.valid) && dcp->connector &&
	    READ_ONCE(dcp->connector->connected))
		dcp_swap_start(dcp, false, &start_req, dcp_swap_started, NULL);
	else
		dcp_drm_crtc_vblank(dcp->crtc);
#else
	struct DCP_FW_NAME(dcp_swap_start_req) start_req = { 0 };
	struct DCP_FW_NAME(dcp_swap_submit_req) *req = &DCP_FW_SWAP(dcp);

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
#if (DCP_FW_H17(DCP_FW_VER) && DCP_FW_VER >= DCP_FW_VERSION(27, 0, 0))
		if (dcp_external_swap_stamps(dcp)) {
			bool surface = false;

			for (unsigned int i = 0; i < SWAP_SURFACES; i++)
				surface |= !req->surf_null[i];
			if (surface)
				req->swap.timestamp[1] = dcp_swap_time();
		}
#endif
		dcp_swap_start(dcp, false, &start_req, dcp_swap_started, NULL);
	} else {
		dcp_drm_crtc_vblank(dcp->crtc);
	}
#endif
}

#if DCP_FW_DESKTOP_26(DCP_FW_VER)
/*
 * GAMMA_LUT.  The scale of the firmware's gamma table is not documented, so
 * the LUT is applied through the firmware's own table, read once: entry i of
 * the result is that table at LUT[i].  This keeps whatever the firmware's
 * table does on top, and a NULL LUT restores it unchanged.
 */
static u32 dcp_gamma_lookup(const __le32 *fw, u16 v)
{
	u32 pos = v * (IOMFB_GAMMA_ENTRIES - 1), i = pos / 0xffff;
	s64 a = le32_to_cpu(fw[i]);

	if (i == IOMFB_GAMMA_ENTRIES - 1)
		return a;
	return a + div_s64((le32_to_cpu(fw[i + 1]) - a) * (pos % 0xffff), 0xffff);
}

static void dcp_gamma_set_done(struct apple_dcp *dcp, void *out, void *cookie)
{
	struct iomfb_gamma_table_resp *resp = out;

	if (resp)
		dev_dbg(dcp->dev, "A421 gamma table status %u\n",
			le32_to_cpu(resp->ret));
	do_swap(dcp, NULL, NULL);
}

static void dcp_gamma_set(struct apple_dcp *dcp)
{
	struct dcp_desktop_state *d = dcp->desktop;
	struct iomfb_gamma_table_req *req = d->gamma_out;
	u32 i;

	*req = *d->gamma_fw;
	for (i = 0; d->gamma_user && i < IOMFB_GAMMA_ENTRIES; i++) {
		req->table[0][i] = cpu_to_le32(dcp_gamma_lookup(d->gamma_fw->table[0],
								d->gamma_lut[i].red));
		req->table[1][i] = cpu_to_le32(dcp_gamma_lookup(d->gamma_fw->table[1],
								d->gamma_lut[i].green));
		req->table[2][i] = cpu_to_le32(dcp_gamma_lookup(d->gamma_fw->table[2],
								d->gamma_lut[i].blue));
	}
	dcp_push(dcp, false, &dcp_set_gamma_table_method, sizeof(*req),
		 sizeof(struct iomfb_gamma_table_resp), req, dcp_gamma_set_done,
		 NULL);
}

static void dcp_gamma_read(struct apple_dcp *dcp, void *out, void *cookie)
{
	struct dcp_desktop_state *d = dcp->desktop;
	struct iomfb_gamma_table_resp *resp = out;
	u32 ch, i;

	/* A table to compose with rises from its first to its last entry. */
	for (ch = 0; resp && !le32_to_cpu(resp->ret) && ch < 3; ch++) {
		for (i = 1; i < IOMFB_GAMMA_ENTRIES; i++)
			if (le32_to_cpu(resp->table[ch][i]) <
			    le32_to_cpu(resp->table[ch][i - 1]))
				break;
		if (i < IOMFB_GAMMA_ENTRIES ||
		    !le32_to_cpu(resp->table[ch][IOMFB_GAMMA_ENTRIES - 1]))
			break;
	}
	if (!resp || ch < 3) {
		/* External firmware can return no table with status 2. */
		if (resp && !dcp_has_panel(dcp) && le32_to_cpu(resp->ret) == 2 &&
		    !memchr_inv(resp->table, 0, sizeof(resp->table)))
			dev_dbg(dcp->dev,
				"A420 returned no gamma table (status 2), GAMMA_LUT ignored\n");
		else
			dev_warn(dcp->dev, "unusable firmware gamma table, GAMMA_LUT ignored\n");
		d->gamma_unusable = true;
		do_swap(dcp, NULL, NULL);
		return;
	}
	memset(d->gamma_fw, 0, sizeof(*d->gamma_fw));
	memcpy(d->gamma_fw->table, resp->table, sizeof(resp->table));
	d->gamma_fw_read = true;
	dev_dbg(dcp->dev, "A420 firmware gamma table read\n");
	dcp_gamma_set(dcp);
}

/* After the colour matrix: send a changed gamma table, then the swap. */
static void dcp_gamma_then_swap(struct apple_dcp *dcp, void *out, void *cookie)
{
	static struct iomfb_gamma_table_req empty;
	struct dcp_desktop_state *d = dcp->desktop;

	if (!d->gamma_pending || d->gamma_unusable) {
		do_swap(dcp, NULL, NULL);
		return;
	}
	d->gamma_pending = false;
	if (d->gamma_fw_read)
		dcp_gamma_set(dcp);
	else
		dcp_push(dcp, false, &dcp_get_gamma_table_method, sizeof(empty),
			 sizeof(struct iomfb_gamma_table_resp), &empty,
			 dcp_gamma_read, NULL);
}

/* Takes a GAMMA_LUT change from the CRTC state; false if there is nothing to send. */
static bool dcp_gamma_update(struct apple_dcp *dcp, struct drm_crtc_state *crtc_state)
{
	struct dcp_desktop_state *d = dcp->desktop;
	bool user = crtc_state->gamma_lut;

	if (!crtc_state->color_mgmt_changed || d->gamma_unusable ||
	    (!user && !d->gamma_user))
		return false;
	if (user && drm_color_lut_size(crtc_state->gamma_lut) != IOMFB_GAMMA_ENTRIES)
		return false;
	if (!d->gamma_fw) {
		d->gamma_fw = devm_kcalloc(dcp->dev, 2, sizeof(*d->gamma_fw),
					   GFP_KERNEL);
		if (!d->gamma_fw)
			return false;
		d->gamma_out = d->gamma_fw + 1;
	}
	if (user)
		memcpy(d->gamma_lut, crtc_state->gamma_lut->data, sizeof(d->gamma_lut));
	d->gamma_user = user;
	return true;
}
#endif

#if DCP_FW_H17(DCP_FW_VER)
void iomfb_present_backlight_h17p(struct apple_dcp *dcp)
{
	do_swap(dcp, NULL, NULL);
}

static void dcp_set_matrix_done(struct apple_dcp *dcp, void *out, void *cookie);

/* The swap after the colour matrix and its brightness correction, as after a power-on. */
void iomfb_present_matrix_h17p(struct apple_dcp *dcp)
{
	struct iomfb_set_matrix_req mat = {
		.location = 9,
	};

	dcp_ctm_to_fw(mat.matrix, NULL);
	iomfb_set_matrix(dcp, false, &mat, dcp_set_matrix_done, NULL);
}
#endif

/* After the colour matrix: the gamma table where there is one, then the swap. */
static void dcp_matrix_next(struct apple_dcp *dcp, void *out, void *cookie)
{
#if DCP_FW_DESKTOP_26(DCP_FW_VER)
	dcp_gamma_then_swap(dcp, NULL, NULL);
#else
	do_swap(dcp, NULL, NULL);
#endif
}

/*
 * The colour matrix reply, then the rest of the colour update and the swap.
 * A refused matrix leaves the previous one in place, so say so.
 */
static void dcp_set_matrix_done(struct apple_dcp *dcp, void *out, void *cookie)
{
	struct iomfb_set_matrix_resp *resp = out;

	if (!resp || resp->ret)
		dev_warn_ratelimited(dcp->dev, "colour matrix not applied (%s %u)\n",
				     resp ? "status" : "no reply",
				     resp ? resp->ret : 0);
	/*
	 * External native26 displays take no brightness correction; their
	 * swap follows the matrix directly.
	 */
	if (dcp_is_ext26(dcp)) {
		dcp_matrix_next(dcp, NULL, NULL);
		return;
	}
	/*
	 * Every known working matrix update follows set_matrix with identity
	 * brightness correction, 0x10000 (1.0 in 16.16), before the swap.
	 * H17P requires it, and on 12.3/13.5 a location 9 matrix stays in
	 * the firmware's cache without it. The reply is not a status.
	 */
	if (dcp_methods[iomfbep_set_brightness_correction].tag[0]) {
		u32 identity = 0x10000;

		dcp_set_brightness_correction(dcp, false, &identity,
					      dcp_matrix_next, NULL);
		return;
	}
	dcp_matrix_next(dcp, NULL, NULL);
}

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

#if DCP_FW_DESKTOP_26(DCP_FW_VER)
struct dcp_modeset_cookie {
	/* Must be first: release_wait_cookie() frees this allocation. */
	struct dcp_wait_cookie wait;
	struct dcp_set_digital_out_mode_req actual;
	struct dcp_set_digital_out_mode_req bootstrap;
	bool use_bootstrap;
	/* Adaptive sync is programmed on (a nonzero minimum rate). */
	bool vrr;
};

/* Finish only after the RPC which contained the unknown callback has replied. */
static bool dcp_modeset_callback_failed(struct apple_dcp *dcp, void *cookie)
{
	if (!READ_ONCE(dcp->desktop->modeset_callback_error))
		return false;
	complete_set_digital_out_mode(dcp, NULL, cookie);
	return true;
}

/*
 * get_mode_info (A364) fills the firmware's mode record for a timing and
 * color pair before set_digital_out_mode (A413) selects it.  The request
 * bytes the firmware does not read are 0xaa, as the firmware's own client
 * leaves them.
 */
static void dcp_get_mode_info(struct apple_dcp *dcp,
			      const struct dcp_set_digital_out_mode_req *mode,
			      dcp_callback_t cb, void *cookie)
{
	struct dcp_get_mode_info_req_v26_6 req;

	memset(&req, 0xaa, sizeof(req));
	memset(req.mode, 0, sizeof(req.mode));
	req.mode_null = 0;
	req.timing_mode_id = mode->timing_mode_id;
	req.color_mode_id = mode->color_mode_id;
	req.result_null = 0;
	memset(req.padding1, 0, sizeof(req.padding1));

	dcp_push(dcp, false, &dcp_get_mode_info_method, sizeof(req),
		 sizeof(struct dcp_get_mode_info_resp_v26_6), &req, cb, cookie);
}

static void dcp_modeset_actual_done(struct apple_dcp *dcp, void *data,
				    void *cookie)
{
	if (dcp_modeset_callback_failed(dcp, cookie))
		return;
	complete_set_digital_out_mode(dcp, data, cookie);
}

static void dcp_modeset_after_actual_info(struct apple_dcp *dcp, void *data,
					  void *cookie)
{
	struct dcp_modeset_cookie *modeset = cookie;

	if (dcp_modeset_callback_failed(dcp, cookie))
		return;
	if (!data) {
		complete_set_digital_out_mode(dcp, NULL, cookie);
		return;
	}
	dcp_set_digital_out_mode(dcp, false, &modeset->actual,
				 dcp_modeset_actual_done, cookie);
}

static void dcp_modeset_after_bootstrap(struct apple_dcp *dcp, void *data,
					void *cookie)
{
	struct dcp_modeset_cookie *modeset = cookie;

	if (dcp_modeset_callback_failed(dcp, cookie))
		return;
	if (!data || get_unaligned_le32(data)) {
		dcp_modeset_actual_done(dcp, data, cookie);
		return;
	}
	dcp_get_mode_info(dcp, &modeset->actual, dcp_modeset_after_actual_info,
			  cookie);
}

static void dcp_modeset_after_bootstrap_info(struct apple_dcp *dcp, void *data,
					     void *cookie)
{
	struct dcp_modeset_cookie *modeset = cookie;

	if (dcp_modeset_callback_failed(dcp, cookie))
		return;
	if (!data) {
		complete_set_digital_out_mode(dcp, NULL, cookie);
		return;
	}
	dcp_set_digital_out_mode(dcp, false, &modeset->bootstrap,
				 dcp_modeset_after_bootstrap, cookie);
}

/* Selects the cookie's mode, through the bootstrap color element first. */
static void dcp_modeset_select(struct apple_dcp *dcp, void *out, void *cookie)
{
	struct dcp_modeset_cookie *modeset = cookie;

	if (modeset->use_bootstrap)
		dcp_get_mode_info(dcp, &modeset->bootstrap,
				  dcp_modeset_after_bootstrap_info, cookie);
	else
		dcp_get_mode_info(dcp, &modeset->actual,
				  dcp_modeset_after_actual_info, cookie);
}

int DCP_FW_NAME(iomfb_modeset)(struct apple_dcp *dcp,
			       struct drm_crtc_state *crtc_state)
{
	struct dcp_display_mode selected, *mode = &selected;
	struct dcp_modeset_cookie *modeset;
	struct dcp_wait_cookie *cookie;
	struct dcp_color_mode *cmode = NULL;
	int ret;

	if (dcp->crashed)
		return -ENODEV;
	/* A timed-out command still owns its channel until firmware replies. */
	if (READ_ONCE(dcp->ch_cmd.depth))
		return -EBUSY;

	if (!lookup_mode(dcp, &crtc_state->mode, mode)) {
		dev_err(dcp->dev, "no match for " DRM_MODE_FMT "\n",
			DRM_MODE_ARG(&crtc_state->mode));
		return -EIO;
	}

	/* An HDR commit scans out through the timing's PQ element. */
	if (dcp->pq_requested && mode->pq.score >= 0)
		mode->color_mode_id = mode->pq.id;

	dev_dbg(dcp->dev,
		"set_digital_out_mode(color:%d timing:%d) " DRM_MODE_FMT "\n",
		mode->color_mode_id, mode->timing_mode_id,
		DRM_MODE_ARG(&crtc_state->mode));
	if (mode->color_mode_id == mode->pq.id && mode->pq.score >= 0)
		cmode = &mode->pq;
	else if (mode->color_mode_id == mode->sdr_rgb.id)
		cmode = &mode->sdr_rgb;
	else if (mode->color_mode_id == mode->sdr_444.id)
		cmode = &mode->sdr_444;
	else if (mode->color_mode_id == mode->sdr.id)
		cmode = &mode->sdr;
	else if (mode->color_mode_id == mode->best.id)
		cmode = &mode->best;
	if (cmode)
		dev_dbg(dcp->dev,
			"color mode depth:%hhu format:%u colorimetry:%u eotf:%u range:%u vrr:%u\n",
			cmode->depth, cmode->format, cmode->colorimetry,
			cmode->eotf, cmode->range, mode->vrr);

	dcp->mode = (struct dcp_set_digital_out_mode_req) {
		.color_mode_id = mode->color_mode_id,
		.timing_mode_id = mode->timing_mode_id
	};
	dcp->output_pq = dcp->pq_requested && mode->pq.score >= 0;

	/* VRR modes need swap timestamps, see iomfb_flush(); so do fixed internal timings. */
	dcp->use_timestamps = mode->needs_timestamps;

	/* A display within one pipe may let the other input have a pipe again. */
	{
		bool narrow = crtc_state->mode.hdisplay <= DCP_DESKTOP_PIPE_MAX_WIDTH;

		WRITE_ONCE(dcp->typec_narrow, narrow);
		if (narrow)
			dcp_fabric_hybrid_kick(dcp, 1000);
	}

	/*
	 * A mode wider than one pipe goes to both pipes of a reserved pair,
	 * with adaptive sync set the same on both.  The pair takes the mode's
	 * VRR range from its first install, whatever the CRTC asks: switching
	 * VRR on later would stop and install the pair again, and an install
	 * that fails (the sink dropped its link meanwhile) leaves the display
	 * dark.  Swaps keep their timestamps, so the compositor still paces it.
	 */
	if (!dcp->main_display) {
		dev_dbg(dcp->dev, "modeset %ux%u vrr %u min %#x enabled %u\n",
			crtc_state->mode.hdisplay, crtc_state->mode.vdisplay,
			mode->vrr, mode->min_vrr, crtc_state->vrr_enabled);
		ret = dcp_peer_drm_modeset_26_6(dcp, crtc_state->mode.hdisplay,
						mode->timing_mode_id,
						mode->color_mode_id,
						mode->vrr ? mode->min_vrr : 0);
		if (ret != -EOPNOTSUPP) {
			dcp->desktop->mode_live = !ret;
			/* The pair runs with the mode's adaptive-sync rate. */
			dcp->vrr_enabled = !ret && mode->vrr;
			if (!ret)
				atomic_inc(&dcp->modesets_done);
			return ret;
		}
	}

	/*
	 * The firmware changes a fixed panel's mode through the swap geometry,
	 * not through get_mode_info and set_digital_out_mode.  Accept the
	 * single fixed timing it advertises, already matched above.
	 */
	if (dcp_has_panel(dcp) && dcp->main_display &&
	    READ_ONCE(dcp->nr_modes) == 1 && !mode->vrr) {
		dcp->desktop->mode_live = true;
		return 0;
	}

	modeset = kzalloc_obj(*modeset);
	if (!modeset)
		return -ENOMEM;
	cookie = &modeset->wait;
	modeset->actual = dcp->mode;
	/*
	 * A VRR timing goes in with adaptive sync on whatever VRR_ENABLED
	 * says, at the mode's minimum rate.  Changing adaptive sync through a
	 * mode set tears the link down (the panel goes dark for about 200 ms),
	 * so VRR_ENABLED only moves the minimum rate on the live mode, see
	 * dcp_vrr_update().  Only the panel takes a new minimum on a live mode;
	 * an external processor keeps installing what VRR_ENABLED asks.  After
	 * an adaptive swap timed out (dcp_swap_watchdog), stay fixed.
	 */
	modeset->vrr = mode->vrr &&
		       (dcp_vrr_live(dcp) || crtc_state->vrr_enabled) &&
		       !READ_ONCE(dcp->desktop->vrr_failed);
	dcp->desktop->vrr_min_on = 0;

	cookie->status = ~0U;
	init_completion(&cookie->done);
	kref_init(&cookie->refcount);
	/* increase refcount to ensure the receiver has a reference */
	kref_get(&cookie->refcount);

	dcp->swap_submit_timestamp = 0;

	/* A pipe without a live mode is bootstrapped first. */
	if (!dcp->desktop->mode_live && mode->has_bootstrap) {
		modeset->use_bootstrap = true;
		modeset->bootstrap = (struct dcp_set_digital_out_mode_req) {
			.color_mode_id = mode->bootstrap_color_id,
			.timing_mode_id = mode->timing_mode_id,
		};
		dev_dbg(dcp->dev,
			 "bootstrapping the display pipe (color:%u timing:%u)\n",
			 modeset->bootstrap.color_mode_id,
			 modeset->bootstrap.timing_mode_id);
	}

	WRITE_ONCE(dcp->desktop->modeset_callback_error, false);
	WRITE_ONCE(dcp->desktop->modeset_active, true);
	/*
	 * Adaptive sync (IOMFBParameter 14, minimum rate in 16.16 Hz, 0 = off)
	 * goes before the mode is selected; the firmware then reports the mode
	 * set with VRR on.  A fixed mode after adaptive sync was on turns it
	 * off explicitly.
	 */
	if (mode->vrr || dcp->vrr_enabled) {
		struct dcp_set_parameter_dcp param = {
			.param = IOMFBPARAM_ADAPTIVE_SYNC,
			.value = { modeset->vrr ? mode->min_vrr : 0 },
			.count = 3,
		};

		dcp_set_parameter_dcp(dcp, false, &param, dcp_modeset_select,
				      cookie);
	} else {
		dcp_modeset_select(dcp, NULL, cookie);
	}

	/*
	 * The DCP firmware has an internal timeout of ~8 seconds for
	 * modesets. Add an extra 500ms to safe side that the modeset
	 * call has returned.
	 */
	ret = wait_for_completion_timeout(&cookie->done,
					  msecs_to_jiffies(8500));

	if (ret == 0) {
		dev_err(dcp->dev, "set_digital_out_mode timed out\n");
		ret = -EIO;
	} else if (ret < 0) {
		dev_err(dcp->dev,
			"waiting on set_digital_out_mode failed:%d\n", ret);
		ret = -EIO;
	} else if (READ_ONCE(dcp->desktop->modeset_callback_error)) {
		dev_err(dcp->dev, "unsupported callback during mode set\n");
		ret = -EOPNOTSUPP;
	} else if (!READ_ONCE(cookie->replied)) {
		dev_err(dcp->dev, "set_digital_out_mode got no reply\n");
		ret = -EIO;
	} else if (READ_ONCE(cookie->status)) {
		dev_err(dcp->dev, "set_digital_out_mode returned 0x%08x\n",
			READ_ONCE(cookie->status));
		ret = -EIO;
	} else {
		dev_dbg(dcp->dev,
			"set_digital_out_mode finished with %d to spare\n",
			jiffies_to_msecs(ret));
		ret = 0;
	}
	WRITE_ONCE(dcp->desktop->modeset_active, false);
	dcp->vrr_enabled = !ret && modeset->vrr;
	if (dcp->vrr_enabled && dcp_vrr_live(dcp)) {
		/*
		 * Fixed refresh holds the minimum at the timing's own rate,
		 * never above the sink's maximum (a 280.001 Hz timing reports
		 * a maximum just over 280 Hz).
		 */
		dcp->desktop->vrr_min_on = mode->min_vrr;
		dcp->desktop->vrr_min_off = drm_mode_vrefresh(&crtc_state->mode) << 16;
		if (mode->max_vrr && mode->max_vrr < dcp->desktop->vrr_min_off)
			dcp->desktop->vrr_min_off = mode->max_vrr;
		dcp->desktop->vrr_min_sent = mode->min_vrr;
	}
	kref_put(&cookie->refcount, release_wait_cookie);

	dcp->desktop->mode_live = !ret;
	if (!ret)
		atomic_inc(&dcp->modesets_done);
	return ret;
}
#else
/* DCP applies Adaptive Sync changes when the display mode is reselected. */
static void dcp_on_set_adaptive_sync(struct apple_dcp *dcp, void *out,
				     void *cookie)
{
#if (DCP_FW_H17(DCP_FW_VER) && DCP_FW_VER >= DCP_FW_VERSION(27, 0, 0))
	if (!out || get_unaligned_le32(out)) {
		struct dcp_wait_cookie *wait = cookie;

		wait->status = -EOPNOTSUPP;
		complete(&wait->done);
		kref_put(&wait->refcount, release_wait_cookie);
		return;
	}
#endif
	dcp_set_digital_out_mode(dcp, false, &dcp->mode,
				 complete_set_digital_out_mode, cookie);
}

static void dcp_set_adaptive_sync(struct apple_dcp *dcp, u32 min_vrr,
				  void *cookie)
{
	struct dcp_set_parameter_dcp param = {
		.param =
#if (DCP_FW_H17(DCP_FW_VER) && DCP_FW_VER >= DCP_FW_VERSION(27, 0, 0))
		15,
#else
		IOMFBPARAM_ADAPTIVE_SYNC,
#endif
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
			       struct drm_crtc_state *crtc_state
#if DCP_FW_H17(DCP_FW_VER) && DCP_FW_VER < DCP_FW_VERSION(27, 0, 0)
			       , struct drm_plane_state *primary
#endif
			       )
{
	struct dcp_display_mode selected, *mode = &selected;
	struct dcp_wait_cookie *cookie;
	struct dcp_color_mode *cmode = NULL;
	bool paired = false;
	bool keep_mode = false;
	int ret;

#if DCP_FW_H17(DCP_FW_VER) && DCP_FW_VER < DCP_FW_VERSION(27, 0, 0)
	if (dcp_is_ext26(dcp)) {
		u32 pitch = primary && primary->fb ? primary->fb->pitches[0] : dcp->ext26_stride;

		if (READ_ONCE(dcp->ext26_blocked))
			return -EIO;
		if (READ_ONCE(dcp->pipe_enabled_h17p) &&
		    (pitch != dcp->ext26_stride ||
		     !drm_mode_equal(&dcp->ext26_mode, &crtc_state->mode))) {
			/* A layout change cannot resume the old gentle-blank surface. */
			ret = dcp_ext26_layout_stop(dcp);
			if (ret)
				return ret;
		}
		dcp->ext26_stride = pitch;
		if ((READ_ONCE(dcp->ext26_blanked) || READ_ONCE(dcp->ext26_normal)) &&
		    READ_ONCE(dcp->pipe_enabled_h17p) &&
		    dcp->ext26_generation == READ_ONCE(dcp->typec_generation) &&
		    drm_mode_equal(&dcp->ext26_mode, &crtc_state->mode)) {
			WRITE_ONCE(dcp->ext26_blanked, false);
			return 0;
		}
		WRITE_ONCE(dcp->ext26_normal, false);
		WRITE_ONCE(dcp->ext26_power_sent, false);
		dcp->ext26_generation = READ_ONCE(dcp->typec_generation);
		reinit_completion(&dcp->ext26_ready);
	}

#endif

	if (!lookup_mode(dcp, &crtc_state->mode, mode)) {
		dev_err(dcp->dev, "no match for " DRM_MODE_FMT "\n",
			DRM_MODE_ARG(&crtc_state->mode));
		return -EIO;
	}

	/* An HDR commit scans out through the timing's PQ element. */
	if (dcp->pq_requested && mode->pq.score >= 0)
		mode->color_mode_id = mode->pq.id;

	/* The connection's audio link runs through the mode set. */
	av_audio_before_modeset(dcp);

	dev_info(dcp->dev,
		 "set_digital_out_mode(color:%d timing:%d) " DRM_MODE_FMT "\n",
		 mode->color_mode_id, mode->timing_mode_id,
		 DRM_MODE_ARG(&crtc_state->mode));
	if (mode->color_mode_id == mode->pq.id && mode->pq.score >= 0)
		cmode = &mode->pq;
	else if (mode->color_mode_id == mode->sdr_rgb.id)
		cmode = &mode->sdr_rgb;
	else if (mode->color_mode_id == mode->sdr_444.id)
		cmode = &mode->sdr_444;
	else if (mode->color_mode_id == mode->sdr.id)
		cmode = &mode->sdr;
	else if (mode->color_mode_id == mode->best.id)
		cmode = &mode->best;
	if (dcp_is_ext26(dcp)) {
		if (mode->sdr_rgb.score < 0 || mode->sdr_rgb.depth != 8 ||
		    mode->sdr_rgb.format != DCP_COLOR_FORMAT_RGB || crtc_state->vrr_enabled)
			return -EINVAL;
		mode->color_mode_id = mode->sdr_rgb.id;
		mode->vrr = false;
		cmode = &mode->sdr_rgb;
	}
	if (cmode)
		dev_info(dcp->dev,
			"set_digital_out_mode() color mode depth:%hhu format:%u "
			"colorimetry:%u eotf:%u range:%u vrr:%u\n", cmode->depth,
			cmode->format, cmode->colorimetry, cmode->eotf,
			cmode->range, mode->vrr);

#if (DCP_FW_H17(DCP_FW_VER) && DCP_FW_VER >= DCP_FW_VERSION(27, 0, 0))
	/*
	 * After a V27 sleep the panel's firmware brings its link and its mode back
	 * itself once frames arrive; macOS sends no mode set then, and one
	 * sent before the link is up waits out the firmware's timeout.
	 */
	keep_mode = dcp->v27_j714s && xchg(&dcp->v27_keep_mode, false) &&
		    dcp->mode.color_mode_id == mode->color_mode_id &&
		    dcp->mode.timing_mode_id == mode->timing_mode_id;
#endif
	dcp->mode = (struct dcp_set_digital_out_mode_req){
		.color_mode_id = mode->color_mode_id,
		.timing_mode_id = mode->timing_mode_id
	};
	dcp->output_pq = dcp->pq_requested && mode->pq.score >= 0;

	/* Built-in ProMotion panels require timestamps even in fixed-120 mode. */
	dcp->use_timestamps = mode->needs_timestamps && dcp->main_display;

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
#if DCP_FW_H17(DCP_FW_VER) && DCP_FW_VER < DCP_FW_VERSION(27, 0, 0)
	if (dcp_is_ext26(dcp)) {
		ret = ext26_open_client(dcp);
		if (ret) {
			kref_put(&cookie->refcount, release_wait_cookie);
			kref_put(&cookie->refcount, release_wait_cookie);
			return ret;
		}
	}
#endif

	if (keep_mode) {
		dev_info(dcp->dev, "keeping the panel's mode after the sleep\n");
		complete_set_digital_out_mode(dcp, NULL, cookie);
	} else if ((DCP_INHERIT_BOOT_MODE || DCP_FW_NAME(dcp_fw27_keep_panel_mode)(dcp)) &&
	    !dcp_is_ext26(dcp) &&
	    dcp->hw.iomfb_method_profile !=
	    DCP_IOMFB_METHODS_H17P_EXTERNAL) {
		dev_info(dcp->dev, "inheriting the bootloader's display mode (%u fixed timing(s))\n",
			 READ_ONCE(dcp->nr_modes));
		complete_set_digital_out_mode(dcp, NULL, cookie);
	} else
#if (DCP_FW_H17(DCP_FW_VER) && DCP_FW_VER >= DCP_FW_VERSION(27, 0, 0))
	if ((ret = dcp_peer_modeset_v27(dcp, mode->timing_mode_id,
				      mode->color_mode_id,
				      crtc_state->vrr_enabled && mode->vrr ? mode->min_vrr : 0)) != -EOPNOTSUPP) {
		cookie->status = ret;
		paired = true;
		if (!ret)
			crtc_state->color_mgmt_changed = true;
		complete_set_digital_out_mode(dcp, NULL, cookie);
	} else if (dcp_peer_needs_pair_v27(dcp, mode->mode.clock)) {
		/* No pair for a timing one engine cannot drive: never single-pipe it. */
		cookie->status = -EINVAL;
		paired = true;
		complete_set_digital_out_mode(dcp, NULL, cookie);
	} else
#endif
	if (mode->vrr &&
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
	/* A paired mode set that failed left the CRTC with no timing. */
	if (paired && READ_ONCE(cookie->status)) {
		ret = (int)READ_ONCE(cookie->status);
		dev_err_ratelimited(dcp->dev, "paired mode set failed: %d\n", ret);
		kref_put(&cookie->refcount, release_wait_cookie);
		return ret;
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
#if DCP_FW_H17(DCP_FW_VER) && DCP_FW_VER < DCP_FW_VERSION(27, 0, 0)
	if (dcp_is_ext26(dcp)) {
		ret = READ_ONCE(cookie->replied) && !READ_ONCE(cookie->status) ? 0 : -EREMOTEIO;
		kref_put(&cookie->refcount, release_wait_cookie);
		if (!ret)
			ret = ext26_activate(dcp);
		if (!ret)
			drm_mode_copy(&dcp->ext26_mode, &crtc_state->mode);
		return ret;
	}
#endif
	kref_put(&cookie->refcount, release_wait_cookie);
	dcp->vrr_enabled = mode->vrr && crtc_state->vrr_enabled;
	/* A firmware 27 pair took them on both engines (dcp_peer_finish_v27()). */
	if (dcp->hw.iomfb_method_profile == DCP_IOMFB_METHODS_H17P_EXTERNAL ||
	    (dcp->fw_compat == DCP_FIRMWARE_V_27_0 && !dcp_has_panel(dcp) && !paired))
		dcp_external_hdr_hints(dcp, dcp->output_pq);
	dcp_link_report(dcp, &crtc_state->mode);

	return 0;
}
#endif

#if DCP_FW_DESKTOP_26(DCP_FW_VER)
/*
 * VRR_ENABLED on a live VRR timing.  The firmware takes a new minimum rate
 * (IOMFBParameter 14) without a mode set: the mode's own minimum lets the
 * display follow the frames, the nominal rate holds it fixed.  Sent ahead
 * of the frame's colour updates and swap, which @next continues with.
 * Returns true when it took over the frame.
 */
static bool dcp_vrr_update(struct apple_dcp *dcp, struct drm_crtc_state *crtc_state,
			   dcp_callback_t next)
{
	struct dcp_desktop_state *d = dcp->desktop;
	struct dcp_set_parameter_dcp param = {
		.param = IOMFBPARAM_ADAPTIVE_SYNC,
		.count = 3,
	};
	u32 min;

	if (!dcp->vrr_enabled || !d->vrr_min_on)
		return false;
	min = crtc_state->vrr_enabled ? d->vrr_min_on : d->vrr_min_off;
	if (min == d->vrr_min_sent)
		return false;
	d->vrr_min_sent = min;
	/* minRR (16.16 Hz), then mediaTargetRate and the fractional rate */
	param.value[0] = min;
	dev_dbg(dcp->dev, "adaptive sync minimum %#x (VRR_ENABLED %u)\n", min,
		crtc_state->vrr_enabled);
	dcp_set_parameter_dcp(dcp, false, &param, next, NULL);
	return true;
}

/* After an adaptive sync update: the frame's colour matrix, gamma and swap. */
static void dcp_matrix_then_swap(struct apple_dcp *dcp, void *out, void *cookie)
{
	iomfb_set_matrix(dcp, false, &dcp->desktop->matrix, dcp_set_matrix_done, NULL);
}
#endif

void DCP_FW_NAME(iomfb_flush)(struct apple_dcp *dcp, struct drm_crtc *crtc, struct drm_atomic_commit *commit)
{
	struct drm_plane *plane;
	struct drm_plane_state *new_state, *old_state;
	struct drm_crtc_state *crtc_state;
	struct DCP_FW_NAME(dcp_swap_submit_req) *req = &DCP_FW_SWAP(dcp);
	int plane_idx, l;
	int has_surface = 0;

#if !DCP_FW_DESKTOP_26(DCP_FW_VER)
	bool update_brightness;
#endif

	crtc_state = drm_atomic_get_new_crtc_state(commit, crtc);

	/* Reset all surfaces to defaults */
	memset(req, 0, sizeof(*req));
	for (l = 0; l < SWAP_SURFACES; l++)
		req->surf_null[l] = true;
#if DCP_FW_VER >= DCP_FW_VERSION(13, 2, 0)
#if !DCP_FW_H17(DCP_FW_VER)
	for (l = 0; l < ARRAY_SIZE(req->surf2_null); l++)
		req->surf2_null[l] = true;
#endif
	req->unknown_pointer_null = true;
	req->unknown_output_null = true;
#endif
#if DCP_FW_DESKTOP_26(DCP_FW_VER)
	/* The firmware's own client sets both on every visible swap. */
	req->swap.presents_surface = 1;
	req->swap.has_completion = 1;
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

#if !DCP_FW_H17(DCP_FW_VER) && !DCP_FW_DESKTOP_26(DCP_FW_VER)
		req->swap.surf_ids[apl_plane->iomfb_surf] = plane->base.id;
#endif

		if (dcp->notch_height > 0)
			req->swap.dst_rect[l].y += dcp->notch_height;

		req->surf_iova[l] = apple_state->iova;
		req->surf[l].base = apple_state->surf;
#if DCP_FW_H17(DCP_FW_VER)
		req->surf[l].base.surface_id = 0;
#endif

		/* Internal panels: tag untagged RGB with the panel's own
		 * primaries (P3-D65, as its EDID reports), so the firmware
		 * shows the compositor's values unconverted. External
		 * displays keep the unspecified tag; user space reads their
		 * colorimetry from the EDID.
		 */
		if (dcp->connector_type == DRM_MODE_CONNECTOR_eDP &&
		    req->surf[l].base.colorspace == DCP_COLORSPACE_UNSPECIFIED)
			req->surf[l].base.colorspace = DCP_COLORSPACE_P3_D65;

		/*
		 * On a PQ output the compositor has already encoded RGB
		 * surfaces for the sink: BT.2020 primaries, PQ code values.
		 * An EDR panel takes the same and maps them into its headroom.
		 */
		if ((dcp->output_pq || dcp->output_edr) &&
		    !new_state->fb->format->is_yuv) {
			req->surf[l].base.colorspace = DCP_COLORSPACE_BT2020;
			req->surf[l].base.xfer_func = DCP_XFER_FUNC_PQ;
		}

#if DCP_FW_H17(DCP_FW_VER)
		/* H17P external SDR RGB needs the unspecified (sRGB) source
		 * tag. P3-D65 tagging washes out the same pixel values on this
		 * profile.
		 * Keep YUV, other source spaces and other profiles unchanged.
		 */
		if (dcp->hw.iomfb_method_profile ==
			    DCP_IOMFB_METHODS_H17P_EXTERNAL &&
		    !new_state->fb->format->is_yuv &&
		    req->surf[l].base.colorspace == DCP_COLORSPACE_P3_D65)
			req->surf[l].base.colorspace = DCP_COLORSPACE_UNSPECIFIED;
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

#if DCP_FW_DESKTOP_26(DCP_FW_VER)
	{
		u64 deadline = dcp_desktop_present_deadline(has_surface, dcp->use_timestamps);

		/* Types 1, 2 and 7 carry the same desktop presentation deadline. */
		req->swap.timestamp[0] = deadline;
		req->swap.timestamp[1] = deadline;
		req->swap.timestamp[6] = deadline;
	}
#else
	if (has_surface && (dcp->use_timestamps || dcp->vrr_enabled)) {
		u64 timestamp = dcp_swap_time();
		u64 submit_timestamp =
			dcp_swap_prev_time(timestamp, dcp->swap_submit_timestamp);

		/*
		 * The firmware takes continuous-time values here, in its own
		 * timebase (see dcp_swap_time()). Empirical
		 * testing shows that using the current submission time together
		 * with the previous accepted swap makes DCP follow swap pacing.
		 */
		/* Firmware 12.x/13.x requires timestamp types 1, 2, and 7. */
		req->swap.timestamp[0] = timestamp;
		req->swap.timestamp[1] = submit_timestamp;
		req->swap.timestamp[6] = timestamp;

		trace_iomfb_vrr_timestamps(dcp, dcp->vrr_enabled, timestamp,
					   submit_timestamp);
	}
#endif

	/* These fields should be set together */
	req->swap.swap_completed = req->swap.swap_enabled;

#if DCP_FW_DESKTOP_26(DCP_FW_VER)
	/* The shared ramp is encoded after swap_start, at submission time. */
	dcp->brightness.update = false;
#else
	/* update brightness if changed */
	update_brightness = dcp_has_panel(dcp) && dcp->brightness.update;
#if DCP_FW_H17(DCP_FW_VER)
	update_brightness = update_brightness &&
			    dcp->hw.iomfb_method_profile == DCP_IOMFB_METHODS_H17G;
#endif
	if (update_brightness) {
#if DCP_FW_H17(DCP_FW_VER)
		/*
		 * H17G firmware ignores the DAC words and takes nits in the
		 * same swap bytes as H17P. Without a requested or reported
		 * level, send nothing rather than 0 nits.
		 */
		int nits = dcp->brightness.bl_dev ?
			   dcp->brightness.bl_dev->props.brightness :
			   dcp->brightness.nits;

		if (nits > 0)
			iomfb_encode_backlight_swap_h17p((u8 *)&req->swap, nits,
							 dcp->brightness.maximum,
							 true);
#else
#if DCP_FW_LAYOUT_14_7(DCP_FW_VER)
		req->swap.bl_update = 1;
#else
		req->swap.bl_unk = 1;
#endif
		req->swap.bl_value = dcp->brightness.dac;
		req->swap.bl_power = 0x40;
#endif
		dcp->brightness.update = false;
	}
#endif

	if (crtc_state->color_mgmt_changed) {
		struct iomfb_set_matrix_req mat = {
			.location = 9,
		};

		dcp_ctm_to_fw(mat.matrix, crtc_state->ctm ?
			      crtc_state->ctm->data : NULL);

#if (DCP_FW_H17(DCP_FW_VER) && DCP_FW_VER >= DCP_FW_VERSION(27, 0, 0))
		if (dcp_peer_color_v27(dcp, &mat)) {
			dcp_peer_swap_error(dcp, "color matrix", -EIO);
			return;
		}
#endif
#if DCP_FW_DESKTOP_26(DCP_FW_VER)
		dcp->desktop->gamma_pending = dcp_gamma_update(dcp, crtc_state);
		dcp->desktop->matrix = mat;
		if (dcp_vrr_update(dcp, crtc_state, dcp_matrix_then_swap))
			return;
#endif
		iomfb_set_matrix(dcp, false, &mat, dcp_set_matrix_done, NULL);
	} else {
#if DCP_FW_DESKTOP_26(DCP_FW_VER)
		if (dcp_vrr_update(dcp, crtc_state, do_swap))
			return;
#endif
		do_swap(dcp, NULL, NULL);
	}
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
	/*
	 * Desktop 26.x publishes no initial panel level that would register
	 * the backlight (see iomfb_cb_pr_publish()): the DRM master registers
	 * it once its device is, see dcp_backlight_takeover_registered().
	 */
	complete(&dcp->start_done);
}

#if DCP_FW_DESKTOP_26(DCP_FW_VER)
static void res_max_panel_brightness(struct apple_dcp *dcp, void *out, void *cookie)
{
	u32 peak = out ? get_unaligned_le32(out) >> 16 : 0;

	/* Only a peak above the SDR range gives EDR headroom. */
	if (peak > dcp->brightness.maximum && peak <= 10000) {
		dev_info(dcp->dev, "panel peak luminance %u nits\n", peak);
		WRITE_ONCE(dcp->panel.peak_nits, peak);
	}
	if (out)
		dcp_is_main_display(dcp, false, res_is_main_display, NULL);
	else
		res_is_main_display(dcp, NULL, NULL);
}
#endif

static void init_3(struct apple_dcp *dcp, void *out, void *cookie)
{
#if DCP_FW_DESKTOP_26(DCP_FW_VER)
	if (dcp_has_panel(dcp)) {
		dcp_push(dcp, false, &dcp_max_panel_brightness_method, 0,
			 sizeof(u32), NULL, res_max_panel_brightness, NULL);
		return;
	}
#endif
	dcp_is_main_display(dcp, false, res_is_main_display, NULL);
}

#if DCP_FW_H17(DCP_FW_VER)
/*
 * The client-open sequence H17P firmware expects before the first swap, in
 * H17P method numbers (dcp_push() applies the H17G aliases):
 *
 *   A445 A411 A445 A415 A450 A031 x9 A414 A478 A457 A474 A478 A476 A478 A412
 *
 * (A031 x14 with notify_clients_form=2: see DCP_STEP_A031.)
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
	DCP_STEP_A031,	/* repeats itself once per notify-clients state */
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
		 * Cumulative states: each call repeats the previous set of
		 * notification clients and adds one, so state n has n
		 * clients. The step re-queues itself until every state is
		 * sent. notify_clients_form 0 writes the client numbers into
		 * the first slots; 1 and 2 write the flag form fw27 firmware
		 * expects, a 1 at each client's index (9 or 14 states).
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
		/* the order clients are added in, state n has the first n */
		static const u8 notify_client_order[] = {
			6, 7, 8, 10, 11, 12, 25, 26, 13, 14, 15, 16, 17,
		};
		struct dcp_pipe_cfg_031 cfg = {};
		int form = READ_ONCE(notify_clients_form);
		unsigned int states = form == 2 ? 14 : 9;
		unsigned int step = dcp->a031_step;
		unsigned int i;

		if (step >= states)
			step = states - 1;
		if (form == 1 || form == 2) {
			for (i = 0; i < step && i < ARRAY_SIZE(notify_client_order); i++)
				cfg.clients[notify_client_order[i]] = cpu_to_le32(1);
		} else {
			for (i = 0; i < step; i++)
				cfg.clients[i] = cpu_to_le32(notify_clients[step][i]);
		}
		dev_dbg(dcp->dev, "notify clients state %u of %u, form %d\n",
			step, states, form);
		dcp->a031_step++;
		/* the next state, or the step after A031 once all are sent */
		if (dcp->a031_step < states)
			ncookie = (void *)idx;
		else
			dev_info(dcp->dev, "notify clients: form %d, %u states\n",
				 form, states);

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
#if DCP_FW_DESKTOP_26(DCP_FW_VER)
	struct DCP_FW_NAME(dcp_set_power_state_req) req =
		dcp_power_state_req(0, DCP_POWER_TRANSITION_SHUTDOWN);
#else
	struct DCP_FW_NAME(dcp_set_power_state_req) req = {
		/* defaults are ok */
	};
#endif
	struct dcp_wait_cookie *cookie;
	int ret;

#if DCP_FW_H17(DCP_FW_VER)
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
