// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Copyright The Asahi Linux Contributors */

#include <linux/bitops.h>
#include <linux/soc/apple/rtkit.h>
#include <drm/drm_gem.h>
#include <linux/unaligned.h>

#include "dcp-link.h"
#include "dcpext26-wire.h"
#include "dcpext26-lifecycle.h"
#include "iomfb_h17p.h"
#include "iomfb_v12_3.h"
#include "iomfb_v13_3.h"
#include "iomfb_v14_7_tpl.h"
#include "version_utils.h"

struct dcp_callback_size {
	u32 in_len;
	u32 out_len;
	bool valid;
	/* Lengths bound what the handler reads and writes; no capture exists. */
	bool minimum;
};

/*
 * Callbacks that only arrive after startup: present notifications, swap
 * aborts, the backlight re-send request, wall-clock and clock-rate queries.
 * Their record lengths are not in the startup observations, so accept any
 * record that holds what the handler reads and the reply it writes.  The
 * dispatcher zeroes the rest of a longer reply.
 */
#define DCP_CB_AT_LEAST(in, out) \
	{ .in_len = (in), .out_len = (out), .valid = true, .minimum = true }

/* R-IOMFB-startup-run21 and R-ANALYTICS Linux boundary observations. */
static const struct dcp_callback_size callback_sizes[IOMFB_MAX_CB] = {
	[0] = { 0x0, 0x4, true },
	[1] = { 0x0, 0x4, true },
	[2] = DCP_CB_AT_LEAST(0x0, 0x0),
	[3] = { 0x4, 0x14, true },
	[6] = { 0x54, 0x50, true },
	[100] = { 0x0, 0x0, true },
	[101] = { 0x0, 0x4, true },
	[102] = { 0x44, 0x0, true },
	[104] = { 0x44, 0x0, true },
	[107] = DCP_CB_AT_LEAST(0x0, 0x0),
	[108] = { 0x0, 0x4, true },
	[109] = { 0x0, 0x4, true },
	[110] = { 0x0, 0x4, true },
	[111] = { 0x0, 0x4, true },
	[112] = { 0x0, 0x4, true },
	[113] = { 0x0, 0x4, true },
	[114] = { 0x1044, 0x1004, true },
	[117] = DCP_CB_AT_LEAST(0x0, 0x0),
	[118] = DCP_CB_AT_LEAST(0x0, 0x4),
	[119] = DCP_CB_AT_LEAST(0x0, 0x4),
	[120] = DCP_CB_AT_LEAST(0x0, 0x4),
	[121] = { 0x0, 0x4, true },
	[122] = DCP_CB_AT_LEAST(0x0, 0x1),
	[123] = { 0x0, 0x4, true },
	[124] = DCP_CB_AT_LEAST(0x0, 0x4),
	[125] = { 0x64, 0x24, true },
	[127] = { 0x4, 0x4, true },
	[128] = { 0x1008, 0x4, true },
	[129] = { 0x40, 0x4, true },
	[130] = DCP_CB_AT_LEAST(0x22, 0x14),
	[201] = { 0xc, 0x10, true },
	[202] = { 0x1a, 0x0, true },
	[206] = { 0x0, 0x4, true },
	[207] = { 0x0, 0x4, true },
	[208] = DCP_CB_AT_LEAST(0x0, 0x0),
	[209] = DCP_CB_AT_LEAST(0x0, 0x8),
	[300] = { 0x10, 0x0, true },
	[400] = { 0x4c, 0xc04, true },
	[401] = { 0x50, 0xc, true },
	[404] = DCP_CB_AT_LEAST(0x0, 0x0),
	[406] = { 0x48, 0x0, true },
	[408] = DCP_CB_AT_LEAST(0x8, 0x8),
	[411] = { 0x10, 0x1c, true },
	[413] = { 0x1048, 0x4, true },
	[414] = { 0x50, 0x4, true },
	[415] = { 0x4c, 0x4, true },
	[451] = { 0x14, 0x1c, true },
	[452] = DCP_CB_AT_LEAST(0x18, 0x14),
	[454] = { 0x4, 0x1, true },
	[552] = { 0x1044, 0x4, true },
	[561] = { 0x1044, 0x4, true },
	[563] = { 0x4c, 0x4, true },
	[565] = { 0x48, 0x4, true },
	[567] = DCP_CB_AT_LEAST(0x0, 0x1),
	[572] = DCP_CB_AT_LEAST(0x0, 0x4),
	[574] = { 0x4, 0x8, true },
	[575] = { 0x58, 0x4c, true },
	[576] = DCP_CB_AT_LEAST(0x0, 0x0),
	[582] = { 0x8, 0x4, true },
	[583] = DCP_CB_AT_LEAST(0x0, 0x0),
	[584] = DCP_CB_AT_LEAST(0x0, 0x0),
	[585] = DCP_CB_AT_LEAST(0xe4, 0xe0),
	[590] = { 0x730, 0x0, true },
	[592] = DCP_CB_AT_LEAST(0x11, 0x0),
	[593] = DCP_CB_AT_LEAST(0x4, 0x0),
	[594] = DCP_CB_AT_LEAST(0x1, 0x0),
	[595] = DCP_CB_AT_LEAST(0x0, 0x0),
	[597] = DCP_CB_AT_LEAST(0x0, 0x1),
	[598] = DCP_CB_AT_LEAST(0x0, 0x1),
	[599] = { 0x0, 0x0, true },
};

bool iomfb_validate_callback_h17p(int tag, u32 in_len, u32 out_len)
{
	const struct dcp_callback_size *size;

	if (tag < 0 || tag >= ARRAY_SIZE(callback_sizes))
		return false;
	size = &callback_sizes[tag];
	if (!size->valid)
		return false;
	if (size->minimum)
		return in_len >= size->in_len && out_len >= size->out_len;
	return size->in_len == in_len && size->out_len == out_len;
}

struct dcp_h17p_hotplug_request {
	__le64 connected;
	u8 tiled_display[0x4c];
	u8 tiled_display_null;
	u8 padding[3];
} __packed;

struct dcp_h17p_swap_info_request {
	u8 swap_info[0xe0];
	u8 swap_info_null;
	u8 padding[3];
} __packed;

static_assert(sizeof(struct dcp_h17p_hotplug_request) == 0x58);
static_assert(sizeof(struct dcp_h17p_swap_info_request) == 0xe4);
/* D590 swap_complete_ap_gated is 0x730 bytes on the wire. */
static_assert(sizeof(struct dc_swap_complete_resp_h17p) == 0x730);
static_assert(offsetof(struct dc_swap_complete_resp_h17p, swap_id) == 0);

void iomfb_serialize_present_h17p(struct dcp_present_h17p *wire,
				  const struct dcp_swap_submit_req_h17p *request)
{
	size_t tail_offset = offsetof(struct dcp_swap_submit_req_h17p, surf_iova);
	size_t tail_size = sizeof(*request) - tail_offset;

	static_assert(sizeof(request->swap) == sizeof(wire->swap) + 2);
	static_assert(sizeof(request->surf) == sizeof(wire->surf));
	static_assert(sizeof(*request) -
		      offsetof(struct dcp_swap_submit_req_h17p, surf_iova) ==
		      sizeof(wire->tail) - 2);

	memcpy(wire->swap, &request->swap, sizeof(wire->swap));
	memcpy(wire->surf, request->surf, sizeof(wire->surf));
	memcpy(wire->tail, (const u8 *)request + tail_offset, tail_size);
	memset(wire->tail + tail_size, 0, sizeof(wire->tail) - tail_size);
}

/* Integer commanded nits are exact binary64 values; no kernel FP is needed. */
static u64 iomfb_nits_binary64_h17p(u32 nits)
{
	u64 value = 0;
	unsigned int exponent;

	if (nits) {
		exponent = fls(nits) - 1;
		value = (u64)(exponent + 1023) << 52;
		value |= ((u64)nits << (52 - exponent)) & GENMASK_ULL(51, 0);
	}
	return value;
}

void iomfb_encode_backlight_h17p(struct dcp_present_h17p *wire, u32 nits,
				 u32 maximum, bool update)
{
	u64 ceiling = iomfb_nits_binary64_h17p(maximum);
	u64 unity = iomfb_nits_binary64_h17p(1);

	/* This profile does not use the older DAC/power words. */
	memset(wire->swap + 0x32f, 0, 0xd);
	memset(wire->swap + 0x354, 0, 0x9a);
	/* The ceiling pair must remain valid even on a brightness-idle frame. */
	put_unaligned_le64(ceiling, wire->swap + 0x36e);
	put_unaligned_le64(ceiling, wire->swap + 0x376);
	if (!update)
		return;

	/* Linux brightness changes and soft DPMS use the same activation bytes. */
	put_unaligned_le32(1, wire->swap + 0x354);
	wire->swap[0x358] = 1;
	wire->swap[0x359] = 1;
	wire->swap[0x35a] = 1;
	put_unaligned_le16(1, wire->swap + 0x35c);
	put_unaligned_le64(iomfb_nits_binary64_h17p(nits), wire->swap + 0x35e);
	put_unaligned_le64(unity, wire->swap + 0x37e);
	put_unaligned_le64(unity, wire->swap + 0x3e6);
}

static const struct dcp_method_entry dcp_methods[dcpep_num_methods] = {
	IOMFB_METHOD("A000", dcpep_late_init_signal),
	[dcpep_setup_video_limits] = { "dcpep_setup_video_limits",
		{'A', '0', '2', '5'}, {'A', '0', '2', '9'}, {'A', '0', '2', '9'} }, /* (0,0) */
	IOMFB_METHOD("A131", iomfbep_a131_pmu_service_matched), /* nested in D206 */
	IOMFB_METHOD("A132", iomfbep_a132_backlight_service_matched), /* nested in D207 */
	IOMFB_METHOD_EXT26("A385", "A389", dcpep_set_create_dfb), /* first call inside D121; (0,0) */
	IOMFB_METHOD_EXT26("A386", "A390", iomfbep_a358_vi_set_temperature_hint), /* nested in D100 */
	IOMFB_METHOD("A401", dcpep_start_signal),
	IOMFB_METHOD("A406", dcpep_swap_start),
	/*
	 * swap_start moved A407 -> A406 on H17P, but swap_submit did not move
	 * with it: every swap is A406 followed by A408, A407 is unused, and the
	 * A408 response is 0x0c bytes (dcp_swap_submit_resp_h17p).
	 */
	IOMFB_METHOD_EXT26("A408", "A407", dcpep_swap_submit),
	IOMFB_METHOD("A411", dcpep_set_display_device),
	IOMFB_METHOD("A412", dcpep_is_main_display),
	IOMFB_METHOD("A413", dcpep_set_digital_out_mode),
	IOMFB_METHOD("A423", iomfbep_set_matrix),
	IOMFB_METHOD("A427", iomfbep_get_color_remap_mode),
	IOMFB_METHOD("A442", dcpep_set_parameter_dcp), /* (0x28, 0x4) */
	IOMFB_METHOD("A446", dcpep_create_default_fb), /* follows A385 in D121 */
	/*
	 * The default-framebuffer block the firmware expects inside D121,
	 * straight after A446 and before A025.  Sizes are (in, out).
	 */
	IOMFB_METHOD_H17("A035", "A039", dcpep_get_dfb_compression_info), /* (0x8, 0x8) */
	IOMFB_METHOD_H17("A034", "A038", dcpep_get_dfb_info), /* (0x2c, 0x2c) */
	IOMFB_METHOD("A103", dcpep_get_dfb_layout),           /* (0xc, 0x8) */
	IOMFB_METHOD("A445", dcpep_get_dfb_state),            /* (0x4, 0x8) */
	IOMFB_METHOD("A104", dcpep_set_dfb_dimensions),       /* (0x8, 0) w,h */
	IOMFB_METHOD("A105", dcpep_commit_dfb_info),          /* (0, 0) */
	IOMFB_METHOD_H17("A033", "A037", dcpep_dfb_query),    /* (0, 0x4) */
	IOMFB_METHOD("A380", dcpep_dfb_ready_query),          /* (0, 0x4) */
	IOMFB_METHOD("A415", dcpep_pipe_cfg_415),             /* (0xc, 0xc) zeros */
	/* update_notify_clients_dcp */
	IOMFB_METHOD_H17("A031", "A035", dcpep_pipe_cfg_031), /* (0x6c, 0) */
	IOMFB_METHOD("A414", dcpep_pipe_cfg_414),             /* (0x8, 0x8) zeros */
	IOMFB_METHOD("A478", dcpep_pipe_query_478),           /* (0, 0x4) */
	IOMFB_METHOD("A474", dcpep_pipe_query_474),           /* (0, 0x4) */
	IOMFB_METHOD("A476", dcpep_pipe_set_476),             /* (0x4, 0) in 1 */
	IOMFB_METHOD("A428", dcpep_pipe_set_428),             /* (0x4, 0x4) 0x10000 */
	IOMFB_METHOD("A450", dcpep_enable_disable_video_power_savings), /* in = u32 0 */
	IOMFB_METHOD("A457", dcpep_first_client_open), /* (0, 0) */
	/* D561 (displayMinRefreshInterval) arrives while A465 is in flight */
	IOMFB_METHOD("A465", dcpep_set_display_refresh_properties),
	IOMFB_METHOD("A468", dcpep_flush_supports_power), /* follows A025 in D121, in = 1 */
	/*
	 * iomfbep_last_client_close and iomfbep_abort_swaps_dcp are deliberately
	 * absent: their H17P numbers are unverified, so the teardown paths skip
	 * them (DCP_HAS_CLIENT_TEARDOWN in iomfb_template.c).
	 */
	IOMFB_METHOD("A473", dcpep_set_power_state), /* out-of-band; out 0x8 */
	IOMFB_METHOD("A472", dcpep_register_dfb_surface), /* default FB surface, nested in D582 */
};

#define DCP_FW h17p
#define DCP_FW_VER DCP_FW_VERSION(26, 6, 0)

static int ext26_encode(struct apple_dcp *dcp, struct dcp_swap_submit_req_h17p *request);
static int ext26_activate(struct apple_dcp *dcp);
static int ext26_open_client(struct apple_dcp *dcp);
static u8 ext26_prop_start(struct apple_dcp *dcp, u32 advertised);
static void ext26_start(struct apple_dcp *dcp);
static bool ext26_abort(struct apple_dcp *dcp, int tag, void *out, void *input);

#include "iomfb_template.c"

void iomfb_apply_opaque_x_h17p(struct apple_dcp *dcp, dcp_callback_t callback,
			       void *cookie)
{
	static const struct dcp_method_entry method = {
		.name = "apply_property",
		.tag = { 'A', '3', '5', '2' },
	};
	struct dcp_apply_property_h17p property = dcp_opaque_x_property_h17p();

	dcp_push(dcp, false, &method, sizeof(property), sizeof(u32), &property,
		 callback, cookie);
}

static bool trampoline_rt_bandwidth_h17p(struct apple_dcp *dcp, int tag,
					 void *out, void *in)
{
	/*
	 * The bandwidth scratch and doorbell come from the DCP's DT node; the
	 * apple,bw-doorbell binding has no offset cell.
	 */
	u64 scratch = dcp->disp_bw_scratch_res.start +
		      dcp->disp_bw_scratch_offset;
	u64 clock_request = dcp->disp_bw_doorbell_res.start;

	trace_iomfb_callback(dcp, tag, __func__);
	if (apple_dcp_h17p_rt_bw_encode_reply(in, out, scratch, clock_request))
		dev_warn(dcp->dev, "D003 bandwidth request is malformed\n");
	return true;
}

static bool trampoline_get_frequency_h17p(struct apple_dcp *dcp, int tag,
					   void *out, void *in)
{
	const struct apple_dcp_h17p_clock_request *request = in;
	u64 rate;
	int ret;

	trace_iomfb_callback(dcp, tag, __func__);
	ret = apple_dcp_h17p_clock_rate(request, clk_get_rate(dcp->clk),
					  clk_get_rate(dcp->clk_194), &rate);
	if (ret) {
		dev_warn(dcp->dev, "unknown display clock ID %#x\n",
			 le32_to_cpu(request->clock_id));
		rate = 0;
	}

	*(__le64 *)out = cpu_to_le64(rate);
	return true;
}

static bool trampoline_analytics_h17p(struct apple_dcp *dcp, int tag,
				      void *out, void *in)
{
	if (dcp->hw.iomfb_method_profile == DCP_IOMFB_METHODS_H17G)
		return trampoline_zero(dcp, tag, out, in);

	trace_iomfb_callback(dcp, tag, __func__);
	/* The dispatcher has validated the D114 reply size. */
	memset(out, 0, 0x1004);
	*(u8 *)out = 'd';
	return true;
}

static bool trampoline_hotplug_h17p(struct apple_dcp *dcp, int tag,
				    void *out, void *in)
{
	const struct dcp_h17p_hotplug_request *request = in;

	if (!(request->tiled_display_null & 1))
		memcpy(out, request->tiled_display,
		       sizeof(request->tiled_display));
	return trampoline_hotplug(dcp, tag, out, in);
}

static bool trampoline_swap_info_h17p(struct apple_dcp *dcp, int tag,
				      void *out, void *in)
{
	const struct dcp_h17p_swap_info_request *request = in;

	trace_iomfb_callback(dcp, tag, __func__);
	if (!(request->swap_info_null & 1))
		memcpy(out, request->swap_info, sizeof(request->swap_info));
	return true;
}

struct dcp_h17p_provider_request {
	u8 scope[4];
	char name[64];
	__le32 capacity;
	u8 reserved[4];
} __packed;

struct dcp_h17p_provider_reply {
	u8 data[0xc00];
	__le32 length;
} __packed;

struct dcp_h17p_provider_property {
	const char *name;
	const char *dt_name;
	u32 length;
	const u8 *legacy_data;
};

static_assert(sizeof(struct dcp_h17p_provider_request) == 0x4c);
static_assert(sizeof(struct dcp_h17p_provider_reply) == 0xc04);

/* Compatibility data matches the independently decoded J700 disp0 ADT. */
static const u8 j700_provider_data_0[] = {
	0x0d, 0x00, 0x00, 0x00
};

static const u8 j700_provider_data_1[] = {
	0x01, 0x00, 0x00, 0x00
};

static const u8 j700_provider_data_2[] = {
	0x02, 0x00, 0x00, 0x00, 0x0a, 0x00, 0x00, 0x00, 0x32, 0x00, 0x00, 0x00,
	0x64, 0x00, 0x00, 0x00, 0x96, 0x00, 0x00, 0x00, 0xc8, 0x00, 0x00, 0x00,
	0xfa, 0x00, 0x00, 0x00, 0x2c, 0x01, 0x00, 0x00, 0x5e, 0x01, 0x00, 0x00,
	0x90, 0x01, 0x00, 0x00, 0xc2, 0x01, 0x00, 0x00, 0xf4, 0x01, 0x00, 0x00,
	0x0d, 0x02, 0x00, 0x00
};

static const u8 j700_provider_data_3[] = {
	0x64, 0x00, 0x00, 0x00
};

static const u8 j700_provider_data_4[] = {
	0x34, 0x03, 0x00, 0x00, 0x84, 0x03, 0x00, 0x00, 0x24, 0x05, 0x00, 0x00,
	0x52, 0x07, 0x00, 0x00, 0x11, 0x09, 0x00, 0x00, 0xde, 0x0a, 0x00, 0x00,
	0xd2, 0x0c, 0x00, 0x00, 0xe2, 0x0e, 0x00, 0x00, 0xfb, 0x10, 0x00, 0x00,
	0x64, 0x13, 0x00, 0x00, 0xd5, 0x15, 0x00, 0x00, 0x68, 0x18, 0x00, 0x00,
	0xa2, 0x19, 0x00, 0x00
};

/* Only these provider transfers are known to match the disp0 ADT. */
static const struct dcp_h17p_provider_property provider_properties[] = {
	{ "power-lut-data-x", "apple,power-lut-data-x", 4, j700_provider_data_0 },
	{ "power-lut-data-y", "apple,power-lut-data-y", 4, j700_provider_data_1 },
	{ "power-lut-data-xindex", "apple,power-lut-data-xindex", 52, j700_provider_data_2 },
	{ "power-lut-data-yindex", "apple,power-lut-data-yindex", 4, j700_provider_data_3 },
	{ "power-lut-data-lut", "apple,power-lut-data-lut", 52, j700_provider_data_4 },
	{ "power-lut-vbatt-cur-nominal", "apple,power-lut-vbatt-cur-nominal", 0, NULL },
};

static const struct dcp_h17p_provider_property *
dcp_provider_property_h17p(const struct dcp_h17p_provider_request *request)
{
	unsigned int i;

	if (memcmp(request->scope, "VORP", sizeof(request->scope)) ||
	    le32_to_cpu(request->capacity) !=
				 sizeof_field(struct dcp_h17p_provider_reply, data) ||
	    memchr_inv(request->reserved, 0, sizeof(request->reserved)) ||
	    strnlen(request->name, sizeof(request->name)) == sizeof(request->name))
		return NULL;

	/* Bytes following the first NUL were not initialized by the firmware. */
	for (i = 0; i < ARRAY_SIZE(provider_properties); i++)
		if (!strcmp(request->name, provider_properties[i].name))
			return &provider_properties[i];
	return NULL;
}

static bool trampoline_provider_property_h17p(struct apple_dcp *dcp, int tag,
					      void *out, void *in)
{
	const struct dcp_h17p_provider_property *property;
	struct dcp_h17p_provider_reply *reply = out;
	struct device_node *node = dcp->dev->of_node;
	int length, ret;

	/* The dispatcher already cleared the exact H17G output length. */
	if (dcp->hw.iomfb_method_profile == DCP_IOMFB_METHODS_H17G)
		return true;

	trace_iomfb_callback(dcp, tag, __func__);
	property = dcp_provider_property_h17p(in);
	if (!property) {
		dev_err(dcp->dev, "unsupported D400 provider property request\n");
		goto fail;
	}

	length = of_property_count_u8_elems(node, property->dt_name);
	if (!property->length) {
		/* Only this absent property has a known empty reply. */
		if (of_property_present(node, property->dt_name))
			goto fail;
	} else if (!of_property_present(node, property->dt_name) &&
		   of_machine_is_compatible("apple,j700") &&
		   (of_device_is_compatible(node, "apple,t8140-dcp") ||
		    (dcp->hw.iomfb_method_profile == DCP_IOMFB_METHODS_H17P_EXTERNAL &&
		     of_device_is_compatible(node, "apple,t8140-dcpext")))) {
		/* Older loaders lack the newly described table properties. */
		memset(reply, 0, sizeof(*reply));
		memcpy(reply->data, property->legacy_data, property->length);
		reply->length = cpu_to_le32(property->length);
		dev_info_once(dcp->dev,
			      "missing provider property %s (%u bytes); using the J700 defaults\n",
			      property->dt_name, property->length);
		return true;
	} else if (length != (int)property->length) {
		dev_err(dcp->dev, "invalid provider property %s: %d bytes\n",
			property->dt_name, length);
		goto fail;
	}

	memset(reply, 0, sizeof(*reply));
	if (property->length) {
		ret = of_property_read_u8_array(node, property->dt_name,
						reply->data, property->length);
		if (ret)
			goto fail;
	}
	reply->length = cpu_to_le32(property->length);
	return true;

fail:
	WRITE_ONCE(dcp->crashed, true);
	return false;
}

static bool trampoline_luma_span_h17p(struct apple_dcp *dcp, int tag,
				      void *out, void *in)
{
	/* Preserve the older profile's already-cleared unknown-callback reply. */
	if (dcp->hw.iomfb_method_profile == DCP_IOMFB_METHODS_H17G)
		return true;

	trace_iomfb_callback(dcp, tag, __func__);
	if (get_unaligned_le32(in)) {
		dev_err(dcp->dev, "unsupported D574 range selector\n");
		WRITE_ONCE(dcp->crashed, true);
		return false;
	}

	/* Selector zero receives binary32 219.0 followed by a zero word. */
	put_unaligned_le64(0x435b0000, out);
	return true;
}

static bool trampoline_get_uint_prop_h17p(struct apple_dcp *dcp, int tag,
					  void *out, void *in)
{
	const struct dcp_get_uint_prop_req *request = in;

	if (dcp->hw.iomfb_method_profile == DCP_IOMFB_METHODS_H17G)
		return trampoline_get_uint_prop(dcp, tag, out, in);

	trace_iomfb_callback(dcp, tag, __func__);
	/* The observed startup query uses Asahi's existing Temperature reply. */
	if (memcmp(request->obj, "SUMP", sizeof(request->obj)) ||
	    strnlen(request->key, sizeof(request->key)) == sizeof(request->key) ||
	    strcmp(request->key, "Temperature") || request->value_null ||
	    memchr_inv(request->padding, 0, sizeof(request->padding))) {
		dev_err(dcp->dev, "unsupported D401 integer property query\n");
		WRITE_ONCE(dcp->crashed, true);
		return false;
	}

	put_unaligned_le64(3029, out);
	*((u8 *)out + offsetof(struct dcp_get_uint_prop_resp, ret)) = true;
	return true;
}

static bool trampoline_allocate_buffer_h17p(struct apple_dcp *dcp, int tag,
					    void *out, void *in)
{
	const struct dcp_allocate_buffer_req *wire = in;
	struct dcp_allocate_buffer_req request = { 0 };
	struct dcp_allocate_buffer_resp reply;

	if (dcp->hw.iomfb_method_profile == DCP_IOMFB_METHODS_H17G)
		return trampoline_allocate_buffer(dcp, tag, out, in);

	static_assert(sizeof(*wire) == 0x14);
	static_assert(sizeof(reply) == 0x1c);
	static_assert(offsetof(struct dcp_allocate_buffer_req, size) == 4);
	static_assert(offsetof(struct dcp_allocate_buffer_req, unk2) == 12);
	static_assert(offsetof(struct dcp_allocate_buffer_req, paddr_null) == 16);

	trace_iomfb_callback(dcp, tag, __func__);
	request.size = get_unaligned_le64(&wire->size);
	reply = dcpep_cb_allocate_buffer(dcp, &request);
	if (!reply.dva_size) {
		/* Exhaustion and allocation errors use the all-zero reply. */
		memset(out, 0, sizeof(reply));
		return true;
	}

	put_unaligned_le64(reply.paddr, out);
	put_unaligned_le64(reply.dva, (u8 *)out + 8);
	put_unaligned_le64(reply.dva_size, (u8 *)out + 16);
	put_unaligned_le32(reply.mem_desc_id, (u8 *)out + 24);
	return true;
}

static bool trampoline_map_piodma_h17p(struct apple_dcp *dcp, int tag,
				       void *out, void *in)
{
	const struct dcp_map_buf_req *wire = in;
	struct dcp_map_buf_req request = { 0 };
	struct dcp_map_buf_resp_h17p reply = { 0 };
	u64 buffer;

	if (dcp->hw.iomfb_method_profile == DCP_IOMFB_METHODS_H17G)
		return trampoline_map_piodma(dcp, tag, out, in);

	static_assert(sizeof(*wire) == 0xc);
	static_assert(sizeof(reply) == 0x10);
	static_assert(offsetof(struct dcp_map_buf_req, unk) == 8);
	static_assert(offsetof(struct dcp_map_buf_req, dva_null) == 11);

	trace_iomfb_callback(dcp, tag, __func__);
	buffer = get_unaligned_le64(&wire->buffer);
	request.buffer = buffer;
	if (!dcpep_map_piodma(dcp, &request, &reply)) {
		/* A rejected mapping echoes the descriptor with a zero DVA. */
		put_unaligned_le32(buffer, out);
		put_unaligned_le64(0, (u8 *)out + 4);
		put_unaligned_le32(0, (u8 *)out + 12);
		return true;
	}

	put_unaligned_le32(reply.buffer, out);
	put_unaligned_le64(reply.dva, (u8 *)out + 4);
	put_unaligned_le32(reply.unk, (u8 *)out + 12);
	return true;
}

static bool trampoline_unmap_piodma_h17p(struct apple_dcp *dcp, int tag,
					 void *out, void *in)
{
	const struct dcp_unmap_buf_resp *wire = in;
	struct dcp_unmap_buf_resp request = { 0 };

	if (dcp->hw.iomfb_method_profile == DCP_IOMFB_METHODS_H17G)
		return trampoline_unmap_piodma(dcp, tag, out, in);

	static_assert(sizeof(*wire) == 0x1a);
	static_assert(offsetof(struct dcp_unmap_buf_resp, vaddr) == 8);
	static_assert(offsetof(struct dcp_unmap_buf_resp, dva) == 16);
	static_assert(offsetof(struct dcp_unmap_buf_resp, buf_null) == 25);

	trace_iomfb_callback(dcp, tag, __func__);
	request.buffer = get_unaligned_le64(&wire->buffer);
	request.vaddr = get_unaligned_le64(&wire->vaddr);
	request.dva = get_unaligned_le64(&wire->dva);
	request.unk = wire->unk;
	request.buf_null = wire->buf_null;
	dcpep_unmap_piodma(dcp, &request);
	return true;
}

static bool
trampoline_release_mem_desc_h17p(struct apple_dcp *dcp, int tag, void *out,
				 void *in)
{
	u32 id;

	if (dcp->hw.iomfb_method_profile == DCP_IOMFB_METHODS_H17G)
		return trampoline_release_mem_desc(dcp, tag, out, in);

	trace_iomfb_callback(dcp, tag, __func__);
	id = get_unaligned_le32(in);
	*(u8 *)out = dcpep_cb_release_mem_desc(dcp, &id);
	return true;
}

/* H17P callback numbering is not a uniform shift of the v13.5 table. */
static const iomfb_cb_handler cb_handlers[IOMFB_MAX_CB] = {
	[0] = dcpep_cb_d000_h17p, /* acked after a nested A033 */
	[1] = trampoline_true,
	[2] = trampoline_nop,
	[3] = trampoline_rt_bandwidth_h17p,
	[6] = trampoline_set_frame_sync_props_h17p, /* 0x54/0x50, identity scale */
	[100] = iomfbep_cb_match_pmu_service, /* match_pmu_service */
	[101] = trampoline_zero,
	[102] = trampoline_nop, /* set_number_property */
	[104] = trampoline_nop, /* set_boolean_property */
	[107] = trampoline_nop,
	[108] = trampoline_true, /* create_provider_service */
	[109] = trampoline_true, /* create_product_service */
	[110] = trampoline_true, /* create_PMU_service */
	[111] = trampoline_true, /* create_iomfb_service */
	[112] = trampoline_create_backlight_service, /* create_backlight_service */
	[113] = trampoline_true, /* create_nvram_service */
	[114] = trampoline_analytics_h17p, /* CoreAnalyticsSendEvent */
	[117] = trampoline_nop, /* set_idle_caching_state_ap */
	[118] = trampoline_zero, /* upload_trace_start */
	[119] = trampoline_zero, /* upload_trace_chunk */
	[120] = trampoline_zero, /* upload_trace_end */
	[121] = dcpep_cb_boot_1, /* start_hardware_boot */
	[122] = trampoline_false, /* is_dark_boot */
	[123] = trampoline_false, /* is_waking_from_hibernate */
	[124] = trampoline_zero, /* detect_fastsim */
	[125] = trampoline_read_edt_data, /* read_edt_data */
	[127] = trampoline_prop_start, /* setDCPAVPropStart */
	[128] = trampoline_prop_chunk, /* setDCPAVPropChunk */
	[129] = trampoline_prop_end, /* setDCPAVPropEnd */
	[130] = trampoline_allocate_bandwidth, /* allocate_bandwidth */
	[201] = trampoline_map_piodma_h17p, /* map_buf */
	[202] = trampoline_unmap_piodma_h17p, /* unmap_buf */
	[206] = iomfbep_cb_match_pmu_service_2, /* match_pmu_service */
	[207] = iomfbep_cb_match_backlight_service, /* match_backlight_service */
	[208] = trampoline_nop, /* update_backlight_factor_prop */
	[209] = trampoline_get_time, /* get_calendar_time_ms */
	[300] = trampoline_pr_publish,
	[400] = trampoline_provider_property_h17p,
	[401] = trampoline_get_uint_prop_h17p, /* get_uint_prop */
	[404] = trampoline_nop, /* set_uint_prop */
	[406] = trampoline_set_fx_prop, /* set_fx_prop */
	[408] = trampoline_get_frequency_h17p, /* getClockFrequency */
	[411] = trampoline_map_reg, /* mapDeviceMemoryWithIndex */
	[413] = trampoline_true, /* setProperty */
	[414] = trampoline_sr_set_property_int, /* setProperty */
	[415] = trampoline_true, /* setProperty */
	[451] = trampoline_allocate_buffer_h17p, /* allocate_buffer */
	[452] = trampoline_map_physical, /* prepare */
	[454] = trampoline_release_mem_desc_h17p, /* release_descriptor */
	[552] = trampoline_true,
	[561] = trampoline_true,
	[563] = trampoline_true,
	[565] = trampoline_true,
	[567] = trampoline_true,
	[572] = trampoline_zero, /* powerUpDART */
	[574] = trampoline_luma_span_h17p,
	[575] = trampoline_hotplug_h17p,
	[576] = trampoline_nop,
	[582] = iomfbep_cb_create_dfb_surface, /* create_default_fb_surface */
	[583] = trampoline_nop, /* clear_default_surface */
	[584] = trampoline_nop, /* swap_notify_gated */
	[585] = trampoline_swap_info_h17p, /* swap_info_notify_dispatch */
	[590] = trampoline_swap_complete, /* swap_complete_ap_gated */
	[592] = trampoline_swap_complete_intent_gated, /* swap_complete_intent_gated */
	[593] = trampoline_abort_swap_ap_gated, /* abort_swap_ap_gated */
	[594] = trampoline_enable_backlight_message_ap_gated,
	[595] = trampoline_nop, /* setSystemConsoleMode */
	[597] = trampoline_false, /* isDFBAllocated */
	[598] = trampoline_false,
	[599] = trampoline_nop, /* find_swap_function_gated */
};

/* Included with the public H17P template helpers in scope. */

static bool ext26_callback_size(int tag, u32 input, u32 output)
{
	switch (tag) {
	case 3: return input == 4 && output == 60;
	case 6: return input == 84 && output == 80;
	case 107: return input == 64 && !output;
	case 115: return input == 16 && output == 8;
	case 116: return input == 16 && output == 4;
	case 134: return !input && output == 4;
	case 408: return input == 8 && output == 8;
	case 454: return input == 4 && output == 4;
	case 567: return input == 128 && output == 4;
	case 572: return input == 4 && output == 4;
	case 576: return input == 4 && !output;
	case 588: return !input && !output;
	default: return iomfb_validate_callback_h17p(tag, input, output);
	}
}

static u8 ext26_prop_start(struct apple_dcp *dcp, u32 advertised)
{
	struct dcp_chunks *chunks = &dcp->chunks;

	if (!advertised || advertised > SZ_1M + 1)
		return false;
	kfree(chunks->data);
	memset(chunks, 0, sizeof(*chunks));
	chunks->length = advertised - 1;
	chunks->contiguous = true;
	chunks->modes_generation = dcp_modes_transfer_begin(dcp);
	dcp_firmware_metadata_begin(dcp);
	chunks->data = kzalloc(max_t(size_t, 1, chunks->length), GFP_KERNEL);
	return !!chunks->data;
}

struct ext26_boot {
	unsigned int completed;
	u32 one;
};

static void ext26_boot_next(struct apple_dcp *dcp, void *out, void *cookie)
{
	struct ext26_boot *boot = cookie;
	const struct dcp_method_entry *method;
	u32 in = 0, reply = 0;

	if (READ_ONCE(dcp->crashed) || (boot->completed && !out) ||
	    ((boot->completed == 5 || boot->completed == 6) &&
	     get_unaligned_le32(out) != 1)) {
		WRITE_ONCE(dcp->crashed, true);
		dev_err(dcp->dev,
			"native26 D121 startup refused: completed=%u last=%s reply=%#x present=%u; session retained\n",
			boot->completed, boot->completed == 5 ? "A000" :
			boot->completed == 6 ? "A465" : "earlier call",
			out && (boot->completed == 2 || boot->completed == 5 ||
				boot->completed == 6) ? get_unaligned_le32(out) : 0, !!out);
		kfree(boot);
		return;
	}
	if (boot->completed == 5 || boot->completed == 6)
		dev_dbg(dcp->dev, "native26 D121 %s reply=%#x accepted\n",
			 boot->completed == 5 ? "A000" : "A465", get_unaligned_le32(out));
	switch (boot->completed++) {
	case 0:
		method = &dcp_methods[dcpep_set_create_dfb];
		break;
	case 1:
		method = &dcp_methods[dcpep_create_default_fb];
		reply = 4;
		break;
	case 2:
		method = &dcp_methods[dcpep_setup_video_limits];
		break;
	case 3:
		method = &dcp_methods[dcpep_flush_supports_power];
		in = 4;
		break;
	case 4:
		method = &dcp_methods[dcpep_late_init_signal];
		in = reply = 4;
		break;
	case 5:
		method = &dcp_methods[dcpep_set_display_refresh_properties];
		reply = 4;
		break;
	default:
		kfree(boot);
		boot_done(dcp, NULL, NULL);
		dev_dbg(dcp->dev, "native26 D121 startup complete\n");
		return;
	}
	dcp_push(dcp, false, method, in, reply, &boot->one, ext26_boot_next, boot);
}

static bool ext26_boot_callback(struct apple_dcp *dcp, int tag, void *out, void *in)
{
	struct ext26_boot *boot = kzalloc_obj(*boot);

	if (!boot) {
		WRITE_ONCE(dcp->crashed, true);
		return false;
	}
	boot->one = 1;
	ext26_boot_next(dcp, NULL, boot);
	return false;
}

static bool ext26_property(struct apple_dcp *dcp, int tag, void *out, void *input)
{
	const u8 *in = input;

	if (tag == 107 && (!strncmp((const char *)in, "TimingElements", 64) ||
			   !strncmp((const char *)in, "DisplayAttributes", 64))) {
		dcp_modes_begin_attachment(dcp);
		if (dcp->connector) {
			dcp->connector->connected = false;
			apple_connector_invalidate_edid(dcp->connector);
			dcp_queue_hotplug(dcp->connector);
		}
	}
	if (memcmp(in, "NormalModeActive", sizeof("NormalModeActive")))
		return true;
	if (tag == 107) {
		WRITE_ONCE(dcp->ext26_normal, false);
		return true;
	}
	bool on = tag == 104 ? !!in[64] : !!get_unaligned_le32(in + 64);

	if (!on) {
		WRITE_ONCE(dcp->ext26_normal, false);
		return true;
	}
	if (READ_ONCE(dcp->ext26_power_sent) &&
	    dcp->ext26_rx_ns > READ_ONCE(dcp->ext26_power_ns) &&
	    dcp->ext26_generation == READ_ONCE(dcp->typec_generation)) {
		WRITE_ONCE(dcp->ext26_normal, true);
		complete_all(&dcp->ext26_ready);
		dev_dbg(dcp->dev, "native26 NormalModeActive after power publication\n");
	}
	return true;
}

static bool ext26_bandwidth(struct apple_dcp *dcp, int tag, void *out, void *in)
{
	memset(out, 0, 60);
	put_unaligned_le32(1, (u8 *)out + 56);
	return true;
}

static bool ext26_frame_sync(struct apple_dcp *dcp, int tag, void *out, void *input)
{
	const u8 *in = input;

	memset(out, 0, 80);
	if (!(in[80] & 1))
		memcpy(out, in, 80);
	return true;
}

static bool ext26_defaults(struct apple_dcp *dcp, int tag, void *out, void *input)
{
	const u8 *in = input;
	u32 count = get_unaligned_le32(in + 64);

	memset(out, 0, 36);
	if (count <= 8)
		memcpy(out, in + 68, count * 4);
	return true;
}

static bool ext26_clock(struct apple_dcp *dcp, int tag, void *out, void *input)
{
	const u8 *in = input;
	u32 index = get_unaligned_le32(in + 4);
	u64 rate = 0;

	if (!memcmp(in, "VORP", 4) && index < 2)
		rate = index ? 24000000 : 48000000;
	put_unaligned_le64(rate, out);
	return true;
}

static bool ext26_analytics(struct apple_dcp *dcp, int tag, void *out, void *input)
{
	const u8 *in = input;

	memset(out, 0, 4100);
	if (memchr(in, 0, 64) && !in[4160])
		memcpy(out, in + 64, 4096);
	return true;
}

static bool ext26_tiling(struct apple_dcp *dcp, int tag, void *out, void *input)
{
	memset(out, 0, 8);
	if (((u8 *)input)[12] <= 1)
		((u8 *)out)[4] = 1;
	return true;
}

static bool ext26_release(struct apple_dcp *dcp, int tag, void *out, void *in)
{
	u32 id = get_unaligned_le32(in);

	memset(out, 0, 4);
	/* Logical descriptor release never proves DMA retirement. */
	if (id && id < DCP_MAX_MAPPINGS && test_bit(id, dcp->memdesc_map) &&
	    !test_and_set_bit(id, dcp->ext26_released))
		*(u8 *)out = 1;
	return true;
}

static bool ext26_allocate(struct apple_dcp *dcp, int tag, void *out, void *input)
{
	const u8 *in = input;
	u64 bytes = get_unaligned_le64(in + 4);
	u32 alignment = get_unaligned_le32(in + 12);

	memset(out, 0, 28);
	if (get_unaligned_le32(in) != 0x703 || !is_power_of_2(alignment) ||
	    alignment > SZ_16K || in[16] || in[17] || in[18] || !bytes ||
	    bytes > SZ_64M || ALIGN(bytes, SZ_16K) > SZ_256M - dcp->ext26_buffer_bytes)
		return true;
	trampoline_allocate_buffer_h17p(dcp, tag, out, input);
	if (get_unaligned_le32((u8 *)out + 24))
		dcp->ext26_buffer_bytes += ALIGN(bytes, SZ_16K);
	return true;
}

static bool ext26_hotplug(struct apple_dcp *dcp, int tag, void *out, void *input)
{
	const u8 *in = input;
	u64 connected = get_unaligned_le64(in);

	memset(out, 0, 76);
	if (!(in[84] & 1))
		memcpy(out, in + 8, 76);

	dcpep_cb_hotplug(dcp, &connected);
	return true;
}

static void ext26_started(struct apple_dcp *dcp, void *out, void *cookie)
{
	if (!out || READ_ONCE(dcp->crashed) || get_unaligned_le32(out) != 1) {
		WRITE_ONCE(dcp->crashed, true);
		complete_all(&dcp->start_done);
		return;
	}
	dcp_ext26_open_services(dcp);
	dcp->main_display = false;
	dcp->active = true;
	dev_dbg(dcp->dev, "native26 A401 complete; service OPEN and route admission enabled\n");
	complete_all(&dcp->start_done);
}

static void ext26_client_reply(struct apple_dcp *dcp, void *out, void *cookie)
{
	struct dcp_wait_cookie *wait = cookie;

	WRITE_ONCE(wait->replied, !!out);
	complete(&wait->done);
	kref_put(&wait->refcount, release_wait_cookie);
}

static int ext26_open_client(struct apple_dcp *dcp)
{
	struct dcp_wait_cookie *wait;
	int ret;

	if (dcp->ext26_client_open)
		return 0;
	wait = kzalloc_obj(*wait);
	if (!wait)
		return -ENOMEM;
	init_completion(&wait->done);
	kref_init(&wait->refcount);
	kref_get(&wait->refcount);
	dcp_first_client_open(dcp, false, ext26_client_reply, wait);
	ret = wait_for_completion_timeout(&wait->done, msecs_to_jiffies(5000)) &&
	      READ_ONCE(wait->replied) && !READ_ONCE(dcp->crashed) ? 0 : -ETIMEDOUT;
	kref_put(&wait->refcount, release_wait_cookie);
	if (!ret) {
		dcp->ext26_client_open = true;
		dev_dbg(dcp->dev, "native26 A457 complete before first mode\n");
	} else {
		WRITE_ONCE(dcp->ext26_blocked, true);
		WRITE_ONCE(dcp->crashed, true);
	}
	return ret;
}

static void ext26_main_done(struct apple_dcp *dcp, void *out, void *cookie)
{
	if (!out) {
		WRITE_ONCE(dcp->crashed, true);
		return;
	}
	dcp_ack(dcp, DCP_CONTEXT_CB);
}

static bool ext26_main_query(struct apple_dcp *dcp, int tag, void *out, void *in)
{
	dcp_is_main_display(dcp, false, ext26_main_done, NULL);
	return false;
}

static bool ext26_stride(struct apple_dcp *dcp, int tag, void *out, void *in)
{
	put_unaligned_le32(dcp->ext26_stride, out);
	return true;
}

static bool ext26_match_backlight(struct apple_dcp *dcp, int tag, void *out, void *in)
{
	iomfb_a132_backlight_service_matched(dcp, false, complete_backlight_service_matched, out);
	return false;
}

static bool ext26_map_piodma(struct apple_dcp *dcp, int tag, void *out, void *input)
{
	const u8 *in = input;
	u64 id = get_unaligned_le64(in);

	memset(out, 0, 16);
	if (!id || id >= DCP_MAX_MAPPINGS || get_unaligned_le32(in + 8) != 1 ||
	    !test_bit(id, dcp->memdesc_map) || test_bit(id, dcp->ext26_released))
		return true;
	if (dcp->memdesc[id].piodma_mapped) {
		put_unaligned_le32(id, out);
		put_unaligned_le64(dcp->memdesc[id].dva, (u8 *)out + 4);
		return true;
	}
	return trampoline_map_piodma_h17p(dcp, tag, out, input);
}

static void ext26_start(struct apple_dcp *dcp)
{
	iomfb_cb_handler *handlers = kcalloc(IOMFB_MAX_CB, sizeof(*handlers), GFP_KERNEL);

	if (!handlers) {
		WRITE_ONCE(dcp->crashed, true);
		complete(&dcp->start_done);
		return;
	}
	memcpy(handlers, cb_handlers, sizeof(cb_handlers));
	handlers[0] = handlers[112] = trampoline_true;
	handlers[3] = ext26_bandwidth;
	handlers[6] = ext26_frame_sync;
	handlers[102] = handlers[104] = handlers[107] = ext26_property;
	handlers[114] = ext26_analytics;
	handlers[115] = ext26_tiling;
	handlers[116] = handlers[134] = trampoline_zero;
	handlers[588] = trampoline_nop;
	handlers[121] = ext26_boot_callback;
	handlers[101] = ext26_stride;
	handlers[125] = ext26_defaults;
	handlers[201] = ext26_map_piodma;
	handlers[207] = ext26_match_backlight;
	handlers[130] = handlers[411] = handlers[452] = trampoline_zero;
	handlers[202] = handlers[592] = handlers[594] = trampoline_nop;
	handlers[400] = handlers[401] = handlers[574] = trampoline_zero;
	handlers[408] = ext26_clock;
	handlers[451] = ext26_allocate;
	handlers[454] = ext26_release;
	handlers[575] = ext26_hotplug;
	handlers[582] = trampoline_zero;
	handlers[593] = ext26_abort;
	handlers[599] = ext26_main_query;
	dcp->ext26_handlers = handlers;
	dcp->cb_handlers = handlers;
	dcp_start_signal(dcp, false, ext26_started, NULL);
}

static int ext26_encode(struct apple_dcp *dcp, struct dcp_swap_submit_req_h17p *request)
{
	struct dcpext26_primary primary, *selected = NULL;
	const struct dcp_swap_submit_req_h17p *effective = request;
	const struct dcp_surface *surface;
	struct iomfb_scanout_h17p *scanout = dcp->iomfb.next_scanout;
	struct drm_framebuffer *fb = scanout ? scanout->fb[0] : NULL;

	if (READ_ONCE(dcp->ext26_blocked) || !READ_ONCE(dcp->ext26_normal))
		return -EPIPE;
	if (!(request->swap.swap_enabled & BIT(0)) && dcp->iomfb.scanout)
		effective = &dcp->iomfb.scanout->request;
	surface = &effective->surf[0].base;
	if (!effective->surf_null[0]) {
		if (!fb || !fb->obj[0])
			return -EINVAL;
		primary = (struct dcpext26_primary) {
			.address = effective->surf_iova[0], .backing_bytes = fb->obj[0]->size,
			.width = surface->width, .height = surface->height, .pitch = surface->stride,
		};
		selected = &primary;
	}
	return dcpext26_primary_encode(dcp->ext26_present, sizeof(dcp->ext26_present),
				       request->swap.swap_id, 0, selected);
}

static void ext26_power_reply(struct apple_dcp *dcp, void *out, void *cookie)
{
	struct dcp_wait_cookie *wait = cookie;

	wait->status = out && get_unaligned_le64(out) == 1 ? 0 : ~0U;
	complete(&wait->done);
	kref_put(&wait->refcount, release_wait_cookie);
}

static bool ext26_abort(struct apple_dcp *dcp, int tag, void *out, void *input)
{
	u32 id = get_unaligned_le32(input);

	if (iomfb_ext26_control_complete(dcp, id, true))
		return true;
	bool ack = trampoline_abort_swap_ap_gated(dcp, tag, out, input);

	WRITE_ONCE(dcp->ext26_blocked, true);
	WRITE_ONCE(dcp->crashed, true);
	return ack;
}

static int ext26_activate(struct apple_dcp *dcp)
{
	struct dcp_set_power_state_req_h17p request = { .unklong = 1 };
	struct dcp_wait_cookie *wait;
	int ret;

	wait = kzalloc_obj(*wait);
	if (!wait)
		return -ENOMEM;
	wait->status = ~0U;
	init_completion(&wait->done);
	kref_init(&wait->refcount);
	kref_get(&wait->refcount);
	WRITE_ONCE(dcp->external_power_attempted, true);
	dcp_set_power_state(dcp, false, &request, ext26_power_reply, wait);
	ret = wait_for_completion_timeout(&wait->done, msecs_to_jiffies(5000));
	ret = ret && !READ_ONCE(wait->status) ? 0 : -EIO;
	kref_put(&wait->refcount, release_wait_cookie);
	if (!ret && !wait_for_completion_timeout(&dcp->ext26_ready, msecs_to_jiffies(5000)))
		ret = -ETIMEDOUT;
	if (!ret && (!READ_ONCE(dcp->ext26_normal) || READ_ONCE(dcp->crashed) ||
		     READ_ONCE(dcp->ext26_blocked)))
		ret = -EIO;
	if (!ret) {
		WRITE_ONCE(dcp->pipe_enabled_h17p, true);
		dev_dbg(dcp->dev, "native26 mode and power ready; primary scanout admitted\n");
	}
	return ret;
}

bool iomfb_check_callback_h17p(struct apple_dcp *dcp, int tag, u32 in_len,
			       u32 out_len)
{
	return dcp_is_ext26(dcp) ? ext26_callback_size(tag, in_len, out_len) :
		iomfb_validate_callback_h17p(tag, in_len, out_len);
}

static_assert(sizeof(struct dcp_set_power_state_resp) == 8);

struct dcp_external_power_cookie {
	struct dcp_wait_cookie wait;
	bool on;
};

/* A473 echoes the accepted state with zero status; DMA ownership is unchanged. */
static void iomfb_external_power_complete(struct apple_dcp *dcp, void *out, void *data)
{
	struct dcp_external_power_cookie *cookie = data;
	const struct dcp_packet_header *header;
	int ret = -EPROTO;

	if (out) {
		header = out - sizeof(struct dcp_set_power_state_req_h17p) - sizeof(*header);
		if (header->in_len == sizeof(struct dcp_set_power_state_req_h17p) &&
		    header->out_len == sizeof(struct dcp_set_power_state_resp)) {
			u64 reply = get_unaligned_le64(out);

			ret = reply == (u64)cookie->on ? 0 : -EREMOTEIO;
		}
	}
	WRITE_ONCE(cookie->wait.status, ret);
	complete(&cookie->wait.done);
	kref_put(&cookie->wait.refcount, release_wait_cookie);
}

static bool iomfb_external_reply_size(void *out, u32 in_len, u32 out_len)
{
	const struct dcp_packet_header *header;

	if (!out)
		return false;
	header = out - in_len - sizeof(*header);
	return header->in_len == in_len && header->out_len == out_len;
}

static void iomfb_external_parameter_complete(struct apple_dcp *dcp, void *out, void *data)
{
	struct dcp_set_power_state_req_h17p request = { .unklong = 1 };

	if (!iomfb_external_reply_size(out, sizeof(struct dcp_set_parameter_dcp), sizeof(u32))) {
		iomfb_external_power_complete(dcp, NULL, data);
		return;
	}
	dcp_set_power_state(dcp, true, &request, iomfb_external_power_complete, data);
}

static void iomfb_external_device_complete(struct apple_dcp *dcp, void *out, void *data)
{
	struct dcp_set_parameter_dcp parameter = {
		.param = IOMFBPARAM_ADAPTIVE_SYNC,
		.count = 3,
	};

	if (!iomfb_external_reply_size(out, sizeof(u32), sizeof(u32))) {
		iomfb_external_power_complete(dcp, NULL, data);
		return;
	}
	/* Preserve the existing external power-on preparation and payload semantics. */
	dcp_set_parameter_dcp(dcp, false, &parameter, iomfb_external_parameter_complete, data);
}

int iomfb_external_power_h17p(struct apple_dcp *dcp, bool on)
{
	struct dcp_set_power_state_req_h17p request = { .unklong = on };
	struct dcp_external_power_cookie *cookie;
	int ret;

	if (READ_ONCE(dcp->crashed) || READ_ONCE(dcp->iomfb.stopped))
		return -EIO;
	cookie = kzalloc_obj(*cookie);
	if (!cookie)
		return -ENOMEM;
	cookie->on = on;
	cookie->wait.status = -ETIMEDOUT;
	init_completion(&cookie->wait.done);
	kref_init(&cookie->wait.refcount);
	kref_get(&cookie->wait.refcount);
	if (dcp_is_ext26(dcp)) {
		dcp_set_power_state(dcp, false, &request, iomfb_external_power_complete, cookie);
	} else if (on) {
		u32 handle = 2;

		iomfb_opaque_x_reset_h17p(dcp);
		dcp_set_display_device(dcp, false, &handle, iomfb_external_device_complete, cookie);
	} else {
		dcp_set_power_state(dcp, true, &request, iomfb_external_power_complete, cookie);
	}
	ret = wait_for_completion_timeout(&cookie->wait.done, msecs_to_jiffies(dcp_is_ext26(dcp) ? 5000 : 500));
	ret = ret ? (int)READ_ONCE(cookie->wait.status) : -ETIMEDOUT;
	kref_put(&cookie->wait.refcount, release_wait_cookie);
	return ret;
}

void iomfb_start_h17p(struct apple_dcp *dcp)
{
	if (dcp_is_ext26(dcp)) {
		ext26_start(dcp);
		return;
	}
	WRITE_ONCE(dcp->pipe_enabled_h17p, false);
	dcp->cb_handlers = cb_handlers;
	dcp_start_signal(dcp, false, dcp_started, NULL);
}

#if IS_ENABLED(CONFIG_DRM_APPLE_KUNIT_TEST)
/* Returns the first handled tag without reply bounds, or -1. */
int iomfb_h17p_first_unbounded_callback(void)
{
	int tag;

	for (tag = 0; tag < IOMFB_MAX_CB; tag++)
		if (cb_handlers[tag] && !callback_sizes[tag].valid)
			return tag;
	return -1;
}

/*
 * Hands a completion of @swap_id to the swap_complete_ap_gated callback, as
 * the firmware would.  The caller holds the IOMFB lock, as the receiver does.
 */
int iomfb_h17p_deliver_swap_complete(struct apple_dcp *dcp, u32 swap_id)
{
	struct dc_swap_complete_resp_h17p *resp;

	lockdep_assert_held(&dcp->iomfb.lock);
	resp = kzalloc_obj(*resp);
	if (!resp)
		return -ENOMEM;
	resp->swap_id = swap_id;
	cb_handlers[590](dcp, 590, NULL, resp);
	kfree(resp);
	return 0;
}
#endif

#undef DCP_FW_VER
#undef DCP_FW

struct ext26_control {
	struct dcpext26_exchange exchange;
	struct completion done;
	u8 wire[DCPEXT26_SUBMIT_BYTES];
};

static void ext26_control_signal(struct apple_dcp *dcp, struct ext26_control *control)
{
	if (dcpext26_exchange_finished(&control->exchange)) {
		if (control->exchange.failed) {
			WRITE_ONCE(dcp->crashed, true);
			WRITE_ONCE(dcp->ext26_blocked, true);
		}
		dcp->ext26_control_pending = false;
		complete_all(&control->done);
	}
}

bool iomfb_ext26_control_complete(struct apple_dcp *dcp, u32 id, bool aborted)
{
	struct ext26_control *control = dcp->ext26_control;

	if (!dcp_is_ext26(dcp))
		return false;
	if (id && id == dcp->ext26_control_id)
		return true;
	if (!control || !dcpext26_exchange_complete(&control->exchange, id))
		return false;
	if (aborted)
		control->exchange.failed = true;
	ext26_control_signal(dcp, control);
	return true;
}

static void ext26_null_submitted(struct apple_dcp *dcp, void *reply, void *cookie)
{
	struct ext26_control *control = cookie;

	control->exchange.replied = true;
	if (dcpext26_submit_reply(reply, 12))
		control->exchange.failed = true;
	ext26_control_signal(dcp, control);
}

static void ext26_null_started(struct apple_dcp *dcp, void *reply, void *cookie)
{
	struct ext26_control *control = cookie;
	u32 id;

	if (dcpext26_start_reply(reply, 8, &id) ||
	    !dcpext26_exchange_begin(&control->exchange, id) ||
	    dcpext26_primary_encode(control->wire, sizeof(control->wire), id, 0, NULL)) {
		control->exchange.failed = true;
		ext26_control_signal(dcp, control);
		return;
	}
	dcp_push(dcp, false, &dcp_methods[dcpep_swap_submit], sizeof(control->wire), 12,
		 control->wire, ext26_null_submitted, control);
}

/* Called by the lifecycle worker, never from RX or the shared DRM tail. */
int iomfb_ext26_blank(struct apple_dcp *dcp)
{
	struct ext26_control *control;
	u8 start[16] = {};
	int ret;

	ret = iomfb_queue_settle(dcp, msecs_to_jiffies(5000));
	if (ret)
		return ret;
	control = kzalloc_obj(*control);
	if (!control)
		return -ENOMEM;
	init_completion(&control->done);
	mutex_lock(&dcp->iomfb.lock);
	if (dcp->ext26_control || dcp->present_state_h17p.pending || dcp->crashed) {
		mutex_unlock(&dcp->iomfb.lock);
		kfree(control);
		return -EBUSY;
	}
	dcp->ext26_control = control;
	dcp->ext26_control_pending = true;
	mutex_unlock(&dcp->iomfb.lock);
	dcp_push(dcp, false, &dcp_methods[dcpep_swap_start], sizeof(start), 8,
		 start, ext26_null_started, control);
	ret = wait_for_completion_timeout(&control->done, msecs_to_jiffies(5000));
	/* RX no longer owns a successful control before its storage is reclaimed. */
	if (ret)
		apple_rtkit_flush_rx(dcp->rtk);
	mutex_lock(&dcp->iomfb.lock);
	if (!ret || control->exchange.failed || dcp->crashed) {
		WRITE_ONCE(dcp->crashed, true);
		ret = -EIO;
		/* Possibly published control storage remains owned until reset. */
	} else {
		dcp->ext26_control_id = control->exchange.id;
		dcp->ext26_control = NULL;
		kfree(control);
		ret = 0;
	}
	mutex_unlock(&dcp->iomfb.lock);
	return ret;
}
