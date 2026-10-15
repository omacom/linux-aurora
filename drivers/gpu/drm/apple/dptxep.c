// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Copyright 2022 Sven Peter <sven@svenpeter.dev> */

#include <linux/unaligned.h>
#include <linux/bitfield.h>
#include <linux/completion.h>
#include <linux/module.h>
#include <linux/phy/phy.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/printk.h>

#include <linux/soc/apple/dp-tunnel.h>
#include <linux/string.h>

#include "afk.h"
#include "dcp.h"
#include "dcp-hdmi.h"
#include "dptxep.h"
#include "parser.h"
#include "trace.h"

struct dcpdptx_connection_cmd {
	__le32 unk;
	__le32 target;
} __attribute__((packed));

struct dcpdptx_hotplug_cmd {
	u8 _pad0[16];
	__le32 unk;
} __attribute__((packed));

struct dptxport_apcall_link_rate {
	__le32 retcode;
	u8 _unk0[12];
	__le32 link_rate;
	u8 _unk1[12];
} __attribute__((packed));

struct dptxport_apcall_lane_count {
	__le32 retcode;
	u8 _unk0[12];
	__le64 lane_count;
	u8 _unk1[8];
} __attribute__((packed));

struct dptxport_apcall_set_active_lane_count {
	__le32 retcode;
	u8 _unk0[12];
	__le64 lane_count;
	u8 _unk1[8];
} __packed;

struct dptxport_apcall_get_support {
	__le32 retcode;
	u8 _unk0[12];
	__le32 supported;
	u8 _unk1[12];
} __attribute__((packed));

struct dptxport_apcall_max_drive_settings {
	__le32 retcode;
	u8 _unk0[12];
	__le32 max_drive_settings[2];
	u8 _unk1[8];
};

struct dptxport_apcall_drive_settings {
	__le32 retcode;
	u8 _unk0[12];
	__le32 unk1;
	__le32 unk2;
	__le32 unk3;
	__le32 unk4;
	__le32 unk5;
	__le32 unk6;
	__le32 unk7;
};

struct dptxport_apcall_set_tiled {
	__le32 retcode;
};

/*
 * SetTiledDisplayHints payload, as DCP sends it for an LG UltraFine 5K
 * (offsets in bytes): tile location (column, row) at 0x08/0x0c, tiles
 * across and down at 0x30/0x34, the whole display's size at 0x38/0x3c.
 */
#define DPTX_TILE_HINT_LOC_X	0x08
#define DPTX_TILE_HINT_LOC_Y	0x0c
#define DPTX_TILE_HINT_TILES_H	0x30
#define DPTX_TILE_HINT_TILES_V	0x34
#define DPTX_TILE_HINT_WIDTH	0x38
#define DPTX_TILE_HINT_HEIGHT	0x3c
#define DPTX_TILE_HINT_MIN_SIZE	0x40

/*
 * Ported from aurora-silicon/linux#8: a Thunderbolt DP tunnel uses the same
 * plain CORE|ATC|DIE|CONNECTED target as a direct alt-mode PHY. CORE is the
 * DFP port (0 = dpphy, 1/2 = dpin0/dpin1, dcp->dptx_dfp_port), ATC is the
 * route's own ATC index -- no separate DPIN field.
 */
static u32 dptxport_remote_target(struct apple_dcp *dcp, u8 core, u8 atc,
				  u8 die)
{
	return FIELD_PREP(DCPDPTX_REMOTE_PORT_CORE, core) |
	       FIELD_PREP(DCPDPTX_REMOTE_PORT_ATC, atc) |
	       FIELD_PREP(DCPDPTX_REMOTE_PORT_DIE, die) |
	       DCPDPTX_REMOTE_PORT_CONNECTED;
}

int dptxport_validate_connection(struct apple_epic_service *service, u8 core,
				 u8 atc, u8 die)
{
	struct dptx_port *dptx = service->cookie;
	struct dcpdptx_connection_cmd cmd, resp;
	int ret;
	u32 target = dptxport_remote_target(service->ep->dcp, core, atc, die);

	/*
	 * attributes: role (0 = direct PHY, 1 = Thunderbolt/USB4 DP IN) |
	 * supportsHPD << 8. A real, hardware-validated reference
	 * implementation of the equivalent Thunderbolt DP tunnel mechanism
	 * on a different SoC (aurora-silicon/linux#8) adds this exact role
	 * bit for its tunnel routes; our own analog-DPIN path has never set
	 * it, always sending a plain direct-PHY attributes value even
	 * though this is a genuinely USB4-tunneled connection. DCP firmware
	 * appears to treat a failed AUX probe as fatal for role=0 but keeps
	 * training regardless for role=1 (a Thunderbolt tunnel's AUX
	 * proxying may not respond as fast as a direct PHY's): repeated
	 * test runs with substantially different Activate/crossbar ordering
	 * all reached the identical INACTIVE_SINK_DETECTED/DPRX-timeout
	 * outcome, which points at a signal we are not sending at all
	 * rather than a timing issue fixable by reordering.
	 */
	u32 attrs = 0x100 | (dcp_is_usb4_output(service->ep->dcp) ? 1 : 0);

	trace_dptxport_validate_connection(dptx, core, atc, die);
	dptx->validate_calls++;
	dev_dbg(service->ep->dcp->dev,
		 "DPTX validate: call #%u this boot target=0x%x core=%u atc=%u die=%u attrs=0x%x caller=%pS\n",
		 dptx->validate_calls, target, core, atc, die,
		 attrs, __builtin_return_address(0));

	cmd.target = cpu_to_le32(target);
	cmd.unk = cpu_to_le32(attrs);
	ret = afk_service_call(service, 0, 12, &cmd, sizeof(cmd), 40, &resp,
			       sizeof(resp), 40);
	if (ret)
		return ret;

	if (le32_to_cpu(resp.target) != target) {
		dev_warn(service->ep->dcp->dev,
			 "validate_connection: target reply 0x%x (sent 0x%x)\n",
			 le32_to_cpu(resp.target), target);
		return -EINVAL;
	}
	if (le32_to_cpu(resp.unk) != attrs) {
		/* only the Thunderbolt DP IN role is allowed to differ */
		if (!dcp_is_usb4_output(service->ep->dcp)) {
			dev_warn(service->ep->dcp->dev,
				 "validate_connection: attrs reply 0x%x (sent 0x%x), rejecting\n",
				 le32_to_cpu(resp.unk), attrs);
			return -EINVAL;
		}
		dev_info(service->ep->dcp->dev,
			 "validate_connection: attrs reply 0x%x (sent 0x%x)\n",
			 le32_to_cpu(resp.unk), attrs);
	}

	return 0;
}

int dptxport_connect(struct apple_epic_service *service, u8 core, u8 atc,
		     u8 die, bool supports_hpd)
{
	struct dptx_port *dptx = service->cookie;
	struct dcpdptx_connection_cmd cmd, resp;
	/* same role bit as dptxport_validate_connection() above */
	u32 unk_field = (supports_hpd ? DCPDPTX_REMOTE_PORT_SUPPORTS_HPD : 0) |
			(dcp_is_usb4_output(service->ep->dcp) ? 1 : 0);
	int ret;
	u32 target = dptxport_remote_target(service->ep->dcp, core, atc, die);

	trace_dptxport_connect(dptx, core, atc, die);
	dptx->connect_calls++;
	dev_dbg(service->ep->dcp->dev,
		 "DPTX connect: call #%u this boot target=0x%x unk=0x%x caller=%pS\n",
		 dptx->connect_calls, target, unk_field,
		 __builtin_return_address(0));

	cmd.target = cpu_to_le32(target);
	cmd.unk = cpu_to_le32(unk_field);
	ret = afk_service_call(service, 0, 11, &cmd, sizeof(cmd), 24, &resp,
			       sizeof(resp), 24);
	if (ret)
		return ret;

	if (le32_to_cpu(resp.target) != target) {
		dev_warn(service->ep->dcp->dev,
			 "connect: target reply 0x%x (sent 0x%x)\n",
			 le32_to_cpu(resp.target), target);
		return -EINVAL;
	}
	if (le32_to_cpu(resp.unk) != unk_field)
		dev_notice(service->ep->dcp->dev, "unexpected unk field in reply: 0x%x (0x%x)\n",
			  le32_to_cpu(resp.unk), unk_field);

	return 0;
}

int dptxport_request_display(struct apple_epic_service *service)
{
	struct dptx_port *dptx = service->cookie;
	int ret;

	dptx->request_calls++;
	dev_dbg(service->ep->dcp->dev,
		 "DPTX request_display: call #%u this boot caller=%pS\n",
		 dptx->request_calls, __builtin_return_address(0));
	ret = afk_service_call(service, 0, 6, NULL, 0, 16, NULL, 0, 16);
	dev_dbg(service->ep->dcp->dev,
		 "DPTX request_display: call #%u result=%d\n",
		 dptx->request_calls, ret);
	return ret;
}

int dptxport_release_display(struct apple_epic_service *service)
{
	struct dptx_port *dptx = service->cookie;
	int ret;

	dptx->release_calls++;
	dev_dbg(service->ep->dcp->dev,
		 "DPTX release_display: call #%u this boot caller=%pS\n",
		 dptx->release_calls, __builtin_return_address(0));
	ret = afk_service_call(service, 0, 7, NULL, 0, 16, NULL, 0, 16);
	dev_dbg(service->ep->dcp->dev,
		 "DPTX release_display: call #%u result=%d\n",
		 dptx->release_calls, ret);
	return ret;
}

int dptxport_set_hpd_timeout(struct apple_epic_service *service, bool hpd,
			     unsigned int timeout_ms)
{
	struct dcpdptx_hotplug_cmd cmd, resp;
	int ret;

	memset(&cmd, 0, sizeof(cmd));

	if (hpd)
		cmd.unk = cpu_to_le32(1);

	ret = afk_service_call_timeout(service, 8, 8, &cmd, sizeof(cmd), 12,
				       &resp, sizeof(resp), 12, timeout_ms);
	if (ret)
		return ret;
	if (le32_to_cpu(resp.unk) != hpd) {
		dev_warn(service->ep->dcp->dev,
			 "set_hpd: unk reply 0x%x (sent hpd=%d)\n",
			 le32_to_cpu(resp.unk), hpd);
		return -EINVAL;
	}
	return 0;
}

int dptxport_set_hpd(struct apple_epic_service *service, bool hpd)
{
	return dptxport_set_hpd_timeout(service, hpd, MSEC_PER_SEC);
}

/* The sink raised IRQ_HPD: the firmware services the request as a DP source. */
int dptxport_sink_irq(struct apple_epic_service *service)
{
	return afk_service_call(service, 8, 9, NULL, 0, 16, NULL, 0, 16);
}

static int
dptxport_call_get_max_drive_settings(struct apple_epic_service *service,
				     void *reply_, size_t reply_size)
{
	struct dptxport_apcall_max_drive_settings *reply = reply_;

	if (reply_size < sizeof(*reply))
		return -EINVAL;

	reply->retcode = cpu_to_le32(0);
	reply->max_drive_settings[0] = cpu_to_le32(0x3);
	reply->max_drive_settings[1] = cpu_to_le32(0x3);

	return 0;
}

static int
dptxport_call_get_drive_settings(struct apple_epic_service *service,
				     const void *request_, size_t request_size,
				     void *reply_, size_t reply_size)
{
	struct dptx_port *dptx = service->cookie;
	const struct dptxport_apcall_drive_settings *request = request_;
	struct dptxport_apcall_drive_settings *reply = reply_;

	if (reply_size < sizeof(*reply) || request_size < sizeof(*request))
		return -EINVAL;

	*reply = *request;

	/* Clear the rest of the buffer */
	memset(reply_ + sizeof(*reply), 0, reply_size - sizeof(*reply));

	/*
	 * retcode appears to be lane count, seeing 2 for USB-C dp alt mode
	 * with lanes splitted for DP/USB3.
	 */
	if (le32_to_cpu(reply->retcode) != dptx->lane_count)
		dev_err(service->ep->dcp->dev,
			"get_drive_settings: unexpected retcode %d\n",
			reply->retcode);

	reply->retcode = cpu_to_le32(dptx->lane_count);
	reply->unk5 = cpu_to_le32(dptx->drive_settings[0]);
	reply->unk6 = cpu_to_le32(0);
	reply->unk7 = cpu_to_le32(dptx->drive_settings[1]);

	return 0;
}

static int
dptxport_call_set_drive_settings(struct apple_epic_service *service,
				     const void *request_, size_t request_size,
				     void *reply_, size_t reply_size)
{
	struct dptx_port *dptx = service->cookie;
	const struct dptxport_apcall_drive_settings *request = request_;
	struct dptxport_apcall_drive_settings *reply = reply_;

	if (reply_size < sizeof(*reply) || request_size < sizeof(*request))
		return -EINVAL;

	*reply = *request;
	reply->retcode = cpu_to_le32(0);

	dev_info(service->ep->dcp->dev, "set_drive_settings: %d:%d:%d:%d:%d:%d:%d\n",
		 request->unk1, request->unk2, request->unk3, request->unk4,
		 request->unk5, request->unk6, request->unk7);

	dptx->drive_settings[0] = le32_to_cpu(reply->unk5);
	dptx->drive_settings[1] = le32_to_cpu(reply->unk7);

	return 0;
}

static int dptxport_call_get_max_link_rate(struct apple_epic_service *service,
					   void *reply_, size_t reply_size)
{
	struct dptxport_apcall_link_rate *reply = reply_;

	if (reply_size < sizeof(*reply))
		return -EINVAL;

	reply->retcode = cpu_to_le32(0);
	reply->link_rate = cpu_to_le32(LINK_RATE_HBR3);

	return 0;
}

static int dptxport_call_get_max_lane_count(struct apple_epic_service *service,
					   void *reply_, size_t reply_size)
{
	struct dptxport_apcall_lane_count *reply = reply_;
	struct dptx_port *dptx = service->cookie;
	struct apple_dcp *dcp = service->ep->dcp;
	union phy_configure_opts phy_ops;
	int ret;

	if (reply_size < sizeof(*reply))
		return -EINVAL;

	if (!dptx->atcphy) {
		/* USB4 DP IN: no ATC DP PHY to validate. */
		dptx->lane_count = 4;
		reply->retcode = cpu_to_le32(0);
		reply->lane_count = cpu_to_le64(4);
		dev_info(dcp->dev, "get_max_lane_count: USB4 DP IN, 4 lanes\n");
		return 0;
	}

	ret = phy_validate(dptx->atcphy, PHY_MODE_DP, 0, &phy_ops);
	if (ret < 0) {
		dev_err(dcp->dev, "phy_validate failed: %d\n", ret);
		reply->retcode = cpu_to_le32(1);
		reply->lane_count = cpu_to_le64(0);
	} else {
		if (phy_ops.dp.lanes < 2) {
			// phy_validate might return 0 lanes if atc phy is not
			// yet switched to DP mode
			dev_dbg(dcp->dev, "get_max_lane_count: phy lanes: %d\n",
				phy_ops.dp.lanes);
			// default to 4 lanes
			dptx->lane_count = 4;
		} else {
			dptx->lane_count = phy_ops.dp.lanes;
		}
		reply->retcode = cpu_to_le32(0);
		reply->lane_count = cpu_to_le64(dptx->lane_count);
	}

	return 0;
}

static int dptxport_call_set_active_lane_count(struct apple_epic_service *service,
					       const void *data, size_t data_size,
					       void *reply_, size_t reply_size)
{
	struct dptx_port *dptx = service->cookie;
	struct apple_dcp *dcp = service->ep->dcp;
	const struct dptxport_apcall_set_active_lane_count *request = data;
	struct dptxport_apcall_set_active_lane_count *reply = reply_;
	int ret = 0;
	int retcode = 0;

	if (reply_size < sizeof(*reply))
		return -1;
	if (data_size < sizeof(*request))
		return -1;

	u64 lane_count = le64_to_cpu(request->lane_count);

	if (dptx->lane_count < lane_count)
		dev_err(dcp->dev, "set_active_lane_count: unexpected lane "
			"count:%llu phy: %d\n", lane_count, dptx->lane_count);

	switch (lane_count) {
	case 0 ... 2:
	case 4:
		dptx->phy_ops.dp.lanes = lane_count;
		/* USB4 DP IN has no ATC PHY; still accept the lane count. */
		dptx->phy_ops.dp.set_lanes =
			dcp_is_usb4_output(dcp) || (dcp->dptx_phy > 3);
		break;
	default:
		dev_err(dcp->dev, "set_active_lane_count: invalid lane count:%llu\n", lane_count);
		retcode = 1;
		lane_count = 0;
		break;
	}

	if (dptx->phy_ops.dp.set_lanes) {
		if (dptx->atcphy && !dcp_is_usb4_output(dcp)) {
			ret = phy_configure(dptx->atcphy, &dptx->phy_ops);
			if (ret)
				return ret;
		}
		dptx->phy_ops.dp.set_lanes = 0;
		dptx->lane_count = lane_count;
	}

	reply->retcode = cpu_to_le32(retcode);
	reply->lane_count = cpu_to_le64(lane_count);

	if (lane_count > 0) {
		dev_info(dcp->dev, "USB4/DPTX: SET_ACTIVE_LANE_COUNT %llu\n",
			 lane_count);
		/*
		 * dcp_usb4_drm_allowed() (usb4_force_dptx) used to gate this
		 * for the old manual-training sysfs knob (module_param_cb
		 * usb4_dptx_train), removed wholesale in 0dc9f50 when
		 * apple_dcp_tb_dp_tunnel() replaced it with automatic tunnel
		 * detection -- but usb4_force_dptx itself, and this gate,
		 * were left behind with no way left to ever set it true.
		 * dcp_dptx_connect()'s single wait_for_completion_timeout()
		 * on linkcfg_completion is now shared by both the alt-mode
		 * and USB4-tunnel paths (also part of 0dc9f50's unification),
		 * so gating it here meant every tunneled connect timed out
		 * after a fully successful DPRX/AUX handshake and lane
		 * negotiation -- firmware does not send FORCE_HOTPLUG_DETECT
		 * (the only other completion site) for a tunnel. The
		 * reference (aurora-silicon/linux#8) completes
		 * linkcfg_completion here unconditionally; usb4_lane_completion
		 * has no consumer anywhere in this tree, so this drops both the
		 * dead gate and the dead completion and matches the reference.
		 */
		complete(&dptx->linkcfg_completion);
	}

	return ret;
}

static int dptxport_call_get_link_rate(struct apple_epic_service *service,
				       void *reply_, size_t reply_size)
{
	struct dptx_port *dptx = service->cookie;
	struct dptxport_apcall_link_rate *reply = reply_;

	if (reply_size < sizeof(*reply))
		return -EINVAL;

	reply->retcode = cpu_to_le32(0);
	reply->link_rate = cpu_to_le32(dptx->link_rate);

	return 0;
}

static int
dptxport_call_will_change_link_config(struct apple_epic_service *service)
{
	struct dptx_port *dptx = service->cookie;

	dptx->phy_ops.dp.set_lanes = 0;
	dptx->phy_ops.dp.set_rate = 0;
	dptx->phy_ops.dp.set_voltages = 0;

	return 0;
}

static int
dptxport_call_did_change_link_config(struct apple_epic_service *service)
{
	/*
	 * Ported from aurora-silicon/linux#8: bringing the tunnel crossbar
	 * connection up (dcp_tunnel_crossbar_up()) now happens in
	 * dptxport_call(), after this succeeds, gated on
	 * dptx_tunnel && link_rate -- not inside this handler, and its
	 * result is never propagated back to DCP as a call failure.
	 */

	/* assume the link config did change and wait a little bit */
	mdelay(10);

	return 0;
}

static int dptxport_call_set_link_rate(struct apple_epic_service *service,
				       const void *data, size_t data_size,
				       void *reply_, size_t reply_size)
{
	struct dptx_port *dptx = service->cookie;
	const struct dptxport_apcall_link_rate *request = data;
	struct dptxport_apcall_link_rate *reply = reply_;
	u32 link_rate, phy_link_rate;
	bool phy_set_rate = false;
	int ret;

	if (reply_size < sizeof(*reply))
		return -EINVAL;
	if (data_size < sizeof(*request))
		return -EINVAL;

	link_rate = le32_to_cpu(request->link_rate);
	trace_dptxport_call_set_link_rate(dptx, link_rate);
	dev_info(service->ep->dcp->dev, "DPTXPort: SET_LINK_RATE 0x%x\n",
		 link_rate);

	switch (link_rate) {
	case LINK_RATE_RBR:
		phy_link_rate = 1620;
		phy_set_rate = true;
		break;
	case LINK_RATE_HBR:
		phy_link_rate = 2700;
		phy_set_rate = true;
		break;
	case LINK_RATE_HBR2:
		phy_link_rate = 5400;
		phy_set_rate = true;
		break;
	case LINK_RATE_HBR3:
		phy_link_rate = 8100;
		phy_set_rate = true;
		break;
	case 0:
		phy_link_rate = 0;
		phy_set_rate = true;
		break;
	default:
		dev_err(service->ep->dcp->dev,
			"DPTXPort: Unsupported link rate 0x%x requested\n",
			link_rate);
		link_rate = 0;
		phy_set_rate = false;
		break;
	}

	if (phy_set_rate) {
		/*
		 * Ported from aurora-silicon/linux#8: a Thunderbolt DP tunnel
		 * starts/stops its own pixel clock instead of configuring the
		 * PHY directly, and never fails the apcall over it -- a
		 * failed clock just keeps the crossbar connection down
		 * (dcp->tb_clock_ok), which dcp_tunnel_crossbar_up() checks.
		 */
		if (dptx->atcphy && service->ep->dcp->dptx_tunnel) {
			dcp_tunnel_set_rate(service->ep->dcp, dptx->atcphy,
					    dptx->unit, link_rate);
		} else if (dptx->atcphy) {
			/* No clocked crossbar output across a PHY rate change. */
			dcp_direct_crossbar_link(service->ep->dcp, false);
			dptx->phy_ops.dp.link_rate = phy_link_rate;
			dptx->phy_ops.dp.set_rate = 1;
			ret = phy_configure(dptx->atcphy, &dptx->phy_ops);
			if (ret)
				return ret;
		}

		dptx->link_rate = dptx->pending_link_rate = link_rate;
	}

	//dptx->pending_link_rate = link_rate;
	reply->retcode = cpu_to_le32(0);
	reply->link_rate = cpu_to_le32(link_rate);

	return 0;
}

static int dptxport_call_get_supports_hpd(struct apple_epic_service *service,
					  void *reply_, size_t reply_size)
{
	struct apple_dcp *dcp = service->ep->dcp;
	struct dptxport_apcall_get_support *reply = reply_;

	if (reply_size < sizeof(*reply))
		return -EINVAL;

	reply->retcode = cpu_to_le32(0);
	/*
	 * Analog DPIN CORE=1 already uses AFK set_hpd (returns 0).
	 * Denying HPD here made request_display ACTIVATE then 22/24
	 * DEVICE_NOT_STARTED (~5.5s). Advertise HPD so firmware uses
	 * that path instead of waiting for a PHY start.
	 */
	reply->supported = cpu_to_le32(dcp_is_typec_output(dcp) &&
				       !dcp_ext_dptx_607(dcp) ? 1 : 0);
	dev_info(dcp->dev, "DPTXPort: GET_SUPPORTS_HPD %u usb4=%d\n",
		 le32_to_cpu(reply->supported), dcp_is_usb4_output(dcp));
	return 0;
}

static int
dptxport_call_get_supports_downspread(struct apple_epic_service *service,
				      void *reply_, size_t reply_size)
{
	struct dptxport_apcall_get_support *reply = reply_;

	if (reply_size < sizeof(*reply))
		return -EINVAL;

	reply->retcode = cpu_to_le32(0);
	reply->supported = cpu_to_le32(0);
	return 0;
}

static int dptxport_call_set_tiled_display_hint(struct apple_epic_service *service,
						 const void *data, size_t data_size,
						 void *reply_, size_t reply_size)
{
	struct dptxport_apcall_set_tiled *reply = reply_;
	struct dptx_port *dptx = service->cookie;
	const u8 *hint = data;

	if (reply_size < sizeof(*reply))
		return -EINVAL;

	if (dptx)
		smp_store_release(&dptx->tile_hint, false);
	if (dptx && data_size >= DPTX_TILE_HINT_MIN_SIZE) {
		u32 h = get_unaligned_le32(hint + DPTX_TILE_HINT_TILES_H);
		u32 v = get_unaligned_le32(hint + DPTX_TILE_HINT_TILES_V);
		u32 x = get_unaligned_le32(hint + DPTX_TILE_HINT_LOC_X);
		u32 y = get_unaligned_le32(hint + DPTX_TILE_HINT_LOC_Y);

		dptx->tiles_h = min(h, 255U);
		dptx->tiles_v = min(v, 255U);
		dptx->tile_x = min(x, 255U);
		dptx->tile_y = min(y, 255U);
		/* Only the two horizontal tiles carried by these two ports fit. */
		smp_store_release(&dptx->tile_hint,
				  h == 2 && v == 1 && x == dptx->unit && y == 0 &&
				  get_unaligned_le32(hint + DPTX_TILE_HINT_WIDTH) &&
				  get_unaligned_le32(hint + DPTX_TILE_HINT_HEIGHT));
		if (h > 1 || v > 1)
			dev_info(service->ep->dcp->dev,
				 "DPTXPort: port %u carries tile (%u,%u) of a %ux%u tiled %ux%u display\n",
				 dptx->unit, x, y, h, v,
				 get_unaligned_le32(hint + DPTX_TILE_HINT_WIDTH),
				 get_unaligned_le32(hint + DPTX_TILE_HINT_HEIGHT));
	}

	reply->retcode = cpu_to_le32(1);
	return 0;
}

static int
dptxport_call_activate(struct apple_epic_service *service,
		       const void *data, size_t data_size,
		       void *reply, size_t reply_size)
{
	struct dptx_port *dptx = service->cookie;
	struct apple_dcp *dcp = service->ep->dcp;

	/*
	 * Ported from aurora-silicon/linux#8: no crossbar, and no PHY mode
	 * change for a tunnel (dptx->atcphy must stay in USB4/TBT mode the
	 * whole connection, see dcp_tunnel_set_rate()). The DP IN adapter is
	 * only woken (DPTX_INACTIVE=0) here, via dcp_tunnel_dpin_activate();
	 * waking it earlier hangs the machine. Activate always replies
	 * success to DCP.
	 */
	if (dptx->atcphy && !dcp->phy_managed_by_typec)
		phy_set_mode_ext(dptx->atcphy, PHY_MODE_DP, dcp->index);
	if (dcp->dptx_tunnel)
		dcp_tunnel_dpin_activate(dcp, dptx->unit, true);

	memcpy(reply, data, min(reply_size, data_size));
	if (reply_size >= 4)
		memset(reply, 0, 4);

	return 0;
}

static int
dptxport_call_deactivate(struct apple_epic_service *service,
		       const void *data, size_t data_size,
		       void *reply, size_t reply_size)
{
	struct dptx_port *dptx = service->cookie;
	struct apple_dcp *dcp = service->ep->dcp;

	dev_info(dcp->dev, "DPTXPort: DEACTIVATE\n");
	if (dcp->dptx_tunnel)
		dcp_tunnel_dpin_activate(dcp, dptx->unit, false);
	else
		dcp_direct_crossbar_link(dcp, false);
	if (dptx->atcphy && !dcp->phy_managed_by_typec)
		phy_set_mode_ext(dptx->atcphy, PHY_MODE_INVALID, 0);

	memcpy(reply, data, min(reply_size, data_size));
	if (reply_size >= 4)
		memset(reply, 0, 4);

	return 0;
}

static int dptxport_call(struct apple_epic_service *service, u32 idx,
			 const void *data, size_t data_size, void *reply,
			 size_t reply_size)
{
	struct dptx_port *dptx = service->cookie;
	trace_dptxport_apcall(dptx, idx, data_size);
	dev_dbg(service->ep->dcp->dev, "DPTXPort: APCALL %u (%zu bytes)\n",
		 idx, data_size);
	if (data_size)
		print_hex_dump_debug("DPTXPort: apcall data: ",
				     DUMP_PREFIX_OFFSET, 16, 1, data,
				     min(data_size, (size_t)64), true);

	switch (idx) {
	case DPTX_APCALL_WILL_CHANGE_LINKG_CONFIG:
		/*
		 * Ported from aurora-silicon/linux#8: a re-link on an
		 * established tunnel takes the crossbar connection down
		 * first.
		 */
		if (service->ep->dcp->dptx_tunnel && dptx->link_rate)
			dcp_tunnel_crossbar_down(service->ep->dcp, dptx->unit);
		else if (dptx->link_rate)
			dcp_direct_crossbar_link(service->ep->dcp, false);
		return dptxport_call_will_change_link_config(service);
	case DPTX_APCALL_DID_CHANGE_LINK_CONFIG: {
		int ret = dptxport_call_did_change_link_config(service);

		if (!ret && service->ep->dcp->dptx_tunnel && dptx->link_rate)
			dcp_tunnel_crossbar_up(service->ep->dcp, dptx->unit);
		else if (!ret && dptx->link_rate)
			dcp_direct_crossbar_link(service->ep->dcp, true);
		if (!ret)
			dcp_hdmi_link_configured(service->ep->dcp, dptx->link_rate);
		return ret;
	}
	case DPTX_APCALL_GET_MAX_LINK_RATE:
		return dptxport_call_get_max_link_rate(service, reply,
						       reply_size);
	case DPTX_APCALL_GET_LINK_RATE:
		return dptxport_call_get_link_rate(service, reply, reply_size);
	case DPTX_APCALL_SET_LINK_RATE:
		return dptxport_call_set_link_rate(service, data, data_size,
						   reply, reply_size);
	case DPTX_APCALL_GET_MAX_LANE_COUNT:
		return dptxport_call_get_max_lane_count(service, reply, reply_size);
	case DPTX_APCALL_GET_ACTIVE_LANE_COUNT: {
		struct dptxport_apcall_lane_count *lc = reply;

		if (reply_size < sizeof(*lc))
			return -EINVAL;
		lc->retcode = cpu_to_le32(0);
		lc->lane_count = cpu_to_le64(dptx->lane_count);
		return 0;
	}
        case DPTX_APCALL_SET_ACTIVE_LANE_COUNT:
		return dptxport_call_set_active_lane_count(service, data, data_size,
							   reply, reply_size);
	case DPTX_APCALL_GET_SUPPORTS_HPD:
		return dptxport_call_get_supports_hpd(service, reply,
						      reply_size);
	case DPTX_APCALL_GET_SUPPORTS_DOWN_SPREAD:
		return dptxport_call_get_supports_downspread(service, reply,
							     reply_size);
	case DPTX_APCALL_GET_MAX_DRIVE_SETTINGS:
		return dptxport_call_get_max_drive_settings(service, reply,
							    reply_size);
	case DPTX_APCALL_DEVICE_NOT_RESPONDING:
	case DPTX_APCALL_DEVICE_BUSY_TIMEOUT:
	case DPTX_APCALL_DEVICE_NOT_STARTED:
		dev_warn(service->ep->dcp->dev,
			 "DPTXPort: firmware reports link fault %u on target %u:%u\n",
			 idx, service->ep->dcp->dptx_die,
			 service->ep->dcp->dptx_phy);
		memcpy(reply, data, min(reply_size, data_size));
		return 0;
	case DPTX_APCALL_SET_TILED_DISPLAY_HINTS: {
		int ret;

		memcpy(reply, data, min(reply_size, data_size));
		ret = dptxport_call_set_tiled_display_hint(service, data,
							   data_size, reply,
							   reply_size);
		/* The T8122 dcpext gets 0, as from AsahiLinux/linux#607. */
		if (!ret && reply_size >= 4 && dcp_ext_dptx_607(service->ep->dcp))
			memset(reply, 0, 4);
		return ret;
	}
	case DPTX_APCALL_GET_DRIVE_SETTINGS:
		return dptxport_call_get_drive_settings(service, data, data_size,
							reply, reply_size);
	case DPTX_APCALL_SET_DRIVE_SETTINGS:
		return dptxport_call_set_drive_settings(service, data, data_size,
							reply, reply_size);
        case DPTX_APCALL_ACTIVATE:
		return dptxport_call_activate(service, data, data_size,
					      reply, reply_size);
	case DPTX_APCALL_DEACTIVATE:
		return dptxport_call_deactivate(service, data, data_size,
						reply, reply_size);
	case DPTX_APCALL_FORCE_HOTPLUG_DETECT:
		dev_info(service->ep->dcp->dev,
			 "DPTXPort: FORCE_HOTPLUG_DETECT\n");
		complete(&dptx->linkcfg_completion);
		memcpy(reply, data, min(reply_size, data_size));
		if (reply_size >= 4)
			memset(reply, 0, 4);
		return 0;
	case DPTX_APCALL_INACTIVE_SINK_DETECTED:
		/*
		 * Normal prelude to link training on USB4. Ack and wait for
		 * SET_ACTIVE_LANE_COUNT. Treating this as failure made
		 * firmware DEACTIVATE after it had already set lanes/rate.
		 */
		dev_info(service->ep->dcp->dev,
			 "DPTXPort: INACTIVE_SINK_DETECTED (keep waiting for lanes)\n");
		memcpy(reply, data, min(reply_size, data_size));
		if (reply_size >= 4)
			memset(reply, 0, 4);
		return 0;
	default:
		/* just try to ACK and hope for the best... */
		dev_info(service->ep->dcp->dev, "DPTXPort: acking unhandled call %u\n",
			idx);
		fallthrough;
	case DPTX_APCALL_GET_DOWN_SPREAD:
	case DPTX_APCALL_SET_DOWN_SPREAD:
	case DPTX_APCALL_SET_ACTIVE:
		memcpy(reply, data, min(reply_size, data_size));
		if (reply_size >= 4)
			memset(reply, 0, 4);
		return 0;
	}
}

static void dptxport_init(struct apple_epic_service *service, const char *name,
			  const char *class, s64 unit)
{
	bool v14_7 = dcp_fw_is_14_7(service->ep->dcp);

	if (!name)
		return;
	/* The J613 internal pipe's own panel port is never bound. */
	if (service->ep->dcp->clamshell && !strcmp(name, "dcp-lpdptx-port-epic"))
		return;
	if (strcmp(name, "dcpdptx-port-epic") &&
	    (!v14_7 || strcmp(name, "dcp-lpdptx-port-epic")))
		return;
	/* 14.7 may announce the port without its class. */
	if (v14_7 ? class && class[0] && strcmp(class, "AppleDCPDPTXRemotePort") :
		    !class || strcmp(class, "AppleDCPDPTXRemotePort"))
		return;

	trace_dptxport_init(service->ep->dcp, unit);

	switch (unit) {
	case 0:
	case 1:
		if (service->ep->dcp->dptxport[unit].enabled) {
			dev_err(service->ep->dcp->dev,
				"DPTXPort: unit %lld already exists\n", unit);
			return;
		}
		service->ep->dcp->dptxport[unit].unit = unit;
		service->ep->dcp->dptxport[unit].service = service;
		service->cookie = (void *)&service->ep->dcp->dptxport[unit];
		/* Publish the complete service before boot-time tunnel routing. */
		smp_store_release(&service->ep->dcp->dptxport[unit].enabled, true);
		complete(&service->ep->dcp->dptxport[unit].enable_completion);
		/* A connect that found no port yet is retried from process context. */
		if (service->ep->dcp->external_native)
			schedule_work(&service->ep->dcp->external_ready_work);
		break;
	default:
		dev_err(service->ep->dcp->dev, "DPTXPort: invalid unit %lld\n",
			unit);
	}
}

static const struct apple_epic_service_ops dptxep_ops[] = {
	{
		.name = "AppleDCPDPTXRemotePort",
		.init = dptxport_init,
		.call = dptxport_call,
	},
	{}
};

/* 14.7 also announces the port under its EPIC names. */
static const struct apple_epic_service_ops dptxep_v14_7_ops[] = {
	{
		.name = "AppleDCPDPTXRemotePort",
		.init = dptxport_init,
		.call = dptxport_call,
	},
	{
		.name = "dcpdptx-port-epic",
		.init = dptxport_init,
		.call = dptxport_call,
	},
	{
		.name = "dcp-lpdptx-port-epic",
		.init = dptxport_init,
		.call = dptxport_call,
	},
	{}
};

int dptxep_init(struct apple_dcp *dcp)
{
	int ret;
	u32 port;
	unsigned long timeout = msecs_to_jiffies(1000);

	init_completion(&dcp->dptxport[0].enable_completion);
	init_completion(&dcp->dptxport[1].enable_completion);
	init_completion(&dcp->dptxport[0].linkcfg_completion);
	init_completion(&dcp->dptxport[1].linkcfg_completion);

	dcp->dptxep = afk_init(dcp, DPTX_ENDPOINT,
			       dcp_fw_is_14_7(dcp) ?
			       dptxep_v14_7_ops : dptxep_ops);
	if (IS_ERR(dcp->dptxep))
		return PTR_ERR(dcp->dptxep);

	ret = afk_start(dcp->dptxep);
	if (ret)
		return ret;

	for (port = 0; port < dcp->hw.num_dptx_ports; port++) {
		ret = wait_for_completion_timeout(&dcp->dptxport[port].enable_completion,
						timeout);
		if (!ret) {
			/*
			 * 14.7 does not announce the port during boot. A fake
			 * channel 0 is interface_id 0, and the firmware answers
			 * kIOReturnNoDevice (0xe00002c0). Leave the endpoint
			 * up so a later announce can still bind.
			 */
			if (dcp_fw_is_14_7(dcp)) {
				dev_info(dcp->dev,
					 "DPTX port %u not announced yet\n", port);
				return 0;
			}
			return -ETIMEDOUT;
		} else if (ret < 0)
			return ret;
		timeout = ret;
	}

	return 0;
}

static void dpav_ctrl_bind(struct apple_epic_service *service, const char *name,
			   const char *class, s64 unit)
{
	struct apple_dcp *dcp = service->ep->dcp;

	dev_info(dcp->dev,
		 "DPAV service %s class %s unit %lld channel %u\n",
		 name ? name : "", class ? class : "", unit, service->channel);
	if (name && !strcmp(name, "dcpav-controller-epic")) {
		dcp->dpav_ctrl = service;
		complete_all(&dcp->dpav_ctrl_ready);
	}
}

static const struct apple_epic_service_ops dpav_ctrl_ops[] = {
	{
		.name = "dcpav-controller-epic",
		.init = dpav_ctrl_bind,
	},
	{
		.name = "DCPAVController",
		.init = dpav_ctrl_bind,
	},
	{
		.name = "dcpdp-controller-epic",
		.init = dpav_ctrl_bind,
	},
	{}
};

int dpav_ctrl_init(struct apple_dcp *dcp)
{
	int ret;

	init_completion(&dcp->dpav_ctrl_ready);
	dcp->dpavctrlep = afk_init(dcp, DPAV_CTRL_ENDPOINT, dpav_ctrl_ops);
	if (IS_ERR(dcp->dpavctrlep))
		return PTR_ERR(dcp->dpavctrlep);

	ret = afk_start(dcp->dpavctrlep);
	if (ret) {
		dev_warn(dcp->dev, "DPAV controller endpoint failed to start: %d\n",
			 ret);
		return 0;
	}

	if (!wait_for_completion_timeout(&dcp->dpav_ctrl_ready,
					 msecs_to_jiffies(1000)))
		dev_info(dcp->dev, "DPAV controller did not announce a service\n");
	return 0;
}
