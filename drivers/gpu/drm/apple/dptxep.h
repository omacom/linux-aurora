#ifndef __APPLE_DCP_DPTXEP_H__
#define __APPLE_DCP_DPTXEP_H__

#include <linux/phy/phy.h>
#include <linux/mux/consumer.h>

enum dptx_apcall {
	DPTX_APCALL_ACTIVATE = 0,
	DPTX_APCALL_DEACTIVATE = 1,
	DPTX_APCALL_GET_MAX_DRIVE_SETTINGS = 2,
	DPTX_APCALL_SET_DRIVE_SETTINGS = 3,
	DPTX_APCALL_GET_DRIVE_SETTINGS = 4,
	DPTX_APCALL_WILL_CHANGE_LINKG_CONFIG = 5,
	DPTX_APCALL_DID_CHANGE_LINK_CONFIG = 6,
	DPTX_APCALL_GET_MAX_LINK_RATE = 7,
	DPTX_APCALL_GET_LINK_RATE = 8,
	DPTX_APCALL_SET_LINK_RATE = 9,
	DPTX_APCALL_GET_MAX_LANE_COUNT = 10,
	DPTX_APCALL_GET_ACTIVE_LANE_COUNT = 11,
	DPTX_APCALL_SET_ACTIVE_LANE_COUNT = 12,
	DPTX_APCALL_GET_SUPPORTS_DOWN_SPREAD = 13,
	DPTX_APCALL_GET_DOWN_SPREAD = 14,
	DPTX_APCALL_SET_DOWN_SPREAD = 15,
	DPTX_APCALL_GET_SUPPORTS_LANE_MAPPING = 16,
	DPTX_APCALL_SET_LANE_MAP = 17,
	DPTX_APCALL_GET_SUPPORTS_HPD = 18,
	DPTX_APCALL_FORCE_HOTPLUG_DETECT = 19,
	DPTX_APCALL_INACTIVE_SINK_DETECTED = 20,
	DPTX_APCALL_SET_TILED_DISPLAY_HINTS = 21,
	DPTX_APCALL_DEVICE_NOT_RESPONDING = 22,
	DPTX_APCALL_DEVICE_BUSY_TIMEOUT = 23,
	DPTX_APCALL_DEVICE_NOT_STARTED = 24,
	DPTX_APCALL_GET_VIRTUAL_DEVICE_MODE = 26,
};

#define DCPDPTX_REMOTE_PORT_CORE GENMASK(3, 0)
#define DCPDPTX_REMOTE_PORT_ATC GENMASK(7, 4)
#define DCPDPTX_REMOTE_PORT_DIE GENMASK(11, 8)
#define DCPDPTX_REMOTE_PORT_CONNECTED BIT(15)
#define DCPDPTX_REMOTE_PORT_SUPPORTS_HPD BIT(8)

enum dptx_link_rate {
	LINK_RATE_RBR = 0x06,
	LINK_RATE_HBR = 0x0a,
	LINK_RATE_HBR2 = 0x14,
	LINK_RATE_HBR3 = 0x1e,
};

struct apple_epic_service;

struct dptx_port {
	bool enabled, connected;
	/* A published session closed; unlike initial discovery, this is terminal. */
	bool closed;
	struct completion enable_completion;
	struct completion linkcfg_completion;
	u32 unit;
	struct apple_epic_service *service;
	union phy_configure_opts phy_ops;
	struct phy *atcphy;
	struct mux_control *mux;
	u32 lane_count;
	u32 link_rate, pending_link_rate;
	u32 drive_settings[2];
	u32 trained_voltage[4], trained_pre[4];
	/* The 14.7 external role: the sink is inactive (callback 20) until a link-up. */
	bool sink_inactive;
	/*
	 * Per-boot invocation counters for the validate/connect/request/
	 * release AFK calls, used together with the caller return address
	 * logged alongside them to correlate call ordering when diagnosing
	 * link-training sequencing issues.
	 */
	u32 validate_calls, connect_calls, request_calls, release_calls;
	/*
	 * The sink's tile topology as DCP reports it in SetTiledDisplayHints:
	 * tiles across and down, and this stream's tile. Valid once
	 * tile_hint is set; cleared whenever the port (re)connects.
	 */
	bool tile_hint;
	u8 tiles_h, tiles_v, tile_x, tile_y;
};

int dptxport_validate_connection(struct apple_epic_service *service, u8 core,
				 u8 atc, u8 die);
int dptxport_connect(struct apple_epic_service *service, u8 core, u8 atc,
		     u8 die, bool supports_hpd);
int dptxport_request_display(struct apple_epic_service *service);
int dptxport_release_display(struct apple_epic_service *service);
int dptxport_set_hpd(struct apple_epic_service *service, bool hpd);
int dptxport_set_hpd_timeout(struct apple_epic_service *service, bool hpd,
			     unsigned int timeout_ms);
int dptxport_sink_irq(struct apple_epic_service *service);


struct apple_dcp;
int dpav_ctrl_init(struct apple_dcp *dcp);
#endif
