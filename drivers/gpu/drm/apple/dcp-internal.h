// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Copyright 2021 Alyssa Rosenzweig */

#ifndef __APPLE_DCP_INTERNAL_H__
#define __APPLE_DCP_INTERNAL_H__

#include <linux/backlight.h>
#include <linux/device.h>
#include <linux/ioport.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/mux/consumer.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/usb/typec_mux.h>

#include "dcp-fabric.h"
#include "dcp-fabric-session.h"
#include "dptxep.h"
#include "iomfb.h"
#include "iomfb-state.h"
#include "iomfb_v12_3.h"
#include "iomfb_v13_3.h"
#include "iomfb_v14_7.h"
#include "iomfb_v14_7_tpl.h"
#include "iomfb_v26_6.h"
#include "epic/dpavservep.h"

#define DCP_MAX_PLANES 4

struct apple_dcp_afkep;

void dcp_swap_watchdog_arm(struct apple_dcp *dcp);
void dcp_swap_watchdog_complete(struct apple_dcp *dcp);
void dcp_retry_placeholder_edid(struct apple_dcp *dcp,
				const struct drm_edid *drm_edid);

struct dcpav_service_epic;

enum dcp_firmware_version {
	DCP_FIRMWARE_UNKNOWN,
	DCP_FIRMWARE_V_12_3,
	DCP_FIRMWARE_V_13_5,
	DCP_FIRMWARE_V_14_7,
	DCP_FIRMWARE_V_26_6,
	/*
	 * 14.7 firmware on a processor that Linux starts itself and drives
	 * through the classic IOMFB template (iomfb_v14_7_tpl.c), as M1/M2
	 * external processors: the T8122 dcpext. Its AFK, DPTX and DPAV
	 * endpoints speak the 14.7 protocol (dcp_fw_is_14_7()).
	 */
	DCP_FIRMWARE_V_14_7_TEMPLATE,
};

enum {
	SYSTEM_ENDPOINT = 0x20,
	TEST_ENDPOINT = 0x21,
	DCP_EXPERT_ENDPOINT = 0x22,
	DISP0_ENDPOINT = 0x23,
	/* DPAV controller: publishes the port service used by DPTX 0x2a. */
	DPAV_CTRL_ENDPOINT = 0x24,
	DPAVSERV_ENDPOINT = 0x28,
	AV_ENDPOINT = 0x29,
	DPTX_ENDPOINT = 0x2a,
	HDCP_ENDPOINT = 0x2b,
	REMOTE_ALLOC_ENDPOINT = 0x2d,
	IOMFB_ENDPOINT = 0x37,
};

/* Temporary backing for a chunked transfer via setDCPAVPropStart/Chunk/End */
struct dcp_chunks {
	u64 modes_generation;
	size_t length;
	void *data;
};

#define DCP_MAX_MAPPINGS (128) /* should be enough */
#define MAX_DISP_REGISTERS (7)

struct dcp_mem_descriptor {
	size_t size;
	void *buf;
	dma_addr_t dva;
	struct sg_table map;
	u64 reg;
};

/* Limit on call stack depth (arbitrary). Some nesting is required */
#define DCP_MAX_CALL_DEPTH 8

typedef void (*dcp_callback_t)(struct apple_dcp *, void *, void *);

struct dcp_channel {
	dcp_callback_t callbacks[DCP_MAX_CALL_DEPTH];
	void *cookies[DCP_MAX_CALL_DEPTH];
	void *output[DCP_MAX_CALL_DEPTH];
	u16 end[DCP_MAX_CALL_DEPTH];

	/* Current depth of the call stack. Less than DCP_MAX_CALL_DEPTH */
	u8 depth;
	/* Already warned about busy channel */
	bool warned_busy;
};

struct dcp_fb_reference {
	struct list_head head;
	struct drm_framebuffer *fb;
	u32 swap_id;
};

#define MAX_NOTCH_HEIGHT 160

struct dcp_brightness {
	struct backlight_device *bl_dev;
	u32 maximum;
	u32 dac;
	int nits;
	int scale;
	bool update;
};

struct audiosrv_data;

/** laptop/AiO integrated panel parameters from DT */
struct dcp_panel {
	/// panel width in millimeter
	int width_mm;
	/// panel height in millimeter
	int height_mm;
	/// panel has a mini-LED backlight
	bool has_mini_led;
};

struct apple_dcp_hw_data {
	u32 num_dptx_ports;
	bool t6020_tunnel_flow;
};

/* TODO: move IOMFB members to its own struct */
struct dcpext_scanout;
struct dcp_hdmi;

struct apple_dcp {
	struct dcp_fabric_pipeline fabric;
	struct device *dev;
	struct platform_device *piodma;
	struct iommu_domain *iommu_dom;
	struct apple_rtkit *rtk;
	struct apple_crtc *crtc;
	struct apple_connector *connector;

	struct apple_dcp_hw_data hw;

	/* firmware version and compatible firmware version */
	enum dcp_firmware_version fw_compat;
	bool external;
	/*
	 * T8122 dcpext: the panel's DRM device never waits on this processor.
	 * A probe or bind failure leaves it in the device as an unusable pipe
	 * (ext_dead): no CRTC, Type-C routes disabled or ignored, RTKit
	 * messages dropped. See dcp_platform_probe().
	 */
	bool ext_failsafe;
	bool ext_dead;
	/*
	 * A T6030 external processor in the main DRM device: the kernel starts
	 * its firmware on first use, its modes come from the attached display.
	 * Clear for the manual diagnostic path (separate scanout, explicit start).
	 */
	bool external_native;
	struct dcpext_scanout *dcpext_scanout;
	bool external_link_ready; /* Full connect/HPD handshake completed. */
	bool external_suspended; /* hpd_mutex serializes PM against explicit start */
	struct work_struct external_work;
	/* Native: retries pending connects once DPTX ports are announced. */
	struct work_struct external_ready_work;
	/* Native: the firmware's start state, see iomfb_v14_7.c. */
	unsigned int external_phase;
	/* Native: bounded recovery of a display link or mode, see dcp.c. */
	struct delayed_work external_retry_wq;
	atomic_t external_retries;
	/* typec_generation when the pending retry was queued */
	u64 external_retry_generation;
	/*
	 * Native: the connector is kept connected while the display is
	 * described again, until the pending retry, once per connection
	 * (typec_generation + 1 of the last hold); see iomfb_v14_7.c.
	 */
	atomic_t external_held;
	u64 external_held_connection;
	/*
	 * Native: DRM unbound this pipe; firmware callbacks leave its
	 * connector and CRTC alone. Set under the session lock.
	 */
	bool external_detached;
	atomic_t external_requested;

	/* DCP_FIRMWARE_V_14_7 state; outlives this device once RTKit runs. */
	struct apple_dcp_v14 *v14;
	/*
	 * J613: the internal panel's pipe also has a Type-C route. The driver
	 * tells its firmware the lid state, and powers the panel off with its
	 * CRTC while the lid is closed, as macOS does (see dcp_lid_*()).
	 */
	bool clamshell;
	struct work_struct clamshell_start_work;
	/*
	 * The lid is closed and the panel and its CRTC off: the route may
	 * drive a Type-C display (see dcp_typec_pipeline_changed()). While the
	 * lid is closed, and until a display that had the pipe let go of it,
	 * the panel's connector is reported disconnected (clamshell_panel_away;
	 * clamshell_panel_work).
	 */
	bool clamshell_free;
	bool clamshell_panel_away;
	/* The pipe's one CRTC is on (dcp_poweron() to dcp_poweroff()). */
	bool clamshell_crtc_on;
	struct delayed_work clamshell_panel_work;
	struct apple_dcp_v26 *v26;

	/* Coprocessor control register */
	void __iomem *coproc_reg;

	/* DCP has crashed */
	bool crashed;

	DECLARE_BITMAP(iomfb_surfaces, DCP_MAX_PLANES);

	/************* IOMFB **************************************************
	 * everything below is mostly used inside IOMFB but it could make     *
	 * sense to keep some of the members in apple_dcp.                    *
	 **********************************************************************/

	/* clock rate request by dcp in */
	struct clk *clk;

	/* DCP shared memory */
	void *shmem;

	/* Display registers mappable to the DCP */
	struct resource *disp_registers[MAX_DISP_REGISTERS];
	unsigned int nr_disp_registers;

	struct resource disp_bw_scratch_res;
	struct resource disp_bw_doorbell_res;
	u32 disp_bw_scratch_index;
	u32 disp_bw_scratch_offset;
	u32 disp_bw_doorbell_index;
	u32 disp_bw_doorbell_offset;

	u32 index;

	/* Bitmap of memory descriptors used for mappings made by the DCP */
	DECLARE_BITMAP(memdesc_map, DCP_MAX_MAPPINGS);

	/* Indexed table of memory descriptors */
	struct dcp_mem_descriptor memdesc[DCP_MAX_MAPPINGS];

	struct dcp_channel ch_cmd, ch_oobcmd;
	struct dcp_channel ch_cb, ch_oobcb, ch_async, ch_oobasync;

	/* iomfb EP callback handlers */
	const iomfb_cb_handler *cb_handlers;

	/* Active chunked transfer. There can only be one at a time. */
	struct dcp_chunks chunks;

	/* Queued swap. Owned by the DCP to avoid per-swap memory allocation */
	union {
		struct dcp_swap_submit_req_v12_3 v12_3;
		struct dcp_swap_submit_req_v13_3 v13_3;
		struct dcp_swap_submit_req_v14_7_0 v14_7_0;
	} swap;

	/* swap id of the last completed swap */
	u32 last_swap_id;
	ktime_t swap_start;
	u64 swap_submit_timestamp;

	/* Current display mode */
	struct dcp_mode_state mode_state;
	/* One HPD pulse after a placeholder EDID, per Type-C connection. */
	bool placeholder_retried;
	u64 typec_generation;	/* hpd_mutex: identifies the current connection */
	/* hpd_mutex: Type-C displays unplugged so far; see dcp_poweroff() */
	u64 typec_unplugs;
	/* typec_unplugs when the CRTC was last powered on */
	u64 crtc_unplugs;
	u64 placeholder_generation;
	struct delayed_work placeholder_edid_wq;
	bool use_timestamps;
	bool vrr_enabled;
	struct dcp_set_digital_out_mode_req mode;

	/* completion for active turning true */
	struct completion start_done;

	/* Is the DCP booted? */
	bool active;

	/* eDP display without DP-HDMI conversion */
	bool main_display;

	/* clear all surfaces on init */
	bool surfaces_cleared;

	/* enable CRC calculation */
	bool crc_enabled;

	/* Modes valid for the connected display */
	/* Readers copy a mode before dropping this lock. */
	struct mutex modes_lock;
	struct dcp_display_mode *modes;
	unsigned int nr_modes;
	u64 modes_generation;
	u64 dimensions_generation;
	struct work_struct dimensions_wq;
	bool modes_admitted;
	bool modes_provisional;

	/* Attributes of the connector */
	int connector_type;
	int fixed_connector_type;

	/* Attributes of the connected display */
	int width_mm, height_mm;
	/* an external display whose backlight DCP can drive */
	bool ext_backlight;

	unsigned notch_height;

	/* Workqueue for sending vblank events when a dcp swap is not possible */
	struct work_struct vblank_wq;

	/* Completes a Type-C swap that DCP dropped, and recovers the pipe. */
	struct delayed_work swap_watchdog_wq;
	unsigned int swap_watchdog_retrains;

	/* List of referenced drm_framebuffers which can be unreferenced
	 * on the next successfully completed swap.
	 */
	struct list_head swapped_out_fbs;

	struct dcp_brightness brightness;
	/* Workqueue for updating the initial brightness */
	struct work_struct bl_register_wq;
	struct mutex bl_register_mutex;
	/* Workqueue for updating the brightness */
	struct work_struct bl_update_wq;

	/* integrated panel if present */
	struct dcp_panel panel;

	struct apple_dcp_afkep *systemep;
	struct completion systemep_done;

	struct apple_dcp_afkep *ibootep;
	struct apple_dcp_afkep *dcpavservep;
	struct dcpavserv dcpavserv;

	struct apple_dcp_afkep *avep;
	struct audiosrv_data *audiosrv;

	struct apple_dcp_afkep *dptxep;

	struct apple_dcp_afkep *dpavctrlep;
	struct apple_epic_service *dpav_ctrl;
	struct completion dpav_ctrl_ready;
	bool dpav_ctrl_open;

	struct dptx_port dptxport[2];

	/* debugfs entries */
	struct dentry *ep_debugfs[0x20];

	/* these fields are output port specific */
	struct phy *phy;
	struct phy *fixed_phy;
	struct mux_control *xbar;
	struct typec_mux *typec_mux;
	struct apple_dcp_typec_route typec_routes[DCP_MAX_TYPEC_ROUTES];
	struct apple_dcp_typec_route *active_typec_route;
	u32 nr_typec_routes;
	bool phy_managed_by_typec;
	bool typec_cable_connected;
	/* DPTX feeds a Thunderbolt DP IN adapter, not the Type-C PHY lanes */
	bool dptx_tunnel;
	/* DFP port in the DPTX target: 0 = dpphy, 1 = dpin0, 2 = dpin1 */
	u8 dptx_dfp_port;
	/* wakes/sleeps the Thunderbolt DP IN adapter from DCP Activate/Deactivate */
	int (*tb_dpin_set_active)(void *ctx, bool active);
	void *tb_dpin_ctx;
	u64 tb_generation; /* tb_lock: admitted callback cookie, including attach */
	bool tb_retiring; /* fabric lock: pipeline reserved until binding drain */
	/*
	 * Serializes the Thunderbolt DP IN callback and tunnel crossbar state
	 * between DCP apcalls and tunnel teardown; never held while waiting
	 * for DCP.
	 */
	struct mutex tb_lock;
	bool tb_clock_ok;
	/* tb_lock: the direct DP PHY crossbar output whose clocks run (T6030). */
	struct mux_control *direct_xbar_up;
	/*
	 * Tiled display (LG UltraFine 5K): the second Thunderbolt DP IN of the
	 * same port drives DPTX port 1 of this pipeline, so DCP sees both tiles
	 * and presents one display. Under tb_lock like the fields above.
	 */
	struct {
		bool active;
		bool ready; /* admitted second-stream setup */
		struct mux_control *xbar;	/* the port's dpin1 crossbar control */
		int mux_state;			/* dispextN, DPTX port 1 */
		bool xbar_up;
		bool clock_ok;
		u64 generation;			/* the Thunderbolt binding's */
		int (*set_active)(void *binding, bool active);
		void *binding;
	} split;
	/* CRTC powered off while the Type-C cable stays attached */
	bool typec_crtc_off;
	bool typec_follow_start;
	bool typec_follow_retiring;
	u64 typec_follow_gen;
	/* IOMFB reports its video interface ready after DPTX link training. */
	struct completion typec_iomfb_hpd_ready;
	struct delayed_work typec_reconnect_wq;
	u32 typec_reconnect_tries;
	struct dcp_fabric_session typec_reconnect_session; /* hpd_mutex */
	bool typec_work_enabled; /* hpd_mutex; IRQ recovery admission */

	struct gpio_desc *hdmi_hpd;
	struct gpio_desc *hdmi_pwren;
	struct gpio_desc *dp2hdmi_pwren;

	struct mutex hpd_mutex;

	u32 dptx_phy;
	u32 dptx_die;
	u32 fixed_dptx_phy;
	u32 fixed_mux_index;
	bool fixed_route_selected;
	struct apple_connector *fixed_connector;
	struct apple_connector *typec_connector;
	int hdmi_hpd_irq;
	/* Hardirq, resume and expiry share one generation under this lock. */
	spinlock_t hdmi_presence_lock;
	struct dcp_fabric_presence hdmi_presence;
	unsigned long hdmi_edge_jiffies;
	bool hdmi_edge_seen;
	struct delayed_work hdmi_settle_wq;
	struct delayed_work hdmi_recheck_wq;
	/* T6030 HDMI converter service requests, see dcp-hdmi.c */
	struct dcp_hdmi *hdmi;
};

void dcp_drm_crtc_page_flip(struct apple_dcp *dcp, ktime_t now);
void dcp_handle_hotplug_actions(struct apple_dcp *dcp, unsigned int action);

int dcp_backlight_register(struct apple_dcp *dcp);
int dcp_backlight_update(struct apple_dcp *dcp);
s32 dcp_ext_backlight_value(struct apple_dcp *dcp);
bool dcp_has_panel(struct apple_dcp *dcp);

/*
 * Does @dcp run 14.7 firmware, whichever IOMFB path drives it? For the
 * firmware protocol of the AFK, DPTX and DPAV endpoints. The IOMFB path
 * itself is chosen by fw_compat: DCP_FIRMWARE_V_14_7 is Aurora's DCPLink
 * session, DCP_FIRMWARE_V_14_7_TEMPLATE the classic IOMFB template.
 */
static inline bool dcp_fw_is_14_7(const struct apple_dcp *dcp)
{
	return dcp->fw_compat == DCP_FIRMWARE_V_14_7 ||
	       dcp->fw_compat == DCP_FIRMWARE_V_14_7_TEMPLATE;
}

/*
 * The internal panel's processor on Aurora's DCPLink session (T6030, T8122).
 * On J613 it also has a Type-C display route, which the display gate enables
 * at boot together with the dcpext.
 */
static inline bool dcp_is_v14_panel(const struct apple_dcp *dcp)
{
	return dcp->fw_compat == DCP_FIRMWARE_V_14_7 && !dcp->external;
}

/* J613 internal pipe: its route took a Type-C display; the lid changed. */
void dcp_clamshell_attached(struct apple_dcp *dcp, struct device_node *connector_np);
void dcp_clamshell_changed(struct apple_dcp *dcp);

/*
 * The T8122 dcpext connects its DPTX port as AsahiLinux/linux#607 does,
 * the kernel this firmware was first driven with: no validation of the
 * target, a connect without the supports-HPD attribute, GET_SUPPORTS_HPD
 * answered with 0, HPD asserted only after the link-configuration wait,
 * and tiled display hints acknowledged with 0. So does the J613 internal
 * pipe on its Type-C route, whose firmware is the same image. Every other
 * DCP keeps the Type-C model of dcp_dptx_connect_session().
 */
static inline bool dcp_ext_dptx_607(const struct apple_dcp *dcp)
{
	return dcp->fw_compat == DCP_FIRMWARE_V_14_7_TEMPLATE || dcp->clamshell;
}

#define DCP_AUDIO_MAX_CHANS 15

#endif /* __APPLE_DCP_INTERNAL_H__ */
