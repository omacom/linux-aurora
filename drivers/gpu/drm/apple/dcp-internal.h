// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Copyright 2021 Alyssa Rosenzweig */

#ifndef __APPLE_DCP_INTERNAL_H__
#define __APPLE_DCP_INTERNAL_H__

#include <linux/backlight.h>
#include <linux/device.h>
#include <linux/ioport.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/mux/consumer.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/usb/typec_mux.h>

#include "dcp-link.h"
#include "dptxep.h"
#include "iomfb.h"
#include "iomfb_h17p.h"
#include "iomfb_v12_3.h"
#include "iomfb_v13_3.h"
#include "iomfb-surface.h"
#include "epic/dpavservep.h"

#define DCP_MAX_PLANES APPLE_DCP_MAX_SURFACES
#define DCP_MAX_TYPEC_ROUTES 4

struct apple_dcp;

struct apple_dcp_afkep;
struct apple_dcp_typec_port;

struct apple_dcp_typec_route {
	struct apple_dcp *dcp;
	struct apple_dcp_typec_port *port;
	struct list_head port_link;
	struct phy *phy;
	struct mux_control *xbar;
	struct typec_mux_dev *typec_mux;
	u32 dptx_phy;
	u32 mux_index;
	bool selected;
};

bool dcp_is_typec_output(struct apple_dcp *dcp);

struct dcpav_service_epic;

enum dcp_firmware_version {
	DCP_FIRMWARE_UNKNOWN,
	DCP_FIRMWARE_V_12_3,
	DCP_FIRMWARE_V_13_5,
	DCP_FIRMWARE_H17P,
};

enum {
	SYSTEM_ENDPOINT = 0x20,
	TEST_ENDPOINT = 0x21,
	DCP_EXPERT_ENDPOINT = 0x22,
	DISP0_ENDPOINT = 0x23,
	/*
	 * The DisplayPort control endpoints.  m1n1's working dcpext bring-up
	 * (proxyclient/experiments/dcpext_iboot.py) starts 0x24 and 0x27 and
	 * waits for their dcpav/dcpdp controller services *before* the port
	 * service on 0x2a appears.  Leaving them unstarted is why this driver
	 * saw a GETBUF on 0x2a and then silence for ever.
	 */
	DPTX_CONTROL_ENDPOINT = 0x24,
	DPAVSERV_ENDPOINT = 0x28,
	AV_ENDPOINT = 0x29,
	DPTX_ENDPOINT = 0x2a,
	HDCP_ENDPOINT = 0x2b,
	/*
	 * T8140/H17P puts the DisplayPort port service somewhere else.
	 *
	 * Traced on this machine's own macOS (build 25G83, the same DCP
	 * firmware Linux talks to): AppleDCPDPTXRemotePortProxy -- the object
	 * matching EPICName "dcpdptx-port-epic" -- hangs off DCPEXTEndpoint13,
	 * i.e. 0x2c, and its sibling HDCP interface off 14 = 0x2d.  Endpoint
	 * numbering is confirmed by DCPAVControllerProxy on 5 = 0x24, which is
	 * exactly where we independently saw the two AppleDCPDPTXController
	 * services announce from Linux.
	 *
	 * 0x2a is a real endpoint here and completes the whole AFK handshake
	 * (INIT_ACK, GETBUF, INIT_TX, INIT_RX, START_ACK) -- it simply never
	 * publishes anything, on macOS either.  Waiting on it is why the port
	 * never appeared.
	 */
	DPTX_PORT_ENDPOINT_H17P = 0x2c,
	/*
	 * Same trace puts the DPTX HDCP interface on 14 = 0x2d, which is the
	 * value upstream already uses for REMOTE_ALLOC_ENDPOINT -- and iomfb.c
	 * unconditionally starts that one.  Unused so far, and deliberately
	 * kept: it records that on H17P the endpoint we start as "remotealloc"
	 * is what macOS drives as HDCP.  Anything that grows a dispatch on 0x2d
	 * has to resolve that first.
	 */
	HDCP_ENDPOINT_H17P = 0x2d,
	REMOTE_ALLOC_ENDPOINT = 0x2d,
	IOMFB_ENDPOINT = 0x37,
};

/* Temporary backing for a chunked transfer via setDCPAVPropStart/Chunk/End */
struct dcp_chunks {
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
	/* live iommu mappings installed by dcpep_cb_map_piodma() */
	bool piodma_mapped;
};

/* Limit on call stack depth (arbitrary). Some nesting is required */
#define DCP_MAX_CALL_DEPTH 8

typedef void (*dcp_callback_t)(struct apple_dcp *, void *, void *);

struct dcp_channel {
	dcp_callback_t callbacks[DCP_MAX_CALL_DEPTH];
	void *cookies[DCP_MAX_CALL_DEPTH];
	void *output[DCP_MAX_CALL_DEPTH];
	u16 end[DCP_MAX_CALL_DEPTH];
	u64 link_message[DCP_MAX_CALL_DEPTH];
	u32 link_size[DCP_MAX_CALL_DEPTH];

	/* Tag of each outstanding call, so a stall can name itself */
	char tag[DCP_MAX_CALL_DEPTH][4];
	unsigned long pushed_at[DCP_MAX_CALL_DEPTH];

	/* Current depth of the call stack. Less than DCP_MAX_CALL_DEPTH */
	u8 depth;
	bool warned_busy;
};

struct dcp_fb_reference {
	struct list_head head;
	struct drm_framebuffer *fb;
	/*
	 * Id of the swap that unbinds @fb, i.e. the one that puts its
	 * replacement on screen.  Only meaningful once @armed is set: the
	 * firmware assigns the id in the swap_start reply, which happens after
	 * the atomic commit that displaced @fb has already queued this entry.
	 */
	u32 swap_id;
	bool armed;
};

#define MAX_NOTCH_HEIGHT 160

struct dcp_brightness {
	struct backlight_device *bl_dev;
	u32 maximum;
	u32 dac;
	int nits;
	int scale;
	bool update;
	/*
	 * H17P: the last non-zero level D006 reported, in millinits (D006
	 * reads 0 on every frame that does not change the level), and the
	 * level the last brightness-carrying swap sent (-1: none yet).
	 */
	u32 h17p_reported_millinits;
	int h17p_sent_nits;
	/*
	 * H17P: DPMS off leaves the pipe up (soft_off); the backlight stands
	 * in for the power-off by fading to blank_nits (blanked) and brings
	 * soft_off_nits -- the level asked for last -- back at DPMS on.
	 */
	bool soft_off;
	bool blanked;
	int soft_off_nits;
	/*
	 * H17P: the level is ramped toward the requested one over ramp_ms
	 * (dcp_backlight.c) instead of jumping there in one swap.  nits is
	 * the level in flight, target_nits where it is heading, ramp_from/
	 * ramp_start the anchor the interpolation runs from.
	 */
	int target_nits;
	int ramp_from;
	ktime_t ramp_start;
	struct delayed_work ramp_work;
	/* when the previous request arrived, and how long this fade runs */
	ktime_t last_request;
	unsigned int ramp_duration_ms;
	/* consecutive fade steps no compositor swap has carried */
	unsigned int idle_frames;
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

enum dcp_iomfb_method_profile {
	DCP_IOMFB_METHODS_DEFAULT,
	DCP_IOMFB_METHODS_H17G,
};

struct apple_dcp_hw_data {
	u32 num_dptx_ports;
	enum dcp_iomfb_method_profile iomfb_method_profile;
	bool adopt_running_coprocessor;
	/*
	 * Attach to an existing RTKit session instead of restarting it.  This
	 * uses explicit START_EP messages for the selected DCP services when
	 * the bootloader already consumed endpoint discovery.
	 */
	bool adopt_live_session;
	enum dcp_firmware_version firmware_compat;
	/* T8140's legacy DT omits the native H17 PMC window. */
	bool legacy_pmc_binding;
};

#define DCP_CALLBACK_TRACE_COUNT 16
struct dcp_callback_trace {
	char tag[4];
	u32 in_len;
	u32 out_len;
	u8 input[16];
};

/* TODO: move IOMFB members to its own struct */
struct apple_dcp {
	struct device *dev;
	struct platform_device *piodma;
	struct iommu_domain *iommu_dom;
	/* domain of the display-subsystem device, which owns the scanout stream */
	struct iommu_domain *disp_iommu_dom;
	/* the display-subsystem device, whose DMA domain the pipe scans out of */
	struct device *disp_dev;
	/* which of the nine cumulative A031 notify-client states to send next */
	unsigned int a031_step;
	struct apple_rtkit *rtk;
	struct apple_crtc *crtc;
	struct apple_connector *connector;

	struct apple_dcp_hw_data hw;
	spinlock_t callback_trace_lock;
	u32 callback_trace_count;
	struct dcp_callback_trace callback_trace[DCP_CALLBACK_TRACE_COUNT];

	/* firmware version and compatible firmware version */
	enum dcp_firmware_version fw_compat;

	/* Coprocessor control register */
	void __iomem *coproc_reg;

	/* DCP has crashed */
	bool crashed;
	/* Ignore inherited application-endpoint traffic while quiescing RTKit. */
	bool adopting_session;
	struct apple_dcp_surface_order iomfb_surfaces;


	/************* IOMFB **************************************************
	 * everything below is mostly used inside IOMFB but it could make     *
	 * sense to keep some of the members in apple_dcp.                    *
	 **********************************************************************/

	/* clock rate request by dcp in */
	struct clk *clk;
	struct clk *clk_194;

	/* DCP shared memory */
	void *shmem;
	dma_addr_t shmem_iova;
	u32 link_top[2][APPLE_DCP_LINK_STREAMS_PER_SIDE];
	u16 link_version;
	u32 link_firmware_hash;
	bool link_ready;

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
		struct dcp_swap_submit_req_h17p h17p;
	} swap;

	/* Atomic flushes waiting for the single-flight command channel. */
	struct list_head pending_flushes;
	/* Serializes the queue with command-channel dispatch. */
	struct mutex pending_flush_lock;
	/*
	 * H17P: the last swap that put a surface on the panel, as built by
	 * iomfb_flush() before the wire fix-up.  Re-presenting it is how a
	 * brightness change reaches the firmware while the CRTC is off (see
	 * iomfb_present_brightness()).
	 */
	struct dcp_swap_submit_req_h17p h17p_last_swap;
	struct dcp_swap_submit_req_h17p h17p_pending_swap;
	bool h17p_have_last_swap;

	/* swap id of the last completed swap */
	u32 last_swap_id;
	/* last_swap_id is only meaningful after the first swap completes */
	bool have_swap_complete;
	/*
	 * H17P: GP alpha divide (runtime property 0x49) has been turned off
	 * since the last full power-on; X formats need it off (see
	 * dcp_h17p_fix_swap_surfaces()).
	 */
	bool h17p_alpha_div_off;
	bool h17p_swapped_since_on;
	/* result handler and cookie of the call issued ahead of the next swap */
	dcp_callback_t diag_res;
	void *diag_cookie;
	ktime_t swap_start;
	u64 swap_submit_timestamp;

	/* Current display mode */
	bool during_modeset;
	/* hotplug reported while a modeset was in flight, applied afterwards */
	bool pending_hotplug;
	bool pending_hotplug_connected;
	bool valid_mode;
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
	struct dcp_display_mode *modes;
	unsigned int nr_modes;

	/* Attributes of the connector */
	int connector_type;
	int fixed_connector_type;

	/* Attributes of the connected display */
	int width_mm, height_mm;

	unsigned notch_height;

	/* Workqueue for sending vblank events when a dcp swap is not possible */
	struct delayed_work vblank_wq;

	/* Names a command the firmware has stopped answering */
	struct delayed_work stall_wq;
	bool warned_stall;
	bool dumped_crashlog;
	/* the DCP crashlog region, so a stall can read the fresh record */
	void *crashlog_buf;
	size_t crashlog_size;

	/* List of referenced drm_framebuffers which can be unreferenced
	 * on the next successfully completed swap.
	 */
	struct list_head swapped_out_fbs;
	/*
	 * swapped_out_fbs is appended to from the DRM atomic commit thread
	 * (dcp_flush -> .atomic_flush) and armed/drained from the RTKit
	 * workqueue that runs the DCP callbacks.  Two different threads.
	 */
	spinlock_t swapped_out_lock;

	struct dcp_brightness brightness;
	/* Workqueue for updating the initial brightness */
	struct work_struct bl_register_wq;
	struct mutex bl_register_mutex;
	/* Workqueue for updating the brightness */
	struct delayed_work bl_update_wq;

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

	/* DisplayPort controller endpoint; see DPTX_CONTROL_ENDPOINT above */
	struct apple_dcp_afkep *dptxcontrolep;
	/* the two AppleDCPDPTXController services it announces, units 1 and 3 */
	struct apple_epic_service *dptxctl_av;
	struct apple_epic_service *dptxctl_dp;

	struct dptx_port dptxport[2];

	/* debugfs entries */
	struct dentry *ep_debugfs[0x20];

	/* these fields are output port specific */
	struct phy *phy;
	struct phy *fixed_phy;
	struct mux_control *xbar;
	bool xbar_selected;
	struct typec_mux *typec_mux;
	struct apple_dcp_typec_route typec_routes[DCP_MAX_TYPEC_ROUTES];
	struct apple_dcp_typec_route *active_typec_route;
	u32 nr_typec_routes;
	bool phy_managed_by_typec;
	bool typec_cable_connected;
	/* CRTC powered off while the Type-C cable stays attached */
	bool typec_crtc_off;
	/* Keep the unplug shutdown owner until the next CRTC enable. */
	bool typec_unplugged;
	struct delayed_work typec_reconnect_wq;
	struct delayed_work typec_fabric_retrain_wq;
	u32 typec_reconnect_tries;

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
};

void dcp_drm_crtc_page_flip(struct apple_dcp *dcp, ktime_t now);

int dcp_backlight_register(struct apple_dcp *dcp);
int dcp_backlight_update(struct apple_dcp *dcp);
void dcp_backlight_soft_off(struct apple_dcp *dcp);
void dcp_backlight_soft_on(struct apple_dcp *dcp);
u64 dcp_nits_to_double_bits(u32 nits);
extern bool dcp_swap_trace;
extern unsigned int dcp_h17p_surf_flags;
extern int dcp_blend_crc_reads;
extern char dcp_h17p_swap_poke[128];
extern char dcp_rtp_request[32];
extern int dcp_dither_request;
extern int dcp_pipe_crc_reads;
extern unsigned int dcp_pipe_crc_idx;
u32 dcp_h17p_max_nits(struct apple_dcp *dcp);
u32 dcp_h17p_initial_nits(struct apple_dcp *dcp);
int dcp_h17p_present_brightness(struct apple_dcp *dcp);
bool dcp_has_panel(struct apple_dcp *dcp);

#define DCP_AUDIO_MAX_CHANS 15

void dcp_dump_crashlog(struct apple_dcp *dcp);

#endif /* __APPLE_DCP_INTERNAL_H__ */
