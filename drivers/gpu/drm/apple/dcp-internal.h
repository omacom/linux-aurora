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

#include "dcp-fabric.h"
#include "dcp-fabric-session.h"
#include "dcp-relink.h"
#include "dptxep.h"
#include "dcp_backlight.h"
#include "dcp-display-metadata.h"
#include "dcp-metadata-epoch.h"
#include "dcp-180hz.h"
#include "iomfb.h"
#include "iomfb_h17p.h"
#include "iomfb-state.h"
#include "iomfb_v12_3.h"
#include "iomfb_v13_3.h"
#include "iomfb_v14_7.h"
#include "iomfb_v14_7_tpl.h"
#include "iomfb_v26_6.h"
#include "epic/dpavservep.h"

#define DCP_MAX_PLANES 4

struct apple_dcp_afkep;

/* Snapshot of a completed present, independent of the latest DRM state. */
struct iomfb_scanout_h17p {
	struct dcp_swap_submit_req_h17p request;
	struct drm_framebuffer *fb[SWAP_SURFACES];
};

/* One operation, including all of its nested replies and present completion. */
struct iomfb_transaction {
	struct list_head link;
	void (*start)(struct apple_dcp *dcp, struct iomfb_transaction *transaction);
	void (*release)(struct iomfb_transaction *transaction);
	struct dcp_backlight_present backlight;
	bool backlight_reserved;
	bool backlight_failed;
	bool brightness_only;
	bool presentation;
	bool completed;
};
struct apple_dcp_typec_port;

void dcp_swap_watchdog_arm(struct apple_dcp *dcp);
void dcp_swap_watchdog_complete(struct apple_dcp *dcp);
void dcp_retry_placeholder_edid(struct apple_dcp *dcp,
				const struct drm_edid *drm_edid);
bool dcp_typec_relink(struct apple_dcp *dcp);
bool dcp_typec_sink_relink(struct apple_dcp *dcp);

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
	DCP_FIRMWARE_H17P,
};

enum {
	SYSTEM_ENDPOINT = 0x20,
	TEST_ENDPOINT = 0x21,
	DCP_EXPERT_ENDPOINT = 0x22,
	DISP0_ENDPOINT = 0x23,
	/* DPAV controller: publishes the port service used by DPTX 0x2a. */
	DPAV_CTRL_ENDPOINT = 0x24,
	DPTX_CONTROLLER_ENDPOINT = 0x24,
	DPAVSERV_ENDPOINT = 0x28,
	DPAVSERV_ENDPOINT_V2 = 0x2a,
	AV_ENDPOINT = 0x29,
	DPTX_ENDPOINT = 0x2a,
	DPTX_ENDPOINT_V2 = 0x2c,
	HDCP_ENDPOINT = 0x2b,
	REMOTE_ALLOC_ENDPOINT = 0x2d,
	IOMFB_ENDPOINT = 0x37,
};

/* Temporary backing for a chunked transfer via setDCPAVPropStart/Chunk/End */
struct dcp_chunks {
	u64 modes_generation;
	size_t length;
	void *data;
	size_t received;
	bool contiguous;
	bool payload_complete;
	struct dcp_metadata_ticket metadata_ticket;
	u64 av_service_epoch; /* Captured at property start, never at publication. */
	struct dcp_metadata_ticket mode_ticket;
	u64 automatic_connection_sequence;
	bool automatic_service_pending;
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
	/* Header of each pending AP command record, as dcp_push() wrote it */
	struct dcp_packet_header header[DCP_MAX_CALL_DEPTH];
	u16 end[DCP_MAX_CALL_DEPTH];

	/* Current depth of the call stack. Less than DCP_MAX_CALL_DEPTH */
	u8 depth;
	/* Already warned about busy channel */
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
	DCP_IOMFB_METHODS_H17P_EXTERNAL,
};

struct apple_dcp_hw_data {
	u32 num_dptx_ports;
	bool t6020_tunnel_flow;
	enum dcp_iomfb_method_profile iomfb_method_profile;
	/*
	 * The bootloader leaves the coprocessor running.  Attach to it with a
	 * standard RTKit wake instead of starting or restarting the ASC.
	 */
	bool adopt_live_session;
	enum dcp_firmware_version firmware_compat;
	/* Firmware-visible clock aperture, independent of the AP PMGR window. */
	const struct resource *firmware_clock;
	u32 firmware_display_windows;
	u32 firmware_scratch;
	u32 firmware_request;
};

/* TODO: move IOMFB members to its own struct */
struct dcpext_scanout;
struct dcp_hdmi;

struct apple_dcp {
	struct dcp_fabric_pipeline fabric;
	/* An adopted session has no confirmed firmware DMA stop boundary. */
	bool retain_dma;
	bool quiescing;
	bool drm_retained;
	struct device *dev;
	struct platform_device *piodma;
	bool piodma_created;
	struct iommu_domain *iommu_dom;
	/* which of the nine cumulative A031 notify-client states to send next */
	unsigned int a031_step;
	struct apple_rtkit *rtk;
	struct apple_crtc *crtc;
	struct apple_connector *connector;

	struct apple_dcp_hw_data hw;
	bool external_prepared;
	/* Serializes external connection, power, and CRTC generation changes. */
	struct mutex external_lock;
	u64 external_generation;
	u64 external_crtc_generation;
	/* iomfb.lock: epoch used by receive-side deferred stop requests. */
	u64 external_rx_generation;
	u64 external_stop_generation;
	bool external_cable;
	/* RX tombstone: initially unknown; only physical OOB events change it. */
	bool external_physically_absent;
	bool external_running;
	bool external_power_attempted;
	/* iomfb.lock: replay the latest callback after activation succeeds. */
	bool external_hotplug_pending;
	bool external_hotplug_connected;
	bool external_stopped;
	int external_error;
	struct work_struct external_stop_work;


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
	/*
	 * Native: the DP/HDMI audio endpoint (EP 0x29) starts once per
	 * session, after the first display's EDID; see dcp_external_audio_start().
	 */
	struct work_struct external_audio_work;
	bool external_audio_started;

	/* DCP_FIRMWARE_V_14_7 state; outlives this device once RTKit runs. */
	struct apple_dcp_v14 *v14;
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
	struct clk *clk_194;

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
	struct dcp_180hz_proof rate_proof;
	struct delayed_work rate_retry_wq;
	bool rate_retry_initialized;
	struct {
		/* Protects connection tickets and cached metadata snapshots. */
		spinlock_t lock;
		struct dcp_metadata_epoch epoch;
		struct dcp_display_metadata attributes;
		struct drm_display_mode preferred;
		struct dcp_metadata_ticket attributes_ticket, timing_ticket;
		bool attributes_valid, timing_valid, hpd_seen;
	} metadata;

	/* Queued swap. Owned by the DCP to avoid per-swap memory allocation */
	union {
		struct dcp_swap_submit_req_v12_3 v12_3;
		struct dcp_swap_submit_req_v13_3 v13_3;
		struct dcp_swap_submit_req_v14_7_0 v14_7_0;
		struct dcp_swap_submit_req_h17p h17p;
	} swap;

	/* swap id of the last completed swap */
	u32 last_swap_id;
	/* last_swap_id is only meaningful after the first swap completes */
	bool have_swap_complete;
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
	/* replugs replayed for a Type-C sink that lost its link */
	spinlock_t relink_lock;
	struct dcp_relink_state relink;
	struct delayed_work typec_relink_wq;
	/* the driver holds the sink's HPD low itself; hpd_mutex */
	bool typec_hpd_held;
	bool use_timestamps;
	bool vrr_enabled;
	struct dcp_set_digital_out_mode_req mode;

	/* completion for active turning true */
	struct completion start_done;

	/* Is the DCP booted? */
	bool active;
	/* A successful initial enable keeps the H17P pipe running across DPMS. */
	bool pipe_enabled_h17p;
	/* Messages from the coprocessor are held in the mailbox for sleep. */
	bool rx_held;

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

	/* List of referenced framebuffers pending a completed replacement swap.
	 * The commit and RTKit work queues share it under this lock.
	 */
	struct list_head swapped_out_fbs;
	/* Protects swapped_out_fbs. */
	struct mutex swapped_out_fbs_lock;

	struct dcp_brightness brightness;
	/* Workqueue for updating the initial brightness */
	struct work_struct bl_register_wq;
	struct mutex bl_register_mutex;
	/* Workqueue for updating the brightness */
	struct work_struct bl_update_wq;
	/* Registers the H17P backlight when the loader reports no level. */
	struct delayed_work bl_fallback_wq;

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
	struct apple_dcp_afkep *dptx_controller_ep;
	struct completion dptx_controller_ready;

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
	/* dcp_poweron() reconnects DPTX; its own modeset follows the HPD */
	bool typec_poweron_hpd;
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

	/* H17P policy shared by backlight, commit, and reply workers. */
	struct {
		/* Protects state and callback installation. */
		spinlock_t lock;
		struct dcp_backlight_state state;
		void (*kick)(struct apple_dcp *dcp);
		/* Steps a running fade; fade_start is its start in jiffies. */
		struct delayed_work fade_work;
		unsigned long fade_start;
		/* Rate limit for firmware re-send requests. */
		unsigned long resent_at;
		bool resent;
	} backlight;

	/* Staging wire record; serialization never mutates the atomic inputs. */
	struct dcp_present_h17p present_h17p;
	struct dcp_present_state_h17p present_state_h17p;
	DECLARE_BITMAP(unknown_callbacks, IOMFB_MAX_CB);

	/* Serializes H17P transmit preparation with RTKit receive callbacks. */
	struct {
		/* Protects queue, channel stacks, and the current owner. */
		struct mutex lock;
		struct list_head pending;
		struct iomfb_transaction *active;
		struct task_struct *owner;
		struct work_struct work;
		struct delayed_work timeout;
		struct delayed_work backlight_retry;
		unsigned long deadline;
		unsigned int queued;
		bool stopped;
		bool presentation_paused;
		bool backlight_queued;
		atomic_t opaque_x_state;
		struct iomfb_scanout_h17p *scanout;
		struct iomfb_scanout_h17p *next_scanout;
	} iomfb;
};

void dcp_firmware_metadata_connection(struct apple_dcp *dcp, u64 generation, bool connected);
void dcp_firmware_metadata_hotplug(struct apple_dcp *dcp, bool connected);
void dcp_firmware_metadata_begin(struct apple_dcp *dcp);
void dcp_firmware_metadata_attributes(struct apple_dcp *dcp,
				      const struct dcp_display_metadata *attributes);
void dcp_firmware_metadata_timing(struct apple_dcp *dcp);
bool dcp_firmware_metadata_current(struct apple_dcp *dcp, u64 generation);
const struct drm_edid *dcp_firmware_edid(struct apple_dcp *dcp, u64 *generation);

void iomfb_scanout_complete_h17p(struct apple_dcp *dcp);
void iomfb_present_backlight_h17p(struct apple_dcp *dcp);
bool iomfb_present_brightness_only_h17p(struct apple_dcp *dcp);
bool iomfb_present_complete_h17p(struct apple_dcp *dcp);
void iomfb_present_failed_h17p(struct apple_dcp *dcp);
bool iomfb_apply_backlight_h17p(struct apple_dcp *dcp,
				const struct dcp_swap_submit_req_h17p *request,
				struct dcp_present_h17p *wire);
void iomfb_apply_opaque_x_h17p(struct apple_dcp *dcp, dcp_callback_t callback,
			       void *cookie);
void iomfb_opaque_x_reset_h17p(struct apple_dcp *dcp);
int iomfb_configure_backlight_h17p(struct apple_dcp *dcp, u32 maximum,
				   bool inherited_valid, u32 inherited,
				   bool default_valid, u32 default_nits);
void iomfb_queue_init(struct apple_dcp *dcp);
void iomfb_queue_stop(struct apple_dcp *dcp);
bool iomfb_queue_drain(struct apple_dcp *dcp, unsigned long timeout);
int iomfb_queue_settle(struct apple_dcp *dcp, unsigned long timeout);
void iomfb_pause_presentation(struct apple_dcp *dcp, bool pause);
int iomfb_external_power_h17p(struct apple_dcp *dcp, bool on);
int iomfb_queue(struct apple_dcp *dcp, struct iomfb_transaction *transaction);
void iomfb_queue_crc_h17p(struct apple_dcp *dcp, u32 swap_id);

void dcp_drm_crtc_page_flip(struct apple_dcp *dcp, ktime_t now);
void dcp_handle_hotplug_actions(struct apple_dcp *dcp, unsigned int action);
bool dcp_external_hotplug(struct apple_dcp *dcp, bool connected);

int dcp_backlight_init(struct apple_dcp *dcp, struct device *dev);
void dcp_backlight_stop(struct apple_dcp *dcp);
void dcp_backlight_finish_fade(struct apple_dcp *dcp);
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
 * The T6030 external processor (the J514S/J516S built-in HDMI port and
 * USB-C displays) driven natively through Aurora's DCPLink session: DCP
 * firmware 14.7's external role, whose DP/HDMI audio service differs from
 * the M1/M2 one (see av.c). The manual diagnostic pipe (dcpext_scanout) and
 * the T8122 template path are not this.
 */
static inline bool dcp_is_v14_7_external(const struct apple_dcp *dcp)
{
	return dcp->fw_compat == DCP_FIRMWARE_V_14_7 && dcp->external_native;
}

/* The 14.7 internal panel (J514S/J516S), presented through DCPLink. */
static inline bool dcp_is_v14_7_panel(const struct apple_dcp *dcp)
{
	return dcp->fw_compat == DCP_FIRMWARE_V_14_7 && !dcp->external;
}

/* Native external pipe: start its audio endpoint, once, after the first EDID. */
void dcp_external_audio_start(struct apple_dcp *dcp);

/*
 * The T8122 dcpext connects its DPTX port as AsahiLinux/linux#607 does,
 * the kernel this firmware was first driven with: no validation of the
 * target, a connect without the supports-HPD attribute, GET_SUPPORTS_HPD
 * answered with 0, HPD asserted only after the link-configuration wait,
 * and tiled display hints acknowledged with 0. Every other DCP keeps the
 * Type-C model of dcp_dptx_connect_session().
 */
static inline bool dcp_ext_dptx_607(const struct apple_dcp *dcp)
{
	return dcp->fw_compat == DCP_FIRMWARE_V_14_7_TEMPLATE;
}

/* The T8140 external processor (dispext0) running H17P firmware. */
static inline bool dcp_is_external_h17p(const struct apple_dcp *dcp)
{
	return dcp->hw.iomfb_method_profile == DCP_IOMFB_METHODS_H17P_EXTERNAL;
}

#define DCP_AUDIO_MAX_CHANS 15

#endif /* __APPLE_DCP_INTERNAL_H__ */
