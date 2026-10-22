// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Copyright 2021 Alyssa Rosenzweig */

#ifndef __APPLE_DCP_INTERNAL_H__
#define __APPLE_DCP_INTERNAL_H__

#include <drm/drm_mode.h>
#include <linux/atomic.h>
#include <linux/backlight.h>
#include <linux/device.h>
#include <linux/ioport.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/spinlock.h>
#include <linux/mux/consumer.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/usb/typec_mux.h>

#include "dcp-fabric.h"
#include "dcp-peer-v26_6.h"
#include "dcp-fabric-session.h"
#include "dcp-relink.h"
#include "dptxep.h"
#include "dcp_backlight.h"
#include "dcp-announce.h"
#include "dcp-display-metadata.h"
#include "dcp-metadata-epoch.h"
#include "dcp-panel-edid.h"
#include "dcp-180hz.h"
#include "iomfb.h"
#include "iomfb_h17p.h"
#include "iomfb_v27.h"
#include "iomfb-state.h"
#include "iomfb_v12_3.h"
#include "iomfb_v13_3.h"
#include "iomfb_v14_7.h"
#include "iomfb_v14_7_tpl.h"
#include "iomfb_v26_6.h"
#include "iomfb_v26_6_desktop.h"
#include "dcp-peer-frames-v27.h"
#include "epic/dpavservep.h"
#include "version_utils.h"

#define DCP_MAX_PLANES 4
/* Bounds the CRTC power cycles a rebuilt T6020 tunnel may trigger. */
#define DCP_TUNNEL_POWER_CYCLES 2

struct apple_dcp;
struct iomfb_m4;
struct ext26_control;
struct apple_dcp_afkep;
struct dcp_method_entry;

/* Snapshot of a completed present, independent of the latest DRM state. */
struct iomfb_scanout_h17p {
	struct dcp_swap_submit_req_h17p request;
	struct drm_framebuffer *fb[SWAP_SURFACES];
	/* On iomfb.dropped while the firmware may still read a dropped present. */
	struct list_head link;
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
bool dcp_swap_lost(struct apple_dcp *dcp, u32 swap_id);
void dcp_retry_placeholder_edid(struct apple_dcp *dcp,
				const struct drm_edid *drm_edid);
bool dcp_typec_relink(struct apple_dcp *dcp);
bool dcp_typec_sink_relink(struct apple_dcp *dcp);

struct dcpav_service_epic;

enum {
	SYSTEM_ENDPOINT = 0x20,
	TEST_ENDPOINT = 0x21,
	DCP_EXPERT_ENDPOINT = 0x22,
	DISP0_ENDPOINT = 0x23,
	DPTX_CONTROLLER_ENDPOINT = 0x24,
	/*
	 * The same endpoint on T6030 14.7 firmware: the DPAV controller that
	 * publishes the port service used by DPTX 0x2a.
	 */
	DPAV_CTRL_ENDPOINT = 0x24,
	/* Desktop 26.x (T6041) auxiliary endpoints, see dcp_start_desktop(). */
	EPIC25_ENDPOINT = 0x25,
	DPSAC_ENDPOINT = 0x26,
	DPDEV_ENDPOINT = 0x27,
	EPIC2C_ENDPOINT = 0x2c,
	DPAVSERV_ENDPOINT = 0x28,
	DPAVSERV_ENDPOINT_V2 = 0x2a,
	AV_ENDPOINT = 0x29,
	/* Desktop 26.x separates the audio service from AV device control. */
	AV_AUDIO_ENDPOINT_DESKTOP = 0x2b,
	DPTX_ENDPOINT = 0x2a,
	DPTX_ENDPOINT_V2 = 0x2c,
	/*
	 * Compact desktop/M4 firmware: the panel processor advertises
	 * dcpav-video-interface-epic on 0x2b, the external ones the audio
	 * service (AV_AUDIO_ENDPOINT_DESKTOP).  Nothing HDCP lives here.
	 */
	AV_VIDEO_ENDPOINT = 0x2b,
	/*
	 * 0x2d is the remote allocator for the IOMFB firmware (H17P, 27.0
	 * internal), which must be started before IOMFB.  Compact desktop/M4
	 * firmware advertises dcpdptx-hdcp-interface on it instead, one per
	 * processor (disp0, dispext0, dispext1).  Bind HDCP by service name.
	 */
	REMOTE_ALLOC_ENDPOINT = 0x2d,
	DPTX_HDCP_ENDPOINT = 0x2d,
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
	/* A level was written through the backlight device (desktop 26.x). */
	bool requested;
	/*
	 * Fade between levels where the firmware takes a DAC value in each
	 * swap.  level is the one set last, in nits; crtc mutex.
	 */
	struct delayed_work fade_work;
	unsigned long fade_start;
	u32 level;
	u32 fade_from;
	u32 fade_to;
	bool fading;
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
	/// peak panel luminance in nits, for EDR (desktop 26.x; 0 if unknown)
	u32 peak_nits;
	/// EDR headroom ceiling x1000 from the ADT (desktop 26.x; 0 if none)
	u32 edr_ceiling;
	/// calibrated colour target from the raw ADT; NULL if not given
	const struct dcp_color_space *color_target;
	/// make and product for the base EDID, from the panel's compatible
	struct dcp_panel_id id;
};

/*
 * The internal display's power words for hw.s2r, see dcp_s2r_domains_off(): its domains'
 * offsets in "ps", its retention controls in "pwrgate" (the PMGR's per-device "retention"
 * words: flip-flops 3 off / 0 on, memories 0 off / 1 on), and the DART streams its firmware
 * and scanout use.
 */
struct dcp_s2r_layout {
	u32 disp_cpu;
	u32 disp_fe;
	u32 disp_sys;
	u32 ret_ff;
	u32 ret_mem;
	u32 dcp_streams;
	u32 disp_streams;
	/* "pmc-dcp" counts the powered DCPs: T8132 and T8140 only */
	bool pmc_dcp;
};

/*
 * Identity of a fixed internal panel, for the EDID published on its
 * connector.  Any processor with an internal panel allocates it; the boot
 * loader's ADT supplies the fallback, and firmware metadata or a raw panel
 * EDID replace that when the firmware reports one.  metadata.lock.
 */
struct dcp_panel_identity {
	/* Identity supplied by this fixed panel, independent of optional color data. */
	struct dcp_display_metadata metadata;
	struct dcp_display_metadata adt;
	u64 generation;
	bool metadata_valid;
	u8 raw_edid[4096];
	size_t raw_edid_size;
};

enum dcp_iomfb_method_profile {
	DCP_IOMFB_METHODS_DEFAULT,
	DCP_IOMFB_METHODS_H17G,
	DCP_IOMFB_METHODS_H17P_EXTERNAL,
	DCP_IOMFB_METHODS_J613_EXT26,
};

struct apple_dcp_hw_data {
	u8 firmware_dma_bits;
	u32 num_dptx_ports;
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
	/* M2 Pro/Max laptop T6020 Thunderbolt flow, from the match data */
	bool t6020_tunnel_flow;
	/*
	 * A processor Linux starts itself on 14.7 firmware through the IOMFB
	 * template whose AFK, DPTX and DPAV endpoints speak the 14.7 protocol
	 * (the T8122 dcpext), see dcp_fw_is_14_7().
	 */
	bool fw_14_7_endpoints;
	/*
	 * An external processor with no DCPAV service to read the EDID
	 * through, whose sinks' EDID is built from their firmware metadata
	 * (desktop 26.x), see dcp_sink_metadata().
	 */
	bool sink_metadata_edid;
	/*
	 * One info line per connection of an external display with its link, sink and mode,
	 * see dcp_link_report(). With it come a search of every firmware log line and the
	 * EDID's CRC at get_modes; without it none of the three is done. Cleared at probe on
	 * every board but the one it has run on (dcp.c, dcp_j714s_ext()).
	 */
	bool link_report;
	/*
	 * The endpoint the DPTX port services are announced on, where it is not
	 * the firmware's default (DPTX_ENDPOINT_V2 on firmware 27): the T8142
	 * dcpext announces them on DPTX_ENDPOINT. 0: the default.
	 */
	u8 dptx_endpoint;
	/*
	 * Open the DPTX controller endpoint before the ports, as for H17P
	 * externals: the T8142 dcpext announces AppleDCPDPTXController there.
	 */
	bool dptx_controller;
	/*
	 * With H17P firmware: a firmware-driven system sleep stops the internal processor's
	 * firmware and powers it down, as DPMS off does, then restarts it warm, see
	 * dcp_s2r_suspend(). @s2r_ext: the same for an external one, see dcp_s2r_ext_suspend().
	 */
	bool s2r;
	bool s2r_ext;
	/* The firmware crashes on any mode but the 120 Hz boot timing, see dcp_mode_settable(). */
	bool only_120hz_modes;
	/* A firmware 27 sleep in progress, and how it ended */
	bool v27_sleep_protocol;
	bool v27_sleep_recovery;
	/* Required with @s2r */
	const struct dcp_s2r_layout *s2r_layout;
	/* @s2r with derived power words: off unless appledrm.s2r_untested is set */
	bool s2r_untested;
	/*
	 * @s2r without a secure mailbox: the SoC's ADT has no dcp-exclave-mailbox (T8142, on
	 * firmware 27.0 and 27.2.1), so there is no secure port to idle before the IOP.
	 */
	bool s2r_no_secure_port;
};

enum dcp_test_point {
	DCP_TEST_ANNOUNCE_TIMER_DONE,		/* past hpd_mutex in the timeout work */
	DCP_TEST_UNBIND_ANNOUNCE_STOPPED,	/* unbind drained the timeout work */
	DCP_TEST_HOTPLUG_EVENT,			/* the hotplug worker sends its event */
};

/*
 * DCP_FIRMWARE_V_26_6_DESKTOP state, see iomfb_v26_6_desktop.c; allocated
 * for such a DCP only.
 */
struct dcp_desktop_state {
	/* Protected by backlight.lock, independent of framebuffer retirement. */
	struct dcp_backlight_swap backlight;
	bool inherited_brightness_valid;
	u32 inherited_brightness;

	/* Queued swap; too large for the shared swap union. */
	struct dcp_swap_submit_req_v26_6_desktop swap;
	/* Opaque client identity the firmware's IOMFB calls carry. */
	u64 client;
	/*
	 * A mode was set since the display last went away.  Until then the
	 * firmware's generic pipe is bootstrapped through a virtual color
	 * element before the real one is selected.
	 */
	bool mode_live;
	atomic_t identity_log_count[4];
	/* RX records an unsupported callback until the modeset RPC has drained. */
	bool modeset_active;
	bool modeset_callback_error;
	/* An adaptive swap that timed out keeps fixed refresh until the next boot. */
	bool vrr_failed;
	/*
	 * Adaptive sync of a VRR timing installed by this pipe alone: the
	 * minimum rates for VRR_ENABLED on and off (16.16 Hz), and the one
	 * the firmware was last given.  Zero when not installed that way.
	 */
	u32 vrr_min_on, vrr_min_off, vrr_min_sent;
	/* The frame's colour matrix, sent after an adaptive sync update. */
	struct iomfb_set_matrix_req matrix;
	/*
	 * HDR10 static metadata (external processors): the pinned
	 * dcpav-service-epic service, the Type 1 body last sent on this link
	 * (taken or refused), if any, and whether the firmware took one, so
	 * that the link carries a Dynamic Range and Mastering InfoFrame.
	 * hdr_lock protects all of them.
	 */
	spinlock_t hdr_lock;
	struct apple_epic_service *av_service;
	u8 hdr_sent[24];
	bool hdr_sent_valid;
	bool hdr_infoframe;
	u32 present_id;
	bool present_pending;
	/*
	 * GAMMA_LUT: the firmware's own table, read before the first change;
	 * the table being sent; the LUT it is composed from, if any.
	 */
	struct iomfb_gamma_table_req *gamma_fw, *gamma_out;
	struct drm_color_lut gamma_lut[IOMFB_GAMMA_ENTRIES];
	bool gamma_fw_read, gamma_user, gamma_pending, gamma_unusable;
	/*
	 * Endpoints 0x20..0x3f this processor's start brought up beside the
	 * system endpoint (bit = endpoint - 0x20), and those it skipped.
	 */
	struct apple_dcp_afkep *graph[0x20];
	u32 graph_started, graph_skipped;
	/* DPAVCTRL (0x24): its two AV controllers and the deferred service. */
	struct apple_epic_service *dpavctrl_av_controller[2];
	struct apple_epic_service *dpavctrl_late_service;
	struct completion dpavctrl_ready;
	/*
	 * Takeover trial: power-on and a mode set ran before the boot
	 * framebuffer was evicted; the first atomic enable reuses them.
	 */
	bool trial_powered, trial_modeset_ok;
	/* The takeover kept the loader's mode without setting it. */
	bool takeover_adopted;
	u32 trial_timing_mode_id, trial_color_mode_id;
	/* Result of the last power-on: 0 ok, 1 timeout, 2 failed. */
	u8 poweron_result;
	/* The takeover was refused: leave the DCP alone until the reboot. */
	bool takeover_aborted;
	/*
	 * External processors: the firmware powered the pipe off with the
	 * display (unplug); the next swap needs a power-on and a mode set.
	 */
	bool ext_pipe_off;
};

/* TODO: move IOMFB members to its own struct */
struct dcpext_scanout;
struct dcp_hdmi;

bool dcp_desktop_present_complete(struct apple_dcp *dcp, u32 id);
void dcp_panel_metadata_replace(struct apple_dcp *dcp,
				const struct dcp_display_metadata *metadata);
const struct drm_edid *dcp_panel_edid_snapshot(struct apple_dcp *dcp,
					       const struct dcp_display_mode *modes,
					       unsigned int count, u64 *generation);
bool dcp_panel_metadata_current(struct apple_dcp *dcp, u64 generation);

void dcp_s2r_dart_power(struct apple_dcp *dcp, u32 on);

struct apple_dcp {
	/* Generation-specific wire/transaction state, sharing the common KMS owner. */
	struct iomfb_m4 *v26_6;
	/* An adopted session has no confirmed firmware DMA stop boundary. */
	bool retain_dma;
	/*
	 * Allocates AFK rings where the DMA API cannot, on an adopted
	 * J713 external DCP whose DART only takes the retained allocator.
	 */
	void *(*afk_alloc)(struct apple_dcp *dcp, size_t size, dma_addr_t *dva);
	bool quiescing;
	bool drm_retained;
	/* vblank_wq and swap_watchdog_wq disabled by the last unbind */
	bool vblank_works_disabled;
	struct dcp_fabric_pipeline fabric;
	struct device *dev;
	/* Peer DCP for dual-pipe (desktop 26.6): the primary reserves an idle
	 * peer pipe; the peer records its owner.  Protected by dcp_peer_lock.
	 */
	struct apple_dcp *peer_reserved;
	struct apple_dcp *peer_owner;
	/*
	 * This pipeline's Type-C display is wider than one pipe and found no
	 * peer above it: it needs the hybrid, which was taken when it came.
	 */
	bool peer_starved;
	/*
	 * Desktop 26.x hybrid with apple,typec-hybrid-first: its HDMI display
	 * has given the pipeline to a Type-C display wider than one pipe and
	 * waits until that one fits one pipe again or leaves.
	 */
	bool hdmi_yielded;
	/* The display on this pipeline runs a mode within one pipe. */
	bool typec_narrow;
	struct delayed_work hybrid_wq;
	struct dcp_peer_tiling peer_tiling;
	bool peer_unbinding;
	/*
	 * T6050: the platform limit left this display no pair when last
	 * evaluated (dcp_peer_hints_v27(), dcp_peer_limit_update_v27()). Under
	 * dcp_peer_lock.
	 */
	bool peer_limited;
	/* Firmware display endpoint (its Disp_Index), apple,dcp-endpoint;
	 * separate from index, which also selects the doorbell and PHY mode. */
	u32 peer_endpoint;
	/* Paired DRM scanout on the primary; under dcp_peer_lock. */
	bool peer_active, peer_failed;
	/* The DRM mode the pair was set to (timing, color mode IDs, minRR). */
	u32 peer_mode_timing, peer_mode_color, peer_min_vrr;
	/* On the peer: its first completion since the pair was set was logged. */
	bool peer_complete_logged;
	struct platform_device *piodma;
	bool piodma_created;
	struct iommu_domain *iommu_dom;
	/* which of the nine cumulative A031 notify-client states to send next */
	unsigned int a031_step;
	struct apple_rtkit *rtk;
	struct apple_crtc *crtc;
	struct apple_connector *connector;
	/* Protected by dcp_peer_lock; never take the physical route lock in
	 * an AFK callback (a route operation may be waiting for that reply).
	 */
	struct dcp_peer_frames_v27 peer_frames;

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
	/* The last stop sent nothing and stays owned; see external_stop_retry_wq. */
	bool external_stop_deferred;
	unsigned int external_stop_retries;
	int external_error;
	struct work_struct external_stop_work;
	struct delayed_work external_stop_retry_wq;


	/* firmware version and compatible firmware version */
	enum dcp_firmware_version fw_compat;
	/* Bumped by every iomfb poweron; see afk_compact_slot(). */
	u32 afk_power_gen;
	/* Firmware 27.x: the method IDs of this build (iomfb_v27_methods()). */
	const struct dcp_method_entry *v27_methods;
	bool external;
	/* A desktop 26.x external processor (apple,t6041-dcpext) */
	bool desktop_external;
	/*
	 * A failsafe dcpext (dcp_ext_is_failsafe(): T8122's, T6041's, T6050's):
	 * the DRM device of the other displays never waits on this processor.
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
	/* Native26 control/property epochs, serialized by the IOMFB owner. */
	struct completion ext26_link_ready;
	bool ext26_services_open;
	bool ext26_client_open;
	struct apple_dcp_afkep *ext26_aux[3];
	u32 ext26_stride;
	u64 ext26_edid_generation;
	bool ext26_edid_tried;
	u64 ext26_rx_ns;
	u64 ext26_power_ns;
	u64 ext26_generation;
	u64 ext26_buffer_bytes;
	bool ext26_blocked;
	bool ext26_pm_work_off;
	bool ext26_stopping;
	bool ext26_blanked;
	bool ext26_pm_preparing;
	bool ext26_control_pending;
	bool ext26_early_complete;
	u32 ext26_early_id;
	u64 ext26_early_ns;
	u32 ext26_control_id;
	struct ext26_control *ext26_control;
	struct work_struct ext26_blank_work;
	struct list_head ext26_retained;
	u64 ext26_retained_bytes;
	unsigned int ext26_epochs;
	struct drm_display_mode ext26_mode;
	DECLARE_BITMAP(ext26_released, DCP_MAX_MAPPINGS);
	bool ext26_power_sent;
	bool ext26_normal;
	struct completion ext26_ready;
	iomfb_cb_handler *ext26_handlers;
	u8 ext26_present[0x1bd8];

	/* CoreAnalytics events dumped in full (dcpep_cb_core_analytics_v27). */
	u32 analytics_dumps;

	DECLARE_BITMAP(iomfb_surfaces, DCP_MAX_PLANES);

	/************* IOMFB **************************************************
	 * everything below is mostly used inside IOMFB but it could make     *
	 * sense to keep some of the members in apple_dcp.                    *
	 **********************************************************************/

	/* clock rate request by dcp in */
	struct clk *clk;
	struct clk *clk_194;
	struct clk_bulk_data *firmware_clocks;
	unsigned int num_firmware_clocks;
	u64 disp_firmware_base[MAX_DISP_REGISTERS];
	u32 dfb_format, dfb_compression;
	u64 dfb_region_size;
	bool dfb_format_valid, dfb_compression_valid;
	struct list_head iomfb_boolean_properties;
	u32 iomfb_boolean_property_count;
	u32 external_pipes_v27, external_displays_v27, external_suppression_v27;
	struct dcp_swap_submit_req_v27 present_v27;
	/* Both paired wire records belong to the primary endpoint lock. */
	struct dcp_swap_submit_req_v27 peer_present_v27;
	/*
	 * Swaps go out as A407 with its longer request, from these records: on the J714s with
	 * the 27.0 method table, where A407 has run (v27_methods is then the J714s's copy of
	 * that table). Everywhere else A408 and the records above, as before.
	 */
	bool v27_a407;
	struct dcp_swap_submit_req_v27_a407 present_v27_a407;
	struct dcp_swap_submit_req_v27_a407 peer_present_v27_a407;

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
		/* The connection whose TimingElements filled the mode catalogue. */
		struct dcp_metadata_ticket catalog_ticket;
		u64 catalog_connection_sequence;
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
	/* swap id of the last started swap */
	u32 started_swap_id;
	/*
	 * A started swap whose flip a commit gave up on
	 * (dcp_drm_crtc_flip_lost()); event_lock. Firmware that does not track
	 * its presents ignores its completion, see dcp_swap_lost().
	 */
	u32 lost_swap_id;
	bool swap_lost;
	ktime_t swap_start;
	u64 swap_submit_timestamp;

	/* Current display mode */
	struct dcp_mode_state mode_state;
	/* hotplug worker: power cycles since the mode was last valid */
	unsigned int tunnel_power_cycles;
	/* One HPD pulse after a placeholder EDID, per Type-C connection. */
	bool placeholder_retried;
	u64 typec_generation;	/* hpd_mutex: identifies the current connection */
	/* hpd_mutex: Type-C display sessions released; see dcp_poweroff() */
	u64 typec_unplugs;
	/* typec_unplugs when the CRTC last claimed its display session */
	u64 crtc_unplugs;
	u64 placeholder_generation;
	struct delayed_work placeholder_edid_wq;
	/* replugs replayed for a Type-C sink that lost its link */
	spinlock_t relink_lock;
	struct dcp_relink_state relink;
	struct delayed_work typec_relink_wq;
	/* The driver holds the sink HPD low itself; hpd_mutex. */
	bool typec_hpd_held;
	bool use_timestamps;
	bool vrr_enabled;
	/*
	 * The mode set asks for a PQ signal; output_pq: the one set carries it;
	 * output_edr: a panel shows the PQ content as EDR, edr_sent: swaps
	 * still carry EDR state the firmware must be told to drop.
	 */
	bool pq_requested;
	bool output_pq;
	bool output_edr;
	bool edr_sent;
	/*
	 * EDR headroom the panel was last given, x1000 (0: not in use), the
	 * SDR white in nits it was given for, and a power off since, after
	 * which the ramp starts again from 1.0 (see dcp_edr_prepare()).
	 */
	u32 edr_hm;
	u32 edr_sdr;
	bool edr_restart;
	/* Firmware 27: when the headroom last moved, and short of its target. */
	ktime_t edr_step;
	bool edr_ramping;
	/* SDR white in nits of the present being built (H17P backlight) */
	u32 edr_present_sdr;
	struct dcp_set_digital_out_mode_req mode;

	/* completion for active turning true */
	struct completion start_done;
	/* IOMFB has published fresh modes and acknowledged a Type-C display. */
	struct completion typec_display_ready;

	/* Is the DCP booted? */
	bool active;
	/* A successful initial enable keeps the H17P pipe running across DPMS. */
	bool pipe_enabled_h17p;
	/* Messages from the coprocessor are held in the mailbox for sleep. */
	bool rx_held;

	/* A firmware-driven suspend stops the firmware, see dcp_s2r_suspend(). */
	struct apple_dcp_afkep *sleep_v27_ep[0x20];
	/* The J714s normal method table, panel services and swap profile. */
	bool v27_j714s;
	/* Sleep endpoint inventory and control replies; not a normal RPC profile. */
	bool v27_sleep_transport;
	struct dcp_v27_sleep *sleep_v27;
	bool sleep_v27_closed;
	/*
	 * Its display on a USB-C port was released for a firmware sleep and the DRM device
	 * has not resumed since: the CRTC's saved mode is not restored (it resumes disabled,
	 * and the display's return has user space set a mode again).
	 */
	bool sleep_v27_torn;
	/* V27 sleep: the wait for an HPD low's answer while a display that never returned is let go */
	unsigned int v27_hpd_low_ms;
	/* Unhandled application traffic permanently invalidates the sleep's inventory. */
	bool sleep_v27_unhandled;
	bool s2r_active;
	bool s2r_stopped;
	/* DPMS off stopped the firmware and lowered the domains; DPMS on brings them back. */
	bool dpms_powered_off;
	/* Refused its idle with the display off: the firmware still runs, the display is dark. */
	bool s2r_refused_off;
	/* The firmware's secure mailbox, where it has one. */
	struct apple_rtkit *secure_rtk;
	bool secure_answered;
	bool secure_polling;
	/* Polled every jiffy until then, see dcp_secure_poll(). */
	unsigned long secure_fast_until;
	int secure_map_ret;
	s64 secure_map_us;
	struct delayed_work secure_poll_wq;
	/* Stopped and not restarted, which outlives a suspend that gets refused. */
	bool s2r_fw_down;
	/* DCPEXT kept down through a dark wake, see dcp_s2r_dark_finish(). */
	bool s2r_dark_held;
	struct notifier_block s2r_pm_nb;
	/* This driver raised the display domains, and nothing has lowered them since. */
	bool s2r_domains_up;
	/* Firmware started by the resume, panel still dark: DPMS on lights it. */
	bool s2r_prewarmed;
	struct work_struct s2r_prewarm_work;
	struct delayed_work s2r_prewarm_off_wq;
	/* Orders the start from the resume and its timeout against DPMS on and off. */
	struct mutex s2r_prewarm_lock;
	/* This wake lights the panel from the resume too: full wake, lid read open. */
	bool s2r_wake_light;
	/* Lit by the resume, with the CRTC still off: DPMS on has nothing left to power. */
	bool s2r_wake_lit;
	ktime_t s2r_wake_lit_at;
	/* The retained scanout is from before the sleep; with hold, no present reuses it. */
	bool s2r_scanout_stale;
	bool s2r_scanout_hold;
	/* Counted in pmc-dcp, the number of powered DCPs */
	bool pmc_up;
	bool s2r_power_on;
	bool s2r_power_oob;
	u8 s2r_step;
	struct completion s2r_dart_done;
	struct device *s2r_disp_dart;
	struct device *s2r_dcp_dart;

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
	/* When the latest arm expires, in jiffies. */
	unsigned long swap_watchdog_deadline;

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
	/* The last link report, and the connection it was for; DSC as the firmware last named it */
	char link_line[120];
	u64 link_line_generation;
	u8 link_dsc;

	/* V27 sleep: the panel's next mode set is the mode it already has */
	bool v27_keep_mode;
	/* V27 sleep: the output area of the call being pushed holds this (an in/out word) */
	const void *push_out_prefill;

	/* firmware 27 panel: its AV and DP service endpoints, by endpoint - 0x20 */
	struct apple_dcp_afkep *panelav[0x20];
	struct workqueue_struct *panelav_wq;

	/* debugfs entries */
	struct dentry *ep_debugfs[0x20];

	/* these fields are output port specific */
	struct phy *phy;
	struct phy *fixed_phy;
	/*
	 * The crossbar control a link configuration takes up and down: the
	 * fixed output's, or on desktop 26.x the active Type-C route's.
	 */
	struct mux_control *xbar;
	/* The fixed (HDMI) output's own crossbar control, selected for it. */
	struct mux_control *fixed_xbar;
	struct typec_mux *typec_mux;
	struct apple_dcp_typec_route typec_routes[DCP_MAX_TYPEC_ROUTES];
	struct apple_dcp_typec_route *active_typec_route;
	u32 nr_typec_routes;
	bool phy_managed_by_typec;
	bool typec_cable_connected;
	/*
	 * T8132 dcpext: a teardown call timed out; the firmware's late
	 * Activate/Deactivate belong to it until its answer is in.
	 */
	bool t8132_release_late;
	/* DPTX feeds a Thunderbolt DP IN adapter, not the Type-C PHY lanes */
	bool dptx_tunnel;
	/* DFP port in the DPTX target: 0 = dpphy, 1 = dpin0, 2 = dpin1 */
	u8 dptx_dfp_port;
	/* wakes/sleeps the Thunderbolt DP IN adapter from DCP Activate/Deactivate */
	int (*tb_dpin_set_active)(void *ctx, bool active);
	void *tb_dpin_ctx;
	u64 tb_generation; /* tb_lock: admitted callback cookie, including attach */
	/*
	 * Serializes the Thunderbolt DP IN callback and tunnel crossbar state
	 * between DCP apcalls and tunnel teardown; never held while waiting
	 * for DCP.
	 */
	struct mutex tb_lock;
	bool tb_clock_ok;
	/*
	 * T8132 dcpext: its tunnel pixel clock was refused (the PHY runs the
	 * other DP IN at another link rate), so the tunnel cannot show.
	 * tb_lock to set; read anywhere.
	 */
	bool tb_clock_refused;
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
	/* A Type-C CRTC enable runs: its own mode set follows, the link is not bad */
	bool typec_enable_pending;
	/* IOMFB reports its video interface ready after DPTX link training. */
	struct completion typec_iomfb_hpd_ready;
	struct delayed_work typec_reconnect_wq;
	u32 typec_reconnect_tries;
	struct dcp_fabric_session typec_reconnect_session; /* hpd_mutex */
	/* hpd_mutex: what DRM reports for the external display, and when */
	struct dcp_announce announce;
	struct delayed_work announce_wq;
	bool typec_work_enabled; /* hpd_mutex; IRQ recovery admission */
#if IS_ENABLED(CONFIG_DRM_APPLE_KUNIT_TEST)
	/* Lets a test stop work where it races, see dcp_test_point(). */
	void (*test_point)(struct apple_dcp *dcp, enum dcp_test_point point);
	/* Lets a test answer IOMFB method calls in place of the firmware. */
	void (*test_push)(struct apple_dcp *dcp, const struct dcp_method_entry *call,
			  dcp_callback_t cb, void *cookie);
#endif

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

	/* H17P policy shared by backlight, commit, and reply workers. */
	struct {
		/* Protects state and callback installation. */
		spinlock_t lock;
		struct dcp_backlight_state state;
		void (*kick)(struct apple_dcp *dcp);
		/* Steps a running fade; fade_start is its start in jiffies. */
		struct delayed_work fade_work;
		struct work_struct notify_work;
		struct delayed_work retry_work;
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
		spinlock_t callback_lock;
		struct list_head callbacks;
		struct work_struct callback_work;
		unsigned int callback_count;
		struct delayed_work timeout;
		struct delayed_work backlight_retry;
		/* A firmware 27 panel's EDR ramp on a still frame (iomfb_edr_tick()) */
		struct delayed_work edr_tick;
		unsigned long deadline;
		unsigned int queued;
		bool stopped;
		bool presentation_paused;
		bool backlight_queued;
		atomic_t opaque_x_state;
		struct iomfb_scanout_h17p *scanout;
		struct iomfb_scanout_h17p *next_scanout;
		/*
		 * Presents the firmware dropped, until it completes a later one:
		 * at most one per present retired in present_state_h17p.
		 */
		struct list_head dropped;
	} iomfb;
	/* Hardirq, resume and expiry share one generation under this lock. */
	spinlock_t hdmi_presence_lock;
	struct dcp_fabric_presence hdmi_presence;
	/* The state before the current series of HPD edges began. */
	enum dcp_fabric_presence_state hdmi_presence_prev;
	/*
	 * A desktop 26.x Type-C display that came back from system sleep with a
	 * partial mode list (the sink was not ready for its EDID yet) is
	 * connected again after a retrain found no mode.
	 */
	struct delayed_work link_recovery_wq;
	unsigned int link_recovery_tries;
	unsigned int link_recovery_snap;
	atomic_t modesets_done;
	unsigned long hdmi_edge_jiffies;
	bool hdmi_edge_seen;
	struct delayed_work hdmi_settle_wq;
	struct delayed_work hdmi_recheck_wq;
	/* T6030 HDMI converter service requests, see dcp-hdmi.c */
	struct dcp_hdmi *hdmi;

	/*
	 * Interrupts of a Thunderbolt DP tunnel's DP IN adapter, to pass to
	 * the DPTX port; see apple_dcp_tb_dp_event().
	 */
	struct work_struct tb_event_work;
	/* Serializes event delivery against attachment retirement. */
	struct mutex tb_event_lock;
	struct apple_dcp_typec_route *tb_event_route;
	u64 tb_event_generation;
	unsigned long tb_events;
	bool tb_event_hpd;
	unsigned int tb_irq_hpd_count;
	unsigned int tb_hpd_count;
	ktime_t tb_irq_hpd_last;

	/* Kept last: the template instances share the layout above. */
	struct dcp_desktop_state *desktop;
	/* Internal panel identity; NULL without an internal panel. */
	struct dcp_panel_identity *panel_identity;
};

/* T8132 firmware uses its own wire codec, with the common DRM/KMS owner. */
static inline bool iomfb_is_m4(const struct apple_dcp *dcp)
{
	return IS_ENABLED(CONFIG_DRM_APPLE_DCP_T8132) &&
	       dcp->fw_compat == DCP_FIRMWARE_M4;
}

/* Presents carry nits; one serialized IOMFB owner. */
static inline bool dcp_presents_nits(const struct apple_dcp *dcp)
{
	return ((dcp->fw_compat == DCP_FIRMWARE_H17P ||
	         dcp->fw_compat == DCP_FIRMWARE_V_27_0) &&
		dcp->hw.iomfb_method_profile != DCP_IOMFB_METHODS_H17G) ||
	       iomfb_is_m4(dcp);
}

/* An adopted firmware session that fails stop and is never torn down. */
static inline bool dcp_fail_stop(const struct apple_dcp *dcp)
{
	return dcp->fw_compat == DCP_FIRMWARE_H17P ||
	       dcp->fw_compat == DCP_FIRMWARE_V_27_0 ||
	       iomfb_is_m4(dcp);
}

bool dcp_firmware_metadata_enabled(struct apple_dcp *dcp);
void dcp_firmware_metadata_connection(struct apple_dcp *dcp, u64 generation, bool connected);
void dcp_firmware_metadata_hotplug(struct apple_dcp *dcp, bool connected);
void dcp_firmware_metadata_announced(struct apple_dcp *dcp);
void dcp_firmware_metadata_begin(struct apple_dcp *dcp);
void dcp_firmware_metadata_attributes(struct apple_dcp *dcp,
				      const struct dcp_display_metadata *attributes);
void dcp_firmware_metadata_timing(struct apple_dcp *dcp);
bool dcp_firmware_metadata_current(struct apple_dcp *dcp, u64 generation);
const struct drm_edid *dcp_firmware_edid(struct apple_dcp *dcp, u64 *generation,
					 u32 *defaults);

void iomfb_scanout_complete_h17p(struct apple_dcp *dcp);
void iomfb_present_backlight_h17p(struct apple_dcp *dcp);
void iomfb_present_matrix_h17p(struct apple_dcp *dcp);
bool iomfb_present_brightness_only_h17p(struct apple_dcp *dcp);
bool iomfb_present_complete_h17p(struct apple_dcp *dcp);
void iomfb_present_failed_h17p(struct apple_dcp *dcp);
bool iomfb_retire_dropped_present_h17p(struct apple_dcp *dcp);
bool iomfb_present_has_surface_h17p(const struct dcp_swap_submit_req_h17p *request);
bool iomfb_apply_backlight_h17p(struct apple_dcp *dcp,
				const struct dcp_swap_submit_req_h17p *request,
				struct dcp_present_h17p *wire);
void iomfb_apply_opaque_x_h17p(struct apple_dcp *dcp, dcp_callback_t callback,
			       void *cookie);
void iomfb_opaque_x_reset_h17p(struct apple_dcp *dcp);
int iomfb_configure_backlight_h17p(struct apple_dcp *dcp, u32 maximum,
				   bool inherited_valid, u32 inherited,
				   bool default_valid, u32 default_nits);
int iomfb_queue_callback(struct apple_dcp *dcp, dcp_callback_t callback,
			 const void *data, size_t size, void *cookie);
void iomfb_queue_init(struct apple_dcp *dcp);
void iomfb_queue_stop(struct apple_dcp *dcp);
void iomfb_reset_transport(struct apple_dcp *dcp);
bool iomfb_queue_drain(struct apple_dcp *dcp, unsigned long timeout);
int iomfb_queue_settle(struct apple_dcp *dcp, unsigned long timeout);
void iomfb_pause_presentation(struct apple_dcp *dcp, bool pause);
int iomfb_external_power_h17p(struct apple_dcp *dcp, bool on);
int iomfb_queue(struct apple_dcp *dcp, struct iomfb_transaction *transaction);
void iomfb_queue_crc_h17p(struct apple_dcp *dcp, u32 swap_id);

void dcp_drm_crtc_page_flip(struct apple_dcp *dcp, ktime_t now);
void dcp_handle_hotplug_actions(struct apple_dcp *dcp, unsigned int action);
bool dcp_external_hotplug(struct apple_dcp *dcp, bool connected);
void dcp_announce_arm(struct apple_dcp *dcp);
void dcp_announce_work(struct work_struct *work);
void dcp_external_crtc_released(struct apple_dcp *dcp);
void dcp_external_link_fault(struct apple_dcp *dcp);
irqreturn_t dcp_desktop_dp2hdmi_hpd(struct apple_dcp *dcp);

/* A KUnit test can wait here for the work it races with. */
static inline void dcp_test_point(struct apple_dcp *dcp, enum dcp_test_point point)
{
#if IS_ENABLED(CONFIG_DRM_APPLE_KUNIT_TEST)
	void (*test_point)(struct apple_dcp *dcp, enum dcp_test_point point);

	test_point = READ_ONCE(dcp->test_point);
	if (test_point)
		test_point(dcp, point);
#endif
}

#if IS_ENABLED(CONFIG_KUNIT)
void dcp_work_register_backlight(struct work_struct *work);
void dcp_comp_unbind(struct device *dev, struct device *main, void *data);
void dcp_platform_shutdown(struct platform_device *pdev);
void dcp_disable_typec_work(struct apple_dcp *dcp, bool release_cable);
void dcp_enable_typec_work(struct apple_dcp *dcp);
void dcp_swap_watchdog(struct work_struct *work);
void iomfb_scanout_put_h17p(struct apple_dcp *dcp, struct iomfb_scanout_h17p *scanout);
#endif

int dcp_backlight_init(struct apple_dcp *dcp, struct device *dev);
void dcp_backlight_stop(struct apple_dcp *dcp);
void dcp_backlight_finish_fade(struct apple_dcp *dcp);
int dcp_backlight_register(struct apple_dcp *dcp);
int dcp_backlight_update(struct apple_dcp *dcp);
s32 dcp_ext_backlight_value(struct apple_dcp *dcp);
bool dcp_has_panel(struct apple_dcp *dcp);
extern int fw27_panel_boot_mode;
extern int notify_clients_form;

/*
 * The desktop 26.6 panel takes VRR_ENABLED on its live mode, through the
 * adaptive sync minimum rate (see dcp_vrr_update()).  The external
 * processors ignore a new minimum on a live mode.
 */
/*
 * Desktop 26.x swaps that the watchdog follows: adaptive ones, and those on a
 * loader mode the takeover adopted until one of them completes.
 */
static inline bool dcp_desktop_swap_watched(struct apple_dcp *dcp)
{
	return dcp->fw_compat == DCP_FIRMWARE_V_26_6_DESKTOP &&
	       (READ_ONCE(dcp->vrr_enabled) ||
		(dcp->desktop && READ_ONCE(dcp->desktop->takeover_adopted)));
}

static inline void dcp_takeover_adopt_proven(struct apple_dcp *dcp)
{
	if (dcp->fw_compat == DCP_FIRMWARE_V_26_6_DESKTOP && dcp->desktop)
		WRITE_ONCE(dcp->desktop->takeover_adopted, false);
}

static inline bool dcp_vrr_live(struct apple_dcp *dcp)
{
	return dcp->fw_compat == DCP_FIRMWARE_V_26_6_DESKTOP && dcp->main_display &&
	       dcp_has_panel(dcp);
}

/*
 * Does @dcp's firmware speak the 14.7 AFK, DPTX and DPAV endpoint protocol,
 * whichever IOMFB path drives it? DCP_FIRMWARE_V_14_7 is the native DCPLink
 * session; on DCP_FIRMWARE_V_14_7_TEMPLATE only the processors marked by
 * their match data (the T8122 dcpext): generic M1/M2 DCPs on 14.7.0 keep the
 * endpoint handling they had.
 */
static inline bool dcp_fw_is_14_7(const struct apple_dcp *dcp)
{
	return dcp->fw_compat == DCP_FIRMWARE_V_14_7 ||
	       (dcp->fw_compat == DCP_FIRMWARE_V_14_7_TEMPLATE &&
		dcp->hw.fw_14_7_endpoints);
}

static inline bool dcp_is_ext26(const struct apple_dcp *dcp)
{
	return dcp->hw.iomfb_method_profile == DCP_IOMFB_METHODS_J613_EXT26;
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

/*
 * Firmware 27's external processors (the T6050 DCPEXTs): their DP/HDMI
 * audio service is the desktop one, on endpoint 0x2b (see av.c).
 */
static inline bool dcp_is_v27_external(const struct apple_dcp *dcp)
{
	return dcp->fw_compat == DCP_FIRMWARE_V_27_0 && dcp->hw.num_dptx_ports;
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
	/* Not the generic M1/M2 DCPs this tree also runs on the template. */
	return (dcp->fw_compat == DCP_FIRMWARE_V_14_7_TEMPLATE &&
		dcp->hw.fw_14_7_endpoints);
}

/*
 * Whether this DCP reads a surface's element geometry as the surface record
 * defines it: bytes per element (pix_size, a linear plane's tile_size), the
 * element's width and height in pixels, and compression information only for
 * a compressed plane. Only firmware 27 (DCP_FIRMWARE_V_27_0, which only
 * T6050's DCP hw data selects) is tested with it. Earlier firmware keeps the
 * fixed 1/1/1 and has_compr_info 1 it has run on: before widening this,
 * check a linear XRGB8888 and an NV12 swap, and the default framebuffer, on
 * 12.x-26.x firmware.
 */
static inline bool dcp_surface_element_geometry(const struct apple_dcp *dcp)
{
	return dcp->fw_compat == DCP_FIRMWARE_V_27_0;
}

/*
 * Whether @pa, a physical address the DCP firmware placed a buffer at, can be
 * DRAM. On Apple SoCs every device register window lies below DRAM, so an
 * address below all of Linux's RAM may be a device's registers and is never
 * mapped as memory. The firmware's own segments lie in DRAM beside Linux's RAM.
 */
static inline bool dcp_fw_phys_in_dram(phys_addr_t pa)
{
	return pa && region_intersects(0, pa, IORESOURCE_SYSTEM_RAM,
				       IORES_DESC_NONE) != REGION_DISJOINT;
}

#define DCP_AUDIO_MAX_CHANS 15

void dcp_trace_panel_property(struct apple_dcp *dcp, int tag, const char *key,
			      size_t key_len, const void *blob, size_t len);

void dcp_panel_edid_receive(struct apple_dcp *dcp, const void *blob, size_t len);

#endif /* __APPLE_DCP_INTERNAL_H__ */
