// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Copyright 2021 Alyssa Rosenzweig */

#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/mux/driver.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/slab.h>
#include <linux/soc/apple/dp-tunnel.h>
#include <linux/usb/typec_altmode.h>
#include <linux/usb/typec_dp.h>
#include <linux/workqueue.h>

#include <drm/drm_file.h>

#include <kunit/static_stub.h>

#include "afk.h"
#include "dcp.h"
#include "dcp-fabric.h"
#include "dcp-fabric-effects.h"
#include "dcp-hdmi.h"
#include "dcpext_scanout.h"
#include "ibootep.h"
#include "dcp-internal.h"
#include "parser.h"

static bool tiled_split = true;
module_param(tiled_split, bool, 0644);
MODULE_PARM_DESC(tiled_split,
		 "Drive both halves of a tiled Thunderbolt display (LG UltraFine 5K) on one pipeline, through its two DPTX ports (default: on)");

static bool typec_follow_crtc = true;
module_param(typec_follow_crtc, bool, 0444);
MODULE_PARM_DESC(typec_follow_crtc,
		 "Let direct DP-alt routes follow their selected CRTC (default: on)");

struct apple_dcp_typec_port {
	struct dcp_fabric_port core;
	struct dcp_fabric_plan plan;
	struct list_head link;
	struct list_head routes;
	struct device_node *connector_np;
	struct apple_dcp_typec_route *owner;
	struct apple_dcp_typec_route *secondary_owner;
	/* Keep a port on its last DCP while that pipeline remains free. */
	struct apple_dcp_typec_route *preferred_route;
	/* DRM connector for this physical port, driven by whichever DCP owns it */
	struct apple_connector *connector;
	/* A second logical stream through this port's USB4 dock. */
	struct apple_connector *secondary_connector;
	/* tiled_split: the pipeline whose DPTX port 1 carries dpin1 */
	struct apple_dcp *split_dcp;
	/* how long dpin1 waits for dpin0's tile hint before routing normally */
	unsigned long tile_hint_deadline;
	/* last mux state acted on, to collapse the per-candidate notifications */
	struct typec_altmode *applied_alt;
	unsigned long applied_mode;
	u32 applied_status;
	u32 applied_conf;
	bool applied_valid;
	/* SAFE preparation retains this owner until the set phase completes. */
	bool safe_pending;
	bool safe_finish_ready;
	bool hpd;
	/* Direct DP-alt: the port is in DP mode, routed or not, and its HPD. */
	bool dp_wanted;
	bool dp_hpd;
	/* the wait for a pipeline was logged since the port entered DP mode */
	bool busy_noted;
	/* dcp_typec_rebalance_locked()'s plan for the port's two streams */
	struct apple_dcp_typec_route *target;
	struct apple_dcp_typec_route *secondary_target;
	/* left out of the plan: its planned pipeline is held by a tunnel */
	bool plan_dark;
};

/*
 * HDMI hotplug often blinks: a monitor waking up or switching inputs drops
 * HPD for a second or so.  A Thunderbolt tunnel never moves once it has a
 * pipeline, so without dual-stream docks one may not take the hybrid until
 * HDMI has been quiet for this long, or a tunnel waiting for a pipeline
 * would take it in the blink and keep the returning HDMI display dark.
 */
#define DCP_HDMI_HOLD_MS	10000

static DEFINE_MUTEX(dcp_typec_fabric_lock);

static LIST_HEAD(dcp_typec_ports);
/* Current DTs wire the same stream capacity on every physical port. */
static atomic_t dcp_dual_stream_routes = ATOMIC_INIT(0);

static enum dcp_fabric_presence_state dcp_hdmi_presence(struct apple_dcp *dcp);
static void dcp_hdmi_update_locked(struct apple_dcp *dcp);
static int dcp_hdmi_read_hpd(void *ctx);
static bool dcp_hdmi_borrowed(void *ctx);

bool dcp_is_typec_output(struct apple_dcp *dcp)
{
	return dcp->active_typec_route ||
	       dcp->fixed_connector_type == DRM_MODE_CONNECTOR_USB;
}

bool dcp_is_usb4_output(struct apple_dcp *dcp)
{
	return dcp->active_typec_route && dcp->active_typec_route->tunnel;
}

/* SoC flow support and board wiring are separate qualifications. */
bool dcp_uses_t6020_tunnel_flow(struct apple_dcp *dcp)
{
	struct apple_dcp_typec_route *route = READ_ONCE(dcp->active_typec_route);
	bool connector_wired = route && apple_dp_tunnel_dual_stream(route->port->connector_np);

	return dcp_fabric_t6020_flow(dcp_is_usb4_output(dcp),
				     dcp->hw.t6020_tunnel_flow, connector_wired);
}

bool dcp_hdmi_port_attached(struct apple_dcp *dcp)
{
	KUNIT_STATIC_STUB_REDIRECT(dcp_hdmi_port_attached, dcp);

	return dcp->hdmi_hpd && !READ_ONCE(dcp->active_typec_route) &&
	       gpiod_get_value_cansleep(dcp->hdmi_hpd) > 0;
}

static bool dcp_typec_route_is_dp(const struct typec_mux_state *state)
{
	return state->alt && state->alt->svid == USB_TYPEC_DP_SID &&
	       state->mode >= TYPEC_DP_STATE_A &&
	       state->mode <= TYPEC_DP_STATE_F;
}

/* Keep a live fixed output on its own pipeline. */
static bool dcp_typec_route_fixed_output_busy(struct apple_dcp_typec_route *route)
{
	struct apple_dcp *dcp = route->dcp;

	if (dcp->fixed_connector_type == DRM_MODE_CONNECTOR_USB)
		return false;
	/*
	 * The 14.7 firmware drives the internal panel on the IOMFB path.
	 * The DPTX service is a separate output, so the live panel does not
	 * occupy the Type-C route. An external processor's fixed output (a
	 * display behind the HDMI port) does, as on M1/M2.
	 */
	if (dcp->fw_compat == DCP_FIRMWARE_V_14_7 && !dcp->external)
		return false;
	return dcp_fabric_fixed_busy(false,
				     false,
				     dcp->fixed_connector && dcp->fixed_connector->connected,
				     !(dcp->fixed_connector && dcp->fixed_connector->connected) &&
				     dcp->hdmi_hpd &&
				     gpiod_get_value_cansleep(dcp->hdmi_hpd));
}

/* A port in SAFE retains this pipeline until the mux set phase completes. */
static bool dcp_typec_safe_held(struct apple_dcp *dcp)
{
	struct apple_dcp_typec_port *port;

	list_for_each_entry(port, &dcp_typec_ports, link)
		if (port->safe_pending && port->owner && port->owner->dcp == dcp)
			return true;
	return false;
}

/*
 * A pipeline that can take no Type-C display: its fixed output is live, or
 * it is a T6030 external processor that failed (handoff check, start, or a
 * stopped session), which drives nothing until reboot. Planned around and
 * passed over like a busy one, so that a healthy pipeline takes the port.
 */
static bool dcp_typec_route_busy(struct apple_dcp_typec_route *route)
{
	struct apple_dcp *dcp = route->dcp;

	return dcp_typec_route_fixed_output_busy(route) ||
	       (dcp->external_native && iomfb_v14_7_external_failed(dcp));
}

static bool dcp_typec_route_available(struct apple_dcp_typec_route *route)
{
	struct dcp_fabric_pipeline pipeline = {
		.owned = !!route->dcp->active_typec_route ||
			 dcp_typec_safe_held(route->dcp),
		.fixed_busy = dcp_typec_route_busy(route),
	};

	struct dcp_fabric_policy policy = { .dual_stream = dcp_typec_ordered_dual_stream() };

	pipeline.presence = dcp_hdmi_presence(route->dcp);
	return dcp_fabric_available(&pipeline, &policy);
}

/*
 * Machines whose USB4 docks carry two independent DP streams through one
 * port: DPIN0 drives the hybrid dcpext0, DPIN1 the Type-C-only dcpext1, and
 * each port has a second connector for the DPIN1 stream.
 */
bool dcp_typec_dual_stream(void)
{
	return atomic_read(&dcp_dual_stream_routes) > 0;
}

/* Keep native T6030 routes pinned to their owning processor, on either DPIN. */
bool dcp_typec_ordered_dual_stream(void)
{
	return dcp_fabric_ordered_dual_stream(dcp_typec_dual_stream(),
					    of_machine_is_compatible("apple,t6030"));
}

/*
 * Do the Type-C routes of @dcp follow the CRTC a modeset pairs them with
 * (see dcp_typec_follow_crtc())? Direct routes and Thunderbolt tunnels,
 * on 12.3/13.5 only.
 */
bool dcp_typec_follows_crtc(struct apple_dcp *dcp)
{
	return typec_follow_crtc && dcp->nr_typec_routes &&
	       !dcp->external && !dcp->external_native &&
	       (dcp->fw_compat == DCP_FIRMWARE_V_12_3 ||
		dcp->fw_compat == DCP_FIRMWARE_V_13_5);
}

/* Do all pipelines that can drive @port let its routes follow their CRTC? */
static bool dcp_typec_port_follows(struct apple_dcp_typec_port *port)
{
	struct apple_dcp_typec_route *route;

	list_for_each_entry(route, &port->routes, port_link)
		if (!dcp_typec_follows_crtc(route->dcp))
			return false;
	return !list_empty(&port->routes);
}

/* Is a routed port's possible_crtcs narrowed to the pipeline driving it? */
static bool dcp_typec_narrows(struct apple_dcp *dcp)
{
	/* This tree narrows on dual-stream machines too. */
	return !dcp_typec_follows_crtc(dcp);
}

bool dcp_is_typec_only(struct platform_device *pdev)
{
	struct apple_dcp *dcp = platform_get_drvdata(pdev);

	return !dcp->fixed_phy;
}

static int dcp_route_clock_rate(struct apple_dcp_typec_route *route,
				struct phy *phy, unsigned int dpin, u8 rate)
{
	int ret;

	if (of_machine_is_compatible("apple,t6030")) {
		typeof(apple_atc_dp_tunnel_rate_source) *fn;

		if (dcp_fabric_t6030_pclk(route->mux_index, dpin) < 0)
			return -EOPNOTSUPP;
		fn = symbol_get(apple_atc_dp_tunnel_rate_source);
		if (fn) {
			ret = fn(phy, route->mux_index, dpin, rate);
			symbol_put(apple_atc_dp_tunnel_rate_source);
			return ret;
		}
		/* An older ATC provider clocks source 0 only, as before. */
		if (route->mux_index != 0)
			return -ENOENT;
	}
	{
		typeof(apple_atc_dp_tunnel_rate) *fn;

		fn = symbol_get(apple_atc_dp_tunnel_rate);
		if (!fn)
			return -ENOENT;
		ret = fn(phy, dpin, rate);
		symbol_put(apple_atc_dp_tunnel_rate);
	}
	return ret;
}

static int dcp_typec_route_activate(struct apple_dcp_typec_route *route,
				    struct mux_control *xbar);
static int dcp_typec_route_deactivate(struct apple_dcp_typec_route *route);
static int dcp_dpxbar_preselect(struct mux_control *mux, int state);

static int dcp_dpxbar_tunnel_select_source(struct mux_control *mux, int state)
{
	typeof(&apple_dpxbar_tunnel_select_source) select =
		symbol_get(apple_dpxbar_tunnel_select_source);
	int ret;

	if (!select)
		return -ENOENT;
	ret = select(mux, state);
	symbol_put(apple_dpxbar_tunnel_select_source);
	return ret;
}

static int dcp_typec_route_activate(struct apple_dcp_typec_route *route,
				    struct mux_control *xbar)
{
	struct apple_dcp *dcp = route->dcp;
	struct apple_connector *connector =
		xbar != route->xbar && route->tunnel_dpin == 1 ?
		route->port->secondary_connector : route->port->connector;
	int ret;

	dcp_firmware_metadata_connection(dcp, READ_ONCE(dcp->typec_generation), false);

	/*
	 * The fixed output's HPD handler leaves disconnects to DCP, so the port
	 * can still be marked connected to a display that is gone. Release it
	 * (a no-op otherwise), or connecting the borrowed route returns early.
	 */
	ret = dcp_dptx_disconnect(dcp, 0);
	if (ret)
		return ret;

	if (dcp->fixed_route_selected) {
		ret = mux_control_deselect(dcp->xbar);
		dcp->fixed_route_selected = false;
		if (ret)
			return ret;
	}

	/*
	 * Thunderbolt DP IN: the crossbar connection may only be brought up
	 * once DCP has configured the link and the tunnel pixel clock runs,
	 * see dcp_tunnel_crossbar_up(). Just remember the output here.
	 */
	if (xbar != route->xbar && route->mux_index) {
		/*
		 * The T602X crossbar must point a DP IN at its pipeline before
		 * DCP probes AUX; other crossbars select it at link-up and
		 * report -EOPNOTSUPP here.
		 */
		ret = dcp_dpxbar_tunnel_select_source(xbar, route->mux_index);
		if (ret == -EOPNOTSUPP)
			ret = 0;
	} else {
		ret = xbar == route->xbar ?
			mux_control_select(xbar, route->mux_index) : 0;
	}
	if (ret) {
		if (dcp->xbar) {
			int restore_ret;

			restore_ret = mux_control_select(dcp->xbar,
							 dcp->fixed_mux_index);
			if (!restore_ret)
				dcp->fixed_route_selected = true;
			else
				dev_err(dcp->dev,
					"failed to restore fixed display route: %d\n",
					restore_ret);
		}
		return ret;
	}

	dcp_modes_begin_attachment(dcp);
	apple_connector_edid_set_live(dcp->connector, false);
	dcp->phy = route->phy;
	dcp->dptx_phy = route->dptx_phy;
	dcp->connector_type = DRM_MODE_CONNECTOR_USB;
	WRITE_ONCE(dcp->ext_backlight, false);
	if (connector) {
		apple_connector_set_pipeline(connector, to_platform_device(dcp->dev));
		dcp->typec_connector = connector;
		dcp->connector = connector;

		/*
		 * Narrow the port to the pipeline now driving it.  The encoder
		 * spans every pipeline that could, which is what lets userspace
		 * see the port as usable at all -- but only one of them is
		 * routed to the display, and userspace has no way to tell which.
		 * Offering it the choice makes it pair the port with a pipeline
		 * holding a different monitor's mode list, and the modeset is
		 * rejected with no way for it to recover.  The hotplug that
		 * follows publishes the actual binding on dual-stream machines
		 * too. Clients must re-read encoder topology after a hotplug.
		 * Pipelines whose direct routes follow the CRTC they are
		 * paired with keep the full set: the pairing is the
		 * compositor's to make there.
		 */
		if (connector->port_encoder && dcp->crtc &&
		    dcp_typec_narrows(dcp))
			connector->port_encoder->possible_crtcs =
				dcp_fabric_connector_mask(false,
							  true, true,
							  drm_crtc_index(&dcp->crtc->base),
							  connector->candidate_crtcs);
	}
	dcp->active_typec_route = route;
	scoped_guard(mutex, &dcp->tb_lock) {
		route->deactivate_xbar_done = false;
		route->deactivate_error = 0;
		route->active_xbar = xbar;
		route->tunnel = xbar != route->xbar;
		route->xbar_up = !route->tunnel;
		/* A newly selected output starts with its clocks off. */
		dcp->direct_xbar_up = NULL;
		dcp->dptx_tunnel = route->tunnel;
		/* crossbar controls are dpphy, dpin0, dpin1: same order as the DFP port */
		dcp->dptx_dfp_port = route->tunnel ? xbar - &route->xbar->chip->mux[0] : 0;
		dcp->tb_clock_ok = false;
	}
	route->selected = true;

	dev_info(dcp->dev, "allocated Type-C DPTX PHY %u\n", route->dptx_phy);
	return 0;
}

static int dcp_typec_route_deactivate(struct apple_dcp_typec_route *route)
{
	struct apple_dcp *dcp = route->dcp;
	struct apple_connector *connector = dcp->typec_connector;
	struct mux_control *active_xbar = route->active_xbar;
	bool was_tunnel = route->tunnel;
	bool ended_attachment;
	int ret = 0;

	guard(mutex)(&dcp->external_lock);
	if (dcp_is_external_h17p(dcp) &&
	    (dcp->external_error || (dcp->external_power_attempted && !dcp->external_stopped)))
		return dcp->external_error ?: -EAGAIN;

	dcp_firmware_metadata_connection(dcp, READ_ONCE(dcp->typec_generation), false);

	/*
	 * Under tb_lock so a DCP link (re)configuration can neither select the
	 * crossbar nor restart the tunnel pixel clock behind our back.
	 */
	scoped_guard(mutex, &dcp->tb_lock) {
		if (route->deactivate_error)
			return route->deactivate_error;
		if (!route->deactivate_xbar_done) {
			if (!route->tunnel || route->xbar_up)
				ret = mux_control_deselect(route->active_xbar ?: route->xbar);
			else
				ret = dcp_dpxbar_preselect(route->active_xbar, MUX_IDLE_DISCONNECT);
			/* Deselect always unlocks: never consume that selection twice. */
			route->deactivate_xbar_done = true;
			route->xbar_up = false;
			if (ret) {
				/* Idle is unproven; retain the owner and the real error. */
				route->deactivate_error = ret;
				return ret;
			}
		}

		if (route->tunnel && dcp->phy) {
			/* the tunnel pixel clock must not outlive the tunnel */
			int err = dcp_route_clock_rate(route, dcp->phy, route->tunnel_dpin, 0);

			if (err)
				dev_warn(dcp->dev, "DP tunnel clock stop failed: %d\n", err);
		}
		route->active_xbar = NULL;
		route->tunnel = false;
		dcp->direct_xbar_up = NULL;
		dcp->dptx_tunnel = false;
		dcp->dptx_dfp_port = 0;
		dcp->tb_dpin_set_active = NULL;
		dcp->tb_dpin_ctx = NULL;
		dcp->tb_generation = 0;
		route->tunnel_generation = 0;
		dcp->tb_clock_ok = false;
	}
	if (was_tunnel && route->mux_index) {
		int sel = dcp_dpxbar_tunnel_select_source(active_xbar, -1);

		if (sel && sel != -EOPNOTSUPP)
			dev_warn(dcp->dev, "DP tunnel source reset failed: %d\n", sel);
	}
	if (dcp_is_external_h17p(dcp))
		dpavservep_detach(dcp);
	route->selected = false;
	ended_attachment = dcp_modes_end_typec(dcp, route);
	if (dcp->active_typec_route == route)
		dcp->active_typec_route = NULL;

	if (connector && connector->dcp == to_platform_device(dcp->dev)) {
		/*
		 * Until the port is activated again it has no pipeline behind
		 * it, and nothing can read modes or EDID from it.  Report it
		 * disconnected for that window: leaving a connected connector
		 * whose ->dcp is NULL lets anything probing it in between --
		 * a compositor starting up while the fabric is still settling
		 * -- see an output it cannot get a mode for, and give up on
		 * it.  The new pipeline marks it connected again once the
		 * display has come back up on it.
		 */
		WRITE_ONCE(connector->connected, false);
		/* hotplug work queued before this checks for it */
		apple_connector_set_pipeline(connector, NULL);
		/* no pipeline, so no backlight */
		schedule_work(&connector->bl_sync_wq);

		/* Unrouted: the port could go to any of its pipelines again. */
		if (connector->port_encoder)
			connector->port_encoder->possible_crtcs =
				connector->candidate_crtcs;
	}
	if (ended_attachment && dcp->fixed_connector) {
		apple_connector_set_pipeline(dcp->fixed_connector,
					     to_platform_device(dcp->dev));
	}
	dcp->typec_connector = NULL;
	dcp->connector = dcp->fixed_connector;
	WRITE_ONCE(dcp->ext_backlight, false);

	if (dcp->fixed_connector_type != DRM_MODE_CONNECTOR_USB) {
		dcp->connector_type = dcp->fixed_connector_type;

		/*
		 * Only hand the pipeline back to its fixed output if that output
		 * is live.  Re-targeting the DPTX endpoint at the fixed PHY while
		 * nothing is attached there leaves DCP unable to train a link on a
		 * later Type-C target: it answers DEVICE_NOT_STARTED and every
		 * following DPTX call times out.  Park on a Type-C PHY instead,
		 * for the same reason the USB-C-only case does below.
		 */
		if (dcp->hdmi_hpd && gpiod_get_value_cansleep(dcp->hdmi_hpd)) {
			dcp->phy = dcp->fixed_phy;
			dcp->dptx_phy = dcp->fixed_dptx_phy;

			if (dcp->xbar && !dcp->fixed_route_selected) {
				ret = mux_control_select(dcp->xbar,
							 dcp->fixed_mux_index);
				if (ret)
					return ret;
				dcp->fixed_route_selected = true;
			}
		} else {
			dcp->phy = dcp->typec_routes[0].phy;
			dcp->dptx_phy = dcp->typec_routes[0].dptx_phy;
		}
	} else {
		/* Keep DPTX endpoint discovery working before a cable is attached. */
		dcp->phy = dcp->typec_routes[0].phy;
		dcp->dptx_phy = dcp->typec_routes[0].dptx_phy;
		dcp->connector_type = DRM_MODE_CONNECTOR_USB;
	}

	return 0;
}

/*
 * The DRM device once it has bound, that is once every pipeline has its
 * CRTC and every port its connectors; NULL before.
 */
static struct drm_device *dcp_typec_drm(void)
{
	struct apple_dcp_typec_port *port;
	struct apple_dcp_typec_route *route;
	struct drm_device *drm = NULL;

	list_for_each_entry(port, &dcp_typec_ports, link) {
		if (!port->connector || !port->secondary_connector)
			return NULL;
		list_for_each_entry(route, &port->routes, port_link) {
			if (!route->dcp->crtc)
				return NULL;
			drm = route->dcp->crtc->base.dev;
		}
	}

	return drm;
}

/*
 * Has a compositor (or boot splash) taken the display?  It paired its
 * connectors with CRTCs when it started and keeps that pairing, also
 * across a VT switch, where it drops DRM master but keeps the device
 * open; moving routes under it would hand connectors to pipelines driving
 * other displays.  So the routes are frozen while any open file has ever
 * been master, and thaw when the last such file is closed.
 *
 * Taken under dcp_typec_fabric_lock, next to the routing it decides, so
 * the order is the fabric lock, then filelist_mutex.  Nothing nests them
 * the other way: filelist_mutex only covers list edits and walks, and
 * drm_release() drops it before drm_file_free() calls postclose, which is
 * where a close takes the fabric lock.  drm_open() makes a file master
 * before adding it here, but its owner cannot have paired anything before
 * open() returns, and it re-probes on the hotplugs the moves send.
 */
static bool dcp_typec_frozen(struct drm_device *drm)
{
	struct drm_file *file;

	guard(mutex)(&drm->filelist_mutex);
	list_for_each_entry(file, &drm->filelist, lhead) {
		/* set under master_mutex, and only ever from false to true */
		if (READ_ONCE(file->was_master))
			return true;
	}

	return false;
}

/* Are the routes kept in compositor pairing order right now? */
static bool dcp_typec_keep_order(void)
{
	struct drm_device *drm;

	if (!dcp_typec_ordered_dual_stream())
		return false;
	drm = dcp_typec_drm();
	if (!drm || !READ_ONCE(drm->registered))
		return false;
	return dcp_fabric_keep_order(true, true, dcp_typec_frozen(drm));
}

static void dcp_typec_pipeline_freed(void);
static int dcp_fixed_output_select(struct apple_dcp *dcp);

static bool dcp_hdmi_settle_enabled(struct apple_dcp *dcp)
{
	return dcp->hdmi_hpd &&
	       dcp_fabric_hdmi_settle_enabled(!!dcp->fixed_phy, dcp->nr_typec_routes,
					      dcp_typec_ordered_dual_stream());
}

static enum dcp_fabric_presence_state dcp_hdmi_presence(struct apple_dcp *dcp)
{
	enum dcp_fabric_presence_state state;
	unsigned long flags;

	if (!dcp_hdmi_settle_enabled(dcp))
		return DCP_FABRIC_ABSENT;
	spin_lock_irqsave(&dcp->hdmi_presence_lock, flags);
	state = dcp->hdmi_presence.state;
	spin_unlock_irqrestore(&dcp->hdmi_presence_lock, flags);
	return state;
}

/* The hardirq records an edge before any borrower can observe HPD low. */
static u64 dcp_hdmi_edge(struct apple_dcp *dcp)
{
	unsigned long flags;
	u64 generation;

	if (!dcp_hdmi_settle_enabled(dcp))
		return 0;
	spin_lock_irqsave(&dcp->hdmi_presence_lock, flags);
	generation = dcp_fabric_presence_edge(&dcp->hdmi_presence, jiffies,
					      msecs_to_jiffies(DCP_HDMI_HOLD_MS));
	spin_unlock_irqrestore(&dcp->hdmi_presence_lock, flags);
	return generation;
}

static void dcp_hdmi_schedule(struct apple_dcp *dcp)
{
	unsigned long flags, delay = 0;
	bool settling;

	if (!dcp_hdmi_settle_enabled(dcp))
		return;
	spin_lock_irqsave(&dcp->hdmi_presence_lock, flags);
	settling = dcp->hdmi_presence.state == DCP_FABRIC_SETTLING;
	if (settling && time_after(dcp->hdmi_presence.deadline, jiffies))
		delay = dcp->hdmi_presence.deadline - jiffies;
	spin_unlock_irqrestore(&dcp->hdmi_presence_lock, flags);
	if (settling)
		mod_delayed_work(system_freezable_wq, &dcp->hdmi_settle_wq, delay);
	else
		cancel_delayed_work(&dcp->hdmi_settle_wq);
}

static bool dcp_hdmi_sample(struct apple_dcp *dcp, u64 generation, int level)
{
	unsigned long flags;
	bool accepted;

	if (!dcp_hdmi_settle_enabled(dcp))
		return false;
	/* A failed GPIO read cannot establish absence. Keep a full guard. */
	spin_lock_irqsave(&dcp->hdmi_presence_lock, flags);
	accepted = dcp_fabric_presence_sample(&dcp->hdmi_presence, generation,
					      level > 0, jiffies,
					      msecs_to_jiffies(DCP_HDMI_HOLD_MS));
	spin_unlock_irqrestore(&dcp->hdmi_presence_lock, flags);
	dcp_hdmi_schedule(dcp);
	return accepted;
}

static void dcp_hdmi_capacity_available(void *ctx)
{
	dcp_typec_pipeline_freed();
}

static void dcp_hdmi_settle_work(struct work_struct *work)
{
	struct apple_dcp *dcp = container_of(to_delayed_work(work),
					     struct apple_dcp, hdmi_settle_wq);
	unsigned long flags;
	u64 generation;
	bool available;
	int level;

	guard(mutex)(&dcp_typec_fabric_lock);
	if (!dcp_hdmi_settle_enabled(dcp))
		return;
	spin_lock_irqsave(&dcp->hdmi_presence_lock, flags);
	generation = dcp->hdmi_presence.generation;
	spin_unlock_irqrestore(&dcp->hdmi_presence_lock, flags);
	level = gpiod_get_value_cansleep(dcp->hdmi_hpd);
	if (level < 0) {
		dcp_hdmi_sample(dcp, generation, level);
		return;
	}
	spin_lock_irqsave(&dcp->hdmi_presence_lock, flags);
	available = dcp_fabric_presence_expire(&dcp->hdmi_presence, generation,
					       level > 0, jiffies);
	spin_unlock_irqrestore(&dcp->hdmi_presence_lock, flags);
	dcp_hdmi_schedule(dcp);
	/* Notify direct ports once when HDMI capacity becomes available. */
	dcp_fabric_run_capacity(available, dcp_hdmi_capacity_available, dcp);
}

static void dcp_hdmi_recheck_work(struct work_struct *work)
{
	struct apple_dcp *dcp = container_of(to_delayed_work(work),
					     struct apple_dcp, hdmi_recheck_wq);
	unsigned long flags;
	u64 generation;
	bool changed;
	int level;

	/* Oneshot unmask precedes thread completion. Never wait under fabric. */
	synchronize_irq(dcp->hdmi_hpd_irq);
	guard(mutex)(&dcp_typec_fabric_lock);
	if (!dcp_hdmi_settle_enabled(dcp))
		return;
	spin_lock_irqsave(&dcp->hdmi_presence_lock, flags);
	generation = dcp->hdmi_presence.generation;
	spin_unlock_irqrestore(&dcp->hdmi_presence_lock, flags);
	level = gpiod_get_value_cansleep(dcp->hdmi_hpd);
	if (level < 0) {
		dcp_hdmi_sample(dcp, generation, level);
		return;
	}
	spin_lock_irqsave(&dcp->hdmi_presence_lock, flags);
	changed = dcp_fabric_presence_recheck(&dcp->hdmi_presence, generation,
					      level > 0, jiffies,
					      msecs_to_jiffies(DCP_HDMI_HOLD_MS));
	spin_unlock_irqrestore(&dcp->hdmi_presence_lock, flags);
	if (!changed)
		return;
	dcp_hdmi_schedule(dcp);
	dcp_hdmi_update_locked(dcp);
	/* Catch a level change during this update's firmware/debounce waits. */
	mod_delayed_work(system_freezable_wq, &dcp->hdmi_recheck_wq, 0);
}

void dcp_fabric_init(struct apple_dcp *dcp)
{
	spin_lock_init(&dcp->hdmi_presence_lock);
	dcp->hdmi_presence.state = DCP_FABRIC_ABSENT;
	INIT_DELAYED_WORK(&dcp->hdmi_settle_wq, dcp_hdmi_settle_work);
	INIT_DELAYED_WORK(&dcp->hdmi_recheck_wq, dcp_hdmi_recheck_work);
	disable_delayed_work(&dcp->hdmi_settle_wq);
	disable_delayed_work(&dcp->hdmi_recheck_wq);
}

/* Caller enables the HPD IRQ first; a later edge invalidates this sample. */
static u64 dcp_resume_presence_edge(void *ctx)
{
	return dcp_hdmi_edge(ctx);
}

static bool dcp_resume_presence_sample(void *ctx, u64 generation, int level)
{
	return dcp_hdmi_sample(ctx, generation, level);
}

static const struct dcp_fabric_resume_sample_ops dcp_resume_sample_ops = {
	.edge = dcp_resume_presence_edge,
	.read_hpd = dcp_hdmi_read_hpd,
	.sample = dcp_resume_presence_sample,
};

void dcp_fabric_hdmi_resume(struct apple_dcp *dcp)
{
	if (!dcp_hdmi_settle_enabled(dcp))
		return;
	guard(mutex)(&dcp_typec_fabric_lock);
	dcp_fabric_run_resume_sample(true, &dcp_resume_sample_ops, dcp);
}

/*
 * T6030 HDMI output: the ATC PHY that runs it as four-lane DP, and the
 * crossbar it goes through, are in power domains that system sleep may
 * switch off, losing their setup, and the converter is powered through
 * GPIOs. Set them up again as probe did: converter power on, the PHY taken
 * out of DP and back into it, and the crossbar output selected again if the
 * HDMI route holds it. No HDMI link is up when this runs.
 */
static void dcp_fixed_hdmi_reinit_locked(struct apple_dcp *dcp, const char *why)
{
	int ret = 0;

	lockdep_assert_held(&dcp_typec_fabric_lock);
	if (!dcp->external_native || !dcp->fixed_phy || !dcp->hdmi_hpd)
		return;
	dev_info(dcp->dev, "HDMI: setting up the output again (%s)\n", why);
	if (dcp->hdmi_pwren)
		gpiod_set_value_cansleep(dcp->hdmi_pwren, 1);
	if (dcp->dp2hdmi_pwren)
		gpiod_set_value_cansleep(dcp->dp2hdmi_pwren, 1);
	if (dcp->typec_mux && dcp->phy_managed_by_typec) {
		struct typec_altmode alt = { .svid = USB_TYPEC_DP_SID };
		struct typec_mux_state state = { .mode = TYPEC_STATE_SAFE };

		ret = typec_mux_set(dcp->typec_mux, &state);
		if (!ret) {
			state.alt = &alt;
			state.mode = TYPEC_DP_STATE_C;
			ret = typec_mux_set(dcp->typec_mux, &state);
		}
		if (ret)
			dev_warn(dcp->dev, "HDMI: PHY not set up again: %d\n", ret);
	}
	if (dcp->xbar && dcp->fixed_route_selected && !dcp->active_typec_route) {
		scoped_guard(mutex, &dcp->tb_lock)
			dcp->direct_xbar_up = NULL;
		ret = mux_control_deselect(dcp->xbar);
		dcp->fixed_route_selected = false;
		if (!ret)
			ret = dcp_fixed_output_select(dcp);
		if (ret)
			dev_warn(dcp->dev, "HDMI: crossbar output not selected again: %d\n", ret);
	}
}

void dcp_fabric_hdmi_reinit(struct apple_dcp *dcp, const char *why)
{
	guard(mutex)(&dcp_typec_fabric_lock);
	dcp_fixed_hdmi_reinit_locked(dcp, why);
}

/* A Thunderbolt tunnel holds its pipeline; direct moves never steal it. */
static bool dcp_typec_tunnel_held(struct apple_dcp *dcp)
{
	return dcp->active_typec_route && dcp->active_typec_route->tunnel;
}

static bool dcp_tb_services_ready(struct apple_dcp *dcp);

static struct apple_dcp_typec_route *
dcp_fabric_real_route(struct dcp_fabric_route *route)
{
	return route ? container_of(route, struct apple_dcp_typec_route, core) :
		       NULL;
}

static void dcp_fabric_snapshot_port(struct apple_dcp_typec_port *port,
				     bool tunnel)
{
	struct apple_dcp_typec_route *route;
	struct dcp_fabric_route **tail = &port->core.routes;

	lockdep_assert_held(&dcp_typec_fabric_lock);
	port->core.owner[0] = port->owner ? &port->owner->core : NULL;
	port->core.owner[1] =
		port->secondary_owner ? &port->secondary_owner->core : NULL;
	port->core.preferred =
		port->preferred_route ? &port->preferred_route->core : NULL;
	port->core.wanted = port->dp_wanted;
	port->core.hpd = port->dp_hpd;
	port->core.candidate_crtcs[0] =
		port->connector ? port->connector->candidate_crtcs : 0;
	port->core.candidate_crtcs[1] =
		port->secondary_connector ?
			port->secondary_connector->candidate_crtcs :
			0;
	port->core.plan = &port->plan;
	list_for_each_entry(route, &port->routes, port_link) {
		struct apple_dcp *dcp = route->dcp;
		struct dcp_fabric_pipeline *pipeline = &dcp->fabric;

		pipeline->bound = !!dcp->crtc;
		pipeline->crtc_index =
			dcp->crtc ? drm_crtc_index(&dcp->crtc->base) : 0;
		pipeline->has_fixed = !!dcp->fixed_phy;
		pipeline->fixed_busy = dcp_typec_route_busy(route);
		pipeline->owned = !!dcp->active_typec_route ||
				  dcp_typec_safe_held(dcp);
		pipeline->tunnel_held = dcp_typec_tunnel_held(dcp);
		pipeline->presence = dcp_hdmi_presence(dcp);
		pipeline->terminal = tunnel && dcpext_scanout_terminal(dcp);
		pipeline->services_ready = !tunnel ||
					   dcp_tb_services_ready(dcp);
		pipeline->external = dcp->external;
		route->core.pipeline = pipeline;
		route->core.tunnel = route->tunnel;
		route->core.dpin = route->tunnel_dpin;
		route->core.tunnel_clock_blocked = dcp_fabric_tunnel_clock_blocked(
			of_machine_is_compatible("apple,t6030"), route->mux_index);
		*tail = &route->core;
		tail = &route->core.next;
	}
	*tail = NULL;
}

/*
 * The pairing a compositor starting now would make, wherever the streams
 * sit at the moment: pipelines in CRTC index order, each to the first
 * stream that wants it.  A pipeline whose fixed output is live is left to
 * that output, unless a tunnel holds it: the output cannot have it back
 * then, and its connector reads disconnected.
 *
 * Tunnels are planned in connector order like any other stream, not
 * seated on the pipelines they hold: the compositor pairs their connectors
 * by order too, and seating them would plan the direct streams around
 * pairings it never makes.  A direct stream planned onto a pipeline a
 * tunnel holds cannot have it and stays dark, so it is left out and the
 * pass re-run, or every stream after it would be planned one pipeline
 * off.  Streams only ever drop out, so that settles within one pass per
 * port.  It also brings a tunnel's plan onto the pipeline it holds
 * wherever direct streams ahead of it were in the way.  A tunnel still
 * planned elsewhere cannot be brought there by any direct stream dropping
 * out, and the compositor pairs its connector wrongly whatever they do.
 */
static void dcp_typec_plan(struct drm_device *drm,
			   struct apple_dcp_typec_port *arriving, unsigned int dpin)
{
	struct apple_dcp_typec_port *port;
	struct dcp_fabric_port *ports = NULL, **port_tail = &ports;
	struct dcp_fabric_pipeline *pipelines = NULL,
				   **pipeline_tail = &pipelines;
	struct drm_crtc *crtc;

	list_for_each_entry(port, &dcp_typec_ports, link) {
		dcp_fabric_snapshot_port(port, false);
		*port_tail = &port->core;
		port_tail = &port->core.next;
	}
	*port_tail = NULL;
	drm_for_each_crtc(crtc, drm) {
		struct apple_dcp *dcp =
			platform_get_drvdata(to_apple_crtc(crtc)->dcp);

		if (!dcp->nr_typec_routes)
			continue;
		*pipeline_tail = &dcp->fabric;
		pipeline_tail = &dcp->fabric.next;
	}
	*pipeline_tail = NULL;
	dcp_fabric_plan(pipelines, ports, arriving ? &arriving->core : NULL,
			dpin);
	list_for_each_entry(port, &dcp_typec_ports, link) {
		port->target = dcp_fabric_real_route(port->plan.target[0]);
		port->secondary_target =
			dcp_fabric_real_route(port->plan.target[1]);
		port->plan_dark = port->plan.dark;
	}
}

/* Replay HPD to the pipeline a direct DP-alt port has just been given. */
static void dcp_typec_port_attach(struct apple_dcp_typec_port *port)
{
	struct apple_dcp *dcp = port->owner->dcp;

	port->hpd = port->dp_hpd;
	if (!port->hpd)
		return;

	WRITE_ONCE(dcp->typec_cable_connected, true);
	if (dcp->typec_connector)
		dcp_dptx_connect_oob(to_platform_device(dcp->dev), 0);
}

/*
 * The pipeline a port without one takes: the one it last had if that is
 * free, as a compositor keeps a reconnected connector's CRTC, otherwise the
 * best-ranked free one.  Without dual-stream docks a free Type-C-only
 * pipeline comes first even when the port last had the hybrid, which an HDMI
 * display needs, unless the route would only follow its CRTC to the hybrid
 * (see dcp_fabric_score()).
 */
static struct apple_dcp_typec_route *
dcp_typec_free_route(struct apple_dcp_typec_port *port)
{
	struct dcp_fabric_policy policy = {
		.dual_stream = dcp_typec_ordered_dual_stream(),
		.follow = dcp_typec_port_follows(port),
	};

	dcp_fabric_snapshot_port(port, false);
	return dcp_fabric_real_route(dcp_fabric_free_route(&port->core,
							   &policy));
}

/*
 * Route the direct DP-alt ports left waiting for a pipeline.  Their DP
 * state is not reported again while it stays the same, so nothing else
 * would.  Each goes back to the pipeline it last had if that is free, as
 * a compositor keeps a reconnected connector's CRTC, and otherwise takes
 * the lowest free one, the CRTC a compositor gives a new connector.
 */
static void dcp_typec_route_waiting(void)
{
	struct apple_dcp_typec_port *port;
	struct apple_dcp_typec_route *route;

	list_for_each_entry(port, &dcp_typec_ports, link) {
		struct dcp_fabric_port state = {
			.owner[0] = port->owner ? &port->owner->core : NULL,
			.wanted = port->dp_wanted,
			.hpd = port->dp_hpd,
		};

		if (!dcp_fabric_waiting(&state))
			continue;

		route = dcp_typec_free_route(port);
		if (!route || dcp_typec_route_activate(route, route->xbar))
			continue;
		port->owner = route;

		dcp_typec_port_attach(port);
	}
}

/*
 * Prefer connector order when placing Type-C streams before a DRM master
 * starts. This is an allocation policy, not a promise about the CRTC that
 * userspace must choose: each connected encoder advertises only its actual
 * route, and atomic encoder selection checks that ownership. Route changes
 * publish the new mask through hotplug; the plan uses candidate_crtcs so
 * narrowing the advertised mask does not remove hardware routing choices.
 *
 * Only direct DP-alt routes move: each goes to exactly its planned
 * pipeline.  A Thunderbolt tunnel never moves once set up, so where one
 * sits on a pipeline planned for a direct stream, or the plan has nothing
 * for it, that stream stays unrouted and its connector disconnected.
 * Placing it on some other pipeline would have the compositor cross both
 * displays; dark is the better failure.  @arriving/@dpin is a tunnel
 * stream asking for a pipeline, planned like any other; its route is
 * returned for apple_dcp_tb_dp_tunnel() to set up, or NULL if the plan
 * has none for it or another tunnel holds that one.
 *
 * A move is an unplug and replug.  Every route that moves is taken down
 * first, so that two never share a pipeline midway, then each goes up on
 * its new pipeline with its HPD replayed; the hotplugs this sends make
 * fbdev and userspace re-probe.
 */
struct dcp_rebalance_context {
	struct drm_device *drm;
	struct apple_dcp_typec_port *arriving;
	unsigned int dpin;
	struct apple_dcp_typec_route *planned;
};

struct dcp_deactivate_context {
	struct apple_dcp_typec_port *port;
	struct apple_dcp_typec_route *owner;
	struct apple_dcp *dcp;
};

static int dcp_rebalance_route_deactivate(void *data)
{
	struct dcp_deactivate_context *ctx = data;

	return dcp_typec_route_deactivate(ctx->owner);
}

static bool dcp_rebalance_route_selected(void *data)
{
	struct dcp_deactivate_context *ctx = data;

	return ctx->owner->selected;
}

static bool dcp_rebalance_fixed_live(void *data)
{
	struct dcp_deactivate_context *ctx = data;
	struct apple_dcp *dcp = ctx->dcp;

	return dcp->hdmi_hpd && dcp->active && gpiod_get_value_cansleep(dcp->hdmi_hpd);
}

static void dcp_rebalance_keep_owner(void *data)
{
	struct dcp_deactivate_context *ctx = data;

	dcp_typec_port_attach(ctx->port);
}

static void dcp_rebalance_clear_owner(void *data)
{
	struct dcp_deactivate_context *ctx = data;

	ctx->port->owner = NULL;
}

static void dcp_rebalance_connect_fixed(void *data)
{
	struct dcp_deactivate_context *ctx = data;

	dcp_dptx_connect(ctx->dcp, 0);
}

static const struct dcp_fabric_deactivate_ops dcp_rebalance_deactivate_ops = {
	.deactivate = dcp_rebalance_route_deactivate,
	.selected = dcp_rebalance_route_selected,
	.fixed_live = dcp_rebalance_fixed_live,
	.keep_owner = dcp_rebalance_keep_owner,
	.clear_owner = dcp_rebalance_clear_owner,
	.connect_fixed = dcp_rebalance_connect_fixed,
};

static void dcp_rebalance_plan(void *data)
{
	struct dcp_rebalance_context *ctx = data;

	dcp_typec_plan(ctx->drm, ctx->arriving, ctx->dpin);
	if (ctx->arriving) {
		ctx->planned = ctx->dpin ? ctx->arriving->secondary_target : ctx->arriving->target;
		if (ctx->planned && dcp_typec_tunnel_held(ctx->planned->dcp)) {
			ctx->planned = NULL;
			dcp_typec_plan(ctx->drm, NULL, 0);
		}
	}
}

static void *dcp_rebalance_first(void *data)
{
	return list_first_entry_or_null(&dcp_typec_ports, struct apple_dcp_typec_port, link);
}

static void *dcp_rebalance_next(void *data, void *entry)
{
	struct apple_dcp_typec_port *port = entry;

	return list_is_last(&port->link, &dcp_typec_ports) ? NULL : list_next_entry(port, link);
}

static bool dcp_rebalance_deactivate(void *data, void *entry)
{
	struct apple_dcp_typec_port *port = entry;
	struct apple_dcp_typec_route *owner = port->owner;
	struct apple_dcp *dcp;
	struct dcp_deactivate_context ctx;
	int ret;

	/* A port in SAFE keeps its owner until the set phase completes. */
	if (port->safe_pending)
		return false;
	if (!dcp_fabric_movable(owner ? &owner->core : NULL,
				port->target ? &port->target->core : NULL))
		return false;
	dcp = owner->dcp;
	dev_info(dcp->dev, "re-routing %pOF from %s to %s for Type-C connector order\n",
		 port->connector_np, dev_name(dcp->dev),
		 port->target ? dev_name(port->target->dcp->dev) : "none");
	if (port->hpd || dcp->typec_cable_connected ||
	    (dcp->typec_connector && dcp->typec_connector->connected)) {
		ret = dcp_dptx_disconnect_oob(to_platform_device(dcp->dev), 0);
		/* H17P external pipes keep their own stop recovery, as before. */
		if (ret && !dcp_is_external_h17p(dcp))
			return true;
	}
	port->hpd = false;
	ctx.port = port;
	ctx.owner = owner;
	ctx.dcp = dcp;
	return dcp_fabric_run_deactivate(&dcp_rebalance_deactivate_ops, &ctx);
}

static bool dcp_rebalance_activate(void *data, void *entry)
{
	struct dcp_rebalance_context *ctx = data;
	struct apple_dcp_typec_port *port = entry;
	struct apple_dcp_typec_route *route = port->target;

	if (port->owner || !port->dp_wanted || !port->dp_hpd || port == ctx->arriving)
		return false;
	if (!route || !dcp_typec_route_available(route)) {
		port->applied_valid = false;
		return false;
	}
	if (dcp_typec_route_activate(route, route->xbar)) {
		port->applied_valid = false;
		return true;
	}
	port->owner = route;
	port->preferred_route = route;
	dcp_typec_port_attach(port);
	return false;
}

static const struct dcp_fabric_rebalance_ops dcp_rebalance_ops = {
	.plan = dcp_rebalance_plan,
	.first = dcp_rebalance_first,
	.next = dcp_rebalance_next,
	.deactivate = dcp_rebalance_deactivate,
	.activate = dcp_rebalance_activate,
};

static struct apple_dcp_typec_route *
dcp_typec_rebalance_locked(struct apple_dcp_typec_port *arriving, unsigned int dpin)
{
	struct dcp_rebalance_context ctx = {
		.drm = dcp_typec_drm(),
		.arriving = arriving,
		.dpin = dpin,
	};

	lockdep_assert_held(&dcp_typec_fabric_lock);
	dcp_fabric_run_rebalance(&dcp_rebalance_ops, &ctx);
	return ctx.planned;
}

static bool dcp_typec_hdmi_reclaim(struct apple_dcp *dcp);

/*
 * A display is plugged into the HDMI port while its pipeline, the hybrid,
 * drives a Type-C port.  Without dual-stream docks nothing moves for it: a
 * compositor keeps its connector-to-CRTC pairing and could miss a moved
 * display's brief unplug.  The HDMI display waits and is handed the hybrid
 * when the Type-C display lets it go (see dcp_typec_route_deactivate()),
 * unless the Type-C display can be moved out of its way (see
 * dcp_typec_hdmi_reclaim()).
 */
static void dcp_typec_hdmi_waits(struct apple_dcp *dcp)
{
	struct apple_dcp_typec_route *owner = READ_ONCE(dcp->active_typec_route);

	lockdep_assert_held(&dcp_typec_fabric_lock);

	if (owner && dcp_typec_hdmi_reclaim(dcp))
		return;
	if (owner)
		dev_info(dcp->dev, "HDMI display waits: its pipeline drives the %s on %pOF\n",
			 owner->tunnel ? "Thunderbolt display" : "display",
			 owner->port->connector_np);
}

/*
 * A route has let its pipeline go.  Until a compositor owns the display of a
 * dual-stream machine the plan places everything anew; otherwise the
 * pipeline goes to a port left waiting for one.  The caller has already
 * handed a hybrid back to a live HDMI display.
 */
static void dcp_typec_pipeline_freed(void)
{
	if (dcp_fabric_capacity_action(dcp_typec_keep_order()) == DCP_FABRIC_REBALANCE)
		dcp_typec_rebalance_locked(NULL, 0);
	else
		dcp_typec_route_waiting();
}

/*
 * Re-run the pairing pass from outside the fabric: once DRM is registered,
 * as ports routed before that could not follow it, and when the last
 * compositor or boot splash has closed the device, as the next one pairs
 * the connectors from scratch.
 */
void dcp_typec_reorder(void)
{
	if (!dcp_typec_ordered_dual_stream())
		return;

	guard(mutex)(&dcp_typec_fabric_lock);

	if (dcp_typec_keep_order())
		dcp_typec_rebalance_locked(NULL, 0);
}

static int dcp_typec_route_prepare(struct typec_mux_dev *mux,
				   struct typec_mux_state *state)
{
	struct apple_dcp_typec_route *route = typec_mux_get_drvdata(mux);
	struct apple_dcp_typec_port *port = route->port;
	struct apple_dcp *dcp;
	int ret;

	guard(mutex)(&dcp_typec_fabric_lock);
	if (state->mode != TYPEC_STATE_SAFE)
		return port->safe_pending ? -EAGAIN : 0;
	if (!port->owner || port->owner->tunnel)
		return 0;
	dcp = port->owner->dcp;
	/* Stop new admission, but retain the owner until the set phase succeeds. */
	if (!port->safe_pending)
		port->safe_finish_ready = false;
	port->safe_pending = true;
	port->applied_valid = false;
	port->dp_wanted = false;
	port->dp_hpd = false;
	/* HPD absence alone is not proof of a completed firmware shutdown. */
	if (dcp_is_external_h17p(dcp)) {
		scoped_guard(mutex, &dcp->external_lock) {
			if (dcp->external_error)
				return dcp->external_error;
			if (!dcp->external_cable && dcp->external_stopped &&
			    !READ_ONCE(dcp->dptxport[0].connected)) {
				port->hpd = false;
				return 0;
			}
		}
	}
	ret = dcp_dptx_disconnect_oob(to_platform_device(dcp->dev), 0);
	if (ret)
		return ret;
	port->hpd = false;
	return 0;
}

static int dcp_typec_route_finish(struct typec_mux_dev *mux,
				  struct typec_mux_state *state)
{
	struct apple_dcp_typec_port *port =
		((struct apple_dcp_typec_route *)typec_mux_get_drvdata(mux))->port;
	int ret;

	guard(mutex)(&dcp_typec_fabric_lock);
	if (state->mode != TYPEC_STATE_SAFE || !port->safe_pending ||
	    port->safe_finish_ready)
		return 0;
	ret = port->owner ? dcp_typec_route_deactivate(port->owner) : 0;
	if (ret)
		return ret;
	port->safe_finish_ready = true;
	return 0;
}

static void dcp_typec_route_commit(struct typec_mux_dev *mux,
				   struct typec_mux_state *state)
{
	struct apple_dcp_typec_port *port =
		((struct apple_dcp_typec_route *)typec_mux_get_drvdata(mux))->port;

	guard(mutex)(&dcp_typec_fabric_lock);
	if (state->mode != TYPEC_STATE_SAFE || !port->safe_pending)
		return;
	if (port->owner)
		port->preferred_route = port->owner;
	port->owner = NULL;
	port->safe_pending = false;
	port->safe_finish_ready = false;
	port->applied_alt = state->alt;
	port->applied_mode = state->mode;
	port->applied_status = 0;
	port->applied_conf = 0;
	port->applied_valid = true;
	dcp_typec_pipeline_freed();
}

/*
 * A DP display on @port gets no pipeline: every one that could drive it
 * drives another display. Say so once per plug, with the display's HPD up;
 * the port controller only logs the -ENOSPC. When one of those displays goes,
 * the freed pipeline takes a waiting display without a replug (see
 * dcp_typec_pipeline_freed()).
 */
static int dcp_typec_port_busy(struct apple_dcp_typec_port *port,
			       struct apple_dcp *dcp)
{
	const char *label = NULL;

	/* This tree reports a port without a pipeline as -ENOSPC. */
	if (port->busy_noted)
		return -ENOSPC;
	port->busy_noted = true;
	of_property_read_string(port->connector_np, "label", &label);
	dev_info(dcp->dev,
		 "display on %s waits for a free external display pipeline (each one drives another display); it lights up on its own when one of those displays is unplugged\n",
		 label ?: "a USB-C port");
	return -ENOSPC;
}

static int dcp_typec_route_set(struct typec_mux_dev *mux,
			       struct typec_mux_state *state)
{
	struct apple_dcp_typec_route *route = typec_mux_get_drvdata(mux);
	struct apple_dcp_typec_port *port = route->port;
	struct apple_dcp_typec_route *best = NULL;
	bool is_dp = dcp_typec_route_is_dp(state);
	struct typec_displayport_data *dp_data = is_dp ? state->data : NULL;
	u32 dp_status = dp_data ? dp_data->status : 0;
	u32 dp_conf = dp_data ? dp_data->conf : 0;
	bool hpd = dp_data && (dp_data->status & DP_STATUS_HPD_STATE);
	bool was_counted;
	int ret = 0;

	/* A failed T8122 dcpext drives nothing; USB on the port goes on. */
	if (READ_ONCE(route->dcp->ext_dead))
		return 0;

	guard(mutex)(&dcp_typec_fabric_lock);

	if (state->mode == TYPEC_STATE_SAFE && port->safe_pending)
		return 0;
	/* The previous SAFE transition has not finished; retry. */
	if (port->safe_pending)
		return -EAGAIN;

	/*
	 * Every candidate route for this port is notified with the same state,
	 * so only the first to arrive does the work; the others return early.
	 * That includes a sink's IRQ_HPD request: the port controller sends a
	 * new one as a changed status, clearing IRQ_HPD first when the route
	 * still holds the previous request (cd321x_dp_send()).
	 *
	 * Deliberately not a nominated coordinator: fwnode_typec_mux_get()
	 * caps the providers one connector may have and drops the remainder
	 * without a word, so a nominated route might never be called at all --
	 * and the port would then never be routed.
	 */
	if (port->applied_valid && port->applied_alt == state->alt &&
	    port->applied_mode == state->mode &&
	    port->applied_status == dp_status && port->applied_conf == dp_conf)
		return 0;

	port->applied_alt = state->alt;
	port->applied_mode = state->mode;
	port->applied_status = dp_status;
	port->applied_conf = dp_conf;
	/* Failed route acquisition must remain retryable on the next update. */
	port->applied_valid = false;
	/* Keep a direct route until DCP acknowledges release of its display. */
	if (!hpd && port->owner && !port->owner->tunnel) {
		struct apple_dcp *dcp = port->owner->dcp;

		if (port->hpd || dcp->typec_cable_connected ||
		    (dcp->typec_connector && dcp->typec_connector->connected)) {
			ret = dcp_dptx_disconnect_oob(to_platform_device(dcp->dev), 0);
			/* H17P external pipes keep their own stop recovery, as before. */
			if (ret && !dcp_is_external_h17p(dcp))
				return ret;
		}
	}

	/* did the pairing pass count this port's direct stream so far? */
	was_counted = port->dp_wanted && port->dp_hpd;

	if (!is_dp) {
		port->dp_wanted = false;
		port->dp_hpd = false;
		port->busy_noted = false;

		/* a Thunderbolt/USB4 DP tunnel is torn down by its own path */
		if (port->owner && port->owner->tunnel) {
			port->applied_valid = true;
			return 0;
		}
		if (port->owner) {
			struct apple_dcp *dcp = port->owner->dcp;

			port->preferred_route = port->owner;
			port->hpd = false;
			ret = dcp_typec_route_deactivate(port->owner);
			if (ret)
				return ret;
			port->owner = NULL;

			if (dcp->hdmi_hpd && dcp->active &&
			    gpiod_get_value_cansleep(dcp->hdmi_hpd))
				dcp_dptx_connect(dcp, 0);
			dcp_typec_pipeline_freed();
		} else if (was_counted && dcp_typec_keep_order()) {
			/* a stream the plan left dark is gone: plan the rest */
			dcp_typec_rebalance_locked(NULL, 0);
		}

		port->safe_pending = false;
		/* Data-only USB4 changes do not invalidate other display routes. */
		port->applied_valid = true;
		return 0;
	}

	/*
	 * A Thunderbolt DP tunnel still owns the port (its teardown is on the
	 * way): don't act on or remember this state; the tunnel teardown drops
	 * what was recorded so the next update is applied.
	 */
	if (port->owner && port->owner->tunnel) {
		port->applied_valid = false;
		return 0;
	}

	port->dp_wanted = true;
	port->dp_hpd = hpd;

	/*
	 * Until a compositor owns the display, a direct DP-alt stream has a
	 * pipeline only while its sink asserts HPD, and the pairing pass
	 * places it: HPD coming or going is the stream connecting or
	 * disconnecting as far as a compositor can tell.  Otherwise the port
	 * takes the lowest free pipeline on DP entry, as a compositor does.
	 */
	if (dcp_typec_keep_order()) {
		struct apple_dcp_typec_route *owner = port->owner;

		/* it connects or disconnects, or is not routed as it should be */
		if (hpd != was_counted || !owner != !hpd)
			dcp_typec_rebalance_locked(NULL, 0);
		if (!port->owner) {
			if (hpd)
				return dcp_typec_port_busy(port, route->dcp);
			port->applied_valid = true;
			return 0;
		}
		/* just attached, its HPD replayed: nothing left to apply */
		if (port->owner != owner) {
			port->applied_valid = true;
			return 0;
		}
	}

	if (!port->owner) {
		best = dcp_typec_free_route(port);
		/* Every pipeline is driving another port's display. */
		if (!best)
			return hpd ? dcp_typec_port_busy(port, route->dcp) : -ENOSPC;

		ret = dcp_typec_route_activate(best, best->xbar);
		if (ret)
			return ret;
		port->owner = best;
		/* a later wait of this plug is reported again */
		port->busy_noted = false;
	}

	if (hpd && !port->hpd) {
		struct apple_dcp *dcp = port->owner->dcp;

		WRITE_ONCE(dcp->typec_cable_connected, true);
		if (dcp->typec_connector)
			dcp_dptx_connect_oob(to_platform_device(dcp->dev), 0);
	} else if (hpd && dp_data && (dp_data->status & DP_STATUS_IRQ_HPD)) {
		struct apple_dcp *dcp = port->owner->dcp;

		/*
		 * A T6030 external processor services the request itself, and
		 * reads the display again if it changed; elsewhere the active
		 * mode is applied again. H17P always applies it again. A running
		 * relink connects the sink anew; see dcp_typec_sink_relink().
		 */
		if (dcp->typec_connector && dcp->fw_compat == DCP_FIRMWARE_H17P)
			dcp_retrain_oob(dcp->typec_connector);
		else if (dcp->typec_connector && dcp->external_native)
			dcp_external_sink_irq(dcp);
		else if (dcp->typec_connector && !dcp_typec_sink_relink(dcp)) {
			if (!READ_ONCE(dcp->typec_connector->connected)) {
				ret = dcp_dptx_recover_irq(dcp);
				if (ret)
					return ret;
			} else {
				dcp_retrain_oob(dcp->typec_connector);
			}
		}
	}
	port->hpd = hpd;
	port->applied_valid = true;

	return 0;
}

/* Limited to the T6030 14.7 pipeline; other T602X tunnels keep their order. */
static bool dcp_t602x_tunnel_route(struct apple_dcp_typec_route *route)
{
	return route && route->tunnel && route->active_xbar &&
		route->dcp->fw_compat == DCP_FIRMWARE_V_14_7 &&
		of_device_is_compatible(route->active_xbar->chip->dev.parent->of_node,
					"apple,t6020-display-crossbar");
}

/*
 * Crossbar connection up/down, looked up at runtime so appledrm does not
 * require the crossbar driver to be built.
 */
static int dcp_dpxbar_link(struct mux_control *mux, bool up)
{
	typeof(&apple_dpxbar_link_up) fn;
	int ret;

	fn = up ? symbol_get(apple_dpxbar_link_up) :
		  symbol_get(apple_dpxbar_link_down);
	if (!fn)
		return -ENOENT;
	ret = fn(mux);
	if (up)
		symbol_put(apple_dpxbar_link_up);
	else
		symbol_put(apple_dpxbar_link_down);
	return ret;
}

static int dcp_dpxbar_preselect(struct mux_control *mux, int state)
{
	typeof(&apple_dpxbar_preselect) fn = symbol_get(apple_dpxbar_preselect);
	int ret;

	if (!fn)
		return -ENOENT;
	ret = fn(mux, state);
	symbol_put(apple_dpxbar_preselect);
	return ret;
}

/*
 * Thunderbolt DP IN, before DCP hears about the display: point the DP IN
 * output at this pipeline and wake the ATC's DP clock path. The crossbar
 * connection itself still only comes up in dcp_tunnel_crossbar_up(), but an
 * output left at its idle source (dispext0) only suits the pipeline that is
 * dispext0: any other one activates the link, gets no answer from the sink
 * and gives up with DEVICE_NOT_RESPONDING before it ever sets a link rate.
 */
static void dcp_tunnel_prepare(struct apple_dcp_typec_route *route,
			       struct mux_control *xbar)
{
	typeof(&apple_atc_dp_tunnel_open) open;
	struct apple_dcp *dcp = route->dcp;
	int ret;

	ret = dcp_dpxbar_preselect(xbar, route->mux_index);
	if (ret && ret != -EOPNOTSUPP)
		dev_warn(dcp->dev, "DP tunnel crossbar preselect failed: %d\n", ret);

	open = symbol_get(apple_atc_dp_tunnel_open);
	if (!open)
		return;
	ret = open(dcp->phy);
	symbol_put(apple_atc_dp_tunnel_open);
	if (ret && ret != -EOPNOTSUPP)
		dev_warn(dcp->dev, "DP tunnel PHY open failed: %d\n", ret);
}

/* DP IN adapter handshake through the thunderbolt glue; tb_lock held */
static int dcp_tunnel_dpin_locked(struct apple_dcp *dcp, bool active)
{
	lockdep_assert_held(&dcp->tb_lock);
	/* Only the binding admitted for the route DCP drives is called. */
	if (!dcp->active_typec_route ||
	    !dcp_fabric_callback_valid(dcp->active_typec_route->tunnel_generation,
				       dcp->tb_generation))
		return -ESTALE;
	if (!dcp->tb_dpin_set_active)
		return -ENODEV;
	return dcp->tb_dpin_set_active(dcp->tb_dpin_ctx, active);
}

/*
 * Thunderbolt DP IN, from DCP's DidChangeLinkConfiguration once a link rate
 * is set: bring the crossbar connection up (FIFO/PCLK/ATC enables) now that
 * the tunnel pixel clock runs, and re-assert the DP IN adapter's
 * DPTX_INACTIVE=0 afterwards. The first time this is the mux selection; after
 * a re-link only the clocks are brought back (the mux selection and the ATC
 * output enable are kept, see dcp_tunnel_crossbar_down()).
 * Runs inside a DCP apcall: only tb_lock, which nobody holds across a DCP call.
 */
static int dcp_tunnel_split_up_locked(struct apple_dcp *dcp)
{
	int ret;

	lockdep_assert_held(&dcp->tb_lock);
	if (!dcp->split.active || !dcp->split.generation || !dcp->split.xbar)
		return -ENODEV;
	if (!dcp->split.clock_ok) {
		dev_warn(dcp->dev, "tiled: no DP tunnel pixel clock, dpin1 crossbar left down\n");
		return -EIO;
	}
	if (!dcp->split.xbar_up) {
		/* never block a DCP call on the mux semaphore */
		ret = mux_control_try_select(dcp->split.xbar, dcp->split.mux_state);
		if (!ret)
			dcp->split.xbar_up = true;
	} else {
		ret = dcp_dpxbar_link(dcp->split.xbar, true);
	}
	if (ret) {
		dev_warn(dcp->dev, "tiled: dpin1 crossbar up failed: %d\n", ret);
		return ret;
	}
	if (!dcp->split.set_active)
		return -ENODEV;
	return dcp->split.set_active(dcp->split.binding, true);
}

int dcp_tunnel_crossbar_up(struct apple_dcp *dcp, u32 unit)
{
	struct apple_dcp_typec_route *route;
	int ret;

	guard(mutex)(&dcp->tb_lock);
	if (unit)
		return dcp_tunnel_split_up_locked(dcp);
	route = dcp->active_typec_route;
	if (!route || !route->tunnel || !route->active_xbar ||
	    route->deactivate_xbar_done)
		return -ENODEV;
	if (!dcp_fabric_callback_valid(route->tunnel_generation, dcp->tb_generation))
		return -ESTALE;
	if (!dcp->tb_clock_ok) {
		dev_warn(dcp->dev, "no DP tunnel pixel clock, crossbar left down\n");
		return -EIO;
	}
	if (!route->xbar_up) {
		/* never block a DCP call on the mux semaphore */
		ret = mux_control_try_select(route->active_xbar, route->mux_index);
		if (!ret)
			route->xbar_up = true;
		/* T602X DP IN selection reserves and routes the mux only. */
		if (!ret && dcp_t602x_tunnel_route(route))
			ret = dcp_dpxbar_link(route->active_xbar, true);
	} else {
		ret = dcp_dpxbar_link(route->active_xbar, true);
	}
	if (ret) {
		dev_warn(dcp->dev, "DP tunnel crossbar up failed: %d\n", ret);
		return ret;
	}
	return dcp_tunnel_dpin_locked(dcp, true);
}

/*
 * Thunderbolt DP IN, from DCP's WillChangeLinkConfiguration on an established
 * link: DP IN inactive, crossbar clocks down (mux selection and ATC output
 * enable kept). DP IN goes active again in dcp_tunnel_crossbar_up().
 */
int dcp_tunnel_crossbar_down(struct apple_dcp *dcp, u32 unit)
{
	struct apple_dcp_typec_route *route;

	guard(mutex)(&dcp->tb_lock);
	if (unit) {
		if (!dcp->split.active || !dcp->split.xbar_up)
			return 0;
		if (dcp->split.set_active)
			dcp->split.set_active(dcp->split.binding, false);
		return dcp_dpxbar_link(dcp->split.xbar, false);
	}
	route = dcp->active_typec_route;
	if (!route || !route->tunnel || !route->xbar_up)
		return 0;
	if (!dcp_fabric_callback_valid(route->tunnel_generation, dcp->tb_generation))
		return -ESTALE;
	dcp_tunnel_dpin_locked(dcp, false);
	return dcp_dpxbar_link(route->active_xbar, false);
}

/*
 * T6030 and T6031 route a direct DP PHY output of their crossbar on select
 * and clock it only while a link runs, as they do their DP IN outputs. From
 * DCP's link configuration calls: clocks up once a link is configured at a
 * nonzero rate, down before the rate changes and when DCP deactivates the
 * port.
 * The output is the borrowed Type-C route's, or the fixed route's. Other
 * SoCs clock it at select. Runs inside a DCP apcall: only tb_lock.
 */
int dcp_direct_crossbar_link(struct apple_dcp *dcp, bool up)
{
	struct apple_dcp_typec_route *route;
	struct mux_control *xbar = NULL;
	int ret;

	/* The crossbar driver runs both in its T6030 mode (t6030_dpin). */
	if (!of_machine_is_compatible("apple,t6030") &&
	    !of_machine_is_compatible("apple,t6031"))
		return 0;
	guard(mutex)(&dcp->tb_lock);
	route = dcp->active_typec_route;
	if (route && !route->tunnel && route->selected)
		xbar = route->active_xbar ?: route->xbar;
	else if (!route && dcp->fixed_route_selected)
		xbar = dcp->xbar;
	/* A deselected output went idle, and its clocks with it. */
	if (dcp->direct_xbar_up && dcp->direct_xbar_up != xbar)
		dcp->direct_xbar_up = NULL;
	if (!xbar || !!dcp->direct_xbar_up == up)
		return 0;
	ret = dcp_dpxbar_link(xbar, up);
	if (ret) {
		dev_warn(dcp->dev, "DP crossbar output %s failed: %d\n", up ? "up" : "down", ret);
		/* Only a running output can be taken down again. */
		if (up)
			return ret;
	}
	dcp->direct_xbar_up = up ? xbar : NULL;
	return ret;
}

/*
 * Thunderbolt DP IN, from DCP's SetLinkRate: start (rate != 0) or stop the
 * tunnel pixel clock. A stopped clock also takes the crossbar connection down.
 */
int dcp_tunnel_set_rate(struct apple_dcp *dcp, struct phy *phy, u32 unit,
			u32 link_rate)
{
	struct apple_dcp_typec_route *route;
	int ret;

	guard(mutex)(&dcp->tb_lock);
	if (!dcp->dptx_tunnel)
		return -ENODEV;
	route = dcp->active_typec_route;
	if (!route)
		return -ENODEV;
	if (!dcp_fabric_callback_valid(route->tunnel_generation, dcp->tb_generation))
		return -ESTALE;
	if (of_machine_is_compatible("apple,t6030") &&
	    (unit || dcp->dptx_dfp_port != 1 + route->tunnel_dpin ||
	     apple_dp_tunnel_t6030_pclk(route->mux_index,
				      route->tunnel_dpin) < 0))
		return -EOPNOTSUPP;
	if (unit) {
		/* Each DP IN owns a reference, including on a shared clock. */
		ret = 0;
		if (!dcp->split.active || !dcp->split.generation) {
			ret = -ENODEV;
		} else {
			if (!link_rate && dcp->split.xbar_up)
				dcp_dpxbar_link(dcp->split.xbar, false);
			ret = dcp_route_clock_rate(route, phy, 1, link_rate);
		}
		dcp->split.clock_ok = !ret && link_rate;
		if (ret)
			dev_warn(dcp->dev, "tiled: dpin1 pixel clock (rate 0x%x) failed: %d\n",
				 link_rate, ret);
		return ret;
	}
	if (!link_rate && route->xbar_up)
		dcp_dpxbar_link(route->active_xbar, false);
	ret = dcp_route_clock_rate(route, phy, route->tunnel_dpin, link_rate);
	dcp->tb_clock_ok = !ret && link_rate;
	if (ret)
		dev_warn(dcp->dev, "DP tunnel pixel clock (rate 0x%x) failed: %d\n",
			 link_rate, ret);
	return ret;
}

/* Thunderbolt DP IN: DCP Activate/Deactivate */
int dcp_tunnel_dpin_activate(struct apple_dcp *dcp, u32 unit, bool active)
{
	struct apple_dcp_typec_route *route;
	int ret;

	guard(mutex)(&dcp->tb_lock);
	if (!dcp->dptx_tunnel)
		return 0;
	if (unit) {
		if (!dcp->split.active || !dcp->split.generation || !dcp->split.set_active)
			return 0;
		return dcp->split.set_active(dcp->split.binding, active);
	}
	/* Route the upstream engine before firmware begins AUX negotiation.
	 * T602X DP IN .set does not enable clocks; DidChange does that later.
	 */
	route = dcp->active_typec_route;
	if (!route || !dcp_fabric_callback_valid(route->tunnel_generation, dcp->tb_generation))
		return -ESTALE;
	if (active && dcp_t602x_tunnel_route(route)) {
		if (!route->xbar_up) {
			ret = mux_control_try_select(route->active_xbar, route->mux_index);
			if (ret)
				return ret;
			route->xbar_up = true;
		}
	}
	return dcp_tunnel_dpin_locked(dcp, active);
}

/*
 * Thunderbolt DP tunnels: the host router's DP IN adapters sit behind the
 * crossbar's dpin0/dpin1 outputs of the port's ATC. When the Thunderbolt
 * connection manager has set up a tunnel from one of them, route a free
 * display pipeline there and tell DCP a display is attached, so it trains
 * the link (and completes DPRX) through the tunnel.
 */
static bool dcp_tb_services_ready(struct apple_dcp *dcp)
{
	if (!dcp->external || dcp->fw_compat != DCP_FIRMWARE_V_14_7)
		return true;
	/* A native processor starts for the tunnel; the tunnel retries meanwhile. */
	if (dcp->external_native)
		return iomfb_v14_7_external_ready(dcp);
	/* Pairs with DPTX RemotePort publication before tunnel acquisition. */
	return smp_load_acquire(&dcp->dptxport[0].enabled) && ibootep_is_ready(dcp);
}

struct dcp_tb_attach_context {
	u64 generation;
	struct apple_dcp_typec_port *port;
	struct apple_dcp_typec_route **slot;
	struct apple_dcp_typec_route *best;
	struct apple_dcp_typec_route *planned;
	struct mux_control *ctl;
	unsigned int dpin;
	bool active;
	int (*set_active)(void *binding, bool active);
	void *binding;
};

/* The crossbar control feeding DP IN @dpin of @route's port to its pipeline */
static struct mux_control *
dcp_typec_tunnel_ctl(struct apple_dcp_typec_route *route, unsigned int dpin)
{
	if (dpin > 1 ||
	    (dcp_fabric_tunnel_clock_blocked(of_machine_is_compatible("apple,t6030"),
					     route->mux_index) & BIT(dpin)))
		return NULL;
	if (route->dpin[dpin])
		return route->dpin[dpin];
	/* Legacy DT ABI: unnamed DPIN controls share the DP-alt chip. */
	if (route->xbar != &route->xbar->chip->mux[0] ||
	    route->xbar->chip->controllers < 3)
		return NULL;
	return &route->xbar->chip->mux[1 + dpin];
}

static int dcp_tb_candidate(void *data)
{
	struct dcp_tb_attach_context *ctx = data;
	struct apple_dcp_typec_port *port = ctx->port;
	struct apple_dcp_typec_route *best, *planned = NULL;
	struct mux_control *ctl;
	unsigned int dpin = ctx->dpin;
	bool ordered;
	int ret;

	/*
	 * Until a compositor owns the display, the pairing pass decides where
	 * the stream goes, moving direct DP-alt routes out of its way, and
	 * whether it gets a pipeline at all.
	 */
	ordered = dcp_typec_keep_order();
	if (ordered)
		planned = dcp_typec_rebalance_locked(port, dpin);
	ctx->planned = planned;

	{
		struct dcp_fabric_policy policy = {
			.dual_stream = dcp_typec_ordered_dual_stream(),
			.follow = dcp_typec_port_follows(port),
		};
		struct dcp_fabric_route *chosen;
		bool connector_present = dpin ? !!port->secondary_connector : !!port->connector;

		dcp_fabric_snapshot_port(port, true);
		chosen = dcp_fabric_tunnel_candidate(&port->core,
						     &policy, planned ? &planned->core : NULL,
						     ordered, dpin,
						     connector_present,
						     &ret);
		best = dcp_fabric_real_route(chosen);
		if (!best)
			return ret;
	}

	ctl = dcp_typec_tunnel_ctl(best, dpin);
	if (!ctl)
		return -EOPNOTSUPP;

	ctx->best = best;
	ctx->ctl = ctl;
	return 0;
}

static int dcp_tb_activate(void *data)
{
	struct dcp_tb_attach_context *ctx = data;
	struct apple_dcp *dcp = ctx->best->dcp;

	/* Admitted before the route goes live: Activate may run before we return. */
	scoped_guard(mutex, &dcp->tb_lock) {
		dcp->tb_dpin_set_active = ctx->set_active;
		dcp->tb_dpin_ctx = ctx->binding;
		dcp->tb_generation = ctx->generation;
		ctx->best->tunnel_generation = ctx->generation;
	}
	ctx->best->tunnel_dpin = ctx->dpin;
	return dcp_typec_route_activate(ctx->best, ctx->ctl);
}

static void dcp_tb_rollback(void *data)
{
	struct dcp_tb_attach_context *ctx = data;
	struct apple_dcp *dcp = ctx->best->dcp;

	/* The failed attempt's binding is retired before the provider hears of it. */
	guard(mutex)(&dcp->tb_lock);
	dcp->tb_dpin_set_active = NULL;
	dcp->tb_dpin_ctx = NULL;
	dcp->tb_generation = 0;
	ctx->best->tunnel_generation = 0;
}

static void dcp_tb_claim(void *data)
{
	struct dcp_tb_attach_context *ctx = data;
	struct apple_dcp_typec_route *best = ctx->best;
	struct apple_dcp_typec_port *port = ctx->port;
	struct apple_dcp *dcp = best->dcp;
	unsigned int dpin = ctx->dpin;

	*ctx->slot = best;
	if (dpin == 0)
		port->tile_hint_deadline = jiffies + msecs_to_jiffies(2000);
	/* the port is in USB4 mode, not DP-alt */
	port->dp_wanted = false;
	dcp_tunnel_prepare(best, ctx->ctl);

	dev_info(dcp->dev, "display routed to Thunderbolt DP tunnel dpin%u\n", dpin);

	if (dcp->fw_compat == DCP_FIRMWARE_V_14_7 && dcp->dptxep &&
	    !dcp->dptxport[0].enabled)
		dev_warn(dcp->dev, "DPTX port not announced, not opening the controller\n");

	/*
	 * The DP IN adapter may only be woken (DPTX_INACTIVE=0) while DCP
	 * drives the DPTX, i.e. from DCP's Activate call; waking it earlier
	 * hangs the machine. dptxep calls set_active back from Activate and
	 * Deactivate (set above, before the route became a tunnel).
	 */
	if (!dcp->typec_connector && !dcp->external)
		dev_warn(dcp->dev, "no Type-C connector for the DP tunnel\n");
	WRITE_ONCE(dcp->typec_cable_connected, true);
	port->hpd = true;
}

static bool dcp_tb_external(void *data)
{
	struct dcp_tb_attach_context *ctx = data;

	return ctx->best->dcp->external;
}

static bool dcp_tb_has_connector(void *data)
{
	struct dcp_tb_attach_context *ctx = data;

	return !!ctx->best->dcp->typec_connector;
}

static void dcp_tb_queue_reconnect(void *data)
{
	struct dcp_tb_attach_context *ctx = data;
	struct apple_dcp *dcp = ctx->best->dcp;

	dcp->typec_reconnect_tries = 0;
	dcp_queue_typec_reconnect(dcp, 0);
}

static void dcp_tb_connect_oob(void *data)
{
	struct dcp_tb_attach_context *ctx = data;

	dcp_dptx_connect_oob(to_platform_device(ctx->best->dcp->dev), 0);
}

static const struct dcp_fabric_attach_ops dcp_tb_attach_ops = {
	.candidate = dcp_tb_candidate,
	.activate = dcp_tb_activate,
	.rollback = dcp_tb_rollback,
	.claim = dcp_tb_claim,
	.external = dcp_tb_external,
	.has_connector = dcp_tb_has_connector,
	.queue_reconnect = dcp_tb_queue_reconnect,
	.connect_oob = dcp_tb_connect_oob,
};

/*
 * Retire @dcp's Thunderbolt binding (and @route's, if given): no DCP callback,
 * crossbar or clock change reaches it once this returns, and reconnect work
 * queued or running for its session finds that session revoked. The source
 * drains with the fabric lock dropped and the pipeline reserved by
 * tb_retiring; this tree has no tb_retiring and keeps the fabric lock across
 * the wait, as dcp_dptx_disconnect_oob() always has. Neither worker takes it.
 */
static void dcp_tb_binding_retire(struct apple_dcp *dcp,
				  struct apple_dcp_typec_route *route)
{
	lockdep_assert_held(&dcp_typec_fabric_lock);
	scoped_guard(mutex, &dcp->tb_lock) {
		dcp->tb_generation = 0;
		if (route)
			route->tunnel_generation = 0;
		dcp->tb_dpin_set_active = NULL;
		dcp->tb_dpin_ctx = NULL;
	}
	cancel_delayed_work_sync(&dcp->typec_reconnect_wq);
}

/*
 * tiled_split: take the second half of a tiled display (dpin1) onto DPTX
 * port 1 of the pipeline that already drives dpin0 on this port. DCP pairs
 * the two tiles itself (PAVSplitDisplay) and presents one display; on its
 * own pipeline the second tile is refused ("no single tile support").
 * Fabric lock held.
 */
static int dcp_tb_split_teardown_locked(struct apple_dcp_typec_port *port,
					bool force)
{
	struct apple_dcp *dcp = port->split_dcp;
	int ret;

	lockdep_assert_held(&dcp_typec_fabric_lock);
	if (!dcp)
		return 0;
	if (dcp->dptxport[1].enabled && dcp->dptxport[1].connected) {
		ret = dptxport_set_hpd(dcp->dptxport[1].service, false);
		if (ret && !force)
			return ret;
	}
	ret = dcp_dptx_disconnect(dcp, 1);
	if (ret && !force)
		return ret;
	scoped_guard(mutex, &dcp->tb_lock) {
		/* Retire the callback before dropping its clock reference. */
		dcp->split.active = false;
		if (dcp->phy) {
			typeof(apple_atc_dp_tunnel_rate) *stop =
				symbol_get(apple_atc_dp_tunnel_rate);

			if (stop) {
				stop(dcp->phy, 1, 0);
				symbol_put(apple_atc_dp_tunnel_rate);
			}
		}
		if (dcp->split.xbar) {
			if (dcp->split.xbar_up)
				mux_control_deselect(dcp->split.xbar);
			else
				dcp_dpxbar_preselect(dcp->split.xbar,
						     MUX_IDLE_DISCONNECT);
			dcp_dpxbar_tunnel_select_source(dcp->split.xbar, -1);
		}
		dcp->split.xbar = NULL;
		dcp->split.xbar_up = false;
		dcp->split.clock_ok = false;
		dcp->split.ready = false;
		dcp->split.generation = 0;
		dcp->split.set_active = NULL;
		dcp->split.binding = NULL;
	}
	port->split_dcp = NULL;
	dev_info(dcp->dev, "tiled: dpin1 released from DPTX port 1\n");
	return 0;
}

/*
 * Feed dpin1 into DPTX port 1 of @route's pipeline, with its binding. The
 * split goes live only once the crossbar carries dpin1 there; a failure is
 * undone and returned.
 */
static int dcp_tb_split_join_locked(struct apple_dcp_typec_port *port,
				    struct apple_dcp_typec_route *route,
				    struct mux_control *xbar, u64 generation,
				    int (*set_active)(void *binding, bool active),
				    void *binding)
{
	struct apple_dcp *dcp = route->dcp;
	int ret, state = route->mux_index | 1;

	if (!xbar || port->split_dcp || dcp->split.active)
		return -EBUSY;

	lockdep_assert_held(&dcp_typec_fabric_lock);
	/* T602X points the DP IN at its pipeline before DCP probes AUX */
	ret = dcp_dpxbar_tunnel_select_source(xbar, state);
	if (ret && ret != -EOPNOTSUPP) {
		dev_warn(dcp->dev, "tiled: dpin1 source select failed: %d\n", ret);
		goto unwind;
	}
	ret = dcp_dpxbar_preselect(xbar, state);
	if (ret && ret != -EOPNOTSUPP) {
		dev_warn(dcp->dev, "tiled: dpin1 crossbar preselect failed: %d\n", ret);
		goto unwind;
	}

	scoped_guard(mutex, &dcp->tb_lock) {
		dcp->split.xbar = xbar;
		dcp->split.mux_state = state;
		dcp->split.xbar_up = false;
		dcp->split.clock_ok = false;
		dcp->split.generation = generation;
		dcp->split.set_active = set_active;
		dcp->split.binding = binding;
		dcp->split.ready = false;
		dcp->split.active = true;
	}
	port->split_dcp = dcp;
	dev_info(dcp->dev,
		 "tiled: dpin1 joins dpin0 on DPTX port 1 (crossbar state %d)\n",
		 state);
	return 0;

unwind:
	dcp_dpxbar_preselect(xbar, MUX_IDLE_DISCONNECT);
	dcp_dpxbar_tunnel_select_source(xbar, -1);
	return ret;
}

/*
 * Can no pipeline that drives @port take the second half of a tiled display?
 * A tile joins DPTX port 1 of the pipeline that drives dpin0, which an
 * external display processor never does (see dcp_tb_split_locked()). That is
 * fixed when the processor probes, before its routes are registered, so it
 * holds before any of its services are up. On T6030 every Type-C pipeline is
 * an external processor. A port with no routes is not judged here.
 */
static bool dcp_typec_port_untileable(struct apple_dcp_typec_port *port)
{
	struct apple_dcp_typec_route *route;

	list_for_each_entry(route, &port->routes, port_link)
		if (!route->dcp->external)
			return false;
	return !list_empty(&port->routes);
}

/* -EOPNOTSUPP: not a tile, route dpin1 as a display of its own. */
static int dcp_tb_split_locked(struct apple_dcp_typec_port *port,
			       const struct dcp_tb_attach_context *req)
{
	struct apple_dcp_typec_route *route = port->owner;
	struct mux_control *xbar;
	struct apple_dcp *dcp;
	int ret, release;

	lockdep_assert_held(&dcp_typec_fabric_lock);
	/* A retired provider leaves only hardware ownership to retry. */
	if (port->split_dcp && !port->split_dcp->split.generation) {
		ret = dcp_tb_split_teardown_locked(port, false);
		if (ret || !req->active)
			return ret;
	}
	/* Only the live cookie and binding may repeat or end a split dpin1. */
	if (port->split_dcp) {
		dcp = port->split_dcp;
		scoped_guard(mutex, &dcp->tb_lock)
			ret = dcp_fabric_binding_request(dcp->split.generation,
							 req->generation, req->active,
							 dcp->split.set_active == req->set_active &&
							 dcp->split.binding == req->binding);
		if (ret < 0)
			return ret;
		if (req->active)
			return dcp->split.ready ? 0 : -EAGAIN;
		return dcp_tb_split_teardown_locked(port, false);
	}
	if (!req->active)
		return -EOPNOTSUPP;
	/* No tile is possible here: there is no dpin0 to wait for. */
	if (dcp_typec_port_untileable(port))
		return -EOPNOTSUPP;
	/* dpin0 first: the Thunderbolt side retries on -ENODEV */
	if (!route || !route->tunnel || route->tunnel_dpin != 0)
		return -ENODEV;
	dcp = route->dcp;
	if (dcp->external || !dcp->dptxport[1].enabled)
		return -EOPNOTSUPP;
	xbar = route->dpin[1];
	if (!xbar) {
		/* Legacy DT ABI: unnamed DPIN controls share the DP-alt chip. */
		if (route->xbar != &route->xbar->chip->mux[0] ||
		    route->xbar->chip->controllers < 3)
			return -EOPNOTSUPP;
		xbar = &route->xbar->chip->mux[2];
	}
	/*
	 * Only the second half of a tiled display goes here; two separate
	 * monitors behind a dock keep their own pipelines. DCP reports the
	 * topology for dpin0 shortly after its link comes up: wait for that,
	 * but not for ever, as a sink may never send one.
	 */
	if (!smp_load_acquire(&dcp->dptxport[0].tile_hint)) {
		if (time_before(jiffies, port->tile_hint_deadline))
			return -ENODEV;
		return -EOPNOTSUPP;
	}
	ret = dcp_tb_split_join_locked(port, route, xbar, req->generation,
				       req->set_active, req->binding);
	if (ret)
		return ret;

	ret = dcp_dptx_connect(dcp, 1);
	if (ret) {
		dev_warn(dcp->dev, "tiled: DPTX port 1 connect failed: %d\n", ret);
		/* dpin0 keeps its route and stays a single-stream display */
		release = dcp_tb_split_teardown_locked(port, false);

		if (release)
			return release;
		/* This is a failed tile setup, not an independent display. */
		return ret == -EOPNOTSUPP ? -EIO : ret;
	}
	scoped_guard(mutex, &dcp->tb_lock)
		dcp->split.ready = true;
	return 0;
}

static int dcp_tb_release_locked(struct apple_dcp_typec_port *port,
				 unsigned int dpin)
{
	struct apple_dcp_typec_route **slot = dpin ? &port->secondary_owner : &port->owner;
	struct apple_dcp *dcp;
	int ret;

	/* the second tile goes before the route it shares */
	if (dpin == 0) {
		ret = dcp_tb_split_teardown_locked(port, false);
		if (ret)
			return ret;
	}
	dcp = (*slot)->dcp;
	/* A refused display release keeps the route and its binding owned. */
	if (port->hpd || dcp->typec_cable_connected) {
		ret = dcp_dptx_disconnect_oob(to_platform_device(dcp->dev), 0);
		if (ret)
			return ret;
	}
	ret = dcp_typec_route_deactivate(*slot);
	if (ret) {
		/*
		 * The caller's context is going away regardless: retire the
		 * binding, the route stays owned for the next request to release.
		 */
		scoped_guard(mutex, &dcp->tb_lock) {
			dcp->dptx_tunnel = false;
			dcp->tb_dpin_set_active = NULL;
			dcp->tb_dpin_ctx = NULL;
			dcp->tb_generation = 0;
			(*slot)->tunnel_generation = 0;
		}
		return ret;
	}
	*slot = NULL;
	port->hpd = !!(port->owner || port->secondary_owner);
	/* re-apply the next Type-C mux state in full */
	port->applied_valid = false;
	if (dcp->hdmi_hpd && dcp->active &&
	    gpiod_get_value_cansleep(dcp->hdmi_hpd))
		dcp_dptx_connect(dcp, 0);
	dcp_typec_pipeline_freed();
	return 0;
}

static int dcp_tb_dispatch_request(void *data, const struct dcp_fabric_port *found)
{
	struct dcp_tb_attach_context *ctx = data;
	struct apple_dcp_typec_port *port =
		container_of(found, struct apple_dcp_typec_port, core);
	struct apple_dcp_typec_route **slot;
	struct apple_dcp *dcp;
	unsigned int dpin = ctx->dpin;
	int ret;

	slot = dpin ? &port->secondary_owner : &port->owner;

	if (dpin == 1 && (tiled_split || port->split_dcp)) {
		ret = dcp_tb_split_locked(port, ctx);
		if (ret != -EOPNOTSUPP)
			return ret;
	}
	/* A retired binding leaves only hardware ownership to retry. */
	if (*slot && (*slot)->tunnel && (*slot)->tunnel_dpin == dpin &&
	    !(*slot)->tunnel_generation) {
		ret = dcp_tb_release_locked(port, dpin);
		if (ret || !ctx->active)
			return ret;
	}
	/*
	 * Only the live cookie and binding may repeat or end a handed stream; a
	 * mismatch is -ESTALE before anything changes.
	 */
	if (*slot && (*slot)->tunnel && (*slot)->tunnel_dpin == dpin) {
		dcp = (*slot)->dcp;
		scoped_guard(mutex, &dcp->tb_lock) {
			bool same = dcp->tb_dpin_set_active == ctx->set_active &&
				    dcp->tb_dpin_ctx == ctx->binding;

			ret = dcp_fabric_binding_request((*slot)->tunnel_generation,
							 ctx->generation, ctx->active, same);
		}
		if (ret)
			return ret > 0 ? 0 : ret;
	} else if (!ctx->active) {
		return -ESTALE;
	}

	if (!ctx->active)
		return dcp_tb_release_locked(port, dpin);

	{
		struct dcp_fabric_port state = {
			.owner[0] = port->owner ? &port->owner->core : NULL,
			.owner[1] = port->secondary_owner ?
					    &port->secondary_owner->core :
					    NULL,
		};

		if (port->owner) {
			port->owner->core.tunnel = port->owner->tunnel;
			port->owner->core.dpin = port->owner->tunnel_dpin;
		}
		if (port->secondary_owner) {
			port->secondary_owner->core.tunnel =
				port->secondary_owner->tunnel;
			port->secondary_owner->core.dpin =
				port->secondary_owner->tunnel_dpin;
		}
		ret = dcp_fabric_tunnel_slot(&state, dpin);
		if (ret > 0)
			return 0;
		if (ret) {
			if (ret == -EADDRINUSE)
				dev_warn((*slot)->dcp->dev,
					 "port already routed, not taking DP tunnel dpin%u\n",
					 dpin);
			return ret;
		}
	}

	ctx->port = port;
	ctx->slot = slot;
	ret = dcp_fabric_run_attach(&dcp_tb_attach_ops, ctx);
	/* A failed arriving stream releases its reserved plan to the others. */
	if (ret && ret != -ESHUTDOWN && ctx->planned)
		dcp_typec_rebalance_locked(NULL, 0);
	return ret;
}

/*
 * Failed attach and every detach return before the provider frees its
 * binding: the request's own cookie (and, for an attach, its own binding) is
 * retired. A retired route keeps its hardware ownership for the next request
 * to release (see dcp_tb_dispatch_request()).
 */
static void dcp_tb_finish_request(struct apple_dcp_typec_port *port,
				  const struct dcp_tb_attach_context *request)
{
	struct apple_dcp_typec_route *route = request->dpin ?
		port->secondary_owner : port->owner;
	struct apple_dcp *dcp;
	bool matching;

	if (request->dpin == 1 && port->split_dcp) {
		dcp = port->split_dcp;
		scoped_guard(mutex, &dcp->tb_lock) {
			if (dcp->split.generation == request->generation &&
			    (!request->active ||
			     (dcp->split.binding == request->binding &&
			      dcp->split.set_active == request->set_active))) {
				dcp->split.set_active = NULL;
				dcp->split.binding = NULL;
				dcp->split.generation = 0;
				dcp->split.ready = false;
			}
		}
	}
	if (!route || !route->tunnel)
		return;
	dcp = route->dcp;
	scoped_guard(mutex, &dcp->tb_lock)
		matching = dcp->tb_generation == request->generation &&
			   (!request->active ||
			    (dcp->tb_dpin_ctx == request->binding &&
			     dcp->tb_dpin_set_active == request->set_active));
	if (matching)
		dcp_tb_binding_retire(dcp, route);
}

static int dcp_tb_dispatch(void *data, const struct dcp_fabric_port *found)
{
	struct dcp_tb_attach_context *request = data;
	struct apple_dcp_typec_port *port =
		container_of(found, struct apple_dcp_typec_port, core);
	int ret = dcp_tb_dispatch_request(data, found);

	if (ret || !request->active)
		dcp_tb_finish_request(port, request);
	return ret;
}

int apple_dcp_tb_dp_tunnel(struct device_node *connector_np, unsigned int dpin,
			   u64 generation, bool active,
			   int (*set_active)(void *binding, bool active), void *binding)
{
	struct apple_dcp_typec_port *pos;
	struct dcp_fabric_port *ports = NULL, **tail = &ports;
	struct dcp_tb_attach_context request = {
		.generation = generation,
		.dpin = dpin,
		.active = active,
		.set_active = set_active,
		.binding = binding,
	};

	if (!connector_np || dpin > 1 || !generation || (active && !set_active))
		return -EINVAL;
	guard(mutex)(&dcp_typec_fabric_lock);
	list_for_each_entry(pos, &dcp_typec_ports, link) {
		pos->core.key = (unsigned long)pos->connector_np;
		*tail = &pos->core;
		tail = &pos->core.next;
	}
	*tail = NULL;
	return dcp_fabric_run_request(ports, (unsigned long)connector_np, dpin,
				      dcp_tb_dispatch, &request);
}
EXPORT_SYMBOL_GPL(apple_dcp_tb_dp_tunnel);

/*
 * Direct DP-alt routes and Thunderbolt tunnels follow the CRTC selected by
 * a modeset on 12.3/13.5.  A tunnel keeps its Thunderbolt binding: only the
 * crossbar output of its DP IN changes pipeline, and the binding is revoked
 * on the pipeline it leaves before it is installed on the one it joins.
 * Firmware 14.7 and native external processors keep their existing path.
 */
struct dcp_typec_follow {
	struct apple_dcp_typec_port *port;
	struct apple_dcp_typec_route *from;
	struct apple_dcp_typec_route *to;
	struct apple_dcp_typec_route *holder;
	struct apple_dcp_typec_route *holder_back;
	enum dcp_fabric_follow action;
};

/* Catalog snapshots are allocated before either route is disturbed. */
struct dcp_typec_follow_slot {
	struct apple_dcp_typec_route *from;
	struct apple_dcp_typec_route *to;
	struct apple_dcp_typec_port *port;
	struct apple_connector *connector;
	struct dcp_display_mode *modes;
	unsigned int nr_modes;
	u64 generation;
	u64 attachment_generation;
	bool was_active;
	bool restored;
	/* a tunnel's DP IN and Thunderbolt binding, carried along */
	bool tunnel;
	unsigned int dpin;
	u64 tunnel_generation;
	int (*set_active)(void *binding, bool active);
	void *binding;
	/* tiled_split: the second tile, on DPTX port 1 of the same pipeline */
	bool split;
	u64 split_generation;
	int (*split_set_active)(void *binding, bool active);
	void *split_binding;
};

/* Record @route's second tile, if it has one, for the pipeline it joins. */
static void dcp_follow_save_split(struct apple_dcp_typec_route *route,
				  struct dcp_typec_follow_slot *slot)
{
	struct apple_dcp *dcp = route->dcp;

	if (route->port->split_dcp != dcp)
		return;
	scoped_guard(mutex, &dcp->tb_lock) {
		/* A retired second stream cannot be carried into the move. */
		if (!dcp->split.generation || !dcp->split.set_active)
			return;
		slot->split = true;
		slot->split_generation = dcp->split.generation;
		slot->split_set_active = dcp->split.set_active;
		slot->split_binding = dcp->split.binding;
	}
}

/* Can @route's pipeline take a second tile? */
static bool dcp_follow_split_fits(struct apple_dcp_typec_route *route)
{
	return dcp_typec_tunnel_ctl(route, 1) && !route->dcp->external &&
	       route->dcp->dptxport[1].enabled;
}

struct dcp_typec_follow_context {
	struct dcp_typec_follow_slot slots[2];
	struct drm_atomic_commit *state;
};

/*
 * The fabric lock, from a modeset.  A holder of it can wait for the
 * modeset locks: a display that goes sends a hotplug event, and with no
 * compositor running the fbdev client answers that with a commit.  Rather
 * than deadlock with it, a modeset gives up after a while, and is then
 * refused or left on its pipeline as if the route could not follow.
 */
#define DCP_FOLLOW_LOCK_MS	3000

/* This tree serializes Thunderbolt handoffs under the fabric lock too. */
static bool dcp_typec_follow_lock(bool handoff)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(DCP_FOLLOW_LOCK_MS);

	for (;;) {
		if (mutex_trylock(&dcp_typec_fabric_lock))
			return true;
		if (time_after(jiffies, timeout))
			return false;
		msleep(20);
	}
}

static void dcp_typec_follow_unlock(bool handoff)
{
	mutex_unlock(&dcp_typec_fabric_lock);
}

static struct apple_dcp_typec_port *
dcp_typec_port_of(struct apple_connector *connector, bool *secondary)
{
	struct apple_dcp_typec_port *port;

	list_for_each_entry(port, &dcp_typec_ports, link) {
		if (port->connector != connector &&
		    port->secondary_connector != connector)
			continue;
		*secondary = port->secondary_connector == connector;
		return port;
	}
	return NULL;
}

static struct apple_dcp_typec_route *
dcp_typec_port_route(struct apple_dcp_typec_port *port, struct apple_dcp *dcp)
{
	struct apple_dcp_typec_route *route;

	list_for_each_entry(route, &port->routes, port_link)
		if (route->dcp == dcp)
			return route;
	return NULL;
}

/*
 * Is the display of @connector off in @state, or to be shown on @back's
 * pipeline, so that its route may give @crtc's pipeline up for that one?
 */
bool dcp_typec_follow_holder_off(struct drm_atomic_commit *state,
				 struct drm_crtc *crtc,
				 struct apple_connector *connector,
				 struct apple_dcp *back)
{
	struct drm_connector_state *conn_state = NULL;
	struct drm_crtc_state *crtc_state;
	struct drm_crtc *other;

	if (connector) {
		conn_state = drm_atomic_get_new_connector_state(state, &connector->base);
	}
	/* Check reserved this holder; never inspect a later commit's live state. */
	if (!conn_state)
		return false;
	other = conn_state->crtc;
	if (!other)
		return true;
	if (other == crtc)
		return false;
	if (back->crtc && other == &back->crtc->base)
		return true;
	crtc_state = drm_atomic_get_new_crtc_state(state, other);
	return crtc_state && !crtc_state->active;
}

/* May @connector's route follow @crtc to @dcp's pipeline, and how? */
static int dcp_typec_follow_decide(struct apple_dcp *dcp, struct drm_crtc *crtc,
				   struct drm_atomic_commit *state,
				   struct apple_connector *connector,
				   struct dcp_typec_follow *follow)
{
	struct dcp_fabric_policy policy = { .dual_stream = dcp_typec_ordered_dual_stream() };
	struct apple_dcp_typec_route *from, *to, *holder, *back = NULL;
	struct apple_dcp_typec_port *port, *other = NULL;
	bool secondary, off = false;

	lockdep_assert_held(&dcp_typec_fabric_lock);
	memset(follow, 0, sizeof(*follow));
	follow->action = DCP_FABRIC_FOLLOW_REFUSE;
	port = dcp_typec_port_of(connector, &secondary);
	if (!port)
		return -EINVAL;
	from = secondary ? port->secondary_owner : port->owner;
	to = dcp_typec_port_route(port, dcp);
	if (from && from == to) {
		follow->action = DCP_FABRIC_FOLLOW_STAY;
		return 0;
	}
	/*
	 * Not routed (yet): nothing to follow, the modeset is checked as it
	 * stands, as when resume restores a display whose tunnel is not back.
	 */
	if (!from || !READ_ONCE(connector->connected))
		return -ENOENT;
	/* A dock's second stream has one pipeline, and the plan rules first. */
	/* A port in SAFE keeps its owner until the mux set phase completes. */
	if (secondary || port->safe_pending ||
	    !dcp_typec_follows_crtc(from->dcp) || dcp_typec_keep_order())
		return -EINVAL;
	holder = dcp->active_typec_route;
	if (holder) {
		other = holder->port;
		if (holder != other->owner || other->safe_pending)
			return -EINVAL;
		back = dcp_typec_port_route(other, from->dcp);
		off = dcp_typec_follow_holder_off(state, crtc, other->connector,
						  from->dcp);
		/* the port's own snapshot below decides for @dcp's pipeline */
		dcp_fabric_snapshot_port(other, holder->tunnel);
	}
	dcp_fabric_snapshot_port(port, from->tunnel);
	/* Follow-only reservation; arrival allocation retains its existing policy. */
	if (to)
		to->core.pipeline->fixed_recent = dcp->hdmi_hpd && dcp->fixed_phy &&
			READ_ONCE(dcp->hdmi_edge_seen) &&
			time_before(jiffies, READ_ONCE(dcp->hdmi_edge_jiffies) +
				    msecs_to_jiffies(DCP_HDMI_HOLD_MS));
	follow->action = dcp_fabric_follow(&from->core, to ? &to->core : NULL,
					   holder ? &holder->core : NULL,
					   back ? &back->core : NULL, off, &policy);
	/* A tunnel's DP IN must reach the pipeline it is given. */
	if (follow->action == DCP_FABRIC_FOLLOW_REFUSE ||
	    (from->tunnel && !dcp_typec_tunnel_ctl(to, from->tunnel_dpin)) ||
	    (port->split_dcp == from->dcp && !dcp_follow_split_fits(to)) ||
	    (follow->action == DCP_FABRIC_FOLLOW_SWAP && holder->tunnel &&
	     !dcp_typec_tunnel_ctl(back, holder->tunnel_dpin)) ||
	    (follow->action == DCP_FABRIC_FOLLOW_SWAP &&
	     holder->port->split_dcp == holder->dcp && !dcp_follow_split_fits(back)))
		return -EINVAL;
	follow->port = port;
	follow->from = from;
	follow->to = to;
	follow->holder = holder;
	follow->holder_back = back;
	return 0;
}

static int dcp_follow_prepare(void *data, unsigned int index)
{
	struct dcp_typec_follow_context *ctx = data;
	struct dcp_typec_follow_slot *slot = &ctx->slots[index];

	slot->port = slot->from->port;
	slot->connector = slot->from->dcp->typec_connector;
	slot->tunnel = slot->from->tunnel;
	slot->dpin = slot->from->tunnel_dpin;
	if (slot->tunnel) {
		struct apple_dcp *dcp = slot->from->dcp;

		scoped_guard(mutex, &dcp->tb_lock) {
			slot->tunnel_generation = slot->from->tunnel_generation;
			slot->set_active = dcp->tb_dpin_set_active;
			slot->binding = dcp->tb_dpin_ctx;
		}
		if (!slot->tunnel_generation || !slot->set_active)
			return -ESTALE;
	}
	dcp_follow_save_split(slot->from, slot);
	{
		struct drm_crtc *crtc = &slot->from->dcp->crtc->base;
		struct drm_crtc_state *old = drm_atomic_get_old_crtc_state(ctx->state, crtc);

		slot->was_active = old && old->active;
	}
	slot->modes = dcp_modes_dup(slot->from->dcp, slot->connector,
				    &slot->nr_modes, &slot->generation);
	if (IS_ERR(slot->modes)) {
		int ret = PTR_ERR(slot->modes);

		slot->modes = NULL;
		return ret;
	}
	return 0;
}

static int dcp_follow_validate(void *data, unsigned int index)
{
	struct dcp_typec_follow_context *ctx = data;
	struct dcp_typec_follow_slot *slot = &ctx->slots[index];
	struct apple_dcp *dcp = slot->from->dcp;

	guard(mutex)(&dcp->modes_lock);
	if (dcp->modes_generation != slot->generation ||
	    !dcp_modes_for_connector(dcp, slot->connector) ||
	    dcp->active_typec_route != slot->from ||
	    READ_ONCE(slot->connector->dcp) != to_platform_device(dcp->dev))
		return -ESTALE;
	return 0;
}

static int dcp_follow_release(struct apple_dcp_typec_route *route)
{
	struct apple_dcp *dcp = route->dcp;
	bool tunnel = route->tunnel;
	int ret;

	/* Revoked sessions cannot touch the new route after a reconnect wait. */
	ret = dcp_dptx_park(dcp);
	if (ret)
		return ret;
	if (route->port->split_dcp == dcp) {
		ret = dcp_tb_split_teardown_locked(route->port, false);
		if (ret)
			return ret;
	}
	apple_connector_edid_set_live(dcp->typec_connector, false);
	dcp_modes_begin_attachment(dcp);
	dcp->typec_connector = NULL;
	WRITE_ONCE(dcp->connector, dcp->fixed_connector);
	/* The binding moves on: no callback may reach it through here. */
	if (tunnel) {
		scoped_guard(mutex, &dcp->tb_lock) {
			dcp->tb_generation = 0;
			route->tunnel_generation = 0;
			dcp->tb_dpin_set_active = NULL;
			dcp->tb_dpin_ctx = NULL;
		}
	}
	scoped_guard(mutex, &dcp->hpd_mutex)
		WRITE_ONCE(dcp->typec_cable_connected, false);
	ret = dcp_typec_route_deactivate(route);
	if (route->port->owner == route)
		route->port->owner = NULL;
	return ret;
}

static int dcp_follow_detach(void *data, unsigned int index, bool destination)
{
	struct dcp_typec_follow_context *ctx = data;
	struct dcp_typec_follow_slot *slot = &ctx->slots[index];

	return dcp_follow_release(destination ? slot->to : slot->from);
}

static bool dcp_follow_owned(struct apple_dcp_typec_route *route)
{
	return route->port->owner == route && route->selected &&
	       route->dcp->active_typec_route == route;
}

static bool dcp_follow_retained(void *data, unsigned int index, bool destination)
{
	struct dcp_typec_follow_context *ctx = data;
	struct dcp_typec_follow_slot *slot = &ctx->slots[index];

	return dcp_follow_owned(destination ? slot->to : slot->from);
}

/* Route @slot's display through @route, a tunnel with its binding. */
static int dcp_follow_activate(struct apple_dcp_typec_route *route,
			       struct dcp_typec_follow_slot *slot)
{
	struct apple_dcp *dcp = route->dcp;
	struct mux_control *ctl = route->xbar;
	int ret;

	/* Reserve both stream outputs before publishing the new main route. */
	if (slot->split) {
		ret = dcp_tb_split_join_locked(slot->port, route,
					       dcp_typec_tunnel_ctl(route, 1),
					       slot->split_generation,
					       slot->split_set_active,
					       slot->split_binding);
		if (ret)
			return ret;
	}
	if (slot->tunnel) {
		ctl = dcp_typec_tunnel_ctl(route, slot->dpin);
		if (!ctl) {
			if (slot->split)
				dcp_tb_split_teardown_locked(slot->port, false);
			return -EOPNOTSUPP;
		}
		scoped_guard(mutex, &dcp->tb_lock) {
			dcp->tb_dpin_set_active = slot->set_active;
			dcp->tb_dpin_ctx = slot->binding;
			dcp->tb_generation = slot->tunnel_generation;
			route->tunnel_generation = slot->tunnel_generation;
		}
		route->tunnel_dpin = slot->dpin;
	}
	ret = dcp_typec_route_activate(route, ctl);
	if (ret) {
		if (slot->split)
			dcp_tb_split_teardown_locked(slot->port, false);
		if (slot->tunnel) {
			scoped_guard(mutex, &dcp->tb_lock) {
				dcp->tb_dpin_set_active = NULL;
				dcp->tb_dpin_ctx = NULL;
				dcp->tb_generation = 0;
				route->tunnel_generation = 0;
			}
		}
		return ret;
	}
	if (slot->tunnel)
		dcp_tunnel_prepare(route, ctl);
	if (slot->split) {
		scoped_guard(mutex, &dcp->tb_lock)
			dcp->split.ready = true;
	}
	slot->port->owner = route;
	slot->port->preferred_route = route;
	return 0;
}

static int dcp_follow_attach(void *data, unsigned int index, bool restore)
{
	struct dcp_typec_follow_context *ctx = data;
	struct dcp_typec_follow_slot *slot = &ctx->slots[index];
	struct apple_dcp_typec_route *route = restore ? slot->from : slot->to;
	struct apple_dcp *dcp = route->dcp;
	int ret;

	if (restore && dcp->active_typec_route && dcp->active_typec_route != route)
		return -EBUSY;
	ret = dcp_follow_activate(route, slot);
	if (ret)
		return ret;
	slot->attachment_generation = dcp_modes_transfer_begin(dcp);
	scoped_guard(mutex, &dcp->hpd_mutex) {
		WRITE_ONCE(dcp->typec_cable_connected, true);
		dcp->typec_generation++;
		dcp->typec_follow_start = true;
		dcp->typec_follow_gen = dcp->typec_generation;
		dcp->typec_reconnect_tries = 0;
		dcp->placeholder_retried = false;
		reinit_completion(&dcp->typec_iomfb_hpd_ready);
		WRITE_ONCE(dcp->typec_crtc_off, true);
	}
	return 0;
}

static void dcp_follow_publish(void *data, unsigned int index, bool restore)
{
	struct dcp_typec_follow_context *ctx = data;
	struct dcp_typec_follow_slot *slot = &ctx->slots[index];
	struct apple_dcp *dcp = (restore ? slot->from : slot->to)->dcp;

	dcp_modes_adopt(dcp, slot->connector, slot->attachment_generation, slot->modes,
			slot->nr_modes);
	slot->modes = NULL;
	slot->restored = restore;
}

static void dcp_follow_lost(void *data, unsigned int index)
{
	struct dcp_typec_follow_context *ctx = data;
	struct apple_connector *connector = ctx->slots[index].connector;

	if (connector) {
		WRITE_ONCE(connector->connected, false);
		apple_connector_set_pipeline(connector, NULL);
		dcp_queue_hotplug(connector);
	}
}

static const struct dcp_fabric_follow_ops dcp_follow_ops = {
	.prepare = dcp_follow_prepare,
	.validate = dcp_follow_validate,
	.detach = dcp_follow_detach,
	.retained = dcp_follow_retained,
	.attach = dcp_follow_attach,
	.publish = dcp_follow_publish,
	.lost = dcp_follow_lost,
};

static int dcp_typec_follow_move(struct dcp_typec_follow *follow,
				 struct drm_atomic_commit *state)
{
	struct dcp_typec_follow_context ctx = {
		.state = state,
		.slots[0] = { .from = follow->from, .to = follow->to },
		.slots[1] = { .from = follow->holder, .to = follow->holder_back },
	};
	unsigned int count = follow->action == DCP_FABRIC_FOLLOW_SWAP ? 2 : 1;
	unsigned int i;
	int ret;

	/* Keep newly queued and already waiting sessions outside the handoff. */
	for (i = 0; i < count; i++) {
		WRITE_ONCE(ctx.slots[i].from->dcp->typec_follow_retiring, true);
		WRITE_ONCE(ctx.slots[i].to->dcp->typec_follow_retiring, true);
	}
	ret = dcp_fabric_follow_execute(&dcp_follow_ops, &ctx, count);
	if (ret)
		to_apple_atomic_state(state)->failed_routes |=
			drm_crtc_mask(&follow->from->dcp->crtc->base) |
			drm_crtc_mask(&follow->to->dcp->crtc->base);
	for (i = 0; i < count; i++) {
		WRITE_ONCE(ctx.slots[i].from->dcp->typec_follow_retiring, false);
		WRITE_ONCE(ctx.slots[i].to->dcp->typec_follow_retiring, false);
	}
	for (i = 0; i < count; i++) {
		struct dcp_typec_follow_slot *slot = &ctx.slots[i];

		if (slot->restored && slot->was_active) {
			int error = dcp_dptx_connect(slot->from->dcp, 0);

			if (error)
				dev_err(slot->from->dcp->dev,
					"restored route link could not reconnect: %d\n", error);
		}
	}
	if (!ret && count == 1) {
		struct apple_dcp *freed = ctx.slots[0].from->dcp;

		if (freed->hdmi_hpd && freed->active &&
		    gpiod_get_value_cansleep(freed->hdmi_hpd))
			dcp_dptx_connect(freed, 0);
		dcp_typec_route_waiting();
	}
	kfree(ctx.slots[0].modes);
	kfree(ctx.slots[1].modes);
	if (ret) {
		for (i = 0; i < count; i++)
			if (ctx.slots[i].connector)
				dcp_route_failure_notify(ctx.slots[i].connector);
	}
	return ret;
}

/*
 * atomic_check: may the Type-C displays @state puts on @crtc be driven by
 * its pipeline @dcp? Validate the source catalog under fabric ownership;
 * commit revalidates the route before making any destructive changes.
 */
int dcp_typec_follow_check(struct apple_dcp *dcp, struct drm_crtc *crtc,
			   struct drm_atomic_commit *state,
			   const struct drm_display_mode *mode)
{
	struct drm_connector_state *conn_state;
	struct dcp_typec_follow follow;
	struct drm_connector *conn;
	int i, ret;

	for_each_new_connector_in_state(state, conn, conn_state, i) {
		struct apple_connector *connector = to_apple_connector(conn);

		if (conn_state->crtc != crtc || !connector->port_encoder ||
		    READ_ONCE(connector->dcp) == to_platform_device(dcp->dev))
			continue;
		if (!state->allow_modeset)
			return -EINVAL;
		/* Include both sides in DRM dependency ordering, including off holders. */
		{
			struct drm_crtc *other;

			drm_for_each_crtc(other, crtc->dev) {
				struct drm_crtc_state *other_state;

				other_state = drm_atomic_get_crtc_state(state, other);
				if (IS_ERR(other_state))
					return PTR_ERR(other_state);
			}
		}
retry_holder:
		if (!dcp_typec_follow_lock(false))
			return -EBUSY;
		if (dcp->active_typec_route) {
			struct apple_connector *holder = dcp->active_typec_route->port->connector;

			if (holder && !drm_atomic_get_new_connector_state(state, &holder->base)) {
				struct drm_connector_state *holder_state;

				drm_connector_get(&holder->base);
				dcp_typec_follow_unlock(false);
				/* Acquire DRM state without holding either fabric mutex. */
				holder_state = drm_atomic_get_connector_state(state, &holder->base);
				drm_connector_put(&holder->base);
				if (IS_ERR(holder_state))
					return PTR_ERR(holder_state);
				goto retry_holder;
			}
		}
		ret = dcp_typec_follow_decide(dcp, crtc, state, connector, &follow);
		if (!ret && !dcp_has_mode(follow.action == DCP_FABRIC_FOLLOW_STAY ?
					 dcp : follow.from->dcp, mode))
			ret = -EINVAL;
		dcp_typec_follow_unlock(false);
		if (ret == -ENOENT)
			continue;
		if (ret) {
			dev_info_ratelimited(dcp->dev, "%s cannot follow its CRTC here\n",
					     conn->name);
			return ret;
		}
		return 0;
	}
	return dcp_has_mode(dcp, mode) ? 0 : -EINVAL;
}

static bool dcp_typec_direct_commit(struct drm_crtc *crtc,
				    struct drm_atomic_commit *state)
{
	struct drm_connector_state *conn_state;
	struct drm_connector *conn;
	int i;

	for_each_new_connector_in_state(state, conn, conn_state, i) {
		struct apple_connector *connector = to_apple_connector(conn);
		struct platform_device *pdev = READ_ONCE(connector->dcp);
		struct apple_dcp_typec_route *route;
		struct apple_dcp *source;

		if (conn_state->crtc != crtc || !connector->port_encoder || !pdev)
			continue;
		source = platform_get_drvdata(pdev);
		/* The second half of a swap still needs its handoff identity held. */
		if (pdev == to_apple_crtc(crtc)->dcp) {
			if (dcp_crtc_needs_route_start(source))
				return true;
			continue;
		}
		route = READ_ONCE(source->active_typec_route);
		if (route && dcp_typec_follows_crtc(source))
			return true;
	}
	return false;
}

static void dcp_follow_fail_commit(struct drm_crtc *crtc, struct drm_atomic_commit *state)
{
	struct drm_connector_state *conn_state;
	struct drm_connector *conn;
	int i;

	to_apple_atomic_state(state)->failed_routes |= drm_crtc_mask(crtc);
	for_each_new_connector_in_state(state, conn, conn_state, i) {
		struct apple_connector *connector = to_apple_connector(conn);
		struct platform_device *pdev = READ_ONCE(connector->dcp);
		struct apple_dcp *source;

		if (conn_state->crtc != crtc || !connector->port_encoder || !pdev)
			continue;
		source = platform_get_drvdata(pdev);
		if (source->crtc && dcp_typec_follows_crtc(source))
			to_apple_atomic_state(state)->failed_routes |=
				drm_crtc_mask(&source->crtc->base);
	}
}

/* Hold route identity through power-on/modeset; caller releases on every exit. */
int dcp_typec_follow_crtc(struct drm_crtc *crtc, struct drm_atomic_commit *state,
			  bool *locked)
{
	struct platform_device *pdev = to_apple_crtc(crtc)->dcp;
	struct apple_dcp *dcp = platform_get_drvdata(pdev);
	struct drm_connector_state *conn_state;
	struct dcp_typec_follow follow;
	struct drm_connector *conn;
	int i, ret;

	*locked = false;
	if (!dcp_typec_follows_crtc(dcp) || !dcp_typec_direct_commit(crtc, state))
		return 0;
	if (!dcp_typec_follow_lock(true)) {
		dcp_follow_fail_commit(crtc, state);
		return -EBUSY;
	}
	*locked = true;
	for_each_new_connector_in_state(state, conn, conn_state, i) {
		struct apple_connector *connector = to_apple_connector(conn);

		if (conn_state->crtc != crtc || !connector->port_encoder ||
		    READ_ONCE(connector->dcp) == pdev)
			continue;
		ret = dcp_typec_follow_decide(dcp, crtc, state, connector, &follow);
		if (!ret && follow.action != DCP_FABRIC_FOLLOW_STAY) {
			struct drm_crtc_state *new = drm_atomic_get_new_crtc_state(state, crtc);

			if (!dcp_has_mode(follow.from->dcp, &new->mode))
				ret = -EINVAL;
			else
				ret = dcp_typec_follow_move(&follow, state);
		}
		if (ret && ret != -ENOENT) {
			dcp_follow_fail_commit(crtc, state);
			return ret;
		}
	}
	return 0;
}

void dcp_typec_follow_done(bool locked)
{
	if (locked)
		dcp_typec_follow_unlock(true);
}

static struct apple_dcp_typec_port *
dcp_typec_port_get(struct device_node *connector_np)
{
	struct apple_dcp_typec_port *port, *pos;

	lockdep_assert_held(&dcp_typec_fabric_lock);

	list_for_each_entry(port, &dcp_typec_ports, link) {
		if (port->connector_np == connector_np) {
			of_node_put(connector_np);
			return port;
		}
	}

	port = kzalloc_obj(*port);
	if (!port) {
		of_node_put(connector_np);
		return NULL;
	}

	INIT_LIST_HEAD(&port->routes);
	port->connector_np = connector_np;

	/*
	 * Insert in device-tree order rather than DCP probe order.  The list
	 * index becomes the DRM connector index, and userspace keys its
	 * per-monitor configuration (scale, rotation, layout) on the connector
	 * name -- so it has to mean the same physical port on every boot.
	 */
	list_for_each_entry(pos, &dcp_typec_ports, link)
		if (strcmp(of_node_full_name(connector_np),
			   of_node_full_name(pos->connector_np)) < 0)
			break;
	list_add_tail(&port->link, &pos->link);
	return port;
}

static struct apple_dcp_typec_port *dcp_typec_port_by_index(unsigned int idx)
{
	struct apple_dcp_typec_port *port;
	unsigned int i = 0;

	lockdep_assert_held(&dcp_typec_fabric_lock);

	list_for_each_entry(port, &dcp_typec_ports, link)
		if (i++ == idx)
			return port;

	return NULL;
}

unsigned int dcp_typec_nr_ports(void)
{
	struct apple_dcp_typec_port *port;
	unsigned int n = 0;

	guard(mutex)(&dcp_typec_fabric_lock);

	list_for_each_entry(port, &dcp_typec_ports, link)
		n++;

	return n;
}

struct device_node *dcp_typec_port_of_node(unsigned int idx)
{
	struct apple_dcp_typec_port *port;

	guard(mutex)(&dcp_typec_fabric_lock);

	port = dcp_typec_port_by_index(idx);

	return port ? port->connector_np : NULL;
}

bool dcp_typec_port_has_candidate(unsigned int idx, struct platform_device *pdev)
{
	struct apple_dcp_typec_port *port;
	struct apple_dcp_typec_route *route;

	guard(mutex)(&dcp_typec_fabric_lock);

	port = dcp_typec_port_by_index(idx);
	if (!port)
		return false;

	list_for_each_entry(route, &port->routes, port_link)
		if (route->dcp->dev == &pdev->dev)
			return true;

	return false;
}

/*
 * Is every pipeline that can drive port @idx unusable (a failed T8122
 * dcpext)? Such a port gets no connector, rather than failing the DRM
 * device of the internal panel.
 */
bool dcp_typec_port_unusable(unsigned int idx)
{
	struct apple_dcp_typec_port *port;
	struct apple_dcp_typec_route *route;

	guard(mutex)(&dcp_typec_fabric_lock);

	port = dcp_typec_port_by_index(idx);
	if (!port || list_empty(&port->routes))
		return false;

	list_for_each_entry(route, &port->routes, port_link)
		if (!READ_ONCE(route->dcp->ext_dead))
			return false;

	return true;
}

void dcp_typec_port_set_connector(unsigned int idx, bool secondary,
				  struct apple_connector *connector)
{
	struct apple_dcp_typec_port *port;
	struct apple_dcp_typec_route *owner;

	guard(mutex)(&dcp_typec_fabric_lock);

	port = dcp_typec_port_by_index(idx);
	if (!port)
		return;

	if (secondary) {
		port->secondary_connector = connector;
		owner = port->secondary_owner;
	} else {
		port->connector = connector;
		owner = port->owner;
	}

	/*
	 * The port may already have been routed, either before DRM bound or
	 * while these connectors were being created.  Adopt that owner now,
	 * otherwise its display would be reported on the pipeline's fixed
	 * connector instead of the port it is actually plugged into.
	 */
	if (owner) {
		struct apple_dcp *dcp = owner->dcp;

		if (dcp->crtc && connector->port_encoder && dcp_typec_narrows(dcp))
			connector->port_encoder->possible_crtcs =
				dcp_fabric_connector_mask(false,
							  true, true,
							  drm_crtc_index(&dcp->crtc->base),
							  connector->candidate_crtcs);

		apple_connector_set_pipeline(connector, to_platform_device(dcp->dev));
		dcp->typec_connector = connector;
		dcp->connector = connector;
		dcp->connector_type = DRM_MODE_CONNECTOR_USB;
	}
}

static void dcp_typec_route_unregister(void *data)
{
	struct apple_dcp_typec_route *route = data;
	struct apple_dcp_typec_port *port = route->port;

	typec_mux_unregister(route->typec_mux);

	guard(mutex)(&dcp_typec_fabric_lock);
	if (port->preferred_route == route)
		port->preferred_route = NULL;
	if (port->split_dcp == route->dcp)
		dcp_tb_split_teardown_locked(port, true);
	if (port->owner == route) {
		struct apple_dcp *dcp = route->dcp;

		if (route->tunnel)
			dcp_tb_binding_retire(dcp, route);
		if (port->hpd || dcp->typec_cable_connected)
			dcp_dptx_disconnect_oob(to_platform_device(dcp->dev), 0);
		port->hpd = false;
		if (dcp_typec_route_deactivate(route) && dcp_is_external_h17p(dcp))
			return;
		port->owner = NULL;
	}
	if (port->secondary_owner == route) {
		struct apple_dcp *dcp = route->dcp;

		if (route->tunnel)
			dcp_tb_binding_retire(dcp, route);
		if (port->hpd || dcp->typec_cable_connected)
			dcp_dptx_disconnect_oob(to_platform_device(dcp->dev), 0);
		if (dcp_typec_route_deactivate(route) && dcp_is_external_h17p(dcp))
			return;
		port->secondary_owner = NULL;
	}
	port->hpd = !!(port->owner || port->secondary_owner);
	list_del(&route->port_link);
	if (route->dual_stream)
		atomic_dec(&dcp_dual_stream_routes);
	if (list_empty(&port->routes)) {
		list_del(&port->link);
		of_node_put(port->connector_np);
		kfree(port);
	}
}

static unsigned int dcp_typec_route_endpoints(struct device_node *connector_np)
{
	struct device_node *port __free(device_node) =
		of_graph_get_port_by_id(connector_np, 3);
	unsigned int count = 0;

	if (!port)
		return 0;
	for_each_of_graph_port_endpoint(port, endpoint)
		count++;
	return count;
}

/* Look up capability controls without selecting or changing any mux. */
static int dcp_typec_route_wiring(struct apple_dcp_typec_route *route,
				  struct device_node *connector_np, unsigned int index)
{
	struct device *dev = route->dcp->dev;
	struct mux_control *legacy = NULL;
	enum dcp_fabric_wiring wiring;
	char name[24];
	unsigned int i;
	bool named[2];

	for (i = 0; i < 2; i++) {
		snprintf(name, sizeof(name), "typec%u-dpin%u", index, i);
		named[i] = of_property_match_string(dev->of_node, "mux-control-names", name) >= 0;
		if (!named[i])
			continue;
		route->dpin[i] = devm_mux_control_get(dev, name);
		if (IS_ERR(route->dpin[i]))
			return PTR_ERR(route->dpin[i]);
		if (route->xbar != &route->xbar->chip->mux[0] ||
		    route->xbar->chip->controllers < 3 ||
		    route->dpin[i] != &route->xbar->chip->mux[1 + i])
			return -EINVAL;
	}
	if (!named[0] && !named[1]) {
		snprintf(name, sizeof(name), "typec%u-usb4", index);
		if (of_property_match_string(dev->of_node, "mux-control-names", name) >= 0) {
			legacy = devm_mux_control_get(dev, name);
			if (IS_ERR(legacy))
				return PTR_ERR(legacy);
			if (route->xbar != &route->xbar->chip->mux[0] ||
			    route->xbar->chip->controllers < 3 ||
			    legacy != &route->xbar->chip->mux[1])
				return -EINVAL;
		}
	}
	if (!named[0] && !named[1] && !legacy &&
	    apple_dp_tunnel_t6030_legacy_mux(dev->of_node, index)) {
		if (route->xbar != &route->xbar->chip->mux[0] ||
		    route->xbar->chip->controllers < 3)
			return -EINVAL;
		legacy = &route->xbar->chip->mux[1];
	}
	wiring = dcp_fabric_wiring(named[0], named[1], !!legacy,
				   dcp_typec_route_endpoints(connector_np));
	if (wiring == DCP_FABRIC_INVALID_WIRING)
		return -EINVAL;
	route->dual_stream = wiring != DCP_FABRIC_SINGLE_STREAM;
	if (wiring == DCP_FABRIC_DUAL_LEGACY) {
		/* Old j414/j416 ESP DTBs name cell 1 but never name cell 2. */
		route->dpin[0] = legacy;
		route->dpin[1] = &route->xbar->chip->mux[2];
		dev_warn_once(dev, "implicit DPIN wiring; update the device tree\n");
	}
	return 0;
}

struct dcp_route_probe_context {
	struct apple_dcp *dcp;
	struct device_node *routes;
};

static int dcp_register_typec_routes_present(void *data)
{
	struct dcp_route_probe_context *ctx = data;
	struct apple_dcp *dcp = ctx->dcp;
	struct device_node *routes = ctx->routes;
	struct device *dev = dcp->dev;
	u32 route_index;
	int ret;

	for_each_available_child_of_node_scoped(routes, route_np) {
		struct apple_dcp_typec_route *route;
		struct device_node *endpoint __free(device_node) = NULL;
		struct device_node *connector_np;
		struct typec_mux_desc desc = {};
		const char *name, *mux_name;

		if (dcp->nr_typec_routes == DCP_MAX_TYPEC_ROUTES)
			return dev_err_probe(dev, -E2BIG, "Too many Type-C display routes\n");

		ret = of_property_read_u32(route_np, "reg", &route_index);
		if (ret)
			return dev_err_probe(dev, ret, "%pOF: missing route index\n", route_np);
		if (route_index >= DCP_MAX_TYPEC_ROUTES)
			return dev_err_probe(dev, -EINVAL, "%pOF: invalid route index %u\n",
					     route_np, route_index);

		name = devm_kasprintf(dev, GFP_KERNEL, "typec%u", route_index);
		if (!name)
			return -ENOMEM;

		/*
		 * The DT lookups above are per-DCP, but the typec_mux class is
		 * global. Several DCPs can offer a route to the same Type-C port,
		 * so the registered mux needs a name unique across all of them.
		 */
		mux_name = devm_kasprintf(dev, GFP_KERNEL, "%s-typec%u",
					  dev_name(dev), route_index);
		if (!mux_name)
			return -ENOMEM;

		route = &dcp->typec_routes[dcp->nr_typec_routes];
		route->dcp = dcp;
		INIT_LIST_HEAD(&route->port_link);
		route->phy = devm_phy_get(dev, name);
		if (IS_ERR(route->phy))
			return dev_err_probe(dev, PTR_ERR(route->phy),
					     "%pOF: failed to get DP PHY\n", route_np);

		route->xbar = devm_mux_control_get(dev, name);
		if (IS_ERR(route->xbar))
			return dev_err_probe(dev, PTR_ERR(route->xbar),
					     "%pOF: failed to get display crossbar\n", route_np);

		ret = of_property_read_u32_index(dev->of_node, "apple,typec-mux-indices",
						 route_index, &route->mux_index);
		if (ret)
			return dev_err_probe(dev, ret, "%pOF: missing crossbar state\n",
					     route_np);

		ret = of_property_read_u32_index(dev->of_node, "apple,typec-dptx-phys",
						 route_index, &route->dptx_phy);
		if (ret)
			return dev_err_probe(dev, ret, "%pOF: missing DPTX PHY index\n",
					     route_np);

		endpoint = of_graph_get_next_endpoint(route_np, NULL);
		if (!endpoint)
			return dev_err_probe(dev, -EINVAL,
					     "%pOF: missing Type-C graph endpoint\n",
					     route_np);
		connector_np = of_graph_get_remote_port_parent(endpoint);
		if (!connector_np)
			return dev_err_probe(dev, -EINVAL,
					     "%pOF: missing Type-C connector\n",
					     route_np);

		ret = dcp_typec_route_wiring(route, connector_np, route_index);
		if (ret) {
			of_node_put(connector_np);
			return dev_err_probe(dev, ret, "%pOF: invalid DPIN wiring\n", route_np);
		}

		mutex_lock(&dcp_typec_fabric_lock);
		route->port = dcp_typec_port_get(connector_np);
		if (route->port) {
			list_add_tail(&route->port_link, &route->port->routes);
			if (route->dual_stream)
				atomic_inc(&dcp_dual_stream_routes);
		}
		mutex_unlock(&dcp_typec_fabric_lock);
		if (!route->port)
			return -ENOMEM;

		desc.fwnode = of_fwnode_handle(route_np);
		desc.set = dcp_typec_route_set;
		desc.prepare = dcp_typec_route_prepare;
		desc.finish = dcp_typec_route_finish;
		desc.commit = dcp_typec_route_commit;
		desc.name = mux_name;
		desc.drvdata = route;
		route->typec_mux = typec_mux_register(dev, &desc);
		if (IS_ERR(route->typec_mux)) {
			mutex_lock(&dcp_typec_fabric_lock);
			list_del(&route->port_link);
			if (route->dual_stream)
				atomic_dec(&dcp_dual_stream_routes);
			if (list_empty(&route->port->routes)) {
				list_del(&route->port->link);
				of_node_put(route->port->connector_np);
				kfree(route->port);
			}
			mutex_unlock(&dcp_typec_fabric_lock);
			return dev_err_probe(dev, PTR_ERR(route->typec_mux),
					     "%pOF: failed to register Type-C route\n", route_np);
		}

		ret = devm_add_action_or_reset(dev, dcp_typec_route_unregister, route);
		if (ret)
			return ret;

		if (!dcp->phy)
			dcp->phy = route->phy;
		dcp->nr_typec_routes++;
	}

	if (!dcp->nr_typec_routes)
		return dev_err_probe(dev, -EINVAL, "Type-C route container is empty\n");

	dcp->phy_managed_by_typec = true;
	return 0;
}

/*
 * Disables every Type-C route of @dcp in the device tree, for good. A
 * connector passes over unavailable routes when it looks for its display
 * routes, so its port no longer waits for this processor.
 */
void dcp_typec_routes_disable(struct apple_dcp *dcp)
{
	struct device_node *routes __free(device_node) =
		of_get_child_by_name(dcp->dev->of_node, "typec-routes");
	struct device_node *route;
	struct of_changeset *cs;
	int ret = 0;

	if (!routes)
		return;
	cs = kzalloc(sizeof(*cs), GFP_KERNEL);
	if (!cs)
		return;
	of_changeset_init(cs);
	for_each_available_child_of_node(routes, route) {
		ret = of_changeset_update_prop_string(cs, route, "status", "disabled");
		if (ret) {
			of_node_put(route);
			break;
		}
	}
	if (!ret)
		ret = of_changeset_apply(cs);
	if (ret) {
		dev_err(dcp->dev, "could not disable the Type-C display routes: %d\n", ret);
		of_changeset_destroy(cs);
		kfree(cs);
	}
	/* Applied, it stays: the live tree refers to it. */
}

int dcp_register_typec_routes(struct apple_dcp *dcp)
{
	struct device_node *routes __free(device_node) =
		of_get_child_by_name(dcp->dev->of_node, "typec-routes");
	struct dcp_route_probe_context ctx = { .dcp = dcp, .routes = routes };

	return dcp_fabric_run_probe(!!routes, dcp_register_typec_routes_present, &ctx);
}

/*
 * A hybrid let go while its HDMI port was empty was parked on a Type-C PHY
 * (see dcp_typec_route_deactivate()).  Point it at the HDMI output again
 * before connecting a display that has arrived there.
 */
static void dcp_fixed_restore_phy(void *ctx)
{
	struct apple_dcp *dcp = ctx;

	dcp->phy = dcp->fixed_phy;
	dcp->dptx_phy = dcp->fixed_dptx_phy;
}

static int dcp_fixed_select_mux(void *ctx)
{
	struct apple_dcp *dcp = ctx;
	int ret = mux_control_select(dcp->xbar, dcp->fixed_mux_index);

	if (!ret)
		dcp->fixed_route_selected = true;
	return ret;
}

static const struct dcp_fabric_fixed_ops dcp_fixed_ops = {
	.restore_phy = dcp_fixed_restore_phy,
	.select_mux = dcp_fixed_select_mux,
};

static int dcp_fixed_output_select(struct apple_dcp *dcp)
{
	lockdep_assert_held(&dcp_typec_fabric_lock);
	return dcp_fabric_run_fixed(!!dcp->fixed_phy, !!dcp->active_typec_route,
				    dcp->xbar && !dcp->fixed_route_selected,
				    &dcp_fixed_ops, dcp);
}

/*
 * Any HDMI HPD edge: a display is there or just was.  Start the hold at the
 * edge itself, before a borrower can take the fabric lock ahead of the
 * thread.
 */
irqreturn_t dcp_dp2hdmi_hpd_edge(int irq, void *data)
{
	struct apple_dcp *dcp = data;

	WRITE_ONCE(dcp->hdmi_edge_jiffies, jiffies);
	WRITE_ONCE(dcp->hdmi_edge_seen, true);
	dcp_hdmi_hpd_edge(data);
	dcp_hdmi_edge(data);

	return IRQ_WAKE_THREAD;
}

static bool dcp_hdmi_borrowed(void *ctx)
{
	struct apple_dcp *dcp = ctx;

	return !!READ_ONCE(dcp->active_typec_route);
}

static bool dcp_hdmi_keep_order(void *ctx)
{
	return dcp_typec_keep_order();
}

static bool dcp_hdmi_dual_stream(void *ctx)
{
	return dcp_typec_ordered_dual_stream();
}

static int dcp_hdmi_read_hpd(void *ctx)
{
	struct apple_dcp *dcp = ctx;

	return gpiod_get_value_cansleep(dcp->hdmi_hpd);
}

static void dcp_hdmi_wait(void *ctx, unsigned int ms)
{
	msleep(ms);
}

static void dcp_hdmi_rebalance(void *ctx)
{
	dcp_typec_rebalance_locked(NULL, 0);
}

static void dcp_hdmi_waiting(void *ctx)
{
	dcp_typec_hdmi_waits(ctx);
}

static void dcp_hdmi_observed(void *ctx, bool high, bool debounced)
{
	struct apple_dcp *dcp = ctx;

	if (debounced)
		dev_info(dcp->dev, "DP2HDMI HPD irq, 500ms debounce: connected:%d\n", high);
	else
		dev_info(dcp->dev, "DP2HDMI HPD irq, connected:%d\n", high);
}

/* A failed HDMI connect that a retry may fix; see dcp_fabric_hdmi_retry(). */
static void dcp_hdmi_connect_failed(struct apple_dcp *dcp, int ret)
{
	if (dcp->external_native && ret && ret != -EAGAIN && ret != -ESTALE &&
	    ret != -ESHUTDOWN)
		dcp_external_retry(dcp, "HDMI display link not set up", ret, 1000);
}

static void dcp_hdmi_connect_fixed(void *ctx)
{
	struct apple_dcp *dcp = ctx;
	int ret = dcp_fixed_output_select(dcp);

	/*
	 * A new HDMI attach gets its own retries. The converter's HPD pulses
	 * also land here: while a display is kept connected through a
	 * withdrawal, keep the retry that bounds that.
	 */
	if (dcp->external_native && !atomic_read(&dcp->external_held)) {
		atomic_set(&dcp->external_retries, 0);
		cancel_delayed_work(&dcp->external_retry_wq);
	}
	if (ret)
		dev_err(dcp->dev, "could not select the HDMI output: %d\n", ret);
	else
		ret = dcp_dptx_connect(dcp, 0);
	dcp_hdmi_connect_failed(dcp, ret);
}

/*
 * An HDMI display has arrived, and stayed, while its hybrid drives a
 * Type-C display that another free pipeline can drive too.  Where routes
 * follow their CRTC, hand the hybrid back: the Type-C display goes, the
 * HDMI display connects, and the Type-C display returns on the other
 * pipeline.  The compositor sees an unplug and a replug, drops the CRTC it
 * gave the Type-C display, and finds the HDMI display there before the
 * Type-C display is back to be paired with the other CRTC.  In the other
 * order it would pair the returning display with the hybrid's CRTC again.
 * Without such a pipeline, or on dual-stream machines, the HDMI display
 * waits as before.
 */
#define DCP_RECLAIM_HDMI_MS	3000

struct dcp_hdmi_reclaim_context {
	struct apple_dcp *from;
	struct apple_dcp_typec_route *owner;
	struct apple_dcp_typec_route *target;
	struct dcp_typec_follow_slot slot;
};

static int dcp_reclaim_release(void *data)
{
	struct dcp_hdmi_reclaim_context *ctx = data;

	return dcp_follow_release(ctx->owner);
}

static bool dcp_reclaim_retained(void *data)
{
	struct dcp_hdmi_reclaim_context *ctx = data;

	return dcp_follow_owned(ctx->owner);
}

static void dcp_reclaim_unplug(void *data)
{
	struct dcp_hdmi_reclaim_context *ctx = data;
	struct apple_connector *connector = ctx->slot.connector;

	if (connector) {
		WRITE_ONCE(connector->connected, false);
		apple_connector_set_pipeline(connector, NULL);
		dcp_queue_hotplug(connector);
	}
}

static void dcp_reclaim_connect_hdmi(void *data)
{
	struct dcp_hdmi_reclaim_context *ctx = data;
	struct apple_dcp *dcp = ctx->from;
	struct apple_connector *hdmi = dcp->fixed_connector;
	unsigned long timeout;

	/* Park revoked the old attachment before this fixed output starts. */
	WRITE_ONCE(dcp->typec_follow_retiring, false);
	dcp_hdmi_connect_fixed(dcp);
	timeout = jiffies + msecs_to_jiffies(DCP_RECLAIM_HDMI_MS);
	while (hdmi && !READ_ONCE(hdmi->connected) && time_before(jiffies, timeout))
		msleep(20);
}

static int dcp_reclaim_activate(void *data, bool restore)
{
	struct dcp_hdmi_reclaim_context *ctx = data;
	struct apple_dcp_typec_route *route = restore ? ctx->owner : ctx->target;
	int ret = dcp_follow_activate(route, &ctx->slot);

	if (ret)
		dev_err(route->dcp->dev, "could not %s display route on %pOF: %d\n",
			 restore ? "restore" : "move", ctx->slot.port->connector_np, ret);
	return ret;
}

static void dcp_reclaim_publish(void *data, bool restore)
{
	struct dcp_hdmi_reclaim_context *ctx = data;
	struct apple_dcp *to = (restore ? ctx->owner : ctx->target)->dcp;

	/* Connected on a fresh attachment; no CRTC is on yet. */
	scoped_guard(mutex, &to->hpd_mutex) {
		WRITE_ONCE(to->typec_cable_connected, true);
		to->typec_generation++;
		to->typec_follow_start = false;
		WRITE_ONCE(to->typec_crtc_off, false);
	}
	WRITE_ONCE(ctx->owner->dcp->typec_follow_retiring, false);
	WRITE_ONCE(ctx->target->dcp->typec_follow_retiring, false);
	dcp_dptx_connect_oob(to_platform_device(to->dev), 0);
}

static void dcp_reclaim_lost(void *data)
{
	struct dcp_hdmi_reclaim_context *ctx = data;

	if (ctx->slot.connector)
		dcp_route_failure_notify(ctx->slot.connector);
}

static const struct dcp_fabric_reclaim_ops dcp_reclaim_ops = {
	.release = dcp_reclaim_release,
	.retained = dcp_reclaim_retained,
	.unplug = dcp_reclaim_unplug,
	.connect_hdmi = dcp_reclaim_connect_hdmi,
	.activate = dcp_reclaim_activate,
	.publish = dcp_reclaim_publish,
	.lost = dcp_reclaim_lost,
};

static bool dcp_typec_hdmi_reclaim(struct apple_dcp *dcp)
{
	struct apple_dcp_typec_route *owner = dcp->active_typec_route;
	struct dcp_typec_follow_slot slot = {};
	struct apple_dcp_typec_route *route, *target = NULL;
	struct apple_connector *connector;
	struct apple_dcp_typec_port *port;
	struct apple_dcp *to;
	struct dcp_hdmi_reclaim_context ctx;

	lockdep_assert_held(&dcp_typec_fabric_lock);
	if (!owner || dcp_typec_ordered_dual_stream())
		return false;
	port = owner->port;
	/* A port in SAFE keeps its owner until the mux set phase completes. */
	if (port->owner != owner || port->safe_pending ||
	    !dcp_typec_port_follows(port))
		return false;
	list_for_each_entry(route, &port->routes, port_link) {
		if (route == owner || !route->dcp->crtc ||
		    !dcp_typec_route_available(route) ||
		    (owner->tunnel && !dcp_typec_tunnel_ctl(route, owner->tunnel_dpin)) ||
		    (port->split_dcp == dcp && !dcp_follow_split_fits(route)))
			continue;
		target = route;
		break;
	}
	if (!target)
		return false;

	/* not a blink */
	msleep(500);
	if (gpiod_get_value_cansleep(dcp->hdmi_hpd) <= 0)
		return false;

	connector = dcp->typec_connector;
	slot.port = port;
	slot.connector = connector;
	slot.tunnel = owner->tunnel;
	slot.dpin = owner->tunnel_dpin;
	if (slot.tunnel) {
		scoped_guard(mutex, &dcp->tb_lock) {
			slot.tunnel_generation = owner->tunnel_generation;
			slot.set_active = dcp->tb_dpin_set_active;
			slot.binding = dcp->tb_dpin_ctx;
		}
		if (!slot.tunnel_generation || !slot.set_active)
			return false;
	}
	dcp_follow_save_split(owner, &slot);
	to = target->dcp;
	dev_info(dcp->dev, "HDMI display takes its pipeline back, %s on %pOF moves to %s\n",
		 connector ? connector->base.name : "the display",
		 port->connector_np, dev_name(to->dev));
	ctx = (struct dcp_hdmi_reclaim_context) {
		.from = dcp, .owner = owner, .target = target, .slot = slot,
	};
	WRITE_ONCE(dcp->typec_follow_retiring, true);
	WRITE_ONCE(to->typec_follow_retiring, true);
	dcp_fabric_reclaim_execute(&dcp_reclaim_ops, &ctx);
	WRITE_ONCE(dcp->typec_follow_retiring, false);
	WRITE_ONCE(to->typec_follow_retiring, false);
	return true;
}

/*
 * The bounded retry of a native external pipe, for its HDMI output: the
 * display is attached, but its link did not come up. Set the output up
 * again, in case the PHY or crossbar lost their setup, and connect again.
 */
void dcp_fabric_hdmi_retry(struct apple_dcp *dcp)
{
	int ret;

	guard(mutex)(&dcp_typec_fabric_lock);
	if (dcp->active_typec_route || gpiod_get_value_cansleep(dcp->hdmi_hpd) <= 0) {
		dev_info(dcp->dev, "display retry: the HDMI port has no display any more\n");
		return;
	}
	dev_info(dcp->dev, "display retry: connecting the HDMI display again\n");
	/* Releases a half-made connection; a no-op otherwise. */
	ret = dcp_dptx_disconnect(dcp, 0);
	if (ret) {
		dcp_hdmi_connect_failed(dcp, ret);
		return;
	}
	dcp_fixed_hdmi_reinit_locked(dcp, "its display link did not come up");
	ret = dcp_fixed_output_select(dcp);
	if (!ret)
		ret = dcp_dptx_connect(dcp, 0);
	dcp_hdmi_connect_failed(dcp, ret);
}

static const struct dcp_fabric_hdmi_ops dcp_hdmi_ops = {
	.borrowed = dcp_hdmi_borrowed,
	.keep_order = dcp_hdmi_keep_order,
	.dual_stream = dcp_hdmi_dual_stream,
	.read_hpd = dcp_hdmi_read_hpd,
	.wait = dcp_hdmi_wait,
	.rebalance = dcp_hdmi_rebalance,
	.waiting = dcp_hdmi_waiting,
	.observed = dcp_hdmi_observed,
	.connect_fixed = dcp_hdmi_connect_fixed,
};

static void dcp_hdmi_update_locked(struct apple_dcp *dcp)
{
	unsigned long flags;
	u64 generation;
	int level;

	lockdep_assert_held(&dcp_typec_fabric_lock);
	if (dcp_hdmi_settle_enabled(dcp)) {
		spin_lock_irqsave(&dcp->hdmi_presence_lock, flags);
		generation = dcp->hdmi_presence.generation;
		spin_unlock_irqrestore(&dcp->hdmi_presence_lock, flags);
		level = gpiod_get_value_cansleep(dcp->hdmi_hpd);
		dcp_hdmi_sample(dcp, generation, level);
	}
	dcp_fabric_run_hdmi(&dcp_hdmi_ops, dcp);
}

irqreturn_t dcp_dp2hdmi_hpd(int irq, void *data)
{
	struct apple_dcp *dcp = data;

	guard(mutex)(&dcp_typec_fabric_lock);
	dcp_hdmi_update_locked(dcp);
	/* The GPIO edge detector is off throughout an oneshot handler. */
	if (dcp_hdmi_settle_enabled(dcp))
		mod_delayed_work(system_freezable_wq, &dcp->hdmi_recheck_wq, 0);
	return IRQ_HANDLED;
}

void dcp_link(struct platform_device *pdev, struct apple_crtc *crtc,
	      struct apple_connector *connector)
{
	struct apple_dcp *dcp = platform_get_drvdata(pdev);

	WRITE_ONCE(dcp->crtc, crtc);

	/*
	 * Type-C connectors belong to physical ports and are bound by the
	 * display fabric when a pipeline takes a route, so a pipeline with no
	 * fixed output simply has no connector until then.
	 */
	if (!connector)
		return;

	dcp->fixed_connector = connector;
	if (!dcp->active_typec_route) {
		dcp->connector = connector;
		dcp->connector_type = dcp->fixed_connector_type;
	}
}

/* Called after component unbind has drained all firmware callbacks. The
 * platform devices and their Type-C muxes can outlive this DRM instance.
 */
void dcp_unlink(struct drm_device *drm)
{
	struct apple_dcp_typec_port *port;
	struct drm_crtc *crtc;

	guard(mutex)(&dcp_typec_fabric_lock);

	list_for_each_entry(port, &dcp_typec_ports, link) {
		if (port->connector && port->connector->base.dev == drm)
			port->connector = NULL;
		if (port->secondary_connector &&
		    port->secondary_connector->base.dev == drm)
			port->secondary_connector = NULL;
	}

	drm_for_each_crtc(crtc, drm) {
		struct apple_crtc *apple_crtc = to_apple_crtc(crtc);
		struct apple_dcp *dcp;

		if (!apple_crtc->dcp)
			continue;
		dcp = platform_get_drvdata(apple_crtc->dcp);
		if (dcp->crtc != apple_crtc)
			continue;
		/*
		 * A DCP that kept its DMA pinned this DRM device for its late
		 * firmware callbacks; its references stay valid, keep them.
		 */
		if (READ_ONCE(dcp->retain_dma))
			continue;
		dcp->crtc = NULL;
		dcp->connector = NULL;
		dcp->fixed_connector = NULL;
		dcp->typec_connector = NULL;
		WRITE_ONCE(dcp->ext_backlight, false);
	}
}

void dcp_fabric_shutdown_dptx(struct apple_dcp *dcp)
{
	if (dcp->dptxep) {
		/* Mux/tunnel callbacks must stop using the service before its
		 * endpoint is released. Firmware callbacks are drained by AFK.
		 */
		guard(mutex)(&dcp_typec_fabric_lock);

		afk_shutdown(dcp->dptxep);
		dcp->dptxep = NULL;
		scoped_guard(mutex, &dcp->hpd_mutex) {
			for (int i = 0; i < ARRAY_SIZE(dcp->dptxport); i++) {
				dcp->dptxport[i].enabled = false;
				dcp->dptxport[i].connected = false;
				dcp->dptxport[i].service = NULL;
			}
		}
	}
}
