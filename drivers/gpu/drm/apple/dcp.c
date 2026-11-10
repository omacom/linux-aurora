// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Copyright 2021 Alyssa Rosenzweig */

#include <linux/align.h>
#include <linux/bitmap.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/component.h>
#include <linux/delay.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/dma-mapping.h>
#include <linux/gpio/consumer.h>
#include <linux/iommu.h>
#include <linux/jiffies.h>
#include <linux/kconfig.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/of_platform.h>
#include <linux/slab.h>
#include <linux/soc/apple/rtkit.h>
#include <linux/string.h>
#include <linux/usb/typec_altmode.h>
#include <linux/usb/typec_dp.h>
#include <linux/usb/typec_mux.h>
#include <linux/workqueue.h>

#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_module.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_vblank.h>

#include "afk.h"
#include "av.h"
#include "dcp.h"
#include "dcp-internal.h"
#include "dcp-lifecycle.h"
#include "iomfb.h"
#include "parser.h"
#include "trace.h"

#define APPLE_DCP_COPROC_CPU_CONTROL	 0x44
#define APPLE_DCP_COPROC_CPU_STATUS	 0x48
#define APPLE_DCP_RTKIT_HANDOFF_PROPERTY "apple,dcp-rtkit-quiesced"

#define DCP_BOOT_TIMEOUT msecs_to_jiffies(1000)

static bool show_notch;
module_param(show_notch, bool, 0644);
MODULE_PARM_DESC(show_notch, "Use the full display height and shows the notch");

bool hdmi_audio;
module_param(hdmi_audio, bool, 0644);
MODULE_PARM_DESC(hdmi_audio, "Enable unstable HDMI audio support");

static bool unstable_edid = true;
module_param(unstable_edid, bool, 0644);
MODULE_PARM_DESC(unstable_edid, "Enable unstable EDID retrival support");

/* Diagnostic for a preloaded, still-running DCP. The native T8142 macOS
 * trace starts with INIT (0x220), HELLO and EPMAP without resetting the ASC.
 * Keep this opt-in until that handshake is qualified on the Linux handoff.
 */
static bool live_discovery;
module_param(live_discovery, bool, 0400);
MODULE_PARM_DESC(live_discovery, "Negotiate RTKit on the inherited live DCP without an ASC reset");

struct apple_dcp_typec_port {
	struct list_head link;
	struct list_head routes;
	struct device_node *connector_np;
	struct apple_dcp_typec_route *owner;
	/* DRM connector for this physical port, driven by whichever DCP owns it */
	struct apple_connector *connector;
	/* last mux state acted on, to collapse the per-candidate notifications */
	struct typec_altmode *applied_alt;
	unsigned long applied_mode;
	u32 applied_status;
	u32 applied_conf;
	bool applied_valid;
	bool hpd;
};

static DEFINE_MUTEX(dcp_typec_fabric_lock);
static LIST_HEAD(dcp_typec_ports);

static int dcp_dptx_connect(struct apple_dcp *dcp, u32 port);
static void disconnected_hpd_event(struct apple_connector *connector);

bool dcp_is_typec_output(struct apple_dcp *dcp)
{
	return dcp->active_typec_route ||
	       dcp->fixed_connector_type == DRM_MODE_CONNECTOR_USB;
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
	if (dcp->fixed_connector && dcp->fixed_connector->connected)
		return true;
	if (dcp->hdmi_hpd && gpiod_get_value_cansleep(dcp->hdmi_hpd))
		return true;

	return false;
}

static bool dcp_typec_route_available(struct apple_dcp_typec_route *route)
{
	return !route->dcp->active_typec_route &&
	       !dcp_typec_route_fixed_output_busy(route);
}

static int dcp_typec_route_activate(struct apple_dcp_typec_route *route);
static int dcp_typec_route_deactivate(struct apple_dcp_typec_route *route);
static int dcp_dptx_disconnect(struct apple_dcp *dcp, u32 port);
static void _dcp_poweroff(struct apple_dcp *dcp);

/*
 * Pipelines are ranked by CRTC index so the fabric's choice is a pure function
 * of the topology rather than of plug order.  A pipeline whose fixed output is
 * live is not a candidate at all, so a hybrid is only ever ranked here when it
 * is genuinely free.
 */
static unsigned int dcp_typec_route_score(struct apple_dcp_typec_route *route)
{
	struct apple_dcp *dcp = route->dcp;

	if (!dcp->crtc)
		return UINT_MAX - 1;

	return drm_crtc_index(&dcp->crtc->base);
}

static int dcp_typec_route_activate(struct apple_dcp_typec_route *route)
{
	struct apple_dcp *dcp = route->dcp;
	int ret;

	/*
	 * The fixed output's HPD handler leaves disconnects to DCP, so the port
	 * can still be marked connected to a display that is gone. Release it
	 * (a no-op otherwise), or connecting the borrowed route returns early.
	 */
	dcp_dptx_disconnect(dcp, 0);

	if (dcp->fixed_route_selected) {
		ret = mux_control_deselect(dcp->xbar);
		if (ret)
			return ret;
		dcp->fixed_route_selected = false;
	}

	ret = mux_control_select(route->xbar, route->mux_index);
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

	dcp->phy = route->phy;
	dcp->dptx_phy = route->dptx_phy;
	dcp->connector_type = DRM_MODE_CONNECTOR_USB;
	if (route->port->connector) {
		route->port->connector->dcp = to_platform_device(dcp->dev);
		dcp->typec_connector = route->port->connector;
		dcp->connector = route->port->connector;

		/*
		 * Narrow the port to the pipeline now driving it.  The encoder
		 * spans every pipeline that could, which is what lets userspace
		 * see the port as usable at all -- but only one of them is
		 * routed to the display, and userspace has no way to tell which.
		 * Offering it the choice makes it pair the port with a pipeline
		 * holding a different monitor's mode list, and the modeset is
		 * rejected with no way for it to recover.  The hotplug that
		 * follows makes it re-read this.
		 */
		if (route->port->connector->port_encoder && dcp->crtc)
			route->port->connector->port_encoder->possible_crtcs =
				drm_crtc_mask(&dcp->crtc->base);
	}
	dcp->active_typec_route = route;
	route->selected = true;

	dev_info(dcp->dev, "allocated Type-C DPTX PHY %u\n", route->dptx_phy);
	return 0;
}

static int dcp_typec_route_deactivate(struct apple_dcp_typec_route *route)
{
	struct apple_dcp *dcp = route->dcp;
	int ret;

	ret = mux_control_deselect(route->xbar);
	if (ret)
		return ret;

	route->selected = false;
	if (dcp->active_typec_route == route)
		dcp->active_typec_route = NULL;

	if (route->port->connector &&
	    route->port->connector->dcp == to_platform_device(dcp->dev)) {
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
		WRITE_ONCE(route->port->connector->connected, false);
		route->port->connector->dcp = NULL;

		/* Unrouted: the port could go to any of its pipelines again. */
		if (route->port->connector->port_encoder)
			route->port->connector->port_encoder->possible_crtcs =
				route->port->connector->candidate_crtcs;
	}
	dcp->typec_connector = NULL;
	dcp->connector = dcp->fixed_connector;

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

			if (dcp->xbar) {
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

static void dcp_typec_retrain_work(struct work_struct *work)
{
	struct apple_dcp *dcp =
		container_of(to_delayed_work(work), struct apple_dcp,
			     typec_fabric_retrain_wq);

	if (READ_ONCE(dcp->active_typec_route) && dcp->typec_connector)
		dcp_retrain_oob(dcp->typec_connector);
}

static void dcp_typec_retrain_active_routes(void)
{
	struct apple_dcp_typec_port *port;

	list_for_each_entry(port, &dcp_typec_ports, link) {
		if (!port->owner)
			continue;
		mod_delayed_work(system_freezable_wq,
				 &port->owner->dcp->typec_fabric_retrain_wq,
				 msecs_to_jiffies(200));
	}
}

static int dcp_typec_route_set(struct typec_mux_dev *mux,
			       struct typec_mux_state *state)
{
	struct apple_dcp_typec_route *route = typec_mux_get_drvdata(mux);
	struct apple_dcp_typec_port *port = route->port;
	struct apple_dcp_typec_route *candidate, *best = NULL;
	bool is_dp = dcp_typec_route_is_dp(state);
	struct typec_displayport_data *dp_data = is_dp ? state->data : NULL;
	u32 dp_status = dp_data ? dp_data->status : 0;
	u32 dp_conf = dp_data ? dp_data->conf : 0;
	unsigned int best_score = UINT_MAX;
	bool hpd;
	int ret = 0;

	guard(mutex)(&dcp_typec_fabric_lock);

	/*
	 * Every candidate route for this port is notified with the same state,
	 * so only the first to arrive does the work; the others return early.
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
	port->applied_valid = true;

	if (!is_dp) {
		if (port->owner) {
			struct apple_dcp *dcp = port->owner->dcp;

			if (port->hpd || dcp->typec_cable_connected ||
			    (dcp->typec_connector &&
			     dcp->typec_connector->connected))
				dcp_dptx_disconnect_oob(to_platform_device(dcp->dev), 0);
			port->hpd = false;
			ret = dcp_typec_route_deactivate(port->owner);
			if (ret)
				return ret;
			port->owner = NULL;
			if (dcp->hdmi_hpd && dcp->active &&
			    gpiod_get_value_cansleep(dcp->hdmi_hpd))
				dcp_dptx_connect(dcp, 0);
		}

		if (state->mode == TYPEC_MODE_USB4)
			dcp_typec_retrain_active_routes();
		return 0;
	}

	if (!port->owner) {
		list_for_each_entry(candidate, &port->routes, port_link) {
			unsigned int score;

			if (!dcp_typec_route_available(candidate))
				continue;
			score = dcp_typec_route_score(candidate);
			if (score < best_score) {
				best = candidate;
				best_score = score;
			}
		}

		if (!best)
			return -EBUSY;

		ret = dcp_typec_route_activate(best);
		if (ret)
			return ret;
		port->owner = best;
	}


	hpd = dp_data && (dp_data->status & DP_STATUS_HPD_STATE);
	if (!hpd && port->hpd) {
		dcp_dptx_disconnect_oob(to_platform_device(port->owner->dcp->dev), 0);
	} else if (hpd && !port->hpd) {
		struct apple_dcp *dcp = port->owner->dcp;

		WRITE_ONCE(dcp->typec_cable_connected, true);
		if (dcp->typec_connector)
			dcp_dptx_connect_oob(to_platform_device(dcp->dev), 0);
	} else if (hpd && dp_data && (dp_data->status & DP_STATUS_IRQ_HPD)) {
		struct apple_dcp *dcp = port->owner->dcp;

		if (dcp->typec_connector)
			dcp_retrain_oob(dcp->typec_connector);
	}
	port->hpd = hpd;

	return 0;
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

void dcp_typec_port_set_connector(unsigned int idx,
				  struct apple_connector *connector)
{
	struct apple_dcp_typec_port *port;

	guard(mutex)(&dcp_typec_fabric_lock);

	port = dcp_typec_port_by_index(idx);
	if (!port)
		return;

	port->connector = connector;

	/*
	 * The port may already have been routed, either before DRM bound or
	 * while these connectors were being created.  Adopt that owner now,
	 * otherwise its display would be reported on the pipeline's fixed
	 * connector instead of the port it is actually plugged into.
	 */
	if (port->owner) {
		struct apple_dcp *dcp = port->owner->dcp;

		connector->dcp = to_platform_device(dcp->dev);
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
	if (port->owner == route) {
		struct apple_dcp *dcp = route->dcp;

		if (port->hpd || dcp->typec_cable_connected)
			dcp_dptx_disconnect_oob(to_platform_device(dcp->dev), 0);
		port->hpd = false;
		dcp_typec_route_deactivate(route);
		port->owner = NULL;
	}
	list_del(&route->port_link);
	if (list_empty(&port->routes)) {
		list_del(&port->link);
		of_node_put(port->connector_np);
		kfree(port);
	}
}

static int dcp_register_typec_routes(struct apple_dcp *dcp)
{
	struct device_node *routes __free(device_node) =
		of_get_child_by_name(dcp->dev->of_node, "typec-routes");
	struct device *dev = dcp->dev;
	u32 route_index;
	int ret;

	if (!routes)
		return 0;

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

		mutex_lock(&dcp_typec_fabric_lock);
		route->port = dcp_typec_port_get(connector_np);
		if (route->port)
			list_add_tail(&route->port_link, &route->port->routes);
		mutex_unlock(&dcp_typec_fabric_lock);
		if (!route->port)
			return -ENOMEM;

		desc.fwnode = of_fwnode_handle(route_np);
		desc.set = dcp_typec_route_set;
		desc.name = mux_name;
		desc.drvdata = route;
		route->typec_mux = typec_mux_register(dev, &desc);
		if (IS_ERR(route->typec_mux)) {
			mutex_lock(&dcp_typec_fabric_lock);
			list_del(&route->port_link);
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


/* copied and simplified from drm_vblank.c */
static void send_vblank_event(struct drm_device *dev,
		struct drm_pending_vblank_event *e,
		u64 seq, ktime_t now)
{
	struct timespec64 tv;

	if (e->event.base.type != DRM_EVENT_FLIP_COMPLETE)
		return;

	tv = ktime_to_timespec64(now);
	e->event.vbl.sequence = seq;
	/*
		* e->event is a user space structure, with hardcoded unsigned
		* 32-bit seconds/microseconds. This is safe as we always use
		* monotonic timestamps since linux-4.15
		*/
	e->event.vbl.tv_sec = tv.tv_sec;
	e->event.vbl.tv_usec = tv.tv_nsec / 1000;

	/*
	 * Use the same timestamp for any associated fence signal to avoid
	 * mismatch in timestamps for vsync & fence events triggered by the
	 * same HW event. Frameworks like SurfaceFlinger in Android expects the
	 * retire-fence timestamp to match exactly with HW vsync as it uses it
	 * for its software vsync modeling.
	 */
	drm_send_event_timestamp_locked(dev, &e->base, now);
}

/**
 * dcp_crtc_send_page_flip_event - helper to send vblank event after pageflip
 *
 * Compensate for unknown slack between page flip and arrival of the
 * swap_complete callback. Minimal observed duration on DCP with HDMI output
 * was around 2.3 ms. If the fb swap was submitted closer to the expected
 * swap_complete it gets a penalty of one frame duration. This is on the border
 * of unreasonable considering that Apple advertises support for 240 Hz (frame
 * duration of 4.167 ms).
 * It is unreasonable considering kwin's kms commit scheduling. Kwin commits
 * 1.5 ms + the mode's vblank time before the expected next page flip
 * completion. This results in presenting at half the display's rate for HDMI
 * outputs.
 * This might be a difference between dcp and dcpext.
 */
static void dcp_crtc_send_page_flip_event(struct apple_crtc *crtc,
					  struct drm_pending_vblank_event *e,
					  ktime_t now, ktime_t start)
{
	struct drm_device *dev = crtc->base.dev;
	u64 seq;
	unsigned int pipe = drm_crtc_index(&crtc->base);
	ktime_t flip;

	seq = 0;
	if (start != KTIME_MIN) {
		s64 delta = ktime_us_delta(now, start);
		if (delta <= 500)
			flip = now;
		else if (delta >= 2500)
			flip = ktime_sub_us(now, 1000);
		else
			flip = ktime_sub_us(now, (delta - 500) / 2);
	} else {
		flip = now;
	}
	e->pipe = pipe;
	send_vblank_event(dev, e, seq, flip);
}

/* HACK: moved here to avoid circular dependency between apple_drv and dcp */
void dcp_drm_crtc_vblank(struct apple_crtc *crtc)
{
	unsigned long flags;

	spin_lock_irqsave(&crtc->base.dev->event_lock, flags);
	if (crtc->event) {
		drm_crtc_send_vblank_event(&crtc->base, crtc->event);
		crtc->event = NULL;
	}
	spin_unlock_irqrestore(&crtc->base.dev->event_lock, flags);
}

void dcp_drm_crtc_page_flip(struct apple_dcp *dcp, ktime_t now)
{
	unsigned long flags;
	struct apple_crtc *crtc = dcp->crtc;

	spin_lock_irqsave(&crtc->base.dev->event_lock, flags);
	if (crtc->event) {
		if (crtc->event->event.base.type == DRM_EVENT_FLIP_COMPLETE)
			dcp_crtc_send_page_flip_event(crtc, crtc->event, now, dcp->swap_start);
		else
			drm_crtc_send_vblank_event(&crtc->base, crtc->event);
		crtc->event = NULL;
		dcp->swap_start = KTIME_MIN;
	}
	spin_unlock_irqrestore(&crtc->base.dev->event_lock, flags);
}

void dcp_set_dimensions(struct apple_dcp *dcp)
{
	int i;
	int width_mm = dcp->width_mm;
	int height_mm = dcp->height_mm;

	if (width_mm == 0 || height_mm == 0) {
		width_mm = dcp->panel.width_mm;
		height_mm = dcp->panel.height_mm;
	}

	/* Set the connector info */
	if (dcp->connector) {
		struct drm_connector *connector = &dcp->connector->base;

		mutex_lock(&connector->dev->mode_config.mutex);
		connector->display_info.width_mm = width_mm;
		connector->display_info.height_mm = height_mm;
		mutex_unlock(&connector->dev->mode_config.mutex);
	}

	/*
	 * Fix up any probed modes. Modes are created when parsing
	 * TimingElements, dimensions are calculated when parsing
	 * DisplayAttributes, and TimingElements may be sent first
	 */
	for (i = 0; i < dcp->nr_modes; ++i) {
		dcp->modes[i].mode.width_mm = width_mm;
		dcp->modes[i].mode.height_mm = height_mm;
	}
}

bool dcp_has_panel(struct apple_dcp *dcp)
{
	return dcp->panel.width_mm > 0;
}

int dcp_set_crc(struct drm_crtc *crtc, bool enabled)
{
	struct apple_crtc *ac = to_apple_crtc(crtc);
	struct apple_dcp *dcp = platform_get_drvdata(ac->dcp);

	dcp->crc_enabled = enabled;

	return 0;
}

/*
 * Helper to send a DRM vblank event. We do not know how call swap_submit_dcp
 * without surfaces. To avoid timeouts in drm_atomic_helper_wait_for_vblanks
 * send a vblank event via a workqueue.
 */
static void dcp_delayed_vblank(struct work_struct *work)
{
	struct apple_dcp *dcp;

	dcp = container_of(to_delayed_work(work), struct apple_dcp, vblank_wq);
	dcp_drm_crtc_vblank(dcp->crtc);
}

static void dcp_recv_msg(void *cookie, u8 endpoint, u64 message)
{
	struct apple_dcp *dcp = cookie;

	trace_dcp_recv_msg(dcp, endpoint, message);
	if (dcp->adopting_session)
		return;

	/*
	 * We start every endpoint macOS starts (see dcp_start_aux_endpoints),
	 * but only a few have an AFK instance behind them.  An endpoint that is
	 * live but unbacked must log, never dereference a NULL afkep.
	 */
	switch (endpoint) {
	case IOMFB_ENDPOINT:
		return iomfb_recv_msg(dcp, message);
	case AV_ENDPOINT:
		if (dcp->avep)
			afk_receive_message(dcp->avep, message);
		else
			goto unbacked;
		return;
	case SYSTEM_ENDPOINT:
		if (dcp->systemep)
			afk_receive_message(dcp->systemep, message);
		else
			goto unbacked;
		return;
	case DISP0_ENDPOINT:
		if (dcp->ibootep)
			afk_receive_message(dcp->ibootep, message);
		else
			goto unbacked;
		return;
	case DPAVSERV_ENDPOINT:
		if (dcp->dcpavservep)
			afk_receive_message(dcp->dcpavservep, message);
		else
			goto unbacked;
		return;
	case DPTX_ENDPOINT:
	case DPTX_PORT_ENDPOINT_H17P:
		if (dcp->dptxep)
			afk_receive_message(dcp->dptxep, message);
		else
			goto unbacked;
		return;
	case DPTX_CONTROL_ENDPOINT:
		if (dcp->dptxcontrolep)
			afk_receive_message(dcp->dptxcontrolep, message);
		else
			goto unbacked;
		return;
	default:
		goto unbacked;
	}

unbacked:
	dev_info_ratelimited(dcp->dev, "ep %#04x: %#llx\n", endpoint, message);
}


static void dcp_rtk_crashed(void *cookie, const void *crashlog, size_t crashlog_size)
{
	struct apple_dcp *dcp = cookie;

	dcp->crashed = true;
	dev_err(dcp->dev, "DCP has crashed\n");
	if (dcp->connector) {
		dcp->connector->connected = 0;
		drm_edid_free(dcp->connector->drm_edid);
		dcp->connector->drm_edid = NULL;
		schedule_work(&dcp->connector->hotplug_wq);
	}
	complete(&dcp->start_done);
}

void dcp_dump_crashlog(struct apple_dcp *dcp)
{
	const char *p = dcp->crashlog_buf;
	size_t i, run = 0, printed = 0;

	if (!p || dcp->dumped_crashlog)
		return;
	dcp->dumped_crashlog = true;

	for (i = 0; i < dcp->crashlog_size && printed < 40; i++) {
		char c = p[i];

		if (c >= 0x20 && c < 0x7f) {
			run++;
			continue;
		}
		if (run >= 6) {
			dev_err(dcp->dev, "crashlog: %.*s\n",
				(int)min(run, (size_t)140), p + i - run);
			printed++;
		}
		run = 0;
	}
	if (!printed)
		dev_err(dcp->dev, "crashlog: nothing printable\n");
}

static int dcp_rtk_shmem_setup(void *cookie, struct apple_rtkit_shmem *bfr)
{
	struct apple_dcp *dcp = cookie;

	if (bfr->iova) {
		struct iommu_domain *domain =
			iommu_get_domain_for_dev(dcp->dev);
		phys_addr_t phy_addr;

		if (!domain)
			return -ENOMEM;

		// TODO: get map from device-tree
		phy_addr = iommu_iova_to_phys(domain, bfr->iova);
		if (!phy_addr)
			return -ENOMEM;

		/*
		 * The firmware writes these regions (its crash log above all)
		 * behind the CPU's back, so map them uncached: with a
		 * write-back mapping the crash-log probe re-read stale zeros
		 * from its own first read for the whole 36 s it looked, while
		 * a scan of the untouched remainder found the log.
		 */
		bfr->buffer = memremap(phy_addr, bfr->size, MEMREMAP_WC);
		if (!bfr->buffer)
			return -ENOMEM;

		bfr->is_mapped = true;
		dev_info(dcp->dev,
			 "shmem_setup: iova: %lx -> pa: %lx -> iomem: %lx size: %zx\n",
			 (uintptr_t)bfr->iova, (uintptr_t)phy_addr,
			 (uintptr_t)bfr->buffer, (size_t)bfr->size);

		/*
		 * The DCP hands us this region when it reports a crash on the
		 * crashlog endpoint, which it does not advertise in its EPMAP,
		 * so apple-rtkit only warns and never dumps it.  Dump the head
		 * ourselves -- it is the only account of why the firmware died.
		 *
		 * Bounded hard: a few dozen strings, once per session.  An
		 * unbounded dump drowns the 115200 console and hides the very
		 * thing it is meant to show.
		 */
		/*
		 * Remember the region rather than printing it here.  The DCP
		 * publishes it at RTKit init and the contents are whatever
		 * survived in DRAM from a previous life, so dumping it now
		 * makes an inherited panic look like a fresh one.  It is read
		 * from the stall watchdog instead, once a new record has
		 * actually been written.
		 */
		if (bfr->size >= SZ_128K) {
			dcp->crashlog_buf = bfr->buffer;
			dcp->crashlog_size = bfr->size;
		}

		if (0) {
			const char *p = bfr->buffer;
			size_t i, run = 0, printed = 0;

			dcp->dumped_crashlog = true;
			for (i = 0; i < bfr->size && printed < 24; i++) {
				char c = p[i];

				if (c >= 0x20 && c < 0x7f) {
					run++;
					continue;
				}
				if (run >= 5) {
					dev_err(dcp->dev, "crashlog: %.*s\n",
						(int)min(run, (size_t)120),
						p + i - run);
					printed++;
				}
				run = 0;
			}
			if (!printed)
				dev_err(dcp->dev,
					"crashlog: no printable records in %zx bytes\n",
					(size_t)bfr->size);
		}
	} else {
		bfr->buffer = dma_alloc_coherent(dcp->dev, bfr->size,
						 &bfr->iova, GFP_KERNEL);
		if (!bfr->buffer)
			return -ENOMEM;

		dev_info(dcp->dev, "shmem_setup: iova: %lx, buffer: %lx\n",
			 (uintptr_t)bfr->iova, (uintptr_t)bfr->buffer);
	}

	return 0;
}

static void dcp_rtk_shmem_destroy(void *cookie, struct apple_rtkit_shmem *bfr)
{
	struct apple_dcp *dcp = cookie;

	if (bfr->is_mapped)
		memunmap(bfr->buffer);
	else
		dma_free_coherent(dcp->dev, bfr->size, bfr->buffer, bfr->iova);
}

static struct apple_rtkit_ops rtkit_ops = {
	.crashed = dcp_rtk_crashed,
	.recv_message = dcp_recv_msg,
	.shmem_setup = dcp_rtk_shmem_setup,
	.shmem_destroy = dcp_rtk_shmem_destroy,
};

int dcp_start_rtkit_endpoint(struct apple_dcp *dcp, u8 endpoint)
{
	u64 message;

	if (!dcp->hw.adopt_live_session || live_discovery)
		return apple_rtkit_start_ep(dcp->rtk, endpoint);
	if (endpoint < 0x20 || endpoint >= 0x40)
		return -EINVAL;

	/* START_EP for an application service in the running generation. The
	 * loader already consumed EPMAP; do not pretend discovery was repeated.
	 */
	message = FIELD_PREP(GENMASK_ULL(59, 52), 5) |
		  FIELD_PREP(GENMASK_ULL(39, 32), endpoint) | BIT_ULL(1);
	return apple_rtkit_send_message(dcp->rtk, 0, message, NULL, false);
}

void dcp_send_message(struct apple_dcp *dcp, u8 endpoint, u64 message)
{
	trace_dcp_send_msg(dcp, endpoint, message);
	apple_rtkit_send_message(dcp->rtk, endpoint, message, NULL,
				 true);
}

int dcp_crtc_atomic_check(struct drm_crtc *crtc, struct drm_atomic_state *state)
{
	struct platform_device *pdev = to_apple_crtc(crtc)->dcp;
	struct apple_dcp *dcp = platform_get_drvdata(pdev);
	struct drm_crtc_state *crtc_state;
	bool needs_modeset;

	if (dcp->crashed)
		return -EINVAL;

	crtc_state = drm_atomic_get_new_crtc_state(state, crtc);

	needs_modeset = drm_atomic_crtc_needs_modeset(crtc_state) || !dcp->valid_mode;
	if (!needs_modeset && (!dcp->connector || !dcp->connector->connected)) {
		/*
		 * Resume restores the mode before the firmware reports the
		 * display back, so a plane-only commit lands here while the
		 * connector is still marked disconnected.  Rejecting it makes
		 * the compositor fail every flip and give up on the output;
		 * dcp_flush() defers the commit until the link returns.
		 */
		dev_dbg(dcp->dev,
			"crtc_atomic_check: deferring commit, link still down\n");
	}

	return 0;
}

int dcp_get_connector_type(struct platform_device *pdev)
{
	struct apple_dcp *dcp = platform_get_drvdata(pdev);

	return dcp->fixed_connector_type;
}

bool dcp_has_typec_routes(struct platform_device *pdev)
{
	struct apple_dcp *dcp = platform_get_drvdata(pdev);

	return dcp->nr_typec_routes;
}

#define DPTX_CONNECT_TIMEOUT msecs_to_jiffies(2000)
#define DPTX_RECONNECT_DELAY msecs_to_jiffies(1000)
#define DPTX_RECONNECT_RETRIES 5

static int dcp_dptx_connect(struct apple_dcp *dcp, u32 port)
{
	int ret = 0;

	if (!dcp->phy) {
		dev_warn(dcp->dev, "dcp_dptx_connect: missing phy\n");
		return -ENODEV;
	}
	dev_info(dcp->dev,
		 "%s(port=%d) target=%u:%u typec=%d route=%s conn_type=%d connected=%d\n",
		 __func__, port, dcp->dptx_die, dcp->dptx_phy,
		 dcp_is_typec_output(dcp),
		 dcp->active_typec_route ? "borrowed" : "fixed",
		 dcp->connector_type, dcp->dptxport[port].connected);

	mutex_lock(&dcp->hpd_mutex);
	if (!dcp->dptxport[port].enabled) {
		dev_warn(dcp->dev, "dcp_dptx_connect: dptx service for port %d not enabled\n", port);
		ret = -ENODEV;
		goto out_unlock;
	}

	if (dcp->dptxport[port].connected)
		goto out_unlock;

	reinit_completion(&dcp->dptxport[port].linkcfg_completion);
	dcp->dptxport[port].atcphy = dcp->phy;
	ret = dptxport_validate_connection(dcp->dptxport[port].service, 0,
					   dcp->dptx_phy, dcp->dptx_die);
	if (ret) {
		dev_err(dcp->dev,
			"dcp_dptx_connect: failed to validate DPTX target %u:%u: %d\n",
			dcp->dptx_die, dcp->dptx_phy, ret);
		goto out_unlock;
	}

	ret = dptxport_connect(dcp->dptxport[port].service, 0,
			       dcp->dptx_phy, dcp->dptx_die,
		       dcp_is_typec_output(dcp));
	if (ret) {
		dev_err(dcp->dev,
			"dcp_dptx_connect: failed to connect DPTX target %u:%u: %d\n",
			dcp->dptx_die, dcp->dptx_phy, ret);
		goto out_unlock;
	}

	ret = dptxport_request_display(dcp->dptxport[port].service);
	if (ret) {
		dev_err(dcp->dev,
			"dcp_dptx_connect: failed to request display: %d\n",
			ret);
		goto out_release;
	}
	dcp->dptxport[port].connected = true;
	if (dcp_is_typec_output(dcp)) {
		ret = dptxport_set_hpd(dcp->dptxport[port].service, true);
		if (ret) {
			dev_err(dcp->dev,
				"dcp_dptx_connect: failed to assert Type-C HPD: %d\n",
				ret);
			dcp->dptxport[port].connected = false;
			goto out_release;
		}
	}

	mutex_unlock(&dcp->hpd_mutex);
	ret = wait_for_completion_timeout(&dcp->dptxport[port].linkcfg_completion,
				    DPTX_CONNECT_TIMEOUT);
	if (!ret) {
		dev_err(dcp->dev,
			"dcp_dptx_connect: timed out waiting for port %u link configuration\n",
			port);
		ret = -ETIMEDOUT;
		goto out_disconnect;
	}

	dev_dbg(dcp->dev, "dcp_dptx_connect: waited %d ms for link\n",
		jiffies_to_msecs(DPTX_CONNECT_TIMEOUT - ret));

	usleep_range(5, 10);

	if (dcp->connector_type == DRM_MODE_CONNECTOR_DisplayPort)
		dptxport_set_hpd(dcp->dptxport[port].service, true);

	if (dcp->avep)
		av_service_connect(dcp);

	return 0;

out_disconnect:
	mutex_lock(&dcp->hpd_mutex);
	dcp->dptxport[port].connected = false;
out_release:
	dptxport_release_display(dcp->dptxport[port].service);

out_unlock:
	mutex_unlock(&dcp->hpd_mutex);
	return ret;
}

static void dcp_typec_reconnect_work(struct work_struct *work)
{
	struct apple_dcp *dcp =
		container_of(to_delayed_work(work), struct apple_dcp,
			     typec_reconnect_wq);
	int ret;

	if (!READ_ONCE(dcp->typec_cable_connected))
		return;

	ret = dcp_dptx_connect(dcp, 0);
	if (!ret) {
		dcp->typec_reconnect_tries = 0;
		return;
	}

	if (++dcp->typec_reconnect_tries < DPTX_RECONNECT_RETRIES) {
		mod_delayed_work(system_freezable_wq, &dcp->typec_reconnect_wq,
				 DPTX_RECONNECT_DELAY);
		return;
	}

	dev_err(dcp->dev, "Type-C DPTX reconnect failed after %u retries: %d\n",
		dcp->typec_reconnect_tries, ret);
}

static void disconnected_hpd_event(struct apple_connector *con)
{
	if (con && con->connected) {
		con->connected = 0;
		drm_edid_free(con->drm_edid);
		con->drm_edid = NULL;
		drm_kms_helper_connector_hotplug_event(&con->base);
	}
}

static int dcp_dptx_disconnect(struct apple_dcp *dcp, u32 port)
{
	dev_info(dcp->dev, "%s(port=%d)\n", __func__, port);

	mutex_lock(&dcp->hpd_mutex);
	if (dcp->dptxport[port].enabled && dcp->dptxport[port].connected) {
		dptxport_release_display(dcp->dptxport[port].service);
		dcp->dptxport[port].connected = false;
	}
	mutex_unlock(&dcp->hpd_mutex);

	return 0;
}

/*
 * A Type-C mux provided by the DCP itself, so that a DisplayPort alt-mode
 * event reaches it.
 *
 * The obvious route -- typec_displayport -> drm_connector_oob_hotplug_event()
 * -> apple_connector_oob_hotplug() -> dcp_dptx_connect_oob() -- cannot work on
 * this machine.  typec_displayport binds a *partner* alt mode, and the CD321x
 * firmware owns alt-mode negotiation here: the PD driver registers the port's
 * own alt modes but never calls typec_partner_register_altmode(), so there is
 * no partner alt mode, the module never binds, and that entire chain is inert.
 *
 * Measured on J700 2026-09-22 with a DisplayLink UD-ULTC4K: the CD321x
 * negotiates DP (dp=1, pin assignment D) and the ATC PHY enters
 * APPLE_ATCPHY_MODE_USB3_DP successfully, while the external DCP sits powered
 * down from boot ("HPD not present, skipping mode export") and is never told a
 * display arrived.
 *
 * Registering the DCP as a second mux on the same connector means the
 * typec_mux_set() that already drives the ATC PHY drives the DCP too, with no
 * partner alt mode and no DRM out-of-band hotplug involved.  This is the
 * notification half of the Type-C route fabric in DJ's linux-aurora
 * b486dc811e58, without the multi-DCP arbitration that machine needs and this
 * one does not: the J700 has one external DCP, one ATC PHY and exactly one
 * DP-capable connector.
 */
/*
 * Which display crossbar input feeds the ATC0 DisplayPort output.
 *
 * From the J700 ADT, display-crossbar0:
 *   dp-switch-ufp-endpoint = 1   (dispext1)
 *   dp-switch-ufp-port     = 0
 *   dp-switch-dfp-endpoint = 0   (atc0)
 * and the crossbar selector is (ufp_endpoint << 1) | (ufp_port & 1) = 2.
 *
 * Selector 0 is NOT "the first input" -- it is the idle/disconnected value,
 * which is why routing 0 looked like it worked (the mux logged a state
 * change) while connecting the DCP's real DP stream to nothing at all.
 */
#define DCP_DPXBAR_SELECTOR 2

static int dcp_typec_mux_set(struct typec_mux_dev *mux,
			     struct typec_mux_state *state)
{
	struct apple_dcp *dcp = typec_mux_get_drvdata(mux);
	bool is_dp = state->alt && state->alt->svid == USB_TYPEC_DP_SID;

	dev_info(dcp->dev, "J700_DCP_TYPEC_MUX: mode=%lu svid=%#x -> %s\n",
		 state->mode, state->alt ? state->alt->svid : 0,
		 is_dp ? "connect" : "disconnect");

	/*
	 * Deliberately do NOT deselect the crossbar here.  The Type-C mux
	 * delivers idle/registration events with mode=0 svid=0 right after
	 * probe, and treating those as "unplugged" tore the route down again
	 * immediately -- the probe-time select lasted microseconds.  This
	 * machine has one external DCP and one DP-capable connector, so the
	 * dispext0 -> DP route is static; leaving it up costs nothing and
	 * keeps the firmware's view of the core stable across unplugs.
	 */
	if (!is_dp)
		return dcp_dptx_disconnect_oob(to_platform_device(dcp->dev), 0);

	/*
	 * Route dispext0 onto the ATC0 DP lanes.  Until this happens the DCP's
	 * detection on the external core never leaves "noconnect", so the
	 * firmware never creates an AppleDCPDPTXVirtualPort, never starts
	 * AppleDCPDPTXRemotePort, and never announces "dcpdptx-port-epic".
	 */
	if (!IS_ERR_OR_NULL(dcp->xbar) && !dcp->xbar_selected) {
		int xret = mux_control_select(dcp->xbar, DCP_DPXBAR_SELECTOR);

		if (xret)
			dev_warn(dcp->dev, "J700_DPXBAR: select failed: %d\n",
				 xret);
		else {
			dcp->xbar_selected = true;
			dev_info(dcp->dev, "J700_DPXBAR: selected dispext0 -> DP\n");
		}
	}

	/*
	 * The firmware only creates the AppleDCPDPTXRemotePort object -- and so
	 * only announces "dcpdptx-port-epic" on 0x2a -- once its DisplayPort
	 * controller has powered the rails and seen a device arrive.  Its own
	 * log says so: "No device added after powering on the rails. HPD=%d",
	 * then "HPD not present, skipping mode export" and a power-down.
	 *
	 * Driving the controller at DCP start cannot work on this machine: the
	 * rear connector carries the debug cable at boot, so there is no sink
	 * to find.  A DP-Alt event is the first moment a sink actually exists,
	 * so kick the controller here and give the port a chance to appear
	 * before asking for the connection.
	 */
	if (!IS_ERR_OR_NULL(dcp->dptxcontrolep) && !dcp->dptxport[0].enabled) {
		dptx_control_kick(dcp);

		if (dptxep_wait_ports(dcp))
			dev_warn(dcp->dev,
				 "DPTX port still absent after the hotplug kick; the connect below will fail\n");
		else
			dev_info(dcp->dev, "J700_DPTX_PORTS_READY (after hotplug kick)\n");
	}

	return dcp_dptx_connect_oob(to_platform_device(dcp->dev), 0);
}

static void dcp_typec_mux_unregister(void *data)
{
	typec_mux_unregister(data);
}

static int dcp_register_typec_mux(struct apple_dcp *dcp)
{
	struct device *dev = dcp->dev;
	struct typec_mux_desc desc = {};
	struct device_node *route_np;
	struct typec_mux_dev *mux;
	int ret;

	/*
	 * Optional: only a board whose DT wires a "typec-route" child with
	 * mode-switch and a graph endpoint to the connector gets one.
	 */
	route_np = of_get_child_by_name(dev->of_node, "typec-route");
	if (!route_np)
		return 0;

	desc.fwnode = of_fwnode_handle(route_np);
	desc.set = dcp_typec_mux_set;
	desc.name = "dcp-dp";
	desc.drvdata = dcp;

	mux = typec_mux_register(dev, &desc);
	of_node_put(route_np);
	if (IS_ERR(mux))
		return dev_err_probe(dev, PTR_ERR(mux),
				     "failed to register the DCP Type-C mux\n");

	ret = devm_add_action_or_reset(dev, dcp_typec_mux_unregister, mux);
	if (ret)
		return ret;

	dev_info(dev, "J700_DCP_TYPEC_MUX: registered\n");
	return 0;
}

int dcp_dptx_connect_oob(struct platform_device *pdev, u32 port)
{
	struct apple_dcp *dcp = platform_get_drvdata(pdev);
	int ret;

	if (dcp_is_typec_output(dcp)) {
		WRITE_ONCE(dcp->typec_cable_connected, true);
		dcp->typec_reconnect_tries = 0;
		cancel_delayed_work(&dcp->typec_reconnect_wq);
	}

	ret = dcp_dptx_connect(dcp, port);
	if (ret && dcp_is_typec_output(dcp))
		mod_delayed_work(system_freezable_wq, &dcp->typec_reconnect_wq,
				 DPTX_RECONNECT_DELAY);

	return ret;
}

int dcp_dptx_disconnect_oob(struct platform_device *pdev, u32 port)
{
	struct apple_dcp *dcp = platform_get_drvdata(pdev);

	if (dcp_is_typec_output(dcp)) {
		WRITE_ONCE(dcp->typec_unplugged, true);
		WRITE_ONCE(dcp->typec_cable_connected, false);
		WRITE_ONCE(dcp->typec_crtc_off, false);
		cancel_delayed_work(&dcp->typec_reconnect_wq);
	}

	/*
	 * H17P's HPD teardown powers down a live scanout pipe and can stop the
	 * coprocessor before displayRelease replies. Quiesce the external pipe
	 * while the DPTX session is still valid, retaining the last surface.
	 * Publishing the DRM disconnect comes afterward so its CRTC disable
	 * cannot race this power transition.
	 */
	if (dcp->fw_compat == DCP_FIRMWARE_H17P &&
	    READ_ONCE(dcp->typec_unplugged) &&
	    dcp->dptxport[port].enabled && dcp->dptxport[port].connected) {
		dev_info(dcp->dev, "Type-C unplug: quiescing external pipe before HPD removal\n");
		_dcp_poweroff(dcp);
	}

	disconnected_hpd_event(dcp->connector);

	if (dcp->avep)
		av_service_disconnect(dcp);

	if (dcp->dptxport[port].enabled)
		dptxport_set_hpd(dcp->dptxport[port].service, false);

	return dcp_dptx_disconnect(dcp, port);
}

static irqreturn_t dcp_dp2hdmi_hpd(int irq, void *data)
{
	struct apple_dcp *dcp = data;
	bool connected;

	guard(mutex)(&dcp_typec_fabric_lock);

	if (READ_ONCE(dcp->active_typec_route))
		return IRQ_HANDLED;
	connected = gpiod_get_value_cansleep(dcp->hdmi_hpd);

	/* do nothing on disconnect and trust that dcp detects it itself.
	 * Parallel disconnect HPDs result drm disabling the CRTC even when it
	 * should not.
	 * The interrupt should be changed to rising but for now the disconnect
	 * IRQs might be helpful for debugging.
	 */
	dev_info(dcp->dev, "DP2HDMI HPD irq, connected:%d\n", connected);

	if (connected) {
		msleep(500);
		connected = gpiod_get_value_cansleep(dcp->hdmi_hpd);
		dev_info(dcp->dev, "DP2HDMI HPD irq, 500ms debounce: connected:%d\n", connected);
	}

	if (connected)
		dcp_dptx_connect(dcp, 0);

	return IRQ_HANDLED;
}

void dcp_link(struct platform_device *pdev, struct apple_crtc *crtc,
	      struct apple_connector *connector)
{
	struct apple_dcp *dcp = platform_get_drvdata(pdev);

	dcp->crtc = crtc;

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


bool dcp_fw_compat_is_12_x(struct platform_device *pdev)
{
	struct apple_dcp *dcp = platform_get_drvdata(pdev);

	return dcp->fw_compat == DCP_FIRMWARE_V_12_3;
}

bool dcp_fw_compat_is_h17p(struct platform_device *pdev)
{
	struct apple_dcp *dcp = platform_get_drvdata(pdev);

	return dcp->fw_compat == DCP_FIRMWARE_H17P;
}

const u32 *dcp_get_iomfb_surfaces(struct platform_device *pdev, u32 *count)
{
	struct apple_dcp *dcp = platform_get_drvdata(pdev);

	*count = dcp->iomfb_surfaces.count;
	return dcp->iomfb_surfaces.id;
}

int dcp_start(struct platform_device *pdev)
{
	struct apple_dcp *dcp = platform_get_drvdata(pdev);
	int ret;

	init_completion(&dcp->start_done);

	/*
	 * The coprocessor start can fail (or be recovered from) without tearing
	 * the DRM side down, so this can be reached with no RTKit instance at
	 * all.  Starting an endpoint then dereferences NULL inside
	 * apple_rtkit_start_ep() and takes the machine with it.
	 */
	if (!dcp->rtk || IS_ERR(dcp->rtk)) {
		dev_err(dcp->dev,
			"cannot start endpoints: RTKit is not initialised\n");
		return -ENODEV;
	}

	/* start RTKit endpoints */
	ret = systemep_init(dcp);
	if (ret)
		dev_warn(dcp->dev, "Failed to start system endpoint: %d\n", ret);

	if (unstable_edid && !dcp_has_panel(dcp)) {
		ret = dpavservep_init(dcp);
		if (ret)
			dev_warn(dcp->dev, "Failed to start DPAVSERV endpoint: %d",
				 ret);
	}

	if (dcp->phy && dcp->fw_compat >= DCP_FIRMWARE_V_13_5) {
		ret = ibootep_init(dcp);
		if (ret)
			dev_warn(dcp->dev, "Failed to start IBOOT endpoint: %d\n",
				 ret);

		/*
		 * Deliberately after iomfb_start_rtkit() below on this SoC --
		 * see the comment there.  Starting the endpoint is cheap and
		 * must still happen first so the service announcement has
		 * somewhere to land; only the wait is deferred.
		 */
		/*
		 * Start the DP controller endpoints before the port endpoint.
		 * The firmware does not bring its DisplayPort stack up until
		 * they exist, and until it does, 0x2a answers a GETBUF and
		 * then never announces "dcpdptx-port-epic".
		 */
		ret = dptx_control_init(dcp);
		if (ret)
			dev_warn(dcp->dev,
				 "Failed to start DP control endpoints: %d\n",
				 ret);

		ret = dptxep_init(dcp);
		if (ret) {
			dev_warn(dcp->dev, "Failed to start DPTX endpoint: %d\n",
				 ret);
#ifdef DCP_DPTX_DISCONNECT_ON_INIT
		/*
		 * This disconnect / connect cycle on init is only necessary
		 * when using dcp0 on j473, j474s and presumedly j475c.
		 * Since dcp0 is not used at the moment let's avoid this
		 * since it is possibly the cause for startup issues.
		 */
		} else if (dcp->dptxport[0].enabled) {
			bool connected;
			/* force disconnect on start - necessary if the display
			 * is already up from m1n1
			 */
			dptxport_set_hpd(dcp->dptxport[0].service, false);
			dptxport_release_display(dcp->dptxport[0].service);
			usleep_range(10 * USEC_PER_MSEC, 25 * USEC_PER_MSEC);

			connected = gpiod_get_value_cansleep(dcp->hdmi_hpd);
			dev_info(dcp->dev, "%s: DP2HDMI HPD connected:%d\n", __func__, connected);

			// necessary on j473/j474 but not on j314c
			if (connected)
				dcp_dptx_connect(dcp, 0);
#endif
		}
	} else if (dcp->phy) {
		dev_warn(dcp->dev, "OS firmware incompatible with dptxport EP\n");
	}
	ret = iomfb_start_rtkit(dcp);
	if (ret)
		dev_err(dcp->dev, "Failed to start IOMFB endpoint: %d\n", ret);

	/*
	 * Only now can the DisplayPort port service turn up: the firmware does
	 * not announce it until IOMFB is running.  Waiting for it inside
	 * dptxep_init() above deadlocked the two against each other -- see the
	 * comment on dptxep_wait_ports().
	 */
	if (dcp->phy && !IS_ERR_OR_NULL(dcp->dptxcontrolep))
		dptx_control_kick(dcp);

	if (dcp->phy && !IS_ERR_OR_NULL(dcp->dptxep)) {
		int dret = dptxep_wait_ports(dcp);

		if (dret)
			dev_warn(dcp->dev,
				 "DPTX port service did not appear after IOMFB start: %d\n",
				 dret);
		else
			dev_info(dcp->dev, "J700_DPTX_PORTS_READY\n");
	}

#if IS_ENABLED(CONFIG_DRM_APPLE_AUDIO)
	if (hdmi_audio) {
		ret = avep_init(dcp);
		if (ret)
			dev_warn(dcp->dev, "Failed to start AV endpoint: %d", ret);
		ret = 0;
	}
#endif

	return ret;
}

static void _dcp_poweroff(struct apple_dcp *dcp)
{
	switch (dcp->fw_compat) {
	case DCP_FIRMWARE_V_12_3:
		iomfb_poweroff_v12_3(dcp);
		break;
	case DCP_FIRMWARE_V_13_5:
		iomfb_poweroff_v13_3(dcp);
		break;
	case DCP_FIRMWARE_H17P:
		iomfb_poweroff_h17p(dcp);
		break;
	default:
		WARN_ONCE(true, "Unexpected firmware version: %u\n", dcp->fw_compat);
		break;
	}
}

static int dcp_enable_dp2hdmi_hpd(struct apple_dcp *dcp)
{
	if (dcp_is_typec_output(dcp)) {
		if (READ_ONCE(dcp->typec_cable_connected))
			dcp_dptx_connect(dcp, 0);
	} else if (dcp->hdmi_hpd) {
		/* Check HPD before enabling the edge-triggered IRQ. */
		bool connected = gpiod_get_value_cansleep(dcp->hdmi_hpd);
		dev_info(dcp->dev, "%s: DP2HDMI HPD connected:%d\n", __func__, connected);

		if (connected)
			dcp_dptx_connect(dcp, 0);
		else
			_dcp_poweroff(dcp);
	}

	if (dcp->hdmi_hpd_irq)
		enable_irq(dcp->hdmi_hpd_irq);

	return 0;
}

int dcp_wait_ready(struct platform_device *pdev, u64 timeout)
{
	struct apple_dcp *dcp = platform_get_drvdata(pdev);
	int ret;

	if (dcp->crashed)
		return -ENODEV;
	if (dcp->active)
		return dcp_enable_dp2hdmi_hpd(dcp);
	if (timeout <= 0)
		return -ETIMEDOUT;

	ret = wait_for_completion_timeout(&dcp->start_done, timeout);
	if (ret < 0)
		return ret;

	if (dcp->crashed)
		return -ENODEV;

	if (dcp->active)
		dcp_enable_dp2hdmi_hpd(dcp);

	return dcp->active ? 0 : -ETIMEDOUT;
}

static void __maybe_unused dcp_sleep(struct apple_dcp *dcp)
{
	switch (dcp->fw_compat) {
	case DCP_FIRMWARE_V_12_3:
		iomfb_sleep_v12_3(dcp);
		break;
	case DCP_FIRMWARE_V_13_5:
		iomfb_sleep_v13_3(dcp);
		break;
	case DCP_FIRMWARE_H17P:
		iomfb_sleep_h17p(dcp);
		break;
	default:
		WARN_ONCE(true, "Unexpected firmware version: %u\n", dcp->fw_compat);
		break;
	}
}

void dcp_poweron(struct platform_device *pdev)
{
	struct apple_dcp *dcp = platform_get_drvdata(pdev);
	int ret;

	/* CRTC commits serialize the old disable before this new enable. */
	WRITE_ONCE(dcp->typec_unplugged, false);

	if (dcp_is_typec_output(dcp)) {
		WRITE_ONCE(dcp->typec_crtc_off, false);

		/*
		 * A Type-C CRTC disable releases its DPTX session. Re-establish it
		 * synchronously before IOMFB is powered back on.
		 */
		if (READ_ONCE(dcp->typec_cable_connected)) {
			cancel_delayed_work(&dcp->typec_reconnect_wq);
			dcp->typec_reconnect_tries = 0;
			ret = dcp_dptx_connect(dcp, 0);
			if (ret)
				mod_delayed_work(system_freezable_wq,
						 &dcp->typec_reconnect_wq,
						 DPTX_RECONNECT_DELAY);
		}
	} else if (dcp->hdmi_hpd) {
		bool connected = gpiod_get_value_cansleep(dcp->hdmi_hpd);
		dev_info(dcp->dev, "%s: DP2HDMI HPD connected:%d\n", __func__, connected);

		if (connected)
			dcp_dptx_connect(dcp, 0);
	}

	switch (dcp->fw_compat) {
	case DCP_FIRMWARE_V_12_3:
		iomfb_poweron_v12_3(dcp);
		break;
	case DCP_FIRMWARE_V_13_5:
		iomfb_poweron_v13_3(dcp);
		break;
	case DCP_FIRMWARE_H17P:
		iomfb_poweron_h17p(dcp);
		break;
	default:
		WARN_ONCE(true, "Unexpected firmware version: %u\n", dcp->fw_compat);
		break;
	}

	if (dcp->avep)
		av_service_connect(dcp);
}

void dcp_poweroff(struct platform_device *pdev)
{
	struct apple_dcp *dcp = platform_get_drvdata(pdev);
	int ret;

	/*
	 * Cable removal already deasserts HPD and releases the DPTX display in
	 * dcp_dptx_disconnect_oob(). H17P powers the pipe down as part of that
	 * operation. Starting a surface-clear swap in parallel wedges A406
	 * during the shutdown and prevents every later reconnect.
	 *
	 * Remember the unplug until the next CRTC enable, not merely until the
	 * next cable event: userspace may disable the old CRTC after the route
	 * has been released, or after a quick replug has marked the cable live.
	 * The atomic-disable path retains the old framebuffers until a later
	 * completed swap, so bypassing the clear does not free a live surface.
	 */
	if (dcp->fw_compat == DCP_FIRMWARE_H17P &&
	    READ_ONCE(dcp->typec_unplugged)) {
		dev_info(dcp->dev,
			 "Type-C unplug: DPTX owns shutdown; skipping CRTC clear swap\n");
		return;
	}

	if (dcp->avep)
		av_service_disconnect(dcp);

	/*
	 * Powering a Type-C CRTC off drops DCP's synthetic HPD, and the firmware
	 * reports that as an unplug. The display is still attached: keep the
	 * connector connected (see dcpep_cb_hotplug()) and let dcp_poweron()
	 * re-establish the DPTX session. Recreating it here instead makes the
	 * display vanish and come back, and compositors light a returning
	 * display, so DPMS off never sticks. Cable removal is reported through
	 * the Type-C mux.
	 */
	if (dcp_is_typec_output(dcp) && READ_ONCE(dcp->typec_cable_connected))
		WRITE_ONCE(dcp->typec_crtc_off, true);

	_dcp_poweroff(dcp);

	if (dcp_is_typec_output(dcp)) {
		/* DCP owns a synthetic HPD for Type-C. Release it with the CRTC. */
		if (dcp->dptxport[0].enabled && dcp->dptxport[0].connected) {
			ret = dptxport_set_hpd(dcp->dptxport[0].service, false);
			if (ret)
				dev_warn(dcp->dev,
					 "failed to deassert Type-C DPTX HPD: %d\n", ret);
			dcp_dptx_disconnect(dcp, 0);
		}
	} else if (dcp->hdmi_hpd) {
		bool connected = gpiod_get_value_cansleep(dcp->hdmi_hpd);
		if (!connected) {
			disconnected_hpd_event(dcp->connector);
			dcp_dptx_disconnect(dcp, 0);
		}
	}
}

static void dcp_work_register_backlight(struct work_struct *work)
{
	int ret;
	struct apple_dcp *dcp;

	dcp = container_of(work, struct apple_dcp, bl_register_wq);

	mutex_lock(&dcp->bl_register_mutex);
	if (dcp->brightness.bl_dev)
		goto out_unlock;

	/* try to register backlight device, */
	ret = dcp_backlight_register(dcp);
	if (ret) {
		dev_err(dcp->dev, "Unable to register backlight device\n");
		dcp->brightness.maximum = 0;
	}

out_unlock:
	mutex_unlock(&dcp->bl_register_mutex);
}

static void dcp_work_update_backlight(struct work_struct *work)
{
	struct apple_dcp *dcp;

	dcp = container_of(to_delayed_work(work), struct apple_dcp,
			   bl_update_wq);

	dcp_backlight_update(dcp);
}

static int dcp_create_piodma_iommu_dev(struct apple_dcp *dcp)
{
	int ret;
	struct device_node *node __free(device_node) = of_get_child_by_name(dcp->dev->of_node, "piodma");

	if (!node)
		return dev_err_probe(dcp->dev, -ENODEV,
				     "Failed to get piodma child DT node\n");

	dcp->piodma = of_platform_device_create(node, NULL, dcp->dev);
	if (!dcp->piodma) {
		/*
		 * of_platform_device_create() returns NULL when the node already
		 * has a platform device, which happens whenever a previous bind
		 * attempt created it and then failed further down (the caller's
		 * later error paths do not destroy it).  Probe is retried - a
		 * deferral, or a component re-bind - and would then fail here
		 * forever with -ENODEV even though nothing is actually wrong.
		 * Observed on J700 as:
		 *   piodma: Adding to iommu group 4          <- first attempt
		 *   Failed to create piodma pdev ... -ENODEV <- every retry
		 * Adopt the existing device instead of giving up.
		 */
		dcp->piodma = of_find_device_by_node(node);
		if (!dcp->piodma)
			return dev_err_probe(dcp->dev, -ENODEV,
					     "Failed to create piodma pdev for %pOF\n",
					     node);
		dev_info(dcp->dev, "reusing existing piodma pdev for %pOF\n",
			 node);
	}

	ret = dma_set_mask_and_coherent(&dcp->piodma->dev, DMA_BIT_MASK(42));
	if (ret)
		goto err_destroy_pdev;

	ret = of_dma_configure(&dcp->piodma->dev, node, true);
	if (ret) {
		ret = dev_err_probe(dcp->dev, ret,
			"Failed to configure IOMMU child DMA\n");
		goto err_destroy_pdev;
	}

	dcp->iommu_dom = iommu_get_domain_for_dev(&dcp->piodma->dev);
	if (IS_ERR(dcp->iommu_dom)) {
		ret = dev_err_probe(dcp->dev, PTR_ERR(dcp->iommu_dom),
				    "Failed to get default iommu domain for "
				    "piodma device\n");
		dcp->iommu_dom = NULL;
		goto err_destroy_pdev;
	}

	return 0;
err_destroy_pdev:
	of_platform_device_destroy(&dcp->piodma->dev, NULL);
	return ret;
}

static int dcp_get_bw_scratch_reg(struct apple_dcp *dcp, u32 expected)
{
	struct of_phandle_args ph_args;
	u32 addr_idx, disp_idx, offset;
	int ret;

	ret = of_parse_phandle_with_args(dcp->dev->of_node, "apple,bw-scratch",
				   "#apple,bw-scratch-cells", 0, &ph_args);
	if (ret < 0) {
		dev_err(dcp->dev, "Failed to read 'apple,bw-scratch': %d\n", ret);
		return ret;
	}

	if (ph_args.args_count != 3) {
		dev_err(dcp->dev, "Unexpected 'apple,bw-scratch' arg count %d\n",
			ph_args.args_count);
		ret = -EINVAL;
		goto err_of_node_put;
	}

	addr_idx = ph_args.args[0];
	disp_idx = ph_args.args[1];
	offset = ph_args.args[2];

	if (disp_idx != expected || disp_idx >= MAX_DISP_REGISTERS) {
		dev_err(dcp->dev, "Unexpected disp_reg value in 'apple,bw-scratch': %d\n",
			disp_idx);
		ret = -EINVAL;
		goto err_of_node_put;
	}

	ret = of_address_to_resource(ph_args.np, addr_idx, &dcp->disp_bw_scratch_res);
	if (ret < 0) {
		dev_err(dcp->dev, "Failed to get 'apple,bw-scratch' resource %d from %pOF\n",
			addr_idx, ph_args.np);
		goto err_of_node_put;
	}
	if (offset > resource_size(&dcp->disp_bw_scratch_res) - 4) {
		ret = -EINVAL;
		goto err_of_node_put;
	}

	dcp->disp_registers[disp_idx] = &dcp->disp_bw_scratch_res;
	dcp->disp_bw_scratch_index = disp_idx;
	dcp->disp_bw_scratch_offset = offset;
	ret = 0;

err_of_node_put:
	of_node_put(ph_args.np);
	return ret;
}

static int dcp_get_bw_doorbell_reg(struct apple_dcp *dcp, u32 expected)
{
	struct of_phandle_args ph_args;
	u32 addr_idx, disp_idx;
	int ret;

	ret = of_parse_phandle_with_args(dcp->dev->of_node, "apple,bw-doorbell",
				   "#apple,bw-doorbell-cells", 0, &ph_args);
	if (ret < 0) {
		dev_err(dcp->dev, "Failed to read 'apple,bw-doorbell': %d\n", ret);
		return ret;
	}

	if (ph_args.args_count != 2) {
		dev_err(dcp->dev, "Unexpected 'apple,bw-doorbell' arg count %d\n",
			ph_args.args_count);
		ret = -EINVAL;
		goto err_of_node_put;
	}

	addr_idx = ph_args.args[0];
	disp_idx = ph_args.args[1];

	if (disp_idx != expected || disp_idx >= MAX_DISP_REGISTERS) {
		dev_err(dcp->dev, "Unexpected disp_reg value in 'apple,bw-doorbell': %d\n",
			disp_idx);
		ret = -EINVAL;
		goto err_of_node_put;
	}

	ret = of_address_to_resource(ph_args.np, addr_idx, &dcp->disp_bw_doorbell_res);
	if (ret < 0) {
		dev_err(dcp->dev, "Failed to get 'apple,bw-doorbell' resource %d from %pOF\n",
			addr_idx, ph_args.np);
		goto err_of_node_put;
	}
	dcp->disp_bw_doorbell_index = disp_idx;
	dcp->disp_registers[disp_idx] = &dcp->disp_bw_doorbell_res;
	ret = 0;

err_of_node_put:
	of_node_put(ph_args.np);
	return ret;
}

/*
 * Count the "disp-*" entries in reg-names.
 *
 * The positional form below assumes the reg layout is [coproc, disp-0 ...],
 * i.e. exactly one non-disp register, and derives the count as
 * num_resources - 1.  That is wrong on H17P/T8140, whose layout is
 * [coproc, disp-0 .. disp-4, iop-vbar]: the trailing iop-vbar inflates the
 * count to 6 and the index loop then binds disp_registers[5] to iop-vbar.
 * The inflated count also makes the "apple,bw-scratch" disp_reg check reject
 * the device tree's (correct) value of 5, which fails the probe with
 *   Unexpected disp_reg value in 'apple,bw-scratch': 5
 *   failed to find display registers
 *
 * Returns a negative errno when reg-names is absent, so callers fall back.
 */
static int dcp_count_disp_regs(struct device *dev)
{
	int n = of_property_count_strings(dev->of_node, "reg-names");
	const char *name;
	int i, count = 0;

	if (n <= 0)
		return -EINVAL;

	for (i = 0; i < n; ++i) {
		if (of_property_read_string_index(dev->of_node, "reg-names", i,
						  &name))
			return -EINVAL;
		if (!strncmp(name, "disp-", 5))
			count++;
	}

	return count ? count : -EINVAL;
}

static int dcp_get_disp_regs(struct apple_dcp *dcp)
{
	struct platform_device *pdev = to_platform_device(dcp->dev);
	int count = dcp_count_disp_regs(dcp->dev);
	bool by_name = count > 0;
	int i, ret;

	/* Device trees without reg-names keep the historical positional form. */
	if (!by_name)
		count = pdev->num_resources - 1;

	if (count <= 0 || count > MAX_DISP_REGISTERS)
		return -EINVAL;

	for (i = 0; i < count; ++i) {
		if (by_name) {
			char name[8];

			snprintf(name, sizeof(name), "disp-%d", i);
			dcp->disp_registers[i] = platform_get_resource_byname(
				pdev, IORESOURCE_MEM, name);
		} else {
			dcp->disp_registers[i] = platform_get_resource(
				pdev, IORESOURCE_MEM, 1 + i);
		}
		if (!dcp->disp_registers[i]) {
			dev_err(dcp->dev, "missing display register %d\n", i);
			return -EINVAL;
		}
	}

	/* load pmgr bandwidth scratch resource and offset */
	ret = dcp_get_bw_scratch_reg(dcp, count);
	if (ret < 0)
		return ret;
	count += 1;

	/* load pmgr bandwidth doorbell resource if present (only on t8103) */
	if (of_property_present(dcp->dev->of_node, "apple,bw-doorbell")) {
		ret = dcp_get_bw_doorbell_reg(dcp, count);
		if (ret < 0)
			return ret;
		count += 1;
	}

	dcp->nr_disp_registers = count;
	return 0;
}

#define DCP_FW_VERSION_MIN_LEN	3
#define DCP_FW_VERSION_MAX_LEN	5
#define DCP_FW_VERSION_STR_LEN	(DCP_FW_VERSION_MAX_LEN * 4)

static int dcp_read_fw_version(struct device *dev, const char *name,
			       char *version_str)
{
	u32 ver[DCP_FW_VERSION_MAX_LEN];
	int len_str;
	int len;

	len = of_property_read_variable_u32_array(dev->of_node, name, ver,
						  DCP_FW_VERSION_MIN_LEN,
						  DCP_FW_VERSION_MAX_LEN);

	switch (len) {
	case 3:
		len_str = scnprintf(version_str, DCP_FW_VERSION_STR_LEN,
				    "%d.%d.%d", ver[0], ver[1], ver[2]);
		break;
	case 4:
		len_str = scnprintf(version_str, DCP_FW_VERSION_STR_LEN,
				    "%d.%d.%d.%d", ver[0], ver[1], ver[2],
				    ver[3]);
		break;
	case 5:
		len_str = scnprintf(version_str, DCP_FW_VERSION_STR_LEN,
				    "%d.%d.%d.%d.%d", ver[0], ver[1], ver[2],
				    ver[3], ver[4]);
		break;
	default:
		len_str = strscpy(version_str, "UNKNOWN",
				  DCP_FW_VERSION_STR_LEN);
		if (len >= 0)
			len = -EOVERFLOW;
		break;
	}

	if (len_str >= DCP_FW_VERSION_STR_LEN)
		dev_warn(dev, "'%s' truncated: '%s'\n", name, version_str);

	return len;
}

static enum dcp_firmware_version dcp_check_firmware_version(struct device *dev)
{
	const struct apple_dcp_hw_data *hw = of_device_get_match_data(dev);
	char compat_str[DCP_FW_VERSION_STR_LEN];
	char fw_str[DCP_FW_VERSION_STR_LEN];
	int ret;

	if (hw->firmware_compat != DCP_FIRMWARE_UNKNOWN)
		return hw->firmware_compat;

	/* firmware version is just informative */
	dcp_read_fw_version(dev, "apple,firmware-version", fw_str);

	ret = dcp_read_fw_version(dev, "apple,firmware-compat", compat_str);
	if (ret < 0) {
		dev_err(dev, "Could not read 'apple,firmware-compat': %d\n", ret);
		return DCP_FIRMWARE_UNKNOWN;
	}

	if (strncmp(compat_str, "12.3.0", sizeof(compat_str)) == 0)
		return DCP_FIRMWARE_V_12_3;
	/*
	 * m1n1 reports firmware version 13.5 as compatible with 13.3. This is
	 * only true for the iomfb endpoint. The interface for the dptx-port
	 * endpoint changed between 13.3 and 13.5. The driver will only support
	 * firmware 13.5. Check the actual firmware version for compat version
	 * 13.3 until m1n1 reports 13.5 as "firmware-compat".
	 */
	else if ((strncmp(compat_str, "13.3.0", sizeof(compat_str)) == 0) &&
		 (strncmp(fw_str, "13.5.0", sizeof(compat_str)) == 0))
		return DCP_FIRMWARE_V_13_5;
	else if (strncmp(compat_str, "13.5.0", sizeof(compat_str)) == 0)
		return DCP_FIRMWARE_V_13_5;

	dev_err(dev, "DCP firmware-compat %s (FW: %s) is not supported\n",
		compat_str, fw_str);

	return DCP_FIRMWARE_UNKNOWN;
}

static int dcp_connector_type_from_dt(struct device_node *np)
{
	if (of_property_match_string(np, "apple,connector-type", "HDMI-A") >= 0)
		return DRM_MODE_CONNECTOR_HDMIA;
	if (of_property_match_string(np, "apple,connector-type", "DP") >= 0)
		return DRM_MODE_CONNECTOR_DisplayPort;
	if (of_property_match_string(np, "apple,connector-type", "USB-C") >= 0)
		return DRM_MODE_CONNECTOR_USB;

	return DRM_MODE_CONNECTOR_Unknown;
}

#define APPLE_PMGR_PS_ACTUAL	GENMASK(7, 4)
#define APPLE_PMGR_PS_TARGET	GENMASK(3, 0)
#define APPLE_PMGR_PS_ACTIVE	0xf

/*
 * Power up the "pmc" domain for the DCP.
 *
 * As soon as the AP acks the fifth D563 (pmgrConfigDispPerfState), the DCP's
 * RTKPMGR_pmc_boost_disp_perf_state path reads the PMC idle register at
 * physical 0x3028aa140.  That aperture belongs to the T8140 "pmc" power
 * domain, which the ADT marks initially-on but which Linux leaves off because
 * no device claims it.  The read then takes an L2C access fault and the
 * coprocessor dies with a data abort -- silently, with no further mailbox
 * traffic, immediately after the boot property storm, which looks exactly like
 * a hang.  Turn the domain on before booting the firmware.
 */
static void dcp_enable_pmc_domain(struct apple_dcp *dcp)
{
	struct device_node *np;
	const char *label;
	u32 offset, val = 0;
	int i;

	for_each_compatible_node(np, NULL, "apple,t8103-pmgr-pwrstate") {
		struct regmap *regmap;

		if (of_property_read_string(np, "label", &label) ||
		    strcmp(label, "pmc"))
			continue;

		if (of_property_read_u32_index(np, "reg", 0, &offset)) {
			dev_warn(dcp->dev, "pmc: no reg offset\n");
			break;
		}

		regmap = syscon_node_to_regmap(np->parent);
		if (IS_ERR(regmap)) {
			dev_warn(dcp->dev, "pmc: no parent regmap: %ld\n",
				 PTR_ERR(regmap));
			break;
		}

		if (regmap_update_bits(regmap, offset, APPLE_PMGR_PS_TARGET,
				       FIELD_PREP(APPLE_PMGR_PS_TARGET,
						  APPLE_PMGR_PS_ACTIVE))) {
			dev_warn(dcp->dev, "pmc: failed to request active\n");
			break;
		}

		for (i = 0; i < 200; i++) {
			if (regmap_read(regmap, offset, &val))
				break;
			if (FIELD_GET(APPLE_PMGR_PS_ACTUAL, val) ==
			    APPLE_PMGR_PS_ACTIVE) {
				dev_info(dcp->dev, "pmc domain on (%#x)\n", val);
				of_node_put(np);
				return;
			}
			udelay(10);
		}
		dev_warn(dcp->dev, "pmc domain did not activate (%#x)\n", val);
		of_node_put(np);
		return;
	}

	dev_warn(dcp->dev, "pmc power domain not found\n");
}

/*
 * Reset the display coprocessor's ASC core.
 *
 * Once Linux has driven a DCP session the firmware stops answering the
 * management endpoint: it will not re-HELLO on INIT (it is already ON) and it
 * will not acknowledge a power-state change either, so neither apple_rtkit_wake
 * nor apple_rtkit_quiesce can recover it and the module could previously only
 * be reloaded by rebooting the machine.
 *
 * CPU_CONTROL.RUN is a raw register in the ASC wrapper and needs no cooperation
 * from the firmware, so clearing it stops the core no matter how wedged it is.
 * The firmware image itself is left alone -- iBoot mapped it into the DCP's own
 * DART and nothing here touches those mappings -- so setting RUN again restarts
 * it from its reset vector and it comes back with a fresh HELLO.
 */
static int dcp_asc_restart(struct apple_dcp *dcp)
{
	u32 ctrl, status;
	int i;

	ctrl = readl_relaxed(dcp->coproc_reg + APPLE_DCP_COPROC_CPU_CONTROL);
	writel_relaxed(ctrl & ~APPLE_DCP_COPROC_CPU_CONTROL_RUN,
		       dcp->coproc_reg + APPLE_DCP_COPROC_CPU_CONTROL);

	/*
	 * T8140 clears CPU_STATUS.RUNNING when the core halts but never sets
	 * the STOPPED bit (status goes 0x6d -> 0x48), so gate on RUNNING going
	 * away rather than on STOPPED appearing.
	 */
	for (i = 0; i < 500; i++) {
		status = readl_relaxed(dcp->coproc_reg +
				       APPLE_DCP_COPROC_CPU_STATUS);
		if (!(status & APPLE_DCP_COPROC_CPU_STATUS_RUNNING) ||
		    (status & APPLE_DCP_COPROC_CPU_STATUS_STOPPED))
			break;
		usleep_range(100, 200);
	}
	if (status & APPLE_DCP_COPROC_CPU_STATUS_RUNNING) {
		dev_err(dcp->dev, "DCP ASC did not stop (status %#x)\n", status);
		return -ETIMEDOUT;
	}

	dev_info(dcp->dev, "DCP ASC stopped (status %#x), restarting it\n",
		 status);
	usleep_range(1000, 2000);
	writel_relaxed(ctrl | APPLE_DCP_COPROC_CPU_CONTROL_RUN,
		       dcp->coproc_reg + APPLE_DCP_COPROC_CPU_CONTROL);
	return 0;
}





static int dcp_restart_inherited_session(struct apple_dcp *dcp)
{
	int ret;

	if (dcp->fw_compat != DCP_FIRMWARE_H17P)
		return apple_rtkit_quiesce(dcp->rtk);

	/*
	 * J700's inherited DCP rejects AP QUIESCED. The dcp-neo m1n1
	 * bring-up established IOP OFF and its exact state-zero ACK instead
	 * (4a927cdcae729f1b12b2173117e7f96b3271dca9). OFF is not a normal
	 * sleep state: stop and restart the ASC before negotiating a new
	 * RTKit session. Never reset a session whose OFF request timed out.
	 */
	ret = apple_rtkit_poweroff_adopted(dcp->rtk);
	if (ret)
		return dev_err_probe(dcp->dev, ret,
				     "Inherited DCP IOP OFF not acknowledged\n");

	dev_info(dcp->dev, "inherited DCP acknowledged IOP OFF\n");
	return dcp_asc_restart(dcp);
}

static int dcp_comp_bind(struct device *dev, struct device *main, void *data)
{
	struct device_node *panel_np;
	struct apple_dcp *dcp = dev_get_drvdata(dev);
	enum apple_dcp_coprocessor_start start_mode;
	u32 cpu_ctrl, cpu_status;
	u32 handoff_state;
	bool loader_quiesced;
	int ret;

	/*
	 * The coprocessor scans out through the display-subsystem device's
	 * IOMMU stream, not through piodma's, so buffers it asks us to map for
	 * piodma must also be visible in that domain or the scanout faults.
	 */
	dcp->disp_dev = main;
	dcp->disp_iommu_dom = main ? iommu_get_domain_for_dev(main) : NULL;
	if (IS_ERR(dcp->disp_iommu_dom))
		dcp->disp_iommu_dom = NULL;

	/*
	 * Give the scanout stream a landing page at the bottom of its window.
	 *
	 * m1n1 stops the DCP ASC before handing over, so unlike macOS -- which
	 * adopts the generation iBoot left running -- our firmware comes up with
	 * no active surface.  It enables the pipe anyway at the end of the boot
	 * chain and fetches from a null base.  The DART rejects that, the reject
	 * becomes an AXI error, and the coprocessor takes an NMI
	 * (apiodma_irq_AxiErr) before we can drive a modeset.
	 *
	 * One zeroed page mapped at the base of the stream's own window turns
	 * that fatal reject into a harmless read of zeros.  This only *adds* a
	 * translation at an index the bootloader does not use; it never rewrites
	 * an inherited one.
	 */
	/*
	 * Deliberately do not add mappings here.  iommu_map() on this domain
	 * makes apple-dart copy our *clone* of the inherited L1 over the
	 * hardware's, so anything the live firmware has added to its own tables
	 * since we cloned them vanishes underneath it and the coprocessor takes
	 * an axi_rd_err NMI.  The bootloader's translations are left alone.
	 */

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(42));
	if (ret)
		return ret;

	dcp->coproc_reg = devm_platform_ioremap_resource_byname(to_platform_device(dev), "coproc");
	if (IS_ERR(dcp->coproc_reg))
		return PTR_ERR(dcp->coproc_reg);

	cpu_ctrl = readl_relaxed(dcp->coproc_reg +
				 APPLE_DCP_COPROC_CPU_CONTROL);
	cpu_status = readl_relaxed(dcp->coproc_reg +
				   APPLE_DCP_COPROC_CPU_STATUS);
	loader_quiesced = of_chosen &&
		!of_property_read_u32(of_chosen,
				      APPLE_DCP_RTKIT_HANDOFF_PROPERTY,
				      &handoff_state) &&
		handoff_state == 1;
	start_mode = apple_dcp_coprocessor_start_mode(
		loader_quiesced, dcp->hw.adopt_running_coprocessor,
		cpu_ctrl, cpu_status);
	if (dcp->hw.adopt_live_session &&
	    (loader_quiesced || !(cpu_ctrl & APPLE_DCP_COPROC_CPU_CONTROL_RUN)))
		return dev_err_probe(dev, -EOPNOTSUPP,
				     "This DCP requires a live inherited session\n");

	of_property_read_u32(dev->of_node, "apple,dcp-index",
					   &dcp->index);
	of_property_read_u32(dev->of_node, "apple,dptx-phy",
					   &dcp->dptx_phy);
	of_property_read_u32(dev->of_node, "apple,dptx-die",
					   &dcp->dptx_die);
	if (dcp->index || dcp->dptx_phy || dcp->dptx_die)
		dev_info(dev, "DCP index:%u dptx target phy: %u dptx die: %u\n",
			 dcp->index, dcp->dptx_phy, dcp->dptx_die);
	mutex_init(&dcp->hpd_mutex);

	if (!show_notch)
		ret = of_property_read_u32(dev->of_node, "apple,notch-height",
					   &dcp->notch_height);

	if (dcp->notch_height > MAX_NOTCH_HEIGHT)
		dcp->notch_height = MAX_NOTCH_HEIGHT;
	if (dcp->notch_height > 0)
		dev_info(dev, "Detected display with notch of %u pixel\n", dcp->notch_height);

	/* initialize brightness scale to a sensible default to avoid divide by 0*/
	dcp->brightness.scale = 65536;
	dcp->brightness.h17p_sent_nits = -1;
	panel_np = of_get_compatible_child(dev->of_node, "apple,panel-mini-led");
	if (panel_np)
		dcp->panel.has_mini_led = true;
	else
		panel_np = of_get_compatible_child(dev->of_node, "apple,panel");

	if (panel_np) {
		const char height_prop[2][16] = { "adj-height-mm", "height-mm" };

		if (of_device_is_available(panel_np)) {
			ret = of_property_read_u32(panel_np, "apple,max-brightness",
						   &dcp->brightness.maximum);
			if (ret)
				dev_err(dev, "Missing property 'apple,max-brightness'\n");
		}

		of_property_read_u32(panel_np, "width-mm", &dcp->panel.width_mm);
		/* use adjusted height as long as the notch is hidden */
		of_property_read_u32(panel_np, height_prop[!dcp->notch_height],
				     &dcp->panel.height_mm);

		of_node_put(panel_np);
		dcp->fixed_connector_type = DRM_MODE_CONNECTOR_eDP;
		dcp->connector_type = DRM_MODE_CONNECTOR_eDP;
		INIT_WORK(&dcp->bl_register_wq, dcp_work_register_backlight);
		mutex_init(&dcp->bl_register_mutex);
		INIT_DELAYED_WORK(&dcp->bl_update_wq, dcp_work_update_backlight);
	}

	ret = dcp_create_piodma_iommu_dev(dcp);
	if (ret || !dcp->iommu_dom)
		return dev_err_probe(dev, ret,
				"Failed to created PIODMA iommu child device");

	ret = dcp_get_disp_regs(dcp);
	if (ret) {
		dev_err(dev, "failed to find display registers\n");
		return ret;
	}

	dcp->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(dcp->clk))
		return dev_err_probe(dev, PTR_ERR(dcp->clk),
				     "Unable to find clock\n");
	dcp->clk_194 = devm_clk_get_optional(dev, "clock-194");
	if (IS_ERR(dcp->clk_194))
		return dev_err_probe(dev, PTR_ERR(dcp->clk_194),
				     "Unable to find clock 0x194\n");

	bitmap_zero(dcp->memdesc_map, DCP_MAX_MAPPINGS);
	// TDOD: mem_desc IDs start at 1, for simplicity just skip '0' entry
	set_bit(0, dcp->memdesc_map);

	INIT_DELAYED_WORK(&dcp->vblank_wq, dcp_delayed_vblank);
	INIT_DELAYED_WORK(&dcp->stall_wq, dcp_stall_check);
	INIT_LIST_HEAD(&dcp->pending_flushes);
	mutex_init(&dcp->pending_flush_lock);
	spin_lock_init(&dcp->callback_trace_lock);
	dcp->callback_trace_count = 0;

	dcp->swapped_out_fbs =
		(struct list_head)LIST_HEAD_INIT(dcp->swapped_out_fbs);
	spin_lock_init(&dcp->swapped_out_lock);

	if (dcp->fw_compat == DCP_FIRMWARE_H17P && !dcp->hw.adopt_live_session)
		dcp_enable_pmc_domain(dcp);

	if (start_mode == APPLE_DCP_COPROCESSOR_ADOPT_AND_RESTART &&
	    !dcp->hw.adopt_live_session) {
		dev_info(dev,
			 "quiescing inherited DCP session (control=%#x status=%#x)\n",
			 cpu_ctrl, cpu_status);
		dcp->adopting_session = true;
		dcp->rtk = devm_apple_rtkit_init_adopted(dev, dcp, "mbox", 0,
							  &rtkit_ops);
		if (IS_ERR(dcp->rtk)) {
			dcp->adopting_session = false;
			return dev_err_probe(dev, PTR_ERR(dcp->rtk),
					     "Failed to adopt RTKit\n");
		}

		ret = dcp_restart_inherited_session(dcp);
		dcp->adopting_session = false;
		if (ret)
			return dev_err_probe(dev, ret,
					     "Failed to quiesce inherited RTKit: %d\n",
					     ret);
	} else {
		if (start_mode == APPLE_DCP_COPROCESSOR_QUIESCED_RESTART)
			dev_info(dev, "restarting loader-quiesced DCP session\n");
		if (dcp->hw.adopt_live_session && !live_discovery) {
			dcp->adopting_session = true;
			dcp->rtk = devm_apple_rtkit_init_adopted(dev, dcp,
								 "mbox", 0,
								 &rtkit_ops);
			dcp->adopting_session = false;
		} else {
			dcp->rtk = devm_apple_rtkit_init(dev, dcp, "mbox", 0,
							 &rtkit_ops);
		}
		if (IS_ERR(dcp->rtk))
			return dev_err_probe(dev, PTR_ERR(dcp->rtk),
					     "Failed to initialize RTKit\n");
	}

	if (dcp->hw.adopt_live_session && live_discovery) {
		/* No CPU-control, power-domain or DART writes on this path. The
		 * native trace receives HELLO in response to this standard INIT.
		 * A timeout is evidence to retain; do not fall back to a reset.
		 */
		dev_info(dev, "negotiating RTKit on the live DCP without a reset\n");
		ret = apple_rtkit_wake(dcp->rtk);
	} else if (dcp->hw.adopt_live_session) {
		/*
		 * The loader consumed discovery. Start selected services explicitly
		 * through dcp_start_rtkit_endpoint(), preserving the running CPU and
		 * its firmware memory mappings.
		 */
		dev_info(dev, "adopting the live DCP session without a restart\n");
		ret = 0;
	} else {
		cpu_ctrl = readl_relaxed(dcp->coproc_reg +
					 APPLE_DCP_COPROC_CPU_CONTROL);
		writel_relaxed(cpu_ctrl | APPLE_DCP_COPROC_CPU_CONTROL_RUN,
			       dcp->coproc_reg + APPLE_DCP_COPROC_CPU_CONTROL);

		ret = apple_rtkit_wake(dcp->rtk);
	}
	if (ret && start_mode == APPLE_DCP_COPROCESSOR_QUIESCED_RESTART) {
		/*
		 * INIT went unanswered.  That happens when the session we found
		 * is not the one the loader quiesced but one an earlier probe
		 * of this driver already brought up: RTKit is ON, so it ignores
		 * INIT and never sends HELLO.  Adopt that session, quiesce it
		 * back to the state the loader would have left it in, and boot
		 * it again.  Without this, a failed probe or a module reload
		 * can only be recovered by rebooting the machine.
		 */
		dev_info(dev,
			 "DCP did not answer INIT; adopting the running session to restart it\n");
		devm_apple_rtkit_free(dev, dcp->rtk);
		dcp->rtk = NULL;
		dcp->adopting_session = true;
		dcp->rtk = devm_apple_rtkit_init_adopted(dev, dcp, "mbox", 0,
							 &rtkit_ops);
		if (IS_ERR(dcp->rtk)) {
			dcp->adopting_session = false;
			return dev_err_probe(dev, PTR_ERR(dcp->rtk),
					     "Failed to adopt running RTKit\n");
		}

		ret = dcp_restart_inherited_session(dcp);
		dcp->adopting_session = false;
		if (ret && dcp->fw_compat == DCP_FIRMWARE_H17P)
			return ret;
		if (!ret)
			ret = apple_rtkit_wake(dcp->rtk);

		if (ret) {
			/*
			 * The firmware is not answering the mailbox at all.
			 * Reset the ASC core itself and cold-boot it; that
			 * needs no cooperation from the running firmware.
			 */
			dev_info(dev,
				 "DCP will not quiesce (%d); resetting the ASC core\n",
				 ret);
			devm_apple_rtkit_free(dev, dcp->rtk);
			dcp->rtk = NULL;
			ret = dcp_asc_restart(dcp);
			if (ret)
				return dev_err_probe(dev, ret,
						     "Failed to reset DCP ASC: %d\n",
						     ret);

			dcp->rtk = devm_apple_rtkit_init(dev, dcp, "mbox", 0,
							 &rtkit_ops);
			if (IS_ERR(dcp->rtk))
				return dev_err_probe(dev, PTR_ERR(dcp->rtk),
						     "Failed to re-init RTKit\n");

			ret = apple_rtkit_wake(dcp->rtk);
		}
	}
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to boot RTKit: %d\n", ret);
	return ret;
}

/*
 * We need to shutdown DCP before tearing down the display subsystem. Otherwise
 * the DCP will crash and briefly flash a green screen of death.
 */
static void dcp_comp_unbind(struct device *dev, struct device *main, void *data)
{
	struct apple_dcp *dcp = dev_get_drvdata(dev);

	if (!dcp)
		return;
	cancel_delayed_work_sync(&dcp->stall_wq);

	if (dcp->hdmi_hpd_irq)
		disable_irq(dcp->hdmi_hpd_irq);

	typec_mux_put(dcp->typec_mux);

	if (dcp->avep) {
		av_service_disconnect(dcp);
		afk_shutdown(dcp->avep);
		dcp->avep = NULL;
	}

	if (dcp->dptxep) {
		afk_shutdown(dcp->dptxep);
		dcp->dptxep = NULL;
	}

	if (dcp->ibootep) {
		afk_shutdown(dcp->ibootep);
		dcp->ibootep = NULL;
	}

	if (dcp->systemep) {
		afk_shutdown(dcp->systemep);
		dcp->systemep = NULL;
	}

	if (dcp->dcpavservep) {
		afk_shutdown(dcp->dcpavservep);
		dcp->dcpavservep = NULL;
	}

	dcp_flush_shutdown(dcp);

	if (dcp->shmem)
		iomfb_shutdown(dcp);

	if (dcp->piodma) {
		dcp->iommu_dom = NULL;
		of_platform_device_destroy(&dcp->piodma->dev, NULL);
		dcp->piodma = NULL;
	}

	if (dcp->connector_type == DRM_MODE_CONNECTOR_eDP) {
		cancel_work_sync(&dcp->bl_register_wq);
		cancel_delayed_work_sync(&dcp->bl_update_wq);
	}
	cancel_delayed_work_sync(&dcp->typec_reconnect_wq);
	cancel_delayed_work_sync(&dcp->typec_fabric_retrain_wq);
	cancel_delayed_work_sync(&dcp->vblank_wq);

	/* The component framework releases the bind's devres group after
	 * unbind, including optional clocks whose managed value can be NULL.
	 */
	dcp->clk = NULL;
	dcp->clk_194 = NULL;
}

static const struct component_ops dcp_comp_ops = {
	.bind	= dcp_comp_bind,
	.unbind	= dcp_comp_unbind,
};

static int dcp_platform_probe(struct platform_device *pdev)
{
	enum dcp_firmware_version fw_compat;
	struct device *dev = &pdev->dev;
	struct apple_dcp *dcp;
	int num_surfaces;
	u32 surface_ids[DCP_MAX_PLANES];
	u32 mux_index;
	int ret;

	fw_compat = dcp_check_firmware_version(dev);
	if (fw_compat == DCP_FIRMWARE_UNKNOWN)
		return -ENODEV;

	/* Check for "apple,bw-scratch" to avoid probing appledrm with outdated
	 * device trees. This prevents replacing simpledrm and ending up without
	 * display.
	 */
	if (!of_property_present(dev->of_node, "apple,bw-scratch"))
		return dev_err_probe(dev, -ENODEV, "Incompatible devicetree! "
			"Use devicetree matching this kernel.\n");

	dcp = devm_kzalloc(dev, sizeof(*dcp), GFP_KERNEL);
	if (!dcp)
		return -ENOMEM;

	dcp->fw_compat = fw_compat;
	dcp->dev = dev;
	dcp->hw = *(struct apple_dcp_hw_data *)of_device_get_match_data(dev);
	dcp->fixed_connector_type = dcp_connector_type_from_dt(dev->of_node);
	dcp->connector_type = dcp->fixed_connector_type;
	of_property_read_u32(dev->of_node, "apple,dcp-index", &dcp->index);
	of_property_read_u32(dev->of_node, "apple,dptx-phy", &dcp->dptx_phy);
	of_property_read_u32(dev->of_node, "apple,dptx-die", &dcp->dptx_die);
	dcp->fixed_dptx_phy = dcp->dptx_phy;
	INIT_DELAYED_WORK(&dcp->typec_reconnect_wq,
			  dcp_typec_reconnect_work);
	INIT_DELAYED_WORK(&dcp->typec_fabric_retrain_wq,
			  dcp_typec_retrain_work);

	if (!of_property_present(dev->of_node, "apple,iomfb-surfaces")) {
		ret = apple_dcp_surface_order_init(&dcp->iomfb_surfaces, NULL, 0);
	} else {
		num_surfaces = of_property_count_u32_elems(dev->of_node,
							  "apple,iomfb-surfaces");
		if (num_surfaces <= 0 || num_surfaces > DCP_MAX_PLANES)
			return dev_err_probe(dev,
				num_surfaces < 0 ? num_surfaces : -EINVAL,
				"invalid apple,iomfb-surfaces length %d\n",
				num_surfaces);
		ret = of_property_read_u32_array(dev->of_node,
						 "apple,iomfb-surfaces",
						 surface_ids, num_surfaces);
		if (!ret)
			ret = apple_dcp_surface_order_init(&dcp->iomfb_surfaces,
							   surface_ids,
							   num_surfaces);
	}
	if (ret)
		return dev_err_probe(dev, ret,
				     "invalid apple,iomfb-surfaces order\n");

	platform_set_drvdata(pdev, dcp);

	dcp->phy = devm_phy_optional_get(dev, "dp-phy");
	if (IS_ERR(dcp->phy)) {
		dev_err(dev, "Failed to get dp-phy: %ld\n", PTR_ERR(dcp->phy));
		return PTR_ERR(dcp->phy);
	}
	dcp->fixed_phy = dcp->phy;


	if (dcp->phy) {
		int ret;
		/*
		 * Request DP2HDMI related GPIOs as optional for DP-altmode
		 * compatibility. J180D misses a dp2hdmi-pwren GPIO in the
		 * template ADT. TODO: check device ADT
		 */
		dcp->hdmi_hpd = devm_gpiod_get_optional(dev, "hdmi-hpd", GPIOD_IN);
		if (IS_ERR(dcp->hdmi_hpd))
			return PTR_ERR(dcp->hdmi_hpd);
		if (dcp->hdmi_hpd) {
			int irq = gpiod_to_irq(dcp->hdmi_hpd);
			if (irq < 0) {
				dev_err(dev, "failed to translate HDMI hpd GPIO to IRQ\n");
				return irq;
			}
			dcp->hdmi_hpd_irq = irq;

			ret = devm_request_threaded_irq(dev, dcp->hdmi_hpd_irq,
						NULL, dcp_dp2hdmi_hpd,
						IRQF_ONESHOT | IRQF_NO_AUTOEN |
						IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
						"dp2hdmi-hpd-irq", dcp);
			if (ret < 0) {
				dev_err(dev, "failed to request HDMI hpd irq %d: %d\n",
					irq, ret);
				return ret;
			}
		}

		/*
		 * Power DP2HDMI on as it is required for the HPD irq.
		 * TODO: check if one is sufficient for the hpd to save power
		 *       on battery powered Macbooks.
		 */
		dcp->hdmi_pwren = devm_gpiod_get_optional(dev, "hdmi-pwren", GPIOD_OUT_HIGH);
		if (IS_ERR(dcp->hdmi_pwren))
			return PTR_ERR(dcp->hdmi_pwren);

		dcp->dp2hdmi_pwren = devm_gpiod_get_optional(dev, "dp2hdmi-pwren", GPIOD_OUT_HIGH);
		if (IS_ERR(dcp->dp2hdmi_pwren))
			return PTR_ERR(dcp->dp2hdmi_pwren);

		/*
		 * Acquire the display crossbar even when there is no
		 * "mux-index".  On J700 the crossbar that routes dispext0 to
		 * the ATC0 DP lanes must be selected when a DP-Alt sink
		 * arrives, not at probe: selecting it here would force the ATC
		 * PHY to DP-only while the rear connector still carries the
		 * debug cable.  dcp_typec_mux_set() does the select instead.
		 */
		if (of_property_present(dev->of_node, "mux-controls") &&
		    !of_property_present(dev->of_node, "mux-index")) {
			dcp->xbar = devm_mux_control_get(dev, "dp-xbar");
			if (IS_ERR(dcp->xbar)) {
				dev_err(dev, "Failed to get dp-xbar: %ld\n",
					PTR_ERR(dcp->xbar));
				return PTR_ERR(dcp->xbar);
			}
			/*
			 * Also route it now, before the DCP runs its own
			 * external-core detection.  With the crossbar left
			 * "disconnected" at init the firmware latches that core
			 * as noconnect and never looks again: routing it later,
			 * on the DP-Alt event, produced no firmware reaction at
			 * all.  This is a display-side mux, so selecting it does
			 * not disturb USB or the debug cable on the connector.
			 */
			ret = mux_control_select(dcp->xbar, DCP_DPXBAR_SELECTOR);
			if (ret) {
				dev_warn(dev, "J700_DPXBAR: probe select failed: %d\n",
					 ret);
			} else {
				dcp->xbar_selected = true;
				dev_info(dev, "J700_DPXBAR: selected at probe\n");
			}
		}

		/* A DCP may have both a fixed HDMI/DP route and allocatable Type-C
		 * routes. Keep the fixed route selected until the allocator borrows
		 * this otherwise-idle pipeline for a Type-C display.
		 */
		ret = dcp->fixed_phy ?
			of_property_read_u32(dev->of_node, "mux-index", &mux_index) :
			-ENODATA;
		if (!ret) {
			dcp->fixed_mux_index = mux_index;
			dcp->xbar = devm_mux_control_get(dev, "dp-xbar");
			if (IS_ERR(dcp->xbar)) {
				dev_err(dev, "Failed to get dp-xbar: %ld\n", PTR_ERR(dcp->xbar));
				return PTR_ERR(dcp->xbar);
			}
			ret = mux_control_select(dcp->xbar, mux_index);
			if (ret)
				dev_warn(dev, "mux_control_select failed: %d\n", ret);
			else
				dcp->fixed_route_selected = true;

			/*
			 * Switch atcphy to DP-only. should move to a Macbook Pro
			 * 14-/16-inch specific DP-to-HDMI drm_bridge.
			 */
			dcp->typec_mux = fwnode_typec_mux_get(dev_fwnode(dcp->dev));
			if (!IS_ERR_OR_NULL(dcp->typec_mux)) {
				struct typec_altmode alt = {
					.svid = USB_TYPEC_DP_SID,
				};
				struct typec_mux_state state = {
					.alt = &alt,
					.mode = TYPEC_DP_STATE_C,
				};
				int ret = typec_mux_set(dcp->typec_mux, &state);
				dev_info(dev, "typec_mux_set() returned: %d\n", ret);
				if (!ret)
					dcp->phy_managed_by_typec = true;
			} else {
				dev_info(dev, "fwnode_typec_mux_get() returned: %ld\n",
						IS_ERR(dcp->typec_mux) ? PTR_ERR(dcp->typec_mux) : 0);
				dcp->typec_mux = NULL;
			}
		}
	}

	ret = dcp_register_typec_routes(dcp);
	if (ret)
		return ret;
	ret = dcp_register_typec_mux(dcp);
	if (ret)
		return ret;

	return component_add(&pdev->dev, &dcp_comp_ops);
}

static void dcp_platform_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &dcp_comp_ops);
}

static void dcp_platform_shutdown(struct platform_device *pdev)
{
	component_del(&pdev->dev, &dcp_comp_ops);
}

static int dcp_platform_suspend(struct device *dev)
{
	struct apple_dcp *dcp = dev_get_drvdata(dev);

	if (dcp->avep)
		av_service_disconnect(dcp);

	if (dcp->hdmi_hpd_irq) {
		disable_irq(dcp->hdmi_hpd_irq);
		if (!dcp->active_typec_route) {
			disconnected_hpd_event(dcp->connector);
			dcp_dptx_disconnect(dcp, 0);
		}
	}
	/*
	 * Set the device as a wakeup device, which forces its power
	 * domains to stay on. We need this as we do not support full
	 * shutdown properly yet.
	 */
	device_set_wakeup_path(dev);

	return 0;
}

static int dcp_platform_resume(struct device *dev)
{
	struct apple_dcp *dcp = dev_get_drvdata(dev);

	if (dcp->hdmi_hpd_irq)
		enable_irq(dcp->hdmi_hpd_irq);

	if (dcp->avep)
		av_service_connect(dcp);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(dcp_platform_pm_ops,
				dcp_platform_suspend, dcp_platform_resume);


static const struct apple_dcp_hw_data apple_dcp_hw_t6020 = {
	.num_dptx_ports = 1,
};

static const struct apple_dcp_hw_data apple_dcp_hw_t8112 = {
	.num_dptx_ports = 2,
};

static const struct apple_dcp_hw_data apple_dcp_hw_t8140 = {
	.num_dptx_ports = 0,
	.adopt_running_coprocessor = true,
	.firmware_compat = DCP_FIRMWARE_H17P,
	.legacy_pmc_binding = true,
	/*
	 * Keep adopt_live_session disabled until apple-rtkit can recover the
	 * loader's endpoint map.  An adopted instance without that map cannot
	 * start DCP services.
	 */
};

/* M5 bring-up uses the shared H17 parser with a live firmware generation.
 * The wire format remains subject to validation on the target.
 */
static const struct apple_dcp_hw_data apple_dcp_hw_t8142 = {
	.num_dptx_ports = 0,
	.iomfb_method_profile = DCP_IOMFB_METHODS_H17G,
	.adopt_running_coprocessor = true,
	.adopt_live_session = true,
	.firmware_compat = DCP_FIRMWARE_H17P,
};

/*
 * The J700's two DCPs share one compatible, so they shared one hw_data and
 * with it num_dptx_ports = 0 -- which silently disabled DisplayPort on the
 * external pipe.  dptxep_init()'s port loop runs num_dptx_ports times, so at
 * zero it never waits for a port to enable, dptxport[0].enabled stays false
 * for ever, and dcp_dptx_connect() bails at "dptx service for port 0 not
 * enabled".  Everywhere else the split is by compatible: "apple,dcp" has 0
 * ports and "apple,dcpext" has 2.
 *
 * One port, not two: this machine has a single ATC PHY and a single
 * DP-capable connector (the front connector has no ATC endpoint at all), so
 * there is one DisplayPort to enable.  adopt_running_coprocessor stays on --
 * m1n1 hands this DCP over running and stopping the ASC kills the firmware.
 */
static const struct apple_dcp_hw_data apple_dcp_hw_t8140_ext = {
	.num_dptx_ports = 1,
	.adopt_running_coprocessor = true,
};

static const struct apple_dcp_hw_data apple_dcp_hw_dcp = {
	.num_dptx_ports = 0,
};

static const struct apple_dcp_hw_data apple_dcp_hw_dcpext = {
	.num_dptx_ports = 2,
};

static const struct of_device_id of_match[] = {
	{ .compatible = "apple,t6020-dcp", .data = &apple_dcp_hw_t6020,  },
	{ .compatible = "apple,t8112-dcp", .data = &apple_dcp_hw_t8112,  },
	{ .compatible = "apple,t8140-dcpext", .data = &apple_dcp_hw_t8140_ext, },
	{ .compatible = "apple,t8140-dcp", .data = &apple_dcp_hw_t8140,  },
	{ .compatible = "apple,t8142-dcp", .data = &apple_dcp_hw_t8142,  },
	{ .compatible = "apple,dcp",       .data = &apple_dcp_hw_dcp,    },
	{ .compatible = "apple,dcpext",    .data = &apple_dcp_hw_dcpext, },
	{}
};
MODULE_DEVICE_TABLE(of, of_match);

static struct platform_driver apple_platform_driver = {
	.probe		= dcp_platform_probe,
	.remove		= dcp_platform_remove,
	.shutdown	= dcp_platform_shutdown,
	.driver	= {
		.name = "apple-dcp",
		.of_match_table	= of_match,
		.pm = pm_sleep_ptr(&dcp_platform_pm_ops),
	},
};

void __init dcp_register(void)
{
	platform_driver_register(&apple_platform_driver);
}

void __exit dcp_unregister(void)
{
	platform_driver_unregister(&apple_platform_driver);
}
