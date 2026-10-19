// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * HDMI port of the T6030 (M3 Pro) MacBook Pros.
 *
 * The port hangs off the first external display coprocessor: it drives
 * ATC3 as a four-lane DisplayPort PHY into an on-board DP-to-HDMI
 * converter. Power, hotplug and the fixed route work as for the M1/M2 Pro
 * MacBook Pro HDMI port (hdmi-hpd, hdmi-pwren and dp2hdmi-pwren GPIOs, see
 * dcp.c and dcp-fabric.c).
 *
 * Like any DisplayPort sink, the converter asks the source for service by
 * pulsing HPD low for 0.5 to 1 ms (IRQ_HPD). It does that on link status
 * changes and while it trains an HDMI FRL link to the display, which the
 * modes beyond HDMI 2.0 rates need. The DPTX firmware services a request
 * only once it is told that the sink raised one. This file tells it:
 *
 *  - for every short low pulse on the HPD GPIO while the HDMI link is up;
 *  - every 100 ms while the firmware holds the sink (from its connection
 *    to its release), because not every request of the converter shows up
 *    as a pulse on the GPIO, and on a board whose HPD GPIO sleeps (the
 *    J514S line is on the AOP) none does: without the service the firmware
 *    never learns of the converter's events (sink metadata, FRL readiness).
 *
 * Only the T6030 external coprocessor with a fixed HDMI route uses this.
 */

#include <linux/atomic.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/jiffies.h>
#include <linux/ktime.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include <drm/drm_mode.h>

#include "afk.h"
#include "dcp-hdmi.h"
#include "dcp-internal.h"
#include "dptxep.h"

/* Longest HPD low time still taken as a service request (DP: at most 1 ms). */
#define DCP_HDMI_IRQ_HPD_MAX_NS		(2 * NSEC_PER_MSEC)
/* Requests are polled at this period while the sink is held. */
#define DCP_HDMI_SERVICE_PERIOD_MS	100

/* DPTX remote port call: the sink raised an IRQ_HPD service request. */
#define DPTX_GROUP_HOTPLUG		8
#define DPTX_CMD_SINK_IRQ		9
#define DPTX_SINK_IRQ_PAD		16

struct dcp_hdmi {
	struct apple_dcp *dcp;
	struct delayed_work work;
	/* Hard IRQ context only: time of the last falling HPD edge, or 0. */
	u64 falling_ns;
	/* Short pulses seen and not yet passed on. */
	atomic_t pulses;
	/* The firmware holds the sink: requests are polled. */
	bool armed;
	/* Polls that the firmware refused, in a row. */
	unsigned int refused;
	/* The HPD GPIO can be read from hard IRQ context. */
	bool timed_edges;
	/* Work context only. */
	bool announced;
};

static bool dcp_hdmi_link_up(struct apple_dcp *dcp)
{
	/* A Type-C display that borrowed this pipeline has its own HPD. */
	return !READ_ONCE(dcp->crashed) && !READ_ONCE(dcp->active_typec_route) &&
	       READ_ONCE(dcp->dptxport[0].enabled) &&
	       READ_ONCE(dcp->dptxport[0].connected);
}

/*
 * No hpd_mutex: the firmware may be waiting for this request while another
 * DPTX call made under that lock is still outstanding. AFK runs concurrent
 * service calls on one endpoint.
 */
static int dcp_hdmi_sink_irq(struct apple_dcp *dcp)
{
	struct apple_epic_service *service = READ_ONCE(dcp->dptxport[0].service);

	if (!service)
		return -ENODEV;
	return afk_service_call(service, DPTX_GROUP_HOTPLUG, DPTX_CMD_SINK_IRQ,
				NULL, 0, DPTX_SINK_IRQ_PAD, NULL, 0,
				DPTX_SINK_IRQ_PAD);
}

static void dcp_hdmi_work(struct work_struct *work)
{
	struct dcp_hdmi *hdmi = container_of(to_delayed_work(work),
					     struct dcp_hdmi, work);
	struct apple_dcp *dcp = hdmi->dcp;
	bool pulse = atomic_xchg(&hdmi->pulses, 0) > 0;
	bool poll = READ_ONCE(hdmi->armed);
	int ret;

	if (!pulse && !poll)
		return;
	/* The link rate can be set before the connection is marked up. */
	if (!dcp_hdmi_link_up(dcp))
		goto next;

	ret = dcp_hdmi_sink_irq(dcp);
	if (ret) {
		dev_warn_ratelimited(dcp->dev,
				     "HDMI: converter service request not delivered: %d\n",
				     ret);
		/* Don't keep polling a firmware that refuses the request. */
		if (++hdmi->refused >= 3) {
			WRITE_ONCE(hdmi->armed, false);
			dev_warn(dcp->dev, "HDMI: converter service stopped until the next sink\n");
		}
		return;
	}
	hdmi->refused = 0;
	if (!hdmi->announced) {
		hdmi->announced = true;
		dev_info(dcp->dev, "HDMI: forwarding converter service requests (%s)\n",
			 pulse ? "HPD pulse" : "sink held");
	} else {
		dev_dbg(dcp->dev, "HDMI: converter service request (%s)\n",
			pulse ? "HPD pulse" : "poll");
	}

next:
	if (READ_ONCE(hdmi->armed))
		mod_delayed_work(system_freezable_wq, &hdmi->work,
				 msecs_to_jiffies(DCP_HDMI_SERVICE_PERIOD_MS));
}

/*
 * Hard IRQ context, from the HPD GPIO's edge handler, on every edge.
 * Records the low time of each pulse and queues short ones.
 */
void dcp_hdmi_hpd_edge(struct apple_dcp *dcp)
{
	struct dcp_hdmi *hdmi = dcp->hdmi;
	u64 now, low;
	int level;

	if (!hdmi || !hdmi->timed_edges)
		return;

	now = ktime_get_ns();
	level = gpiod_get_value(dcp->hdmi_hpd);
	if (level < 0)
		return;
	if (!level) {
		hdmi->falling_ns = now;
		return;
	}

	/* A rising edge without a falling one: the pulse ended before we read it. */
	low = hdmi->falling_ns ? now - hdmi->falling_ns : 0;
	hdmi->falling_ns = 0;
	if (low > DCP_HDMI_IRQ_HPD_MAX_NS)
		return;	/* A cable change, handled by the HPD presence logic. */

	atomic_inc(&hdmi->pulses);
	mod_delayed_work(system_freezable_wq, &hdmi->work, 0);
}

/*
 * From DPTX DidChangeLinkConfiguration, with the rate that is now set: the
 * converter trains an HDMI FRL link to the display behind an HBR3 link and
 * raises its requests then, so one goes out at once.
 */
void dcp_hdmi_link_configured(struct apple_dcp *dcp, u32 link_rate)
{
	struct dcp_hdmi *hdmi = dcp->hdmi;

	if (!hdmi || !READ_ONCE(hdmi->armed) || link_rate != LINK_RATE_HBR3)
		return;
	mod_delayed_work(system_freezable_wq, &hdmi->work, 0);
}

/*
 * The firmware holds the sink (from REQUEST_DISPLAY to its release): its
 * requests are polled meanwhile. The first poll goes out once the link is up
 * (dcp_hdmi_link_up()); the work re-arms itself until the sink is released.
 */
void dcp_hdmi_sink_held(struct apple_dcp *dcp, bool held)
{
	struct dcp_hdmi *hdmi = dcp->hdmi;

	if (!hdmi || READ_ONCE(hdmi->armed) == held)
		return;
	WRITE_ONCE(hdmi->armed, held);
	hdmi->refused = 0;
	if (held)
		mod_delayed_work(system_freezable_wq, &hdmi->work,
				 msecs_to_jiffies(DCP_HDMI_SERVICE_PERIOD_MS));
}

void dcp_hdmi_enable(struct apple_dcp *dcp)
{
	if (dcp->hdmi)
		enable_delayed_work(&dcp->hdmi->work);
}

/* Returns with no request in flight; later edges queue nothing. */
void dcp_hdmi_disable(struct apple_dcp *dcp)
{
	if (dcp->hdmi)
		disable_delayed_work_sync(&dcp->hdmi->work);
}

static void dcp_hdmi_release(void *data)
{
	struct dcp_hdmi *hdmi = data;

	/* The HPD IRQ was requested later, so it is already freed. */
	hdmi->dcp->hdmi = NULL;
	disable_delayed_work_sync(&hdmi->work);
}

/*
 * Called from probe once the HPD GPIO is known and before its IRQ is
 * requested, so that the IRQ is released before this state on unbind.
 */
int dcp_hdmi_init(struct apple_dcp *dcp)
{
	struct dcp_hdmi *hdmi;
	int ret;

	if (!dcp->hdmi_hpd ||
	    dcp->fixed_connector_type != DRM_MODE_CONNECTOR_HDMIA ||
	    !of_device_is_compatible(dcp->dev->of_node, "apple,t6030-dcpext"))
		return 0;

	hdmi = devm_kzalloc(dcp->dev, sizeof(*hdmi), GFP_KERNEL);
	if (!hdmi)
		return -ENOMEM;

	hdmi->dcp = dcp;
	atomic_set(&hdmi->pulses, 0);
	hdmi->timed_edges = !gpiod_cansleep(dcp->hdmi_hpd);
	INIT_DELAYED_WORK(&hdmi->work, dcp_hdmi_work);
	/* Balanced by dcp_hdmi_enable() once the DPTX endpoint may be used. */
	disable_delayed_work(&hdmi->work);

	ret = devm_add_action_or_reset(dcp->dev, dcp_hdmi_release, hdmi);
	if (ret)
		return ret;

	dcp->hdmi = hdmi;
	if (!hdmi->timed_edges)
		dev_info(dcp->dev, "HDMI: HPD GPIO can sleep, converter requests are polled only\n");
	return 0;
}
