// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Native HDMI ports: an external display coprocessor driving a DisplayPort
 * PHY into an on-board DP-to-HDMI converter, as on the M1 and later Pro/Max
 * MacBook Pros, the Mac minis and the Mac Studios. Power, hotplug and the
 * fixed route use the hdmi-hpd, hdmi-pwren and dp2hdmi-pwren GPIOs (see
 * dcp.c and dcp-fabric.c).
 *
 * Like any DisplayPort sink, the converter asks the source for service by
 * pulsing HPD low for 0.5 to 1 ms (IRQ_HPD). It does that on link status
 * changes, on HDMI-CEC TX completion and RX frame arrival, and while it trains
 * an HDMI FRL link to the display, which the modes beyond HDMI 2.0 rates need.
 * The DPTX firmware services a request only once it is told that the sink
 * raised one. This file tells it:
 *
 *  - for every short low pulse on the HPD GPIO while the HDMI link is up,
 *    handling the falling edge in hard IRQ context with a 2 ms hrtimer so an
 *    IRQF_ONESHOT threaded handler does not mask the rising edge for 500 ms,
 *    and waking the threaded HPD handler via irq_wake_thread() if HPD stays
 *    low past 2 ms;
 *  - every 100 ms for a while after the firmware configured an HBR3 link,
 *    the rate those modes use, because not every request of the converter
 *    shows up as a pulse on the GPIO.
 *
 * Every coprocessor with a fixed HDMI-A connector and an HDMI HPD GPIO in
 * its DT node uses this: IRQ_HPD is DisplayPort protocol, and the converter
 * sits behind the same DPTX port on every such board. A firmware that does
 * not take the request answers it with an error; that stops the polling.
 */

#include <linux/atomic.h>
#include <linux/cleanup.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/ktime.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>

#include <drm/drm_mode.h>

#include "afk.h"
#include "connector.h"
#include "dcp-fabric.h"
#include "dcp-hdmi.h"
#include "dcp-internal.h"
#include "dptxep.h"

/* Requests are polled this long after an HBR3 link configuration ... */
#define DCP_HDMI_SERVICE_WINDOW_MS	10000
/* ... at this period. */
#define DCP_HDMI_SERVICE_PERIOD_MS	100

/* DPTX remote port call: the sink raised an IRQ_HPD service request. */
#define DPTX_GROUP_HOTPLUG		8
#define DPTX_CMD_SINK_IRQ		9
#define DPTX_SINK_IRQ_PAD		16

struct dcp_hdmi {
	struct apple_dcp *dcp;
	struct delayed_work work;
	struct hrtimer pulse_timer;
	/* Protects falling_ns, late_rising, claimed_ns, and enabled. */
	spinlock_t lock;
	/* Time of the last falling HPD edge while link is up, or 0. */
	u64 falling_ns;
	/* Time when 2 ms timer counted a high pulse, bounding double-count skip. */
	u64 claimed_ns;
	/* True after 2 ms timer expired low, so the next rising edge wakes HPD. */
	bool late_rising;
	/* True if pulse_timer woke the threaded HPD IRQ handler since enable. */
	bool thread_woken;
	/* True between dcp_hdmi_enable() and dcp_hdmi_disable(). */
	bool enabled;
	/* Set if DPTX firmware refuses DPTX_CMD_SINK_IRQ. */
	bool unsupported;
	/* Short pulses seen and not yet passed on. */
	atomic_t pulses;
	/* Poll requests until this time (jiffies); 0 when not polling. */
	unsigned long poll_until;
	/* The HPD GPIO can be read from hard IRQ context. */
	bool timed_edges;
	/* Work context only. */
	bool announced;
};

static bool dcp_hdmi_link_up(struct apple_dcp *dcp)
{
	/*
	 * Desktop 26.x keeps dptxport[0] connected while idle so its firmware
	 * is ready for hotplug; only an attached sink raises IRQ_HPD pulses.
	 */
	if (dcp->fw_compat == DCP_FIRMWARE_V_26_6_DESKTOP &&
	    (!dcp->connector || !READ_ONCE(dcp->connector->connected)))
		return false;

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
	unsigned long until = READ_ONCE(hdmi->poll_until);
	bool pulse = atomic_xchg(&hdmi->pulses, 0) > 0;
	bool poll = until && time_before(jiffies, until);
	int ret;

	if ((!pulse && !poll) || READ_ONCE(hdmi->unsupported))
		return;
	/* The link rate can be set before the connection is marked up. */
	if (!dcp_hdmi_link_up(dcp))
		goto next;

	ret = dcp_hdmi_sink_irq(dcp);
	if (ret) {
		dev_warn_ratelimited(dcp->dev,
				     "HDMI: converter service request not delivered: %d\n",
				     ret);
		/* Don't keep polling or forwarding to firmware that refuses it. */
		cmpxchg(&hdmi->poll_until, until, 0);
		if (ret != -ENODEV && ret != -ETIMEDOUT)
			WRITE_ONCE(hdmi->unsupported, true);
		return;
	}
	if (!hdmi->announced) {
		hdmi->announced = true;
		dev_info(dcp->dev, "HDMI: forwarding converter service requests (%s)\n",
			 pulse ? "HPD pulse" : "HBR3 link");
	} else {
		dev_dbg(dcp->dev, "HDMI: converter service request (%s)\n",
			pulse ? "HPD pulse" : "poll");
	}

next:
	if (poll)
		mod_delayed_work(system_freezable_wq, &hdmi->work,
				 msecs_to_jiffies(DCP_HDMI_SERVICE_PERIOD_MS));
}

static enum hrtimer_restart dcp_hdmi_pulse_timer(struct hrtimer *timer)
{
	struct dcp_hdmi *hdmi = container_of(timer, struct dcp_hdmi, pulse_timer);
	struct apple_dcp *dcp = hdmi->dcp;
	bool real_unplug = false;
	bool short_pulse = false;
	unsigned long flags;
	u64 now;
	int level;

	now = ktime_get_ns();
	level = gpiod_get_value(dcp->hdmi_hpd);

	spin_lock_irqsave(&hdmi->lock, flags);
	if (hdmi->enabled && hdmi->falling_ns) {
		hdmi->falling_ns = 0;
		if (level <= 0) {
			hdmi->late_rising = true;
			hdmi->claimed_ns = 0;
			hdmi->thread_woken = dcp->hdmi_hpd_irq > 0;
			real_unplug = true;
		} else {
			hdmi->late_rising = false;
			hdmi->claimed_ns = now ?: 1;
			short_pulse = true;
		}
	}
	spin_unlock_irqrestore(&hdmi->lock, flags);

	if (real_unplug) {
		dcp_dp2hdmi_record_edge(dcp);
		if (dcp->hdmi_hpd_irq > 0)
			irq_wake_thread(dcp->hdmi_hpd_irq, dcp);
	} else if (short_pulse && !READ_ONCE(hdmi->unsupported)) {
		atomic_inc(&hdmi->pulses);
		mod_delayed_work(system_freezable_wq, &hdmi->work, 0);
	}

	return HRTIMER_NORESTART;
}

/*
 * Hard IRQ context, from the HPD GPIO's edge handler, on every edge.
 * Returns true when handled as short-pulse tracking (keeping the IRQF_ONESHOT
 * GPIO edge detector armed), or false when the threaded HPD handler should run.
 */
bool dcp_hdmi_hpd_edge(struct apple_dcp *dcp)
{
	struct dcp_hdmi *hdmi = dcp->hdmi;
	enum dcp_hdmi_pulse_action action;
	bool cancel_timer = false;
	unsigned long flags;
	u64 now;
	int level;

	if (!hdmi || !hdmi->timed_edges || READ_ONCE(hdmi->unsupported))
		return false;

	if (!dcp_hdmi_link_up(dcp)) {
		spin_lock_irqsave(&hdmi->lock, flags);
		if (hdmi->falling_ns || hdmi->late_rising || hdmi->claimed_ns) {
			hdmi->falling_ns = 0;
			hdmi->late_rising = false;
			hdmi->claimed_ns = 0;
			cancel_timer = true;
		}
		spin_unlock_irqrestore(&hdmi->lock, flags);
		if (cancel_timer)
			hrtimer_try_to_cancel(&hdmi->pulse_timer);
		return false;
	}

	now = ktime_get_ns();
	level = gpiod_get_value(dcp->hdmi_hpd);

	spin_lock_irqsave(&hdmi->lock, flags);
	if (!hdmi->enabled) {
		spin_unlock_irqrestore(&hdmi->lock, flags);
		return false;
	}
	action = dcp_hdmi_classify_edge(&hdmi->falling_ns, &hdmi->late_rising,
					&hdmi->claimed_ns, now, level);
	if (action == DCP_HDMI_PULSE_ARM_TIMER)
		hrtimer_start(&hdmi->pulse_timer,
			      ns_to_ktime(DCP_HDMI_IRQ_HPD_MAX_NS),
			      HRTIMER_MODE_REL);
	spin_unlock_irqrestore(&hdmi->lock, flags);

	switch (action) {
	case DCP_HDMI_PULSE_ARM_TIMER:
		return true;
	case DCP_HDMI_PULSE_CLAIMED:
		return true;
	case DCP_HDMI_PULSE_SHORT:
		hrtimer_try_to_cancel(&hdmi->pulse_timer);
		atomic_inc(&hdmi->pulses);
		mod_delayed_work(system_freezable_wq, &hdmi->work, 0);
		return true;
	case DCP_HDMI_PULSE_PASS:
	default:
		hrtimer_try_to_cancel(&hdmi->pulse_timer);
		return false;
	}
}

/* From DPTX DidChangeLinkConfiguration, with the rate that is now set. */
void dcp_hdmi_link_configured(struct apple_dcp *dcp, u32 link_rate)
{
	struct dcp_hdmi *hdmi = dcp->hdmi;
	unsigned long until;

	if (!hdmi || READ_ONCE(hdmi->unsupported))
		return;
	if (link_rate != LINK_RATE_HBR3) {
		WRITE_ONCE(hdmi->poll_until, 0);
		return;
	}

	until = jiffies + msecs_to_jiffies(DCP_HDMI_SERVICE_WINDOW_MS);
	WRITE_ONCE(hdmi->poll_until, until ?: 1);
	mod_delayed_work(system_freezable_wq, &hdmi->work, 0);
}

void dcp_hdmi_enable(struct apple_dcp *dcp)
{
	struct dcp_hdmi *hdmi = dcp->hdmi;

	if (!hdmi)
		return;

	enable_delayed_work(&hdmi->work);
	scoped_guard(spinlock_irqsave, &hdmi->lock) {
		hdmi->falling_ns = 0;
		hdmi->claimed_ns = 0;
		hdmi->late_rising = false;
		hdmi->thread_woken = false;
		hdmi->enabled = true;
	}
}

/* Returns with no request in flight; later edges queue nothing. */
void dcp_hdmi_disable(struct apple_dcp *dcp)
{
	struct dcp_hdmi *hdmi = dcp->hdmi;
	bool wait_irq = false;

	if (!hdmi)
		return;

	scoped_guard(spinlock_irqsave, &hdmi->lock) {
		hdmi->enabled = false;
		hdmi->falling_ns = 0;
		hdmi->claimed_ns = 0;
		hdmi->late_rising = false;
	}
	hrtimer_cancel(&hdmi->pulse_timer);
	scoped_guard(spinlock_irqsave, &hdmi->lock) {
		wait_irq = hdmi->thread_woken;
		hdmi->thread_woken = false;
	}
	if (wait_irq && dcp->hdmi_hpd_irq > 0)
		synchronize_irq(dcp->hdmi_hpd_irq);
	disable_delayed_work_sync(&hdmi->work);
}

static void dcp_hdmi_release(void *data)
{
	struct dcp_hdmi *hdmi = data;

	/* The HPD IRQ was requested later, so it is already freed. */
	hdmi->dcp->hdmi = NULL;
	scoped_guard(spinlock_irqsave, &hdmi->lock) {
		hdmi->enabled = false;
		hdmi->falling_ns = 0;
		hdmi->claimed_ns = 0;
		hdmi->late_rising = false;
	}
	hrtimer_cancel(&hdmi->pulse_timer);
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
	    dcp->fixed_connector_type != DRM_MODE_CONNECTOR_HDMIA)
		return 0;

	hdmi = devm_kzalloc(dcp->dev, sizeof(*hdmi), GFP_KERNEL);
	if (!hdmi)
		return -ENOMEM;

	hdmi->dcp = dcp;
	spin_lock_init(&hdmi->lock);
	atomic_set(&hdmi->pulses, 0);
	hdmi->timed_edges = !gpiod_cansleep(dcp->hdmi_hpd);
	hrtimer_setup(&hdmi->pulse_timer, dcp_hdmi_pulse_timer, CLOCK_MONOTONIC,
		      HRTIMER_MODE_REL);
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
