/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef __APPLE_DCP_HDMI_H__
#define __APPLE_DCP_HDMI_H__

#include <linux/time64.h>
#include <linux/types.h>

struct apple_dcp;

/* Longest HPD low time still taken as a service request (DP: at most 1 ms). */
#define DCP_HDMI_IRQ_HPD_MAX_NS		(2 * NSEC_PER_MSEC)

enum dcp_hdmi_pulse_action {
	/* Pass edge to the threaded HPD presence handler (IRQ_WAKE_THREAD). */
	DCP_HDMI_PULSE_PASS,
	/* Falling edge: arm 2 ms timer and return IRQ_HANDLED. */
	DCP_HDMI_PULSE_ARM_TIMER,
	/* Timer already counted this short pulse: return IRQ_HANDLED. */
	DCP_HDMI_PULSE_CLAIMED,
	/* Valid short IRQ_HPD pulse: cancel timer, forward sink IRQ, IRQ_HANDLED. */
	DCP_HDMI_PULSE_SHORT,
};

/*
 * Classify an HPD GPIO edge observed while the HDMI link is up.
 * Caller serializes @falling_ns, @late_rising, and @claimed_ns.
 */
static inline enum dcp_hdmi_pulse_action
dcp_hdmi_classify_edge(u64 *falling_ns, bool *late_rising, u64 *claimed_ns,
		       u64 now_ns, int level)
{
	u64 dt;

	if (level < 0)
		return DCP_HDMI_PULSE_PASS;

	if (!level) {
		*late_rising = false;
		*claimed_ns = 0;
		if (!*falling_ns)
			*falling_ns = now_ns ?: 1;
		return DCP_HDMI_PULSE_ARM_TIMER;
	}

	if (!*falling_ns) {
		if (*claimed_ns) {
			u64 since = now_ns >= *claimed_ns ?
				    now_ns - *claimed_ns : 0;

			*claimed_ns = 0;
			if (since <= DCP_HDMI_IRQ_HPD_MAX_NS)
				return DCP_HDMI_PULSE_CLAIMED;
		}
		if (*late_rising) {
			*late_rising = false;
			return DCP_HDMI_PULSE_PASS;
		}
		/* Pulse started and ended before GPIO was sampled in hardirq. */
		return DCP_HDMI_PULSE_SHORT;
	}

	dt = now_ns >= *falling_ns ? now_ns - *falling_ns : 0;
	*falling_ns = 0;
	*late_rising = false;
	*claimed_ns = 0;

	if (dt <= DCP_HDMI_IRQ_HPD_MAX_NS)
		return DCP_HDMI_PULSE_SHORT;

	return DCP_HDMI_PULSE_PASS;
}

/*
 * Service requests of the DP-to-HDMI converter behind a built-in HDMI port.
 * Every call is a no-op for any other DCP.
 */
int dcp_hdmi_init(struct apple_dcp *dcp);
void dcp_hdmi_enable(struct apple_dcp *dcp);
void dcp_hdmi_disable(struct apple_dcp *dcp);
bool dcp_hdmi_hpd_edge(struct apple_dcp *dcp);
void dcp_hdmi_link_configured(struct apple_dcp *dcp, u32 link_rate);

#endif /* __APPLE_DCP_HDMI_H__ */
