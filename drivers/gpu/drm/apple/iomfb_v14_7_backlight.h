/* SPDX-License-Identifier: GPL-2.0-only OR MIT */

#ifndef __APPLE_IOMFB_V14_7_BACKLIGHT_H__
#define __APPLE_IOMFB_V14_7_BACKLIGHT_H__

#include <linux/types.h>

/*
 * Decisions of the 14.7 panel's level path (iomfb_v14_7.c), kept free of
 * driver state so that KUnit runs the same code.
 */

/*
 * Whether brightness-only presents may be queued and run. Set under the
 * gate lock before the work is cancelled: system sleep suspends it until
 * resume, unbind and remove close it for good.
 */
enum dcp_v14_bl_gate {
	DCP_V14_BL_OPEN,
	DCP_V14_BL_SUSPENDED,
	DCP_V14_BL_CLOSED,
};

/* Whether a brightness-only present may be queued, or run once queued. */
static inline bool dcp_v14_bl_may_queue(enum dcp_v14_bl_gate gate)
{
	return gate == DCP_V14_BL_OPEN;
}

static inline enum dcp_v14_bl_gate dcp_v14_bl_gate_suspend(enum dcp_v14_bl_gate gate)
{
	return gate == DCP_V14_BL_OPEN ? DCP_V14_BL_SUSPENDED : gate;
}

static inline enum dcp_v14_bl_gate dcp_v14_bl_gate_resume(enum dcp_v14_bl_gate gate)
{
	return gate == DCP_V14_BL_SUSPENDED ? DCP_V14_BL_OPEN : gate;
}

/*
 * Whether a swap carries the level the backlight policy reserves: only the
 * internal panel of a board whose level block was validated (T6030), only
 * while it shows its own panel (not a J613 internal pipe lent to a Type-C
 * display), and only with an active policy.
 */
static inline bool dcp_v14_swap_takes_nits(bool panel_nits, bool external, bool typec,
					   bool policy_active)
{
	return panel_nits && !external && !typec && policy_active;
}

/*
 * Blanking follows DPMS only once Linux has set a level: before that the
 * panel shows the boot loader's level, and a DPMS cycle must not replace it
 * with the reported default.
 */
static inline bool dcp_v14_bl_follow_dpms(bool panel_nits, bool policy_active,
					  bool level_known)
{
	return panel_nits && policy_active && level_known;
}

struct dcp_v14_bl_facts {
	enum dcp_v14_bl_gate gate;
	bool panel_nits;
	bool external;
	bool typec;
	bool started;
	bool failed;
	bool crashed;
	bool quiescing;
	/* A level is reserved-able: dirty and not in flight. */
	bool pending;
	/* The last present showed a frame (active_fb), not the background. */
	bool frame;
	bool mode_valid;
};

enum dcp_v14_bl_action {
	/* Nothing to do, or not allowed to: no RPC. */
	DCP_V14_BL_SKIP,
	/* Blanked or no frame yet: the next atomic present carries the level. */
	DCP_V14_BL_WAIT,
	/* Present the frame shown last again, with the level. */
	DCP_V14_BL_REPRESENT,
};

static inline enum dcp_v14_bl_action dcp_v14_bl_plan(const struct dcp_v14_bl_facts *f)
{
	if (!dcp_v14_bl_may_queue(f->gate) || !f->panel_nits || f->external || f->typec ||
	    !f->started || f->failed || f->crashed || f->quiescing || !f->pending)
		return DCP_V14_BL_SKIP;
	if (!f->frame || !f->mode_valid)
		return DCP_V14_BL_WAIT;
	return DCP_V14_BL_REPRESENT;
}

#endif /* __APPLE_IOMFB_V14_7_BACKLIGHT_H__ */
