/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/* Copyright 2026 Ryan Murray */

#ifndef __APPLE_DCP_BACKLIGHT_H__
#define __APPLE_DCP_BACKLIGHT_H__

#include <linux/errno.h>
#include <linux/limits.h>
#include <linux/math64.h>
#include <linux/minmax.h>
#include <linux/types.h>

/*
 * Commanded nits, independent of firmware encoding.  target is the requested
 * level and level the one presented while the panel is lit.  They differ only
 * while a fade runs; the caller supplies the fade's clock.
 */
struct dcp_backlight_state {
	u32 maximum;
	u32 target;
	u32 level;
	u32 fade_from;
	u32 actual;
	u32 sent_nits;
	/* Firmware observations confirm output; they never become user requests. */
	u32 reported;
	u64 sequence;
	u8 retries;
	bool ready;
	bool controlled;
	/* An unchanged registration default cannot take over an unknown level. */
	bool preserve_inherited;
	bool dpms_off;
	bool core_blank;
	bool suspended;
	bool dirty;
	bool in_flight;
	/* Fade between lit levels instead of jumping to them. */
	bool fade;
	/* A fade from fade_from to target is running. */
	bool fading;
	/*
	 * The panel shows a level Linux knows: the loader's reported level, or
	 * one a completed present carried.  A default used for registration
	 * without a loader report is not known until a present carries a level.
	 */
	bool level_known;
	bool report_valid;
};

struct dcp_backlight_present {
	u64 sequence;
	u32 nits;
};

/*
 * Public powerlog reports use millinits.  A report slightly above the panel
 * ceiling describes the brightest level the panel can show, so clamp it; one
 * more than twice the ceiling is not a credible panel level.
 */
static inline int dcp_bl_takeover_nits(u32 maximum, u32 millinits, u32 *nits)
{
	if (!maximum || maximum > INT_MAX / 2000)
		return -EINVAL;
	if (millinits / 1000 > 2 * maximum)
		return -ERANGE;
	*nits = min(millinits / 1000, maximum);
	return 0;
}

/*
 * An inherited level comes from a loader report.  A default level is only
 * reported to userspace; it is never presented on its own (see level_known).
 */
static inline int dcp_bl_init(struct dcp_backlight_state *state, u32 maximum,
			      bool inherited_valid, u32 inherited,
			      bool default_valid, u32 default_nits)
{
	u32 nits;

	if (!maximum || maximum > INT_MAX)
		return -EINVAL;
	if (!inherited_valid && !default_valid)
		return -ENODATA;
	nits = inherited_valid ? inherited : default_nits;
	if (nits > maximum)
		return -ERANGE;

	*state = (struct dcp_backlight_state) {
		.maximum = maximum,
		.target = nits,
		.level = nits,
		.actual = nits,
		.reported = nits,
		.ready = true,
		.level_known = inherited_valid,
		.report_valid = inherited_valid,
	};
	return 0;
}

/*
 * A decoded powerlog sample can seed takeover only before Linux control.
 * It reports the level the panel shows, so the level is then known.
 */
static inline bool dcp_bl_seed(struct dcp_backlight_state *state, u32 nits)
{
	if (!state->ready || state->controlled || nits > state->maximum)
		return false;
	state->target = nits;
	state->level = nits;
	state->actual = nits;
	state->level_known = true;
	return true;
}

/*
 * A report can establish takeover once. After Linux has a level or target,
 * keep the observation separate from target, fade progress and completion.
 * In particular, delayed reports cannot restart a fade or request a resend.
 */
static inline bool dcp_bl_report(struct dcp_backlight_state *state, u32 nits)
{
	if (!state->ready || nits > state->maximum)
		return false;
	state->reported = nits;
	state->report_valid = true;
	if (state->controlled || state->level_known)
		return false;
	return dcp_bl_seed(state, nits);
}

/* Report a usable target until a hint or completed present gives the level. */
static inline int dcp_bl_actual(const struct dcp_backlight_state *state)
{
	if (!state->ready)
		return -ENODATA;
	return state->level_known ? state->actual : state->target;
}

static inline bool dcp_bl_lit(const struct dcp_backlight_state *state)
{
	return !state->dpms_off && !state->core_blank && !state->suspended;
}

static inline u32 dcp_bl_effective(const struct dcp_backlight_state *state)
{
	return dcp_bl_lit(state) ? state->level : 0;
}

/* Stop a running fade and present the target directly. */
static inline void dcp_bl_jump(struct dcp_backlight_state *state)
{
	state->fading = false;
	state->level = state->target;
	state->dirty = true;
}

/*
 * With fades enabled, a new level for a panel that stays lit at a known level
 * starts a fade from the level presented last; dcp_bl_fade_step() moves it.
 * Blanking, unblanking and a level the panel may not show still jump.
 */
static inline int dcp_bl_request(struct dcp_backlight_state *state, u32 nits,
				 bool core_blank, bool suspended)
{
	bool was_lit = dcp_bl_lit(state);

	if (!state->ready)
		return -ENODATA;
	if (nits > state->maximum)
		return -ERANGE;
	/* Save/restore clients may echo the reporting default without a change. */
	if (state->preserve_inherited && !state->controlled && !state->level_known &&
	    state->target == nits) {
		state->core_blank = core_blank;
		state->suspended = suspended;
		return 0;
	}
	/* Core blank/suspend notifications do not establish a panel level. */
	if (!state->controlled && !state->level_known && state->target == nits &&
	    (state->core_blank != core_blank || state->suspended != suspended)) {
		state->core_blank = core_blank;
		state->suspended = suspended;
		return 0;
	}
	state->controlled = true;
	state->retries = 0;
	/*
	 * Until a present carries a level, the panel does not show the target,
	 * so even a request for the reported default has to be presented.
	 */
	if (state->target != nits || state->core_blank != core_blank ||
	    state->suspended != suspended || !state->level_known) {
		state->target = nits;
		state->core_blank = core_blank;
		state->suspended = suspended;
		/* With no hint, the reporting target is the first fade's origin. */
		if (state->fade && (state->level_known || state->preserve_inherited) && was_lit &&
		    dcp_bl_lit(state) && state->level != nits) {
			state->fade_from = state->level;
			state->fading = true;
		} else {
			dcp_bl_jump(state);
		}
	}
	return 0;
}

/*
 * Move a running fade to @elapsed of @duration, in any one unit.  The level
 * follows a straight line from the level presented when the fade started,
 * and the last step is the target itself.  Returns true while the fade runs.
 */
static inline bool dcp_bl_fade_step(struct dcp_backlight_state *state,
				    u32 elapsed, u32 duration)
{
	u32 from = state->fade_from, to = state->target, level;

	if (!state->fading)
		return false;
	if (elapsed >= duration) {
		level = to;
		state->fading = false;
	} else if (to >= from) {
		level = from + div_u64(mul_u32_u32(to - from, elapsed), duration);
	} else {
		level = from - div_u64(mul_u32_u32(from - to, elapsed), duration);
	}
	if (state->level != level) {
		state->level = level;
		state->dirty = true;
	}
	return state->fading;
}

static inline int dcp_bl_dpms(struct dcp_backlight_state *state, bool on)
{
	if (!state->ready)
		return -ENODATA;
	state->controlled = true;
	state->retries = 0;
	if (state->dpms_off != !on) {
		state->dpms_off = !on;
		dcp_bl_jump(state);
	}
	return 0;
}

/*
 * The firmware asked for the level again.  Re-send the effective level once
 * Linux controls it and knows it, unless a present already carries or will
 * carry it.  A default level is never sent this way.
 */
static inline bool dcp_bl_resend(struct dcp_backlight_state *state)
{
	if (!state->ready || !state->controlled || !state->level_known ||
	    state->dirty || state->in_flight)
		return false;
	state->dirty = true;
	return true;
}

/*
 * Called by the single outbound queue, with a pinned, accepted scanout or a
 * prepared replacement. Soft-off never releases it. A brightness-only present
 * must use the last accepted surface, not a rejected replacement's DRM state.
 */
static inline int dcp_bl_prepare(struct dcp_backlight_state *state,
				 bool have_surface,
				 struct dcp_backlight_present *present)
{
	if (!state->ready || !have_surface)
		return -ENODATA;
	if (state->in_flight)
		return -EBUSY;
	if (!state->dirty)
		return -EALREADY;

	state->controlled = true;
	state->in_flight = true;
	state->dirty = false;
	state->sent_nits = dcp_bl_effective(state);
	present->nits = state->sent_nits;
	present->sequence = ++state->sequence;
	return 0;
}

/*
 * accepted means both accepted submit and its matching completed present.
 * Rejection/serialization failure cancels the reservation without changing
 * actual. This result is NEVER a framebuffer retirement authorization.
 */
static inline bool dcp_bl_complete(struct dcp_backlight_state *state,
				   u64 sequence, bool accepted)
{
	if (!state->in_flight || sequence != state->sequence)
		return false;
	state->in_flight = false;
	if (accepted) {
		state->actual = state->sent_nits;
		state->retries = 0;
		state->level_known = true;
	} else {
		state->dirty = true;
		if (state->retries < 4)
			state->retries++;
	}
	return true;
}

static inline int dcp_bl_reported_actual(const struct dcp_backlight_state *state, bool strict)
{
	if (strict && !state->level_known)
		return -ENODATA;
	return state->actual;
}

/* Three deferred retries per user request; a permanent rejection stays dirty. */
static inline unsigned int dcp_bl_retry_delay(const struct dcp_backlight_state *state)
{
	if (!state->ready || !state->dirty || state->in_flight ||
	    !state->retries || state->retries > 3)
		return 0;
	return 100U << (state->retries - 1);
}

/* A desktop present can complete before its submit reply arrives. */
struct dcp_backlight_swap {
	struct dcp_backlight_present present;
	u32 id;
	bool pending;
	bool accepted;
	bool completed;
};

static inline int dcp_bl_swap_prepare(struct dcp_backlight_state *state,
				      struct dcp_backlight_swap *swap,
				      u32 id, bool have_surface)
{
	struct dcp_backlight_present present;
	int ret = dcp_bl_prepare(state, have_surface, &present);

	if (ret)
		return ret;
	*swap = (struct dcp_backlight_swap) {
		.present = present,
		.id = id,
		.pending = true,
	};
	return 0;
}

static inline bool dcp_bl_swap_finish(struct dcp_backlight_state *state,
				      struct dcp_backlight_swap *swap,
				      u32 id, bool reply, bool accepted)
{
	if (!swap->pending || swap->id != id)
		return false;
	if (reply)
		swap->accepted = accepted;
	else
		swap->completed = true;
	if ((!reply || accepted) && !(swap->accepted && swap->completed))
		return false;
	swap->pending = false;
	return dcp_bl_complete(state, swap->present.sequence,
			      swap->accepted && swap->completed);
}

/*
 * Firmware that takes the panel level in each present without the policy
 * above: a DAC value (12.3, 13.5, the 14.7 template) or H17G nits.  Its own
 * reports of the level do not follow the presents.  The present that powers
 * the panel off takes it to 0, and the report goes to 0 with it; the present
 * that lights the panel again after the power-on brings no new report.  On a
 * J414s with 13.5 firmware the report stayed 0 for hours under a panel lit
 * at 500 nits, across nine display power cycles.  Once Linux has
 * presented a level, that level is the panel's; before that, only the
 * firmware's report of the loader's level is known.
 */
struct dcp_bl_presented {
	u32 nits;
	bool valid;
};

static inline void dcp_bl_presented_set(struct dcp_bl_presented *presented,
					u32 nits)
{
	presented->nits = nits;
	presented->valid = true;
}

static inline int dcp_bl_presented_actual(const struct dcp_bl_presented *presented,
					  int reported)
{
	return presented->valid ? presented->nits : reported;
}

struct apple_dcp;

void dcp_backlight_presented(struct apple_dcp *dcp, u32 nits);

void dcp_backlight_desktop_power(struct apple_dcp *dcp, bool on);
u32 dcp_backlight_desktop_nits(struct apple_dcp *dcp);
bool dcp_backlight_desktop_prepare(struct apple_dcp *dcp, u32 id,
				   bool have_surface, u32 *nits);
void dcp_backlight_desktop_finish(struct apple_dcp *dcp, u32 id,
				  bool reply, bool accepted);

/*
 * kick only queues work; it must not submit a top-level RPC from the caller.
 * The central IOMFB queue owns surface references, retries, and wire encoding.
 * Configure before registration and keep the callback valid until teardown.
 */
bool dcp_backlight_active(struct apple_dcp *dcp);
bool dcp_backlight_level_known(struct apple_dcp *dcp);
int dcp_backlight_takeover(struct apple_dcp *dcp, u32 millinits);
int dcp_backlight_configure(struct apple_dcp *dcp, u32 maximum,
			    bool inherited_valid, u32 inherited,
			    bool default_valid, u32 default_nits,
			    void (*kick)(struct apple_dcp *dcp));
bool dcp_backlight_seed(struct apple_dcp *dcp, u32 nits);
int dcp_backlight_dpms(struct apple_dcp *dcp, bool on);
int dcp_backlight_prepare(struct apple_dcp *dcp, bool have_surface,
			  struct dcp_backlight_present *present);
bool dcp_backlight_complete(struct apple_dcp *dcp, u64 sequence, bool accepted);
bool dcp_backlight_pending(struct apple_dcp *dcp);
bool dcp_backlight_lit(struct apple_dcp *dcp);
bool dcp_backlight_resend(struct apple_dcp *dcp);
int dcp_backlight_wake_lit(struct apple_dcp *dcp);
unsigned int dcp_backlight_retry_delay(struct apple_dcp *dcp);

#endif /* __APPLE_DCP_BACKLIGHT_H__ */
