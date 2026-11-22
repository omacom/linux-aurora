/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * Software camera indicator: pure state and policy.
 *
 * The kernel draws a small opaque square on a swap layer that userspace
 * cannot address. It is best effort: the camera never waits on it and never
 * depends on it. It is NOT the secure indicator Apple's firmware draws
 * outside the OS: root who can run its own kernel defeats it.
 */
#ifndef DCP_INDICATOR_H
#define DCP_INDICATOR_H

#include <linux/types.h>
#include <linux/errno.h>
#ifndef DCP_IND_USERSPACE
#include <drm/drm_crtc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_plane.h>
#endif

enum dcp_ind_state {
	DCP_IND_OFF,
	/* Requested; no present carrying the square has completed yet. */
	DCP_IND_PENDING,
	/* A present carrying the square completed. */
	DCP_IND_LIT,
};

struct dcp_indicator {
	enum dcp_ind_state state;
	/* Requested and not ended: it survives a DRM unbind, so the next bind shows it again. */
	bool wanted;
	/* A commit refused for a pending flip, and which one is being retried. */
	enum {
		DCP_IND_RETRY_NONE,
		DCP_IND_RETRY_ON,
		DCP_IND_RETRY_OFF,
	} retry;
};

/* One owner at a time: a second request cannot reset a running one. */
static inline int dcp_ind_begin(struct dcp_indicator *ind)
{
	if (ind->state != DCP_IND_OFF)
		return -EBUSY;
	ind->state = DCP_IND_PENDING;
	ind->wanted = true;
	/* This request puts the square on again, so a retried off has no meaning. */
	ind->retry = DCP_IND_RETRY_NONE;
	return 0;
}

/* A completed present: the first one that carried the square confirms it. */
static inline void dcp_ind_present_done(struct dcp_indicator *ind, bool dot_shown)
{
	if (ind->state == DCP_IND_PENDING && dot_shown)
		ind->state = DCP_IND_LIT;
}

/* The request is over: stopped, failed or withdrawn. An on retry has no request left. */
static inline void dcp_ind_end(struct dcp_indicator *ind)
{
	ind->state = DCP_IND_OFF;
	ind->wanted = false;
	ind->retry = DCP_IND_RETRY_NONE;
}

/* The plane is gone (DRM unbind) but the camera may still stream: keep the request. */
static inline void dcp_ind_suspend(struct dcp_indicator *ind)
{
	ind->state = DCP_IND_OFF;
	ind->retry = DCP_IND_RETRY_NONE;
}

struct dcp_ind_ops {
	int (*commit)(void *ctx, bool on, bool wait);
	void (*queue_retry)(void *ctx);
};

/* A pending flip gets one blocking commit from the work item. */
static inline int dcp_ind_commit(struct dcp_indicator *ind, bool on,
				 const struct dcp_ind_ops *ops, void *ctx, bool wait)
{
	int ret = ops->commit(ctx, on, wait);

	ind->retry = DCP_IND_RETRY_NONE;
	if (ret == -EBUSY && !wait) {
		ind->retry = on ? DCP_IND_RETRY_ON : DCP_IND_RETRY_OFF;
		ops->queue_retry(ctx);
		return 0;
	}
	if (ret == -ENODEV) {
		/* Nothing is lit, and the request (wanted) is kept. */
		ind->state = DCP_IND_OFF;
		return 0;
	}
	return ret;
}

static inline int dcp_ind_apply(struct dcp_indicator *ind, bool on,
				const struct dcp_ind_ops *ops, void *ctx)
{
	return dcp_ind_commit(ind, on, ops, ctx, false);
}

/* Do not poll or requeue if the blocking commit fails. */
static inline int dcp_ind_retry(struct dcp_indicator *ind, bool plane,
				const struct dcp_ind_ops *ops, void *ctx)
{
	if (!plane || ind->retry == DCP_IND_RETRY_NONE)
		return 0;
	return dcp_ind_commit(ind, ind->retry == DCP_IND_RETRY_ON, ops, ctx, true);
}

/*
 * The display came back (system resume) and the resume commit is done. The
 * reserved plane comes back dark: suspend disabled it, and the resume commit
 * that names the kernel's framebuffer is put back to that disabled state
 * (DCP_IND_PLANE_RESTORE). So the square is OFF again and a kept request asks
 * anew. A lit plane here means a set(true) landed between the resume commit
 * and this call: its commit is already on the panel, and resetting its state
 * would make the new request an unchanged plane, no swap would follow, and the
 * watch would withdraw the square.
 */
static inline void dcp_ind_display_resumed(struct dcp_indicator *ind, bool plane_dark)
{
	if (plane_dark)
		dcp_ind_suspend(ind);
}

/* Wanted but not on the panel: the next present after a rebind asks again. */
static inline bool dcp_ind_wants_resume(const struct dcp_indicator *ind)
{
	return ind->wanted && ind->state == DCP_IND_OFF;
}

enum dcp_ind_plane_action {
	DCP_IND_PLANE_PASS,
	DCP_IND_PLANE_RESTORE,
	DCP_IND_PLANE_REJECT,
};

/*
 * The reserved plane belongs to the kernel. A commit that clears it (FB_ID=0
 * and CRTC_ID=0), as the console restore does for every plane, still succeeds
 * and the plane is put back to the state it had. Any other userspace change (a
 * client's own framebuffer, a move, a property) is refused with -EINVAL, so a
 * compositor sees it in its TEST_ONLY commit and falls back instead of having
 * its buffer silently dropped. Carried unchanged, or disabled with the CRTC it
 * sits on (DPMS off, suspend, shutdown), the plane is left alone. The kernel's
 * own framebuffer is not userspace: suspend saves it and resume commits it
 * back, and refusing that would fail the resume, so it is put back like a clear
 * (the display resuming then asks for the square again).
 */
static inline enum dcp_ind_plane_action dcp_ind_plane_policy(bool reserved_plane,
							      bool kernel_commit, bool unchanged,
							      bool crtc_going_off, bool has_fb,
							      bool has_crtc, bool own_fb)
{
	if (!reserved_plane || kernel_commit || unchanged || crtc_going_off)
		return DCP_IND_PLANE_PASS;
	if ((!has_fb && !has_crtc) || (own_fb && has_fb))
		return DCP_IND_PLANE_RESTORE;
	return DCP_IND_PLANE_REJECT;
}

/*
 * Gathers what dcp_ind_plane_policy() needs from the plane states of a commit.
 * old_crtc_state is the new state of the CRTC the plane sat on, or NULL when the
 * commit does not touch it.
 */
static inline enum dcp_ind_plane_action
dcp_ind_plane_check(const struct drm_plane_state *old, const struct drm_plane_state *new,
		    const struct drm_crtc_state *old_crtc_state, bool kernel_commit,
		    const struct drm_framebuffer_funcs *own_fb_funcs)
{
	bool unchanged = new->fb == old->fb && new->crtc == old->crtc &&
			 new->crtc_x == old->crtc_x && new->crtc_y == old->crtc_y &&
			 new->crtc_w == old->crtc_w && new->crtc_h == old->crtc_h &&
			 new->src_x == old->src_x && new->src_y == old->src_y &&
			 new->src_w == old->src_w && new->src_h == old->src_h &&
			 new->alpha == old->alpha &&
			 new->pixel_blend_mode == old->pixel_blend_mode &&
			 new->rotation == old->rotation;
	bool going_off = !new->fb && !new->crtc && old_crtc_state && !old_crtc_state->active;

	return dcp_ind_plane_policy(true, kernel_commit, unchanged, going_off, new->fb,
				    new->crtc, new->fb && new->fb->funcs == own_fb_funcs);
}

#endif /* DCP_INDICATOR_H */
