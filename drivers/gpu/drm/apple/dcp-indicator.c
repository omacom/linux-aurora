// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Software camera indicator: a kernel-owned swap layer on the internal panel.
 *
 * The camera driver asks for it before the first frame and drops it after the
 * last. Userspace has no plane, property or ioctl for it. Best effort: the
 * camera does not wait on the result. A software indicator, not the secure
 * one. See dcp-indicator.h.
 */
#include <linux/jiffies.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/soc/apple/display.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_atomic_uapi.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_modeset_lock.h>

#include "dcp.h"
#include "dcp-internal.h"
#include "dcp-indicator.h"
#include "plane.h"

/* DCP rejects blended surfaces smaller than 32x32 (see plane.c). */
#define IND_SIDE		48
#define IND_MARGIN		12
#define IND_BORDER		3
#define IND_FILL		0xff34c759
#define IND_EDGE		0xff000000
#define IND_PRESENT_TIMEOUT	msecs_to_jiffies(1000)

static void dcp_indicator_watch(struct work_struct *work);
static void dcp_indicator_resume(struct work_struct *work);
static void dcp_indicator_retry(struct work_struct *work);

int dcp_indicator_probe(struct apple_dcp *dcp)
{
	struct dcp_camera_indicator *ci = &dcp->camera_ind;
	u32 surf;
	unsigned long top_surf = find_last_bit(dcp->iomfb_surfaces, SWAP_SURFACES);

	spin_lock_init(&ci->lock);
	mutex_init(&ci->ctl);
	INIT_DELAYED_WORK(&ci->watch, dcp_indicator_watch);
	INIT_WORK(&ci->resume, dcp_indicator_resume);
	INIT_WORK(&ci->retry, dcp_indicator_retry);

	if (of_property_read_u32(dcp->dev->of_node, "apple,camera-indicator-surface", &surf))
		return 0;
	/* The swap_done hook that confirms the square is on the non-H17G presents only. */
	if (dcp->hw.iomfb_method_profile == DCP_IOMFB_METHODS_H17G) {
		dev_warn(dcp->dev, "no camera indicator: this DCP profile has no swap_done hook\n");
		return 0;
	}
	/* find_last_bit is the bitmap size when no userspace slot is enabled. */
	if (surf >= SWAP_SURFACES || (top_surf != SWAP_SURFACES && surf <= top_surf))
		return dev_err_probe(dcp->dev, -EINVAL,
				     "camera indicator surface %u is not free above surface %lu\n",
				     surf, top_surf);
	ci->surf = surf;
	ci->has_surf = true;
	return 0;
}

/*
 * From the DCP's component unbind, while the DCP and the drm_device are both
 * alive: a drmm action would run after the DCP's memory is gone. surf and
 * has_surf come from the device tree, so the next bind builds the plane again.
 * A request that is still wanted is kept: the first present after the next
 * bind asks for the square again.
 */
void dcp_indicator_release(struct apple_dcp *dcp)
{
	struct dcp_camera_indicator *ci = &dcp->camera_ind;
	struct drm_framebuffer *fb;
	unsigned long flags;

	if (!ci->has_surf)
		return;
	scoped_guard(mutex, &ci->ctl) {
		fb = ci->fb;
		spin_lock_irqsave(&ci->lock, flags);
		ci->fb = NULL;
		ci->plane = NULL;
		dcp_ind_suspend(&ci->state);
		spin_unlock_irqrestore(&ci->lock, flags);
		if (fb)
			drm_framebuffer_put(fb);
	}
	/* With no fb nothing queues them again, and all take ctl, so cancel after dropping it. */
	cancel_delayed_work_sync(&ci->watch);
	cancel_work_sync(&ci->retry);
	cancel_work_sync(&ci->resume);
}

/* Its funcs name the kernel's own framebuffer; userspace cannot create one with them. */
const struct drm_framebuffer_funcs dcp_indicator_fb_funcs = {
	.destroy = drm_gem_fb_destroy,
	.create_handle = drm_gem_fb_create_handle,
};

static void dcp_indicator_paint(u32 *px)
{
	unsigned int x, y;

	for (y = 0; y < IND_SIDE; y++)
		for (x = 0; x < IND_SIDE; x++)
			px[y * IND_SIDE + x] =
				(x < IND_BORDER || y < IND_BORDER ||
				 x >= IND_SIDE - IND_BORDER || y >= IND_SIDE - IND_BORDER) ?
				IND_EDGE : IND_FILL;
}

/*
 * Reserve the plane for the kernel and build the square it shows. A failure
 * leaves the plane reserved and without a framebuffer: userspace still
 * cannot use it, no indicator is shown, and the camera streams anyway.
 */
int dcp_indicator_attach_plane(struct apple_dcp *dcp, struct drm_device *drm,
			       struct drm_plane *plane)
{
	struct dcp_camera_indicator *ci = &dcp->camera_ind;
	struct drm_mode_fb_cmd2 cmd = {
		.width = IND_SIDE,
		.height = IND_SIDE,
		.pixel_format = DRM_FORMAT_ARGB8888,
		.pitches[0] = IND_SIDE * 4,
	};
	struct drm_gem_dma_object *obj;
	struct drm_framebuffer *fb;
	unsigned long flags;
	int ret;

	to_apple_plane(plane)->indicator = true;
	ci->plane = plane;

	obj = drm_gem_dma_create(drm, PAGE_ALIGN(IND_SIDE * IND_SIDE * 4));
	if (IS_ERR(obj))
		return PTR_ERR(obj);
	dcp_indicator_paint(obj->vaddr);

	fb = kzalloc(sizeof(*fb), GFP_KERNEL);
	if (!fb) {
		drm_gem_object_put(&obj->base);
		return -ENOMEM;
	}
	drm_helper_mode_fill_fb_struct(drm, fb,
				       drm_get_format_info(drm, cmd.pixel_format, cmd.modifier[0]),
				       &cmd);
	fb->obj[0] = &obj->base;
	ret = drm_framebuffer_init(drm, fb, &dcp_indicator_fb_funcs);
	if (ret) {
		drm_gem_object_put(&obj->base);
		kfree(fb);
		return ret;
	}
	/* Kernel-private: userspace must not find it by id. */
	drm_framebuffer_unregister_private(fb);

	spin_lock_irqsave(&ci->lock, flags);
	ci->fb = fb;
	spin_unlock_irqrestore(&ci->lock, flags);
	return 0;
}

/* The atomic state the kernel itself commits: the square, or nothing. */
static int dcp_indicator_commit(struct apple_dcp *dcp, bool on, bool wait)
{
	struct dcp_camera_indicator *ci = &dcp->camera_ind;
	struct drm_crtc *crtc;
	struct drm_modeset_acquire_ctx ctx;
	struct drm_atomic_commit *state;
	struct drm_plane_state *ps;
	struct drm_crtc_state *cs;
	int ret;

	if (!dcp->crtc || !ci->plane || !ci->fb)
		return -ENODEV;
	crtc = &dcp->crtc->base;

	state = drm_atomic_commit_alloc(crtc->dev);
	if (!state)
		return -ENOMEM;
	drm_modeset_acquire_init(&ctx, 0);
	state->acquire_ctx = &ctx;
retry:
	cs = drm_atomic_get_crtc_state(state, crtc);
	if (IS_ERR(cs)) {
		ret = PTR_ERR(cs);
		goto out;
	}
	ps = drm_atomic_get_plane_state(state, ci->plane);
	if (IS_ERR(ps)) {
		ret = PTR_ERR(ps);
		goto out;
	}
	/*
	 * A swap that carries only the indicator plane has no primary surface, and the H17P
	 * firmware path drops it (backlight prepare -ENODATA), so an off commit never
	 * reached the panel. Re-present the CRTC's other planes with every indicator swap.
	 */
	ret = drm_atomic_add_affected_planes(state, crtc);
	if (ret)
		goto out;
	if (on) {
		if (!cs->active) {
			ret = -ENODEV;
			goto out;
		}
		ret = drm_atomic_set_crtc_for_plane(ps, crtc);
		if (ret)
			goto out;
		drm_atomic_set_fb_for_plane(ps, ci->fb);
		ps->src_x = 0;
		ps->src_y = 0;
		ps->src_w = IND_SIDE << 16;
		ps->src_h = IND_SIDE << 16;
		ps->crtc_x = cs->mode.hdisplay - IND_SIDE - IND_MARGIN;
		ps->crtc_y = IND_MARGIN;
		ps->crtc_w = IND_SIDE;
		ps->crtc_h = IND_SIDE;
	} else {
		ret = drm_atomic_set_crtc_for_plane(ps, NULL);
		if (ret)
			goto out;
		drm_atomic_set_fb_for_plane(ps, NULL);
	}
	WRITE_ONCE(to_apple_plane(ci->plane)->indicator_commit, state);
	ret = wait ? drm_atomic_commit(state) : drm_atomic_nonblocking_commit(state);
	WRITE_ONCE(to_apple_plane(ci->plane)->indicator_commit, NULL);
out:
	if (ret == -EDEADLK) {
		drm_atomic_commit_clear(state);
		ret = drm_modeset_backoff(&ctx);
		if (!ret)
			goto retry;
	}
	drm_atomic_commit_put(state);
	drm_modeset_drop_locks(&ctx);
	drm_modeset_acquire_fini(&ctx);
	return ret;
}

static void dcp_indicator_end(struct apple_dcp *dcp)
{
	struct dcp_camera_indicator *ci = &dcp->camera_ind;
	unsigned long flags;

	spin_lock_irqsave(&ci->lock, flags);
	dcp_ind_end(&ci->state);
	spin_unlock_irqrestore(&ci->lock, flags);
}

static int dcp_indicator_do_commit(void *ctx, bool on, bool wait)
{
	return dcp_indicator_commit(ctx, on, wait);
}

static void dcp_indicator_queue_retry(void *ctx)
{
	struct apple_dcp *dcp = ctx;

	schedule_work(&dcp->camera_ind.retry);
}

static const struct dcp_ind_ops dcp_indicator_ops = {
	.commit = dcp_indicator_do_commit,
	.queue_retry = dcp_indicator_queue_retry,
};

/* ctl held. A pending flip defers removal to the blocking work item. */
static void dcp_indicator_off(struct apple_dcp *dcp)
{
	int ret = dcp_ind_apply(&dcp->camera_ind.state, false, &dcp_indicator_ops, dcp);

	if (ret)
		dev_warn(dcp->dev, "camera indicator off commit failed: %d\n", ret);
}

static void dcp_indicator_retry(struct work_struct *work)
{
	struct dcp_camera_indicator *ci =
		container_of(work, struct dcp_camera_indicator, retry);
	struct apple_dcp *dcp = container_of(ci, struct apple_dcp, camera_ind);
	int ret;

	guard(mutex)(&ci->ctl);
	ret = dcp_ind_retry(&ci->state, ci->fb, &dcp_indicator_ops, dcp);
	if (ret) {
		dev_warn(dcp->dev, "camera indicator commit failed: %d\n", ret);
		dcp_indicator_end(dcp);
	}
}

/*
 * One second after a request, a square that no present has carried to the
 * panel is withdrawn and said so. Nothing waits for this: the camera streams
 * from the moment set(true) returns.
 */
static void dcp_indicator_watch(struct work_struct *work)
{
	struct dcp_camera_indicator *ci =
		container_of(work, struct dcp_camera_indicator, watch.work);
	struct apple_dcp *dcp = container_of(ci, struct apple_dcp, camera_ind);
	unsigned long flags;
	bool pending;

	guard(mutex)(&ci->ctl);
	/* Armed for an earlier request that was stopped and asked for again: the newer watch decides. */
	if (time_before(jiffies, ci->deadline))
		return;
	spin_lock_irqsave(&ci->lock, flags);
	pending = ci->state.state == DCP_IND_PENDING;
	spin_unlock_irqrestore(&ci->lock, flags);
	if (!pending)
		return;

	dev_warn(dcp->dev, "camera indicator not presented within %u ms, withdrawn\n",
		 jiffies_to_msecs(IND_PRESENT_TIMEOUT));
	dcp_indicator_end(dcp);
	dcp_indicator_off(dcp);
}

/* ctl held. Commits the square and arms the watch that withdraws it if no present carries it. */
static int dcp_indicator_request(struct apple_dcp *dcp)
{
	struct dcp_camera_indicator *ci = &dcp->camera_ind;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&ci->lock, flags);
	ret = dcp_ind_begin(&ci->state);
	spin_unlock_irqrestore(&ci->lock, flags);
	if (ret)
		return ret;

	/* A refused commit changed nothing on the plane, so there is no square to take off. */
	ret = dcp_ind_apply(&ci->state, true, &dcp_indicator_ops, dcp);
	if (ret) {
		dcp_indicator_end(dcp);
		return ret;
	}
	/* A panel that was not ready kept the request: the display resuming asks again. */
	spin_lock_irqsave(&ci->lock, flags);
	ret = dcp_ind_wants_resume(&ci->state);
	spin_unlock_irqrestore(&ci->lock, flags);
	if (ret) {
		dev_dbg(dcp->dev, "camera indicator kept: the panel is not ready\n");
		return 0;
	}
	ci->deadline = jiffies + IND_PRESENT_TIMEOUT;
	mod_delayed_work(system_percpu_wq, &ci->watch, IND_PRESENT_TIMEOUT);
	return 0;
}

/*
 * The display resumed and the resume commit is done: the plane came back dark,
 * so the square is OFF now and a request that still stands is asked for again.
 * A lit plane is a set(true) that landed after the resume commit; it is left
 * alone (see dcp_ind_display_resumed).
 * ctl and the plane lock are taken in the order dcp_indicator_commit takes them,
 * so a request cannot land between reading the plane and resetting the state.
 */
void dcp_indicator_display_resumed(struct apple_dcp *dcp)
{
	struct dcp_camera_indicator *ci;
	unsigned long flags;
	bool dark;

	/* A CRTC whose DCP is not bound has no drvdata. */
	if (!dcp)
		return;
	ci = &dcp->camera_ind;
	if (!ci->has_surf)
		return;
	guard(mutex)(&ci->ctl);
	if (!ci->plane)
		return;
	drm_modeset_lock(&ci->plane->mutex, NULL);
	dark = !ci->plane->state || !ci->plane->state->fb;
	drm_modeset_unlock(&ci->plane->mutex);
	spin_lock_irqsave(&ci->lock, flags);
	dcp_ind_display_resumed(&ci->state, dark);
	if (ci->fb && dcp_ind_wants_resume(&ci->state))
		schedule_work(&ci->resume);
	spin_unlock_irqrestore(&ci->lock, flags);
}

/* After a DRM rebind: the camera still streams, so the square is asked for again. */
static void dcp_indicator_resume(struct work_struct *work)
{
	struct dcp_camera_indicator *ci =
		container_of(work, struct dcp_camera_indicator, resume);
	struct apple_dcp *dcp = container_of(ci, struct apple_dcp, camera_ind);
	unsigned long flags;
	bool resume;
	int ret;

	guard(mutex)(&ci->ctl);
	spin_lock_irqsave(&ci->lock, flags);
	resume = ci->fb && dcp_ind_wants_resume(&ci->state);
	spin_unlock_irqrestore(&ci->lock, flags);
	if (!resume)
		return;
	ret = dcp_indicator_request(dcp);
	if (ret)
		dev_warn(dcp->dev, "camera indicator not restored after a display rebind: %d\n", ret);
}

/*
 * Return when the request is submitted or queued. The camera does not wait
 * for the present that confirms the indicator. On stop, a pending flip defers
 * the removal to the work item.
 */
int apple_dcp_camera_indicator_set(struct device *dev, bool on)
{
	struct apple_dcp *dcp;
	struct dcp_camera_indicator *ci;

	/* The device lock keeps the DCP bound, and its state alive, for the call. */
	guard(device)(dev);
	dcp = dev_get_drvdata(dev);
	if (!dcp || !dev->driver)
		return on ? -ENODEV : 0;
	ci = &dcp->camera_ind;
	/* Set from the device tree at probe, so a DCP that never probed it has no lock to take. */
	if (!ci->has_surf)
		return on ? -ENODEV : 0;

	guard(mutex)(&ci->ctl);
	if (!on) {
		cancel_delayed_work(&ci->watch);
		dcp_indicator_end(dcp);
		if (ci->fb)
			dcp_indicator_off(dcp);
		return 0;
	}
	if (!ci->fb)
		return -ENODEV;
	return dcp_indicator_request(dcp);
}
EXPORT_SYMBOL_GPL(apple_dcp_camera_indicator_set);

/* A completed present: the first one that carried the square confirms it. */
void dcp_indicator_swap_done(struct apple_dcp *dcp, struct drm_framebuffer *slot_fb)
{
	struct dcp_camera_indicator *ci = &dcp->camera_ind;
	unsigned long flags;

	if (!ci->has_surf)
		return;
	spin_lock_irqsave(&ci->lock, flags);
	dcp_ind_present_done(&ci->state, slot_fb && slot_fb == ci->fb);
	/* Queued under the lock: release drops the fb under it, so no work is queued after it. */
	if (ci->fb && dcp_ind_wants_resume(&ci->state))
		schedule_work(&ci->resume);
	spin_unlock_irqrestore(&ci->lock, flags);
}
